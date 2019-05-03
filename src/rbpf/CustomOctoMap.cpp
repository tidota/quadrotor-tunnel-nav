//#include "maps-precomp.h" // Precomp header

#include <cstdlib>

#include <ros/console.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <mrpt/utils/pimpl.h>
PIMPL_IMPLEMENT(octomap::OcTree);

#include <mrpt/maps/COctoMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CMemoryChunk.h>

//#include "COctoMapBase_impl.h"

#include "rbpf/CustomOctoMap.h"

// Explicit instantiation:
template class mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>;

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::opengl;

//  =========== Begin of Map definition ============
//MAP_DEFINITION_REGISTER("CustomOctoMap,octoMap", mrpt::maps::CustomOctoMap)

// These are defined because the linker otherwise complains.
void  CustomOctoMap::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	COctoMap::writeToStream(out, version);
}
void CustomOctoMap::readFromStream(mrpt::utils::CStream &in, int version)
{
	COctoMap::readFromStream(in, version);
}


void CustomOctoMap::setResolution(const double& res)
{
	PIMPL_GET_CONSTREF(OcTree, m_octomap).setResolution(res);
}

double CustomOctoMap::internal_computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom )
{
	octomap::point3d     sensorPt;
	octomap::Pointcloud  scan;

	if (!internal_build_PointCloud_for_observation(obs,&takenFrom, sensorPt, scan))
		return 0; // Nothing to do.

	octomap::OcTreeKey key;
	const size_t N=scan.size();

	const int search_range = (resolution_ > 0.5)?(int)(1/resolution_)-1: 1;
	double prob_buff;
	double weight_total;
	double log_lik = 0;
	for (size_t i=0;i<N;i+=likelihoodOptions.decimation)
	{
		bool done = false;
		const octomap::point3d target = scan.getPoint(i);
		octomap::OcTreeNode *node;

		const octomap::point3d direction = target - sensorPt;
		const double dist = direction.norm();
		const octomap::point3d origin = sensorPt + direction * ((resolution_/2.0 < dist)? (resolution_/2.0 / dist): 0);
		//ROS_INFO_STREAM("sensorPt: " << sensorPt.x() << ", " << sensorPt.y() << ", " << sensorPt.z());
		//ROS_INFO_STREAM("target: " << target.x() << ", " << target.y() << ", " << target.z());
		//ROS_INFO_STREAM("direction: " << direction.x() << ", " << direction.y() << ", " << direction.z());

		octomap::point3d hit;
		if (PIMPL_GET_REF(OCTREE, m_octomap).castRay(
				origin, direction, hit,
				true, dist + resolution_*2)) //ignoreUnknownCells = true, maxRange
		{
			if (PIMPL_GET_REF(OCTREE, m_octomap).coordToKeyChecked(hit, key) &&
			 (node = PIMPL_GET_REF(OCTREE, m_octomap).search(key,0 /*depth*/)))
			{
				double err = (target - hit).norm();
				double sigma = resolution_/4; // standard deviation: 95% is in one cell length

				log_lik += std::log(node->getOccupancy())
					- std::log(2*3.14159*sigma*sigma)/2.0 - err*err/sigma/sigma/2.0;
				done = true;
			}
		}
		// if an occupied cell is not hit, multiply 0.5 (so add log(0.5))
		if (done == false)
			log_lik += std::log(0.5);

		if (PIMPL_GET_REF(OCTREE, m_octomap).coordToKeyChecked(target, key))
		{
			prob_buff = 0;
			weight_total = 0;
			key[0] -= search_range;
			key[1] -= search_range;
			key[2] -= search_range;
			for (int ix = -search_range; ix <= search_range; ++ix)
			{
				for (int iy = -search_range; iy <= search_range; ++iy)
				{
					for (int iz = -search_range; iz <= search_range; ++iz)
					{
						octomap::point3d p = PIMPL_GET_REF(OCTREE, m_octomap).keyToCoord(key);
						const double diffLen = (p - target).norm();
						double prob;
						double weight;
						if (diffLen < (search_range + 0.5) * resolution_ && // in the sphere
							(node = PIMPL_GET_REF(OCTREE, m_octomap).search(key,0 /*depth*/)) &&
							(prob = node->getOccupancy()) > 0.5)
						{
							const double sigma = resolution_ / 2;
							weight = 1/std::sqrt(2*3.14159*sigma*sigma)
															/ std::exp(diffLen*diffLen/2/sigma/sigma);
															//(1 + 3*search_range
															//- std::abs(ix) - std::abs(iy) - std::abs(iz));
//							double weight = (1 + 3*search_range
									 //- std::abs(ix) - std::abs(iy) - std::abs(iz));
							prob_buff += prob * weight;
							weight_total += weight;
						}
						++key[2];
					}
					key[2] -= 2*search_range;
					++key[1];
				}
				key[1] -= 2*search_range;
				++key[0];
			}
			if (weight_total != 0)
				log_lik += std::log(prob_buff / weight_total);
			else
				log_lik += std::log(0.5);
		}
		else
		{
			log_lik += std::log(0.5);
		}
	}

	//ROS_INFO_STREAM("HEEEEELOOOOOOOOO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	return log_lik;
}


IMPLEMENTS_SERIALIZABLE(CustomOctoMap, COctoMap,mrpt::maps)
