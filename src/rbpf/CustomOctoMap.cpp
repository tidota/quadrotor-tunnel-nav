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

	const mrpt::math::TPoint3D origin(takenFrom.x(),takenFrom.y(),takenFrom.z());
	const
		mrpt::math::TPoint3D direction
			= mrpt::poses::CPose3D(
					0,0,0,takenFrom.yaw(),takenFrom.pitch(),takenFrom.roll())
			+ mrpt::poses::CPoint3D(1,0,0);

	octomap::OcTreeKey key;
	const size_t N=scan.size();

	const int search_range = 2;
	double weight_total;
	double log_lik = 0;
	for (size_t i=0;i<N;i+=likelihoodOptions.decimation)
	{
		bool done = false;
		const octomap::point3d target = scan.getPoint(i);
		octomap::OcTreeNode *node;

		octomap::point3d hit;
		if (PIMPL_GET_REF(OCTREE, m_octomap).castRay(
				octomap::point3d(origin.x,origin.y,origin.z),
				octomap::point3d(direction.x,direction.y,direction.z),
				hit,true,10.0)) //ignoreUnknownCells = true, maxRange = 10.0
		{
			octomap::OcTreeKey hkey;
			if (PIMPL_GET_REF(OCTREE, m_octomap).coordToKeyChecked(target, key) &&
					PIMPL_GET_REF(OCTREE, m_octomap).coordToKeyChecked(hit, hkey))
			{
				if (key == hkey
					&& (node = PIMPL_GET_REF(OCTREE, m_octomap).search(key,0 /*depth*/)))
				{
					log_lik += std::log(0.9999999999999999999999);
					ROS_INFO("hit!!!!!!!!!!");
				}
				else
				{
					log_lik -= std::log(0.0000000000000000000001); // penalty
					ROS_INFO("miss!!!!!!!!!!!");
				}
				done = true;
			}
		}

		if (done == false &&
			PIMPL_GET_REF(OCTREE, m_octomap).coordToKeyChecked(target, key))
		{
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
						node = PIMPL_GET_REF(OCTREE, m_octomap).search(key,0 /*depth*/);
						if (node)
						{
							double prob = node->getOccupancy();
							double weight = (1 + 3*search_range
									 - std::abs(ix) - std::abs(iy) - std::abs(iz));
							log_lik
								+= std::log(prob) * weight;
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
				log_lik /= weight_total;
		}
	}

	//ROS_INFO_STREAM("HEEEEELOOOOOOOOO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	return log_lik;
}


IMPLEMENTS_SERIALIZABLE(CustomOctoMap, COctoMap,mrpt::maps)
