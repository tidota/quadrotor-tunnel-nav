//#include "maps-precomp.h" // Precomp header

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

	double log_lik = 0;
	for (size_t i=0;i<N;i+=likelihoodOptions.decimation)
	{
		if (PIMPL_GET_REF(OCTREE, m_octomap).coordToKeyChecked(scan.getPoint(i), key))
		{
			octomap::OcTreeNode *node = PIMPL_GET_REF(OCTREE, m_octomap).search(key,0 /*depth*/);
			if (node)
				log_lik += std::log(node->getOccupancy());
		}
	}

	//ROS_INFO_STREAM("HEEEEELOOOOOOOOO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	return log_lik;
}


IMPLEMENTS_SERIALIZABLE(CustomOctoMap, COctoMap,mrpt::maps)
