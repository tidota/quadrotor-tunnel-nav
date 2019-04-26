#ifndef CUSTOM_OCTOMAP_H
#define CUSTOM_OCTOMAP_H

#include <mrpt/maps/COctoMap.h>

namespace mrpt
{
	namespace maps
	{
	//DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CustomOctoMap , CMetricMap, MAPS_IMPEXP )

		class CustomOctoMap : public COctoMap
		{
			// This must be added to any CSerializable derived class:
			//DEFINE_SERIALIZABLE( CustomOctoMap )

		 public:
			 CustomOctoMap(const double resolution=0.10): COctoMap(resolution){} //!< Default constructor
			 virtual ~CustomOctoMap(){} //!< Destructor

		private:
			virtual double	 internal_computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom ) MRPT_OVERRIDE;

		}; // End of class def.
		//DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CustomOctoMap , CMetricMap, MAPS_IMPEXP )
	} // End of namespace

} // End of namespace

#endif
