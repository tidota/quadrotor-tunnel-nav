#ifndef CUSTOM_OCTOMAP_H
#define CUSTOM_OCTOMAP_H

#include <mrpt/maps/COctoMap.h>

namespace mrpt
{
	namespace maps
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CustomOctoMap , COctoMap, MAPS_IMPEXP )

		class MAPS_IMPEXP CustomOctoMap : public COctoMap
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CustomOctoMap )

		 public:
			 CustomOctoMap(const double resolution=0.10): COctoMap(resolution), resolution_(resolution){} //!< Default constructor
			 virtual ~CustomOctoMap(){} //!< Destructor

			 void setResolution(const double& res);

		private:
			virtual double	 internal_computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom ) MRPT_OVERRIDE;

			const double resolution_;
		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CustomOctoMap , COctoMap, MAPS_IMPEXP )
	} // End of namespace

} // End of namespace

#endif
