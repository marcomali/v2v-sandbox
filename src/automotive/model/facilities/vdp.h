#ifndef VDP_H
#define VDP_H

#include "ns3/CAM.h"
#include "asn_utils.h"

namespace ns3
{
  class VDP
  {
    public:
      typedef struct CAM_mandatory_data {
        Speed_t speed;
        Longitude_t longitude;
        Latitude_t latitude;
        Altitude_t altitude;
        PosConfidenceEllipse_t PosConfidenceEllipse;
        LongitudinalAcceleration_t longAcceleration;
        Heading_t heading;
        DriveDirection_t driveDirection;
        Curvature_t curvature;
        CurvatureCalculationMode_t curvature_calculation_mode_t;
        VehicleLength_t VehicleLength;
        VehicleWidth_t VehicleWidth;
        YawRate_t yawRate;
      } CAM_mandatory_data_t;

      virtual CAM_mandatory_data_t getCAMMandatoryData() = 0;

      // These methods refer to optional fields in mandatory containers
      // If the information is not provided, they should be implemented
      // such that NULL is returned for each unavailable information
      // The derived class shall manage the memory allocation for all
      // these methods, including memory cleanup
      virtual AccelerationControl_t *getAccelerationControl() = 0;
      virtual LanePosition_t *getLanePosition() = 0;
      virtual SteeringWheelAngle_t *getSteeringWheelAngle() = 0;
      virtual LateralAcceleration_t *getLateralAcceleration() = 0;
      virtual VerticalAcceleration_t *getVerticalAcceleration() = 0;
      virtual PerformanceClass_t *getPerformanceClass() = 0;
      virtual CenDsrcTollingZone_t *getCenDsrcTollingZone() = 0;

      // As all the above methods, returning a pointer, perform
      // a memory allocation, the derived class should implement
      // a way to let the CA Basic Service free the memory after
      // encoding (with ASN.1) a certain optional field
      template
      <typename T> virtual void vdpFree(T* optional_field) = 0;

      // Optional container methods. As before, they can return NULL
      // if these containers are unavailable
      virtual RSUContainerHighFrequency_t *getRsuContainerHighFrequency();
      virtual LowFrequencyContainer_t *getLowFrequencyContainer();
      virtual SpecialVehicleContainer_t *getSpecialVehicleContainer();

      void setFixedVehicleLength(VehicleLength_t vehicle_length)
      {
         m_vehicle_length=vehicle_length;
      }

      void setFixedVehicleWidth(VehicleWidth_t vehicle_width)
      {
         m_vehicle_width=vehicle_width;
      }

      protected:
        VehicleLength_t m_vehicle_length;
        VehicleWidth_t m_vehicle_width;
  };
}
#endif // VDP_H
