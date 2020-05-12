#ifndef VDPTRACI_H
#define VDPTRACI_H

#include "vdp.h"
#include "ns3/traci-client.h"

namespace ns3 {
  class VDPTraCI : public VDP
  {
  public:
    VDPTraCI(Ptr<TraciClient> traci_client);

    CAM_mandatory_data_t getCAMMandatoryData();

    AccelerationControl_t *getAccelerationControl() {return NULL;}
    LanePosition_t *getLanePosition();
    SteeringWheelAngle_t *getSteeringWheelAngle() {return NULL;}
    LateralAcceleration_t *getLateralAcceleration() {return NULL;}
    VerticalAcceleration_t *getVerticalAcceleration() {return NULL;}
    PerformanceClass_t *getPerformanceClass() {return NULL;}
    CenDsrcTollingZone_t *getCenDsrcTollingZone() {return NULL;}

    template
    <typename T> virtual void vdpFree(T* optional_field) {
      if(optional_field!=NULL)
        {
          free(optional_field);
        }
    }

    RSUContainerHighFrequency_t *getRsuContainerHighFrequency() {return NULL;}
    LowFrequencyContainer_t *getLowFrequencyContainer() {return NULL;}
    SpecialVehicleContainer_t *getSpecialVehicleContainer() {return NULL;}

    private:
      std::string m_vehicle_id;
      Ptr<TraciClient> m_traci_client;
  };
}

#endif // VDPTRACI_H
