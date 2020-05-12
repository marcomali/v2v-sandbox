#ifndef CABASICSERVICE_H
#define CABASICSERVICE_H

#include "ns3/socket.h"
#include "ns3/core-module.h"
#include "ns3/traci-client.h"

namespace ns3
{
  typedef enum {
    CAM_NO_ERROR=0,
    CAM_WRONG_INTERVAL=1
  } CABasicService_error_t;

  class CABasicService
  {
  public:
    CABasicService();
    void receiveCAM(Ptr<Socket> socket);
    CABasicService_error_t setT_CheckCamGen(long interval_ms);

    const long T_GenCamMin_ms = 100;    
    const long T_GenCamMax_ms = 1000;

  private:
    void triggerCAM();
    CAdata decodeCAM(CAM_t &cam);
    CAM_t encodeCAM(CAdata cam_data);

    long m_T_CheckCamGen_ms_interval=100;
  };
}

#endif // CABASICSERVICE_H
