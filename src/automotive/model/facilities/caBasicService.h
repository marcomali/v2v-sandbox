#ifndef CABASICSERVICE_H
#define CABASICSERVICE_H

#include "ns3/socket.h"
#include "ns3/core-module.h"
#include "ns3/vdpTraci.h"

namespace ns3
{
  typedef enum {
    CAM_NO_ERROR=0,
    CAM_WRONG_INTERVAL=1
  } CABasicService_error_t;

  class CABasicService
  {
  public:
    CABasicService(VDP *vdp,bool real_time);
    CABasicService(VDP *vdp,bool real_time,Ptr<Socket> socket_tx,Ptr<Socket> socket_rx);
    void receiveCam(Ptr<Socket> socket);
    void changeNGenCamMax(int16_t N_GenCamMax) {m_N_GenCamMax=N_GenCamMax;}

    template <typename T> void startCamDissemination(void);
    template <typename T> void startCamDissemination(double desync_s);

    const long T_GenCamMin_ms = 100;    
    const long T_GenCamMax_ms = 1000;

  private:
    template <typename T> void checkCamConditions();
    void generateCam();
    //CAdata decodeCam(CAM_t &cam);
    //CAM_t encodeCam(CAdata cam_data);
    int64_t computeTimestampUInt64();

    long m_T_CheckCamGen_ms;
    long m_T_GenCam_ms;
    int16_t m_N_GenCam;
    int16_t m_N_GenCamMax;

    int64_t lastCamGen;
    int64_t lastCamGenLowFrequency;
    int64_t lastCamGenSpecialVehicle;

    bool m_real_time;
    VDP *m_vdp;

    Ptr<Socket> m_socket_tx; // Socket TX
    Ptr<Socket> m_socket_rx; // Socket RX

    // Previous CAM relevant values
    double m_prev_heading;
    double m_prev_distance;
    double m_prev_speed;
  };
}

#endif // CABASICSERVICE_H
