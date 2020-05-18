#ifndef CABASICSERVICE_H
#define CABASICSERVICE_H

#include "ns3/socket.h"
#include "ns3/core-module.h"
#include "ns3/vdpTraci.h"
#include "ns3/asn_utils.h"

namespace ns3
{
  typedef enum {
    CAM_NO_ERROR=0,
    CAM_WRONG_INTERVAL=1,
    CAM_ALLOC_ERROR=2,
    CAM_NULL_VDP=3,
    CAM_NO_RSU_CONTAINER=4,
    CAM_ASN1_UPER_ENC_ERROR=5
  } CABasicService_error_t;

  class CABasicService
  {
  public:
    CABasicService();
    CABasicService(unsigned long fixed_stationid,long fixed_stationtype, VDP *vdp, bool real_time, bool is_vehicle);
    CABasicService(unsigned long fixed_stationid,long fixed_stationtype,VDP *vdp,bool real_time,bool is_vehicle,Ptr<Socket> socket_tx);
    void receiveCam(Ptr<Socket> socket);
    void changeNGenCamMax(int16_t N_GenCamMax) {m_N_GenCamMax=N_GenCamMax;}
    void changeRSUGenInterval(long RSU_GenCam_ms) {m_RSU_GenCam_ms=RSU_GenCam_ms;}

    template <typename T> void startCamDissemination(void);
    template <typename T> void startCamDissemination(double desync_s);

    const long T_GenCamMin_ms = 100;    
    const long T_GenCamMax_ms = 1000;

  private:
    template <typename T> void checkCamConditions();
    template <typename T> void initDissemination();
    template <typename T> void RSUDissemination();
    template <typename T> CABasicService_error_t generateAndEncodeCam();
    //CAdata decodeCam(CAM_t &cam);
    //CAM_t encodeCam(CAdata cam_data);
    int64_t computeTimestampUInt64();

    long m_T_CheckCamGen_ms;
    long m_T_GenCam_ms;
    int16_t m_N_GenCam;
    int16_t m_N_GenCamMax;

    long m_RSU_GenCam_ms; // CAM generation interval for RSU ITS-Ss

    int64_t lastCamGen;
    int64_t lastCamGenLowFrequency;
    int64_t lastCamGenSpecialVehicle;

    bool m_real_time;
    bool m_vehicle;
    VDP *m_vdp;

    Ptr<Socket> m_socket_tx; // Socket TX

    StationID_t m_station_id;
    StationType_t m_stationtype;

    // Previous CAM relevant values
    double m_prev_heading;
    double m_prev_distance;
    double m_prev_speed;
  };
}

#endif // CABASICSERVICE_H
