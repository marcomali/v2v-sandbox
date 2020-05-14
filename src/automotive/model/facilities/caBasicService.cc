#include "caBasicService.h"

namespace ns3
{
  CABasicService::CABasicService(VDP *vdp, bool real_time)
  {
    // Setting a default value of m_T_CheckCamGen_ms equal to 100 ms (i.e. T_GenCamMin_ms)
    m_T_CheckCamGen_ms=T_GenCamMin_ms;
    m_vdp=vdp;
    m_real_time=real_time;

    m_socket_tx=NULL;
    m_socket_rx=NULL;

    m_prev_heading=-1;
    m_prev_speed=-1;
    m_prev_distance=-1;

    m_T_GenCam_ms=T_GenCamMax_ms;
    m_T_CheckCamGen_ms=T_GenCamMin_ms;

    // Set to 3 as described by the ETSI EN 302 637-2 V1.3.1 standard
    m_N_GenCamMax=3;
  }

  CABasicService::CABasicService(VDP *vdp, bool real_time, Ptr<Socket> socket_tx, Ptr<Socket> socket_rx)
  {
    CABasicService(vdp,real_time);

    m_socket_tx=socket_tx;
    m_socket_rx=socket_rx;
  }

  template <typename T> void
  CABasicService::startCamDissemination()
  {
    Simulator::Schedule (Seconds(0), &CABasicService::checkCamConditions<T>, this);
  }

  template <typename T> void
  CABasicService::startCamDissemination(double desync_s)
  {
    Simulator::Schedule (Seconds (desync_s), &CABasicService::checkCamConditions<T>, this);
  }

  template <typename T> void
  CABasicService::checkCamConditions()
  {
    T *vdp = static_cast<T *> (m_vdp);
    int64_t now=computeTimestampUInt64 ();

    if(std::is_base_of<VDP, T>::value && !std::is_same<VDP, T>::value)
      {
        NS_FATAL_ERROR("Error. A wrong VDP derived class has been passed to checkCamConditions().");
      }

    // If no initial CAM has been triggered before checkCamConditions() has been called, throw an error
    if(m_prev_heading==-1 || m_prev_speed==-1 || m_prev_distance==-1)
      {
        NS_FATAL_ERROR("Error. checkCamConditions() was called before sending any CAM and this is not allowed.");
      }
    /*
     * ETSI EN 302 637-2 V1.3.1 chap. 6.1.3 condition 1) (no DCC)
     * One of the following ITS-S dynamics related conditions is given:
    */

    /* 1a)
     * The absolute difference between the current heading of the originating
     * ITS-S and the heading included in the CAM previously transmitted by the
     * originating ITS-S exceeds 4°;
    */
    double head_diff = vdp->getHeadingValue () - m_prev_heading;
    head_diff += (head_diff>180.0) ? -360.0 : (head_diff<-180.0) ? 360.0 : 0.0;
    if (head_diff > 4.0 || head_diff < -4.0)
      {
        generateCam ();
        m_N_GenCam=1;
        m_T_GenCam_ms=now-lastCamGen;
      }

    /* 1b)
     * the distance between the current position of the originating ITS-S and
     * the position included in the CAM previously transmitted by the originating
     * ITS-S exceeds 4 m;
    */
    double pos_diff = vdp->getTravelledDistance () - m_prev_distance;
    if (pos_diff > 4.0 || pos_diff < -4.0)
      {
        generateCam ();
        m_N_GenCam=1;
        m_T_GenCam_ms=now-lastCamGen;
      }

    /* 1c)
     * he absolute difference between the current speed of the originating ITS-S
     * and the speed included in the CAM previously transmitted by the originating
     * ITS-S exceeds 0,5 m/s.
    */
    double speed_diff = vdp->getSpeedValue () - m_prev_speed;
    if (speed_diff > 0.5 || speed_diff < -0.5)
      {
        generateCam ();
        m_N_GenCam=1;
        m_T_GenCam_ms=now-lastCamGen;
      }

    /* 2)
     * The time elapsed since the last CAM generation is equal to or greater than T_GenCam
    */
    if(now-lastCamGen>=m_T_GenCam_ms)
      {
         generateCam ();

         m_N_GenCam++;
         if(m_N_GenCam>=m_N_GenCamMax)
           {
             m_N_GenCam=0;
             m_T_GenCam_ms=T_GenCamMax_ms;
           }
      }

    Simulator::Schedule (MilliSeconds(m_T_CheckCamGen_ms), &CABasicService::checkCamConditions<T>, this);
  }

  void
  CABasicService::generateCam()
  {
    m_T_GenCam_ms = computeTimestampUInt64 () - lastCamGen;


    lastCamGen=computeTimestampUInt64 ();
  }

  int64_t
  CABasicService::computeTimestampUInt64()
  {
    int64_t int_tstamp=0;

    if (!m_real_time)
      {
        int_tstamp=Simulator::Now ().GetNanoSeconds ();
      }
    else
      {
        struct timespec tv;

        clock_gettime (CLOCK_MONOTONIC, &tv);

        int_tstamp=tv.tv_sec*1e9+tv.tv_nsec;
      }
    return int_tstamp;
  }
}
