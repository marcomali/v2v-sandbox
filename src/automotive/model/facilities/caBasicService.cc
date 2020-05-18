#include "caBasicService.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("CABasicService");

  CABasicService::CABasicService()
  {
    m_station_id = ULONG_MAX;
    m_stationtype = LONG_MAX;
    m_socket_tx=NULL;
    m_vdp=NULL;
    m_real_time=false;
    m_T_CheckCamGen_ms=T_GenCamMin_ms;
    m_prev_heading=-1;
    m_prev_speed=-1;
    m_prev_distance=-1;

    m_T_GenCam_ms=T_GenCamMax_ms;
    m_T_CheckCamGen_ms=T_GenCamMin_ms;

    lastCamGen=-1;
    lastCamGenLowFrequency=-1;
    lastCamGenSpecialVehicle=-1;

    m_N_GenCamMax=3;

    m_vehicle=true;

    // CAM generation interval for RSU ITS-Ss (default: 1 s)
    m_RSU_GenCam_ms=1000;
  }

  CABasicService::CABasicService(unsigned long fixed_stationid,long fixed_stationtype,VDP *vdp, bool real_time, bool is_vehicle)
  {
    m_station_id = (StationID_t) fixed_stationid;
    m_stationtype = (StationType_t) fixed_stationtype;

    // Setting a default value of m_T_CheckCamGen_ms equal to 100 ms (i.e. T_GenCamMin_ms)
    m_T_CheckCamGen_ms=T_GenCamMin_ms;
    m_vdp=vdp;
    m_real_time=real_time;

    m_socket_tx=NULL;

    m_prev_heading=-1;
    m_prev_speed=-1;
    m_prev_distance=-1;

    m_T_GenCam_ms=T_GenCamMax_ms;
    m_T_CheckCamGen_ms=T_GenCamMin_ms;

    lastCamGen=-1;
    lastCamGenLowFrequency=-1;
    lastCamGenSpecialVehicle=-1;

    // Set to 3 as described by the ETSI EN 302 637-2 V1.3.1 standard
    m_N_GenCamMax=3;

    m_vehicle=is_vehicle;

    // CAM generation interval for RSU ITS-Ss (default: 1 s)
    m_RSU_GenCam_ms=1000;
  }

  CABasicService::CABasicService(unsigned long fixed_stationid,long fixed_stationtype,VDP *vdp, bool real_time, bool is_vehicle, Ptr<Socket> socket_tx)
  {
    CABasicService(fixed_stationid,fixed_stationtype,vdp,real_time,is_vehicle);

    m_socket_tx=socket_tx;
  }

  template <typename T> void
  CABasicService::startCamDissemination()
  {
    if(m_vehicle)
      {
        Simulator::Schedule (Seconds(0), &CABasicService::initDissemination<T>, this);
      }
    else
      {
        Simulator::Schedule (Seconds (0), &CABasicService::RSUDissemination<T>, this);
      }
  }

  template <typename T> void
  CABasicService::startCamDissemination(double desync_s)
  {
    if(m_vehicle)
      {
        Simulator::Schedule (Seconds (desync_s), &CABasicService::initDissemination<T>, this);
      }
    else
      {
        Simulator::Schedule (Seconds (desync_s), &CABasicService::RSUDissemination<T>, this);
      }
  }

  template <typename T> void
  CABasicService::initDissemination()
  {
    generateAndEncodeCam<T>();
    Simulator::Schedule (MilliSeconds(m_T_CheckCamGen_ms), &CABasicService::checkCamConditions<T>, this);
  }

  template <typename T> void
  CABasicService::RSUDissemination()
  {
    generateAndEncodeCam<T>();
    Simulator::Schedule (MilliSeconds(m_RSU_GenCam_ms), &CABasicService::RSUDissemination<T>, this);
  }

  template <typename T> void
  CABasicService::checkCamConditions()
  {
    T *vdp = static_cast<T *> (m_vdp);
    int64_t now=computeTimestampUInt64 ();
    bool condition_verified=false;
    bool dyn_cond_verified=false;

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
        if(generateAndEncodeCam<T> ()==CAM_NO_ERROR)
          {
            m_N_GenCam=1;
            m_T_GenCam_ms=now-lastCamGen;
            condition_verified=true;
            dyn_cond_verified=true;
          }
      }

    /* 1b)
     * the distance between the current position of the originating ITS-S and
     * the position included in the CAM previously transmitted by the originating
     * ITS-S exceeds 4 m;
    */
    double pos_diff = vdp->getTravelledDistance () - m_prev_distance;
    if (!condition_verified && (pos_diff > 4.0 || pos_diff < -4.0))
      {
        if(generateAndEncodeCam<T> ()==CAM_NO_ERROR)
          {
            m_N_GenCam=1;
            m_T_GenCam_ms=now-lastCamGen;
            condition_verified=true;
            dyn_cond_verified=true;
          }
      }

    /* 1c)
     * he absolute difference between the current speed of the originating ITS-S
     * and the speed included in the CAM previously transmitted by the originating
     * ITS-S exceeds 0,5 m/s.
    */
    double speed_diff = vdp->getSpeedValue () - m_prev_speed;
    if (!condition_verified && (speed_diff > 0.5 || speed_diff < -0.5))
      {
        if(generateAndEncodeCam<T> ()==CAM_NO_ERROR)
          {
            m_N_GenCam=1;
            m_T_GenCam_ms=now-lastCamGen;
            condition_verified=true;
            dyn_cond_verified=true;
          }
      }

    /* 2)
     * The time elapsed since the last CAM generation is equal to or greater than T_GenCam
    */
    if(!condition_verified && (now-lastCamGen>=m_T_GenCam_ms))
      {
         if(generateAndEncodeCam<T> ()==CAM_NO_ERROR)
           {

             if(dyn_cond_verified==true)
               {
                 m_N_GenCam++;
                 if(m_N_GenCam>=m_N_GenCamMax)
                   {
                     m_N_GenCam=0;
                     m_T_GenCam_ms=T_GenCamMax_ms;
                     dyn_cond_verified=false;
                   }
               }
           }
      }

    Simulator::Schedule (MilliSeconds(m_T_CheckCamGen_ms), &CABasicService::checkCamConditions<T>, this);
  }

  template <typename T> CABasicService_error_t
  CABasicService::generateAndEncodeCam()
  {
    CAM_t *cam;
    T *vdp = static_cast<T *> (m_vdp);
    VDP::CAM_mandatory_data_t cam_mandatory_data;

    // Optional CAM data pointers
    AccelerationControl_t *accelerationcontrol;
    LanePosition_t *laneposition;
    SteeringWheelAngle_t *steeringwheelangle;
    LateralAcceleration_t *lateralacceleration;
    VerticalAcceleration_t *verticalacceleration;
    PerformanceClass_t *performanceclass;
    CenDsrcTollingZone_t *tollingzone;

    RSUContainerHighFrequency_t* rsu_container;

    if(vdp==NULL)
      {
        return CAM_NULL_VDP;
      }

    if(m_vehicle==false)
      {
        rsu_container=vdp->getRsuContainerHighFrequency();

        if(rsu_container==NULL)
          {
            NS_LOG_ERROR("Cannot send RSU CAM: the current VDP does not provide any RSU High Frequency Container.");
            return CAM_NO_RSU_CONTAINER;
          }
      }

    /* Collect data for mandatory containers */
    cam=(CAM_t*) calloc(1, sizeof(CAM_t));
    if(cam==NULL)
      {
        return CAM_ALLOC_ERROR;
      }

    cam_mandatory_data=vdp->getCAMMandatoryData();

    /* Fill the header */
    cam->header.messageID = FIX_CAMID;
    cam->header.protocolVersion = FIX_PROT_VERS;
    cam->header.stationID = m_station_id;

    /*
     * Compute the generationDeltaTime, "computed as the time corresponding to the
     * time of the reference position in the CAM, considered as time of the CAM generation.
     * The value of the generationDeltaTime shall be wrapped to 65 536. This value shall be set as the
     * remainder of the corresponding value of TimestampIts divided by 65 536 as below:
     * generationDeltaTime = TimestampIts mod 65 536"
    */
    cam->cam.generationDeltaTime = compute_timestampIts () % 65536;

    /* Fill the basicContainer's station type */
    cam->cam.camParameters.basicContainer.stationType = m_stationtype;

    if(m_vehicle==true)
      {
        /* Fill the basicContainer */
        cam->cam.camParameters.basicContainer.referencePosition.altitude = cam_mandatory_data.altitude;
        cam->cam.camParameters.basicContainer.referencePosition.latitude = cam_mandatory_data.latitude;
        cam->cam.camParameters.basicContainer.referencePosition.longitude = cam_mandatory_data.longitude;
        cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse = cam_mandatory_data.posConfidenceEllipse;

        /* Fill the highFrequencyContainer */
        cam->cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading = cam_mandatory_data.heading;
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed = cam_mandatory_data.speed;
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection = cam_mandatory_data.driveDirection;
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength = cam_mandatory_data.VehicleLength;
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth = cam_mandatory_data.VehicleWidth;
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration = cam_mandatory_data.longAcceleration;
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature = cam_mandatory_data.curvature;
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode = cam_mandatory_data.curvature_calculation_mode_t;
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate = cam_mandatory_data.yawRate;

        // Manage optional data
        accelerationcontrol = vdp->getAccelerationControl();
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.accelerationControl = accelerationcontrol;

        laneposition = vdp->getLanePosition();
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lanePosition = laneposition;

        steeringwheelangle = vdp->getSteeringWheelAngle();
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.steeringWheelAngle = steeringwheelangle;

        lateralacceleration=vdp->getLateralAcceleration();
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration = lateralacceleration;

        verticalacceleration=vdp->getVerticalAcceleration();
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.verticalAcceleration = verticalacceleration;

        performanceclass=vdp->getPerformanceClass();
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.performanceClass = performanceclass;

        tollingzone=vdp->getCenDsrcTollingZone();
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.cenDsrcTollingZone = tollingzone;
      }
   else
      {
        /* Fill the basicContainer */
        /* There is still no full RSU support in this release */
        cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence = AltitudeValue_unavailable;
        cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
        cam->cam.camParameters.basicContainer.referencePosition.latitude = Latitude_unavailable;
        cam->cam.camParameters.basicContainer.referencePosition.longitude = Longitude_unavailable;
        cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
        cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
        cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
        /* Fill the highFrequencyContainer */
        cam->cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_rsuContainerHighFrequency;
        cam->cam.camParameters.highFrequencyContainer.choice.rsuContainerHighFrequency = rsu_container;
      }

    LowFrequencyContainer_t *lowfrequencycontainer=vdp->getLowFrequencyContainer();

    if(lowfrequencycontainer!=NULL)
      {
        // Send a low frequency container only if at least 500 ms have passed since the last CAM with a low frequency container
        if(lastCamGenLowFrequency==-1 ||(computeTimestampUInt64 ()-lastCamGenLowFrequency)>=500)
          {
            cam->cam.camParameters.lowFrequencyContainer = lowfrequencycontainer;
            lastCamGenLowFrequency=computeTimestampUInt64 ();
          }
      }

    SpecialVehicleContainer_t *specialvehiclecontainer=vdp->getSpecialVehicleContainer();

    if(specialvehiclecontainer!=NULL)
      {
        // Send a low frequency container only if at least 500 ms have passed since the last CAM with a low frequency container
        if(lastCamGenSpecialVehicle==-1 ||(computeTimestampUInt64 ()-lastCamGenSpecialVehicle)>=500)
          {
            cam->cam.camParameters.specialVehicleContainer = specialvehiclecontainer;
            lastCamGenSpecialVehicle=computeTimestampUInt64 ();
          }
      }

    /* Construct CAM and pass it to the lower layers (now UDP, in the future BTP and GeoNetworking, then UDP) */
    /** Encoding **/
    char errbuff[ERRORBUFF_LEN];
    size_t errlen=sizeof(errbuff);

    if(asn_check_constraints(&asn_DEF_CAM,(CAM_t *)cam,errbuff,&errlen) == -1) {
        NS_LOG_ERROR("Unable to validate the ASN.1 contraints for the current CAM."<<std::endl);
        NS_LOG_ERROR("Details: " << errbuff << std::endl);
        return CAM_ASN1_UPER_ENC_ERROR;
    }

    asn_encode_to_new_buffer_result_t encode_result = asn_encode_to_new_buffer(NULL,ATS_UNALIGNED_BASIC_PER,&asn_DEF_CAM, cam);
    if (encode_result.result.encoded==-1)
      {
        return CAM_ASN1_UPER_ENC_ERROR;
      }

    Ptr<Packet> packet = Create<Packet> ((uint8_t*) encode_result.buffer, encode_result.result.encoded+1);
    m_socket_tx->Send (packet);

    // Store the time in which the last CAM (i.e. this one) has been generated and successfully sent
    lastCamGen = computeTimestampUInt64 ();

    // Free all the previously allocated memory
    if(m_vehicle==true)
      {
        // After encoding, we can free the previosly allocated optional data
        if(accelerationcontrol) vdp->vdpFree(accelerationcontrol);
        if(laneposition) vdp->vdpFree(laneposition);
        if(steeringwheelangle) vdp->vdpFree(steeringwheelangle);
        if(lateralacceleration) vdp->vdpFree(lateralacceleration);
        if(verticalacceleration) vdp->vdpFree(verticalacceleration);
        if(performanceclass) vdp->vdpFree(performanceclass);
        if(tollingzone) vdp->vdpFree(tollingzone);
      }
    else
      {
        if(rsu_container) vdp->vdpFree(rsu_container);
      }

    if(lowfrequencycontainer) vdp->vdpFree(lowfrequencycontainer);
    if(specialvehiclecontainer) vdp->vdpFree(specialvehiclecontainer);
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
