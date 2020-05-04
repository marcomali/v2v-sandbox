/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright 2007 University of Washington
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Edited by Marco Malinverno, Politecnico di Torino (marco.malinverno@polito.it)
*/

#include "v2v-CAM-DENM-sender.h"

extern "C"
{
  #include "asn1/CAM.h"
  #include "asn1/DENM.h"
}

namespace ns3
{

  NS_LOG_COMPONENT_DEFINE("v2v-CAM-DENM-sender");

  NS_OBJECT_ENSURE_REGISTERED(CAMDENMSender);

  TypeId
  CAMDENMSender::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::CAMDENMSender")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<CAMDENMSender> ()
        .AddAttribute ("Port",
            "The port on which the client will listen for incoming packets.",
            UintegerValue (9),
            MakeUintegerAccessor (&CAMDENMSender::m_port),
            MakeUintegerChecker<uint16_t> ())
        .AddAttribute ("IpAddr",
            "IpAddr",
            Ipv4AddressValue ("10.0.0.1"),
            MakeIpv4AddressAccessor (&CAMDENMSender::m_ipAddress),
            MakeIpv4AddressChecker ())
        .AddAttribute ("RealTime",
            "To compute properly timestamps",
            BooleanValue(false),
            MakeBooleanAccessor (&CAMDENMSender::m_real_time),
            MakeBooleanChecker ())
        .AddAttribute ("ServerAddr",
            "Ip Addr of the server",
            StringValue("10.0.0.1"),
            MakeStringAccessor (&CAMDENMSender::m_server_addr),
            MakeStringChecker ())
        .AddAttribute ("Model",
            "Physical and MAC layer communication model",
            StringValue (""),
            MakeStringAccessor (&CAMDENMSender::m_model),
            MakeStringChecker ())
        .AddAttribute ("ASN",
            "If true, it uses ASN.1 to encode and decode CAMs and DENMs",
            BooleanValue(false),
            MakeBooleanAccessor (&CAMDENMSender::m_asn),
            MakeBooleanChecker ());
        return tid;
  }

  CAMDENMSender::CAMDENMSender ()
  {
    NS_LOG_FUNCTION(this);
    m_socket = 0;
    m_port = 0;
    m_sequence = 0;
    m_actionId = 0;
  }

  CAMDENMSender::~CAMDENMSender ()
  {
    NS_LOG_FUNCTION(this);
    m_socket = 0;
    m_socket2 = 0;
  }

  void
  CAMDENMSender::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  CAMDENMSender::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);

    // Create the UDP sockets for the client
    // m_socket -> tx
    // m_socket2 -> rx
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    m_socket = Socket::CreateSocket (GetNode (), tid);
    if (m_socket->Bind () == -1)
      {
        NS_FATAL_ERROR ("Failed to bind client socket");
      }
    if(m_model=="80211p")
      m_socket->Connect (InetSocketAddress (Ipv4Address::GetBroadcast (),9));
    else if(m_model=="cv2x")
      m_socket->Connect (InetSocketAddress(m_ipAddress,9));
    else
      NS_FATAL_ERROR ("No communication model set - check simulation script");

    m_socket->SetAllowBroadcast (true);
    m_socket->ShutdownRecv();

    m_socket2 = Socket::CreateSocket (GetNode (), tid);
    if (m_socket2->Bind (InetSocketAddress (Ipv4Address::GetAny (), 9)) == -1)
      {
        NS_FATAL_ERROR ("Failed to bind client socket");
      }
    // Make the callback to handle received packets
    m_socket2->SetRecvCallback (MakeCallback (&CAMDENMSender::HandleRead, this));
  }

  void
  CAMDENMSender::StopApplication ()
  {
    NS_LOG_FUNCTION(this);

    if (m_socket != 0)
      {
        m_socket->Close ();
        m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
        m_socket = 0;
      }
    if (m_socket2 != 0)
      {
        m_socket2->Close ();
        m_socket2->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
        m_socket2 = 0;
      }
  }

  void
  CAMDENMSender::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

  void
  CAMDENMSender::SendCam(ca_data_t cam)
  {
    if (m_asn)
      CAMDENMSender::Populate_and_send_asn_cam(cam);
    else
      CAMDENMSender::Populate_and_send_normal_cam(cam);
  }

  int CAMDENMSender::SendDenm(den_data_t denm)
  {
    if (m_asn)
      return CAMDENMSender::Populate_and_send_asn_denm(denm);
    else
      return CAMDENMSender::Populate_and_send_normal_denm(denm);
  }

  void
  CAMDENMSender::Populate_and_send_normal_cam(ca_data_t cam)
  {
    std::ostringstream msg;

    /* Create the message to be sent in plain text */
    msg << "CAM," << cam.id << ","
        << cam.longitude << ","
        << cam.latitude << ","
        << cam.altitude_value << ","
        << cam.speed_value << ","
        << cam.longAcc_value << ","
        << cam.heading_value << ","
        << cam.timestamp << ",end\0";

    // Tweak: add +1, otherwise some strange character are received at the end of the packet
    uint16_t packetSize = msg.str ().length () + 1;
    Ptr<Packet> packet = Create<Packet> ((uint8_t*) msg.str ().c_str (), packetSize);

    // Send packet through the interface
    m_socket->Send(packet);
  }

  void
  CAMDENMSender::Populate_and_send_asn_cam(ca_data_t data)
  {
    /* Here a ASN.1 CAM is encoded, following ETSI EN 302 637-3, ETSI EN 302 637-2 and ETSI TS 102 894-2 encoding rules
     * in square brakets the unit used to encode the data */

    CAM_t *cam = (CAM_t*) calloc(1, sizeof(CAM_t));

    /* Install the high freq container */
    cam->cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    /* Generation delta time [ms since 2004-01-01]. In case the scheduler is not real time, we have to use simulation time,
     * otherwise timestamps will be not reliable */
    cam->cam.generationDeltaTime = (GenerationDeltaTime_t)data.timestamp;

    /* Station Type */
    cam->cam.camParameters.basicContainer.stationType = (StationType_t)data.type;

    /* Positions */
    //altitude [0,01 m]
    cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence = (AltitudeConfidence_t)data.altitude_conf;
    cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue = (AltitudeValue_t)data.altitude_value;

    //latitude WGS84 [0,1 microdegree]
    cam->cam.camParameters.basicContainer.referencePosition.latitude = (Latitude_t) data.latitude;

    //longitude WGS84 [0,1 microdegree]
    cam->cam.camParameters.basicContainer.referencePosition.longitude = (Longitude_t) data.longitude;

    /* Heading WGS84 north [0.1 degree] */
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence = (HeadingConfidence_t) data.heading_conf;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue = (HeadingValue_t) data.heading_value;

    /* Speed [0.01 m/s] */
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence = (SpeedConfidence_t) data.speed_conf;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = (SpeedValue_t) data.speed_value;

    /* Acceleration [0.1 m/s^2] */
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence = (AccelerationConfidence_t) data.longAcc_conf;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = (LongitudinalAccelerationValue_t) data.longAcc_value;

    /* Length and width of car [0.1 m] */
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication = (VehicleLengthConfidenceIndication_t) data.length_conf;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue = (VehicleLengthValue_t) data.length_value;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth = (VehicleWidth_t) data.width;

    /* Other needed fields */
    cam->header.protocolVersion = data.proto;
    cam->header.stationID = (StationID_t) data.id;
    cam->header.messageID = data.messageid;

    /* We filled just some fields, it is possible to fill them all to match any purpose */

    /** Encoding **/
    void *buffer = NULL;
    asn_per_constraints_s *constraints = NULL;
    ssize_t ec = uper_encode_to_new_buffer(&asn_DEF_CAM, constraints, cam, &buffer);
    if (ec==-1)
      {
        std::cout << "Cannot encode CAM." << std::endl;
        return;
      }

    /** Packet creation **/
    Ptr<Packet> packet = Create<Packet> ((uint8_t*) buffer, ec+1);

    m_socket->Send (packet);

    ASN_STRUCT_FREE(asn_DEF_CAM,cam);
  }

  int
  CAMDENMSender::Populate_and_send_normal_denm(den_data_t denm)
  {
    // Generate the packet
    std::ostringstream msg;

    long timestamp;
    if(m_real_time)
      {
        timestamp = compute_timestampIts ()%65536;
      }
    else
      {
        struct timespec tv = compute_timestamp ();
        timestamp = (tv.tv_nsec/1000000)%65536;
      }

//  Overflow problem - FIX IT
    /* If validity is expired return 0 */
//    if ((timestamp - data.detectiontime) > data.validity*1000)
//      return 0;

    m_sequence++;
    m_actionId++;

    msg << "DENM,"
        << denm.detectiontime << ","
        << timestamp << ","
        << denm.stationid << ","
        << m_sequence << ","
        << denm.evpos_lat << ","
        << denm.evpos_long << ",end\0";

    //Tweak: add +1, otherwise some random characters are received at the end of the packet
    uint16_t packetSize = msg.str ().length () + 1;
    Ptr<Packet> packet = Create<Packet> ((uint8_t*) msg.str ().c_str (), packetSize);

    m_socket->Send (packet);
    return m_actionId;
  }

  int
  CAMDENMSender::Populate_and_send_asn_denm(den_data_t data)
  {
    /* Here a ASN.1 DENM is encoded, following ETSI EN 302 637-3, ETSI EN 302 637-2 and ETSI TS 102 894-2 encoding rules
     * in square brakets the unit used to transfer the data */

    /* First you have to check if the DENM validity has expired */
    long timestamp;
    if(m_real_time)
      {
        timestamp = compute_timestampIts ()%65536;
      }
    else
      {
        struct timespec tv = compute_timestamp ();
        timestamp = (tv.tv_nsec/1000000)%65536;
      }

//  Overflow problem - FIX IT
    /* If validity is expired return 0 */
//    if ((timestamp - data.detectiontime) > data.validity*1000)
//      return 0;

    m_sequence++;
    m_actionId++;

    DENM_t *denm = (DENM_t*) calloc(1, sizeof(DENM_t));

    /* Header */
    denm->header.messageID = data.messageid;
    denm->header.stationID = data.stationid;
    denm->header.protocolVersion = data.proto;

    /* Management container */
    /* Set actionID and sequence */
    denm->denm.management.actionID.sequenceNumber = m_sequence;
    denm->denm.management.actionID.originatingStationID = m_actionId;

    /* Detection time [ms since 2004-01-01] (time at which the event is detected). */
    INTEGER_t detection_time;
    memset(&detection_time, 0, sizeof(detection_time));
    asn_long2INTEGER(&detection_time, data.detectiontime);
    denm->denm.management.detectionTime=detection_time;

    /* Reference time [ms since 2004-01-01] (time at wich the DENM is generated). In case the scheduler is not real time,
     * we have to use simulation time, otherwise timestamps will be not reliable */
    INTEGER_t ref_time;
    memset(&ref_time, 0, sizeof(ref_time));
    asn_long2INTEGER(&ref_time, timestamp);
    denm->denm.management.referenceTime = ref_time;

    /* Station Type */
    denm->denm.management.stationType = data.stationtype;

    denm->denm.management.eventPosition.latitude=data.evpos_lat;
    denm->denm.management.eventPosition.longitude=data.evpos_long;

//    denm->denm.alacarte = (AlacarteContainer_t*)calloc(1, sizeof(AlacarteContainer_t));
//      denm->denm.alacarte->externalTemperature = (Temperature_t*)calloc(1, sizeof(Temperature_t));

//    Temperature_t temp = 52;
//    denm->denm.alacarte->externalTemperature = &temp;

    /** Encoding **/
    void *buffer = NULL;
    asn_per_constraints_s *constraints = NULL;
    ssize_t ec = uper_encode_to_new_buffer(&asn_DEF_DENM, constraints, denm, &buffer);
    if (ec==-1)
      {
        std::cout << "Cannot encode DENM" << std::endl;
        return 0;
      }

    Ptr<Packet> packet = Create<Packet> ((uint8_t*) buffer, ec+1);
    m_socket->Send (packet);

    ASN_STRUCT_FREE(asn_DEF_DENM,denm);
    return m_actionId;
  }

  void
  CAMDENMSender::HandleRead (Ptr<Socket> socket)
  {
    NS_LOG_FUNCTION(this << socket);
    Ptr<Packet> packet;
    Address from;
    packet = socket->RecvFrom (from);

    uint8_t *buffer = new uint8_t[packet->GetSize ()];
    packet->CopyData (buffer, packet->GetSize ()-1);

    if (m_asn)
      {
        /** Decoding **/
        /* Try to decode it as a CAM and as a DENM, then check which decoding is ok */
        void *decoded_=NULL;
        asn_dec_rval_t rval,rval2;
        rval = uper_decode(NULL, &asn_DEF_CAM, &decoded_, buffer, packet->GetSize ()-1, 0, 0);

        void *decoded2_=NULL;
        rval2 = uper_decode (NULL, &asn_DEF_DENM, &decoded2_, buffer, packet->GetSize ()-1, 0, 0);

        if (rval.code != RC_OK && rval2.code != RC_OK)
          {
            std::cout << "ASN.1 decoding failed!" << std::endl;
            return;
          }

        /* Parse it as a CAM, read its type, if it is a DENM send it to decodeDenm */
        CAM_t *decoded = (CAM_t *) decoded_;
        DENM_t *decoded2 = (DENM_t *) decoded2_;

        if (decoded->header.messageID == FIX_CAMID)
          {
            CAMDENMSender::Decode_asn_cam(buffer,packet->GetSize ()-1);
          }
        else if (decoded2->header.messageID == FIX_DENMID)
          {
            CAMDENMSender::Decode_asn_denm (buffer,packet->GetSize ()-1);
          }
        else
          std::cout << "Unknown ASN.1 packet received" << std::endl;

        ASN_STRUCT_FREE(asn_DEF_CAM,decoded);
        ASN_STRUCT_FREE(asn_DEF_DENM,decoded2);

      }
    else
      {
        std::vector<std::string> values;
        std::string s = std::string ((char*) buffer);
        //std::cout << "Packet received - content:" << s << std::endl;
        std::stringstream ss(s);
        std::string element;
        while (std::getline(ss, element, ',')) {
            values.push_back (element);
          }

        if(values[0]=="CAM")
          CAMDENMSender::Decode_normal_cam(buffer);
        else if (values[0]=="DENM")
          CAMDENMSender::Decode_normal_denm(buffer);
        else
          std::cout << "Unknown packet received" << std::endl;
      }
  }

  void
  CAMDENMSender::Decode_asn_cam(uint8_t *buffer, uint32_t size)
  {
    /** Decoding **/
    void *decoded_=NULL;
    asn_dec_rval_t rval;
    ca_data_t cam;

    rval = uper_decode(0, &asn_DEF_CAM, &decoded_, buffer, size, 0, 1);

    if (rval.code == RC_FAIL)
      {
        std::cout << "CAM ASN.1 decoding failed!"<< std::endl;
        return;
      }

    CAM_t *decoded = (CAM_t *) decoded_;

    if (decoded->header.messageID == FIX_CAMID)
      {
        /* Now in "decoded" you have the CAM */
        /* Build your CAM strategy here! */
        cam.longitude = (long)decoded->cam.camParameters.basicContainer.referencePosition.longitude;
        cam.latitude = (long)decoded->cam.camParameters.basicContainer.referencePosition.latitude;
        cam.altitude_conf = (long)decoded->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence;
        cam.altitude_value = (long)decoded->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue;
        cam.heading_conf = (long)decoded->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence;
        cam.heading_value = (long)decoded->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue;
        cam.id = (long)decoded->header.stationID;
        cam.length_conf = (long)decoded->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication;
        cam.length_value = (long)decoded->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue;
        cam.width = (long)decoded->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth;
        cam.longAcc_conf = (long)decoded->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence;
        cam.longAcc_value = (long)decoded->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue;
        cam.messageid = (int)decoded->header.messageID;
        cam.proto = (int)decoded->header.protocolVersion;
        cam.speed_conf = (long)decoded->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence;
        cam.speed_value = (long)decoded->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue;
        cam.timestamp = (long)decoded->cam.generationDeltaTime;
        cam.type = (long)decoded->cam.camParameters.basicContainer.stationType;

        Ptr<appSample> app = GetNode()->GetApplication (1)->GetObject<appSample> ();
        app->receiveCAM (cam);
      }

    ASN_STRUCT_FREE(asn_DEF_CAM,decoded);
  }

  void
  CAMDENMSender::Decode_normal_cam(uint8_t *buffer)
  {
    ca_data_t cam;
    std::vector<std::string> values;
    std::string s = std::string ((char*) buffer);
    //std::cout << "Packet received - content:" << s << std::endl;
    std::stringstream ss(s);
    std::string element;
    while (std::getline(ss, element, ',')) {
        values.push_back (element);
      }

    if(values[0]=="CAM")
      {
        cam.messageid = FIX_CAMID;
        cam.id = std::stol(values[1]);
        cam.latitude = std::stol(values[2]);
        cam.longitude = std::stol(values[3]);
        cam.altitude_value = std::stol(values[4]);
        cam.speed_value = std::stol(values[5]);
        cam.longAcc_value = std::stol(values[6]);
        cam.heading_value = std::stol(values[7]);
        cam.timestamp = std::stol(values[8]);


        Ptr<appSample> app = GetNode()->GetApplication (1)->GetObject<appSample> ();
        app->receiveCAM (cam);
      }
  }
  void
  CAMDENMSender::Decode_asn_denm(uint8_t *buffer,uint32_t size)
  {
    /** Decoding **/
//    void *decoded_=NULL;
//    asn_dec_rval_t rval;
//    den_data_t denm;

//    rval = uper_decode(0, &asn_DEF_DENM, &decoded_, buffer, size, 0, 1);

//    if (rval.code == RC_FAIL)
//      {
//        std::cout << "DENM ASN.1 decoding failed!" << std::endl;
//        return;
//      }

//    DENM_t *decoded = (DENM_t *) decoded_;

//    if (decoded->header.messageID==FIX_DENMID)
//      {
//        denm.proto = (int)decoded->header.protocolVersion;
//        denm.stationid = (long)decoded->header.stationID;
//        denm.messageid = (int)decoded->header.messageID;

//        denm.sequence = (int)decoded->denm.management.actionID.sequenceNumber;
//        denm.actionid = (long)decoded->denm.management.actionID.originatingStationID;

//        long detection_time;
//        memset(&detection_time, 0, sizeof(detection_time));
//        asn_INTEGER2long (&decoded->denm.management.detectionTime,&detection_time);
//        denm.detectiontime =detection_time;

//        long ref_time;
//        memset(&ref_time, 0, sizeof(ref_time));
//        asn_INTEGER2long (&decoded->denm.management.referenceTime,&ref_time);
//        denm.referencetime = ref_time;

//        denm.stationtype = (long)decoded->denm.management.stationType;

//        denm.evpos_lat = (long)decoded->denm.management.eventPosition.latitude;
//        denm.evpos_long = (long)decoded->denm.management.eventPosition.longitude;

//        std::cout << "temp "<< *decoded->denm.alacarte->externalTemperature << std::endl;

        //Ptr<appSample> app = GetNode()->GetApplication (1)->GetObject<appSample> ();
        //app->receiveDENM (denm);
      //}
    //ASN_STRUCT_FREE(asn_DEF_DENM,decoded);
  }

  void
  CAMDENMSender::Decode_normal_denm(uint8_t *buffer)
  {
//    den_data_t denm;
//    std::vector<std::string> values;
//    std::string s = std::string ((char*) buffer);
//    //std::cout << "Packet received - content:" << s << std::endl;
//    std::stringstream ss(s);
//    std::string element;
//    while (std::getline(ss, element, ',')) {
//        values.push_back (element);
//      }

//    if(values[0]=="DENM")
//      {
//        denm.messageid = FIX_DENMID;
//        denm.detectiontime =  std::stol(values[1]);
//        denm.referencetime = std::stol(values[2]);
//        denm.stationid = std::stoi(values[3]);
//        denm.sequence = std::stoi(values[4]);
//        denm.evpos_lat = std::stol(values[5]);
//        denm.evpos_long = std::stol(values[6]);

//        Ptr<appSample> app = GetNode()->GetApplication (1)->GetObject<appSample> ();
//        //app->receiveDENM (denm);
//      }
  }

  struct timespec
  CAMDENMSender::compute_timestamp ()
  {
    struct timespec tv;
    if (!m_real_time)
      {
        double nanosec =  Simulator::Now ().GetNanoSeconds ();
        tv.tv_sec = 0;
        tv.tv_nsec = nanosec;
      }
    else
      {
        clock_gettime (CLOCK_MONOTONIC, &tv);
      }
    return tv;
  }


}





