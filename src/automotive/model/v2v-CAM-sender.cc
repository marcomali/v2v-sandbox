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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Edited by Marco Malinverno, Politecnico di Torino (name.surname@polito.it)
*/

#include "ns3/log.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"
#include "ns3/address-utils.h"
#include "ns3/nstime.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/socket.h"
#include "ns3/udp-socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include <string>
#include <stdlib.h>
#include <algorithm>
#include "ns3/udp-socket-factory.h"
#include <sys/time.h>
#include <sys/types.h>
#include "ns3/pointer.h"
#include "ns3/trace-source-accessor.h"

#include <errno.h>

#include "v2v-CAM-sender.h"
extern "C"
{
  #include "asn1/CAM.h"
  #include "asn1/DENM.h"
}

namespace ns3
{

  NS_LOG_COMPONENT_DEFINE("v2v-CAM-sender");

  NS_OBJECT_ENSURE_REGISTERED(CAMSender);

  std::pair <double, double> XY2LongLat(double x, double y)
  {

    /*TODO: check this formula */
    double z = 0; //altitude
    double r = 6371000; //Earth radius in mt
    double lat = asin(z/r);
    double lon = 0;
    if (x>0)
      {
        lon=2*atan(y/(sqrt(pow(x,2)+pow(y,2))+x));
      }
    else
      {
        lon=2*atan((sqrt(pow(x,2)+pow(y,2))-x)/y);
      }

     return std::pair <double, double> (lon, lat);
  }

  long setConfidence(double confidence, const int defConfidence, double value, const int defValue)
  {
      if(value==defValue)
          return defConfidence;
      else
          return confidence;
  }

  long retValue(double value, int defValue, int fix, int fixNeeded)
  {
      if(fix < fixNeeded)
          return defValue;
      else
          return value;
  }

  TypeId
  CAMSender::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::CAMSender")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<CAMSender> ()
        .AddAttribute ("Port",
            "The port on which the client will listen for incoming packets.",
            UintegerValue (9),
            MakeUintegerAccessor (&CAMSender::m_port),
            MakeUintegerChecker<uint16_t> ())
        .AddAttribute ("NodePrefix",
            "The prefix used to idefy vehicles in SUMO.",
            StringValue ("veh"),
            MakeStringAccessor (&CAMSender::m_veh_prefix),
            MakeStringChecker ())
        .AddAttribute ("IpAddr",
            "IpAddr",
            Ipv4AddressValue ("10.0.0.1"),
            MakeIpv4AddressAccessor (&CAMSender::m_ipAddress),
            MakeIpv4AddressChecker ())
        .AddAttribute ("Client",
            "TraCI client for SUMO",
            PointerValue (0),
            MakePointerAccessor (&CAMSender::m_client),
            MakePointerChecker<TraciClient> ())
        .AddAttribute ("Index",
            "Index of the current node",
            IntegerValue (1),
            MakeIntegerAccessor (&CAMSender::m_index),
            MakeIntegerChecker<int> ())
        .AddAttribute ("SendCam",
            "If it is true, the branch sending the CAM is activated.",
            BooleanValue (true),
            MakeBooleanAccessor (&CAMSender::m_send_cam),
            MakeBooleanChecker ())
        .AddAttribute ("RealTime",
            "To compute properly timestamps",
            BooleanValue(false),
            MakeBooleanAccessor (&CAMSender::m_real_time),
            MakeBooleanChecker ())
        .AddAttribute ("PrintSummary",
            "To print summary at the end of simulation",
            BooleanValue(false),
            MakeBooleanAccessor (&CAMSender::m_print_summary),
            MakeBooleanChecker ())
        .AddAttribute ("ServerAddr",
            "Ip Addr of the server",
            StringValue("10.0.0.1"),
            MakeStringAccessor (&CAMSender::m_server_addr),
            MakeStringChecker ())
        .AddAttribute ("CAMIntertime",
            "Time between two consecutive CAMs",
            DoubleValue(0.1),
            MakeDoubleAccessor (&CAMSender::m_cam_intertime),
            MakeDoubleChecker<double> ())
        .AddAttribute ("ASN",
            "If true, it uses ASN.1 to encode and decode CAMs and DENMs",
            BooleanValue(false),
            MakeBooleanAccessor (&CAMSender::m_asn),
            MakeBooleanChecker ());

        return tid;
  }

  CAMSender::CAMSender ()
  {
    NS_LOG_FUNCTION(this);
    m_socket = 0;
    m_port = 0;
    m_client = nullptr;
    m_veh_prefix = "veh";
    m_cam_seq = 0;
    m_cam_sent = 0;
    m_denm_sent = 0;
    m_msg_received = 0;
    m_print_summary = true;
    m_already_print = false;
  }

  CAMSender::~CAMSender ()
  {
    NS_LOG_FUNCTION(this);
    m_socket = 0;
    m_socket2 = 0;
  }

  void
  CAMSender::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  CAMSender::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);
    RngSeedManager::SetSeed (m_index);

    // Create the UDP sockets for the client
    // m_socket -> tx
    // m_socket2 -> rx
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    m_socket = Socket::CreateSocket (GetNode (), tid);
    m_socket->Bind ();
    m_socket->Connect (InetSocketAddress(m_ipAddress,9));
    m_socket->SetAllowBroadcast (true);
    m_socket->ShutdownRecv();

    m_socket2 = Socket::CreateSocket (GetNode (), tid);
    m_socket2->Bind(InetSocketAddress (Ipv4Address::GetAny (), 9));

    // Make the callback to handle received packets
    m_socket2->SetRecvCallback (MakeCallback (&CAMSender::HandleRead, this));

    m_id = m_client->GetVehicleId (this->GetNode ());
    // Schedule CAM dissemination
    if (m_send_cam)
       m_sendCamEvent = Simulator::Schedule (Seconds (1.0), &CAMSender::SendCam, this);

    /* If we are in realtime, save a base timestamp to start from */
    if(m_real_time)
      {
        struct timespec tv;
        clock_gettime (CLOCK_MONOTONIC, &tv);
        m_start_ms = tv.tv_sec*1000 + (tv.tv_nsec/1000000);
      }
  }

  void
  CAMSender::StopApplication ()
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
    Simulator::Remove(m_sendCamEvent);

    if (m_print_summary && !m_already_print)
      {
        std::cout << "INFO-" << m_id << ",DENM-SENT:" << m_denm_sent << ",CAM-SENT:" << m_cam_sent << ",MSG-RECEIVED:" << m_msg_received << std::endl;
        m_already_print=true;
      }
  }

  void
  CAMSender::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

  void
  CAMSender::SendCam()
  {
    /*DEBUG: Print position*/
//    Ptr<MobilityModel> mob = this->GetNode ()->GetObject<MobilityModel> ();
//    std::cout << "x:" << mob->GetPosition ().x << std::endl;
//    std::cout << "y:" << mob->GetPosition ().y << std::endl;

    /* This block computes the timestamp. If realtime-> use system time. Else use sumo sim time */
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
    /* End timestamp computation */

    if (m_asn)
      CAMSender::Populate_and_send_asn_cam(tv);
    else
      CAMSender::Populate_and_send_normal_cam(tv);

    // Schedule next CAM
    m_sendCamEvent = Simulator::Schedule (Seconds (m_cam_intertime), &CAMSender::SendCam, this);
  }

  void
  CAMSender::Populate_and_send_normal_cam(struct timespec tv)
  {
    std::ostringstream msg;

    double x = m_client->TraCIAPI::vehicle.getPosition(m_id).x;
    double y = m_client->TraCIAPI::vehicle.getPosition(m_id).y;
    std::pair<double,double> lonlat = XY2LongLat (x,y);


    /* Create the message to be sent in plain text */
    msg << "CAM," << m_id << ","
        << lonlat.first << ","
        << lonlat.second << ","
        << m_client->TraCIAPI::vehicle.getSpeed(m_id) << ","
        << m_client->TraCIAPI::vehicle.getAcceleration (m_id) << ","
        << m_client->TraCIAPI::vehicle.getAngle (m_id) << ","
        << tv.tv_sec << "," << tv.tv_nsec << ","
        << m_cam_seq << ",end\0";

    // Tweak: add +1, otherwise some strange character are received at the end of the packet
    uint16_t packetSize = msg.str ().length () + 1;
    Ptr<Packet> packet = Create<Packet> ((uint8_t*) msg.str ().c_str (), packetSize);

    m_cam_seq++;
    m_cam_sent++;
    // Send packet through the interface
    m_socket->Send(packet);
  }


  void
  CAMSender::Populate_and_send_asn_cam(struct timespec tv)
  {
    CAM_t *cam = (CAM_t*) calloc(1, sizeof(CAM_t));

    /* Install the high freq container */
    cam->cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    /* Generation time (ms since time reference) */
    GenerationDeltaTime_t gen_time;
    if(m_real_time)
      {
        long now_ms = tv.tv_sec*1000 + (tv.tv_nsec/1000000);
        gen_time=now_ms-m_start_ms;
      }
    else
      gen_time = (tv.tv_nsec/1000000)%65536;

    cam->cam.generationDeltaTime = gen_time;

    /* ID, e CAM seq */
    cam->header.protocolVersion=FIX_PROT_VERS;
    cam->header.stationID = std::stol (m_id.substr (3));
    cam->header.messageID=FIX_CAMID;

    /* YawRate angle */
    YawRate yawR;
    double angle = m_client->TraCIAPI::vehicle.getAngle (m_id);
    yawR.yawRateValue=(YawRateValue_t)retValue(angle,DEF_YAWRATE,0,0);
    yawR.yawRateConfidence=setConfidence(FIX_YAWRATE_CONF,DEF_YAWRATE_CONF,angle,DEF_YAWRATE);
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate=yawR;

    /* Heading */
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue=DEF_HEADING;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence=DEF_HEADING_CONF;

    /* Positions */
    double x = m_client->TraCIAPI::vehicle.getPosition(m_id).x;
    double y = m_client->TraCIAPI::vehicle.getPosition(m_id).y;
    std::pair<double,double> lonlat = XY2LongLat (x,y);

    //altitude
    cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence=0;
    cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue=0;
    //latitude
    Latitude_t latitudeT=(Latitude_t)retValue(lonlat.second*DOT_ONE_MICRO,DEF_LATITUDE,FIX2D,FIX2D);
    cam->cam.camParameters.basicContainer.referencePosition.latitude=latitudeT;
    //longitude
    Longitude_t longitudeT=(Longitude_t)retValue(lonlat.first*DOT_ONE_MICRO,DEF_LONGITUDE,FIX2D,FIX2D);
    cam->cam.camParameters.basicContainer.referencePosition.longitude=longitudeT;

    /* Speed */
    Speed_t vel;
    double speed=m_client->TraCIAPI::vehicle.getSpeed(m_id)*100;
    vel.speedValue=(SpeedValue_t)retValue(speed,DEF_SPEED,0,0);
    vel.speedConfidence=setConfidence(FIX_SPEED_CONF,DEF_SPEED_CONF,speed,DEF_SPEED);
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed=vel;

    /* Acceleration */
    LongitudinalAcceleration_t longAcc;
    double acc=m_client->TraCIAPI::vehicle.getAcceleration (m_id);
    longAcc.longitudinalAccelerationValue=(LongitudinalAccelerationValue_t)retValue(acc,DEF_ACCELERATION,0,0);
    longAcc.longitudinalAccelerationConfidence=setConfidence(FIX_ACCEL_CONF,DEF_ACCEL_CONF,acc,DEF_ACCELERATION);
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration=longAcc;

    /* Length and width of car */
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue =m_client->TraCIAPI::vehicle.getLength (m_id)*10;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication=VehicleLengthConfidenceIndication_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth =m_client->TraCIAPI::vehicle.getWidth (m_id)*10;

    /* */
    /* We filled just some fields, you can fill them all to match any purpose */

    /** Encoding **/
    void *buffer = NULL;
    asn_per_constraints_s *constraints = NULL;
    ssize_t ec = uper_encode_to_new_buffer(&asn_DEF_CAM, constraints, cam, &buffer);
    if (ec==-1)
      {
        NS_LOG_INFO("Cannot encode CAM");
        return;
      }
    //xer_fprint (stdout,&asn_DEF_CAM,cam); //Print what you encoded

    /** Packet creation **/
    Ptr<Packet> packet = Create<Packet> ((uint8_t*) buffer, ec+1);

    m_socket->Send (packet);
    m_cam_seq++;
    m_cam_sent++;

    ASN_STRUCT_FREE(asn_DEF_CAM,cam);
  }

  void
  CAMSender::Populate_and_send_normal_denm(struct timespec tv)
  {

    // Generate the packet
    std::ostringstream msg;

    msg << "DENM,"
        << tv.tv_sec << ","
        << tv.tv_nsec << ",end\0";

    //Tweak: add +1, otherwise some random characters are received at the end of the packet
    uint16_t packetSize = msg.str ().length () + 1;
    Ptr<Packet> packet = Create<Packet> ((uint8_t*) msg.str ().c_str (), packetSize);

    m_socket->Send (packet);
    m_denm_sent++;
  }

  void
  CAMSender::Populate_and_send_asn_denm(struct timespec tv)
  {
    /* First DENM */
    DENM_t *denm1 = (DENM_t*) calloc(1, sizeof(DENM_t));

    /* The DENM here is filled with useless values. Be careful that some of the fields are mandatory */

    long gen_time;
    if(m_real_time)
      {
        long now_ms = tv.tv_sec*1000 + (tv.tv_nsec/1000000);
        gen_time=now_ms-m_start_ms;
      }
    else
      gen_time = (tv.tv_nsec/1000000)%65536;

    INTEGER_t decttime;
    memset(&decttime, 0, sizeof(decttime));
    asn_ulong2INTEGER(&decttime, int(tv.tv_sec));
    denm1->denm.management.detectionTime=decttime;

    INTEGER_t timeResult;
    memset(&timeResult, 0, sizeof(timeResult));
    asn_ulong2INTEGER(&timeResult, gen_time);
    denm1->denm.management.referenceTime=timeResult;

    denm1->header.stationID=0;
    denm1->header.protocolVersion=FIX_PROT_VERS;

    StationType_t stationType = StationType_passengerCar;
    denm1->denm.management.stationType=stationType;
    denm1->denm.management.actionID.originatingStationID=0;

    denm1->denm.management.eventPosition.latitude=Latitude_unavailable;
    denm1->denm.management.eventPosition.altitude.altitudeConfidence=AltitudeConfidence_unavailable;
    denm1->denm.management.eventPosition.altitude.altitudeValue=AltitudeValue_unavailable;
    denm1->denm.management.eventPosition.longitude=Longitude_unavailable;

    /* We encode the seq_num in sequenceNumber and the precedence in messageID (FIX THIS) */
    denm1->header.messageID=FIX_DENMID;
    denm1->denm.management.actionID.sequenceNumber=0;

    /** Encoding **/
    void *buffer1 = NULL;
    asn_per_constraints_s *constraints = NULL;
    ssize_t ec = uper_encode_to_new_buffer(&asn_DEF_DENM, constraints, denm1, &buffer1);
    if (ec==-1)
      {
        NS_LOG_INFO("Cannot encode DENM");
        return;
      }
    Ptr<Packet> packet = Create<Packet> ((uint8_t*) buffer1, ec+1);
    m_socket->Send (packet);

    m_denm_sent++;
    ASN_STRUCT_FREE(asn_DEF_DENM,denm1);
  }


  void
  CAMSender::HandleRead (Ptr<Socket> socket)
  {
    NS_LOG_FUNCTION(this << socket);
    Ptr<Packet> packet;
    Address from;
    packet = socket->RecvFrom (from);

    uint8_t *buffer = new uint8_t[packet->GetSize ()];
    packet->CopyData (buffer, packet->GetSize ()-1);

    m_msg_received++;

    if (m_asn)
      {
        /** Decoding **/
        void *decoded_=NULL;
        asn_dec_rval_t rval, rval2;
        /* Try to decode it as a CAM and as a DENM, then check which decoding is ok */
        rval = uper_decode(NULL, &asn_DEF_CAM, &decoded_, buffer, packet->GetSize ()-1, 0, 0);
        void *decoded2_=NULL;
        rval2 = uper_decode (NULL, &asn_DEF_DENM, &decoded2_, buffer, packet->GetSize ()-1, 0, 0);

        /* Parse it as a CAM, read its type, if it is a DENM re-parse it as a DENM*/
        CAM_t *decoded = (CAM_t *) decoded_;
        if (decoded->header.messageID == FIX_CAMID)
          {
            /* It is a CAM!*/
            //xer_fprint (stdout,&asn_DEF_CAM,decoded2); //Print what you encoded
            //std::cout << "CAM in ASN.1 format received!" << std::endl;

            /* Now in "decoded" you have the CAM */
            /* Build your strategy here */
          }
        else if (decoded->header.messageID == FIX_DENMID)
          {
            DENM_t *decoded2 = (DENM_t *) decoded2_;
            //xer_fprint (stdout,&asn_DEF_DENM,decoded); //Print what you encoded
            //std::cout << "DENM in ASN.1 format received!" << std::endl;

            /* Now in "decoded2" you have the DENM */
            /* Build your strategy here */
            ASN_STRUCT_FREE(asn_DEF_DENM,decoded2);
          }
        else
            std::cout << "Unknown packet received" << std::endl;

        ASN_STRUCT_FREE(asn_DEF_CAM,decoded);

      }

    else
      {
        std::string s = std::string ((char*) buffer);
//        std::cout << "Packet received by:" << m_id << " - from:" << from << " - content:" << s << std::endl;
        std::vector<std::string> values;
        std::stringstream ss(s);
        std::string element;
        while (std::getline(ss, element, ',')) {
            values.push_back (element);
          }
//        if (values[0]=="CAM")
//          //Implement CAM strategy here
//        else if (values[0]=="DENM")
//          //Implement DENM strategy here
//        else
//          std::cout << "Unknown packet received by " << m_id << std::endl;
      }
  }

  /* This function is used to calculate the delay for packet reception */
  double
  CAMSender::time_diff(double sec1, double usec1, double sec2, double usec2)
    {
            double tot1 , tot2 , diff;
            tot1 = sec1 + (usec1 / 1000000000.0);
            tot2 = sec2 + (usec2 / 1000000000.0);
            diff = tot2 - tot1;
            return diff;
    }
}





