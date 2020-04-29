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

 * Edited by Marco Malinverno, Politecnico di Torino (marco.malinverno@polito.it)
*/
#include "appSample.h"

extern "C"
{
  #include "asn1/CAM.h"
  #include "asn1/DENM.h"
}

namespace ns3
{

  NS_LOG_COMPONENT_DEFINE("appSample");

  NS_OBJECT_ENSURE_REGISTERED(appSample);

  long retValue(double value, int defValue, int fix, int fixNeeded)
  {
      if(fix<fixNeeded)
          return defValue;
      else
          return value;
  }

  TypeId
  appSample::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::appSample")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<appSample> ()
        .AddAttribute ("LonLat",
            "If it is true, position are sent through lonlat (not XY).",
            BooleanValue (false),
            MakeBooleanAccessor (&appSample::m_lon_lat),
            MakeBooleanChecker ())
        .AddAttribute ("CAMIntertime",
            "Time between two consecutive CAMs",
            DoubleValue(0.1),
            MakeDoubleAccessor (&appSample::m_cam_intertime),
            MakeDoubleChecker<double> ())
        .AddAttribute ("RealTime",
            "To compute properly timestamps",
            BooleanValue(false),
            MakeBooleanAccessor (&appSample::m_real_time),
            MakeBooleanChecker ())
        .AddAttribute ("IpAddr",
            "IpAddr",
            Ipv4AddressValue ("10.0.0.1"),
            MakeIpv4AddressAccessor (&appSample::m_ipAddress),
            MakeIpv4AddressChecker ())
        .AddAttribute ("DENMIntertime",
            "Time between two consecutive DENMs",
            DoubleValue(0.5),
            MakeDoubleAccessor (&appSample::m_denm_intertime),
            MakeDoubleChecker<double> ())
        .AddAttribute ("ASN",
            "If true, it uses ASN.1 to encode and decode CAMs and DENMs",
            BooleanValue(false),
            MakeBooleanAccessor (&appSample::m_asn),
            MakeBooleanChecker ())
        .AddAttribute ("SendCam",
            "If it is true, the branch sending the CAM is activated.",
            BooleanValue (true),
            MakeBooleanAccessor (&appSample::m_send_cam),
            MakeBooleanChecker ())
        .AddAttribute ("SendDenm",
            "If true, emergency vehicle broadcast DENMs",
            BooleanValue(true),
            MakeBooleanAccessor (&appSample::m_send_denm),
            MakeBooleanChecker ())
        .AddAttribute ("PrintSummary",
            "To print summary at the end of simulation",
            BooleanValue(false),
            MakeBooleanAccessor (&appSample::m_print_summary),
            MakeBooleanChecker ())
        .AddAttribute ("CSV",
            "CSV log name",
            StringValue (),
            MakeStringAccessor (&appSample::m_csv_name),
            MakeStringChecker ())
        .AddAttribute ("Model",
            "Physical and MAC layer communication model",
            StringValue (""),
            MakeStringAccessor (&appSample::m_model),
            MakeStringChecker ())
        .AddAttribute ("Client",
            "TraCI client for SUMO",
            PointerValue (0),
            MakePointerAccessor (&appSample::m_client),
            MakePointerChecker<TraciClient> ());
        return tid;
  }

  appSample::appSample ()
  {
    NS_LOG_FUNCTION(this);
    m_client = nullptr;
    m_print_summary = true;
    m_already_print = false;

    m_cam_sent = 0;
    m_denm_sent = 0;
    m_cam_received = 0;
    m_denm_received = 0;
  }

  appSample::~appSample ()
  {
    NS_LOG_FUNCTION(this);
  }

  void
  appSample::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  appSample::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);
    /* Save the vehicles informations */
    m_id = m_client->GetVehicleId (this->GetNode ());

    m_type = m_client->TraCIAPI::vehicle.getVehicleClass (m_id);
    m_max_speed = m_client->TraCIAPI::vehicle.getMaxSpeed (m_id);

    appSample::testDENFacility();

    return;

    /* Schedule CAM dissemination */
    std::srand(Simulator::Now().GetNanoSeconds ());
    if (m_send_cam)
      {
         double desync = ((double)std::rand()/RAND_MAX);
         m_send_cam_ev = Simulator::Schedule (Seconds (desync), &appSample::TriggerCam, this);
      }
    /* If it is an emergency vehicle, schedule a DENM send, and repeat it with frequency 2Hz */
    if (m_type=="emergency" && m_send_denm)
      {
        double desync = ((double)std::rand()/RAND_MAX)/4;
        m_send_denm_ev = Simulator::Schedule (Seconds (desync), &appSample::TriggerDenm, this);
      }

    if (!m_csv_name.empty ())
      {
        m_csv_ofstream_cam.open (m_csv_name+"-"+m_id+"-CAM.csv",std::ofstream::trunc);
        m_csv_ofstream_denm.open (m_csv_name+"-"+m_id+"-DENM.csv",std::ofstream::trunc);
        m_csv_ofstream_cam << "messageId,camId,timestamp,latitude,longitude,altitude,heading,speed,acceleration,MeasuredDelayms" << std::endl;
        m_csv_ofstream_denm << "messageId,sequence,referenceTime,detectionTime,stationId,MeasuredDelayms" << std::endl;
      }

    //[TBR]
    if (m_type=="emergency"&&!m_csv_name.empty ())
      m_csv_ofstream_speed.open ("speed"+m_id+".csv",std::ofstream::trunc);
  }

  void
  appSample::StopApplication ()
  {
    NS_LOG_FUNCTION(this);
    Simulator::Remove(m_speed_event);
    Simulator::Remove(m_send_denm_ev);
    Simulator::Remove(m_send_cam_ev);

    if (!m_csv_name.empty ())
      {
        m_csv_ofstream_cam.close ();
        m_csv_ofstream_denm.close ();
      }

    //[TBR]
    if (m_type=="emergency"&&!m_csv_name.empty ())
      m_csv_ofstream_speed.close ();


    if (m_print_summary && !m_already_print)
      {
        std::cout << "INFO-" << m_id
                  << ",DENM-SENT:" << m_denm_sent
                  << ",CAM-SENT:" << m_cam_sent
                  << ",DENM-RECEIVED:" << m_denm_received
                  << ",CAM-RECEIVED:" << m_cam_received
                  << std::endl;
        m_already_print=true;
      }

    m_denService.cleanup();
  }

  void
  appSample::testDENFacility()
  {
    StationType_t stationtype;

    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    m_socket = Socket::CreateSocket (GetNode (), tid);

    if (m_socket->Bind () == -1)
      {
        NS_FATAL_ERROR ("Failed to bind client socket");
      }

    if(m_model=="80211p")
      m_socket->Connect (InetSocketAddress (Ipv4Address::GetBroadcast (),19));
    else if(m_model=="cv2x")
      m_socket->Connect (InetSocketAddress(m_ipAddress,9));
    else
      NS_FATAL_ERROR ("No communication model set - check simulation script");
    m_socket->SetAllowBroadcast (true);
    m_socket->ShutdownRecv();

    m_socket2 = Socket::CreateSocket (GetNode (), tid);

    if (m_socket2->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
      {
        NS_FATAL_ERROR ("Failed to bind client socket");
      }
    // Make the callback to handle received packets
    m_socket2->SetRecvCallback (MakeCallback (&DENBasicService::receiveDENM, &m_denService));

    /* Station Type */
    if (m_type=="passenger")
      stationtype = StationType_passengerCar;
    else if (m_type=="emergency")
      stationtype = StationType_specialVehicles;
    else
      stationtype = StationType_unknown;

    m_denService.setSockets (m_socket,m_socket2);
    m_denService.setStationProperties (std::stol(m_id.substr (3)), (long)stationtype);
    m_denService.addDENRxCallback (std::bind(&appSample::receiveDENM_new,this,std::placeholders::_1));

    Simulator::Schedule (Seconds (5), &appSample::testDENData, this);
  }

  void
  appSample::testDENData()
  {
    ActionID_t actionid;
    denData data;
    DENBasicService_error_t trigger_retval;
    std::string my_edge = m_client->TraCIAPI::vehicle.getRoadID (m_id);
    long my_edge_hash = (long)std::hash<std::string>{}(my_edge)%10000;
    long my_pos_on_edge = m_client->TraCIAPI::vehicle.getLanePosition (m_id);
    data.setDenmMandatoryFields (compute_timestampIts(),my_edge_hash,my_pos_on_edge);

    data.setDenmRepetition (700000,70000);

    trigger_retval=m_denService.appDENM_trigger (data,actionid);
    if(trigger_retval!=DENM_NO_ERROR)
      {
        std::cout<<"Cannot trigger DENM. Error code: "<<trigger_retval<<std::endl;
      }
  }

  void
  appSample::receiveDENM_new (denData denm)
  {
    std::cout << "veh: "<< m_id <<" said to us: 'Good job guys!!!!!!!!'" << std::endl;
  }


  void
  appSample::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

  void
  appSample::TriggerDenm ()
  {
    /* Build DENM data */
    /* FIX: implement other containers, and use them!! */
    long timestamp;
    if(m_real_time)
      {
        timestamp = appSample::compute_timestampIts ()%65536;
      }
    else
      {
        struct timespec tv = compute_timestamp ();
        timestamp = (tv.tv_nsec/1000000)%65536;
      }

    den_data_t denm;
    denm.detectiontime = timestamp;
    denm.messageid = FIX_DENMID;
    denm.proto = FIX_PROT_VERS;

    /* Station Type */
    if (m_type=="passenger")
      denm.stationtype = StationType_passengerCar;
    else if (m_type=="emergency")
      denm.stationtype = StationType_specialVehicles;
    else
      denm.stationtype = StationType_unknown;

    /* In order to encode the info about the edge, the string containing the edgeId is hashed and placed inside latitude
     * This is not allowed from ETSI (of course..) but we needed a way to transmit those infos in a DENM */
    std::string my_edge = m_client->TraCIAPI::vehicle.getRoadID (m_id);
    long my_edge_hash = (long)std::hash<std::string>{}(my_edge)%10000;
    long my_pos_on_edge = m_client->TraCIAPI::vehicle.getLanePosition (m_id);
    denm.evpos_lat = my_edge_hash;
    denm.evpos_long = my_pos_on_edge;
    denm.validity = 10; // seconds
    denm.stationid = std::stol (m_id.substr (3));

    Ptr<CAMDENMSender> app = GetNode()->GetApplication (0)->GetObject<CAMDENMSender> ();
    int app_ret = app->SendDenm(denm);

    if (app_ret)
      m_denm_sent++;

    m_send_denm_ev = Simulator::Schedule (Seconds (0.5), &appSample::TriggerDenm, this);
  }

  void
  appSample::TriggerCam()
  {
    //[TBR]
    if (m_type=="emergency"&&!m_csv_name.empty ())
        m_csv_ofstream_speed << m_client->TraCIAPI::vehicle.getSpeed (m_id) << std::endl;

    /* Build CAM data */
    ca_data_t cam;

    /* Generation delta time [ms since 2004-01-01]. In case the scheduler is not real time, we have to use simulation time,
     * otherwise timestamps will be not reliable */
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
    cam.timestamp = timestamp;

    /* Station Type */
    if (m_type=="passenger")
      cam.type = StationType_passengerCar;
    else if (m_type=="emergency")
      cam.type = StationType_specialVehicles;
    else
      cam.type = StationType_unknown;

    /* Positions - the standard is followed only if m_lonlat is true */
    libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition(m_id);
    if (m_lon_lat)
        pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

    //altitude [0,01 m]
    cam.altitude_conf = AltitudeConfidence_unavailable;
    cam.altitude_value = AltitudeValue_unavailable;

    //latitude WGS84 [0,1 microdegree]
    cam.latitude = (long)retValue(pos.y*DOT_ONE_MICRO,DEF_LATITUDE,0,0);
    //longitude WGS84 [0,1 microdegree]
    cam.longitude = (long)retValue(pos.x*DOT_ONE_MICRO,DEF_LONGITUDE,0,0);

    /* Heading WGS84 north [0.1 degree] */
    double angle = m_client->TraCIAPI::vehicle.getAngle (m_id);
    cam.heading_value = (double)retValue (angle*DECI,DEF_HEADING,0,0);
    cam.heading_conf = HeadingConfidence_unavailable;

    /* Speed [0.01 m/s] */
    double speed=m_client->TraCIAPI::vehicle.getSpeed(m_id);
    cam.speed_value = (long)retValue(speed*CENTI,DEF_SPEED,0,0);
    cam.speed_conf = SpeedConfidence_unavailable;

    /* Acceleration [0.1 m/s^2] */
    double acc=m_client->TraCIAPI::vehicle.getAcceleration (m_id);
    cam.longAcc_value = (long)retValue(acc*DECI,DEF_ACCELERATION,0,0);
    cam.longAcc_conf = AccelerationConfidence_unavailable;

    /* Length and width of car [0.1 m] */
    double veh_length = m_client->TraCIAPI::vehicle.getLength (m_id);
    cam.length_value = (double)retValue (veh_length*DECI,DEF_LENGTH,0,0);
    cam.length_conf = VehicleLengthConfidenceIndication_unavailable;
    double veh_width = m_client->TraCIAPI::vehicle.getWidth (m_id);
    cam.width = (long)retValue (veh_width*DECI,DEF_WIDTH,0,0);

    /* Proto version, id and msg id */
    cam.proto = FIX_PROT_VERS;
    cam.id = std::stol (m_id.substr (3));
    cam.messageid = FIX_CAMID;

    //[tbr]
    struct timespec tv2 = compute_timestamp ();
    cam.latitude = (tv2.tv_nsec/1000)%900000000;

    Ptr<CAMDENMSender> app = GetNode()->GetApplication (0)->GetObject<CAMDENMSender> ();
    app->SendCam (cam);

    m_cam_sent++;

    m_send_cam_ev = Simulator::Schedule (Seconds (m_cam_intertime), &appSample::TriggerCam, this);
  }

  void
  appSample::receiveCAM (ca_data_t cam)
  {
    /* Implement CAM strategy here */
   m_cam_received++;
   long timestamp;
   if(m_real_time)
     {
       timestamp = compute_timestampIts ()%65536;
     }
   else
     {
       struct timespec tv = compute_timestamp ();
       //timestamp = (tv.tv_nsec/1000000)%65536; // [TBR]
       timestamp = (tv.tv_nsec/1000)%900000000;//[TBR]
     }
   if (!m_csv_name.empty ())
     {
       long delay = timestamp-cam.latitude;//[tbr]
       m_csv_ofstream_cam << cam.messageid << "," << cam.id << ",";
       m_csv_ofstream_cam << cam.timestamp << "," << (double)cam.latitude/DOT_ONE_MICRO << ",";
       m_csv_ofstream_cam << (double)cam.longitude/DOT_ONE_MICRO << "," << (cam.altitude_value!=AltitudeValue_unavailable ? (double)cam.altitude_value/DOT_ONE_MICRO : AltitudeValue_unavailable) << ",";
       m_csv_ofstream_cam << (double)cam.heading_value/DECI << "," << (double)cam.speed_value/CENTI << ",";
       m_csv_ofstream_cam << (double)cam.longAcc_value/DECI << "," << delay << std::endl;
     }
  }

  void
  appSample::receiveDENM (den_data_t denm)
  {
    m_denm_received++;

    /* Implement DENM strategy here */

    /* In this case the vehicle that receives a DENM should check:
     * 1) That himself is not an emergency vehicle. In case it is, do nothing.
     * 2) That the DENM is sent by a vehicle that is in the same edge. To do so, the information extracted by m_client->TraCIAPI::vehicle.getRoadID (m_id)
     * is hashed using the function: int my_edge_hash = (int)std::hash<std::string>{}(my_edge)%10000 and included in the DENM. In case of ASN.1 format,
     * it is in the field "latitude", otherwise is the 2nd element. If the vehicle is not in the same edge, do nothing.
     * 3) In case that the vehicle is in the same edge, check if it is already passed. To do so, the information from m_client->TraCIAPI::vehicle.getLanePosition (m_id)
     * is included in the DENM. In case of ASN.1 it is in the longitude field. In case it is already passed, do nothing.
     *
     * If all the control are passed, then the vehicle should slow down by setting its maximum speed to 50% of the original, then it should change lane,
     * by choosing the "rightmost" (i.e. with index 0), in order to facilitate the emergency vehicle takeover. For visualization purposes, it will change
     * color during this phase.
     */
    std::string my_edge = m_client->TraCIAPI::vehicle.getRoadID (m_id);
    long my_edge_hash = (long)std::hash<std::string>{}(my_edge)%10000;
    long my_edge_pos = m_client->TraCIAPI::vehicle.getLanePosition (m_id);


    if (m_type!="emergency")
      {
        if (denm.evpos_lat == my_edge_hash)
          {
            if (denm.evpos_long < my_edge_pos)
              {
                /* Slowdown only if you are not in the takeover lane,
                 * otherwise the ambulance may be stuck behind */
                if (m_client->TraCIAPI::vehicle.getLaneIndex (m_id) == 0)
                  {
                    m_client->TraCIAPI::vehicle.changeLane (m_id,0,3);
                    m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed*0.5);
                    libsumo::TraCIColor orange;
                    orange.r=232;orange.g=126;orange.b=4;orange.a=255;
                    m_client->TraCIAPI::vehicle.setColor (m_id,orange);

                    Simulator::Remove(m_speed_event);
                    m_speed_event = Simulator::Schedule (Seconds (3.0), &appSample::SetMaxSpeed, this);
                  }
                else
                  {
                    m_client->TraCIAPI::vehicle.changeLane (m_id,0,3);
                    m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed*1.5);
                    libsumo::TraCIColor green;
                    green.r=0;green.g=128;green.b=80;green.a=255;
                    m_client->TraCIAPI::vehicle.setColor (m_id,green);

                    Simulator::Remove(m_speed_event);
                    m_speed_event = Simulator::Schedule (Seconds (3.0), &appSample::SetMaxSpeed, this);
                  }
              }
            else if (denm.evpos_long - 50 < my_edge_pos)
              {
                m_client->TraCIAPI::vehicle.changeLane (m_id,0,3);
                m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed*0.5);
                libsumo::TraCIColor orange;
                orange.r=232;orange.g=126;orange.b=4;orange.a=255;
                m_client->TraCIAPI::vehicle.setColor (m_id,orange);

                Simulator::Remove(m_speed_event);
                m_speed_event = Simulator::Schedule (Seconds (3.0), &appSample::SetMaxSpeed, this);
              }
          }
      }

    if (!m_csv_name.empty ())
      {
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
        long delay = timestamp - denm.referencetime;
        m_csv_ofstream_denm << denm.messageid << "," << denm.sequence << ",";
        m_csv_ofstream_denm << denm.referencetime << "," << denm.detectiontime << ",";
        m_csv_ofstream_denm << denm.stationid << "," << delay << std::endl;
      }
  }

  void
  appSample::SetMaxSpeed ()
  {
    libsumo::TraCIColor normal;
    normal.r=255;normal.g=255;normal.b=0;normal.a=255;
    m_client->TraCIAPI::vehicle.setColor (m_id, normal);
    m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed);
  }

  struct timespec
  appSample::compute_timestamp ()
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

  long
  appSample::compute_timestampIts ()
  {
    /* To get millisec since  2004-01-01T00:00:00:000Z */
    auto time = std::chrono::system_clock::now(); // get the current time
    auto since_epoch = time.time_since_epoch(); // get the duration since epoch
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch); // convert it in millisecond since epoch

    long elapsed_since_2004 = millis.count() - TIME_SHIFT; // in TIME_SHIFT we saved the millisec from epoch to 2004-01-01
    return elapsed_since_2004;
  }
}





