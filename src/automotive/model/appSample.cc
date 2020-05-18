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

    /* Create the Sockets for TX and RX */
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

    /* Set Station Type in DENBasicService */
    StationType_t stationtype;
    if (m_type=="passenger")
      stationtype = StationType_passengerCar;
    else if (m_type=="emergency")
      stationtype = StationType_specialVehicles;
    else
      stationtype = StationType_unknown;

    /* Set sockets, callback and station properties in DENBasicService */
    m_denService.setSocketTx (m_socket);
    m_denService.setStationProperties (std::stol(m_id.substr (3)), (long)stationtype);
    m_denService.addDENRxCallback (std::bind(&appSample::receiveDENM,this,std::placeholders::_1));

    /* Set sockets, callback, station properties and TraCI VDP in CABasicService */
    m_caService.setSocketTx (m_socket);
    m_caService.setStationProperties (std::stol(m_id.substr (3)), (long)stationtype);
    m_caService.addCARxCallback (std::bind(&appSample::receiveCAM,this,std::placeholders::_1));

    m_vdp.setProperties(m_client,m_id);
    m_caService.setVDP(&m_vdp);

    /* Schedule CAM dissemination */
    std::srand(Simulator::Now().GetNanoSeconds ());
    if (m_send_cam)
      {
         double desync = ((double)std::rand()/RAND_MAX);
         m_caService.startCamDissemination(desync);

         //m_send_cam_ev = Simulator::Schedule (Seconds (desync), &appSample::TriggerCam, this);
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
        m_csv_ofstream_cam << "messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration" << std::endl;
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
  appSample::UpdateDenm(ActionID_t actionid)
  {
    DENBasicService_error_t update_retval;
    denData data;
    std::string my_edge = m_client->TraCIAPI::vehicle.getRoadID (m_id);
    long my_edge_hash = ((std::hash<std::string>{}(my_edge)%900000000));
    long my_pos_on_edge = (m_client->TraCIAPI::vehicle.getLanePosition (m_id));

    data.setDenmMandatoryFields (compute_timestampIts(),my_edge_hash,my_pos_on_edge);
    update_retval = m_denService.appDENM_update (data,actionid);
    if(update_retval!=DENM_NO_ERROR)
      {
        NS_LOG_ERROR("Cannot update DENM. Error code: " << update_retval);
      }
    else
      {
        m_denm_sent++;
      }

    if(m_denm_intertime>0)
      {
        Simulator::Schedule (Seconds (m_denm_intertime), &appSample::UpdateDenm, this, actionid);
      }
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

    ActionID_t actionid;
    denData data;
    DENBasicService_error_t trigger_retval;

    /*
     * Insert the edge value and position on edge in the eventposition of DENM message.
     * As there are no fields, in a standard DENM, to code this information, we used this
     * workaround to include those values.
     * The module operation (%) is performed in order to obtain ASN.1 in-range values.
    */
    std::string my_edge = m_client->TraCIAPI::vehicle.getRoadID (m_id);
    long my_edge_hash = ((std::hash<std::string>{}(my_edge)%900000000));
    long my_pos_on_edge = (m_client->TraCIAPI::vehicle.getLanePosition (m_id));

    data.setDenmMandatoryFields (compute_timestampIts(),my_edge_hash,my_pos_on_edge);

    trigger_retval=m_denService.appDENM_trigger (data,actionid);
    if(trigger_retval!=DENM_NO_ERROR)
      {
        NS_LOG_ERROR("Cannot trigger DENM. Error code: " << trigger_retval);
      }
    else
      {
        m_denm_sent++;
      }

    if(m_denm_intertime>0)
      {
        Simulator::Schedule (Seconds (m_denm_intertime), &appSample::UpdateDenm, this, actionid);
      }
  }

  void
  appSample::receiveCAM (CAM_t *cam)
  {
    /* Implement CAM strategy here */
   m_cam_received++;

   if (!m_csv_name.empty ())
     {
       // messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration
       m_csv_ofstream_cam << cam->header.messageID << "," << cam->header.stationID << ",";
       m_csv_ofstream_cam << cam->cam.generationDeltaTime << "," << (double)cam->cam.camParameters.basicContainer.referencePosition.latitude/DOT_ONE_MICRO << ",";
       m_csv_ofstream_cam << (double)cam->cam.camParameters.basicContainer.referencePosition.longitude/DOT_ONE_MICRO << "," ;
       m_csv_ofstream_cam << (double)cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue/DECI << "," << (double)cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue/CENTI << ",";
       m_csv_ofstream_cam << (double)cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue/DECI << std::endl;
     }

   // Free the received CAM data structure
   ASN_STRUCT_FREE(asn_DEF_CAM,cam);
  }

  void
  appSample::receiveDENM (denData denm)
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
    long my_edge_hash = ((std::hash<std::string>{}(my_edge)%900000000));
    long my_pos_on_edge = (m_client->TraCIAPI::vehicle.getLanePosition (m_id));

    double denm_edge_hash = ((double) denm.getDenmMgmtLatitude ());
    double denm_pos_on_edge = ((double)denm.getDenmMgmtLongitude ());

    if (m_type!="emergency")
      {
        if (denm_edge_hash == my_edge_hash)
          {
            if (denm_pos_on_edge < my_pos_on_edge)
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
            else if (denm_pos_on_edge - 50 < my_pos_on_edge)
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
        long delay = timestamp - denm.getDenmMgmtReferenceTime();
        m_csv_ofstream_denm << denm.getDenmHeaderMessageID() << "," << denm.getDenmActionID().sequenceNumber << ",";
        m_csv_ofstream_denm << denm.getDenmMgmtReferenceTime() << "," << denm.getDenmMgmtDetectionTime ()<< ",";
        m_csv_ofstream_denm << denm.getDenmHeaderStationID() << "," << delay << std::endl;
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


}





