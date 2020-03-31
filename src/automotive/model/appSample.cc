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
#include <errno.h>

#include "appSample.h"
#include "v2v-CAM-DENM-sender.h"
#include <unordered_map>

namespace ns3
{

  NS_LOG_COMPONENT_DEFINE("appSample");

  NS_OBJECT_ENSURE_REGISTERED(appSample);

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
        .AddAttribute ("ASN",
            "If true, it uses ASN.1 to encode and decode CAMs and DENMs",
            BooleanValue(false),
            MakeBooleanAccessor (&appSample::m_asn),
            MakeBooleanChecker ())
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

    /* If it is an emergency vehicle, schedule a DENM send, and repeat it with frequency 2Hz */
    std::string my_type = m_client->TraCIAPI::vehicle.getVehicleClass (m_id);
    if (my_type=="emergency")
      {
         Simulator::Schedule (Seconds (1.0), &appSample::sendDENM, this);
      }

  }

  void
  appSample::StopApplication ()
  {
    NS_LOG_FUNCTION(this);
  }

  void
  appSample::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

  void
  appSample::sendDENM()
  {
    Ptr<CAMDENMSender> cam_sender_app = GetNode()->GetApplication (0)->GetObject<CAMDENMSender> ();
    if (m_asn)
      cam_sender_app->Populate_and_send_asn_denm ();
    else
      cam_sender_app->Populate_and_send_normal_denm ();

    Simulator::Schedule (Seconds (0.5), &appSample::sendDENM, this);
  }

  void
  appSample::receiveCAM (cam_field_t cam)
  {
    /* Implement CAM strategy here */
  }

  void
  appSample::receiveDENM (denm_field_t denm)
  {
    /* Implement DENM strategy here */

    /* In this case the vehicle that receives a DENM should check:
     * 1) That himself is not an emergency vehicle. In case it is, do nothing.
     * 2) That the DENM is sent by a vehicle that is in the same edge. To do so, the information extracted by m_client->TraCIAPI::vehicle.getRoadID (m_id)
     * is hashed using the function: int my_edge_hash = (int)std::hash<std::string>{}(my_edge)%10000 and included in the DENM. In case of ASN.1 format,
     * it is in the field "latitude", otherwise is the 2nd element. If the vehicle is not in the same edge, do nothing.
     * 3) In case the vehicle is in the same edge, check if it is already passed. To do so, the information from m_client->TraCIAPI::vehicle.getLanePosition (m_id)
     * is included in the DENM. In case of ASN.1 it is in the longitude field. In case it is already passed, do nothing.
     *
     * If all the control are passed, then the vehicle should slow down by setting its maximum speed to 60% of the original, then it should change lane,
     * by choosing the "rightmost" (i.e. with index 0), in order to facilitate the emergency vehicle takeover. For visualization purposes, it will change
     * color during this phase.
     */

    std::string my_edge = m_client->TraCIAPI::vehicle.getRoadID (m_id);
    int my_edge_hash = (int)std::hash<std::string>{}(my_edge)%10000;
    double my_edge_pos = m_client->TraCIAPI::vehicle.getLanePosition (m_id);

    if (m_type!="emergency")
      {
        if (my_edge_hash == denm.edge_hash)
          {
            if (denm.pos_on_edge < my_edge_pos)
              {
                m_client->TraCIAPI::vehicle.slowDown (m_id, m_max_speed*0.6, 10);
                m_client->TraCIAPI::vehicle.changeLane (m_id,0,10);
                libsumo::TraCIColor orange;
                orange.r=232;orange.g=126;orange.b=4;orange.a=255;
                m_client->TraCIAPI::vehicle.setColor (m_id,orange);
                m_change_color = Simulator::Schedule (Seconds (10.0), &appSample::ChangeColor, this);
              }
          }
      }
  }

  void
  appSample::ChangeColor ()
  {
    libsumo::TraCIColor normal;
    normal.r=255;normal.g=255;normal.b=0;normal.a=255;
    m_client->TraCIAPI::vehicle.setColor (m_id, normal);
  }
}





