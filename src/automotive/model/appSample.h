#ifndef APPSAMPLE_H
#define APPSAMPLE_H

#include "ns3/traci-client.h"
#include "ns3/application.h"
#include <unordered_map>

#include "v2v-CAM-DENM-sender.h"

namespace ns3 {

class appSample : public Application
{
public:

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  appSample ();

  virtual ~appSample ();

  void StopApplicationNow ();

  /**
   * \brief Handle a CAM reception.
   *
   * This function is called by the application receiving the CAM.
   *
   * \param the struct containing the info of the packet that was received.
   */
  void receiveCAM (ca_data_t cam);

  /**
   * \brief Handle a DENM reception.
   *
   * This function is called by the application receiving the DENM.
   *
   * \param the struct containing the info of the packet that was received.
   */
  void receiveDENM (den_data_t denm);

protected:
  virtual void DoDispose (void);

private:

  /**
   * \brief chenge color of the vehicle.
   *
   * This function just change the color of the vehicle, turning it to yellow.
   *
   */
  void ChangeColor ();
  /**
   * @brief This function compute the timestamps
  */
  struct timespec compute_timestamp();

  /**
   * @brief This function compute the milliseconds elapsed from 2004-01-01
  */
  long compute_timestampIts ();

  void TriggerCam(void);
  void TriggerDenm(void);

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  Ptr<TraciClient> m_client; //!< TraCI client
  std::string m_id; //!< vehicle id
  std::string m_type; //!< vehicle type
  bool m_lon_lat; //!< Use LonLat instead of XY
  bool m_asn; //!< To decide if ASN.1 is used
  double m_max_speed; //!< To save initial veh max speed
  bool m_send_denm;  //!< To decide if DENM dissemination is active or not
  bool m_send_cam;  //!< To decide if CAM dissemination is active or not
  double m_cam_intertime; //!< Time between two consecutives CAMs
  double m_denm_intertime; //!< Time between two consecutives CAMs
  bool m_print_summary; //!< To print a small summary when vehicle leaves the simulation
  bool m_already_print; //!< To avoid printing two summary
  bool m_real_time; //!< To decide wheter to use realtime scheduler

  /* Counters */
  int m_cam_sent;
  int m_cam_received;
  int m_denm_sent;
  int m_denm_received;

  EventId m_change_color; //!< Event to change the vehicle color
  EventId m_send_denm_ev; //!< Event to send the DENM
  EventId m_send_cam_ev; //!< Event to send the CAM
};

} // namespace ns3

#endif /* APP_H */

