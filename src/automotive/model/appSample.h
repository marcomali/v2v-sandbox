#ifndef APPSAMPLE_H
#define APPSAMPLE_H

#include "ns3/traci-client.h"
#include "ns3/application.h"

typedef struct cam_field_t
{
  std::pair<double,double> pos;
  double speed;
  double acceleration;
  double angle;
} m_cam_field;

typedef struct denm_field_t
{
  int edge_hash;
  double pos_on_edge;
} m_denm_field;


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
  void receiveCAM (cam_field_t cam);

  /**
   * \brief Handle a DENM reception.
   *
   * This function is called by the application receiving the DENM.
   *
   * \param the struct containing the info of the packet that was received.
   */
  void receiveDENM (denm_field_t denm);

protected:
  virtual void DoDispose (void);

private:

  /**
   * \brief send a DENM.
   *
   * This function is calls the method in v2v-CAM-DENM-sender to send the DENM.
   *
   */
  void sendDENM();

  /**
   * \brief chenge color of the vehicle.
   *
   * This function just change the color of the vehicle, turning it to yellow.
   *
   */
  void ChangeColor ();

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  Ptr<TraciClient> m_client; //!< TraCI client
  std::string m_id; //!< vehicle id
  std::string m_type; //!< vehicle type
  bool m_lon_lat; //!< Use LonLat instead of XY
  bool m_asn; //!< To decide if ASN.1 is used
  double m_max_speed; //!< To save initial veh max speed


  EventId m_change_color; //!< Event to change the vehicle color

};

} // namespace ns3

#endif /* APP_H */

