#ifndef V2V_CAM_DENM_SENDER_H
#define V2V_CAM_DENM_SENDER_H

#include "ns3/socket.h"
#include "utils.h"
#include "ns3/appSample.h"
#include "ns3/asn_utils.h"

#include <chrono>

namespace ns3 {

class Socket;
class Packet;

class CAMDENMSender : public Application
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  CAMDENMSender ();

  virtual ~CAMDENMSender ();

  void StopApplicationNow ();

  /**
   * @brief This function is used to send CAMs - called from APP
   */
  void SendCam(ca_data_t cam);

  /**
   * @brief This function is used to send DENMs - called from APP
   */
  int SendDenm(den_data_t denm);

protected:
  virtual void DoDispose (void);

private:

  virtual void StartApplication (void);
  virtual void StopApplication (void);
  
  /**
   * @brief This function is to encode and send a DENM using ASN.1
  */
  int Populate_and_send_asn_denm(den_data_t denm);

  /**
   * @brief This function is to encode and send a DENM in plain text
  */
  int Populate_and_send_normal_denm(den_data_t denm);

  /**
   * @brief This function is to encode and send a CAM using ASN.1
  */
  void Populate_and_send_asn_cam(ca_data_t cam);

  /**
   * @brief This function is to send a CAM in plain text
  */
  void Populate_and_send_normal_cam(ca_data_t cam);
  /**
   * @brief This function is to decode a DENM using ASN.1
  */
  void Decode_asn_denm(uint8_t *buffer,uint32_t size);
  /**
   * @brief This function is to decode a DENM in plain text
  */
  void Decode_normal_denm(uint8_t *buffer);
  /**
   * @brief This function is to decode a CAM using ASN.1
  */
  void Decode_asn_cam(uint8_t *buffer,uint32_t size);
  /**
   * @brief This function is to decode a CAM in plain text
  */
  void Decode_normal_cam(uint8_t *buffer);
  /**
   * \brief Handle a packet reception.
   *
   * This function is called by lower layers.
   *
   * \param the socket the packet was received to.
   */
  void HandleRead (Ptr<Socket> socket);

  /**
   * @brief This function compute the timestamps
  */
  struct timespec compute_timestamp();

  Ptr<Socket> m_socket; //!< Socket TX
  Ptr<Socket> m_socket2; //!< Soc  long m_sequence;
  uint16_t m_port;  //!< Port on which client will listen for traffic information
  bool m_real_time; //!< To decide if using realtime scheduler
  bool m_asn; //!< To decide if ASN.1 is used
  std::string m_server_addr; //!< Remote addr

  long m_sequence; //!< Sequence number of DENMs
  long m_actionId; //!< ActionID number of DENMs

  std::string m_model; //!< Communication Model (possible values: 80211p and cv2x)

  Ipv4Address m_ipAddress;

};

} // namespace ns3

#endif /* V2V_CAM_SENDER_H */

