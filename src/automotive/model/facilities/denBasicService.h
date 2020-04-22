#ifndef DENBASICSERVICE_H
#define DENBASICSERVICE_H

#include "asn_utils.h"
#include "denData.h"
//#include "originatingITSSTableEntry.h"
#include "ns3/application.h"
#include "ns3/core-module.h"
#include "ns3/socket.h"

//Epoch time at 2004-01-01
#define TIME_SHIFT 1072915200000

namespace ns3 {

  typedef enum {
    DENM_NO_ERROR,
    DENM_ALLOC_ERROR,
    DENM_WRONG_DE_DATA,
    DENM_T_O_VALIDITY_EXPIRED,
    DENM_ASN1_UPER_ENC_ERROR
  } DENBasicService_error_t;

  class DENBasicService
  {
    public:
    DENBasicService(unsigned long fixed_stationid,long fixed_stationtype,Ptr<Socket> socket_tx,Ptr<Socket> socket_rx);

    static long GetTimestampIts (void);

    DENBasicService_error_t appDENM_trigger(denData data, ActionID_t &actionid);
    DENBasicService_error_t appDENM_update(denData data);
    DENBasicService_error_t appDENM_terminate(denData data);

  private:
    template <typename T> static int asn_maybe_assign_optional_data(T *data, T **asn_structure);

    uint16_t m_port;
    bool m_real_time;
    std::string m_model;

    StationID_t m_station_id;
    StationType_t m_stationtype;
    uint16_t m_seq_number;

    Ptr<Socket> m_socket_tx; // Socket TX
    Ptr<Socket> m_socket_rx; // Socket RX

  };

}


#endif // DENBASICSERVICE_H
