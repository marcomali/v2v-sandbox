#ifndef DENBASICSERVICE_H
#define DENBASICSERVICE_H

#include "asn_utils.h"
#include "denData.h"
#include "ITSSTableEntry.h"
#include "ns3/core-module.h"
#include "ns3/socket.h"

//Epoch time at 2004-01-01
#define TIME_SHIFT 1072915200000

#define V_O_VALIDITY_INDEX 0
#define T_REPETITION_INDEX 1
#define T_REPETITION_DURATION_INDEX 2

namespace ns3 {

  typedef enum {
    DENM_NO_ERROR,
    DENM_ALLOC_ERROR,
    DENM_WRONG_DE_DATA,
    DENM_WRONG_TABLE_DATA,
    DENM_T_O_VALIDITY_EXPIRED,
    DENM_ASN1_UPER_ENC_ERROR,
    DENM_UNKNOWN_ACTIONID,
    DENM_UNKNOWN_ACTIONID_RECEIVING,
    DENM_UNKNOWN_ACTIONID_ORIGINATING,
    DENM_NON_ACTIVE_ACTIONID_RECEIVING,
    DENM_NON_ACTIVE_ACTIONID_ORIGINATING
  } DENBasicService_error_t;

  class DENBasicService
  {
    public:
    DENBasicService(unsigned long fixed_stationid,long fixed_stationtype,Ptr<Socket> socket_tx,Ptr<Socket> socket_rx);

    static long GetTimestampIts (void);

    DENBasicService_error_t appDENM_trigger(denData data, ActionID_t &actionid);
    DENBasicService_error_t appDENM_update(denData data, const ActionID_t actionid);
    DENBasicService_error_t appDENM_termination(denData data, const ActionID_t actionid);

  private:
    DENBasicService_error_t fillDENM(DENM_t &denm, denData &data, const ActionID_t actionID,long referenceTimeLong);

    template<typename MEM_PTR> void setDENTimer(Timer timer,Time delay,MEM_PTR callback_fcn,ActionID_t actionID);

    void T_O_ValidityStop(ActionID_t entry_actionid);
    void T_RepetitionDurationStop(ActionID_t entry_actionid);
    void T_RepetitionStop(ActionID_t entry_actionid);

    template <typename T> static int asn_maybe_assign_optional_data(T *data, T **asn_structure);

    uint16_t m_port;
    bool m_real_time;
    std::string m_model;

    StationID_t m_station_id;
    StationType_t m_stationtype;
    uint16_t m_seq_number;

    Ptr<Socket> m_socket_tx; // Socket TX
    Ptr<Socket> m_socket_rx; // Socket RX

    std::map<std::pair<unsigned long,long>,ITSSTableEntry> m_originatingITSSTable;
    std::map<std::pair<unsigned long,long>,ITSSTableEntry> m_receivingITSSTable;

    std::map<std::pair<unsigned long,long>,std::tuple<Timer*,Timer*,Timer*>> m_timerTable;
  };

}


#endif // DENBASICSERVICE_H
