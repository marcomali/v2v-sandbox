#ifndef DENBASICSERVICE_H
#define DENBASICSERVICE_H

#include "asn_utils.h"
#include "denData.h"
#include "ITSSOriginatingTableEntry.h"
#include "ITSSReceivingTableEntry.h"
#include "ns3/core-module.h"
#include "ns3/socket.h"
#include <functional>

//Epoch time at 2004-01-01
#define TIME_SHIFT 1072915200000

#define V_O_VALIDITY_INDEX 0
#define T_REPETITION_INDEX 1
#define T_REPETITION_DURATION_INDEX 2

namespace ns3 {

  typedef enum {
    DENM_NO_ERROR=0,
    DENM_ATTRIBUTES_UNSET=1,
    DENM_ALLOC_ERROR=2,
    DENM_WRONG_DE_DATA=3,
    DENM_WRONG_TABLE_DATA=4,
    DENM_T_O_VALIDITY_EXPIRED=5,
    DENM_ASN1_UPER_ENC_ERROR=6,
    DENM_UNKNOWN_ACTIONID=7,
    DENM_UNKNOWN_ACTIONID_RECEIVING=8,
    DENM_UNKNOWN_ACTIONID_ORIGINATING=9,
    DENM_NON_ACTIVE_ACTIONID_RECEIVING=10,
    DENM_NON_ACTIVE_ACTIONID_ORIGINATING=11
  } DENBasicService_error_t;

  class DENBasicService
  {
    public:
    DENBasicService();
    DENBasicService(unsigned long fixed_stationid,long fixed_stationtype,Ptr<Socket> socket_tx,Ptr<Socket> socket_rx);

    void addDENRxCallback(std::function<void(denData)> rx_callback) {m_DENReceiveCallback=rx_callback;}

    static long GetTimestampIts (void);

    DENBasicService_error_t appDENM_trigger(denData data, ActionID_t &actionid);
    DENBasicService_error_t appDENM_update(denData data, const ActionID_t actionid);
    DENBasicService_error_t appDENM_termination(denData data, const ActionID_t actionid);
    void receiveDENM(Ptr<Socket> socket);

    void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype) {m_station_id=fixed_stationid; m_stationtype=fixed_stationtype;}
    void setStationID(unsigned long fixed_stationid) {m_station_id=fixed_stationid;}
    void setStationType(long fixed_stationtype) {m_stationtype=fixed_stationtype;}

    void setSockets(Ptr<Socket> socket_tx,Ptr<Socket> socket_rx) {m_socket_tx=socket_tx; m_socket_rx=socket_rx;}
    void setSocketTx(Ptr<Socket> socket_tx) {m_socket_tx=socket_tx;}
    void setSocketRx(Ptr<Socket> socket_rx) {m_socket_rx=socket_rx;}

    /* Cleanup function - always call this before terminating the simulation */
    void cleanup(void);

  private:
    bool CheckMainAttributes(void);

    DENBasicService_error_t fillDENM(DENM_t *denm, denData &data, const ActionID_t actionID,long referenceTimeLong);

    template<typename MEM_PTR> void setDENTimer(Timer &timer,Time delay,MEM_PTR callback_fcn,ActionID_t actionID);

    void T_O_ValidityStop(ActionID_t entry_actionid);
    void T_RepetitionDurationStop(ActionID_t entry_actionid);
    void T_RepetitionStop(ActionID_t entry_actionid);

    void T_R_ValidityStop(ActionID_t entry_actionid);

    template <typename T> static int asn_maybe_assign_optional_data(T *data, T **asn_structure);

    std::function<void(denData)> m_DENReceiveCallback;

    uint16_t m_port;
    bool m_real_time;
    std::string m_model;

    StationID_t m_station_id;
    StationType_t m_stationtype;
    uint16_t m_seq_number;

    Ptr<Socket> m_socket_tx; // Socket TX
    Ptr<Socket> m_socket_rx; // Socket RX

    std::map<std::pair<unsigned long,long>,ITSSOriginatingTableEntry> m_originatingITSSTable;
    std::map<std::pair<unsigned long,long>,ITSSReceivingTableEntry> m_receivingITSSTable;

    std::map<std::pair<unsigned long,long>,std::tuple<Timer,Timer,Timer>> m_originatingTimerTable;
    std::map<std::pair<unsigned long,long>,Timer> m_T_R_Validity_Table;

  };

}


#endif // DENBASICSERVICE_H
