#include "denBasicService.h"
#include "../asn1/asn_application.h"



namespace ns3 {

  template <typename T> int
  DENBasicService::asn_maybe_assign_optional_data(T *data, T **asn_structure) {
    if(data==NULL) {
        return 0;
      }

    *asn_structure = (T*)calloc(1, sizeof(T));

    if(*asn_structure == NULL)
        return -1;

    *(*asn_structure) = *data;

    return 1;
  }

  DENBasicService::DENBasicService(unsigned long fixed_stationid,long fixed_stationtype,Ptr<Socket> socket_tx,Ptr<Socket> socket_rx)
  {
    m_station_id = (StationID_t) fixed_stationid;
    m_stationtype = (StationType_t) fixed_stationtype;
    m_seq_number = 0;
    m_socket_tx = socket_tx;
    m_socket_rx = socket_rx;
  }

  DENBasicService_error_t
  DENBasicService::appDENM_trigger(denData data, ActionID_t &actionid)
  {
    denData::denDataManagement mgmt_data;
    //DENM_t *denm = (DENM_t*) calloc(1, sizeof(DENM_t));
    DENM_t denm;

    if(!data.isDenDataRight())
        return DENM_WRONG_DE_DATA;

    /* 1. If validity is expired return DENM_T_O_VALIDITY_EXPIRED */
    if (GetTimestampIts () > data.getDenmMgmtDetectionTime () + data.getDenmMgmtValidityDuration ())
        return DENM_T_O_VALIDITY_EXPIRED;

    /* 2. Assign unused actionID value */
    actionid.originatingStationID = m_station_id;
    actionid.sequenceNumber = m_seq_number;
    m_seq_number++;

    /* 3. Get Management Container */
    mgmt_data=data.getDenmMgmtData_asn_types ();

    /* 4. Transmission interval */
    if(asn_maybe_assign_optional_data<TransmissionInterval_t>(mgmt_data.transmissionInterval,&denm.denm.management.transmissionInterval)==-1)
        return DENM_ALLOC_ERROR;

    /* 5. Set all the containers [to be continued] */
    /* Header */
    denm.header.messageID = FIX_DENMID;
    denm.header.protocolVersion = FIX_PROT_VERS;
    denm.header.stationID = m_station_id;

    /* Management Container */
    denm.denm.management.actionID = actionid;
    denm.denm.management.eventPosition = mgmt_data.eventPosition;
    denm.denm.management.detectionTime = mgmt_data.detectionTime;
    if(asn_maybe_assign_optional_data<RelevanceDistance_t>(mgmt_data.relevanceDistance,&denm.denm.management.relevanceDistance)==-1)
        return DENM_ALLOC_ERROR;
    if(asn_maybe_assign_optional_data<RelevanceTrafficDirection_t>(mgmt_data.relevanceTrafficDirection,&denm.denm.management.relevanceTrafficDirection)==-1)
        return DENM_ALLOC_ERROR;
    if(asn_maybe_assign_optional_data<Termination_t>(mgmt_data.termination,&denm.denm.management.termination)==-1)
        return DENM_ALLOC_ERROR;

    memset(&denm.denm.management.referenceTime, 0, sizeof(denm.denm.management.referenceTime));
    asn_long2INTEGER(&denm.denm.management.referenceTime, GetTimestampIts ());

    denm.denm.management.stationType=m_stationtype;
    /* Add the other containers */

    //originatingITSSTableEntry entry(denm, originatingITSSTableEntry::STATE_ACTIVE);

    /* 6. 7. Construct DENM and pass it to the lower layers (now UDP, in the future BTP and GeoNetworking, then UDP) */
    /** Encoding **/
    asn_encode_to_new_buffer_result_t encode_result = asn_encode_to_new_buffer(NULL,ATS_UNALIGNED_BASIC_PER,&asn_DEF_DENM, &denm);
    if (encode_result.result.encoded==-1)
      {
        return DENM_ASN1_UPER_ENC_ERROR;
      }

    Ptr<Packet> packet = Create<Packet> ((uint8_t*) encode_result.buffer, encode_result.result.encoded+1);
    m_socket_tx->Send (packet);

    /* Create a new class, called OriginatingITSSMessageTable, in which u save all
     * the needed elements, including timers.*/
    //T_O_Validity.SetFunction();
    return DENM_NO_ERROR;
  }

  long
  DENBasicService::GetTimestampIts()
  {
    /* To get millisec since  2004-01-01T00:00:00:000Z */
    auto time = std::chrono::system_clock::now(); // get the current time
    auto since_epoch = time.time_since_epoch(); // get the duration since epoch
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch); // convert it in millisecond since epoch

    long elapsed_since_2004 = millis.count() - TIME_SHIFT; // in TIME_SHIFT we saved the millisec from epoch to 2004-01-01
    return elapsed_since_2004;
  }



}
