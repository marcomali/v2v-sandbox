#include "denBasicService.h"
#include "../asn1/asn_application.h"

namespace ns3 {
  DENBasicService::DENBasicService()
  {
    m_station_id = ULONG_MAX;
    m_stationtype = LONG_MAX;
    m_seq_number = 0;
    m_socket_tx = NULL;
    m_socket_rx = NULL;
  }

  bool
  DENBasicService::CheckMainAttributes()
  {
    return m_station_id!=ULONG_MAX && m_stationtype!=LONG_MAX && m_socket_tx!=NULL && m_socket_rx!=NULL;
  }

    template<typename MEM_PTR> void
     DENBasicService::setDENTimer(Timer &timer,Time delay,MEM_PTR callback_fcn,ActionID_t actionID) {
      if(timer.IsRunning ())
        {
           timer.Cancel();
        }

      timer.SetFunction(callback_fcn,this);
      timer.SetArguments(actionID);
      timer.SetDelay (delay);

      timer.Schedule ();
    }

  DENBasicService_error_t
  DENBasicService::fillDENM(DENM_t *denm, denData &data, const ActionID_t actionID,long referenceTimeLong)
  {
    denData::denDataManagement mgmt_data;

    /* 1. Get Management Container */
    mgmt_data=data.getDenmMgmtData_asn_types ();

    /* 2. Transmission interval */
    if(asn_maybe_assign_optional_data<TransmissionInterval_t>(mgmt_data.transmissionInterval,&denm->denm.management.transmissionInterval)==-1)
        return DENM_ALLOC_ERROR;

    /* 3. Set all the containers [to be continued] */
    /* Header */
    denm->header.messageID = FIX_DENMID;
    denm->header.protocolVersion = FIX_PROT_VERS;
    denm->header.stationID = m_station_id;

    /* Management Container */
    denm->denm.management.actionID = actionID;
    denm->denm.management.eventPosition = mgmt_data.eventPosition;
    denm->denm.management.detectionTime = mgmt_data.detectionTime;
    if(asn_maybe_assign_optional_data<RelevanceDistance_t>(mgmt_data.relevanceDistance,&denm->denm.management.relevanceDistance)==-1)
        return DENM_ALLOC_ERROR;
    if(asn_maybe_assign_optional_data<RelevanceTrafficDirection_t>(mgmt_data.relevanceTrafficDirection,&denm->denm.management.relevanceTrafficDirection)==-1)
        return DENM_ALLOC_ERROR;
    if(asn_maybe_assign_optional_data<Termination_t>(mgmt_data.termination,&denm->denm.management.termination)==-1)
        return DENM_ALLOC_ERROR;

    memset(&(denm->denm.management.referenceTime), 0, sizeof(denm->denm.management.referenceTime));
    asn_long2INTEGER(&(denm->denm.management.referenceTime), referenceTimeLong);

    denm->denm.management.stationType=m_stationtype;

    /* Add the other containers */

    return DENM_NO_ERROR;
  }

  template <typename T> int
  DENBasicService::asn_maybe_assign_optional_data(T *data, T **asn_structure)
  {
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
    DENBasicService_error_t fillDENM_rval=DENM_NO_ERROR;
    DENM_t *denm;

    if(!CheckMainAttributes ())
      {
        return DENM_ATTRIBUTES_UNSET;
      }

    if(!data.isDenDataRight())
        return DENM_WRONG_DE_DATA;

    denm=(DENM_t*) calloc(1, sizeof(DENM_t));
    if(denm==NULL)
      {
        return DENM_ALLOC_ERROR;
      }

    /* 1. If validity is expired return DENM_T_O_VALIDITY_EXPIRED */
    if (GetTimestampIts () > data.getDenmMgmtDetectionTime () + data.getDenmMgmtValidityDuration ())
        return DENM_T_O_VALIDITY_EXPIRED;

    /* 2. Assign unused actionID value */
    actionid.originatingStationID = m_station_id;
    actionid.sequenceNumber = m_seq_number;

    std::pair <unsigned long, long> map_index = std::make_pair((unsigned long)m_station_id,(long)m_seq_number);

    m_seq_number++;

    /* 3. 4. 5. Manage Transmission interval and fill DENM */
    fillDENM_rval=fillDENM(denm,data,actionid,GetTimestampIts ());

    if(fillDENM_rval!=DENM_NO_ERROR)
      {
        return fillDENM_rval;
      }

    /* 6. 7. Construct DENM and pass it to the lower layers (now UDP, in the future BTP and GeoNetworking, then UDP) */
    /** Encoding **/
    asn_encode_to_new_buffer_result_t encode_result = asn_encode_to_new_buffer(NULL,ATS_UNALIGNED_BASIC_PER,&asn_DEF_DENM, denm);
    if (encode_result.result.encoded==-1)
      {
        return DENM_ASN1_UPER_ENC_ERROR;
      }

    Ptr<Packet> packet = Create<Packet> ((uint8_t*) encode_result.buffer, encode_result.result.encoded+1);
    m_socket_tx->Send (packet);

    /* 8. 9. Create an entry in the originating ITS-S message table and set the state to ACTIVE and start the T_O_Validity timer. */
    /* As all the timers are stored, in this case, for each entry in the table, we have to set them before saving the new entry to a map, which is done as last operation. */
    /* We are basically adding the current entry, containing the already UPER encoded DENM packet, to a map of <ActionID,ITSSOriginatingTableEntry> */
    ITSSOriginatingTableEntry entry(*packet, ITSSOriginatingTableEntry::STATE_ACTIVE,actionid);

    m_originatingTimerTable.emplace(map_index,std::tuple<Timer,Timer,Timer>());

    DENBasicService::setDENTimer(std::get<V_O_VALIDITY_INDEX>(m_originatingTimerTable[map_index]),MilliSeconds(data.getDenmMgmtValidityDuration ()),&DENBasicService::T_O_ValidityStop,actionid);

    /* 10. Calculate and start timers T_RepetitionDuration and T_Repetition when both parameters in denData are > 0 */
    if(data.getDenmRepetitionDuration ()>0 && data.getDenmRepetitionInterval ()>0)
      {
        DENBasicService::setDENTimer(std::get<T_REPETITION_INDEX>(m_originatingTimerTable[map_index]),MilliSeconds(data.getDenmRepetitionInterval()),&DENBasicService::T_RepetitionStop,actionid);
        DENBasicService::setDENTimer(std::get<T_REPETITION_DURATION_INDEX>(m_originatingTimerTable[map_index]),MilliSeconds(data.getDenmRepetitionDuration ()),&DENBasicService::T_RepetitionDurationStop,actionid);
      }

    /* 11. Finally create the entry after starting all the timers (it refers to the step '8' of appDENM_trigger in ETSI EN302 637-3 V1.3.1 */

    m_originatingITSSTable[map_index]=entry;

    /* 12. Send actionID to the requesting ITS-S application. This is requested by the standard, but we are already reporting the actionID using &actionID */

    return DENM_NO_ERROR;
  }

  DENBasicService_error_t
  DENBasicService::appDENM_update(denData data, const ActionID_t actionid)
  {
    DENBasicService_error_t fillDENM_rval=DENM_NO_ERROR;
    std::pair <unsigned long, long> map_index = std::make_pair((unsigned long)actionid.originatingStationID,(long)actionid.sequenceNumber);

    DENM_t *denm;

    if(!CheckMainAttributes ())
      {
        return DENM_ATTRIBUTES_UNSET;
      }

    denm=(DENM_t*) calloc(1, sizeof(DENM_t));
    if(denm==NULL)
      {
        return DENM_ALLOC_ERROR;
      }

    /* 1. If validity is expired return DENM_T_O_VALIDITY_EXPIRED */
    if (GetTimestampIts () > data.getDenmMgmtDetectionTime () + data.getDenmMgmtValidityDuration ())
        return DENM_T_O_VALIDITY_EXPIRED;

    /* 2. Compare actionID in the application request with entries in the originating ITS-S message table (i.e. m_originatingITSSTable, implemented as a map) */
    /* Gather also the proper entry in the table, if available. */
    std::map<std::pair<unsigned long,long>, ITSSOriginatingTableEntry>::iterator entry_map_it = m_originatingITSSTable.find(map_index);
    if (entry_map_it != m_originatingITSSTable.end())
      {
        return DENM_UNKNOWN_ACTIONID;
      }

    /* 3. Stop T_O_Validity, T_RepetitionDuration and T_Repetition (if they were started - this check is already performed by the setTimer* methods) */
    std::get<V_O_VALIDITY_INDEX>(m_originatingTimerTable[map_index]).Cancel();
    std::get<T_REPETITION_INDEX>(m_originatingTimerTable[map_index]).Cancel();
    std::get<T_REPETITION_DURATION_INDEX>(m_originatingTimerTable[map_index]).Cancel();

    /* 4. 5. 6. Manage transmission interval, reference time and fill DENM */
    fillDENM_rval=fillDENM(denm,data,actionid,GetTimestampIts ());

    if(fillDENM_rval!=DENM_NO_ERROR)
      {
        return fillDENM_rval;
      }

    /* 7. 8. Construct DENM and pass it to the lower layers (now UDP, in the future BTP and GeoNetworking, then UDP) */
    /** Encoding **/
    asn_encode_to_new_buffer_result_t encode_result = asn_encode_to_new_buffer(NULL,ATS_UNALIGNED_BASIC_PER,&asn_DEF_DENM, &denm);
    if (encode_result.result.encoded==-1)
      {
        return DENM_ASN1_UPER_ENC_ERROR;
      }

    Ptr<Packet> packet = Create<Packet> ((uint8_t*) encode_result.buffer, encode_result.result.encoded+1);
    m_socket_tx->Send (packet);

    /* 9. Update the entry in the originating ITS-S message table. */
    entry_map_it->second.setDENMPacket(*packet);

    /* 10. Start timer T_O_Validity. */
    DENBasicService::setDENTimer(std::get<V_O_VALIDITY_INDEX>(m_originatingTimerTable[map_index]),MilliSeconds(data.getDenmMgmtValidityDuration ()),&DENBasicService::T_O_ValidityStop,actionid);

    /* 11. Calculate and start timers T_RepetitionDuration and T_Repetition when both parameters in denData are > 0 */
    if(data.getDenmRepetitionDuration ()>0 && data.getDenmRepetitionInterval ()>0)
      {
        DENBasicService::setDENTimer(std::get<T_REPETITION_INDEX>(m_originatingTimerTable[map_index]),MilliSeconds(data.getDenmRepetitionInterval()),&DENBasicService::T_RepetitionStop,actionid);
        DENBasicService::setDENTimer(std::get<T_REPETITION_DURATION_INDEX>(m_originatingTimerTable[map_index]),MilliSeconds(data.getDenmRepetitionDuration ()),&DENBasicService::T_RepetitionDurationStop,actionid);
      }

    return DENM_NO_ERROR;
  }

  DENBasicService_error_t
  DENBasicService::appDENM_termination(denData data, const ActionID_t actionid)
  {
    DENM_t *denm;
    uint8_t termination=0;
    Termination_t asn_termination;
    long referenceTime;

    DENBasicService_error_t fillDENM_rval=DENM_NO_ERROR;

    if(!CheckMainAttributes ())
      {
        return DENM_ATTRIBUTES_UNSET;
      }

    denm=(DENM_t*) calloc(1, sizeof(DENM_t));
    if(denm==NULL)
      {
        return DENM_ALLOC_ERROR;
      }

    std::pair <unsigned long, long> map_index = std::make_pair((unsigned long)actionid.originatingStationID,(long)actionid.sequenceNumber);

    /* 1. If validity is expired return DENM_T_O_VALIDITY_EXPIRED */
    if (GetTimestampIts () > data.getDenmMgmtDetectionTime () + data.getDenmMgmtValidityDuration ())
        return DENM_T_O_VALIDITY_EXPIRED;

    /* 2. Compare actionID in the application request with entries in the originating ITS-S message table and the receiving ITS-S message table */
    std::map<std::pair<unsigned long,long>, ITSSOriginatingTableEntry>::iterator entry_originating_table = m_originatingITSSTable.find(map_index);
    /* 2a. If actionID exists in the originating ITS-S message table and the entry state is ACTIVE, then set termination to isCancellation.*/
    if (entry_originating_table != m_originatingITSSTable.end())
      {
        return DENM_UNKNOWN_ACTIONID_ORIGINATING;
      }
    else if(entry_originating_table->second.getStatus()==ITSSOriginatingTableEntry::STATE_ACTIVE)
      {
        asn_termination=Termination_isCancellation;
        if(asn_maybe_assign_optional_data<Termination_t>(&asn_termination,&denm->denm.management.termination)==-1)
            return DENM_ALLOC_ERROR;
        else
          termination=0;
      }
    else
      {
        return DENM_NON_ACTIVE_ACTIONID_ORIGINATING;
      }

    /* 2b. If actionID exists in the receiving ITS-S message table and the entry state is ACTIVE, then set termination to isNegation.*/
    std::map<std::pair<unsigned long,long>, ITSSReceivingTableEntry>::iterator entry_receiving_table = m_receivingITSSTable.find(map_index);
    if (entry_receiving_table != m_receivingITSSTable.end())
      {
        return DENM_UNKNOWN_ACTIONID_RECEIVING;
      }
    else if(entry_receiving_table->second.getStatus()==ITSSReceivingTableEntry::STATE_ACTIVE)
      {
        asn_termination=Termination_isNegation;
        if(asn_maybe_assign_optional_data<Termination_t>(&asn_termination,&denm->denm.management.termination)==-1)
            return DENM_ALLOC_ERROR;
        else
          termination=1;
      }
    else
      {
        return DENM_NON_ACTIVE_ACTIONID_RECEIVING;
      }

    if(termination==1)
      {
        referenceTime=entry_receiving_table->second.getReferenceTime();

        if(referenceTime==-1)
          {
            return DENM_WRONG_TABLE_DATA;
          }
      }
    else
      {
        referenceTime=GetTimestampIts ();
      }

    /* 3. Set referenceTime to current time (for termination = 0) or to the receiving table entry's reference time (for termination = 1) and fill DENM */
    fillDENM_rval=fillDENM(denm,data,actionid,referenceTime);
    if(fillDENM_rval!=DENM_NO_ERROR)
      {
        return fillDENM_rval;
      }

    /* 4a. Gather the proper timers from the timers table */
    std::map<std::pair<unsigned long,long>, std::tuple<Timer,Timer,Timer>>::iterator entry_timers_table = m_originatingTimerTable.find(map_index);

    /* 4b. Stop T_O_Validity, T_RepetitionDuration and T_Repetition (if they were started - this check is already performed by the setTimer* methods) */
    std::get<V_O_VALIDITY_INDEX>(entry_timers_table->second).Cancel();
    std::get<T_REPETITION_INDEX>(entry_timers_table->second).Cancel();
    std::get<T_REPETITION_DURATION_INDEX>(entry_timers_table->second).Cancel();

    /* 5. Construct DENM and pass it to the lower layers (now UDP, in the future BTP and GeoNetworking, then UDP) */
    /** Encoding **/
    asn_encode_to_new_buffer_result_t encode_result = asn_encode_to_new_buffer(NULL,ATS_UNALIGNED_BASIC_PER,&asn_DEF_DENM, &denm);
    if (encode_result.result.encoded==-1)
      {
        return DENM_ASN1_UPER_ENC_ERROR;
      }

    Ptr<Packet> packet = Create<Packet> ((uint8_t*) encode_result.buffer, encode_result.result.encoded+1);
    m_socket_tx->Send (packet);

    /* 6a. If termination is set to 1, create an entry in the originating ITS-S message table and set the state to NEGATED. */
    if(termination==1)
      {
        ITSSOriginatingTableEntry entry(*packet, ITSSOriginatingTableEntry::STATE_NEGATED,actionid);
        m_originatingITSSTable[map_index]=entry;
      }
    /* 6b. If termination is set to 0, update the entry in the originating ITS-S message table and set the state to CANCELLED. */
    else
      {
        entry_originating_table->second.setDENMPacket(*packet);
        entry_originating_table->second.setStatus(ITSSOriginatingTableEntry::STATE_CANCELLED);
      }

    /* 7. Start timer T_O_Validity. */
    DENBasicService::setDENTimer(std::get<V_O_VALIDITY_INDEX>(entry_timers_table->second),MilliSeconds(data.getDenmMgmtValidityDuration ()),&DENBasicService::T_O_ValidityStop,actionid);

    /* 8. Calculate and start timers T_RepetitionDuration and T_Repetition when both parameters in denData are > 0 */
    if(data.getDenmRepetitionDuration ()>0 && data.getDenmRepetitionInterval ()>0)
      {
        DENBasicService::setDENTimer(std::get<T_REPETITION_INDEX>(entry_timers_table->second),MilliSeconds(data.getDenmRepetitionInterval()),&DENBasicService::T_RepetitionStop,actionid);
        DENBasicService::setDENTimer(std::get<T_REPETITION_DURATION_INDEX>(entry_timers_table->second),MilliSeconds(data.getDenmRepetitionDuration ()),&DENBasicService::T_RepetitionDurationStop,actionid);
      }

    return DENM_NO_ERROR;
  }

  void
  DENBasicService::receiveDENM(Ptr<Socket> socket)
  {
    Ptr<Packet> packet;
    DENM_t *decoded_denm;
    denData den_data;
    Address from;
    ValidityDuration_t validityDuration;
    ActionID_t actionID;
    long detectionTime_long;
    long referenceTime_long;
    std::pair <unsigned long, long> map_index;

    packet = socket->RecvFrom (from);

    uint8_t *buffer = new uint8_t[packet->GetSize ()];
    packet->CopyData (buffer, packet->GetSize ()-1);

    if(!CheckMainAttributes ())
      {
        NS_FATAL_ERROR("DENBasicService has unset parameters. Cannot receive any data.");
        return;
      }

    /* Try to check if the received packet is really a DENM */
    if (buffer[1]!=FIX_DENMID)
      {
        return;
      }

    /** Decoding **/
    void *decoded_=NULL;
    asn_dec_rval_t decode_result;

    do {
      decode_result = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_DENM, &decoded_, buffer, packet->GetSize ()-1);
    } while(decode_result.code==RC_WMORE);

    if(decode_result.code!=RC_OK || decoded_==NULL) {
        return;
      }

    decoded_denm = (DENM_t *) decoded_;

    /* Compute T_R_Validity expiration time */

    validityDuration = decoded_denm->denm.management.validityDuration != NULL ? *(decoded_denm->denm.management.validityDuration) : DEN_DEFAULT_VALIDITY_MS;
    asn_INTEGER2long (&decoded_denm->denm.management.detectionTime,&detectionTime_long);
    asn_INTEGER2long (&decoded_denm->denm.management.referenceTime,&referenceTime_long);

    /* 1. If validity is expired return without performing further steps */
    if (GetTimestampIts () > detectionTime_long + (long)validityDuration)
        return;

    /* Lookup entries in the receiving ITS-S message table with the received actionID */
    actionID=decoded_denm->denm.management.actionID;
    map_index = std::make_pair((unsigned long)actionID.originatingStationID,(long)actionID.sequenceNumber);

    std::map<std::pair<unsigned long,long>, ITSSReceivingTableEntry>::iterator entry_rx_map_it = m_receivingITSSTable.find(map_index);

    if (entry_rx_map_it == m_receivingITSSTable.end ())
      {
        /* a. If entry does not exist in the receiving ITS-S message table, check if termination data exists in the
            received DENM. */
        if(decoded_denm->denm.management.termination!=NULL &&
           (*(decoded_denm->denm.management.termination)==Termination_isCancellation ||
           *(decoded_denm->denm.management.termination)==Termination_isNegation))
          {
            /* if yes, discard the received DENM and omit execution of further steps. */
            return;
          }
        else
          {
            /* if not, create an entry in the receiving ITS-S message table with the received DENM and set the state to ACTIVE (SSP is not yet implemented) */
            ITSSReceivingTableEntry entry(*packet,ITSSReceivingTableEntry::STATE_ACTIVE,actionID,referenceTime_long,detectionTime_long);
            m_receivingITSSTable[map_index]=entry;
          }
      }
    else
      {
        /* b. If entry does exist in the receiving ITS-S message table, check if the received referenceTime is less than the entry referenceTime,
         * or the received detectionTime is less than the entry detectionTime */
        long stored_reference_time = entry_rx_map_it->second.getReferenceTime ();
        long stored_detection_time =  entry_rx_map_it->second.getDetectionTime ();

        if (referenceTime_long < stored_reference_time || detectionTime_long < stored_detection_time)
          {
            /* i. if yes, discard received DENM and omit execution of further steps. */
            return;
          }
        else
          {
            /* ii. Otherwise, check if the received DENM is a repeated DENM of the entry, i.e. the received referenceTime equals to the entry
             * referenceTime, the received detectionTime equals to the entry
             * detectionTime, and the received termination value equals to the entry state */
            if(referenceTime_long == stored_reference_time &&
               detectionTime_long == stored_detection_time &&
               (
                 (decoded_denm->denm.management.termination==NULL &&  !entry_rx_map_it->second.isTerminationSet()) ||
                 (decoded_denm->denm.management.termination!=NULL && *(decoded_denm->denm.management.termination)==entry_rx_map_it->second.getTermination ())
               ))
              {
                /* 1. If yes, discard received DENM and omit execution of further steps. */
                return;
              }
            else
              {
                /* 2. Otherwise, update the entry in receiving ITS-S message table, set entry state according
                 * to the termination value of the received DENM. (SSP is not yet implemented) */
                ITSSReceivingTableEntry entry(*packet,ITSSReceivingTableEntry::STATE_ACTIVE,actionID,referenceTime_long,detectionTime_long,decoded_denm->denm.management.termination);
                entry_rx_map_it->second=entry;
              }
          }

      }

    /* Start/restart T_R_Validity timer. */
    if(m_T_R_Validity_Table.find(map_index) == m_T_R_Validity_Table.end()) {
      m_T_R_Validity_Table[map_index] = Timer();
    }

    DENBasicService::setDENTimer(m_T_R_Validity_Table[map_index],MilliSeconds((long)validityDuration),&DENBasicService::T_R_ValidityStop,actionID);

    m_DENReceiveCallback(den_data);
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

  void
  DENBasicService::T_O_ValidityStop(ActionID_t entry_actionid)
  {
    std::pair <unsigned long, long> map_index = std::make_pair((unsigned long)entry_actionid.originatingStationID,(long)entry_actionid.sequenceNumber);

    std::get<T_REPETITION_INDEX>(m_originatingTimerTable[map_index]).Cancel();
    std::get<T_REPETITION_DURATION_INDEX>(m_originatingTimerTable[map_index]).Cancel();

    // When an entry expires, discard also the information related to its (now stopped) timers
    m_originatingTimerTable.erase (map_index);
    m_originatingITSSTable.erase (map_index);
  }

  void
  DENBasicService::T_RepetitionDurationStop(ActionID_t entry_actionid)
  {
    std::pair <unsigned long, long> map_index = std::make_pair((unsigned long)entry_actionid.originatingStationID,(long)entry_actionid.sequenceNumber);

    std::get<T_REPETITION_INDEX>(m_originatingTimerTable[map_index]).Cancel();
  }

  void
  DENBasicService::T_RepetitionStop(ActionID_t entry_actionid)
  {
    std::pair <unsigned long, long> map_index = std::make_pair((unsigned long)entry_actionid.originatingStationID,(long)entry_actionid.sequenceNumber);

    Ptr<Packet> packet = Create<Packet> (m_originatingITSSTable[map_index].getDENMPacket ());
    m_socket_tx->Send(packet);

    // Restart timer
    std::get<T_REPETITION_INDEX>(m_originatingTimerTable[map_index]).Schedule();
  }

  void
  DENBasicService::T_R_ValidityStop(ActionID_t entry_actionid)
  {
    std::pair <unsigned long, long> map_index = std::make_pair((unsigned long)entry_actionid.originatingStationID,(long)entry_actionid.sequenceNumber);

    // When an entry expires, discard also the information related to its (now stopped) timers
    m_T_R_Validity_Table.erase (map_index);
    m_receivingITSSTable.erase (map_index);
  }

  /* This cleanup function will attemp to stop any possibly still-running timer */
  void
  DENBasicService::cleanup(void)
  {
    for(std::map<std::pair<unsigned long,long>, std::tuple<Timer,Timer,Timer>>::iterator m_originatingTimerTable_it=m_originatingTimerTable.begin ();
        m_originatingTimerTable_it!=m_originatingTimerTable.end ();
        m_originatingTimerTable_it++)
      {
         std::get<V_O_VALIDITY_INDEX>(m_originatingTimerTable_it->second).Cancel();
         std::get<T_REPETITION_INDEX>(m_originatingTimerTable_it->second).Cancel();
         std::get<T_REPETITION_DURATION_INDEX>(m_originatingTimerTable_it->second).Cancel();
      }

    for(std::map<std::pair<unsigned long,long>, Timer>::iterator m_T_R_Validity_Table_it=m_T_R_Validity_Table.begin ();
        m_T_R_Validity_Table_it!=m_T_R_Validity_Table.end ();
        m_T_R_Validity_Table_it++)
      {
         m_T_R_Validity_Table_it->second.Cancel();
      }
  }
}
