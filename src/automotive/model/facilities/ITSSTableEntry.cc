#include "ITSSTableEntry.h"

namespace ns3 {
  ITSSTableEntry::ITSSTableEntry() {
    m_status = STATE_UNSET;
    m_actionid.originatingStationID = 0;
    m_actionid.sequenceNumber = -1;
    m_referenceTime = -1;
  }

  ITSSTableEntry::ITSSTableEntry(Packet asnDenmPacket, denm_table_state_t status, ActionID_t actionID)
  {
    m_denm_encoded = asnDenmPacket;
    m_status = status;
    m_actionid = actionID;
  }

  ITSSTableEntry::ITSSTableEntry(Packet asnDenmPacket, denm_table_state_t status, ActionID_t actionID, long referenceTime)
  {
    m_denm_encoded = asnDenmPacket;
    m_status = status;
    m_actionid = actionID;
    m_referenceTime = referenceTime;
  }
}
