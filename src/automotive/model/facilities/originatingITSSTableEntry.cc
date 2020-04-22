#include "originatingITSSTableEntry.h"

namespace ns3 {
  originatingITSSTableEntry::originatingITSSTableEntry(DENM_t asnDenmStructure, denm_table_state_t status)
  {
    m_denm = asnDenmStructure;
    m_status = status;
  }
}
