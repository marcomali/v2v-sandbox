#ifndef ORIGINATINGITSSTABLEENTRY_H
#define ORIGINATINGITSSTABLEENTRY_H
#include "ns3/DENM.h"
#include "ns3/timer.h"

namespace ns3 {
  class originatingITSSTableEntry
  {
  public:
    typedef enum {
      STATE_ACTIVE,
      STATE_CANCELLED,
      STATE_NEGATED
    } denm_table_state_t;

    originatingITSSTableEntry(DENM_t asnDenmStructure, denm_table_state_t status);

  private:
    DENM_t m_denm;
    denm_table_state_t m_status;
    Timer m_V_O_Validity;
    Timer m_T_Repetition;
    Timer m_T_RepetitionDuration;
  };

}

#endif // ORIGINATINGITSSTABLEENTRY_H
