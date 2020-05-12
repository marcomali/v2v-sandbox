#include "caBasicService.h"

namespace ns3
{
  CABasicService::CABasicService()
  {

  }

  CABasicService_error_t
  CABasicService::setT_CheckCamGen(long interval_ms)
  {
    if(interval_ms > T_GenCamMin_ms)
      {
        m_T_CheckCamGen_ms_interval = T_GenCamMin_ms;
        return CAM_WRONG_INTERVAL;
      }
    else
      {
        m_T_CheckCamGen_ms_interval = interval_ms;
      }

    return CAM_NO_ERROR;
  }
}
