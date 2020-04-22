#ifndef DENDATA_H
#define DENDATA_H

#include "asn_utils.h"
#include "ns3/DENM.h"
#include <cstring>
#include <chrono>

typedef struct CauseCode sCauseCode_t;
typedef struct EventHistory sEventHistory_t;

typedef struct Speed sSpeed_t;
typedef struct Heading sHeading_t;

typedef struct ImpactReductionContainer sImpactReductionContainer_t;
typedef struct RoadWorksContainerExtended sRoadWorksContainerExtended_t;
typedef struct StationaryVehicleContainer sStationaryVehicleContainer_t;

#define MANDATORY_LAT_UNAVAILABLE 900000001
#define MANDATORY_LONG_UNAVAILABLE 1800000001

#define DEN_DEFAULT_VALIDITY_MS 600000

class denData
{

public:
  typedef struct _internals {
      uint32_t repetitionDuration;
      uint32_t repetitionInterval;
      bool isMandatorySet;
  } denDataInternals;

  typedef struct _management {
      TimestampIts_t detectionTime;
      ReferencePosition_t eventPosition;
      ValidityDuration_t *validityDuration;
      TransmissionInterval_t *transmissionInterval;
      ActionID_t actionID;
      Termination_t *termination;
      RelevanceDistance_t *relevanceDistance;
      RelevanceTrafficDirection_t *relevanceTrafficDirection;
      // StationType_t stationType; // Defined during the creation of the DEN Basic service object
  } denDataManagement;

  typedef struct _situation {
      InformationQuality_t informationQuality;
      CauseCode_t eventType;
      sCauseCode_t *linkedCause;
      sEventHistory_t *eventHistory;
  } denDataSituation;

  typedef struct _location {
      sSpeed_t *eventSpeed;
      sHeading_t *eventPositionHeading;
      Traces_t traces;
      RoadType_t *roadType;
  } denDataLocation;

  typedef struct _alacarte {
      LanePosition_t *lanePosition;
      sImpactReductionContainer_t *impactReduction;
      Temperature_t *externalTemperature;
      sRoadWorksContainerExtended_t *roadWorks;
      PositioningSolutionType_t *positioningSolution;
      sStationaryVehicleContainer_t *stationaryVehicle;
  } denDataAlacarte;

public:
  denData();

  /* AppDENM_trigger mandatory setters */
  void setDenmMandatoryFields(long detectionTime_ms, long latReference_deg, long longReference_deg);
  void setDenmMandatoryFields(long detectionTime_ms, long latReference_deg, long longReference_deg, long altitude_m);
  void setDenmMandatoryFields_asn_types(TimestampIts_t detectionTime, ReferencePosition_t eventPosition);

  /* AppDENM_update mandatory setters */
  /* AppDENM_terminate mandatory setters */
  void setDenmMandatoryFields(long originatingStationID, long sequenceNumber, long detectionTime_ms, long latReference_deg, long longReference_deg);
  void setDenmMandatoryFields(long originatingStationID, long sequenceNumber, long detectionTime_ms, long latReference_deg, long longReference_deg, long altitude_m);
  void setDenmMandatoryFields_asn_types(ActionID_t actionID, TimestampIts_t detectionTime, ReferencePosition_t eventPosition);

  /* Optional information setters */

  /* Container getters */
  denDataManagement getDenmMgmtData_asn_types() const{return m_management;}
  denDataSituation getDenmSituationData_asn_types() const{return m_situation;}
  denDataLocation getDenmLocationData_asn_types() const{return m_location;}
  denDataAlacarte getDenmAlacarteData_asn_types() const{return m_alacarte;}

  long getDenmMgmtDetectionTime() const{long detectionTime=0;
                                        asn_INTEGER2long (&m_management.detectionTime,&detectionTime);
                                        return detectionTime;}

  long getDenmMgmtValidityDuration() const{long validityDuration = m_management.validityDuration!=NULL ? *(m_management.validityDuration) : DEN_DEFAULT_VALIDITY_MS;
                                           return validityDuration;}

  /* Object integrity check */
  bool isDenDataRight();

private:
  INTEGER_t asnTimeConvert(long time);

  denDataInternals m_internals;
  denDataManagement m_management;
  denDataSituation m_situation;
  denDataLocation m_location;
  denDataAlacarte m_alacarte;
};

#endif // DENDATA_H
