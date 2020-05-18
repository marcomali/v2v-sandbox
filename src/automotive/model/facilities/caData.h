#ifndef CADATA_H
#define CADATA_H

#include "asn_utils.h"
#include "ns3/CAM.h"
#include <cstring>

typedef struct SteeringWheelAngle sSteeringWheelAngle_t;
typedef struct LateralAcceleration sLateralAcceleration_t;
typedef struct VerticalAcceleration sVerticalAcceleration_t;
typedef struct CenDsrcTollingZone sCenDsrcTollingZone_t;
typedef struct ProtectedCommunicationZonesRSU sProtectedCommunicationZonesRSU_t;

#define MANDATORY_LAT_UNAVAILABLE 900000001
#define MANDATORY_LONG_UNAVAILABLE 1800000001

class caData
{

public:
  typedef struct _header {
    long messageID;
    long protocolVersion;
    StationID_t stationID;
  } caDataHeader;

  typedef struct _basiccontainer {
    StationType_t stationType;
    ReferencePosition_t	referencePosition;
  } caDataBasicContainer;

  typedef struct _hfcontainervehicle {
    Heading_t heading;
    Speed_t speed;
    DriveDirection_t driveDirection;
    VehicleLength_t vehicleLength;
    VehicleWidth_t vehicleWidth;
    LongitudinalAcceleration_t longitudinalAcceleration;
    Curvature_t	curvature;
    CurvatureCalculationMode_t curvatureCalculationMode;
    YawRate_t yawRate;
    AccelerationControl_t *accelerationControl;
    LanePosition_t *lanePosition;
    sSteeringWheelAngle_t *steeringWheelAngle;
    sLateralAcceleration_t *lateralAcceleration;
    sVerticalAcceleration_t *verticalAcceleration;
    PerformanceClass_t *performanceClass;
    sCenDsrcTollingZone_t *cenDsrcTollingZone;
  } caDataHFContainerVehicle;

  typedef struct _hfcontainerrsu {
    sProtectedCommunicationZonesRSU_t protectedCommZonesRSU;
  } caDataHFContainerRSU;

  typedef struct _lfcontainer {
      VehicleRole_t vehicleRole;
      ExteriorLights_t exteriorLights;
      PathHistory_t pathHistory;
  } caDataLFContainer;

  typedef SpecialVehicleContainer_t caDataSpecialVehicleContainer;

public:
  caData();

  /* AppDENM_trigger mandatory setters */
  void setDenmMandatoryFields(long detectionTime_ms, double latReference_deg, double longReference_deg);
  void setDenmMandatoryFields(long detectionTime_ms, double latReference_deg, double longReference_deg, double altitude_m);
  void setDenmMandatoryFields_asn_types(TimestampIts_t detectionTime, ReferencePosition_t eventPosition);

  /*
   * Header setters (they can be used for experimentation purposes, but they shall not be called normally, as the header is typically
   * set inside the DEN Basic Service, without the need of a manual user intervention
  */
  void setDenmHeader(long messageID, long protocolVersion, StationID_t stationID) {m_header.messageID=messageID; m_header.protocolVersion=protocolVersion; m_header.stationID=stationID;}
  void setDenmMessageID(long messageID) {m_header.messageID=messageID;}
  void setDenmProtocolVersion(long protocolVersion) {m_header.protocolVersion=protocolVersion;}
  void setDenmStationID(StationID_t stationID) {m_header.stationID=stationID;}

  /* AppDENM_update mandatory setters */
  /* AppDENM_terminate mandatory setters */
  void setDenmMandatoryFields(long originatingStationID, long sequenceNumber, long detectionTime_ms, double latReference_deg, double longReference_deg);
  void setDenmMandatoryFields(long originatingStationID, long sequenceNumber, long detectionTime_ms, double latReference_deg, double longReference_deg, double altitude_m);
  void setDenmMandatoryFields_asn_types(ActionID_t actionID, TimestampIts_t detectionTime, ReferencePosition_t eventPosition);

  /* receiveDENM setters */
  void setDenmActionID(ActionID_t actionID) {m_management.actionID=actionID;}

  /* Optional information setters */

  /* Header getters */
  long getDenmHeaderMessageID() {return m_header.messageID;}
  long getDenmHeaderProtocolVersion() {return m_header.protocolVersion;}
  long getDenmHeaderStationID() {return m_header.stationID;}

  /* Container getters */
  caDataHeader getDenmHeader_asn_types() const{return m_header;}
  caDataManagement getDenmMgmtData_asn_types() const{return m_management;}
  caDataSituation getDenmSituationData_asn_types() const{return m_situation;}
  caDataLocation getDenmLocationData_asn_types() const{return m_location;}
  caDataAlacarte getDenmAlacarteData_asn_types() const{return m_alacarte;}

  long getDenmMgmtDetectionTime() const{long detectionTime=0;
                                        asn_INTEGER2long (&m_management.detectionTime,&detectionTime);
                                        return detectionTime;}

  long getDenmMgmtValidityDuration() const{long validityDuration = m_management.validityDuration!=NULL ? *(m_management.validityDuration) : DEN_DEFAULT_VALIDITY_S;
                                           return validityDuration;}

  long getDenmMgmtReferenceTime() const{long referenceTime=0;
                                        asn_INTEGER2long (&m_management.referenceTime,&referenceTime);
                                        return referenceTime;}

  long getDenmMgmtLatitude() const{return m_management.eventPosition.latitude;}
  long getDenmMgmtLongitude() const{return m_management.eventPosition.longitude;}
  long getDenmMgmtAltitude() const{return m_management.eventPosition.altitude.altitudeValue;}

  long getDenmHeaderMessageID() const{return m_header.messageID;}
  long getDenmHeaderProtocolVersion () const{return m_header.protocolVersion;}
  unsigned long getDenmHeaderStationID () const{return (unsigned long)m_header.stationID;}

  ActionID_t getDenmActionID() const{return m_management.actionID;}

  /* Internals setters */
  void setDenmRepetition(uint32_t repetitionDuration,uint32_t repetitionInterval) {m_internals.repetitionInterval=repetitionInterval; m_internals.repetitionDuration=repetitionDuration;}
  void setDenmRepetitionInterval(uint32_t repetitionInterval) {m_internals.repetitionInterval=repetitionInterval;}
  void setDenmRepetitionDuration(uint32_t repetitionDuration) {m_internals.repetitionDuration=repetitionDuration;}
  int setValidityDuration(long validityDuration_s);

  /* Internal getters */
  uint32_t getDenmRepetitionDuration() { return m_internals.repetitionDuration; }
  uint32_t getDenmRepetitionInterval() { return m_internals.repetitionInterval; }

  /* Container setters */
  void setDenmMgmtData_asn_types(caDataManagement management) {m_management = management;}
  void setDenmSituationData_asn_types(caDataSituation situation) {m_situation = situation;}
  void setDenmLocationData_asn_types(caDataLocation location) {m_location = location;}
  void setDenmAlacarteData_asn_types(caDataAlacarte alacarte) {m_alacarte = alacarte;}

  /* Object integrity check */
  bool iscaDataRight();

private:
  INTEGER_t asnTimeConvert(long time);

  caDataInternals m_internals;
  caDataHeader m_header;
  caDataManagement m_management;
  caDataSituation m_situation;
  caDataLocation m_location;
  caDataAlacarte m_alacarte;
};

#endif // CADATA_H
