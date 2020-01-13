/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "ITS_Container.asn1"
 * 	`asn1c -fincludes-quoted -gen-PER`
 */

#ifndef	_ProtectedCommunicationZone_H_
#define	_ProtectedCommunicationZone_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ProtectedZoneType.h"
#include "TimestampIts.h"
#include "Latitude.h"
#include "Longitude.h"
#include "ProtectedZoneRadius.h"
#include "ProtectedZoneID.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ProtectedCommunicationZone */
typedef struct ProtectedCommunicationZone {
	ProtectedZoneType_t	 protectedZoneType;
	TimestampIts_t	*expiryTime	/* OPTIONAL */;
	Latitude_t	 protectedZoneLatitude;
	Longitude_t	 protectedZoneLongitude;
	ProtectedZoneRadius_t	*protectedZoneRadius	/* OPTIONAL */;
	ProtectedZoneID_t	*protectedZoneID	/* OPTIONAL */;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ProtectedCommunicationZone_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ProtectedCommunicationZone;

#ifdef __cplusplus
}
#endif

#endif	/* _ProtectedCommunicationZone_H_ */
#include "asn_internal.h"
