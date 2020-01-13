/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "ITS_Container.asn1"
 * 	`asn1c -fincludes-quoted -gen-PER`
 */

#ifndef	_SteeringWheelAngle_H_
#define	_SteeringWheelAngle_H_


#include "asn_application.h"

/* Including external dependencies */
#include "SteeringWheelAngleValue.h"
#include "SteeringWheelAngleConfidence.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SteeringWheelAngle */
typedef struct SteeringWheelAngle {
	SteeringWheelAngleValue_t	 steeringWheelAngleValue;
	SteeringWheelAngleConfidence_t	 steeringWheelAngleConfidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SteeringWheelAngle_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SteeringWheelAngle;

#ifdef __cplusplus
}
#endif

#endif	/* _SteeringWheelAngle_H_ */
#include "asn_internal.h"
