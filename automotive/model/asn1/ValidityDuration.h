/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "ITS_Container.asn1"
 * 	`asn1c -fincludes-quoted -gen-PER`
 */

#ifndef	_ValidityDuration_H_
#define	_ValidityDuration_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ValidityDuration {
	ValidityDuration_timeOfDetection	= 0,
	ValidityDuration_oneSecondAfterDetection	= 1
} e_ValidityDuration;

/* ValidityDuration */
typedef long	 ValidityDuration_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ValidityDuration;
asn_struct_free_f ValidityDuration_free;
asn_struct_print_f ValidityDuration_print;
asn_constr_check_f ValidityDuration_constraint;
ber_type_decoder_f ValidityDuration_decode_ber;
der_type_encoder_f ValidityDuration_encode_der;
xer_type_decoder_f ValidityDuration_decode_xer;
xer_type_encoder_f ValidityDuration_encode_xer;
per_type_decoder_f ValidityDuration_decode_uper;
per_type_encoder_f ValidityDuration_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _ValidityDuration_H_ */
#include "asn_internal.h"
