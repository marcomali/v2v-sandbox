/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "ITS_Container.asn1"
 * 	`asn1c -fincludes-quoted -gen-PER`
 */

#ifndef	_NumberOfOccupants_H_
#define	_NumberOfOccupants_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum NumberOfOccupants {
	NumberOfOccupants_oneOccupant	= 1,
	NumberOfOccupants_unavailable	= 127
} e_NumberOfOccupants;

/* NumberOfOccupants */
typedef long	 NumberOfOccupants_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_NumberOfOccupants;
asn_struct_free_f NumberOfOccupants_free;
asn_struct_print_f NumberOfOccupants_print;
asn_constr_check_f NumberOfOccupants_constraint;
ber_type_decoder_f NumberOfOccupants_decode_ber;
der_type_encoder_f NumberOfOccupants_encode_der;
xer_type_decoder_f NumberOfOccupants_decode_xer;
xer_type_encoder_f NumberOfOccupants_encode_xer;
per_type_decoder_f NumberOfOccupants_decode_uper;
per_type_encoder_f NumberOfOccupants_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _NumberOfOccupants_H_ */
#include "asn_internal.h"
