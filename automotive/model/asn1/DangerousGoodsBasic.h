/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "ITS_Container.asn1"
 * 	`asn1c -fincludes-quoted -gen-PER`
 */

#ifndef	_DangerousGoodsBasic_H_
#define	_DangerousGoodsBasic_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeEnumerated.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DangerousGoodsBasic {
	DangerousGoodsBasic_explosives1	= 0,
	DangerousGoodsBasic_explosives2	= 1,
	DangerousGoodsBasic_explosives3	= 2,
	DangerousGoodsBasic_explosives4	= 3,
	DangerousGoodsBasic_explosives5	= 4,
	DangerousGoodsBasic_explosives6	= 5,
	DangerousGoodsBasic_flammableGases	= 6,
	DangerousGoodsBasic_nonFlammableGases	= 7,
	DangerousGoodsBasic_toxicGases	= 8,
	DangerousGoodsBasic_flammableLiquids	= 9,
	DangerousGoodsBasic_flammableSolids	= 10,
	DangerousGoodsBasic_substancesLiableToSpontaneousCombustion	= 11,
	DangerousGoodsBasic_substancesEmittingFlammableGasesUponContactWithWater	= 12,
	DangerousGoodsBasic_oxidizingSubstances	= 13,
	DangerousGoodsBasic_organicPeroxides	= 14,
	DangerousGoodsBasic_toxicSubstances	= 15,
	DangerousGoodsBasic_infectiousSubstances	= 16,
	DangerousGoodsBasic_radioactiveMaterial	= 17,
	DangerousGoodsBasic_corrosiveSubstances	= 18,
	DangerousGoodsBasic_miscellaneousDangerousSubstances	= 19
} e_DangerousGoodsBasic;

/* DangerousGoodsBasic */
typedef long	 DangerousGoodsBasic_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DangerousGoodsBasic;
asn_struct_free_f DangerousGoodsBasic_free;
asn_struct_print_f DangerousGoodsBasic_print;
asn_constr_check_f DangerousGoodsBasic_constraint;
ber_type_decoder_f DangerousGoodsBasic_decode_ber;
der_type_encoder_f DangerousGoodsBasic_encode_der;
xer_type_decoder_f DangerousGoodsBasic_decode_xer;
xer_type_encoder_f DangerousGoodsBasic_encode_xer;
per_type_decoder_f DangerousGoodsBasic_decode_uper;
per_type_encoder_f DangerousGoodsBasic_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _DangerousGoodsBasic_H_ */
#include "asn_internal.h"
