#ifndef ASN_UTILS_H
#define ASN_UTILS_H

#define FIX_PROT_VERS       0x01
#define FIX_DENMID          0x01
#define DECI                10
#define CENTI               100
#define MILLI               1000
#define MICRO               1000000
#define DOT_ONE_MICRO       10000000

/* Maximum length of an asn1c error message (when decoding fails with respect to certain constraints) */
#define ERRORBUFF_LEN       128

#endif // ASN_UTILS_H

