#ifndef ASN_UTILS_H
#define ASN_UTILS_H

#define FIX_PROT_VERS       0x01
#define FIX_DENMID          0x01
#define DECI                10
#define CENTI               100
#define MILLI               1000
#define MICRO               1000000
#define DOT_ONE_MICRO       10000000

//Epoch time at 2004-01-01
#define TIME_SHIFT 1072915200000

/* Maximum length of an asn1c error message (when decoding fails with respect to certain constraints) */
#define ERRORBUFF_LEN       128

namespace ns3
{
  long compute_timestampIts (void);
}

#endif // ASN_UTILS_H

