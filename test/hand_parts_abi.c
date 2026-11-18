#include "higgs_contracts/higgsnativeinterop.h"
#include <stddef.h>
_Static_assert(sizeof(HIGGS_HandPartsNativeInterface)==24,"native parts table");
_Static_assert(offsetof(HIGGS_HandPartsNativeInterface,ResolveBody)==16,"resolver slot");
_Static_assert(sizeof(HIGGS_HandPartAttribution)==56,"copied parts identity");
int hand_parts_c_contract(void){return 1;}
