#include "higgs_contracts/higgsnativeinterop.h"
#include <stddef.h>
_Static_assert(sizeof(HIGGS_NativeOwnContact)==104,"native packet");
_Static_assert(sizeof(HIGGS_OwnContactNativeInterface)==24,"native table");
_Static_assert(offsetof(HIGGS_NativeOwnContact,pointVelocityA)==56,"sampled A velocity");
_Static_assert(offsetof(HIGGS_OwnContactNativeInterface,Observe)==16,"native observation");
int own_contact_c_contract(void){return 1;}
