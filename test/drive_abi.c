#include "planck_drive_api.h"
#include <assert.h>
#include "support/check.h"
#include <stddef.h>
_Static_assert(sizeof(PlanckDriveWeight)==56,"drive weight");
_Static_assert(sizeof(PlanckDrivePolicy)==56,"drive policy");
_Static_assert(sizeof(PlanckDrivePolicyState)==48,"drive lease state");
_Static_assert(sizeof(PlanckDriveBody)==64,"drive body observation");
_Static_assert(sizeof(PlanckDriveSnapshot)==96,"drive snapshot");
_Static_assert(sizeof(PlanckDriveAPI)==32,"drive version1 table");
_Static_assert(offsetof(PlanckDriveAPI,RequestActorDrivePolicy)==8,"table prefix");
int run_drive_c(const PlanckDriveAPI* api){assert(api&&api->version==1&&api->size==32);return 1;}
