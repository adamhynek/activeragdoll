# Scoped NPC drive policy

Request `GetApiFunction(4)` through the existing PLANCK SKSE transport to obtain the version-1 [PlanckDriveAPI](../include/planck_drive_api.h). It uses revision-2 client tokens, event subscriptions and `ReleaseLease`; official001, the revision-2 table layout and the revision-3 native endpoint bridge are unchanged. This source implements a bounded part of PPB's requested integration, not a complete PPB migration or a replacement pose solver.

## Owned requests and conflict rules

`RequestActorDrivePolicy` accepts an exact published actor reference, optional block-pivot-loosening flag, selected hierarchy/velocity/position gains, and a caller array of body-specific multipliers. Gains are finite values in [0,1]; unselected gain fields must be zero. A multiplier is finite and in [0,8], with an exact current `PlanckBodyRef`. Duplicate bodies, foreign-actor weights, reserved values, missing generations and malformed records are rejected. At most 256 weights fit one request. Inputs are copied before publication; no caller buffer survives the call.

Only one numerical-gain or weight policy may own an actor generation at once. Another request returns `CONFLICT`, including while the previous owner's client is closing and an admitted drive is still in flight. There is no silent registration-order winner. Block-pivot-only leases compose by OR and can coexist with the numerical owner and each other. Releasing one blocker does not release another. Requests are part of the existing 512-resource pool, rather than a second client registry.

A successful request means that the policy is accepted. `CopyDrivePolicyState` distinguishes accepted, observed at a real driver call, and stale; its last-observed sample is not a general gameplay-success receipt. `ReleaseLease` releases a policy, returning `BUSY` while its admitted native call is in flight. `ReleaseClient` closes admission and likewise requires a retry after drainage. Actor/model/body replacement, removal and reset retire the old-generation policies and emit `PLANCK_DRIVE_POLICY_INVALIDATED`. Client handles and subscriptions survive reset; stale policy leases remain explicitly releasable.

## PLANCK remains the writer

The existing outer drive hook resolves the model, ragdoll lineage and exact live body list. Its copied plan is admitted through the same lifecycle owner as the other cooperation APIs. PLANCK's own pivot-loosening branch reads the actor's effective policy instead of changing the global configuration. Already-saved pivots still restore when the saved ease action retires. Gradual pivot interpolation is skipped while blocking, so it cannot overwrite restored originals with an intermediate value.

At the existing inner rigid-body-controller call, PLANCK rechecks the published generations and actual controller/ragdoll/body identities. A scoped adapter applies selected gains to the existing palette and multiplies the engine's own per-body weights. Palette selection uses the current public body-to-palette mapping, with the SDK's entry-zero fallback when its length differs from the body count; a private cache from the previous native call is not an authority. Unmentioned bodies use multiplier one. The resulting hierarchy fraction is capped at one when this policy modifies weights or hierarchy gain, preventing extrapolation beyond the target; the copied body result reports saturation. A zero hierarchy gain is handled without division. Invalid palette indices, nonfinite native weights or mismatched lifetimes leave the engine controls unchanged and report the failure.

The adapter restores exactly the gain fields it changed and the original weight pointer after the existing drive call, including C++ exception unwinding. Other palette fields remain untouched. With no applicable policy, the engine's gains and weight pointer remain unchanged. These are transient controller inputs: no second actuator, additional executable detour, ragdoll-body creation or writable native callback is introduced.

This API does not supply arbitrary pose-track edits, joint-pivot targets, controller replacement, damping/acceleration/snap policies, motion-type changes or PPB's complete pose-conforming pipeline. Those needs remain separate contracts; three gain fields must not be presented as the whole NPC physics role.

## Copied observations and phase

`CopyActorDrive` copies the latest actual drive snapshot and its complete body array atomically. `BUFFER` reports the required capacity; retry with size-tagged output rows. The snapshot names the actor generation, sample, main frame, physics tick, numerical policy, effective pivot-blocking flag, native status and palette count. Its three gain values summarize palette entry zero; each body reports its exact reference, requested multiplier, original engine weight, effective weight and resulting hierarchy fraction. A multi-graph actor's snapshot describes the latest observed driver and the bodies in that call, rather than claiming a simultaneous aggregate of every graph.

`PLANCK_DRIVE_OBSERVED` uses the existing removable event mechanism. Its `subject` is the sample identifier and phase is `PLANCK_PHASE_DRIVE`. An observer should copy the snapshot and compare that sample identifier before using it. Ordinary pose publication preserves the last drive observation; the stored frame/tick distinguishes a previous observation from a current one. Reset clears it. An outer call with no usable inner controller reports that absence instead of inventing successful actuation.

The existing engine drive already owns its graph/world phase. Observers run after the provider registry/lifecycle locks are released, but engine ownership may still be held by the caller. They receive copied data only: no engine mutation, waiting or graph/world acquisition. As with other provider events, new registrations or policy requests from inside a callback return `REENTRANT`; queue them for the next permitted maintenance boundary. Removal and client teardown use the existing nonblocking callback drainage contract.

## Verification

The [offline fixtures and runner](https://github.com/Vhodnoylogin/activeragdoll/blob/ab6291e949d1b00fcb3ebac8ce7d17d983445e1e/tests/run_drive_tests.py) are maintained alongside the provider prototype.
