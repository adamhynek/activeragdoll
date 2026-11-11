# Existing compound-capsule geometry

Request `GetApiFunction(5)` through PLANCK's existing SKSE transport, at PostPostLoad or later. The returned version-1 [PlanckCompoundGeometryAPI](../include/planck_compound_api.h) uses revision-2 clients, subscriptions, request results, cancellation and `ReleaseLease`. Existing tables are unchanged. This implements the physical-owner side of a bounded PPB fitting proposal; it does not port PPB or interpret anatomical regions.

## Coherent copied view

`CopyBodyGeometry` copies one exact published body's list and complete child array under one registry lock. Set each output row's size, query capacity through `BUFFER`, and retry to obtain the view and all rows together. Positions and radii are body-local meters in Havok axes. Each row names its child index, supported shape kind, enabled state, collision filter and capsule geometry where available. Unknown child kinds remain explicitly unknown.

The view carries the exact world/actor/body generation reference, sample and geometry revision. A child index is valid only within that body and revision; it is not a persistent anatomical identifier. Root/list-child replacement, enabled/filter changes and capsule geometry changes advance the body geometry revision. Actor/model/body lifetime replacement invalidates the body reference. No caller receives a native pointer. Only currently active PLANCK ragdolls are covered; this is not a universal corpse or player-body service.

## Atomic fitting requests

`RequestCapsuleEdits` accepts up to 256 distinct existing capsule-child edits for one body, with its expected geometry revision. This version preserves topology, enabled state, filters and materials. It creates no arbitrary capsule, list or rigid body. An invalid radius, nonfinite/out-of-range local point, degenerate segment, duplicate child, stale revision or foreign body is refused. Unknown/non-capsule target children are unsupported. Only one direct or compound geometry request may own a body at a time, including a closing owner with deferred native adoption still pending.

Acceptance is not application. The existing actor maintenance phase acquires graph then world write ownership, captures and publishes the live body, and resolves the proposal against that exact capture. A critical-operation lock postpones application. No provider callback runs under the registry mutex or the adapter's graph/world locks.

## One physical writer and actual adoption

The native adapter clones the existing engine shape wrapper and verifies a private native list and child-info array. It independently clones and retains every child capsule wrapper, refusing shared native aliases. This version therefore applies only to capsule-only lists with valid engine wrappers; a mixed or unwrappable list returns `UNSUPPORTED` without installing a partial result. All geometry edits are prepared on the private copy before the body's setter is called.

PLANCK preserves wrapper material/filter values, child collision filters, list flags and disabled-child masks. It recomputes conservative cached local bounds over every child, including disabled children. The existing rigid-body shape setter installs the list and refreshes broadphase/agent state; mass and inertia are left unchanged.

If the engine defers replacement, the request stays accepted and cannot be cancelled or released early. Completion requires observing the exact owned list on the same body. Only then does the registry publish the actual applied child geometry and advance the revision once. The next ordinary publication does not increment it again merely because PLANCK installed its own clone. A result can be copied after the owner starts closing; another owner cannot take over its pending body.

Installed root and child wrappers and the native body remain retained until a later owned replacement or entity removal. Every retiring wrapper back-pointer is cleared, including when a direct capsule replaces a previously owned list. Reset invalidates API requests without prematurely releasing wrappers still needed by a live or deferred native body. A retained pointer is not proof of world membership; all mutation admission still uses a fresh live graph/world capture.

## Verification and limits

The [offline fixtures and runner](https://github.com/Vhodnoylogin/activeragdoll/blob/ab6291e949d1b00fcb3ebac8ce7d17d983445e1e/tests/run_synergy_tests.py) are maintained alongside the provider prototype.

The complete native DLL also compiles and links against the staged external SDK. Compilation and synthetic engine checks do not establish engine clone/reference/cache behavior, collision quality or actual PPB compatibility in Skyrim. No plugin was installed or game started. The full concept and current qualification limits remain in the [canonical Body Pouches documents](https://github.com/Vhodnoylogin/body-pouches/tree/main/docs/trinity-synergy).
