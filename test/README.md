# Offline contract tests

From a fresh checkout, run:

```sh
python test/run.py --build-dir ../planck-contract-build
```

Requires Python 3.9+ and a 64-bit C11/C++20 compiler with its standard library. On Windows the runner discovers Visual Studio C++ Build Tools through `vswhere`, or uses `cl` from an x64 developer prompt. On Linux/macOS it uses `cc` and `c++`; override them with `--cc` and `--cxx` if needed. The tested configuration is Windows x64, Python 3.12, MSVC 19.51. GNU/Clang command paths are provided but have not been qualified here. No Python packages, CMake, Git history, downloaded fixtures, Skyrim installation, SKSE SDK or Havok headers are required.

Outputs must be outside this checkout. `manifest.json` is written only after every check passes and records source hashes, generated fixture hashes, command arguments, exit codes and the static wiring results. Each compile and executable has a separate `.log`. All assertions remain active under `NDEBUG`; failures print their source expression and exit with code 86. Each subprocess is bounded to 90 seconds. A failed run removes a previous success manifest.

## What executes

The runner compiles the **current** `src/synergy_runtime.cpp` and public headers directly. It never builds a stored copy or an independent model of the service.

| Suite | Boundary and assertions |
| --- | --- |
| `abi.c` + `core.cpp` | C11/C++20 layout, version/size negotiation, original 72-byte actor prefix and additive sample-time tail; malformed sizes and identity outputs; owned clients/endpoints; copied/coherent actor/body snapshots; body/actor epochs and replacement; suspension/reset; hit policies receiving one candidate in either registration order; actual handler receipts versus invalidated target context, denial and cancellation; geometry availability, conflict, cancellation and admitted ownership; scoped endpoint resolver; self/concurrent callback teardown. |
| `contacts_hit` | Real runtime watches, opposite contact sides, geometry attribution, ownership, exact `(epoch, contactId)` lookup, untouched output on failure, late/released/stale watches, exact 64-entry retention and eviction, no latest substitution, retained hit causality after contact eviction, reset, 1024 queued observations plus explicit gap, rule priority/caps and receipts. The native numeric application portion uses the extracted functions described below. |
| `native` | Extracted actor publication, attachment resolution, clutter damage and direct/compound capsule adoption on engine doubles. Checks graph/world lock ordering, borrowed endpoint lifetime, clones, revisions, delayed adoption, closing owners, partial/alias refusal, disabled children, filters, AABB and wrapper retirement. |
| `drive` | Actual drive headers/runtime plus extracted outer/inner hooks on controller doubles: C table ABI, ownership/conflicts, policy staleness, hierarchy/weight composition and clamp, palette mapping, pivot scope, restore on return/exception and busy teardown while admitted. |
| `pose` | Extracted named-body pose adapter on task/graph/world/body doubles: copied position/quaternion/time/frame, size/thread checks, sparse and ambiguous bones, removal/re-add/deletion/address reuse, ragdoll/model replacement and reset. Its `MatchPoseBody` seam is a stub; real runtime matching is exercised in `contacts_hit`. |
| `hand_parts`, `own_contact` | Current classification/forwarding code and C native bridge declarations on provider/body doubles: articulated-part readiness/lifetimes/fallback, nested observation scope, bilateral point velocity, mixed held/equipped directions, disabled/nonpotential guards and provider failures. |
| `dispatch` | Extracted `GetApi` dispatch from current `src/pluginapi.cpp`: unknown requests, legacy/pose routes and proposal tables 2–7. Legacy and pose providers are stubs; this does not load a DLL or send an SKSE message. |

Native fixture templates contain authored assertions and explicit engine doubles. At each run, `run.py` inserts the selected functions/blocks from current `src/main.cpp`, `src/pluginapi.cpp` and the point-velocity wrapper in `src/RE/havok.cpp`. `HitData`'s flag enum is read as source text from `include/RE/misc.h`; that engine header is never included by a compiler. There are no copied dependency definitions or production implementations in `test/`. Missing extraction seams fail the run.

Thirteen supplemental **static source checks** check load/reset, drive pivot, contact-to-hit and populated-melee wiring. They are reported separately and do not execute those enclosing engine call paths.

Two mandatory negative controls change generated output only. Replacing exact contact lookup with latest lookup must fail the retained-contact assertion; omitting `reflectedDamage` must fail the seven-field assertion. These controls must exit 86, so a compiler failure or arbitrary crash cannot count as detection. Positive tests precede the controls and always use unmodified current sources.

## Limits

These tests establish offline contract/control-flow behavior on explicit doubles. They do not build the native plugin, verify Skyrim/Havok layouts, install hooks, exercise SKSE transport, run a solver, measure health loss or qualify game, feedback, visual or experiential correctness. In particular, the seven-field receipt check verifies numeric writes to the matching populated `HitData` double before the extracted handler boundary; a receipt is not health-loss evidence. No game is started.