# PLANCK in the shared VR interaction services

A pocket following an NPC needs a current physical-body pose and a reliable indication that the body was replaced. An anatomical fitting provider also needs a controlled way to propose capsule edits. PLANCK owns those bodies and provides the observation, lifetime and application boundary.

The maintained [English concept](https://github.com/Vhodnoylogin/body-pouches/blob/main/docs/trinity-synergy/concept-en.md) and [Russian concept](https://github.com/Vhodnoylogin/body-pouches/blob/main/docs/trinity-synergy/concept-ru.md) live in Body Pouches. PLANCK owns its NPC physics and physical-hit decision path. HIGGS owns hand state/use and input, VRIK supplies the player pose, PPB supplies anatomy, and Body Slots manages logical regions. Consumers retain their gameplay rules.

The proposed interfaces provide copied actor/body poses and generations, owned attachment endpoints, revision-checked direct/compound capsule edits, scoped actor drive policies, contact observations and hit decisions. A pose reader can use the small read-only query. Mutating services negotiate owned registrations and capabilities separately. Released interface 001 has its original declarations.

The existing actor maintenance, body-removal, drive, contact and hit boundaries publish or apply these requests. Plugin message routing and build declarations expose their tables. Internal optimization and unrelated gameplay repairs are outside this proposal.

Contact samples identify the actual observation stage. Hit admission, invocation of the native hit handler and damage observed from the game are distinct facts. Numeric scaling has a supported field and hit-class scope. PPB private-world samples require their own owner evidence. The [implementation ledger](https://github.com/Vhodnoylogin/body-pouches/blob/main/docs/trinity-synergy/implementation-status.md) records source, build, consumer and game verification separately.
