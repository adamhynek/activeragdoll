"""Build and run PLANCK contracts with only Python and an x64 C11/C++20 toolchain."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
TEST = ROOT / "test"


def block(source, signature):
    """Extract current production text, respecting comments and string literals."""
    start = source.index(signature)
    opening = source.index("{", start)
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.S)
    depth = 0
    for token in tokens.finditer(source, opening):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[start:token.end()]
    raise RuntimeError("Unterminated production block: " + signature)


def replace_once(source, marker, value):
    if source.count(marker) != 1:
        raise RuntimeError("Missing/duplicate fixture seam: " + marker)
    return source.replace(marker, value)


def environment():
    env = dict(os.environ)
    if os.name != "nt" or shutil.which("cl"):
        return env
    # Standard Visual Studio discovery; no machine-specific installation path.
    vswhere = Path(env.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
    if not vswhere.is_file():
        return env
    install = subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-requires",
        "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath", "-utf8"],
        text=True, encoding="utf-8", errors="replace").strip()
    if not install:
        return env
    vcvars = Path(install) / "VC/Auxiliary/Build/vcvars64.bat"
    output = subprocess.check_output(f'cmd.exe /d /s /c "chcp 65001 >nul && call "{vcvars}" >nul && set"',
        text=True, encoding="utf-8", errors="replace")
    for line in output.splitlines():
        if "=" in line and not line.startswith("="):
            key, value = line.split("=", 1)
            env[key.upper()] = value
    return env


def generate(out):
    main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
    misc = (ROOT / "include/RE/misc.h").read_text(encoding="utf-8")
    fixtures = {}

    def fixture(name, substitutions):
        text = (TEST / "fixtures" / (name + ".cpp.in")).read_text(encoding="utf-8")
        for marker, value in substitutions.items():
            text = replace_once(text, marker, value)
        if re.search(r'/\*@\w+@\*/|// @\w+@', text):
            raise RuntimeError("Unfilled fixture marker in " + name)
        path = out / (name + ".cpp")
        path.write_text(text, encoding="utf-8")
        fixtures[name] = path

    geometry_start = main.index("struct SynergyGeometryShape {")
    geometry_end = main.index("void PublishSynergyActor(Actor* actor)\n{", geometry_start)
    fixture("native", {
        "/*@GEOMETRY_NATIVE@*/": main[geometry_start:geometry_end],
        "/*@ACTOR_PUBLICATION@*/": block(main, "void PublishSynergyActor(Actor* actor)\n{"),
        "/*@ENDPOINT_RESOLVER@*/": block(main, "PlanckStatus ResolveSynergyEndpoint(PlanckToken owner,PlanckToken lease,const PlanckEndpoint& endpoint,\n    PlanckEndpointVisitor visitor,void* context)\n{"),
        "/*@CLUTTER_HIT@*/": block(main, "    void ApplyPhysicsDamage("),
    })
    native = fixtures["native"].read_text(encoding="utf-8").replace("struct Listener {", block(main, "void ScaleNumericDamage(") + "\nstruct Listener {")
    fixtures["native"].write_text(native, encoding="utf-8")

    pivot_start = main.index("    const bool allowPivotMotion=PlanckSynergy::MayLoosenPivots(")
    gradual_start = main.index("if (allowPivotMotion)", main.index("// Pivots\n"))
    fixture("drive", {
        "/*@OUTER_DRIVE@*/": block(main, "void DriveToPoseHook(hkbRagdollDriver *driver, hkReal deltaTime, const hkbContext &context, hkbGeneratorOutput &generatorOutput)"),
        "/*@INNER_DRIVE@*/": block(main, "void hkbRagdollDriver_driveToPose_hkaRagdollRigidBodyController_driveToPose_Hook(hkaRagdollRigidBodyController *_this, hkReal deltaTime, hkQsTransform *poseLocalSpace, const hkQsTransform &worldFromModel, hkaKeyFrameHierarchyUtility::Output *stressOut)"),
        "/*@PIVOT_POLICY@*/": main[pivot_start:main.index(";", pivot_start) + 1],
        "/*@RESTORE_CONSTRAINTS@*/": block(main, "void RestoreAndClearPendingConstraintSnapshot(ActiveRagdoll &ragdoll, hkbRagdollDriver *driver, bool blockPivotMotion = false)"),
        "/*@GRADUAL_PIVOT@*/": block(main[gradual_start:], "if (allowPivotMotion)"),
    })
    fixture("hand_parts", {"/*@CLASSIFICATION@*/": "\n".join(block(main, "inline bool " + name + "(hkpRigidBody *rigidBody)") for name in (
        "IsLeftRigidBody", "IsPlayerWeaponRigidBody", "IsPlayerHandRigidBody", "IsHeldRigidBody", "IsHiggsRigidBody"))})
    callback = main[main.index("virtual void contactPointCallback(const hkpContactPointEvent &evnt) override"):]
    fixture("own_contact", {
        "/*@POINT_WRAPPER@*/": block((ROOT / "src/RE/havok.cpp").read_text(encoding="utf-8"), "NiPoint3 hkpRigidBody_getPointVelocity("),
        "/*@FORWARDER@*/": block(main, "struct ContactPairSample {") + ";\n" + block(main, "void ForwardOwnItemCosmeticContact("),
        "/*@ENTRY_GUARD@*/": block(callback, "if (evnt.m_contactPointProperties->m_flags & hkContactPointMaterial::FlagEnum::CONTACT_IS_DISABLED ||"),
        "/*@OWN_BRANCH@*/": block(main, "        if (isAhiggs && isBhiggs) {"),
    })
    adapter_start = main.index("namespace {\n    std::atomic<DWORD> poseTaskThread")
    fixture("pose", {
        "// @ADAPTER@": main[adapter_start:main.index("// This mainly exists", adapter_start)],
        "// @BODYHOOKS@": "\n".join(block(main, "    virtual void " + name + "(hkpEntity *entity) override").replace(" override", "") for name in ("entityRemovedCallback", "entityDeletedCallback")),
    })
    scope_start = main.index("struct MeleeApplicationScope {")
    fixture("contacts_hit", {
        "// @FLAGS@": block(misc[misc.index("struct HitData\n"):], "enum class Flag : UInt32").replace("UInt32", "uint32_t") + ";",
        "// @SCOPE@": main[scope_start:main.index("bool HitActor(", scope_start)],
        "// @APPLICATION@": block(main, "void ScaleNumericDamage(") + "\n" + block(main, "void ApplyPopulatedMeleeFields("),
    })
    fixture("dispatch", {"/*@GET_API@*/": block((ROOT / "src/pluginapi.cpp").read_text(encoding="utf-8"), "void *GetApi(unsigned int revisionNumber)")})

    # These are wiring assertions, separate from executable contract tests.
    checks = {
        "pose reset": "ResetPoseLifetimes();" in block(main, "void ResetObjects()"),
        "world replacement reset": "ResetObjects();" in block(main, "if (world != g_physicsListener.m_world)"),
        "pose listener ownership": "hkpWorld_hasEntityListener" in block(main, "    bool ObservesPoseWorld(bhkWorld* world) {"),
        "preload closes pose": "poseLoadPending = true;" in block(main, "else if (msg->type == SKSEMessagingInterface::kMessage_PreLoadGame)"),
        "postload opens pose": "poseLoadPending = false;" in block(main, "else if (msg->type == SKSEMessagingInterface::kMessage_PostLoadGame || msg->type == SKSEMessagingInterface::kMessage_NewGame)"),
        "drive restores blocked pivots": "RestoreAndClearPendingConstraintSnapshot(*ragdoll, driver, !allowPivotMotion)" in main,
        "drive gradual pivot gate": "if (Config::options.loosenRagdollConstraintPivots && allowPivotMotion)" in main,
        "contact capture": "contactId = PlanckSynergy::HasContactWatches() ? CapturePotentialContact(pair)" in main,
        "melee causal link": "isTwoHanding, isStab, contactId));" in main,
        "clutter causal link": "minMass, minSpeed, contactId);" in main,
        "hit causal link": "g_synergyPhysicsTick.load(), contactId);" in main,
        "melee source target hand scope": "MeleeApplicationScope application(source,target,isOffhand,hit);" in main,
        "populated melee handoff": "if (application) ApplyPopulatedMeleeFields(*application,*hitData);" in main,
    }
    for name, passed in checks.items():
        if not passed:
            raise RuntimeError("Static wiring check failed: " + name)
    return fixtures, checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True, type=Path, help="Generated files and logs, outside this checkout")
    parser.add_argument("--cc", help="C11 compiler (default: cl on Windows, cc elsewhere)")
    parser.add_argument("--cxx", help="C++20 compiler (default: cl on Windows, c++ elsewhere)")
    args = parser.parse_args()
    out = args.build_dir.resolve()
    if out == ROOT or ROOT in out.parents:
        parser.error("Build output must be outside this checkout")
    out.mkdir(parents=True, exist_ok=True)
    (out / "manifest.json").unlink(missing_ok=True)
    env = environment()
    cc = shutil.which(args.cc or ("cl" if os.name == "nt" else "cc"), path=env.get("PATH"))
    cxx = shutil.which(args.cxx or ("cl" if os.name == "nt" else "c++"), path=env.get("PATH"))
    if not cc or not cxx:
        parser.error("Install an x64 C11/C++20 toolchain or supply --cc and --cxx")
    msvc = Path(cxx).stem.lower() in ("cl", "clang-cl")
    if (Path(cc).stem.lower() in ("cl", "clang-cl")) != msvc:
        parser.error("Use matching C/C++ compiler families")
    commands = []
    files = [p for folder in (ROOT / "include", ROOT / "src", TEST) for p in folder.rglob("*") if p.is_file()]
    pins = {str(p.relative_to(ROOT)).replace("\\", "/"): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}

    def run(name, argv, expected=0):
        result = subprocess.run([str(x) for x in argv], cwd=out, env=env, capture_output=True,
            text=True, encoding="utf-8", errors="replace", timeout=90)
        (out / (name + ".log")).write_text(result.stdout + result.stderr, encoding="utf-8")
        commands.append({"name": name, "argv": [str(x) for x in argv], "exitCode": result.returncode, "expected": expected})
        if result.returncode != expected:
            print(result.stdout + result.stderr, end="")
            raise RuntimeError(name + " failed; see " + str(out / (name + ".log")))
        print("PASS " + name, flush=True)

    cflags = ["/nologo", "/utf-8", "/TC", "/std:c11", "/W4", "/WX", "/I" + str(TEST), "/I" + str(ROOT / "include")] if msvc else ["-std=c11", "-Wall", "-Wextra", "-Werror", "-I" + str(TEST), "-I" + str(ROOT / "include")]
    cppflags = ["/nologo", "/utf-8", "/std:c++20", "/EHsc", "/W4", "/WX", "/wd4100", "/O2", "/I" + str(TEST), "/I" + str(ROOT / "include")] if msvc else ["-std=c++20", "-O2", "-Wall", "-Wextra", "-Wno-missing-field-initializers", "-Wno-unused-parameter", "-pthread", "-I" + str(TEST), "-I" + str(ROOT / "include")]
    objects = {}
    for name in ("abi", "drive_abi", "hand_parts_abi", "own_contact_abi"):
        obj = out / (name + (".obj" if msvc else ".o"))
        run(name + "-c11", [cc, *cflags, "/c", TEST / (name + ".c"), "/Fo" + str(obj)] if msvc else [cc, *cflags, "-c", TEST / (name + ".c"), "-o", obj])
        objects[name] = obj

    fixtures, checks = generate(out)
    runtime = ROOT / "src/synergy_runtime.cpp"

    def cpp(name, sources, expected=0):
        exe = out / (name + (".exe" if os.name == "nt" else ""))
        run(name + "-compile", [cxx, *cppflags, *sources, "/Fe" + str(exe)] if msvc else [cxx, *cppflags, *sources, "-o", exe])
        run(name, [exe], expected)

    cpp("core", [TEST / "core.cpp", runtime, objects["abi"]])
    for name, abi, link_core in (("native", None, True), ("drive", "drive_abi", True),
            ("hand_parts", "hand_parts_abi", False), ("own_contact", "own_contact_abi", False),
            ("pose", None, False), ("contacts_hit", None, True), ("dispatch", None, True)):
        cpp(name, [fixtures[name], *([runtime] if link_core else []), *([objects[abi]] if abi else [])])
    print("PASS supplemental static wiring (" + str(len(checks)) + " checks)", flush=True)

    # Mutations exist only in generated output; all positive suites above use current source.
    broken_runtime = out / "negative_latest_runtime.cpp"
    broken_runtime.write_text(replace_once(runtime.read_text(encoding="utf-8"), "*out = sample;",
        "*out = resource->contact->history[(resource->contact->sequence-1)%PLANCK_CONTACT_HISTORY_CAPACITY];"), encoding="utf-8")
    cpp("negative_latest", [fixtures["contacts_hit"], broken_runtime], expected=86)
    broken_fields = out / "negative_fields.cpp"
    broken_fields.write_text(replace_once(fixtures["contacts_hit"].read_text(encoding="utf-8"), "fields.reflectedDamage*=factor;", "/* negative control: omitted reflected damage */"), encoding="utf-8")
    cpp("negative_fields", [broken_fields, runtime], expected=86)
    for p in files:
        key = str(p.relative_to(ROOT)).replace("\\", "/")
        if hashlib.sha256(p.read_bytes()).hexdigest() != pins[key]:
            raise RuntimeError("Source changed during test run: " + key)
    manifest = {"passed": True, "sourcePins": pins, "commands": commands,
        "staticWiringChecks": checks, "generatedFixtures": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in fixtures.values()},
        "negativeControls": {"latestSubstitution": "detected", "missingSeventhDamageField": "detected"},
        "nativePluginBuilt": False, "gameStarts": 0}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print("Evidence: " + str(out / "manifest.json"))


if __name__ == "__main__":
    main()
