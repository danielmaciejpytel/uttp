"""Pre-install gate: every @addField target in the UTPP scripts must be a class already proven to boot.

redscript compiles @addField on a native-only class (no script definition), but the engine then fails at boot with
"Failed to initialize scripts data!" (2026-11-03: @addField(gamevehicleVehicleMountableComponent)). A new target is
allowed only after a boot check of the game with it (then add it to BOOT_VERIFIED). Fields that cannot go on a
script class belong in a ScriptableSystem.

usage: python -I tools/install/check_reds_addfield.py [scripts_dir]   (exit 1 on an unverified target)
"""
import pathlib
import re
import sys

BOOT_VERIFIED = {
    "PlayerPuppet",
    "VehicleComponent",
    "VehicleComponentPS",
    "gameuiCrosshairBaseGameController",
    "CrosshairGameControllerPersistentDot",
}
KNOWN_BAD = {"gamevehicleVehicleMountableComponent"}

root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parents[2] / "plugin/UTPP/scripts")
bad = 0
for f in sorted(root.rglob("*.reds")):
    for n, line in enumerate(f.read_text(encoding="utf-8").splitlines(), 1):
        m = re.match(r"\s*@addField\((\w+)\)", line)
        if m and m.group(1) not in BOOT_VERIFIED:
            bad += 1
            why = "native-only class, fails at boot" if m.group(1) in KNOWN_BAD else "not boot-verified"
            print(f"{f.name}:{n}: @addField({m.group(1)}) - {why}")
print("addField gate:", "FAIL" if bad else "ok", f"({bad} unverified)")
sys.exit(1 if bad else 0)
