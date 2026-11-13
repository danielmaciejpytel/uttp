"""Superseded one-off patch (kept only as a pointer; the male counterpart and C++ test comments refer to this name).

The relaxed one-handed pistol idle curve change for the female overlay (layering_legs / layering_pelvis of
Pose_Pistol1H_Stand_Relax_Idle = 0.0 instead of 1.0) is now part of the baker: see CURVE_OVERRIDES in
tools/mmbake/overlaybake.py. A normal bake of the woman rig reproduces the shipped overlay byte for byte;
`overlaybake.py --no-overrides` gives the plain bake without the change.
"""
import sys

sys.exit("patch_female_pistol_idle_stance.py is superseded: use tools/mmbake/overlaybake.py "
         "(CURVE_OVERRIDES; --no-overrides for the plain bake).")
