Animation playground (test-only): box layout -> world sector with collision
===========================================================================

What it makes
  archive\pc\mod\utpp_playground.archive      one streaming block + one streaming sector
  archive\pc\mod\utpp_playground.archive.xl   registers the block (streaming: blocks:)
The sector contains one static mesh node per box (the base-game unit cube mesh, scaled) and one cooked
collision node with one actor + one box shape per box. Only base-game resources and the nodes of this
sector are referenced. The game folder is never touched by these scripts.

Files
  ue_to_red.py                 converts an exported Unreal cube layout to oriented boxes (metres, game axes)
  playground_sector.py         layout JSON -> sector / block JSON (WolvenKit format) + .xl text
  build_playground_stage.py    JSON -> cr2w -> archive -> stage folder + install manifest
  validate_playground_stage.py unpack + serialize back + compare, lists the stage contents
  spike_layout.json            two boxes (2 m x 4 m footprint; 1.0 m and 2.0 m tall; 3 m gap)

Steps (run from this folder, always with python -I)
  python -I ue_to_red.py --self-test
  python -I ue_to_red.py --input <exported_parts.json>    # writes playground_boxes.json next to the input
  python -I build_playground_stage.py --anchor X Y Z [YAW_DEG]
  python -I validate_playground_stage.py
Output: Build\stages\utpp_playground_spike (install with the project installer, stage name
utpp_playground_spike, manifest utpp_playground_spike.json in Research\inventory) and the manifest.
Work files: Build\work\playground-20261124\_tmp (delete when the round is closed).
Without --anchor the layout is built at the world origin and a warning is printed (anchor is a TODO).

Layout JSON (metres, game axes, relative to the anchor; z = 0 is the ground)
  anchor    x, y, z, yaw_deg  (z = ground height at the anchor; yaw is counter-clockwise seen from above)
  burial_m  boxes are extended this far downward (top face unchanged) so uneven terrain does not leave a gap
  boxes[]   name, center [x, y, z], half [hx, hy, hz], yaw_deg or quat [i, j, k, r]
  mesh      optional base-game cube mesh path (a 1 m cube centred on its pivot)

Coordinate convention
  Unreal: left-handed, X forward, Y right, Z up, centimetres, rotator (roll, pitch, yaw) in degrees.
  Game:   right-handed, X right, Y forward, Z up, metres.
  Mapping (x, y, z) -> (y, x, z) / 100. This is a reflection, so rotations are conjugated:
  R_game = M * R_unreal * M. Yaw changes sign and the two horizontal local axes swap, so box half
  extents become (hy, hx, hz). The local pivot offset ((min + max) / 2 * scale) is mapped the same way,
  so cube meshes with a corner pivot and with a centred pivot both give the right centre.
  Quaternions are written as (i, j, k, r). World positions in the collision data are fixed point
  (metres * 131072); box shape sizes are half extents.

Collision
  Preset "World Static" (collision type Static + PlayerBlocker + VehicleBlocker + NPCBlocker; queries
  Static, Visibility, Shooting ...). Physics material: the one used by box shapes of that preset in the
  base game sectors.

Choosing the anchor
  Stand on flat, open ground in the game, read the player position (world X, Y, Z) and pass it as
  --anchor X Y Z [YAW_DEG], with z = the ground height at that spot.
