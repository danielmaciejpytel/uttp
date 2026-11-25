"""Generate a streaming block and a streaming sector (as WolvenKit JSON) from a box layout.

Layout JSON (metres, game world, see ue_to_red.py for the convention):
    {
      "name": "spike",
      "anchor": {"x": .., "y": .., "z": .., "yaw_deg": ..},   # world position of the layout origin, z = ground level
      "burial_m": 0.3,                                         # boxes extend this far below their bottom face
      "boxes": [ {"name": "..", "center": [x, y, z], "half": [hx, hy, hz], "yaw_deg": 0 | "quat": [i, j, k, r]} ]
    }
Box centre and quaternion are relative to the anchor (rotated by the anchor yaw, then translated).
A box's lowest face is at z + (center z - half z); the burial extends the box downward only, keeping its top face.

Each box becomes one worldStaticMeshNode (unit cube mesh, scaled) and one actor with one box shape in a single
worldCookedCollisionNode.  Everything that is not a node of this sector is a base-game resource.
"""
import json
import math

from ue_to_red import quaternion_to_matrix

MESH_PATH = r'engine\meshes\editor\cube.mesh'
COLLISION_PRESET = 'World Static'
# physics material used by box shapes of the "World Static" preset in the base game sectors
COLLISION_MATERIAL_HASH = '12468419275653881673'
FIXED_POINT_ONE = 131072          # WorldPosition FixedPoint: bits = metres * 2^17
STREAMING_DISTANCE = 2000.0
FLT_MAX = 3.40282347e+38
BUFFER_TYPE = 'WolvenKit.RED4.%s, WolvenKit.RED4, Version=10.0.0.0, Culture=neutral, PublicKeyToken=null'

HEADER = {'WolvenKitVersion': '10.0.0-beta.1', 'WKitJsonVersion': '0.0.9', 'GameVersion': 2500, 'DataType': 'CR2W'}


def fnv1a64(text):
    h = 0xcbf29ce484222325
    for c in text.encode():
        h ^= c
        h = (h * 0x100000001b3) & 0xffffffffffffffff
    return h


def _cname(s):
    return {'$type': 'CName', '$storage': 'string', '$value': s}


def _cname_hash(h):
    return {'$type': 'CName', '$storage': 'uint64', '$value': str(h)}


def _path(p, flags='Soft'):
    return {'DepotPath': {'$type': 'ResourcePath', '$storage': 'string', '$value': p}, 'Flags': flags}


def _zero_path(flags):
    return {'DepotPath': {'$type': 'ResourcePath', '$storage': 'uint64', '$value': '0'}, 'Flags': flags}


def _vec3(x, y, z):
    return {'$type': 'Vector3', 'X': x, 'Y': y, 'Z': z}


def _quat(q):
    return {'$type': 'Quaternion', 'i': q[0], 'j': q[1], 'k': q[2], 'r': q[3]}


def _yaw_quat(yaw_deg):
    h = math.radians(yaw_deg) / 2
    return [0.0, 0.0, math.sin(h), math.cos(h)]


def _q_mul(a, b):
    ai, aj, ak, ar = a
    bi, bj, bk, br = b
    return [ar * bi + ai * br + aj * bk - ak * bj,
            ar * bj - ai * bk + aj * br + ak * bi,
            ar * bk + ai * bj - aj * bi + ak * br,
            ar * br - ai * bi - aj * bj - ak * bk]


def place_boxes(layout):
    """Resolve the layout into world boxes: [{'name', 'center', 'half', 'quat'}] (game world, metres)."""
    a = layout['anchor']
    aq = _yaw_quat(a.get('yaw_deg', 0.0))
    ar = quaternion_to_matrix(aq)
    burial = float(layout.get('burial_m', 0.0))
    out = []
    for b in layout['boxes']:
        q = b['quat'] if 'quat' in b else _yaw_quat(b.get('yaw_deg', 0.0))
        half = list(b['half'])
        center = list(b['center'])
        if burial > 0 and abs(q[0]) < 1e-6 and abs(q[1]) < 1e-6:   # bottom face stays level for yaw-only boxes
            half[2] += burial / 2
            center[2] -= burial / 2
        c = [sum(ar[i][j] * center[j] for j in range(3)) + (a['x'], a['y'], a['z'])[i] for i in range(3)]
        wq = _q_mul(aq, q)
        n = math.sqrt(sum(v * v for v in wq))
        out.append({'name': b['name'], 'center': c, 'half': half, 'quat': [v / n for v in wq]})
    return out


def _aabb(boxes):
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for b in boxes:
        m = quaternion_to_matrix(b['quat'])
        for i in range(3):
            ext = sum(abs(m[i][j]) * b['half'][j] for j in range(3))
            lo[i] = min(lo[i], b['center'][i] - ext)
            hi[i] = max(hi[i], b['center'][i] + ext)
    return lo, hi


def _fixed(m):
    return int(round(m * FIXED_POINT_ONE))


def _setup(node_idx, pos, rot, scale, flags, global_id, instance_index):
    p = {'$type': 'Vector4', 'W': 0, 'X': pos[0], 'Y': pos[1], 'Z': pos[2]}
    ref = _vec3(pos[0], pos[1], pos[2])
    return {
        '$type': 'worldCompiledNodeInstanceSetupInfo',
        'dataGroup': 'Base',
        'externalInplaceResourceHash': _zero_path('Default'),
        'globalNodeId': {'$type': 'worldGlobalNodeID', 'hash': str(global_id)},
        'instanceIndex': instance_index,
        'nodeFlags': flags,
        'nodeIdx': str(node_idx),
        'patchedNode': {'$type': 'NodeRef', '$storage': 'uint64', '$value': '0'},
        'preboostStreamingRefPoint': dict(ref),
        'proxyMeshId': {'$type': 'worldGlobalNodeID', 'hash': '0'},
        'scale': _vec3(*scale),
        'secondaryRefPointDistance': FLT_MAX,
        'secondaryRefPointPosition': _vec3(0, 0, 0),
        'streamingDistance': STREAMING_DISTANCE,
        'streamingPriority': '1',
        'streamingRefPoint': dict(ref),
        'todvisMask': '0',
        'transform': {'$type': 'Transform', 'orientation': _quat(rot), 'position': p},
    }


def _node_base(type_name, debug_name):
    return {'$type': type_name, 'debugName': _cname(debug_name), 'isHostOnly': 0, 'isVisibleInGame': 1,
            'proxyScale': None, 'sourcePrefabHash': '0', 'tag': 'None', 'tagExt': 'None'}


def mesh_node(debug_name, mesh_path=MESH_PATH):
    n = _node_base('worldStaticMeshNode', debug_name)
    n['mesh'] = _path(mesh_path)
    n['meshAppearance'] = _cname('default')
    return n


def collision_node(boxes, sector_hash, debug_name):
    """One cooked collision node with a single inner node holding one actor + one box shape per box."""
    lo, hi = _aabb(boxes)
    center = [(lo[i] + hi[i]) / 2 for i in range(3)]
    extents = [(hi[i] - lo[i]) / 2 for i in range(3)]
    n = len(boxes)
    actors, shapes = [], []
    for idx, b in enumerate(boxes):
        actors.append({
            '$type': 'worldCookedCollisionActor', 'scaleIndex': -1, 'shapeCount': 1, 'shapeStartIndex': idx,
            'transform': {'$type': 'WorldTransform', 'Orientation': _quat(b['quat']),
                          'Position': {'$type': 'WorldPosition',
                                       'x': {'$type': 'FixedPoint', 'Bits': _fixed(b['center'][0])},
                                       'y': {'$type': 'FixedPoint', 'Bits': _fixed(b['center'][1])},
                                       'z': {'$type': 'FixedPoint', 'Bits': _fixed(b['center'][2])}}},
            'uk1': 0})
        shapes.append({
            '$type': 'worldCookedCollisionShape',
            'collisionSourceKey': {'$type': 'PhysicsGeometryBoxData', 'size': _vec3(*b['half'])},
            'index': idx, 'materialCount': 1, 'proxyType': 11,
            'shapePositionIndex': -1, 'shapePresetIndex': 0, 'shapeRotationIndex': -1})
    inner = {'$type': 'worldCookedCollisionInnerNode', 'actors': actors,
             'materialIndices': [0] * n, 'materials': [_cname_hash(COLLISION_MATERIAL_HASH)],
             'presets': [_cname(COLLISION_PRESET)], 'scales': [], 'shapeIndices': list(range(n)),
             'shapePositions': [], 'shapeRotations': [], 'shapes': shapes}
    info = {'$type': 'worldCookedCollisionNodeInfo', 'dataOffset': 0,
            'extents': {'Elements': [extents[0], extents[1], extents[2], 0]},
            'lod': 2, 'numActors': n, 'numMaterialIndices': n, 'numMaterials': 1, 'numPresets': 1, 'numScales': 0,
            'numShapeIndices': n, 'numShapeInfos': n, 'numShapePositions': 0, 'numShapeRotations': 0,
            'sectorHash': str(sector_hash)}
    # The counts in the info record must describe the arrays exactly.
    assert info['numActors'] == len(actors) and info['numShapeInfos'] == len(shapes)
    assert info['numMaterialIndices'] == len(inner['materialIndices']) == sum(s['materialCount'] for s in shapes)
    assert info['numShapeIndices'] == len(inner['shapeIndices'])
    assert info['numMaterials'] == len(inner['materials']) and info['numPresets'] == len(inner['presets'])
    assert info['numShapePositions'] == len(inner['shapePositions']) and info['numShapeRotations'] == len(inner['shapeRotations'])
    assert info['numScales'] == len(inner['scales'])
    node = _node_base('worldCookedCollisionNode', debug_name)
    node['buffer'] = {'BufferId': '1', 'Flags': 0, 'Type': BUFFER_TYPE % 'Types.worldCookedCollisionInfoBuffer',
                      'Data': {'CollisionNodeInfos': [info]}}
    node['compiledData'] = {'BufferId': '2', 'Flags': 7471104, 'Type': BUFFER_TYPE % 'Types.worldCookedCollisionNodeBuffer',
                            'Data': {'CollisionNodes': [inner]}}
    return node, center, extents


def build_sector(boxes, sector_path, mesh_path=MESH_PATH):
    """Returns (sector json, bounding box (lo, hi)). mesh_path: a base-game mesh that is a 1 m cube centred on its pivot."""
    sector_hash = fnv1a64(sector_path)
    nodes, setups = [], []
    for i, b in enumerate(boxes):
        name = 'utpp_playground_%s' % b['name']
        nodes.append({'HandleId': str(i), 'Data': mesh_node(name, mesh_path)})
        scale = (b['half'][0] * 2, b['half'][1] * 2, b['half'][2] * 2)   # the mesh is a 1 m cube centred on its pivot
        setups.append(_setup(i, b['center'], b['quat'], scale, 'FLAG_Building, FLAG_Culling_IsCullable',
                             fnv1a64('utpp_playground:' + b['name']), 4294967295))
    cnode, ccenter, _ext = collision_node(boxes, sector_hash, 'utpp_playground_collision')
    nodes.append({'HandleId': str(len(boxes)), 'Data': cnode})
    setups.append(_setup(len(boxes), ccenter, [0.0, 0.0, 0.0, 1.0], (1.0, 1.0, 1.0), 'FLAG_Building', 0, 0))
    root = {
        '$type': 'worldStreamingSector', 'category': 'AlwaysLoaded', 'cookingPlatform': 'PLATFORM_PC',
        'externInplaceResource': _zero_path('Soft'), 'level': 1, 'localInplaceResource': [],
        'nodeData': {'BufferId': '0', 'Flags': 0, 'Type': BUFFER_TYPE % 'Archive.Buffer.worldNodeDataBuffer',
                     'Data': setups},
        'nodeRefs': [], 'nodes': nodes, 'persistentNodeIndex': 0, 'persistentNodes': json.loads(json.dumps(setups)),
        'variantIndices': [0], 'variantNodes': [], 'version': 68}
    doc = {'Header': dict(HEADER), 'Data': {'Version': 195, 'BuildVersion': 0, 'RootChunk': root, 'EmbeddedFiles': []}}
    return doc, _aabb(boxes)


def build_block(sector_path, bbox, margin=2000.0):
    lo, hi = bbox
    box = {'$type': 'Box',
           'Max': {'$type': 'Vector4', 'W': -FLT_MAX, 'X': hi[0] + margin, 'Y': hi[1] + margin, 'Z': hi[2] + margin},
           'Min': {'$type': 'Vector4', 'W': FLT_MAX, 'X': lo[0] - margin, 'Y': lo[1] - margin, 'Z': lo[2] - margin}}
    desc = {'$type': 'worldStreamingSectorDescriptor',
            'blockIndex': {'$type': 'worldStreamingBlockIndex', 'oup': 'Base', 'rldGridCell': 0},
            'category': 'AlwaysLoaded', 'data': _path(sector_path), 'level': 1, 'numNodeRanges': 1,
            'questPrefabNodeRef': {'$type': 'NodeRef', '$storage': 'uint64', '$value': '0'},
            'streamingBox': box, 'variants': []}
    root = {'$type': 'worldStreamingBlock', 'cookingPlatform': 'PLATFORM_None', 'descriptors': [desc],
            'index': {'$type': 'worldStreamingBlockIndex', 'oup': 'Base', 'rldGridCell': 0}}
    return {'Header': dict(HEADER), 'Data': {'Version': 195, 'BuildVersion': 0, 'RootChunk': root, 'EmbeddedFiles': []}}


def build_xl(block_path):
    return 'streaming:\n  blocks:\n    - %s\n' % block_path
