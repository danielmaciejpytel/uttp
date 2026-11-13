import os
D = os.path.dirname(os.path.abspath(__file__))
base = open(os.path.join(D, 'run3.cet.txt'), encoding='utf-8').read().split(';;')
cmd = {x.split("[night] ")[1].split(" ")[0]: x for x in base}

def setk(tag, key, val):
    return ("local ok,e=pcall(function() return GetMod('utpp_core').Set('%s','%s') end) "
            "Game.UTPPLogLine('[night] %s ok='..tostring(ok)..' '..tostring(e))" % (key, val, tag))

runs = {
    # Female V, Kyubi rifle, aiming kept in third person for the measurement
    'pose_female_utpp': [setk('SET_PACK', 'animation.pack', 'utpp'), setk('SET_SCOPED', 'perspective.aimingScoped', 'tpp'),
                         cmd['SARATOGA'], cmd['HOLSTER'], cmd['EQUIP_W_AFTER'], cmd['DIAG_ON']],
    'pose_female_game': [setk('SET_PACK', 'animation.pack', 'game'), setk('SET_SCOPED', 'perspective.aimingScoped', 'tpp'),
                         cmd['SARATOGA'], cmd['HOLSTER'], cmd['EQUIP_W_AFTER'], cmd['DIAG_ON']],
    # Male V, Saratoga from the save; the last run leaves the defaults in the settings file
    'pose_male_game': [setk('SET_PACK', 'animation.pack', 'game'), setk('SET_SCOPED', 'perspective.aimingScoped', 'fpp'),
                       cmd['EQUIP_W'], cmd['DIAG_ON']],
    'pose_male_utpp': [setk('SET_PACK', 'animation.pack', 'utpp'), setk('SET_SCOPED', 'perspective.aimingScoped', 'fpp'),
                       cmd['EQUIP_W'], cmd['DIAG_ON']],
}
for k, v in runs.items():
    open(os.path.join(D, k + '.cet.txt'), 'w', encoding='utf-8').write(';;'.join(v))
    print(k, len(v))
