import os
D = os.path.dirname(os.path.abspath(__file__))
base = open(os.path.join(D, 'run3.cet.txt'), encoding='utf-8').read().split(';;')
cmd = {x.split("[night] ")[1].split(" ")[0]: x for x in base}


def setk(tag, key, val):
    return ("local ok,e=pcall(function() return GetMod('utpp_core').Set('%s','%s') end) "
            "Game.UTPPLogLine('[night] %s ok='..tostring(ok)..' '..tostring(e))" % (key, val, tag))


defaults = [setk('SET_PACK', 'animation.pack', 'utpp'), setk('SET_SCOPED', 'perspective.aimingScoped', 'fpp')]
runs = {
    # both bodies with the same weapon (Saratoga), mod pack, default settings, camera levelled by the harness
    'parity_female': defaults + [cmd['SARATOGA'], cmd['EQUIP_W_AFTER'], cmd['DIAG_ON']],
    'parity_male': defaults + [cmd['EQUIP_W'], cmd['DIAG_ON']],
}
for k, v in runs.items():
    open(os.path.join(D, k + '.cet.txt'), 'w', encoding='utf-8').write(';;'.join(v))
    print(k, len(v))
