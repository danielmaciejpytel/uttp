import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import gt_night as gn
D = os.path.dirname(os.path.abspath(__file__))
diag = open(os.path.join(D, 'diag_only.cet.txt'), encoding='utf-8').read()
open(os.path.join(D, 'fists_diag.cet.txt'), 'w', encoding='utf-8').write(gn.FISTS_CMD + ';;' + diag)
print(gn.FISTS_CMD[:160])
