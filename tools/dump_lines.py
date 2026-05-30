import sys
path=r'f:\Bruno\Dev\AppleWin\Projets\ORCA\OBJ Viewer Review\DonyGS\DONYGS.cc'
with open(path,'r') as f:
    for n,l in enumerate(f,1):
        if 8095 <= n <= 8115:
            sys.stdout.write(f'{n:5d}: {l}')
