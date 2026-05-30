path=r'f:\Bruno\Dev\AppleWin\Projets\ORCA\OBJ Viewer Review\DonyGS\DONYGS.cc'
with open(path,'r',errors='ignore') as f:
    for n,l in enumerate(f,1):
        if 8098 <= n <= 8112:
            count = len(l) - len(l.lstrip(' '))
            print(f"{n:5d}: indent={count} '{l.rstrip()}'")
