path=r'f:\Bruno\Dev\AppleWin\Projets\ORCA\OBJ Viewer Review\DonyGS\DONYGS.cc'
with open(path,'r',errors='ignore') as f:
    for n,l in enumerate(f,1):
        if n in (8015,8016,8017):
            print(n, 'indent=', len(l)-len(l.lstrip(' ')), repr(l.rstrip()))
