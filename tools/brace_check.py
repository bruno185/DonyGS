import sys
path=r'f:\Bruno\Dev\AppleWin\Projets\ORCA\OBJ Viewer Review\DonyGS\DONYGS.cc'
count = 0
start = 0
with open(path,'r',errors='ignore') as f:
    for n,l in enumerate(f,1):
        if n == 8015:
            start = 1
        if start:
            for ch in l:
                if ch == '{': count += 1
                elif ch == '}': count -= 1
            if n <= 8120:
                print(f"{n:5d}: count={count} {l.rstrip()}")
        if n > 8120:
            break
print('final count', count)
