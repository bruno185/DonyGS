"""Extraction indépendante des appels des sources actives incluses depuis ObjExplorer.cc.

Aucune génération de code ni modification des sources. Les appels indirects sont
résolus aux sites de passage de callbacks ; les données globales ne sont pas des appels."""
from pathlib import Path
import re
import bisect
import subprocess
import collections
ROOT = Path(__file__).resolve().parents[1]
SDK = Path('C:/ProgramData/GoldenGate/Libraries/ORCACDefs')
TOK = re.compile('//[^\\n]*|/\\*[\\s\\S]*?\\*/|"(?:\\\\.|[^"\\\\])*"|\\\'(?:\\\\.|[^\\\'\\\\])*\\\'|[A-Za-z_]\\w*|0[xX][0-9A-Fa-f]+|\\d+|->|[^\\s]')
KEYWORDS = {'if', 'for', 'while', 'switch', 'sizeof', 'return', 'defined', 'asm', '_Alignof'}

def mask_directives(s):
    """Masquer les directives et leurs continuations, en conservant les positions."""
    out = []
    skip = False
    for l in s.splitlines(keepends=True):
        if skip or l.lstrip().startswith('#'):
            skip = l.rstrip().endswith('\\')
            out.append(''.join(('\n' if c == '\n' else ' ' for c in l)))
        else:
            out.append(l)
    return ''.join(out)

def lex(s):
    """Découper le C et l’assembleur sans confondre commentaires, chaînes et code."""
    return [(m[0], m.start()) for m in TOK.finditer(mask_directives(s)) if not m[0].startswith(('//', '/*'))]

def pairs(ts):
    """Associer parenthèses, crochets et accolades ; refuser les délimiteurs déséquilibrés."""
    stack = []
    match = {}
    openers = {'(': ')', '{': '}', '[': ']'}
    for i, (t, _) in enumerate(ts):
        if t in openers:
            stack.append((t, i))
        elif t in openers.values():
            assert stack and openers[stack[-1][0]] == t, (t, i, stack[-1:])
            _, j = stack.pop()
            match[i] = j
            match[j] = i
    assert not stack
    return match

def arguments(ts):
    """Séparer les arguments seulement au niveau externe des délimiteurs."""
    result = []
    start = 0
    depth = 0
    for i, (t, _) in enumerate(ts):
        if t in ('(', '{', '['):
            depth += 1
        elif t in (')', '}', ']'):
            depth -= 1
        elif t == ',' and (not depth):
            result.append(ts[start:i])
            start = i + 1
    if ts:
        result.append(ts[start:])
    return result

def definitions(s, file=None, locations=None):
    """Repérer les définitions au niveau global, avec leur provenance et leurs paramètres."""
    ts = lex(s)
    match = pairs(ts)
    defs = []
    i = 0
    start = 0
    lines = [0] + [m.end() for m in re.finditer('\n', s)]
    while i < len(ts):
        t, pos = ts[i]
        if t == '{':
            end = match[i]
            if i and ts[i - 1][0] == ')':
                op = match[i - 1]
                name = ts[op - 1][0]
                if re.fullmatch('[A-Za-z_]\\w*', name) and name not in KEYWORDS:
                    line = bisect.bisect_right(lines, ts[start][1])
                    loc = (file, line) if locations is None else locations[line - 1]
                    params = arguments(ts[op + 1:i - 1])
                    paramnames = []
                    for pp in params:
                        ids = [t for t, _ in pp if re.fullmatch('[A-Za-z_]\\w*', t)]
                        paramnames.append(ids[-1] if ids else '')
                    defs.append(dict(name=name, file=loc[0], line=loc[1], params=paramnames, tokens=ts[i + 1:end], signature=' '.join((t for t, _ in ts[start:i]))))
            i = end
            start = i + 1
        elif t == ';':
            start = i + 1
        i += 1
    return defs
files = {p.name.lower(): p for p in ROOT.iterdir() if p.suffix.lower() in ('.c', '.cc', '.h')}
todo = ['objexplorer.cc']
sources = {}
while todo:
    key = todo.pop()
    if key in sources:
        continue
    p = files[key]
    s = p.read_text(encoding='utf-8-sig')
    sources[key] = (p.name, s)
    for inc in re.findall('^\\s*#\\s*include\\s*[<"]([^>"\\n]+)', s, re.M):
        if inc.lower() in files:
            todo.append(inc.lower())
rawdefs = [f for name, s in sources.values() for f in definitions(s, name)]
counts = collections.Counter((f['name'] for f in rawdefs))
assert max(counts.values()) == 1, counts
RAW = {f['name']: f for f in rawdefs}
MACROS = {n for _, s in sources.values() for n in re.findall('^\\s*#\\s*define\\s+(\\w+)', s, re.M)}

def graph(defs):
    """Extraire appels C, JSL/JSR, callbacks résolus et appels assembleur locaux ou dynamiques."""
    names = set(defs)
    edges = {n: set() for n in names}
    indirect = {n: set() for n in names}
    external = {n: set() for n in names}
    asm_local = {n: set() for n in names}
    dynamic = {n: [] for n in names}
    refs = {n: set() for n in names}
    sites = {}
    callback_targets = collections.defaultdict(set)
    for n, f in defs.items():
        ts = f['tokens']
        match = pairs(ts)
        sites[n] = []
        called_positions = set()
        for i, (t, _) in enumerate(ts):
            if i + 1 < len(ts) and ts[i + 1][0] == '(' and re.fullmatch('[A-Za-z_]\\w*', t) and (t not in KEYWORDS):
                args = arguments(ts[i + 2:match[i + 1]])
                sites[n].append((t, args))
                called_positions.add(i)
                if t in names:
                    edges[n].add(t)
                elif t not in MACROS:
                    external[n].add(t)
            if t.lower() in ('jsl', 'jsr', 'jml') and i + 1 < len(ts):
                target = ts[i + 1][0]
                called_positions.add(i + 1)
                if target in names:
                    edges[n].add(target)
                elif re.fullmatch('0[xX][0-9A-Fa-f]+|\\d+', target):
                    dynamic[n].append(target)
                elif any((ts[j][0] == target and ts[j + 1][0] == ':' for j in range(len(ts) - 1))):
                    asm_local[n].add(target)
                else:
                    external[n].add('ASM:' + target)
        refs[n] = {t for i, (t, _) in enumerate(ts) if t in names and i not in called_positions}
    for n, calls in sites.items():
        for target, args in calls:
            if target == 'qsort' and len(args) == 4:
                values = [t for t, _ in args[3] if t in names]
                assert len(values) == 1, (n, args)
                indirect[n].update(values)
                refs[n].difference_update(values)
            if target in names:
                for param, arg in zip(defs[target]['params'], args):
                    values = {t for t, _ in arg if t in names}
                    if values:
                        callback_targets[target, param].update(values)
                        refs[n].difference_update(values)
    for n, calls in sites.items():
        for target, args in calls:
            if (n, target) in callback_targets:
                indirect[n].update(callback_targets[n, target])
                external[n].discard(target)
    return dict(direct=edges, indirect=indirect, external=external, local=asm_local, dynamic=dynamic, refs=refs)
G = graph(RAW)

def reachable(g):
    """Parcourir transitivement tous les appels internes depuis main, callbacks compris."""
    seen = set()
    todo = ['main']
    while todo:
        n = todo.pop()
        if n in seen:
            continue
        seen.add(n)
        todo.extend((g['direct'][n] | g['indirect'][n]) - seen)
    return seen

def preprocess(flags):
    """Contrôler une configuration avec GCC utilisé uniquement comme préprocesseur."""
    cmd = ['gcc', '-E', '-x', 'c', '-undef', '-nostdinc', '-I', str(ROOT), '-I', str(SDK), '-D__ORCAC__=1'] + ['-D' + x + '=1' for x in flags] + [str(files['objexplorer.cc'])]
    r = subprocess.run(cmd, capture_output=True, text=True, check=True)
    parts = []
    locations = []
    file = None
    line = 0
    for l in r.stdout.splitlines(keepends=True):
        m = re.match('#\\s+(\\d+)\\s+"([^"]+)"', l)
        if m:
            line = int(m[1])
            p = Path(m[2])
            file = p.name if p.name.lower() in sources and (not p.is_absolute() or p.parent.resolve() == ROOT.resolve()) else None
            parts.append('\n')
            locations.append((None, 0))
        else:
            parts.append(l)
            locations.append((file, line))
            line += 1
    d = {f['name']: f for f in definitions(''.join(parts), locations=locations) if f['file'] is not None}
    return (d, graph(d))

def flatten(d):
    """Transformer les listes d’adjacence en couples appelant/appelé."""
    return {(n, k) for n, s in d.items() for k in s}
