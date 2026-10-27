"""Rejouer les tests d'analyse figés, sans modifier les sources de production."""
from pathlib import Path
import json
import subprocess
import tempfile

BASE = Path(__file__).resolve().parent

def geometry(name):
    return json.loads((BASE / (name + '_geometry.json')).read_text(encoding='utf-8'))

def newell_input(data):
    metrics = data['metrics']
    return str(len(metrics)) + '\n' + ''.join(
        ' '.join(str(m[k]) for k in ('zmin','zmax','zmean','minx','maxx','miny','maxy'))+'\n'
        for m in metrics)

def geo_input(data):
    text = f"{len(data['vertices'])} {len(data['faces'])}\n"
    text += ''.join(' '.join(map(str,v))+'\n' for v in data['observer'])
    for ids, m in zip(data['faces'], data['metrics']):
        values = [m[k] for k in ('zmin','zmax','zmean','minx','maxx','miny','maxy')]
        values += m['plane'] + [len(ids)] + ids
        text += ' '.join(map(str,values))+'\n'
    return text

with tempfile.TemporaryDirectory(prefix='dony_analyse_') as folder:
    executables = {}
    for source in sorted(BASE.glob('test_*.c')):
        exe = Path(folder) / (source.stem + '.exe')
        subprocess.run(['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror',str(source),'-o',str(exe)],check=True)
        executables[source.stem] = exe
    def run(name, data=''):
        return subprocess.run([str(executables[name])],input=data,text=True,stdout=subprocess.PIPE,check=True).stdout
    for name in ('test_bordures','test_cache_masque','test_arrondi_absolu'):
        print(run(name).strip())
    for item in json.loads((BASE/'newell_capacite.json').read_text(encoding='utf-8')):
        actual = list(map(int,run('test_capacite_newell',newell_input(geometry(item['model']))).split()))
        expected = [item[k] for k in ('fast_calls','candidates_if_reached','plane_calls_if_reached','rejected_realloc_bytes')]
        assert actual == expected, (item['model'],actual,expected)
    print('PASS : capacité NEWELL, neuf modèles.')
    for item in json.loads((BASE/'newell_experience_filtre.json').read_text(encoding='utf-8')):
        actual = list(map(int,run('test_filtre_newell',newell_input(geometry(item['model']))).split()))
        assert actual == [item[k] for k in ('fast_calls','candidates','plane_calls','rejected_bytes')]
    print('PASS : expérience filtre NEWELL, neuf modèles.')
    keys = ('passes','adjacent_visits','find_calls','find_probes','insert_probes','probe_limit_misses','insert_drops','swaps','swaps_without_bbox_overlap','plane_calls')
    for item in json.loads((BASE/'geo_rejeu_C.json').read_text(encoding='utf-8')):
        actual = list(map(int,run('test_parcours_geo',geo_input(geometry(item['model']))).split()))
        assert actual == [item[k] for k in keys], item['model']
    print('PASS : GEO contrôlé, six modèles, replis ambigus neutralisés.')
print('Tous les tests figés passent. Aucun chronométrage GS effectué.')
