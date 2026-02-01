import csv
from operator import itemgetter

rows = []
with open('ovscan.csv', newline='') as cf:
    reader = csv.DictReader(cf)
    for r in reader:
        # normalize fields
        r['f1'] = int(r['f1']); r['f2'] = int(r['f2'])
        r['ov_current'] = int(r['ov_current']); r['ov_fixed'] = int(r['ov_fixed']); r['ov_float'] = int(r['ov_float'])
        for k in ['ia1_fixed','ia2_fixed','ia1_float','ia2_float','bbox_area']:
            try: r[k] = float(r[k])
            except: r[k] = 0.0
        # compute diff metric
        r['diff'] = abs(r['ia1_fixed'] - r['ia1_float']) + abs(r['ia2_fixed'] - r['ia2_float'])
        rows.append(r)

# filter decision different between C (current) and Python (float)
diff_dec = [r for r in rows if r['ov_current'] != r['ov_float']]
other_divs = [r for r in rows if r['ov_current'] == r['ov_float'] and (r['ov_fixed'] != r['ov_float'] or r['diff']>0)]

# sort by diff desc
diff_dec.sort(key=itemgetter('diff'), reverse=True)
other_divs.sort(key=itemgetter('diff'), reverse=True)

lines = []
lines.append('## Paires où la décision C (courante) et Python (float) diffèrent\n')
lines.append('\n')
lines.append('| # | Paire | C decision | Python decision | ia1_fixed | ia1_float | ia2_fixed | ia2_float | bbox_area | notes |')
lines.append('|---:|:-----:|:----------:|:---------------:|---------:|---------:|---------:|---------:|---------:|:-----:|')
for i,r in enumerate(diff_dec):
    c = 'YES' if r['ov_current'] else 'NO'
    p = 'YES' if r['ov_float'] else 'NO'
    notes = r['notes'] if r['notes'] else ''
    lines.append(f"| {i+1} | {r['f1']},{r['f2']} | {c} | {p} | {r['ia1_fixed']:.6f} | {r['ia1_float']:.6f} | {r['ia2_fixed']:.6f} | {r['ia2_float']:.6f} | {r['bbox_area']:.2f} | {notes} |")

lines.append('\n')
lines.append('## Autres divergences significatives (fixed != float)\n')
lines.append('\n')
lines.append('| # | Paire | fixed ov | float ov | ia1_fixed | ia1_float | ia2_fixed | ia2_float | bbox_area | notes |')
lines.append('|---:|:-----:|:--------:|:--------:|---------:|---------:|---------:|---------:|---------:|:-----:|')
for i,r in enumerate(other_divs[:20]):
    fv = 'YES' if r['ov_fixed'] else 'NO'
    pv = 'YES' if r['ov_float'] else 'NO'
    notes = r['notes'] if r['notes'] else ''
    lines.append(f"| {i+1} | {r['f1']},{r['f2']} | {fv} | {pv} | {r['ia1_fixed']:.6f} | {r['ia1_float']:.6f} | {r['ia2_fixed']:.6f} | {r['ia2_float']:.6f} | {r['bbox_area']:.2f} | {notes} |")

# write generated md
with open('ovscan_diff_generated.md','w') as gf:
    gf.write('\n'.join(lines))
print('Generated ovscan_diff_generated.md')
