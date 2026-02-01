import csv
from collections import Counter

infile = 'ovscan.csv'
summary = {}
rows = []
with open(infile, newline='') as cf:
    reader = csv.DictReader(cf)
    for r in reader:
        rows.append(r)

total = len(rows)
divergent = sum(1 for r in rows if r['notes'] and 'ov_fixed!=ov_float' in r['notes'])
curdiff = sum(1 for r in rows if r['notes'] and 'cur!=fixed' in r['notes'])
override_changed = sum(1 for r in rows if r['ov_current'] != r['ov_fixed'])

# Find top pairs where absolute difference between fixed and float areas is largest (sum of two orders)
import math
pairs_diff = []
for r in rows:
    try:
        ia1f = float(r['ia1_fixed'])
        ia2f = float(r['ia2_fixed'])
        ia1d = float(r['ia1_float'])
        ia2d = float(r['ia2_float'])
    except:
        continue
    diff = abs(ia1f - ia1d) + abs(ia2f - ia2d)
    pairs_diff.append((diff, r))

pairs_diff.sort(reverse=True, key=lambda x: x[0])

# Write summary
with open('ovscanrpt.txt', 'w') as rf:
    rf.write(f"total_pairs,{total}\n")
    rf.write(f"divergent_pairs,ov_fixed!=ov_float,{divergent}\n")
    rf.write(f"cur_vs_fixed,{curdiff}\n")
    rf.write(f"override_changed (current!=fixed),{override_changed}\n")
    rf.write("\nTop 20 divergences (diff, f1,f2,ov_fixed,ov_float,ia1_fixed,ia1_float,ia2_fixed,ia2_float,notes):\n")
    for i, (d, r) in enumerate(pairs_diff[:20]):
        rf.write(f"{i+1},{d},{r['f1']},{r['f2']},{r['ov_fixed']},{r['ov_float']},{r['ia1_fixed']},{r['ia1_float']},{r['ia2_fixed']},{r['ia2_float']},{r['notes']}\n")

print('Summary written to ovscanrpt.txt')
print('total_pairs', total, 'divergent_pairs', divergent, 'cur_vs_fixed', curdiff, 'override_changed', override_changed)