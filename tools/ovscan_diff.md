# OVScan — Différences C (fixed) vs Python (float)

Résumé basé sur `ovscan.csv` (scan global) :

- Total paires analysées : **96**
- Paires où **fixed ≠ float** : **35**
- Paires où la décision courante (C) diffère du résultat fixed (override appliqué) : **26**

> Les aires sont en pixels². *Python* ci‑dessous correspond au résultat de la passe double (float) ajoutée pour diagnostic.

## Paires où la décision C (courante) et Python (float) diffèrent



| # | Paire | C decision | Python decision | ia1_fixed | ia1_float | ia2_fixed | ia2_float | bbox_area | notes |
|---:|:-----:|:----------:|:---------------:|---------:|---------:|---------:|---------:|---------:|:-----:|
| 1 | 3,50 | NO | YES | 1322.598480 | 19.959961 | 1322.598480 | 19.959961 | 36.00 | ov_fixed!=ov_float |
| 2 | 4,7 | NO | YES | 1322.598480 | 19.959961 | 1322.598480 | 19.959961 | 30.00 | ov_fixed!=ov_float |
| 3 | 7,9 | YES | NO | 34.153800 | 0.000000 | 1867.844258 | 0.000000 | 105.00 | ov_fixed!=ov_float |
| 4 | 3,45 | NO | YES | 62.218712 | 0.000000 | 1322.598480 | 19.959961 | 108.00 | cur!=fixed |
| 5 | 9,44 | YES | NO | 1322.783478 | 0.000000 | 0.000000 | 0.000000 | 1848.00 | ov_fixed!=ov_float |
| 6 | 0,44 | YES | NO | 21.666641 | 0.000000 | 1032.399274 | 262.103717 | 90.00 | ov_fixed!=ov_float |
| 7 | 1,43 | NO | YES | 327.305893 | 25.785569 | 327.305893 | 25.785569 | 80.00 | ov_fixed!=ov_float |
| 8 | 1,44 | NO | YES | 327.305893 | 25.785569 | 327.305893 | 25.785569 | 150.00 | ov_fixed!=ov_float |
| 9 | 0,45 | NO | YES | 78.186962 | 0.000000 | 327.305893 | 25.785569 | 90.00 | cur!=fixed |
| 10 | 1,45 | NO | YES | 210.070390 | 0.000000 | 0.000000 | 42.616837 | 150.00 | ov_fixed!=ov_float |
| 11 | 6,44 | YES | NO | 81.651428 | 0.000000 | 81.651428 | 0.000000 | 105.00 | ov_fixed!=ov_float |
| 12 | 13,18 | YES | NO | 0.000000 | 0.000000 | 141.513092 | 0.000000 | 310.00 | ov_fixed!=ov_float |
| 13 | 6,9 | YES | NO | 52.500000 | 0.000000 | 52.500000 | 0.000000 | 105.00 | ov_fixed!=ov_float |
| 14 | 3,9 | YES | NO | 49.429181 | 0.000000 | 49.429181 | 0.000000 | 54.00 | ov_fixed!=ov_float |
| 15 | 4,9 | YES | NO | 44.262783 | 0.000000 | 44.262783 | 0.000000 | 42.00 | cur!=fixed |
| 16 | 3,44 | YES | NO | 35.466614 | 0.000000 | 35.466614 | 0.000000 | 108.00 | ov_fixed!=ov_float |
| 17 | 12,50 | YES | NO | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 3150.00 | cur!=fixed |


## Autres divergences significatives (fixed != float)



| # | Paire | fixed ov | float ov | ia1_fixed | ia1_float | ia2_fixed | ia2_float | bbox_area | notes |
|---:|:-----:|:--------:|:--------:|---------:|---------:|---------:|---------:|---------:|:-----:|
| 1 | 9,43 | NO | NO | 2615.756077 | 0.000000 | 2615.756077 | 0.000000 | 1344.00 |  |
| 2 | 7,43 | NO | NO | 1867.844258 | 0.000000 | 2091.804016 | 0.000000 | 105.00 |  |
| 3 | 13,60 | NO | NO | 1776.127612 | 0.000000 | 1776.127612 | 0.000000 | 51.00 |  |
| 4 | 15,18 | NO | NO | 1776.127612 | 0.000000 | 1776.127612 | 0.000000 | 80.00 |  |
| 5 | 19,20 | NO | NO | 1776.127612 | 0.000000 | 1776.127612 | 0.000000 | 84.00 |  |
| 6 | 19,21 | NO | NO | 1776.127612 | 0.000000 | 1776.127612 | 0.000000 | 72.00 |  |
| 7 | 19,26 | NO | NO | 1776.127612 | 0.000000 | 1776.127612 | 0.000000 | 15.00 |  |
| 8 | 19,27 | NO | NO | 1776.127612 | 0.000000 | 1776.127612 | 0.000000 | 5.00 |  |
| 9 | 20,21 | NO | NO | 1776.127612 | 0.000000 | 1776.127612 | 0.000000 | 108.00 |  |
| 10 | 20,26 | NO | NO | 1776.127612 | 0.000000 | 1776.127612 | 0.000000 | 14.00 |  |
| 11 | 36,45 | NO | NO | 187.038831 | 0.000000 | 3215.117055 | 0.000000 | 13.00 |  |
| 12 | 9,45 | NO | NO | 3382.663116 | 0.000000 | 0.000000 | 0.000000 | 1512.00 |  |
| 13 | 7,45 | YES | NO | 85.846750 | 0.000000 | 2615.756077 | 0.000000 | 105.00 | ov_fixed!=ov_float;cur!=fixed |
| 14 | 12,43 | YES | NO | 2411.686481 | 0.000000 | 50.878499 | 0.000000 | 1995.00 | ov_fixed!=ov_float;cur!=fixed |
| 15 | 7,44 | NO | NO | 0.524560 | 0.000000 | 2245.876953 | 0.000000 | 105.00 |  |
| 16 | 13,15 | YES | YES | 817.486542 | 207.318381 | 259.219618 | 0.000000 | 315.00 |  |
| 17 | 0,50 | NO | NO | 327.305893 | 25.785569 | 327.305893 | 25.785569 | 24.00 |  |
| 18 | 1,4 | NO | NO | 327.305893 | 25.785569 | 327.305893 | 25.785569 | 24.00 |  |
| 19 | 36,37 | YES | NO | 187.038831 | 0.000000 | 187.038831 | 0.000000 | 555.00 | ov_fixed!=ov_float;cur!=fixed |
| 20 | 36,44 | NO | NO | 187.038831 | 0.000000 | 187.038831 | 0.000000 | 70.00 |  

> Voir `ovscan.csv` pour le tableau complet et `ovscanrpt.txt` pour le résumé ordre/détections.

---

## Interprétation courte
- Les divergences sont non négligeables (35 cas) et impactent la décision dans ~26 cas.
- La plupart des écarts notables sont des cas où l'aire *fixed* est grande (ou parfois petite ~0.5 px²) tandis que l'aire *float* est reportée à 0, ce qui explique les comportements divergents.

## Recommandation (rappel)
- Appliquer un override conservateur : **n’overrider que si l’aire float < 1.0 px²** (ou selon `MIN_INTERSECTION_AREA_PIXELS`).
- Conserver la liste des paires divergentes comme jeu de tests de régression.

---

Fichiers générés par le scan : `ovscan.csv` (détaillé) et `ovscanrpt.txt` (résumé).

Si tu veux, j’ajoute aussi une version Markdown complète du `ovscan.csv` filtrée sur `ov_fixed!=ov_float` ou `cur!=fixed` pour inspection rapide.

## Analyse Visualle :
Face DécisionC DécisionPy : réalité_visuelle  Vainqueur
3,50 NO YES : NO (C)
4,7	NO YES : NO (C)
7,9	YES	NO : YES (C)
3,45 NO	YES : YES (Python)
9,44 YES NO : ?
0,44	YES	NO : YES (C)
1,43	NO	YES : NO (C)
1,44	NO	YES : NO (C)
0,45	NO	YES : YES (Python)
1,45	NO	YES : NO (C)
6,44	YES	NO : YES (C)
13,18	YES	NO : YES (C)
6,9	YES	NO : YES (C)
3,9	YES	NO : YES (C)
4,9	YES	NO : YES (C)
3,44	YES	NO : YES (C)
12,50	YES	NO : YES (C)