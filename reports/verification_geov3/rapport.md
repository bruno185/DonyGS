# Comparaison de GEO V3 avant et après le filtre de chevauchement

## Conclusion

Le nouveau filtre global **n’est pas équivalent** au GEO V3 précédent. Il corrige le faux cycle observé avec les fragments de `intersect2`, mais change aussi la trajectoire des permutations sur des modèles ordinaires. Cette modification ne doit donc pas être présentée comme validée sans régression.

Le code de production n’a pas été modifié pendant cette vérification.

## Protocole

- 21 modèles : objets simples, surfaces courbes, modèles articulés ou complexes, et trois petits cas d’intersection.
- 20 vues par modèle : la vue H=150°, V=20°, rotation=0°, puis 19 vues pseudo-aléatoires reproductibles.
- Graine : `0x6581627`, générateur `état = état * 1664525 + 1013904223`, calcul modulo 2^32.
- Angles : H de 0 à 359°, V de −75 à 75°, rotation de −180 à 180° ; distance 150 et auto-fit du modèle.
- Chaque vue est testée avec et sans culling : **840 configurations**.
- Découpage au chargement uniquement pour `intersect2.obj`, `intersect3.obj` et `intersect.obj`.
- Comparaison des deux corps réels de `geov3_run`, des permutations, puis de la face visible sur une grille 320 × 200.
- Méthode de chevauchement : V1, seuil d’aire de 2 pixels carrés, identique pour les deux versions.
- Les caches sont invalidés et la vue recalculée avant chacune des deux exécutions.

## Limites indispensables

Ce banc tourne sur le PC avec les chemins C, les coordonnées Fixed32 sur 32 bits et les opérations géométriques du programme. Il **n’exécute ni le 65C816 ni QuickDraw sur le GS**.

Les fonctions de région QuickDraw sont remplacées par un résultat indisponible, comptabilisé pour chaque vue. Une vue nécessitant ce repli est exclue du sous-ensemble sans ce défaut de couverture. Cela concerne 380 configurations ; les 460 restantes ne l’appellent dans aucune des deux versions. Le watchdog temporel est désactivé dans le banc.

La comparaison graphique porte sur la **visibilité des faces**, par remplissage pair/impair aux centres des pixels. Elle ne reproduit pas les contours ni toutes les conventions de rasterisation de QuickDraw. Les couleurs du SVG sont des identifiants synthétiques de faces. Un changement de face visible peut être masqué par des couleurs identiques dans le programme.

Le classement « plus près / plus loin » des pixels modifiés provient des plans observateur, avec une tolérance relative de 10⁻⁵. C’est un indicateur de régression à examiner, pas une preuve absolue de la géométrie d’un polygone dégénéré ou non plan.

Les durées PC du JSON ne représentent pas les performances sur GS et ne doivent pas être utilisées pour annoncer un gain ou une perte sur la machine cible.

## Résultats

| Mesure | Toutes les configurations | Sans recours QuickDraw non implémenté |
| --- | ---: | ---: |
| Configurations | 840 | 460 |
| Permutation différente | 566 | 190 |
| Visibilité logicielle différente | 161 | 34 |
| Au moins un pixel choisissant une face plus lointaine | 116 | 18 |

Dans les 460 configurations sans repli QuickDraw, 34 changent la visibilité logicielle et 18 comportent au moins un pixel choisissant une face plus lointaine. Ces 18 configurations totalisent 80 pixels de cette catégorie, contre 732 pixels choisissant une face plus proche. Une amélioration à certains endroits ne garantit donc pas l’absence de régression ailleurs.

| Modèle | Faces après chargement | Configurations | Visibilité différente | Configurations sans repli QD | Dont au moins un pixel plus lointain |
| --- | ---: | ---: | ---: | ---: | ---: |
| `a` | 67 | 40 | 20 | 18 | 2 |
| `cube.obj` | 6 | 40 | 0 | 40 | 0 |
| `cylinder.obj` | 48 | 40 | 0 | 40 | 0 |
| `capsule.obj` | 192 | 40 | 0 | 40 | 0 |
| `jet.obj` | 67 | 40 | 26 | 32 | 12 |
| `star.obj` | 150 | 40 | 38 | 0 | 0 |
| `stool.obj` | 117 | 40 | 1 | 0 | 0 |
| `tore.obj` | 192 | 40 | 0 | 32 | 0 |
| `ring.obj` | 384 | 40 | 4 | 0 | 0 |
| `monkey.obj` | 500 | 40 | 36 | 0 | 0 |
| `other01.obj` | 256 | 40 | 0 | 0 | 0 |
| `other02.obj` | 26 | 40 | 3 | 0 | 0 |
| `other03.obj` | 640 | 40 | 5 | 0 | 0 |
| `gear2.obj` | 452 | 40 | 16 | 0 | 0 |
| `horn.obj` | 144 | 40 | 4 | 38 | 1 |
| `barbar.obj` | 470 | 40 | 6 | 24 | 1 |
| `charpente.obj` | 96 | 40 | 0 | 36 | 0 |
| `cross.obj` | 22 | 40 | 0 | 40 | 0 |
| `intersect2.obj` | 3 | 40 | 0 | 40 | 0 |
| `intersect3.obj` | 3 | 40 | 0 | 40 | 0 |
| `intersect.obj` | 3 | 40 | 2 | 40 | 2 |

## Cas concret à vérifier sur GS

- Modèle : `jet.obj` (79 sommets, 67 faces), sans découpage.
- H=244°, V=51°, rotation=−12°, culling OFF, distance 150, auto-fit.
- Projection du banc : 957,97480774.
- Aucun appel aux replis de région QuickDraw dans les deux exécutions.
- 15 pixels changent de face ; les 15 choisissent une face plus lointaine dans la nouvelle version.
- Exemple : au pixel (127,79), l’ancien tri montre la face 46 à z≈155,54267 ; le nouveau montre la face 10 à z≈155,64864.
- Pour ces deux faces, le test de chevauchement V1 renvoie bien 1 : l’écart est donc lié à la trajectoire globale des échanges, pas simplement à la suppression directe de cet échange précis.

[Comparaison de visibilité et zoom](cas_jet.svg) · [détails des pixels](cas_jet.txt) · [données des deux images](cas_jet.json).

## Suite proposée

Conserver le GEO V3 précédent comme référence de qualité. Le filtre ajouté à tous les échanges est trop général pour garantir les mêmes rendus. La correction du découpage devrait être étudiée plus localement, notamment pour les fragments issus d’une même face, puis soumise à la même comparaison. Aucune nouvelle correction n’a été appliquée dans cet audit.

## Reproduction et pièces jointes

- [Banc C autonome, avec les deux versions capturées](comparaison_geov3.c).
- [Lanceur Python](executer.py), qui normalise seulement les fins de ligne des copies de modèles et compile avec GCC.
- [840 résultats détaillés](resultats.json).
- [Empreintes des sources actives au début de la vérification](empreintes_sources.json).

Depuis la racine du dépôt : `python reports/verification_geov3/executer.py`. Les sources de production ne sont pas modifiées par ce lanceur.
