# Audit de l’arbre d’appels depuis main

Vérification du 27 octobre 2026, sur les sources actives incluses depuis `ObjExplorer.cc`. Les deux pages concernées sont [l’arbre ouvert](arbre_appels.html) et [l’arbre repliable](arbre_appels_fold.html). Le code du programme n’a pas été modifié par cet audit.

## Résultat et corrections

- **18 fichiers inclus, 259 définitions et 259 fonctions accessibles depuis main** dans le graphe statique réunissant les variantes.
- **455 relations directes**, C ou assembleur, et **2 relations indirectes par callback**.
- Les listes recensant les 455 appels directs et leurs relations inverses correspondaient déjà au code : aucun appel direct inventé ou manquant trouvé dans les fiches.
- L’imbrication erronée de l’animation sous `check_intersect` avait été corrigée. Le parcours a maintenant été entièrement reconstruit : les **457 relations internes** sont représentées depuis `main`, au lieu d’un aperçu limité à deux niveaux. Chaque fonction est détaillée une fois ; les références communes et les cycles statiques sont signalés.
- Les deux callbacks étaient indiqués dans les appelants, mais absents des dépendances dans le sens appelant → appelé. Ils sont maintenant explicitement distingués des appels directs dans les deux graphes et le dictionnaire.
- Les signatures, fichiers, lignes de définition, identifiants, liens locaux, ancres, fermetures HTML et parents effectifs des branches sont vérifiés. Une référence du dictionnaire vers le fichier disparu `REVISION_DOCUMENTATION.md` est remplacée par ce rapport.
- Les versions ouverte et repliée décrivent exactement les mêmes fonctions et relations. Les boutons de dépliage agissent aussi sur le parcours complet.

## Relations nécessitant une interprétation

| Relation | Nature et condition |
| --- | --- |
| `main` → `animate_model_rotation` | Appel direct depuis la touche `/`. Aucun appel depuis `check_intersect`. |
| `painter_newell_sancha_fast` → `cmp_faces_by_zmean` | Comparateur invoqué indirectement par `qsort`. |
| `geov3_run` → `repair_geov3_confirm` | Callback `on_exchange`, fourni par `repair_geov3`. `painter_geoV3` passe NULL et ne déclenche pas cet inspecteur. |
| `FaceDepth_ScanKernel` → `depth_scan_edge` | Sous-routine assembleur locale à la fonction, sans définition C distincte. |
| `animation_execute_block` → code généré en RAM | JSL auto-modifiant vers les transitions compilées ; ce n’est pas une fonction C supplémentaire. |

`processModelFast` appelle les méthodes de classement seulement lorsque `framePolyOnly` ne les court-circuite pas. L’animation et les inspecteurs peuvent donc l’appeler pour recalculer la géométrie sans exécuter ces tris. Les arcs du graphe décrivent les appels possibles dans la fonction, pas la preuve que toutes ses branches sont suivies à chaque invocation.

## Méthode

1. Suivre les inclusions locales depuis `ObjExplorer.cc`, y compris les includes locaux entre chevrons. `old/`, `tools/`, `DEPLOY.py` et les définitions externes du SDK ne sont pas des nœuds du graphe.
2. Utiliser une analyse lexicale indépendante de l’ancien générateur : ignorer commentaires et chaînes, équilibrer les délimiteurs, repérer les définitions globales et les appels dans chaque corps. Distinguer JSL/JSR, labels locaux, appels numériques et références de callbacks. Aucune référence à une fonction du projet ne reste non résolue dans cet état des sources.
3. Contrôler les cibles des deux callbacks à leurs sites de passage : argument comparateur de `qsort` et paramètre `on_exchange` de `geov3_run`.
4. Utiliser GCC **uniquement comme préprocesseur**, avec les en-têtes ORCA/C, sur les 16 combinaisons des quatre options assembleur. Toutes les relations directes du graphe sont retrouvées dans au moins une configuration ; aucun appel supplémentaire inexpliqué n’est produit.
5. Parcourir transitivement depuis `main`, puis comparer chaque liste d’adjacence, sa liste inverse et chaque parent HTML réel. Contrôler aussi les listes d’appels du dictionnaire et les métadonnées des fiches.
6. Injecter en mémoire sept erreurs : parent déclaré faux, lien d’ancre absent, fichier lié absent, identifiant dupliqué, appel direct inventé, callback supprimé et ancienne imbrication physique sous `check_intersect`. Le vérificateur les détecte. La génération est également idempotente.

## Configurations contrôlées

`M`, `R`, `S`, `D` indiquent respectivement si `GEOV3_DISABLE_ASM_MUL`, `GEOV3_DISABLE_ASM_RELATION`, `GEOV3_DISABLE_ASM_SCAN` et `FACE_DEPTH_DISABLE_ASM` sont définis à 1. Un tiret signifie que l’option n’est pas définie.

| M | R | S | D | Définitions | Relations directes | Fonctions accessibles |
| --- | --- | --- | --- | ---: | ---: | ---: |
| — | — | — | — | 259 | 454 | 259 |
| — | — | — | 1 | 255 | 450 | 255 |
| — | — | 1 | — | 256 | 451 | 256 |
| — | — | 1 | 1 | 252 | 447 | 252 |
| — | 1 | — | — | 259 | 453 | 256 |
| — | 1 | — | 1 | 255 | 449 | 252 |
| — | 1 | 1 | — | 256 | 450 | 253 |
| — | 1 | 1 | 1 | 252 | 446 | 249 |
| 1 | — | — | — | 255 | 446 | 250 |
| 1 | — | — | 1 | 255 | 446 | 250 |
| 1 | — | 1 | — | 252 | 443 | 247 |
| 1 | — | 1 | 1 | 252 | 443 | 247 |
| 1 | 1 | — | — | 255 | 446 | 250 |
| 1 | 1 | — | 1 | 255 | 446 | 250 |
| 1 | 1 | 1 | — | 252 | 443 | 247 |
| 1 | 1 | 1 | 1 | 252 | 443 | 247 |

Sans option personnalisée : **259 définitions, 454 relations directes, 2 callbacks**. L’arc direct supplémentaire de la réunion des variantes est `calculateFaceDepths` → `fixed_product32_fast`, présent dans le chemin C lorsque le calcul assembleur des profondeurs est désactivé.

## Reproduire le contrôle

Depuis la racine du projet :

```text
python tools/verify_call_graph.py
python tools/verify_call_graph.py --preprocess --self-test
```

Pour synchroniser les relations après une modification :

```text
python tools/verify_call_graph.py --update --preprocess --self-test
```

Les outils locaux sont `tools/call_graph_source.py` et `tools/verify_call_graph.py`. Ils peuvent être conservés localement si ce dossier est exclu des commits ; ils ne modifient aucun fichier C, assembleur, exécutable ou image disque. Le contrôle simple utilise seulement Python. `--preprocess` demande GCC et les en-têtes ORCA/C, détectés ici dans `C:/ProgramData/GoldenGate/Libraries/ORCACDefs` ; `--sdk` permet de changer ce chemin. Une nouvelle fonction sans fiche est signalée : le script n’invente pas son rôle ni son contrat.

## Limites

Ce contrôle établit la cohérence du **graphe statique des appels internes** de cet état du projet. Il ne prouve ni la correction des algorithmes, ni les conditions réellement exécutées, ni l’absence de régression du programme. Les rôles et contrats rédigés ne font pas l’objet d’une nouvelle validation algorithmique. Le fonctionnement interne des bibliothèques, des Toolbox et des blocs de pixels compilés est hors du graphe des fonctions C du projet. Les futurs pointeurs de fonctions ou mécanismes d’appel nouveaux doivent être revus et pris en charge explicitement.

## Empreintes des sources examinées

Préfixes SHA-256, pour identifier l’état vérifié :

| Fichier | Empreinte |
| --- | --- |
| `ObjExplorer.cc` | `045b246767a42ac2` |
| `animation.c` | `7977488c67fecbb1` |
| `asm.h` | `f6c850fbbe08fbef` |
| `color.c` | `92a49e74bd88012d` |
| `declare.c` | `a8de68c09038b0c6` |
| `dony.c` | `a80e6b08c9a72928` |
| `dony_scan_asm.c` | `0c1357d2f5dac28d` |
| `engine.c` | `8dcb3f55b61133f3` |
| `face_depth_asm.c` | `e79c997d6223f4bc` |
| `geo_relation_asm.c` | `2f422deb0f836564` |
| `ordering_cache.c` | `d914a38749b8b1ab` |
| `overlap_raster.c` | `5ffcea0bdf532f28` |
| `screen_save_restore.c` | `ac110bae77c95627` |
| `zbuffer_common.c` | `0c73009488421325` |
| `zbuffer_fullscreen.c` | `0f1fb6d76c7eaa6b` |
| `zbuffer_fullscreen.h` | `892c5242b16c4e8d` |
| `zbuffer_fullscreen_offscreen.c` | `083a3e5f170d859f` |
| `zbuffer_scanline.c` | `ebeedfd78c2731b9` |
