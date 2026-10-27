# Analyse de GEO, NEWELL et des contours Z-buffer

Analyse du 27 octobre 2026. Sources actives de la racine ; CSV initial `bench000.csv` fourni sur le Bureau. **Aucun algorithme de production modifié pendant cette analyse.** Les nouveaux temps de l’émulateur précis ne sont pas encore disponibles.

## Ce qui change les priorités

1. **NEWELL a un défaut de capacité reproductible qui peut masquer un repli sur FAST.** Ce point de justesse et de fiabilité des mesures passe avant la simple recherche de vitesse.
2. **GEO combine beaucoup de déplacements entre faces sans chevauchement projeté et un cache de paires qui sature.** Le mécanisme est reproduit dans un scénario contrôlé ; sa part exacte dans les temps GS reste à mesurer.
3. **Les contours rapides ont plusieurs défauts distincts.** Une incohérence d’arrondi est commune au scanline et à l’offscreen ; l’offscreen ajoute des règles de liaison qui peuvent laisser des trous. Une passe complète de contours n’est pas encore justifiée.

## 1. NEWELL : capacité signée et faux résultat « rapide »

Source : [painter_newell_sancha](<F:/Bruno/Dev/AppleWin/Projets/ORCA/OBJ Viewer Review/DonyGS/engine.c:3221>).

La liste de paires est construite avec le seul recouvrement des intervalles Z. Les rejets X/Y viennent ensuite, dans la boucle de résolution. Les capacités et compteurs sont des `int`, donc 16 bits sur ORCA/C.

Lorsque la capacité 16 384 doit doubler :

```text
16 384 × 2 → 32 768, non représentable en int signé 16 bits
conversion cible observée dans le test : -32 768
(size_t)-32 768 × sizeof(CandidatePair)
= 0xFFFE0000 = 4 294 836 224 octets (CandidatePair : deux int 16 bits)
```

`size_t` est bien un **unsigned long 32 bits** dans les headers ORCA/C installés : ce n’est pas un débordement de size_t à 64 Kio. C’est d’abord un problème de capacité `int` signée.

L’allocation échoue ; le chemin existant appelle FAST et revient. Aucun statut détaillé n’est transmis au benchmark. Le CSV peut alors indiquer `ok` alors que NEWELL n’a pas exécuté sa phase géométrique.

### Reconstruction des candidats à H=30°, V=20°, culling off

Ces nombres proviennent d’une reconstruction fixe 16.16 sur PC, avec la normalisation et l’auto-fit du chargeur. Ce ne sont pas des compteurs enregistrés sur le GS.

| Modèle | Faces | Candidats Z actuels | Candidats après rejet X/Y | Réduction de liste possible |
|---|---:|---:|---:|---:|
| a | 67 | 830 | 244 | 70.6 % |
| gear2.obj | 452 | 18 490 | 1 267 | 93.1 % |
| other03.obj | 640 | 30 163 | 1 214 | 96.0 % |
| ring.obj | 384 | 12 378 | 903 | 92.7 % |
| star.obj | 150 | 4 146 | 931 | 77.5 % |
| other01.obj | 256 | 6 585 | 626 | 90.5 % |
| monkey.obj | 500 | 20 897 | 1 630 | 92.2 % |
| stool.obj | 117 | 2 280 | 300 | 86.8 % |
| tore.obj | 192 | 4 578 | 481 | 89.5 % |

### Vérification du chemin de repli

La fonction C NEWELL a été extraite sans changer son contrôle de parcours. Le type local `newcap` a été ramené à 16 bits dans le harnais hôte ; les deux membres de CandidatePair ont aussi été ramenés à 16 bits. Un allocateur de test refuse les demandes démesurées au lieu d’essayer d’allouer 4 Gio.

- `gear2`, `other03`, `monkey` : **deux appels à FAST**, zéro appel aux plans, demande refusée de 4 294 836 224 octets.
- `ring`, `star`, `other01`, `stool`, `tore`, `a` : pas de repli pour ce motif dans cette vue.
- Les prédicats de plans sont factices dans ce test de capacité ; ils ne sont jamais atteints dans les trois replis. Cela n’affecte donc pas la preuve du repli.

Une expérience supplémentaire, sur la même fonction extraite, place le rejet X/Y avant l’ajout du candidat : les neuf modèles passent alors à un seul appel initial à FAST, sans allocation démesurée. Les prédicats de plans restent synthétiques dans cette expérience structurelle ; elle ne valide pas l’ordre géométrique final.

Les temps de tri NEWELL particulièrement courts du CSV (`gear2` : 4 ticks ; `other03` : 5 ; `monkey` : 4) sont **compatibles avec ce repli**. Leur cause sur le GS doit être confirmée par un statut de sortie ou un compteur, plutôt que par une supposition à partir du temps seul.

### Corrections recommandées, sans les appliquer ici

- Vérifier les capacités avant de doubler, avec calcul intermédiaire en long et borne explicite. Changer seulement `newcap` en long ne suffit pas : capacité stockée, `candidate_count` et indices doivent aussi rester cohérents.
- **Rejeter les boîtes X/Y avant de stocker les candidats.** Ces paires sont déjà acceptées sans déplacement plus tard ; cela réduit la liste sans supprimer un test de plan susceptible de déplacer une face. L’ordre des candidats conservés doit être préservé.
- Remplacer le hash global limité à 4 096 relations par un état attaché à chaque candidat : un bit pour chaque orientation acceptée/inconclusive, et le drapeau de déplacement existant. Une paire candidate n’est construite qu’une fois ; les orientations doivent rester distinguées.
- Signaler explicitement un repli à FAST dans le résultat par méthode du CSV.

**Attention : corriger le repli peut augmenter le temps de NEWELL sur certains modèles**, puisqu’il exécutera réellement le classement demandé. Ce serait un changement de travail effectué, pas une régression à interpréter isolément.

## 2. NEWELL sur ring : trop de candidats et garde saturable

La vue reconstruite de `ring` produit 12 378 candidats Z, mais seulement 903 passent les boîtes X/Y : environ **92,7 % de la liste peut être rejetée avant stockage**.

Le hash de relations ne mémorise que 4 096 entrées. Il consomme aussi des entrées pour les paires rejetées par X/Y. Une fois plein, la fonction continue sans mémoriser les nouvelles décisions. Tout déplacement relance une passe sur l’ensemble des candidats ; les paires non mémorisées peuvent alors être retestées.

Autres coûts structurels confirmés :

- insertion stable avec mise à jour des positions de tous les éléments déplacés ;
- `ray_cast_hierarchical` coûteux sur les paires ambiguës ;
- cache dense des relations géométriques disponible jusqu’à 256 faces seulement ; `ring` en a 384 ;
- caches directs de 256 relations / 64 plans pouvant se remplacer par collision.

**Il n’est pas encore prouvé lequel domine les 360 ticks de ring.** Il faut compter passes, candidats visités, relations mémorisées/refusées, insertions et distance d’insertion, appels aux plans et chemins de ray casting. Le filtre X/Y et un état par candidat sont les premiers changements à tester, avant une refonte en file de travail.

## 3. GEO : cache saturé et ordre global coûteux

Sources : [painter_geoV2](<F:/Bruno/Dev/AppleWin/Projets/ORCA/OBJ Viewer Review/DonyGS/engine.c:3853>), [pair_cache_find](<F:/Bruno/Dev/AppleWin/Projets/ORCA/OBJ Viewer Review/DonyGS/engine.c:3743>), [pair_cache_insert](<F:/Bruno/Dev/AppleWin/Projets/ORCA/OBJ Viewer Review/DonyGS/engine.c:3784>), [pair_cache_slot](<F:/Bruno/Dev/AppleWin/Projets/ORCA/OBJ Viewer Review/DonyGS/engine.c:3714>).

### Observations certaines dans le code

- GEO teste les plans avant les rejets de profondeur et de projection. Une décision de plan peut donc échanger des faces dont les projections sont disjointes.
- La paire est mémorisée après décision. Le cache tente 4 096, 2 048 puis 1 024 entrées ; il ne sonde que huit cases. Une insertion peut être abandonnée après huit collisions, même si une autre partie du cache est libre.
- Les recherches ignorent ensuite toute relation non nulle, quel que soit son signe : pour ce parcours, le cache sert surtout de garde « paire déjà traitée ».
- Chaque lookup et chaque sonde emploient un modulo par une capacité dynamique, alors que toutes les capacités effectivement demandées sont des puissances de deux.
- Les comparaisons de voisins et les reprises de passes restent coûteuses même quand un résultat de plan est déjà en cache.

### Rejeu contrôlé, pas profil GS

Les sommets, plans et profondeurs ont été reconstruits en 16.16. Les chemins ambigus ont volontairement été neutralisés (pas de correction par rayon). La fonction C `painter_geoV2` et les véritables fonctions de cache, extraites du code, reproduisent exactement les compteurs du rejeu Python pour ce scénario.

**Ces nombres isolent un mécanisme ; ils ne constituent pas la trace du benchmark réel avec tous ses ray casts.**

| Modèle | Passes | Visites de voisins | Échanges | Échanges sans bbox projetée commune | Insertions de cache abandonnées |
|---|---:|---:|---:|---:|---:|
| a | 22 | 1 117 | 482 | 439 | 0 |
| gear2 | 156 | 58 044 | 18 051 | 18 014 | 30 972 |
| other03 | 151 | 82 441 | 21 549 | 21 171 | 47 995 |
| ring | 19 | 6 960 | 1 276 | 1 227 | 2 |
| star | 11 | 1 394 | 308 | 269 | 0 |
| monkey | 18 | 8 347 | 1 440 | 1 402 | 6 |

Dans ce scénario, le cache est plein sur gear2 et other03. Les abandons répétés de mémorisation font retravailler les mêmes relations ; la très grande majorité des échanges porte sur des faces sans bbox projetée commune.

### Expérience d’une garde sans éviction

Toujours avec les replis ambigus neutralisés, une garde complète des paires déjà traitées donne :

| Modèle | Appels aux plans : cache actuel | Appels aux plans : garde complète | Occasions de rayon actuelles | Occasions avec garde complète |
|---|---:|---:|---:|---:|
| gear2 | 37 105 | 17 838 | 1 333 | 26 |
| other03 | 57 479 | 15 163 | 1 754 | 38 |
| ring | 1 845 | 1 845 | 35 | 35 |

La garde complète peut modifier les décisions après saturation du cache actuel : il faut vérifier la qualité et l’ordre, et pas seulement les compteurs. Aucun gain en ticks GS n’est déduit de cette expérience.

### Optimisations proposées

1. **Petite modification à faible risque : remplacer les modulos par des masques**, en imposant que la capacité soit une puissance de deux. Test isolé effectué : 600 000 opérations mélangées, trois capacités, lectures dans les deux sens et contenu final identiques. Le gain réel reste à mesurer sur 65C816.
2. Garde de paires sans éviction pour les modèles moyens : bitmap triangulaire d’un bit par paire. Pour 640 faces : 204 480 bits, soit **25 560 octets**, contre environ 24 576 octets pour 4 096 entrées de six octets du cache actuel. Ajouter les offsets de lignes éventuels au bilan mémoire. Pour 452 faces : 12 741 octets. Prévoir un repli mémoire pour les très gros modèles.
3. Plus tard seulement, revoir le parcours global des contraintes réellement pertinentes.

**Ne pas ajouter naïvement `if (!bbox_overlap) continue` dans GEO.** Dans un parcours de voisins, une face disjointe peut séparer deux faces qui doivent être comparées. Exemple abstrait `[P, X, Q]`, où P/Q se chevauchent mais X ne chevauche aucune : ignorer les deux couples voisins peut empêcher toute correction de P/Q. Le filtre anticipé est plus sûr dans la liste globale de candidats NEWELL que dans le parcours adjacent GEO.

## 4. Bordures scanline/offscreen : mécanismes reproduits

### 4.1 Une arête n’a pas le même arrondi quand son sens est inversé

Source commune : [ZBuffer_PrepareScanEdges](<F:/Bruno/Dev/AppleWin/Projets/ORCA/OBJ Viewer Review/DonyGS/zbuffer_common.c:461>). Extraction des hits dans les deux moteurs : `xb + trunc(t/4096)` avec traitement séparé du signe de t, après ajout de 2048 à la partie relative.

Test exécuté avec la fonction C extraite, pour l’arête `(10,0) → (0,3)` et la même arête inversée :

| Ligne | Sens aller | Sens retour |
|---|---:|---:|
| y=0 | 10 | 10 |
| y=1 | 8 | 7 |
| y=2 | 4 | 3 |

Le biais d’arrondi porte sur un déplacement relatif à `xb`, puis ce déplacement négatif est tronqué vers zéro. L’origine entière choisie modifie donc le résultat. Deux faces adjacentes peuvent placer leur bord commun sur des pixels voisins.

**Première correction à essayer :** normaliser la représentation de l’intersection (origine au sommet de y minimal, ou coordonnée absolue fractionnaire) et appliquer un arrondi unique, indépendant du winding. Préserver les règles haut inclus/bas exclu et les bornes après clipping. Cette modification est locale ; aucun chiffre de coût GS n’est encore disponible.

Une expérience locale remplace la coordonnée relative par une coordonnée écran absolue fractionnaire et un arrondi unique. Sur **5 000 arêtes inversées**, soit **370 691 échantillons dans le viewport**, l’ancienne règle diverge **366 165 fois** selon le sens ; la variante expérimentale ne diverge plus. La préparation des profondeurs est inchangée. Ce test confirme l’invariance au sens, pas la correction complète des contours ni leur occultation sur une scène. Voir `test_arrondi_absolu.c` et `experience_arrondi.txt`.

### 4.2 Les égalités de profondeur ne remplacent pas un pixel

Source : [ZBuffer_SpanRun](<F:/Bruno/Dev/AppleWin/Projets/ORCA/OBJ Viewer Review/DonyGS/zbuffer_common.c:595>). Les comparaisons assembleur utilisent `BCS` après CMP : code supérieur **ou égal** refusé. C’est une règle cohérente pour les surfaces, mais un contour calculé après une surface au même code ne peut plus la recolorer. Avec les arrondis d’arêtes, l’identité du premier pixel écrit peut changer d’une ligne à l’autre.

Il ne faut pas passer globalement de `<` à `<=` : on risquerait de faire ressortir les arêtes arrière sur des surfaces à profondeur quantifiée égale.

### 4.3 Les liaisons offscreen ne comblent pas les pixels Z vides

Source : [Off_PlotBridge](<F:/Bruno/Dev/AppleWin/Projets/ORCA/OBJ Viewer Review/DonyGS/zbuffer_fullscreen_offscreen.c:312>), appelé par [Off_BridgeBorderZQ](<F:/Bruno/Dev/AppleWin/Projets/ORCA/OBJ Viewer Review/DonyGS/zbuffer_fullscreen_offscreen.c:349>).

Test de la fonction C extraite pour une profondeur d’arête 30 000 :

| Z existant | Pixel de liaison écrit ? |
|---|---|
| 65 535 (vide) | non |
| 30 001 (arête légèrement devant) | non |
| 30 000 (égalité) | oui |
| 29 999 (arête un code derrière) | oui |
| 29 998 | non |

La liaison peut recolorer un pixel déjà présent, mais elle ne remplit pas un trou vide. Les pentes où l’extrémité d’un span se décale de plusieurs pixels entre deux lignes restent donc vulnérables. Le scanline n’a pas cette liaison de contour.

Écrire simplement tous les pixels vides n’est pas une solution complète : sans mettre leur Z à jour, un polygone arrière ultérieur pourrait les recouvrir ; avec mise à jour, il faut vérifier la profondeur et la localisation géométrique du pixel pour ne pas réintroduire les problèmes du rendu direct.

### 4.4 Bordures horizontales structurellement incomplètes

- Scanline : le contour correspond aux deux extrémités de chaque span ; l’intérieur d’un côté horizontal reçoit la couleur de remplissage.
- Offscreen : `whole_span_is_border` reconnaît `miny` et `maxy`, mais les arêtes sont actives sur `[ya, yb[`. Pour une face à fond horizontal, la ligne `maxy` n’a généralement plus de hits ; cette condition ne suffit donc pas à dessiner le côté bas.

Ce sont des différences de stratégie de contour, pas seulement une mauvaise tolérance Z. Une correction d’arrondi peut améliorer les pointillés sans, à elle seule, reproduire tous les contours du moteur direct.

## 5. Autres points arithmétiques à garder dans les tests

- `Off_PlotBridge` soustrait des codes par des `int` de 16 bits. Exemple cible simulé : code 65 534, Z=0, différence signée obtenue -2 ; la condition `<= 1` accepte alors une liaison très éloignée. Employer une différence en long, ou comparer des différences non signées après avoir établi leur sens. Le même motif existe dans le choix de silhouette droite.
- Le pas de span utilise `((long)z1 - z0) << 16`. Une différence absolue supérieure à 32 767 dépasse la plage signée 32 bits avant division. Cela ne prouve pas le défaut visuel courant, mais doit entrer dans les cas extrêmes.
- Les profondeurs calculées le long d’arêtes projetées arrondies ne sont pas le même modèle numérique que le plan observateur validé du rendu direct. Unifier ce point plus tard peut améliorer la cohérence, avec un coût à mesurer séparément.
- La garde temporelle de `projected_polygons_overlap_v1` est nommée `..._MS`, mais compare des ticks GetTick : la borne 1 000 représente environ 16,7 secondes, pas 1 seconde. Son dépassement rend 0 (« pas d’overlap »), ce qui mélange abandon et résultat géométrique. Aucun dépassement réel n’a été mesuré ici.

## 6. Ordre d’intervention recommandé

| Étape | Travail | Validation indispensable |
|---|---|---|
| 1 | Corriger le dimensionnement et le statut de repli NEWELL ; filtrer X/Y avant stockage | gear2/other03/monkey doivent exécuter NEWELL, pas être silencieusement FAST |
| 2 | Mémorisation par candidat dans NEWELL | Comparer ordre final et image de ring/star/other01 ; compter passes et rayons |
| 3 | Masques du cache GEO, puis garde complète bornée par mémoire | Comparer relations, ordre, qualité ; compter abandons et appels aux plans |
| 4 | Arrondi d’arête commun aux deux Z-buffers rapides | Même arête dans les deux sens, pentes positives/négatives, pan et clipping |
| 5 | Politique ciblée de bordures horizontales et de liaisons | A/jet aux vues 30° et 110°, culling on/off, faces coplanaires et occultées |
| 6 | Seulement si nécessaire, noyau léger de contours spécifique | Mesurer son coût séparément et conserver les gains de scanline |

Ne pas changer toutes ces règles à la fois. Conserver le rendu direct validé comme référence visuelle, sans supposer qu’il est une vérité parfaite pour des géométries dégénérées.

## 7. Ce qu’il reste à mesurer sur le GS

- Une sortie de méthode explicite : complet, repli, limite de passes, allocation impossible. `status=ok` au niveau du modèle n’est pas suffisant.
- NEWELL : candidats Z et X/Y, capacité maximale, passes, visites, état saturé, insertions et distance totale déplacée, appels de rayons fractionnaires / replis entiers / QuickDraw.
- GEO : visites, hits/miss, nombre de sondes, insertions refusées, échanges avec/sans bbox commune et nombre de passages.
- Faire une exécution de diagnostic avec compteurs, puis une exécution sans instrumentation pour les temps : les compteurs coûtent du temps sur 65C816.
- Comparer les essais uniquement à version, orientation, culling, méthode d’overlap et configuration d’émulation identiques. Le nouveau CSV en mode texte n’a pas les mêmes colonnes ni le même périmètre que le CSV initial avec dessin.

## 8. Vérifications effectuées et limites

- Géométrie et nombres de candidats reconstruits sur neuf modèles du dépôt. Sur les huit retrouvés dans le CSV initial, six échelles de projection sont identiques ; ring diffère de 4 unités 16.16 et star de 10, sans changement d’un seul pixel projeté entier lorsque l’échelle du CSV est réinjectée.
- Fonction C NEWELL extraite : reproduction des trois replis avant tout test de plan.
- Fonction C GEO et cache réels : compteurs identiques au rejeu contrôlé, sur six modèles.
- Expérience de garde sans éviction, sur trois modèles, avec prédicats ambigus neutralisés.
- Modulo / masque : 600 000 opérations de cache équivalentes, trois capacités.
- Fonctions communes d’arêtes et de liaison extraites : reproduction des contre-exemples d’arrondi et de visibilité.
- Différences signées 16 bits vérifiées séparément, car un int hôte est généralement 32 bits.

**Pas de chronométrage ni de validation visuelle sur GS/émulateur par l’agent.** La reconstitution numérique peut différer d’un pixel à certaines limites selon les arrondis de bibliothèque ; les seuils de capacité et contre-exemples synthétiques ne dépendent pas d’un temps mesuré sur PC. Aucun pourcentage de gain futur n’est annoncé.

Pour relancer les tests figés : `python executer_tests.py` depuis ce dossier (GCC nécessaire). `reconstruire_geometrie.py` peut régénérer les fixtures depuis les modèles du dépôt.

Les données JSON et les tests C présents dans ce répertoire rendent les observations contrôlables. Le fichier CSV initial n’a pas été modifié. Les sources de production, le binaire et l’image disque n’ont pas été modifiés par ce travail d’analyse.
