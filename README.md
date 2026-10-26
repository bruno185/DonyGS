# Explorateur OBJ 3D pour Apple IIgs

Ce projet charge, oriente et examine des modèles Wavefront OBJ sur Apple IIgs en affichage SHR 320 × 200 à seize couleurs. Il compare sept variantes de l’algorithme du peintre et plusieurs rendus par Z-buffer. Ses inspecteurs permettent d’étudier les ambiguïtés de visibilité, les plans des faces et les différences entre méthodes.

Il rend hommage à **Robert Dony**, auteur de *Calcul des parties cachées*, Masson, 1986.

![Interface principale](Screenshots/main_interface.png)
![Surfaces ombrées](Screenshots/crossSub_shaded.png)
![Recouvrement cyclique](Screenshots/cyclic_overlap.png)

## Documentation

- [Dictionnaire technique](dictionary.md) : formats, structures, conventions, caches et contrats des fonctions des sources actives.
- [Graphe des appels](arbre_appels.html) et [version repliable](arbre_appels_fold.html) : recherche, navigation entre appelants et appelés, fonctions non sélectionnées par l’interface incluses.
- [Bilan de révision](REVISION_DOCUMENTATION.md) : contrôles, limites et candidats à une rationalisation.

Les commentaires et documents sont en français, avec les identifiants et termes informatiques usuels conservés. Ils décrivent le code actuel ; les noms historiques et les chaînes de l’interface ne suffisent pas à déduire son comportement. Les fragments de `old/` et les diagnostics de `tools/` sont hors du périmètre de cette révision.

## Démarrage

1. Exécuter `OBJExplorer` dans l’environnement Apple IIgs préparé pour le programme.
2. Saisir le chemin OBJ. Après un échec, le programme essaie aussi le suffixe `.obj` s’il manque. Une ligne vide quitte le programme.
3. Répondre à la proposition de vérification des intersections. **Cette commande peut modifier la géométrie par découpage** ; elle ne fait pas qu’afficher un diagnostic.
4. Saisir les trois angles puis la distance, ou accepter les valeurs proposées avec Entrée. La vue initiale propose 30° horizontalement, 20° verticalement et 0° de rotation écran.
5. Utiliser les commandes ci-dessous ; Échap quitte la vue principale.

Le chargement centre les sommets à partir de leur boîte objet, propose une distance de trois fois la dimension maximale et ajuste l’échelle à une marge écran de 90 % dans la vue initiale. Une normalisation applique ensuite la formule existante vers une distance suggérée de 150. Elle modifie coordonnées et échelle ; elle ne garantit pas une invariance exacte de perspective pour toutes les profondeurs.

## Commandes principales

Les lettres acceptent majuscules et minuscules. Les commandes des inspecteurs peuvent avoir un autre sens.

| Commande | Action |
| --- | --- |
| Gauche / droite | Angle horizontal −10° / +10° ; pas de 1° avec Open-Apple. |
| Bas / haut | Angle vertical −10° / +10° ; pas de 1° avec Open-Apple. |
| `W` / `X` | Rotation écran +10° / −10° ; pas de 1° avec Open-Apple. |
| `A` / `Z` | Distance −10 % / +10 % ; −50 % / +50 % avec Open-Apple. |
| `+` / `-` | Projection ×1,1 / ×0,9, bornée entre 1 et 10000 ; reprojection et bornes mises à jour, sans nouveau tri. |
| `E` / `R` | Déplacement gauche / droite de dix pixels, ou un avec Open-Apple. |
| `T` / `Y` | Déplacement haut / bas de dix pixels, ou un avec Open-Apple. |
| `0` | Annulation du déplacement écran. |
| `K` | Nouvelle saisie de la vue sans rechargement. |
| `N` | Nouveau chargement et réinitialisation des options explicitement prévues dans `main`. |
| `1` à `7` | Choix de la méthode de classement, selon la table suivante. |
| `O` | Z-buffer scanline V3. |
| `U` | Z-buffer plein écran dans un framebuffer RAM, puis transfert SHR. |
| Open-Apple + `U` | Z-buffer plein écran directement vers SHR. |
| `P` | Filaire / surfaces ; le retour aux surfaces refait le traitement et le tri. |
| `B` | Back-face culling, puis retraitement. |
| `G` / `C` | Palette suivante / affichage des échantillons de couleurs. |
| `>` / `<` | Choix du remplissage / contour ; le contour peut reprendre le remplissage. |
| `8` | Nouveau remplissage aléatoire ; le choix du contour reste inchangé. |
| `9` | Couleurs par défaut, palette zéro et shading désactivé. |
| `!` | Shading par orientation des normales. |
| `J` | Jitter temporaire des sommets pendant le dessin. |
| `I` | Surlignage des paires inconclusives disponibles. |
| Espace | Modèle, paramètres, options et derniers temps mesurés. |
| `V` / `Q` | Inspecteur de face / inspecteur de paire. |
| `D` / `S` | Inspection des faces placées avant / après une cible. |
| `;` | Repair GEO V3 interactif : filaire et surlignage des deux faces avant chaque échange proposé. |
| `(` | Choix de la méthode de chevauchement projeté. |
| `/` | Benchmark des six méthodes comparées, export dans `overlap.all.txt`. |
| `F` | Export de `Faces3D.csv`, `Faces2D.txt` et `FacesOrder.txt`. |
| `*` | Capture brute SHR, premier nom disponible de `screen000.PIC` à `screen999.PIC`. |
| `H` | Aide paginée ; Échap sort de l’aide. |
| Échap | Sortie du programme. |

Dans l’inspecteur `V`, gauche/droite parcourent les identifiants et haut/bas la liste triée. `N` bascule la normale. Espace ouvre les détails et les commandes permettant notamment d’inverser le winding, de masquer et de restaurer des faces. Ces actions modifient le modèle en mémoire, sans réécrire le fichier OBJ.

Les temps sont mesurés en ticks de 1/60 seconde. Ils dépendent du modèle, de la vue, des options et de la machine ; cette documentation ne déduit pas un classement universel des performances.

## Méthodes de classement des faces

Les méthodes de classement produisent `faces.sorted_face_indices`, permutation d’identifiants à base zéro. Le dessin QuickDraw suit cette liste de l’arrière vers l’avant. Les métriques sont calculées avant le tri.

| Touche | Mode | Implémentation actuelle |
| --- | --- | --- |
| `1` | FAST | `painter_newell_sancha_fast` : `z_mean` décroissant, égalités par identifiant ; aucun test de chevauchement. |
| `2` | BUBBLE SORT | `painter_bubble_sort` : échanges de voisins après profondeur, boîtes, chevauchement et plans, avec garde des paires traitées. |
| `3` | NEWELL SANCHA | `painter_newell_sancha` : balayage des intervalles Z, liste de candidats et insertions avec table inverse des positions. |
| `4` | GEO | `painter_geoV2` : correction de voisins par plans puis chevauchement et ray casting ; cache et nombre de passes borné. |
| `5` | GEO V3 | `painter_geoV3` : tri stable par `z_max`, sélection et reprise après permutation ; filtrage assembleur et replis de ray casting. |
| `6` | CORRECT | `painter_correct`, variante 0 : partition selon `plane_d`, puis insertions locales sur le groupe avant. |
| `7` | CORRECT V2 | `painter_correct`, variante 1 : passe arrière et vérifications croisées supplémentaires sans culling initial. |

Aucun de ces sept chemins ne découpe automatiquement les faces pendant le tri. Le découpage appartient à la préparation expérimentale des intersections.

Un ordre global ne peut pas représenter tous les recouvrements cycliques et toutes les interpénétrations. Les gardes de permutations limitent les reprises ; elles ne prouvent pas la correction visuelle du résultat.

## Chevauchement et ray casting

Le menu `(` configure `overlap_method`, initialisé à **V1**, parmi sept méthodes :

- **Simple** : croisements propres et sommets strictement intérieurs ; certains contours coïncidents peuvent être manqués.
- **V1** : filtres, échantillons, seuils et clipping ; zéro peut aussi résulter d’une garde ou d’un échec de ressource.
- **V2 original, Max et Fixed64** : croisements, inclusions et sondes près des milieux d’arêtes ; limites numériques différentes.
- **Raster** : recherche d’un centre de pixel strictement commun aux spans. Une intersection continue peut n’en contenir aucun. **Raster x10** rééchantillonne après reprojection temporaire.

Le wrapper historique `projected_polygons_overlap_v2` possède un choix à la compilation distinct du menu runtime. Le benchmark `/` compare six méthodes, sans Simple ; un désaccord ne désigne pas à lui seul la bonne méthode.

Le clipping Sutherland-Hodgman exige un clip convexe pour l’intersection générale correcte par demi-plans. Essayer les deux sens ne résout pas tous les cas concaves. Malgré le suffixe `fixed`, ses calculs intermédiaires sont en double avant l’éventuel arrondi entier.

**`compute_intersection_centroid_ordered_qd_fixed` renvoie le centre de la bbox de région, pas son centroïde.** Son `out_area2` contient l’aire de cette boîte, et non le double de l’aire exacte de l’intersection. Les conventions des autres interfaces d’aire et de signe sont détaillées dans le dictionnaire.

## Z-buffers

Ils utilisent les coordonnées de vue déjà préparées et décident la visibilité par pixel, sans exiger un classement préalable correct des faces.

| Chemin | Profondeur | Contours |
| --- | --- | --- |
| Scanline V3 actif | Ligne réutilisée de 320 codes 16 bits ; préparation flottante de `1/z`. | Extrémités des spans et interpolation des codes. |
| Plein écran direct | 320 × 200 mots, plan affine de codes Z partagé avec le remplissage. | Passe distincte par Bresenham sur les côtés originaux, masque facultatif et garde de profondeur supplémentaire. |
| Plein écran offscreen | Même table Z et framebuffer RAM de 32000 octets copié par MVN. | Extrémités de spans et liaisons entre lignes ; algorithme différent du rendu direct. |

Les codes sont inversés : plus petit signifie plus proche, `$FFFF` signifie vide. Le plein écran réserve 192 Kio pour garantir deux banques de 64 Kio après alignement ; les 64000 profondeurs logiques occupent 128000 octets. La ligne Z de V3 occupe 640 octets, auxquels s’ajoutent les buffers par sommet, face et arête.

Sans culling, les chemins publics emploient un biais de rapprochement pour les faces de `plane_d > 0`. Ce biais modifie les décisions numériques entre surfaces proches. Les versions scanline flottantes et V2 restent présentes pour comparaison, hors du dispatch public actuel.

## Organisation et données

Le programme forme une unité de compilation par inclusions de fichiers C ; ses modules ne sont pas compilés indépendamment.

| Source | Rôle |
| --- | --- |
| `OBJExplorer.cc` | Commandes, cycles QuickDraw, chargement et choix des traitements. |
| `declare.c` | Formats fixes, trigonométrie, types, limites, états et prototypes. |
| `engine.c` | Chargement, préparation objet, transformation, métriques, classement des faces, dessin et inspection. |
| `ordering_cache.c` | Paires orientées, plans, côtés de sommets, contours et workspace. |
| `dony.c`, `dony_scan_asm.c` | GEO V3 et rejet rapide des candidats. |
| `geo_relation_asm.c`, `face_depth_asm.c` | Classification par plan ; métriques et Newell. |
| `overlap_raster.c` | Chevauchement, clipping, régions, projection subpixel et benchmark. |
| `zbuffer_scanline.c` | Rendu scanline V3, avec ou sans biais, et écriture d’un pixel SHR. |
| `zbuffer_common.c` | Interpolation entière des codes de profondeur et noyau assembleur commun aux spans direct/offscreen. |
| `zbuffer_fullscreen.c`, `.h` | Z-buffer et rendu direct avec contours sur plan. |
| `zbuffer_fullscreen_offscreen.c` | Rendu en RAM et transfert vers SHR. |
| `screen_save_restore.c` | Copie de 32768 octets SHR pour les vues temporaires et effacement paramétrable des lignes du haut. |
| `color.c`, `asm.h` | Sélecteur de couleurs ; clavier et soft switches SHR. |
| `DEPLOY.py` | Compilation Golden Gate, lien et déploiement dans les images disque. |

Les tableaux de coordonnées sont parallèles : objet `x/y/z`, observateur `xo/yo/zo` en 16.16, puis `x2d/y2d` en pixels. Les faces sont à base zéro ; les indices de sommets dans leur buffer restent à base un. `vertex_indices_ptr` contient des offsets dans ce buffer. Le pan s’applique au dessin et ne modifie pas les coordonnées de projection.

Les paquets assembleur et plusieurs buffers C sont partagés : le moteur suppose un fonctionnement synchrone, non réentrant. Les conventions de registres, de banques et de page directe sont documentées dans les noyaux.

## OBJ et limites

Le lecteur traite les lignes commençant par `v ` et `f `. Il échange y et z à l’import. Il retient les indices positifs des faces et saute les suffixes `/vt/vn`, sans interpréter textures, matériaux, normales ou indices relatifs négatifs.

Les maxima déclarés sont 6000 sommets, 6000 faces, seize indices par face et des lignes de 256 octets. Ils ne constituent pas une garantie conjointe : le buffer initial d’indices est réservé pour `MAX_FACES*5` éléments, et sa capacité n’est pas contrôlée à chaque écriture. La borne `readVertices_last_count` provient du compteur antérieur à l’import, pas toujours du nombre effectivement lu.

Il n’existe pas de clipping 3D général au plan proche. Les règles à profondeur nulle diffèrent entre transformation, métriques et zoom. Les formats fixes et différences 16 bits imposent aussi des bornes numériques.

Certains chemins conservent des réserves non contrôlées, des états statiques à cycles de vie distincts. La révision documentaire les décrit sans les corriger ; voir le bilan avant rationalisation.

## Compilation et déploiement

La chaîne locale est **ORCA/C via Golden Gate (`iix`)**, avec les bibliothèques Apple IIgs nécessaires à QuickDraw et GS/OS. Golden Gate doit être installé conformément à la licence détenue ; le dépôt ne fournit pas de remplacement de cette chaîne.

```text
iix compile OBJExplorer.cc
iix -DKeepType=S16 link OBJExplorer keep=OBJExplorer
```

Ces commandes reprennent la compilation et le lien de `DEPLOY.py`. Le script gère aussi des ressources `.rez` facultatives, copie le programme avec Cadius dans deux images disque, nettoie les intermédiaires puis tente un transfert ADB vers Android. Ses chemins et répertoires ProDOS sont propres à la machine de développement et doivent être configurés avant déploiement.

Les voies de comparaison C/assembleur sont sélectionnables par `GEOV3_DISABLE_ASM_MUL`, `GEOV3_DISABLE_ASM_RELATION`, `GEOV3_DISABLE_ASM_SCAN` et `FACE_DEPTH_DISABLE_ASM`. `GEO_ORDER_PROFILE` active des compteurs. Définir ces options avant les inclusions concernées ; la présente révision n’en change aucune.

## Avant de rationaliser

Comparer les appelants, formats, signes, tolérances, règles de bord et cycles de vie des ressources avant de fusionner des fonctions. En particulier, les plans objet et observateur, les différents résultats « avant », les champs d’aire et les deux rendus plein écran ne sont pas interchangeables.

Les graphes décrivent les appels syntaxiques présents, sans évaluer les branches conditionnelles comme ORCA/C. Ils distinguent appels C directs, appels assembleur et références indirectes connues du comparateur `qsort` et de la confirmation des échanges du repair GEO V3.

Les anciennes réparations complète et rapide sont archivées à la fin de `old/chutier.c`. La commande `.` reste retirée. La commande `;` lance désormais le repair GEO V3 : il part de la permutation courante, sans réinitialisation ni tri préalable. Les tests et échanges proviennent du même moteur que le classement GEO V3 accessible par `5`.

Avant chaque échange, les 200 lignes de pixels sont effacées, puis le modèle est redessiné en fil de fer, avec la face P en vert clair et Q en orange. Deux lignes courtes proposent d’afficher P devant Q (« Draw face P in front of face Q »), puis les commandes. Cela correspond à dessiner Q avant P dans la permutation arrière-vers-avant. Les paires sans chevauchement selon la méthode configurée sont ignorées avant toute proposition, y compris en mode automatique. `N` ignore la proposition, Échap arrête en conservant les échanges déjà acceptés, Retour active l’application automatique des suivantes ; une autre touche applique l’échange courant. Le bilan compte propositions, échanges appliqués et refus. Les limites anti-cycle de GEO V3 restent actives : une proposition n’est pas une preuve exhaustive de correction géométrique. Les commandes S/D et Q restent disponibles pour l’examen manuel.

Les vues S/D, Q, V, le repair et l’affichage des identifiants partagent la sauvegarde de visibilité, du mode filaire et du culling. Chaque outil conserve son cycle QuickDraw. Les actions volontaires de V sur les faces sont intégrées à la sauvegarde pour rester effectives à la fermeture. Un échec de sauvegarde annule la vue proprement.
