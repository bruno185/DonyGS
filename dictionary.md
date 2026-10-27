# Dictionnaire technique de l’explorateur OBJ

Ce document décrit les sources actives de la racine. Il accompagne le [guide d’utilisation](README.md), le [graphe des appels](arbre_appels.html), sa [version repliable](arbre_appels_fold.html) et le [bilan de révision](REVISION_DOCUMENTATION.md).

Les contrats ci-dessous sont reconstruits depuis les en-têtes français des définitions. `old/`, `tools/` et `DEPLOY.py` sont exclus de la révision. Les branches de compilation sont inventoriées sans être évaluées : une définition présente n’est pas nécessairement appelée dans la configuration courante.

## Repères et formats

| Terme ou donnée | Signification dans le projet |
| --- | --- |
| `FACE_ORDER_*` | −1 avant, 0 indéterminé, +1 après dans l’ordre de dessin. |
| `RAY_*` | −1 premier plus proche, 0 indéterminé, +1 premier plus loin ; convention distincte de l’ordre de dessin. |
| `PAIR_FRONT_*` | 0 indéterminé, 1 première face devant, 2 seconde face devant. |
| Repère objet | Coordonnées `x/y/z` importées et éventuellement centrées ou normalisées. Les axes OBJ y et z sont échangés à la lecture. |
| Repère observateur | Coordonnées `xo/yo/zo` après rotation et translation de distance ; origine au point d’observation. |
| Projection | `x2d/y2d`, pixels entiers autour de (160,100), avant application du pan. Le chemin subpixel conserve des coordonnées double. |
| `Fixed32` | `long` signé 32 bits en 16.16 sur ORCA/C : unité 65536. Ne pas transposer les tailles natives d’un compilateur hôte. |
| `Fixed64` | Entier signé 64 bits pour les produits et accumulations ; son échelle dépend du calcul. Un produit brut 16.16 × 16.16 est en 32.32. |
| `FIXED_ROUND_TO_INT` | Biais +32768 pour les positifs, −32768 pour les négatifs, puis décalage ; ce n’est pas un arrondi symétrique garanti. |
| Sommet OBJ | Identifiant à base un dans `vertex_indices_buffer` ; retrancher un avant l’accès aux coordonnées. |
| Face | Identifiant à base zéro dans les tableaux parallèles. Une position de tri n’est pas un identifiant de face. |
| `vertex_indices_ptr` | Offset en nombre d’éléments dans le buffer d’indices ; malgré le suffixe, ce n’est pas un pointeur far. |
| `sorted_face_indices` | Permutation des identifiants de faces ; le rendu dessine de l’arrière vers l’avant. |
| Bbox | Boîte englobante alignée sur les axes ; commune aux projections ou séparée en 3D objet. Une bbox commune ne prouve pas un chevauchement de surfaces. |
| `z_min/z_max/z_mean` | Métriques observateur par face ; différentes clés initiales sont utilisées selon la méthode de classement. |
| Plan observateur | `plane_a/b/c/d`, calculés par Newell et centroïde moyen ; sert aux tests de côtés et au culling. |
| Winding | Sens du parcours du contour. L’inverser oppose les quatre coefficients du plan et peut changer le culling. |
| Culling | Rejet optionnel des faces avec `plane_d <= 0`, ajouté aux règles de visibilité propres au parcours. |
| SH | Sutherland-Hodgman, clipping par demi-plans ; clip convexe requis pour le cas général. |
| Ray casting | Intersection d’un rayon écran avec deux plans. Le test n’assure pas à lui seul l’appartenance aux polygones. |
| `1/z` | Profondeur inverse, utilisée pour l’interpolation écran et la quantification des Z-buffers. |
| Code Z | Mot 16 bits inversé : plus petit signifie plus proche, `$FFFF` signifie vide/lointain. Les égalités ne remplacent pas les surfaces dans les noyaux de spans. |
| Span | Intervalle horizontal de pixels à remplir ; les extrémités et l’intérieur peuvent avoir des couleurs distinctes. |
| Scanline | Ligne de balayage ; V3 réutilise une seule ligne Z, avec des réserves auxiliaires par face, arête et sommet. |
| SHGR | Affichage 320 × 200, deux pixels de quatre bits par octet, 160 octets par ligne à partir de `$E1:2000`. |
| Nibble | Demi-octet : x pair dans le nibble haut, x impair dans le nibble bas. |
| Framebuffer offscreen | 32000 octets de pixels en RAM, sans SCB ni palettes, transférés ensuite par MVN. |
| Sauvegarde SHGR | Copie de 32768 octets à partir de `$E1:2000`, distincte du seul framebuffer de pixels. |
| Pointeur far | Adresse ORCA/C stockée sur quatre octets ; l’adressage long du 65C816 utilise la banque. |
| D / DBR / P | Page directe, banque de données et registre d’état du 65C816 ; leur préservation dépend de chaque routine. |
| Paquet assembleur | Variables partagées préparées par C puis copiées dans un cadre de page directe sur la pile ; non réentrant. |
| Stamp / génération | Marqueur de validité ; permet d’invalider une vue sans libérer les allocations ni effacer tous les buffers à chaque appel. |
| Watchdog | Garde d’exécution ; un résultat abandonné ne prouve pas l’absence de chevauchement. |
| Tick | Unité de `GetTick`, 1/60 seconde ; ne pas l’assimiler au nom MS de certaines constantes historiques. |

## Structures et états

| Structure ou groupe | Contenu et cycle de vie |
| --- | --- |
| `VertexArrays3D` | Tableaux parallèles objet, observateur et projetés ; le compteur initial de création représente une capacité, remplacée après lecture. |
| `FaceArrays3D` | Longueurs et offsets des listes, métriques, plans, visibilité et permutation. Les tailles sauvegardées permettent de masquer puis restaurer une face. |
| `Model3D` | Possède les tableaux du modèle et ses métadonnées de centrage/ajustement ; les caches statiques et Z-buffers ont des propriétaires distincts. |
| `ObserverParams` | Trois angles entiers et distance 16.16. Le pan et l’échelle globale sont conservés séparément. |
| `DynamicPolygon` | Format mémoire destiné aux polygones QuickDraw ; handle global réutilisé et verrouillé explicitement. |
| `PairCache` | Cache historique à sondage limité à huit slots ; signe +1 avant, opposé à la relation géométrique. |
| `OrderPairEntry` | Clé complète orientée : paire, génération, échelle, contexte et résultat ; collisions par remplacement. |
| `OrderFixedPlane` | Coefficients, epsilon, époque, côté observateur, admissibilité assembleur et cache compact de côtés des sommets. |
| `OrderContour` | Contour subpixel normalisé et vecteurs d’arêtes ; capacité bornée pour la mémorisation, réserve conservée entre vues. |
| Cache dense de relations | Jusqu’à 256 faces ; deux bits par relation orientée, avec codes inconnu/avant/indéterminé/après. |
| Cache fractionnaire | Blocs de 256 rapports `xo/zo` et `yo/zo` ; indépendants du zoom tant que les coordonnées observateur restent valides. |
| `DonyRayPairs` | Paires non orientées de permutations par rayon pendant une passe ; chaînes de hash avec clés exactes. |
| `FaceDepthScanResult` | Résultat du noyau de métriques : profondeurs, bornes, sommes du centroïde et normale ; division et d restent en C. |
| `RCHZoomPatch` | Sauvegarde des sommets distincts de deux faces, de leurs bbox et de l’échelle pendant un grossissement temporaire. |
| `ZbEdgePacket` | Paquet de 50 octets : plan, profondeurs, pointeurs far et état Bresenham ; copié dans le cadre direct du noyau de contours. |
| `ZbPlane` | Origine et gradients du code Z avec douze fractions, plus accumulateur de ligne scan_raw pour le remplissage plein écran direct. |
| États graphiques | Couleurs, palette, shading, pan, jitter et culling sont partagés. Les fonctions de dessin ne préservent pas toutes le motif de crayon. |

## Conventions à ne pas confondre

| Interface | Résultat et particularités |
| --- | --- |
| `geometric_face_relation` | −1 : f1 avant f2 ; +1 : après ; 0 : indéterminé. Tolérance proportionnelle à la distance. |
| `geo_face_order` | −1 : f1 avant f2 ; +1 : f1 après f2 ; 0 : indéterminé. Utilise le noyau optimisé et sa tolérance adaptative. |
| `PairCache` | +1 : f1 avant f2 ; −1 : sens inverse ; 0 : absent ou indéterminé. |
| `ray_cast_at`, `ray_cast_hierarchical` | +1 : f1 plus loin, donc avant ; −1 : f1 plus proche ; 0 : échec/égalité/indéterminé. |
| `run_raycast_test` | 1 : f1 plus proche ; 2 : f2 plus proche ; 0 : indéterminé. |
| `compute_face_plane`, mode `FACE_PLANE_OBJECT_UNITS` | Produit vectoriel des trois premiers sommets objet convertis par `FIXED_TO_FLOAT` ; non normalisé. |
| `compute_face_plane`, mode `FACE_PLANE_RAW_FIXED` | Même triangle initial, mais doubles contenant les entiers 16.16 bruts ; coefficients à une autre échelle. |
| `calculateFaceDepths` | Normale de Newell sur le contour observateur ; échelle et repère distincts des deux fonctions précédentes. |
| Clipping entier | Aire et centroïde calculés après arrondi du contour ; un succès peut accompagner une aire nulle. |
| Clipping fractionnaire | Double de l’aire positive en pixels carrés ; coordonnées de centroïde subpixel. |
| Chemin QuickDraw | Centre de bbox, pas centroïde ; `out_area2` contient l’aire de bbox, pas le double d’une aire de surface. |
| Repli `tryboth` entier | Peut rendre 1 avec aire nulle et point non validé ; vérifier aussi le marqueur ou l’aire. |
| Chevauchement raster | Présence d’un centre de pixel strictement commun, et non existence de toute aire continue positive. |

## Parcours de traitement

1. `main` réserve les ressources, crée le modèle, lit les sommets puis les faces et propose les paramètres de vue.
2. `processModelFast` invalide les caches, construit la matrice de rotation et projette les sommets de profondeur positive.
3. `calculateFaceDepths` calcule bornes, profondeurs, normale, d et visibilité. Son noyau assembleur est conditionnel ; le chemin C reste présent.
4. La méthode de classement choisie produit la permutation ; en filaire, cette étape est sautée après les métriques.
5. `drawPolygons` suit cette permutation. Les rendus Z-buffer sont déclenchés séparément et réutilisent les données de vue.
6. Les inspecteurs exposent des calculs ou modifient explicitement winding, visibilité et permutation ; les exports décrivent l’état courant sans nécessairement le recalculer.

Le graphe recense les appels C directs et les appels JSL/JSR vers les définitions du périmètre. Le comparateur passé à `qsort` est identifié séparément comme appel indirect. Les dépendances par variables globales ne sont pas des arêtes d’appel.

## Modules

### OBJExplorer.cc

Piloter le chargement des modèles, la préparation des vues et les commandes de l'explorateur Apple IIgs. Le programme permet de comparer les méthodes de classement, les Z-buffers et les outils d'inspection.

La fonction main distingue le chargement, le retraitement géométrique, le redessin, l'attente clavier et la libération par des labels. Les vues textuelles temporaires réutilisent la sauvegarde SHGR pour éviter un nouveau rendu.

Inclut engine.c, qui rassemble les autres sources dans une même unité ORCA/C. Le clavier peut associer Open-Apple à un caractère. Les chaînes visibles de l'interface sont conservées. Hommage à Robert Dony, auteur de Calcul des parties cachées (Masson, 1986).

### declare.c

Définir les types, formats numériques, limites et états partagés de l'explorateur. Les coordonnées objet et observateur utilisent des valeurs signées 16.16 ; les produits et sommes étendus emploient Fixed64.

Regroupe les inclusions Toolbox, les constantes et tables trigonométriques, les tableaux parallèles de sommets et de faces, les options globales, les utilitaires de palette et les prototypes.

Inclus en tête de engine.c. Les tailles natives attendues sont celles d'ORCA/C : int sur 16 bits et long sur 32 bits. Les indices OBJ stockés sont à base un ; les faces et les permutations sont à base zéro. Les prototypes historiques ne prouvent pas qu'un chemin est appelé.

### engine.c

Charger le modèle OBJ, transformer et projeter ses sommets, calculer les métriques des faces, construire un ordre de dessin et fournir les outils de rendu et de diagnostic.

Les segments regroupent chargement et préparation objet, transformation et classement des faces, tests d'ordre et ray casting, dessin QuickDraw, couleurs, inspection et exports. Les implémentations spécialisées sont intégrées par inclusion de sources C.

Inclus par OBJExplorer.cc après les pragmas ORCA/C. Les caches et paquets assembleur sont partagés et non réentrants. Les interfaces de plans, de rayons et de caches n'emploient pas toutes les mêmes signes, échelles et tolérances ; leurs contrats font foi. Aucun algorithme n'est fusionné dans cette révision.

### ordering_cache.c



Les entrées sont associées à un modèle et à une génération de données. Les clés complètes sont vérifiées avant réutilisation ; une collision peut remplacer une entrée, mais ne doit pas produire un faux résultat. Les allocations sont conservées entre invalidations.

Inclus par engine.c après declare.c. Les changements de transformation, d'époque, de tolérance ou d'échelle invalident les entrées concernées. Les buffers et paquets assembleur sont partagés : leur utilisation est synchrone et non réentrante.

### dony.c

Construire un ordre arrière-vers-avant selon une stratégie inspirée de Robert Dony : sélectionner une face, comparer les candidates et reprendre la sélection après permutation. Le repair réutilise ce parcours depuis la permutation courante.

Un tri stable par profondeur maximale prépare le parcours. Les intervalles disjoints sont rejetés, puis les plans sont testés dans les deux sens. Les cas ambigus utilisent le chevauchement projeté et le ray casting hiérarchique. Des stamps et un cache de paires limitent les cycles. Le callback facultatif du repair confirme chaque échange, sans dupliquer les décisions de classement.

Inclus par engine.c ; dony_scan_asm.c fournit le filtrage par défaut. GEOV3_DISABLE_ASM_SCAN sélectionne le filtrage C. Les métriques doivent être préparées avant le tri et rester stables pendant son exécution. Cette variante ne découpe pas les faces.

### dony_scan_asm.c

Sauter en un appel les candidates dont les intervalles de profondeur ou les boîtes projetées excluent le recouvrement. Rendre au C la première position nécessitant les tests géométriques.

Deux wrappers préparent les pointeurs stables, les bornes de la face sélectionnée et la position de départ. Le noyau 65C816 copie le paquet dans un cadre de page directe sur la pile, puis parcourt la permutation sans la modifier.

Inclus par dony.c lorsque le filtrage assembleur est activé. Pointeurs far ORCA/C sur quatre octets, bornes sur 16 bits et profondeur 16.16. Les soustractions d'intervalles conservent les règles du C natif. Paquet global non réentrant ; disposition du cadre et registres décrits au niveau du noyau.

### geo_relation_asm.c

Déterminer si les sommets d'une face se trouvent tous du même côté qu'un observateur, tous du côté opposé, ou de côtés incompatibles par rapport à un plan de référence.

Les wrappers construisent ou réutilisent un paquet de plan préparé. Le noyau évalue les produits signés 32x32 vers 64 bits, décale chaque terme séparément et compare la somme à la bande de tolérance. Le côté exact des sommets est mémorisé par plan.

Inclus par engine.c après GeoV3_Mul32x32. Le noyau requiert des coefficients a/b/c représentables sur 32 bits et les bornes de d validées par ordering_fixed_plane. Le paquet et le multiplicateur sont partagés ; les accès globaux longs évitent une dépendance à DBR.

### face_depth_asm.c

Parcourir les sommets des faces pour calculer profondeurs, bornes projetées, sommes de coordonnées et normale de Newell, en conservant le comportement numérique du parcours C.

Les pointeurs invariants sont préparés une fois ; chaque appel de face fournit sa liste d'indices. Le noyau ferme le cycle des sommets valides et exporte une structure de résultats. La division du centroïde, le terme d du plan et le culling final restent dans calculateFaceDepths.

Inclus conditionnellement par engine.c et dépend de GeoV3_Mul32x32. La somme servant à z_mean reboucle sur 32 bits ; les sommes de centroïde et de Newell sont sur 64 bits. Le décalage des normales intervient après accumulation complète. Les paquets sont non réentrants.

### overlap_raster.c

Fournir les méthodes de chevauchement des faces projetées et les points de comparaison employés par les outils d'ordre. Distinguer géométrie continue, heuristiques avec seuils et échantillonnage raster.

Regroupe le cache de projection fractionnaire, les prédicats de points et segments, le clipping SH, les régions QuickDraw, les variantes Simple/V1/V2/Max/Fixed64/Raster et leur benchmark. Le choix runtime est indépendant du wrapper V2 à sélection compilée.

Inclus par engine.c ; utilise les types communs, les caches d'ordre et les routines de ray casting déclarées auparavant. Le clipping par demi-plans exige un clip convexe pour le cas général. Le chemin nommé centroïde QuickDraw rend un centre de bbox. Buffers et diagnostics partagés, non réentrants.

### color.c

Permettre de choisir la couleur de remplissage ou de contour dans les palettes logicielles de l'explorateur, avec les choix spéciaux aléatoire et contour identique au remplissage.

Une grille 8x2 présente les seize indices de couleur ; une ligne séparée contient les choix spéciaux. Les fonctions de dessin calculent les rectangles, libellés et cadres ; colorChooser traite les flèches et la confirmation.

Inclus par engine.c. Requiert un port QuickDraw actif, les palettes préparées et les fonctions clavier du projet. Le sélecteur modifie l'écran et la palette matérielle ; l'appelant redessine ensuite la scène. Les libellés visibles ne sont pas traduits ici.

### asm.h

Accéder directement au clavier Apple IIgs, à l'état Open-Apple et aux soft switches d'affichage SHGR. Fournir aussi une signature de registres utilisable par un breakpoint d'émulateur.

Les lectures clavier attendent activement le strobe, l'acquittent et rendent soit le masque, soit le caractère, soit un mot associant caractère et modificateur. Les routines d'affichage écrivent le registre $00C029.

Définitions assembleur ORCA/C destinées à l'unité de compilation du projet. Les contrats de taille et de préservation diffèrent entre routines ; ne pas supposer que P ou tous les registres sont restaurés. Les accès matériels longs ne dépendent pas de DBR. Aucun timeout clavier.

### screen_save_restore.c

Conserver une image de 32768 octets à partir de $E1:2000 pour revenir rapidement aux vues graphiques après une inspection textuelle, sans recalculer les polygones. Effacer aussi un nombre choisi de lignes depuis le haut de la zone de pixels.

L'initialisation réserve un handle verrouillé et en décompose l'adresse. Les fonctions saveScreen et restoreScreen patchent les banques d'une instruction MVN, puis transfèrent la zone en un bloc. graph_changed suit l'invalidation de la sauvegarde. screenClearTopLines partage l'effacement du bandeau avant les rendus direct et scanline.

Inclus par engine.c et dépend des outils mémoire Apple IIgs. Appeler l'initialisation une seule fois. Le transfert suppose une implantation compatible avec une banque et un offset 16 bits. Code auto-modifiant inscriptible et non réentrant ; P et DBR sont préservés, contrairement aux registres de transfert.

### zbuffer_common.c

Fournir les réserves et préparations communes aux trois rendus Z-buffer, ainsi que leur noyau de spans 65C816.

Regroupe les buffers typés privés de chaque moteur, la profondeur inverse, la quantification, les débuts de faces, les listes actives, les arêtes scanline/offscreen, leur tri et l'interpolation des spans. Le paquet zspan_p_* et le noyau assembleur servent aux trois rendus.

Inclus par engine.c avant les moteurs Z-buffer. Sur ORCA/C, UWORD16 et unsigned int sont non signés sur 16 bits ; long est signé sur 32 bits. Les produits intermédiaires doivent rester représentables. Les moteurs conservent leurs réserves, échelles, biais et règles de contour propres. Le paquet de spans reste partagé et non réentrant.

### zbuffer_scanline.c

Résoudre la visibilité par pixel avec une seule ligne de profondeur réutilisée, à partir des coordonnées projetées et des métriques déjà préparées par le moteur.

Le dispatch public sélectionne le moteur V3, qui prépare les codes 16 bits de profondeur inverse, les arêtes et les faces actives, puis dessine chaque span par un noyau assembleur. Le fichier fournit aussi les couleurs effectives.

Inclus par engine.c ; écran SHGR 320x200, deux pixels par octet. Sans culling, la variante active emploie un biais. Les buffers statiques et paquets de spans sont non réentrants ; la mémoire Z seule ne résume pas leur consommation totale.

### zbuffer_fullscreen.c

Gérer la table de profondeur plein écran et dessiner directement en SHGR les surfaces et leurs contours, avec un plan affine de codes Z commun aux deux passes.

L'allocation aligne deux banques utilisables et construit les pointeurs de lignes. Le rendu prépare sommets, plans et arêtes, remplit les spans avec arrondi intérieur, puis trace les côtés originaux par Bresenham. Une garde compare aussi la profondeur géométrique des arêtes.

Inclus par engine.c et utilise zbuffer_fullscreen.h. Son noyau de contours est autonome. Init est idempotent ; Shutdown libère les buffers propres à ce rendu. Les codes plus petits sont plus proches et $FFFF représente le vide. Les règles de contour diffèrent du rendu offscreen. Paquets assembleur partagés et non réentrants.

### zbuffer_fullscreen_offscreen.c

Rendre le modèle dans un framebuffer RAM de 32000 octets, puis transférer les pixels vers SHGR en une copie MVN, en utilisant le Z-buffer géré par zbuffer_fullscreen.c.

Le cycle externe est Init, Clear, rendu, FlushToScreen, puis Shutdown. Le noyau prépare des arêtes et spans dont les codes Z sont interpolés. Les contours sont obtenus par extrémités et liaisons entre lignes consécutives ; ils ne reprennent pas le plan de contour du rendu direct.

Inclus par engine.c après le module plein écran. Le handle verrouillé ne doit pas traverser de banque. Les MVN sont patchés à l'exécution, donc non réentrants et inscriptibles. Init doit être appelé une seule fois ; Shutdown ne libère pas les réserves statiques de préparation du rendu.

### zbuffer_fullscreen.h

Déclarer les dimensions, codes de profondeur, types et opérations communes aux rendus plein écran direct et offscreen.

Expose la table de pointeurs de lignes, le cycle de vie du Z-buffer, la quantification de 1/z et le masque facultatif des côtés. Les noyaux assembleur restent internes aux moteurs de rendu.

À inclure après les types nécessaires du projet. Les coordonnées des primitives non clippées doivent être valides. Les pointeurs de lignes ne sont utilisables qu'après initialisation réussie ; le masque d'arêtes est emprunté, jamais possédé par le module.

## Index alphabétique des fonctions

- [add_vertex](#fn-add_vertex) — `engine.c` : Ajouter un sommet au repère objet du modèle.
- [applyPalette](#fn-applyPalette) — `engine.c` : Installer une palette logicielle dans la table matérielle active numéro zéro.
- [benchmark_projected_overlap_all](#fn-benchmark_projected_overlap_all) — `overlap_raster.c` : Comparer six méthodes sur toutes les paires et plusieurs orientations.
- [calculateFaceDepths](#fn-calculateFaceDepths) — `engine.c` : Recalculer les métriques et plans observateur des faces de la vue courante.
- [check_intersect](#fn-check_intersect) — `engine.c` : Examiner les intersections objet et tenter des découpages avant la visualisation.
- [cleanup_list](#fn-cleanup_list) — `engine.c` : Compacter une liste cyclique d'indices en retirant les répétitions adjacentes.
- [cmp_faces_by_zmean](#fn-cmp_faces_by_zmean) — `engine.c` : Comparer deux identifiants de faces pour qsort.
- [colorChooser](#fn-colorChooser) — `color.c` : Exécuter la sélection interactive de couleur jusqu'à confirmation.
- [compare_faces_diagnostic](#fn-compare_faces_diagnostic) — `engine.c` : Comparer plusieurs méthodes d'ordre et conserver leurs résultats séparés.
- [compute2DFromObserver](#fn-compute2DFromObserver) — `engine.c` : Reprojeter une vue observateur existante après changement d'échelle.
- [compute_bbox_intersection](#fn-compute_bbox_intersection) — `overlap_raster.c` : Calculer le rectangle commun aux boîtes projetées de deux faces.
- [compute_bbox_intersection_center](#fn-compute_bbox_intersection_center) — `overlap_raster.c` : Retourner le centre entier du rectangle commun des boîtes de deux faces.
- [compute_face_bboxes3D](#fn-compute_face_bboxes3D) — `engine.c` : Recalculer les six bornes objet de chaque face.
- [compute_face_plane](#fn-compute_face_plane) — `engine.c` : Calculer le plan du triangle initial d'une face dans le repère objet.
- [compute_intersection_centroid_fractional_tryboth](#fn-compute_intersection_centroid_fractional_tryboth) — `overlap_raster.c` : Essayer le clipping subpixel dans les deux sens sans arrondir le centroïde.
- [compute_intersection_centroid_ordered_fixed](#fn-compute_intersection_centroid_ordered_fixed) — `overlap_raster.c` : Appeler le clipping ordonné entier avec un espace de travail réutilisé.
- [compute_intersection_centroid_ordered_fixed_tryboth](#fn-compute_intersection_centroid_ordered_fixed_tryboth) — `overlap_raster.c` : Essayer le clipping entier puis chercher un point de repli commun.
- [compute_intersection_centroid_ordered_fixed_work](#fn-compute_intersection_centroid_ordered_fixed_work) — `overlap_raster.c` : Découper un sujet par les demi-plans du clip et calculer un point central.
- [compute_intersection_centroid_ordered_qd_fixed](#fn-compute_intersection_centroid_ordered_qd_fixed) — `overlap_raster.c` : Retourner le centre de bbox de l'intersection QuickDraw de deux faces.
- [compute_intersection_region_bbox](#fn-compute_intersection_region_bbox) — `overlap_raster.c` : Construire l'intersection de deux régions QuickDraw et en lire la bbox.
- [computeOrientationShading](#fn-computeOrientationShading) — `engine.c` : Calculer une couleur selon l'orientation de chaque normale observateur.
- [cos_deg_int](#fn-cos_deg_int) — `declare.c` : Lire le cosinus d'un angle entier dans la table 16.16.
- [count_edge_crossings](#fn-count_edge_crossings) — `engine.c` : Compter les arêtes de ei qui traversent fj ou y possèdent une extrémité coplanaire intérieure.
- [createModel3D](#fn-createModel3D) — `engine.c` : Allouer le modèle et ses tableaux parallèles à capacité fixe.
- [destroyModel3D](#fn-destroyModel3D) — `engine.c` : Libérer les buffers explicitement possédés par un modèle puis sa structure.
- [display_model_face_ids](#fn-display_model_face_ids) — `engine.c` : Afficher une vue graphique annotée avec les identifiants des faces visibles.
- [DoColor](#fn-DoColor) — `engine.c` : Afficher les seize couleurs disponibles avec leurs indices.
- [dony_ray_pair_record](#fn-dony_ray_pair_record) — `dony.c` : Enregistrer une paire non orientée échangée pendant la passe courante.
- [dony_ray_pair_seen](#fn-dony_ray_pair_seen) — `dony.c` : Tester si une paire non orientée a déjà provoqué un échange dans cette passe.
- [dony_ray_pairs_reserve](#fn-dony_ray_pairs_reserve) — `dony.c` : Réserver les listes de paires utilisées pour empêcher les cycles de ray casting.
- [DoText](#fn-DoText) — `engine.c` : Revenir à l'affichage texte et envoyer le caractère d'effacement écran.
- [drawBlackSquareMarker](#fn-drawBlackSquareMarker) — `color.c` : Rendre visible une case noire avec un cadre blanc permanent.
- [drawFace](#fn-drawFace) — `engine.c` : Dessiner une face isolée pour les outils d'inspection.
- [drawFaceIndex](#fn-drawFaceIndex) — `engine.c` : Écrire l'identifiant d'une face au centre de sa boîte projetée.
- [drawInstructions](#fn-drawInstructions) — `color.c` : Afficher les deux lignes de commandes du sélecteur.
- [drawPaletteNumber](#fn-drawPaletteNumber) — `color.c` : Afficher le type de couleur modifié et le numéro de palette.
- [drawPolygons](#fn-drawPolygons) — `engine.c` : Dessiner les faces affichables dans la permutation calculée par la méthode de classement.
- [drawPolygons_jitter](#fn-drawPolygons_jitter) — `engine.c` : Dessiner une vue avec un déplacement aléatoire temporaire de chaque sommet.
- [drawSelectionBorder](#fn-drawSelectionBorder) — `color.c` : Dessiner ou effacer le cadre de sélection d'une case.
- [drawSquare](#fn-drawSquare) — `color.c` : Dessiner le fond et le contour noir d'une case.
- [drawSquareLabel](#fn-drawSquareLabel) — `color.c` : Écrire le libellé d'une case sous son rectangle.
- [dump_compare_results_to_file](#fn-dump_compare_results_to_file) — `engine.c` : Ajouter le diagnostic textuel d'une paire dans son fichier dédié.
- [dumpFace2DCoordinates](#fn-dumpFace2DCoordinates) — `engine.c` : Exporter les coordonnées écran de chaque sommet de chaque face.
- [dumpFaceEquationsCSV](#fn-dumpFaceEquationsCSV) — `engine.c` : Exporter les plans, profondeurs et coordonnées observateur des faces.
- [dumpSortedFaceIndices](#fn-dumpSortedFaceIndices) — `engine.c` : Exporter la permutation courante dans l'ordre de dessin de la méthode de classement.
- [edge_crosses_interior](#fn-edge_crosses_interior) — `engine.c` : Détecter une arête de ei qui traverse strictement l'intérieur de fj.
- [encodePaletteColor](#fn-encodePaletteColor) — `declare.c` : Encoder une couleur RGB dans le mot $0RGB du matériel SHGR.
- [evaluate_pair_tests](#fn-evaluate_pair_tests) — `engine.c` : Exposer séparément les sept tests d'ordre d'une paire pour le diagnostic.
- [FaceDepth_CentroidCoordinate](#fn-FaceDepth_CentroidCoordinate) — `face_depth_asm.c` : Diviser une somme de coordonnées par le nombre de sommets valides.
- [FaceDepth_ScanFace](#fn-FaceDepth_ScanFace) — `face_depth_asm.c` : Calculer les métriques d'une liste d'indices de face.
- [FaceDepth_ScanKernel](#fn-FaceDepth_ScanKernel) — `face_depth_asm.c` : Parcourir une face pour calculer profondeurs, bornes, centroïde et normale.
- [FaceDepth_ScanSetup](#fn-FaceDepth_ScanSetup) — `face_depth_asm.c` : Préparer les pointeurs de coordonnées pour un parcours complet des faces.
- [faces_share_vertex_or_edge](#fn-faces_share_vertex_or_edge) — `engine.c` : Détecter un identifiant de sommet commun à deux faces.
- [faces_vertices_equal](#fn-faces_vertices_equal) — `overlap_raster.c` : Comparer deux contours projetés modulo rotation cyclique et inversion.
- [fixed_mul32_fast](#fn-fixed_mul32_fast) — `engine.c` : Multiplier deux valeurs 16.16 et conserver les bits 16..47 du produit.
- [fixed_product32_fast](#fn-fixed_product32_fast) — `engine.c` : Calculer le produit signé brut 32x32 vers 64 bits.
- [fixed_product64x32_fast](#fn-fixed_product64x32_fast) — `engine.c` : Calculer un produit brut coefficient 64 bits par coordonnée 32 bits.
- [FIXED_ROUND_TO_INT](#fn-FIXED_ROUND_TO_INT) — `declare.c` : Convertir un Fixed32 en entier en appliquant le biais de demi-unité du moteur.
- [fractional_face_gather](#fn-fractional_face_gather) — `overlap_raster.c` : Préparer le contour subpixel antihoraire d'une face et ses vecteurs d'arêtes.
- [fractional_projection_cache_invalidate](#fn-fractional_projection_cache_invalidate) — `overlap_raster.c` : Invalider les rapports de projection et les caches d'ordre associés.
- [fractional_projection_cache_release](#fn-fractional_projection_cache_release) — `overlap_raster.c` : Libérer les caches d'ordre et de projection appartenant à model.
- [fractional_projection_cached](#fn-fractional_projection_cached) — `overlap_raster.c` : Projeter un sommet en double en mémorisant xo/zo et yo/zo indépendamment du zoom.
- [frameInconclusivePairs](#fn-frameInconclusivePairs) — `engine.c` : Surligner les faces des paires inconclusives conservées par les méthodes de classement.
- [generate_random_colors](#fn-generate_random_colors) — `engine.c` : Générer deux tableaux de couleurs non noires pour les faces.
- [geo_face_order](#fn-geo_face_order) — `engine.c` : Fournir une préférence d'ordre en utilisant le noyau géométrique optimisé.
- [geometric_face_relation](#fn-geometric_face_relation) — `engine.c` : Classer f1 par rapport à f2 en utilisant le plan de f1 dans la vue observateur.
- [GeoRel_Classify32](#fn-GeoRel_Classify32) — `geo_relation_asm.c` : Construire le paquet complet et appeler le noyau de classement.
- [GeoRel_Classify32Kernel](#fn-GeoRel_Classify32Kernel) — `geo_relation_asm.c` : Classer une face candidate par rapport au plan et au côté de l'observateur.
- [GeoRel_ClassifyPrepared](#fn-GeoRel_ClassifyPrepared) — `geo_relation_asm.c` : Classer une face en réutilisant le paquet d'un plan préparé.
- [GeoV3_FindCandidate](#fn-GeoV3_FindCandidate) — `dony_scan_asm.c` : Rechercher une position candidate en réutilisant les bornes de la face P.
- [GeoV3_Mul32x32](#fn-GeoV3_Mul32x32) — `engine.c` : Multiplier exactement deux entiers signés 32 bits et publier le produit 64 bits brut.
- [GeoV3_Mul32x32SetInputs](#fn-GeoV3_Mul32x32SetInputs) — `engine.c` : Préparer les deux opérandes natifs et exécuter le multiplicateur assembleur.
- [GeoV3_MulFixed64Fixed32Shift](#fn-GeoV3_MulFixed64Fixed32Shift) — `engine.c` : Évaluer une contribution de plan : (coefficient * coordinate) >> 16.
- [geov3_run](#fn-geov3_run) — `dony.c` : Partager le parcours GEO V3 entre le classement et le repair interactif.
- [GeoV3_ScanCandidates](#fn-GeoV3_ScanCandidates) — `dony_scan_asm.c` : Trouver le prochain candidat non rejeté par les tests rapides de la méthode de classement.
- [GeoV3_ScanSetup](#fn-GeoV3_ScanSetup) — `dony_scan_asm.c` : Préparer les pointeurs invariants du filtrage pour un appel à la méthode de classement.
- [getFaceFillColor](#fn-getFaceFillColor) — `zbuffer_scanline.c` : Résoudre la couleur de remplissage d'une face.
- [getFaceFrameColor](#fn-getFaceFrameColor) — `zbuffer_scanline.c` : Résoudre la couleur de contour d'une face.
- [getkeypress](#fn-getkeypress) — `asm.h` : Attendre une frappe et retourner son code sur sept bits.
- [getkeypress_openA](#fn-getkeypress_openA) — `asm.h` : Lire une frappe en conservant l'état de la touche Open-Apple.
- [getObserverParams](#fn-getObserverParams) — `engine.c` : Saisir les angles et la distance de vue en conservant les valeurs proposées.
- [getSquareRect](#fn-getSquareRect) — `color.c` : Calculer le rectangle en pixels d'une case du sélecteur.
- [hideFace](#fn-hideFace) — `engine.c` : Masquer une face en conservant sa longueur de liste pour restauration.
- [initPalettes](#fn-initPalettes) — `engine.c` : Construire les quinze palettes de dégradé et demander la capture de la palette zéro.
- [initScreenSaveBuffer](#fn-initScreenSaveBuffer) — `screen_save_restore.c` : Allouer le buffer fixe utilisé par les transferts MVN de l'écran.
- [inspect_face_pair_ui](#fn-inspect_face_pair_ui) — `engine.c` : Piloter l'inspecteur graphique et textuel de deux faces.
- [inspect_faces_order](#fn-inspect_faces_order) — `engine.c` : Inspecter les faces avant ou après une cible et proposer sa correction.
- [inspection_view_begin](#fn-inspection_view_begin) — `engine.c` : Sauvegarder visibilité, mode filaire et culling avant une inspection.
- [inspection_view_end](#fn-inspection_view_end) — `engine.c` : Restaurer la vue puis libérer sa sauvegarde.
- [inspection_view_restore](#fn-inspection_view_restore) — `engine.c` : Restaurer l'état sauvegardé sans libérer le buffer réutilisable.
- [keypress](#fn-keypress) — `asm.h` : Attendre une frappe et acquitter le strobe clavier.
- [loadModel3D](#fn-loadModel3D) — `engine.c` : Charger sommets et faces OBJ dans un modèle déjà alloué.
- [main](#fn-main) — `OBJExplorer.cc` : Piloter le chargement, les vues graphiques et les commandes de l'explorateur.
- [move_element_remove_and_insert](#fn-move_element_remove_and_insert) — `engine.c` : Déplacer un élément d'un tableau en conservant l'ordre des autres.
- [move_element_remove_and_insert_pos](#fn-move_element_remove_and_insert_pos) — `engine.c` : Déplacer un identifiant et maintenir la table inverse des positions.
- [normalize_deg](#fn-normalize_deg) — `declare.c` : Normaliser un angle entier en degrés.
- [normalizeAutoFitDistanceTo150](#fn-normalizeAutoFitDistanceTo150) — `engine.c` : Appliquer la normalisation existante des suggestions de vue vers une distance de 150.
- [Off_BridgeBorderZQ](#fn-Off_BridgeBorderZQ) — `zbuffer_fullscreen_offscreen.c` : Relier deux extrémités de spans par une ligne de contour testée contre Z.
- [Off_PaintSpanZ](#fn-Off_PaintSpanZ) — `zbuffer_fullscreen_offscreen.c` : Dessiner un span offscreen avec extrémités et liaisons de contour.
- [Off_PlotBridge](#fn-Off_PlotBridge) — `zbuffer_fullscreen_offscreen.c` : Renforcer un pixel de liaison de contour sans modifier la profondeur.
- [Off_RenderCore](#fn-Off_RenderCore) — `zbuffer_fullscreen_offscreen.c` : Rendre les surfaces par balayage plein écran offscreen.
- [Offscreen_Clear](#fn-Offscreen_Clear) — `zbuffer_fullscreen_offscreen.c` : Effacer le framebuffer par blocs de seize mots de 16 bits.
- [Offscreen_FlushToScreen](#fn-Offscreen_FlushToScreen) — `zbuffer_fullscreen_offscreen.c` : Copier le framebuffer RAM vers les 32000 octets de pixels SHGR.
- [Offscreen_Init](#fn-Offscreen_Init) — `zbuffer_fullscreen_offscreen.c` : Allouer le framebuffer de 32000 octets en RAM sans franchissement de banque.
- [Offscreen_Shutdown](#fn-Offscreen_Shutdown) — `zbuffer_fullscreen_offscreen.c` : Libérer le framebuffer offscreen détenu par le module.
- [on_seg_ll](#fn-on_seg_ll) — `overlap_raster.c` : Tester si C appartient à la boîte fermée du segment AB.
- [ordering_cache_invalidate](#fn-ordering_cache_invalidate) — `ordering_cache.c` : Changer la génération des caches et désigner leur modèle propriétaire.
- [ordering_cache_release](#fn-ordering_cache_release) — `ordering_cache.c` : Libérer les caches appartenant au modèle détruit.
- [ordering_fixed_plane](#fn-ordering_fixed_plane) — `ordering_cache.c` : Préparer le plan d'une face et les conditions du chemin assembleur.
- [ordering_fixed_plane_reset](#fn-ordering_fixed_plane_reset) — `ordering_cache.c` : Invalider les plans préparés par changement d'époque.
- [ordering_geometry_slot](#fn-ordering_geometry_slot) — `ordering_cache.c` : Obtenir l'octet du cache dense qui contient une relation de plans orientée.
- [ordering_pair_hit](#fn-ordering_pair_hit) — `ordering_cache.c` : Vérifier toutes les composantes de la clé d'une paire mémorisée.
- [ordering_pair_slot](#fn-ordering_pair_slot) — `ordering_cache.c` : Obtenir le slot de hash d'une paire orientée.
- [ordering_pair_store](#fn-ordering_pair_store) — `ordering_cache.c` : Remplacer un slot de paire par le résultat de la vue courante.
- [ordering_plane](#fn-ordering_plane) — `ordering_cache.c` : Convertir les quatre coefficients d'un plan fixe en double, avec cache.
- [ordering_winding_hit](#fn-ordering_winding_hit) — `ordering_cache.c` : Tester la validité d'une aire orientée mémorisée.
- [ordering_winding_slot](#fn-ordering_winding_slot) — `ordering_cache.c` : Obtenir le slot de l'aire orientée d'une face.
- [ordering_winding_store](#fn-ordering_winding_store) — `ordering_cache.c` : Mémoriser le double de l'aire orientée d'une face projetée.
- [ordering_workspace](#fn-ordering_workspace) — `ordering_cache.c` : Obtenir un espace de travail partagé pour le clipping.
- [orient_ll](#fn-orient_ll) — `overlap_raster.c` : Calculer l'orientation signée de trois points entiers A, B et C.
- [overlap_bench_call](#fn-overlap_bench_call) — `overlap_raster.c` : Appeler une des six méthodes chronométrées du benchmark.
- [overlap_cross_double](#fn-overlap_cross_double) — `overlap_raster.c` : Calculer le déterminant orienté de AB et AP en double.
- [overlap_v2_edge_probes](#fn-overlap_v2_edge_probes) — `overlap_raster.c` : Chercher un point intérieur commun près du milieu des arêtes des deux faces.
- [overlap_v2_point_in_face](#fn-overlap_v2_point_in_face) — `overlap_raster.c` : Classer un point double par rapport au contour projeté entier d'une face.
- [overlap_v2_proper_cross](#fn-overlap_v2_proper_cross) — `overlap_raster.c` : Tester un croisement strict de segments doubles AB et CD.
- [ovf_cross](#fn-ovf_cross) — `overlap_raster.c` : Calculer le déterminant de trois points entiers avec intermédiaires ovf64.
- [ovf_inside](#fn-ovf_inside) — `overlap_raster.c` : Classer un point rationnel par rapport à un contour projeté entier.
- [ovf_probe](#fn-ovf_probe) — `overlap_raster.c` : Chercher des sondes rationnelles communes près des milieux d'arêtes.
- [ovmax_inside](#fn-ovmax_inside) — `overlap_raster.c` : Classer un point double avec les tableaux projetés passés directement.
- [ovmax_probe](#fn-ovmax_probe) — `overlap_raster.c` : Sonder les milieux d'arêtes dans la variante à tableaux directs.
- [painter_bubble_sort](#fn-painter_bubble_sort) — `engine.c` : Corriger l'ordre FAST par échanges de voisins avec tests géométriques.
- [painter_correct](#fn-painter_correct) — `engine.c` : Exécuter CORRECT ou CORRECT V2 à partir de la même préparation FAST.
- [painter_geoV2](#fn-painter_geoV2) — `engine.c` : Corriger l'ordre FAST par passes de voisins et cache de paires borné.
- [painter_geoV3](#fn-painter_geoV3) — `dony.c` : Construire l'ordre arrière-vers-avant par sélection et reprise des comparaisons.
- [painter_newell_sancha](#fn-painter_newell_sancha) — `engine.c` : Corriger l'ordre FAST à partir des paires dont les intervalles Z se recouvrent.
- [painter_newell_sancha_fast](#fn-painter_newell_sancha_fast) — `engine.c` : Construire l'ordre FAST par profondeur moyenne décroissante.
- [pair_cache_create](#fn-pair_cache_create) — `engine.c` : Allouer un cache de relations de paires avec sondage borné.
- [pair_cache_destroy](#fn-pair_cache_destroy) — `engine.c` : Libérer un cache de paires et ses slots.
- [pair_cache_find](#fn-pair_cache_find) — `engine.c` : Lire une relation en sondant au plus huit slots.
- [pair_cache_insert](#fn-pair_cache_insert) — `engine.c` : Mémoriser une relation en cherchant au plus huit slots.
- [pair_cache_slot](#fn-pair_cache_slot) — `engine.c` : Calculer le slot initial d'une paire non orientée.
- [pair_order_relation](#fn-pair_order_relation) — `engine.c` : Fournir une préférence d'ordre par profondeur, bornes puis plans.
- [paletteGradientColor](#fn-paletteGradientColor) — `declare.c` : Construire une entrée d'une palette de dégradé.
- [point_in_poly_arrays_int](#fn-point_in_poly_arrays_int) — `overlap_raster.c` : Tester un point entier par parité des traversées d'un rayon horizontal.
- [point_in_poly_int](#fn-point_in_poly_int) — `overlap_raster.c` : Tester l'intérieur strict d'une face projetée à un point entier.
- [poly_signed_area](#fn-poly_signed_area) — `engine.c` : Mesurer l'aire orientée de la projection dominante d'un contour objet.
- [processModelFast](#fn-processModelFast) — `engine.c` : Transformer les sommets, projeter, calculer les faces et exécuter la méthode de classement choisie.
- [projected_polygons_overlap](#fn-projected_polygons_overlap) — `overlap_raster.c` : Choisir la méthode de chevauchement configurée à l'exécution.
- [projected_polygons_overlap_cached](#fn-projected_polygons_overlap_cached) — `overlap_raster.c` : Mémoriser uniquement les chevauchements positifs pour la méthode de classement.
- [projected_polygons_overlap_raster](#fn-projected_polygons_overlap_raster) — `overlap_raster.c` : Chercher un centre de pixel strictement couvert par les deux faces.
- [projected_polygons_overlap_raster_x10](#fn-projected_polygons_overlap_raster_x10) — `overlap_raster.c` : Tester le chevauchement raster après un grossissement temporaire par dix.
- [projected_polygons_overlap_simple](#fn-projected_polygons_overlap_simple) — `overlap_raster.c` : Tester rapidement croisements propres et inclusion stricte de sommets.
- [projected_polygons_overlap_v1](#fn-projected_polygons_overlap_v1) — `overlap_raster.c` : Tester le chevauchement avec les filtres et heuristiques historiques V1.
- [projected_polygons_overlap_v2_fixed64](#fn-projected_polygons_overlap_v2_fixed64) — `overlap_raster.c` : Tester le chevauchement projeté par Fixed64 avec déterminants et sondes rationnelles.
- [projected_polygons_overlap_v2_max](#fn-projected_polygons_overlap_v2_max) — `overlap_raster.c` : Tester le chevauchement projeté par V2 Max avec pointeurs locaux et rejets par boîtes d'arêtes.
- [projected_polygons_overlap_v2_original](#fn-projected_polygons_overlap_v2_original) — `overlap_raster.c` : Tester le chevauchement projeté par V2 original en double.
- [raster_advance_edge](#fn-raster_advance_edge) — `overlap_raster.c` : Avancer une arête préparée d'une ligne raster.
- [raster_edge_intersection](#fn-raster_edge_intersection) — `overlap_raster.c` : Lire l'intersection entière tronquée vers zéro d'une arête préparée.
- [raster_euclidean_divide](#fn-raster_euclidean_divide) — `overlap_raster.c` : Effectuer une division euclidienne avec reste positif ou nul.
- [raster_incremental_scanline](#fn-raster_incremental_scanline) — `overlap_raster.c` : Extraire et trier les intersections d'une ligne à partir d'arêtes préparées.
- [raster_intervals_share_pixel](#fn-raster_intervals_share_pixel) — `overlap_raster.c` : Tester si deux intervalles ouverts contiennent un centre de pixel commun.
- [raster_prepare_edges_incremental](#fn-raster_prepare_edges_incremental) — `overlap_raster.c` : Préparer les arêtes actives pour des lignes raster successives.
- [raster_sort_long](#fn-raster_sort_long) — `overlap_raster.c` : Trier une courte liste d'intersections par insertion croissante.
- [ray_cast_at](#fn-ray_cast_at) — `engine.c` : Comparer deux plans au passage d'un rayon de coordonnées écran entières.
- [ray_cast_at_fractional](#fn-ray_cast_at_fractional) — `engine.c` : Comparer deux plans au passage d'un rayon écran subpixel en double.
- [ray_cast_distances](#fn-ray_cast_distances) — `engine.c` : Calculer les paramètres d'intersection d'un rayon écran avec deux plans.
- [ray_cast_hierarchical](#fn-ray_cast_hierarchical) — `engine.c` : Départager une paire avec grossissement temporaire et cache orienté.
- [ray_cast_hierarchical_impl](#fn-ray_cast_hierarchical_impl) — `engine.c` : Essayer plusieurs points représentatifs pour départager deux faces.
- [rch_gather_pair_vertex_indices](#fn-rch_gather_pair_vertex_indices) — `engine.c` : Rassembler les identifiants distincts des sommets de deux faces.
- [rch_patch_pair_at_scale](#fn-rch_patch_pair_at_scale) — `engine.c` : Sauvegarder puis reprojeter temporairement les sommets de deux faces.
- [rch_recompute_face_bbox_cache](#fn-rch_recompute_face_bbox_cache) — `engine.c` : Recalculer la boîte projetée d'une seule face après patch de projection.
- [rch_unpatch_pair](#fn-rch_unpatch_pair) — `engine.c` : Restaurer l'état temporairement remplacé par rch_patch_pair_at_scale.
- [readFaces_model](#fn-readFaces_model) — `engine.c` : Lire les lignes OBJ « f » dans le buffer compact des faces.
- [ReadPalette](#fn-ReadPalette) — `engine.c` : Lire seize mots de couleur à partir de l'adresse calculée pour une palette.
- [compute_auto_fit](#fn-compute_auto_fit) — `engine.c` : Calculer la distance et l'échelle suggérées pour la vue initiale.
- [readVertices](#fn-readVertices) — `engine.c` : Lire les lignes OBJ « v x y z », centrer la géométrie et proposer une vue.
- [remove_duplicates](#fn-remove_duplicates) — `engine.c` : Retirer toutes les répétitions d'identifiants en conservant leur premier ordre.
- [renderModelFullscreenZBuffer](#fn-renderModelFullscreenZBuffer) — `zbuffer_fullscreen.c` : Choisir la variante de rendu public selon le back-face culling.
- [renderModelFullscreenZBuffer_biasedV2](#fn-renderModelFullscreenZBuffer_biasedV2) — `zbuffer_fullscreen.c` : Appeler ZBuffer_RenderCore avec rapprochement des faces de plane_d>0.
- [renderModelFullscreenZBuffer_fastV2](#fn-renderModelFullscreenZBuffer_fastV2) — `zbuffer_fullscreen.c` : Appeler ZBuffer_RenderCore sans rapprochement des faces de plane_d>0.
- [renderModelFullscreenZBuffer_offscreen](#fn-renderModelFullscreenZBuffer_offscreen) — `zbuffer_fullscreen_offscreen.c` : Choisir la variante de rendu public selon le back-face culling.
- [renderModelFullscreenZBuffer_offscreen_biasedV2](#fn-renderModelFullscreenZBuffer_offscreen_biasedV2) — `zbuffer_fullscreen_offscreen.c` : Appeler Off_RenderCore avec rapprochement des faces de plane_d>0.
- [renderModelFullscreenZBuffer_offscreen_fastV2](#fn-renderModelFullscreenZBuffer_offscreen_fastV2) — `zbuffer_fullscreen_offscreen.c` : Appeler Off_RenderCore sans rapprochement des faces de plane_d>0.
- [renderModelScanlineZBuffer](#fn-renderModelScanlineZBuffer) — `zbuffer_scanline.c` : Choisir la variante de rendu public selon le back-face culling.
- [renderModelScanlineZBuffer_biasedV3](#fn-renderModelScanlineZBuffer_biasedV3) — `zbuffer_scanline.c` : Appeler renderModelScanlineZBuffer_coreV3 avec rapprochement des faces de plane_d>0.
- [renderModelScanlineZBuffer_coreV3](#fn-renderModelScanlineZBuffer_coreV3) — `zbuffer_scanline.c` : Rendre les surfaces par balayage scanline V3 quantifié.
- [renderModelScanlineZBuffer_fastV3](#fn-renderModelScanlineZBuffer_fastV3) — `zbuffer_scanline.c` : Appeler renderModelScanlineZBuffer_coreV3 sans rapprochement des faces de plane_d>0.
- [reorder_poly](#fn-reorder_poly) — `engine.c` : Réordonner une liste de sommets par angle autour de leur centre moyen projeté.
- [repair_geov3](#fn-repair_geov3) — `engine.c` : Inspecter et corriger l'ordre courant par les échanges proposés par GEO V3.
- [repair_geov3_confirm](#fn-repair_geov3_confirm) — `engine.c` : Montrer les deux faces d'un échange GEO V3 et recueillir la décision.
- [restoreAllFaces](#fn-restoreAllFaces) — `engine.c` : Restaurer les longueurs de toutes les faces précédemment masquées.
- [restoreFace](#fn-restoreFace) — `engine.c` : Restaurer la longueur d'une face précédemment masquée.
- [restoreScreen](#fn-restoreScreen) — `screen_save_restore.c` : Restaurer la zone SHGR depuis la dernière sauvegarde.
- [reverseFaceVertexOrder](#fn-reverseFaceVertexOrder) — `engine.c` : Inverser le winding d'une face et son plan observateur.
- [run_raycast_test](#fn-run_raycast_test) — `engine.c` : Adapter le résultat d'un rayon à la convention du diagnostic de paire.
- [saveNextScreenshot](#fn-saveNextScreenshot) — `engine.c` : Choisir le premier nom screen000.PIC à screen999.PIC qui ne peut être ouvert en lecture.
- [saveScreen](#fn-saveScreen) — `screen_save_restore.c` : Copier la zone SHGR vers le buffer préalablement alloué.
- [saveSHGRAsRawPic](#fn-saveSHGRAsRawPic) — `engine.c` : Écrire la zone SHGR brute dans un fichier PIC non compressé.
- [screen2Black](#fn-screen2Black) — `engine.c` : Mettre à zéro les mots SHGR parcourus par la boucle d'effacement.
- [screenClearTopLines](#fn-screenClearTopLines) — `screen_save_restore.c` : Effacer les premières lignes de pixels de l'écran SHGR 320x200.
- [segs_intersect_int](#fn-segs_intersect_int) — `overlap_raster.c` : Tester une intersection propre de deux segments entiers.
- [segs_intersect_int_fixed64](#fn-segs_intersect_int_fixed64) — `overlap_raster.c` : Tester une intersection propre de deux segments entiers.
- [select_overlap_method](#fn-select_overlap_method) — `overlap_raster.c` : Demander un choix runtime de méthode sans modifier directement l'état global.
- [setProDOSFileType](#fn-setProDOSFileType) — `engine.c` : Affecter les métadonnées GS/OS d'un fichier existant.
- [shgroff](#fn-shgroff) — `asm.h` : Désactiver l'affichage SHGR en écrivant $41 dans $00C029.
- [show_face_normal](#fn-show_face_normal) — `engine.c` : Dessiner en jaune la normale de la face sélectionnée dans l'inspecteur.
- [show_help_pager](#fn-show_help_pager) — `engine.c` : Afficher cycliquement les pages d'aide textuelle jusqu'à Échap.
- [showFace](#fn-showFace) — `engine.c` : Explorer une face, sa normale et ses données détaillées dans une vue interactive.
- [sin_deg_int](#fn-sin_deg_int) — `declare.c` : Lire le sinus d'un angle entier dans la table 16.16.
- [SL_ClearLine](#fn-SL_ClearLine) — `zbuffer_scanline.c` : Réinitialiser la ligne Z réutilisée par le rendu scanline V3.
- [SL_PaintSpan](#fn-SL_PaintSpan) — `zbuffer_scanline.c` : Préparer un span V3 qui utilise la seule ligne de profondeur courante.
- [SL_SetScale](#fn-SL_SetScale) — `zbuffer_scanline.c` : Adapter la quantification scanline en réservant de la marge pour le biais.
- [sort_by_zmax](#fn-sort_by_zmax) — `dony.c` : Trier les indices de faces par profondeur maximale décroissante, de façon stable.
- [split_face_by_plane](#fn-split_face_by_plane) — `engine.c` : Découper expérimentalement f1 par le plan objet de f2.
- [strict_inside](#fn-strict_inside) — `engine.c` : Tester l'appartenance stricte d'un point à un polygone entier.
- [updateFace2DBounds](#fn-updateFace2DBounds) — `engine.c` : Recalculer les boîtes écran des faces après reprojection.
- [ZBFS_FreeEdges](#fn-ZBFS_FreeEdges) — `zbuffer_fullscreen.c` : Libérer les arêtes préparées.
- [ZBFS_FreeFaces](#fn-ZBFS_FreeFaces) — `zbuffer_fullscreen.c` : Libérer les tables de faces, couleurs, biais et plans.
- [ZBFS_FreeScratch](#fn-ZBFS_FreeScratch) — `zbuffer_fullscreen.c` : Libérer les trois familles de buffers auxiliaires du plein écran direct.
- [ZBFS_FreeVertices](#fn-ZBFS_FreeVertices) — `zbuffer_fullscreen.c` : Libérer les profondeurs inverses et codes des sommets.
- [ZBuffer_ActivateFaces](#fn-ZBuffer_ActivateFaces) — `zbuffer_common.c` : Ajouter les faces d’une ligne à la liste active ordonnée.
- [ZBuffer_Clear](#fn-ZBuffer_Clear) — `zbuffer_fullscreen.c` : Remplir de $FFFF les 200 lignes de profondeur.
- [ZBuffer_DrawEdge](#fn-ZBuffer_DrawEdge) — `zbuffer_fullscreen.c` : Rasteriser un côté par Bresenham et tester sa visibilité après les surfaces.
- [ZBuffer_FreeFaces](#fn-ZBuffer_FreeFaces) — `zbuffer_common.c` : Libérer les tables communes de préparation des faces.
- [ZBuffer_FreeVertices](#fn-ZBuffer_FreeVertices) — `zbuffer_common.c` : Libérer une réserve de profondeurs de sommets.
- [ZBuffer_Init](#fn-ZBuffer_Init) — `zbuffer_fullscreen.c` : Réserver le Z-buffer plein écran aligné sur une frontière de banque.
- [ZBuffer_LerpZ](#fn-ZBuffer_LerpZ) — `zbuffer_common.c` : Interpoler un code de profondeur lors du clipping horizontal d'un span.
- [ZBuffer_PaintSpanZ](#fn-ZBuffer_PaintSpanZ) — `zbuffer_fullscreen.c` : Préparer un span de surface évalué sur le plan Z commun de la face.
- [ZBuffer_PlaneAt](#fn-ZBuffer_PlaneAt) — `zbuffer_fullscreen.c` : Évaluer le plan Z à un pixel écran sans encore réduire ses fractions.
- [ZBuffer_PlaneCode](#fn-ZBuffer_PlaneCode) — `zbuffer_fullscreen.c` : Arrondir et borner un accumulateur Z à douze bits fractionnaires.
- [ZBuffer_EdgeRun](#fn-ZBuffer_EdgeRun) — `zbuffer_fullscreen.c` : Parcourir une arête préparée et écrire ses pixels visibles en SHGR.
- [ZBuffer_PrepareFaces](#fn-ZBuffer_PrepareFaces) — `zbuffer_common.c` : Filtrer les faces et préparer leurs premières lignes et couleurs.
- [ZBuffer_PrepareInvZ](#fn-ZBuffer_PrepareInvZ) — `zbuffer_common.c` : Préparer les profondeurs inverses des sommets d’une vue.
- [ZBuffer_PrepareScanEdges](#fn-ZBuffer_PrepareScanEdges) — `zbuffer_common.c` : Préparer les arêtes quantifiées communes à scanline et offscreen.
- [ZBuffer_QuantizeAtScale](#fn-ZBuffer_QuantizeAtScale) — `zbuffer_common.c` : Quantifier une profondeur inverse avec une échelle explicite.
- [ZBuffer_QuantizeInvZ](#fn-ZBuffer_QuantizeInvZ) — `zbuffer_fullscreen.c` : Quantifier une profondeur inverse en code 16 bits où petit signifie proche.
- [ZBuffer_RenderCore](#fn-ZBuffer_RenderCore) — `zbuffer_fullscreen.c` : Rendre les surfaces par balayage plein écran direct.
- [ZBuffer_ReserveFaces](#fn-ZBuffer_ReserveFaces) — `zbuffer_common.c` : Garantir les six tables communes de faces.
- [ZBuffer_ReserveVertices](#fn-ZBuffer_ReserveVertices) — `zbuffer_common.c` : Garantir la capacité des trois buffers de profondeur.
- [ZBuffer_SetEdgeFlags](#fn-ZBuffer_SetEdgeFlags) — `zbuffer_fullscreen.c` : Installer un masque facultatif des côtés de faces, sans prendre sa propriété.
- [ZBuffer_SetPlane](#fn-ZBuffer_SetPlane) — `zbuffer_fullscreen.c` : Préparer un plan affine de codes Z dans le repère écran.
- [ZBuffer_SetScaleForFrame](#fn-ZBuffer_SetScaleForFrame) — `zbuffer_fullscreen.c` : Adapter l'échelle de quantification à la profondeur inverse maximale.
- [ZBuffer_Shutdown](#fn-ZBuffer_Shutdown) — `zbuffer_fullscreen.c` : Libérer les ressources du rendu plein écran direct.
- [ZBuffer_SizeFits](#fn-ZBuffer_SizeFits) — `zbuffer_common.c` : Vérifier qu'un tableau de count éléments est représentable par size_t.
- [ZBuffer_SortFaceStarts](#fn-ZBuffer_SortFaceStarts) — `zbuffer_common.c` : Transformer l’histogramme en liste stable des départs de faces.
- [ZBuffer_SortScanHits](#fn-ZBuffer_SortScanHits) — `zbuffer_common.c` : Ordonner les intersections quantifiées d’un span.
- [ZBuffer_SpanRun](#fn-ZBuffer_SpanRun) — `zbuffer_common.c` : Dessiner un span préclippé avec test Z strict dans un noyau 65C816.


## Contrats des fonctions

<a id="fn-add_vertex"></a>

### add_vertex

Définition : [engine.c](engine.c), ligne 1684.

```c
static int add_vertex(Model3D* model, Fixed32 x, Fixed32 y, Fixed32 z)
```

**Rôle :**

Ajouter un sommet au repère objet du modèle.

**Paramètres :**

[entrée] model et x/y/z en 16.16.

**Retour :**

Indice à base zéro, ou -1 pour modèle NULL ou capacité MAX_VERTICES atteinte.

**Sorties et effets de bord :**

coordonnées, compteur augmenté, coordonnées observateur/écran initialisées à zéro et cache fractionnaire invalidé.

**Fonctionnement :**

La transformation de vue devra être recalculée avant d'utiliser le sommet au rendu.

Appels internes directs : [fractional_projection_cache_invalidate](arbre_appels.html#fn-fractional_projection_cache_invalidate).

<a id="fn-applyPalette"></a>

### applyPalette

Définition : [engine.c](engine.c), ligne 6301.

```c
void applyPalette(int palette_num)
```

**Rôle :**

Installer une palette logicielle dans la table matérielle active numéro zéro.

**Paramètres :**

[entrée] palette_num, ramené à 0 s'il n'est pas dans [0,15].

**Retour :**

Aucun.

**Sorties et effets de bord :**

SetColorTable ; requiert QuickDraw actif et palettes déjà préparées.

**Fonctionnement :**

Le numéro sélectionne la source logicielle, pas le numéro de table destination.

Appels internes directs : aucun repéré.

<a id="fn-benchmark_projected_overlap_all"></a>

### benchmark_projected_overlap_all

Définition : [overlap_raster.c](overlap_raster.c), ligne 3618.

```c
static void benchmark_projected_overlap_all(Model3D *model, ObserverParams *params, int angle_count)
```

**Rôle :**

Comparer six méthodes sur toutes les paires et plusieurs orientations.

**Paramètres :**

[entrée] model, params, angle_count proposé (10 si non positif), modifiable au clavier entre 1 et 10000.

**Retour :**

Aucun.

**Fonctionnement :**

Écrase overlap.all.txt. Génère des angles déterministes, recalcule les projections et métriques, chronomètre des blocs répétés hors écritures disque puis exporte décisions et contours pour référence externe. Les désaccords ne désignent pas à eux seuls la méthode correcte ; aucun oracle géométrique n'est exécuté ici. Les angles params sont restaurés à la fin, mais les tableaux de la dernière vue ne sont pas recalculés pour ces angles restaurés dans cette fonction.

Appels internes directs : [calculateFaceDepths](arbre_appels.html#fn-calculateFaceDepths), [cos_deg_int](arbre_appels.html#fn-cos_deg_int), [FIXED_ROUND_TO_INT](arbre_appels.html#fn-FIXED_ROUND_TO_INT), [overlap_bench_call](arbre_appels.html#fn-overlap_bench_call), [sin_deg_int](arbre_appels.html#fn-sin_deg_int).

<a id="fn-calculateFaceDepths"></a>

### calculateFaceDepths

Définition : [engine.c](engine.c), ligne 4967.

```c
void calculateFaceDepths(Model3D* model, Face3D* faces, int face_count)
```

**Rôle :**

Recalculer les métriques et plans observateur des faces de la vue courante.

**Paramètres :**

[entrée] model et face_count ; faces est un paramètre historique non utilisé.

**Retour :**

Aucun.

**Sorties et effets de bord :**

zmin/zmax/zmean, bornes écran, plans de Newell et display_flag.

**Fonctionnement :**

Invalide les caches d'ordre avant de remplacer les métriques. Le parcours est assembleur par défaut si disponible, sinon C ; le centroïde est la moyenne des sommets valides. Les produits bruts de Newell sont cumulés avant un unique décalage de 16 bits ; d utilise ensuite ce centroïde. Une profondeur strictement négative invalide la face ; le culling optionnel rejette d <= 0. Le code distingue ce test du seuil zo <= 0 de la projection. Les faces dégénérées reçoivent des plans nuls ; cela ne force pas ici à lui seul display_flag à zéro. Les indices positifs doivent déjà être valides.

Appels internes directs : [FaceDepth_CentroidCoordinate](arbre_appels.html#fn-FaceDepth_CentroidCoordinate), [FaceDepth_ScanFace](arbre_appels.html#fn-FaceDepth_ScanFace), [FaceDepth_ScanSetup](arbre_appels.html#fn-FaceDepth_ScanSetup), [fixed_product32_fast](arbre_appels.html#fn-fixed_product32_fast), [fixed_product64x32_fast](arbre_appels.html#fn-fixed_product64x32_fast), [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate).

<a id="fn-check_intersect"></a>

### check_intersect

Définition : [engine.c](engine.c), ligne 896.

```c
static void check_intersect(Model3D* model)
```

**Rôle :**

Examiner les intersections objet et tenter des découpages avant la visualisation.

**Paramètres :**

[entrée-sortie] model ; NULL est accepté.

**Retour :**

Aucun.

**Fonctionnement :**

Cette commande modifie potentiellement sommets, faces et bornes ; ce n'est pas un simple diagnostic. Filtre les paires par boîtes 3D, écarte celles partageant un sommet, estime la coplanarité puis compte les traversées avant split_face_by_plane. Recommence après chaque découpage. Les compteurs peuvent inclure plusieurs passages.

**Limites :**

réserves des nouvelles faces/indices non vérifiées ici, échec des bbox non contrôlé et aucun budget de reprises ; outil expérimental.

Appels internes directs : [compute_face_bboxes3D](arbre_appels.html#fn-compute_face_bboxes3D), [compute_face_plane](arbre_appels.html#fn-compute_face_plane), [count_edge_crossings](arbre_appels.html#fn-count_edge_crossings), [edge_crosses_interior](arbre_appels.html#fn-edge_crosses_interior), [faces_share_vertex_or_edge](arbre_appels.html#fn-faces_share_vertex_or_edge), [keypress](arbre_appels.html#fn-keypress), [split_face_by_plane](arbre_appels.html#fn-split_face_by_plane).

<a id="fn-cleanup_list"></a>

### cleanup_list

Définition : [engine.c](engine.c), ligne 1726.

```c
static void cleanup_list(int *arr, int *n)
```

**Rôle :**

Compacter une liste cyclique d'indices en retirant les répétitions adjacentes.

**Paramètres :**

[entrée-sortie] arr et *n.

**Retour :**

Aucun.

**Fonctionnement :**

Supprime aussi la répétition du premier indice en fin de liste ; les répétitions non adjacentes restent présentes.

Appels internes directs : aucun repéré.

<a id="fn-cmp_faces_by_zmean"></a>

### cmp_faces_by_zmean

Définition : [engine.c](engine.c), ligne 2196.

```c
static int cmp_faces_by_zmean(const void* pa, const void* pb)
```

**Rôle :**

Comparer deux identifiants de faces pour qsort.

**Paramètres :**

[entrée] pa/pb, pointeurs vers int ; qsort_faces_ptr_for_cmp, métriques partagées.

**Retour :**

Négatif si a précède b, positif sinon, zéro si même identifiant.

**Fonctionnement :**

Ordre décroissant de z_mean ; les égalités sont départagées par identifiant croissant. Non réentrant en raison du pointeur global de contexte.

Appels internes directs : aucun repéré.

<a id="fn-colorChooser"></a>

### colorChooser

Définition : [color.c](color.c), ligne 383.

```c
int colorChooser(int isFrameChooser, int *palette, int initialSelection)
```

**Rôle :**

Exécuter la sélection interactive de couleur jusqu'à confirmation.

**Paramètres :**

[entrée] isFrameChooser, 0 remplissage ou 1 contour ; initialSelection, choix initial valide.

[entrée-sortie] *palette, numéro 0..15.

**Retour :**

Indice 0..15, COLOR_RANDOM, ou COLOR_SAME_AS_FILL pour le contour.

**Sorties et effets de bord :**

écran redessiné, palette matérielle et crayon modifiés.

**Fonctionnement :**

Haut/bas changent de palette, gauche/droite parcourent les choix ; toute autre touche confirme.

La scène précédente doit être redessinée ou restaurée par l'appelant.

Appels internes directs : [applyPalette](arbre_appels.html#fn-applyPalette), [drawBlackSquareMarker](arbre_appels.html#fn-drawBlackSquareMarker), [drawInstructions](arbre_appels.html#fn-drawInstructions), [drawPaletteNumber](arbre_appels.html#fn-drawPaletteNumber), [drawSelectionBorder](arbre_appels.html#fn-drawSelectionBorder), [drawSquare](arbre_appels.html#fn-drawSquare), [drawSquareLabel](arbre_appels.html#fn-drawSquareLabel), [getkeypress](arbre_appels.html#fn-getkeypress).

<a id="fn-compare_faces_diagnostic"></a>

### compare_faces_diagnostic

Définition : [engine.c](engine.c), ligne 8125.

```c
static void compare_faces_diagnostic(Model3D* model, int f1, int f2, int use_qd, CompareFacesResult *out)
```

**Rôle :**

Comparer plusieurs méthodes d'ordre et conserver leurs résultats séparés.

**Paramètres :**

[entrée] model, f1/f2 valides et use_qd, disponibilité des opérations QuickDraw.

**Retour :**

Aucun.

**Sorties et effets de bord :**

*out réinitialisé

caches et buffers partagés de clipping ; ne corrige pas la permutation.

**Fonctionnement :**

Produit verdicts, boîtes, centroïdes, aires et temps en ticks. Les verdicts 1/2 désignent les positions f1/f2 de la paire ; certaines légendes historiques doivent être confrontées aux valeurs.

Mémorise le rectangle du contour SH dans le résultat, avec le centroïde de la même projection, avant toute réutilisation des buffers partagés.

Appels internes directs : [compute_bbox_intersection](arbre_appels.html#fn-compute_bbox_intersection), [compute_intersection_centroid_ordered_fixed_tryboth](arbre_appels.html#fn-compute_intersection_centroid_ordered_fixed_tryboth), [compute_intersection_centroid_ordered_qd_fixed](arbre_appels.html#fn-compute_intersection_centroid_ordered_qd_fixed), [geo_face_order](arbre_appels.html#fn-geo_face_order), [projected_polygons_overlap](arbre_appels.html#fn-projected_polygons_overlap), [run_raycast_test](arbre_appels.html#fn-run_raycast_test).

<a id="fn-compute2DFromObserver"></a>

### compute2DFromObserver

Définition : [engine.c](engine.c), ligne 5160.

```c
void compute2DFromObserver(Model3D* model, int angle_w)
```

**Rôle :**

Reprojeter une vue observateur existante après changement d'échelle.

**Paramètres :**

[entrée-sortie] model, coordonnées projetées et bornes des faces.

[entrée] angle_w, ignoré car la rotation est déjà contenue dans xo/yo/zo.

**Retour :**

Aucun.

**Fonctionnement :**

Synchronise auto_proj_scale avec l'échelle globale et invalide les caches d'ordre via updateFace2DBounds.

**Préconditions :**

model valide et tous les zo divisés non nuls ; contrairement à processModelFast, aucun rejet zo <= 0 n'est effectué ici. La transformation 3D et les plans ne sont pas recalculés.

Appels internes directs : [FIXED_ROUND_TO_INT](arbre_appels.html#fn-FIXED_ROUND_TO_INT), [updateFace2DBounds](arbre_appels.html#fn-updateFace2DBounds).

<a id="fn-compute_bbox_intersection"></a>

### compute_bbox_intersection

Définition : [overlap_raster.c](overlap_raster.c), ligne 345.

```c
static int compute_bbox_intersection(Model3D* model, int f1, int f2, int *ix0, int *iy0, int *ix1, int *iy1)
```

**Rôle :**

Calculer le rectangle commun aux boîtes projetées de deux faces.

**Paramètres :**

[entrée] model, f1/f2 base zéro

**Retour :**

1 si les intervalles ont un point commun, 0 sinon ou arguments invalides.

**Sorties et effets de bord :**

ix0/iy0/ix1/iy1.

**Fonctionnement :**

Accepte les contacts d'aire nulle ; les sorties ne sont écrites qu'en réussite.

Appels internes directs : aucun repéré.

<a id="fn-compute_bbox_intersection_center"></a>

### compute_bbox_intersection_center

Définition : [overlap_raster.c](overlap_raster.c), ligne 312.

```c
static int compute_bbox_intersection_center(Model3D* model, int f1, int f2, int *outx, int *outy)
```

**Rôle :**

Retourner le centre entier du rectangle commun des boîtes de deux faces.

**Paramètres :**

[entrée] model et f1/f2

**Retour :**

1 en cas de rectangle commun, 0 sinon.

**Sorties et effets de bord :**

outx/outy.

**Fonctionnement :**

Ce point peut être hors des deux polygones.

Appels internes directs : [compute_bbox_intersection](arbre_appels.html#fn-compute_bbox_intersection).

<a id="fn-compute_face_bboxes3D"></a>

### compute_face_bboxes3D

Définition : [engine.c](engine.c), ligne 1951.

```c
static void compute_face_bboxes3D(Model3D* model)
```

**Rôle :**

Recalculer les six bornes objet de chaque face.

**Paramètres :**

[entrée-sortie] model.

**Retour :**

Aucun ; modèle NULL ou sans face est ignoré.

**Fonctionnement :**

Remplace les six tableaux min/max en 16.16. Sur échec d'allocation, les libère tous et les remet à NULL. Les indices des faces doivent être valides. L'appelant doit tenir compte de l'absence possible de bornes après cet échec.

Appels internes directs : aucun repéré.

<a id="fn-compute_face_plane"></a>

### compute_face_plane

Définition : [engine.c](engine.c), ligne 1190.

```c
static void compute_face_plane(Model3D* model, int fidx, double *a, double *b, double *c, double *d, int coordinate_mode)
```

**Rôle :**

Calculer le plan du triangle initial d'une face dans le repère objet.

**Paramètres :**

[entrée] model et fidx : modèle et identifiant de face à base zéro. [entrée] coordinate_mode : FACE_PLANE_RAW_FIXED pour les entiers 16.16 bruts, FACE_PLANE_OBJECT_UNITS pour les coordonnées en unités objet. [sortie] a/b/c/d : pointeurs valides vers les quatre coefficients.

**Retour :**

Aucun ; modèle absent, face invalide ou moins de trois sommets donnent quatre coefficients nuls.

**Fonctionnement :**

Lit les trois premiers sommets puis calcule le produit vectoriel et d, sans normaliser la normale. Le mode objet conserve la conversion par FIXED_TO_FLOAT avant passage en double ; le mode brut convertit les entiers directement en double. Les échelles des coefficients diffèrent entre les modes. Les conversions précèdent toujours les soustractions. Aucun état du modèle n'est modifié.

**Limites :**

Des sommets initiaux colinéaires donnent un plan dégénéré même si la face complète définit un plan. Ce calcul reste distinct de Newell sur le contour observateur dans calculateFaceDepths.

Appels internes directs : aucun repéré.

<a id="fn-compute_intersection_centroid_fractional_tryboth"></a>

### compute_intersection_centroid_fractional_tryboth

Définition : [overlap_raster.c](overlap_raster.c), ligne 1175.

```c
static int compute_intersection_centroid_fractional_tryboth(Model3D *model, int subj, int clip, double *outx, double *outy, double *out_area2)
```

**Rôle :**

Essayer le clipping subpixel dans les deux sens sans arrondir le centroïde.

**Paramètres :**

[entrée] model, subj/clip

**Retour :**

1 si un sens réussit, 0 sinon.

**Sorties et effets de bord :**

outx/outy et out_area2, double de l'aire positive en pixels carrés.

**Fonctionnement :**

Réutilise un workspace partagé ; l'inversion sujet/clip n'annule pas les limites de convexité du clipping. Aucune modification des coordonnées entières du modèle.

Appels internes directs : [compute_intersection_centroid_ordered_fixed_work](arbre_appels.html#fn-compute_intersection_centroid_ordered_fixed_work), [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate), [ordering_workspace](arbre_appels.html#fn-ordering_workspace).

<a id="fn-compute_intersection_centroid_ordered_fixed"></a>

### compute_intersection_centroid_ordered_fixed

Définition : [overlap_raster.c](overlap_raster.c), ligne 1129.

```c
static int compute_intersection_centroid_ordered_fixed(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2)
```

**Rôle :**

Appeler le clipping ordonné entier avec un espace de travail réutilisé.

**Paramètres :**

[entrée] model, subj/clip

**Retour :**

Résultat du noyau, dont 1 peut accompagner une aire nulle.

**Sorties et effets de bord :**

outx/outy facultatifs, out_area2 obligatoire.

cache de workspace et diagnostics globaux ; mémoire non transférée à l'appelant.

Appels internes directs : [compute_intersection_centroid_ordered_fixed_work](arbre_appels.html#fn-compute_intersection_centroid_ordered_fixed_work), [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate), [ordering_workspace](arbre_appels.html#fn-ordering_workspace).

<a id="fn-compute_intersection_centroid_ordered_fixed_tryboth"></a>

### compute_intersection_centroid_ordered_fixed_tryboth

Définition : [overlap_raster.c](overlap_raster.c), ligne 1231.

```c
static int compute_intersection_centroid_ordered_fixed_tryboth(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2, int* out_fallback)
```

**Rôle :**

Essayer le clipping entier puis chercher un point de repli commun.

**Paramètres :**

[entrée] model, subj/clip

**Retour :**

0 sur arguments/réserve invalides ; 1 peut désigner uniquement un repli avec out_area2=0 et out_fallback=1.

**Sorties et effets de bord :**

outx/outy facultatifs, out_area2 obligatoire, out_fallback facultatif.

**Fonctionnement :**

Après les deux sens SH, essaye le centre de bbox puis une grille 3x3 à l'intérieur strict des deux faces. Si rien ne convient, rend quand même le centre disponible. L'appelant doit tester l'aire ou le drapeau, pas seulement le booléen de retour.

Appels internes directs : [compute_bbox_intersection](arbre_appels.html#fn-compute_bbox_intersection), [compute_intersection_centroid_ordered_fixed_work](arbre_appels.html#fn-compute_intersection_centroid_ordered_fixed_work), [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate), [ordering_workspace](arbre_appels.html#fn-ordering_workspace), [point_in_poly_int](arbre_appels.html#fn-point_in_poly_int).

<a id="fn-compute_intersection_centroid_ordered_fixed_work"></a>

### compute_intersection_centroid_ordered_fixed_work

Définition : [overlap_raster.c](overlap_raster.c), ligne 662.

```c
static int compute_intersection_centroid_ordered_fixed_work(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2, double* shared_work, double* fractional_x, double* fractional_y, double* fractional_area2)
```

**Rôle :**

Découper un sujet par les demi-plans du clip et calculer un point central.

**Paramètres :**

[entrée] model, subj/clip à base zéro ; shared_work, espace emprunté assez grand ou NULL pour allocation locale.

**Retour :**

1 si résultat produit, 0 sinon ; dans le chemin entier, 1 peut accompagner une aire nulle.

**Sorties et effets de bord :**

outx/outy facultatifs et out_area2 obligatoire pour le chemin entier ; fractional_x, fractional_y et fractional_area2, tous trois non NULL, activent le chemin subpixel en double.

caches et diagnostics globaux ; ne libère jamais shared_work.

**Fonctionnement :**

Malgré « fixed », le clipping intermédiaire utilise des doubles. Les entrées entières sont arrondies après clipping, puis l'aire et le centroïde utilisent des sommes entières ; le mode fractionnaire conserve la précision subpixel. Précondition géométrique : clip convexe pour l'intersection générale correcte par Sutherland-Hodgman. Limite de travail DEBUG_CLIP_MAX=512 : les écritures excédentaires sont tronquées, sans reconstruction générale d'un contour concave.

Appels internes directs : [fractional_face_gather](arbre_appels.html#fn-fractional_face_gather).

<a id="fn-compute_intersection_centroid_ordered_qd_fixed"></a>

### compute_intersection_centroid_ordered_qd_fixed

Définition : [overlap_raster.c](overlap_raster.c), ligne 1503.

```c
static int compute_intersection_centroid_ordered_qd_fixed(Model3D* model, int subj, int clip, int* outx, int* outy, long long* out_area2)
```

**Rôle :**

Retourner le centre de bbox de l'intersection QuickDraw de deux faces.

**Paramètres :**

[entrée] model, subj/clip

**Retour :**

1 si la région fournit un résultat, 0 sinon.

**Sorties et effets de bord :**

outx/outy et out_area2.

**Fonctionnement :**

Le nom historique est trompeur : ce n'est pas un centroïde de surface. out_area2 est l'aire de bbox, pas un double d'aire. QuickDraw doit être initialisé.

Appels internes directs : [compute_intersection_region_bbox](arbre_appels.html#fn-compute_intersection_region_bbox).

<a id="fn-compute_intersection_region_bbox"></a>

### compute_intersection_region_bbox

Définition : [overlap_raster.c](overlap_raster.c), ligne 1354.

```c
static int compute_intersection_region_bbox(Model3D* model, int subj, int clip, int* ix0, int* iy0, int* ix1, int* iy1, long long* out_area2)
```

**Rôle :**

Construire l'intersection de deux régions QuickDraw et en lire la bbox.

**Paramètres :**

[entrée] model, subj/clip

**Retour :**

Région commune détectée, ou 0 si échec.

**Sorties et effets de bord :**

ix0/iy0/ix1/iy1

out_area2.

**Préconditions :**

QuickDraw initialisé. Alloue trois régions et deux polygones. Attention : out_area2 contient le produit largeur*hauteur de la bbox, pas le double de l'aire de la région. Les tableaux sont lus sans pan ; les ressources QuickDraw sont libérées avant retour sur les chemins pris en charge.

Appels internes directs : aucun repéré.

<a id="fn-computeOrientationShading"></a>

### computeOrientationShading

Définition : [engine.c](engine.c), ligne 6467.

```c
static void computeOrientationShading(Model3D* model)
```

**Rôle :**

Calculer une couleur selon l'orientation de chaque normale observateur.

**Paramètres :**

[entrée] model, plans déjà calculés ; NULL accepté.

**Retour :**

Aucun.

**Sorties et effets de bord :**

face_shade_color, indice 0..15 arrondi depuis |c|/norme.

**Fonctionnement :**

Une normale nulle reçoit COL_FILL_DEFAULT. Ce modèle de shading ne représente pas une source lumineuse positionnée et ne tient pas compte d'une occultation.

Appels internes directs : aucun repéré.

<a id="fn-cos_deg_int"></a>

### cos_deg_int

Définition : [declare.c](declare.c), ligne 231.

```c
static inline Fixed32 cos_deg_int(int deg)
```

**Rôle :**

Lire le cosinus d'un angle entier dans la table 16.16.

**Paramètres :**

[entrée] deg, degrés éventuellement négatifs ou hors tour.

**Retour :**

Cosinus Fixed32.

**Fonctionnement :**

Ramène l'angle dans [0,359] ; aucune interpolation et aucun effet de bord.

Appels internes directs : aucun repéré.

<a id="fn-count_edge_crossings"></a>

### count_edge_crossings

Définition : [engine.c](engine.c), ligne 1323.

```c
static int count_edge_crossings(Model3D* model, int ei, int fj)
```

**Rôle :**

Compter les arêtes de ei qui traversent fj ou y possèdent une extrémité coplanaire intérieure.

**Paramètres :**

[entrée] model, ei et fj, identifiants de faces à base zéro.

**Retour :**

Nombre de traversées comptées.

**Fonctionnement :**

Aucun changement de géométrie. Calcule le plan objet de fj sur les entiers 16.16 bruts, choisit sa projection dominante puis quantifie les tests d'appartenance en coordonnées entières.

**Limites :**

seuil 0,02 appliqué à une évaluation non normalisée du plan ; pertes subpixel et allocations vx/vy non vérifiées. Ne constitue pas un test exact 3D.

Appels internes directs : [compute_face_plane](arbre_appels.html#fn-compute_face_plane), [point_in_poly_arrays_int](arbre_appels.html#fn-point_in_poly_arrays_int).

<a id="fn-createModel3D"></a>

### createModel3D

Définition : [engine.c](engine.c), ligne 68.

```c
Model3D* createModel3D(void)
```

**Rôle :**

Allouer le modèle et ses tableaux parallèles à capacité fixe.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Modèle possédé par l'appelant, ou NULL sur échec traité.

**Sorties et effets de bord :**

allocations et messages de diagnostic avec attente clavier sur échec.

**Fonctionnement :**

Réserver MAX_VERTICES sommets et MAX_FACES métriques de faces ; le buffer d'indices est dimensionné à MAX_FACES*5, et non MAX_FACES*MAX_FACE_VERTICES. vertex_count contient initialement la capacité ; loadModel3D le remplace par le nombre lu. Les géométries et métriques restent à remplir après allocation.

Appels internes directs : [keypress](arbre_appels.html#fn-keypress).

<a id="fn-destroyModel3D"></a>

### destroyModel3D

Définition : [engine.c](engine.c), ligne 425.

```c
void destroyModel3D(Model3D* model)
```

**Rôle :**

Libérer les buffers explicitement possédés par un modèle puis sa structure.

**Paramètres :**

[entrée] model, créé par createModel3D ; NULL est accepté.

**Retour :**

Aucun.

**Fonctionnement :**

Invalide le pointeur fourni sans le remettre à NULL chez l'appelant. Libère aussi le cache de projection fractionnaire associé. Les ressources statiques des méthodes de classement et les Z-buffers ont des cycles de vie distincts.

Appels internes directs : [fractional_projection_cache_release](arbre_appels.html#fn-fractional_projection_cache_release).

<a id="fn-display_model_face_ids"></a>

### display_model_face_ids

Définition : [engine.c](engine.c), ligne 6173.

```c
void display_model_face_ids(Model3D* model, ObserverParams* params, const char* filename)
```

**Rôle :**

Afficher une vue graphique annotée avec les identifiants des faces visibles.

**Paramètres :**

[entrée] model, params requis mais non recalculés ; filename non utilisé.

**Retour :**

Aucun.

**Fonctionnement :**

Sauvegarde les drapeaux et l'option filaire, dessine le modèle puis les faces annotées, attend une touche et revient au texte.

Utilise InspectionViewState pour la sauvegarde et la restauration communes des drapeaux et options ; le cycle de vie graphique reste géré par cet outil.

**Limites :**

un échec de sauvegarde annule la vue avant le dessin.

Appels internes directs : [DoText](arbre_appels.html#fn-DoText), [drawFace](arbre_appels.html#fn-drawFace), [drawPolygons](arbre_appels.html#fn-drawPolygons), [drawPolygons_jitter](arbre_appels.html#fn-drawPolygons_jitter), [inspection_view_begin](arbre_appels.html#fn-inspection_view_begin), [inspection_view_end](arbre_appels.html#fn-inspection_view_end), [keypress](arbre_appels.html#fn-keypress).

<a id="fn-DoColor"></a>

### DoColor

Définition : [engine.c](engine.c), ligne 6375.

```c
void DoColor()
```

**Rôle :**

Afficher les seize couleurs disponibles avec leurs indices.

**Paramètres :**

Aucun paramètre explicite.

**Entrées et sorties implicites :**

[entrée] options globales de mode graphique.

**Retour :**

Aucun.

**Sorties et effets de bord :**

rectangles, textes et motif QuickDraw ; la case noire est encadrée en blanc pour rester visible.

**Fonctionnement :**

Le modèle n'est ni modifié ni recalculé.

Appels internes directs : aucun repéré.

<a id="fn-dony_ray_pair_record"></a>

### dony_ray_pair_record

Définition : [dony.c](dony.c), ligne 154.

```c
static void dony_ray_pair_record(DonyRayPairs *cache, int p, int q, unsigned int generation)
```

**Rôle :**

Enregistrer une paire non orientée échangée pendant la passe courante.

**Paramètres :**

[entrée] cache, p, q et generation.

**Retour :**

Aucun.

**Sorties et effets de bord :**

count, tableaux de paires, tête de bucket et stamp mis à jour.

**Préconditions :**

l'appelant a vérifié la limite de permutations ; une entrée libre doit exister. Les collisions conservent les clés exactes.

Appels internes directs : aucun repéré.

<a id="fn-dony_ray_pair_seen"></a>

### dony_ray_pair_seen

Définition : [dony.c](dony.c), ligne 111.

```c
static int dony_ray_pair_seen(DonyRayPairs *cache, int p, int q, unsigned int generation)
```

**Rôle :**

Tester si une paire non orientée a déjà provoqué un échange dans cette passe.

**Paramètres :**

[entrée] cache ; p et q, faces à base zéro ; generation, passe de sélection.

**Retour :**

1 pour une paire exacte présente, 0 sinon.

**Fonctionnement :**

Aucun effet de bord. Les identifiants sont normalisés, puis les collisions de hash sont parcourues dans une liste chaînée dont les liens sont stockés à base un.

Appels internes directs : aucun repéré.

<a id="fn-dony_ray_pairs_reserve"></a>

### dony_ray_pairs_reserve

Définition : [dony.c](dony.c), ligne 64.

```c
static int dony_ray_pairs_reserve(DonyRayPairs *cache, int nf)
```

**Rôle :**

Réserver les listes de paires utilisées pour empêcher les cycles de ray casting.

**Paramètres :**

[entrée] cache, structure possédant ses buffers ; nf, capacité requise.

**Retour :**

1 si la capacité suffit ou a été augmentée, 0 si une allocation échoue.

**Sorties et effets de bord :**

buffers remplacés seulement après réussite des trois allocations ; count et stamps sont réinitialisés lors de l'agrandissement.

Appels internes directs : aucun repéré.

<a id="fn-DoText"></a>

### DoText

Définition : [engine.c](engine.c), ligne 8653.

```c
void DoText()
```

**Rôle :**

Revenir à l'affichage texte et envoyer le caractère d'effacement écran.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Fonctionnement :**

Appelle shgroff et émet le caractère 12 sur stdout. Ne ferme pas QuickDraw ; l'appelant gère séparément son cycle de vie.

Appels internes directs : [shgroff](arbre_appels.html#fn-shgroff).

<a id="fn-drawBlackSquareMarker"></a>

### drawBlackSquareMarker

Définition : [color.c](color.c), ligne 301.

```c
static void drawBlackSquareMarker(int index, int isFrameChooser)
```

**Rôle :**

Rendre visible une case noire avec un cadre blanc permanent.

**Paramètres :**

[entrée] index et isFrameChooser.

**Retour :**

Aucun.

**Sorties et effets de bord :**

modifie le motif de crayon et dessine un cadre sur le port actif.

Appels internes directs : [getSquareRect](arbre_appels.html#fn-getSquareRect).

<a id="fn-drawFace"></a>

### drawFace

Définition : [engine.c](engine.c), ligne 5711.

```c
void drawFace(Model3D* model, int face_id, int fillPenPat, int show_index)
```

**Rôle :**

Dessiner une face isolée pour les outils d'inspection.

**Paramètres :**

[entrée] model ; face_id à base zéro ; fillPenPat >=0 impose un remplissage, sinon emploie le mode filaire ou le motif courant ; show_index affiche l'ID.

**Retour :**

Aucun.

**Sorties et effets de bord :**

QuickDraw, motif et handle global.

**Fonctionnement :**

Vérifie visibilité, longueur et indices, puis dessine avec pan.

**Préconditions :**

QuickDraw actif.

**Limites :**

si cette fonction alloue elle-même le handle global, la réserve est dimensionnée pour quatre sommets seulement ; elle n'est pas agrandie ici.

Appels internes directs : aucun repéré.

<a id="fn-drawFaceIndex"></a>

### drawFaceIndex

Définition : [engine.c](engine.c), ligne 5848.

```c
void drawFaceIndex(Model3D* model, int face_id)
```

**Rôle :**

Écrire l'identifiant d'une face au centre de sa boîte projetée.

**Paramètres :**

[entrée] model et face_id à base zéro.

**Retour :**

Aucun.

**Fonctionnement :**

Valide visibilité et indices ; applique le pan et l'échelle écran. Conserve et restaure le motif de crayon. Ne dessine pas le contour ni la surface.

Appels internes directs : aucun repéré.

<a id="fn-drawInstructions"></a>

### drawInstructions

Définition : [color.c](color.c), ligne 338.

```c
static void drawInstructions(void)
```

**Rôle :**

Afficher les deux lignes de commandes du sélecteur.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Sorties et effets de bord :**

texte centré au bas du port graphique.

**Fonctionnement :**

La largeur de chaque ligne est mesurée par CStringWidth.

Appels internes directs : aucun repéré.

<a id="fn-drawPaletteNumber"></a>

### drawPaletteNumber

Définition : [color.c](color.c), ligne 265.

```c
static void drawPaletteNumber(int palette, int isFrameChooser)
```

**Rôle :**

Afficher le type de couleur modifié et le numéro de palette.

**Paramètres :**

[entrée] palette, numéro affiché ; isFrameChooser, contour ou remplissage.

**Retour :**

Aucun.

**Sorties et effets de bord :**

texte centré avec CStringWidth sur le port actif.

Appels internes directs : aucun repéré.

<a id="fn-drawPolygons"></a>

### drawPolygons

Définition : [engine.c](engine.c), ligne 5455.

```c
void drawPolygons(Model3D* model, int* vertex_count, int face_count, int vertex_count_total)
```

**Rôle :**

Dessiner les faces affichables dans la permutation calculée par la méthode de classement.

**Paramètres :**

[entrée] model et face_count ; vertex_count et vertex_count_total sont des paramètres historiques non utilisés dans ce corps.

**Retour :**

Aucun.

**Sorties et effets de bord :**

port QuickDraw, crayon et handle global de polygone verrouillé.

**Fonctionnement :**

Construit chaque contour avec pan, rejette les indices invalides et les boîtes hors écran, puis remplit et encadre, ou encadre seulement en mode filaire. La priorité du remplissage est shading, aléatoire, choix utilisateur, défaut.

**Préconditions :**

QuickDraw actif ; au plus MAX_FACE_VERTICES points par face. Ne calcule ni visibilité, ni plans, ni ordre, et n'efface pas l'écran.

Appels internes directs : aucun repéré.

<a id="fn-drawPolygons_jitter"></a>

### drawPolygons_jitter

Définition : [engine.c](engine.c), ligne 5642.

```c
void drawPolygons_jitter(Model3D* model, int* vertex_count, int face_count, int vertex_count_total)
```

**Rôle :**

Dessiner une vue avec un déplacement aléatoire temporaire de chaque sommet.

**Paramètres :**

[entrée] model, vertex_count, face_count et vertex_count_total transmis au dessin.

**Retour :**

Aucun.

**Fonctionnement :**

Modifie x2d/y2d pendant l'appel, puis les restaure. Les déplacements indépendants sont positifs dans [0,jitter_max], pas centrés. Sur échec de réserve, dessine sans jitter. Ne recalcule pas l'ordre ni les plans.

Appels internes directs : [drawPolygons](arbre_appels.html#fn-drawPolygons).

<a id="fn-drawSelectionBorder"></a>

### drawSelectionBorder

Définition : [color.c](color.c), ligne 229.

```c
static void drawSelectionBorder(int index, int isFrameChooser, int erase)
```

**Rôle :**

Dessiner ou effacer le cadre de sélection d'une case.

**Paramètres :**

[entrée] index, isFrameChooser ; erase, vrai pour effacer en noir.

**Retour :**

Aucun.

**Sorties et effets de bord :**

cadre de deux pixels puis taille de crayon remise à 1x1.

**Fonctionnement :**

Le motif courant reste celui utilisé pour le cadre.

Appels internes directs : [getSquareRect](arbre_appels.html#fn-getSquareRect).

<a id="fn-drawSquare"></a>

### drawSquare

Définition : [color.c](color.c), ligne 135.

```c
static void drawSquare(int index, int isFrameChooser)
```

**Rôle :**

Dessiner le fond et le contour noir d'une case.

**Paramètres :**

[entrée] index et isFrameChooser, transmis au calcul du rectangle.

**Retour :**

Aucun.

**Sorties et effets de bord :**

dessin QuickDraw et modification du motif de crayon.

**Fonctionnement :**

Les choix spéciaux utilisent un fond noir plutôt qu'un indice de couleur.

Appels internes directs : [getSquareRect](arbre_appels.html#fn-getSquareRect).

<a id="fn-drawSquareLabel"></a>

### drawSquareLabel

Définition : [color.c](color.c), ligne 181.

```c
static void drawSquareLabel(int index, int isFrameChooser)
```

**Rôle :**

Écrire le libellé d'une case sous son rectangle.

**Paramètres :**

[entrée] index et isFrameChooser.

**Retour :**

Aucun

**Sorties et effets de bord :**

texte sur le port actif.

**Fonctionnement :**

Les indices 0..15 sont numériques ; les choix spéciaux gardent leurs libellés de l'interface. Le centrage emploie CHAR_WIDTH, pas la mesure QuickDraw.

Appels internes directs : [getSquareRect](arbre_appels.html#fn-getSquareRect).

<a id="fn-dump_compare_results_to_file"></a>

### dump_compare_results_to_file

Définition : [engine.c](engine.c), ligne 8082.

```c
static void dump_compare_results_to_file(const CompareFacesResult *out, int f1, int f2)
```

**Rôle :**

Ajouter le diagnostic textuel d'une paire dans son fichier dédié.

**Paramètres :**

[entrée] out, résultat rempli ; f1/f2 pour le nom fNVSfM.txt.

**Retour :**

Aucun.

**Fonctionnement :**

Ouvre en ajout ; NULL ou échec d'ouverture est ignoré. Ne recalcule aucun test.

Appels internes directs : aucun repéré.

<a id="fn-dumpFace2DCoordinates"></a>

### dumpFace2DCoordinates

Définition : [engine.c](engine.c), ligne 8461.

```c
static void dumpFace2DCoordinates(Model3D* model, const char* txt_filename)
```

**Rôle :**

Exporter les coordonnées écran de chaque sommet de chaque face.

**Paramètres :**

[entrée] model et txt_filename.

**Retour :**

Aucun.

**Fonctionnement :**

Écrase le fichier ; écrit IDs de faces base zéro et sommets base un, sans pan appliqué au dessin. Ignore les indices de sommets invalides ; ne recalcule pas la projection.

Appels internes directs : aucun repéré.

<a id="fn-dumpFaceEquationsCSV"></a>

### dumpFaceEquationsCSV

Définition : [engine.c](engine.c), ligne 8337.

```c
void dumpFaceEquationsCSV(Model3D* model, const char* csv_filename, int alt_format)
```

**Rôle :**

Exporter les plans, profondeurs et coordonnées observateur des faces.

**Paramètres :**

[entrée] model et csv_filename ; alt_format vrai choisit séparateur point-virgule et virgule décimale, faux choisit virgule et point décimal.

**Retour :**

Aucun.

**Fonctionnement :**

Écrase le fichier cible ; n'effectue pas de recalcul de vue. Les IDs de face sont à base zéro et les indices de sommets restent au format OBJ.

Appels internes directs : aucun repéré.

<a id="fn-dumpSortedFaceIndices"></a>

### dumpSortedFaceIndices

Définition : [engine.c](engine.c), ligne 8507.

```c
static void dumpSortedFaceIndices(Model3D* model, const char* txt_filename)
```

**Rôle :**

Exporter la permutation courante dans l'ordre de dessin de la méthode de classement.

**Paramètres :**

[entrée] model et txt_filename.

**Retour :**

Aucun.

**Fonctionnement :**

Écrase le fichier ; une ligne par identifiant de face à base zéro, y compris les faces actuellement masquées.

Appels internes directs : aucun repéré.

<a id="fn-edge_crosses_interior"></a>

### edge_crosses_interior

Définition : [engine.c](engine.c), ligne 1098.

```c
static int edge_crosses_interior(Model3D* model, int ei, int fj)
```

**Rôle :**

Détecter une arête de ei qui traverse strictement l'intérieur de fj.

**Paramètres :**

[entrée] model, ei et fj, identifiants de faces à base zéro.

**Retour :**

1 au premier franchissement intérieur, 0 sinon.

**Fonctionnement :**

Aucun changement de géométrie. Calcule le plan objet de fj sur les entiers 16.16 bruts, choisit sa projection dominante puis quantifie les tests d'appartenance en coordonnées entières.

**Limites :**

seuil 0,02 appliqué à une évaluation non normalisée du plan ; pertes subpixel et allocations vx/vy non vérifiées. Ne constitue pas un test exact 3D.

Appels internes directs : [compute_face_plane](arbre_appels.html#fn-compute_face_plane), [strict_inside](arbre_appels.html#fn-strict_inside).

<a id="fn-encodePaletteColor"></a>

### encodePaletteColor

Définition : [declare.c](declare.c), ligne 557.

```c
static GS_Color encodePaletteColor(int r, int g, int b)
```

**Rôle :**

Encoder une couleur RGB dans le mot $0RGB du matériel SHGR.

**Paramètres :**

[entrée] r, g et b, composantes entières bornées ici à [0,15].

**Retour :**

GS_Color 12 bits.

**Fonctionnement :**

Aucun accès matériel ni effet de bord.

Appels internes directs : aucun repéré.

<a id="fn-evaluate_pair_tests"></a>

### evaluate_pair_tests

Définition : [engine.c](engine.c), ligne 4409.

```c
static void evaluate_pair_tests(Model3D* model, int f1, int f2, int out[7])
```

**Rôle :**

Exposer séparément les sept tests d'ordre d'une paire pour le diagnostic.

**Paramètres :**

[entrée] model et f1/f2 à base zéro.

**Retour :**

Aucun.

**Sorties et effets de bord :**

out[0..6]

**Fonctionnement :**

Ordre des cases : profondeur, séparation X, séparation Y, puis quatre tests de côté des plans. -1 valide le sens courant, +1 indique l'inversion, 0 reste indéterminé. X/Y disjoints écrivent -1, ce qui n'est pas une preuve d'occlusion. Les quatre tests de plans utilisent geometric_face_relation et sa tolérance adaptative, comme la décision de S/D. Chaque sens ne valide qu'une des deux hypothèses opposées. Les deux sens sont exposés séparément, même s'ils se contredisent ; la décision globale conserve la priorité du premier sens concluant. Peut remplir les caches géométriques ; ne modifie pas la permutation.

Appels internes directs : [geometric_face_relation](arbre_appels.html#fn-geometric_face_relation).

<a id="fn-FaceDepth_CentroidCoordinate"></a>

### FaceDepth_CentroidCoordinate

Définition : [face_depth_asm.c](face_depth_asm.c), ligne 791.

```c
static Fixed32 FaceDepth_CentroidCoordinate(Fixed64 sum,int count)
```

**Rôle :**

Diviser une somme de coordonnées par le nombre de sommets valides.

**Paramètres :**

[entrée] sum, somme 64 bits ; count, diviseur strictement positif.

**Retour :**

Moyenne Fixed32 tronquée vers zéro.

**Fonctionnement :**

Aucun effet de bord. Réduit à une division 32 bits si possible ; pour count=4, ajoute 3 aux sommes négatives avant le décalage de deux bits afin de conserver la troncature C.

Appels internes directs : aucun repéré.

<a id="fn-FaceDepth_ScanFace"></a>

### FaceDepth_ScanFace

Définition : [face_depth_asm.c](face_depth_asm.c), ligne 758.

```c
static void FaceDepth_ScanFace(FaceArrays3D *faces,int offset,int n)
```

**Rôle :**

Calculer les métriques d'une liste d'indices de face.

**Paramètres :**

[entrée] faces ; offset dans vertex_indices_buffer ; n, nombre d'indices.

**Retour :**

Aucun.

**Sorties et effets de bord :**

face_depth_result ; utilise les coordonnées préparées par FaceDepth_ScanSetup.

**Fonctionnement :**

L'appelant doit garantir la validité de la liste.

Appels internes directs : [FaceDepth_ScanKernel](arbre_appels.html#fn-FaceDepth_ScanKernel).

<a id="fn-FaceDepth_ScanKernel"></a>

### FaceDepth_ScanKernel

Définition : [face_depth_asm.c](face_depth_asm.c), ligne 77.

```c
asm void FaceDepth_ScanKernel(void)
```

**Rôle :**

Parcourir une face pour calculer profondeurs, bornes, centroïde et normale.

**Paramètres :**

Aucun paramètre explicite.

**Entrées et sorties implicites :**

[entrée] face_depth_inputs, pointeurs far puis longueur sur 16 bits.

**Retour :**

Aucun.

**Sorties et effets de bord :**

face_depth_result

**Fonctionnement :**

Noyau et multiplicateur non réentrants.

Pointeurs ORCA/C sur quatre octets, accès globaux longs.

La somme de profondeur conserve son rebouclage 32 bits ; les sommes du centroïde sont 64 bits. Les normales ne sont décalées qu'après fermeture du contour.

**Registres et indicateurs :**

Préserve P, A, X, Y et D.

**Organisation mémoire :**

Cadre D de 148 octets : indices/xo/yo/zo/x2d/y2d à 1/5/9/13/17/21 ; longueur à 25 ; offset/valides/affichage/précédent à 27/29/31/33 ; zmin/zmax/somme32 à 35/39/43 ; bbox à 47..53 ; sommes xyz 64 bits à 55/63/71 ; Newell brut à 79/87/95 ; xyz précédent/premier/courant à 103..113/115..125/127..137 ; temporaires à 139..145.

Appels internes directs : [GeoV3_Mul32x32](arbre_appels.html#fn-GeoV3_Mul32x32).

<a id="fn-FaceDepth_ScanSetup"></a>

### FaceDepth_ScanSetup

Définition : [face_depth_asm.c](face_depth_asm.c), ligne 722.

```c
static void FaceDepth_ScanSetup(VertexArrays3D *vtx)
```

**Rôle :**

Préparer les pointeurs de coordonnées pour un parcours complet des faces.

**Paramètres :**

[entrée] vtx, tableaux observateur et projetés déjà calculés.

**Retour :**

Aucun.

**Sorties et effets de bord :**

paquet global ; les pointeurs doivent rester stables jusqu'à la fin du parcours.

Appels internes directs : aucun repéré.

<a id="fn-faces_share_vertex_or_edge"></a>

### faces_share_vertex_or_edge

Définition : [engine.c](engine.c), ligne 1056.

```c
static int faces_share_vertex_or_edge(FaceArrays3D* faces, int i, int j)
```

**Rôle :**

Détecter un identifiant de sommet commun à deux faces.

**Paramètres :**

[entrée] faces, i et j à base zéro.

**Retour :**

1 au premier identifiant commun, 0 sinon.

**Fonctionnement :**

Aucun effet de bord. Le test ne requiert pas une arête entière commune et ne détecte pas les sommets géométriquement égaux portant des indices distincts.

Appels internes directs : aucun repéré.

<a id="fn-faces_vertices_equal"></a>

### faces_vertices_equal

Définition : [overlap_raster.c](overlap_raster.c), ligne 594.

```c
static int faces_vertices_equal(FaceArrays3D* faces, VertexArrays3D* vtx, int f1, int f2)
```

**Rôle :**

Comparer deux contours projetés modulo rotation cyclique et inversion.

**Paramètres :**

[entrée] faces/vtx et f1/f2.

**Retour :**

1 si les suites de coordonnées entières sont égales dans un des deux sens, 0 sinon.

**Fonctionnement :**

Ne compare pas la profondeur, ni l'identité OBJ des sommets ; une égalité projetée n'est pas une égalité 3D.

Appels internes directs : aucun repéré.

<a id="fn-fixed_mul32_fast"></a>

### fixed_mul32_fast

Définition : [engine.c](engine.c), ligne 2643.

```c
static Fixed32 fixed_mul32_fast(Fixed32 a,Fixed32 b)
```

**Rôle :**

Multiplier deux valeurs 16.16 et conserver les bits 16..47 du produit.

**Paramètres :**

[entrée] a/b en Fixed32.

**Retour :**

Produit Fixed32, sans saturation ni arrondi.

**Fonctionnement :**

Chemin C avec GEOV3_DISABLE_ASM_MUL ; sinon utilise le paquet assembleur partagé. L'extraction du mot central reproduit la réduction à 32 bits après décalage.

Appels internes directs : [GeoV3_Mul32x32SetInputs](arbre_appels.html#fn-GeoV3_Mul32x32SetInputs).

<a id="fn-fixed_product32_fast"></a>

### fixed_product32_fast

Définition : [engine.c](engine.c), ligne 2678.

```c
static Fixed64 fixed_product32_fast(Fixed32 a,Fixed32 b)
```

**Rôle :**

Calculer le produit signé brut 32x32 vers 64 bits.

**Paramètres :**

[entrée] a/b.

**Retour :**

Produit Fixed64, sans décalage de virgule fixe.

**Fonctionnement :**

Utilise le C si GEOV3_DISABLE_ASM_MUL est défini, sinon le paquet assembleur non réentrant.

Appels internes directs : [GeoV3_Mul32x32SetInputs](arbre_appels.html#fn-GeoV3_Mul32x32SetInputs).

<a id="fn-fixed_product64x32_fast"></a>

### fixed_product64x32_fast

Définition : [engine.c](engine.c), ligne 2712.

```c
static Fixed64 fixed_product64x32_fast(Fixed64 a,Fixed32 b)
```

**Rôle :**

Calculer un produit brut coefficient 64 bits par coordonnée 32 bits.

**Paramètres :**

[entrée] a/b.

**Retour :**

Produit Fixed64, sans saturation ; les bornes restent à garantir par l'appelant.

**Fonctionnement :**

Si a est exactement représentable en 32 bits, utilise le multiplicateur rapide ; sinon conserve la multiplication C large.

Appels internes directs : [fixed_product32_fast](arbre_appels.html#fn-fixed_product32_fast).

<a id="fn-FIXED_ROUND_TO_INT"></a>

### FIXED_ROUND_TO_INT

Définition : [declare.c](declare.c), ligne 84.

```c
static inline int FIXED_ROUND_TO_INT(Fixed32 x)
```

**Rôle :**

Convertir un Fixed32 en entier en appliquant le biais de demi-unité du moteur.

**Paramètres :**

[entrée] x, valeur signée 16.16.

**Retour :**

(x+32768)>>16 si x >= 0, (x-32768)>>16 sinon.

**Fonctionnement :**

Aucun effet de bord. Le cas négatif suit le décalage signé du compilateur ; ne pas l'assimiler à un arrondi symétrique au plus proche ni à une troncature vers zéro.

Appels internes directs : aucun repéré.

<a id="fn-fractional_face_gather"></a>

### fractional_face_gather

Définition : [overlap_raster.c](overlap_raster.c), ligne 211.

```c
static int fractional_face_gather(Model3D *model,int face,int n,double scale, double *px,double *py, double **edge_x,double **edge_y)
```

**Rôle :**

Préparer le contour subpixel antihoraire d'une face et ses vecteurs d'arêtes.

**Paramètres :**

[entrée] model, face, n et scale

**Retour :**

1 si contour exploitable, 0 si sommet derrière l'observateur ou aire quasi nulle.

**Sorties et effets de bord :**

px/py de capacité n ; edge_x/edge_y reçoivent des pointeurs empruntés au cache ou NULL.

**Préconditions :**

indices valides, n>=3 et scale cohérent avec l'échelle globale qui sert de clé. Au plus 16 sommets sont mémorisés ; un échec de cache n'empêche pas de retourner le contour dans les buffers fournis.

Appels internes directs : [fractional_projection_cache_invalidate](arbre_appels.html#fn-fractional_projection_cache_invalidate), [fractional_projection_cached](arbre_appels.html#fn-fractional_projection_cached), [ordering_winding_hit](arbre_appels.html#fn-ordering_winding_hit), [ordering_winding_slot](arbre_appels.html#fn-ordering_winding_slot), [ordering_winding_store](arbre_appels.html#fn-ordering_winding_store).

<a id="fn-fractional_projection_cache_invalidate"></a>

### fractional_projection_cache_invalidate

Définition : [overlap_raster.c](overlap_raster.c), ligne 59.

```c
static void fractional_projection_cache_invalidate(Model3D *model)
```

**Rôle :**

Invalider les rapports de projection et les caches d'ordre associés.

**Paramètres :**

[entrée] model, nouveau propriétaire.

**Retour :**

Aucun.

**Fonctionnement :**

Incrémente la génération en conservant les allocations ; au rebouclage, efface stamps et échecs mémorisés.

Appels internes directs : [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate).

<a id="fn-fractional_projection_cache_release"></a>

### fractional_projection_cache_release

Définition : [overlap_raster.c](overlap_raster.c), ligne 97.

```c
static void fractional_projection_cache_release(Model3D *model)
```

**Rôle :**

Libérer les caches d'ordre et de projection appartenant à model.

**Paramètres :**

[entrée] model

**Retour :**

Aucun.

**Fonctionnement :**

Le cache fractionnaire est libéré uniquement si son propriétaire correspond ; tous les pointeurs obtenus deviennent invalides.

Appels internes directs : [ordering_cache_release](arbre_appels.html#fn-ordering_cache_release).

<a id="fn-fractional_projection_cached"></a>

### fractional_projection_cached

Définition : [overlap_raster.c](overlap_raster.c), ligne 141.

```c
static void fractional_projection_cached(Model3D *model, int v, double scale, double *outx, double *outy)
```

**Rôle :**

Projeter un sommet en double en mémorisant xo/zo et yo/zo indépendamment du zoom.

**Paramètres :**

[entrée] model ; v, indice base zéro valide ; scale, pixels par unité projetée.

**Retour :**

Aucun.

**Sorties et effets de bord :**

*outx/*outy, pixels fractionnaires sans pan

**Préconditions :**

zo[v] non nul. Réserve des blocs de 256 sommets à la demande ; sans cache disponible, calcule directement les rapports. Ne pas confondre la borne du cache MAX_VERTICES avec une validation générale de v.

Appels internes directs : [fractional_projection_cache_invalidate](arbre_appels.html#fn-fractional_projection_cache_invalidate).

<a id="fn-frameInconclusivePairs"></a>

### frameInconclusivePairs

Définition : [engine.c](engine.c), ligne 5918.

```c
void frameInconclusivePairs(Model3D* model)
```

**Rôle :**

Surligner les faces des paires inconclusives conservées par les méthodes de classement.

**Paramètres :**

[entrée] model et buffer global inconclusive_pairs.

**Retour :**

Aucun.

**Sorties et effets de bord :**

handle global et crayon QuickDraw ; les faces peuvent être dessinées plusieurs fois.

**Fonctionnement :**

Malgré son nom, remplit les faces en vert clair et les encadre en jaune.

Les tests de boîte de ce diagnostic ne traitent pas le pan de façon identique au dessin des points.

Appels internes directs : aucun repéré.

<a id="fn-generate_random_colors"></a>

### generate_random_colors

Définition : [engine.c](engine.c), ligne 6029.

```c
void generate_random_colors(int face_count)
```

**Rôle :**

Générer deux tableaux de couleurs non noires pour les faces.

**Paramètres :**

[entrée] face_count.

**Retour :**

Aucun.

**Sorties et effets de bord :**

random_fill_colors, random_frame_colors et leur capacité ; valeurs entre 1 et 15 incluses.

**Fonctionnement :**

Consomme rand sans initialiser sa graine ici.

**Limites :**

les retours de realloc ne sont pas vérifiés avant utilisation ; cette révision conserve ce comportement.

Appels internes directs : aucun repéré.

<a id="fn-geo_face_order"></a>

### geo_face_order

Définition : [engine.c](engine.c), ligne 7402.

```c
static int geo_face_order(Model3D* model, int f1, int f2)
```

**Rôle :**

Fournir une préférence d'ordre en utilisant le noyau géométrique optimisé.

**Paramètres :**

[entrée] model : modèle aux métriques observateur à jour ; NULL ignoré. [entrée] f1/f2 : identifiants valides des deux faces.

**Retour :**

-1 si f1 doit précéder f2, +1 si f1 doit suivre f2 ; 0 si aucun des deux tests orientés ne conclut, ou si model est NULL.

**Sorties et effets de bord :**

Peut préparer les plans et remplir les caches du noyau géométrique. Ne modifie ni les faces ni leur permutation et ne vide pas les caches.

**Fonctionnement :**

Teste le plan de f1 contre f2 avec geometric_face_relation. Si ce test ne conclut pas, teste le sens inverse et oppose son résultat. Le premier verdict non nul est prioritaire, comme dans pair_order_relation. Utilise la tolérance proportionnelle à la distance de l'observateur et les chemins C/assembleur du noyau, sans ajout de test de profondeur, de boîte, de chevauchement ou de rayon.

Appels internes directs : [geometric_face_relation](arbre_appels.html#fn-geometric_face_relation).

<a id="fn-geometric_face_relation"></a>

### geometric_face_relation

Définition : [engine.c](engine.c), ligne 2801.

```c
static int geometric_face_relation(Model3D* model, int f1, int f2)
```

**Rôle :**

Classer f1 par rapport à f2 en utilisant le plan de f1 dans la vue observateur.

**Paramètres :**

[entrée] model et f1/f2 à base zéro ; model=NULL demande une remise à zéro.

**Retour :**

-1 si f1 est avant f2, +1 après, 0 si le plan ne tranche pas.

**Sorties et effets de bord :**

caches de plans, de sommets et de relations ; paquets assembleur partagés.

**Fonctionnement :**

Compare les sommets de f2 au côté de l'observateur en ignorant la bande epsilon, proportionnelle à current_observer_distance. Des côtés mixtes rendent 0.

Le chemin assembleur nécessite a/b/c 32 bits et les bornes préparées de d. Les caches denses et de hash vérifient propriétaire, génération et époque. La convention -1 avant est distincte de PairCache et du signe du ray casting.

Appels internes directs : [GeoRel_ClassifyPrepared](arbre_appels.html#fn-GeoRel_ClassifyPrepared), [GeoV3_MulFixed64Fixed32Shift](arbre_appels.html#fn-GeoV3_MulFixed64Fixed32Shift), [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate), [ordering_fixed_plane](arbre_appels.html#fn-ordering_fixed_plane), [ordering_fixed_plane_reset](arbre_appels.html#fn-ordering_fixed_plane_reset), [ordering_geometry_slot](arbre_appels.html#fn-ordering_geometry_slot).

<a id="fn-GeoRel_Classify32"></a>

### GeoRel_Classify32

Définition : [geo_relation_asm.c](geo_relation_asm.c), ligne 522.

```c
static int GeoRel_Classify32(Model3D *model, int face, Fixed32 a, Fixed32 b, Fixed32 c, Fixed64 d, Fixed64 epsilon, int obs, unsigned int *vertex_sides)
```

**Rôle :**

Construire le paquet complet et appeler le noyau de classement.

**Paramètres :**

[entrée] model, face candidate ; a/b/c en 16.16 sur 32 bits ; d et epsilon sur 64 bits ; obs, côté de l'observateur ; vertex_sides, cache de 64 mots.

**Retour :**

-1 avant, +1 après, 0 indéterminé selon le plan fourni.

**Sorties et effets de bord :**

paquet global, cache de côtés et résultat ; invalide le paquet préparé.

**Fonctionnement :**

L'appelant garantit les bornes arithmétiques requises par le noyau.

Appels internes directs : [GeoRel_Classify32Kernel](arbre_appels.html#fn-GeoRel_Classify32Kernel).

<a id="fn-GeoRel_Classify32Kernel"></a>

### GeoRel_Classify32Kernel

Définition : [geo_relation_asm.c](geo_relation_asm.c), ligne 70.

```c
asm void GeoRel_Classify32Kernel(void)
```

**Rôle :**

Classer une face candidate par rapport au plan et au côté de l'observateur.

**Paramètres :**

Aucun paramètre explicite.

**Entrées et sorties implicites :**

[entrée] geo_rel_words

**Retour :**

Aucun.

**Sorties et effets de bord :**

geo_rel_result, -1 avant, +1 après, 0 indéterminé.

**Registres et indicateurs :**

Préserve P, A, X, Y et D ; accès globaux longs indépendants de DBR.

**Organisation mémoire :**

Cadre D, 80 octets : indices à 1 ; xo/yo/zo à 5/9/13 ; reste/obs à 17/19 ; all_same/all_opp à 21/23 ; acc à 25..31 ; epsilon à 33..39 ; -epsilon à 41..47 ; d à 49..55 ; a/b/c à 57..67 ; offsets à 69/71 ; cache far à 73 ; ID/slot à 77/79.

**Préconditions :**

coefficients a/b/c signés 32 bits, |d| <= 2^62, epsilon >= 0, indices et cache de côtés valides. Décale chaque produit de 16 bits avant somme.

Cache partagé valide seulement pour le plan, la tolérance et la vue préparés.

Appels internes directs : [GeoV3_Mul32x32](arbre_appels.html#fn-GeoV3_Mul32x32).

<a id="fn-GeoRel_ClassifyPrepared"></a>

### GeoRel_ClassifyPrepared

Définition : [geo_relation_asm.c](geo_relation_asm.c), ligne 580.

```c
static int GeoRel_ClassifyPrepared(Model3D *model,int face,OrderFixedPlane *plane)
```

**Rôle :**

Classer une face en réutilisant le paquet d'un plan préparé.

**Paramètres :**

[entrée] model, face candidate et plane retourné par ordering_fixed_plane.

**Retour :**

Relation -1/0/+1 du noyau.

**Sorties et effets de bord :**

paquet et cache de côtés du plan.

**Fonctionnement :**

Reconstruit le paquet complet au changement de plan ; sinon remplace uniquement la liste d'indices et sa longueur. Non réentrant : les globals sont partagés.

Appels internes directs : [GeoRel_Classify32](arbre_appels.html#fn-GeoRel_Classify32), [GeoRel_Classify32Kernel](arbre_appels.html#fn-GeoRel_Classify32Kernel).

<a id="fn-GeoV3_FindCandidate"></a>

### GeoV3_FindCandidate

Définition : [dony_scan_asm.c](dony_scan_asm.c), ligne 305.

```c
static int GeoV3_FindCandidate(FaceArrays3D *faces, int p, int start)
```

**Rôle :**

Rechercher une position candidate en réutilisant les bornes de la face P.

**Paramètres :**

[entrée] faces, métriques immuables pendant le tri ; p, face à base zéro ; start, première position à examiner dans sorted.

**Retour :**

Première position non rejetée, ou nf.

**Sorties et effets de bord :**

paquet et résultat globaux.

**Fonctionnement :**

Recharge les bornes uniquement si p change ; le mot haut de zmin est biaisé par XOR $8000 pour permettre une comparaison signée à partir de CMP.

Appels internes directs : [GeoV3_ScanCandidates](arbre_appels.html#fn-GeoV3_ScanCandidates).

<a id="fn-GeoV3_Mul32x32"></a>

### GeoV3_Mul32x32

Définition : [engine.c](engine.c), ligne 2258.

```c
asm void GeoV3_Mul32x32(void)
```

**Rôle :**

Multiplier exactement deux entiers signés 32 bits et publier le produit 64 bits brut.

**Paramètres :**

Aucun paramètre explicite.

**Entrées et sorties implicites :**

[entrée] geo_mul_inputs, a puis b en quatre mots little-endian

**Retour :**

Aucun.

**Sorties et effets de bord :**

geo_mul_p, quatre mots de poids faible à fort.

**Fonctionnement :**

Aucun paramètre de pile C. Deux entrées 16.16 donnent un produit 32.32, sans arrondi ni saturation.

Calcul binaire et accès globaux longs. Non réentrant : les paquets globaux sont partagés.

Algorithme par magnitudes, addition et rotation combinées : chemins 8/16/32 itérations, raccourci pour zéro et un, factorisation des mots bas nuls.

**Registres et indicateurs :**

Préserve P, A, X et D ; Y et DBR ne sont pas modifiés.

**Organisation mémoire :**

Cadre D de 24 octets : signe à 1 ; échange à 3 ; M à 5/7 ; compte à 9 ; facteur 65536 à 11 ; Q à 13/15 ; produit à 17/19/21/23.

Appels internes directs : aucun repéré.

<a id="fn-GeoV3_Mul32x32SetInputs"></a>

### GeoV3_Mul32x32SetInputs

Définition : [engine.c](engine.c), ligne 2612.

```c
static void GeoV3_Mul32x32SetInputs(Fixed32 a, Fixed32 b)
```

**Rôle :**

Préparer les deux opérandes natifs et exécuter le multiplicateur assembleur.

**Paramètres :**

[entrée] a/b, entiers signés 32 bits.

**Retour :**

Aucun ; produit brut dans geo_mul_p. memcpy conserve les bits et l'ordre little-endian ORCA/C.

**Fonctionnement :**

Paquets partagés non réentrants.

Appels internes directs : [GeoV3_Mul32x32](arbre_appels.html#fn-GeoV3_Mul32x32).

<a id="fn-GeoV3_MulFixed64Fixed32Shift"></a>

### GeoV3_MulFixed64Fixed32Shift

Définition : [engine.c](engine.c), ligne 2741.

```c
static Fixed64 GeoV3_MulFixed64Fixed32Shift(Fixed64 coefficient, Fixed32 coordinate)
```

**Rôle :**

Évaluer une contribution de plan : (coefficient * coordinate) >> 16.

**Paramètres :**

[entrée] coefficient 64 bits et coordinate 16.16.

**Retour :**

Terme Fixed64.

**Fonctionnement :**

Choisit le noyau 32x32 uniquement si le coefficient y tient exactement ; les coefficients plus larges utilisent C. Chaque terme est décalé séparément.

Appels internes directs : [GeoV3_Mul32x32SetInputs](arbre_appels.html#fn-GeoV3_Mul32x32SetInputs).

<a id="fn-geov3_run"></a>

### geov3_run

Définition : [dony.c](dony.c), ligne 251.

```c
static int geov3_run(Model3D* model, int face_count, int preserve_order, GeoV3ExchangeCallback on_exchange, void *context)
```

**Rôle :**

Partager le parcours GEO V3 entre le classement et le repair interactif.

**Paramètres :**

[entrée] model et face_count : modèle aux métriques de vue à jour. [entrée] preserve_order : 0 initialise et trie par zmax ; 1 part de la permutation courante, sans tri initial silencieux. [entrée] on_exchange/context : confirmation facultative et son contexte. Le callback reçoit les deux faces, leurs positions avant échange et la raison ; -1 arrête, 0 ignore cet échange, 1 l'applique. NULL applique tout.

**Retour :**

Nombre d'échanges appliqués, y compris avant un arrêt demandé ; -1 en cas d'échec d'allocation du repair. Le classement conserve son erreur fatale historique d'allocation.

**Fonctionnement :**

Réutilise sans duplication le filtrage C/assembleur, les deux tests de plans, les stamps anti-cycle, le chevauchement et le rayon hiérarchique GEO V3. Le callback intervient avant chaque échange, sans modifier la géométrie ni la permutation. Un refus avance au candidat suivant. Les limites de permutations et les cas non résolus restent ceux de GEO V3. Les buffers persistants sont partagés : ce parcours n'est pas réentrant.

Appels internes directs : [dony_ray_pair_record](arbre_appels.html#fn-dony_ray_pair_record), [dony_ray_pair_seen](arbre_appels.html#fn-dony_ray_pair_seen), [dony_ray_pairs_reserve](arbre_appels.html#fn-dony_ray_pairs_reserve), [geometric_face_relation](arbre_appels.html#fn-geometric_face_relation), [GeoV3_FindCandidate](arbre_appels.html#fn-GeoV3_FindCandidate), [GeoV3_ScanSetup](arbre_appels.html#fn-GeoV3_ScanSetup), [projected_polygons_overlap_cached](arbre_appels.html#fn-projected_polygons_overlap_cached), [ray_cast_hierarchical](arbre_appels.html#fn-ray_cast_hierarchical), [sort_by_zmax](arbre_appels.html#fn-sort_by_zmax).

Référence indirecte : `repair_geov3_confirm` via `qsort`.

<a id="fn-GeoV3_ScanCandidates"></a>

### GeoV3_ScanCandidates

Définition : [dony_scan_asm.c](dony_scan_asm.c), ligne 69.

```c
asm void GeoV3_ScanCandidates(void)
```

**Rôle :**

Trouver le prochain candidat non rejeté par les tests rapides de la méthode de classement.

**Paramètres :**

Aucun paramètre explicite.

**Entrées et sorties implicites :**

Entrée : geo_scan_words, préparé par les deux wrappers C.

**Retour :**

Aucun.

**Sorties et effets de bord :**

geo_scan_result, position dans la liste triée, ou nf à épuisement.

**Fonctionnement :**

Ne calcule aucune relation de plan et ne modifie pas l'ordre des faces.

Calculs binaires 16 bits.

Les accès globaux > ignorent DBR.

**Registres et indicateurs :**

Préserve P, A, Y et D ; X et DBR ne sont pas utilisés.

**Organisation mémoire :**

Cadre D de 44 octets : pointeurs far tri/zmax/minx/maxx/miny/maxy à 1/5/9/13/17/21 ; zmin de P à 25/27, bornes de P à 29/31/33/35, position/fin à 37/39, temporaire à 41 et offset Q*2 à 43.

Appels internes directs : aucun repéré.

<a id="fn-GeoV3_ScanSetup"></a>

### GeoV3_ScanSetup

Définition : [dony_scan_asm.c](dony_scan_asm.c), ligne 265.

```c
static void GeoV3_ScanSetup(FaceArrays3D *faces, int *sorted, int nf)
```

**Rôle :**

Préparer les pointeurs invariants du filtrage pour un appel à la méthode de classement.

**Paramètres :**

[entrée] faces, tableaux de métriques ; sorted, indices de faces à base zéro ; nf, nombre de positions valides.

**Retour :**

Aucun.

**Sorties et effets de bord :**

paquet global, limite de recherche et invalidation du P mémorisé.

**Préconditions :**

pointeurs ORCA/C de quatre octets et tableaux stables pendant le tri.

Appels internes directs : aucun repéré.

<a id="fn-getFaceFillColor"></a>

### getFaceFillColor

Définition : [zbuffer_scanline.c](zbuffer_scanline.c), ligne 48.

```c
int getFaceFillColor(int face_id)
```

**Rôle :**

Résoudre la couleur de remplissage d'une face.

**Paramètres :**

[entrée] face_id valide base zéro

**Retour :**

Indice selon priorité shading, aléatoire disponible, choix utilisateur, défaut.

**Fonctionnement :**

Aucun dessin ni modification de l'état. Les sélections spéciales doivent être cohérentes avec les buffers.

Appels internes directs : aucun repéré.

<a id="fn-getFaceFrameColor"></a>

### getFaceFrameColor

Définition : [zbuffer_scanline.c](zbuffer_scanline.c), ligne 83.

```c
int getFaceFrameColor(int face_id, int fill_color)
```

**Rôle :**

Résoudre la couleur de contour d'une face.

**Paramètres :**

[entrée] face_id valide et fill_color effectif

**Retour :**

Remplissage si choix 17, couleur aléatoire disponible si 16, choix explicite ou COL_FRAME.

**Fonctionnement :**

Aucun dessin ; ne recalcule pas le remplissage.

Appels internes directs : aucun repéré.

<a id="fn-getkeypress"></a>

### getkeypress

Définition : [asm.h](asm.h), ligne 85.

```c
asm char getkeypress ()
```

**Rôle :**

Attendre une frappe et retourner son code sur sept bits.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Caractère dans A, sans le bit de strobe.

**Sorties et effets de bord :**

acquitte $00C010 ; attente active.

**Fonctionnement :**

Accumulateur attendu sur 16 bits.

Appels internes directs : aucun repéré.

<a id="fn-getkeypress_openA"></a>

### getkeypress_openA

Définition : [asm.h](asm.h), ligne 126.

```c
asm int getkeypress_openA()
```

**Rôle :**

Lire une frappe en conservant l'état de la touche Open-Apple.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Code clavier dans l'octet bas, Open-Apple dans le bit 8.

**Sorties et effets de bord :**

strobe acquitté ; A est rendu sur 16 bits, P n'est pas restauré.

**Algorithme :**

accès matériels en 8 bits, sauvegarde du caractère sur la pile, transfert du bit 7 de $00C061 vers B, puis reconstitution de B:A.

Appels internes directs : aucun repéré.

<a id="fn-getObserverParams"></a>

### getObserverParams

Définition : [engine.c](engine.c), ligne 5340.

```c
void getObserverParams(ObserverParams* params, Model3D* model)
```

**Rôle :**

Saisir les angles et la distance de vue en conservant les valeurs proposées.

**Paramètres :**

[entrée-sortie] params

[entrée-sortie] model pour les métadonnées d'ajustement, éventuellement NULL.

**Retour :**

Aucun.

**Sorties et effets de bord :**

entrées-sorties texte.

**Fonctionnement :**

Les angles sont lus en degrés entiers, la distance par atof puis conversion 16.16 ; aucune validation numérique forte. Une distance vide ou un échec de lecture réutilise les suggestions si disponibles, sinon 30. Met à jour current_observer_distance ; la projection globale n'est pas directement réaffectée dans cette fonction.

Appels internes directs : aucun repéré.

<a id="fn-getSquareRect"></a>

### getSquareRect

Définition : [color.c](color.c), ligne 84.

```c
static void getSquareRect(int index, int isFrameChooser, ChooserRect *r)
```

**Rôle :**

Calculer le rectangle en pixels d'une case du sélecteur.

**Paramètres :**

[entrée] index, 0..15 ou choix spécial ; isFrameChooser, présence du choix même couleur que le remplissage.

**Retour :**

Aucun.

**Sorties et effets de bord :**

*r

**Fonctionnement :**

Centre une grille 8x2 et une ligne séparée de une ou deux cases spéciales.

Appels internes directs : aucun repéré.

<a id="fn-hideFace"></a>

### hideFace

Définition : [engine.c](engine.c), ligne 7237.

```c
void hideFace(Model3D* model, int face_idx)
```

**Rôle :**

Masquer une face en conservant sa longueur de liste pour restauration.

**Paramètres :**

[entrée] model et face_idx valides.

**Retour :**

Aucun.

**Sorties et effets de bord :**

longueur sauvegardée si positive, vertex_count=0 et display_flag=0.

**Fonctionnement :**

Invalide les caches d'ordre. Ne supprime ni indices ni sommets ; aucun contrôle d'arguments ici.

Appels internes directs : [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate).

<a id="fn-initPalettes"></a>

### initPalettes

Définition : [engine.c](engine.c), ligne 6338.

```c
void initPalettes(void)
```

**Rôle :**

Construire les quinze palettes de dégradé et demander la capture de la palette zéro.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Sorties et effets de bord :**

palettes[1..15] et palette0_loaded=0.

**Fonctionnement :**

La palette système zéro sera lue ultérieurement après le démarrage graphique.

Appels internes directs : [paletteGradientColor](arbre_appels.html#fn-paletteGradientColor).

<a id="fn-initScreenSaveBuffer"></a>

### initScreenSaveBuffer

Définition : [screen_save_restore.c](screen_save_restore.c), ligne 66.

```c
int initScreenSaveBuffer(void)
```

**Rôle :**

Allouer le buffer fixe utilisé par les transferts MVN de l'écran.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

1 si NewHandle réussit sans erreur Toolbox, 0 sinon.

**Sorties et effets de bord :**

gSaveHandle, gSaveBank et gSaveOffset décrivent le bloc verrouillé.

**Limites :**

les transferts suivants utilisent une banque unique et un offset 16 bits ; ils supposent une implantation du bloc compatible avec ces transferts.

Appels internes directs : aucun repéré.

<a id="fn-inspect_face_pair_ui"></a>

### inspect_face_pair_ui

Définition : [engine.c](engine.c), ligne 7626.

```c
static void inspect_face_pair_ui(Model3D* model)
```

**Rôle :**

Piloter l'inspecteur graphique et textuel de deux faces.

**Paramètres :**

[entrée] model ; NULL ou modèle vide ignoré.

**Retour :**

Aucun.

**Fonctionnement :**

Lit les deux identifiants puis permet navigation, comparaison des méthodes, export de diagnostic et changements manuels de permutation. Met temporairement les faces en filaire visibles et restaure options et drapeaux à chaque tour. Les modifications d'ordre demandées restent effectives ; pas de fusion de faces.

Utilise InspectionViewState pour la sauvegarde et la restauration communes des drapeaux et options ; le cycle de vie graphique reste géré par cet outil.

Appels internes directs : [compare_faces_diagnostic](arbre_appels.html#fn-compare_faces_diagnostic), [compute_intersection_region_bbox](arbre_appels.html#fn-compute_intersection_region_bbox), [DoText](arbre_appels.html#fn-DoText), [drawFace](arbre_appels.html#fn-drawFace), [drawPolygons](arbre_appels.html#fn-drawPolygons), [drawPolygons_jitter](arbre_appels.html#fn-drawPolygons_jitter), [dump_compare_results_to_file](arbre_appels.html#fn-dump_compare_results_to_file), [getkeypress](arbre_appels.html#fn-getkeypress), [inspection_view_begin](arbre_appels.html#fn-inspection_view_begin), [inspection_view_end](arbre_appels.html#fn-inspection_view_end), [keypress](arbre_appels.html#fn-keypress), [move_element_remove_and_insert](arbre_appels.html#fn-move_element_remove_and_insert), [ray_cast_distances](arbre_appels.html#fn-ray_cast_distances), [rch_patch_pair_at_scale](arbre_appels.html#fn-rch_patch_pair_at_scale), [rch_unpatch_pair](arbre_appels.html#fn-rch_unpatch_pair).

<a id="fn-inspect_faces_order"></a>

### inspect_faces_order

Définition : [engine.c](engine.c), ligne 6526.

```c
static void inspect_faces_order(Model3D* model, ObserverParams* params, int inspect_after)
```

**Rôle :**

Inspecter les faces avant ou après une cible et proposer sa correction.

**Paramètres :**

[entrée] model et params : contexte valide ; un pointeur NULL annule. [entrée] inspect_after : 0 pour D (faces précédentes), 1 pour S (faces suivantes).

**Retour :**

Aucun.

**Sorties et effets de bord :**

Lit le clavier, affiche les sept tests et une prévisualisation. Déplace la cible dans sorted_face_indices uniquement après une commande A/O. Sauvegarde puis restaure culling, visibilité et option de contour.

**Fonctionnement :**

Parcourt le côté demandé de la permutation, filtre les boîtes puis retient les préférences incompatibles. Sépare les chevauchements réels pour la commande O ; A utilise toutes les candidates. D place la cible avant la plus petite position retenue, S après la plus grande. Le déplacement stable utilise les positions avant retrait de la cible. Les messages et couleurs propres aux deux commandes sont conservés.

Utilise InspectionViewState pour la sauvegarde et la restauration communes des drapeaux et options ; le cycle de vie graphique reste géré par cet outil.

Appels internes directs : [DoText](arbre_appels.html#fn-DoText), [drawFace](arbre_appels.html#fn-drawFace), [drawPolygons](arbre_appels.html#fn-drawPolygons), [drawPolygons_jitter](arbre_appels.html#fn-drawPolygons_jitter), [evaluate_pair_tests](arbre_appels.html#fn-evaluate_pair_tests), [getkeypress](arbre_appels.html#fn-getkeypress), [inspection_view_begin](arbre_appels.html#fn-inspection_view_begin), [inspection_view_end](arbre_appels.html#fn-inspection_view_end), [keypress](arbre_appels.html#fn-keypress), [move_element_remove_and_insert](arbre_appels.html#fn-move_element_remove_and_insert), [pair_order_relation](arbre_appels.html#fn-pair_order_relation), [projected_polygons_overlap](arbre_appels.html#fn-projected_polygons_overlap).

<a id="fn-inspection_view_begin"></a>

### inspection_view_begin

Définition : [engine.c](engine.c), ligne 6077.

```c
static int inspection_view_begin(Model3D *model, InspectionViewState *state)
```

**Rôle :**

Sauvegarder visibilité, mode filaire et culling avant une inspection.

**Paramètres :**

[entrée] model : modèle valide au nombre de faces stable pendant la vue. [sortie] state : sauvegarde non active, possédant ensuite son buffer.

**Retour :**

1 si la sauvegarde est disponible, 0 si l'allocation échoue.

**Fonctionnement :**

Ne modifie ni le modèle ni les options. Les drapeaux sont sauvegardés un par un car display_flag est un tableau d'int, et non d'octets. L'appelant reste responsable de QuickDraw et du choix des faces visibles.

Appels internes directs : aucun repéré.

<a id="fn-inspection_view_end"></a>

### inspection_view_end

Définition : [engine.c](engine.c), ligne 6140.

```c
static void inspection_view_end(Model3D *model, InspectionViewState *state)
```

**Rôle :**

Restaurer la vue puis libérer sa sauvegarde.

**Paramètres :**

[entrée-sortie] model et state : modèle et sauvegarde associée.

**Retour :**

Aucun.

**Fonctionnement :**

Délègue la restauration commune puis libère le buffer et le met à NULL. N'ouvre ni ne ferme de contexte graphique ; un second appel est sans effet.

Appels internes directs : [inspection_view_restore](arbre_appels.html#fn-inspection_view_restore).

<a id="fn-inspection_view_restore"></a>

### inspection_view_restore

Définition : [engine.c](engine.c), ligne 6111.

```c
static void inspection_view_restore(Model3D *model, const InspectionViewState *state)
```

**Rôle :**

Restaurer l'état sauvegardé sans libérer le buffer réutilisable.

**Paramètres :**

[entrée-sortie] model : modèle de la sauvegarde, au même nombre de faces. [entrée] state : sauvegarde active ; flags peut intégrer les actions de V.

**Retour :**

Aucun.

**Fonctionnement :**

Restaure les drapeaux, le filaire et le culling. Ne ferme pas QuickDraw, ne modifie ni la géométrie ni l'ordre des faces. Une sauvegarde sans buffer est ignorée.

Appels internes directs : aucun repéré.

<a id="fn-keypress"></a>

### keypress

Définition : [asm.h](asm.h), ligne 50.

```c
asm int keypress ()
```

**Rôle :**

Attendre une frappe et acquitter le strobe clavier.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Sorties et effets de bord :**

A contient 0x0080 après le masquage, pas le caractère.

lecture de $00C000, écriture de $00C010 ; attente active sans timeout.

**Préconditions :**

accumulateur 16 bits conformément aux constantes employées.

Appels internes directs : aucun repéré.

<a id="fn-loadModel3D"></a>

### loadModel3D

Définition : [engine.c](engine.c), ligne 503.

```c
int loadModel3D(Model3D* model, const char* filename)
```

**Rôle :**

Charger sommets et faces OBJ dans un modèle déjà alloué.

**Paramètres :**

[entrée] filename et model.

**Retour :**

-1 pour arguments invalides ou échec de lecture des sommets ; 0 sinon.

**Sorties et effets de bord :**

coordonnées, listes et compteurs du modèle.

**Fonctionnement :**

Un échec de readFaces_model ne propage pas -1 : il laisse zéro face et un avertissement. readVertices_last_count reçoit le vertex_count antérieur à la lecture ; cette borne ne doit pas être décrite comme le nombre effectivement lu dans cet appel.

Appels internes directs : [readFaces_model](arbre_appels.html#fn-readFaces_model), [readVertices](arbre_appels.html#fn-readVertices).

<a id="fn-main"></a>

### main

Définition : [OBJExplorer.cc](OBJExplorer.cc), ligne 58.

```c
int main()
```

**Rôle :**

Piloter le chargement, les vues graphiques et les commandes de l'explorateur.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

0 à la sortie normale ; 1 sur les chemins d'erreur traités ici.

**Fonctionnement :**

Possède le modèle et coordonne QuickDraw, palettes, sauvegarde SHGR et Z-buffers. Les labels séparent chargement (newmodel), retraitement géométrique (bigloop), redessin (loopReDraw/draw), attente clavier (getakey) et libération (end). Les données numériques restent dans le moteur ; les chaînes visibles de l'interface ne sont pas traduites dans cette révision des commentaires.

Appels internes directs : [applyPalette](arbre_appels.html#fn-applyPalette), [benchmark_projected_overlap_all](arbre_appels.html#fn-benchmark_projected_overlap_all), [check_intersect](arbre_appels.html#fn-check_intersect), [colorChooser](arbre_appels.html#fn-colorChooser), [compute2DFromObserver](arbre_appels.html#fn-compute2DFromObserver), [computeOrientationShading](arbre_appels.html#fn-computeOrientationShading), [createModel3D](arbre_appels.html#fn-createModel3D), [destroyModel3D](arbre_appels.html#fn-destroyModel3D), [display_model_face_ids](arbre_appels.html#fn-display_model_face_ids), [DoColor](arbre_appels.html#fn-DoColor), [DoText](arbre_appels.html#fn-DoText), [drawPolygons](arbre_appels.html#fn-drawPolygons), [drawPolygons_jitter](arbre_appels.html#fn-drawPolygons_jitter), [dumpFace2DCoordinates](arbre_appels.html#fn-dumpFace2DCoordinates), [dumpFaceEquationsCSV](arbre_appels.html#fn-dumpFaceEquationsCSV), [dumpSortedFaceIndices](arbre_appels.html#fn-dumpSortedFaceIndices), [frameInconclusivePairs](arbre_appels.html#fn-frameInconclusivePairs), [generate_random_colors](arbre_appels.html#fn-generate_random_colors), [getkeypress_openA](arbre_appels.html#fn-getkeypress_openA), [getObserverParams](arbre_appels.html#fn-getObserverParams), [initPalettes](arbre_appels.html#fn-initPalettes), [initScreenSaveBuffer](arbre_appels.html#fn-initScreenSaveBuffer), [inspect_face_pair_ui](arbre_appels.html#fn-inspect_face_pair_ui), [inspect_faces_order](arbre_appels.html#fn-inspect_faces_order), [keypress](arbre_appels.html#fn-keypress), [loadModel3D](arbre_appels.html#fn-loadModel3D), [normalize_deg](arbre_appels.html#fn-normalize_deg), [normalizeAutoFitDistanceTo150](arbre_appels.html#fn-normalizeAutoFitDistanceTo150), [Offscreen_Clear](arbre_appels.html#fn-Offscreen_Clear), [Offscreen_FlushToScreen](arbre_appels.html#fn-Offscreen_FlushToScreen), [Offscreen_Init](arbre_appels.html#fn-Offscreen_Init), [Offscreen_Shutdown](arbre_appels.html#fn-Offscreen_Shutdown), [processModelFast](arbre_appels.html#fn-processModelFast), [ReadPalette](arbre_appels.html#fn-ReadPalette), [renderModelFullscreenZBuffer](arbre_appels.html#fn-renderModelFullscreenZBuffer), [renderModelFullscreenZBuffer_offscreen](arbre_appels.html#fn-renderModelFullscreenZBuffer_offscreen), [renderModelScanlineZBuffer](arbre_appels.html#fn-renderModelScanlineZBuffer), [repair_geov3](arbre_appels.html#fn-repair_geov3), [restoreScreen](arbre_appels.html#fn-restoreScreen), [saveNextScreenshot](arbre_appels.html#fn-saveNextScreenshot), [saveScreen](arbre_appels.html#fn-saveScreen), [screen2Black](arbre_appels.html#fn-screen2Black), [select_overlap_method](arbre_appels.html#fn-select_overlap_method), [shgroff](arbre_appels.html#fn-shgroff), [show_help_pager](arbre_appels.html#fn-show_help_pager), [showFace](arbre_appels.html#fn-showFace), [ZBuffer_Init](arbre_appels.html#fn-ZBuffer_Init), [ZBuffer_Shutdown](arbre_appels.html#fn-ZBuffer_Shutdown).

<a id="fn-move_element_remove_and_insert"></a>

### move_element_remove_and_insert

Définition : [engine.c](engine.c), ligne 4313.

```c
static int move_element_remove_and_insert(int *arr, int n, int from, int insert_idx)
```

**Rôle :**

Déplacer un élément d'un tableau en conservant l'ordre des autres.

**Paramètres :**

[entrée-sortie] arr de longueur n

[entrée] from et insert_idx, position finale.

**Retour :**

1 si déplacement, 0 si source invalide ou position inchangée.

**Fonctionnement :**

La destination est bornée à [0,n-1] ; memmove traite les zones qui se chevauchent.

Appels internes directs : aucun repéré.

<a id="fn-move_element_remove_and_insert_pos"></a>

### move_element_remove_and_insert_pos

Définition : [engine.c](engine.c), ligne 4355.

```c
static int move_element_remove_and_insert_pos(int *arr, int n, int from, int insert_idx, int *pos_of_face)
```

**Rôle :**

Déplacer un identifiant et maintenir la table inverse des positions.

**Paramètres :**

[entrée-sortie] arr de longueur n et pos_of_face, facultatif

[entrée] from et insert_idx, position finale bornée.

**Retour :**

1 si déplacement, 0 sinon.

**Fonctionnement :**

Les valeurs d'arr doivent indexer pos_of_face lorsqu'il est fourni.

Délègue le déplacement et la validation à move_element_remove_and_insert, puis met à jour la table inverse uniquement sur le segment affecté. Une table NULL conserve le chemin simple.

Appels internes directs : [move_element_remove_and_insert](arbre_appels.html#fn-move_element_remove_and_insert).

<a id="fn-normalize_deg"></a>

### normalize_deg

Définition : [declare.c](declare.c), ligne 257.

```c
static inline int normalize_deg(int deg)
```

**Rôle :**

Normaliser un angle entier en degrés.

**Paramètres :**

[entrée] deg.

**Retour :**

Représentant dans [0,359] par modulo corrigé si négatif.

**Fonctionnement :**

Aucun effet de bord ; ne modifie pas l'argument de l'appelant.

Appels internes directs : aucun repéré.

<a id="fn-normalizeAutoFitDistanceTo150"></a>

### normalizeAutoFitDistanceTo150

Définition : [engine.c](engine.c), ligne 5277.

```c
void normalizeAutoFitDistanceTo150(Model3D* model)
```

**Rôle :**

Appliquer la normalisation existante des suggestions de vue vers une distance de 150.

**Paramètres :**

[entrée-sortie] model ; NULL, distance ou projection non positives sont ignorés.

**Retour :**

Aucun.

**Sorties et effets de bord :**

géométrie et métadonnées modifiées, message affiché ; la vue dérivée doit ensuite être recalculée par processModelFast.

**Fonctionnement :**

Multiplie les coordonnées objet et la projection par sqrt(150/D), fixe la distance suggérée et current_observer_distance à 150. La formule conserve un rapport d'échelle global ; elle ne démontre pas une invariance exacte de perspective pour des sommets de profondeurs différentes.

Appels internes directs : aucun repéré.

<a id="fn-Off_BridgeBorderZQ"></a>

### Off_BridgeBorderZQ

Définition : [zbuffer_fullscreen_offscreen.c](zbuffer_fullscreen_offscreen.c), ligne 335.

```c
static void Off_BridgeBorderZQ(int x0, int y0, UWORD16 zq0, int x1, int y1, UWORD16 zq1, int color)
```

**Rôle :**

Relier deux extrémités de spans par une ligne de contour testée contre Z.

**Paramètres :**

[entrée] x0/y0,zq0 et x1/y1,zq1 sans pan ; color.

**Retour :**

Aucun.

**Fonctionnement :**

Bresenham sur l'axe majeur et interpolation Z en 16.16 ; applique pan lors du tracé par Off_PlotBridge. Ne remplit pas et ne remplace aucune profondeur.

Appels internes directs : [Off_PlotBridge](arbre_appels.html#fn-Off_PlotBridge).

<a id="fn-Off_PaintSpanZ"></a>

### Off_PaintSpanZ

Définition : [zbuffer_fullscreen_offscreen.c](zbuffer_fullscreen_offscreen.c), ligne 419.

```c
static void Off_PaintSpanZ(int x0, int x1, int y, int screenY, UWORD16 zq0, UWORD16 zq_end, int fillColor, int frameColor, int whole_span_is_border, OffZBufSpanPrev *prev, int z_nudge_lsbs)
```

**Rôle :**

Dessiner un span offscreen avec extrémités et liaisons de contour.

**Paramètres :**

[entrée] x0/x1 inclus et y sans pan, screenY avec pan ; zq0/zq_end, codes ; fillColor/frameColor ; whole_span_is_border, ligne haute/basse ; z_nudge_lsbs. [entrée-sortie] prev, span précédent facultatif de la face.

**Retour :**

Aucun.

**Préconditions :**

span préclippé et Z/framebuffer initialisés. Réduit les codes par nudge sans sous-dépassement, prépare le paquet et trace les liaisons entre spans consécutifs. Diffère du remplissage sur plan Z du rendu direct actuel. Pour deux pixels, le noyau n’utilise pas de pas intérieur ; pour un span de profondeur constante, ce pas vaut zéro sans division.

Appels internes directs : [Off_BridgeBorderZQ](arbre_appels.html#fn-Off_BridgeBorderZQ), [ZBuffer_SpanRun](arbre_appels.html#fn-ZBuffer_SpanRun).

<a id="fn-Off_PlotBridge"></a>

### Off_PlotBridge

Définition : [zbuffer_fullscreen_offscreen.c](zbuffer_fullscreen_offscreen.c), ligne 293.

```c
static void Off_PlotBridge(int sx, int sy, UWORD16 zq, int color)
```

**Rôle :**

Renforcer un pixel de liaison de contour sans modifier la profondeur.

**Paramètres :**

[entrée] sx/sy écran, zq interpolé et color.

**Retour :**

Aucun.

**Fonctionnement :**

Accepte une profondeur égale à Z ou plus lointaine d'au plus un code selon les différences int natives. N'introduit pas une surface plus proche dans Z. Le test de visibilité diffère de celui des contours du rendu direct. Écrit directement le nibble accepté dans le framebuffer : un seul test de coordonnées et aucun second appel par pixel de liaison.

Appels internes directs : aucun repéré.

<a id="fn-Off_RenderCore"></a>

### Off_RenderCore

Définition : [zbuffer_fullscreen_offscreen.c](zbuffer_fullscreen_offscreen.c), ligne 546.

```c
static void Off_RenderCore(Model3D* model, int biased)
```

**Rôle :**

Rendre les surfaces par balayage plein écran offscreen.

**Paramètres :**

[entrée] model valide avec projection et métriques à jour ; biased, active le rapprochement des faces de plane_d>0 lorsque le culling est désactivé.

**Retour :**

Aucun.

**Sorties et effets de bord :**

couleurs, mémoire Z, écran ou framebuffer, buffers statiques non réentrants.

**Fonctionnement :**

N’effectue ni transformation ni classement des faces ; les faces actives sont parcourues par identifiant pour définir les égalités de profondeur. Réserve/réutilise buffers par sommet/face/arête, calcule 1/z, prépare les arêtes, classe les débuts de face par histogramme puis balaie uniquement les faces actives. Profondeurs interpolées aux arêtes/spans ; contours par extrémités et liaisons des lignes successives. Ne reprend pas les contours sur plan du rendu direct.

Le clipping du viewport est appliqué aux spans ; pas de clipping 3D général. Les réserves et le comportement d'échec restent propres à cette version.

Les préparations numériques, types d’arêtes, tri des intersections, listes actives et noyau de spans sont communs aux moteurs compatibles. Les liaisons de contours et le framebuffer restent propres à offscreen.

Appels internes directs : [applyPalette](arbre_appels.html#fn-applyPalette), [Off_PaintSpanZ](arbre_appels.html#fn-Off_PaintSpanZ), [ZBuffer_ActivateFaces](arbre_appels.html#fn-ZBuffer_ActivateFaces), [ZBuffer_Clear](arbre_appels.html#fn-ZBuffer_Clear), [ZBuffer_FreeFaces](arbre_appels.html#fn-ZBuffer_FreeFaces), [ZBuffer_LerpZ](arbre_appels.html#fn-ZBuffer_LerpZ), [ZBuffer_PrepareFaces](arbre_appels.html#fn-ZBuffer_PrepareFaces), [ZBuffer_PrepareInvZ](arbre_appels.html#fn-ZBuffer_PrepareInvZ), [ZBuffer_PrepareScanEdges](arbre_appels.html#fn-ZBuffer_PrepareScanEdges), [ZBuffer_QuantizeInvZ](arbre_appels.html#fn-ZBuffer_QuantizeInvZ), [ZBuffer_ReserveFaces](arbre_appels.html#fn-ZBuffer_ReserveFaces), [ZBuffer_ReserveVertices](arbre_appels.html#fn-ZBuffer_ReserveVertices), [ZBuffer_SetScaleForFrame](arbre_appels.html#fn-ZBuffer_SetScaleForFrame), [ZBuffer_SizeFits](arbre_appels.html#fn-ZBuffer_SizeFits), [ZBuffer_SortFaceStarts](arbre_appels.html#fn-ZBuffer_SortFaceStarts), [ZBuffer_SortScanHits](arbre_appels.html#fn-ZBuffer_SortScanHits).

<a id="fn-Offscreen_Clear"></a>

### Offscreen_Clear

Définition : [zbuffer_fullscreen_offscreen.c](zbuffer_fullscreen_offscreen.c), ligne 153.

```c
void Offscreen_Clear(void)
```

**Rôle :**

Effacer le framebuffer par blocs de seize mots de 16 bits.

**Paramètres :**

Aucun paramètre explicite.

**Entrées et sorties implicites :**

[entrée] handle initialisé, sinon aucun effet.

**Retour :**

Aucun.

**Fonctionnement :**

Le handle attrNoCross garantit un bloc de 32000 octets dans une banque. Écrit seize mots par tour en adressage absolu indexé, sans lecture ni copie chevauchante. Le contenu final reste entièrement nul.

**Registres et indicateurs :**

Préserve P/DBR, modifie A/X/Y ; le buffer doit rester dans une banque. OFF_PIXELS doit être un multiple de 32.

Appels internes directs : aucun repéré.

<a id="fn-Offscreen_FlushToScreen"></a>

### Offscreen_FlushToScreen

Définition : [zbuffer_fullscreen_offscreen.c](zbuffer_fullscreen_offscreen.c), ligne 228.

```c
void Offscreen_FlushToScreen(void)
```

**Rôle :**

Copier le framebuffer RAM vers les 32000 octets de pixels SHGR.

**Paramètres :**

Aucun paramètre explicite.

**Entrées et sorties implicites :**

[entrée] handle offscreen, sinon aucun effet.

**Retour :**

Aucun.

**Fonctionnement :**

MVN patché : source RAM et destination $E1:2000, compteur OFF_PIXELS-1.

Ne copie ni les SCB ni les palettes situés après la zone des pixels.

**Registres et indicateurs :**

Préserve P/DBR, modifie A/X/Y ; code auto-modifiant non réentrant.

Appels internes directs : aucun repéré.

<a id="fn-Offscreen_Init"></a>

### Offscreen_Init

Définition : [zbuffer_fullscreen_offscreen.c](zbuffer_fullscreen_offscreen.c), ligne 74.

```c
int Offscreen_Init(void)
```

**Rôle :**

Allouer le framebuffer de 32000 octets en RAM sans franchissement de banque.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

1 en réussite, 0 sinon.

**Sorties et effets de bord :**

handle verrouillé, banque, offset, table off_row et off_ready.

**Fonctionnement :**

Appeler une fois avant utilisation. Contrairement à ZBuffer_Init, l'appel répété n'est pas idempotent : l'ancien handle n'est pas libéré avant remplacement. Le contenu n'est pas effacé ici.

Appels internes directs : aucun repéré.

<a id="fn-Offscreen_Shutdown"></a>

### Offscreen_Shutdown

Définition : [zbuffer_fullscreen_offscreen.c](zbuffer_fullscreen_offscreen.c), ligne 114.

```c
void Offscreen_Shutdown(void)
```

**Rôle :**

Libérer le framebuffer offscreen détenu par le module.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Fonctionnement :**

Annule off_ready et le handle ; appel répété accepté. Ne libère pas les buffers statiques de préparation de Off_RenderCore ni le Z-buffer.

Appels internes directs : aucun repéré.

<a id="fn-on_seg_ll"></a>

### on_seg_ll

Définition : [overlap_raster.c](overlap_raster.c), ligne 454.

```c
static int on_seg_ll(int ax,int ay,int bx,int by,int cx,int cy)
```

**Rôle :**

Tester si C appartient à la boîte fermée du segment AB.

**Paramètres :**

[entrée] ax/ay, bx/by, cx/cy.

**Retour :**

Booléen, sans effet de bord.

**Fonctionnement :**

Ne teste pas la colinéarité : l'appelant doit aussi vérifier l'orientation nulle.

Appels internes directs : aucun repéré.

<a id="fn-ordering_cache_invalidate"></a>

### ordering_cache_invalidate

Définition : [ordering_cache.c](ordering_cache.c), ligne 144.

```c
static void ordering_cache_invalidate(Model3D *model)
```

**Rôle :**

Changer la génération des caches et désigner leur modèle propriétaire.

**Paramètres :**

[entrée] model, propriétaire de la nouvelle vue.

**Retour :**

Aucun.

**Sorties et effets de bord :**

propriétaire, génération et paquet assembleur invalidé.

**Fonctionnement :**

Les allocations sont conservées. Lorsque le compteur revient à zéro, effacer les stamps avant de repartir à 1 pour ne pas réutiliser une ancienne vue.

Appels internes directs : aucun repéré.

<a id="fn-ordering_cache_release"></a>

### ordering_cache_release

Définition : [ordering_cache.c](ordering_cache.c), ligne 194.

```c
static void ordering_cache_release(Model3D *model)
```

**Rôle :**

Libérer les caches appartenant au modèle détruit.

**Paramètres :**

[entrée] model.

**Retour :**

Aucun ; aucun effet si ce modèle n'est pas propriétaire.

**Sorties et effets de bord :**

buffers libérés, tables et métadonnées remises à zéro.

**Fonctionnement :**

Les pointeurs retournés auparavant par les fonctions de cache deviennent invalides.

Appels internes directs : aucun repéré.

<a id="fn-ordering_fixed_plane"></a>

### ordering_fixed_plane

Définition : [ordering_cache.c](ordering_cache.c), ligne 479.

```c
static OrderFixedPlane *ordering_fixed_plane(Model3D *model,int face,Fixed64 epsilon)
```

**Rôle :**

Préparer le plan d'une face et les conditions du chemin assembleur.

**Paramètres :**

[entrée] model ; face, identifiant valide ; epsilon, tolérance du classement.

**Retour :**

Slot partagé OrderFixedPlane, réutilisable seulement avec les mêmes face, génération, époque et epsilon.

**Sorties et effets de bord :**

coefficients, côté de l'observateur, admissibilité assembleur et cache de côtés des sommets.

**Fonctionnement :**

Une collision remplace le slot.

Le noyau n'est admissible que pour les coefficients 32 bits, |d| <= 2^62 et epsilon positif ou nul.

Appels internes directs : [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate).

<a id="fn-ordering_fixed_plane_reset"></a>

### ordering_fixed_plane_reset

Définition : [ordering_cache.c](ordering_cache.c), ligne 439.

```c
static void ordering_fixed_plane_reset(void)
```

**Rôle :**

Invalider les plans préparés par changement d'époque.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Sorties et effets de bord :**

époque incrémentée et paquet assembleur invalidé ; en cas de rebouclage, efface les plans et invalide le cache dense.

**Fonctionnement :**

Les autres caches de chevauchement et de ray casting sont conservés.

Appels internes directs : aucun repéré.

<a id="fn-ordering_geometry_slot"></a>

### ordering_geometry_slot

Définition : [ordering_cache.c](ordering_cache.c), ligne 615.

```c
static unsigned char *ordering_geometry_slot(Model3D *model,int f1,int f2,Fixed64 epsilon)
```

**Rôle :**

Obtenir l'octet du cache dense qui contient une relation de plans orientée.

**Paramètres :**

[entrée] model ; f1/f2 à base zéro ; epsilon.

**Retour :**

Octet partagé à décoder sur deux bits, ou NULL pour le chemin de repli.

**Fonctionnement :**

Aucun résultat n'est écrit ici. Le cache dense est limité à 256 faces. Génération, époque, nombre de faces et epsilon déclenchent sa remise à zéro ; chaque octet contient quatre codes. Un échec de réserve est mémorisé pour ne pas réallouer en boucle dans la vue.

Appels internes directs : [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate).

<a id="fn-ordering_pair_hit"></a>

### ordering_pair_hit

Définition : [ordering_cache.c](ordering_cache.c), ligne 278.

```c
static int ordering_pair_hit(OrderPairEntry *entry, int f1, int f2, Fixed32 scale, unsigned int context)
```

**Rôle :**

Vérifier toutes les composantes de la clé d'une paire mémorisée.

**Paramètres :**

[entrée] entry ; f1/f2 orientés ; scale, projection ; context, contexte du test.

**Retour :**

Vrai si génération et clé correspondent, faux sinon.

**Fonctionnement :**

Aucun effet de bord.

Appels internes directs : aucun repéré.

<a id="fn-ordering_pair_slot"></a>

### ordering_pair_slot

Définition : [ordering_cache.c](ordering_cache.c), ligne 247.

```c
static OrderPairEntry *ordering_pair_slot(Model3D *model, OrderPairEntry *table, int f1, int f2)
```

**Rôle :**

Obtenir le slot de hash d'une paire orientée.

**Paramètres :**

[entrée] model, propriétaire attendu ; table, cache de ORDER_PAIR_SLOTS entrées ; f1 et f2, identifiants à base zéro.

**Retour :**

Adresse du slot, sans garantir un hit.

**Sorties et effets de bord :**

invalide les caches si le modèle change.

**Fonctionnement :**

Les collisions ne sont pas chaînées ici : ordering_pair_hit doit ensuite vérifier la clé entière.

Appels internes directs : [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate).

<a id="fn-ordering_pair_store"></a>

### ordering_pair_store

Définition : [ordering_cache.c](ordering_cache.c), ligne 310.

```c
static void ordering_pair_store(OrderPairEntry *entry, int f1, int f2, Fixed32 scale, unsigned int context, int result)
```

**Rôle :**

Remplacer un slot de paire par le résultat de la vue courante.

**Paramètres :**

[entrée] f1/f2, scale, context et result.

[entrée-sortie] entry.

**Retour :**

Aucun.

**Fonctionnement :**

La génération est publiée avec la clé complète ; une collision évince l'ancien résultat sans produire de faux hit.

Appels internes directs : aucun repéré.

<a id="fn-ordering_plane"></a>

### ordering_plane

Définition : [ordering_cache.c](ordering_cache.c), ligne 382.

```c
static double *ordering_plane(Model3D *model, int face, double fallback[4])
```

**Rôle :**

Convertir les quatre coefficients d'un plan fixe en double, avec cache.

**Paramètres :**

[entrée] model ; face, identifiant valide ; fallback, tableau de quatre doubles fourni par l'appelant.

**Retour :**

Coefficients en cache, ou fallback renseigné.

**Sorties et effets de bord :**

allocation différée par blocs de 64 faces ; un échec d'allocation n'est pas retenté dans la même génération.

**Fonctionnement :**

Le pointeur n'appartient pas à l'appelant. La borne MAX_FACES ne protège que le cache, pas l'accès aux données du modèle.

Appels internes directs : [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate).

<a id="fn-ordering_winding_hit"></a>

### ordering_winding_hit

Définition : [ordering_cache.c](ordering_cache.c), ligne 554.

```c
static int ordering_winding_hit(OrderWinding *entry,int face,Fixed32 scale)
```

**Rôle :**

Tester la validité d'une aire orientée mémorisée.

**Paramètres :**

[entrée] entry, face et scale.

**Retour :**

Vrai si génération, face et projection correspondent ; faux sinon.

**Fonctionnement :**

Aucun effet de bord.

Appels internes directs : aucun repéré.

<a id="fn-ordering_winding_slot"></a>

### ordering_winding_slot

Définition : [ordering_cache.c](ordering_cache.c), ligne 525.

```c
static OrderWinding *ordering_winding_slot(Model3D *model,int face)
```

**Rôle :**

Obtenir le slot de l'aire orientée d'une face.

**Paramètres :**

[entrée] model et face.

**Retour :**

Slot partagé, dont la validité reste à tester.

**Sorties et effets de bord :**

changement de propriétaire si nécessaire ; collisions par masquage.

Appels internes directs : [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate).

<a id="fn-ordering_winding_store"></a>

### ordering_winding_store

Définition : [ordering_cache.c](ordering_cache.c), ligne 584.

```c
static void ordering_winding_store(OrderWinding *entry,int face,Fixed32 scale,double area2)
```

**Rôle :**

Mémoriser le double de l'aire orientée d'une face projetée.

**Paramètres :**

[entrée] face, scale et area2.

[entrée-sortie] entry

**Retour :**

Aucun.

**Fonctionnement :**

La valeur est marquée avec la génération courante.

Appels internes directs : aucun repéré.

<a id="fn-ordering_workspace"></a>

### ordering_workspace

Définition : [ordering_cache.c](ordering_cache.c), ligne 342.

```c
static double *ordering_workspace(long bytes)
```

**Rôle :**

Obtenir un espace de travail partagé pour le clipping.

**Paramètres :**

[entrée] bytes, taille demandée en octets.

**Retour :**

Buffer existant ou agrandi, NULL si l'agrandissement est impossible.

**Sorties et effets de bord :**

remplace l'ancien buffer uniquement après une allocation réussie ; le contenu précédent n'est pas conservé lors d'un agrandissement.

**Fonctionnement :**

Ne pas libérer le pointeur retourné.

Appels internes directs : aucun repéré.

<a id="fn-orient_ll"></a>

### orient_ll

Définition : [overlap_raster.c](overlap_raster.c), ligne 429.

```c
static long long orient_ll(int ax,int ay,int bx,int by,int cx,int cy)
```

**Rôle :**

Calculer l'orientation signée de trois points entiers A, B et C.

**Paramètres :**

[entrée] ax/ay, bx/by, cx/cy.

**Retour :**

Déterminant 64 bits, nul si colinéaires.

**Fonctionnement :**

Utilise le multiplicateur partagé de produits bruts ; signe positif pour C à gauche de AB dans le repère des coordonnées fournies.

Appels internes directs : [fixed_product32_fast](arbre_appels.html#fn-fixed_product32_fast).

<a id="fn-overlap_bench_call"></a>

### overlap_bench_call

Définition : [overlap_raster.c](overlap_raster.c), ligne 3578.

```c
static int overlap_bench_call(Model3D *model, int f1, int f2, int algorithm)
```

**Rôle :**

Appeler une des six méthodes chronométrées du benchmark.

**Paramètres :**

[entrée] model, f1/f2 ; algorithm 0..5 : V1, original, Max, Fixed64, Raster, Raster x10.

**Retour :**

Booléen normalisé ; 0 pour numéro invalide.

**Fonctionnement :**

Cette numérotation diffère du menu runtime à sept choix qui inclut Simple.

Appels internes directs : [projected_polygons_overlap_raster](arbre_appels.html#fn-projected_polygons_overlap_raster), [projected_polygons_overlap_raster_x10](arbre_appels.html#fn-projected_polygons_overlap_raster_x10), [projected_polygons_overlap_v1](arbre_appels.html#fn-projected_polygons_overlap_v1), [projected_polygons_overlap_v2_fixed64](arbre_appels.html#fn-projected_polygons_overlap_v2_fixed64), [projected_polygons_overlap_v2_max](arbre_appels.html#fn-projected_polygons_overlap_v2_max), [projected_polygons_overlap_v2_original](arbre_appels.html#fn-projected_polygons_overlap_v2_original).

<a id="fn-overlap_cross_double"></a>

### overlap_cross_double

Définition : [overlap_raster.c](overlap_raster.c), ligne 2721.

```c
static double overlap_cross_double(double ax, double ay, double bx, double by, double px, double py)
```

**Rôle :**

Calculer le déterminant orienté de AB et AP en double.

**Paramètres :**

[entrée] ax/ay, bx/by et px/py.

**Retour :**

(bx-ax)*(py-ay)-(by-ay)*(px-ax).

**Fonctionnement :**

Aucun état modifié ; zéro signifie colinéaire selon l'arithmétique double.

Appels internes directs : aucun repéré.

<a id="fn-overlap_v2_edge_probes"></a>

### overlap_v2_edge_probes

Définition : [overlap_raster.c](overlap_raster.c), ligne 2841.

```c
static int overlap_v2_edge_probes(Model3D *model, int f1, int f2)
```

**Rôle :**

Chercher un point intérieur commun près du milieu des arêtes des deux faces.

**Paramètres :**

[entrée] model, f1/f2 valides.

**Retour :**

1 si une sonde est strictement dans les deux faces, 0 sinon.

**Fonctionnement :**

Deux côtés sont essayés à une distance normalisée par |dx|+|dy| et 1024. Échantillonnage heuristique, pas une preuve d'absence.

Appels internes directs : [overlap_v2_point_in_face](arbre_appels.html#fn-overlap_v2_point_in_face).

<a id="fn-overlap_v2_point_in_face"></a>

### overlap_v2_point_in_face

Définition : [overlap_raster.c](overlap_raster.c), ligne 2749.

```c
static int overlap_v2_point_in_face(Model3D *model, int face, double px, double py)
```

**Rôle :**

Classer un point double par rapport au contour projeté entier d'une face.

**Paramètres :**

[entrée] model, face valide et px/py.

**Retour :**

-1 sur le contour, 1 intérieur, 0 extérieur.

**Fonctionnement :**

Parité du rayon horizontal et test explicite de colinéarité. La variante optimisée filtre le test de bord par la boîte de l'arête.

Appels internes directs : [overlap_cross_double](arbre_appels.html#fn-overlap_cross_double).

<a id="fn-overlap_v2_proper_cross"></a>

### overlap_v2_proper_cross

Définition : [overlap_raster.c](overlap_raster.c), ligne 2805.

```c
static int overlap_v2_proper_cross(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy)
```

**Rôle :**

Tester un croisement strict de segments doubles AB et CD.

**Paramètres :**

[entrée] quatre couples ax/ay, bx/by, cx/cy, dx/dy.

**Retour :**

1 si les deux paires de déterminants ont des signes strictement opposés, 0 sinon.

**Fonctionnement :**

Exclut contacts et colinéarité ; aucun état modifié.

Appels internes directs : [overlap_cross_double](arbre_appels.html#fn-overlap_cross_double).

<a id="fn-ovf_cross"></a>

### ovf_cross

Définition : [overlap_raster.c](overlap_raster.c), ligne 3167.

```c
static ovf64 ovf_cross(int ax,int ay,int bx,int by,int px,int py)
```

**Rôle :**

Calculer le déterminant de trois points entiers avec intermédiaires ovf64.

**Paramètres :**

[entrée] ax/ay, bx/by, px/py.

**Retour :**

Déterminant signé ; aucun état modifié.

**Fonctionnement :**

Les conversions précèdent les différences pour éviter leur débordement en int.

Appels internes directs : aucun repéré.

<a id="fn-ovf_inside"></a>

### ovf_inside

Définition : [overlap_raster.c](overlap_raster.c), ligne 3195.

```c
static int ovf_inside(const int *ids,const int *xs,const int *ys, int off,int n,ovf64 nx,ovf64 ny,ovf64 d)
```

**Rôle :**

Classer un point rationnel par rapport à un contour projeté entier.

**Paramètres :**

[entrée] ids base un, xs/ys, off/n ; nx/ny numérateurs et d dénominateur positif.

**Retour :**

-1 bord, 1 intérieur, 0 extérieur.

**Fonctionnement :**

Utilise des produits ovf64 sans division. L'appelant doit garantir que ces produits restent représentables.

Appels internes directs : aucun repéré.

<a id="fn-ovf_probe"></a>

### ovf_probe

Définition : [overlap_raster.c](overlap_raster.c), ligne 3250.

```c
static int ovf_probe(const int *ids,const int *xs,const int *ys, int off1,int n1,int off2,int n2)
```

**Rôle :**

Chercher des sondes rationnelles communes près des milieux d'arêtes.

**Paramètres :**

[entrée] ids base un, xs/ys, off1/n1 et off2/n2.

**Retour :**

Booléen.

**Fonctionnement :**

Encode les décalages normalisés avec d=2048*(|dx|+|dy|), puis appelle ovf_inside. Évite les divisions flottantes ; reste une stratégie de sondage fini.

Appels internes directs : [ovf_inside](arbre_appels.html#fn-ovf_inside).

<a id="fn-ovmax_inside"></a>

### ovmax_inside

Définition : [overlap_raster.c](overlap_raster.c), ligne 2975.

```c
static int ovmax_inside(const int *ids,const int *xs,const int *ys, int off,int n,double px,double py)
```

**Rôle :**

Classer un point double avec les tableaux projetés passés directement.

**Paramètres :**

[entrée] ids à base un ; xs/ys ; off/n décrivant une face non vide ; px/py.

**Retour :**

-1 bord, 1 intérieur, 0 extérieur.

**Fonctionnement :**

Aucun état modifié. Conserve le sommet précédent pour limiter les lectures d'indices.

Appels internes directs : [overlap_cross_double](arbre_appels.html#fn-overlap_cross_double).

<a id="fn-ovmax_probe"></a>

### ovmax_probe

Définition : [overlap_raster.c](overlap_raster.c), ligne 3020.

```c
static int ovmax_probe(const int *ids,const int *xs,const int *ys, int off1,int n1,int off2,int n2)
```

**Rôle :**

Sonder les milieux d'arêtes dans la variante à tableaux directs.

**Paramètres :**

[entrée] ids base un, xs/ys, off1/n1 et off2/n2.

**Retour :**

Booléen de point strictement commun.

**Fonctionnement :**

Même déplacement double normalisé de 1/1024 que V2 original. Ne modifie ni les tableaux ni les projections.

Appels internes directs : [ovmax_inside](arbre_appels.html#fn-ovmax_inside).

<a id="fn-painter_bubble_sort"></a>

### painter_bubble_sort

Définition : [engine.c](engine.c), ligne 3026.

```c
void painter_bubble_sort(Model3D* model, int face_count)
```

**Rôle :**

Corriger l'ordre FAST par échanges de voisins avec tests géométriques.

**Paramètres :**

[entrée] model et face_count.

**Retour :**

Aucun.

**Sorties et effets de bord :**

permutation et paires inconclusives

**Fonctionnement :**

Restreint le parcours au préfixe visible lorsque le culling est actif. Après profondeur et boîtes, teste le chevauchement et les plans. Une table de paires empêche de retraiter les relations déjà rencontrées ; les ambiguïtés sont conservées pour inspection. En cas d'échec de réserve, revient à FAST. Ne découpe aucune face et ne garantit pas de résoudre tous les cycles.

Appels internes directs : [geometric_face_relation](arbre_appels.html#fn-geometric_face_relation), [painter_newell_sancha_fast](arbre_appels.html#fn-painter_newell_sancha_fast), [projected_polygons_overlap](arbre_appels.html#fn-projected_polygons_overlap).

<a id="fn-painter_correct"></a>

### painter_correct

Définition : [engine.c](engine.c), ligne 4029.

```c
static int painter_correct(Model3D* model, int face_count, int use_v2)
```

**Rôle :**

Exécuter CORRECT ou CORRECT V2 à partir de la même préparation FAST.

**Paramètres :**

[entrée] model : modèle aux métriques à jour ; NULL annule le traitement. [entrée] face_count : nombre de faces de la permutation à corriger. [entrée] use_v2 : 0 pour CORRECT, 1 pour CORRECT V2.

**Retour :**

Nombre de déplacements des passes locales ; 0 si aucun traitement ou si une allocation échoue. Les déplacements de la vérification croisée V2 ne sont pas ajoutés à ce compteur, comme dans l'ancienne version.

**Sorties et effets de bord :**

Modifie sorted_face_indices et les caches utilisés par FAST et les tests géométriques. Alloue puis libère les tables temporaires et affiche les messages propres à chaque variante.

**Fonctionnement :**

Désactive le culling pour FAST, partitionne les faces selon plane_d, prépare les positions inverses et les limites de déplacement. Partage la passe avant ; conserve les différences de rejet Y, de validation de position et d'affichage entre les variantes. V2 ajoute la passe arrière et la vérification croisée si le culling était désactivé. Restaure le culling sur le chemin normal et ne découpe aucune face.

La vérification croisée V2 déplace les faces du snapshot avec move_element_remove_and_insert_pos, comme les passes locales ; le compteur moves conserve sa convention.

**Limites :**



**Les retours d'échec d'allocation conservent le comportement antérieur :**

ils ne restaurent pas tous le culling. Le compteur ne prouve pas la correction globale de la permutation.

Appels internes directs : [geo_face_order](arbre_appels.html#fn-geo_face_order), [move_element_remove_and_insert_pos](arbre_appels.html#fn-move_element_remove_and_insert_pos), [painter_newell_sancha_fast](arbre_appels.html#fn-painter_newell_sancha_fast), [projected_polygons_overlap_simple](arbre_appels.html#fn-projected_polygons_overlap_simple).

<a id="fn-painter_geoV2"></a>

### painter_geoV2

Définition : [engine.c](engine.c), ligne 3853.

```c
void painter_geoV2(Model3D* model, int face_count)
```

**Rôle :**

Corriger l'ordre FAST par passes de voisins et cache de paires borné.

**Paramètres :**

[entrée] model et face_count.

**Retour :**

Aucun.

**Sorties et effets de bord :**

permutation et diagnostics d'ambiguïté

**Fonctionnement :**

Teste les plans en premier, puis pour les cas ambigus les intervalles Z, le chevauchement et le ray casting hiérarchique. Réduit la limite au dernier échange et borne les passes à deux fois le nombre visible. Essaie des caches de 4096, 2048 puis 1024 entrées ; sans cache, garde FAST. La convention +1 avant de PairCache doit être convertie depuis les prédicats.

Appels internes directs : [geometric_face_relation](arbre_appels.html#fn-geometric_face_relation), [painter_newell_sancha_fast](arbre_appels.html#fn-painter_newell_sancha_fast), [pair_cache_create](arbre_appels.html#fn-pair_cache_create), [pair_cache_destroy](arbre_appels.html#fn-pair_cache_destroy), [pair_cache_find](arbre_appels.html#fn-pair_cache_find), [pair_cache_insert](arbre_appels.html#fn-pair_cache_insert), [projected_polygons_overlap](arbre_appels.html#fn-projected_polygons_overlap), [ray_cast_hierarchical](arbre_appels.html#fn-ray_cast_hierarchical).

<a id="fn-painter_geoV3"></a>

### painter_geoV3

Définition : [dony.c](dony.c), ligne 529.

```c
void painter_geoV3(Model3D* model, int face_count)
```

**Rôle :**

Construire l'ordre arrière-vers-avant par sélection et reprise des comparaisons.

**Paramètres :**

[entrée] model et face_count, nombre de faces à ordonner.

**Retour :**

Aucun.

**Sorties et effets de bord :**

model->faces.sorted_face_indices

**Fonctionnement :**

Initialise l'ordre canonique, trie par zmax, rejette les intervalles disjoints, puis teste les plans dans les deux sens. Une face devant précéder P est permutée en tête de la passe, qui reprend avec ce nouveau P. En cas d'ambiguïté ou de cycle, utilise le chevauchement projeté et le ray casting hiérarchique. Aucun découpage : des cycles peuvent rester insolubles. État statique conservé entre appels ; arrêt du processus si la réserve échoue.

Appels internes directs : [geov3_run](arbre_appels.html#fn-geov3_run).

<a id="fn-painter_newell_sancha"></a>

### painter_newell_sancha

Définition : [engine.c](engine.c), ligne 3221.

```c
void painter_newell_sancha(Model3D* model, int face_count)
```

**Rôle :**

Corriger l'ordre FAST à partir des paires dont les intervalles Z se recouvrent.

**Paramètres :**

[entrée] model et face_count

**Retour :**

Aucun.

**Sorties et effets de bord :**

permutation

**Fonctionnement :**

Un balayage des faces classées par zmin construit les candidats actifs. Pour chaque paire, les boîtes, les deux relations de plans, le chevauchement projeté puis le ray casting décident si une insertion doit avancer une face. Une paire candidate ne provoque qu'un déplacement via s_moved ; les tables de relations évitent les répétitions. Buffers statiques conservés entre appels. Les paires insolubles peuvent être ajoutées au buffer de diagnostic si disponible. Sur échec d'allocation, reprend FAST ; aucun découpage automatique.

Les insertions stables délèguent le déplacement et la mise à jour des positions inverses à move_element_remove_and_insert_pos.

Appels internes directs : [geometric_face_relation](arbre_appels.html#fn-geometric_face_relation), [move_element_remove_and_insert_pos](arbre_appels.html#fn-move_element_remove_and_insert_pos), [painter_newell_sancha_fast](arbre_appels.html#fn-painter_newell_sancha_fast), [projected_polygons_overlap](arbre_appels.html#fn-projected_polygons_overlap), [ray_cast_hierarchical](arbre_appels.html#fn-ray_cast_hierarchical).

<a id="fn-painter_newell_sancha_fast"></a>

### painter_newell_sancha_fast

Définition : [engine.c](engine.c), ligne 2967.

```c
void painter_newell_sancha_fast(Model3D* model, int face_count)
```

**Rôle :**

Construire l'ordre FAST par profondeur moyenne décroissante.

**Paramètres :**

[entrée] model et face_count.

**Retour :**

Aucun.

**Sorties et effets de bord :**

sorted_face_indices

**Fonctionnement :**

Avec culling, regroupe les faces affichables en tête et les autres en fin, puis trie uniquement le préfixe visible. Sans culling, trie toutes les faces. Ne teste ni plans, ni chevauchement, ni cycles ; le nom historique ne désigne pas ici l'algorithme Newell/Sancha complet.

Appels internes directs : aucun repéré.

Référence indirecte : `cmp_faces_by_zmean` via `qsort`.

<a id="fn-pair_cache_create"></a>

### pair_cache_create

Définition : [engine.c](engine.c), ligne 3682.

```c
static PairCache* pair_cache_create(int capacity)
```

**Rôle :**

Allouer un cache de relations de paires avec sondage borné.

**Paramètres :**

[entrée] capacity strictement positive.

**Retour :**

Cache possédé par l'appelant, ou NULL sur échec.

**Fonctionnement :**

Les slots commencent vides et doivent être libérés par pair_cache_destroy.

Appels internes directs : aucun repéré.

<a id="fn-pair_cache_destroy"></a>

### pair_cache_destroy

Définition : [engine.c](engine.c), ligne 3822.

```c
static void pair_cache_destroy(PairCache* c)
```

**Rôle :**

Libérer un cache de paires et ses slots.

**Paramètres :**

[entrée] c, NULL accepté.

**Retour :**

Aucun ; le pointeur de l'appelant devient invalide.

Appels internes directs : aucun repéré.

<a id="fn-pair_cache_find"></a>

### pair_cache_find

Définition : [engine.c](engine.c), ligne 3743.

```c
static int pair_cache_find(PairCache* c, int f1, int f2)
```

**Rôle :**

Lire une relation en sondant au plus huit slots.

**Paramètres :**

[entrée] c, éventuellement NULL, et f1/f2.

**Retour :**

+1 f1 avant f2, -1 sens inverse, 0 absent ou inconclusif.

**Fonctionnement :**

Une paire stockée en sens inverse rend le signe opposé. Un slot vide termine la recherche ; aucun effet de bord.

Appels internes directs : [pair_cache_slot](arbre_appels.html#fn-pair_cache_slot).

<a id="fn-pair_cache_insert"></a>

### pair_cache_insert

Définition : [engine.c](engine.c), ligne 3784.

```c
static void pair_cache_insert(PairCache* c, int f1, int f2, int8_t relation)
```

**Rôle :**

Mémoriser une relation en cherchant au plus huit slots.

**Paramètres :**

[entrée] c, éventuellement NULL ; f1/f2 ; relation, +1 f1 avant, -1 sens inverse.

**Retour :**

Aucun.

**Sorties et effets de bord :**

slot libre ou correspondant remplacé

**Fonctionnement :**

Si les huit slots sont occupés par d'autres clés, la relation est simplement abandonnée.

Appels internes directs : [pair_cache_slot](arbre_appels.html#fn-pair_cache_slot).

<a id="fn-pair_cache_slot"></a>

### pair_cache_slot

Définition : [engine.c](engine.c), ligne 3714.

```c
static int pair_cache_slot(PairCache* c, int f1, int f2)
```

**Rôle :**

Calculer le slot initial d'une paire non orientée.

**Paramètres :**

[entrée] c, cache de capacité positive ; f1/f2.

**Retour :**

Indice du slot.

**Fonctionnement :**

Trie les deux identifiants avant hash ; aucun test de présence ni effet de bord.

Appels internes directs : aucun repéré.

<a id="fn-pair_order_relation"></a>

### pair_order_relation

Définition : [engine.c](engine.c), ligne 7339.

```c
static int pair_order_relation(Model3D* model, int f1, int f2)
```

**Rôle :**

Fournir une préférence d'ordre par profondeur, bornes puis plans.

**Paramètres :**

[entrée] model et f1/f2 à base zéro

**Retour :**

-1 f1 avant, +1 f2 avant, 0 inconclusif.

**Sorties et effets de bord :**

caches de relations ; ne modifie pas la permutation.

**Fonctionnement :**

Réinitialise explicitement le cache géométrique. Les séparations X/Y produisent une préférence déterministe même sans occlusion ; aucun ray casting ici.

Appels internes directs : [geometric_face_relation](arbre_appels.html#fn-geometric_face_relation).

<a id="fn-paletteGradientColor"></a>

### paletteGradientColor

Définition : [declare.c](declare.c), ligne 588.

```c
static GS_Color paletteGradientColor(int palette_num, int index)
```

**Rôle :**

Construire une entrée d'une palette de dégradé.

**Paramètres :**

[entrée] palette_num, 1 pour les gris, 2..15 pour une teinte cible ; index, position du dégradé.

**Retour :**

Couleur $0RGB ; aucun effet de bord.

**Fonctionnement :**

Pour les teintes, l'indice 8 porte la couleur cible, encadrée par le noir à 0 et le blanc à 15. Les interpolations entières utilisent un biais d'arrondi.

Appels internes directs : [encodePaletteColor](arbre_appels.html#fn-encodePaletteColor).

<a id="fn-point_in_poly_arrays_int"></a>

### point_in_poly_arrays_int

Définition : [overlap_raster.c](overlap_raster.c), ligne 383.

```c
static int point_in_poly_arrays_int(int px, int py, int n, const int vx[], const int vy[])
```

**Rôle :**

Tester un point entier par parité des traversées d'un rayon horizontal.

**Paramètres :**

[entrée] px/py ; n ; vx/vy de longueur n.

**Retour :**

0 ou 1 ; n<3 rend 0.

**Fonctionnement :**

Un sommet exactement égal rend 1, mais les autres contacts de contour ne sont pas traités uniformément comme intérieur. Produits 64 bits après différences natives int : ne pas présenter cette routine comme exacte sur tout le domaine 16 bits.

Appels internes directs : aucun repéré.

<a id="fn-point_in_poly_int"></a>

### point_in_poly_int

Définition : [overlap_raster.c](overlap_raster.c), ligne 549.

```c
static int point_in_poly_int(int px,int py,FaceArrays3D* faces, VertexArrays3D* vtx, int f, int n)
```

**Rôle :**

Tester l'intérieur strict d'une face projetée à un point entier.

**Paramètres :**

[entrée] px/py, faces/vtx, f à base zéro, n longueur valide de sa liste.

**Retour :**

Parité 0/1, avec rejet explicite du contour ; n<=0 rend 0.

**Fonctionnement :**

Utilise une intersection par division entière en long ; les différences restent des int natifs. N'est pas équivalent à la variante tableaux pour les bords.

Appels internes directs : [on_seg_ll](arbre_appels.html#fn-on_seg_ll), [orient_ll](arbre_appels.html#fn-orient_ll).

<a id="fn-poly_signed_area"></a>

### poly_signed_area

Définition : [engine.c](engine.c), ligne 1792.

```c
static double poly_signed_area(Model3D* model, int *list, int n, double a, double b, double c)
```

**Rôle :**

Mesurer l'aire orientée de la projection dominante d'un contour objet.

**Paramètres :**

[entrée] model ; list, indices à base zéro ; n ; a/b/c, normale choisissant l'axe à éliminer.

**Retour :**

Aire signée en unités objet carrées, 0 pour n < 3.

**Fonctionnement :**

Ce n'est pas l'aire 3D normalisée du polygone. Aucun effet de bord.

Appels internes directs : aucun repéré.

<a id="fn-processModelFast"></a>

### processModelFast

Définition : [engine.c](engine.c), ligne 2040.

```c
void processModelFast(Model3D* model, ObserverParams* params, const char* filename)
```

**Rôle :**

Transformer les sommets, projeter, calculer les faces et exécuter la méthode de classement choisie.

**Paramètres :**

[entrée] model, géométrie chargée ; params, angles en degrés et distance 16.16 ; filename, argument conservé par l'interface mais non utilisé ici.

**Retour :**

Aucun.

**Sorties et effets de bord :**

xo/yo/zo, x2d/y2d, métriques et permutation des faces, couleurs de shading.

**Fonctionnement :**

Précalcule la matrice de rotation puis projette les sommets de profondeur positive. Pour zo <= 0, écrit xo=yo=0 et x2d=y2d=-1. En filaire, calcule encore les métriques mais saute le tri. Invalide les caches dépendant de la transformation.

Appels internes directs : [calculateFaceDepths](arbre_appels.html#fn-calculateFaceDepths), [computeOrientationShading](arbre_appels.html#fn-computeOrientationShading), [cos_deg_int](arbre_appels.html#fn-cos_deg_int), [fixed_mul32_fast](arbre_appels.html#fn-fixed_mul32_fast), [FIXED_ROUND_TO_INT](arbre_appels.html#fn-FIXED_ROUND_TO_INT), [fractional_projection_cache_invalidate](arbre_appels.html#fn-fractional_projection_cache_invalidate), [painter_bubble_sort](arbre_appels.html#fn-painter_bubble_sort), [painter_correct](arbre_appels.html#fn-painter_correct), [painter_geoV2](arbre_appels.html#fn-painter_geoV2), [painter_geoV3](arbre_appels.html#fn-painter_geoV3), [painter_newell_sancha](arbre_appels.html#fn-painter_newell_sancha), [painter_newell_sancha_fast](arbre_appels.html#fn-painter_newell_sancha_fast), [sin_deg_int](arbre_appels.html#fn-sin_deg_int).

<a id="fn-projected_polygons_overlap"></a>

### projected_polygons_overlap

Définition : [overlap_raster.c](overlap_raster.c), ligne 3404.

```c
static int projected_polygons_overlap(Model3D *model, int f1, int f2)
```

**Rôle :**

Choisir la méthode de chevauchement configurée à l'exécution.

**Paramètres :**

[entrée] model, f1/f2 ; overlap_method global.

**Retour :**

Booléen de la méthode.

**Fonctionnement :**

Sept choix : Simple, V1, V2 original, V2 Max, Fixed64, Raster, Raster x10. Une valeur inconnue prend V1. Les garanties diffèrent, notamment géométrie continue, seuils heuristiques et échantillonnage de centres de pixels.

Appels internes directs : [projected_polygons_overlap_raster](arbre_appels.html#fn-projected_polygons_overlap_raster), [projected_polygons_overlap_raster_x10](arbre_appels.html#fn-projected_polygons_overlap_raster_x10), [projected_polygons_overlap_simple](arbre_appels.html#fn-projected_polygons_overlap_simple), [projected_polygons_overlap_v1](arbre_appels.html#fn-projected_polygons_overlap_v1), [projected_polygons_overlap_v2_fixed64](arbre_appels.html#fn-projected_polygons_overlap_v2_fixed64), [projected_polygons_overlap_v2_max](arbre_appels.html#fn-projected_polygons_overlap_v2_max), [projected_polygons_overlap_v2_original](arbre_appels.html#fn-projected_polygons_overlap_v2_original).

<a id="fn-projected_polygons_overlap_cached"></a>

### projected_polygons_overlap_cached

Définition : [overlap_raster.c](overlap_raster.c), ligne 3449.

```c
static int projected_polygons_overlap_cached(Model3D *model, int f1, int f2)
```

**Rôle :**

Mémoriser uniquement les chevauchements positifs pour la méthode de classement.

**Paramètres :**

[entrée] model, f1/f2 orientés

**Retour :**

Booléen.

**Sorties et effets de bord :**

cache et compteurs optionnels.

**Fonctionnement :**

Clé : vue, paire, projection et méthode runtime. Un résultat nul peut être un échec de ressource ou de garde, donc n'est pas conservé.

Appels internes directs : [ordering_pair_hit](arbre_appels.html#fn-ordering_pair_hit), [ordering_pair_slot](arbre_appels.html#fn-ordering_pair_slot), [ordering_pair_store](arbre_appels.html#fn-ordering_pair_store), [projected_polygons_overlap](arbre_appels.html#fn-projected_polygons_overlap).

<a id="fn-projected_polygons_overlap_raster"></a>

### projected_polygons_overlap_raster

Définition : [overlap_raster.c](overlap_raster.c), ligne 2586.

```c
static int projected_polygons_overlap_raster( Model3D* model, int f1, int f2)
```

**Rôle :**

Chercher un centre de pixel strictement couvert par les deux faces.

**Paramètres :**

[entrée] model, f1/f2 et boîtes à jour.

**Retour :**

Booléen raster ; 0 aussi pour faces trop courtes ou dépassant RASTER_MAX_INTERSECTIONS.

**Fonctionnement :**

Prépare les arêtes, parcourt les lignes de bbox communes et compare les spans pair/impair. Un chevauchement subpixel peut être absent de cet échantillonnage.

**Limites :**

si la première face n'a pas deux intersections sur une ligne, la seconde n'est pas avancée sur cette ligne ; ne pas généraliser l'exactitude.

Appels internes directs : [raster_incremental_scanline](arbre_appels.html#fn-raster_incremental_scanline), [raster_intervals_share_pixel](arbre_appels.html#fn-raster_intervals_share_pixel), [raster_prepare_edges_incremental](arbre_appels.html#fn-raster_prepare_edges_incremental).

<a id="fn-projected_polygons_overlap_raster_x10"></a>

### projected_polygons_overlap_raster_x10

Définition : [overlap_raster.c](overlap_raster.c), ligne 2153.

```c
static int projected_polygons_overlap_raster_x10( Model3D* model, int f1, int f2)
```

**Rôle :**

Tester le chevauchement raster après un grossissement temporaire par dix.

**Paramètres :**

[entrée] model, f1/f2

**Retour :**

Booléen raster, pas une aire géométrique exacte.

**Fonctionnement :**

Effets temporaires : échelle, coordonnées et boîtes de la paire, restaurées après le test.

**Préconditions :**

profondeurs non nulles et indices valides.

Appels internes directs : [projected_polygons_overlap_raster](arbre_appels.html#fn-projected_polygons_overlap_raster), [rch_patch_pair_at_scale](arbre_appels.html#fn-rch_patch_pair_at_scale), [rch_unpatch_pair](arbre_appels.html#fn-rch_unpatch_pair).

<a id="fn-projected_polygons_overlap_simple"></a>

### projected_polygons_overlap_simple

Définition : [overlap_raster.c](overlap_raster.c), ligne 2069.

```c
static int projected_polygons_overlap_simple(Model3D* model, int f1, int f2)
```

**Rôle :**

Tester rapidement croisements propres et inclusion stricte de sommets.

**Paramètres :**

[entrée] model, f1/f2 valides

**Retour :**

Booléen.

**Fonctionnement :**

Aucun recours au clipping. Les contacts seuls ne suffisent pas. Peut manquer les contours coïncidents ou certains chevauchements dont tous les sommets sont sur le contour ; les filtres de boîtes d'arêtes et divisions entières définissent ses limites.

Appels internes directs : [point_in_poly_int](arbre_appels.html#fn-point_in_poly_int), [segs_intersect_int](arbre_appels.html#fn-segs_intersect_int).

<a id="fn-projected_polygons_overlap_v1"></a>

### projected_polygons_overlap_v1

Définition : [overlap_raster.c](overlap_raster.c), ligne 1544.

```c
static int projected_polygons_overlap_v1(Model3D* model, int f1, int f2)
```

**Rôle :**

Tester le chevauchement avec les filtres et heuristiques historiques V1.

**Paramètres :**

[entrée] model, f1/f2 valides et métriques à jour.

**Retour :**

1 pour chevauchement retenu, 0 pour rejet, cas non concluants ou garde temporelle.

**Sorties et effets de bord :**

compteurs, diagnostics, cache de clipping et lecture de GetTick.

**Fonctionnement :**

Après boîtes et tests de segments/échantillons, recourt au clipping et vérifie ses aires avec seuils et bornes. N'est pas équivalent à un test géométrique exact.

Le watchdog nommé MS compare directement des ticks ; ne pas le décrire en millisecondes.

Appels internes directs : [compute_intersection_centroid_ordered_fixed](arbre_appels.html#fn-compute_intersection_centroid_ordered_fixed), [faces_vertices_equal](arbre_appels.html#fn-faces_vertices_equal), [point_in_poly_int](arbre_appels.html#fn-point_in_poly_int), [segs_intersect_int](arbre_appels.html#fn-segs_intersect_int).

<a id="fn-projected_polygons_overlap_v2_fixed64"></a>

### projected_polygons_overlap_v2_fixed64

Définition : [overlap_raster.c](overlap_raster.c), ligne 3309.

```c
static int projected_polygons_overlap_v2_fixed64(Model3D *model,int f1,int f2)
```

**Rôle :**

Tester le chevauchement projeté par Fixed64 avec déterminants et sondes rationnelles.

**Paramètres :**

[entrée] model et f1/f2 valides

**Retour :**

1 si un test trouve une zone commune, 0 sinon ou face trop courte.

**Fonctionnement :**

Rejette d'abord les boîtes sans intérieur commun, puis essaye croisements propres, sommets strictement intérieurs et sondes près des milieux d'arêtes. Les seules touches de contours ne suffisent pas. Ne calcule pas l'aire d'intersection et ne modifie pas le modèle ; les variantes ne doivent pas être fusionnées sans comparaison de leurs limites numériques.

Appels internes directs : [ovf_cross](arbre_appels.html#fn-ovf_cross), [ovf_inside](arbre_appels.html#fn-ovf_inside), [ovf_probe](arbre_appels.html#fn-ovf_probe).

<a id="fn-projected_polygons_overlap_v2_max"></a>

### projected_polygons_overlap_v2_max

Définition : [overlap_raster.c](overlap_raster.c), ligne 3075.

```c
static int projected_polygons_overlap_v2_max(Model3D *model,int f1,int f2)
```

**Rôle :**

Tester le chevauchement projeté par V2 Max avec pointeurs locaux et rejets par boîtes d'arêtes.

**Paramètres :**

[entrée] model et f1/f2 valides

**Retour :**

1 si un test trouve une zone commune, 0 sinon ou face trop courte.

**Fonctionnement :**

Rejette d'abord les boîtes sans intérieur commun, puis essaye croisements propres, sommets strictement intérieurs et sondes près des milieux d'arêtes. Les seules touches de contours ne suffisent pas. Ne calcule pas l'aire d'intersection et ne modifie pas le modèle ; les variantes ne doivent pas être fusionnées sans comparaison de leurs limites numériques.

Appels internes directs : [overlap_cross_double](arbre_appels.html#fn-overlap_cross_double), [ovmax_inside](arbre_appels.html#fn-ovmax_inside), [ovmax_probe](arbre_appels.html#fn-ovmax_probe).

<a id="fn-projected_polygons_overlap_v2_original"></a>

### projected_polygons_overlap_v2_original

Définition : [overlap_raster.c](overlap_raster.c), ligne 2904.

```c
static int projected_polygons_overlap_v2_original(Model3D *model, int f1, int f2)
```

**Rôle :**

Tester le chevauchement projeté par V2 original en double.

**Paramètres :**

[entrée] model et f1/f2 valides

**Retour :**

1 si un test trouve une zone commune, 0 sinon ou face trop courte.

**Fonctionnement :**

Rejette d'abord les boîtes sans intérieur commun, puis essaye croisements propres, sommets strictement intérieurs et sondes près des milieux d'arêtes. Les seules touches de contours ne suffisent pas. Ne calcule pas l'aire d'intersection et ne modifie pas le modèle ; les variantes ne doivent pas être fusionnées sans comparaison de leurs limites numériques.

Appels internes directs : [overlap_v2_edge_probes](arbre_appels.html#fn-overlap_v2_edge_probes), [overlap_v2_point_in_face](arbre_appels.html#fn-overlap_v2_point_in_face), [overlap_v2_proper_cross](arbre_appels.html#fn-overlap_v2_proper_cross).

<a id="fn-raster_advance_edge"></a>

### raster_advance_edge

Définition : [overlap_raster.c](overlap_raster.c), ligne 2464.

```c
static void raster_advance_edge( RasterPreparedEdge* e)
```

**Rôle :**

Avancer une arête préparée d'une ligne raster.

**Paramètres :**

[entrée-sortie] e.

**Retour :**

Aucun.

**Fonctionnement :**

Ajoute quotient et reste du pas, puis propage une unité au quotient si le reste atteint le diviseur positif.

Appels internes directs : aucun repéré.

<a id="fn-raster_edge_intersection"></a>

### raster_edge_intersection

Définition : [overlap_raster.c](overlap_raster.c), ligne 2498.

```c
static long raster_edge_intersection( const RasterPreparedEdge* e)
```

**Rôle :**

Lire l'intersection entière tronquée vers zéro d'une arête préparée.

**Paramètres :**

[entrée] e

**Retour :**

X doublé.

**Fonctionnement :**

Corrige le quotient euclidien négatif lorsqu'un reste subsiste, puis ajoute l'origine x1. Aucun effet de bord.

Appels internes directs : aucun repéré.

<a id="fn-raster_euclidean_divide"></a>

### raster_euclidean_divide

Définition : [overlap_raster.c](overlap_raster.c), ligne 2320.

```c
static void raster_euclidean_divide( long long numerator, long divisor, long* quotient, long* remainder)
```

**Rôle :**

Effectuer une division euclidienne avec reste positif ou nul.

**Paramètres :**

[entrée] numerator signé 64 bits, divisor strictement positif

**Retour :**

Aucun.

**Sorties et effets de bord :**

*quotient et *remainder en long.

**Fonctionnement :**

Corrige le quotient tronqué de C quand le reste est négatif ; le quotient final doit tenir dans long.

Appels internes directs : aucun repéré.

<a id="fn-raster_incremental_scanline"></a>

### raster_incremental_scanline

Définition : [overlap_raster.c](overlap_raster.c), ligne 2533.

```c
static int raster_incremental_scanline( RasterPreparedEdge* edges, int edge_count, int y, long* intersections)
```

**Rôle :**

Extraire et trier les intersections d'une ligne à partir d'arêtes préparées.

**Paramètres :**

[entrée-sortie] edges

[entrée] edge_count et y entier

**Retour :**

Nombre d'intersections.

**Sorties et effets de bord :**

intersections, capacité edge_count.

**Fonctionnement :**

Avance les arêtes actives d'un pas ; les appels doivent suivre la progression de lignes attendue.

Appels internes directs : [raster_advance_edge](arbre_appels.html#fn-raster_advance_edge), [raster_edge_intersection](arbre_appels.html#fn-raster_edge_intersection), [raster_sort_long](arbre_appels.html#fn-raster_sort_long).

<a id="fn-raster_intervals_share_pixel"></a>

### raster_intervals_share_pixel

Définition : [overlap_raster.c](overlap_raster.c), ligne 2249.

```c
static int raster_intervals_share_pixel( long a1, long a2, long b1, long b2)
```

**Rôle :**

Tester si deux intervalles ouverts contiennent un centre de pixel commun.

**Paramètres :**

[entrée] a1/a2 et b1/b2 en coordonnées doublées ; ordre quelconque des bornes.

**Retour :**

1 si leur intersection contient un entier impair strictement entre les deux limites, 0 sinon.

**Fonctionnement :**

Exclut les contacts sur les limites.

Appels internes directs : aucun repéré.

<a id="fn-raster_prepare_edges_incremental"></a>

### raster_prepare_edges_incremental

Définition : [overlap_raster.c](overlap_raster.c), ligne 2362.

```c
static int raster_prepare_edges_incremental( Model3D* model, int face, int scan_ymin, int scan_ymax, RasterPreparedEdge* edges)
```

**Rôle :**

Préparer les arêtes actives pour des lignes raster successives.

**Paramètres :**

[entrée] model, face, scan_ymin inclus et scan_ymax exclu

**Retour :**

Nombre préparé.

**Sorties et effets de bord :**

edges, capacité au moins le nombre d'arêtes.

**Fonctionnement :**

Travaille en coordonnées doublées, échantillonne aux centres y*2+1 et ignore les horizontales. Les quotients et restes permettent d'avancer sans redédivision.

Appels internes directs : [raster_euclidean_divide](arbre_appels.html#fn-raster_euclidean_divide).

<a id="fn-raster_sort_long"></a>

### raster_sort_long

Définition : [overlap_raster.c](overlap_raster.c), ligne 2210.

```c
static void raster_sort_long(long* values, int count)
```

**Rôle :**

Trier une courte liste d'intersections par insertion croissante.

**Paramètres :**

[entrée-sortie] values de longueur count.

**Retour :**

Aucun ; aucune allocation.

**Fonctionnement :**

Les valeurs égales conservent leur ordre de rencontre.

Appels internes directs : aucun repéré.

<a id="fn-ray_cast_at"></a>

### ray_cast_at

Définition : [engine.c](engine.c), ligne 4535.

```c
static int ray_cast_at(Model3D* model, int f1, int f2, int cx, int cy)
```

**Rôle :**

Comparer deux plans au passage d'un rayon de coordonnées écran entières.

**Paramètres :**

[entrée] model, f1/f2, cx/cy sans pan.

**Retour :**

+1 si f1 est plus loin que f2 (donc à dessiner avant), -1 si elle est plus proche, 0 en cas d'égalité ou d'échec.

**Fonctionnement :**

Cette convention est opposée à geometric_face_relation pour le sens avant. Ne vérifie pas si le point appartient réellement aux deux polygones.

Appels internes directs : [ray_cast_distances](arbre_appels.html#fn-ray_cast_distances).

<a id="fn-ray_cast_at_fractional"></a>

### ray_cast_at_fractional

Définition : [engine.c](engine.c), ligne 4569.

```c
static int ray_cast_at_fractional(Model3D *model, int f1, int f2, double cx, double cy)
```

**Rôle :**

Comparer deux plans au passage d'un rayon écran subpixel en double.

**Paramètres :**

[entrée] model, f1/f2 et cx/cy en pixels fractionnaires sans pan.

**Retour :**

+1 si f1 est plus loin, -1 si elle est plus proche, 0 sinon.

**Sorties et effets de bord :**

cache partagé de conversion des plans.

**Fonctionnement :**

Refuse l'échelle non positive et les dénominateurs quasi nuls. Ne teste pas l'appartenance aux contours.

Appels internes directs : [ordering_plane](arbre_appels.html#fn-ordering_plane).

<a id="fn-ray_cast_distances"></a>

### ray_cast_distances

Définition : [engine.c](engine.c), ligne 4475.

```c
static int ray_cast_distances(Model3D* model, int f1, int f2, int cx, int cy, float *out_tf1, float *out_tf2)
```

**Rôle :**

Calculer les paramètres d'intersection d'un rayon écran avec deux plans.

**Paramètres :**

[entrée] model, f1/f2 à base zéro, cx/cy en pixels sans pan

**Retour :**

1 si les deux dénominateurs sont utilisables, 0 pour arguments invalides ou quasi-parallélisme (seuil 1e-6).

**Sorties et effets de bord :**

*out_tf1 et *out_tf2.

peut remplir le cache de conversion des plans.

**Fonctionnement :**

Le rayon part de l'origine avec direction ((cx-Cx)/scale,(Cy-cy)/scale,1). Ne teste ni appartenance aux contours ni positivité des distances ; échelle non nulle requise.

Appels internes directs : [ordering_plane](arbre_appels.html#fn-ordering_plane).

<a id="fn-ray_cast_hierarchical"></a>

### ray_cast_hierarchical

Définition : [engine.c](engine.c), ligne 4879.

```c
static int ray_cast_hierarchical(Model3D* model, int f1, int f2)
```

**Rôle :**

Départager une paire avec grossissement temporaire et cache orienté.

**Paramètres :**

[entrée] model et f1/f2 valides.

**Retour :**

+1 f1 plus loin, -1 f2 plus loin, 0 si aucune méthode ne conclut.

**Fonctionnement :**

Ne change pas l'ordre des faces. Essaye un centroïde fractionnaire avec projection multipliée par dix. Sinon reprojette temporairement les seuls sommets de la paire pour le chemin entier, puis restaure coordonnées, bornes et échelle. Les résultats non nuls du chemin fractionnaire sont seuls mémorisés ici ; état partagé non réentrant.

Appels internes directs : [compute_intersection_centroid_fractional_tryboth](arbre_appels.html#fn-compute_intersection_centroid_fractional_tryboth), [ordering_pair_hit](arbre_appels.html#fn-ordering_pair_hit), [ordering_pair_slot](arbre_appels.html#fn-ordering_pair_slot), [ordering_pair_store](arbre_appels.html#fn-ordering_pair_store), [ray_cast_at_fractional](arbre_appels.html#fn-ray_cast_at_fractional), [ray_cast_hierarchical_impl](arbre_appels.html#fn-ray_cast_hierarchical_impl), [rch_patch_pair_at_scale](arbre_appels.html#fn-rch_patch_pair_at_scale), [rch_unpatch_pair](arbre_appels.html#fn-rch_unpatch_pair).

<a id="fn-ray_cast_hierarchical_impl"></a>

### ray_cast_hierarchical_impl

Définition : [engine.c](engine.c), ligne 4823.

```c
static int ray_cast_hierarchical_impl(Model3D* model, int f1, int f2)
```

**Rôle :**

Essayer plusieurs points représentatifs pour départager deux faces.

**Paramètres :**

[entrée] model, f1/f2.

**Retour :**

Signe du ray casting (+1 f1 avant, -1 f2 avant), ou 0 si aucune tentative ne tranche.

**Sorties et effets de bord :**

diagnostics et buffers de clipping partagés.

**Fonctionnement :**

Essaye le centroïde SH dans les deux sens, le centre de bbox de région QuickDraw, puis le centre des boîtes communes. Le dernier repli porte sur des boîtes et n'assure pas un point intérieur commun.

Appels internes directs : [compute_bbox_intersection_center](arbre_appels.html#fn-compute_bbox_intersection_center), [compute_intersection_centroid_ordered_fixed_tryboth](arbre_appels.html#fn-compute_intersection_centroid_ordered_fixed_tryboth), [compute_intersection_centroid_ordered_qd_fixed](arbre_appels.html#fn-compute_intersection_centroid_ordered_qd_fixed), [ray_cast_at](arbre_appels.html#fn-ray_cast_at).

<a id="fn-rch_gather_pair_vertex_indices"></a>

### rch_gather_pair_vertex_indices

Définition : [engine.c](engine.c), ligne 4633.

```c
static int rch_gather_pair_vertex_indices(FaceArrays3D* faces, int f1, int f2, int* out_idx)
```

**Rôle :**

Rassembler les identifiants distincts des sommets de deux faces.

**Paramètres :**

[entrée] faces, f1/f2.

**Retour :**

Nombre écrit.

**Sorties et effets de bord :**

out_idx à base zéro

**Préconditions :**

buffer de sortie assez grand pour la somme des longueurs. Déduplique par recherche linéaire en conservant l'ordre de première rencontre.

Appels internes directs : aucun repéré.

<a id="fn-rch_patch_pair_at_scale"></a>

### rch_patch_pair_at_scale

Définition : [engine.c](engine.c), ligne 4723.

```c
static void rch_patch_pair_at_scale(Model3D* model, int f1, int f2, Fixed32 test_scale, RCHZoomPatch* p)
```

**Rôle :**

Sauvegarder puis reprojeter temporairement les sommets de deux faces.

**Paramètres :**

[entrée] model, f1/f2 et test_scale en 16.16.

**Retour :**

Aucun.

**Sorties et effets de bord :**

*p, sauvegarde complète nécessaire à rch_unpatch_pair

**Fonctionnement :**

Modifie x2d/y2d, les deux boîtes et l'échelle globale.

**Préconditions :**

profondeurs non nulles et capacité de p suffisante pour les sommets distincts des deux faces. Aucune transformation 3D.

Appels internes directs : [FIXED_ROUND_TO_INT](arbre_appels.html#fn-FIXED_ROUND_TO_INT), [rch_gather_pair_vertex_indices](arbre_appels.html#fn-rch_gather_pair_vertex_indices), [rch_recompute_face_bbox_cache](arbre_appels.html#fn-rch_recompute_face_bbox_cache).

<a id="fn-rch_recompute_face_bbox_cache"></a>

### rch_recompute_face_bbox_cache

Définition : [engine.c](engine.c), ligne 4679.

```c
static void rch_recompute_face_bbox_cache(Model3D* model, int f)
```

**Rôle :**

Recalculer la boîte projetée d'une seule face après patch de projection.

**Paramètres :**

[entrée] model et f, face non vide valide.

**Retour :**

Aucun.

**Sorties et effets de bord :**

minx/maxx/miny/maxy de f, sans invalidation globale des caches.

**Fonctionnement :**

Cette routine étroite doit être utilisée avec sauvegarde et restauration cohérentes.

Appels internes directs : aucun repéré.

<a id="fn-rch_unpatch_pair"></a>

### rch_unpatch_pair

Définition : [engine.c](engine.c), ligne 4784.

```c
static void rch_unpatch_pair(Model3D* model, RCHZoomPatch* p, int f1, int f2)
```

**Rôle :**

Restaurer l'état temporairement remplacé par rch_patch_pair_at_scale.

**Paramètres :**

[entrée-sortie] model

[entrée] p, sauvegarde correspondante, et f1/f2.

**Retour :**

Aucun.

**Fonctionnement :**

Restaure coordonnées écran des sommets, bornes des deux faces et projection globale. Ne restaure pas d'autres métriques car elles n'ont pas changé.

Appels internes directs : aucun repéré.

<a id="fn-readFaces_model"></a>

### readFaces_model

Définition : [engine.c](engine.c), ligne 757.

```c
int readFaces_model(const char* filename, Model3D* model)
```

**Rôle :**

Lire les lignes OBJ « f » dans le buffer compact des faces.

**Paramètres :**

[entrée] filename et model déjà alloué ; readVertices_last_count sert de borne.

**Retour :**

Nombre de faces non vides, ou -1 sur structure invalide, ouverture impossible ou indice supérieur à la borne globale.

**Sorties et effets de bord :**

listes d'indices à base un, offsets, tailles, compteur total et ordre canonique.

**Fonctionnement :**

Les suffixes /vt/vn sont sautés ; les indices relatifs négatifs ne sont pas interprétés. Au plus 16 indices sont retenus.

**Limites :**

l'écriture du buffer compact ne contrôle pas ici sa capacité totale.

Appels internes directs : aucun repéré.

<a id="fn-ReadPalette"></a>

### ReadPalette

Définition : [engine.c](engine.c), ligne 6262.

```c
GS_Palette ReadPalette(int palette_num)
```

**Rôle :**

Lire seize mots de couleur à partir de l'adresse calculée pour une palette.

**Paramètres :**

[entrée] palette_num.

**Retour :**

GS_Palette copié par valeur ; aucune écriture matérielle.

**Fonctionnement :**

Attention : PALETTE_BASE est un pointeur unsigned int ; son addition avec palette_num*32 est donc mise à l'échelle par sizeof(unsigned int). Le chemin principal appelle cette fonction avec 0, où ce décalage est nul.

Appels internes directs : aucun repéré.

<a id="fn-compute_auto_fit"></a>

### compute_auto_fit

Définition : [engine.c](engine.c), ligne 558.

```c
static void compute_auto_fit(VertexArrays3D *vtx, int vertex_count, Fixed32 max_dim, Model3D *owner)
```

**Rôle :**

Calculer la distance et l'échelle suggérées pour la vue initiale.

**Paramètres :**

[entrée] vtx : sommets déjà centrés ; vertex_count : nombre positif. [entrée] max_dim : plus grande dimension objet en 16.16. [sortie] owner : distance, projection suggérée et drapeau auto_fit_ready.

**Retour :**

Aucun ; ne modifie ni les coordonnées ni les paramètres de la vue active.

**Fonctionnement :**

Distance de trois dimensions maximales, vue 30°/20°, marge de 90 %. Réutilise les produits fixes rapides et simplifie l'échelle commune des rapports de projection. Appelée par readVertices lors du chargement.

Appels internes directs : [cos_deg_int](arbre_appels.html#fn-cos_deg_int), [fixed_mul32_fast](arbre_appels.html#fn-fixed_mul32_fast), [sin_deg_int](arbre_appels.html#fn-sin_deg_int).

<a id="fn-readVertices"></a>

### readVertices

Définition : [engine.c](engine.c), ligne 645.

```c
int readVertices(const char* filename, VertexArrays3D* vtx, int max_vertices, Model3D* owner)
```

**Rôle :**

Lire les lignes OBJ « v x y z », centrer la géométrie et proposer une vue.

**Paramètres :**

[entrée] filename ; max_vertices, capacité de vtx ; owner, modèle associé ou NULL.

**Retour :**

Nombre de sommets lus, ou -1 si fopen échoue ; vertex_count reste à fixer par l'appelant.

**Sorties et effets de bord :**

vtx->x/y/z et, si owner est fourni, centrage et suggestions d'ajustement.

**Fonctionnement :**

Les axes OBJ y et z sont échangés à l'import. L'ajustement utilise la vue 30°/20°, une distance de trois fois la dimension maximale et une marge écran de 90 %. Ne lit ni normales ni coordonnées de texture.

**Optimisation de l’auto-fit :**

Les produits fixes de l’auto-fit utilisent fixed_mul32_fast avec la même réduction 16.16 que le produit 64 bits précédent. L’échelle commune est simplifiée dans xo/zo et yo/zo : quatre divisions flottantes par 65536 sont supprimées pour chaque sommet devant l’observateur. Les deux divisions finales, les angles 30°/20°, la marge de 90 % et la progression sont conservés.

Appels internes directs : [compute_auto_fit](arbre_appels.html#fn-compute_auto_fit), [keypress](arbre_appels.html#fn-keypress).

<a id="fn-remove_duplicates"></a>

### remove_duplicates

Définition : [engine.c](engine.c), ligne 1760.

```c
static void remove_duplicates(int *arr, int *n)
```

**Rôle :**

Retirer toutes les répétitions d'identifiants en conservant leur premier ordre.

**Paramètres :**

[entrée-sortie] arr et *n.

**Retour :**

Aucun.

**Fonctionnement :**

Parcours quadratique de la liste, sans allocation ; ne compare pas les coordonnées des sommets.

Appels internes directs : aucun repéré.

<a id="fn-renderModelFullscreenZBuffer"></a>

### renderModelFullscreenZBuffer

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 1596.

```c
void renderModelFullscreenZBuffer(Model3D* model)
```

**Rôle :**

Choisir la variante de rendu public selon le back-face culling.

**Paramètres :**

[entrée] model valide et vue préparée.

**Retour :**

Aucun.

**Fonctionnement :**

Avec culling, chemin sans biais ; sans culling, chemin qui rapproche les faces plane_d>0. Les buffers nécessaires doivent être initialisés avant cet appel.

Appels internes directs : [renderModelFullscreenZBuffer_biasedV2](arbre_appels.html#fn-renderModelFullscreenZBuffer_biasedV2), [renderModelFullscreenZBuffer_fastV2](arbre_appels.html#fn-renderModelFullscreenZBuffer_fastV2).

<a id="fn-renderModelFullscreenZBuffer_biasedV2"></a>

### renderModelFullscreenZBuffer_biasedV2

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 1569.

```c
void renderModelFullscreenZBuffer_biasedV2(Model3D* model)
```

**Rôle :**

Appeler ZBuffer_RenderCore avec rapprochement des faces de plane_d>0.

**Paramètres :**

[entrée] model valide avec données de vue préparées.

**Retour :**

Aucun.

**Fonctionnement :**

Reprend les préconditions, effets mémoire et règles de dessin du noyau appelé.

Appels internes directs : [ZBuffer_RenderCore](arbre_appels.html#fn-ZBuffer_RenderCore).

<a id="fn-renderModelFullscreenZBuffer_fastV2"></a>

### renderModelFullscreenZBuffer_fastV2

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 1543.

```c
void renderModelFullscreenZBuffer_fastV2(Model3D* model)
```

**Rôle :**

Appeler ZBuffer_RenderCore sans rapprochement des faces de plane_d>0.

**Paramètres :**

[entrée] model valide avec données de vue préparées.

**Retour :**

Aucun.

**Fonctionnement :**

Reprend les préconditions, effets mémoire et règles de dessin du noyau appelé.

Appels internes directs : [ZBuffer_RenderCore](arbre_appels.html#fn-ZBuffer_RenderCore).

<a id="fn-renderModelFullscreenZBuffer_offscreen"></a>

### renderModelFullscreenZBuffer_offscreen

Définition : [zbuffer_fullscreen_offscreen.c](zbuffer_fullscreen_offscreen.c), ligne 815.

```c
void renderModelFullscreenZBuffer_offscreen(Model3D* model)
```

**Rôle :**

Choisir la variante de rendu public selon le back-face culling.

**Paramètres :**

[entrée] model valide et vue préparée.

**Retour :**

Aucun.

**Fonctionnement :**

Avec culling, chemin sans biais ; sans culling, chemin qui rapproche les faces plane_d>0. Les buffers nécessaires doivent être initialisés avant cet appel.

Appels internes directs : [renderModelFullscreenZBuffer_offscreen_biasedV2](arbre_appels.html#fn-renderModelFullscreenZBuffer_offscreen_biasedV2), [renderModelFullscreenZBuffer_offscreen_fastV2](arbre_appels.html#fn-renderModelFullscreenZBuffer_offscreen_fastV2).

<a id="fn-renderModelFullscreenZBuffer_offscreen_biasedV2"></a>

### renderModelFullscreenZBuffer_offscreen_biasedV2

Définition : [zbuffer_fullscreen_offscreen.c](zbuffer_fullscreen_offscreen.c), ligne 788.

```c
void renderModelFullscreenZBuffer_offscreen_biasedV2(Model3D* model)
```

**Rôle :**

Appeler Off_RenderCore avec rapprochement des faces de plane_d>0.

**Paramètres :**

[entrée] model valide avec données de vue préparées.

**Retour :**

Aucun.

**Fonctionnement :**

Reprend les préconditions, effets mémoire et règles de dessin du noyau appelé.

Appels internes directs : [Off_RenderCore](arbre_appels.html#fn-Off_RenderCore).

<a id="fn-renderModelFullscreenZBuffer_offscreen_fastV2"></a>

### renderModelFullscreenZBuffer_offscreen_fastV2

Définition : [zbuffer_fullscreen_offscreen.c](zbuffer_fullscreen_offscreen.c), ligne 762.

```c
void renderModelFullscreenZBuffer_offscreen_fastV2(Model3D* model)
```

**Rôle :**

Appeler Off_RenderCore sans rapprochement des faces de plane_d>0.

**Paramètres :**

[entrée] model valide avec données de vue préparées.

**Retour :**

Aucun.

**Fonctionnement :**

Reprend les préconditions, effets mémoire et règles de dessin du noyau appelé.

Appels internes directs : [Off_RenderCore](arbre_appels.html#fn-Off_RenderCore).

<a id="fn-renderModelScanlineZBuffer"></a>

### renderModelScanlineZBuffer

Définition : [zbuffer_scanline.c](zbuffer_scanline.c), ligne 532.

```c
void renderModelScanlineZBuffer(Model3D* model)
```

**Rôle :**

Choisir la variante de rendu public selon le back-face culling.

**Paramètres :**

[entrée] model valide et vue préparée.

**Retour :**

Aucun.

**Fonctionnement :**

Avec culling, chemin sans biais ; sans culling, chemin qui rapproche les faces plane_d>0. Les deux chemins partagent le même moteur V3.

Appels internes directs : [renderModelScanlineZBuffer_biasedV3](arbre_appels.html#fn-renderModelScanlineZBuffer_biasedV3), [renderModelScanlineZBuffer_fastV3](arbre_appels.html#fn-renderModelScanlineZBuffer_fastV3).

<a id="fn-renderModelScanlineZBuffer_biasedV3"></a>

### renderModelScanlineZBuffer_biasedV3

Définition : [zbuffer_scanline.c](zbuffer_scanline.c), ligne 506.

```c
void renderModelScanlineZBuffer_biasedV3(Model3D* model)
```

**Rôle :**

Appeler renderModelScanlineZBuffer_coreV3 avec rapprochement des faces de plane_d>0.

**Paramètres :**

[entrée] model valide avec données de vue préparées.

**Retour :**

Aucun.

**Fonctionnement :**

Reprend les préconditions, effets mémoire et règles de dessin du noyau appelé.

Appels internes directs : [renderModelScanlineZBuffer_coreV3](arbre_appels.html#fn-renderModelScanlineZBuffer_coreV3).

<a id="fn-renderModelScanlineZBuffer_coreV3"></a>

### renderModelScanlineZBuffer_coreV3

Définition : [zbuffer_scanline.c](zbuffer_scanline.c), ligne 289.

```c
static void renderModelScanlineZBuffer_coreV3(Model3D* model, int biased)
```

**Rôle :**

Rendre les surfaces par balayage scanline V3 quantifié.

**Paramètres :**

[entrée] model valide avec projection et métriques à jour ; biased, active le rapprochement des faces de plane_d>0 lorsque le culling est désactivé.

**Retour :**

Aucun.

**Sorties et effets de bord :**

couleurs, mémoire Z, écran ou framebuffer, buffers statiques non réentrants.

**Fonctionnement :**

N’effectue ni transformation ni classement des faces ; les faces actives sont parcourues par identifiant pour définir les égalités de profondeur. Réserve/réutilise buffers par sommet/face/arête, calcule 1/z, prépare les arêtes, classe les débuts de face par histogramme puis balaie uniquement les faces actives. Ligne Z de 320 codes 16 bits ; arêtes à douze fractions, spans en 16.16 et petit nudge entier.

Le clipping du viewport est appliqué aux spans ; pas de clipping 3D général. Les réserves et le comportement d'échec restent propres à cette version.

Réutilise les réserves typées, les préparations de faces et d’arêtes et le noyau de spans commun. Conserve sa ligne Z, son échelle avec biais inclus et ses règles de contour.

Appels internes directs : [applyPalette](arbre_appels.html#fn-applyPalette), [screenClearTopLines](arbre_appels.html#fn-screenClearTopLines), [SL_ClearLine](arbre_appels.html#fn-SL_ClearLine), [SL_PaintSpan](arbre_appels.html#fn-SL_PaintSpan), [SL_SetScale](arbre_appels.html#fn-SL_SetScale), [ZBuffer_ActivateFaces](arbre_appels.html#fn-ZBuffer_ActivateFaces), [ZBuffer_LerpZ](arbre_appels.html#fn-ZBuffer_LerpZ), [ZBuffer_PrepareFaces](arbre_appels.html#fn-ZBuffer_PrepareFaces), [ZBuffer_PrepareInvZ](arbre_appels.html#fn-ZBuffer_PrepareInvZ), [ZBuffer_PrepareScanEdges](arbre_appels.html#fn-ZBuffer_PrepareScanEdges), [ZBuffer_QuantizeAtScale](arbre_appels.html#fn-ZBuffer_QuantizeAtScale), [ZBuffer_ReserveFaces](arbre_appels.html#fn-ZBuffer_ReserveFaces), [ZBuffer_ReserveVertices](arbre_appels.html#fn-ZBuffer_ReserveVertices), [ZBuffer_SortFaceStarts](arbre_appels.html#fn-ZBuffer_SortFaceStarts), [ZBuffer_SortScanHits](arbre_appels.html#fn-ZBuffer_SortScanHits).

<a id="fn-renderModelScanlineZBuffer_fastV3"></a>

### renderModelScanlineZBuffer_fastV3

Définition : [zbuffer_scanline.c](zbuffer_scanline.c), ligne 479.

```c
void renderModelScanlineZBuffer_fastV3(Model3D* model)
```

**Rôle :**

Appeler renderModelScanlineZBuffer_coreV3 sans rapprochement des faces de plane_d>0.

**Paramètres :**

[entrée] model valide avec données de vue préparées.

**Retour :**

Aucun.

**Fonctionnement :**

Reprend les préconditions, effets mémoire et règles de dessin du noyau appelé.

Appels internes directs : [renderModelScanlineZBuffer_coreV3](arbre_appels.html#fn-renderModelScanlineZBuffer_coreV3).

<a id="fn-reorder_poly"></a>

### reorder_poly

Définition : [engine.c](engine.c), ligne 1853.

```c
static void reorder_poly(Model3D* model, int fidx, int *list, int *n)
```

**Rôle :**

Réordonner une liste de sommets par angle autour de leur centre moyen projeté.

**Paramètres :**

[entrée] model et fidx, plan de référence ; *n, longueur.

[entrée-sortie] list, indices à base zéro.

**Retour :**

Aucun ; laisse la liste inchangée sur échec de réserve.

**Fonctionnement :**

Projection selon la normale dominante, atan2 puis tri par insertion des indices.

**Limites :**

le test d'aire utilise les tableaux px/py dans leur ordre d'origine, pas dans l'ordre des indices triés ; ce n'est pas une garantie générale de winding. Un tri angulaire n'est pas une reconstruction générale des contours concaves.

Appels internes directs : [compute_face_plane](arbre_appels.html#fn-compute_face_plane).

<a id="fn-repair_geov3"></a>

### repair_geov3

Définition : [engine.c](engine.c), ligne 7518.

```c
static void repair_geov3(Model3D* model)
```

**Rôle :**

Inspecter et corriger l'ordre courant par les échanges proposés par GEO V3.

**Paramètres :**

[entrée-sortie] model : modèle aux métriques de vue à jour ; NULL ignoré.

**Retour :**

Aucun ; affiche les nombres de propositions, d'échanges et de refus.

**Fonctionnement :**

Sauvegarde la visibilité puis lance geov3_run sans réinitialiser ni prétrier la permutation. Chaque échange est confirmé dans la vue filaire, dans un contexte QuickDraw maintenu pendant tous les tests. Échap conserve les échanges déjà acceptés ; N ignore la proposition courante ; Retour applique automatiquement les suivantes. Ne modifie ni les sommets, ni les paramètres de vue, ni le mode de tri. Les limites et paires non résolues de GEO V3 restent applicables : les propositions ne constituent pas une preuve d'ordre global sans erreur.

Utilise InspectionViewState pour la sauvegarde et la restauration communes des drapeaux et options ; le cycle de vie graphique reste géré par cet outil.

Appels internes directs : [DoText](arbre_appels.html#fn-DoText), [geov3_run](arbre_appels.html#fn-geov3_run), [inspection_view_begin](arbre_appels.html#fn-inspection_view_begin), [inspection_view_end](arbre_appels.html#fn-inspection_view_end), [keypress](arbre_appels.html#fn-keypress).

<a id="fn-repair_geov3_confirm"></a>

### repair_geov3_confirm

Définition : [engine.c](engine.c), ligne 7446.

```c
static int repair_geov3_confirm(Model3D *model, int p, int q, int pos_p, int pos_q, int reason, void *context)
```

**Rôle :**

Montrer les deux faces d'un échange GEO V3 et recueillir la décision.

**Paramètres :**

[entrée] model, p/q et pos_p/pos_q : état avant l'échange proposé. [entrée] reason : GEOV3_EXCHANGE_PLANES ou GEOV3_EXCHANGE_RAY. [entrée-sortie] context : GeoV3RepairUI, compteurs et mode automatique.

**Retour :**

-1 pour Échap, 0 pour N, 1 pour appliquer ; Retour active l'automatique.

**Fonctionnement :**

Dessine le modèle en fil de fer, P en vert clair et Q en orange, comme l'ancienne UI du repair. Conserve la géométrie et la permutation ; restaure visibilité et framePolyOnly avant de revenir au moteur. En automatique, valide sans ouvrir de vue graphique supplémentaire.

Utilise InspectionViewState pour la sauvegarde et la restauration communes des drapeaux et options ; le cycle de vie graphique reste géré par cet outil.

Appels internes directs : [drawFace](arbre_appels.html#fn-drawFace), [drawPolygons](arbre_appels.html#fn-drawPolygons), [drawPolygons_jitter](arbre_appels.html#fn-drawPolygons_jitter), [getkeypress](arbre_appels.html#fn-getkeypress), [inspection_view_restore](arbre_appels.html#fn-inspection_view_restore), [projected_polygons_overlap_cached](arbre_appels.html#fn-projected_polygons_overlap_cached), [screenClearTopLines](arbre_appels.html#fn-screenClearTopLines).

<a id="fn-restoreAllFaces"></a>

### restoreAllFaces

Définition : [engine.c](engine.c), ligne 7302.

```c
void restoreAllFaces(Model3D* model)
```

**Rôle :**

Restaurer les longueurs de toutes les faces précédemment masquées.

**Paramètres :**

[entrée-sortie] model valide.

**Retour :**

Aucun ; caches d'ordre invalidés.

**Fonctionnement :**

Ne rétablit pas directement les drapeaux d'affichage : la vue doit être recalculée.

Appels internes directs : [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate).

<a id="fn-restoreFace"></a>

### restoreFace

Définition : [engine.c](engine.c), ligne 7271.

```c
void restoreFace(Model3D* model, int face_idx)
```

**Rôle :**

Restaurer la longueur d'une face précédemment masquée.

**Paramètres :**

[entrée] model et face_idx valides.

**Retour :**

Aucun.

**Fonctionnement :**

Si sa longueur est zéro et une sauvegarde existe, la restaure puis efface la sauvegarde. Invalide les caches, mais ne remet pas display_flag à 1 et ne recalcule pas le plan.

Appels internes directs : [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate).

<a id="fn-restoreScreen"></a>

### restoreScreen

Définition : [screen_save_restore.c](screen_save_restore.c), ligne 173.

```c
void restoreScreen(void)
```

**Rôle :**

Restaurer la zone SHGR depuis la dernière sauvegarde.

**Paramètres :**

Aucun paramètre explicite.

**Entrées et sorties implicites :**

Entrée : buffer initialisé et contenant une image valide.

**Retour :**

Aucun ; graph_changed n'est pas modifié.

**Algorithme :**

MVN du buffer vers $E1:2000, avec banques patchées à l'exécution. P et DBR sont restaurés ; A, X et Y sont utilisés. Même contrainte de code auto-modifiant et de banque unique que saveScreen.

Appels internes directs : aucun repéré.

<a id="fn-reverseFaceVertexOrder"></a>

### reverseFaceVertexOrder

Définition : [engine.c](engine.c), ligne 7180.

```c
static void reverseFaceVertexOrder(Model3D* model, int face_idx)
```

**Rôle :**

Inverser le winding d'une face et son plan observateur.

**Paramètres :**

[entrée] model et face_idx à base zéro.

**Retour :**

Aucun ; arguments invalides ignorés.

**Sorties et effets de bord :**

liste inversée, quatre coefficients opposés, display_flag réévalué.

**Fonctionnement :**

Invalide les caches d'ordre ; les coordonnées et profondeurs ne changent pas. Ne retrie pas automatiquement les autres faces.

Appels internes directs : [ordering_cache_invalidate](arbre_appels.html#fn-ordering_cache_invalidate).

<a id="fn-run_raycast_test"></a>

### run_raycast_test

Définition : [engine.c](engine.c), ligne 7592.

```c
static int run_raycast_test(Model3D* model, int f1, int f2, int px, int py)
```

**Rôle :**

Adapter le résultat d'un rayon à la convention du diagnostic de paire.

**Paramètres :**

[entrée] model, f1/f2, px/py entiers sans pan.

**Retour :**

1 si f1 est plus proche, 2 si f2 est plus proche, 0 si indéterminé.

**Fonctionnement :**

Ce ne sont ni des indices de face ni les signes de geometric_face_relation. Peut remplir le cache des plans.

Appels internes directs : [ray_cast_at](arbre_appels.html#fn-ray_cast_at), [ray_cast_distances](arbre_appels.html#fn-ray_cast_distances).

<a id="fn-saveNextScreenshot"></a>

### saveNextScreenshot

Définition : [engine.c](engine.c), ligne 8763.

```c
void saveNextScreenshot(void)
```

**Rôle :**

Choisir le premier nom screen000.PIC à screen999.PIC qui ne peut être ouvert en lecture.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Fonctionnement :**

Appelle saveSHGRAsRawPic pour ce nom, ou affiche un échec si les mille noms sont utilisés. Un échec d'ouverture est traité comme un nom disponible ; la sélection n'est pas une réservation atomique de fichier.

Appels internes directs : [saveSHGRAsRawPic](arbre_appels.html#fn-saveSHGRAsRawPic).

<a id="fn-saveScreen"></a>

### saveScreen

Définition : [screen_save_restore.c](screen_save_restore.c), ligne 112.

```c
void saveScreen(void)
```

**Rôle :**

Copier la zone SHGR vers le buffer préalablement alloué.

**Paramètres :**

Aucun paramètre explicite.

**Entrées et sorties implicites :**

Entrée : banque et offset définis par initScreenSaveBuffer.

**Retour :**

Aucun.

**Sorties et effets de bord :**

buffer mis à jour et graph_changed remis à zéro.

**Algorithme :**

patcher les octets de banque du MVN puis transférer 32768 octets. P et DBR sont restaurés ; A, X et Y sont utilisés. Code auto-modifiant, non réentrant et nécessitant un segment exécutable accessible en écriture.

Appels internes directs : aucun repéré.

<a id="fn-saveSHGRAsRawPic"></a>

### saveSHGRAsRawPic

Définition : [engine.c](engine.c), ligne 8717.

```c
void saveSHGRAsRawPic(const char *filename)
```

**Rôle :**

Écrire la zone SHGR brute dans un fichier PIC non compressé.

**Paramètres :**

[entrée] filename

**Retour :**

Aucun.

**Fonctionnement :**

Lit SHGR_SIZE octets depuis SHGR_BASE, écrase le fichier et affecte le type ProDOS $C1, auxiliaire zéro. Échec d'ouverture affiché ; les erreurs d'écriture ne sont pas contrôlées ici.

Appels internes directs : [setProDOSFileType](arbre_appels.html#fn-setProDOSFileType).

<a id="fn-screen2Black"></a>

### screen2Black

Définition : [engine.c](engine.c), ligne 6427.

```c
void screen2Black()
```

**Rôle :**

Mettre à zéro les mots SHGR parcourus par la boucle d'effacement.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Fonctionnement :**

Écrit de $E1:9CFE vers $E1:2002 inclus. Le mot à $E1:2000 n'est pas atteint : la boucle sort lorsque X devient zéro.

**Préconditions :**

A et X sur 16 bits. A, X et les indicateurs ne sont pas sauvegardés.

Appels internes directs : aucun repéré.

<a id="fn-screenClearTopLines"></a>

### screenClearTopLines

Définition : [screen_save_restore.c](screen_save_restore.c), ligne 236.

```c
static void screenClearTopLines(int line_count)
```

**Rôle :**

Effacer les premières lignes de pixels de l'écran SHGR 320x200.

**Paramètres :**

[entrée] line_count : nombre de lignes depuis le haut ; une valeur négative ou nulle ne fait rien, une valeur supérieure à 200 vaut 200.

**Retour :**

Aucun.

**Sorties et effets de bord :**

Met à zéro 160 octets par ligne depuis $E1:2000. Les tables SCB et palettes ne sont pas modifiées. Ne gère pas graph_changed : l'appelant conserve la responsabilité du cycle de rendu et de sauvegarde SHGR.

**Fonctionnement :**

Borne la hauteur puis calcule l'offset du dernier mot. Le noyau 16 bits descend de deux en deux jusqu'au mot d'offset zéro inclus. L'offset maximal, 31998, reste positif pour le test BPL. Les accès longs ne dépendent pas de DBR. L'offset statique rend la routine non réentrante.

**Registres et indicateurs :**

Le bloc assembleur préserve P, A et X ; Y et DBR ne sont pas utilisés. Le code C de préparation reste soumis à la convention d'appel ORCA/C.

Appels internes directs : aucun repéré.

<a id="fn-segs_intersect_int"></a>

### segs_intersect_int

Définition : [overlap_raster.c](overlap_raster.c), ligne 521.

```c
static int segs_intersect_int(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4)
```

**Rôle :**

Tester une intersection propre de deux segments entiers.

**Paramètres :**

[entrée] x1/y1 et x2/y2 pour AB, x3/y3 et x4/y4 pour CD.

**Retour :**

1 si les extrémités sont strictement de côtés opposés dans les deux sens, 0 sinon.

**Fonctionnement :**

Exclut contacts d'extrémité et recouvrements colinéaires. Les déterminants emploient les produits 64 bits partagés du moteur.

Appels internes directs : [segs_intersect_int_fixed64](arbre_appels.html#fn-segs_intersect_int_fixed64).

<a id="fn-segs_intersect_int_fixed64"></a>

### segs_intersect_int_fixed64

Définition : [overlap_raster.c](overlap_raster.c), ligne 483.

```c
static int segs_intersect_int_fixed64( int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
```

**Rôle :**

Tester une intersection propre de deux segments entiers.

**Paramètres :**

[entrée] x1/y1 et x2/y2 pour AB, x3/y3 et x4/y4 pour CD.

**Retour :**

1 si les extrémités sont strictement de côtés opposés dans les deux sens, 0 sinon.

**Fonctionnement :**

Exclut contacts d'extrémité et recouvrements colinéaires. Les déterminants emploient les produits 64 bits partagés du moteur.

Appels internes directs : [orient_ll](arbre_appels.html#fn-orient_ll).

<a id="fn-select_overlap_method"></a>

### select_overlap_method

Définition : [overlap_raster.c](overlap_raster.c), ligne 3492.

```c
static int select_overlap_method(int current_method)
```

**Rôle :**

Demander un choix runtime de méthode sans modifier directement l'état global.

**Paramètres :**

[entrée] current_method, ramené à V1 si invalide.

**Retour :**

Méthode 0..6 ; entrée vide, zéro ou EOF conserve le choix.

**Sorties et effets de bord :**

terminal uniquement.

**Fonctionnement :**

Vérifie les caractères résiduels après strtol et redemande après erreur.

Appels internes directs : aucun repéré.

<a id="fn-setProDOSFileType"></a>

### setProDOSFileType

Définition : [engine.c](engine.c), ligne 8679.

```c
void setProDOSFileType(const char* filename, int fileType, long auxType)
```

**Rôle :**

Affecter les métadonnées GS/OS d'un fichier existant.

**Paramètres :**

[entrée] filename, nom tenant dans GSString255 ; fileType et auxType.

**Retour :**

Aucun.

**Fonctionnement :**

Construit FileInfoRecGS et appelle SetFileInfoGS avec accès $C3. Ne vérifie ni la longueur maximale ni le résultat Toolbox ; aucun contenu écrit.

Appels internes directs : aucun repéré.

<a id="fn-shgroff"></a>

### shgroff

Définition : [asm.h](asm.h), ligne 171.

```c
asm shgroff ()
```

**Rôle :**

Désactiver l'affichage SHGR en écrivant $41 dans $00C029.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Sorties et effets de bord :**

A et les indicateurs sont modifiés ; la routine termine avec A, X et Y configurés sur 16 bits.

Appels internes directs : aucun repéré.

<a id="fn-show_face_normal"></a>

### show_face_normal

Définition : [engine.c](engine.c), ligne 6803.

```c
static void show_face_normal(Model3D* model, int target_face)
```

**Rôle :**

Dessiner en jaune la normale de la face sélectionnée dans l'inspecteur.

**Paramètres :**

[entrée] model : modèle préparé dans le repère observateur. [entrée] target_face : identifiant valide de la face à inspecter.

**Retour :**

Aucun.

**Sorties et effets de bord :**

Modifie le motif du stylo et dessine dans le contexte graphique actif. Ne modifie pas le modèle ; utilise le pan et l'échelle de projection.

**Fonctionnement :**

Reprend les sommets sauvegardés des faces masquées, calcule leur centre moyen et construit un segment de normale de longueur 50. Projette ses extrémités selon les mêmes conditions que l'ancien bloc de showFace. Ce découpage réduit le nombre de labels internes générés par ORCA/C.

Appels internes directs : [FIXED_ROUND_TO_INT](arbre_appels.html#fn-FIXED_ROUND_TO_INT).

<a id="fn-show_help_pager"></a>

### show_help_pager

Définition : [engine.c](engine.c), ligne 8553.

```c
static void show_help_pager(void)
```

**Rôle :**

Afficher cycliquement les pages d'aide textuelle jusqu'à Échap.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Sorties et effets de bord :**

sortie texte et lecture clavier.

**Fonctionnement :**

Réserve l'espace de l'en-tête et du pied ; recommence à la première page après la dernière. Les chaînes d'aide existantes ne sont pas la référence de cette révision documentaire : certaines sont en retard sur le dispatch clavier.

Appels internes directs : [DoText](arbre_appels.html#fn-DoText), [getkeypress](arbre_appels.html#fn-getkeypress).

<a id="fn-showFace"></a>

### showFace

Définition : [engine.c](engine.c), ligne 6888.

```c
void showFace(Model3D* model, ObserverParams* params, const char* filename)
```

**Rôle :**

Explorer une face, sa normale et ses données détaillées dans une vue interactive.

**Paramètres :**

[entrée] model et params requis ; filename, contexte.

**Retour :**

Aucun.

**Fonctionnement :**

Les flèches naviguent par identifiant ou position triée ; N bascule la normale, Espace ouvre les informations et les commandes de masquage, restauration et inversion du winding. Peut modifier les listes de faces et leurs plans. Sauvegarde les options de rendu et les drapeaux ; l'inspection utilise QuickDraw et les entrées-sorties texte. La sauvegarde commune vérifie l'allocation avant d'ouvrir la vue.

Utilise InspectionViewState pour la sauvegarde et la restauration communes des drapeaux et options ; le cycle de vie graphique reste géré par cet outil.

Appels internes directs : [calculateFaceDepths](arbre_appels.html#fn-calculateFaceDepths), [DoText](arbre_appels.html#fn-DoText), [drawFace](arbre_appels.html#fn-drawFace), [drawFaceIndex](arbre_appels.html#fn-drawFaceIndex), [drawPolygons](arbre_appels.html#fn-drawPolygons), [drawPolygons_jitter](arbre_appels.html#fn-drawPolygons_jitter), [getkeypress](arbre_appels.html#fn-getkeypress), [hideFace](arbre_appels.html#fn-hideFace), [inspection_view_begin](arbre_appels.html#fn-inspection_view_begin), [inspection_view_end](arbre_appels.html#fn-inspection_view_end), [keypress](arbre_appels.html#fn-keypress), [restoreAllFaces](arbre_appels.html#fn-restoreAllFaces), [restoreFace](arbre_appels.html#fn-restoreFace), [reverseFaceVertexOrder](arbre_appels.html#fn-reverseFaceVertexOrder), [show_face_normal](arbre_appels.html#fn-show_face_normal).

<a id="fn-sin_deg_int"></a>

### sin_deg_int

Définition : [declare.c](declare.c), ligne 204.

```c
static inline Fixed32 sin_deg_int(int deg)
```

**Rôle :**

Lire le sinus d'un angle entier dans la table 16.16.

**Paramètres :**

[entrée] deg, degrés éventuellement négatifs ou hors tour.

**Retour :**

Sinus Fixed32.

**Fonctionnement :**

Ramène l'angle dans [0,359] ; aucune interpolation et aucun effet de bord.

Appels internes directs : aucun repéré.

<a id="fn-SL_ClearLine"></a>

### SL_ClearLine

Définition : [zbuffer_scanline.c](zbuffer_scanline.c), ligne 133.

```c
static void SL_ClearLine(void)
```

**Rôle :**

Réinitialiser la ligne Z réutilisée par le rendu scanline V3.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun ; écrit SCREEN_WIDTH mots $FFFF dans sl_zline.

**Fonctionnement :**

Les accès globaux longs évitent la dépendance à DBR.

**Registres et indicateurs :**

Préserve P, modifie A/X.

Appels internes directs : aucun repéré.

<a id="fn-SL_PaintSpan"></a>

### SL_PaintSpan

Définition : [zbuffer_scanline.c](zbuffer_scanline.c), ligne 215.

```c
static void SL_PaintSpan(int x0, int x1, int screenY, SLZWord z0, SLZWord z1, int fillColor, int frameColor, int nudge)
```

**Rôle :**

Préparer un span V3 qui utilise la seule ligne de profondeur courante.

**Paramètres :**

[entrée] x0/x1 inclus sans pan, screenY avec pan, z0/z1 aux extrémités, fillColor/frameColor et nudge en codes.

**Retour :**

Aucun.

**Fonctionnement :**

Span déjà clippé requis. Soustrait nudge avec borne zéro, prépare adresses et parité puis interpole Z en 16.16. Les deux extrémités utilisent la couleur de contour.

Appels internes directs : [ZBuffer_SpanRun](arbre_appels.html#fn-ZBuffer_SpanRun).

<a id="fn-SL_SetScale"></a>

### SL_SetScale

Définition : [zbuffer_scanline.c](zbuffer_scanline.c), ligne 177.

```c
static void SL_SetScale(float max_inv_z)
```

**Rôle :**

Adapter la quantification scanline en réservant de la marge pour le biais.

**Paramètres :**

[entrée] max_inv_z

**Retour :**

Aucun.

**Sorties et effets de bord :**

sl_zscale=cible/(max_inv_z+SL_Z_BIAS) si le dénominateur dépasse 1e-7.

**Fonctionnement :**

Sinon conserve l'échelle précédente.

Appels internes directs : aucun repéré.

<a id="fn-sort_by_zmax"></a>

### sort_by_zmax

Définition : [dony.c](dony.c), ligne 197.

```c
static void sort_by_zmax(int *sorted,Fixed32 *z_max,int nf)
```

**Rôle :**

Trier les indices de faces par profondeur maximale décroissante, de façon stable.

**Paramètres :**

[entrée] z_max, profondeurs par face ; nf, longueur de sorted.

[entrée-sortie] sorted, permutation d'identifiants à base zéro.

**Retour :**

Aucun.

**Algorithme :**

insertion binaire puis déplacement stable partagé ; les clés égales restent dans leur ordre initial. Une clé déjà à sa place évite la recherche.

Appels internes directs : [move_element_remove_and_insert](arbre_appels.html#fn-move_element_remove_and_insert).

<a id="fn-split_face_by_plane"></a>

### split_face_by_plane

Définition : [engine.c](engine.c), ligne 1435.

```c
static int split_face_by_plane(Model3D* model, int f1, int f2, int *out_newverts, int *out_n, int *constraint_verts, int constraint_n, int force_one_edge)
```

**Rôle :**

Découper expérimentalement f1 par le plan objet de f2.

**Paramètres :**

[entrée-sortie] model, dont sommets, faces et buffer d'indices peuvent croître.

[entrée] f1/f2 à base zéro ; constraint_verts et constraint_n, segment de contrainte facultatif à indices base zéro ; force_one_edge, permet de poursuivre sans sommets strictement des deux côtés.

**Retour :**

1 après réécriture de f1 et ajout éventuel des autres contours, 0 sinon.

**Sorties et effets de bord :**

out_newverts et out_n, nouveaux indices à base zéro, au plus MAX_FACE_VERTICES.

**Fonctionnement :**

Classe les sommets, crée les intersections, déduplique et trie les contours, puis réécrit les listes à base un et recalcule les bornes 3D. Ce traitement n'est pas transactionnel : de nouveaux sommets peuvent subsister après un retour 0. Les capacités de faces et d'indices ne sont pas contrôlées ici. Les deux fonctions de plan utilisées ont des échelles différentes ; les tolérances et contraintes doivent être réévaluées avant une rationalisation.

Appels internes directs : [add_vertex](arbre_appels.html#fn-add_vertex), [cleanup_list](arbre_appels.html#fn-cleanup_list), [compute_face_bboxes3D](arbre_appels.html#fn-compute_face_bboxes3D), [compute_face_plane](arbre_appels.html#fn-compute_face_plane), [poly_signed_area](arbre_appels.html#fn-poly_signed_area), [remove_duplicates](arbre_appels.html#fn-remove_duplicates), [reorder_poly](arbre_appels.html#fn-reorder_poly).

<a id="fn-strict_inside"></a>

### strict_inside

Définition : [engine.c](engine.c), ligne 1269.

```c
static int strict_inside(int px, int py, int nfj_local, int *vx_local, int *vy_local)
```

**Rôle :**

Tester l'appartenance stricte d'un point à un polygone entier.

**Paramètres :**

[entrée] px/py ; nfj_local, taille ; vx_local/vy_local, coordonnées des sommets.

**Retour :**

1 uniquement à l'intérieur hors contour, 0 sinon.

**Fonctionnement :**

Aucun effet de bord. Filtre d'abord par point_in_poly_arrays_int puis exclut explicitement les sommets et les points alignés situés sur une arête avec des produits 64 bits.

Appels internes directs : [point_in_poly_arrays_int](arbre_appels.html#fn-point_in_poly_arrays_int).

<a id="fn-updateFace2DBounds"></a>

### updateFace2DBounds

Définition : [engine.c](engine.c), ligne 5213.

```c
static void updateFace2DBounds(Model3D* model)
```

**Rôle :**

Recalculer les boîtes écran des faces après reprojection.

**Paramètres :**

[entrée-sortie] model ; NULL est accepté.

**Retour :**

Aucun.

**Fonctionnement :**

Invalide les caches d'ordre, parcourt les indices et écrit les bornes projetées. Ne recalcule ni plans, ni profondeur, ni display_flag.

Appels internes directs : aucun repéré.

<a id="fn-ZBFS_FreeEdges"></a>

### ZBFS_FreeEdges

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 473.

```c
static void ZBFS_FreeEdges(void)
```

**Rôle :**

Libérer les arêtes préparées.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Fonctionnement :**

Remet les pointeurs et leur capacité à zéro. Utilise uniquement les allocations du rendu plein écran direct.

Appels internes directs : aucun repéré.

<a id="fn-ZBFS_FreeFaces"></a>

### ZBFS_FreeFaces

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 419.

```c
static void ZBFS_FreeFaces(void)
```

**Rôle :**

Libérer les tables de faces, couleurs, biais et plans.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Fonctionnement :**

Remet les pointeurs et leur capacité à zéro. Utilise uniquement les allocations du rendu plein écran direct.

Appels internes directs : [ZBuffer_FreeFaces](arbre_appels.html#fn-ZBuffer_FreeFaces).

<a id="fn-ZBFS_FreeScratch"></a>

### ZBFS_FreeScratch

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 502.

```c
static void ZBFS_FreeScratch(void)
```

**Rôle :**

Libérer les trois familles de buffers auxiliaires du plein écran direct.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Fonctionnement :**

Centralise faces, sommets et arêtes. Ne libère ni le handle Z ni le framebuffer offscreen.

Appels internes directs : [ZBFS_FreeEdges](arbre_appels.html#fn-ZBFS_FreeEdges), [ZBFS_FreeFaces](arbre_appels.html#fn-ZBFS_FreeFaces), [ZBFS_FreeVertices](arbre_appels.html#fn-ZBFS_FreeVertices).

<a id="fn-ZBFS_FreeVertices"></a>

### ZBFS_FreeVertices

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 447.

```c
static void ZBFS_FreeVertices(void)
```

**Rôle :**

Libérer les profondeurs inverses et codes des sommets.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Fonctionnement :**

Remet les pointeurs et leur capacité à zéro. Utilise uniquement les allocations du rendu plein écran direct.

Appels internes directs : [ZBuffer_FreeVertices](arbre_appels.html#fn-ZBuffer_FreeVertices).

<a id="fn-ZBuffer_ActivateFaces"></a>

### ZBuffer_ActivateFaces

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 431.

```c
static int ZBuffer_ActivateFaces(int row, const int *bucket, const int *sorted, int *active, int count)
```

**Rôle :**

Ajouter les faces d’une ligne à la liste active ordonnée.

**Paramètres :**

[entrée] row, bucket et sorted : ligne et départs préparés. [entrée-sortie] active : buffer de faces actives ; count : longueur actuelle.

**Retour :**

Nouvelle longueur de la liste active.

**Fonctionnement :**

Insère chaque nouvelle face par identifiant croissant, sans modifier les bornes ni les règles de fin de vie des faces. Le buffer est dimensionné au nombre total de faces.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_Clear"></a>

### ZBuffer_Clear

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 161.

```c
void ZBuffer_Clear(void)
```

**Rôle :**

Remplir de $FFFF les 200 lignes de profondeur.

**Paramètres :**

Aucun paramètre explicite.

**Entrées et sorties implicites :**

Entrée : zbuf_row préparé.

**Retour :**

Aucun.

**Registres et indicateurs :**

Préserve P, DBR et D ; A/X/Y servent au transfert. Cadre D de six octets : offset de table à 0, pointeur de ligne à 2/4 ; D pointe S+1 après réserve. Chemin rapide par blocs de seize mots avec DBR de la ligne ; chemin [dp],Y pour une ligne traversant une banque. Largeur multiple de 16 requise. Les accès |offset,X imposent le mode absolu, pas la page directe.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_DrawEdge"></a>

### ZBuffer_DrawEdge

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 1103.

```c
static void ZBuffer_DrawEdge(int x0, int y0, UWORD16 edge_z0, int x1, int y1, UWORD16 edge_z1, int color, ZbPlane *plane)
```

**Rôle :**

Rasteriser un côté par Bresenham et tester sa visibilité après les surfaces.

**Paramètres :**

[entrée] x0/y0, edge_z0 et x1/y1, edge_z1 en coordonnées écran et codes de sommets, color et plane commun.

**Retour :**

Aucun.

**Fonctionnement :**

Trie les extrémités selon l'axe majeur, clippe son parcours et avance à la fois le plan Z et la profondeur d'arête. Prépare un paquet par côté pour ZBuffer_EdgeRun ; aucun appel C par pixel ni allocation. Les règles de tie de Bresenham peuvent différer de FramePoly et du remplissage demi-ouvert. Sans clipping initial sur l’axe majeur, les restes de départ sont connus directement. Le quotient et le reste de profondeur utilisent des mots non signés de 16 bits ; le chemin clippé conserve ses calculs long.

Appels internes directs : [ZBuffer_EdgeRun](arbre_appels.html#fn-ZBuffer_EdgeRun), [ZBuffer_PlaneAt](arbre_appels.html#fn-ZBuffer_PlaneAt).

<a id="fn-ZBuffer_FreeFaces"></a>

### ZBuffer_FreeFaces

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 190.

```c
static void ZBuffer_FreeFaces(ZBufferFaceWork *work)
```

**Rôle :**

Libérer les tables communes de préparation des faces.

**Paramètres :**

[entrée-sortie] work : réserve privée du moteur.

**Retour :**

Aucun.

**Fonctionnement :**

Libère débuts, ordre, liste active, couleurs et nudges, puis neutralise la réserve. Les plans du direct et les liaisons offscreen restent possédés par leurs moteurs.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_FreeVertices"></a>

### ZBuffer_FreeVertices

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 123.

```c
static void ZBuffer_FreeVertices(ZBufferVertexWork *work)
```

**Rôle :**

Libérer une réserve de profondeurs de sommets.

**Paramètres :**

[entrée-sortie] work : réserve possédée par un moteur.

**Retour :**

Aucun.

**Fonctionnement :**

Libère les trois buffers et neutralise tous les pointeurs et la capacité. Une réserve vide peut être libérée à nouveau.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_Init"></a>

### ZBuffer_Init

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 63.

```c
int ZBuffer_Init(void)
```

**Rôle :**

Réserver le Z-buffer plein écran aligné sur une frontière de banque.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

1 si déjà initialisé ou allocation réussie, 0 sinon.

**Fonctionnement :**

Demande 192 Kio pour garantir deux banques utilisables après alignement 64 Kio. Prépare zbuf_row pour 200 lignes de 320 mots puis efface les profondeurs. Le handle fixe reste possédé par le module jusqu'à ZBuffer_Shutdown.

Appels internes directs : [ZBuffer_Clear](arbre_appels.html#fn-ZBuffer_Clear).

<a id="fn-ZBuffer_LerpZ"></a>

### ZBuffer_LerpZ

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 52.

```c
static UWORD16 ZBuffer_LerpZ(UWORD16 za, UWORD16 zb, int xa, int xb, int x)
```

**Rôle :**

Interpoler un code de profondeur lors du clipping horizontal d'un span.

**Paramètres :**

[entrée] za/zb : codes de profondeur aux abscisses xa/xb. [entrée] xa/xb : abscisses des extrémités ; x : abscisse demandée.

**Retour :**

za si xa == xb ; sinon za + (zb-za)*(x-xa)/(xb-xa), converti en UWORD16.

**Fonctionnement :**

Conserve les opérations en long et la troncature de la division entière des anciennes variantes scanline et offscreen. Aucun état modifié.

**Limites :**

Aucune saturation ; les différences et produits doivent être valides dans les types intermédiaires. L'appelant choisit le domaine de x.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_PaintSpanZ"></a>

### ZBuffer_PaintSpanZ

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 675.

```c
static void ZBuffer_PaintSpanZ(int x0, int x1, int screenY, int fillColor, ZbPlane *plane, unsigned long row_raw)
```

**Rôle :**

Préparer un span de surface évalué sur le plan Z commun de la face.

**Paramètres :**

[entrée] x0/x1 inclus sans pan, screenY avec pan, fillColor et plane.

**Retour :**

Aucun ; écrit profondeur et écran via ZBuffer_SpanRun.

**Préconditions :**

coordonnées déjà clippées et lignes valides. Prépare codes exacts aux extrémités et pas 16.16 depuis le gradient à douze fractions.

**Optimisation :**

Le plan de ligne row_raw est fourni en x écran nul et avancé de gy par ligne ; une seule multiplication par gx localise le début du span, sans changer les codes Z.

Appels internes directs : [ZBuffer_PlaneCode](arbre_appels.html#fn-ZBuffer_PlaneCode), [ZBuffer_SpanRun](arbre_appels.html#fn-ZBuffer_SpanRun).

<a id="fn-ZBuffer_PlaneAt"></a>

### ZBuffer_PlaneAt

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 611.

```c
static unsigned long ZBuffer_PlaneAt(ZbPlane *p, long x, long y)
```

**Rôle :**

Évaluer le plan Z à un pixel écran sans encore réduire ses fractions.

**Paramètres :**

[entrée] p, x/y en pixels incluant pan.

**Retour :**

Accumulateur modulo 2^32, avec douze bits fractionnaires.

**Fonctionnement :**

Aucun effet de bord ; saturation faite séparément.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_PlaneCode"></a>

### ZBuffer_PlaneCode

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 641.

```c
static UWORD16 ZBuffer_PlaneCode(unsigned long z)
```

**Rôle :**

Arrondir et borner un accumulateur Z à douze bits fractionnaires.

**Paramètres :**

[entrée] z, représentation modulo 2^32.

**Retour :**

Code 16 bits ; valeur avec bit de signe après ajout du demi-pas donne 0 ; débordement positif donne $FFFF.

**Fonctionnement :**

Aucun effet de bord.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_EdgeRun"></a>

### ZBuffer_EdgeRun

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 782.

```c
static void ZBuffer_EdgeRun(void)
```

**Rôle :**

Parcourir une arête préparée et écrire ses pixels visibles en SHGR.

**Paramètres :**

Aucun paramètre explicite ; lit zbfs_edge_packet, préparé par DrawEdge.

**Retour :**

Aucun ; met à jour les profondeurs et les nibbles des pixels acceptés.

**Fonctionnement :**

Copie le paquet de 50 octets dans un cadre direct de 52 octets, puis avance Bresenham, les profondeurs et les pointeurs sans appel C par pixel. Les profondeurs égales restent dessinables ; un contour strictement plus proche doit aussi passer le test de profondeur géométrique de l'arête. Vérifie les coordonnées avant tout accès mémoire. Les sommes de restes propagent la retenue pour les arêtes dont l'axe majeur dépasse 32767.

**Registres et contraintes :**

Préserve P et D ; DBR inchangé, A/X/Y volatils. Accumulateur 16 bits sauf pendant l'écriture du nibble. Offsets 0..49 : paquet ; 50 : code Z local. Pointeurs far avancés sur 32 bits pour franchir les banques du Z-buffer. Appel synchrone et non réentrant ; count positif, major/minor <= 65535.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_PrepareFaces"></a>

### ZBuffer_PrepareFaces

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 348.

```c
static void ZBuffer_PrepareFaces(FaceArrays3D *faces, int count, int y_lo, int y_hi, int height, int biased, int *start, unsigned char *fill, unsigned char *frame, unsigned char *nudge, int *bucket)
```

**Rôle :**

Filtrer les faces et préparer leurs premières lignes et couleurs.

**Paramètres :**

[entrée] faces/count : faces de la vue ; y_lo/y_hi : bornes verticales ; height : hauteur écran ; biased : biais actif.

**[sortie] start, fill, frame, nudge : buffers par face ; bucket :**

histogramme de height+1 entiers.

**Retour :**

Aucun.

**Fonctionnement :**

Initialise l’histogramme, ignore les faces masquées, courtes ou hors écran, puis conserve les règles de couleurs et le nudge de deux unités pour les faces avant biaisées. Les arêtes restent préparées par les moteurs ou le helper dédié.

Appels internes directs : [getFaceFillColor](arbre_appels.html#fn-getFaceFillColor), [getFaceFrameColor](arbre_appels.html#fn-getFaceFrameColor).

<a id="fn-ZBuffer_PrepareInvZ"></a>

### ZBuffer_PrepareInvZ

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 303.

```c
static float ZBuffer_PrepareInvZ(VertexArrays3D *vtx, int count, float *inv_z)
```

**Rôle :**

Préparer les profondeurs inverses des sommets d’une vue.

**Paramètres :**

[entrée] vtx : tableaux de sommets ; count : nombre de sommets. [sortie] inv_z : buffer de count flottants déjà alloué.

**Retour :**

Maximum des profondeurs inverses, ou zéro.

**Fonctionnement :**

Calcule directement 65536/(float)zo pour les profondeurs 16.16 positives et écrit zéro sinon. La simplification par une puissance de deux garde les mêmes arrondis. La plus petite profondeur fixe positive fournit le maximum des inverses sans comparaison flottante supplémentaire.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_PrepareScanEdges"></a>

### ZBuffer_PrepareScanEdges

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 470.

```c
static void ZBuffer_PrepareScanEdges(VertexArrays3D *vtx, FaceArrays3D *faces, int face, const UWORD16 *zsrc, int y_lo, long rounding, ZBufferScanEdge *ed)
```

**Rôle :**

Préparer les arêtes quantifiées communes à scanline et offscreen.

**Paramètres :**

[entrée] vtx, faces et face : contour projeté ; zsrc : codes Z par sommet ; y_lo : première ligne ; rounding : biais d’arrondi. [sortie] ed : buffer de vertex_count[face] arêtes.

**Retour :**

Aucun.

**Fonctionnement :**

Conserve le format 12 bits fractionnaires, les arêtes horizontales neutralisées, les divisions signées et le clipping vertical des anciens moteurs. Le rendu direct garde son calcul de plan et ses arêtes propres.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_QuantizeAtScale"></a>

### ZBuffer_QuantizeAtScale

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 267.

```c
static UWORD16 ZBuffer_QuantizeAtScale(float inv_z, float scale)
```

**Rôle :**

Quantifier une profondeur inverse avec une échelle explicite.

**Paramètres :**

[entrée] inv_z : profondeur inverse ; scale : échelle du moteur appelant.

**Retour :**

Code 16 bits inversé et saturé ; 65535 représente le fond.

**Fonctionnement :**

Conserve les opérations float, l’arrondi par +0,5 puis la conversion en long des deux anciens quantificateurs. Ne modifie aucune échelle.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_QuantizeInvZ"></a>

### ZBuffer_QuantizeInvZ

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 359.

```c
UWORD16 ZBuffer_QuantizeInvZ(float inv_z)
```

**Rôle :**

Quantifier une profondeur inverse en code 16 bits où petit signifie proche.

**Paramètres :**

[entrée] inv_z, profondeur inverse flottante ; valeurs négatives ramenées à zéro.

**Retour :**

65535 - round(inv_z*échelle), après saturation de la valeur quantifiée dans [0,65535]. $FFFF est la valeur vide/lointaine.

**Fonctionnement :**

Aucun changement d'échelle.

Appels internes directs : [ZBuffer_QuantizeAtScale](arbre_appels.html#fn-ZBuffer_QuantizeAtScale).

<a id="fn-ZBuffer_RenderCore"></a>

### ZBuffer_RenderCore

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 1255.

```c
static void ZBuffer_RenderCore(Model3D* model, int biased)
```

**Rôle :**

Rendre les surfaces par balayage plein écran direct.

**Paramètres :**

[entrée] model valide avec projection et métriques à jour ; biased, active le rapprochement des faces de plane_d>0 lorsque le culling est désactivé.

**Retour :**

Aucun.

**Sorties et effets de bord :**

couleurs, mémoire Z, écran ou framebuffer, buffers statiques non réentrants.

**Fonctionnement :**

N’effectue ni transformation ni classement des faces ; les faces actives sont parcourues par identifiant pour définir les égalités de profondeur. Réserve/réutilise buffers par sommet/face/arête, calcule 1/z, prépare les arêtes, classe les débuts de face par histogramme puis balaie uniquement les faces actives. Plan Z commun au remplissage et aux contours ; arrondi intérieur ceil/floor, puis côtés originaux par Bresenham et masque facultatif.

Le clipping du viewport est appliqué aux spans ; pas de clipping 3D général. Les réserves et le comportement d'échec restent propres à cette version.

Les réserves, inverses de profondeur, filtres de faces, histogrammes et listes actives passent par les primitives communes. Le plan de profondeur et les contours du rendu direct restent distincts.

**Optimisation :**

Le remplissage avance le plan de chaque face par addition de gy à chaque ligne, y compris sans span ; les contours conservent leur calcul initial.

Appels internes directs : [applyPalette](arbre_appels.html#fn-applyPalette), [screenClearTopLines](arbre_appels.html#fn-screenClearTopLines), [ZBFS_FreeEdges](arbre_appels.html#fn-ZBFS_FreeEdges), [ZBFS_FreeFaces](arbre_appels.html#fn-ZBFS_FreeFaces), [ZBuffer_ActivateFaces](arbre_appels.html#fn-ZBuffer_ActivateFaces), [ZBuffer_Clear](arbre_appels.html#fn-ZBuffer_Clear), [ZBuffer_DrawEdge](arbre_appels.html#fn-ZBuffer_DrawEdge), [ZBuffer_PaintSpanZ](arbre_appels.html#fn-ZBuffer_PaintSpanZ), [ZBuffer_PlaneAt](arbre_appels.html#fn-ZBuffer_PlaneAt), [ZBuffer_PrepareFaces](arbre_appels.html#fn-ZBuffer_PrepareFaces), [ZBuffer_PrepareInvZ](arbre_appels.html#fn-ZBuffer_PrepareInvZ), [ZBuffer_QuantizeInvZ](arbre_appels.html#fn-ZBuffer_QuantizeInvZ), [ZBuffer_ReserveFaces](arbre_appels.html#fn-ZBuffer_ReserveFaces), [ZBuffer_ReserveVertices](arbre_appels.html#fn-ZBuffer_ReserveVertices), [ZBuffer_SetPlane](arbre_appels.html#fn-ZBuffer_SetPlane), [ZBuffer_SetScaleForFrame](arbre_appels.html#fn-ZBuffer_SetScaleForFrame), [ZBuffer_SizeFits](arbre_appels.html#fn-ZBuffer_SizeFits), [ZBuffer_SortFaceStarts](arbre_appels.html#fn-ZBuffer_SortFaceStarts).

<a id="fn-ZBuffer_ReserveFaces"></a>

### ZBuffer_ReserveFaces

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 219.

```c
static int ZBuffer_ReserveFaces(ZBufferFaceWork *work, int count)
```

**Rôle :**

Garantir les six tables communes de faces.

**Paramètres :**

[entrée-sortie] work : réserve privée ; count : nombre de faces.

**Retour :**

1 si les tables sont disponibles, 0 sinon.

**Fonctionnement :**

Réutilise la capacité suffisante, vérifie les tailles puis alloue les six tables. Sur échec, libère le jeu incomplet et laisse une réserve vide pour le prochain essai.

Appels internes directs : [ZBuffer_FreeFaces](arbre_appels.html#fn-ZBuffer_FreeFaces), [ZBuffer_SizeFits](arbre_appels.html#fn-ZBuffer_SizeFits).

<a id="fn-ZBuffer_ReserveVertices"></a>

### ZBuffer_ReserveVertices

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 152.

```c
static int ZBuffer_ReserveVertices(ZBufferVertexWork *work, int count)
```

**Rôle :**

Garantir la capacité des trois buffers de profondeur.

**Paramètres :**

[entrée-sortie] work : réserve privée du moteur ; count : capacité requise.

**Retour :**

1 si disponible, 0 si taille invalide ou échec d’allocation.

**Fonctionnement :**

Réutilise la réserve suffisante. Sinon libère l’ancienne, vérifie les tailles puis alloue les trois buffers ; un échec laisse une réserve entièrement vide.

Appels internes directs : [ZBuffer_FreeVertices](arbre_appels.html#fn-ZBuffer_FreeVertices), [ZBuffer_SizeFits](arbre_appels.html#fn-ZBuffer_SizeFits).

<a id="fn-ZBuffer_SetEdgeFlags"></a>

### ZBuffer_SetEdgeFlags

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 733.

```c
void ZBuffer_SetEdgeFlags(const unsigned char *flags, unsigned long count)
```

**Rôle :**

Installer un masque facultatif des côtés de faces, sans prendre sa propriété.

**Paramètres :**

[entrée] flags, un octet par entrée du buffer d'indices ; count, longueur.

**Retour :**

Aucun.

**Fonctionnement :**

Zéro masque un côté, non-zéro l'autorise ; NULL rétablit tous les côtés. Le buffer doit rester vivant pendant le rendu qui l'utilise.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_SetPlane"></a>

### ZBuffer_SetPlane

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 542.

```c
static void ZBuffer_SetPlane(ZbPlane *p, VertexArrays3D *vtx, FaceArrays3D *faces, int f, UWORD16 *zsrc, int nudge, float code_bias)
```

**Rôle :**

Préparer un plan affine de codes Z dans le repère écran.

**Paramètres :**

[entrée] vtx/faces : coordonnées et plans observateur à jour ; f valide. zsrc : codes des sommets pour le repli dégénéré ; nudge : rapprochement en codes ; code_bias : biais déjà converti en codes, nul sans rapprochement.

**Retour :**

Aucun ; écrit *p, avec pan et gradients à douze bits fractionnaires.

**Fonctionnement :**

Réutilise le plan observateur de Newell calculé par calculateFaceDepths : pour le rayon écran (u,v,1), 1/z = -(a*u+b*v+c)/d. Les coordonnées projetées arrondies servent uniquement à choisir l’ancrage, jamais à reconstruire les gradients. Un plan passant par l’observateur ou une projection non positive conserve la profondeur constante de repli. Le biais et la quantification sont appliqués après le calcul du plan. Les coefficients conservent leur échelle entière commune, qui se simplifie dans les rapports : les conversions intermédiaires en double et les divisions par 65536 sont supprimées. L’inverse de projection et le biais en codes sont préparés une fois par vue ; il reste une division flottante par face.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_SetScaleForFrame"></a>

### ZBuffer_SetScaleForFrame

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 329.

```c
void ZBuffer_SetScaleForFrame(float max_inv_z)
```

**Rôle :**

Adapter l'échelle de quantification à la profondeur inverse maximale.

**Paramètres :**

[entrée] max_inv_z.

**Retour :**

Aucun.

**Sorties et effets de bord :**

zbuffer_scale, cible/max_inv_z si max_inv_z > 1e-7 ; sinon conserve l'échelle précédente.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_Shutdown"></a>

### ZBuffer_Shutdown

Définition : [zbuffer_fullscreen.c](zbuffer_fullscreen.c), ligne 118.

```c
void ZBuffer_Shutdown(void)
```

**Rôle :**

Libérer les ressources du rendu plein écran direct.

**Paramètres :**

Aucun paramètre explicite.

**Retour :**

Aucun.

**Fonctionnement :**

Libère scratch et handle Z, annule lignes et banques. Oublie le masque d'arêtes emprunté sans le libérer. Appel répété accepté. Ne libère pas les allocations statiques propres aux autres moteurs de rendu.

Appels internes directs : [ZBFS_FreeScratch](arbre_appels.html#fn-ZBFS_FreeScratch), [ZBuffer_SetEdgeFlags](arbre_appels.html#fn-ZBuffer_SetEdgeFlags).

<a id="fn-ZBuffer_SizeFits"></a>

### ZBuffer_SizeFits

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 96.

```c
static int ZBuffer_SizeFits(int count, size_t element_size)
```

**Rôle :**

Vérifier qu'un tableau de count éléments est représentable par size_t.

**Paramètres :**

[entrée] count positif attendu et element_size non nul.

**Retour :**

Booléen.

**Fonctionnement :**

La division du maximum size_t évite de déborder lors du produit de taille.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_SortFaceStarts"></a>

### ZBuffer_SortFaceStarts

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 396.

```c
static void ZBuffer_SortFaceStarts(int count, int height, const int *start, int *bucket, int *cursor, int *sorted)
```

**Rôle :**

Transformer l’histogramme en liste stable des départs de faces.

**Paramètres :**

[entrée] count, height et start : dimensions et première ligne par face.

**[entrée-sortie] bucket : histogramme devenant offsets ; cursor :**

workspace ; sorted : liste de sortie.

**Retour :**

Aucun.

**Fonctionnement :**

Effectue les sommes préfixes puis distribue les faces par identifiant croissant. Préserve les égalités de profondeur et l’ordre d’activation des anciens parcours.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_SortScanHits"></a>

### ZBuffer_SortScanHits

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 537.

```c
static void ZBuffer_SortScanHits(ZBufferScanHit *hits, int hit_count)
```

**Rôle :**

Ordonner les intersections quantifiées d’un span.

**Paramètres :**

[entrée-sortie] hits : tableau de count intersections ; count : longueur valide.

**Retour :**

Aucun.

**Fonctionnement :**

Échange les deux intersections si nécessaire, sinon utilise l’insertion stable selon x. Les codes Z restent attachés à leurs abscisses.

Appels internes directs : aucun repéré.

<a id="fn-ZBuffer_SpanRun"></a>

### ZBuffer_SpanRun

Définition : [zbuffer_common.c](zbuffer_common.c), ligne 606.

```c
static void ZBuffer_SpanRun(void)
```

**Rôle :**

Dessiner un span préclippé avec test Z strict dans un noyau 65C816.

**Paramètres :**

Aucun paramètre explicite.

**Entrées et sorties implicites :**

Entrée : paquet partagé zspan_p_* : pointeurs Z et pixels far, parité gauche, codes gauche/droite, couleurs gauche/intérieur/droite, accumulateur et pas 16.16, nombre de pixels intérieurs et mode 0 (gauche seul) ou 1 (span complet).

**Retour :**

Aucun ; écrit Z et les nibbles de couleur.

**Registres et indicateurs :**

Préserve P et D ; A/X/Y modifiés, DBR inchangé. Mode décimal supposé désactivé. Cadre D de 32 octets : zacc à 0/2, zstep à 4/6, zptr à 8/10, pptr à 12/14, compte à 16, couleur basse/haute à 18/20, parité à 22, masque/valeur à 24/26. Les profondeurs [8],Y peuvent franchir une banque ; le pointeur de pixels n'est incrémenté que sur 16 bits et doit rester dans sa banque. Traite les extrémités exactement et l'intérieur par paires pair/impair. Les pixels de même code Z ne sont pas remplacés. Non réentrant.

**Optimisation :**

Quand les deux pixels intérieurs passent le test Z strict, leur octet couleur est écrit en une fois ; sinon seul le nibble visible change. Le mot 28 du cadre direct contient la couleur des deux pixels.

Appels internes directs : aucun repéré.
