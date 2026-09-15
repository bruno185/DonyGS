# Dictionnaire des fonctions – OBJExplorer

*(ordre alphabétique, liste complète)*

| Fonction | Description |
|----------|-------------|
| **add_vertex** | Ajoute un nouveau sommet aux tableaux du modèle et retourne son index 0-based. Retourne -1 si la capacité est dépassée. |
| **applyPalette** | Charge et active une palette QuickDraw parmi celles initialisées. |
| **calculateFaceDepths** | Calcule pour chaque face z_min, z_max, z_mean, les coefficients de plan et les bbox 2D. |
| **check_intersect** | Détecte les interpénétrations 3D entre faces et peut découper les polygones en conflit via split_face_by_plane. |
| **check_sort_repair** | Vérifie l'ordre du tri painter et effectue des réparations minimales (ray-cast + déplacements). Mode interactif possible. |
| **check_sort_repair_fast** | Variante rapide de check_sort_repair utilisant le centroïde QuickDraw comme point de test. |
| **cleanup_list** | Supprime les indices consécutifs en double dans une liste de sommets de polygone (y compris le wrap-around). |
| **clip_face_plane** | Clippe le polygone d'une face contre un demi-espace défini par un plan ; produit une nouvelle liste de sommets. |
| **cmp_faces_by_zmean** | Comparateur pour qsort : ordonne les faces par z_mean décroissant (tie-break sur l'index). |
| **compare_faces_diagnostic** | Exécute la batterie complète de tests (Z, bbox, overlap, géométrie 3D, centroïdes, raycast) sur une paire de faces. |
| **compute2DFromObserver** | Recalcule uniquement les coordonnées écran 2D à partir de l'espace observateur (sans re-trier). |
| **compute_bbox_intersection** | Calcule le rectangle d'intersection des bbox 2D de deux faces. Retourne 0 si pas de chevauchement. |
| **compute_bbox_intersection_center** | Variante qui retourne le centre de l'intersection des bbox 2D. |
| **compute_face_bboxes3D** | Calcule les bounding-boxes 3D (min/max X/Y/Z) de toutes les faces en espace objet. |
| **compute_face_plane** | Calcule l'équation de plan ax+by+cz+d=0 d'une face à partir de ses trois premiers sommets (coords Fixed). |
| **compute_face_plane_float** | Même calcul que compute_face_plane mais en flottant (compatible avec les distances FLOAT). |
| **compute_intersection_centroid_ordered_fixed** | Calcule le centroïde de l'intersection de deux polygones projetés via clipping Sutherland-Hodgman (Fixed). |
| **compute_intersection_centroid_ordered_fixed_tryboth** | Essaie les deux ordres de clipping (f1→f2 et f2→f1) pour obtenir un centroïde d'intersection fiable. |
| **compute_intersection_centroid_ordered_qd_fixed** | Calcule le centroïde de l'intersection via régions QuickDraw (plus rapide, dépend de QD). |
| **compute_intersection_region_bbox** | Retourne la bbox de la région d'intersection QuickDraw de deux faces. |
| **computeOrientationShading** | Calcule un shading par face basé sur l'orientation de la normale par rapport à l'observateur. |
| **count_edge_crossings** | Compte combien d'arêtes d'une face croisent le plan (et l'intérieur) d'une autre face. |
| **createModel3D** | Alloue dynamiquement toute la structure Model3D (sommets, faces, plans, buffers). Retourne NULL en cas d'échec. |
| **debug_two_faces** | Outil de debug qui affiche des informations détaillées sur une paire de faces donnée. |
| **destroyModel3D** | Libère toutes les allocations mémoire associées à un modèle. |
| **display_model_face_ids** | Affiche le modèle en fil de fer avec les numéros de faces au centre de chaque polygone. |
| **DoColor** | Affiche la palette de couleurs courante à l'écran (outil de debug visuel). |
| **DoText** | Bascule l'écran en mode texte et efface l'affichage. |
| **drawFace** | Dessine une face unique (remplie ou filaire) avec la couleur demandée. |
| **drawFaceIndex** | Affiche le numéro d'index d'une face à proximité de son centroïde projeté. |
| **drawPixel** | Pose un pixel couleur aux coordonnées écran données (utilisé par le Z-buffer scanline). |
| **drawPolygons** | Parcourt sorted_face_indices et dessine toutes les faces visibles (mode rempli). |
| **drawPolygons_jitter** | Variante de drawPolygons avec jitter anti-aliasing léger. |
| **dump_compare_results_to_file** | Écrit les résultats textuels d'un diagnostic de paire de faces dans un fichier fXvsfY.txt. |
| **dumpFace2DCoordinates** | Exporte les coordonnées 2D projetées de chaque face dans un fichier texte. |
| **dumpFaceEquationsCSV** | Exporte les équations de plan, profondeurs et indices de sommets de toutes les faces en CSV. |
| **dumpSortedFaceIndices** | Exporte l'ordre final de dessin (sorted_face_indices) dans un fichier texte. |
| **edge_crosses_interior** | Teste si une arête d'une face traverse strictement l'intérieur d'une autre face (pas seulement la frontière). |
| **edge_intersects_face** | Teste si une arête d'une face intersecte le polygone d'une autre face (y compris cas coplanaires). |
| **enforce_mandatory_after_relations** | Réapplique les contraintes « face A doit être après face B » après chaque réparation d'ordre. |
| **evaluate_pair_tests** | Évalue les tests planaires de base entre deux faces (utilisé par les inspecteurs). |
| **faces_share_vertex_or_edge** | Retourne vrai si deux faces partagent au moins un sommet (ou une arête complète). |
| **faces_vertices_equal** | Compare les listes de sommets de deux faces pour détecter une égalité géométrique. |
| **frameInconclusivePairs** | Surligne à l'écran les paires de faces déclarées « inconclusive » par le painter. |
| **generate_random_colors** | Attribue des couleurs aléatoires à toutes les faces (mode fill/frame random). |
| **geo_face_order** | Détermine l'ordre géométrique de deux faces via pair_plane_before / pair_plane_after. Retourne -1, 0 ou 1. |
| **geometric_face_relation** | Test planaire Fixed64 optimisé : décide si f1 est devant/derrière f2 selon le côté de l'observateur. |
| **getFaceFillColor** | Retourne la couleur de remplissage effective d'une face (utilisateur, random ou défaut). |
| **getFaceFrameColor** | Retourne la couleur de contour effective d'une face. |
| **getObserverParams** | Dialogue interactif pour saisir/modifier angles et distance de l'observateur. |
| **hideFace** | Cache une face en sauvegardant son vertex_count et en le mettant à 0. |
| **initPalettes** | Initialise les tables de palettes QuickDraw utilisées par le programme. |
| **inspect_face_pair_ui** | Inspecteur interactif graphique : navigue entre paires de faces, lance tous les diagnostics, permet de réordonner. |
| **inspect_faces_after** | Inspecte les faces placées après une cible dans le tri alors qu'elles devraient être avant. |
| **inspect_faces_before** | Inspecte les faces placées avant une cible dans le tri alors qu'elles devraient être après. |
| **loadModel3D** | Charge un fichier OBJ complet (sommets + faces) dans une structure Model3D déjà allouée. |
| **main** | Point d'entrée du programme : charge le modèle, gère la boucle interactive et dispatch les actions clavier. |
| **move_element_remove_and_insert** | Déplace un élément dans un tableau d'indices (retrait puis insertion à une nouvelle position). |
| **move_element_remove_and_insert_pos** | Même opération en maintenant à jour un tableau inverse pos_of_face. |
| **normalizeAutoFitDistanceTo150** | Normalise la distance observateur à 150 tout en conservant la taille apparente du modèle. |
| **on_seg_ll** | Teste si un point se trouve sur un segment (utilitaire géométrique 2D, long long). |
| **orient_ll** | Calcule l'orientation (sens de rotation) de trois points (utilitaire 2D, long long). |
| **painter_correct** | Painter de correction locale : utilise les tests planaires pour réordonner les faces mal placées. |
| **painter_correctV2** | Variante expérimentale de painter_correct avec détection et split de faces pathologiques. |
| **painter_geoV2** | Painter géométrique : bubble-sort avec tests planaires + ray-cast, accéléré par un cache de paires. |
| **painter_geoV3** | Painter style sélection (Dony) avec restart, protection anti-boucle (dd_flag / dd_flag_raycast) ; conflits non résolus via ray_cast_hierarchical. |
| **painter_newell_sancha** | Implémentation complète Newell/Sancha en Fixed : tri initial + corrections par tests d'overlap et de plans. |
| **painter_newell_sancha_fast** | Tri ultra-rapide par z_mean uniquement (base de tous les autres painters). |
| **painter_newell_sancha_old** | Ancienne version du painter Newell/Sancha (conservée pour référence). |
| **pair_cache_create** | Crée une table de hachage de capacité fixe pour mémoriser les relations d'ordre déjà calculées. |
| **pair_cache_destroy** | Libère la mémoire du cache de paires. |
| **pair_cache_find** | Recherche une relation d'ordre déjà connue entre deux faces dans le cache. |
| **pair_cache_insert** | Insère une relation d'ordre (f1 avant/après f2) dans le cache. |
| **pair_cache_max_capacity_for_budget** | Calcule la capacité max du cache compatible avec le budget mémoire IIGS. |
| **pair_cache_next_pow2** | Calcule la puissance de 2 supérieure ou égale à n (pour le dimensionnement du cache). |
| **pair_cache_next_pow2_floor** | Calcule la puissance de 2 inférieure ou égale à n. |
| **pair_cache_slot** | Calcule le slot de hachage pour une paire de faces. |
| **pair_epsilon** | Retourne le max des epsilons précalculés de deux faces (epsilon de paire pour les tests planaires). |
| **pair_order_relation** | Détermine la relation d'ordre entre deux faces (wrapper autour des tests planaires). |
| **pair_plane_after** | Teste si f1 est entièrement derrière le plan de f2 par rapport à l'observateur. |
| **pair_plane_before** | Teste si f2 est entièrement devant le plan de f1 par rapport à l'observateur. |
| **pair_plane_before_debug** | Version debug interactive de pair_plane_before (affiche les détails sommet par sommet). |
| **pair_plane_geometric_tests** | Version diagnostique interactive des tests planaires (Tests #1 à #4 détaillés). |
| **plane_side_test** | Test de côté de plan (logique Dony) : vérifie qu'aucun sommet de other_face n'est clairement du côté interdit de plane_face. |
| **point_in_poly_arrays_int** | Test point-in-polygon sur des tableaux d'entiers (ray casting 2D). |
| **point_in_poly_int** | Test point-in-polygon pour une face du modèle (utilise les indices de sommets projetés). |
| **point_seg_dist2_fixed_int** | Distance au carré (Fixed) d'un point à un segment 2D, entièrement en arithmétique entière. |
| **poly_signed_area** | Calcule l'aire signée d'un polygone projeté sur le plan de sa normale. |
| **precompute_plane_epsilons** | Précalcule un epsilon par face (0,1 % de la longueur de la normale, plancher 0,01) pour éviter sqrt dans la boucle chaude de painter_geoV3. |
| **processModelFast** | Pipeline principal : transformation Fixed32 → projection 2D → calcul des profondeurs → appel du painter. |
| **projected_polygons_overlap** | Détecte si deux polygones projetés se chevauchent réellement (intersection d'arêtes + containment). |
| **projected_polygons_overlap_old** | Ancienne implémentation de la détection d'overlap 2D (conservée pour compatibilité). |
| **projected_polygons_overlap_simple** | Version simplifiée/rapide de la détection d'overlap (utilisée dans les painters de correction). |
| **ray_cast_at** | Lance un rayon à un pixel (x,y) et compare les profondeurs des deux faces. |
| **ray_cast_distances** | Calcule les distances observateur → plans des deux faces en un point écran donné. |
| **ray_cast_hierarchical** | Ray-cast multi-stratégie : essaie centroïde QD, centroïde SH, puis centre de bbox. |
| **readFaces_model** | Parse les faces (f) d'un fichier OBJ et les stocke dans les tableaux parallèles du modèle. |
| **readVertices** | Parse les sommets (v) d'un fichier OBJ, convertit en Fixed32, centre le modèle et calcule l'auto-fit. |
| **remove_duplicates** | Élimine tout index apparaissant plusieurs fois dans une liste de sommets (conserve la première occurrence). |
| **renderModelScanlineZBuffer** | Point d'entrée du rendu alternatif par Z-buffer scanline. |
| **renderModelScanlineZBuffer_biased** | Variante du Z-buffer avec biais pour réduire les artefacts de coplanarité. |
| **renderModelScanlineZBuffer_fast** | Version optimisée du Z-buffer scanline. |
| **renderModelScanlineZBuffer_old** | Ancienne implémentation du Z-buffer scanline. |
| **reorder_poly** | Réordonne les sommets d'un polygone en sens trigonométrique autour de son centroïde. |
| **restoreAllFaces** | Restaure toutes les faces précédemment cachées. |
| **restoreFace** | Restaure une face cachée à partir de son saved_vertex_count. |
| **reverseFaceVertexOrder** | Inverse l'ordre des sommets d'une face (change le sens de la normale). |
| **run_raycast_test** | Wrapper : exécute un ray-cast en un point et retourne 1 (f1 devant), 2 (f2 devant) ou 0. |
| **saveNextScreenshot** | Trouve un nom de fichier libre screenXXX.PIC et sauvegarde l'écran SHR. |
| **saveSHRAsRawPic** | Écrit les 32 Ko de mémoire SHR dans un fichier PIC non compressé et fixe le type ProDOS. |
| **screen2Black** | Efface l'écran graphique en noir. |
| **segs_intersect_int** | Teste l'intersection de deux segments 2D (délègue à segs_intersect_int_fixed64). |
| **segs_intersect_int_fixed64** | Test d'intersection de segments 2D en arithmétique Fixed64, avec tolérance de proximité. |
| **segs_intersect_unit_tests** | Jeux de tests unitaires pour valider segs_intersect_int / fixed64. |
| **set_use_fixed_clipping** | Active/désactive le clipping Fixed64 (actuellement forcé à 1, setter commenté). |
| **SetColor** | Définit une entrée RGB dans une palette QuickDraw. |
| **setProDOSFileType** | Définit le type et le sous-type ProDOS d'un fichier via GS/OS. |
| **show_help_pager** | Affiche l'aide clavier page par page (20 lignes max). |
| **show_inspect_faces_with_message** | Affiche le modèle en fil de fer, surligne deux faces et attend une touche après un message. |
| **showFace** | Mode interactif pour examiner une face isolée (navigation, hide/restore, inversion de normale…). |
| **sort_by_zmax** | Tri par insertion des faces selon z_max décroissant (tri initial de painter_geoV3). |
| **split_face_by_plane** | Découpe une face par le plan d'une autre face ; crée de nouveaux sommets et fragments. |
| **strict_inside** | Teste si un point est strictement à l'intérieur d'un polygone (rejette les points sur les bords). |
| **updateFace2DBounds** | Met à jour les bbox 2D de toutes les faces à partir des coordonnées projetées courantes. |
