/*
 * ============================================================================
 *  dony.c — Classement GEO V3 par sélection et reprise
 * ============================================================================
 *
 *  Rôle :
 *    Construire un ordre arrière-vers-avant selon une stratégie inspirée de
 *    Robert Dony : sélectionner une face, comparer les candidates et
 *    reprendre la sélection après permutation. Le repair réutilise ce
 *    parcours depuis la permutation courante.
 *
 *  Organisation :
 *    Un tri stable par profondeur maximale prépare le parcours. Les
 *    intervalles disjoints sont rejetés, puis les plans sont testés dans les
 *    deux sens. Les cas ambigus utilisent le chevauchement projeté et le ray
 *    casting hiérarchique. Des stamps et un cache de paires limitent les
 *    cycles. Le callback facultatif du repair confirme chaque échange,
 *    sans dupliquer les décisions de classement.
 *    Tout échange exige le chevauchement validé par la méthode configurée,
 *    y compris lorsqu’il découle directement des tests de plans.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c ; dony_scan_asm.c fournit le filtrage par défaut.
 *    GEOV3_DISABLE_ASM_SCAN sélectionne le filtrage C. Les métriques doivent
 *    être préparées avant le tri et rester stables pendant son exécution.
 *    Cette variante ne découpe pas les faces.
 *
 * ============================================================================
 */
#if !defined(GEOV3_DISABLE_ASM_SCAN)
#include "dony_scan_asm.c"
#endif


#define DONY_RAY_PAIR_BUCKETS 256
typedef struct {
    unsigned int *first, *second, *next;
    unsigned int heads[DONY_RAY_PAIR_BUCKETS];
    unsigned int stamps[DONY_RAY_PAIR_BUCKETS];
    int capacity;
    unsigned int count;
} DonyRayPairs;


/*
 * ----------------------------------------------------------------------------
 *  dony_ray_pairs_reserve
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Réserver les listes de paires utilisées pour empêcher les cycles de ray
 *    casting.
 *
 *  Paramètres :
 *    [entrée] cache, structure possédant ses buffers ; nf, capacité requise.
 *
 *  Retour :
 *    1 si la capacité suffit ou a été augmentée, 0 si une allocation échoue.
 *
 *  Sorties et effets de bord :
 *    buffers remplacés seulement après réussite des trois allocations ;
 *    count et stamps sont réinitialisés lors de l'agrandissement.
 *
 * ----------------------------------------------------------------------------
 */
static int dony_ray_pairs_reserve(DonyRayPairs *cache, int nf)
{
    unsigned int *a, *b, *next;
    /* Réutiliser la capacité existante ; sinon allouer un nouveau jeu
     * complet avant de remplacer l'ancien.
     */
    if (nf <= cache->capacity) return 1;
    a = (unsigned int*)malloc((size_t)nf * sizeof(unsigned int));
    b = (unsigned int*)malloc((size_t)nf * sizeof(unsigned int));
    next = (unsigned int*)malloc((size_t)nf * sizeof(unsigned int));
    /* Libérer seulement les nouvelles allocations en cas d'échec. */
    if (!a || !b || !next) {
        free(a); free(b); free(next);
        return 0;
    }
    free(cache->first); free(cache->second); free(cache->next);
    cache->first = a; cache->second = b; cache->next = next;
    cache->capacity = nf;
    cache->count = 0;
    memset(cache->stamps, 0, sizeof(cache->stamps));
    return 1;
}


/*
 * ----------------------------------------------------------------------------
 *  dony_ray_pair_seen
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester si une paire non orientée a déjà provoqué un échange dans cette
 *    passe.
 *
 *  Paramètres :
 *    [entrée] cache ; p et q, faces à base zéro ; generation, passe de
 *    sélection.
 *
 *  Retour :
 *    1 pour une paire exacte présente, 0 sinon.
 *
 *  Fonctionnement :
 *    Aucun effet de bord. Les identifiants sont normalisés, puis les
 *    collisions de hash sont parcourues dans une liste chaînée dont les
 *    liens sont stockés à base un.
 *
 * ----------------------------------------------------------------------------
 */
static int dony_ray_pair_seen(DonyRayPairs *cache, int p, int q,
                              unsigned int generation)
{
    unsigned int a=(unsigned int)p,b=(unsigned int)q,bucket;
    /* Normaliser la paire pour que (p,q) et (q,p) partagent la même clé. */
    if(a>b){unsigned int swap=a;a=b;b=swap;}
    bucket=(((a << 5)+a)^b) & (DONY_RAY_PAIR_BUCKETS-1);
    unsigned int entry;
    /* Ignorer les buckets d'une autre passe, puis comparer les clés complètes. */
    if (cache->stamps[bucket] != generation) return 0;
    entry = cache->heads[bucket];
    while (entry) {
        unsigned int i = entry - 1;
        if (cache->first[i] == a && cache->second[i] == b) return 1;
        entry = cache->next[i];
    }
    return 0;
}


/*
 * ----------------------------------------------------------------------------
 *  dony_ray_pair_record
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Enregistrer une paire non orientée échangée pendant la passe courante.
 *
 *  Paramètres :
 *    [entrée] cache, p, q et generation.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    count, tableaux de paires, tête de bucket et stamp mis à jour.
 *
 *  Préconditions :
 *    l'appelant a vérifié la limite de permutations ; une entrée libre doit
 *    exister. Les collisions conservent les clés exactes.
 *
 * ----------------------------------------------------------------------------
 */
static void dony_ray_pair_record(DonyRayPairs *cache, int p, int q,
                                 unsigned int generation)
{
    unsigned int a=(unsigned int)p,b=(unsigned int)q,bucket;
    if(a>b){unsigned int swap=a;a=b;b=swap;}
    bucket=(((a << 5)+a)^b) & (DONY_RAY_PAIR_BUCKETS-1);
    /* Ajouter l'entrée en tête de chaîne ; la capacité est contrôlée par
     * l'appelant.
     */
    unsigned int i = cache->count++;

    cache->first[i] = a;
    cache->second[i] = b;
    cache->next[i] = (cache->stamps[bucket] == generation) ? cache->heads[bucket] : 0;
    cache->heads[bucket] = i + 1;
    cache->stamps[bucket] = generation;
}


/*
 * ----------------------------------------------------------------------------
 *  sort_by_zmax
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Trier les indices de faces par profondeur maximale décroissante, de
 *    façon stable.
 *
 *  Paramètres :
 *    [entrée] z_max, profondeurs par face ; nf, longueur de sorted.
 *
 *    [entrée-sortie] sorted, permutation d'identifiants à base zéro.
 *
 *  Retour :
 *    Aucun.
 *
 *  Algorithme :
 *    insertion binaire puis déplacement stable partagé ; les clés égales
 *    restent dans leur
 *    ordre initial. Une clé déjà à sa place évite la recherche.
 *
 * ----------------------------------------------------------------------------
 */
static void sort_by_zmax(int *sorted,Fixed32 *z_max,int nf)
{
    int i;
    /* Étendre le préfixe trié sans déplacer les faces de profondeur égale. */
    for(i=1;i<nf;++i){
        int key=sorted[i];
        Fixed32 depth=z_max[key];
        int lo,hi;
        if(z_max[sorted[i-1]]>=depth)continue;
        lo=0;hi=i;
        /* Chercher la position après les clés égales afin de préserver la stabilité. */
        while(lo<hi){
            int mid=lo+(hi-lo)/2;

            if(z_max[sorted[mid]]>=depth)lo=mid+1;
            else hi=mid;
        }
        /* Déplacer la clé par la même primitive stable que les autres listes. */
        (void)move_element_remove_and_insert(sorted, nf, i, lo);
    }
}


/*
 * ----------------------------------------------------------------------------
 *  geov3_run
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Partager le parcours GEO V3 entre le classement et le repair interactif.
 *
 *  Paramètres :
 *    [entrée] model et face_count : modèle aux métriques de vue à jour.
 *    [entrée] preserve_order : 0 initialise et trie par zmax ; 1 part de la
 *    permutation courante, sans tri initial silencieux.
 *    [entrée] on_exchange/context : confirmation facultative et son contexte.
 *    Le callback reçoit les deux faces, leurs positions avant échange et la
 *    raison ; -1 arrête, 0 ignore cet échange, 1 l'applique. NULL applique tout.
 *
 *  Retour :
 *    Nombre d'échanges appliqués, y compris avant un arrêt demandé ; -1 en
 *    cas d'échec d'allocation du repair. Le classement conserve son erreur
 *    fatale historique d'allocation.
 *
 *  Fonctionnement :
 *    Réutilise sans duplication le filtrage C/assembleur, les deux tests de
 *    plans, les stamps anti-cycle, le chevauchement et le rayon hiérarchique
 *    GEO V3. Le callback intervient avant chaque échange, sans modifier la
 *    géométrie ni la permutation. Un refus avance au candidat suivant.
 *    Les échanges par plans comme par rayon exigent le chevauchement projeté
 *    configuré avant de consommer la limite de permutations ou les stamps.
 *    Les limites de permutations et les cas non résolus restent ceux de GEO V3.
 *    Les buffers persistants sont partagés : ce parcours n'est pas réentrant.
 *
 * ----------------------------------------------------------------------------
 */
static int geov3_run(Model3D* model, int face_count, int preserve_order,
                     GeoV3ExchangeCallback on_exchange, void *context)
{
    if (!model || face_count <= 0) return 0;
    FaceArrays3D* faces = &model->faces;
    int* sorted = faces->sorted_face_indices;

    static unsigned int* dd_flag = NULL;
    static DonyRayPairs ray_pairs;
    static int allocated = 0;
    static unsigned int generation = 0;

    int nf = face_count;
    int fs = 0;
    int exchanges = 0;


    /* Invalider explicitement les relations préparées avant le tri de cette vue. */
    (void)geometric_face_relation(NULL, 0, 0);

    /* Agrandir les buffers persistants de sélection et de prévention des cycles. */
    if (face_count > allocated) {
        free(dd_flag);

        dd_flag = (unsigned int*)calloc((size_t)face_count, sizeof(unsigned int));

        if (!dd_flag || !dony_ray_pairs_reserve(&ray_pairs, face_count)) {
            fprintf(stderr, "painter_geoV3: allocation failed\n");
            if (preserve_order) {
                free(dd_flag); dd_flag = NULL; allocated = 0;
                memset(ray_pairs.stamps, 0, sizeof(ray_pairs.stamps));
                return -1;
            }
            exit(1);
        }

        allocated = face_count;
        generation = 0;
    }


    /* Repartir de l'ordre canonique pour que les égalités ne dépendent pas
     * de la vue précédente.
     */
    if (!preserve_order) {
        for (int i = 0; i < nf; ++i) sorted[i] = i;
        sort_by_zmax(sorted, faces->z_max, nf);
    }
    /* Le filtrage parcourt la permutation choisie, sans supposer que le
     * repair a préalablement trié les profondeurs.
     */
#if !defined(GEOV3_DISABLE_ASM_SCAN)
    GeoV3_ScanSetup(faces, sorted, nf);
#endif

    /* Fixer successivement la prochaine face à dessiner au début de la
     * partie non traitée.
     */
    while (fs < nf - 1) {
        int p;
        int f2;
        int permutes_this_pass;


        /* Attribuer un stamp à la passe ; après rebouclage, effacer toute la
         * capacité avant de réutiliser 1.
         */
        ++generation;
        if (generation == 0) {
            memset(dd_flag, 0, (size_t)allocated * sizeof(unsigned int));
            memset(ray_pairs.stamps, 0, sizeof(ray_pairs.stamps));
            generation = 1;
        }

        ray_pairs.count = 0;
        p = sorted[fs];
        f2 = fs + 1;
        permutes_this_pass = 0;

        /* Comparer P aux candidats restants ; un échange impose de
         * recommencer la sélection.
         */
        while (f2 < nf) {
            int q;
#if !defined(GEOV3_DISABLE_ASM_SCAN)
            f2 = GeoV3_FindCandidate(faces, p, f2);
            if (f2 >= nf) break;
#endif
            q = sorted[f2];

#if defined(GEOV3_DISABLE_ASM_SCAN)


            /* Chemin C optionnel : éliminer les intervalles de profondeur séparés. */
            if (faces->z_max[q] <= faces->z_min[p])
                goto skip_q;


            {
                /* Rejeter les bornes X sans recouvrement strict, avec les
                 * soustractions natives 16 bits.
                 */
                int dx;

                dx = (faces->maxx[p] > faces->maxx[q])
                   ? faces->minx[p] - faces->maxx[q]
                   : faces->minx[q] - faces->maxx[p];

                if (dx >= 0)
                    goto skip_q;
            }


            {
                /* Appliquer le même rejet aux bornes Y. */
                int dy;

                dy = (faces->maxy[p] > faces->maxy[q])
                   ? faces->miny[p] - faces->maxy[q]
                   : faces->miny[q] - faces->maxy[p];

                if (dy >= 0)
                    goto skip_q;
            }

#endif


            {
                int geo_pq;
                int geo_qp;

                /* Tester d'abord P/Q ; le sens inverse est inutile lorsque
                 * le premier résultat valide déjà P avant Q.
                 */
                geo_pq = geometric_face_relation(model, p, q);


                if (geo_pq == -1)
                    goto skip_q;

                geo_qp = geometric_face_relation(model, q, p);


                if (geo_qp == 1)
                    goto skip_q;


                /* Si la géométrie impose Q avant P, échanger sauf si Q a
                 * déjà été écartée pendant cette passe.
                 */
                if (geo_pq == 1 || geo_qp == -1) {
                    if (dd_flag[q] != generation)
                        goto do_permute;

                    goto do_split_giveup;
                }


                goto do_split_giveup;
            }

/* Limiter les permutations pour garantir la terminaison et reprendre avec le
 * nouveau P.
 */
do_permute:
            /* Des fragments coplanaires peuvent être classés de part et
             * d’autre après quantification. Ne pas échanger des contours
             * sans aire commune : cet échange consommerait inutilement les
             * protections anti-cycle et pourrait bloquer une vraie correction.
             * Le repair applique désormais exactement le même filtre ici. */
            if (!projected_polygons_overlap_cached(model, p, q))
                goto skip_q;

            if (++permutes_this_pass > face_count)
                goto do_split_giveup;

            /* Proposer exactement l'échange décidé par les tests de plans. */
            if (on_exchange) {
                int decision = on_exchange(model, p, q, fs, f2,
                                           GEOV3_EXCHANGE_PLANES, context);
                if (decision < 0) return exchanges;
                if (!decision) goto skip_q;
            }
            sorted[fs]=q;
            sorted[f2]=p;
            ++exchanges;


            dd_flag[p] = generation;

            p = q;
            f2 = fs + 1;

            continue;

/* Remplacer le découpage absent par chevauchement et ray casting, sans
 * rééchanger une paire déjà mémorisée.
 */
do_split_giveup:


            if (!dony_ray_pair_seen(&ray_pairs, p, q, generation) &&
                projected_polygons_overlap_cached(model, p, q)) {
                int rc;

                rc = ray_cast_hierarchical(model, p, q);


                /* Un rayon impose Q avant P ; enregistrer la paire avant de
                 * redémarrer les comparaisons.
                 */
                if (rc < 0) {
                    if (++permutes_this_pass > face_count)
                        goto skip_q;

                    /* Même confirmation pour le repli par rayon hiérarchique. */
                    if (on_exchange) {
                        int decision = on_exchange(model, p, q, fs, f2,
                                                   GEOV3_EXCHANGE_RAY, context);
                        if (decision < 0) return exchanges;
                        if (!decision) goto skip_q;
                    }
                    dony_ray_pair_record(&ray_pairs, p, q, generation);

                    sorted[fs]=q;
                    sorted[f2]=p;
                    ++exchanges;

                    dd_flag[p] = generation;


                    p = q;
                    f2 = fs + 1;

                    continue;
                }


                if (rc > 0)
                    goto skip_q;
            }

/* Avancer après une paire acceptée ou sans solution ; terminer la passe à
 * épuisement des candidats.
 */
skip_q:
            ++f2;
        }

        ++fs;
    }
    return exchanges;
}

/*
 * ----------------------------------------------------------------------------
 *  painter_geoV3
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Construire l'ordre arrière-vers-avant par sélection et reprise des
 *    comparaisons.
 *
 *  Paramètres :
 *    [entrée] model et face_count, nombre de faces à ordonner.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    model->faces.sorted_face_indices
 *
 *  Fonctionnement :
 *    Initialise l'ordre canonique, trie par zmax, rejette les intervalles
 *    disjoints, puis teste les plans dans les deux sens. Une face devant
 *    précéder P est permutée en tête de la passe, qui reprend avec ce
 *    nouveau P. En cas d'ambiguïté ou de cycle, utilise le chevauchement
 *    projeté et le ray casting hiérarchique. Aucun découpage : des cycles
 *    peuvent rester insolubles. État statique conservé entre appels ; arrêt
 *    du processus si la réserve échoue.
 *    Un échange par plans sans chevauchement projeté est écarté : les fragments
 *    adjacents issus d’un découpage ne doivent pas créer un faux cycle.
 *
 * ----------------------------------------------------------------------------
 */
void painter_geoV3(Model3D* model, int face_count)
{
    /* Le classement normal conserve son ordre canonique et son tri par zmax. */
    (void)geov3_run(model, face_count, 0, NULL, NULL);
}
