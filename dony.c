/*
 * ============================================================================
 *  dony.c — Classement GEO V3 par sélection et reprise
 * ============================================================================
 *
 *  Rôle :
 *    Construire un ordre arrière-vers-avant selon une stratégie inspirée de
 *    Robert Dony : sélectionner une face, comparer les candidates et
 *    reprendre la sélection après permutation.
 *
 *  Organisation :
 *    Un tri stable par profondeur maximale prépare le parcours. Les
 *    intervalles disjoints sont rejetés, puis les plans sont testés dans les
 *    deux sens. Les cas ambigus utilisent le chevauchement projeté et le ray
 *    casting hiérarchique. Des stamps et un cache de paires limitent les
 *    cycles.
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
 *    insertion binaire puis memmove ; les clés égales restent dans leur
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
        memmove(&sorted[lo+1],&sorted[lo],(size_t)(i-lo)*sizeof(int));
        sorted[lo]=key;
    }
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
 *
 * ----------------------------------------------------------------------------
 */
void painter_geoV3(Model3D* model, int face_count)
{
    FaceArrays3D* faces = &model->faces;
    int* sorted = faces->sorted_face_indices;

    static unsigned int* dd_flag = NULL;
    static DonyRayPairs ray_pairs;
    static int allocated = 0;
    static unsigned int generation = 0;

    int nf = face_count;
    int fs = 0;


    /* Invalider explicitement les relations préparées avant le tri de cette vue. */
    (void)geometric_face_relation(NULL, 0, 0);

    /* Agrandir les buffers persistants de sélection et de prévention des cycles. */
    if (face_count > allocated) {
        free(dd_flag);

        dd_flag = (unsigned int*)calloc((size_t)face_count, sizeof(unsigned int));

        if (!dd_flag || !dony_ray_pairs_reserve(&ray_pairs, face_count)) {
            fprintf(stderr, "painter_geoV3: allocation failed\n");
            exit(1);
        }

        allocated = face_count;
        generation = 0;
    }


    /* Repartir de l'ordre canonique pour que les égalités ne dépendent pas
     * de la vue précédente.
     */
    for (int i = 0; i < nf; ++i)
        sorted[i] = i;


    /* Produire l'ordre initial et préparer le filtrage assembleur si
     * celui-ci est activé.
     */
    sort_by_zmax(sorted, faces->z_max, nf);
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


            if (++permutes_this_pass > face_count)
                goto do_split_giveup;

            sorted[fs]=q;
            sorted[f2]=p;


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

                    dony_ray_pair_record(&ray_pairs, p, q, generation);

                    sorted[fs]=q;
                    sorted[f2]=p;

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
}
