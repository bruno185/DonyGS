/*
 * ============================================================================
 *  ordering_cache.c — Caches des calculs d’ordre géométrique
 * ============================================================================
 *
 *  Rôle :
 *    Regrouper les résultats réutilisables pendant le traitement d'une vue :
 *    relations entre faces, plans préparés, côtés des sommets et contours
 *    projetés. Ces caches évitent de répéter les calculs géométriques
 *    coûteux.
 *
 *  Organisation :
 *    Les entrées sont associées à un modèle et à une génération de données.
 *    Les clés complètes sont vérifiées avant réutilisation ; une collision
 *    peut remplacer une entrée, mais ne doit pas produire un faux résultat.
 *    Les allocations sont conservées entre invalidations.
 *
 *  Dépendances et contraintes :
 *    Inclus par engine.c après declare.c. Les changements de transformation,
 *    d'époque, de tolérance ou d'échelle invalident les entrées concernées.
 *    Les buffers et paquets assembleur sont partagés : leur utilisation est
 *    synchrone et non réentrante.
 *
 * ============================================================================
 */
#define ORDER_PAIR_SLOTS 256
#define ORDER_PLANE_BLOCK_SIZE 64
#define ORDER_PLANE_BLOCKS ((MAX_FACES + ORDER_PLANE_BLOCK_SIZE - 1) / ORDER_PLANE_BLOCK_SIZE)
/* Clé orientée : inverser f1 et f2 produit une autre requête, notamment pour
 * sujet et clip.
 */
typedef struct {
    int f1, f2;
    Fixed32 scale;
    unsigned int generation, context;
    int result;
} OrderPairEntry;
typedef struct {
    double plane[ORDER_PLANE_BLOCK_SIZE][4];
    unsigned int stamp[ORDER_PLANE_BLOCK_SIZE];
} OrderPlaneBlock;


#define ORDER_FIXED_PLANE_SLOTS 64
#define ORDER_WINDING_SLOTS 128
#define ORDER_VERTEX_SIDE_SLOTS 64
typedef struct {
    Fixed64 a,b,c,d,epsilon;
    Fixed32 a32,b32,c32;
    int face,observer,asm_ready;
    unsigned int generation,epoch;


    /* Côté exact par sommet : identifiant OBJ sur 13 bits et code sur deux
     * bits ; 0 vide, $2000 sur le plan, $4000 positif, $6000 négatif.
     */
    unsigned int vertex_sides[ORDER_VERTEX_SIDE_SLOTS];
} OrderFixedPlane;
typedef struct {
    double area2;
    Fixed32 scale;
    int face;
    unsigned int generation;
} OrderWinding;


#define ORDER_CONTOUR_SLOTS 64
#define ORDER_CONTOUR_MAX_VERTICES 16
typedef struct {
    /* Buffer de doubles : x puis y du contour antihoraire, suivis de dx puis
     * dy ; chaque groupe a capacity éléments.
     */
    double *data;
    Fixed32 scale;
    int face,count,capacity;
    unsigned int generation,failed_generation;
} OrderContour;


/* Codes denses sur deux bits : 0 inconnu, 1 avant, 2 indéterminé, 3 après ;
 * au plus 16 Kio pour 256 faces.
 */
#define ORDER_GEOMETRY_DENSE_LIMIT 256
static unsigned char *order_geometry_pairs=NULL;
static unsigned char *order_geometry_rows[ORDER_GEOMETRY_DENSE_LIMIT];
static unsigned int order_geometry_capacity=0,order_geometry_stride=0;
static unsigned int order_geometry_generation=0,order_geometry_epoch=0;
static unsigned int order_geometry_failed=0;
static int order_geometry_faces=0;
static Fixed64 order_geometry_epsilon=0;
static OrderContour order_contours[ORDER_CONTOUR_SLOTS];
static OrderFixedPlane order_fixed_planes[ORDER_FIXED_PLANE_SLOTS];
static OrderFixedPlane *order_asm_packet_plane=NULL;
static unsigned int order_fixed_plane_epoch=0;
static OrderWinding order_windings[ORDER_WINDING_SLOTS];
static OrderPairEntry order_ray_pairs[ORDER_PAIR_SLOTS];
static OrderPairEntry order_overlap_pairs[ORDER_PAIR_SLOTS];
static OrderPlaneBlock *order_planes[ORDER_PLANE_BLOCKS];
static unsigned int order_plane_failed[ORDER_PLANE_BLOCKS];
static Model3D *order_cache_owner = NULL;
static unsigned int order_cache_generation = 0;
static double *order_sh_workspace = NULL;
static long order_sh_capacity = 0;


/* Compteurs optionnels de réutilisation ; sans GEO_ORDER_PROFILE,
 * ORDER_COUNT est sans effet.
 */
#if defined(GEO_ORDER_PROFILE)
static struct {
    unsigned long ray_calls, ray_hits, overlap_calls, overlap_hits;
    unsigned long sh_calls, sh_allocations, plane_conversions;
} order_profile;
#define ORDER_COUNT(field) (++order_profile.field)
#else
#define ORDER_COUNT(field) ((void)0)
#endif


/*
 * ----------------------------------------------------------------------------
 *  ordering_cache_invalidate
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Changer la génération des caches et désigner leur modèle propriétaire.
 *
 *  Paramètres :
 *    [entrée] model, propriétaire de la nouvelle vue.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    propriétaire, génération et paquet assembleur invalidé.
 *
 *  Fonctionnement :
 *    Les allocations sont conservées. Lorsque le compteur revient à zéro,
 *    effacer les stamps avant de repartir à 1 pour ne pas réutiliser une
 *    ancienne vue.
 *
 * ----------------------------------------------------------------------------
 */
static void ordering_cache_invalidate(Model3D *model)
{
    int i;
    /* Changer de vue sans libérer les allocations ; invalider le paquet
     * assembleur.
     */
    order_cache_owner = model;
    order_asm_packet_plane=NULL;
    ++order_cache_generation;
    /* Éliminer tous les anciens stamps avant de réutiliser la génération 1. */
    if (order_cache_generation == 0) {
        order_geometry_generation=0;order_geometry_failed=0;
        for(i=0;i<ORDER_CONTOUR_SLOTS;++i){
            order_contours[i].generation=0;order_contours[i].failed_generation=0;
        }
        memset(order_fixed_planes,0,sizeof(order_fixed_planes));
        memset(order_windings,0,sizeof(order_windings));
        memset(order_ray_pairs, 0, sizeof(order_ray_pairs));
        memset(order_overlap_pairs, 0, sizeof(order_overlap_pairs));
        memset(order_plane_failed, 0, sizeof(order_plane_failed));
        for (i = 0; i < ORDER_PLANE_BLOCKS; ++i)
            if (order_planes[i]) memset(order_planes[i]->stamp, 0, sizeof(order_planes[i]->stamp));
        order_cache_generation = 1;
    }
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_cache_release
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Libérer les caches appartenant au modèle détruit.
 *
 *  Paramètres :
 *    [entrée] model.
 *
 *  Retour :
 *    Aucun ; aucun effet si ce modèle n'est pas propriétaire.
 *
 *  Sorties et effets de bord :
 *    buffers libérés, tables et métadonnées remises à zéro.
 *
 *  Fonctionnement :
 *    Les pointeurs retournés auparavant par les fonctions de cache
 *    deviennent invalides.
 *
 * ----------------------------------------------------------------------------
 */
static void ordering_cache_release(Model3D *model)
{
    int i;
    /* Libérer uniquement les ressources du propriétaire pour éviter de
     * toucher à une autre vue.
     */
    if (order_cache_owner != model) return;
    order_asm_packet_plane=NULL;
    free(order_geometry_pairs);order_geometry_pairs=NULL;
    order_geometry_capacity=order_geometry_stride=order_geometry_generation=0;
    order_geometry_epoch=order_geometry_failed=0;order_geometry_faces=0;
    /* Libérer contours, blocs de plans et espace de clipping avant de
     * réinitialiser les métadonnées.
     */
    for(i=0;i<ORDER_CONTOUR_SLOTS;++i)free(order_contours[i].data);
    memset(order_contours,0,sizeof(order_contours));
    for (i = 0; i < ORDER_PLANE_BLOCKS; ++i) {
        free(order_planes[i]);order_planes[i] = NULL;
    }
    free(order_sh_workspace);order_sh_workspace = NULL;order_sh_capacity = 0;
    memset(order_ray_pairs, 0, sizeof(order_ray_pairs));
    memset(order_overlap_pairs, 0, sizeof(order_overlap_pairs));
    memset(order_plane_failed, 0, sizeof(order_plane_failed));
    memset(order_fixed_planes,0,sizeof(order_fixed_planes));
    memset(order_windings,0,sizeof(order_windings));
    order_cache_owner = NULL;order_cache_generation = 0;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_pair_slot
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Obtenir le slot de hash d'une paire orientée.
 *
 *  Paramètres :
 *    [entrée] model, propriétaire attendu ; table, cache de ORDER_PAIR_SLOTS
 *    entrées ; f1 et f2, identifiants à base zéro.
 *
 *  Retour :
 *    Adresse du slot, sans garantir un hit.
 *
 *  Sorties et effets de bord :
 *    invalide les caches si le modèle change.
 *
 *  Fonctionnement :
 *    Les collisions ne sont pas chaînées ici : ordering_pair_hit doit
 *    ensuite vérifier la clé entière.
 *
 * ----------------------------------------------------------------------------
 */
static OrderPairEntry *ordering_pair_slot(Model3D *model, OrderPairEntry *table,
                                         int f1, int f2)
{
    unsigned int slot;
    if (order_cache_owner != model) ordering_cache_invalidate(model);
    /* Calculer un hash orienté ; le slot seul ne valide jamais un résultat. */
    slot = (((unsigned int)f1 * 33U) ^ (unsigned int)f2) & (ORDER_PAIR_SLOTS - 1);
    return &table[slot];
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_pair_hit
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Vérifier toutes les composantes de la clé d'une paire mémorisée.
 *
 *  Paramètres :
 *    [entrée] entry ; f1/f2 orientés ; scale, projection ; context, contexte
 *    du test.
 *
 *  Retour :
 *    Vrai si génération et clé correspondent, faux sinon.
 *
 *  Fonctionnement :
 *    Aucun effet de bord.
 *
 * ----------------------------------------------------------------------------
 */
static int ordering_pair_hit(OrderPairEntry *entry, int f1, int f2,
                              Fixed32 scale, unsigned int context)
{
    /* Contrôler ensemble la vue, la paire, l'échelle et le contexte. */
    return entry->generation == order_cache_generation &&
           entry->f1 == f1 && entry->f2 == f2 && entry->scale == scale &&
           entry->context == context;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_pair_store
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Remplacer un slot de paire par le résultat de la vue courante.
 *
 *  Paramètres :
 *    [entrée] f1/f2, scale, context et result.
 *
 *    [entrée-sortie] entry.
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    La génération est publiée avec la clé complète ; une collision évince
 *    l'ancien résultat sans produire de faux hit.
 *
 * ----------------------------------------------------------------------------
 */
static void ordering_pair_store(OrderPairEntry *entry, int f1, int f2,
                                Fixed32 scale, unsigned int context, int result)
{
    /* Écrire le résultat avec sa clé complète et sa génération. */
    entry->f1=f1;entry->f2=f2;entry->scale=scale;entry->context=context;
    entry->result=result;entry->generation=order_cache_generation;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_workspace
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Obtenir un espace de travail partagé pour le clipping.
 *
 *  Paramètres :
 *    [entrée] bytes, taille demandée en octets.
 *
 *  Retour :
 *    Buffer existant ou agrandi, NULL si l'agrandissement est impossible.
 *
 *  Sorties et effets de bord :
 *    remplace l'ancien buffer uniquement après une allocation réussie ; le
 *    contenu précédent n'est pas conservé lors d'un agrandissement.
 *
 *  Fonctionnement :
 *    Ne pas libérer le pointeur retourné.
 *
 * ----------------------------------------------------------------------------
 */
static double *ordering_workspace(long bytes)
{
    /* Agrandir à la demande sans perdre l'ancien buffer si malloc échoue. */
    if (bytes > order_sh_capacity) {
        double *grown;
        if (bytes <= 0 || (unsigned long)bytes > (unsigned long)((size_t)-1)) return NULL;
        grown=(double*)malloc((size_t)bytes);
        if (!grown) return NULL;
        free(order_sh_workspace);order_sh_workspace=grown;order_sh_capacity=bytes;
        ORDER_COUNT(sh_allocations);
    }
    return order_sh_workspace;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_plane
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Convertir les quatre coefficients d'un plan fixe en double, avec cache.
 *
 *  Paramètres :
 *    [entrée] model ; face, identifiant valide ; fallback, tableau de quatre
 *    doubles fourni par l'appelant.
 *
 *  Retour :
 *    Coefficients en cache, ou fallback renseigné.
 *
 *  Sorties et effets de bord :
 *    allocation différée par blocs de 64 faces ; un échec d'allocation n'est
 *    pas retenté dans la même génération.
 *
 *  Fonctionnement :
 *    Le pointeur n'appartient pas à l'appelant. La borne MAX_FACES ne
 *    protège que le cache, pas l'accès aux données du modèle.
 *
 * ----------------------------------------------------------------------------
 */
static double *ordering_plane(Model3D *model, int face, double fallback[4])
{
    OrderPlaneBlock *block=NULL;
    double *out=fallback;
    int b,slot;
    if (order_cache_owner != model) ordering_cache_invalidate(model);
    /* Chercher un bloc de plans ; utiliser fallback si le stockage du cache
     * est indisponible.
     */
    if (face >= 0 && face < MAX_FACES) {
        b=face / ORDER_PLANE_BLOCK_SIZE;slot=face % ORDER_PLANE_BLOCK_SIZE;
        block=order_planes[b];
        if (!block && order_plane_failed[b] != order_cache_generation) {
            block=(OrderPlaneBlock*)calloc(1,sizeof(OrderPlaneBlock));order_planes[b]=block;
            if (!block) order_plane_failed[b]=order_cache_generation;
        }
        if (block) {
            out=block->plane[slot];
            if (block->stamp[slot] == order_cache_generation) return out;
            block->stamp[slot]=order_cache_generation;
        }
    }
    /* Convertir exactement les quatre coefficients de la face et compter
     * cette conversion si le profiling est activé.
     */
    out[0]=FIXED64_TO_FLOAT(model->faces.plane_a[face]);
    out[1]=FIXED64_TO_FLOAT(model->faces.plane_b[face]);
    out[2]=FIXED64_TO_FLOAT(model->faces.plane_c[face]);
    out[3]=FIXED64_TO_FLOAT(model->faces.plane_d[face]);
    ORDER_COUNT(plane_conversions);
    return out;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_fixed_plane_reset
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Invalider les plans préparés par changement d'époque.
 *
 *  Paramètres :
 *    Aucun paramètre explicite.
 *
 *  Retour :
 *    Aucun.
 *
 *  Sorties et effets de bord :
 *    époque incrémentée et paquet assembleur invalidé ; en cas de
 *    rebouclage, efface les plans et invalide le cache dense.
 *
 *  Fonctionnement :
 *    Les autres caches de chevauchement et de ray casting sont conservés.
 *
 * ----------------------------------------------------------------------------
 */
static void ordering_fixed_plane_reset(void)
{
    order_asm_packet_plane=NULL;
    /* Distinguer une remise à zéro géométrique d'un changement complet de vue. */
    ++order_fixed_plane_epoch;
    if(order_fixed_plane_epoch==0){
        memset(order_fixed_planes,0,sizeof(order_fixed_planes));
        order_geometry_generation=0;
    }
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_fixed_plane
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Préparer le plan d'une face et les conditions du chemin assembleur.
 *
 *  Paramètres :
 *    [entrée] model ; face, identifiant valide ; epsilon, tolérance du
 *    classement.
 *
 *  Retour :
 *    Slot partagé OrderFixedPlane, réutilisable seulement avec les mêmes
 *    face, génération, époque et epsilon.
 *
 *  Sorties et effets de bord :
 *    coefficients, côté de l'observateur, admissibilité assembleur et cache
 *    de côtés des sommets.
 *
 *  Fonctionnement :
 *    Une collision remplace le slot.
 *
 *    Le noyau n'est admissible que pour les coefficients 32 bits, |d| <=
 *    2^62 et epsilon positif ou nul.
 *
 * ----------------------------------------------------------------------------
 */
static OrderFixedPlane *ordering_fixed_plane(Model3D *model,int face,Fixed64 epsilon)
{
    OrderFixedPlane *p;
    FaceArrays3D *faces=&model->faces;
    if(order_cache_owner!=model)ordering_cache_invalidate(model);
    /* Vérifier la clé du plan avant de réutiliser coefficients et côtés des
     * sommets.
     */
    p=&order_fixed_planes[(unsigned int)face & (ORDER_FIXED_PLANE_SLOTS-1)];
    if(p->generation==order_cache_generation && p->epoch==order_fixed_plane_epoch &&
       p->face==face && p->epsilon==epsilon)return p;
    /* Réinitialiser les côtés et préparer la variante 32 bits seulement si
     * les bornes numériques le permettent.
     */
    memset(p->vertex_sides,0,sizeof(p->vertex_sides));
    p->a=faces->plane_a[face];p->b=faces->plane_b[face];
    p->c=faces->plane_c[face];p->d=faces->plane_d[face];
    p->a32=(Fixed32)p->a;p->b32=(Fixed32)p->b;p->c32=(Fixed32)p->c;
    p->observer=(p->d>epsilon)?1:((p->d < -epsilon)?-1:0);
    p->asm_ready=(p->a==(Fixed64)p->a32 && p->b==(Fixed64)p->b32 &&
                  p->c==(Fixed64)p->c32 && p->d>=-4611686018427387904LL &&
                  p->d<=4611686018427387904LL && epsilon>=0);
    p->face=face;p->epsilon=epsilon;p->generation=order_cache_generation;p->epoch=order_fixed_plane_epoch;
    return p;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_winding_slot
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Obtenir le slot de l'aire orientée d'une face.
 *
 *  Paramètres :
 *    [entrée] model et face.
 *
 *  Retour :
 *    Slot partagé, dont la validité reste à tester.
 *
 *  Sorties et effets de bord :
 *    changement de propriétaire si nécessaire ; collisions par masquage.
 *
 * ----------------------------------------------------------------------------
 */
static OrderWinding *ordering_winding_slot(Model3D *model,int face)
{
    if(order_cache_owner!=model)ordering_cache_invalidate(model);
    /* Sélectionner le slot ; une comparaison de clé reste nécessaire avant
     * lecture.
     */
    return &order_windings[(unsigned int)face & (ORDER_WINDING_SLOTS-1)];
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_winding_hit
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Tester la validité d'une aire orientée mémorisée.
 *
 *  Paramètres :
 *    [entrée] entry, face et scale.
 *
 *  Retour :
 *    Vrai si génération, face et projection correspondent ; faux sinon.
 *
 *  Fonctionnement :
 *    Aucun effet de bord.
 *
 * ----------------------------------------------------------------------------
 */
static int ordering_winding_hit(OrderWinding *entry,int face,Fixed32 scale)
{
    /* Comparer également l'échelle : une autre projection peut inverser ou
     * modifier l'aire.
     */
    return entry->generation==order_cache_generation && entry->face==face && entry->scale==scale;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_winding_store
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Mémoriser le double de l'aire orientée d'une face projetée.
 *
 *  Paramètres :
 *    [entrée] face, scale et area2.
 *
 *    [entrée-sortie] entry
 *
 *  Retour :
 *    Aucun.
 *
 *  Fonctionnement :
 *    La valeur est marquée avec la génération courante.
 *
 * ----------------------------------------------------------------------------
 */
static void ordering_winding_store(OrderWinding *entry,int face,Fixed32 scale,double area2)
{
    /* Associer l'aire orientée à la face et à sa projection dans cette vue. */
    entry->face=face;entry->scale=scale;entry->area2=area2;
    entry->generation=order_cache_generation;
}


/*
 * ----------------------------------------------------------------------------
 *  ordering_geometry_slot
 * ----------------------------------------------------------------------------
 *
 *  Rôle :
 *    Obtenir l'octet du cache dense qui contient une relation de plans
 *    orientée.
 *
 *  Paramètres :
 *    [entrée] model ; f1/f2 à base zéro ; epsilon.
 *
 *  Retour :
 *    Octet partagé à décoder sur deux bits, ou NULL pour le chemin de repli.
 *
 *  Fonctionnement :
 *    Aucun résultat n'est écrit ici. Le cache dense est limité à 256 faces.
 *    Génération, époque, nombre de faces et epsilon déclenchent sa remise à
 *    zéro ; chaque octet contient quatre codes. Un échec de réserve est
 *    mémorisé pour ne pas réallouer en boucle dans la vue.
 *
 * ----------------------------------------------------------------------------
 */
static unsigned char *ordering_geometry_slot(Model3D *model,int f1,int f2,Fixed64 epsilon)
{
    int nf=model->faces.face_count;
    unsigned int stride,bytes;
    /* Refuser les modèles trop grands et les indices hors limites pour le
     * cache dense.
     */
    if(nf<=0 || nf>ORDER_GEOMETRY_DENSE_LIMIT ||
       (unsigned int)f1>=(unsigned int)nf || (unsigned int)f2>=(unsigned int)nf)return NULL;
    if(order_cache_owner!=model)ordering_cache_invalidate(model);
    stride=((unsigned int)nf+3U)>>2;
    bytes=(unsigned int)nf*stride;
    /* Réserver le tableau compact, sans répéter un échec d'allocation dans
     * la même génération.
     */
    if(order_geometry_capacity<bytes){
        unsigned char *grown;
        if(order_geometry_failed==order_cache_generation)return NULL;
        grown=(unsigned char*)malloc(bytes);
        if(!grown){order_geometry_failed=order_cache_generation;return NULL;}
        free(order_geometry_pairs);order_geometry_pairs=grown;order_geometry_capacity=bytes;
        order_geometry_generation=0;
    }
    /* Effacer les codes après changement de clé et construire une fois les
     * pointeurs de lignes.
     */
    if(order_geometry_generation!=order_cache_generation ||
       order_geometry_epoch!=order_fixed_plane_epoch || order_geometry_faces!=nf ||
       order_geometry_epsilon!=epsilon){
        unsigned char *row=order_geometry_pairs;
        int i;
        memset(order_geometry_pairs,0,bytes);


        for(i=0;i<nf;++i){order_geometry_rows[i]=row;row+=stride;}
        order_geometry_stride=stride;order_geometry_faces=nf;
        order_geometry_epoch=order_fixed_plane_epoch;order_geometry_epsilon=epsilon;
        order_geometry_generation=order_cache_generation;
    }
    /* Retourner l'octet partagé ; le choix des deux bits de f2 incombe à
     * l'appelant.
     */
    return order_geometry_rows[f1]+((unsigned int)f2>>2);
}
