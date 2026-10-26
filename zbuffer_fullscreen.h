/*
 * ============================================================================
 *  zbuffer_fullscreen.h — Interface et formats du Z-buffer plein écran
 * ============================================================================
 *
 *  Rôle :
 *    Déclarer les dimensions, codes de profondeur, types et opérations
 *    communes aux rendus plein écran direct et offscreen.
 *
 *  Organisation :
 *    Expose la table de pointeurs de lignes, le cycle de vie du Z-buffer, la
 *    quantification de 1/z, le test Z et le masque facultatif des côtés.
 *    Contient aussi deux primitives assembleur historiques à paramètres
 *    ORCA/C.
 *
 *  Dépendances et contraintes :
 *    À inclure après les types nécessaires du projet. Les coordonnées des
 *    primitives non clippées doivent être valides. Les pointeurs de lignes
 *    ne sont utilisables qu'après initialisation réussie ; le masque
 *    d'arêtes est emprunté, jamais possédé par le module.
 *
 * ============================================================================
 */
#ifndef ZBUFFER_FULLSCREEN_H
#define ZBUFFER_FULLSCREEN_H


#ifndef ZBUFFER_BORDER_TOLERANCE
/* Constante publique conservée ; vérifier ses usages avant de lui attribuer
 * un effet sur tous les contours.
 */
#define ZBUFFER_BORDER_TOLERANCE 0
#endif


/* Masque emprunté : un octet par côté indexé comme vertex_indices_buffer ; 0
 * masque, NULL réactive tout. L'appelant conserve la propriété.
 */
void ZBuffer_SetEdgeFlags(const unsigned char *flags, unsigned long count);

/* Chaque ligne contient 320 mots 16 bits ; les pointeurs far peuvent
 * traverser une frontière de banque.
 */
#define ZBUF_WIDTH       320
#define ZBUF_HEIGHT      200
#define ZBUF_ROW_WORDS   ZBUF_WIDTH
#define ZBUF_ROW_BYTES   (ZBUF_WIDTH * 2)
#define ZBUF_FAR_VALUE   0xFFFFU


#define ZBUFFER_INV_Z_SCALE  7979000.0f
#define ZBUFFER_TARGET_MAX_CODE  60000.0f


/* Adapter l'échelle au maximum 1/z ; conserve la précédente pour un maximum
 * trop petit.
 */
void ZBuffer_SetScaleForFrame(float max_inv_z);


typedef unsigned short  UWORD16;
typedef unsigned long   ULONG32;
typedef short           SWORD16;


typedef UWORD16 * FarWordPtr;


/* Table partagée de lignes, valide uniquement entre initialisation réussie
 * et shutdown.
 */
extern FarWordPtr zbuf_row[ZBUF_HEIGHT];


/* Cycle de vie : Init idempotent (1 réussite), Shutdown libère aussi le
 * scratch du rendu direct, Clear écrit les lignes à $FFFF.
 */
int  ZBuffer_Init(void);
void ZBuffer_Shutdown(void);
void ZBuffer_Clear(void);

#define ZBuffer_RowPtr(y)  (zbuf_row[y])

/* Quantifier 1/z en code inversé. TestAndSet exige x/y valides et remplace
 * seulement une profondeur strictement plus lointaine.
 */
UWORD16 ZBuffer_QuantizeInvZ(float inv_z);
int ZBuffer_TestAndSet(int x, int y, UWORD16 z);

/* Primitives assembleur historiques : effacement de deux banques complètes
 * ou suite de codes sans dessin ; paramètres ORCA/C.
 */
asm void ZBuffer_ClearFast(int bank_lo, int bank_hi);
asm void ZBuffer_TestSetRow(FarWordPtr zbuf_ptr, UWORD16 z_start,
                             SWORD16 dz_step, int count);


#endif
