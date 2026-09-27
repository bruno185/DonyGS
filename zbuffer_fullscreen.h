/* zbuffer_fullscreen.h
 *
 * Z-buffer plein ecran 16 bits pour Apple IIGS (320x200, mode SHR).
 * Remplace le Z-buffer par scanline (une seule ligne, reinitialisee a
 * chaque ligne, en float natif) par un tampon de profondeur PERSISTANT
 * sur toute la frame, en entier 16 bits, reparti sur 2 banks de 64 Ko.
 *
 * ATTENTION PRECISION (decision explicite, risque assume) :
 * le renderer scanline existant garde deliberement la profondeur en
 * float32 (voir l'entete de renderModelScanlineZBuffer_old) a cause
 * d'un cas reel deja diagnostique : deux faces (13 et 18) dont les
 * profondeurs interobserver s'entrelacent a moins de 1% d'ecart dans
 * leur zone de recouvrement a l'ecran -- un Fixed32 (16.16) s'etait
 * revele insuffisant pour ce cas. Un entier 16 bits offre MOINS de
 * precision utile qu'un Fixed32 pour la meme plage de valeurs, donc
 * ce Z-buffer plein ecran peut reproduire le meme defaut d'occlusion
 * sur des faces proches similaires. Retenu malgre tout par choix.
 * ZBUFFER_INV_Z_SCALE ci-dessous doit etre calibre empiriquement sur
 * la plage reelle de 1/z de vos modeles pour exploiter au mieux les
 * 16 bits disponibles (voir ZBuffer_QuantizeInvZ).
 *
 * Codage stocke : "distance inversee" 16 bits, PAS 1/z directement :
 *   valeur_stockee = 0xFFFF - clamp(round(inv_z * ZBUFFER_INV_Z_SCALE), 0, 0xFFFF)
 * Ce choix garde la meme convention "plus petit = plus proche, 0xFFFF
 * = vide/jamais dessine" que le premier jet de ce module, ce qui
 * permet un test a une seule branche (BCS) dans la boucle assembleur
 * plutot que deux. inv_z = 0 (point a l'infini) mappe naturellement
 * sur 0xFFFF, donc "vide" et "infiniment loin" sont la meme valeur --
 * pas de cas particulier a gerer pour l'etat initial.
 */

#ifndef ZBUFFER_FULLSCREEN_H
#define ZBUFFER_FULLSCREEN_H

#define ZBUF_WIDTH       320
#define ZBUF_HEIGHT      200
#define ZBUF_ROW_WORDS   ZBUF_WIDTH               /* 320 mots 16 bits / ligne */
#define ZBUF_ROW_BYTES   (ZBUF_WIDTH * 2)          /* 640 octets / ligne       */
#define ZBUF_FAR_VALUE   0xFFFFU                   /* "vide" = infiniment loin */

/* A CALIBRER sur la plage reelle de inv_z (1/zo) de vos modeles.
 * Trop petit -> tout se tasse pres de 0xFFFF (perte de resolution
 * pres de la camera, ou les conflits de profondeur sont les plus
 * genants). Trop grand -> saturation (clamp) pour les points proches,
 * ce qui les rend tous "a egalite" en profondeur. Methode simple :
 * loguer temporairement min/max de inv_z sur quelques scenes typiques
 * et choisir ZBUFFER_INV_Z_SCALE pour que le inv_z max observe soit
 * proche de 65535 / ZBUFFER_INV_Z_SCALE.
 */
#define ZBUFFER_INV_Z_SCALE  7979000.0f
/* Valeur de secours uniquement, utilisee tant que
 * ZBuffer_SetScaleForFrame() n'a pas encore ete appelee au moins une
 * fois (ex. tout premier frame). En temps normal l'echelle reelle est
 * recalculee CHAQUE FRAME par ZBuffer_SetScaleForFrame() a partir du
 * inv_z max de la scene en cours -- voir plus bas. Une echelle fixe a
 * ete testee et s'est reveleee incorrecte des que le niveau de zoom
 * change (le max de inv_z change avec le zoom, une echelle calibree
 * sur une scene sature a 0xFFFF sur une autre plus zoomee, faisant
 * passer plusieurs faces a egalite de profondeur -> ordre aleatoire). */

#define ZBUFFER_TARGET_MAX_CODE  60000.0f  /* marge sous 65535 */

/* Recalcule l'echelle de quantification pour la frame en cours, a
 * partir du plus grand inv_z reellement observe sur cette frame (pas
 * une constante figee). A appeler UNE FOIS PAR FRAME, apres avoir
 * calcule inv_z[] pour tous les sommets visibles et avant le premier
 * appel a ZBuffer_QuantizeInvZ de cette frame. Ne fait rien (garde
 * l'echelle precedente) si max_inv_z est nul ou negatif (scene vide/
 * degenerescente), pour eviter une division par zero.
 */
void ZBuffer_SetScaleForFrame(float max_inv_z);

typedef unsigned short  UWORD16;
typedef unsigned long   ULONG32;
typedef short           SWORD16;   /* pour dz_step signe */

/* Pointeur 24 bits (bank:offset). Sur ORCA/C pour l'IIGS, un pointeur
 * "normal" est deja un pointeur 24 bits (pas de distinction near/far
 * a declarer) -- pas de mot-cle special necessaire, contrairement a
 * ce qui etait suppose dans une version precedente de ce fichier
 * (le mot-cle "far" n'existe pas dans ce compilateur : erreur de
 * compilation confirmee).
 */
typedef UWORD16 * FarWordPtr;

/* Table de 200 pointeurs far, un par ligne d'ecran, precalculee une
 * seule fois par ZBuffer_Init(). Adressage lineaire simple
 * (base + y*ZBUF_ROW_BYTES) : AUCUN decoupage special n'est requis
 * pour eviter qu'une ligne chevauche la frontiere entre les deux
 * banks, car l'adressage indirect long du 65816 ([ptr],y) fait une
 * vraie addition 24 bits avec report dans l'octet de banque -- voir
 * la note d'architecture en tete de ZBuffer_TestSetRow.
 */
extern FarWordPtr zbuf_row[ZBUF_HEIGHT];

/* Alloue 2 banks (128 Ko utiles, alignes sur une frontiere de bank)
 * via le memory manager GS/OS, construit zbuf_row[], puis appelle
 * ZBuffer_Clear(). Retourne 0 si l'allocation echoue, 1 sinon.
 */
int  ZBuffer_Init(void);

/* Libere le bloc alloue par ZBuffer_Init. */
void ZBuffer_Shutdown(void);

/* Remet toute la frame a ZBUF_FAR_VALUE. Version C, lisible, pour le
 * debug -- preferer ZBuffer_ClearFast (asm) pour le rendu reel.
 */
void ZBuffer_Clear(void);

/* Pointeur far vers le debut de la ligne y (0..199). */
#define ZBuffer_RowPtr(y)  (zbuf_row[y])

/* Convertit un 1/z natif (float) en la representation 16 bits
 * stockee dans le Z-buffer (voir le codage explique en tete de ce
 * fichier). A utiliser une fois par extremite de span (2 appels),
 * pas par pixel -- l'interpolation par pixel se fait ensuite en
 * entier via dz_step, cf. ZBuffer_TestSetRow.
 */
UWORD16 ZBuffer_QuantizeInvZ(float inv_z);

/* Test-and-set haut niveau, un pixel a la fois (debug / chemins non
 * critiques). Retourne 1 si z est plus proche (0xFFFF - inv_z code)
 * que ce qui etait deja stocke, 0 sinon.
 */
int ZBuffer_TestAndSet(int x, int y, UWORD16 z);

/* ---- routines assembleur en ligne (voir zbuffer_fullscreen.c) ---- */

/* Remplit les DEUX banks du Z-buffer avec ZBUF_FAR_VALUE. A appeler
 * une fois par frame. bank_lo/bank_hi = numeros de bank du Z-buffer
 * (bank_hi = bank_lo + 1), obtenus a l'init a partir de l'octet de
 * poids fort du pointeur de base alloue.
 */
asm void ZBuffer_ClearFast(int bank_lo, int bank_hi);

/* Teste et met a jour "count" pixels consecutifs d'une ligne du
 * Z-buffer, profondeur interpolee lineairement (z_start, +dz_step
 * signe par pixel). zbuf_ptr = ZBuffer_RowPtr(y) decale de x_start
 * mots (voir integration dans le renderer). Utilise l'adressage
 * indirect long [ptr],y : correct meme si le span chevauche la
 * frontiere entre les deux banks (voir note d'architecture dans
 * zbuffer_fullscreen.c).
 *
 * Ne fait QUE le test/ecriture de profondeur -- l'ecriture couleur
 * (nibble pack/unpack dans le framebuffer SHR, cf. drawPixel) reste
 * a fusionner separement ; voir la note de fin de fichier .c sur le
 * cout du changement de largeur d'accumulateur (SEP/REP) que ca
 * implique en boucle serree.
 */
asm void ZBuffer_TestSetRow(FarWordPtr zbuf_ptr, UWORD16 z_start,
                             SWORD16 dz_step, int count);

#endif /* ZBUFFER_FULLSCREEN_H */
