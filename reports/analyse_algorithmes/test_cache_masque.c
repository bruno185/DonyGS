#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
typedef struct {int face1,face2;int8_t relation,occupied;} PairCacheEntry;
typedef struct {PairCacheEntry *slots;int capacity;}PairCache;
static PairCache* pair_cache_create(int capacity) {
    /* Allouer la structure puis ses slots remis à zéro ; libérer la
     * structure si la seconde allocation échoue.
     */
    PairCache* c = (PairCache*)malloc(sizeof(PairCache));
    if (!c) return NULL;
    c->capacity = capacity;
    c->slots = (PairCacheEntry*)calloc(capacity, sizeof(PairCacheEntry));
    if (!c->slots) { free(c); return NULL; }
    return c;
}
static int pair_cache_slot(PairCache* c, int f1, int f2) {

    /* Normaliser les identifiants et réduire le hash modulo la capacité. */
    int a = (f1 < f2) ? f1 : f2;
    int b = (f1 < f2) ? f2 : f1;
    unsigned int h = (unsigned int)(a * 2654435761u ^ b * 2246822519u);
    return (int)(h % (unsigned int)c->capacity);
}
static int pair_cache_find(PairCache* c, int f1, int f2) {
    int slot, i;
    if (!c) return 0;
    slot = pair_cache_slot(c, f1, f2);
    /* Sonder huit cases au maximum, comparer les clés exactes et réorienter
     * le résultat.
     */
    for (i = 0; i < 8; i++) {
        int pos = (slot + i) % c->capacity;
        PairCacheEntry* e = &c->slots[pos];
        if (!e->occupied) return 0;
        if ((e->face1 == f1 && e->face2 == f2)) return  e->relation;
        if ((e->face1 == f2 && e->face2 == f1)) return -e->relation;
    }
    return 0;
}
static void pair_cache_insert(PairCache* c, int f1, int f2, int8_t relation) {
    int slot, i;
    if (!c) return;
    slot = pair_cache_slot(c, f1, f2);
    /* Réutiliser le premier slot vide ou la paire déjà présente, dans l'un
     * ou l'autre sens.
     */
    for (i = 0; i < 8; i++) {
        int pos = (slot + i) % c->capacity;
        PairCacheEntry* e = &c->slots[pos];
        if (!e->occupied ||
            (e->face1 == f1 && e->face2 == f2) ||
            (e->face1 == f2 && e->face2 == f1)) {
            e->face1    = f1;
            e->face2    = f2;
            e->relation = relation;
            e->occupied = 1;
            return;
        }
    }
}
static void pair_cache_destroy(PairCache* c) {
    /* Libérer les slots avant la structure propriétaire. */
    if (c) { free(c->slots); free(c); }
}static PairCache* pair_cache_create_mask(int capacity) {
    /* Allouer la structure puis ses slots remis à zéro ; libérer la
     * structure si la seconde allocation échoue.
     */
    PairCache* c = (PairCache*)malloc(sizeof(PairCache));
    if (!c) return NULL;
    c->capacity = capacity;
    c->slots = (PairCacheEntry*)calloc(capacity, sizeof(PairCacheEntry));
    if (!c->slots) { free(c); return NULL; }
    return c;
}
static int pair_cache_slot_mask(PairCache* c, int f1, int f2) {

    /* Normaliser les identifiants et réduire le hash modulo la capacité. */
    int a = (f1 < f2) ? f1 : f2;
    int b = (f1 < f2) ? f2 : f1;
    unsigned int h = (unsigned int)(a * 2654435761u ^ b * 2246822519u);
    return (int)(h & ((unsigned int)c->capacity - 1U));
}
static int pair_cache_find_mask(PairCache* c, int f1, int f2) {
    int slot, i;
    if (!c) return 0;
    slot = pair_cache_slot_mask(c, f1, f2);
    /* Sonder huit cases au maximum, comparer les clés exactes et réorienter
     * le résultat.
     */
    for (i = 0; i < 8; i++) {
        int pos = (slot + i) & (c->capacity - 1);
        PairCacheEntry* e = &c->slots[pos];
        if (!e->occupied) return 0;
        if ((e->face1 == f1 && e->face2 == f2)) return  e->relation;
        if ((e->face1 == f2 && e->face2 == f1)) return -e->relation;
    }
    return 0;
}
static void pair_cache_insert_mask(PairCache* c, int f1, int f2, int8_t relation) {
    int slot, i;
    if (!c) return;
    slot = pair_cache_slot_mask(c, f1, f2);
    /* Réutiliser le premier slot vide ou la paire déjà présente, dans l'un
     * ou l'autre sens.
     */
    for (i = 0; i < 8; i++) {
        int pos = (slot + i) & (c->capacity - 1);
        PairCacheEntry* e = &c->slots[pos];
        if (!e->occupied ||
            (e->face1 == f1 && e->face2 == f2) ||
            (e->face1 == f2 && e->face2 == f1)) {
            e->face1    = f1;
            e->face2    = f2;
            e->relation = relation;
            e->occupied = 1;
            return;
        }
    }
}
static void pair_cache_destroy_mask(PairCache* c) {
    /* Libérer les slots avant la structure propriétaire. */
    if (c) { free(c->slots); free(c); }
}
static uint32_t seed=816;static uint32_t rnd(void){seed=seed*1664525u+1013904223u;return seed;}
int main(void){int caps[]={1024,2048,4096},c,i;for(c=0;c<3;c++){PairCache*a=pair_cache_create(caps[c]),*b=pair_cache_create_mask(caps[c]);for(i=0;i<200000;i++){int f=rnd()%640,g=rnd()%640;assert(pair_cache_slot(a,f,g)==pair_cache_slot_mask(b,f,g));if(i%3){int8_t rel=(rnd()&1)?1:-1;pair_cache_insert(a,f,g,rel);pair_cache_insert_mask(b,f,g,rel);}assert(pair_cache_find(a,f,g)==pair_cache_find_mask(b,f,g));assert(pair_cache_find(a,g,f)==pair_cache_find_mask(b,g,f));}assert(memcmp(a->slots,b->slots,caps[c]*sizeof(PairCacheEntry))==0);pair_cache_destroy(a);pair_cache_destroy_mask(b);}puts("PASS: modulo versus mask: 600000 mixed operations, all 3 capacities, reverse relations and final cache contents identical.");return 0;}
