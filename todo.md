# Notes 

NOTE: 2026-01-15 - Commits 1f089f3 & 8a537d0 : Interactive face inspection and navigation
- `inspect_faces_before` & `inspect_faces_after`: Added dual-mode interactive move
  * Touche 'A': déplace target en considérant TOUTES les faces mal placées
  * Touche 'O': déplace target en considérant SEULEMENT les faces avec overlap
  * Affichage du nombre d'overlaps dans le message de statut
  * Gestion mémoire: tableau overlaps conservé jusqu'à la fin pour les opérations de déplacement
- Nouvelle fonction `showFace` (touche 'V'):
  * Affiche une face en filled vert sur le modèle en wireframe
  * Navigation interactive sans quitter le mode graphique:
    - Flèches gauche/droite: face ID précédente/suivante (avec wraparound)
    - Flèches haut/bas: position suivante/précédente dans sorted_face_indices
    - N'importe quelle autre touche: sortie
  * Affichage de l'ID de la face et de sa position dans la liste triée
- Compile et push effectués (branche main).

REMOVED: 2026-01-14 - Commit 067edb4 : painter_super removed (see commit for original implementation)
- Reason: experimental mode regressed intermittently; removed per request.
- Compile & push performed.
SUPER (abandonné) :
tu va faire une version 2 de painter_correct, qui va s'appeler painter_super. Elle fera exactement la même chose mais : dans la partie after, si target est déplaccée, mémorise cette face et l'indice. Si elle doit ensuite etre déplacée, elle ne peut être placée à un indice inférieur. Il y a donc 2 marquages  : un pour les before (existant) et un our les after (à faire dans painter_super). Refais ensuite une passe before + after, en respectant les marques.

NOTE: 2026-01-13 - Commit 339afdf : painter_correct - éviter oscillations
- Ajout de `min_allowed_pos[]` pour marquer les cibles déplacées dans la passe BEFORE.
- Clamp dans la passe AFTER pour empêcher qu'une face déplacée retourne à un index plus élevé.
- Compile et push effectués (branche main).



## fixed:
probleme de overlap : 
avion, 90,20,0 (+ zoom) : o => 45 et 2 : overlap = NO : erreur !
avion, 110,20,0 (+ zoom) : o => 45 et 3 : overlap = YES : erreur !
Enregistrer dans un fichier les coordonnees des faces du modele ?

## fixed:
probleme fil de fer : à voir... solution provisoire : dans bigloop, faire un processModelFast(model, &params, filename);
Solution  ==> modification de processModelFast : if framePolyOnly : pas de tri des faces. Et bigloop quand framePolyOnly = 0.

## TODO 

TODO:
avec les angle 350 / 50, avion, les faces 3 et 44 sont en overlap, et sont inversées suivant le test after.

TODO:
aVérifier les angles h = 240, h = 330, h = 190.



## Discussions 1
IA : 
ourquoi (brièvement) 💡
Si tu compares A,B et B,C avec des tests décisifs (→ A<B et B<C), puis A vs C est inconclusif et tu appliques Zmin pour trancher, tu peux obtenir C<A par Zmin.
Résultat : A<B, B<C, C<A → cycle (non‑transitif).
Autrement dit, mélanger des relations « décidées » et des tie‑breakers locaux ne garantit pas la transitivité du comparateur global.
Solution recommandée 🔧
Construire d’abord un graphe dirigé à partir des relations décisives (arêtes f1→f2 quand test dit « f1 avant f2 »).
Tenter un tri topologique (Kahn) sur ce graphe :
si acyclique → ordre compatible avec toutes relations décisives ;
si cycle détecté → résoudre le cycle en orientant/arbitrant les arêtes du cycle selon Zmin (ou une heuristique déterministe), puis relancer le topo.
Ceci garantit qu’on respecte prioritairement les relations décidées et qu’on n’introduit pas d’ordre contradictoire arbitraire.
Proposition d’implémentation
J’implémente le pipeline : construire graphe → topo → detection de cycles → résolution par Zmin → topo final, avec logs (mode debug).
Optionnel : un mode « agressif » qui applique ensuite swaps adjacents pour rapprocher/corriger l’ordre effectif dans la structure existante.
Tu veux que je commence par l’implémentation du tri

## Discussion 2
Parfait — je vais analyser rapidement quelles parties de painter_correct sont de bons candidats pour une réécriture en assembleur et proposer un plan sûr.Parfait! Voici une analyse courte et un plan proposé pour décider si (et quoi) réécrire en assembleur. Ma recommandation: mesurer d'abord, optimiser en C quand possible, puis réécrire un petit noyau en assembleur si ça paye.

🔎 Où se trouvent probablement les goulots d'étranglement
Boucles de balayage (window / full-scan) — itérations sur indices et accès mémoire fréquents. Bon candidat si le travail dans la boucle est très simple (comparaisons, index arithmetic).
Indexation des caches (f * n + t) — multiplication et accès mémoire répétés; on peut éviter la multiplication par incréments/pointeurs.
Mise à jour des positions après déplacement (move_element_remove_and_insert) — memmove + boucle de mise à jour de pos_of_face. Un bon candidat pour assembler un déplacement spécial qui met à jour pos_of_face en un seul passage.
Tests AABB (bbox) — simples comparaisons entières ; micro‑optimisable mais gains modestes.
projected_polygons_overlap / pair_order_relation — fonctions géométriques lourdes et branchées; possibles gains importants mais coût élevé (complexité, risque) si réécrites en assembleur. Mieux d'optimiser algorithmiquement / en C d'abord.
✅ Avantages et risques de l'assembleur
Avantages: possible accélération significative sur petits noyaux très chauds (déplacements, tests simples), utilisation de registres et d'accès mémoire séquentiels optimisés pour la CPU 65C816.
Risques: code plus complexe, difficile à maintenir/debugger, portabilité nulle, risque d'erreurs subtiles (surtout sur manipulations d'indices ou d'allocations). Gains limités si le vrai coût est dans des tests géométriques coûteux (fuites de cache ou opérations flottantes).
Plan recommandé (sûr et itératif) 🔧
Mesurer — remettre brièvement des compteurs/instrumentation (ou utiliser ceux que vous aviez) pour confirmer les hotspots (boucles, memmove, overlap calls).
Micro-optimisations en C (faible risque) :
Eviter multiplications répétées en tenant un pointeur / indice de ligne de cache.
Marquer petites fonctions static inline pour inlining.
Fusionner la mise à jour memmove + pos_of_face dans une seule boucle C si ce n'est pas déjà fait.
Prototype assembleur ciblé :
Commencer par move_element_remove_and_insert en assembleur 65C816 (déplacement optimisé + mise à jour pos_of_face).
Écrire tests unitaires / cas réels pour valider exactitude et performance.
Re-mesurer. Si gain suffisant → garder et documenter. Sinon → annuler et évaluer autre cible (p.ex. hot loop d’itération).
Considérations avancées : si projected_polygons_overlap est le vrai hotspot, d’abord envisager algorithmes alternatifs (caches, bounding hierarchy) avant d’attaquer en assembleur.
Proposition concrète
Je peux ajouter rapidement des compteurs légers (si vous le voulez) et lancer un essai avec votre modèle lourd pour confirmer où optimiser.
Ou je peux implémenter un prototype assembleur pour move_element_remove_and_insert (faible risque, gros potentiel pour les cas où beaucoup de déplacements ont lieu).
Que préférez-vous :

Je lance la mesure (ajout de compteurs + build + test) ?
Je prototype l’assembleur sur la fonction de move et je vous fournis le patch + bench ?
Dites-moi quelle option vous voulez et je m’en occupe.


## Algo BRUNO : 
voilà ce qu'on va faire : dans une nouvelle fonciton painter (painter_correct, par exemple), tu vas :

Pour chaque face, que j'appelle target :
1/ faire un test exactement comme inspect_before (sans les output à l'écran). Si tu trouves 1 ou plusieurs faces mal placées ET qui overlap avec target,(qui devraient être après dans la liste), place target dans la liste AVANT le face trouvée ayant le plus petit indice.
2/ fait un test exactement comme inspect_after (sans les output à l'écran). Si tu trouves 1 ou plusieurs faces mal placées ET qui overlap avec target, (qui devraient être avant dans la liste), place target dans la liste APRES le face trouvée aayant le plus grand indice.
Fais le 

Complément : 
Dans la partie before (partie 1) de painter_correct  on marque target comme déplacée et on mémorise sa nouvelle position. Puis dans la partie after (partie 2) si on doit déplacer un face, et que cette face a déjà été déplacée, on ne peut pas la déplacer à un indice supérieur (mais on peut la déplacer à un indice inférieur). Fais juste ça, rien de plus.

# inspect_faces_before & inspect_faces_after
a la fin de la fonction inspect_faces_before, après l'affichage des polygones, si le nombre de faces mal placées > 0, il faut permettre à l'utilisateur d'appuyer sur la touche F pour permuter les faces en déplacçant target (= la face choisie par l'utilisateur) AVANT le face trouvée ayant le plus petit indice.

Fais la même chose avec inspect_faces_after : si le nombre de faces mal placées > 0, il faut permettre à l'utilisateur d'appuyer sur la touche F pour permuter les faces en déplacçant target (= la face choisie par l'utilisateur) APRES le face trouvée aayant le plus grand indice.


Créer tout d'abord une fonction before qui prend 2 id de face en argument.
Cette fonction a pour objectif de déterminer si la 1ère face en argument doit être placée AVANT (ordre de dessin) dans la sorted list.
Cette fonction fait l'équivalent des tests 4 5 de painter_newell_sancha : si test 4 réussi : return 1, si test 4 échoue, exécute test 7, si test 5 réussit : return 1, si test 5 échoue return -1. Si tous les points testés ont une valeur > -epsilon et <epsilon, alors les 2 faces sont coplanaire, after renvoie 0 
 
Créer tout d'abord une fonction after qui prend 2 id de face en argument.
Cette fonction a pour objectif de déterminer si la 1ère face en argument doit être placée APRES (ordre de dessin) dans la sorted list.
Cette fonction fait l'équivalent des tests 6 7 de painter_newell_sancha : si test 6 réussi : return 1, si test 6 échoue, exécute test 7, si test 7 réussit : return 1, si test 5 échoue return -1. Si tous les points testés ont une valeur > -epsilon et <epsilon, alors les 2 faces sont coplanaires, after renvoie 0 


Alloué de la mémoire pour une liste d'ID, = nombre de face total de l'objet (sans culling).
Pour chaque face, que j'appelle target :

1/ établir une liste des faces qui chevauchent target en x ou en y
2/ Reduire cette liste en supprimant les sans sans overlap avec target.
3/ Coupe cette liste en 2  : les faces placées avant dans la sorted_list et celles placées après.
4/ pour les faces placées avant dans la sorted list : appliquer la fonciton before. Si une ou plusieurs face ont -1 en retour de before, place target dans la liste AVANT la face trouvée ayant le plus petit indice.
5/ pour les faces placées après dans la sorted list : appliquer la fonciton after. Si une ou plusieurs face ont -1 en retour de before, place target dans la liste AVANT la face trouvée ayant le plus petit indice.
