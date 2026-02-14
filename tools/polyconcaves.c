#include <stdbool.h>
#include <float.h>

typedef struct { double x, y; } Vec;

/*----------------------------------------------------------
   Produit vectoriel 2D
----------------------------------------------------------*/
static double cross(Vec a, Vec b) {
    return a.x * b.y - a.y * b.x;
}

/*----------------------------------------------------------
   Test si un point est exactement sur un segment
   → dans notre logique : touching → pas inside strict
----------------------------------------------------------*/
static bool point_on_segment(Vec p, Vec a, Vec b) {
    // Test colinéarité
    double c = cross((Vec){b.x - a.x, b.y - a.y},
                     (Vec){p.x - a.x, p.y - a.y});
    if (c != 0.0)
        return false;

    // Test projection dans le segment
    double dot = (p.x - a.x)*(b.x - a.x) + (p.y - a.y)*(b.y - a.y);
    if (dot < 0.0)
        return false;

    double len2 = (b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y);
    if (dot > len2)
        return false;

    return true;
}

/*----------------------------------------------------------
   Ray casting robuste — inside strict (exclut le bord)
----------------------------------------------------------*/
static bool point_in_polygon_strict(Vec p, Vec *poly, int n) {
    bool inside = false;

    for (int i = 0, j = n - 1; i < n; j = i++) {
        Vec a = poly[j];
        Vec b = poly[i];

        // 1) Si le point est sur une arête → touching → pas inside strict
        if (point_on_segment(p, a, b))
            return false;

        // 2) Ray casting classique, version robuste
        bool cond = ((a.y > p.y) != (b.y > p.y));
        if (cond) {
            double x_intersect =
                a.x + (b.x - a.x) * (p.y - a.y) / (b.y - a.y);

            if (x_intersect > p.x)
                inside = !inside;
        }
    }

    return inside;
}

/*----------------------------------------------------------
   Intersection stricte entre deux segments
   → pas aux extrémités
   → pas colinéaire
----------------------------------------------------------*/
static bool segments_intersect_strict(Vec p1, Vec p2, Vec q1, Vec q2) {
    Vec r = {p2.x - p1.x, p2.y - p1.y};
    Vec s = {q2.x - q1.x, q2.y - q1.y};

    double rxs = cross(r, s);
    if (rxs == 0.0)
        return false; // parallèles ou colinéaires → pas intersection stricte

    double t = cross((Vec){q1.x - p1.x, q1.y - p1.y}, s) / rxs;
    double u = cross((Vec){q1.x - p1.x, q1.y - p1.y}, r) / rxs;

    return (t > 0.0 && t < 1.0 && u > 0.0 && u < 1.0);
}

/*----------------------------------------------------------
   Test général concave/convexe — overlap strict (non touching)
----------------------------------------------------------*/
bool polygons_overlap_strict(Vec *A, int nA, Vec *B, int nB) {

    // 1. Intersection stricte des arêtes
    for (int i = 0; i < nA; i++) {
        Vec a1 = A[i];
        Vec a2 = A[(i+1)%nA];

        for (int j = 0; j < nB; j++) {
            Vec b1 = B[j];
            Vec b2 = B[(j+1)%nB];

            if (segments_intersect_strict(a1, a2, b1, b2))
                return true;
        }
    }

    // 2. Un point de A strictement dans B
    if (point_in_polygon_strict(A[0], B, nB))
        return true;

    // 3. Un point de B strictement dans A
    if (point_in_polygon_strict(B[0], A, nA))
        return true;

    return false;
}
