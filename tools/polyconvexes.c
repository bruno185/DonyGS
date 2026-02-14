#include <stdbool.h>
#include <float.h>

typedef struct { double x, y; } Vec;

static double cross(Vec a, Vec b) {
    return a.x * b.y - a.y * b.x;
}

static bool point_on_segment(Vec p, Vec a, Vec b) {
    // Test colinéarité
    double c = cross((Vec){b.x - a.x, b.y - a.y},
                     (Vec){p.x - a.x, p.y - a.y});
    if (c != 0.0) return false;

    // Test projection dans le segment
    double dot = (p.x - a.x)*(b.x - a.x) + (p.y - a.y)*(b.y - a.y);
    if (dot < 0.0) return false;

    double len2 = (b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y);
    if (dot > len2) return false;

    return true;
}

// Ray casting robuste, inside strict (exclut le bord)
bool point_in_polygon_strict(Vec p, Vec *poly, int n) {
    bool inside = false;

    for (int i = 0, j = n - 1; i < n; j = i++) {
        Vec a = poly[j];
        Vec b = poly[i];

        // 1) Si le point est sur une arête → touching → pas inside strict
        if (point_on_segment(p, a, b)) {
            return false;
        }

        // 2) Ray casting classique, en évitant les cas dégénérés
        bool cond1 = (a.y > p.y) != (b.y > p.y);
        if (cond1) {
            double x_intersect =
                a.x + (b.x - a.x) * (p.y - a.y) / (b.y - a.y);

            if (x_intersect > p.x) {
                inside = !inside;
            }
        }
    }

    return inside;
}
