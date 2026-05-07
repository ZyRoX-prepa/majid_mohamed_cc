#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <time.h>

#define MAX_NEIGHBORS_3D 15

struct Drone {
    int id;
    float x;
    float y;
    float z;
};

struct PairResult {
    float dist2;
    struct Drone *a;
    struct Drone *b;
};

static float distance_squared(struct Drone *d1, struct Drone *d2) {
    float dx;
    float dy;
    float dz;

    dx = d1->x - d2->x;
    dy = d1->y - d2->y;
    dz = d1->z - d2->z;

    return dx * dx + dy * dy + dz * dz;
}

static int compare_drone_ptr(struct Drone *a, struct Drone *b, int key) {
    if (key == 0) {
        if (a->x < b->x) {
            return -1;
        }
        if (a->x > b->x) {
            return 1;
        }
        if (a->id < b->id) {
            return -1;
        }
        if (a->id > b->id) {
            return 1;
        }
        return 0;
    }

    if (a->y < b->y) {
        return -1;
    }
    if (a->y > b->y) {
        return 1;
    }
    if (a->id < b->id) {
        return -1;
    }
    if (a->id > b->id) {
        return 1;
    }
    return 0;
}

static void merge_ptr(struct Drone **arr, struct Drone **temp, int left, int mid, int right, int key) {
    int i;
    int j;
    int k;

    i = left;
    j = mid + 1;
    k = left;

    while (i <= mid && j <= right) {
        if (compare_drone_ptr(*(arr + i), *(arr + j), key) <= 0) {
            *(temp + k) = *(arr + i);
            i = i + 1;
        } else {
            *(temp + k) = *(arr + j);
            j = j + 1;
        }
        k = k + 1;
    }

    while (i <= mid) {
        *(temp + k) = *(arr + i);
        i = i + 1;
        k = k + 1;
    }

    while (j <= right) {
        *(temp + k) = *(arr + j);
        j = j + 1;
        k = k + 1;
    }

    i = left;
    while (i <= right) {
        *(arr + i) = *(temp + i);
        i = i + 1;
    }
}

static void merge_sort_ptr(struct Drone **arr, struct Drone **temp, int left, int right, int key) {
    int mid;

    if (left >= right) {
        return;
    }

    mid = left + (right - left) / 2;
    merge_sort_ptr(arr, temp, left, mid, key);
    merge_sort_ptr(arr, temp, mid + 1, right, key);
    merge_ptr(arr, temp, left, mid, right, key);
}

static struct PairResult min_pair(struct PairResult p1, struct PairResult p2) {
    if (p1.dist2 <= p2.dist2) {
        return p1;
    }
    return p2;
}

static struct PairResult brute_force(struct Drone **px, int n) {
    struct PairResult best;
    int i;
    int j;

    best.dist2 = FLT_MAX;
    best.a = NULL;
    best.b = NULL;

    i = 0;
    while (i < n) {
        j = i + 1;
        while (j < n) {
            float d2;
            d2 = distance_squared(*(px + i), *(px + j));
            if (d2 < best.dist2) {
                best.dist2 = d2;
                best.a = *(px + i);
                best.b = *(px + j);
            }
            j = j + 1;
        }
        i = i + 1;
    }

    return best;
}

static struct PairResult closest_pair_rec(
    struct Drone **px,
    struct Drone **py,
    int n,
    struct Drone *base,
    int *marks,
    int *stamp_counter
) {
    struct PairResult best;
    int mid;
    float mid_x;
    struct Drone **left_y;
    struct Drone **right_y;
    int left_size;
    int right_size;
    int i;
    struct PairResult left_best;
    struct PairResult right_best;
    struct Drone **strip;
    int strip_size;
    int stamp;

    if (n <= 3) {
        return brute_force(px, n);
    }

    mid = n / 2;
    mid_x = (*(px + mid))->x;
    stamp = *stamp_counter;
    *stamp_counter = *stamp_counter + 1;

    left_y = (struct Drone **)malloc((size_t)mid * sizeof(struct Drone *));
    right_y = (struct Drone **)malloc((size_t)(n - mid) * sizeof(struct Drone *));
    if (left_y == NULL || right_y == NULL) {
        free(left_y);
        free(right_y);
        best.dist2 = FLT_MAX;
        best.a = NULL;
        best.b = NULL;
        return best;
    }

    left_size = 0;
    right_size = 0;
    i = 0;
    while (i < mid) {
        int idx_left;
        idx_left = (int)(*(px + i) - base);
        *(marks + idx_left) = stamp;
        i = i + 1;
    }

    i = 0;
    while (i < n) {
        struct Drone *current;
        int idx_current;
        current = *(py + i);
        idx_current = (int)(current - base);

        if (*(marks + idx_current) == stamp) {
            *(left_y + left_size) = current;
            left_size = left_size + 1;
        } else {
            *(right_y + right_size) = current;
            right_size = right_size + 1;
        }
        i = i + 1;
    }

    left_best = closest_pair_rec(px, left_y, mid, base, marks, stamp_counter);
    right_best = closest_pair_rec(px + mid, right_y, n - mid, base, marks, stamp_counter);
    best = min_pair(left_best, right_best);

    strip = (struct Drone **)malloc((size_t)n * sizeof(struct Drone *));
    if (strip == NULL) {
        free(left_y);
        free(right_y);
        return best;
    }

    strip_size = 0;
    i = 0;
    while (i < n) {
        float dx;
        dx = (*(py + i))->x - mid_x;
        if (dx * dx < best.dist2) {
            *(strip + strip_size) = *(py + i);
            strip_size = strip_size + 1;
        }
        i = i + 1;
    }

    i = 0;
    while (i < strip_size) {
        int j;
        int compared;

        j = i + 1;
        compared = 0;
        /*
         * Dans la bande centrale 3D, un nombre borné de voisins suffit
         * pour conserver la complexité linéaire de l'étape de fusion.
         */
        while (j < strip_size && compared < MAX_NEIGHBORS_3D) {
            float dy;
            float dy2;
            float dz;
            float dz2;

            dy = (*(strip + j))->y - (*(strip + i))->y;
            dy2 = dy * dy;
            dz = (*(strip + j))->z - (*(strip + i))->z;
            dz2 = dz * dz;

            if (dy2 >= best.dist2) {
                break;
            }

            if (dz2 < best.dist2) {
                float d2;
                d2 = distance_squared(*(strip + i), *(strip + j));
                if (d2 < best.dist2) {
                    best.dist2 = d2;
                    best.a = *(strip + i);
                    best.b = *(strip + j);
                }
            }

            j = j + 1;
            compared = compared + 1;
        }
        i = i + 1;
    }

    free(strip);
    free(left_y);
    free(right_y);

    return best;
}

static struct PairResult find_closest_pair(struct Drone *swarm, int n) {
    struct Drone **px;
    struct Drone **py;
    struct Drone **temp;
    struct PairResult result;
    int i;
    int *marks;
    int stamp_counter;

    result.dist2 = FLT_MAX;
    result.a = NULL;
    result.b = NULL;

    px = (struct Drone **)malloc((size_t)n * sizeof(struct Drone *));
    py = (struct Drone **)malloc((size_t)n * sizeof(struct Drone *));
    temp = (struct Drone **)malloc((size_t)n * sizeof(struct Drone *));
    marks = (int *)malloc((size_t)n * sizeof(int));

    if (px == NULL || py == NULL || temp == NULL || marks == NULL) {
        free(px);
        free(py);
        free(temp);
        free(marks);
        return result;
    }

    i = 0;
    while (i < n) {
        *(px + i) = swarm + i;
        *(py + i) = swarm + i;
        *(marks + i) = 0;
        i = i + 1;
    }

    merge_sort_ptr(px, temp, 0, n - 1, 0);
    merge_sort_ptr(py, temp, 0, n - 1, 1);
    stamp_counter = 1;
    result = closest_pair_rec(px, py, n, swarm, marks, &stamp_counter);

    free(px);
    free(py);
    free(temp);
    free(marks);

    return result;
}

int main(void) {
    int n;
    struct Drone *swarm;
    int i;
    struct PairResult closest;

    n = 10000;

    swarm = (struct Drone *)malloc((size_t)n * sizeof(struct Drone));
    if (swarm == NULL) {
        printf("Erreur: allocation dynamique du bloc continu impossible.\n");
        return 1;
    }

    srand((unsigned int)time(NULL));

    i = 0;
    while (i < n) {
        /*
         * On se déplace dans le bloc mémoire avec (swarm + i).
         * Puis on déréférence explicitement l'adresse pour écrire chaque champ.
         * Aucune indexation par crochets n'est utilisée.
         */
        (*(swarm + i)).id = i;
        (*(swarm + i)).x = (float)(rand() % 10000) / 10.0f;
        (*(swarm + i)).y = (float)(rand() % 10000) / 10.0f;
        (*(swarm + i)).z = (float)(rand() % 10000) / 10.0f;
        i = i + 1;
    }

    closest = find_closest_pair(swarm, n);

    if (closest.a == NULL || closest.b == NULL) {
        printf("Erreur: le calcul de la paire la plus proche a échoué.\n");
        free(swarm);
        return 1;
    }

    printf("Drone A: id=%d (x=%.2f, y=%.2f, z=%.2f)\n",
           closest.a->id,
           closest.a->x,
           closest.a->y,
           closest.a->z);
    printf("Drone B: id=%d (x=%.2f, y=%.2f, z=%.2f)\n",
           closest.b->id,
           closest.b->x,
           closest.b->y,
           closest.b->z);
    printf("Distance^2 minimale = %.6f\n", closest.dist2);

    free(swarm);
    return 0;
}
