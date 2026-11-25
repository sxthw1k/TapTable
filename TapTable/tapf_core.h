/*
 * tapf_core.h - TapTable final version: dataset and kNN logic (no hardware code)
 *
 * Included once by main.c. The same file is compiled on the laptop by the
 * test harness, so the board and the laptop run exactly the same logic.
 *
 * Changes from Stage 4 (tap4_core.h):
 *   - 9 features instead of 12: the three peak-time features are gone
 *     (with them, new users got 3.5% wrong answers; without, 1.7%)
 *   - the factory dataset (S K T R L, 600 taps) is built into the program
 *     (sktrl_data.h, 10 bytes per tap), so no loader program is needed
 *   - votes needed are fixed: 6 of 9 (3 of 5 while a zone has < 30 taps)
 *
 * WHAT IS IN HERE
 *   make_features()  raw tap numbers -> the 9 features
 *   stats_compute()  mean and 1/spread of each feature over a set of taps
 *   knn_vote()       k nearest stored taps -> zone with most votes
 *   choose_k()       k = 5 (needs 3) while any zone has < 30 taps,
 *                    otherwise k = 9 (needs 6)
 *   cal_apply()      what happens after a calibration:
 *                      1. drift check (merge calibrations only)
 *                      2. merge, or replace everything (full recalibration)
 *                      3. zero-vote check: delete taps none of whose 9
 *                         nearest neighbours share their zone (never takes
 *                         a zone below 20 taps)
 *                      4. trim each zone to 120, oldest session first,
 *                         never the new taps
 *                      5. new means/spreads and new k
 *   load_builtin()   fill the dataset from the built-in S K T R L taps
 *
 * WHERE EACH PART IS USED (sections of main.c)
 *   Section 11  load_builtin, dataset_refresh, stats_compute, choose_k
 *   Section 14  make_features
 *   Section 15  classify, knn_nearest
 *   Section 21  cal_apply (uses knn_vote, knn_nearest, dataset_compact)
 *
 * WHY kNN
 *   Three sensors give each spot on the pad its own pattern (how the
 *   vibration is shared between them and the order it arrives in). The
 *   patterns of neighbouring zones overlap and differ from person to
 *   person, so no fixed rule separates all six zones well (a hand-made
 *   rule got 5.6% wrong; kNN 1.9%). kNN compares a tap with stored taps:
 *   it needs no training step, so a calibration is just adding taps, and
 *   every answer can be explained ("6 of the 9 nearest taps were Z3").
 */
#ifndef TAPF_CORE_H
#define TAPF_CORE_H

#include <stdint.h>

/* ------------------------------ settings ---------------------------- */
#define NF                  9       /* features per tap                       */
#define NZ                  6       /* zones                                  */
#define CAP_PER_ZONE        120     /* most taps kept per zone                */
#define MAXN                (NZ * CAP_PER_ZONE)        /* 720 stored taps     */
#define CAL_MERGE_PER_ZONE  20      /* taps per zone, merge calibration       */
#define CAL_FULL_PER_ZONE   20      /* taps per zone, full recalibration (same as merge) */
#define MAXNEW              (NZ * CAL_FULL_PER_ZONE)   /* 120 new taps max    */
#define WORKN               (MAXN + MAXNEW)            /* 900 while merging   */

#define K_SMALL             5       /* k while any zone has < K_BIG_FROM taps */
#define NEED_SMALL          3
#define K_BIG               9       /* k once every zone has >= K_BIG_FROM    */
#define NEED_BIG            6
#define K_BIG_FROM          30
#define KMAX                9

#define ZV_K                9       /* zero-vote check looks at 9 neighbours  */
#define ZV_MIN_ZONE         20      /* never delete a zone below 20 taps      */

#define DRIFT_GAP_PCT       20      /* new->new minus old->new above this
                                       = setup changed = full recalibration   */
#define DRIFT_K             5       /* new->new uses k = 5, needs 3           */
#define DRIFT_NEED          3

#define TAP_WIN             80      /* capture window (onset 255 -> 80)       */

/* ------------------------------ data -------------------------------- */
typedef struct {
    float    x[WORKN][NF];          /* features                               */
    uint8_t  y[WORKN];              /* zone 0..5                              */
    uint16_t s[WORKN];              /* session number (higher = newer)        */
    uint32_t n;                     /* taps in use                            */
    uint32_t next_session;          /* number the next calibration gets       */
} dataset_t;

typedef struct {
    float mu[NF];                   /* mean of each feature                   */
    float isd[NF];                  /* 1 / standard deviation                 */
} stats_t;

typedef struct {                    /* report of one calibration              */
    int      forced;                /* 1 = forced full (TAMPER held 3 s)      */
    int      full;                  /* 1 = old data was replaced              */
    int      checked;               /* 1 = drift check was run                */
    uint32_t nnew;                  /* new taps                               */
    uint32_t on_ok;                 /* old data -> new taps: correct          */
    uint32_t nn_ok;                 /* new taps -> each other: correct        */
    int32_t  gap10;                 /* gap in tenths of a percent point       */
    uint32_t zv_removed;            /* zero-vote deletions                    */
    uint32_t zv_zone[NZ];
    uint32_t trimmed;               /* removed by the 120 cap                 */
    uint16_t session;               /* session number given to the new taps   */
} calres_t;

static dataset_t D;                 /* the stored dataset (in RAM)            */
static stats_t   ST;                /* its means and spreads                  */
static int       g_k = K_BIG, g_need = NEED_BIG;

static float     NX[MAXNEW][NF];    /* taps of the calibration in progress    */
static uint8_t   NY[MAXNEW];
static uint32_t  NN;

static uint8_t   s_mark[WORKN];     /* scratch: taps to delete                */

/* ---------------------------- features ------------------------------ */
/* p = peaks, e = energies, o = onsets (sample 0..79, 255 = never) */
/* Section 14. Why these features:
 *   - SHARES (peak and energy of one sensor divided by the sum of all
 *     three) describe WHERE the tap was, not HOW HARD: a soft and a hard
 *     tap on the same spot give almost the same shares.
 *   - ONSET DELAYS (ms after the sensor that felt the tap first) show
 *     which sensor is nearest and how far the others are. At 4 kHz one
 *     sample is 0.25 ms, and taps reach the 3 sensors over about 1.75 ms.
 *   - Peak times (used until the interim) were dropped: in about 1 tap in
 *     9 the peak was at the end of the 20 ms window, so it carried no
 *     information. Strangers got 4.6% wrong with them, 1.9% without.
 */
static void make_features(const int32_t *p, const uint32_t *e, const int32_t *o, float *f)
{
    float ps = (float)(p[0] + p[1] + p[2]);
    float es = (float)e[0] + (float)e[1] + (float)e[2];
    float on[3], t0;
    if (ps < 1.0f) ps = 1.0f;                     /* never divide by 0 */
    if (es < 1.0f) es = 1.0f;
    for (int s = 0; s < 3; s++) on[s] = (o[s] >= 255) ? (float)TAP_WIN : (float)o[s];
    t0 = on[0];
    if (on[1] < t0) t0 = on[1];
    if (on[2] < t0) t0 = on[2];
    for (int s = 0; s < 3; s++) {
        f[s]     = (float)p[s] / ps;                   /* peak share   */
        f[3 + s] = (float)e[s] / es;                   /* energy share */
        f[6 + s] = (on[s] - t0) * 0.25f;               /* onset delay after the first sensor, ms */
    }
}

/* --------------------------- statistics ----------------------------- */
static float core_sqrt(float x) { return (x > 0.0f) ? __builtin_sqrtf(x) : 0.0f; }

/* Section 11. Mean and spread of each feature over the stored taps.
 * kNN compares taps feature by feature, so every feature is put on the
 * same scale first: (value - mean) / spread. Without this the onset
 * delays (up to about 6 ms) would count far more than the shares (0..1).
 * isd = 1/spread is stored so the kNN loop multiplies instead of divides.
 */
static void stats_compute(const float (*x)[NF], uint32_t n, stats_t *st)
{
    for (int j = 0; j < NF; j++) {
        float m = 0.0f, v = 0.0f;
        for (uint32_t i = 0; i < n; i++) m += x[i][j];
        m /= (float)(n ? n : 1u);
        for (uint32_t i = 0; i < n; i++) { float d = x[i][j] - m; v += d * d; }
        v /= (float)(n ? n : 1u);
        st->mu[j]  = m;
        st->isd[j] = (v > 1e-12f) ? 1.0f / core_sqrt(v) : 1.0f;
    }
}

static void zone_counts(const uint8_t *y, uint32_t n, uint32_t *cnt)
{
    for (int z = 0; z < NZ; z++) cnt[z] = 0;
    for (uint32_t i = 0; i < n; i++) cnt[y[i]]++;
}

/* Section 11. How many neighbours vote, and how many must agree.
 *   Normal:          k = 9, 6 must agree. With 9 votes there is room
 *                    for an "unsure" answer: 6 of 9 removes about 40% of
 *                    the wrong answers of a simple majority (5 of 9) and
 *                    costs under 1 point of correct answers.
 *   Small dataset:   k = 5, 3 must agree, while any zone has < 30 taps
 *                    (only after a full recalibration, 20 per zone): the
 *                    9 nearest would then reach into other zones.
 */
static void choose_k(void)
{
    uint32_t cnt[NZ], mn;
    zone_counts(D.y, D.n, cnt);
    mn = cnt[0];
    for (int z = 1; z < NZ; z++) if (cnt[z] < mn) mn = cnt[z];
    if (mn >= K_BIG_FROM) { g_k = K_BIG;   g_need = NEED_BIG;   }
    else                  { g_k = K_SMALL; g_need = NEED_SMALL; }
}

/* ------------------------------ kNN --------------------------------- */
/* Finds the k taps of (x, y, n) nearest to q (skipping index 'skip', or -1),
   after standardising with st. Writes their zones to lab[0..k-1] (-1 if
   fewer than k taps). Ties keep the earlier tap. */
/* Section 15. The core of kNN.
 *   Distance = sum over the 9 features of (difference after scaling)^2.
 *   The square root is not taken: it does not change which taps are
 *   nearest. The k best are kept in a small sorted list (insertion
 *   sort), so one pass over all stored taps is enough; no big sort.
 */
static void knn_nearest(const float (*x)[NF], const uint8_t *y, uint32_t n,
                        const stats_t *st, const float *q, int32_t skip,
                        int k, int *lab)
{
    float bestd[KMAX], zq[NF];
    for (int j = 0; j < NF; j++) zq[j] = (q[j] - st->mu[j]) * st->isd[j];  /* scale the new tap once */
    for (int i = 0; i < k; i++) { bestd[i] = 3.0e38f; lab[i] = -1; }    /* list starts "infinitely far" */
    for (uint32_t i = 0; i < n; i++) {
        float d = 0.0f;
        if ((int32_t)i == skip) continue;          /* a tap must not vote for itself */
        for (int j = 0; j < NF; j++) {
            float t = (x[i][j] - st->mu[j]) * st->isd[j] - zq[j];
            d += t * t;
        }
        if (d < bestd[k - 1]) {                  /* keep the k nearest, sorted */
            int p = k - 1;
            while (p > 0 && bestd[p - 1] > d) {     /* shift farther ones down one place */
                bestd[p] = bestd[p - 1]; lab[p] = lab[p - 1]; p--;
            }
            bestd[p] = d; lab[p] = y[i];
        }
    }
}

/* Returns the zone (0..5) with the most votes among the k nearest
   (first zone wins a tie) and its vote count in *votes. The caller
   compares *votes with 'need'. */
/* Section 21 (drift check during calibration). Same as classify() below,
 * but works on any set of taps, e.g. the new calibration taps alone. */
static int knn_vote(const float (*x)[NF], const uint8_t *y, uint32_t n,
                    const stats_t *st, const float *q, int32_t skip,
                    int k, int *votes)
{
    int lab[KMAX], v[NZ] = { 0 }, zone = 0;
    knn_nearest(x, y, n, st, q, skip, k, lab);
    for (int i = 0; i < k; i++) if (lab[i] >= 0) v[lab[i]]++;
    for (int z = 1; z < NZ; z++) if (v[z] > v[zone]) zone = z;
    *votes = v[zone];
    return zone;
}

/* For the Keil Watch window: the last tap's neighbours and votes */
volatile int8_t  g_nn_zone[KMAX];   /* zones (1..6) of the k nearest stored taps */
volatile uint8_t g_zone_votes[NZ];  /* votes for Z1..Z6 among them              */

/* Classify one tap against the stored dataset: zone 0..5, or -1 = unsure.
   Same decision as knn_vote(), but also keeps the neighbours for the
   Watch window. */
/* Section 15. Called for every tap in Play and Password mode. */
static int classify(const float *f, int *votes)
{
    int lab[KMAX], v[NZ] = { 0 }, zone = 0;
    knn_nearest((const float (*)[NF])D.x, D.y, D.n, &ST, f, -1, g_k, lab);
    for (int i = 0; i < KMAX; i++) g_nn_zone[i] = (int8_t)((i < g_k && lab[i] >= 0) ? lab[i] + 1 : 0);
    for (int i = 0; i < g_k; i++) if (lab[i] >= 0) v[lab[i]]++;
    for (int z = 1; z < NZ; z++) if (v[z] > v[zone]) zone = z;
    for (int z = 0; z < NZ; z++) g_zone_votes[z] = (uint8_t)v[z];
    *votes = v[zone];
    return (*votes >= g_need) ? zone : -1;   /* too few votes: unsure, the board stays silent */
}

/* ----------------------- dataset maintenance ------------------------ */
/* Section 11 / 21. Must be called whenever the stored taps change. */
static void dataset_refresh(void)            /* after any change */
{
    stats_compute((const float (*)[NF])D.x, D.n, &ST);
    choose_k();
}

/* Section 21. Close the gaps left by deleted taps, keeping their order
 * (oldest first), which the trimming step relies on. */
static void dataset_compact(void)            /* remove taps marked in s_mark */
{
    uint32_t w = 0;
    for (uint32_t i = 0; i < D.n; i++) {
        if (s_mark[i]) continue;
        if (w != i) {
            for (int j = 0; j < NF; j++) D.x[w][j] = D.x[i][j];
            D.y[w] = D.y[i];
            D.s[w] = D.s[i];
        }
        w++;
    }
    D.n = w;
}

/* ------------------------ built-in dataset -------------------------- */
#include "sktrl_data.h"

/* Fill D with the built-in S K T R L taps (the factory dataset) */
/* Section 11. The factory dataset is stored as 10 bytes per tap (shares
 * x 255 and onset samples) instead of 9 floats (36 bytes): 6 KB instead
 * of 24 KB of program flash. It is rebuilt into floats here at start-up
 * and after a factory reset.
 */
static void load_builtin(void)
{
    for (uint32_t i = 0; i < FACT_N; i++) {
        const uint8_t *t = FACT[i];
        float on[3], t0;
        for (int s = 0; s < 3; s++)               /* timing: same rule as make_features */
            on[s] = (t[6 + s] >= 255u) ? (float)TAP_WIN : (float)t[6 + s];
        t0 = on[0];
        if (on[1] < t0) t0 = on[1];
        if (on[2] < t0) t0 = on[2];
        for (int s = 0; s < 3; s++) {
            D.x[i][s]     = (float)t[s]     / 255.0f;     /* peak share   */
            D.x[i][3 + s] = (float)t[3 + s] / 255.0f;     /* energy share */
            D.x[i][6 + s] = (on[s] - t0) * 0.25f;         /* onset delay  */
        }
        D.y[i] = (uint8_t)(t[9] & 0x0Fu);
        D.s[i] = (uint16_t)(t[9] >> 4);
    }
    D.n = FACT_N;
    D.next_session = FACT_NEXT;
    dataset_refresh();
}

/* Everything that happens after the last calibration tap.
   Uses the NN new taps in NX/NY. forced = 1: forced full recalibration. */
/* Section 21. The steps and the reasons:
 *   1. Drift check (merge only). Score the new taps two ways: against the
 *      stored taps, and against each other. If they agree with each other
 *      more than 20 points better, the setup has changed (pad moved,
 *      sensor loosened, another table) and old taps would mislead, so the
 *      board does a full recalibration instead. Our 4 volunteers outside
 *      the dataset scored between -3.3 and +4.1 points, far below 20.
 *   2. Merge (add the new taps) or replace everything (full).
 *   3. Misfit removal: a tap none of whose 9 nearest taps share its zone
 *      was almost certainly tapped on the wrong spot. Stricter rules
 *      (delete if fewer than 2..5 of 9 agree) removed 1.5 to 3 times as
 *      many taps, mostly genuine ones, and wrong answers
 *      stayed at 1.7% either way. A zone never drops below 20 taps.
 *   4. Cap of 120 taps per zone: keeps memory and kNN time bounded.
 *      Oldest sessions go first; the taps just recorded are never trimmed.
 *   5. New means, spreads and k.
 */
static void cal_apply(int forced, calres_t *r)
{
    uint32_t cnt[NZ];
    uint16_t sess = (uint16_t)D.next_session++;

    for (int z = 0; z < NZ; z++) r->zv_zone[z] = 0;
    r->forced = forced; r->full = forced; r->checked = 0;
    r->nnew = NN; r->on_ok = 0; r->nn_ok = 0; r->gap10 = 0;
    r->zv_removed = 0; r->trimmed = 0; r->session = sess;

    /* 1. drift check (merge calibrations only) */
    if (!forced && D.n > 0) {
        stats_t sn;
        int v;
        r->checked = 1;
        for (uint32_t i = 0; i < NN; i++)          /* old data -> new taps */
            if (classify(NX[i], &v) == NY[i]) r->on_ok++;
        stats_compute((const float (*)[NF])NX, NN, &sn);
        for (uint32_t i = 0; i < NN; i++) {        /* new taps -> each other */
            int z = knn_vote((const float (*)[NF])NX, NY, NN, &sn, NX[i], (int32_t)i,
                             DRIFT_K, &v);
            if (v >= DRIFT_NEED && z == NY[i]) r->nn_ok++;
        }
        r->gap10 = (int32_t)(((int32_t)r->nn_ok - (int32_t)r->on_ok) * 1000 / (int32_t)NN);
        if (r->gap10 > DRIFT_GAP_PCT * 10) r->full = 1;
    }

    /* 2. merge, or start again from the new taps only */
    if (r->full) D.n = 0;
    for (uint32_t i = 0; i < NN; i++) {
        for (int j = 0; j < NF; j++) D.x[D.n][j] = NX[i][j];
        D.y[D.n] = NY[i];
        D.s[D.n] = sess;
        D.n++;
    }

    /* 3. zero-vote check: none of the 9 nearest share the zone -> delete */
    {
        stats_t sa;
        int lab[KMAX];
        stats_compute((const float (*)[NF])D.x, D.n, &sa);
        zone_counts(D.y, D.n, cnt);
        for (uint32_t i = 0; i < D.n; i++) {
            int same = 0;
            s_mark[i] = 0;
            knn_nearest((const float (*)[NF])D.x, D.y, D.n, &sa, D.x[i], (int32_t)i,
                        ZV_K, lab);
            for (int q = 0; q < ZV_K; q++) if (lab[q] == D.y[i]) same++;
            if (same == 0 && cnt[D.y[i]] > ZV_MIN_ZONE) {   /* keep >= 20 per zone */
                s_mark[i] = 1;
                cnt[D.y[i]]--;
                r->zv_removed++;
                r->zv_zone[D.y[i]]++;
            }
        }
        dataset_compact();
    }

    /* 4. trim each zone to 120: oldest first (taps are stored oldest
          first), never the taps of this calibration */
    zone_counts(D.y, D.n, cnt);
    for (uint32_t i = 0; i < D.n; i++) {
        s_mark[i] = 0;
        if (cnt[D.y[i]] > CAP_PER_ZONE && D.s[i] != sess) {
            s_mark[i] = 1;
            cnt[D.y[i]]--;
            r->trimmed++;
        }
    }
    dataset_compact();

    /* 5. new means and spreads, new k */
    dataset_refresh();
    NN = 0;
}

#endif
