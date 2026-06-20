#pragma once
#include "config.h"
#include <cmath>

// ─── Flipper collision avoidance (dynamic joint limits) ──────────────────────
// The front and rear flipper on the SAME side share a side-view plane and can
// collide when they lean toward each other. This header is pure geometry: it
// clamps a flipper's integrated TARGET so its length-y segment never closes to
// within FLIPPER_COLLISION_MARGIN_M of the paired flipper's segment.
//
// Side-view frame (per side, looking at the left/right face of the robot):
//   • rear pivot at the origin, front pivot at (+x, 0)  — x = pivot spacing;
//   • each flipper is a segment of length y from its pivot to the tip;
//   • angle 0° points forward (+x); angle increases toward "up" (CCW).
// A collision zone exists only when 2y > x > y (long enough to reach, pivots
// far enough not to always overlap) — enforced below.
//
// Flipper index order matches the rest of the firmware: 0=FL, 1=FR, 2=RL, 3=RR.
// Pairs (front ↔ rear) are FL↔RL (left) and FR↔RR (right).

#if FLIPPER_COLLISION_AVOID_ENABLE
static_assert(2.0f * FLIPPER_LENGTH_M > FLIPPER_PIVOT_SPACING_M,
              "Flipper collision: need 2*FLIPPER_LENGTH_M > FLIPPER_PIVOT_SPACING_M");
static_assert(FLIPPER_PIVOT_SPACING_M > FLIPPER_LENGTH_M,
              "Flipper collision: need FLIPPER_PIVOT_SPACING_M > FLIPPER_LENGTH_M");
#endif

namespace FlipperCollision {

// Front/rear role and same-side partner for each flipper index.
inline int  partner(int idx) { static const int P[4] = { 2, 3, 0, 1 }; return P[idx]; }
inline bool isRear(int idx)  { static const bool R[4] = { false, false, true, true }; return R[idx]; }

namespace detail {

// Reported-angle → shared geometric frame:  geo = sign * (reported − offset).
inline float toGeoDeg(int idx, float reported_deg) {
    static const float kOff[4]  = { FLIPPER_GEO_OFFSET_FL, FLIPPER_GEO_OFFSET_FR,
                                    FLIPPER_GEO_OFFSET_RL, FLIPPER_GEO_OFFSET_RR };
    static const float kSign[4] = { FLIPPER_GEO_SIGN_FL, FLIPPER_GEO_SIGN_FR,
                                    FLIPPER_GEO_SIGN_RL, FLIPPER_GEO_SIGN_RR };
    return kSign[idx] * (reported_deg - kOff[idx]);
}

inline float clamp01(float t) { return t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t); }

// Squared distance from point p to segment a–b (2D).
inline float pointSegDist2(float px, float py,
                           float ax, float ay, float bx, float by) {
    const float vx = bx - ax, vy = by - ay;
    const float wx = px - ax, wy = py - ay;
    const float vv = vx * vx + vy * vy;
    const float t  = clamp01(vv > 1e-12f ? (wx * vx + wy * vy) / vv : 0.0f);
    const float dx = px - (ax + t * vx), dy = py - (ay + t * vy);
    return dx * dx + dy * dy;
}

inline float cross(float ox, float oy, float ax, float ay, float bx, float by) {
    return (ax - ox) * (by - oy) - (ay - oy) * (bx - ox);
}

// Do segments a–b and c–d properly straddle each other?
inline bool segsIntersect(float ax, float ay, float bx, float by,
                          float cx, float cy, float dx, float dy) {
    const float d1 = cross(cx, cy, dx, dy, ax, ay);
    const float d2 = cross(cx, cy, dx, dy, bx, by);
    const float d3 = cross(ax, ay, bx, by, cx, cy);
    const float d4 = cross(ax, ay, bx, by, dx, dy);
    return (((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)));
}

// Minimum distance between segments a–b and c–d (2D). 0 if they cross; the
// near-touch / collinear cases fall out of the endpoint-to-segment minima.
inline float segSegDist(float ax, float ay, float bx, float by,
                        float cx, float cy, float dx, float dy) {
    if (segsIntersect(ax, ay, bx, by, cx, cy, dx, dy)) return 0.0f;
    float m = pointSegDist2(ax, ay, cx, cy, dx, dy);
    m = fminf(m, pointSegDist2(bx, by, cx, cy, dx, dy));
    m = fminf(m, pointSegDist2(cx, cy, ax, ay, bx, by));
    m = fminf(m, pointSegDist2(dx, dy, ax, ay, bx, by));
    return sqrtf(m);
}

// Clearance between the two flipper segments given their geometric-frame angles.
inline float pairClearance(float rear_geo_deg, float front_geo_deg) {
    constexpr float DEG2RAD = 3.14159265359f / 180.0f;
    const float X = FLIPPER_PIVOT_SPACING_M;
    const float Y = FLIPPER_LENGTH_M;
    const float ar = rear_geo_deg  * DEG2RAD;
    const float af = front_geo_deg * DEG2RAD;
    // rear:  (0,0) → (Y·cos ar, Y·sin ar);  front: (X,0) → (X+Y·cos af, Y·sin af)
    return segSegDist(0.0f, 0.0f, Y * cosf(ar), Y * sinf(ar),
                      X, 0.0f, X + Y * cosf(af), Y * sinf(af));
}

}  // namespace detail

// Clamp `candidate` (the freshly integrated target for flipper `idx`) against
// the same-side partner's MEASURED angle. Returns `candidate` when it keeps the
// pair at least FLIPPER_COLLISION_MARGIN_M apart, or when it does not reduce the
// clearance (so an already-too-close pose can always open up again). Otherwise
// returns `current` — i.e. the flipper holds at the boundary. `measured` is the
// 4-element FL,FR,RL,RR reported-angle array.
inline float clampTarget(int idx, float candidate_deg, float current_deg,
                         const float measured[4]) {
    const float other = detail::toGeoDeg(partner(idx), measured[partner(idx)]);
    const float cand  = detail::toGeoDeg(idx, candidate_deg);
    const float cur   = detail::toGeoDeg(idx, current_deg);

    float d_cand, d_cur;
    if (isRear(idx)) {
        d_cand = detail::pairClearance(cand, other);
        d_cur  = detail::pairClearance(cur,  other);
    } else {
        d_cand = detail::pairClearance(other, cand);
        d_cur  = detail::pairClearance(other, cur);
    }

    if (d_cand >= FLIPPER_COLLISION_MARGIN_M) return candidate_deg;  // safely clear
    if (d_cand >= d_cur)                      return candidate_deg;  // not closing → allow
    return current_deg;                                              // would close → hold
}

}  // namespace FlipperCollision
