#!/usr/bin/env python3
"""
FCL-based self-collision distance checker for the actuated arm chain.

Loads one **low-poly** collision mesh per link (see
``scripts/make_collision_meshes.py`` — the full-res CAD STLs are ~20x too heavy
for a 100 Hz query), builds an FCL BVH per link, and reports the signed
clearance between every *checkable* link pair for a given joint configuration.

"Checkable" = not in the **allowed-collision matrix (ACM)**. The ACM is built
once at startup by sampling random valid configurations and dropping pairs that
are effectively always in contact (adjacent links sharing a joint) or can never
touch (saves per-cycle work). This mirrors what MoveIt's SRDF does, but computed
on the fly so there is no SRDF to maintain.

This module is pure geometry + ROS-agnostic. The *policy* (how to damp or veto
motion when a pair gets close) lives in ``sdls_servo``; this just answers
"how far apart is everything right now".
"""
from __future__ import annotations

import os

import numpy as np

try:
    import hppfcl as fcl
    import trimesh
    HAVE_DEPS = True
    _IMPORT_ERROR = ''
except Exception as exc:  # pragma: no cover - depends on host install
    HAVE_DEPS = False
    _IMPORT_ERROR = str(exc)


class SelfCollision:
    """Per-pair link clearance via FCL BVH distance queries.

    Parameters
    ----------
    kin : ArmKinematics
        Provides ``link_poses(q)`` (world transforms keyed by link name) and the
        joint limits used to sample the ACM.
    mesh_dir : str
        Directory holding ``<LinkName>.stl`` low-poly collision proxies.
    link_names : list[str]
        Candidate links to check (those without a mesh file are skipped, e.g.
        ``base_link``).
    acm_samples : int
        Random configs sampled to build the allowed-collision matrix.
    acm_always : float
        Pairs in contact in >= this fraction of samples are treated as
        permanently-allowed (adjacent links) and dropped.
    logger : callable | None
        Optional ``str -> None`` for status logging.
    """

    def __init__(self, kin, mesh_dir, link_names, acm_samples=2000,
                 acm_always=0.90, near_band=0.08, logger=None):
        if not HAVE_DEPS:
            raise ImportError(
                f'python-fcl + trimesh required for collision checking ({_IMPORT_ERROR})')
        if not mesh_dir or not os.path.isdir(mesh_dir):
            raise FileNotFoundError(f'collision mesh dir not found: {mesh_dir!r}')

        self.kin = kin
        self._log = logger or (lambda _m: None)
        self.near_band = float(near_band)
        self.links = [ln for ln in link_names
                      if os.path.exists(os.path.join(mesh_dir, ln + '.stl'))]
        if len(self.links) < 2:
            raise FileNotFoundError(
                f'need >=2 link meshes in {mesh_dir}; found {self.links}')

        self._objs = {}
        self._corners = {}      # 8 local AABB corners per link, for broadphase
        for ln in self.links:
            self._objs[ln], self._corners[ln] = self._load(
                os.path.join(mesh_dir, ln + '.stl'))
        self._dreq = fcl.DistanceRequest()
        self._creq = fcl.CollisionRequest()

        # Links directly connected by a joint always meet at that joint, so they
        # are permanently "in collision" — never check them (SRDF "Adjacent").
        chain = [kin.base] + list(kin.child_links)
        self._adjacent = {frozenset((chain[i], chain[i + 1]))
                          for i in range(len(chain) - 1)}

        all_pairs = [(a, b) for i, a in enumerate(self.links)
                     for b in self.links[i + 1:]]
        self.pairs = self._build_acm(all_pairs, acm_samples, acm_always)

    # ── geometry ──────────────────────────────────────────────────────
    @staticmethod
    def _load(path):
        m = trimesh.load(path, force='mesh')
        m.merge_vertices()
        verts = np.asarray(m.vertices, dtype=np.float64)
        faces = np.asarray(m.faces, dtype=np.int64)
        model = fcl.BVHModel()
        model.beginModel(len(verts), len(faces))
        model.addSubModel(verts, faces)
        model.endModel()
        lo, hi = verts.min(axis=0), verts.max(axis=0)
        corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                            for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        return fcl.CollisionObject(model, fcl.Transform()), corners

    def _place_one(self, ln, T):
        self._objs[ln].setTransform(fcl.Transform(
            np.ascontiguousarray(T[:3, :3]), np.ascontiguousarray(T[:3, 3])))

    def _place_all(self, q):
        poses = self.kin.link_poses(q)
        for ln in self.links:
            self._place_one(ln, poses[ln])

    @staticmethod
    def _aabb_gap(a, b):
        """Lower-bound distance between two world AABBs (min,max tuples)."""
        amin, amax = a
        bmin, bmax = b
        gap = np.maximum(np.maximum(amin - bmax, bmin - amax), 0.0)
        return float(np.linalg.norm(gap))

    def distances(self, q):
        """Signed clearance (m) for checkable pairs within the near band.

        An AABB broadphase skips any pair whose bounding boxes are farther apart
        than ``near_band`` (their true clearance can only be larger), so the
        expensive FCL query runs only for pairs that could actually constrain
        motion. Pairs beyond the band are omitted (treated as "far"). ``0.0`` =
        touching/penetrating (FCL returns a -1 sentinel on overlap).
        """
        poses = self.kin.link_poses(q)
        aabb = {}
        for ln in self.links:
            T = poses[ln]
            c = self._corners[ln] @ T[:3, :3].T + T[:3, 3]
            aabb[ln] = (c.min(axis=0), c.max(axis=0))

        out = {}
        placed = set()
        for a, b in self.pairs:
            if self._aabb_gap(aabb[a], aabb[b]) > self.near_band:
                continue
            for ln in (a, b):
                if ln not in placed:
                    self._place_one(ln, poses[ln])
                    placed.add(ln)
            res = fcl.DistanceResult()
            fcl.distance(self._objs[a], self._objs[b], self._dreq, res)
            d = res.min_distance
            out[(a, b)] = 0.0 if d < 0.0 else d
        return out

    def min_clearance(self, q):
        """Smallest clearance over near pairs (m); inf if all beyond near_band."""
        d = self.distances(q)
        return min(d.values()) if d else float('inf')

    def in_collision(self, q, margin=0.0):
        """Boolean self-collision predicate for the path planner.

        ``margin == 0``: True only if some checkable pair is actually touching
        (a fast collide query over every pair). ``margin > 0``: True if any pair's
        clearance is below ``margin`` (a safety buffer, via the broadphase-gated
        distance query). Adjacent / always-touching / unreachable pairs are
        already excluded from ``self.pairs`` by the ACM, so they never trip this.
        """
        if margin > 0.0:
            return self.min_clearance(q) < margin
        self._place_all(q)
        return any(self._in_contact(a, b) for a, b in self.pairs)

    def _in_contact(self, a, b):
        res = fcl.CollisionResult()
        return fcl.collide(self._objs[a], self._objs[b], self._creq, res) > 0

    # ── allowed-collision matrix ──────────────────────────────────────
    def _build_acm(self, all_pairs, n_samples, always):
        lo, up = self.kin.lower, self.kin.upper
        rng = np.random.default_rng(0)

        # adjacency is dropped unconditionally (no sampling needed)
        sampled = [p for p in all_pairs if frozenset(p) not in self._adjacent]
        n_adj = len(all_pairs) - len(sampled)

        hits = {p: 0 for p in sampled}
        for _ in range(max(n_samples, 1)):
            q = lo + (up - lo) * rng.random(len(lo))
            self._place_all(q)
            for p in sampled:
                if self._in_contact(*p):
                    hits[p] += 1

        kept, always_pairs, never_pairs = [], [], []
        for p in sampled:
            frac = hits[p] / max(n_samples, 1)
            if frac >= always:
                always_pairs.append(p)      # effectively always touching
            elif frac == 0.0:
                never_pairs.append(p)        # geometrically unreachable
            else:
                kept.append(p)
        self._log(
            f'self-collision ACM: {len(kept)} active pairs (dropped {n_adj} '
            f'adjacent, {len(always_pairs)} always-touching, {len(never_pairs)} '
            f'unreachable) of {len(all_pairs)} total')
        return kept
