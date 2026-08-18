# VERDICT: WP6.2 R3.1A BCCM RESIDENCY LIFECYCLE CORRECTION

**Status:** PASSED WITH EVIDENCE  
**Timestamp:** 2026-08-18  
**Component:** BCCM Curved Frustum Visibility Residency / Table Lifecycle (`multinet_ext`)  
**DLL SHA-256:** `BD42FFC6176E612509F233E35C433B968DA3FB22999B59B7D010CC50A0A6D5B9`  

---

## Executive Summary

WP6.2 R3.1A repairs the production residency lifecycle defect where historical visibility leases accumulated indefinitely in the fixed 256-slot table upon camera translation.

Under the authoritative Candidate-Cut Lifetime Law:
1. **Immediate Stale Purge**: Visibility residency may outlive frustum membership, but it **never outlives membership in the current BCCM candidate cut**. Any lease belonging to a block that leaves the candidate cut is purged immediately prior to candidate matching.
2. **Deterministic Capacity Guarantee**: Active residency leases $\le$ current candidate count on every frame ($\le 64$ for LOD0, $\le 48$ for LOD1..7). Table capacity exhaustion is strictly zero across all travel.
3. **Hard Correctness Invariant Retained**: $\text{ExactVisibleSet} \subseteq \text{ResidentVisibleSet}$ holds across 4,560,000 tested moving-cut block states with 0 false negatives.
4. **Honest MultiMesh Churn Reduction**: MultiMesh dirty rewrites reduced by 13.2% under pure rotation (Case A) and by 63.4% under high-speed translation + rotation (Case B).
5. **Clean Telemetry & Inspector Interface**: Populated streaming diagnostics with actual measured frame uploads, added stale purge / capacity exhaustion counters, and exposed `r3_debug_curved_frustum_culling` in the Godot Inspector.

---

## 1. Stale-Lease Defect Reproduction (`BCCM-R3-RESIDENCY-STALE-LEASE-REPRO-01`)

Under exact LOD0 production snap law ($32\text{ m}$ block width, $64\text{ m}$ snap, $8\times8 = 64$ candidate grid, $10\text{ km/s}$ @ $90\text{ FPS}$), the uncorrected R3.1 logic only aged/removed leases matching current candidates. Historical leases accumulated until the 256-slot array saturated at Frame 7, after which new guard-visible candidates were denied leases:

```
## Step 1: Reproducing Stale-Lease Defect under Uncorrected Logic...
  frame 0: lease_count=64
  frame 1: lease_count=80
  frame 2: lease_count=112
  frame 3: lease_count=144
  frame 4: lease_count=160
  frame 5: lease_count=192
  frame 6: lease_count=224
  frame 7: lease_count=256 (Table capacity saturated)
  frame 8: lease_count=256 (16 current blocks unable to acquire a lease)
  frame 9: lease_count=256 (48 current blocks unable to acquire a lease)
  frame 10: lease_count=256 (64 current blocks unable to acquire a lease)
[PASS] BCCM-R3-RESIDENCY-STALE-LEASE-REPRO-01: Defect reproduced matching production snap law.
```

---

## 2. Corrected Table-Lifetime Architecture

On every LOD update, before processing current candidate leases:
1. **Enumerate Current Candidates**: Construct current candidate identities ($bx = \text{center\_bx} + du, bv = \text{center\_bv} + dv$).
2. **Step 0 Stale Purge**: Check all active leases in `level.residency_leases`. If any lease's physical block identity is absent from the current candidate set, set `lease.has_lease = false` and increment `residency_stale_entries_purged`. This purge is immediate and exempt from the 2-per-LOD temporal eviction budget because the candidate cut ownership transfer has already completed.
3. **Step 0 Compaction**: In-place compaction ensures no gaps/holes exist before new lease admission.
4. **Current-Cut Matching**: Match current candidates against remaining active leases.
5. **Guard Lease Grant / Refresh**: Guard-visible candidates refresh or grant $0.20\text{ s}$ leases.
6. **Current-Cut Staggered Eviction**: Candidates in the current cut that lose guard visibility age by $\Delta t$. Expired candidates enter the $\le 2$ per LOD eviction budget.
7. **Hard Invariant Enforcement**: `exact_visible => is_visible = true`.
8. **Final Compaction**: Compact active leases.

---

## 3. Verification Evidence & Test Gates

### 3.1 Native Test Suite Results

| Test Fixture | Gate Name | Verdict | Metrics / Coverage |
| :--- | :--- | :--- | :--- |
| `bccm_r3_temporal_coherence_fixture.exe` | `BCCM-R3-RESIDENCY-STALE-LEASE-REPRO-01` | **PASSED** | Exact 11-frame reproduction matching production snap law |
| `bccm_r3_temporal_coherence_fixture.exe` | `BCCM-R3-RESIDENT-SUPERSET-01` | **PASSED** | 4,560,000 moving-cut block states tested, **0 false negatives** |
| `bccm_r3_temporal_coherence_fixture.exe` | `BCCM-R3-RESIDENCY-CUT-LIFETIME-01` | **PASSED** | $\text{leases} \le \text{candidates}$ on 100% of updates across 5 speeds & 6 FPS |
| `bccm_r3_temporal_coherence_fixture.exe` | `BCCM-R3-RESIDENCY-CAPACITY-01` | **PASSED** | 100 km straight + 100 km diagonal travel, **0 capacity exhaustions** |
| `bccm_r3_temporal_coherence_fixture.exe` | `BCCM-R3-FAST-TURN-CORRECTNESS-01` | **PASSED** | 100% of 85 newly exact-visible blocks admitted immediately |
| `bccm_r3_culling_fixture.exe` | `BCCM-R3-CULLING-SUITE` | **PASSED** | 4 gates passed (flat identity, frame transition, finite bounds, freeze) |
| `bccm_high_speed_snap_fixture.exe` | High-Speed Cut Suite | **PASSED** | 9 gates passed (snap aliasing, hole movement, rebase invariance) |
| `bccm_freeze_presentation_fixture.exe` | Freeze Update Suite | **PASSED** | 5 gates passed (navigation matrix, vertical regression, multi-rebase) |
| `bccm_morph_fixture.exe` | Morph & Topology Proof Suite | **PASSED** | 17 gates passed (recursive live-parent equivalence, GPU shader parity) |

### 3.2 Long-Travel Capacity & Maxima (100 km Travel @ 10 km/s, 90 FPS)

```
100 km Straight Travel Max Lease Counts:
  LOD 0: max_leases=40 (<= 64), stale_purged=15394, capacity_exhaustions=0
  LOD 1: max_leases=24 (<= 48), stale_purged=7305, capacity_exhaustions=0
  LOD 2: max_leases=25 (<= 48), stale_purged=4338, capacity_exhaustions=0
  LOD 3: max_leases=25 (<= 48), stale_purged=2389, capacity_exhaustions=0
  LOD 4: max_leases=25 (<= 48), stale_purged=1275, capacity_exhaustions=0
  LOD 5: max_leases=25 (<= 48), stale_purged=627, capacity_exhaustions=0
  LOD 6: max_leases=24 (<= 48), stale_purged=264, capacity_exhaustions=0
  LOD 7: max_leases=23 (<= 48), stale_purged=144, capacity_exhaustions=0

100 km Diagonal Travel Max Lease Counts:
  LOD 0: max_leases=53 (<= 64), stale_purged=23244, capacity_exhaustions=0
  LOD 1: max_leases=37 (<= 48), stale_purged=13138, capacity_exhaustions=0
  LOD 2: max_leases=37 (<= 48), stale_purged=6852, capacity_exhaustions=0
  LOD 3: max_leases=37 (<= 48), stale_purged=3542, capacity_exhaustions=0
  LOD 4: max_leases=37 (<= 48), stale_purged=1854, capacity_exhaustions=0
  LOD 5: max_leases=37 (<= 48), stale_purged=952, capacity_exhaustions=0
  LOD 6: max_leases=37 (<= 48), stale_purged=476, capacity_exhaustions=0
  LOD 7: max_leases=22 (<= 48), stale_purged=221, capacity_exhaustions=0
```

### 3.3 Honest Temporal Coherence Metrics

#### Case A: Pure Rotation, Fixed Cut (160 frames, $\pm 8^\circ$ yaw @ 60 FPS)
- **Exact R3**: Additions = 568, Removals = 415, MultiMesh Dirty Rewrites = 295
- **Corrected R3.1**: Additions = 316, Removals = 120, MultiMesh Dirty Rewrites = 256
- **Turnover Reduction**: **55.6%**
- **Dirty Buffer Rewrite Reduction**: **13.2%**

#### Case B: Translation (100 m/s) + Rotation with Moving/Snapping Cut (160 frames @ 60 FPS)
- **Exact R3**: Additions = 584, Removals = 419, MultiMesh Dirty Rewrites = 525
- **Corrected R3.1**: Additions = 383, Removals = 177, MultiMesh Dirty Rewrites = 192
- **Turnover Reduction**: **44.2%**
- **Dirty Buffer Rewrite Reduction**: **63.4%**

---

## 4. Rule 3 Static Analysis & Build Receipt

- **Files Scanned:** 46 C++ files in `166.96 ms`
- **Heap Violations Found:** 0 (`[GATE PASSED]`)
- **Build Toolchain:** MSVC `14.44.35207` + Ninja via `build_multinet.bat`
- **GDExtension Deployed:**
  - `multinet_ext/build/windows-editor/multinet.dll`
  - `Project-Nations/addons/multinet/bin/multinet.windows.debug.x86_64.dll`
  - `%TEMP%/multinet_hotreload/multinet.windows.debug.x86_64.dll`
- **Tri-Copy SHA-256:** `BD42FFC6176E612509F233E35C433B968DA3FB22999B59B7D010CC50A0A6D5B9`

---

## 5. Clean Diagnostics & Inspector Surface

- `StreamingDiagnosticsSnapshot.multimesh_buffers_rewritten` and `total_instance_bytes_uploaded` populated with live measured values.
- New counters `residency_stale_entries_purged` and `residency_capacity_exhaustions` exposed via telemetry and Godot debug dictionary.
- Inspector property surfaced as `r3_debug_curved_frustum_culling` (label "R3 Debug / Curved Frustum Culling A-B", default `true`).
