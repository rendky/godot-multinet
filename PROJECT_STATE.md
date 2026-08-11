# Multinet Project State

## Current verified state

- The Godot startup crash is fixed. The native dump reported `0xC00000FD` in `Multinet::to_surface_position`.
- The cause was mutual recursion: `to_surface_position()` called `remap_address()`, which called `to_surface_position()` again.
- `to_surface_position()` now performs the unit conversion directly and calls `remap_position()`.
- The topology reversibility check now exercises left, right, top, and bottom crossings on all six faces.
- `build_multinet.bat` passes the Rule 3 allocation gate, compiles, links, and deploys the DLL.
- Headless runtime and headless editor smokes exit with code 0 and report all existing startup gates passed.
- The build DLL and deployed DLL had matching SHA-256 hashes after the crash fix.

**P0 Rendering Fixes Completed:**
1. **Negative-axis terrain loss**: `BlockClipmapRenderer` separated canonical identity from placement. Numeric and rendered crossing probes no longer show terrain loss caused by signed chunk placement.
2. **Distant LOD height/normal aliasing**: Nyquist filtering fades high-frequency noise by LOD spacing and octave frequency. The flat-normal fixture passes across LOD0–LOD7; coarse LOD faceting is still visible in rendered evidence.
3. **Macro sphere origin mismatch**: Macro sphere indices flipped to CCW (no longer inside-out). The shader anchors to `active_origin.y`, and FBM mapping is pulled directly from the geometry generator `UV` mapping. Culling disabled to prevent missing patches.
4. **Frustum Culling / Altitude Drops**: Inner clipmap LOD generation radius is forced to >= 8 to prevent concentric square holes. `horizon_distance` calculation augmented by `max_elevation` to prevent low-altitude vanishing. `altitude_lod_bias` logic now calculates height logarithmically relative to the highest mountain peak to ensure accurate detail is preserved.

The worktree contains substantial pre-existing CS-T1 through CS-T5 edits and untracked source. Treat them as user-owned. Do not clean, revert, stage, commit, delete, rename, or move them unless explicitly requested.

## Architecture assessment

The intended split is sound:

- `SurfacePosition64` and `SurfaceAddress` own canonical six-chart identity.
- `SurfaceFrame` owns a local Euclidean unfolding.
- Godot rendering uses camera-relative terrain rings.
- Curved Horizon Presentation bends the near-field tangent plane visually.
- The macro profile supplies a coarse full sphere at flight altitude.

The roadmap is useful, but CS-T4 and CS-T5 are prototype-functional rather than qualified-complete. Current gates do not yet prove every feature named by those phases. In particular, LOD geomorphing is deferred in the shader, HZB culling is absent, and actual GPU readback seam parity is not tested. Closed-surface holonomy remains expected; the runtime now separates true canonical transport from stable flat presentation placement instead of pretending the six charts form one global rigid frame.

Do not begin CS-T6 collision or CS-T7 networking until the render-frame contract below is closed.

## Build and smoke commands

```powershell
cmd.exe /c build_multinet.bat
```

```powershell
cmd.exe /c "D:\Engines\Custom\godot-multinet\bin\godot.windows.editor.dev.x86_64.console.exe --path D:\Projects\Games\Project-Nations --headless --quit-after 2 --disable-crash-handler"
```

```powershell
cmd.exe /c "D:\Engines\Custom\godot-multinet\bin\godot.windows.editor.dev.x86_64.console.exe --path D:\Projects\Games\Project-Nations --editor --headless --quit --disable-crash-handler"
```

The build directory may require one policy-controlled approved command because sandboxed Ninja could not create `.ninja_lock`. Do not interpret that permission failure as a compiler failure.

## Review evidence expected from the implementation pass

- Exact files and functions changed.
- Rule 3/build output.
- Runtime and editor smoke exit codes.
- Candidate and visible counts for negative X/Z crossings.
- Captures or numeric probes below and above the macro threshold.
- Confirmation that no existing dirty work was reverted or overwritten.

## WP6.0 final verified state (2026-08-09)

- WP6.0 world-domain generalization and the closed-domain wrapping corrective pass are implemented in the dirty worktree. Finite rectangles and closed six-face domains own topology, extents, area, region counts, and separate canonical/presentation identities.
- `TerrainPresentationBlockKey` and `TerrainSamplePatchKey` now remain distinct through BCCM candidate placement, hierarchy, page demand, source records, GPU slots, stale selection, retirement, LRU, CPU page sampling, and shader analytic sampling.
- Runtime navigation transports the canonical face/u/v position but renders in a stable flat X/Z frame with real altitude. The debug summary exposes position, speed, transition source/destination/edge, LOD counts, placement failures, duplicate retention, and maximum patch crossings.
- Closed equivalent sides below 0.079 km are rejected because an ordinary 32 m LOD0 block must fit within a face span. The 100 km Project-Nations test setting is supported.
- Final `build_multinet.bat` passed Rule 3 for 46 files and deployed the DLL. The build DLL, preflight copy, and Project-Nations deployed DLL had identical SHA-256 hashes at validation time.
- Fresh native receipts pass: WORLD-DOMAIN-AREA-01; 100 km shared-edge and Hybrid page agreement; presentation-distinct cache identity; 2 km multi-wrap; EDITOR-CANONICAL-NAV-01; FINITE-BOUNDARY-01; canonicalization; BCCM flat-normal LOD; and the full WP5.2 regression.
- The authoritative rendered evidence is `res://WP6_RUNTIME_WRAP_EVIDENCE.tscn`. Its uncut window-only video and console log came from the same Vulkan Godot process and record a real face 0 to face 5 transition at 100 km with process exit 0.
- The earlier crashing editor-viewport probe is superseded and is not evidence. `WP6_RENDERED_WIREFRAME_PREFLIGHT.png` is also explicitly non-evidence because the requested wireframe mode did not activate.
- No new wrap discontinuity is visible in sampled crossing frames, but ordinary coarse BCCM/LOD faceting remains. Do not claim all visual terrain defects are fixed.
- CHP was not started. CPU-versus-FP32-reference parity passed, but actual GPU readback numerical parity remains untested. No automatic `LIVE VISIBLE FLAT TERRAIN: 0` conclusion is made.
- Final receipts, hashes, launch commands, and the source-inclusive archive are under `multinet_ext/src/evidence/WP6/` and `multinet_ext/`.
- Final archive: `multinet_ext/WP6_COORDINATE_WRAP_FINAL_EVIDENCE_20260809.zip`; 18,640,383 bytes; SHA-256 `03D47D436584465340AAE9868BC21A89737889E1930130179812E316128D5B31`. Its archive root is the complete `src/` tree (215 entries), including the rendered video, same-process log, manifest, manifest hash, and source-hash ledger.

## WP6 corner-seam corrective work in progress (2026-08-09)

- User review found that the first coherent V2 cube-net correction welded the open corner crack into a sharp near-vertical cliff. This is a real failed seam, not acceptable continuity.
- The saved 100 km camera regression measured a 48,836 m neighbouring-sample jump with the global cube-net root. Rebasing the same cube net at the camera still left a 23,306.3 m jump over the full 32 km BCCM window. A flat cube net cannot cover a cube-vertex neighbourhood without a cut or overlap.
- Closed BCCM sampling now has `TERRAIN_SAMPLE_PATCH_MAPPING_LOGICAL_CHART_V3`. BCCM vertices stay flat; canonical Terrain lookup uses an observer-local logical-sphere exponential chart derived from the existing COBE projection and its local differential. This is sample mapping only, not CHP geometry curvature.
- CPU page generation and GPU analytic height/normal sampling use the same V3 chart root and presentation offsets. The renderer sends the root direction and COBE-scale angular tangents to the shader. Closed block owner identity is derived from the V3-mapped block centre.
- At the reported cliff camera, the 64 m neighbour scan over 32 km now measures 70.9999 m, within the calculated 76.0283 m COBE Jacobian bound. No kilometre-scale branch remains in that fixture. Cross-LOD shared-point disagreement remains 0 m.
- Temporal stability is not visually accepted yet. Re-rooting after a 1 m observer move can change the far-corner canonical lookup by up to 1.03252 m over the 32 km window. This is continuous but may look like distant terrain swimming. Dynamic chart roots also change Hybrid page identity while moving; no moving-Hybrid performance qualification exists yet.
- `build_multinet.bat` passes and deploys. `world_domain_fixture`, full `bccm_surface_edge_fixture`, `canonicalization_fixture`, `bccm_flat_normal_fixture`, and `editor_domain_navigation_fixture` pass. A normal Vulkan editor preflight loads `test.tscn`, initializes both BCCM materials, and exits cleanly with no shader, GDExtension, or GDScript error.
- `wp6_editor_transition_probe.gd` now uses explicit float functions/types in both Project-Nations and the evidence copy. The legacy editor tracker now calls `publish_editor_view_camera()` instead of creating an invalid cross-tree camera NodePath.
- The custom headless editor path can still fail copying `multinet.dll` to `~multinet.dll` and then crash in `GDExtensionManager::ensure_extensions_loaded`. The normal Vulkan preflight is clean; do not treat that headless loader defect as a Terrain compiler verdict.
- The existing WP6 evidence ZIP predates V3 and is not proof of this corrective pass. Do not refresh/package evidence until the user visually retests the reported cliff location and checks motion/performance.

## WP6 motion-skip review and correction (2026-08-09)

- The 26.97 s user recording was reviewed at 1 fps, 4 fps, and frame-by-frame around the visible holds/jumps. It contains 809 captured frames. The overlay shows sustained ground speeds near 20,000 m/s through the main motion intervals: at 30 fps that is roughly 667 m of travel per captured frame. Face changes and large silhouette changes at that speed are not, by themselves, missing Terrain tiles.
- During the recording, closed placement failures remain zero and all active LODs remain populated. No missing-block event was found. The real implementation defect was that ordinary horizontal movement incremented `unfolding_generation`, making each editor/runtime update claim a new flat presentation lattice.
- Ordinary movement now keeps the presentation generation stable while the V3 sampling chart still follows canonical face/U/V authority. Explicit canonical resets/teleports remain generation boundaries. Renderer logical-chart uniforms are change-detected from the actual encoded root and root presentation position rather than assuming generation changes on motion.
- This does not erase the known V3 projection tradeoff: the measured far-corner canonical reprojection remains up to 1.03252 m for a 1 m observer move over the 32 km test window, and moving Hybrid pages still have chart-dependent identities. User visual/performance retest is required; do not claim the recorded skipping is fully eliminated yet.
- The overlay now reports `presentation generation`. It should remain constant during ordinary navigation and change only after a real reset/teleport/domain rebuild.
- The dead private GDScript `_frame_epoch` variable was removed from the runnable and evidence controller copies. The adapter remains the sole epoch authority. Runnable/evidence controller, editor-camera sync, and domain-diagnostic scripts have pairwise identical SHA-256 hashes.
- `sync_editor_camera.gd` now defers BCCM NodePath resolution until its preview node is inside a SceneTree. A captured verbose Vulkan preflight loaded `test.tscn`, reloaded `canonical_observer_controller.gd` without the unused-variable warning, and initialized both BCCM materials. That captured run exposed the preview-path error before the guard; a subsequent warm start/quit exited after the guard but its console output was not retained, so interactive confirmation of that secondary guard remains appropriate.
- `build_multinet.bat` passes Rule 3 for 46 files, reports no remaining Ninja work, and deploys. Fresh `world_domain_fixture`, full `bccm_surface_edge_fixture`, `canonicalization_fixture`, `bccm_flat_normal_fixture`, and `editor_domain_navigation_fixture` runs all exit 0.

## WP6 positional-stability correction awaiting visual acceptance (2026-08-10)

- The newer 25.07 s recording isolates a real presentation defect: while the editor camera reverses over the same area, an existing mountain relocates in the flat view even though LODs remain populated. This is not explained by movement speed or missing blocks.
- The direct cause was movement-time rebasing of `unfolding_root_frame` and its flat X/Z anchor. The V3 logical-sphere chart then remapped every already-visible presentation coordinate whenever the observer moved.
- Runtime and editor movement now keep the logical sampling chart and flat anchor immutable for the current presentation generation. A reset, teleport, or domain rebuild may create a new generation; ordinary navigation may not.
- Finite editor navigation now captures the raw-editor-to-canonical offset when the finite domain is initialized, clamps canonical X/Z to the exact finite bounds, and keeps the accepted presentation origin at the boundary if the editor camera travels farther outside. This replaces the old behavior where switching wrapping off at a far closed-world coordinate placed every candidate outside the finite domain and made terrain disappear.
- The debug summary now reports the sampling-chart anchor X/Z and face. These values are diagnostic state, not fabricated terrain state.
- The corrected DLL is `multinet_ext/build/windows-editor/multinet.dll`, 10,718,208 bytes, SHA-256 `55E69D2A329209FD758D1E143BC380D79BE3EF3F32D4EB506C2457E173439E0E`.
- `build_multinet.bat` passes Rule 3 for 46 files and reports no remaining Ninja work. `world_domain_fixture`, `editor_domain_navigation_fixture`, `canonicalization_fixture`, `bccm_flat_normal_fixture`, and the full `bccm_surface_edge_fixture` all exit 0 on this source state.
- A fresh normal Vulkan editor is running against the corrected DLL for the user's exact same-mountain forward/back test and wrapping-off test. Do not package or claim visual closure until the user accepts both behaviors.
- The fixed observer-local chart removes movement-time reprojection in its active region, but it does not prove that one flat chart can cover an entire sphere without an antipodal singularity. A future atlas/rebase policy remains a separate long-distance design question; CHP is still out of scope.

## WP6 finite-origin and small-chart correction awaiting visual acceptance (2026-08-10)

- User video at 10/20 km exposed three additional integration defects: finite canonical clamping dragged the presentation to the switch location, partially intersecting coarse blocks left detached edge strips, and closed outer rings sampled the exponential chart beyond its one-to-one region, producing concentric repeated terrain.
- Finite editor X/Z now maps directly to the world-fixed rectangle centred at Godot origin. The raw editor camera remains free outside it; canonical diagnostics clamp to the real boundary and report outside state. Toggling from a distant closed presentation no longer establishes a finite-world offset.
- Finite partially intersecting blocks are now placed symmetrically even when their minimum corner is outside. The vertex shader collapses outside vertices onto the exact finite rectangle before height/normal evaluation, so no coarse-ring geometry may survive beyond the domain edge.
- Finite presentation binding now carries the observer altitude as the frame origin. Combined with the terrain corner's negative frame-relative altitude, canonical ground remains at Godot Y=0 instead of drifting with the editor camera.
- Closed flat presentation now derives an additional effective BCCM prefix from the entire terminal-ring corner radius and the nearer-hemisphere radius (`pi/2 * logical_area_radius`). This prevents the exponential chart from folding outer rings back over tiny closed worlds. Expected effective counts are 5 at 10 km and 6 at 20 km with the ordinary radius-4 profile; 100 km and larger retain all eight levels.
- Godot editor X/Z remains unbounded presentation intent in closed mode; canonical face/U/V is the bounded authority. Runtime already keeps its physical camera at local X/Z zero. No editor-camera teleport was added.
- Root `build_multinet.bat` passes Rule 3 and deploys. World-domain, editor-navigation, canonicalization, flat-normal, and the full WP5.2 surface-edge/page-replacement fixtures pass. The WP5 suite first had one asynchronous fallback page remain under parallel CPU contention, then passed 7/7 motion scenarios when rerun alone.
- Corrected DLL: 10,718,208 bytes; SHA-256 `4BB612473823A31AA7171C6F5E80758FC53ABB6F61AA0D4355E65FC2F891D8A5`.
- A fresh normal Vulkan editor is open for finite-edge/origin and 10/20 km closed-chart visual review. Do not package evidence or claim visual closure until the user accepts the rendered result.

## WP6 finite retention and bounded chart V4 awaiting visual acceptance (2026-08-10)

- The reduced closed LOD prefix did not remove the 10 km whirlpool. The live renderer was receiving the reduced level count; the remaining radial terrain came from the V3 exponential map itself (`cos(angle)` / `sin(angle)`), not from noise or an ignored rebuild.
- New closed sample patches use `TERRAIN_SAMPLE_PATCH_MAPPING_BOUNDED_CHART_V4`. V4 maps the local tangent plane with `normalize(root_direction + angular_tangent)`, so increasing presentation distance cannot orbit or repeat the sphere. V3 remains decodable for legacy identities; V4 has a distinct patch identity and GPU instance bit.
- CPU page sampling and GPU analytic sampling both dispatch by the patch mapping version. Hybrid pages cannot alias V3 and V4 identities.
- Finite candidate generation again uses the clamped canonical boundary observer while outside, but finite geometry remains fixed at Godot world origin. This retains the last full boundary-centred clipmap instead of peeling it away one camera-centred slice at a time; raw editor frustum culling still applies.
- V4 extension DLL: 10,718,208 bytes; SHA-256 `28812DDB9BCA7DC9D0342E96BA05D41ECBF63A08A3EDE3BA9417F008011B564E`.
- `test.tscn` and the V4 shader load in a normal Vulkan editor with no GDExtension, GDScript, or shader errors. Visual finite-retention and no-whirlpool acceptance is pending.
- The root build compiled and linked the V4 DLL, but Windows stalled while relinking the large `bccm_surface_edge_fixture` twice. The stalled build trees were terminated by exact PID without deleting outputs. The full WP5 fixture must be relinked and rerun before packaging; do not claim that regression gate on V4 yet.

## WP6 local exponential chart V5 verification (2026-08-11)

- The V4 bounded tangent chart was correctly identified as the source of the remaining distance stretch: its gnomonic-style normalization changes metre scale as presentation distance grows. This was projection distortion, not noise discontinuity.
- New closed-world patches use `TERRAIN_SAMPLE_PATCH_MAPPING_LOCAL_EXP_CHART_V5`. V3 remains decodable for old identities; V4 remains decodable for compatibility, but new CPU/GPU work uses the observer-local exponential chart.
- Closed flat presentation now limits the retained clipmap footprint to half the logical-area radius. The runtime and editor rebase the chart root after one quarter of that safe radius, keeping the active BCCM locally metric without moving the actual Godot camera or canonical observer.
- V5 extension DLL and deployed DLL match at SHA-256 `F648C93B7636C0339B53F407E2D50692D2FB1F683CB7E3E3B78DE74AA335D9E1` (10,718,208 bytes). `build_multinet.bat` linked the extension and relinked `world_domain_fixture.exe`.
- Fresh fixtures pass: world-domain V5 mapping/level-cap and shared-edge checks; canonicalization; BCCM flat-normal LOD0-LOD7; editor closed transitions, finite boundary, below-ground altitude and origin anchoring; and the full WP5.2 surface fixture including all crack-boundary fixtures, CPU/FP32 parity, and Milestone-B gates.
- Normal rendered Vulkan Godot loads `test.tscn` with the V5 DLL and no GDExtension, shader, or GDScript errors. The runtime wrap receipt records a real face 0→5 transition. The editor transition receipt drives the actual viewport from `x=0` to `x=24,412.415 m`, crosses face 0→5, and completes at effective LOD6 without an error marker.
- A captured far-edge frame shows locally shaped terrain at `x=24,412.4 m`; the remaining diamond boundary is the deliberate local-chart coverage limit, not a stretched/whirlpool projection. This is a rendered preflight, not a claim that distant-world coverage is complete.
- CHP remains out of scope. Actual GPU numerical readback parity remains untested. Do not claim all visual terrain defects are fixed or package a final evidence archive until the user accepts the far-edge visual behavior.
