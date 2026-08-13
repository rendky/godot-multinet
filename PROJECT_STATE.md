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

## WP6 transition diagnostics and face cue preflight (2026-08-11)

- Closed-mode diagnostics now receive the actual camera horizontal heading from both the runtime Camera3D and the editor viewport camera. The adapter computes each current-face edge's destination face, outward flat direction, heading dot, edge distance, and signed along-heading distance. The overlay reports every edge as toward/away/side; finite mode does not show closed transitions.
- Closed-mode shader materials now select six deterministic colors from a separate packed canonical owner-face field; sampling face bits remain untouched. Adjacent face-owned patches therefore retain mixed colors at a crossing instead of recoloring the whole chart when the sampling root changes. Finite mode remains the existing analytic green. The color uniforms are bound only on the closed material path.
- `build_multinet.bat` passed Rule 3 and deployed matching build/deployed DLLs with SHA-256 `073AA97535CE798F57F5CE779FA5CCFE904BD78CB63CF5964049293E90ED13CA`.
- `world_domain_fixture`, `canonicalization_fixture`, `bccm_flat_normal_fixture`, and `editor_domain_navigation_fixture` all passed. Normal Vulkan `test.tscn` and the bounded WP5 live smoke load without GDScript, shader, or GDExtension errors after the typed-overlay fix.
- The prior sealed `WP6_DOMAIN_DIAGNOSTICS.gd` evidence file was not overwritten. The exact runnable copy is preserved as `multinet_ext/src/evidence/WP6/WP6_DOMAIN_DIAGNOSTICS_FEATURES_20260811.gd` with SHA-256 `D19F1BE5F5EFD66670F9F32444D458D2580F6B0DF5F1E78741F8B0CD15F2DE7E`.
- Full `WP5_LIVE_SCENE_FINAL.tscn` attempts on this V5 build reached phase 2 but hit the existing frame-based phase timeout before an ExactResident state was observed. No full new Gate 6 video is claimed. The source-inclusive feature proof is `multinet_ext/WP6_FEATURES_IMPLEMENTATION_PROOF_20260811.zip` (SHA-256 `B399717877CEBC2E81FC0780F00F3EF76C8D909787F108465B6BC636235B9C69`); the locked prior WP5 video was excluded from this archive.

## WP6 motion skip and face-color boundary correction (2026-08-11)

- Review of `godot.windows.editor.dev.x86_6403.46.14_11-08-26-10.mp4` isolated the visible jump: `frame epoch=63` remained stable while `presentation generation` advanced `80 -> 81 -> 82`; the overlay also showed populated LODs, zero wrap-placement failures, and zero patch crossings. The discrete terrain relocation was therefore a same-face V5 chart re-root and page-identity rebuild, not a camera-speed or missing-tile event.
- Runtime and editor chart re-rooting now require a real canonical face transition. Same-face motion keeps one presentation generation and one sampling-chart root, so the visible clipmap cannot be invalidated merely by crossing the local radius threshold. A later atlas handoff is still needed for intentionally distant coverage.
- Closed-world face tinting now classifies the interpolated canonical direction in the shader and selects the face color from that classification. The owner-face field remains only as a fallback; the visible color seam is no longer tied to a clipmap block boundary.
- `build_multinet.bat` passed Rule 3, relinked the extension, and deployed matching build/deployed DLLs at SHA-256 `1298AC74BF29651038FD85C9FCFE96D63E2335825AD17163959EBF4075026CF2`.
- Fresh `world_domain_fixture`, `bccm_flat_normal_fixture`, `editor_domain_navigation_fixture`, `canonicalization_fixture`, and the full `bccm_surface_edge_fixture` passed, including 7/7 asynchronous motion scenarios and WP5.2 Milestone-B gates. A rendered runtime `test.tscn` smoke exited 0 on Vulkan/GTX 860M and initialized both BCCM materials. Visual acceptance of the user's forward/back recording still remains pending.
- The self-contained source handoff is `multinet_ext/WP6_MOTION_SKIP_FACE_COLOR_PROOF_20260811.zip` (20,865,486 bytes; SHA-256 `9411D7261D2C8D4D14290C6D371D878FF9AC0216FC0D9A8821F16D9AAA8C5FFA`). It contains the full `src/` tree, the proof text, and the exact rendered runtime log.

## WP6 face/corner transition root-stability correction (2026-08-11)

- The remaining visible transition pop was a chart-authority mismatch: navigation advances through the six-face edge table, while V5 terrain samples an exponential chart. Replacing the V5 root with the separately-walked observer frame at a face crossing changes the canonical address under already-visible vertices. The effect is most obvious where two or three faces meet.
- Runtime and editor no longer hard-rebase the V5 chart on a canonical face transition. The existing root and presentation generation stay stable during ordinary travel, so a face crossing cannot invalidate every visible page or relocate terrain under the camera. A future continuous atlas handoff is still required before claiming arbitrary-distance metric coverage; this correction deliberately does not disguise that limit.
- Editor transition diagnostics now latch the most recent actual topology event instead of resetting to zero on the next viewport tick. They expose initial face, final face, final edge, crossing count, and the event frame epoch. The runnable overlay is mirrored exactly at `multinet_ext/src/evidence/WP6/WP6_DOMAIN_DIAGNOSTICS_CORNER_RECEIPT_20260811.gd`.
- `build_multinet.bat` rebuilt and deployed matching DLLs at SHA-256 `F17C2C5A57D68101221983ECA74A51C428FECCA4B44A4EB7D17292CF8FA59A0F`. Core fixtures and a normal rendered Vulkan `test.tscn` smoke passed. No new external visual-review claim is made here.

## WP6 frozen-V5 chart regression under investigation (2026-08-11)

- User visual review confirms the face/corner transition pop is gone, but exposes a separate V5 regression: after the observer travels far from the frozen chart root, the flat mesh remains planar while its canonical Terrain lookup follows the exponential chart. Terrain detail and the canonical face-color boundary then visibly arc/curl. This is not noise, a tile crack, or a camera transform fault.
- The current flat-chart level cap only budgets the terminal clipmap radius around the observer. It does not include observer-to-chart-root displacement, so it can report a nominally safe local footprint while rendering it far outside the root's locally metric neighbourhood.
- Do not restore the old hard root rebase: that is the mechanism that produced the visible two-/three-face terrain relocation. A correct arbitrary-distance answer needs a continuous local-chart atlas/handoff and stable canonical page identity; it cannot be honestly claimed from a single frozen planar chart. CHP remains out of scope.
- The current radius-4 profile produces about 8.192 km outer coverage at S=100 km. Radius 6/8 would be about 12.288/16.384 km respectively at the current outer LOD, but radius 8 already reaches the 256 candidate-instance capacity. Any user setting must expose a guarded local-coverage budget and its actual effective extent, never promise global flat coverage.
- Implemented the guarded `closed_flat_coverage_radius_blocks` Inspector setting (1..8) and read-only `closed_flat_visible_extent_km`. It rebuilds the renderer rather than changing fixed-capacity buffers live. New diagnostics report the full V5 footprint: root-to-observer offset plus terminal-ring corner, and whether that full footprint remains within the local chart radius.
- The coverage property is deliberately not a cure for the visual curl: increasing it makes the local-chart budget tighter. `world_domain_fixture` was rebuilt and passed its new radius 4/6/8 extent checks and radius-9 capacity rejection, together with its existing topology/page/LOD checks.
- Root `build_multinet.bat` built and deployed matching extension copies at SHA-256 `EE6B962154C7F1279B921B5055DDC52D4CAA8FC0D35AA6B1EBF37FB196E289B5`. A normal Vulkan `test.tscn` preflight with isolated user data initialized both BCCM materials and emitted no extension, shader, or GDScript error. This proves loading only; visual curl remains unresolved and must not be packaged as closed evidence.

## WP6 analytic V5 local-chart tracking corrective pass awaiting visual/performance acceptance (2026-08-11)

- The frozen-root mitigation is rejected for normal navigation. Once the observer travelled far from its root, V5 evaluated large exponential-map angles for every visible vertex and normal sample. This curved face tints away from the real observer boundary, made the transition receipt appear unrelated to the visible seam, and plausibly caused the reported severe GPU slowdown.
- In `AnalyticBase` with analytic page prewarming off, runtime and editor now update the V5 chart root and presentation anchor from the actual canonical observer on every movement tick. This is continuous motion rather than a threshold rebase, keeps V5 inputs local, and does not invalidate chart-addressed page payloads because none are active in this mode. The presentation generation remains stable.
- Hybrid and analytic-prewarm modes deliberately retain their immutable chart root. Their page identities include the sample chart; moving that root needs a page-aware atlas handoff and was not papered over here.
- New overlay output names the chart mode and whether local tracking is active. In the normal interactive test it must read `AnalyticBase local chart | tracking=true`; the footprint should show observer offset near zero and remain inside the safe radius.
- `build_multinet.bat` passed Rule 3 and deployed matching DLL copies at SHA-256 `455EB447B71B448E2255A8ECFC1680BAE732929F3F29BFF6496CA9C26D7FA4E5`. Fresh `editor_domain_navigation_fixture`, `world_domain_fixture`, and `bccm_flat_normal_fixture` all passed. A normal Vulkan `test.tscn` smoke initialized both BCCM materials with no extension, shader, or GDScript error. Visual continuity and recovered FPS are still user-review gates; do not package this as closed evidence yet.

## WP6 three-face analytic chart-basis correction awaiting visual acceptance (2026-08-11)

- Review of `godot.windows.editor.dev.x86_6458.00.19_11-08-26-12.mp4` confirmed that the remaining trouble is concentrated where three canonical cube faces meet. Navigation itself still uses canonical six-face edge transport; neither the editor Camera3D nor the runtime camera is teleported by coordinate wrapping.
- The V5 analytic renderer did, however, rebuild its local exponential-chart tangents from that transported flat frame every tick. At a cube vertex two valid edge aliases exist. Their discrete ordering can alter that flat basis even though the canonical direction is continuous, producing a visible terrain/chart rotation.
- Analytic V5 now persists a geometric chart basis and transports it by the shortest 3D rotation between successive canonical directions. The face alias can still change for authority/diagnostics, but it no longer decides the terrain chart orientation. Hybrid and page-prewarm paths are intentionally untouched because their page identities remain chart-addressed.
- `world_domain_fixture` now forces a diagonal two-edge cube-vertex crossing and records `corner_chart_continuous_transport=1`, checking tangentness and metric preservation after the compound transition. Fresh `world_domain_fixture`, `editor_domain_navigation_fixture`, and `bccm_flat_normal_fixture` all pass.
- Root `build_multinet.bat` rebuilt and deployed matching DLL copies at SHA-256 `66CB497056E1BBE58D2B76577C82C2F1EE12C9A11D8E4BFCF669D77BF6A81070`. This is a code/fixture result only. The user must visually repeat the three-face route before the issue is considered closed or evidence is packaged.

## WP6 three-face input-frame correction awaiting visual acceptance (2026-08-11)

- User confirmed the three-face terrain skip is gone, then reported that WASD becomes directionally wrong at and beyond the same corner. This exposed the remaining split authority: V5 rendering used the transported geometric chart basis while navigation still converted flat input through the discrete `active_frame` face basis.
- Closed `AnalyticBase` navigation now projects presentation-plane motion through the active V5 chart tangent vectors and solves the current COBE face Jacobian for the required U/V movement. `active_frame` remains responsible for canonical edge transport and receipts, but no longer controls player heading in this mode. Finite, Hybrid, and analytic-prewarm paths are unchanged.
- `world_domain_fixture` now verifies that a diagonal corner input mapped through the chart advances in the rendered-chart direction. Fresh world-domain, editor-navigation, and flat-normal fixtures pass.
- Root `build_multinet.bat` rebuilt and deployed matching DLL copies at SHA-256 `6D4B59DDEDBC866D142C9F38C2431122ADFD43DED98B72631013E797397D8D08`. User visual retest of slow WASD motion across a three-face meeting is still required. Do not package closed evidence before that result.

## WP6 deterministic analytic heading correction awaiting 5,000 km visual acceptance (2026-08-11)

- The continuous V5 chart basis previously used shortest-rotation transport for every observer movement. That removed the local three-face alias pop, but it also accumulated path-dependent rotation over long closed-world travel. At 5,000 km this made flat WASD intent feel spherical again near a three-face meeting.
- AnalyticBase local tracking now derives its chart X/Z directions deterministically from the canonical direction, using the initial PositiveX presentation X direction as a fixed reference. The same canonical position therefore receives the same heading regardless of the route used to reach it. The renderer and analytic input mapping consume this same chart.
- The reference has two topologically unavoidable poles at the +/-Z direction. The old short transport is retained only as a guarded fallback inside that degenerate pair. This is an honest local fallback, not a completed global pole-atlas solution.
- `world_domain_fixture` was rebuilt and passed with `corner_chart_deterministic_heading=1`; `editor_domain_navigation_fixture` and `bccm_flat_normal_fixture` also passed. The root batch compiled and copied `multinet.dll` to Project-Nations before its unrelated full `bccm_surface_edge_fixture` incremental-link artifact failed. Do not claim the full surface fixture or final visual acceptance from this pass.
- Required user review: use the normal Project-Nations editor with a 5,000 km closed equivalent side, make slow WASD passes through the prior three-face area, then revisit it after long travel. The expected result is stable flat steering and no new terrain/chart rotation. Do not package closed evidence until that review passes.

## WP6 ordered corner traversal and analytic uniform update (2026-08-11)

- The deterministic-heading experiment was rejected after the long-world steering regression persisted. Closed AnalyticBase input is back on the transported canonical frame; V5 rendering retains its separate continuously transported geometric chart basis.
- Canonical frame advancement now consumes a flat movement segment in boundary order. At a simultaneous U/V corner hit it chooses the exit using the motion's inward component, transports across that edge, then projects the remaining segment into the new frame. This removes the old endpoint-only alias choice that could make a first two-face crossing defer or vertically skew at a cube vertex.
- Per-tick AnalyticBase V5 chart values now use four global shader parameters once per renderer update instead of writing nine material parameters across every LOD. This targets the reported severe CPU/render-server overhead while leaving Hybrid/page chart identity behavior unchanged.
- Fresh `world_domain_fixture.exe` passed `corner_path_ordered_traversal=1`, continuous chart transport, 3,264 shared-edge samples, zero cross-LOD physical gap, hybrid page edge identity, and multi-wrap. Build and deployed extension DLL are byte-identical: SHA-256 `9C32FF127843F88B9E1874CC558BE7B71839A39C2FAAB7339CEACFAB0AEDBE4E` (10,765,312 bytes).
- No rendered user acceptance is claimed. The remaining gate is normal Project-Nations visual/performance review of the prior 5,000 km three-face route.

## WP6 V5 motion/render authority correction awaiting rendered retest (2026-08-11)

- The ordered cube traversal test was not sufficient: the V5 renderer sampled a continuous exponential chart while editor/runtime movement still converted the same held presentation vector through the discrete transported cube frame. At a three-face junction this lets the observer move away from the point the mesh visibly represents.
- Closed V5 movement now maps the presentation delta through `try_map_logical_chart_delta_to_face_delta()` in both the runtime controller path and the editor viewport path. Finite navigation keeps its direct rectangle mapping. A chart mapping failure does not silently fall back to a different cube-frame direction.
- The world-domain fixture now checks two consecutive V5 presentation-plane steps through the near-corner route against the actual V5 sampled direction. It passed as `corner_v5_motion_alignment=1` together with continuous chart transport, ordered crossing, shared-edge, page, and multi-wrap checks.
- V5 analytic shader evaluation now uses a guarded high-order local exponential polynomial inside the allowed chart radius, with the exact trigonometric path retained outside it. This removes repeated sin/cos evaluations from height and normal sampling in the normal guarded path; no measured FPS claim is made yet.
- Root `build_multinet.bat` passed Rule 3 and freshly linked the extension and fixture. The deployed Project-Nations DLL exactly matches the build DLL: SHA-256 `1966DE829966694A2372889DFA6D59EB6848ED4D8D5C5AAF1CEB19427FCB2777` (10,765,312 bytes). User rendered review of held-key three-face crossing and FPS remains required.

## WP6 analytic block-atlas handoff for three-face route (2026-08-12)

- The reported terrain-follow failure is not frame-rate lag. In ordinary closed `AnalyticBase`, moving the V5 exponential-chart root every observer tick changed the canonical address of already-visible terrain. The error accumulated most visibly after a two-face route entered a third face: the camera kept advancing while the flat terrain appeared to trail behind.
- Ordinary `AnalyticBase` without analytic page prewarming now derives every visible patch through the existing V1 canonical sample-patch atlas from the current transported observer frame and presentation offset. It no longer binds or samples the moving V5 chart on that path. Hybrid and page/prewarm paths remain on their prior V5/page identity route; committed page semantics were not changed.
- Closed AnalyticBase editor/runtime motion uses the transported cube frame to match that atlas. The V5 chart delta mapper stays limited to the paths that actually render the V5 chart.
- `world_domain_fixture` now proves both the prior one-edge case (`analytic_block_atlas_fixed_patch=1`) and the exact diagonal two-edge route into a third face (`analytic_block_atlas_corner_fixed_patch=1`). The full fixture passed with `STATUS: PASSED WITH EVIDENCE`; existing V5 movement, shared-edge, cross-LOD, hybrid-page, and multi-wrap checks also passed.
- The required root `build_multinet.bat` passed Rule 3 and completed. Built and deployed extension copies match exactly at SHA-256 `43773AC0E896CC61880B49DE3F94B50C70A40DA41DBFAB7A87B737A1AA4222BA` (10,772,480 bytes). This is a code/fixture pass only. The user must still visually review the previous two-face-to-third-face route before this defect is called closed or a new proof archive is created.

## WP6 failed analytic atlas experiment rolled back (2026-08-12)

- The V1 sample-patch handoff described above was rejected immediately by rendered inspection. Although its patch-key fixture passed, it combined face-local V1 patch transforms with the existing global flat presentation placement. The result was discontinuous vertical face walls and an unacceptable GPU cost. The key-only test was therefore not sufficient evidence for a renderer handoff.
- The experimental ordinary-AnalyticBase V1 atlas selection, matching cube-frame motion bypass, V1 test gates, and V5-global binding bypass were removed. Ordinary AnalyticBase is back on the prior V5 logical patch/render path; V5 closed motion remains paired to that renderer. No Terrain/BCCM/page semantics outside this failed experiment were changed.
- Required root `build_multinet.bat` passed Rule 3 and deployed matching rollback DLLs at SHA-256 `52E692B08908CDF96E7D4CB17CB4ECE99F6D5FA9161601B4ADCC79494C504ED4` (10,769,408 bytes). Fresh `world_domain_fixture` passed with V5 corner motion, continuous corner chart transport, ordered traversal, shared edges, cross-LOD zero gap, Hybrid page identity, and multi-wrap receipts.
- The screenshot-level regression should now be gone. The original terrain-follow issue remains open; do not claim it is solved and do not create a proof archive from this rollback. Any future fix needs an explicit renderer-placement proof, not patch-key equality alone.

## WP6 editor terrain-follow / high-altitude diagnostic receipt (2026-08-12)

- Re-reviewed `godot.windows.editor.dev.x86_6402.20.09_12-08-26-13.mp4`. The camera rises roughly from 23 km to 34 km while X/Z changes by under 2 km. With the current roughly 8 km flat-coverage footprint, terrain directly below it can legitimately appear small and far away at that altitude. The recording alone does not establish a horizontal terrain-placement fault.
- There is nevertheless a real unsafe code path: if closed-chart motion or canonical frame advance is rejected, the editor render update still consumes the latest editor X/Z while the physical presentation origin remains at the last accepted observer position. That can create genuine horizontal anchor drift, but the old video did not expose the needed receipts to prove it fired.
- Added diagnostic-only adapter summary values: `editor_presentation_anchor_lag_m`, `editor_rejected_chart_motion_count`, and `editor_rejected_frame_advance_count`. The mirrored runnable/canonical WP6 diagnostic overlay shows them as `presentation anchor lag` and `rejected chart/frame`; it does not derive any world math or change terrain/page/renderer behavior.
- Root `build_multinet.bat` passed Rule 3 and compiled/linked/deployed the adapter change. Built and Project-Nations debug DLLs are byte-identical at SHA-256 `EE085635E2BD31F61433495EB45AAC10AC1D2B90638521D10DDF7384E582BC9F` (10,769,408 bytes). The normal sandboxed build still cannot create `.ninja_lock`; the scoped normal root build was used, without ACL changes or cleanup.
- Required next visual receipt: repeat the same editor climb and three-face route with `test.tscn`. A healthy result keeps anchor lag essentially zero and rejected chart/frame at `0 / 0`. If either counter rises, treat it as a confirmed transport defect; if both remain zero, the apparent distance is the current finite coverage footprint versus altitude and needs a coverage/frustum design decision, not a terrain-follow patch. Do not package this as closed evidence yet.

## WP6 three-face editor anchor and scale-conditioned V5 fix awaiting rendered acceptance (2026-08-12)

- The newer recording corrected the prior interpretation: the camera did not return to a side view. It rotated 180 degrees to show terrain already left behind. The defect is therefore a closed-editor presentation-anchor split, concentrated around a two-face route entering a third face.
- Closed editor presentation X/Z now follows the actual editor camera every update, even if a canonical chart-map or frame-advance attempt is rejected. Rejection counters remain visible, so this prevents the grid being stranded without concealing a canonical transport failure.
- The V5 presentation-to-face mapper used an absolute `1e-24` Jacobian determinant cutoff. Because its basis is expressed per metre, that threshold rejects valid well-conditioned maps as the closed-world side grows. It now uses a scale-invariant relative conditioning test. This is the code path that explains the issue becoming worse at 5,000 km and Earth-scale experimentation.
- `world_domain_fixture` now walks 4,096 V5 presentation steps past a three-face corner at both 100 km and 5,000 km. A freshly linked no-PDB probe passed with `corner_v5_continuous_editor_walk=1` and `STATUS: PASSED WITH EVIDENCE`.
- The normal debug CMake linker repeatedly stalled on generated DLL/PDB outputs after source compilation. A clean no-PDB link of the same freshly compiled debug objects succeeded instead. The build-tree and Project-Nations addon DLLs are byte-identical at SHA-256 `61CE63E7E804D8D6AD92300CF4A7448078BFAF679D48701A8DE47D50A07E3E72` (7,789,056 bytes). Do not call the visual defect closed until the user retests the original yellow/purple-to-red three-face route in the editor.

## WP6 chart-relative precision correction and face-colour visibility toggle awaiting visual acceptance (2026-08-13)

- Re-review of `godot.windows.editor.dev.x86_6401.22.01_13-08-26-15.mp4` found the terrain-following and Earth-scale three-face math visually stable. The remaining shimmer is consistent with high-frequency analytic octaves being evaluated unchanged on coarse outer clipmap rings; it is not evidence of another face transport failure.
- The first attempted LOD-footprint octave filter was rejected after the follow-up recording: the shimmer scales with world size and distance from the centre, which points to coordinate precision rather than a simple ring Nyquist problem. That filter was removed, restoring the original octave stack.
- The generated BCCM shader now consumes chart-relative block offsets for logical-chart instances. The CPU submission packs each block centre relative to the chart root before converting to float; the shader no longer reconstructs two large absolute presentation coordinates and subtracts them in float. This is a render-coordinate precision change only; domain, navigation, face ownership/classification, page identity, and committed-delta semantics are unchanged.
- Added the inspector property `face_colors_enabled` on `MultinetBCCMNode3D`. It updates all existing materials immediately; when disabled, closed-world terrain uses the normal green base colour instead of the six diagnostic face colours. The default remains enabled.
- Changed translation units compiled cleanly with MSVC into a separate writable staging directory. The official `build_multinet.bat` still stops before compilation because the generated build directory cannot create `.ninja_lock`; no source ACLs were changed. A no-PDB debug DLL link succeeded and was copied to the build-tree and Project-Nations addon targets; both match SHA-256 `AA11B8B23A9605DF3F3490CC000E8513454F5888F45033BC79A28A6404F1EBA7`.
- No rendered retest has been performed after this shader change. The user must verify that distant-ring shimmer is reduced and that toggling `face_colors_enabled` preserves geometry and returns the base green material. Do not call the shimmer fix closed until that visual check passes.
- Follow-up recording review shows the dark speckle pattern is stable on terrain slopes and reads as shadow-map acne/shimmer under the test scene's enabled `DirectionalLight3D` shadows. This remains a diagnosis, not a renderer proof; the next minimal preflight should disable only scene shadow casting to separate lighting artefact from terrain sampling before any further terrain change.

## WP6 Earth-scale normal sampling correction awaiting visual retest (2026-08-13)

- The remaining Earth-scale shimmer was traced to the normal path, not face transport: the shader differentiated a planetary-radius surface with a fixed 0.5 m sample while all physical positions were float-valued. That makes neighbouring height samples quantize differently as the camera moves, which presents as dark lighting speckle.
- The shader now chooses a normal sample distance of at least the active LOD spacing and, in closed mode, at least `logical_area_radius_m * 1e-6`. Height evaluation, noise octaves, chart mapping, face ownership, page identity, and face-colour classification are unchanged.
- The first deployment also included an incorrect chart-relative `INSTANCE_CUSTOM` packing change. The user-visible result was a hard block-layout regression: repeated strips, gaps, and mismatched terrain tiles. That packing change has been removed. The shader again receives the established block-index contract and reconstructs the logical chart presentation coordinate as before.
- The updated translation units compiled cleanly. The official `build_multinet.bat` remains blocked before compilation by the generated `.ninja_lock`; a no-PDB link of the existing extension objects plus the updated shader/renderer objects succeeded. Build-tree and Project-Nations DLLs now match at SHA-256 `BD45D1288D6B835C230AC5C14260BB8999E071DD805B45165A43141687FD7E82` (7,847,936 bytes).
- `editor_domain_navigation_fixture` passed. The existing `world_domain_fixture.exe` artifact is unreadable and was not treated as a test result. No rendered retest has been run; the user must repeat the Earth-scale distant movement and inspect whether the speckle/shimmer is materially reduced before this is considered closed.
- The user then reported that the live Earth-scale view appeared uniformly green. Source and DLL audit confirms the face-colour feature was not removed: the node property, six shader colours, material binding, and fragment classification are all present in the deployed binary. `Project-Nations/test.tscn` now explicitly sets `face_colors_enabled = true` so the diagnostic scene cannot inherit an ambiguous serialized/default state. This scene edit does not alter terrain geometry or navigation.
- A stale `terrain_adapter.cpp` object had in fact been included in the prior manual link, so the deployed binary contained the shader-side colour code but not the Godot class registration/property methods. That explains both symptoms at once: no inspector toggle and uniform green terrain. The adapter was recompiled and relinked with the current renderer/shader; the adaptive Earth-scale normal-sampling experiment was removed, restoring the original fixed normal step. The current matching DLL hash is `9CFC230B87A3C9F5DD28588F39BC5CC973F15A3D818CD6EBC5CAB077B6BB1435` (7,849,472 bytes). A full Godot restart is required because the extension is non-reloadable.
