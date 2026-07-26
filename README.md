# godot-multinet

`godot-multinet` is a C++23 engine fork built on Godot 4. It exists specifically to power **Infront**, a game that needs a world that behaves honestly. Stock Godot is a remarkable foundation. We love it, we respect it, hence why we chose it. Could have started from scratch but that would have cost us a lot of time... thing is, it just doesn't ship with systemic terrain systems, a deterministic network recovery setup , or zero-allocation runtime laws out of the box!

The doctrine simple and straightforward. one procedural truth birthing multiple bounded representations. We do many cool things with this simple rule.

---

## Why Multinet Exists

Infront (a game you should be looking forward to) features dynamic water, structural destruction, deep nation simulation, and thousands of persistent entities. The accepted event order doesn't need to exist twice. Stock engine nodes are helpers.

What broke the default approach? Well, scene trees and raw node hierarchies get heavy real fast when thousands of entities need physics, navigation, and weather forcing at the same time. Multinet moves semantic state into data-oriented domain systems while keeping Godot rendering pixels and managing window lifecycles.

---

## Core Doctrine & Architecture

Every system owns its state! they publish immutable snapshots, and fail elegantly when hardware can't keep up.

- **Canon Layer:** Owns accepted durable event ordering, idempotency, and recovery metadata. If canon is confused, the world confused. So no, canon is never confused!
- **Multinetwork (multinet origin):** Does transport, interest management, and spatial correction delivery. Transport is not ownership!
- **Jolt Physics:** Collision truth and physical queries.
- **CADENCE (proprietary motion system thinned out for public use):** Does locomotion, gait, pose, and contact expectation.
- **Weather & BoltzField:** Does atmospheric forcing and coarse optical transport.
- **LivingWorldRendering (LWR):** Handles representation, direct lighting, and final pixels.


---

## Platform Floors & Runtime Law

Multinet is being designed to run on humble hardware without stuttering. THE GOAL is...

- **Portable Floor:** Samsung A21s, iPhone 7 Plus.
- **Desktop Floor:** GTX 860M 2GB, i7-4710HQ, 16GB DDR3 RAM, 5400 RPM HDD.

### Hard Runtime Limits
1. **Zero hot-path heap allocations.** Arenas handle transient data. Bounded pools handle persistent entities.
2. **Zero synchronous gameplay readbacks.** No waiting on GPU, disk, or network in frame loops.
3. **FP64 World Coordinates.** FP64 handles global persistence. FP32 or integer region-local coords handle rendering and Jolt.

If a feature causes frame stalls on the portable floor, that feature has a design question to answer. Such is life.

---

## Determinism

Procedural generation and network validation rely on `SquirrelNoise5 v1` as the canonical integer hash. Identical coordinate bits, seed, and algorithm version return the exact same `uint32_t` across scalar C++, SIMD C++, Godot shaders, and mobile shaders.

- **Exact Parity:** Hashes, seeds, IDs, integer masks, material classes, topology flags.
- **Tolerance-Gated stuff:** Heights, gradients, normals.
- **Cosmetic:** Microdetail, particles.

---

## Code Boundaries & Building

New engine modules live under `modules/`. Internal kernels use C++23 PODs, `std::span`, arenas, and data-oriented containers.

```bash
# Build editor binary (SCons)
scons platform=windows target=editor dev_build=yes
```

This readme will expand over time as we build. This is just the beginning.
