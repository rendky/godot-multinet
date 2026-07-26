# Godot-multinet

`Godot-multinet` is a C++23 engine fork built on Godot 4. It exists specifically to power **Infront**, a game that needs a world that behaves honestly. Stock Godot is a remarkable foundation. We love it, we respect it, hence why we chose it. Could have started from scratch but that would have cost us a lot of time... thing is, it just doesn't ship with systemic terrain systems, a deterministic network recovery setup , or zero-allocation runtime laws out of the box!

The concept we are going with simple and straightforward is *"one procedural truth birthing multiple bounded representations"*. We do many cool things with this rule.

---

## Why Does multinet Have to Exist

Because of **Infront!** (a game you should be looking forward to btw) which is intended to feature dynamic water, structural destruction, deep nation simulation, and thousands of persistent entities. Existing engine modules are great helpers in making Infront real, but the new and highly experimental toys we want to implement would make Infront *distinct!* 

What broke the default approach? Well, we reasonably predict scene trees and raw node hierarchies getting heavy real fast when thousands of entities need physics, navigation, and weather forcing at the same time. With multinet, we want to move the semantic state into data-oriented domain systems hence keeping the Godot base rendering pixels and managing window lifecycles most of the time.

---

## Core Rules and Thinking Style

Every system must own its state! this would make them publish immutable snapshots, and fail elegantly when hardware can't keep up.

- Anything considered **Canon** must own accepted durable event ordering, idempotency, and recovery metadata. If canon is confused, the world is confused. So no, canon is never confused!
- **Multinetwork (multinet origin)** does transport, interest management, and spatial correction delivery. Transport is not ownership!
- Dealing with Collision truth and physical queries is **Jolt Physics**'s job. 
- **CADENCE (proprietary motion system thinned out for public use)** does locomotion, gait, pose, and contact expectation.
- Another unknown proprietary system we'll be thinning out is **BoltzField**, it would handle atmospherics, weather and optical transport.
- **LivingWorldRendering (LWR)** is native to multinet actually. It's just named for eccentricism, but it handles representation, direct lighting, and final pixel stuff.

---

## Platform Floors, Runtime Law

Multinet is being designed to run on humble hardware without stuttering. Our target is as follows:

- **Portable Floor:** Samsung A21s, iPhone 7 Plus.
- **Desktop Floor:** GTX 860M 2GB, i7-4710HQ, 16GB DDR3 RAM, 5400 RPM HDD.

### Hard Limits we'll be enforcing
1. Zero hot-path heap allocations so arenas handle transient data and bounded pools handle persistent entities.
2. Zero synchronous gameplay readbacks to avoid waiting on GPU, disk, or network in frame loops.
3. Couldn't make it a Zero this time around but **FP64 World Coordinates** handles global persistence. FP32 or integer region-local coords handle rendering and Jolt.

If a feature causes frame stalls on the portable floor, that feature has a design question to answer for us. You included! Such is life.

---

## Regarding DETERMINISM which we consider very important.

Procedural generation and network validation rely on SquirrelNoise5 as our primary integer hash because we love it and it's fast. 
It means identical coordinate bits, seed, and algorithm version return the exact same `uint32_t` across scalar C++, SIMD C++, Godot shaders, and mobile shaders.

- **Exact Parity:** Hashes, seeds, IDs, integer masks, material classes, topology flags.
- **Tolerance-Gated stuff:** Heights, gradients, normals.
- **Cosmetic:** Microdetail, particles.

---

## Boundaries which are bound to change as we build.

 Multinet basically lives under `modules/`. Internal kernels use C++23 PODs, `std::span`, arenas, and data-oriented containers.

```bash
# Build editor binary (SCons)
scons platform=windows target=editor dev_build=yes
```

This readme will expand over time as we build. This is just the beginning.
