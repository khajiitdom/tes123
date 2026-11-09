# GT Contact Bridge Core (Phase 2)

This directory is an isolated, engine-independent foundation for Phase 2. It does not patch or link
FSMP, Skyrim/CommonLibSSE, Bullet, or the v0.7 visual limiter.

## What is fixed in ABI v1.1

- C-compatible POD structures only.
- Caller-owned frame and sample buffers only.
- Opaque integer identity tokens; no engine pointer may be dereferenced across the DLL boundary.
- The consumer atomically replaces a fixed active-skeleton allowlist.  An empty
  allowlist publishes empty frames, so unrelated actors cannot consume the
  global contact cap before routing.
- Explicit ABI, `structSize`, known-flag, and reserved-zero validation.
- Every successful target replacement returns a monotonically increasing,
  non-zero `targetGeneration`.  `copyLatestFrame` treats the generation placed
  in the caller's frame header as an ownership expectation.  It returns
  `TARGET_GENERATION_MISMATCH` without changing frame/sample output when that
  expectation differs from the producer's current target generation.  When
  ownership still matches but the retained publication belongs to the prior
  generation, it returns `NO_FRAME` until a current-generation frame is
  published.  The distinction lets a consumer recover a lost epoch without
  generation churn during an ordinary one-frame transition.  Target
  replacement and reconnect therefore cannot replay a frame captured for an
  earlier allowlist/session.
- Contact positions use FSMP local-world coordinates; every frame carries the origin offset.

## Collector behavior

- Fixed storage: 2,048 retained observations and 512 published samples.
- Endpoint orientation is canonicalized and normals/relative velocity are flipped consistently.
- Contacts are deduplicated using endpoint identity plus quantized contact points and normal.
- The strongest duplicate is retained.
- On overflow, a total priority order retains the deepest contacts, then highest impulse, then highest
  approach speed. This makes the published set independent of insertion order.
- The selected set is sorted back into canonical identity/contact order before publication.
- Publication and copying are mutex-protected. Collecting a new frame does not erase the last complete
  frame, so readers never observe a partially built snapshot. A retained frame
  is visible only to a caller expecting its exact target generation.
- ABI v1.1 has one process-global target/capture owner.  It is an exclusive
  single-consumer bridge; multiple independent consumers require a future
  registration-handle/refcount ABI and are not supported by this version.

`GTFSMP_GetContactBridgeV1` is deliberately declaration-only here. Its implementation belongs in the
future version-specific FSMP producer patch. The runtime consumer must validate the returned descriptor
before calling `setCaptureEnabled`, `setTargetSkeletons`, or `copyLatestFrame`.

## Build

```powershell
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```
