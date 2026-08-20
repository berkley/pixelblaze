# Pixelblaze patterns — working notes

Animation patterns for ElectroMage Pixelblaze LED controllers. Files use `.js`
but the language is **not** full JavaScript — see the
[language reference](https://electromage.com/docs/language-reference).

Most of what follows was established by measuring real hardware, not by reading
docs. Facts marked **measured** were verified on a device; anything unmarked is
convention or inference. Re-verify before relying on a measured number in a new
context — the devices differ.

## Devices

| Name | Pixels | Notes |
|---|---|---|
| DJ Booth Lightbar | 2560 | 160x16 banner, v3.51, **has the sensor board** |
| Cube | 1280 | 5-sided, 16x16 faces |
| Cuboid / Light Cuboid 1 & 2 | 1024 | 4 walls, 8x32, no cap |
| Lightstick L1, Beacon1/2 | 256 | |

**IP addresses change and must never be trusted.** The Lightbar alone moved
`.128` → `.152` → `.145` over a few days, and `.145` had previously belonged to
a different device. Always confirm identity before writing:

```python
Pixelblaze(ip).getConfigSettings()["name"]
```

To find a device, sweep the /24 for port 81 and read names:
`for i in $(seq 1 254); do (nc -z -w 1 192.168.68.$i 81 && echo $i) & done; wait`

## Toolchain

`tools/` is **untracked on purpose** — it holds a venv, not source. It provides
`pixelblaze-client 1.1.7` under Python 3.14 at `tools/.venv/bin/python`.

### pixelblaze-client gotchas — all of these cost real debugging time

- **No public `close()`.** Call `pb._close()` at the end of every script.
  Without it each run leaks a websocket until the device times it out, and the
  device has only a few slots. Exhausting them locks out the web UI too. There
  is **no API to close another client's socket** — the only fix is `pb.reboot()`
  (patterns and maps live in flash, so nothing is lost) or a power cycle.
- **`getPatternList()` serves a 10-minute cache.** Pass `forceRefresh=True` or
  you will read stale data and draw wrong conclusions.
- **`deletePattern()` is fire-and-forget** (`expectedResponse=None`). If the
  script exits immediately the socket closes before the device acts and the
  delete silently does not happen. Sleep ~3s, then confirm with a forced refresh.
- **`setActivePatternByName()` throws** if the pattern was saved on the same
  connection — the cached list has not caught up. Look the id up from a
  force-refreshed list and use `setActivePattern(pid)`.
- **`savePattern()` requires a `previewImage`** (any JPEG bytes will do) and is
  keyword-only. Pass `id=` an existing pattern id to replace rather than
  duplicate.
- Connections fail transiently. Wrap construction in a retry loop with a few
  seconds' backoff; also wrap `getActiveVariables()`, which intermittently
  returns `None` and then raises inside `json.loads`.

### Uploading

```python
pb.setMapFunction(text)   # writes /pixelmap.txt + /pixelmap.dat, persists to flash
pb.compilePattern(src)    # compile-check WITHOUT uploading — always do this first
pb.savePattern(previewImage=jpeg, sourceCode=src, name=n, id=existing_or_None)
```

`compilePattern` catches language errors in one round trip and is much faster
than uploading and looking at the bar.

## Pixelmaps

Live in `pixelmaps/`, one `.js` per physical object. Format is a **bare
anonymous function expression** — no name, no trailing semicolon — pasted into
the Mapper tab:

```js
function (pixelCount) { ... return map }
```

Map code runs in the **browser**, not the MCU, so full JS is available
(`Math.floor`, closures, array methods). Patterns get none of that.

**Each axis is normalised independently to 0..1** (verified in
`Pixelblaze.createMapData`, which takes per-dimension min/max). Consequences:

- Absolute units and aspect ratios never reach a pattern. A 160x16 bar presents
  to `render3D` as a full unit cube.
- This is why `Lava Lamp 3D.js` carries `zAspect = 0.35` to undo the stretch.
  Expect to need similar compensation, more severe on the Lightbar.
- Coordinates are **0..1 exclusive** at seams; exact 0/1 edge tests are
  unreliable there (see `Test - Cuboid Wiring Check.js`).

Emit `[x, y, z]` with **z = height** to match `cuboid.js` / `cube.js`, so the
existing 3D patterns work unchanged.

### The Lightbar map — `pixelmaps/halfcuboid-bar.js`

5 half cuboids end to end, each with two 8x32 panels on the two faces meeting at
the edge facing the viewer. Reads as a **160 wide x 16 tall** banner. Expander
channels 0–4, 512 px each, startIndex 0/512/1024/1536/2048 — plain sequential,
so one map covers everything.

Wiring, **calibrated against the hardware** (do not change without re-running
the wiring check): 32 columns of 8 per face, serpentine; pixels 0–255 of each
cuboid are the upper face; cuboid 0 is at the left end; **each column enters at
the face's OUTER edge and runs inward to the ridge** (this was the one flag that
differed from the expected build).

`flatten` flag at the top of the file:
- `false` → 3D chevron `[x, depth, height]`, for `render3D` patterns.
- `true` → flat 160x16 plane, for 2D patterns.

Needed because the 3D map's **depth axis is degenerate** — both faces share a
depth for a given row — so `render2D` on the 3D map collapses the banner into an
8-deep smear. Only one map fits on the device at a time.

`Test - Halfcuboid Bar Wiring Check.js` calibrates it. Its `stage` is an
exported var, drivable over the websocket with `setActiveVariables({"stage": n})`.
**Diagnostics on this bar must use distinct hues, never brightness** — dimmed
pixels are not discernible by eye on it.

## The Lightbar's index order snakes

Within each half cuboid the index runs left→right along the upper face one 8px
column at a time, then back right→left along the lower face. **A 1D effect
crosses the bar as a ten-leg zigzag, not a clean sweep.** This decides 1D vs 2D
per pattern:

- **Wave fields survive it** — the snake reads as texture. `Marching Rainbow`,
  `Millipede`, `Oasis`, `Edgeburst` stay 1D.
- **Discrete moving objects do not.** `KITT` and `Sparks Center` are restaged to
  2D; a 1D KITT zigzags across faces ten times per pass instead of sweeping.
- **`Lightning ZAP` stays 1D deliberately** — the snake is free jaggedness for a
  bolt.

Note Pixelblaze calls the **most specific** render function a pattern exports,
so adding `render2D`/`render3D` to a 1D pattern silently restages it onto the
map. That is a real behaviour change, not a no-op.

## Performance — the constraint that shapes everything

**Measured: the Lightbar sustains 8–22 fps at 2560 pixels.** Stock patterns land
around 9–11. Rules that follow:

- **`render` must be close to an array read.** Capture per-pixel constants
  (grid position, radial coordinates) **once during the first render pass** into
  arrays, then look them up. A pattern can only learn its pixels' coordinates by
  being handed them one at a time, so frame 1 renders black and builds the table
  (`frames >= 2` ⇒ `mapped`). Measured 15.5 fps vs 10.6 for the same effect with
  per-pixel maths.
- **Scatter, never gather.** Looping every entity inside every pixel is the
  classic trap: Aurora's upstream did 2560 px x 10 waves = 25,600 lambda calls a
  frame. Build a 160-entry column field in `beforeRender` instead — 1,600
  evaluations, identical output. Same change was needed for Comets and Ripples.
- **Exploit locality.** A wavefront is a thin ring, so touch only the ~24
  columns around it, not all 160 (~290 column writes/frame vs 1920).
- **Avoid per-pixel `perlin`** — 2560 calls/frame is out of reach. Plasma sums
  two 1D noise strips, one indexed by column and one by `col+row`; the diagonal
  indexing is what stops it reading as plaid. 336 calls/frame.
- **A precomputed random table beats the repo's `sin()`-based `hash01`** —
  measured 6.3 fps cost vs 7.6 for the same effect.
- **Beat-gate expensive effects.** Work that only runs during a post-kick flash
  is nearly free.
- **Array cap is 10240 elements.** One 2560 buffer is 25%; three is 75%. Budget
  before writing.

### Framerate measurement is unreliable

Single `getFPS()` readings on this device are noise-dominated. A bisect once
showed *removing* code measuring slower, which is impossible. **Only trust
same-session A/B comparisons holding everything else constant**, and say so when
reporting. Also: an open web UI streams a live preview over the websocket and
costs real frames, which confounds readings.

## Sound-reactive patterns

Requires the audio expansion board. `light` stays `-1` when absent — use that to
fall back to a free-running timer so the pattern still animates.

**Copy the beat detector verbatim from `Sound - Fast Pulse 3D.js:55-92`**:
sub-bass bins 0–3 only (kicks trigger it, snares/claps with content higher up do
not), octave correction (folds a 2x or 0.5x detection back), and outlier
rejection. Patterns are standalone — there is no import mechanism, so every file
carries its own copy. That is already the repo's practice.

**Per-bin AGC is not optional on this device.** Measured `frequencyData` runs
**0.0003–0.003**, an order of magnitude below the 0.006–0.02 the repo's
fixed-gain patterns assume, so any constant scale factor is wrong. Use the
School-A normalisation from `Sound - Cube Frequency Bands 3D.js:165-191`:
rolling mean at `dw = delta / 2000`, peak decaying `0.997`/frame, dynamic range
floored at `max(peak - avg, peak * 0.2, 0.00005)`.

Drive moves from **onsets**, with a tempo clock as backup — onsets land tighter
on the kick, and the clock carries motion through breakdowns and missed beats.

**Known gap, affects every sound pattern here:** there is no absolute noise
gate. The detector normalises against its own recent dynamic range, so in a
quiet room it fires on room noise. Fixing it needs a bass reading from silence,
after which one threshold can be retrofitted across all of them.

## Animation gotchas learned the hard way

- **Decay must be `pow(x, dt)`, not a per-frame multiply.** The repo's older
  trail idiom uses a flat multiply, so tail length changes with framerate — and
  this bar swings 10→21 fps.
- **Anything deposited per-frame rather than per-`dt` is framerate-dependent.**
  Sparks' point deposition gave peak heat 4.3 / 4.9 / 5.4 at 10 / 16 / 21 fps.
  Depositing along the *swept segment* fixed both that and dotted trails (fast
  objects skip columns between frames — at 10fps ~30k frames crossed >1 column).
- **Sub-pixel placement is mandatory for small moving things.** A hard-edged 4px
  box has only five possible positions during a one-cell move, so it reads as a
  five-frame flipbook however high the framerate. Partial edge coverage gave 21
  distinct brightness levels instead of 2.
- **Sample-at-frame-boundaries misses extremes.** KITT's leader essentially
  never lands on the triangle's peak and stopped up to 11 columns short at 16fps;
  detect the turnaround and run the fill to the end.
- **A band must be wider than the distance a front travels per frame**, or it
  steps clean over things. Scale it with `dt`.
- **`wave(p)` is zero at birth.** Using it as a life envelope makes anything
  spawned on a kick invisible for seconds (0.007 brightness at 0.15s). Use fast
  attack / slow release for beat-spawned entities.
- **Persistent effects that light otherwise-dark pixels destroy patterns that
  depend on a dark rest.** Continuous colour corruption wrecked Edgeburst;
  beat-gating it fixed it.
- **Trails: decay the buffer lazily inside `render`**, not in a separate sweep —
  each pixel is touched exactly once a frame and tails become nearly free
  (idiom from `Bouncing Box Trails 2D.js:479-491`).
- Round to the nearest known position rather than binning a continuous
  coordinate: `round(x * 159)`, not `floor(x * 160)` — the latter drops the last
  column into a 161st bin when x reaches exactly 1.0, which it does.

## Language notes

Sliders come from exported `slider*` **functions**, not vars — the function is
called with the 0..1 value. Exported **vars** appear in Vars Watch and are
readable/writable over the websocket (`getActiveVariables` /
`setActiveVariables`), which is how patterns are driven and diagnosed remotely
without watching the bar.

Every pattern should export the `render3D` / `render2D` / `render` trio, each
delegating to the next, so it degrades on other devices.

**Verified available on v3.51**: `perlin(x,y,z,seed)`, `perlinTurbulence`,
`perlinFbm`, `perlinRidge`, `setPerlinWrap`, `smoothstep`, `hypot`, `atan2`,
`mod`, `translate`, `pow`, `wave`, `triangle`, `clamp`, `sqrt`, array `.sum()` /
`.length` / `.forEach`, nested array literals, function pointers, `while`,
`continue`, compound `for` conditions, `hsvPicker*` exports.

**Measured ranges**: `perlin` returns −0.82..0.83, mean ~0 — signed, use
directly as a flow field. `perlinTurbulence` returns 0.007..1.12 — unsigned,
re-centre it. `time()`'s argument is in units of **65.536 s**.

Useful shapes: `wave()` of 0..1 is a smooth 0→1→0 envelope (no pop on respawn);
`triangle()` mirrors space or time and already bounces at the boundary, so it
beats a hand-flipped direction flag; `v*v` or `pow(v,n)` for perceptual
brightness.

## Verification workflow

Always simulate before uploading. Port the state maths to node in the scratchpad
and check bounds, NaN, termination, and **framerate independence at both 10 and
21 fps**. This has caught real bugs in nearly every stateful pattern here —
KITT's short sweep, Sparks' heat drift, Aurora's cost, ripple reflection depth,
population control that ignored its own slider.

For map changes, the strongest single check is that the flattened variant covers
the 160x16 grid **exactly once** — no duplicate or missing cell. Device
coordinates should then match the local computation to within one quantisation
step (1/65535).

Then: `compilePattern` → `savePattern` → `setActivePattern` → measure and poll
exported diagnostics with music playing.

## Repo conventions

- Patterns live flat in the root; maps in `pixelmaps/`.
- Prefixes: `Motion:` accelerometer, `Sound:` audio-reactive, `Test -`
  diagnostics. Suffix `2D`/`3D` for the geometry assumed.
- **Sound-reactive conversions are saved alongside their originals, never
  replacing them.** Ports from the community library credit the upstream author
  in the file header.
- Header comments explain *why*, especially where a value was calibrated against
  hardware or a structure was forced by measurement.
- Work on a branch and open a PR; commit only when asked.

## Community pattern library

<https://patterns.electromage.com> — 300 patterns. The site is client-rendered,
so fetching the HTML gets you "Loading…". The bundle exposes a JSON API that
returns **full pattern source**:

```
/api/v1/patterns?order=popular&limit=100&offset=0
```

Paginate; `limit=300` still returns 100.
