/*
  CYLINDER BOXES ENGINE (32 around x 8 tall, zigzag-wired height=8)
  - Variable number of boxes (BOX_COUNT)
  - Variable box sizes (SIZE_MIN..SIZE_MAX)
  - Per-box mass (MASS_MIN..MASS_MAX) affects collisions
  - Wrap in X (cylinder), bounce in Y
  - Impulse-based collisions (no intersections), stable speed
  - Rainbow base hue per box + radial hue from center outward
  - Trails (per-pixel persistence) with TRAIL_DECAY
  - Background glitches: sparkles + tearing bands
  - “Explode into pixels” glitch events (particle bursts)

  NOTE: This pattern deliberately ignores render2D(x,y) coords and uses index->(around,height)
  to match your mapper’s zigzag wiring.
*/

/////////////////////
// CORE CONFIG
/////////////////////

//matrix size - 32x8 array wrapped around a cylinder
A            = 32         // around
W            = 8          // tall

//box variables
BOX_COUNT    = 4             // number of boxes
SIZE_MIN     = 1          // box size range (pixels)
SIZE_MAX     = 3  

// Motion / physics
TARGET_SPEED = 5.0        // pixels/sec, enforced (prevents slowing)
SOLVER_ITERS = 6          // collision solver passes
RESTITUTION  = 0.98       // bounciness (1 = perfectly elastic)
POS_SLOP     = 0.01       // overlap allowed before correction
POS_PERCENT  = 0.75       // overlap correction strength (0..1)

// Color / trails
HUE_RATE_SEC = 12         // seconds per full hue cycle (bigger = slower)
RADIAL_HUE_SPAN = 0.60    // how much hue shifts from center->edge
TRAIL_DECAY  = 0.7         // 0..1 (higher = longer trails)

// Background glitches
GLITCH_RATE  = 0.0       // sparkle chance per pixel per glitch frame
GLITCH_VAL   = 0.0       // sparkle brightness

TEAR_CHANCE  = 0.00           // tearing band active chance per tear frame
TEAR_MAX_SHIFT = 7        // max X shift during tearing
TEAR_FPS     = 10         // tear config changes per second
TEAR_VAL     = 0.0         // tearing band brightness

// Explode glitch (particle bursts)
EXPLODE_RATE = 0.15         // bursts per second (0.08 ~= one every ~12s avg)
PARTICLE_COUNT = 20       // particles in a burst
PARTICLE_TTL   = 0.75     // seconds
PARTICLE_VAL   = 0.65     // max brightness
PARTICLE_RADIUS = 2.5     // visual radius (pixels)

// Per-box mass range
MASS_MIN = 0.8
MASS_MAX = 2.2

/////////////////////
// INTERNAL STATE
/////////////////////

PI2 = 6.283185307179586

// boxes
ax = array(BOX_COUNT)   // top-left X (float, wraps)
hy = array(BOX_COUNT)   // top-left Y (float, bounces)
vx = array(BOX_COUNT)
vy = array(BOX_COUNT)
bs = array(BOX_COUNT)   // size in pixels (int)
m  = array(BOX_COUNT)   // mass
invM = array(BOX_COUNT) // 1/mass
bh = array(BOX_COUNT)   // base hue

// trails (allocated once we know pixelCount)
trailV = 0
trailH = 0

// particles
pxp = array(PARTICLE_COUNT)   // x
pyp = array(PARTICLE_COUNT)   // y
pvr = array(PARTICLE_COUNT)   // vx
pvt = array(PARTICLE_COUNT)   // vy
ptt = array(PARTICLE_COUNT)   // ttl remaining
phu = array(PARTICLE_COUNT)   // hue
pActive = 0

// tearing state
tearRow0 = 0; tearRow1 = 0; tearRow2 = 0
tearShift0 = 0; tearShift1 = 0; tearShift2 = 0
tearOn0 = 0; tearOn1 = 0; tearOn2 = 0
tearHue0 = 0; tearHue1 = 0; tearHue2 = 0
gFrame = 0

// init boxes once
inited = 0

/////////////////////
// HELPERS
/////////////////////

function wrap01(v) { return v - floor(v) }
function wrapA(v) { return (v % A + A) % A }

function clamp01(v) { return clamp(v, 0, 1) }

// signed shortest difference a-b on [0,A)
function wrapDxSigned(a, b) {
  d = a - b
  if (d >  A * 0.5) d -= A
  if (d < -A * 0.5) d += A
  return d
}

function hash01(n) {
  return wrap01(sin(n * 12.9898 + 78.233) * 43758.5453)
}

function randRange(a, b) { return a + (b - a) * random(1) }

function keepSpeed(i) {
  sp = sqrt(vx[i]*vx[i] + vy[i]*vy[i])
  if (sp > 0.0001) {
    s = TARGET_SPEED / sp
    vx[i] *= s
    vy[i] *= s
  }
}

function clampY(i) {
  maxH = W - bs[i]
  if (hy[i] < 0)    { hy[i] = -hy[i];         vy[i] = -vy[i] * RESTITUTION }
  if (hy[i] > maxH) { hy[i] = 2*maxH - hy[i]; vy[i] = -vy[i] * RESTITUTION }
}

function radialHue(px, py, i, baseHue) {
  // px already potentially wrapped/shifted in X; align it near the box
  dxw = wrapDxSigned(px, ax[i])
  pxx = ax[i] + dxw

  cx = ax[i] + bs[i]*0.5
  cy = hy[i] + bs[i]*0.5

  dx = pxx - cx
  dy = py - cy

  rMax = (bs[i] * 0.5) * 1.4142
  dist = sqrt(dx*dx + dy*dy) / rMax
  dist = clamp(dist, 0, 1)

  return wrap01(baseHue + dist * RADIAL_HUE_SPAN)
}

// Soft filled box (anti-aliased edges), wrapped X
function softBox(px, py, i) {
  // align px near the box in wrapped space
  dx = wrapDxSigned(px, ax[i])
  pxx = ax[i] + dx

  left   = pxx - ax[i]
  right  = (ax[i] + bs[i]) - pxx
  top    = py - hy[i]
  bottom = (hy[i] + bs[i]) - py

  d = min(min(left, right), min(top, bottom))
  soft = 0.8
  return clamp((d + soft) / soft, 0, 1)
}

// AABB collision with variable sizes, wrapped X, impulse + positional correction
function collide(i, j) {
  si = bs[i]; sj = bs[j]

  cxi = ax[i] + si*0.5
  cyi = hy[i] + si*0.5
  cxj = ax[j] + sj*0.5
  cyj = hy[j] + sj*0.5

  dx = wrapDxSigned(cxi, cxj)
  dy = cyi - cyj

  adx = abs(dx)
  ady = abs(dy)

  // overlap thresholds are half-extents sum
  hx = (si + sj) * 0.5
  hySum = (si + sj) * 0.5

  if (adx < hx && ady < hySum) {
    px = hx - adx
    py = hySum - ady

    // collision normal = minimum penetration axis
    if (px < py) { nx = (dx >= 0) ? 1 : -1; ny = 0; pen = px }
    else         { nx = 0; ny = (dy >= 0) ? 1 : -1; pen = py }

    // positional correction split by inverse mass
    invSum = invM[i] + invM[j]
    if (invSum < 0.0001) invSum = 1

    corr = POS_PERCENT * max(pen - POS_SLOP, 0)
    ci = corr * (invM[i] / invSum)
    cj = corr * (invM[j] / invSum)

    ax[i] += nx * ci
    hy[i] += ny * ci
    ax[j] -= nx * cj
    hy[j] -= ny * cj

    ax[i] = wrapA(ax[i])
    ax[j] = wrapA(ax[j])
    clampY(i)
    clampY(j)

    // relative velocity along normal
    rvx = vx[i] - vx[j]
    rvy = vy[i] - vy[j]
    velN = rvx*nx + rvy*ny

    // impulse (only if approaching)
    if (velN < 0) {
      jImp = -(1 + RESTITUTION) * velN / (invM[i] + invM[j])
      ix = jImp * nx
      iy = jImp * ny
      vx[i] += ix * invM[i]
      vy[i] += iy * invM[i]
      vx[j] -= ix * invM[j]
      vy[j] -= iy * invM[j]
    }
  }
}

/////////////////////
// PARTICLES (explode glitch)
/////////////////////

function spawnBurst() {
  // pick random center
  cx = floor(random(A))
  cy = floor(random(W))
  baseHue = random(1)

  for (p = 0; p < PARTICLE_COUNT; p++) {
    pxp[p] = cx + random(1) * 0.8
    pyp[p] = cy + random(1) * 0.8
    ang = random(1) * PI2
    sp  = randRange(6.0, 11.0)
    pvr[p] = cos(ang) * sp
    pvt[p] = sin(ang) * sp
    ptt[p] = PARTICLE_TTL * randRange(0.7, 1.0)
    phu[p] = wrap01(baseHue + random(1) * 0.25)
  }
  pActive = 1
}

function updateParticles(dt) {
  if (!pActive) return
  alive = 0

  for (p = 0; p < PARTICLE_COUNT; p++) {
    if (ptt[p] > 0) {
      ptt[p] -= dt
      if (ptt[p] > 0) {
        // integrate; wrap X, bounce Y
        pxp[p] = wrapA(pxp[p] + pvr[p] * dt)
        pyp[p] += pvt[p] * dt

        if (pyp[p] < 0) { pyp[p] = -pyp[p]; pvt[p] = -pvt[p] * 0.7 }
        if (pyp[p] > (W - 1)) { pyp[p] = 2*(W - 1) - pyp[p]; pvt[p] = -pvt[p] * 0.7 }

        // mild drag so it looks glitchy, not ballistic forever
        pvr[p] *= 0.985
        pvt[p] *= 0.985

        alive = 1
      }
    }
  }
  pActive = alive
}

function particleAt(px, py) {
  if (!pActive) { pVal = 0; pHue = 0; return }

  best = 0
  bestHue = 0

  for (p = 0; p < PARTICLE_COUNT; p++) {
    if (ptt[p] > 0) {
      dx = wrapDxSigned(px, pxp[p])
      dy = py - pyp[p]
      d2 = dx*dx + dy*dy
      // soft radial falloff
      r2 = PARTICLE_RADIUS * PARTICLE_RADIUS
      if (d2 < r2) {
        a = 1 - (d2 / r2)
        // fade with ttl
        fade = ptt[p] / PARTICLE_TTL
        v = a * fade
        if (v > best) { best = v; bestHue = phu[p] }
      }
    }
  }

  pVal = best * PARTICLE_VAL
  pHue = bestHue
}

/////////////////////
// INIT
/////////////////////

function initIfNeeded() {
  if (inited) return
  inited = 1

  // allocate trails once pixelCount known
  if (!trailV) {
    trailV = array(pixelCount)
    trailH = array(pixelCount)
    for (i = 0; i < pixelCount; i++) { trailV[i] = 0; trailH[i] = 0 }
  }

  // init boxes
  for (i = 0; i < BOX_COUNT; i++) {
    bs[i] = floor(randRange(SIZE_MIN, SIZE_MAX + 0.999))
    m[i] = randRange(MASS_MIN, MASS_MAX)
    invM[i] = 1 / m[i]

    // spread them around; keep within vertical range
    ax[i] = wrapA((A / BOX_COUNT) * i + random(1) * 0.5)
    hy[i] = randRange(0, (W - bs[i]) > 0 ? (W - bs[i]) : 0)

    ang = (i / BOX_COUNT) * PI2
    vx[i] = cos(ang) * TARGET_SPEED
    vy[i] = sin(ang) * TARGET_SPEED
    keepSpeed(i)

    bh[i] = i / BOX_COUNT
  }
}

/////////////////////
// BEFORE RENDER
/////////////////////

export function beforeRender(delta) {
  initIfNeeded()

  dt = min(delta / 1000, 0.05)

  // maybe trigger a burst
  if (random(1) < (EXPLODE_RATE * dt)) spawnBurst()
  updateParticles(dt)

  // integrate boxes
  for (i = 0; i < BOX_COUNT; i++) {
    ax[i] = wrapA(ax[i] + vx[i] * dt)
    hy[i] += vy[i] * dt
    clampY(i)
  }

  // collisions
  for (k = 0; k < SOLVER_ITERS; k++) {
    for (i = 0; i < BOX_COUNT; i++) {
      for (j = i + 1; j < BOX_COUNT; j++) {
        collide(i, j)
      }
    }
  }

  // enforce constant speed (prevents slowdown)
  for (i = 0; i < BOX_COUNT; i++) keepSpeed(i)

  // base hues
  t = time(HUE_RATE_SEC / 65.536)
  for (i = 0; i < BOX_COUNT; i++) {
    bh[i] = wrap01(t + i / BOX_COUNT)
  }

  // glitch + tear frames
  gFrame = floor(time(18 / 65.536) * 1000)
  tearFrame = floor(time((1 / TEAR_FPS) / 65.536) * 100000)

  // 3 tear bands
  r0 = hash01(tearFrame * 31.0 + 1.0)
  r1 = hash01(tearFrame * 37.0 + 2.0)
  r2 = hash01(tearFrame * 41.0 + 3.0)

  tearRow0 = floor(hash01(tearFrame * 53.0 + 4.0) * W)
  tearRow1 = floor(hash01(tearFrame * 59.0 + 5.0) * W)
  tearRow2 = floor(hash01(tearFrame * 61.0 + 6.0) * W)

  tearShift0 = floor((hash01(tearFrame * 67.0 + 7.0) * 2 - 1) * TEAR_MAX_SHIFT)
  tearShift1 = floor((hash01(tearFrame * 71.0 + 8.0) * 2 - 1) * TEAR_MAX_SHIFT)
  tearShift2 = floor((hash01(tearFrame * 73.0 + 9.0) * 2 - 1) * TEAR_MAX_SHIFT)

  tearOn0 = (r0 < TEAR_CHANCE) ? 1 : 0
  tearOn1 = (r1 < TEAR_CHANCE) ? 1 : 0
  tearOn2 = (r2 < TEAR_CHANCE) ? 1 : 0

  tearHue0 = hash01(tearFrame * 79.0 + 10.0)
  tearHue1 = hash01(tearFrame * 83.0 + 11.0)
  tearHue2 = hash01(tearFrame * 89.0 + 12.0)
}

/////////////////////
// RENDER
/////////////////////

export function render2D(index, x, y) {
  // mapper-derived coords (your zigzag):
  around = floor(index / W)     // 0..31
  h = index % W                 // 0..7
  if (around % 2 == 1) h = (W - 1) - h

  pxBase = around + 0.5
  py = h + 0.5

  // tearing shifts the sampling px (illusion)
  px = pxBase
  tearShift = 0
  tearHue = 0
  tearOn = 0

  if (tearOn0 && h == tearRow0) { tearShift = tearShift0; tearHue = tearHue0; tearOn = 1 }
  if (tearOn1 && h == tearRow1) { tearShift = tearShift1; tearHue = tearHue1; tearOn = 1 }
  if (tearOn2 && h == tearRow2) { tearShift = tearShift2; tearHue = tearHue2; tearOn = 1 }

  if (tearOn) px = wrapA(px + tearShift)

  // boxes (dominant hue, additive value)
  bestV = 0
  hue = 0
  total = 0

  for (i = 0; i < BOX_COUNT; i++) {
    v = softBox(px, py, i)
    total += v
    if (v > bestV) {
      bestV = v
      hue = radialHue(px, py, i, bh[i])
    }
  }

  // particles (explode bursts)
  particleAt(px, py) // sets pVal, pHue

  // background sparkles
  r = hash01(index + gFrame * 97.0)
  sparkle = (r < GLITCH_RATE) ? 1 : 0
  gAmt = sparkle ? GLITCH_VAL : 0
  gHue = hash01(index * 3.0 + gFrame * 19.0)

  // choose output before trails
  outV = 0
  outH = 0

  if (total > 0) {
    outH = hue
    outV = clamp(total + (tearOn ? TEAR_VAL * 0.15 : 0) + pVal * 0.65, 0, 1)
  } else if (pVal > 0) {
    outH = pHue
    outV = clamp(pVal, 0, 1)
  } else if (tearOn) {
    outH = tearHue
    outV = TEAR_VAL
  } else if (gAmt > 0) {
    outH = gHue
    outV = gAmt
  } else {
    outH = 0
    outV = 0
  }

  // TRAILS: max-hold with decay; keep hue of the brighter contributor
  prevV = trailV[index]
  holdV = prevV * TRAIL_DECAY

  if (outV >= holdV) {
    trailV[index] = outV
    trailH[index] = outH
    hsv(outH, 1, outV)
  } else {
    trailV[index] = holdV
    hsv(trailH[index], 1, holdV)
  }
}

export function render(index) {
  hsv(0, 0, 0)
}
