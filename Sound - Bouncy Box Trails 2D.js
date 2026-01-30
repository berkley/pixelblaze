/*
  CYLINDER BOXES ENGINE (32 around x 8 tall, zigzag-wired height=8) + SOUND REACTIVE
  Change requested:
  - Bass *beats* trigger the tearing (instead of random chance)
  - Tear color is ROYGBIV-ish based on dominant frequency (spectrum peak)

  Sensor Expansion Board:
  - Uses frequencyData[32] + energyAverage
*/

/////////////////////
// CORE CONFIG
/////////////////////

A = 32
W = 8

// boxes
BOX_COUNT = 4
SIZE_MIN = 1
SIZE_MAX = 3

// Motion / physics
TARGET_SPEED = 5.0
SOLVER_ITERS = 6
RESTITUTION = 0.98
POS_SLOP = 0.01
POS_PERCENT = 0.75

// Color / trails
HUE_RATE_SEC = 12
RADIAL_HUE_SPAN = 0.60
TRAIL_DECAY = 0.70

// Sparkles (optional)
ENABLE_GLITCH = 0
GLITCH_RATE = 0.012
GLITCH_VAL  = 0.35

// Tearing bands (now beat-triggered)
ENABLE_TEAR = 1
TEAR_MAX_SHIFT = 7
TEAR_FPS = 30          // how often tear rows/shift can change while tearing
TEAR_VAL = 0.3         // brightness of tear bands
TEAR_STRENGTH = 0.8    // 0..1 how “on” the tear is during a beat pulse

// Beat -> tear pulse shaping
BEAT_ENABLE = 1
BEAT_SENS = 1.4          // higher = fewer triggers
BEAT_FLOOR = 0.015        // ignore bass under this
BEAT_MIN_GAP = 0.14    // seconds between beat triggers
TEAR_PULSE_SEC = 0.12  // tear visibility duration per beat

// Explode glitch (particles)
ENABLE_EXPLODE = 1
EXPLODE_RATE = 0.15
PARTICLE_COUNT = 20
PARTICLE_TTL = 0.75
PARTICLE_VAL = 0.65
PARTICLE_RADIUS = 2.5

// Per-box mass
MASS_MIN = 0.8
MASS_MAX = 2.2

ENABLE_TRAILS = 1
REINIT = 0

/////////////////////
// SOUND (Sensor Expansion Board)
/////////////////////

export var frequencyData = array(32)
export var energyAverage = 0
export var maxFrequencyMagnitude = 0
export var maxFrequency = 0

SOUND_ENABLE = 1
SOUND_GAIN = 5   
SOUND_FLOOR = 0.01 
SOUND_SMOOTH = 0.18

audio = 0
bass = 0
mids = 0
treble = 0

// sound-derived controls
SPEED_NOW = TARGET_SPEED
TRAIL_NOW = TRAIL_DECAY
EXPLODE_NOW = EXPLODE_RATE
BRIGHT_NOW = 1.0

/////////////////////
// INTERNAL STATE
/////////////////////

PI2 = 6.283185307179586

ax = array(BOX_COUNT)
hy = array(BOX_COUNT)
vx = array(BOX_COUNT)
vy = array(BOX_COUNT)
bs = array(BOX_COUNT)
m  = array(BOX_COUNT)
invM = array(BOX_COUNT)
bh = array(BOX_COUNT)

trailV = 0
trailH = 0

pxp = array(PARTICLE_COUNT)
pyp = array(PARTICLE_COUNT)
pvr = array(PARTICLE_COUNT)
pvt = array(PARTICLE_COUNT)
ptt = array(PARTICLE_COUNT)
phu = array(PARTICLE_COUNT)
pActive = 0

// tearing state
tearRow0 = 0; tearRow1 = 0; tearRow2 = 0
tearShift0 = 0; tearShift1 = 0; tearShift2 = 0
tearOn0 = 0; tearOn1 = 0; tearOn2 = 0
tearHue0 = 0; tearHue1 = 0; tearHue2 = 0
gFrame = 0
tearFrame = 0

// beat/tear pulse
bassAvg = 0
beatCooldown = 0
tearPulse = 0
tearHueBeat = 0

inited = 0

/////////////////////
// HELPERS
/////////////////////

function wrap01(v) { return v - floor(v) }
function wrapA(v) { return (v % A + A) % A }

function wrapDxSigned(a, b) {
  d = a - b
  if (d >  A * 0.5) d -= A
  if (d < -A * 0.5) d += A
  return d
}

function hash01(n) { return wrap01(sin(n * 12.9898 + 78.233) * 43758.5453) }
function randRange(a, b) { return a + (b - a) * random(1) }

// ROYGBIV-ish hue mapping (piecewise linear through 7 stops)
function royHue(t) {
  // t in 0..1
  t = clamp(t, 0, 1)
  // hue stops in HSV space (rough but works)
  // R(0.00) O(0.08) Y(0.16) G(0.33) B(0.58) I(0.66) V(0.78)
  if (t < 1/6) return 0.00 + (0.08 - 0.00) * (t * 6)
  if (t < 2/6) return 0.08 + (0.16 - 0.08) * ((t - 1/6) * 6)
  if (t < 3/6) return 0.16 + (0.33 - 0.16) * ((t - 2/6) * 6)
  if (t < 4/6) return 0.33 + (0.58 - 0.33) * ((t - 3/6) * 6)
  if (t < 5/6) return 0.58 + (0.66 - 0.58) * ((t - 4/6) * 6)
  return 0.66 + (0.78 - 0.66) * ((t - 5/6) * 6)
}

function spectrumPeakHue() {
  // find dominant bin (0..31)
  maxv = 0
  maxi = 0
  for (i = 0; i < 32; i++) {
    v = frequencyData[i]
    if (v > maxv) { maxv = v; maxi = i }
  }
  // map bin index to 0..1 then to ROYGBIV hue
  return royHue(maxi / 31)
}

// --- SOUND ---
function updateAudio() {
  if (!SOUND_ENABLE) { audio = 0; bass = 0; mids = 0; treble = 0; return }

  lvl = energyAverage * SOUND_GAIN
  lvl = max(0, lvl - SOUND_FLOOR)
  lvl = clamp(lvl, 0, 1)

  b = 0; m2 = 0; t2 = 0
  for (i = 0; i < 4; i++) b += frequencyData[i]
  for (i = 4; i < 14; i++) m2 += frequencyData[i]
  for (i = 14; i < 32; i++) t2 += frequencyData[i]

  b = clamp(b / 4, 0, 1)
  m2 = clamp(m2 / 10, 0, 1)
  t2 = clamp(t2 / 18, 0, 1)

  audio  = audio  + (lvl - audio)  * SOUND_SMOOTH
  bass   = bass   + (b   - bass)   * SOUND_SMOOTH
  mids   = mids   + (m2  - mids)   * SOUND_SMOOTH
  treble = treble + (t2  - treble) * SOUND_SMOOTH
}

function applySoundToControls() {
  speedMul = 0.65 + audio * 1.6
  SPEED_NOW = TARGET_SPEED * speedMul

  TRAIL_NOW = clamp(TRAIL_DECAY + mids * 0.20, 0.35, 0.93)

  EXPLODE_NOW = EXPLODE_RATE * (0.35 + bass * 2.5)

  BRIGHT_NOW = 0.45 + audio * 0.75
}

// --- beat detection -> tearPulse ---
function updateBeatAndTear(dt) {
  if (!BEAT_ENABLE || !SOUND_ENABLE || !ENABLE_TEAR) {
    tearPulse = max(0, tearPulse - dt / TEAR_PULSE_SEC)
    beatCooldown = max(0, beatCooldown - dt)
    return
  }

  // instantaneous bass (use unsmoothed-ish from bins)
  bi = 0
  for (i = 0; i < 4; i++) bi += frequencyData[i]
  bi = clamp(bi / 4, 0, 1)

  // slow average bass
  bassAvg = bassAvg + (bi - bassAvg) * 0.05

  beatCooldown = max(0, beatCooldown - dt)

  // beat condition: bass spike above avg by sensitivity, with floor + cooldown
  if (beatCooldown <= 0 && bi > BEAT_FLOOR && bi > bassAvg * BEAT_SENS) {
    beatCooldown = BEAT_MIN_GAP
    tearPulse = 1.0
    tearHueBeat = spectrumPeakHue()
  } else {
    // decay tearPulse
    tearPulse = max(0, tearPulse - dt / TEAR_PULSE_SEC)
  }

  // while tearing, keep hue tracking peak a bit (feels more “frequency-colored”)
  if (tearPulse > 0.05) {
    tearHueBeat = spectrumPeakHue()
  }
}

// --- motion ---
function keepSpeed(i) {
  sp = sqrt(vx[i]*vx[i] + vy[i]*vy[i])
  if (sp > 0.0001) {
    s = SPEED_NOW / sp
    vx[i] *= s
    vy[i] *= s
  }
}

function clampY(i) {
  maxH = W - bs[i]
  if (hy[i] < 0)    { hy[i] = -hy[i];         vy[i] = -vy[i] * RESTITUTION }
  if (hy[i] > maxH) { hy[i] = 2*maxH - hy[i]; vy[i] = -vy[i] * RESTITUTION }
}

// --- color ---
function radialHue(px, py, i, baseHue) {
  dxw = wrapDxSigned(px, ax[i])
  pxx = ax[i] + dxw

  cx = ax[i] + bs[i]*0.5
  cy = hy[i] + bs[i]*0.5

  dx = pxx - cx
  dy = py - cy

  rMax = max((bs[i] * 0.5) * 1.4142, 0.75)
  dist = sqrt(dx*dx + dy*dy) / rMax
  dist = clamp(dist, 0, 1)

  return wrap01(baseHue + dist * RADIAL_HUE_SPAN)
}

function softBox(px, py, i) {
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

  hx = (si + sj) * 0.5
  hySum = (si + sj) * 0.5

  if (adx < hx && ady < hySum) {
    px = hx - adx
    py = hySum - ady

    if (px < py) { nx = (dx >= 0) ? 1 : -1; ny = 0; pen = px }
    else         { nx = 0; ny = (dy >= 0) ? 1 : -1; pen = py }

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

    rvx = vx[i] - vx[j]
    rvy = vy[i] - vy[j]
    velN = rvx*nx + rvy*ny

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
// PARTICLES
/////////////////////

function spawnBurst() {
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
        pxp[p] = wrapA(pxp[p] + pvr[p] * dt)
        pyp[p] += pvt[p] * dt

        if (pyp[p] < 0) { pyp[p] = -pyp[p]; pvt[p] = -pvt[p] * 0.7 }
        if (pyp[p] > (W - 1)) { pyp[p] = 2*(W - 1) - pyp[p]; pvt[p] = -pvt[p] * 0.7 }

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
      r2 = PARTICLE_RADIUS * PARTICLE_RADIUS
      if (d2 < r2) {
        a = 1 - (d2 / r2)
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

  if (!trailV) {
    trailV = array(pixelCount)
    trailH = array(pixelCount)
    for (i = 0; i < pixelCount; i++) { trailV[i] = 0; trailH[i] = 0 }
  }

  for (i = 0; i < BOX_COUNT; i++) {
    bs[i] = floor(randRange(SIZE_MIN, SIZE_MAX + 0.999))
    m[i] = randRange(MASS_MIN, MASS_MAX)
    invM[i] = 1 / m[i]

    ax[i] = wrapA((A / BOX_COUNT) * i + random(1) * 0.5)
    hy[i] = randRange(0, (W - bs[i]) > 0 ? (W - bs[i]) : 0)

    ang = (i / BOX_COUNT) * PI2
    vx[i] = cos(ang) * TARGET_SPEED
    vy[i] = sin(ang) * TARGET_SPEED

    bh[i] = i / BOX_COUNT
  }
}

/////////////////////
// BEFORE RENDER
/////////////////////

export function beforeRender(delta) {
  if (REINIT) { inited = 0; REINIT = 0 }
  initIfNeeded()

  dt = min(delta / 1000, 0.05)

  updateAudio()
  applySoundToControls()
  updateBeatAndTear(dt)

  // particles (sound reactive rate)
  if (ENABLE_EXPLODE) {
    if (random(1) < (EXPLODE_NOW * dt)) spawnBurst()
    updateParticles(dt)
  } else {
    pActive = 0
  }

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

  // enforce speed (sound reactive)
  for (i = 0; i < BOX_COUNT; i++) keepSpeed(i)

  // base hues (still move slowly)
  t = time(HUE_RATE_SEC / 65.536)
  for (i = 0; i < BOX_COUNT; i++) bh[i] = wrap01(t + i / BOX_COUNT)

  // frames for glitch + tear selection
  gFrame = floor(time(18 / 65.536) * 1000)
  tearFrame = floor(time((1 / TEAR_FPS) / 65.536) * 100000)

  // Beat-driven tear enable (0..1 pulse) -> band activation probability
  tearProb = clamp(tearPulse * TEAR_STRENGTH, 0, 1)

  // 3 tear bands pick rows/shift; ON only when beat pulse is active
  r0 = hash01(tearFrame * 31.0 + 1.0)
  r1 = hash01(tearFrame * 37.0 + 2.0)
  r2 = hash01(tearFrame * 41.0 + 3.0)

  tearRow0 = floor(hash01(tearFrame * 53.0 + 4.0) * W)
  tearRow1 = floor(hash01(tearFrame * 59.0 + 5.0) * W)
  tearRow2 = floor(hash01(tearFrame * 61.0 + 6.0) * W)

  tearShift0 = floor((hash01(tearFrame * 67.0 + 7.0) * 2 - 1) * TEAR_MAX_SHIFT)
  tearShift1 = floor((hash01(tearFrame * 71.0 + 8.0) * 2 - 1) * TEAR_MAX_SHIFT)
  tearShift2 = floor((hash01(tearFrame * 73.0 + 9.0) * 2 - 1) * TEAR_MAX_SHIFT)

  tearOn0 = (r0 < tearProb) ? 1 : 0
  tearOn1 = (r1 < tearProb) ? 1 : 0
  tearOn2 = (r2 < tearProb) ? 1 : 0

  // Tear color from dominant frequency (ROYGBIV)
  tearHue0 = tearHueBeat
  tearHue1 = tearHueBeat
  tearHue2 = tearHueBeat
}

/////////////////////
// RENDER
/////////////////////

export function render2D(index, x, y) {
  around = floor(index / W)
  h = index % W
  if (around % 2 == 1) h = (W - 1) - h

  pxBase = around + 0.5
  py = h + 0.5

  // tearing shifts sample px (illusion)
  px = pxBase
  tearShift = 0
  tearHue = 0
  tearOn = 0

  if (ENABLE_TEAR) {
    if (tearOn0 && h == tearRow0) { tearShift = tearShift0; tearHue = tearHue0; tearOn = 1 }
    if (tearOn1 && h == tearRow1) { tearShift = tearShift1; tearHue = tearHue1; tearOn = 1 }
    if (tearOn2 && h == tearRow2) { tearShift = tearShift2; tearHue = tearHue2; tearOn = 1 }
    if (tearOn) px = wrapA(px + tearShift)
  }

  // boxes
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

  // particles
  particleAt(px, py) // sets pVal, pHue

  // optional sparkles
  gAmt = 0; gHue = 0
  if (ENABLE_GLITCH) {
    r = hash01(index + gFrame * 97.0)
    sparkle = (r < GLITCH_RATE) ? 1 : 0
    gAmt = sparkle ? GLITCH_VAL : 0
    gHue = hash01(index * 3.0 + gFrame * 19.0)
  }

  // compose
  outV = 0
  outH = 0

  if (total > 0) {
    outH = hue
    outV = clamp(total + (tearOn ? TEAR_VAL * 0.20 : 0) + pVal * 0.65, 0, 1)
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

  // sound brightness pump
  outV = clamp(outV * BRIGHT_NOW, 0, 1)

  if (!ENABLE_TRAILS) {
    hsv(outH, 1, outV)
    return
  }

  // TRAILS: max-hold with decay
  prevV = trailV[index]
  holdV = prevV * TRAIL_NOW

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
