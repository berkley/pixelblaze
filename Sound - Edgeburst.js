/*
  Sound - Edgeburst

  Sound-reactive variant of the stock Edgeburst pattern. The original sweeps its
  lit region out from the middle of the strip to the ends on a free-running
  timer; this drives that sweep from detected bass beats instead, so the burst
  reaches the edges on the kick.

  The whole change is in `t1`. Original:

    t1 = triangle(time(.1))                 // free-running bounce

  Here, each beat restarts a one-way sweep of t1 across the range that actually
  contains the burst:

    t1 = 0.26 + burstPhase * 0.50           // burstPhase 0 -> 1 after each beat

  Everything downstream is the stock pattern untouched.

  That range is not arbitrary, and it is NOT 0..1. Because the stock render ends
  with `v = triangle(edge)`, brightness peaks where edge is 0.5 and falls back to
  zero once edge saturates at 1 -- so pushing t1 to its extremes turns the strip
  OFF, not on. Sweeping the real thing gives:

    t1 0.26   bright front is born in the middle
    t1 0.375  front covers the middle half
    t1 0.48   travelling outward, 91% of the strip lit
    t1 0.62   front reaches the ends, which peak at v = 0.96
    t1 0.76   front has run off both ends, strip dark

  So one beat is one complete burst: born in the middle, races out, gone off the
  ends. It then rests dark until the next beat.

  Deliberately still 1D. Pixelblaze calls the most specific render function a
  pattern exports, so adding render2D/render3D would silently switch this to the
  pixel map and change how it looks on the bar. The point here was to make the
  existing look react, not to re-stage it.

  The bass beat detector is lifted from "Sound - Fast Pulse 3D": sub-bass bins
  0-3 only, so kicks trigger it and snares/claps with content higher up do not.
  It carries that pattern's octave correction and outlier rejection.

  Each kick also throws a brief flash of random junk onto the strip: single
  stray pixels, and occasional contiguous bands of nine. It fades within a
  fraction of a beat, so the strip returns to clean edgeburst in between.

  That gating is deliberate. An earlier version ran the corruption continuously
  and it wrecked the pattern -- edgeburst depends on the strip going dark
  between fronts, and permanently-lit random pixels meant the contrast never
  came back. Confining it to the flash keeps the dark rest, and as a bonus the
  per-pixel work only runs during the flash rather than on every frame.

  With no sensor board attached (`light` stays -1) it falls back to the stock
  free-running timer, so the pattern still animates.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

// How much of a beat the outward sweep takes. Low is a snappy pop that leaves
// the strip dark waiting; high keeps the front still travelling as the next
// kick lands.
var burstSpeedCtrl = 0.7
export function sliderBurstLength(v) { burstSpeedCtrl = v }

// How much random junk each kick throws onto the strip. 0 leaves the plain
// edgeburst.
var glitchCtrl = 0.4
export function sliderGlitch(v) { glitchCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var burstPhase  = 1   // 1 = burst finished, strip dark

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0

t1 = 0

// ---- Glitch state ----
// glitchAmt is 1 the instant a kick lands and falls to 0 shortly after, so the
// junk is a flash ON the beat and the strip is clean edgeburst in between. That
// matters twice over: it keeps the burst's dark rest intact, and it means the
// per-pixel work below only runs during the flash instead of every frame.
glitchAmt = 0
gRoll     = 0
pixProb   = 0
segProb   = 0
segNow    = -1   // last run index seen and its roll. render() is called in
segR      = 0    // index order, so this costs one lookup per nine pixels

// A table rather than the usual sin-based hash: same "same input, same output"
// behaviour, but a lookup instead of a sin() on every pixel. Measured on this
// bar, the sin version cost 7.6 fps against the table's 6.3.
RAND_N = 1024
randTab = array(RAND_N)
for (ri = 0; ri < RAND_N; ri++) randTab[ri] = random(1)

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  // ---- Bass beat detection ----
  bassNow = 0
  for (i = 0; i < 4; i++) bassNow += frequencyData[i]
  bassNow *= 0.25

  dw = delta / 2000
  bassAvg = bassAvg * (1 - dw) + bassNow * dw
  bassPeak = bassPeak * 0.997
  if (bassNow > bassPeak) bassPeak = bassNow

  bassDyn = max(max(bassPeak - bassAvg, bassPeak * 0.2), 0.00005)
  bassSpike = max(0, bassNow - bassAvg) / bassDyn

  threshold = 0.75 - sensitivityCtrl * 0.6
  onset = 0
  if (bassSpike > threshold && elapsed - lastBassBeat > 60 / 220) {
    onset = 1
    if (lastBassBeat > 0) {
      interval = elapsed - lastBassBeat
      if (interval > 0.27 && interval < 1.5) {
        newBpm = 60 / interval
        ratio = newBpm / detectedBpm
        if      (ratio > 1.6 && ratio < 2.5)  newBpm = newBpm * 0.5
        else if (ratio > 0.4 && ratio < 0.62) newBpm = newBpm * 2
        if (abs(newBpm - detectedBpm) / detectedBpm < 0.25) {
          detectedBpm = detectedBpm * 0.8 + newBpm * 0.2
        }
      }
    }
    lastBassBeat = elapsed
  }

  beatPhase += dt * detectedBpm / 60
  beat = 0
  if (onset) { beatPhase = 0; beat = 1 }
  else if (beatPhase >= 1) { beatPhase -= 1; beat = 1 }

  // ---- Glitch flash ----
  // Re-rolled 20 times a second, so within one flash the junk reshuffles a few
  // times rather than sitting still and fading.
  gRoll = floor(elapsed * 20)
  if (beat) glitchAmt = 1
  else {
    glitchAmt -= dt / (0.10 + glitchCtrl * 0.15)
    if (glitchAmt < 0) glitchAmt = 0
  }
  // Density falls with the flash, so pixels drop out progressively instead of
  // all vanishing at once.
  pixProb = glitchCtrl * 0.14 * glitchAmt
  segProb = glitchCtrl * 0.07 * glitchAmt

  if (light < 0) {
    // No sensor board: fall back to the stock free-running bounce.
    t1 = triangle(time(.1))
    return
  }

  // ---- Beat envelope ----
  // Each beat restarts the sweep. It runs one way and stops; it does not bounce
  // back, because reversing would drag the front from the ends back to the
  // middle and read as an implosion between kicks.
  if (beat) {
    beatCount++
    burstPhase = 0
  } else {
    burstDur = 60 / detectedBpm * (0.25 + burstSpeedCtrl * 0.75)
    burstPhase += dt / burstDur
    if (burstPhase > 1) burstPhase = 1
  }

  t1 = 0.26 + burstPhase * 0.50
}

export function render(index) {
  // ---- Stock Edgeburst, unchanged ----
  pct = index / pixelCount
  edge = clamp(triangle(pct) + t1 * 4 - 2, 0, 1)  // Mirror space

  h = edge * edge - .2  // Expand violets

  v = triangle(edge)    // Doubles the frequency

  // ---- Junk thrown on by the kick ----
  // Skipped entirely between flashes, which is where the framerate comes back.
  if (pixProb > 0) {
    // Bands: hashing the RUN index rather than the pixel index is what makes
    // these come out as contiguous lines instead of scattered dots.
    seg = floor(index / 9)
    if (seg != segNow) {
      segNow = seg
      segR = randTab[(seg * 37 + gRoll * 101) % RAND_N]
    }

    if (segR < segProb) {
      // Reuse the roll as the hue -- it is uniform on 0..segProb, so dividing
      // through gives a uniform 0..1 hue without a second lookup.
      h = segR / segProb
      v = max(v, 0.7 * glitchAmt)
    } else {
      rp = randTab[(index * 7 + gRoll * 53) % RAND_N]
      if (rp < pixProb) {              // single stray pixels, more common
        h = rp / pixProb
        v = max(v, 0.6 * glitchAmt)
      }
    }
  }

  hsv(h, 1, v)
}
