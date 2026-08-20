/*
  Sound - Coronal Mass Ejection

  Sound-reactive version of "Coronal Mass Ejection" by ZRanger1, from the
  ElectroMage pattern library (21 votes, https://patterns.electromage.com).
  Saved alongside anything already on the device rather than replacing it.

  The original converts each pixel to radial coordinates, samples a
  perlinTurbulence field in that space, and pushes it through smoothstep to turn
  the noise into discrete flares radiating from a white-hot core. Here the core
  brightness, flare reach and how fast the field churns all follow the music,
  and each kick throws an ejection: a surge that drives flares outward and
  decays over the following beat.

  TWO CHANGES WERE FORCED BY THIS HARDWARE.

  First, the radial coordinates are cached. The original calls atan2() and
  hypot() on every pixel of every frame; at 2560 pixels on a controller that
  manages 10-20 fps that is pure waste, because a pixel's position never
  changes. Both are computed once during the first render pass and read from
  arrays after.

  Second, this is a 160x16 bar, not the square display the pattern was written
  for. Pixelblaze normalises each axis independently, so the circular field
  becomes an extreme ellipse and the "star" is smeared the full length of the
  bar. That turns out to suit it: instead of a sun with flares, you get an
  ejection erupting from the middle of the bar and firing along it. The radial
  maths is left alone -- the aspect ratio does the restaging by itself.

  What remains expensive is one perlinTurbulence call per pixel, which cannot be
  cached because the field moves. That is the framerate cost of this pattern and
  it is inherent to the effect.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

var churnCtrl = 0.5         // how fast the noise field boils
export function sliderChurn(v) { churnCtrl = v }

var coreCtrl = 0.5          // size of the white-hot core
export function sliderCoreSize(v) { coreCtrl = v }

var ejectCtrl = 0.6         // how violently a kick throws an ejection
export function sliderEjection(v) { ejectCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var energyNorm  = 0
export var ejection    = 0

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0
totAvg       = 0
totPeak      = 0

// Cached radial coordinates -- the pattern's single biggest saving here.
pxAngle = array(pixelCount)
pxDist  = array(pixelCount)

frames = 0
mapped = 0

t1 = 0
noiseTime = 0
noiseYTime = 0
coreSize = 0.1
c2 = 0.025

setPerlinWrap(3, 256, 256)

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  frames++
  if (frames >= 2) mapped = 1

  dw = delta / 2000

  // ---- Sustained loudness ----
  totalNow = 0
  for (i = 0; i < 32; i++) totalNow += frequencyData[i]
  totalNow /= 32
  totAvg = totAvg * (1 - dw) + totalNow * dw
  totPeak = totPeak * 0.997
  if (totalNow > totPeak) totPeak = totalNow
  energyNorm = min(totalNow / max(totPeak, 0.00005), 1)

  // ---- Bass beat detection (from Sound - Fast Pulse 3D) ----
  bassNow = 0
  for (i = 0; i < 4; i++) bassNow += frequencyData[i]
  bassNow *= 0.25

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
  if (onset) { beatPhase = 0; beatCount++; ejection = min(bassSpike, 1) }
  else if (beatPhase >= 1) { beatPhase -= 1; beatCount++; ejection = min(bassSpike, 1) }

  // Ejection decays over roughly one beat, so each kick gets its own eruption.
  ejection -= dt / (60 / detectedBpm)
  if (ejection < 0) ejection = 0

  // ---- Field animation ----
  // The original drove these from free-running time(); here they churn faster
  // when the music is loud and surge on the ejection.
  churn = (0.25 + churnCtrl * 1.5) * (0.3 + energyNorm * 1.2 + ejection * 1.5)
  t1 += dt * 0.05
  if (t1 >= 1) t1 -= 1
  noiseTime += dt * churn * 2.6
  if (noiseTime >= 256) noiseTime -= 256
  noiseYTime += dt * churn * 3.2
  if (noiseYTime >= 256) noiseYTime -= 256

  // Core swells on the beat.
  coreSize = (0.05 + coreCtrl * 0.16) * (1 + ejection * ejectCtrl * 2.2)
  c2 = coreSize / 4
}

export function render3D(index, x, y, z) {
  if (!mapped) {
    // Cache the radial coordinates once. The original recomputed atan2/hypot on
    // every pixel every frame; a pixel's position never changes, so that is
    // 5120 trig calls a frame thrown away.
    cx = x - 0.5
    cy = z - 0.5
    pxDist[index] = hypot(cx, cy)
    pxAngle[index] = atan2(cy, cx)
    hsv(0, 0, 0)
  } else {
    ang = pxAngle[index]
    d = pxDist[index]

    // The original's field, unchanged.
    v = 1 - perlinTurbulence(ang, d - noiseYTime, noiseTime, 1.5, .25, 3)
    v = max(smoothstep(0.675, 1, v), (1 - ((d * v) - c2) / coreSize))
    v = v * v * v

    hsv(t1 - (0.125 * v), 6.5 * d - v, v)
  }
}

export function render2D(index, x, y) {
  // On the flattened map y is height across the unrolled banner.
  render3D(index, x, 0, y)
}

export function render(index) {
  // No map: no radial space to work in, so fall back to a simple pulsing glow.
  v = (1 - abs(index / pixelCount - 0.5) * 2)
  v = v * v * (0.2 + ejection)
  hsv(t1, 1 - v, v)
}
