/*
  Sound - Cube Frequency Bands 3D

  Audio-reactive pattern for a 5-panel LED cube driven by a Pixelblaze
  with the SB 1.0 sensor expansion board.

  - Low-frequency beats spawn colored horizontal bands that start at the
    bottom panel and travel up the four side walls. Hue tracks the
    dominant bass FFT bin (bin 0 = red → bin 7 = yellow-orange).
  - High-frequency beats spawn vertical "curtain" bands that sweep
    around the cube longitudinally. Hue tracks the dominant treble bin
    (bin 20 = cyan → bin 31 = magenta).

  Motion speed follows a running BPM estimate from inter-beat intervals.

  Performance notes (matters at 1280 pixels):
  - atan2 / rxy fall-off are cached per pixel after the first frame.
  - Every band's color, 1/width, and angular trig is precomputed once
    per frame in beforeRender.
  - Sweeping bands use a cos(angle-diff) = dot-product test instead of
    a floor()-based angle wrap, so the inner loop is branch-light.
*/

// ---- UI sliders ----
// Pixelblaze creates UI sliders from exported `slider*` *functions*,
// not vars. Each function is invoked with the slider's current 0..1
// value whenever it moves.
var bassSensCtrl    = 0.6
var trebleSensCtrl  = 0.6
var bassWidthCtrl   = 0.4
var trebleWidthCtrl = 0.4
var brightnessCtrl  = 0.9
var testModeCtrl    = 0

export function sliderBassSensitivity(v)   { bassSensCtrl    = v }
export function sliderTrebleSensitivity(v) { trebleSensCtrl  = v }
export function sliderBassWidth(v)         { bassWidthCtrl   = v }
export function sliderTrebleWidth(v)       { trebleWidthCtrl = v }
export function sliderBrightness(v)        { brightnessCtrl  = v }
export function sliderTestMode(v)          { testModeCtrl    = v }

// ---- Sensor board inputs ----
export var light            = -1
export var frequencyData    = array(32)
export var energyAverage    = 0

// ---- Diagnostic exports ----
export var bassNow        = 0
export var trebleNow      = 0
export var bassAvg        = 0
export var trebleAvg      = 0
export var bassPeak       = 0
export var treblePeak     = 0
export var bassSpike      = 0
export var trebleSpike    = 0
export var bassBeatThresh = 0
export var detectedBpm    = 120

// ---- Band pool (reduced from 16 → 10 for perf) ----
maxBands   = 10
bandType   = array(maxBands)
bandHue    = array(maxBands)
bandAge    = array(maxBands)
bandLife   = array(maxBands)
bandPos    = array(maxBands)   // z for rising, theta for sweeping
bandVel    = array(maxBands)
bandWidth  = array(maxBands)
bandActive = array(maxBands)

// Per-band per-frame precomputes (set in beforeRender).
bandR           = array(maxBands)   // color × fade × brightness
bandG           = array(maxBands)
bandB           = array(maxBands)
bandInvWidth    = array(maxBands)
bandCos         = array(maxBands)
bandSin         = array(maxBands)
bandWidthCos    = array(maxBands)
bandInvCosDelta = array(maxBands)

for (i = 0; i < maxBands; i++) bandActive[i] = 0

// ---- Per-pixel cache. The map is fixed, so atan2 and the radial
// fall-off only need to be computed once per pixel for the lifetime of
// the pattern. We lazy-fill on the first call per index.
pxCos    = array(pixelCount)
pxSin    = array(pixelCount)
pxRxy    = array(pixelCount)
pxCached = array(pixelCount)
for (i = 0; i < pixelCount; i++) pxCached[i] = 0

// ---- Beat / timing state ----
lastBassBeat   = -10
lastTrebleBeat = -10
elapsed        = 0
beatPeriod     = 0.5
testAccum      = 0

function findFreeBand() {
  for (i = 0; i < maxBands; i++) {
    if (!bandActive[i]) return i
  }
  oldest = 0
  oldestAge = bandAge[0]
  for (i = 1; i < maxBands; i++) {
    if (bandAge[i] > oldestAge) { oldestAge = bandAge[i]; oldest = i }
  }
  return oldest
}

function spawnRising(hue, intensity) {
  i = findFreeBand()
  bandType[i]   = 0
  bandHue[i]    = hue
  bandAge[i]    = 0
  bandLife[i]   = beatPeriod * 2
  bandPos[i]    = 1.05
  bandVel[i]    = -1.2 / bandLife[i]
  bandWidth[i]  = 0.05 + bassWidthCtrl * 0.12 + clamp(intensity, 0, 1) * 0.06
  bandActive[i] = 1
}

function spawnSweeping(hue, intensity) {
  i = findFreeBand()
  bandType[i]   = 1
  bandHue[i]    = hue
  bandAge[i]    = 0
  bandLife[i]   = beatPeriod * 4
  bandPos[i]    = random(1) * 2 * PI
  bandVel[i]    = (random(1) > 0.5 ? 1 : -1) * 2 * PI / bandLife[i]
  bandWidth[i]  = 0.15 + trebleWidthCtrl * 0.45 + clamp(intensity, 0, 1) * 0.12
  bandActive[i] = 1
}

hr = 0; hg = 0; hb = 0
function h2rgb(h, v) {
  hh = (h - floor(h)) * 6
  hi = floor(hh)
  f  = hh - hi
  q  = v * (1 - f)
  t  = v * f
  if      (hi == 0) { hr = v; hg = t; hb = 0 }
  else if (hi == 1) { hr = q; hg = v; hb = 0 }
  else if (hi == 2) { hr = 0; hg = v; hb = t }
  else if (hi == 3) { hr = 0; hg = q; hb = v }
  else if (hi == 4) { hr = t; hg = 0; hb = v }
  else              { hr = v; hg = 0; hb = q }
}

// Simulated audio fallback for use without the SB.
simBpm     = 120
simMeasure = 4 * 60 / simBpm / 65.536

function simulateAudio() {
  tM = time(simMeasure)
  for (i = 0; i < 32; i++) frequencyData[i] = 0

  beat = (-4 * tM + 5) % 1
  beat *= 0.02 * pow(beat, 4)
  for (i = 0; i < 10; i++) frequencyData[i] += beat * (10 - i) / 10

  claps = 0.006 * square(2 * tM - 0.5, 0.10)
  for (i = 9; i < 18; i++) frequencyData[i] += claps * (0.7 + 0.6 * random(0.5))

  hh = 0.012 * square(4 * tM - 0.5, 0.05)
  for (i = 22; i < 30; i++) frequencyData[i] += hh * (0.8 + random(0.4))

  energyAverage = beat * 0.5 + claps * 0.5 + hh * 0.5
}

export function beforeRender(delta) {
  dt = delta * 0.001
  elapsed += dt

  if (light == -1) simulateAudio()

  // ---- Aggregate energy ----
  bassNow = 0
  for (i = 0; i < 8; i++) bassNow += frequencyData[i]
  bassNow *= 0.125

  trebleNow = 0
  for (i = 20; i < 32; i++) trebleNow += frequencyData[i]
  trebleNow *= 1 / 12

  // ---- Rolling baseline + slow-decay peak per band ----
  dw = delta / 2000
  bassAvg   = bassAvg   * (1 - dw) + bassNow   * dw
  trebleAvg = trebleAvg * (1 - dw) + trebleNow * dw

  bassPeak   = bassPeak   * 0.997
  treblePeak = treblePeak * 0.997
  if (bassNow   > bassPeak)   bassPeak   = bassNow
  if (trebleNow > treblePeak) treblePeak = trebleNow

  bassDyn   = max(max(bassPeak   - bassAvg,   bassPeak   * 0.2), 0.00005)
  trebleDyn = max(max(treblePeak - trebleAvg, treblePeak * 0.2), 0.00005)

  bassSpike   = max(0, bassNow   - bassAvg)   / bassDyn
  trebleSpike = max(0, trebleNow - trebleAvg) / trebleDyn

  bassBeatThresh   = 0.65 - bassSensCtrl   * 0.40
  trebleBeatThresh = 0.55 - trebleSensCtrl * 0.35

  minGap = 60 / 220

  if (bassSpike > bassBeatThresh && elapsed - lastBassBeat > minGap) {
    peakBin = 0
    peakVal = frequencyData[0]
    for (i = 1; i < 8; i++) {
      if (frequencyData[i] > peakVal) { peakVal = frequencyData[i]; peakBin = i }
    }
    hue = peakBin / 7 * 0.14
    spawnRising(hue, bassSpike)

    if (lastBassBeat > 0) {
      interval = elapsed - lastBassBeat
      if (interval > 0.27 && interval < 1.5) {
        newBpm = 60 / interval
        detectedBpm = detectedBpm * 0.6 + newBpm * 0.4
        beatPeriod = 60 / detectedBpm
      }
    }
    lastBassBeat = elapsed
  }

  if (trebleSpike > trebleBeatThresh &&
      elapsed - lastTrebleBeat > minGap * 0.5) {
    peakBin = 20
    peakVal = frequencyData[20]
    for (i = 21; i < 32; i++) {
      if (frequencyData[i] > peakVal) { peakVal = frequencyData[i]; peakBin = i }
    }
    hue = 0.5 + (peakBin - 20) / 11 * 0.35
    spawnSweeping(hue, trebleSpike)
    lastTrebleBeat = elapsed
  }

  if (testModeCtrl > 0) {
    testAccum += dt
    testInterval = 0.15 + (1 - testModeCtrl) * 1.0
    if (testAccum >= testInterval) {
      testAccum = 0
      if (random(1) < 0.6) {
        spawnRising(random(1) * 0.14, 0.5)
      } else {
        spawnSweeping(0.5 + random(1) * 0.35, 0.5)
      }
    }
  }

  // ---- Advance bands AND precompute their render-time constants ----
  for (i = 0; i < maxBands; i++) {
    if (bandActive[i]) {
      bandAge[i] += dt
      bandPos[i] += bandVel[i] * dt
      if (bandAge[i] >= bandLife[i]) {
        bandActive[i] = 0
        bandR[i] = 0; bandG[i] = 0; bandB[i] = 0
      } else {
        p = bandAge[i] / bandLife[i]
        // Hold full for first third of life, then linear taper.
        fade = clamp((1 - p) * 1.5, 0, 1) * brightnessCtrl
        h2rgb(bandHue[i], fade)
        bandR[i] = hr
        bandG[i] = hg
        bandB[i] = hb
        bandInvWidth[i] = 1 / bandWidth[i]
        if (bandType[i] == 1) {
          // Sweeping: precompute cos/sin of position and cos of width.
          bandCos[i] = cos(bandPos[i])
          bandSin[i] = sin(bandPos[i])
          wc = cos(bandWidth[i])
          bandWidthCos[i]    = wc
          bandInvCosDelta[i] = 1 / max(1 - wc, 0.0001)
        }
      }
    }
  }
}

export function render3D(index, x, y, z) {
  // ---- One-shot per-pixel cache ----
  if (!pxCached[index]) {
    cx = x - 0.5
    cy = y - 0.5
    rxy2 = cx * cx + cy * cy
    rxy  = sqrt(rxy2)
    if (rxy > 0.0001) {
      pxCos[index] = cx / rxy
      pxSin[index] = cy / rxy
    } else {
      pxCos[index] = 1
      pxSin[index] = 0
    }
    pxRxy[index]    = clamp(rxy2 * 6, 0, 1)
    pxCached[index] = 1
  }
  pCos = pxCos[index]
  pSin = pxSin[index]
  pRxy = pxRxy[index]

  r = 0; g = 0; b = 0

  for (i = 0; i < maxBands; i++) {
    if (bandActive[i]) {
      if (bandType[i] == 0) {
        // Rising band.
        dist = abs(z - bandPos[i])
        if (dist < bandWidth[i]) {
          bri = 1 - dist * bandInvWidth[i]
          r += bandR[i] * bri
          g += bandG[i] * bri
          b += bandB[i] * bri
        }
      } else {
        // Sweeping band: dot product = cos(angle-diff).
        dotp = pCos * bandCos[i] + pSin * bandSin[i]
        if (dotp > bandWidthCos[i]) {
          bri = (dotp - bandWidthCos[i]) * bandInvCosDelta[i] * pRxy
          r += bandR[i] * bri
          g += bandG[i] * bri
          b += bandB[i] * bri
        }
      }
    }
  }

  rgb(r, g, b)
}

export function render2D(index, x, y) {
  render3D(index, x, y, 0.5)
}

export function render(index) {
  render3D(index, index / pixelCount, 0.5, 0.5)
}
