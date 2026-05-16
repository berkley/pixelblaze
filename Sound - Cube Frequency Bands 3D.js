/*
  Sound - Cube Frequency Bands 3D

  Audio-reactive pattern for a 5-panel LED cube driven by a Pixelblaze
  with the SB 1.0 sensor expansion board.

  - Low-frequency beats spawn colored horizontal bands that start at the
    bottom panel and travel up the four side walls. Hue is set by the
    dominant bass FFT bin (bin 0 = pure red, working toward yellow).
  - High-frequency beats spawn vertical "curtain" bands that sweep around
    the cube longitudinally. Hue is set by the dominant treble bin (low
    end = cyan, top end = magenta).

  Motion speed for both band types is locked to a running BPM estimate
  derived from the gap between detected bass beats.

  The SB 1.0 emits tiny FFT magnitudes (often well under 0.01 even on
  loud music). Rather than fight that with a fixed gain, we track each
  band's recent rolling average AND recent peak, and report each beat as
  a fraction of that dynamic range. So `bassSpike` is dimensionless and
  in roughly 0..1: 0 means current ≈ recent average, 1 means current ≈
  recent peak. Thresholds work across any signal scale.
*/

export var sliderBassSensitivity   = 0.6   // higher = more bass bands
export var sliderTrebleSensitivity = 0.6   // higher = more treble bands
export var sliderBassWidth         = 0.4
export var sliderTrebleWidth       = 0.4
export var sliderBrightness        = 0.9
export var sliderTestMode          = 0     // > 0: spawn bands on a timer (no audio)

// ---- Sensor board inputs ----
// Set by the SB firmware. If `light` stays at -1 we know no board is
// attached and fall back to a simulated audio signal.
export var light            = -1
export var frequencyData    = array(32)
export var energyAverage    = 0

// ---- Diagnostic exports (visible in Vars Watch for tuning) ----
export var bassNow        = 0
export var trebleNow      = 0
export var bassAvg        = 0
export var trebleAvg      = 0
export var bassPeak       = 0
export var treblePeak     = 0
export var bassSpike      = 0   // 0..~1, normalized to recent dynamic range
export var trebleSpike    = 0
export var bassBeatThresh = 0
export var detectedBpm    = 120

// ---- Band pool ----
maxBands   = 16
bandType   = array(maxBands)
bandHue    = array(maxBands)
bandAge    = array(maxBands)
bandLife   = array(maxBands)
bandPos    = array(maxBands)
bandVel    = array(maxBands)
bandWidth  = array(maxBands)
bandActive = array(maxBands)

for (i = 0; i < maxBands; i++) bandActive[i] = 0

// ---- Beat / timing state ----
lastBassBeat   = -10
lastTrebleBeat = -10
elapsed        = 0
beatPeriod     = 0.5

hasRising   = 0
hasSweeping = 0
testAccum   = 0

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
  bandWidth[i]  = 0.05 + sliderBassWidth * 0.12 + clamp(intensity, 0, 1) * 0.06
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
  bandWidth[i]  = 0.15 + sliderTrebleWidth * 0.45 + clamp(intensity, 0, 1) * 0.12
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

// Simulated audio if no SB is attached.
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

  // ---- Rolling baselines ----
  // Slow average over ~2s gives a "calm" reference. Beats sit above it.
  dw = delta / 2000
  bassAvg   = bassAvg   * (1 - dw) + bassNow   * dw
  trebleAvg = trebleAvg * (1 - dw) + trebleNow * dw

  // ---- Recent peaks (slow decay, jump to match new highs) ----
  // Tracks the loudest energy in the last ~1.5s in each band. Together
  // with the running average this defines the local dynamic range.
  bassPeak   = bassPeak   * 0.997
  treblePeak = treblePeak * 0.997
  if (bassNow   > bassPeak)   bassPeak   = bassNow
  if (trebleNow > treblePeak) treblePeak = trebleNow

  // ---- Normalized spike: where in [avg .. peak] is `now`? ----
  // Floors on the denominator keep things sane during startup / true
  // silence (when peak ≈ avg ≈ 0).
  bassDyn   = max(max(bassPeak   - bassAvg,   bassPeak   * 0.2), 0.00005)
  trebleDyn = max(max(treblePeak - trebleAvg, treblePeak * 0.2), 0.00005)

  bassSpike   = max(0, bassNow   - bassAvg)   / bassDyn
  trebleSpike = max(0, trebleNow - trebleAvg) / trebleDyn

  // ---- Thresholds: fraction of dynamic range required to trigger ----
  // Sensitivity slider at 0.5 sits in the middle of the band.
  bassBeatThresh   = 0.65 - sliderBassSensitivity   * 0.40   // 0.25..0.65
  trebleBeatThresh = 0.55 - sliderTrebleSensitivity * 0.35   // 0.20..0.55

  minGap = 60 / 220

  // ---- Bass beat ----
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

  // ---- Treble beat ----
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

  // ---- Test mode (audio-independent diagnostic) ----
  if (sliderTestMode > 0) {
    testAccum += dt
    testInterval = 0.15 + (1 - sliderTestMode) * 1.0
    if (testAccum >= testInterval) {
      testAccum = 0
      if (random(1) < 0.6) {
        spawnRising(random(1) * 0.14, 0.5)
      } else {
        spawnSweeping(0.5 + random(1) * 0.35, 0.5)
      }
    }
  }

  // ---- Advance bands ----
  hasRising   = 0
  hasSweeping = 0
  for (i = 0; i < maxBands; i++) {
    if (bandActive[i]) {
      bandAge[i] += dt
      bandPos[i] += bandVel[i] * dt
      if (bandAge[i] >= bandLife[i]) {
        bandActive[i] = 0
      } else {
        if (bandType[i] == 0) hasRising   = 1
        else                  hasSweeping = 1
      }
    }
  }
}

export function render3D(index, x, y, z) {
  r = 0; g = 0; b = 0

  pxAng = 0
  rxyWeight = 0
  if (hasSweeping) {
    cx = x - 0.5
    cy = y - 0.5
    pxAng = atan2(cy, cx)
    rxy2 = cx * cx + cy * cy
    rxyWeight = clamp(rxy2 * 6, 0, 1)
  }

  for (i = 0; i < maxBands; i++) {
    if (bandActive[i]) {
      p = bandAge[i] / bandLife[i]
      fade = clamp((1 - p) * 1.5, 0, 1)

      if (bandType[i] == 0) {
        dist = abs(z - bandPos[i])
        if (dist < bandWidth[i]) {
          bri = (1 - dist / bandWidth[i]) * fade
          h2rgb(bandHue[i], bri)
          r += hr; g += hg; b += hb
        }
      } else {
        ad = pxAng - bandPos[i]
        ad = ad - 2 * PI * floor((ad + PI) / (2 * PI))
        ad = abs(ad)
        if (ad < bandWidth[i]) {
          bri = (1 - ad / bandWidth[i]) * fade * rxyWeight
          h2rgb(bandHue[i], bri)
          r += hr; g += hg; b += hb
        }
      }
    }
  }

  rgb(r * sliderBrightness, g * sliderBrightness, b * sliderBrightness)
}

export function render2D(index, x, y) {
  render3D(index, x, y, 0.5)
}

export function render(index) {
  render3D(index, index / pixelCount, 0.5, 0.5)
}
