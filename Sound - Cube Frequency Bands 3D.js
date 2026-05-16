/*
  Sound - Cube Frequency Bands 3D

  Audio-reactive pattern for a 5-panel LED cube driven by a Pixelblaze
  with the SB 1.0 sensor expansion board.

  - Low-frequency beats (bass) spawn colored horizontal bands that start
    at the bottom panel and travel up the four side walls. Hue is set by
    the dominant bass FFT bin (bin 0 = pure red, working toward yellow).
  - High-frequency beats (treble) spawn vertical "curtain" bands that
    sweep around the cube longitudinally. Hue is set by the dominant
    treble bin (low end = cyan, top end = magenta).

  Motion speed for both band types is locked to a running BPM estimate
  derived from the gap between detected bass beats, so the animation
  scales with the cadence of the music.

  Requires the SB sensor board so frequencyData[] is populated.
*/

export var sliderBassSensitivity   = 0.5
export var sliderTrebleSensitivity = 0.5
export var sliderBassWidth         = 0.4   // thickness of rising bands
export var sliderTrebleWidth       = 0.4   // angular width of sweeping bands
export var sliderBrightness        = 0.9

// ---- Band pool ----
maxBands = 16
bandType   = array(maxBands)   // 0 = rising (vertical), 1 = sweeping (rotational)
bandHue    = array(maxBands)
bandAge    = array(maxBands)
bandLife   = array(maxBands)
bandPos    = array(maxBands)   // z for rising, theta for sweeping
bandVel    = array(maxBands)
bandWidth  = array(maxBands)
bandActive = array(maxBands)

for (i = 0; i < maxBands; i++) bandActive[i] = 0

// ---- Audio analysis state ----
bassAvg        = 0.05
trebleAvg      = 0.05
lastBassBeat   = -10
lastTrebleBeat = -10
elapsed        = 0
bpm            = 120
beatPeriod     = 0.5

// Tracking flags so per-pixel work can be skipped when nothing's active.
hasRising    = 0
hasSweeping  = 0

function findFreeBand() {
  for (i = 0; i < maxBands; i++) {
    if (!bandActive[i]) return i
  }
  // No free slot — recycle the oldest active band.
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
  // Rise from bottom (z=1) through the top (z<0) in roughly 2 beats.
  bandLife[i]   = beatPeriod * 2
  bandPos[i]    = 1.05
  bandVel[i]    = -1.2 / bandLife[i]
  bandWidth[i]  = 0.05 + sliderBassWidth * 0.12 + intensity * 0.05
  bandActive[i] = 1
}

function spawnSweeping(hue, intensity) {
  i = findFreeBand()
  bandType[i]   = 1
  bandHue[i]    = hue
  bandAge[i]    = 0
  // One full revolution every ~4 beats.
  bandLife[i]   = beatPeriod * 4
  bandPos[i]    = random(1) * 2 * PI
  bandVel[i]    = (random(1) > 0.5 ? 1 : -1) * 2 * PI / bandLife[i]
  bandWidth[i]  = 0.15 + sliderTrebleWidth * 0.45 + intensity * 0.1
  bandActive[i] = 1
}

// HSV -> RGB (assumes s = 1). Writes globals hr, hg, hb so callers can
// accumulate into the per-pixel color without allocating temporaries.
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

export function beforeRender(delta) {
  dt = delta * 0.001
  elapsed += dt

  // ---- Aggregate energy in two FFT regions ----
  // Bass: bins 0..7 (lowest frequencies).
  bassNow = 0
  for (i = 0; i < 8; i++) bassNow += frequencyData[i]
  bassNow *= 0.125

  // Treble: bins 20..31 (upper end).
  trebleNow = 0
  for (i = 20; i < 32; i++) trebleNow += frequencyData[i]
  trebleNow *= 1 / 12

  // Long-running averages adapt to the noise floor of the current track.
  bassAvg   = bassAvg   * 0.96 + bassNow   * 0.04
  trebleAvg = trebleAvg * 0.94 + trebleNow * 0.06

  // Adaptive thresholds. Higher sensitivity = more bands fire.
  bassThresh   = bassAvg   * (1.6 - sliderBassSensitivity   * 0.9) + 0.04
  trebleThresh = trebleAvg * (1.6 - sliderTrebleSensitivity * 0.9) + 0.04

  // Minimum gap between detections (caps at ~220 BPM).
  minGap = 60 / 220

  // ---- Bass beat detection ----
  if (bassNow > bassThresh && bassNow > 0.05 &&
      elapsed - lastBassBeat > minGap) {

    // Find the dominant bass bin to pick the hue.
    peakBin = 0
    peakVal = frequencyData[0]
    for (i = 1; i < 8; i++) {
      if (frequencyData[i] > peakVal) { peakVal = frequencyData[i]; peakBin = i }
    }
    // 0 = red, 7 = yellow-orange.
    hue = peakBin / 7 * 0.14
    spawnRising(hue, peakVal)

    // BPM estimate from inter-beat interval.
    if (lastBassBeat > 0) {
      interval = elapsed - lastBassBeat
      if (interval > 0.27 && interval < 1.5) {
        newBpm = 60 / interval
        bpm = bpm * 0.6 + newBpm * 0.4
        beatPeriod = 60 / bpm
      }
    }
    lastBassBeat = elapsed
  }

  // ---- Treble beat detection ----
  if (trebleNow > trebleThresh && trebleNow > 0.04 &&
      elapsed - lastTrebleBeat > minGap * 0.5) {

    peakBin = 20
    peakVal = frequencyData[20]
    for (i = 21; i < 32; i++) {
      if (frequencyData[i] > peakVal) { peakVal = frequencyData[i]; peakBin = i }
    }
    // 20 -> cyan (0.5), 31 -> magenta (0.85).
    hue = 0.5 + (peakBin - 20) / 11 * 0.35
    spawnSweeping(hue, peakVal)
    lastTrebleBeat = elapsed
  }

  // ---- Advance bands and rebuild type flags ----
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

  // Compute the pixel's angle around the cube's vertical axis only when
  // a sweeping band is actually active.
  pxAng = 0
  rxyWeight = 0
  if (hasSweeping) {
    cx = x - 0.5
    cy = y - 0.5
    pxAng = atan2(cy, cx)
    // Fade rotational contribution near the central axis of the bottom
    // panel — atan2 is unstable there and we don't want a chaotic
    // fan effect on the central pixels.
    rxy2 = cx * cx + cy * cy
    rxyWeight = clamp(rxy2 * 6, 0, 1)
  }

  for (i = 0; i < maxBands; i++) {
    if (bandActive[i]) {
      // Hold full brightness for the first third of the band's life,
      // then taper linearly to zero. Bands stay clearly visible while
      // they traverse the cube and only fade as they reach the end.
      p = bandAge[i] / bandLife[i]
      fade = clamp((1 - p) * 1.5, 0, 1)

      if (bandType[i] == 0) {
        // Rising band: distance from band's current z to the pixel's z.
        dist = abs(z - bandPos[i])
        if (dist < bandWidth[i]) {
          bri = (1 - dist / bandWidth[i]) * fade
          h2rgb(bandHue[i], bri)
          r += hr; g += hg; b += hb
        }
      } else {
        // Sweeping band: wrapped angular distance.
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
