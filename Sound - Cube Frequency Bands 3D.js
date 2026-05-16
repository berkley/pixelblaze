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
  derived from the gap between detected bass beats.

  The SB 1.0 emits very small FFT magnitudes (peaks well under 0.1 even
  on loud music). A PI controller adjusts an internal gain ("sensitivity")
  so that the scaled energy averages around targetFill regardless of
  input volume. Beats are then detected as positive deviations from each
  band's rolling average, in that scaled space.

  Falls back to simulated audio if no SB is detected (light == -1).
*/

export var sliderBassSensitivity   = 0.5   // higher = more bass bands
export var sliderTrebleSensitivity = 0.5   // higher = more treble bands
export var sliderBassWidth         = 0.4
export var sliderTrebleWidth       = 0.4
export var sliderBrightness        = 0.9
export var sliderTestMode          = 0     // > 0: spawn bands on a timer (no audio)

// ---- Sensor board inputs ----
// Set by the SB firmware. If `light` stays at -1 we know no board is
// attached and we fall back to a simulated audio signal.
export var light            = -1
export var frequencyData    = array(32)
export var energyAverage    = 0

// ---- Diagnostic exports (visible in Vars Watch for tuning) ----
export var bassNow      = 0
export var trebleNow    = 0
export var bassAvg      = 0
export var trebleAvg    = 0
export var bassSpike    = 0
export var trebleSpike  = 0
export var sensitivity  = 50
export var detectedBpm  = 120

// ---- PI controller for auto gain ----
// Targets a normalized energy level of `targetFill`. The integrator
// (pic[2]) compensates for steady-state error, so quiet rooms drive
// sensitivity high (up to pic[4]) and loud rooms pull it back down.
targetFill = 0.2
pic = array(5)
pic[0] = 0.2    // Kp
pic[1] = 0.15   // Ki
pic[2] = 50     // integrator start
pic[3] = 0      // min
pic[4] = 400    // max

function calcPI(err) {
  pic[2] = clamp(pic[2] + err, pic[3], pic[4])
  return max(pic[0] * err + pic[1] * pic[2], 0.3)
}

// ---- Band pool ----
maxBands   = 16
bandType   = array(maxBands)   // 0 = rising vertical, 1 = sweeping rotational
bandHue    = array(maxBands)
bandAge    = array(maxBands)
bandLife   = array(maxBands)
bandPos    = array(maxBands)   // z for rising, theta for sweeping
bandVel    = array(maxBands)
bandWidth  = array(maxBands)
bandActive = array(maxBands)

for (i = 0; i < maxBands; i++) bandActive[i] = 0

// ---- Beat / timing state ----
lastBassBeat   = -10
lastTrebleBeat = -10
elapsed        = 0
beatPeriod     = 0.5

// Skip flags so per-pixel render can bail when nothing's drawing.
hasRising   = 0
hasSweeping = 0

// Test-mode timer
testAccum = 0

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
  bandWidth[i]  = 0.05 + sliderBassWidth * 0.12 + clamp(intensity, 0, 2) * 0.04
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
  bandWidth[i]  = 0.15 + sliderTrebleWidth * 0.45 + clamp(intensity, 0, 2) * 0.08
  bandActive[i] = 1
}

// HSV -> RGB (sat = 1). Writes globals hr/hg/hb so callers can accumulate.
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

// ---- Simulated audio (used when no SB is detected) ----
// A simple 120-BPM 4-on-the-floor pattern: kick on every quarter, claps
// on offbeats, hi-hat on 2 and 4. Lets you test rendering without the
// sensor board attached.
simBpm = 120
simMeasure = 4 * 60 / simBpm / 65.536

function simulateAudio() {
  tM = time(simMeasure)
  for (i = 0; i < 32; i++) frequencyData[i] = 0

  // Kick — energy in lowest bins
  beat = (-4 * tM + 5) % 1
  beat *= 0.02 * pow(beat, 4)
  for (i = 0; i < 10; i++) frequencyData[i] += beat * (10 - i) / 10

  // Claps — energy in low-mid bins on offbeats
  claps = 0.006 * square(2 * tM - 0.5, 0.10)
  for (i = 9; i < 18; i++) frequencyData[i] += claps * (0.7 + 0.6 * random(0.5))

  // Hi-hat — energy in high bins on 2 and 4
  hh = 0.012 * square(4 * tM - 0.5, 0.05)
  for (i = 22; i < 30; i++) frequencyData[i] += hh * (0.8 + random(0.4))

  energyAverage = beat * 0.5 + claps * 0.5 + hh * 0.5
}

export function beforeRender(delta) {
  dt = delta * 0.001
  elapsed += dt

  // ---- Audio source ----
  // If no sensor board is attached, run the simulator at 40 Hz so the
  // rendering pipeline can be verified.
  if (light == -1) simulateAudio()

  // ---- Auto gain ----
  // Drive sensitivity so that sensitivity * energyAverage ≈ targetFill.
  // Quiet rooms ramp sensitivity up; loud rooms pull it back down.
  scaledEnergy = sensitivity * energyAverage
  sensitivity  = calcPI(targetFill - scaledEnergy)

  // ---- Aggregate energy in two FFT regions (raw, no gain) ----
  bassNow = 0
  for (i = 0; i < 8; i++) bassNow += frequencyData[i]
  bassNow *= 0.125

  trebleNow = 0
  for (i = 20; i < 32; i++) trebleNow += frequencyData[i]
  trebleNow *= 1 / 12

  // Rolling averages of the raw values over ~1500 ms.
  dw = delta / 1500
  bassAvg   = bassAvg   * (1 - dw) + bassNow   * dw
  trebleAvg = trebleAvg * (1 - dw) + trebleNow * dw

  // Spike = how far current sits above its rolling average, in the
  // gain-normalized space. Negative deviations are clipped to 0.
  bassSpike   = max(0, bassNow   - bassAvg)   * sensitivity
  trebleSpike = max(0, trebleNow - trebleAvg) * sensitivity

  // Beat thresholds: higher sensitivity slider lowers the bar.
  bassBeatThresh   = 0.50 - sliderBassSensitivity   * 0.40   // 0.10..0.50
  trebleBeatThresh = 0.40 - sliderTrebleSensitivity * 0.32   // 0.08..0.40

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

  // ---- Test mode (audio-independent) ----
  // Spawn alternating bands on a timer at a rate set by the slider.
  // Useful to confirm the rendering pipeline works when audio isn't
  // triggering anything yet.
  if (sliderTestMode > 0) {
    testAccum += dt
    testInterval = 0.15 + (1 - sliderTestMode) * 1.0   // 0.15..1.15 s
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
