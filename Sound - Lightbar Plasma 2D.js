/*
  Sound - Lightbar Plasma 2D

  For the DJ Booth Lightbar (pixelmaps/halfcuboid-bar.js): a slow perlin colour
  field drifting along the 160x16 banner, with its drift speed, palette and
  brightness driven by the music.

  The interesting problem here is cost. A true 2D noise field means a perlin call
  per pixel, and 2560 of those a frame is far beyond what this controller can do
  at 10-20 fps. Sampling a coarse grid and interpolating is the usual answer, but
  bilinear interpolation is still ~10 operations on every pixel.

  Instead the field is built from two 1D noise strips:

    colF[col]          -- noise along the bar
    diagF[col + row]   -- noise along the DIAGONAL

  and a pixel is just their sum. Two array reads and an add. The diagonal term is
  what stops this being a separable field: a purely per-column plus per-row
  construction reads as plaid, whereas indexing the second strip by col+row tilts
  its bands across the bar and the two interfere into something that genuinely
  looks like plasma. Total cost is 336 perlin calls a frame in beforeRender
  instead of 2560, and render stays a lookup.

  perlin(x, y, z, seed) measured -0.82..0.83 with mean ~0 on this controller, so
  the two strips sum to roughly -1.7..1.7 and are mapped from there.

  Audio drives three things: sustained loudness sets the drift speed and overall
  brightness, the spectral centroid slides the base hue, and each kick gives the
  field a brightness lift that decays away.

  Works against either variant of the map: render3D uses x and z, and render2D
  reconstructs z from the flattened 160x16 plane.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

var scaleCtrl = 0.5         // how tight the noise structure is
export function sliderScale(v) { scaleCtrl = v }

var driftCtrl = 0.45        // base drift speed
export function sliderDrift(v) { driftCtrl = v }

var spreadCtrl = 0.5        // how far hue ranges across the field
export function sliderHueSpread(v) { spreadCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var energyNorm  = 0
export var centroid    = 0.5

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0
totAvg       = 0
totPeak      = 0

// ---- Geometry ----
// The scan quantises positions to QMAX steps while counting how many distinct
// ones exist; it only has to exceed the finest display this will meet.
QMAX = 256
MAXCOLS = 256
MAXROWS = 256
colSeen = array(QMAX)   // presence flags, then rewritten as rank tables
rowSeen = array(QMAX)
nCols = 1
nRows = 1

colF  = array(MAXCOLS)
diagF = array(MAXCOLS + MAXROWS)
colOf = array(pixelCount)
rowOf = array(pixelCount)

driftA = 0
driftB = 0
hueBase = 0
beatLift = 0
frames = 0
phase = 0        // 0 = scanning, 1 = assigning, 2 = running

diagN = 2
function onSized() { diagN = nCols + nRows }
export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  frames++

  // ---- Startup: measure the display's real resolution ----
  if (phase == 0) {
    if (frames >= 2) {
      n = 0
      for (q = 0; q < QMAX; q++) {
        if (colSeen[q] > 0) { colSeen[q] = n; n++ } else colSeen[q] = -1
      }
      nCols = n > 0 ? n : 1
      if (nCols > MAXCOLS) nCols = MAXCOLS
      m = 0
      for (q = 0; q < QMAX; q++) {
        if (rowSeen[q] > 0) { rowSeen[q] = m; m++ } else rowSeen[q] = -1
      }
      nRows = m > 0 ? m : 1
      if (nRows > MAXROWS) nRows = MAXROWS
      onSized()
      phase = 1
    }
    return
  }
  if (phase == 1) { phase = 2; return }

  dw = delta / 2000

  // ---- Loudness and spectral centroid ----
  totalNow = 0
  wsum = 0
  for (i = 0; i < 32; i++) {
    e = frequencyData[i]
    totalNow += e
    wsum += e * i
  }
  cNow = totalNow > 0.00002 ? (wsum / totalNow) / 31 : 0.5
  centroid = centroid * 0.96 + cNow * 0.04
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
  if (onset) { beatPhase = 0; beatCount++; beatLift = 1 }
  else if (beatPhase >= 1) { beatPhase -= 1; beatCount++; beatLift = 1 }

  // Beat lift decays over roughly half a beat.
  beatLift -= dt / (60 / detectedBpm * 0.5)
  if (beatLift < 0) beatLift = 0

  // ---- Advance the two noise strips ----
  // Louder music drifts faster. The two run at different rates so their
  // interference keeps evolving rather than sliding as one rigid picture.
  spd = (0.15 + driftCtrl * 0.85) * (0.4 + energyNorm * 1.6)
  driftA += dt * spd * 0.6
  driftB += dt * spd * 0.37
  if (driftA >= 256) driftA -= 256
  if (driftB >= 256) driftB -= 256

  // Spatial frequency spans a constant number of noise units across the
  // display, so structure looks the same at any resolution.
  sc = (1.9 + scaleCtrl * 7.2) / nCols
  for (c = 0; c < nCols; c++) colF[c] = perlin(c * sc, 0, driftA, 0)
  for (d = 0; d < diagN; d++) diagF[d] = perlin(0, d * sc * 1.35, driftB, 0)

  // Base hue slides with the spectral centroid: bassy music sits red, bright
  // music slides toward blue.
  hueBase = hueBase * 0.97 + (centroid * 0.6) * 0.03
}

export function render3D(index, x, y, z) {
  if (phase == 0) {
    // Pass 1: record which quantised positions this display actually has.
    colSeen[round(x * (QMAX - 1))] = 1
    rowSeen[round((1 - z) * (QMAX - 1))] = 1
    hsv(0, 0, 0)
  } else if (phase == 1) {
    // Pass 2: ranks exist, so give this pixel its dense cell.
    c = colSeen[round(x * (QMAX - 1))]
    r = rowSeen[round((1 - z) * (QMAX - 1))]
    colOf[index] = c < 0 ? 0 : c
    rowOf[index] = r < 0 ? 0 : r
    hsv(0, 0, 0)
  } else {
    c = colOf[index]
    n = colF[c] + diagF[c + rowOf[index]]     // about -1.7 .. 1.7

    h = hueBase + n * (0.08 + spreadCtrl * 0.35)
    h = h - floor(h)

    v = 0.5 + n * 0.32                        // arithmetic only, no wave() call
    if (v < 0) v = 0
    else if (v > 1) v = 1
    v = v * (0.35 + energyNorm * 0.5 + beatLift * 0.3)

    hsv(h, 1, v)
  }
}

export function render2D(index, x, y) {
  // On the flattened map y is height across the unrolled banner.
  render3D(index, x, 0, y)
}

export function render(index) {
  // No map: fall back to the column strip alone over the raw index.
  c = floor(index / pixelCount * nCols)
  n = colF[c]
  h = hueBase + n * 0.3
  hsv(h - floor(h), 1, 0.5 + n * 0.4)
}
