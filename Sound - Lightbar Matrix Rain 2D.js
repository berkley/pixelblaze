/*
  Sound - Lightbar Matrix Rain 2D

  For the DJ Booth Lightbar (pixelmaps/halfcuboid-bar.js): 40 columns of falling
  drops down the 16px height of the banner, each leaving a fading trail behind
  it. Drops are launched by the kick, fall at a tempo-locked rate, and each
  column's colour and brightness are tied to its own slice of the spectrum -- so
  the rain visibly plays the music across the bar.

  The bar is only 16 pixels tall, so a fall is short. That is worked with rather
  than against: drops are launched in bursts on the beat so the whole bar bursts
  into rain and drains, rather than trickling continuously and reading as noise.

  Columns are 4px wide (160 / 40), which matches the box patterns and keeps the
  per-column state arrays tiny. The per-pixel work is two array reads, a
  subtract and a compare -- the column group and row are captured once during
  the first render pass rather than computed per pixel per frame, which on this
  controller is the difference between ~15fps and ~10fps.

  Each column maps to one of the 32 FFT bins, so bass columns sit at the left in
  reds and treble columns at the right in violets, and a column brightens when
  its own bin is loud.

  Works against either variant of the map: render3D uses x and z, and render2D
  reconstructs z from the flattened 160x16 plane.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

var densityCtrl = 0.5       // how many columns launch per kick
export function sliderDensity(v) { densityCtrl = v }

var trailCtrl = 0.5         // trail length behind each drop
export function sliderTrail(v) { trailCtrl = v }

var fallCtrl = 0.5          // beats for one fall down the bar
export function sliderFallSpeed(v) { fallCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var liveDrops   = 0

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0

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
bins = 32

MAXDROPS = 64
head  = array(MAXDROPS)    // row position of each drop; < -900 means idle
hue   = array(MAXDROPS)
bandSpike = array(bins)
bandLvl   = array(bins)
bandPeak  = array(bins)

// Column group and row per pixel, captured once. Storing the GROUP rather than
// the column saves a divide on every pixel of every frame.
colGrp = array(pixelCount)
rowOf  = array(pixelCount)

fallRate = 8
trailLen = 6
frames = 0
phase = 0        // 0 = scanning, 1 = assigning, 2 = running

for (initIdx = 0; initIdx < MAXDROPS; initIdx++) head[initIdx] = -1000

cols = 1
grpW = 1
function onSized() {
  // One drop column per ~4 display columns, capped by the fixed arrays.
  cols = max(2, min(MAXDROPS, ceil(nCols / 4)))
  grpW = nCols / cols
  for (ic = 0; ic < cols; ic++) head[ic] = -1000
}
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

  // ---- Per-bin AGC ----
  // Raw frequencyData on this device runs 0.0003..0.003, well below what the
  // repo's fixed-gain patterns assume, so each bin is normalised against its
  // own rolling mean and decaying peak.
  for (i = 0; i < bins; i++) {
    lvl = frequencyData[i]
    bandLvl[i] = bandLvl[i] * (1 - dw) + lvl * dw
    bandPeak[i] = bandPeak[i] * 0.997
    if (lvl > bandPeak[i]) bandPeak[i] = lvl
    dyn = max(max(bandPeak[i] - bandLvl[i], bandPeak[i] * 0.2), 0.00005)
    bandSpike[i] = min(max(0, lvl - bandLvl[i]) / dyn, 1)
  }

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
  beat = 0
  if (onset) { beatPhase = 0; beat = 1 }
  else if (beatPhase >= 1) { beatPhase -= 1; beat = 1 }

  trailLen = 2 + trailCtrl * 10

  // Rows per second, from beats per fall. Tempo-locked so the rain lands with
  // the music rather than drifting against it.
  beatsPerFall = 0.5 + fallCtrl * 3
  fallRate = (nRows + trailLen) / (beatsPerFall * 60 / detectedBpm)

  // ---- Launch drops on the kick ----
  if (beat) {
    beatCount++
    want = 2 + floor(densityCtrl * min(bassSpike + 0.3, 1) * (cols - 2))
    tries = 0
    launched = 0
    while (launched < want && tries < cols * 2) {
      c = floor(random(cols))
      tries++
      if (head[c] < -900) {
        // Start above the top edge so the drop enters rather than appearing.
        head[c] = -random(3)
        hue[c] = (c / (cols - 1)) * 0.72   // bass=red at the left, treble=violet
        launched++
      }
    }
  }

  // ---- Fall ----
  live = 0
  for (c = 0; c < cols; c++) {
    if (head[c] < -900) continue
    head[c] += fallRate * dt
    // Retire once the whole trail has cleared the bottom edge.
    if (head[c] - trailLen > nRows) head[c] = -1000
    else live++
  }
  liveDrops = live
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
    colGrp[index] = floor((c < 0 ? 0 : c) / grpW)
    rowOf[index] = r < 0 ? 0 : r
    hsv(0, 0, 0)
  } else {
    g = colGrp[index]
    h = head[g]
    d = h - rowOf[index]          // distance BEHIND the head, i.e. above it
    if (d < 0 || d > trailLen) {
      hsv(0, 0, 0)
    } else {
      v = 1 - d / trailLen
      v = v * v
      // The column's own bin drives how hot it burns.
      v = v * (0.3 + 0.7 * bandSpike[floor(g * bins / cols)])
      // The leading pixel washes out to white, as the head of a drop should.
      hsv(hue[g], d < 1 ? 0.15 : 1, v)
    }
  }
}

export function render2D(index, x, y) {
  // On the flattened map y is height across the unrolled banner.
  render3D(index, x, 0, y)
}

export function render(index) {
  // No map: approximate the grid from the raw index.
  g = floor(index / pixelCount * cols)
  hsv(hue[g], 1, head[g] > -900 ? 0.6 : 0)
}
