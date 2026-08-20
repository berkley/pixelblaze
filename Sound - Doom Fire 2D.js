/*
  Sound - Doom Fire 2D

  Sound-reactive version of "Doom Fire (v2.0) 2D" by JEM (ZRanger1), from the
  ElectroMage pattern library (https://patterns.electromage.com), which is in
  turn based on the convolution fire from the PSX port of DOOM. Saved alongside
  anything already on the device rather than replacing it.

  The original seeds a hot bottom row and repeatedly computes each cell from the
  one below it, minus a random cooling term, with a wandering wind offset. No
  noise fields -- the fire emerges purely from that convolution.

  THE INTERESTING CHANGE: the bottom row is where fire is born, and that row has
  one cell per column of the display. So instead of the original's slow
  sinusoidal wander, each column's base heat is driven by its own FFT bin. The
  fire is therefore hottest, and burns highest, wherever the music is loud --
  bass at the left, treble at the right. It is a spectrum analyser made of
  flame, and it needed no change to the fire algorithm itself, only to what
  feeds it.

  Beats additionally cut the cooling rate for a moment, so the whole wall of
  flame leaps on the kick, and hard hits kick the wind sideways.

  Sized 160x16 for this bar rather than the original's 16x16. That makes the
  simulation 160x15 = 2400 cells, which is heavy enough that it is throttled to
  a fixed rate rather than run every frame -- fire looks correct at ~20
  simulation steps a second and there is no point spending frames on more.

  Fire rises from the bottom outer edge, over the ridge, to the top outer edge.
  Row indices are flipped relative to height accordingly: the source row is the
  bar's bottom, which is z = 0.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

var heightCtrl = 0.55       // flame height (inverse of cooling)
export function sliderFlameHeight(v) { heightCtrl = v }

var windCtrl = 0.35         // how readily the wind shifts
export function sliderWind(v) { windCtrl = v }

var spectrumCtrl = 0.7      // how strongly the spectrum shapes the base row
export function sliderSpectrum(v) { spectrumCtrl = v }

export function hsvPickerFlame(h, s, v) { baseHue = h; baseBri = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var leap        = 0

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0

// ---- Display / simulation size ----
var width = 160
var height = 16
var arrayWidth = width + 2       // a margin column each side, so wind cannot
var arrayHeight = height + 1     // index off the end and the source row fits
var lastRow = arrayHeight - 1
var lastCol = width + 1

var buffer1 = array(arrayWidth)
var buffer2 = array(arrayWidth)
var pb1, pb2

var baseHue = 0.02
var baseBri = 0.85
var maxCooling = 0.275
var windDirection = 0
var frameTimer = 9999
var simulationSpeed = 50         // ms between simulation steps

bins = 32
bandLvl   = array(bins)
bandPeak  = array(bins)
bandSpike = array(bins)

function allocateFrameBuffers() {
  for (var i = 0; i < arrayWidth; i++) {
    buffer1[i] = array(arrayHeight)
    buffer2[i] = array(arrayHeight)
  }
  pb1 = buffer1
  pb2 = buffer2
}

function initBuffers() {
  for (var i = 0; i < arrayWidth; i++) {
    pb1[i][lastRow] = 1
    pb2[i][lastRow] = 1
  }
}

function swapBuffers() { var tmp = pb1; pb1 = pb2; pb2 = tmp }

// The source row, driven by the spectrum. Each column maps to the FFT bin that
// covers it, so loud bands burn hot and quiet ones barely smoulder.
function perturbFromSpectrum() {
  for (var i = 0; i < arrayWidth; i++) {
    b = floor((i - 1) * bins / width)
    if (b < 0) b = 0
    else if (b > bins - 1) b = bins - 1
    // Floor of 0.55 so the fire never dies out entirely between hits.
    pb2[i][lastRow] = 0.55 + spectrumCtrl * bandSpike[b] * 0.75
  }
}

// Fire is hottest at the bottom and cools as it rises. Each cell derives from
// the one below it, offset by the wind. Unchanged from the original apart from
// cooling being modulated by the beat.
function doFire() {
  swapBuffers()

  if (windStrength > 0) {
    windDirection = (random(1) < windStrength) ? floor(random(3)) - 1 : windDirection
  }

  for (var x = 1; x < lastCol; x++) {
    for (var y = 1; y < lastRow; y++) {
      var r = random(coolNow) * (y / lastRow)
      var windFx = (abs((lastRow / 2) - y) / lastRow)
      windFx = x + (random(1) < 0.5 - windFx) * windDirection
      pb2[x][y] = max(0, pb1[windFx][y + 1] - r)
    }
  }
}

allocateFrameBuffers()
initBuffers()

coolNow = 0.275
windStrength = 0.1

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  dw = delta / 2000

  // ---- Per-bin AGC ----
  // Raw frequencyData on this device runs 0.0003..0.003, far below what
  // fixed-gain patterns assume, so each bin is normalised against its own
  // rolling mean and decaying peak.
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
  if (onset) { beatPhase = 0; beatCount++; leap = min(bassSpike, 1) }
  else if (beatPhase >= 1) { beatPhase -= 1; beatCount++; leap = min(bassSpike, 1) }

  leap -= dt / (60 / detectedBpm * 0.6)
  if (leap < 0) leap = 0

  // Less cooling means taller flames, so the beat makes the wall leap.
  base = (1 - heightCtrl)
  base = base * base
  coolNow = max(4 * base, 0.1) * (1 - leap * 0.55)

  // Hard hits shove the wind about.
  windStrength = (windCtrl * windCtrl) / 2 * (1 + leap * 2)

  // ---- Throttled simulation ----
  // 2400 cells a step is too much to run every frame, and fire reads correctly
  // at about 20 steps a second regardless of the render rate.
  frameTimer += delta
  if (frameTimer > simulationSpeed) {
    doFire()
    perturbFromSpectrum()
    frameTimer = 0
  }
}

export function render3D(index, x, y, z) {
  // Column index, with the 1-cell margin the wind needs.
  cx = 1 + x * width
  // Row 0 is the top of the buffer and lastRow is the source, so height has to
  // be flipped: the fire is born at the bar's bottom edge, z = 0.
  cy = (1 - z) * height
  bri = pb2[cx][cy]
  bri = bri * bri * bri
  hsv(baseHue + (0.05 * bri), 1.3 - bri / 4, baseBri * bri)
}

export function render2D(index, x, y) {
  // On the flattened map y is height across the unrolled banner.
  render3D(index, x, 0, y)
}

export function render(index) {
  // No map: read the middle of the fire along the strip.
  bri = pb2[1 + (index / pixelCount) * width][height / 2]
  bri = bri * bri * bri
  hsv(baseHue + (0.05 * bri), 1.3 - bri / 4, baseBri * bri)
}
