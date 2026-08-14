/*
  Sound - KITT

  Sound-reactive variant of the stock KITT scanner, saved alongside it rather
  than replacing it.

  The original runs a leader along the strip leaving a fading trail, reversing
  at each end, at a fixed speed (pixelCount / 800). Here the sweep is locked to
  the detected tempo so the scanner reaches each end on a beat, the trail length
  follows sustained loudness, and the hue follows whichever part of the spectrum
  is loudest.

  RESTAGED TO 2D, which is a real departure and worth being explicit about. On
  this bar the pixel index snakes: within each half cuboid it runs left to right
  along the upper face one 8px column at a time, then back right to left along
  the lower face. A 1D sweep therefore crosses the bar as a TEN-LEG ZIGZAG, not
  as one clean pass -- which destroys the single thing KITT is about. So the
  leader is swept along x on the 160x16 grid instead, lighting a full-height
  column. "Sound - Edgeburst" kept its 1D form because a symmetric expanding
  field survives the snake; a discrete moving object does not.

  Position comes from triangle(sweepPhase) rather than a hand-flipped direction
  flag. triangle() already bounces 0 -> 1 -> 0, so the reversal lands exactly on
  the phase boundary and therefore exactly on a beat, with no drift to
  accumulate and no edge case at the ends.

  Columns between the previous frame's leader and this one are filled in, as the
  stock pattern does -- at 15fps and a fast sweep the leader can cross several
  columns per frame, and without that the trail comes out as dashes.

  With no sensor board attached (`light` stays -1) it free-runs at 120 BPM.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

// Beats for one full there-and-back sweep.
var sweepCtrl = 0.3
export function sliderBeatsPerSweep(v) { sweepCtrl = v }

// Trail length. Loud passages shorten it a little on top of this.
var trailCtrl = 0.5
export function sliderTrail(v) { trailCtrl = v }

// How far the hue drifts from KITT red toward the loudest bin. 0 stays red.
var hueCtrl = 0.35
export function sliderSpectrumHue(v) { hueCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var energyNorm  = 0
export var leaderX     = 0

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0
totAvg       = 0
totPeak      = 0

// ---- Geometry ----
barW = 160
barH = 16
gridSize = barW * barH

// The scanner lights a full-height column, so only the column index is needed
// per pixel -- no row lookup, and one 2560-element array instead of two.
colOf = array(gridSize)
colV  = array(barW)    // trail brightness per column

sweepPhase = 0
lastLeader = 0
lastDir    = 1   // sign of the leader's travel, for detecting turnarounds
hue = 0
frames = 0
mapped = 0

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  // One full render pass has completed by the second beforeRender, so colOf[]
  // is populated.
  frames++
  if (frames >= 2) mapped = 1

  dw = delta / 2000

  // ---- Sustained loudness and the loudest bin ----
  totalNow = 0
  peakBin = 0
  peakVal = 0
  for (i = 0; i < 32; i++) {
    e = frequencyData[i]
    totalNow += e
    if (e > peakVal) { peakVal = e; peakBin = i }
  }
  totalNow /= 32
  totAvg = totAvg * (1 - dw) + totalNow * dw
  totPeak = totPeak * 0.997
  if (totalNow > totPeak) totPeak = totalNow
  energyNorm = min(totalNow / max(totPeak, 0.00005), 1)

  // Drift from red toward the loudest bin's hue, smoothed so it does not flick.
  hueTarget = hueCtrl * (peakBin / 31) * 0.6
  hue = hue * 0.9 + hueTarget * 0.1

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
  if (onset) { beatPhase = 0; beatCount++ }
  else if (beatPhase >= 1) { beatPhase -= 1; beatCount++ }

  // ---- Sweep ----
  // One triangle cycle is a there-and-back, so it spans 2x beatsPerSweep beats.
  beatsPerSweep = 1 + sweepCtrl * 7
  sweepPhase += dt * (detectedBpm / 60) / (beatsPerSweep * 2)
  if (sweepPhase >= 1) sweepPhase -= 1

  leaderX = triangle(sweepPhase) * (barW - 1)

  // ---- Trail ----
  // Half-life in seconds rather than a per-frame multiply: the framerate on this
  // bar swings 10..21 and a per-frame decay would visibly change trail length
  // with it. Loud passages shorten the trail slightly.
  halfLife = (0.05 + trailCtrl * 0.55) * (1 - energyNorm * 0.3)
  fade = pow(0.5, dt / max(halfLife, 0.02))
  for (c = 0; c < barW; c++) colV[c] *= fade

  // Light every column between the last leader position and this one, so a fast
  // sweep leaves a continuous streak rather than dashes.
  lo = min(lastLeader, leaderX)
  hi = max(lastLeader, leaderX)

  // Turnarounds need extending to the very end. The leader is only sampled at
  // frame boundaries, so it essentially never lands exactly on the triangle's
  // peak -- measured, the scanner stopped up to 11 columns short at 16fps on a
  // fast sweep, and the region past both samples was never filled because the
  // direction had already reversed. Detect the reversal and run the fill out to
  // the end it just bounced off.
  dir = leaderX - lastLeader
  if (dir > 0 && lastDir <= 0) lo = 0              // just bounced off the left
  else if (dir < 0 && lastDir >= 0) hi = barW - 1  // just bounced off the right
  if (dir != 0) lastDir = dir

  c = floor(lo)
  while (c <= hi) {
    if (c >= 0 && c < barW) colV[c] = 1
    c++
  }
  lastLeader = leaderX
}

export function render3D(index, x, y, z) {
  if (!mapped) {
    // First pass: learn this pixel's column. Rounding to the nearest of the 160
    // known positions rather than binning -- the mapper places pixels at exactly
    // col/159, and floor(x * 160) would drop the last column into a 161st bin.
    colOf[index] = round(x * (barW - 1))
    hsv(0, 0, 0)
  } else {
    v = colV[colOf[index]]
    v = v * v * v      // stock gamma: keeps the trail tight and the head hot
    hsv(hue, 1, v)
  }
}

export function render2D(index, x, y) {
  render3D(index, x, 0, y)
}

export function render(index) {
  // No map: fall back to the stock 1D behaviour over the raw index.
  v = colV[floor(index / pixelCount * barW)]
  v = v * v * v
  hsv(hue, 1, v)
}
