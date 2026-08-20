/*
  Sound - Lightbar Ripples 2D

  For the DJ Booth Lightbar (pixelmaps/halfcuboid-bar.js): every kick drops a
  ripple somewhere along the 160x16 banner. Each one expands as a pair of
  wavefronts travelling in opposite directions, bouncing off the ends of the bar
  and passing through each other, fading as they go.

  The cost model is what shapes this. A ripple field is a natural fit for
  "compute per pixel", but at 2560 pixels times several ripples that is tens of
  thousands of distance tests a frame on a controller that manages 10-20 fps.
  Two things avoid it:

    1. The field only varies along x, so it lives in a 160-entry column array
       computed in beforeRender, and render is a lookup.
    2. A wavefront is a THIN RING, so only the ~24 columns around each front are
       touched -- not all 160. Six ripples with two fronts each is about 290
       column writes a frame rather than 160 x 6 x 2 = 1920.

  Reflections come from folding a front's position back into range whenever it
  runs past an end, repeatedly, so a long-lived ripple keeps bouncing rather
  than stopping at the first wall.

  Colour is taken from the loudest bin at the moment the ripple is born, so the
  bar's palette follows what the music is doing. A slow vertical shimmer keeps
  it from reading as flat columns.

  Works against either variant of the map: render3D uses x and z, and render2D
  reconstructs z from the flattened 160x16 plane.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

var speedCtrl = 0.5         // how fast the fronts travel
export function sliderSpeed(v) { speedCtrl = v }

var widthCtrl = 0.45        // how thick each wavefront is
export function sliderWidth(v) { widthCtrl = v }

var decayCtrl = 0.5         // how long a ripple survives
export function sliderDecay(v) { decayCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm  = 120
export var bassSpike    = 0
export var beatCount    = 0
export var liveRipples  = 0

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0

// ---- Geometry ----
barW = 160
barH = 16
gridSize = barW * barH
maxRipples = 6

ripX   = array(maxRipples)   // origin column
ripAge = array(maxRipples)   // seconds since birth; < 0 means the slot is free
ripHue = array(maxRipples)
ripLife = array(maxRipples)

colVal = array(barW)
colHue = array(barW)
rowMul = array(barH)

colOf = array(gridSize)
rowOf = array(gridSize)

frames = 0
mapped = 0
shimmer = 0

for (initIdx = 0; initIdx < maxRipples; initIdx++) ripAge[initIdx] = -1

// Fold a position back into 0..barW-1, bouncing off both ends. Loops because a
// fast, long-lived front can overshoot the bar several times over.
function fold(p) {
  lim = barW - 1
  while (p < 0 || p > lim) {
    if (p < 0) p = -p
    if (p > lim) p = 2 * lim - p
  }
  return p
}

// Paint one wavefront: a narrow band centred on `pos`, brightest at the centre.
function paintFront(pos, amp, hue, halfW) {
  c = floor(pos - halfW)
  cEnd = floor(pos + halfW)
  if (c < 0) c = 0
  if (cEnd > barW - 1) cEnd = barW - 1
  while (c <= cEnd) {
    d = abs(c - pos) / halfW
    if (d < 1) {
      f = (1 - d) * (1 - d) * amp
      if (f > colVal[c]) { colVal[c] = f; colHue[c] = hue }
    }
    c++
  }
}

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  frames++
  if (frames >= 2) mapped = 1

  dw = delta / 2000

  // ---- Loudest bin, for colouring new ripples ----
  peakBin = 0
  peakVal = 0
  for (i = 0; i < 32; i++) {
    if (frequencyData[i] > peakVal) { peakVal = frequencyData[i]; peakBin = i }
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

  // ---- Drop a ripple on the kick ----
  // Oldest slot is recycled when the pool is full, so a hard run of beats keeps
  // producing new ripples instead of being silently dropped.
  if (beat) {
    beatCount++
    slot = -1
    for (r = 0; r < maxRipples; r++) if (ripAge[r] < 0) { slot = r; break }
    if (slot < 0) {
      oldest = 0
      for (r = 1; r < maxRipples; r++) if (ripAge[r] > ripAge[oldest]) oldest = r
      slot = oldest
    }
    ripX[slot] = random(barW)
    ripAge[slot] = 0
    ripHue[slot] = (peakBin / 31) * 0.72
    ripLife[slot] = 1.0 + decayCtrl * 3.0
  }

  // ---- Clear the column field ----
  for (c = 0; c < barW; c++) colVal[c] = 0

  // ---- Advance and paint ----
  speed = 40 + speedCtrl * 190          // columns per second
  halfW = 4 + widthCtrl * 14
  live = 0
  for (r = 0; r < maxRipples; r++) {
    if (ripAge[r] < 0) continue
    ripAge[r] += dt
    if (ripAge[r] >= ripLife[r]) { ripAge[r] = -1; continue }
    live++

    radius = ripAge[r] * speed
    amp = 1 - ripAge[r] / ripLife[r]
    amp = amp * amp                      // fade out smoothly rather than linearly

    paintFront(fold(ripX[r] - radius), amp, ripHue[r], halfW)
    paintFront(fold(ripX[r] + radius), amp, ripHue[r], halfW)
  }
  liveRipples = live

  // ---- Slow vertical shimmer ----
  // Without this the bar reads as flat vertical columns; 16 values a frame is
  // nothing next to the column field.
  shimmer += dt * 0.35
  if (shimmer >= 256) shimmer -= 256
  for (rw = 0; rw < barH; rw++) {
    rowMul[rw] = 0.72 + 0.28 * wave(shimmer + rw / barH * 0.4)
  }
}

export function render3D(index, x, y, z) {
  if (!mapped) {
    // First pass: learn this pixel's cell. Rounding to the nearest of the known
    // positions rather than binning -- the mapper places pixels at exactly
    // col/159 and (15-row)/15, and floor(x * 160) would drop the last column
    // into a 161st bin at x = 1.
    colOf[index] = round(x * (barW - 1))
    rowOf[index] = round((1 - z) * (barH - 1))
    hsv(0, 0, 0)
  } else {
    c = colOf[index]
    v = colVal[c] * rowMul[rowOf[index]]
    if (v > 0.004) hsv(colHue[c], 1, v)
    else hsv(0, 0, 0)
  }
}

export function render2D(index, x, y) {
  // On the flattened map y is height across the unrolled banner.
  render3D(index, x, 0, y)
}

export function render(index) {
  // No map: fall back to 1D over the raw index.
  c = floor(index / pixelCount * barW)
  hsv(colHue[c], 1, colVal[c])
}
