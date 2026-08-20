/*
  Sound - Aurora Borealis 2D

  Sound-reactive version of "1D Aurora Borealis" from the ElectroMage pattern
  library (20 votes, https://patterns.electromage.com), itself a port of
  Mazn1191's Arduino-Borealis. Saved alongside anything already on the device
  rather than replacing it.

  The original maintains a set of coloured waves, each with a centre, width,
  lifetime and drift direction, and alpha-blends whichever overlap a given
  pixel. Waves are born, drift, occasionally reverse, and die.

  THE REWRITE THAT MATTERED: the original evaluates every wave inside render,
  i.e. once per pixel. On a 160x16 bar that is 2560 pixels x 10 waves = 25,600
  lambda invocations a frame, on a controller that manages 10-20 fps. The wave
  field only varies along the bar, so it is now computed ONCE into a 160-entry
  RGB column field in beforeRender -- 1,600 evaluations instead of 25,600 -- and
  render is three array reads. The visual result is identical; it is purely a
  question of where the loop lives. (This is the same gather-to-scatter change
  the Comets and Ripples patterns needed.)

  Audio drives it in three ways:
    - Kicks spawn new waves, so curtains appear on the beat instead of only when
      an old one dies.
    - Drift speed follows the detected tempo.
    - The spectral centroid picks the palette weighting: bass-heavy music
      favours the pinks and purples, brighter music the greens and turquoises.

  A gentle vertical gradient is added over the original's flat 1D output, so the
  curtains hang down the bar's 16px height rather than reading as solid columns.

  Colours are the original's palette, blended in RGB and emitted with rgb().
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

var speedCtrl = 0.4         // drift speed
export function sliderSpeed(v) { speedCtrl = v }

var widthCtrl = 0.35        // how broad each curtain is
export function sliderWidth(v) { widthCtrl = v }

var countCtrl = 0.5         // how many curtains at once
export function sliderNumWaves(v) { countCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var liveWaves   = 0
export var centroid    = 0.5

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

// ---- Palette (the original's five colours, 0..1) ----
palR = array(5); palG = array(5); palB = array(5)
palR[0] = 0.067; palG[0] = 0.694; palB[0] = 0.051   // greenish
palR[1] = 0.580; palG[1] = 0.949; palB[1] = 0.020   // greenish
palR[2] = 0.098; palG[2] = 0.678; palB[2] = 0.475   // turquoise
palR[3] = 0.980; palG[3] = 0.302; palB[3] = 0.498   // pink
palR[4] = 0.671; palG[4] = 0.396; palB[4] = 0.867   // purple

// ---- Waves ----
maxWaves = 10
wAge   = array(maxWaves)
wTtl   = array(maxWaves)
wCol   = array(maxWaves)
wAlpha = array(maxWaves)
wWidth = array(maxWaves)
wCent  = array(maxWaves)
wDir   = array(maxWaves)
wSpeed = array(maxWaves)

// Column colour field, and the per-pixel lookups.
colR = array(barW); colG = array(barW); colB = array(barW)
rowMul = array(barH)
colOf = array(gridSize)
rowOf = array(gridSize)

frames = 0
mapped = 0
numWaves = 6

for (initIdx = 0; initIdx < maxWaves; initIdx++) wTtl[initIdx] = -1

// Pick a palette entry, weighted by where the music's energy sits. Bass-heavy
// favours pink/purple, bright favours green/turquoise -- the original chose
// between fixed weightings with a slider; here the music chooses.
function pickColor() {
  bassBias = 1 - centroid
  w0 = 6 + centroid * 26
  w1 = 6 + centroid * 26
  w2 = 6 + centroid * 14
  w3 = 2 + bassBias * 30
  w4 = 1 + bassBias * 30
  tot = w0 + w1 + w2 + w3 + w4
  r = random(tot)
  if (r < w0) return 0
  r -= w0; if (r < w1) return 1
  r -= w1; if (r < w2) return 2
  r -= w2; if (r < w3) return 3
  return 4
}

function spawn(s) {
  wAge[s] = 0
  wTtl[s] = 6 + random(12)                    // seconds
  wCol[s] = pickColor()
  wAlpha[s] = 0.5 + random(0.5)
  wWidth[s] = (0.06 + random(widthCtrl)) * barW
  wCent[s] = random(barW)
  wDir[s] = random(1) >= 0.5 ? 1 : -1
  wSpeed[s] = (4 + random(14)) * (0.4 + speedCtrl * 1.6)   // columns per second
}

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  frames++
  if (frames >= 2) mapped = 1

  dw = delta / 2000

  // ---- Spectral centroid ----
  totalNow = 0
  wsum = 0
  for (i = 0; i < 32; i++) {
    e = frequencyData[i]
    totalNow += e
    wsum += e * i
  }
  cNow = totalNow > 0.00002 ? (wsum / totalNow) / 31 : 0.5
  centroid = centroid * 0.97 + cNow * 0.03

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

  numWaves = 2 + floor(countCtrl * (maxWaves - 2))

  // ---- Move, retire and spawn ----
  live = 0
  for (s = 0; s < maxWaves; s++) {
    if (s >= numWaves) { wTtl[s] = -1; continue }
    if (wTtl[s] < 0) continue

    // Real aurorae wander, so waves occasionally reverse.
    if (random(1) < 0.03) wDir[s] = -wDir[s]
    wCent[s] += wSpeed[s] * wDir[s] * dt
    wAge[s] += dt

    dead = 0
    if (wAge[s] > wTtl[s]) dead = 1
    if (wCent[s] + wWidth[s] < 0) dead = 1
    if (wCent[s] - wWidth[s] > barW) dead = 1
    if (dead) { wTtl[s] = -1; continue }
    live++
  }

  // A kick brings in a new curtain if there is room for one.
  if (beat) {
    beatCount++
    for (s = 0; s < numWaves; s++) {
      if (wTtl[s] < 0) { spawn(s); live++; break }
    }
  }
  // Keep the sky populated even without beats.
  for (s = 0; s < numWaves; s++) {
    if (wTtl[s] < 0 && random(1) < 0.6 * dt) { spawn(s); live++ }
  }
  liveWaves = live

  // ---- Build the column field ----
  // This is the whole point of the rewrite: once per column, not once per pixel.
  for (c = 0; c < barW; c++) { colR[c] = 0; colG[c] = 0; colB[c] = 0 }

  for (s = 0; s < maxWaves; s++) {
    if (s >= numWaves || wTtl[s] < 0) continue
    // Brightest at half the wave's life, as in the original.
    ageF = triangle(sqrt(wAge[s] / wTtl[s]))
    ctr = wCent[s]
    wid = wWidth[s]
    lo = floor(ctr - wid); hi = floor(ctr + wid)
    if (lo < 0) lo = 0
    if (hi > barW - 1) hi = barW - 1
    tr = palR[wCol[s]]; tg = palG[wCol[s]]; tb = palB[wCol[s]]
    c = lo
    while (c <= hi) {
      off = abs(c - ctr)
      if (off < wid) {
        a = (1 - sqrt(off / wid)) * ageF * wAlpha[s]
        colR[c] = tr * a + colR[c] * (1 - a)
        colG[c] = tg * a + colG[c] * (1 - a)
        colB[c] = tb * a + colB[c] * (1 - a)
      }
      c++
    }
  }

  // Curtains hang: brighter toward the top of the bar, softly falling away.
  for (rw = 0; rw < barH; rw++) rowMul[rw] = 0.45 + 0.55 * (1 - rw / (barH - 1))
}

export function render3D(index, x, y, z) {
  if (!mapped) {
    // First pass: learn this pixel's cell. Rounding to the nearest of the known
    // positions rather than binning -- the mapper places pixels at exactly
    // col/159 and (15-row)/15, and floor(x * 160) would drop the last column
    // into a 161st bin at x = 1.
    colOf[index] = round(x * (barW - 1))
    rowOf[index] = round((1 - z) * (barH - 1))
    rgb(0, 0, 0)
  } else {
    c = colOf[index]
    m = rowMul[rowOf[index]]
    rgb(colR[c] * m, colG[c] * m, colB[c] * m)
  }
}

export function render2D(index, x, y) {
  // On the flattened map y is height across the unrolled banner.
  render3D(index, x, 0, y)
}

export function render(index) {
  // No map: the column field still works over the raw index.
  c = floor(index / pixelCount * barW)
  rgb(colR[c], colG[c], colB[c])
}
