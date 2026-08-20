/*
  Sound - Lightbar Comets 2D

  For the DJ Booth Lightbar (pixelmaps/halfcuboid-bar.js): points streaking along
  the 160x16 banner leaving motion-blur tails. Kicks launch them, sustained
  loudness sets how fast they fly. The bar's extreme aspect ratio is the point --
  a comet crossing 160 pixels reads as real speed in a way it never would on a
  square panel.

  PORTABLE ACROSS DISPLAYS. This originally hardcoded the Lightbar's 160x16
  grid and produced garbage elsewhere: on a 1280-pixel cube it lit 4.8% of the
  display with 47% of all brightness in ten pixels. Two changes fix it, the same
  pair that fixed "Sound - Lightbar Beat Boxes 2D":

    - The lookup runs FORWARD. It used to build a grid-position-to-pixel table
      and write through it; cells no pixel claimed held 0, so those writes piled
      onto pixel 0. The framebuffer now lives in GRID space and each pixel reads
      the cell it occupies.
    - The grid is MEASURED, not assumed. Startup discovers how many distinct
      column and row positions the display really has.

  Moving the buffer to grid space relocates the trail decay from render into
  beforeRender. That costs nothing overall -- the decay now runs over grid cells
  instead of pixels, and on the bar those counts are equal -- but it is now a
  real sweep rather than a free ride, and it MUST be there: several pixels can
  share one cell, and decaying in render would decay a shared cell once per
  pixel sharing it.

  Speeds are expressed as fractions of the display's width and height per
  second, not pixels per second. A comet crossing a 160-wide bar at 190px/s
  would cross an 18-wide cube nine times too fast otherwise.

  Architecture otherwise as before:
    - Decay is pow(0.5, dt/halfLife), not a per-frame multiply, because the
      framerate swings 10..27 across these displays and a per-frame decay
      visibly shortens tails whenever it dips.
    - Comets are splatted bilinearly across the four pixels around their
      fractional position. Sub-pixel placement is not optional: snapping a small
      sprite to whole pixels turns smooth motion into visible stepping.
    - beforeRender splats into the same buffer with max-wins, which also latches
      each trail's colour to whatever laid it down.

  Comets travel horizontally at a slight vertical drift, so they cross the ridge
  between the two faces rather than staying on one. They are retired when they
  leave the bar rather than wrapping -- a comet should exit, and letting them
  wrap made the bar feel like a treadmill.

  Colour comes from the loudest bin at launch, so a bass-heavy track throws red
  comets and a bright one throws violet.

  Works against either variant of the map: render3D uses x and z, and render2D
  reconstructs z from the flattened 160x16 plane.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

var rateCtrl = 0.5          // how many launch per kick
export function sliderLaunchRate(v) { rateCtrl = v }

var speedCtrl = 0.5         // base travel speed
export function sliderSpeed(v) { speedCtrl = v }

var tailCtrl = 0.45         // tail half-life
export function sliderTail(v) { tailCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var liveComets  = 0
export var energyNorm  = 0

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
MAXGRID = 2560
maxComets = 24

// colSeen/rowSeen serve twice: presence flags during the scan, then rewritten
// in place as rank tables mapping a quantised position to a dense index.
colSeen = array(QMAX)
rowSeen = array(QMAX)
nCols = 1
nRows = 1
gridSize = 1

cx   = array(maxComets)
cy   = array(maxComets)
cvx  = array(maxComets)
cvy  = array(maxComets)
chue = array(maxComets)
calive = array(maxComets)

// Framebuffer in GRID space. gridVal doubles as the occupancy flag and is never
// cleared -- it decays.
gridHue = array(MAXGRID)
gridVal = array(MAXGRID)
pxCell  = array(pixelCount)   // pixel index -> its grid cell. Sized to the device.

decay = 0.5
frames = 0
phase = 0        // 0 = scanning positions, 1 = assigning pixels, 2 = running

for (initIdx = 0; initIdx < maxComets; initIdx++) calive[initIdx] = 0

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  frames++

  // ---- Startup: measure the display, then assign pixels to cells ----
  if (phase == 0) {
    if (frames >= 2) {
      n = 0
      for (q = 0; q < QMAX; q++) {
        if (colSeen[q] > 0) { colSeen[q] = n; n++ } else colSeen[q] = -1
      }
      nCols = n > 0 ? n : 1
      m = 0
      for (q = 0; q < QMAX; q++) {
        if (rowSeen[q] > 0) { rowSeen[q] = m; m++ } else rowSeen[q] = -1
      }
      nRows = m > 0 ? m : 1
      gridSize = nCols * nRows
      if (gridSize > MAXGRID) gridSize = MAXGRID
      phase = 1
    }
    return
  }
  if (phase == 1) { phase = 2; return }

  dw = delta / 2000

  // ---- Loudness and the loudest bin ----
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

  // Widths per second, scaled to the measured grid. Expressed in pixels this
  // would cross a narrow display many times too fast.
  baseSpeed = (0.375 + speedCtrl * 1.625) * nCols * (0.45 + energyNorm * 1.1)

  // ---- Launch on the kick ----
  if (beat) {
    beatCount++
    want = 1 + floor(rateCtrl * min(bassSpike + 0.4, 1) * 7)
    launched = 0
    for (i = 0; i < maxComets && launched < want; i++) {
      if (!calive[i]) {
        // Enter from whichever end, just off the bar so they fly in.
        rightward = random(1) > 0.5
        cx[i] = rightward ? -2 : nCols + 1
        cy[i] = random(nRows - 1)
        cvx[i] = (rightward ? 1 : -1) * baseSpeed * (0.7 + random(0.6))
        // Drift as a fraction of height per second, so it reads the same on a
        // 16-row bar and a 32-row cuboid.
        cvy[i] = (random(1) * 2 - 1) * 0.19 * nRows
        chue[i] = (peakBin / 31) * 0.72
        calive[i] = 1
        launched++
      }
    }
  }

  // ---- Tail decay ----
  // Over grid cells, not pixels: several pixels can share a cell, so decaying
  // in render would decay a shared cell once per pixel sharing it.
  halfLife = 0.03 + tailCtrl * 0.35
  decay = pow(0.5, dt / halfLife)
  for (ci = 0; ci < gridSize; ci++) {
    gv = gridVal[ci]
    if (gv > 0) gridVal[ci] = gv > 0.004 ? gv * decay : 0
  }

  // ---- Move and splat ----
  live = 0
  {
    for (i = 0; i < maxComets; i++) {
      if (!calive[i]) continue
      nx = cx[i] + cvx[i] * dt
      ny = cy[i] + cvy[i] * dt

      // Retire once clear of the bar. Comets exit rather than wrap.
      if (nx < -4 || nx > nCols + 4) { calive[i] = 0; continue }
      if (ny < 0) { ny = 0; cvy[i] = -cvy[i] }
      else if (ny > nRows - 1) { ny = nRows - 1; cvy[i] = -cvy[i] }

      cx[i] = nx
      cy[i] = ny
      live++

      // Only splat the part that is actually over the bar.
      if (nx < 0 || nx > nCols - 1) continue

      ix = floor(nx); fx = nx - ix
      iy = floor(ny); fy = ny - iy
      ix1 = ix + 1
      if (ix1 > nCols - 1) ix1 = nCols - 1

      w00 = (1 - fx) * (1 - fy)
      w10 = fx * (1 - fy)
      w01 = (1 - fx) * fy
      w11 = fx * fy
      hu = chue[i]

      rb = iy * nCols
      p = rb + ix;  if (w00 > gridVal[p]) { gridVal[p] = w00; gridHue[p] = hu }
      p = rb + ix1; if (w10 > gridVal[p]) { gridVal[p] = w10; gridHue[p] = hu }
      if (iy + 1 < nRows) {
        rb = (iy + 1) * nCols
        p = rb + ix;  if (w01 > gridVal[p]) { gridVal[p] = w01; gridHue[p] = hu }
        p = rb + ix1; if (w11 > gridVal[p]) { gridVal[p] = w11; gridHue[p] = hu }
      }
    }
  }
  liveComets = live
}

export function render3D(index, x, y, z) {
  if (phase == 0) {
    // Pass 1: record which quantised positions this display actually has.
    colSeen[round(x * (QMAX - 1))] = 1
    rowSeen[round((1 - z) * (QMAX - 1))] = 1
    hsv(0, 0, 0)
  } else if (phase == 1) {
    // Pass 2: the ranks exist, so give this pixel its dense cell.
    c = colSeen[round(x * (QMAX - 1))]
    r = rowSeen[round((1 - z) * (QMAX - 1))]
    if (c < 0) c = 0
    if (r < 0) r = 0
    pxCell[index] = r * nCols + c
    hsv(0, 0, 0)
  } else {
    // Forward lookup: this pixel reads the cell it occupies.
    p = pxCell[index]
    v = gridVal[p]
    if (v > 0.004) hsv(gridHue[p], 1, v)
    else hsv(0, 0, 0)
  }
}

export function render2D(index, x, y) {
  // On the flattened map y is height across the unrolled banner.
  render3D(index, x, 0, y)
}

export function render(index) {
  // No map: the framebuffer has no meaning without coordinates, so show a plain
  // travelling dot instead.
  hsv(0.6, 1, abs((index / pixelCount) - (elapsed % 1)) < 0.02 ? 1 : 0)
}
