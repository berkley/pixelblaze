/*
  Sound - Lightbar Comets 2D

  For the DJ Booth Lightbar (pixelmaps/halfcuboid-bar.js): points streaking along
  the 160x16 banner leaving motion-blur tails. Kicks launch them, sustained
  loudness sets how fast they fly. The bar's extreme aspect ratio is the point --
  a comet crossing 160 pixels reads as real speed in a way it never would on a
  square panel.

  Architecture is lifted from "Sound - Lightbar Wind Particles 2D", which solved
  the same problems:

    - Tails come from a framebuffer decayed lazily INSIDE render rather than in a
      separate sweep, so every pixel is touched exactly once a frame and the
      trails are close to free.
    - Decay is pow(0.5, dt/halfLife), not a per-frame multiply, because the
      framerate here swings 10..21 and a per-frame decay visibly shortens tails
      whenever it dips.
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
barW = 160
barH = 16
gridSize = barW * barH
maxComets = 24

cx   = array(maxComets)
cy   = array(maxComets)
cvx  = array(maxComets)
cvy  = array(maxComets)
chue = array(maxComets)
calive = array(maxComets)

// Framebuffer, indexed by pixel index. pixVal doubles as the occupancy flag, so
// only it needs clearing -- and it is not cleared at all here, it decays.
pixHue = array(gridSize)
pixVal = array(gridSize)
idxOf  = array(gridSize)   // grid position (row * barW + col) -> pixel index

decay = 0.5
frames = 0
mapped = 0

for (initIdx = 0; initIdx < maxComets; initIdx++) calive[initIdx] = 0

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  frames++
  if (frames >= 2) mapped = 1

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

  baseSpeed = (60 + speedCtrl * 260) * (0.45 + energyNorm * 1.1)

  // ---- Launch on the kick ----
  if (beat) {
    beatCount++
    want = 1 + floor(rateCtrl * min(bassSpike + 0.4, 1) * 7)
    launched = 0
    for (i = 0; i < maxComets && launched < want; i++) {
      if (!calive[i]) {
        // Enter from whichever end, just off the bar so they fly in.
        rightward = random(1) > 0.5
        cx[i] = rightward ? -2 : barW + 1
        cy[i] = random(barH - 1)
        cvx[i] = (rightward ? 1 : -1) * baseSpeed * (0.7 + random(0.6))
        cvy[i] = (random(1) * 2 - 1) * 3      // slight drift across the ridge
        chue[i] = (peakBin / 31) * 0.72
        calive[i] = 1
        launched++
      }
    }
  }

  // ---- Tail decay for this frame ----
  halfLife = 0.03 + tailCtrl * 0.35
  decay = pow(0.5, dt / halfLife)

  // ---- Move and splat ----
  live = 0
  if (mapped) {
    for (i = 0; i < maxComets; i++) {
      if (!calive[i]) continue
      nx = cx[i] + cvx[i] * dt
      ny = cy[i] + cvy[i] * dt

      // Retire once clear of the bar. Comets exit rather than wrap.
      if (nx < -4 || nx > barW + 4) { calive[i] = 0; continue }
      if (ny < 0) { ny = 0; cvy[i] = -cvy[i] }
      else if (ny > barH - 1) { ny = barH - 1; cvy[i] = -cvy[i] }

      cx[i] = nx
      cy[i] = ny
      live++

      // Only splat the part that is actually over the bar.
      if (nx < 0 || nx > barW - 1) continue

      ix = floor(nx); fx = nx - ix
      iy = floor(ny); fy = ny - iy
      ix1 = ix + 1
      if (ix1 > barW - 1) ix1 = barW - 1

      w00 = (1 - fx) * (1 - fy)
      w10 = fx * (1 - fy)
      w01 = (1 - fx) * fy
      w11 = fx * fy
      hu = chue[i]

      rb = iy * barW
      p = idxOf[rb + ix];  if (w00 > pixVal[p]) { pixVal[p] = w00; pixHue[p] = hu }
      p = idxOf[rb + ix1]; if (w10 > pixVal[p]) { pixVal[p] = w10; pixHue[p] = hu }
      if (iy + 1 < barH) {
        rb = (iy + 1) * barW
        p = idxOf[rb + ix];  if (w01 > pixVal[p]) { pixVal[p] = w01; pixHue[p] = hu }
        p = idxOf[rb + ix1]; if (w11 > pixVal[p]) { pixVal[p] = w11; pixHue[p] = hu }
      }
    }
  }
  liveComets = live
}

export function render3D(index, x, y, z) {
  if (!mapped) {
    // First pass: learn where this pixel sits on the grid. Rounding to the
    // nearest of the known positions rather than binning -- the mapper places
    // pixels at exactly col/159 and (15-row)/15, and floor(x * 160) would drop
    // the last column into a 161st bin at x = 1.
    col = round(x * (barW - 1))
    row = round((1 - z) * (barH - 1))
    idxOf[row * barW + col] = index
    hsv(0, 0, 0)
  } else {
    // Trail decay happens here rather than in a separate sweep, so every pixel
    // is touched exactly once per frame.
    v = pixVal[index] * decay
    pixVal[index] = v
    if (v > 0.004) hsv(pixHue[index], 1, v)
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
