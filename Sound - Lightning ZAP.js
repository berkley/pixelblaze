/*
  Sound - Lightning ZAP

  Sound-reactive version of "lightning ZAP!" by the ElectroMage pattern library
  (26 votes, https://patterns.electromage.com). Saved alongside anything already
  on the device rather than replacing it.

  The original walks a cursor along the strip, and every time a randomly-timed
  countdown expires it writes a bolt of random length forward from that cursor,
  resetting to the start with a long pause once it reaches the end. Everything
  decays continuously, so what you see is a white bolt stuttering along and
  fading behind itself.

  Here the timer is gone: bolts strike on the KICK. Their length comes from how
  hard the bass hit, so a heavy beat throws a long bolt and a light one throws a
  flicker. The strike position is randomised rather than marching from a cursor,
  because a bolt that walks predictably along the bar reads as a progress meter
  and not as lightning. A hard enough kick additionally throws a dim full-bar
  flash, like the sky lighting up behind the strike.

  Kept 1D on purpose, which is a departure from how I handled other conversions.
  On this bar the pixel index snakes -- left to right along each half cuboid's
  upper face, then back along its lower -- so a 1D run does not map to a
  straight line in space. For most effects that is a defect. For lightning it is
  free jaggedness: a bolt breaks across the two faces at unpredictable angles,
  which is exactly what a bolt should do.

  Decay follows the original's `p -= p * fade * dt` form, which is already
  dt-scaled and so holds up across this bar's 10-21 fps swing.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

var sizeCtrl = 0.5          // bolt length
export function sliderBoltSize(v) { sizeCtrl = v }

var fadeCtrl = 0.5          // how fast bolts die away
export function sliderFade(v) { fadeCtrl = v }

var flashCtrl = 0.45        // how readily a hard kick lights the whole bar
export function sliderSkyFlash(v) { flashCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var skyFlash    = 0
export var lastBolt    = 0

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0

pixels = array(pixelCount)
hue = 0.62          // the original's cold blue-white
fade = 15

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  dw = delta / 2000

  // ---- Loudest bin, for tinting the bolt ----
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

  // ---- Decay everything ----
  // The original's form, which is already dt-scaled. The tiny constant subtract
  // is what stops a nearly-dead bolt lingering forever at a value too small to
  // see but large enough to keep the pixel technically lit.
  f = (4 + fadeCtrl * 22) * dt
  for (i = 0; i < pixelCount; i++) {
    p = pixels[i]
    if (p > 0) {
      p -= p * f + 0.0002
      pixels[i] = p > 0 ? p : 0
    }
  }

  // ---- Strike ----
  if (beat) {
    beatCount++
    power = min(bassSpike, 1)

    // Length scales with the hit. Bolts are struck at a random position rather
    // than marched along from a cursor, which would read as a progress bar.
    boltMin = floor(pixelCount / 40)
    boltLen = boltMin + floor(power * sizeCtrl * pixelCount / 3)
    start = floor(random(pixelCount - boltLen))
    lastBolt = boltLen

    e = start + boltLen
    i = start
    while (i < e) { pixels[i] = 1; i++ }

    // A hard kick also lights the sky behind the bolt.
    if (power > 1 - flashCtrl * 0.8) skyFlash = power * flashCtrl

    hue = 0.55 + (peakBin / 31) * 0.14   // blue-white, drifting with the spectrum
  }

  skyFlash -= dt * 3.5
  if (skyFlash < 0) skyFlash = 0
}

export function render(index) {
  v = pixels[index] + skyFlash * 0.25
  if (v > 1) v = 1
  // The original renders bolts as pure white; saturation rises as they fade so
  // the dying tail goes blue rather than grey.
  hsv(hue, 1 - v, v)
}
