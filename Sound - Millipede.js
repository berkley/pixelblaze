/*
  Sound - Millipede

  Sound-reactive variant of the stock Millipede, saved alongside it rather than
  replacing it.

  The original captures the wave that travels along a millipede's feet using two
  free-running timers and a remainder trick, with two constants:

    speed = 20        // t1 = time(1/speed), t2 = time(2/speed)
    legs  = 10        // how many "feet" fit on the strip

  Both become audio-driven. The timers turn into accumulators advancing at the
  detected tempo, so the wave crawls in time with the music. `legs` follows the
  SPECTRAL CENTROID -- the energy-weighted average bin -- so bass-heavy music
  gives a few fat slow legs and bright, hi-hat-driven music gives many thin
  ones. The centroid is smoothed, because a leg count that jitters frame to
  frame just looks like noise rather than like the creature changing gait.

  Everything in render() is the stock pattern untouched.

  Deliberately still 1D. On this bar the pixel index snakes -- left to right
  along each half cuboid's upper face, then back along its lower face -- so a 1D
  effect crosses the bar as a ten-leg zigzag. That breaks discrete moving
  objects, but this is a wave field, so the snake reads as texture. Adding
  render2D would restage it onto the pixel map and change the look.

  With no sensor board attached (`light` stays -1) it falls back to the stock
  free-running timers and a fixed leg count.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

// Beats per full crawl cycle. Low = scurrying, high = a slow amble.
var crawlCtrl = 0.4
export function sliderBeatsPerCrawl(v) { crawlCtrl = v }

// How far the leg count is allowed to swing with the spectrum. At 0 it stays
// at the stock 10.
var legSwingCtrl = 0.6
export function sliderLegSwing(v) { legSwingCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var centroid    = 0.5   // 0 = all bass, 1 = all treble

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0

t1 = 0
t2 = 0
legs = 10

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  dw = delta / 2000

  // ---- Spectral centroid ----
  // Energy-weighted mean bin, normalised to 0..1. Falls back to the middle when
  // there is nothing to weigh, so silence does not slam the legs to one end.
  wsum = 0
  esum = 0
  for (i = 0; i < 32; i++) {
    e = frequencyData[i]
    esum += e
    wsum += e * i
  }
  cNow = esum > 0.00002 ? (wsum / esum) / 31 : 0.5
  // Slow smoothing: a leg count that jitters every frame reads as noise.
  centroid = centroid * 0.94 + cNow * 0.06

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

  if (light < 0) {
    // No sensor board: the stock timers and leg count.
    t1 = time(1 / 20)
    t2 = time(2 / 20)
    legs = 10
    return
  }

  // ---- Tempo-driven timers ----
  // t2 runs at half t1's rate, preserving the original's 1:2 relationship.
  beatsPerCrawl = 1 + crawlCtrl * 11
  beatsPerSec = detectedBpm / 60
  t1 += dt * beatsPerSec / beatsPerCrawl
  t2 += dt * beatsPerSec / (beatsPerCrawl * 2)
  if (t1 >= 256) t1 -= 256
  if (t2 >= 256) t2 -= 256

  // ---- Leg count from the centroid ----
  legs = 10 + legSwingCtrl * (centroid - 0.5) * 20   // ~4..16 at full swing
}

// ---- Stock Millipede render, unchanged ----
export function render(index) {
  h = index / pixelCount + wave(t1)
  h += (index / pixelCount + t2) * legs / 2 % .5
  v = wave(h + t2)
  v = v * v // Gamma correction
  hsv(h, 1, v)
}
