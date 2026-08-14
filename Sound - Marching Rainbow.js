/*
  Sound - Marching Rainbow

  Sound-reactive variant of the stock Marching Rainbow, saved alongside it rather
  than replacing it.

  The original is a triple-nested wave rainbow viewed through a slowly marching
  interference sieve, driven entirely by two free-running timers:

    t1 = time(.1)     // ~6.6s per cycle -- the rainbow itself
    t2 = time(.05)    // ~3.3s per cycle -- the sieve that marches across it

  Both become accumulators advancing at a rate derived from the detected tempo,
  so the sieve marches in time with the music instead of on its own clock. On
  top of that each kick nudges the sieve forward by a fraction of a cycle, which
  makes the marchers visibly step on the beat rather than merely drifting at the
  right average speed. Overall brightness follows sustained loudness.

  Everything in render() is the stock pattern untouched.

  Deliberately still 1D. On this bar the pixel index snakes -- left to right
  along each half cuboid's upper face, then back along its lower face -- so a 1D
  effect crosses the bar as a ten-leg zigzag. That breaks discrete moving
  objects, but this is a wave field, so the snake reads as texture rather than
  as breakage. Adding render2D would restage it onto the pixel map and change
  the look, which is not the point of a conversion.

  With no sensor board attached (`light` stays -1) it falls back to the stock
  free-running timers.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

// Beats per full march of the sieve. Low = frantic, high = a slow crawl.
var marchCtrl = 0.45
export function sliderBeatsPerMarch(v) { marchCtrl = v }

// How hard each kick kicks the sieve forward.
var stepCtrl = 0.4
export function sliderBeatStep(v) { stepCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var energyNorm  = 0

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0
totAvg       = 0
totPeak      = 0

t1 = 0
t2 = 0

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  dw = delta / 2000

  // ---- Sustained loudness (a level, not a transient) ----
  totalNow = 0
  for (i = 0; i < 32; i++) totalNow += frequencyData[i]
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

  if (light < 0) {
    // No sensor board: the stock free-running timers.
    t1 = time(.1)
    t2 = time(.05)
    return
  }

  // ---- Tempo-driven timers ----
  // beatsPerCycle spans 2..18; the rainbow runs at half the sieve's rate, as in
  // the original where t1's period was double t2's.
  beatsPerMarch = 2 + marchCtrl * 16
  beatsPerSec = detectedBpm / 60

  t2 += dt * beatsPerSec / beatsPerMarch
  t1 += dt * beatsPerSec / (beatsPerMarch * 2)

  if (beat) {
    beatCount++
    t2 += stepCtrl * 0.12   // visible step on the kick
  }

  // wave() takes any input but keep these bounded for the fixed-point maths.
  if (t1 >= 256) t1 -= 256
  if (t2 >= 256) t2 -= 256
}

// ---- Stock Marching Rainbow render, unchanged apart from the brightness trim ----
export function render(index) {
  pct = index / pixelCount // Percent this pixel is into the overall strip

  h = wave(wave(wave(t1 + pct)) - pct)

  w1 = wave(t1 + pct)
  w2 = wave(t2 - pct * 10)

  v = w1 - w2

  // Breathe with sustained loudness, floored so it never goes fully dark.
  v = v * (0.45 + energyNorm * 0.55)

  hsv(h, 1, v)
}
