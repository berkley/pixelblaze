/*
  Sound - Oasis

  Sound-reactive version of "Oasis" by JEM (ZRanger1), from the ElectroMage
  pattern library (26 votes, https://patterns.electromage.com), itself inspired
  by FastLED's Pacifica. Saved alongside anything already on the device rather
  than replacing it.

  The original sums four travelling wave layers of different wavelengths and
  speeds, two moving each way, through a gamma lookup table. The interference
  between them is what produces the slow, water-like light.

  The look is untouched -- same four layers, same gamma table, same summation.
  What changed is what drives them:

    - The layers' phases were free-running time() calls. They are now
      accumulators advancing at rates derived from the detected tempo, so the
      water moves with the music rather than beside it.
    - Whitecaps rise on the kick, so the surface foams on the beat and settles.
    - The base hue slides with the spectral centroid: bass-heavy music sits
      green-blue, brighter music moves toward cyan.
    - Overall depth follows sustained loudness.

  This is the calm one in the set -- it still reacts, but it is meant to be
  watchable between sets rather than during them.

  Kept 1D, as the original is. On this bar the index snakes across the two
  faces, which for a wave field reads as texture rather than as breakage -- the
  same reasoning that kept Marching Rainbow and Millipede in 1D.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

var speedCtrl = 0.5         // how fast the water moves, in beats per traverse
export function sliderSpeed(v) { speedCtrl = v }

var capsCtrl = 0.5          // how much the kick foams the surface
export function sliderWhitecaps(v) { capsCtrl = v }

var wavelenCtrl = 0.5       // wavelength scale
export function sliderWavelength(v) { wavelenCtrl = v; setup() }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var energyNorm  = 0
export var centroid    = 0.5
export var foam        = 0

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0
totAvg       = 0
totPeak      = 0

// ---- Wave layers ----
// Four layers: wavelength divisor, travel direction, and a phase this pattern
// advances itself rather than reading from time().
lDiv  = array(4)
lDir  = array(4)
lRate = array(4)     // relative speed, scaled to tempo at run time
lPh   = array(4)
lOff  = array(4)     // phase in pixels, recomputed each frame

gamma = array(512)
baseHue = 0.5
whiteCaps = 1.46
depth = 0.65

function setup() {
  wavelenScale = 0.15 + 2 * (1 - wavelenCtrl)
  // Same four layers as the original: two forward, two reverse, with the
  // wavelengths scaled so density holds regardless of strip length.
  lDiv[0] = 21 * wavelenScale; lDir[0] =  1; lRate[0] = 1 / 10
  lDiv[1] =  9 * wavelenScale; lDir[1] =  1; lRate[1] = 1 / 6
  lDiv[2] = 11 * wavelenScale; lDir[2] = -1; lRate[2] = 1 / 15
  lDiv[3] =  5 * wavelenScale; lDir[3] = -1; lRate[3] = 1 / 22
}

// gamma lookup: the original's shaping of each wave into sharp crests
offs = -PI / 2
for (gi = 0; gi < 512; gi++) gamma[gi] = pow(wave(offs + (gi / 512)), 4)

setup()

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  dw = delta / 2000

  // ---- Loudness and spectral centroid ----
  totalNow = 0
  wsum = 0
  for (i = 0; i < 32; i++) {
    e = frequencyData[i]
    totalNow += e
    wsum += e * i
  }
  cNow = totalNow > 0.00002 ? (wsum / totalNow) / 31 : 0.5
  centroid = centroid * 0.97 + cNow * 0.03
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
  if (onset) { beatPhase = 0; beatCount++; foam = min(bassSpike, 1) }
  else if (beatPhase >= 1) { beatPhase -= 1; beatCount++; foam = min(bassSpike, 1) }

  foam -= dt / (60 / detectedBpm * 0.8)
  if (foam < 0) foam = 0

  // ---- Advance the layers ----
  // beatsPerTraverse is how many beats the slowest layer takes to cross the
  // strip; the others keep their original relative rates.
  beatsPerTraverse = 6 + (1 - speedCtrl) * 26
  beatsPerSec = detectedBpm / 60
  for (i = 0; i < 4; i++) {
    lPh[i] += dt * beatsPerSec / beatsPerTraverse * lRate[i] * 10
    if (lPh[i] >= 1) lPh[i] -= 1
    p = lDir[i] > 0 ? lPh[i] : 1 - lPh[i]
    lOff[i] = pixelCount * p
  }

  // The original wobbled layer 4's wavelength on a slow triangle; here that
  // wobble follows loudness instead.
  lDiv[3] = (5 * (0.15 + 2 * (1 - wavelenCtrl))) * (0.9 + energyNorm * 0.25)

  baseHue = 0.42 + centroid * 0.18
  whiteCaps = 1 + (1 - capsCtrl) - foam * capsCtrl * 0.9
  depth = 0.35 + energyNorm * 0.5
}

// Sum of all four waves at a pixel, gamma corrected -- the original's core.
function gammatron(index) {
  n = ((index + lOff[0]) * lDiv[0]) / pixelCount
  v = gamma[511 * (n % 1)]
  n = ((index + lOff[1]) * lDiv[1]) / pixelCount
  v += gamma[511 * (n % 1)]
  n = ((index + lOff[2]) * lDiv[2]) / pixelCount
  v += gamma[511 * (n % 1)]
  n = ((index + lOff[3]) * lDiv[3]) / pixelCount
  v += gamma[511 * (n % 1)]
  return v / 4
}

export function render(index) {
  v = gammatron(index)
  h = baseHue - (depth * v * 0.3)
  s = whiteCaps - v
  hsv(h, s, v)
}
