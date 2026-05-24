/*
  Sound - Fast Pulse 3D

  Sound-reactive variant of the stock Fast Pulse pattern. The phase that
  sweeps parallel layers through 3D space is locked to a running BPM
  estimate from detected bass beats. `sliderBeatsPerLayer` controls how
  many beats elapse between consecutive layers passing through — at
  slider = 0 a layer crosses on every beat, at slider = 1 it takes 8.

  If no SB is detected (or the music is silent) the BPM estimate sits
  at its 120 BPM default and the animation just runs at that tempo.

  Original visual:
    v = triangle(3 * wave(t1) + a*x + b*y + c*z)
  Sound-reactive replacement (linear, beat-locked phase):
    v = triangle(layerPhase + a*x + b*y + c*z)

  Spatial rotation (a/b/c) is unchanged and runs on its own slow
  oscillators, so the cube keeps tumbling continuously between beats.
*/

// ---- UI control ----
// Pixelblaze creates UI sliders from exported `slider*` *functions*, not
// vars. The function is invoked with the slider's current 0..1 value
// whenever it moves.
var beatsPerLayerCtrl = 0   // 0..1, set by the slider below
export function sliderBeatsPerLayer(v) { beatsPerLayerCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm   = 120
export var bassSpike     = 0
export var beatsPerLayer = 1

// ---- State ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
layerPhase   = 0

t1 = 0
a  = 0
b  = 0
c  = 0

export function beforeRender(delta) {
  dt = delta * 0.001
  elapsed += dt

  // ---- Bass beat detection (drives BPM only — no spawning) ----
  // Same peak-relative spike approach as the Cube Frequency Bands
  // pattern: works across any volume of music on the SB 1.0.
  bassNow = 0
  for (i = 0; i < 8; i++) bassNow += frequencyData[i]
  bassNow *= 0.125

  dw = delta / 2000
  bassAvg  = bassAvg  * (1 - dw) + bassNow * dw
  bassPeak = bassPeak * 0.997
  if (bassNow > bassPeak) bassPeak = bassNow

  bassDyn   = max(max(bassPeak - bassAvg, bassPeak * 0.2), 0.00005)
  bassSpike = max(0, bassNow - bassAvg) / bassDyn

  if (bassSpike > 0.45 && elapsed - lastBassBeat > 60 / 220) {
    if (lastBassBeat > 0) {
      interval = elapsed - lastBassBeat
      if (interval > 0.27 && interval < 1.5) {
        newBpm = 60 / interval
        detectedBpm = detectedBpm * 0.6 + newBpm * 0.4
      }
    }
    lastBassBeat = elapsed
  }

  // ---- Beat-locked layer phase ----
  // One full triangle cycle (one layer) every `beatsPerLayer` beats.
  beatsPerLayer = 1 + beatsPerLayerCtrl * 7
  layerPhase += dt * detectedBpm / 60 / beatsPerLayer

  // ---- Original animation state (rotation + hue) ----
  t1 = time(.1)
  a  = sin(time(.10) * PI2)
  b  = sin(time(.05) * PI2)
  c  = sin(time(.07) * PI2)
}

export function render3D(index, x, y, z) {
  v = triangle(layerPhase + a * x + b * y + c * z)
  v = pow(v, 5)
  s = v < .8
  hsv(t1, s, v)
}

export function render2D(index, x, y) {
  render3D(index, x, y, 0)
}

export function render(index) {
  v = triangle(layerPhase + index / pixelCount)
  v = pow(v, 5)
  s = v < .9
  hsv(t1, s, v)
}
