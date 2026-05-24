/*
  Sound - Spotlights 3D

  Beat-synced variant of the stock Spotlights pattern. The rotation angle
  (= the spin) is locked to a running BPM estimate from detected bass
  beats. `sliderBeatsPerSpin` controls how many beats elapse for one full
  revolution — at slider = 0 it spins every beat, at slider = 1 it takes
  8 beats per full turn.

  The rotation AXIS (t1, t2, t3) still drifts on its own slow triangle
  waves, so the cones still wobble and re-orient continuously between
  beats. Only the spin rate around that axis is audio-driven.

  Falls back to 120 BPM if no SB is connected, so the pattern still
  animates without audio.
*/

scale = 1 / (PI * PI)   // How wide the "spotlights" are

// ---- UI ----
var beatsPerSpinCtrl = 0.4   // 0 = 1 beat/turn, 1 = 8 beats/turn (default ≈ 3.8)
export function sliderBeatsPerSpin(v) { beatsPerSpinCtrl = v }

// Manual BPM. 0 = use auto-detected BPM; otherwise scales 100..150 BPM.
var manualBpmCtrl = 0
export function sliderManualBpm(v) { manualBpmCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics (visible in Vars Watch) ----
export var detectedBpm  = 120
export var bassSpike    = 0
export var beatsPerSpin = 1

// ---- Beat detection state ----
var bassAvg      = 0
var bassPeak     = 0
var lastBassBeat = -10
var elapsed      = 0
var bpmSamples   = 0     // bootstrap counter

// ---- Beat-locked spin phase ----
var spinPhase = 0

export function beforeRender(delta) {
  dt = delta * 0.001
  elapsed += dt

  // ---- Sub-bass beat detection (drives BPM only) ----
  // Bins 0-3 are tight enough that the BPM tracks kicks, not claps.
  bassNow = 0
  for (i = 0; i < 4; i++) bassNow += frequencyData[i]
  bassNow *= 0.25

  dw = delta / 2000
  bassAvg  = bassAvg  * (1 - dw) + bassNow * dw
  bassPeak = bassPeak * 0.997
  if (bassNow > bassPeak) bassPeak = bassNow

  bassDyn   = max(max(bassPeak - bassAvg, bassPeak * 0.2), 0.00005)
  bassSpike = max(0, bassNow - bassAvg) / bassDyn

  // Higher threshold (0.6) so we only fire on near-peak hits — real
  // kicks, not low-level transients between them.
  if (bassSpike > 0.6 && elapsed - lastBassBeat > 60 / 220) {
    if (lastBassBeat > 0) {
      interval = elapsed - lastBassBeat
      if (interval > 0.27 && interval < 1.5) {
        newBpm = 60 / interval
        bpmSamples = bpmSamples + 1

        if (bpmSamples < 8) {
          // Bootstrap: trust raw data so detectedBpm rapidly converges
          // to the real tempo, even if the 120 default is far off.
          detectedBpm = detectedBpm * 0.5 + newBpm * 0.5
        } else {
          // Steady state: snap to nearest 1×/2×/0.5× of current
          // estimate, then only accept if within 25% of current.
          ratio = newBpm / detectedBpm
          if      (ratio > 1.6 && ratio < 2.5)  newBpm = newBpm * 0.5
          else if (ratio > 0.4 && ratio < 0.62) newBpm = newBpm * 2
          if (abs(newBpm - detectedBpm) / detectedBpm < 0.25) {
            detectedBpm = detectedBpm * 0.8 + newBpm * 0.2
          }
        }
      }
    }
    lastBassBeat = elapsed
  }

  // Manual BPM override. Slider at 0 = use auto-detected BPM.
  // Any non-zero value scales linearly across 100..150 BPM.
  if (manualBpmCtrl > 0) {
    detectedBpm = 100 + manualBpmCtrl * 50
  }

  // ---- Beat-locked spin ----
  // spinPhase advances by 1 full rotation every `beatsPerSpin` beats.
  beatsPerSpin = 1 + beatsPerSpinCtrl * 7
  spinPhase += dt * detectedBpm / 60 / beatsPerSpin

  // ---- Axis drift (continuous, independent of audio) ----
  // Original used time(.03 / speed) etc; keep that wobble going so the
  // spotlights still slowly re-orient between revolutions.
  t1 = 2 * triangle(time(.03)) - 1
  t2 = 2 * triangle(time(.04)) - 1
  t3 = 2 * triangle(time(.05)) - 1

  setupRotationMatrix(t1, t2, t3, spinPhase * PI2)
}

export function render3D(index, _x, _y, _z) {
  x = _x - 0.5; y = _y - 0.5; z = _z - 0.5

  rotate3D(x, y, z)

  dist = abs(rz) - sqrt(rx * rx / scale + ry * ry / scale)
  dist = clamp(dist, -1, 1)

  //  magenta,  white center,  sub-pixel rendered border
  hsv(0.97, 1 - dist, pow((1 + dist), 4))
}

export function render2D(index, x, y) {
  render3D(index, x, y, 0)
}

export function render(index) {
  render3D(index, index / pixelCount * 2, 0, 0)
}

// ---- Rotation matrix helpers (unchanged from the original) ----

var R = array(3); for (i = 0; i < 3; i++) R[i] = array(3)

function setupRotationMatrix(ux, uy, uz, angle) {
  length = sqrt(ux * ux + uy * uy + uz * uz)
  ux /= length; uy /= length; uz /= length

  cosa = cos(angle); sina = sin(angle)
  ccosa = 1 - cosa
  xyccosa = ux * uy * ccosa
  xzccosa = ux * uz * ccosa
  yzccosa = uy * uz * ccosa
  xsina = ux * sina; ysina = uy * sina; zsina = uz * sina

  R[0][0] = cosa + ux * ux * ccosa
  R[0][1] = xyccosa - zsina
  R[0][2] = xzccosa + ysina
  R[1][0] = xyccosa + zsina
  R[1][1] = cosa + uy * uy * ccosa
  R[1][2] = yzccosa - xsina
  R[2][0] = xzccosa - ysina
  R[2][1] = yzccosa + xsina
  R[2][2] = cosa + uz * uz * ccosa
}

var rx, ry, rz
function rotate3D(x, y, z) {
  rx = R[0][0] * x + R[0][1] * y + R[0][2] * z
  ry = R[1][0] * x + R[1][1] * y + R[1][2] * z
  rz = R[2][0] * x + R[2][1] * y + R[2][2] * z
}
