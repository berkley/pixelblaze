/*
  Centrifugal Slide — Accelerometer-driven pulse rings
  Spinning the ring spawns vibrant color pulses at center (pixel 29)
  that travel outward toward both ends.
*/

export var accelerometer

// --- Sliders ---
var saturation = 1
export function sliderSaturation(v) {
  saturation = v
}

var speedMul = 1
export function sliderSpeed(v) {
  speedMul = 0.2 + v * 4.8
}

// --- Constants ---
CENTER = 29
NUM = 59
MAX_PULSES = 16
PULSE_WIDTH = 2.5

// --- Pulse pool ---
// Each pulse: [position, hue, brightness, active]
pPos = array(MAX_PULSES)
pHue = array(MAX_PULSES)
pBri = array(MAX_PULSES)
pAct = array(MAX_PULSES)

// Per-pixel output buffer
pixH = array(NUM)
pixV = array(NUM)

// --- Spin detection state ---
prevAngle = 0
smoothSpin = 0
initialized = 0
spawnAccum = 0

export function beforeRender(delta) {
  dt = delta / 1000

  // Compute angle of acceleration vector in XY plane
  angle = atan2(accelerometer[1], accelerometer[0])

  if (initialized) {
    da = angle - prevAngle
    if (da > PI) da -= PI2
    if (da < -PI) da += PI2

    spinSpeed = da / dt
    smoothSpin = mix(smoothSpin, spinSpeed, 0.3)
  }
  initialized = 1
  prevAngle = angle

  force = abs(smoothSpin)

  // Spawn pulses based on spin intensity
  // Low threshold so gentle motion triggers pulses
  spawnRate = clamp(force * 0.8, 0, 12)
  spawnAccum += spawnRate * dt

  while (spawnAccum >= 1) {
    spawnAccum -= 1
    // Find inactive slot and spawn a pulse
    for (j = 0; j < MAX_PULSES; j++) {
      if (pAct[j] == 0) {
        pPos[j] = 0
        pHue[j] = random(1)
        pBri[j] = 1
        pAct[j] = 1
        break
      }
    }
  }

  // Advance and fade pulses
  pulseSpeed = (5 + force * 3) * speedMul
  for (j = 0; j < MAX_PULSES; j++) {
    if (pAct[j]) {
      pPos[j] += pulseSpeed * dt
      pBri[j] = max(0, pBri[j] - dt * 0.6)
      // Deactivate when past the end or fully faded
      if (pPos[j] > CENTER || pBri[j] <= 0) {
        pAct[j] = 0
      }
    }
  }

  // Clear pixel buffer
  for (i = 0; i < NUM; i++) {
    pixH[i] = 0
    pixV[i] = 0
  }

  // Render pulses into pixel buffer
  for (j = 0; j < MAX_PULSES; j++) {
    if (pAct[j] == 0) continue
    pos = pPos[j]
    h = pHue[j]
    bri = pBri[j] * pBri[j]

    // Light pixels on both sides of center
    for (k = -PULSE_WIDTH; k <= PULSE_WIDTH; k++) {
      dist = abs(k)
      falloff = 1 - dist / (PULSE_WIDTH + 0.5)
      if (falloff <= 0) continue
      intensity = bri * falloff * falloff

      // Left side of center
      idx = CENTER - floor(pos + 0.5) + k
      if (idx >= 0 && idx < NUM) {
        if (intensity > pixV[idx]) {
          pixH[idx] = h
          pixV[idx] = intensity
        }
      }

      // Right side of center
      idx = CENTER + floor(pos + 0.5) + k
      if (idx >= 0 && idx < NUM && floor(pos + 0.5) > 0) {
        if (intensity > pixV[idx]) {
          pixH[idx] = h
          pixV[idx] = intensity
        }
      }
    }
  }
}

export function render(index) {
  hsv(pixH[index], saturation, pixV[index])
}
