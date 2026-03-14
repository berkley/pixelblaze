/*
  Centrifugal Slide — Accelerometer-driven pulse rings
  Spinning the ring spawns vibrant color pulses at center (pixel 29)
  that travel outward toward both ends. When idle, pulses orbit the ring.
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

var pulseWidth = 2.5
export function sliderPulseLength(v) {
  pulseWidth = 1 + v * 7
}

var maxPulses = 16
export function sliderMaxPulses(v) {
  maxPulses = floor(4 + v * 20)
}

// Axis pair for spin detection: 0-0.33 = XY, 0.34-0.66 = XZ, 0.67-1 = YZ
var axisA = 0
var axisB = 1
export function sliderAxis(v) {
  if (v < 0.33) {
    axisA = 0; axisB = 1
  } else if (v < 0.66) {
    axisA = 0; axisB = 2
  } else {
    axisA = 1; axisB = 2
  }
}

// --- Constants ---
CENTER = 29
NUM = 59
POOL = 24

// --- Pulse pool ---
// pPos: radial distance from center (active mode)
// pOrb: orbital position on ring 0..NUM (idle mode)
pPos = array(POOL)
pOrb = array(POOL)
pHue = array(POOL)
pBri = array(POOL)
pAct = array(POOL)

// Per-pixel output buffer
pixH = array(NUM)
pixV = array(NUM)

// --- Spin detection state ---
prevAngle = 0
smoothSpin = 0
initialized = 0
spawnAccum = 0
idleSpawnAccum = 0

// Activity level: 1 = spinning, 0 = idle
activity = 0

export function beforeRender(delta) {
  dt = delta / 1000

  // Compute angle of acceleration vector in XY plane
  angle = atan2(accelerometer[axisB], accelerometer[axisA])

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

  // Update activity level — ramps up fast, decays slowly
  targetAct = clamp(force * 0.4, 0, 1)
  if (targetAct > activity) {
    activity = mix(activity, targetAct, 0.4)
  } else {
    activity = mix(activity, targetAct, 0.03)
  }

  idle = 1 - activity

  // --- Active mode: spawn radial pulses from center ---
  spawnRate = clamp(force * 0.8, 0, 12)
  spawnAccum += spawnRate * dt

  while (spawnAccum >= 1) {
    spawnAccum -= 1
    for (j = 0; j < POOL; j++) {
      if (j >= maxPulses) break
      if (pAct[j] == 0) {
        pPos[j] = 0
        pOrb[j] = random(NUM)
        pHue[j] = random(1)
        pBri[j] = 1
        pAct[j] = 1
        break
      }
    }
  }

  // --- Idle mode: slow trickle of orbital pulses ---
  idleSpawnAccum += idle * 1.5 * dt
  while (idleSpawnAccum >= 1) {
    idleSpawnAccum -= 1
    for (j = 0; j < POOL; j++) {
      if (j >= maxPulses) break
      if (pAct[j] == 0) {
        pPos[j] = 0
        pOrb[j] = random(NUM)
        pHue[j] = random(1)
        pBri[j] = 1
        pAct[j] = 1
        break
      }
    }
  }

  // --- Advance pulses ---
  // Radial speed: fast when active
  radialSpeed = (5 + force * 3) * speedMul
  // Orbital speed: slow constant drift
  orbitalSpeed = 3 * speedMul

  for (j = 0; j < POOL; j++) {
    if (pAct[j] == 0) continue

    // Advance radial position (used in active mode)
    pPos[j] += radialSpeed * dt * activity + 1.5 * dt * idle

    // Advance orbital position (used in idle mode)
    pOrb[j] = mod(pOrb[j] + orbitalSpeed * dt, NUM)

    // Fade: slower when idle so pulses linger
    fadeRate = mix(0.3, 0.8, activity)
    pBri[j] = max(0, pBri[j] - dt * fadeRate)

    // Deactivate when fully faded, or radial past end in active mode
    if (pBri[j] <= 0) {
      pAct[j] = 0
    } else if (activity > 0.5 && pPos[j] > CENTER) {
      pAct[j] = 0
    }
  }

  // --- Clear pixel buffer ---
  for (i = 0; i < NUM; i++) {
    pixH[i] = 0
    pixV[i] = 0
  }

  // --- Render pulses into pixel buffer ---
  for (j = 0; j < POOL; j++) {
    if (pAct[j] == 0) continue
    h = pHue[j]
    bri = pBri[j] * pBri[j]
    pw = floor(pulseWidth + 0.5)

    // --- Radial rendering (active mode) ---
    if (activity > 0.01) {
      pos = pPos[j]
      radBri = bri * activity

      for (k = -pw; k <= pw; k++) {
        dist = abs(k)
        falloff = 1 - dist / (pulseWidth + 0.5)
        if (falloff <= 0) continue
        intensity = radBri * falloff * falloff

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

    // --- Orbital rendering (idle mode) ---
    if (idle > 0.01) {
      orbCenter = floor(pOrb[j] + 0.5)
      orbBri = bri * idle

      for (k = -pw; k <= pw; k++) {
        dist = abs(k)
        falloff = 1 - dist / (pulseWidth + 0.5)
        if (falloff <= 0) continue
        intensity = orbBri * falloff * falloff

        idx = mod(orbCenter + k, NUM)
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
