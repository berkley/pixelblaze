/*
  Spin cycle - Accelerometer controlled + Shake explosion
*/
export var accelerometer

var sensitivity = 100
export function sliderSensitivity(v) {
  sensitivity = 10 + v * 490
}

var shakeThresh = 1.5
export function sliderShakeThreshold(v) {
  shakeThresh = v * 5
}

var explodeDecay = 2.5
export function sliderExplodeDecay(v) {
  explodeDecay = v * 5
}

var explodeAmount = 1
export function sliderExplodeAmount(v) {
  explodeAmount = v
}

// Shake detection
SHAKE_COOLDOWN = 1.0   // seconds before re-triggering

pos = 0
smoothAx = 0
smoothMag = 0
shakeCooldown = 0
explodeAmt = 0
explodePhase = 0

export function beforeRender(delta) {
  dt = delta / 1000

  // Spin cycle
  smoothAx = mix(smoothAx, accelerometer[1], 0.1)
  pos = pos + smoothAx * delta / sensitivity
  pos = mod(pos, 1)
  t1 = pos

  // Shake detection — spike in total magnitude above slow baseline
  mag = hypot3(accelerometer[0], accelerometer[1], accelerometer[2])
  smoothMag = mix(smoothMag, mag, 0.02)
  shakeCooldown = max(0, shakeCooldown - dt)

  if (shakeCooldown <= 0 && mag > smoothMag * shakeThresh) {
    explodeAmt = explodeAmount
    explodePhase = random(1)    // random hue offset each explosion
    shakeCooldown = SHAKE_COOLDOWN
  }

  // Rainbow spins faster at peak, slows as it fades
  explodePhase = mod(explodePhase + explodeAmt * dt * 0.5, 1)

  explodeAmt = max(0, explodeAmt - dt / explodeDecay)
}

export function render(index) {
  pct = index / pixelCount

  // Base spin cycle
  h = pct * (5 * wave(t1) + 5) + 2 * wave(t1)
  h = h % .5 + t1
  v = triangle(5 * pct + 10 * t1)
  v = v * v * v

  // Explosion: white flash → rainbow → fade
  eH = pct + explodePhase
  eS = min(1, (1 - explodeAmt) * 2)  // white at peak, full color by halfway
  eV = explodeAmt * explodeAmt        // sharp bright burst

  h = mix(h, eH, explodeAmt)
  s = mix(1, eS, explodeAmt)
  v = min(1, v + eV)

  hsv(h, s, v)
}
