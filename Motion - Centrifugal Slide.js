/*
  Centrifugal Slide — Accelerometer-driven outward rainbow flow
  Spinning the ring pushes rainbow colors from center (pixel 29)
  toward both ends, like centrifugal force.
*/

export var accelerometer

// --- Sliders ---
var saturation = 1
export function sliderSaturation(v) {
  saturation = v
}

var speedMul = 1
export function sliderSpeed(v) {
  speedMul = 0.1 + v * 4.9
}

// --- State ---
CENTER = 29
NUM = 59

hues = array(NUM)
vals = array(NUM)

// Initialize with dim rainbow
for (i = 0; i < NUM; i++) {
  hues[i] = i / NUM
  vals[i] = 0.1
}

prevAngle = 0
spinSpeed = 0
smoothSpin = 0
initialized = 0

export function beforeRender(delta) {
  dt = delta / 1000

  // Compute angle of acceleration vector in XY plane
  angle = atan2(accelerometer[1], accelerometer[0])

  if (initialized) {
    // Delta angle with wraparound handling
    da = angle - prevAngle
    if (da > PI) da -= PI2
    if (da < -PI) da += PI2

    // Smooth the angular velocity
    spinSpeed = da / dt
    smoothSpin = mix(smoothSpin, spinSpeed, 0.15)
  }
  initialized = 1
  prevAngle = angle

  // Force magnitude drives the slide
  force = abs(smoothSpin)

  // Blend amount: how much each pixel shifts outward this frame
  blend = clamp(force * speedMul * dt * 0.15, 0, 0.95)

  // Slide left half outward: indices 0..28 pull from toward center
  for (i = 0; i < CENTER; i++) {
    hues[i] = mix(hues[i], hues[i + 1], blend)
    vals[i] = mix(vals[i], vals[i + 1], blend)
  }

  // Slide right half outward: indices 58..30 pull from toward center
  for (i = NUM - 1; i > CENTER; i--) {
    hues[i] = mix(hues[i], hues[i - 1], blend)
    vals[i] = mix(vals[i], vals[i - 1], blend)
  }

  // Inject cycling rainbow at center
  rainbowHue = time(0.05)
  hues[CENTER] = rainbowHue
  vals[CENTER] = 1

  // Gentle brightness fade at the very ends
  vals[0] = vals[0] * 0.95
  vals[NUM - 1] = vals[NUM - 1] * 0.95
}

export function render(index) {
  hsv(hues[index], saturation, vals[index] * vals[index])
}
