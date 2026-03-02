// POV Smiley Face — Accelerometer-driven
// Persistence of vision pattern for a spinning staff with 59 pixels.
// Uses the accelerometer to track spin angle and measure speed.
// Brightness scales with spin speed to compensate for POV duty cycle.
//
// Note: This pattern uses render(index) rather than render2D because
// the physical radial position (distance from staff center) is needed
// for the filled face, which the ring pixel map doesn't provide.

export var accelerometer

var center = floor(pixelCount / 2)
var maxR = max(center, 1)
var PI2 = PI * 2

// Smiley face geometry (normalized -1 to 1 coordinate space)
var faceR = 0.88        // Face circle radius
var eyeR = 0.13         // Eye radius
var eyeX = 0.3          // Eye horizontal offset from center
var eyeY = 0.28         // Eye vertical offset above center
var mouthCY = 0.05      // Mouth arc center, just below face center
var mouthR1 = 0.15      // Mouth arc inner radius
var mouthR2 = 0.30      // Mouth arc outer radius

// Accelerometer rotation tracking
var sensitivity = 100
export function sliderSensitivity(v) {
  sensitivity = 10 + v * 490
}

var pos = 0
var smoothAx = 0
var spinSpeed = 0
var bright = 1
var angle

export function beforeRender(delta) {
  // Track spin angle from accelerometer
  smoothAx = mix(smoothAx, accelerometer[1], 0.1)
  pos += smoothAx * delta / sensitivity
  pos = mod(pos, 1)
  angle = pos * PI2

  // Measure spin speed (smoothed magnitude of accelerometer)
  spinSpeed = mix(spinSpeed, abs(accelerometer[1]), 0.05)

  // Brightness scales with speed: dim when still, bright when spinning fast.
  // This compensates for the POV duty cycle — faster spin means each pixel
  // is visible for less time, so it needs to be brighter.
  bright = clamp(0.2 + spinSpeed * 3, 0.2, 1)
}

export function render(index) {
  // Radial position: distance from staff center, normalized 0-1
  var offset = index - center
  var r = abs(offset) / maxR

  // Pixels on opposite sides of center point in opposite directions
  var a = angle
  if (offset < 0) a += PI

  // Convert polar to cartesian
  var x = r * cos(a)
  var y = r * sin(a)

  // Outside face circle — pixel off
  if (x * x + y * y > faceR * faceR) {
    rgb(0, 0, 0)
    return
  }

  // Left eye
  var dx = x + eyeX
  var dy = y - eyeY
  if (dx * dx + dy * dy < eyeR * eyeR) {
    rgb(0, 0, 0)
    return
  }

  // Right eye
  dx = x - eyeX
  dy = y - eyeY
  if (dx * dx + dy * dy < eyeR * eyeR) {
    rgb(0, 0, 0)
    return
  }

  // Mouth: arc below center
  dy = y + mouthCY
  var md2 = x * x + dy * dy
  if (md2 > mouthR1 * mouthR1 && md2 < mouthR2 * mouthR2 && y < -mouthCY) {
    rgb(0, 0, 0)
    return
  }

  // Yellow face — brightness scales with spin speed
  rgb(bright, bright * 0.8, 0)
}
