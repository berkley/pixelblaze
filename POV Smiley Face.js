// POV Smiley Face — Accelerometer-driven
// Persistence of vision pattern for a spinning staff with 59 pixels.
// Uses atan2 on accelerometer axes to get the absolute angle from gravity.
// This locks the face orientation to gravity — eyes stay up, mouth stays down.
//
// If the face appears rotated, adjust the Phase Offset slider.
// If features don't appear at all, try swapping the accelerometer axes
// in the atan2 call (e.g., accelerometer[0] and accelerometer[2]).

export var accelerometer

var center = floor(pixelCount / 2)
var maxR = max(center, 1)
var PI2 = PI * 2

// Smiley face geometry (normalized -1 to 1 coordinate space)
var faceR = 1            // Face circle radius
var eyeR = 0.13          // Eye radius
var eyeX = 0.3           // Eye horizontal offset from center
var eyeY = 0.28          // Eye vertical offset above center
var mouthCY = 0.05       // Mouth arc center, just below face center
var mouthR1 = 0.15       // Mouth arc inner radius
var mouthR2 = 0.30       // Mouth arc outer radius

// Phase offset to calibrate which direction is "up"
var phaseOffset = 0
export function sliderPhaseOffset(v) {
  phaseOffset = v * PI2
}

var spinSpeed = 0
var bright = 1
var angle
var prevAngle = 0

export function beforeRender(delta) {
  // Absolute angle from gravity using atan2.
  // Uses Y and Z accelerometer axes (perpendicular to the staff axis).
  // This keeps the face locked in space — no drift, no sensitivity tuning.
  angle = atan2(accelerometer[2], accelerometer[1]) + phaseOffset

  // Measure spin speed from how fast the angle changes
  var dAngle = angle - prevAngle
  if (dAngle > PI) dAngle -= PI2
  if (dAngle < -PI) dAngle += PI2
  spinSpeed = mix(spinSpeed, abs(dAngle) / max(delta, 1) * 100, 0.05)
  prevAngle = angle

  // Brightness scales with speed: dim when still, bright when spinning fast.
  // This compensates for the POV duty cycle — faster spin means each pixel
  // is visible for less time, so it needs to be brighter.
  bright = clamp(0.2 + spinSpeed * 0.5, 0.2, 1)
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
    rgb(1, 0, 0)
    return
  }

  // Right eye
  dx = x - eyeX
  dy = y - eyeY
  if (dx * dx + dy * dy < eyeR * eyeR) {
    rgb(1, 0, 0)
    return
  }

  // Mouth: arc below center
  dy = y + mouthCY
  var md2 = x * x + dy * dy
  if (md2 > mouthR1 * mouthR1 && md2 < mouthR2 * mouthR2 && y < -mouthCY) {
    rgb(0, 0, 1)
    return
  }

  // Yellow face — brightness scales with spin speed
  rgb(bright, bright * 0.8, 0)
}
