// POV Smiley Face
// Persistence of vision pattern for a spinning staff with 59 pixels.
// When you spin the staff, a yellow smiley face appears in the air.
//
// Use the Speed slider to match the animation rate to your spin speed.
// The face is rendered geometrically: yellow circle with black eyes and
// a black arc mouth.

var center = floor(pixelCount / 2)
var maxR = max(center, 1)
var PI2 = PI * 2

// Smiley face geometry (normalized -1 to 1 coordinate space)
var faceR = 0.88        // Face circle radius
var eyeR = 0.13         // Eye radius
var eyeX = 0.3          // Eye horizontal offset from center
var eyeY = 0.28         // Eye vertical offset above center
var mouthCY = 0.12      // Mouth arc center, below face center
var mouthR1 = 0.25      // Mouth arc inner radius
var mouthR2 = 0.42      // Mouth arc outer radius

// Rotation speed - adjust with slider to match your actual spin rate
var speed = 0.03

export function sliderSpeed(v) {
  speed = 0.01 + v * 0.08
}

var angle

export function beforeRender(delta) {
  angle = time(speed) * PI2
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

  // Outside face circle - pixel off
  var d2 = x * x + y * y
  if (d2 > faceR * faceR) {
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

  // Inside face - yellow
  rgb(1, 0.8, 0)
}
