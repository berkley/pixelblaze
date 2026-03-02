/*
  Cosmic Giggle Beacon
  Five color bands rotate around the Z axis like an airport beacon.
  Bands: green=8px, cyan=6px, blue=6px, magenta=6px, yellow=6px (total=32px)
*/

export var brightness = 0.8
export var speedCtrl = 0.25   // 0..1 → 0..2 rev/sec
export var direction = 1      // 1 or -1

export var r0 = 0.0, g0 = 1.0, b0 = 0.0   // green
export var r1 = 0.0, g1 = 1.0, b1 = 1.0   // cyan
export var r2 = 0.0, g2 = 0.0, b2 = 1.0   // blue
export var r3 = 1.0, g3 = 0.0, b3 = 1.0   // magenta
export var r4 = 1.0, g4 = 0.5, b4 = 0.0   // yellow

var W = 32
var phase = 0

// Precompute column-to-band (green=8px, others=6px, total=32px)
var colBand = array(W)
for (var c = 0; c < W; c++) {
  if (c < 8)       colBand[c] = 0  // green  (8px)
  else if (c < 14) colBand[c] = 1  // cyan   (6px)
  else if (c < 20) colBand[c] = 2  // blue   (6px)
  else if (c < 26) colBand[c] = 3  // magenta(6px)
  else             colBand[c] = 4  // yellow (6px)
}

function frac1(v) { return v - floor(v) }

export function beforeRender(delta) {
  phase = frac1(phase + direction * 2 * speedCtrl * (delta * 0.001))
}

function setBandRGB(band) {
  if (band == 0)      rgb(r0*brightness, g0*brightness, b0*brightness)
  else if (band == 1) rgb(r1*brightness, g1*brightness, b1*brightness)
  else if (band == 2) rgb(r2*brightness, g2*brightness, b2*brightness)
  else if (band == 3) rgb(r3*brightness, g3*brightness, b3*brightness)
  else                rgb(r4*brightness, g4*brightness, b4*brightness)
}

export function render3D(index, x, y, z) {
  // XY plane angle around Z axis; subtract 0.5 because Pixelblaze coords are 0..1
  var angle = frac1(atan2(y - 0.5, x - 0.5) / (2 * PI) + 0.5 + phase)
  var col = floor(angle * W)
  setBandRGB(colBand[col])
}
