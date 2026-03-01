//Mimics the CG beacon with green, blue, cyan, pink and yellow bands
//will render a 2D surface with appropriate pixel map.

// Master controls (0..1 sliders)
export var brightness = 0.8
export var speedCtrl = 0.25   // 0..1 mapped to 0..2 rev/sec
export var direction = 1      // 1 or -1 (type it in)

// Wiring/layout
export var serpentine = 1     // 1 = snaked rows, 0 = straight

// Band RGB controls (0..1 sliders each)
export var r0 = 0.0, g0 = 1.0, b0 = 0.0
export var r1 = 0.0, g1 = 1.0, b1 = 1.0
export var r2 = 0.0, g2 = 0.0, b2 = 1.0
export var r3 = 1.0, g3 = 0.0, b3 = 1.0
export var r4 = 1.0, g4 = 0.5, b4 = 0.0

var W = 32
var phase = 0 // 0..1

function frac1(x) { return x - floor(x) }

export function beforeRender(delta) {
  // Map speedCtrl (0..1) to 0..2 revolutions per second
  var revsPerSecond = 2 * speedCtrl

  phase = frac1(phase + direction * revsPerSecond * (delta * 0.001))
}

function setBandRGB(band) {
  if (band == 0) rgb(r0 * brightness, g0 * brightness, b0 * brightness)
  else if (band == 1) rgb(r1 * brightness, g1 * brightness, b1 * brightness)
  else if (band == 2) rgb(r2 * brightness, g2 * brightness, b2 * brightness)
  else if (band == 3) rgb(r3 * brightness, g3 * brightness, b3 * brightness)
  else rgb(r4 * brightness, g4 * brightness, b4 * brightness)
}

function renderDrum(xTurns) {
  var p = frac1(xTurns + phase)
  var band = floor(p * 5) // 0..4
  setBandRGB(band)
}

// 32x8 Cylinder Drum: 5 adjacent full-height color bands, always on
export function render2D(index, x, y) {
  renderDrum(x)
}

export function render(index) {
  // fallback mapping if no 2D layout is configured
  var x = index % W
  var y = floor(index / W)
  if (serpentine && (y % 2 == 1)) x = (W - 1) - x

  renderDrum(x / (W - 1))
}
