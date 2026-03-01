export var accelerometer
export var smoothAx
export var pos

var sensitivity = 100  // lower = faster. Tune to taste.

export function sliderSensitivity(v) {
  sensitivity = 10 + v * 490  // slider maps 0..1 to range 10..500
}

pos = 0
smoothAx = 0

export function beforeRender(delta) {
  smoothAx = mix(smoothAx, accelerometer[1], 0.1)
  pos = pos + smoothAx * delta / sensitivity
  pos = mod(pos, 1)
  t1 = pos
}

export function render(index) {
  pct = index / pixelCount
  h = pct * (5 * wave(t1) + 5) + 2 * wave(t1)
  h = h % .5 + t1
  v = triangle(5 * pct + 10 * t1)
  v = v * v * v
  hsv(h, 1, v)
}
