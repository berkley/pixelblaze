/*
  Cosmic Giggle Beacon
  Five color beams rotate together like a multi-color lighthouse beacon.
  Each beam is ~5 pixels wide with dark gaps between them on a 32-row cylinder.
*/

t1 = 0

export function beforeRender(delta) {
  t1 = (t1 + delta / 6000) % 1
}

export function render3D(index, x, y, z) {
  angle = (atan2(y, x) / (2 * PI) + 0.5 + t1) % 1

  // 5 beams equally spaced; each beam is 5 pixels wide on a 32-row cylinder
  // posInSection goes 0..1 within each 1/5 slice; beam fills the first 78% (~5px), rest is dark
  section = angle * 5
  beamIndex = floor(section)
  posInSection = section % 1

  if (posInSection < 5 / 32 * 5) {
    hsv(beamIndex / 5, 1, 1)
  } else {
    hsv(0, 0, 0)
  }
}
