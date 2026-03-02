/*
  Cosmic Giggle Beacon
  Five equal color bands rotate around a cylinder axis.
  Each band is ~6 pixels wide (5 bands around a 32-row cylinder).
*/

t1 = 0

export function beforeRender(delta) {
  t1 = (t1 + delta / 8000) % 1
}

export function render3D(index, x, y, z) {
  // atan2(y, x) returns -PI..PI; normalize to 0..1 and add rotation offset
  angle = (atan2(y, x) / (2 * PI) + 0.5 + t1) % 1

  // Divide circle into 5 equal bands
  band = floor(angle * 5)
  h = band / 5

  hsv(h, 1, 1)
}
