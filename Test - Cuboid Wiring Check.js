/*
  Test - Cuboid Wiring Check

  Diagnostic pattern for cuboid.js. Colors each of the 4 walls solid
  (front=red, right=green, back=blue, left=magenta) with a bright white
  band near the top of every face. If the pixel map's panel order,
  serpentine direction, or "starts at top" assumption is wrong, the
  wall colors and/or the white band will land on the wrong physical
  face or edge, which is easy to spot by eye.
*/

export function render3D(index, x, y, z) {
  // Faces meet at corners where |x| == |y|; away from a corner, whichever
  // coordinate is larger in magnitude tells us which pair of walls we're on.
  if (abs(y) >= abs(x)) {
    h = (y < 0) ? 0 : 0.6      // front = red, back = blue
  } else {
    h = (x > 0) ? 0.33 : 0.85  // right = green, left = magenta
  }

  top = z > 0.8  // top ~2 rows of an 8-row-tall panel

  if (top) {
    hsv(0, 0, 1)  // white band marks the top edge
  } else {
    hsv(h, 1, 1)
  }
}

export function render2D(index, x, y) {
  render3D(index, x, y, 0.5)
}

export function render(index) {
  render3D(index, index / pixelCount, 0.5, 0.5)
}
