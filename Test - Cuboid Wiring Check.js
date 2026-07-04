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
  // Pixelblaze normalizes mapper output to 0..1 world units per axis, so we
  // can't test the original map's negative coordinates directly. Instead,
  // find which axis is "pinned" to a wall (sitting at the 0 or 1 extreme)
  // vs which axis sweeps across the face (the other axis, spanning 0..1).
  ex = min(x, 1 - x)  // how close x is to an edge (0 = right at an edge)
  ey = min(y, 1 - y)  // how close y is to an edge

  if (ex < ey) {
    h = (x < 0.5) ? 0.85 : 0.33  // left = magenta, right = green
  } else {
    h = (y < 0.5) ? 0 : 0.6      // front = red, back = blue
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
