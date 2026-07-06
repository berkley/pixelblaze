/*
  Test - Cuboid Wiring Check

  Diagnostic pattern for cuboid.js. Colors each of the 4 walls solid
  (front=red, right=green, back=blue, left=magenta) with a bright white
  band near the top of every face. If the pixel map's panel order,
  serpentine direction, or "starts at top" assumption is wrong, the
  wall colors and/or the white band will land on the wrong physical
  face or edge, which is easy to spot by eye.
*/

panelPixels = pixelCount / 4

export function render3D(index, x, y, z) {
  // Deriving the wall from x/y is unreliable right at a seam corner: both
  // coordinates sit near an edge simultaneously there, and Pixelblaze's
  // mapper normalization lands them just shy of 0/1 (values are "0..1
  // exclusive"), so tiny float noise flips which wall a corner pixel is
  // classified as. The panel index has no such ambiguity, so use that.
  panel = floor(index / panelPixels)
  if (panel == 0)      { h = 0    }  // front = red
  else if (panel == 1) { h = 0.33 }  // right = green
  else if (panel == 2) { h = 0.6  }  // back = blue
  else                 { h = 0.85 }  // left = magenta

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
