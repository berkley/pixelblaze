/*
  Lightbar Boxes 2D

  For the DJ Booth Lightbar (pixelmaps/halfcuboid-bar.js): 5 half cuboids end to
  end, read as a 160x16 banner across the two faces.

  Tiles the whole surface with 4x4 boxes -- 40 across by 4 down, 160 boxes -- each
  a different hue. 4 divides the 8-row faces evenly, so every box sits wholly on
  one face and none straddles the ridge; the middle two rows of boxes meet along
  the corner edge.

  Column and row are recovered by rounding to the nearest of the 160 x 16 known
  pixel positions rather than by binning a continuous coordinate. The mapper
  places pixels at exactly col/159 and (15-row)/15, so rounding lands on the
  right cell every time -- binning with floor(x * 160) would drop the last column
  into a 161st bin when x reaches exactly 1.0.

  Works against either variant of the map: render3D uses x and z, and render2D
  reconstructs z from the flattened 160x16 plane.
*/

barW = 160
barH = 16
boxSize = 4
boxesX = barW / boxSize  // 40

export function render3D(index, x, y, z) {
  col = round(x * (barW - 1))        // 0..159 along the bar
  row = round((1 - z) * (barH - 1))  // 0..15, 0 at the upper outer edge

  bx = floor(col / boxSize)  // 0..39
  by = floor(row / boxSize)  // 0..3

  // Golden-ratio step across, quarter-turn down: adjacent boxes land far apart
  // on the colour wheel in both directions, so the tiling reads as a grid.
  // Kept as small products for the 16.16 fixed-point maths.
  h = (bx * 0.381966 + by * 0.25) % 1

  hsv(h, 1, 1)
}

export function render2D(index, x, y) {
  // On the flattened map y is height across the unrolled banner.
  render3D(index, x, 0, y)
}

export function render(index) {
  render3D(index, index / (pixelCount - 1), 0, 0.5)
}
