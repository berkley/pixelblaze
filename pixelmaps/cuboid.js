// Pixel map for a 4-sided, closed-wall LED cuboid.
// 4 panels wired sequentially. Each panel is one wall: an 8x32 LED matrix
// mounted "portrait" -- 8 pixels wide (around the loop) x 32 pixels tall --
// so adjacent panels touch along their long (32-pixel) vertical edges at the
// cuboid corners. Each panel is wired as 32 horizontal rows of 8 pixels,
// serpentine (consecutive rows alternate direction). Panels are assigned to
// the 4 walls in order (front, left, back, right) going clockwise (viewed
// from above), so the wiring runs continuously around the loop with matching
// seams at each corner.
//
// entersAtTop[] flips the vertical direction per panel for the walls whose
// first wired row sits at the top rather than the bottom; calibrated against
// the wiring-check test pattern on the real cuboid.

function (pixelCount) {
  var wallW = 8       // pixels around the loop, per wall (short matrix axis)
  var wallH = 32      // pixels tall (long matrix axis)
  var panelPixels = wallW * wallH  // 256
  var halfSize = wallW / 2         // footprint half-width, in pixel units
  var entersAtTop = [false, true, false, true]  // per panel: front, left, back, right
  var map = []

  for (i = 0; i < pixelCount; i++) {
    panel = Math.floor(i / panelPixels)
    local = i % panelPixels
    strip = Math.floor(local / wallW)  // 0..31, which horizontal row (height)
    idx = local % wallW                // 0..7, position along the row

    // serpentine: consecutive 8-pixel rows alternate direction
    hpos = (strip % 2 == 0) ? idx : (wallW - 1 - idx)
    u = hpos / (wallW - 1)             // 0..1 across the 8-pixel wall width

    z = strip / (wallH - 1)            // 0 at first row, 1 at last
    if (!entersAtTop[panel]) z = 1 - z

    if (panel == 0) {        // front wall: y = -halfSize
      x = halfSize - u * (2 * halfSize)
      y = -halfSize
    } else if (panel == 1) {  // left wall: x = -halfSize
      x = -halfSize
      y = -halfSize + u * (2 * halfSize)
    } else if (panel == 2) {  // back wall: y = +halfSize
      x = -halfSize + u * (2 * halfSize)
      y = halfSize
    } else {                  // right wall: x = +halfSize
      x = halfSize
      y = halfSize - u * (2 * halfSize)
    }

    map.push([x, y, z])
  }
  return map
}
