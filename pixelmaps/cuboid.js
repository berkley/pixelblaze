// Pixel map for a 4-sided, closed-wall LED cuboid.
// 4 panels wired sequentially, each an 8x32 pixel array mounted
// 8 pixels tall x 32 pixels wide. Each panel is wired as 32 vertical
// strips of 8 pixels, serpentine. Panels are assigned to the 4 walls in
// order (front, right, back, left) going counter-clockwise (viewed from
// above), so the wiring runs continuously around the loop with matching
// seams at each corner.
//
// The jumper between panels lands at whichever height is physically
// closest, so entry side alternates per panel rather than always being
// "top": confirmed by wiring-check test pattern (front/left enter at the
// top of their first strip, right/back enter at the bottom).

function (pixelCount) {
  var stripLen = 8    // pixels per vertical strip (panel height)
  var cols = 32       // strips per panel (panel width)
  var panelPixels = stripLen * cols  // 256
  var halfSize = cols / stripLen / 2  // footprint half-width, in strip-height units
  var entersAtTop = [true, false, false, true]  // per panel: front, right, back, left
  var map = []

  for (i = 0; i < pixelCount; i++) {
    panel = Math.floor(i / panelPixels)
    local = i % panelPixels
    col = Math.floor(local / stripLen)  // 0..31, which vertical strip
    row = local % stripLen              // 0..7, position within the strip

    // serpentine: even strips run top-to-bottom, odd strips bottom-to-top
    physRow = (col % 2 == 0) ? row : (stripLen - 1 - row)
    z = (stripLen - 1 - physRow) / (stripLen - 1)  // 0 at bottom, 1 at top
    if (!entersAtTop[panel]) z = 1 - z
    u = col / (cols - 1)  // 0..1 across the face

    if (panel == 0) {        // front wall: y = -halfSize
      x = -halfSize + u * (2 * halfSize)
      y = -halfSize
    } else if (panel == 1) {  // right wall: x = +halfSize
      x = halfSize
      y = -halfSize + u * (2 * halfSize)
    } else if (panel == 2) {  // back wall: y = +halfSize
      x = halfSize - u * (2 * halfSize)
      y = halfSize
    } else {                  // left wall: x = -halfSize
      x = -halfSize
      y = halfSize - u * (2 * halfSize)
    }

    map.push([x, y, z])
  }
  return map
}
