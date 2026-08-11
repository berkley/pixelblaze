// Pixel map for a long bar built from 5 "half cuboids" mounted end to end.
//
// Each half cuboid is a square-section box with LED panels on only 2 of its 4
// faces -- the two adjacent faces that meet at the edge pointing toward the
// viewer. Each panel is an 8x32 matrix mounted "landscape": 32 pixels along the
// bar's length x 8 pixels across the face. So one half cuboid is 2 x 256 = 512
// pixels, and the whole bar is 5 x 512 = 2560.
//
// Each half cuboid gets its own channel on the 8-channel output expander, but
// the Pixelblaze still addresses all 2560 pixels as one sequential index space,
// so a single map covers the whole bar.
//
// Viewed head-on you look straight down the long edge where the two faces meet,
// both panels visible at once, and the bar reads as a 160 wide x 16 tall banner
// (5 x 32 columns across, 8 rows per face x 2 faces down):
//
//         viewer
//           |
//       row0 \            z = +5.30   upper face, outer edge
//             \
//       row7   V ridge    z = +0.35   \  the two rows flanking
//       row8   ^ ridge    z = -0.35   /  the corner
//             /
//      row15 /            z = -5.30   lower face, outer edge
//
// Each panel is wired as 32 columns of 8 pixels, serpentine (consecutive
// columns alternate direction), so the pixel index advances fastest across the
// 160-pixel length. Within a half cuboid, pixels 0-255 are the upper face and
// 256-511 the lower. Half cuboid 0 (expander channel 0) is at the left end, and
// each column enters at the face's OUTER edge and runs inward to the ridge.
//
// Calibrated 2026-08-10 against the real bar ("DJ Booth Lightbar",
// 192.168.68.128, v3.51, 2560 px) using "Test - Halfcuboid Bar Wiring Check".
// The expander confirms the sequential indexing this map assumes: channels 0-4,
// 512 pixels each, startIndex 0 / 512 / 1024 / 1536 / 2048.
//
// Pixel centers sit half a pixel out from the ridge rather than on it, so the
// two rows flanking the corner (7 and 8) land at distinct heights instead of
// collapsing onto z = 0.
//
// NOTE: Pixelblaze's mapper normalizes each axis independently to 0..1, so the
// absolute units and the 45-degree scale factor below wash out before a pattern
// sees them -- only relative placement within each axis survives. The units are
// kept honest here for readability.

function (pixelCount) {
  // ---- calibration flags -------------------------------------------------
  // These values are calibrated against the real bar -- don't change them
  // unless the hardware changes. If it does, run "Test - Halfcuboid Bar Wiring
  // Check" and flip whichever come out wrong. Only the two *EntersAtRidge flags
  // differed from the expected build.

  // Emit a flat 160x16 plane instead of the 3D chevron. The 3D map's y (depth)
  // is degenerate -- both faces share the same y for a given row -- so 2D
  // patterns, which see [x, y], would collapse into an 8-deep smear. Set true
  // when running 2D patterns, false for 3D patterns.
  var flatten = false

  var swapFaces = false            // true: pixels 0-255 are the LOWER face
  var upperEntersAtRidge = false   // upper face: is k=0 the ridge pixel (vs. the outer edge)?
  var lowerEntersAtRidge = false   // lower face: same question
  var upperRunsLeftToRight = true  // upper face: is column 0 at the half cuboid's left end?
  var lowerRunsLeftToRight = false // lower face: defaults opposite -- one strip feeding
                                   // both faces doubles back on the second one
  var reverseCuboidOrder = false   // true: half cuboid 0 sits at the RIGHT end
  // ------------------------------------------------------------------------

  var faceW = 32                    // pixels along the bar, per face (long matrix axis)
  var faceH = 8                     // pixels across the face (short matrix axis)
  var facePixels = faceW * faceH    // 256
  var cuboidPixels = facePixels * 2 // 512
  var cuboidCount = 5
  var barW = cuboidCount * faceW    // 160
  var barH = faceH * 2              // 16
  var s = Math.SQRT1_2              // both faces sit at 45 degrees to the viewer

  // Indexed by face: 0 = upper, 1 = lower. Promote either of these to a
  // 5-element array indexed by `cuboid` if one module turns out to be mounted
  // flipped relative to the others.
  var entersAtRidge = [upperEntersAtRidge, lowerEntersAtRidge]
  var runsLeftToRight = [upperRunsLeftToRight, lowerRunsLeftToRight]

  var map = []

  for (i = 0; i < pixelCount; i++) {
    cuboid = Math.floor(i / cuboidPixels)
    if (reverseCuboidOrder) cuboid = cuboidCount - 1 - cuboid

    local = i % cuboidPixels
    face = Math.floor(local / facePixels)   // 0 or 1
    if (swapFaces) face = 1 - face          // 0 = upper, 1 = lower

    fLocal = local % facePixels
    col = Math.floor(fLocal / faceH)        // 0..31, advances along the bar
    k = fLocal % faceH                      // 0..7 within the column

    // serpentine: consecutive 8-pixel columns alternate direction
    if (col % 2 == 1) k = faceH - 1 - k
    // k is now distance out from the ridge, 0 = closest
    if (!entersAtRidge[face]) k = faceH - 1 - k
    if (!runsLeftToRight[face]) col = faceW - 1 - col

    x = cuboid * faceW + col                // 0..159
    row = (face == 0) ? (faceH - 1 - k) : (faceH + k)  // 0..15, 0 at the top

    if (flatten) {
      map.push([x, barH - 1 - row])         // plain 160x16 plane, y up
    } else {
      t = (k + 0.5) * s                     // distance back/out from the ridge
      map.push([x, t, (face == 0) ? t : -t])
    }
  }
  return map
}
