/*
  Test - Halfcuboid Bar Wiring Check

  Diagnostic pattern for pixelmaps/halfcuboid-bar.js -- the 160x16 bar built
  from 5 half cuboids. Every element is a distinct HUE, never a brightness
  difference: a dim band is impossible to pick out by eye on a bar this bright.

  stage 0 (default) shows everything at once:

    upper face  solid RED        wrong face red     -> swapFaces
    lower face  solid GREEN      striped, not solid -> faces aren't contiguous blocks
    ridge rows  solid WHITE      lands on the outer -> *EntersAtRidge
                                 top/bottom edges
    pixels 0-15 solid BLUE       marks where the strip physically enters

  stages 1-7 isolate one element at a time, lighting nothing else, for when the
  combined view is ambiguous. `stage` is an exported variable, so it can be
  driven over the websocket with setActiveVariables({"stage": n}) rather than
  having to click through the web UI.

    1 upper face only        5 entry marker only
    2 lower face only        6 row sweep, top -> bottom
    3 ridge rows only        7 column sweep, left -> right
    4 outer edge rows only

  Stages 3 and 4 are complements: exactly one of them lights the corner edge
  facing the viewer, and the other lights the two far edges. The sweeps confirm
  ordering: stage 6 must walk from the upper outer edge down across the ridge to
  the lower outer edge, stage 7 from the left end to the right.

  Unlike "Test - Cuboid Wiring Check", which keys off floor(index/panelPixels)
  precisely because seam-corner coordinates misclassify, this pattern keys off
  the map coordinates on purpose: the point is to validate the map, so a wrong
  map has to look wrong. Only the entry marker uses index.

  Works against either variant of the map -- render2D reconstructs depth and
  height from the flattened 160x16 plane, so you get the same picture with
  flatten = true.

  NOTE: stages 0-2 light 1280+ pixels solid. Keep the global brightness slider
  low while testing.
*/

export var stage = 0

// Deliberately shorter than one full 8-pixel column: at 16 it covered two whole
// columns and so touched both the ridge and the outer edge, which made its
// position across the face meaningless as a diagnostic.
entryMarkerPixels = 4
rows = 16

export function render3D(index, x, y, z) {
  row = round((1 - z) * (rows - 1))  // 0 at the upper outer edge, 15 at the lower
  isUpper = z > 0.5                  // faces sit at z 0.533..1 and 0..0.467
  isRidge = y < 0.1                  // the two rows flanking the corner
  isOuter = y > 0.9                  // the outer top and bottom edges
  isEntry = index < entryMarkerPixels

  h = 0; s = 1; v = 0

  if (stage == 0) {
    v = 1
    if (isEntry)      { h = 0.6;  s = 1 }  // blue
    else if (isRidge) { h = 0;    s = 0 }  // white
    else if (isUpper) { h = 0;    s = 1 }  // red
    else              { h = 0.33; s = 1 }  // green
  } else if (stage == 1) {
    v = isUpper; h = 0; s = 1
  } else if (stage == 2) {
    v = !isUpper; h = 0.33; s = 1
  } else if (stage == 3) {
    v = isRidge; s = 0
  } else if (stage == 4) {
    v = isOuter; s = 0
  } else if (stage == 5) {
    v = isEntry; h = 0.6; s = 1
  } else if (stage == 6) {
    v = row == floor(time(0.3) * rows) % rows; s = 0
  } else if (stage == 7) {
    v = abs(x - time(0.3)) < 0.02; s = 0
  }

  hsv(h, s, v)
}

export function render2D(index, x, y) {
  // On the flattened map y is height across the unrolled 160x16 banner, with
  // the ridge in the middle, so depth is the distance out from mid-height.
  render3D(index, x, abs(y - 0.5) / 0.5, y)
}

export function render(index) {
  render3D(index, index / pixelCount, 0.5, 0.5)
}
