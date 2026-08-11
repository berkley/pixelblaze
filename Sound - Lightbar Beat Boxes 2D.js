/*
  Sound - Lightbar Beat Boxes 2D

  For the DJ Booth Lightbar (pixelmaps/halfcuboid-bar.js): the 160x16 banner
  tiled with 4x4 boxes, 40 across by 4 down. A few cells are left BLANK, and on
  every beat boxes slide into the blanks -- a sliding-puzzle board that shuffles
  itself in time with the music.

  Each blank picks a random direction and pulls a RUN of boxes one step toward
  itself; the blank ends up at the far end of the run. So a handful of blanks
  still sets dozens of boxes moving per beat, while every individual box travels
  exactly one cell.

  Boxes are drawn at FRACTIONAL positions with sub-pixel edge coverage, which is
  what makes the movement read as sliding at all. Snapped to whole pixels, a
  4-pixel box has only five possible positions during a one-cell move, so the
  slide is a five-frame flipbook however high the framerate goes -- it reads as
  blinking, not motion. Partial coverage on the leading and trailing edges gives
  the eye a continuous position instead.

  Two kinds of edge get handled differently. Where a box borders empty space its
  edge pixels are simply dimmed by their coverage, so the box appears to ease
  into the blank. Where two boxes of a sliding run meet, both contribute to the
  shared pixel: the coverages SUM (so the seam stays at full brightness rather
  than dipping) while the hue is blended between them along the shorter way
  round the colour wheel. Without that blend the colour boundary would still
  snap a whole pixel at a time and the interior of a run would flicker even
  though its outer edges were smooth.

  Moves are always axis-aligned, so only one axis is ever fractional. That keeps
  the coverage maths to one extra row or column per box, and it means the only
  pixels taking the slower blending path are the ~8 edge pixels of each box that
  is actually moving.

  A run stops when it hits the edge of the board, another blank, or a box that
  already moved this beat, so no box is ever asked to make two moves at once and
  two blanks can never land on the same cell. Directions are tried in random
  order, so a blocked first choice doesn't waste the beat.

  Because sliding boxes overlap and leave gaps, hues can't just be looked up per
  grid cell. beforeRender rasterizes into pixHue[], a framebuffer indexed by
  PIXEL INDEX rather than grid position, so render3D is a single array read with
  no coordinate maths -- that runs 2560 times a frame and measurably dominated
  the framerate when the maths was done per pixel. This way measured 15.5 fps
  against 10.6 for the static tiling pattern.

  The grid-position-to-pixel-index table that makes that possible (idxOf[]) is
  captured during the first render pass, since a pattern can only learn its
  pixels' coordinates by being handed them one at a time. Frame 1 therefore
  renders black and builds the table; everything after uses it.

  The bass beat detector is lifted from "Sound - Fast Pulse 3D": sub-bass bins
  0-3 only, so kicks trigger it and snares/claps with content higher up do not.
  It carries that pattern's octave correction and outlier rejection.

  How the tempo is used, precisely: moves are driven by a beat clock running at
  `detectedBpm`, but every confident onset resets that clock's phase and moves
  immediately. With music playing the onsets dominate and moves land on the
  kick; the BPM estimate is what keeps things moving through breakdowns and
  missed beats, and free-runs at 120 BPM if no sensor board is attached.

  Works against either variant of the map: render3D uses x and z, and render2D
  reconstructs z from the flattened 160x16 plane.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars; each
// is called with the slider's 0..1 value whenever it moves.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

// Number of blank cells, 1..20. Changing this rebuilds the board.
var blanksCtrl = 0.28       // -> 6 blanks
export function sliderBlanks(v) { blanksCtrl = v }

var runLengthCtrl = 0.5     // longest run a single blank can pull, 1..10 boxes
export function sliderRunLength(v) { runLengthCtrl = v }

// Slide duration as a fraction of one beat. At 0 the slide is ~5% of a beat,
// i.e. effectively an instant jump; at 1 a box is still travelling when the
// next beat lands. Defaulted high: the more frames a slide spans, the more
// distinct sub-pixel positions the eye gets to see.
var slideTimeCtrl = 0.85
export function sliderSlideTime(v) { slideTimeCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm = 120
export var bassSpike   = 0
export var beatCount   = 0
export var boxesMoved  = 0
export var blankCount  = 0

// ---- Geometry ----
barW = 160
barH = 16
boxSize = 4
boxesX = barW / boxSize          // 40
boxesY = barH / boxSize          // 4
cellCount = boxesX * boxesY      // 160
gridSize = barW * barH           // 2560
maxBlanks = 20

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0

// ---- Board state ----
// cellX/cellY is where a box rests; srcX/srcY is where its current slide began.
// A box with src == cell is stationary, so no separate "moving" flag is needed.
// boxAt[] is the reverse index, cell -> box index, or -1 for a blank cell.
boxHue = array(cellCount)
cellX  = array(cellCount)
cellY  = array(cellCount)
srcX   = array(cellCount)
srcY   = array(cellCount)
movedFlag = array(cellCount)
boxAt  = array(cellCount)
cellList = array(cellCount)   // scratch, shuffled when building the board

blankX = array(maxBlanks)
blankY = array(maxBlanks)
dirOrder = array(4)

// Framebuffers, both indexed by pixel index. pixVal doubles as the occupancy
// flag -- 0 means nothing is drawn there, so only pixVal needs clearing each
// frame and pixHue can be left stale.
pixHue = array(gridSize)   // hue 0..1
pixVal = array(gridSize)   // accumulated coverage 0..1
idxOf  = array(gridSize)   // grid position (row * barW + col) -> pixel index

nBlanks = 0
nBoxes = 0
slideProgress = 1
sliding = 0
needRaster = 1
frames = 0
mapped = 0

function buildBoard(nb) {
  nBlanks = nb
  nBoxes = cellCount - nBlanks
  blankCount = nBlanks

  for (bc = 0; bc < cellCount; bc++) {
    boxAt[bc] = -1
    cellList[bc] = bc
  }
  // Shuffle the cell list; the first nBlanks entries become the blanks and the
  // rest take boxes, so the blanks start scattered rather than in a clump.
  for (bi = cellCount - 1; bi > 0; bi--) {
    bj = floor(random(bi + 1))
    bt = cellList[bi]; cellList[bi] = cellList[bj]; cellList[bj] = bt
  }

  for (bk = 0; bk < nBlanks; bk++) {
    blankX[bk] = cellList[bk] % boxesX
    blankY[bk] = floor(cellList[bk] / boxesX)
  }
  for (bm = 0; bm < nBoxes; bm++) {
    bcell = cellList[nBlanks + bm]
    bcx = bcell % boxesX
    bcy = floor(bcell / boxesX)
    cellX[bm] = bcx; cellY[bm] = bcy
    srcX[bm]  = bcx; srcY[bm]  = bcy
    boxAt[bcell] = bm
    // Golden-ratio step across, quarter-turn down: adjacent boxes land far
    // apart on the colour wheel, so the board reads as distinct tiles.
    boxHue[bm] = (bcx * 0.381966 + bcy * 0.25) % 1
  }
  needRaster = 1
}

buildBoard(6)

// Complete any slide in flight by snapping every box onto its destination.
// Must run before choosing new moves, or a box still travelling would snap back
// to its old origin when the shared progress resets to 0.
function snapAll() {
  for (sn = 0; sn < nBoxes; sn++) {
    srcX[sn] = cellX[sn]
    srcY[sn] = cellY[sn]
  }
}

// How many boxes can be pulled toward blank (bx,by) from direction (dx,dy),
// capped at maxRun. Stops at the board edge, at another blank, or at a box that
// has already moved this beat.
function runLength(bx, by, dx, dy, maxRun) {
  k = 0
  while (k < maxRun) {
    px = bx + (k + 1) * dx
    py = by + (k + 1) * dy
    if (px < 0 || px >= boxesX || py < 0 || py >= boxesY) return k
    pb = boxAt[py * boxesX + px]
    if (pb < 0) return k
    if (movedFlag[pb]) return k
    k++
  }
  return k
}

function startSlides() {
  for (mf = 0; mf < nBoxes; mf++) movedFlag[mf] = 0
  maxRun = 1 + floor(runLengthCtrl * 9)   // 1..10
  moved = 0

  for (bl = 0; bl < nBlanks; bl++) {
    // Try the four directions in random order so a blocked first pick doesn't
    // cost the blank its move.
    for (d = 0; d < 4; d++) dirOrder[d] = d
    for (d = 3; d > 0; d--) {
      dj = floor(random(d + 1))
      dt = dirOrder[d]; dirOrder[d] = dirOrder[dj]; dirOrder[dj] = dt
    }

    bx = blankX[bl]
    by = blankY[bl]
    k = 0
    dx = 0
    dy = 0
    for (d = 0; d < 4 && k == 0; d++) {
      dd = dirOrder[d]
      dx = (dd == 0) - (dd == 1)   // 1, -1, 0, 0
      dy = (dd == 2) - (dd == 3)   // 0, 0, 1, -1
      k = runLength(bx, by, dx, dy, maxRun)
    }
    if (k == 0) continue

    // Shift the run one step toward the blank, nearest box first so each cell
    // is vacated before the next box arrives in it.
    for (j = 1; j <= k; j++) {
      ox = bx + j * dx
      oy = by + j * dy
      b = boxAt[oy * boxesX + ox]
      nx = bx + (j - 1) * dx
      ny = by + (j - 1) * dy
      srcX[b] = ox; srcY[b] = oy
      cellX[b] = nx; cellY[b] = ny
      movedFlag[b] = 1
      boxAt[ny * boxesX + nx] = b
      moved++
    }
    // The blank ends up where the last box of the run came from.
    blankX[bl] = bx + k * dx
    blankY[bl] = by + k * dy
    boxAt[blankY[bl] * boxesX + blankX[bl]] = -1
  }

  boxesMoved = moved
}

function rasterize() {
  for (ci = 0; ci < gridSize; ci++) pixVal[ci] = 0

  // Ease-out: the box leaves on the beat and settles into place, which feels
  // like a shove. Smoothstep's slow start wastes frames sitting still, and
  // frames are the scarce resource here.
  ep = slideProgress * (2 - slideProgress)

  for (bi = 0; bi < nBoxes; bi++) {
    fx = (srcX[bi] + (cellX[bi] - srcX[bi]) * ep) * boxSize
    fy = (srcY[bi] + (cellY[bi] - srcY[bi]) * ep) * boxSize
    ix = floor(fx); fracX = fx - ix
    iy = floor(fy); fracY = fy - iy
    // Snap away rounding dust, so a box that has arrived doesn't claim a
    // phantom extra column one pixel off the end of the board.
    if (fracX < 0.002) fracX = 0
    if (fracY < 0.002) fracY = 0

    nCols = fracX > 0 ? boxSize + 1 : boxSize
    nRows = fracY > 0 ? boxSize + 1 : boxSize
    if (ix + nCols > barW) nCols = barW - ix
    if (iy + nRows > barH) nRows = barH - iy

    hu = boxHue[bi]
    for (ry = 0; ry < nRows; ry++) {
      covY = 1
      if (fracY > 0) {
        if (ry == 0) covY = 1 - fracY
        else if (ry == boxSize) covY = fracY
      }
      rowBase = (iy + ry) * barW + ix
      for (rx = 0; rx < nCols; rx++) {
        covX = 1
        if (fracX > 0) {
          if (rx == 0) covX = 1 - fracX
          else if (rx == boxSize) covX = fracX
        }
        cov = covX * covY
        p = idxOf[rowBase + rx]

        if (cov >= 0.999) {
          // Fast path: interior of a box, and the overwhelming majority of
          // pixels. Stationary boxes take it for every pixel they own.
          pixHue[p] = hu
          pixVal[p] = 1
        } else if (cov > 0.004) {
          pv = pixVal[p]
          if (pv <= 0) {
            pixHue[p] = hu
            pixVal[p] = cov
          } else {
            // Shared with the neighbouring box of a run: sum the coverage and
            // blend the hue the short way round the wheel.
            dh = hu - pixHue[p]
            if (dh > 0.5) dh -= 1
            else if (dh < -0.5) dh += 1
            pixHue[p] = (pixHue[p] + dh * (cov / (pv + cov)) + 1) % 1
            pixVal[p] = pv + cov
          }
        }
      }
    }
  }
}

export function beforeRender(delta) {
  dt = delta * 0.001
  elapsed += dt

  // One full render pass has completed by the second beforeRender, so idxOf[]
  // is populated and the framebuffer can be used from here on.
  frames++
  if (frames >= 2) mapped = 1

  wantBlanks = 1 + floor(blanksCtrl * 19)   // 1..20
  if (wantBlanks != nBlanks) buildBoard(wantBlanks)

  // ---- Bass beat detection ----
  bassNow = 0
  for (i = 0; i < 4; i++) bassNow += frequencyData[i]
  bassNow *= 0.25

  dw = delta / 2000
  bassAvg  = bassAvg  * (1 - dw) + bassNow * dw
  bassPeak = bassPeak * 0.997
  if (bassNow > bassPeak) bassPeak = bassNow

  bassDyn   = max(max(bassPeak - bassAvg, bassPeak * 0.2), 0.00005)
  bassSpike = max(0, bassNow - bassAvg) / bassDyn

  threshold = 0.75 - sensitivityCtrl * 0.6   // slider right = more sensitive
  onset = 0

  if (bassSpike > threshold && elapsed - lastBassBeat > 60 / 220) {
    onset = 1
    if (lastBassBeat > 0) {
      interval = elapsed - lastBassBeat
      if (interval > 0.27 && interval < 1.5) {
        newBpm = 60 / interval

        // Fold a detection at 2x tempo (a spurious between-beat hit) or 0.5x
        // (we missed a real beat) back onto the current estimate.
        ratio = newBpm / detectedBpm
        if      (ratio > 1.6 && ratio < 2.5)  newBpm = newBpm * 0.5
        else if (ratio > 0.4 && ratio < 0.62) newBpm = newBpm * 2

        // Blend only if the corrected candidate is close; drop outliers.
        if (abs(newBpm - detectedBpm) / detectedBpm < 0.25) {
          detectedBpm = detectedBpm * 0.8 + newBpm * 0.2
        }
      }
    }
    lastBassBeat = elapsed
  }

  // ---- Beat clock ----
  // An onset takes priority and re-phases the clock, so a beat never fires
  // twice for the same kick.
  beatPhase += dt * detectedBpm / 60
  beat = 0
  if (onset) { beatPhase = 0; beat = 1 }
  else if (beatPhase >= 1) { beatPhase -= 1; beat = 1 }

  if (beat) {
    beatCount++
    snapAll()
    startSlides()
    slideProgress = 0
    sliding = 1
  }

  // ---- Slide animation ----
  rasterNow = needRaster
  if (sliding) {
    beatSeconds = 60 / detectedBpm
    slideDur = max(0.02, beatSeconds * (0.05 + slideTimeCtrl * 0.95))
    slideProgress += dt / slideDur
    if (slideProgress >= 1) {
      slideProgress = 1
      sliding = 0
      snapAll()
    }
    // Set even on the frame that finishes the slide: that frame rasterizes at
    // progress 1 and so draws the resting layout. Without it the display would
    // be left holding the second-to-last frame of the slide.
    rasterNow = 1
  }

  if (mapped && rasterNow) {
    rasterize()
    needRaster = 0
  }
}

export function render3D(index, x, y, z) {
  if (!mapped) {
    // First pass: learn where this pixel sits on the 160x16 grid. Rounding to
    // the nearest of the known positions rather than binning a continuous
    // coordinate -- the mapper places pixels at exactly col/159 and (15-row)/15,
    // and floor(x * 160) would drop the last column into a 161st bin at x = 1.
    col = round(x * (barW - 1))
    row = round((1 - z) * (barH - 1))
    idxOf[row * barW + col] = index
    hsv(0, 0, 0)
  } else {
    v = pixVal[index]
    if (v > 0) hsv(pixHue[index], 1, min(v, 1))
    else hsv(0, 0, 0)
  }
}

export function render2D(index, x, y) {
  // On the flattened map y is height across the unrolled banner.
  render3D(index, x, 0, y)
}

export function render(index) {
  // No map at all: fall back to a plain 1D run of the box colours. Deliberately
  // does not touch idxOf[], which has no meaning without 2D/3D coordinates.
  hsv(boxHue[floor(index / pixelCount * nBoxes)], 1, 1)
}
