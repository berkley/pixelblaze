/*
  Sound - Lightbar Beat Boxes 2D

  A sliding-puzzle board of coloured boxes. A few cells are left BLANK, and on
  every beat boxes slide into the blanks -- the board shuffles itself in time
  with the music.

  Each blank picks a random direction and pulls a RUN of boxes one step toward
  itself; the blank ends up at the far end of the run. So a handful of blanks
  still sets dozens of boxes moving per beat, while every individual box travels
  exactly one cell. That one-cell limit is deliberate: one cell is a few pixels,
  which animates smoothly, whereas an earlier version that sent boxes to random
  cells covered ~80 pixels in four frames and read as teleporting.

  A run stops at the edge of the board, at another blank, or at a box that
  already moved this beat, so no box is asked to make two moves at once and two
  blanks can never land on the same cell.

  ---------------------------------------------------------------------------
  PORTABLE ACROSS DISPLAYS. This pattern originally hardcoded the DJ Booth
  Lightbar's 160x16 grid, which made it produce garbage on anything else. On a
  1280-pixel cube, measured: only 18 of the assumed 160 columns exist and only
  286 of the 2560 grid cells were ever populated, so 89% of lookups returned 0
  and dumped the whole pattern onto pixel 0 -- it lit 22.6% of the cube where
  the equivalent static tiling lit 100%.

  Two things fix that, and both matter independently:

  1. THE LOOKUP RUNS FORWARD, NOT BACKWARD. The old code built a table from grid
     position to pixel index and wrote through it; any cell no pixel claimed
     still held 0, so those writes landed on pixel 0. Now each pixel stores the
     cell IT occupies (pxCell) and simply reads that cell's colour. A cell no
     pixel claims is now merely unread, which is harmless. This alone removes
     the collapse, and costs nothing -- it is still one array read per pixel.

  2. THE GRID IS MEASURED, NOT ASSUMED. Startup spends two render passes
     discovering how many DISTINCT column and row positions the display
     actually has, then compacts them into a dense range. The board is sized
     from that. A 16x16-faced cube reports ~18 columns and gets a 5x4 board; the
     160x16 bar reports 160 and gets its original 40x4.

  Startup therefore has three phases (see `phase`): mark which positions exist,
  assign each pixel its cell, then run. The first two frames render black.
  ---------------------------------------------------------------------------

  Because sliding boxes overlap and leave gaps, colours cannot just be looked up
  per board cell; beforeRender rasterises into a fine-grained cell buffer and
  render reads its own cell out of it.

  The bass beat detector is lifted from "Sound - Fast Pulse 3D": sub-bass bins
  0-3 only, so kicks trigger it and snares/claps with content higher up do not.
  It carries that pattern's octave correction and outlier rejection. Onsets
  drive the moves and re-phase a clock running at the detected BPM; the clock is
  what keeps things moving through breakdowns, and free-runs at 120 BPM with no
  sensor board attached.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

// Number of blank cells, as a fraction of the board. Changing it rebuilds.
var blanksCtrl = 0.28
export function sliderBlanks(v) { blanksCtrl = v }

var runLengthCtrl = 0.5     // longest run a single blank can pull
export function sliderRunLength(v) { runLengthCtrl = v }

// Slide duration as a fraction of one beat.
var slideTimeCtrl = 0.45
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
export var gridCols    = 0   // measured, not assumed
export var gridRows    = 0
export var boardCols   = 0
export var boardRows   = 0

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0

// ---- Sizing ----
// QMAX is the resolution the startup scan quantises positions to while counting
// how many distinct ones exist. It only has to exceed the finest display this
// pattern will meet; the bar needs 160.
QMAX = 256
MAXCELLS = 160        // ceiling on board cells, so the board arrays stay fixed
MAXGRID = 2560        // ceiling on fine cells

// colSeen/rowSeen are used twice: first as presence flags during the scan, then
// rewritten in place as rank tables mapping a quantised position to a dense
// index. Reusing them saves two more QMAX-sized arrays against the 10240 cap.
colSeen = array(QMAX)
rowSeen = array(QMAX)

nCols = 1        // distinct positions the display actually has
nRows = 1
boxSize = 4      // fine cells per box, raised if the board would exceed MAXCELLS
boxesX = 1
boxesY = 1
cellCount = 1
gridSize = 1

// ---- Board state ----
boxHue = array(MAXCELLS)
cellX  = array(MAXCELLS)
cellY  = array(MAXCELLS)
srcX   = array(MAXCELLS)
srcY   = array(MAXCELLS)
movedFlag = array(MAXCELLS)
boxAt  = array(MAXCELLS)
cellList = array(MAXCELLS)

maxBlanks = 20
blankX = array(maxBlanks)
blankY = array(maxBlanks)
dirOrder = array(4)

// Fine cell buffer, indexed by (row * nCols + col) -- grid space, not pixel
// space, which is what makes the lookup forward.
gridHue = array(MAXGRID)
gridVal = array(MAXGRID)
pxCell  = array(pixelCount)   // pixel index -> its fine cell. Sized to the device.

nBlanks = 0
nBoxes = 0
slideProgress = 1
sliding = 0
needRaster = 1
frames = 0
phase = 0        // 0 = scanning positions, 1 = assigning pixels, 2 = running

function buildBoard(nb) {
  if (nb > cellCount - 1) nb = cellCount - 1
  if (nb < 1) nb = 1
  nBlanks = nb
  nBoxes = cellCount - nBlanks
  blankCount = nBlanks

  for (bc = 0; bc < cellCount; bc++) { boxAt[bc] = -1; cellList[bc] = bc }
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

// Called once, after the scan has counted the display's real resolution.
function setupBoard() {
  gridCols = nCols
  gridRows = nRows
  gridSize = nCols * nRows

  // Aim for boxes about 4 fine cells across, but grow them if that would make
  // more board cells than the fixed arrays hold.
  boxSize = 4
  boxesX = ceil(nCols / boxSize)
  boxesY = ceil(nRows / boxSize)
  while (boxesX * boxesY > MAXCELLS) {
    boxSize++
    boxesX = ceil(nCols / boxSize)
    boxesY = ceil(nRows / boxSize)
  }
  if (boxesX < 2) boxesX = 2
  if (boxesY < 1) boxesY = 1
  cellCount = boxesX * boxesY
  boardCols = boxesX
  boardRows = boxesY

  buildBoard(1 + floor(blanksCtrl * (cellCount * 0.25)))
}

function snapAll() {
  for (sn = 0; sn < nBoxes; sn++) { srcX[sn] = cellX[sn]; srcY[sn] = cellY[sn] }
}

// How many boxes can be pulled toward blank (bx,by) from direction (dx,dy).
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
  maxRun = 1 + floor(runLengthCtrl * 9)
  moved = 0

  for (bl = 0; bl < nBlanks; bl++) {
    for (d = 0; d < 4; d++) dirOrder[d] = d
    for (d = 3; d > 0; d--) {
      dj = floor(random(d + 1))
      dt2 = dirOrder[d]; dirOrder[d] = dirOrder[dj]; dirOrder[dj] = dt2
    }

    bx = blankX[bl]
    by = blankY[bl]
    k = 0; dx = 0; dy = 0
    for (d = 0; d < 4 && k == 0; d++) {
      dd = dirOrder[d]
      dx = (dd == 0) - (dd == 1)
      dy = (dd == 2) - (dd == 3)
      k = runLength(bx, by, dx, dy, maxRun)
    }
    if (k == 0) continue

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
    blankX[bl] = bx + k * dx
    blankY[bl] = by + k * dy
    boxAt[blankY[bl] * boxesX + blankX[bl]] = -1
  }
  boxesMoved = moved
}

function rasterize() {
  for (ci = 0; ci < gridSize; ci++) gridVal[ci] = 0

  // Ease-out: the box leaves on the beat and settles into place.
  ep = slideProgress * (2 - slideProgress)

  for (bi = 0; bi < nBoxes; bi++) {
    fx = (srcX[bi] + (cellX[bi] - srcX[bi]) * ep) * boxSize
    fy = (srcY[bi] + (cellY[bi] - srcY[bi]) * ep) * boxSize
    ix = floor(fx); fracX = fx - ix
    iy = floor(fy); fracY = fy - iy
    if (fracX < 0.002) fracX = 0
    if (fracY < 0.002) fracY = 0

    nc = fracX > 0 ? boxSize + 1 : boxSize
    nr = fracY > 0 ? boxSize + 1 : boxSize
    if (ix + nc > nCols) nc = nCols - ix
    if (iy + nr > nRows) nr = nRows - iy

    hu = boxHue[bi]
    for (ry = 0; ry < nr; ry++) {
      covY = 1
      if (fracY > 0) {
        if (ry == 0) covY = 1 - fracY
        else if (ry == boxSize) covY = fracY
      }
      rowBase = (iy + ry) * nCols + ix
      for (rx = 0; rx < nc; rx++) {
        covX = 1
        if (fracX > 0) {
          if (rx == 0) covX = 1 - fracX
          else if (rx == boxSize) covX = fracX
        }
        cov = covX * covY
        p = rowBase + rx

        if (cov >= 0.999) {
          gridHue[p] = hu
          gridVal[p] = 1
        } else if (cov > 0.004) {
          pv = gridVal[p]
          if (pv <= 0) {
            gridHue[p] = hu
            gridVal[p] = cov
          } else {
            // Shared with the neighbouring box of a run: sum the coverage and
            // blend the hue the short way round the wheel.
            dh = hu - gridHue[p]
            if (dh > 0.5) dh -= 1
            else if (dh < -0.5) dh += 1
            gridHue[p] = (gridHue[p] + dh * (cov / (pv + cov)) + 1) % 1
            gridVal[p] = pv + cov
          }
        }
      }
    }
  }
}

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt
  frames++

  // ---- Startup: measure the display, then assign pixels to cells ----
  if (phase == 0) {
    if (frames >= 2) {
      // Compact the quantised positions that actually exist into a dense range,
      // rewriting the presence flags in place as rank numbers.
      n = 0
      for (q = 0; q < QMAX; q++) {
        if (colSeen[q] > 0) { colSeen[q] = n; n++ } else colSeen[q] = -1
      }
      nCols = n > 0 ? n : 1
      m = 0
      for (q = 0; q < QMAX; q++) {
        if (rowSeen[q] > 0) { rowSeen[q] = m; m++ } else rowSeen[q] = -1
      }
      nRows = m > 0 ? m : 1
      setupBoard()
      phase = 1
    }
    return
  }
  if (phase == 1) {
    // One full render pass has now assigned every pixel its cell.
    phase = 2
    return
  }

  dw = delta / 2000

  // ---- Bass beat detection (from Sound - Fast Pulse 3D) ----
  bassNow = 0
  for (i = 0; i < 4; i++) bassNow += frequencyData[i]
  bassNow *= 0.25

  bassAvg = bassAvg * (1 - dw) + bassNow * dw
  bassPeak = bassPeak * 0.997
  if (bassNow > bassPeak) bassPeak = bassNow

  bassDyn = max(max(bassPeak - bassAvg, bassPeak * 0.2), 0.00005)
  bassSpike = max(0, bassNow - bassAvg) / bassDyn

  threshold = 0.75 - sensitivityCtrl * 0.6
  onset = 0
  if (bassSpike > threshold && elapsed - lastBassBeat > 60 / 220) {
    onset = 1
    if (lastBassBeat > 0) {
      interval = elapsed - lastBassBeat
      if (interval > 0.27 && interval < 1.5) {
        newBpm = 60 / interval
        ratio = newBpm / detectedBpm
        if      (ratio > 1.6 && ratio < 2.5)  newBpm = newBpm * 0.5
        else if (ratio > 0.4 && ratio < 0.62) newBpm = newBpm * 2
        if (abs(newBpm - detectedBpm) / detectedBpm < 0.25) {
          detectedBpm = detectedBpm * 0.8 + newBpm * 0.2
        }
      }
    }
    lastBassBeat = elapsed
  }

  beatPhase += dt * detectedBpm / 60
  beat = 0
  if (onset) { beatPhase = 0; beat = 1 }
  else if (beatPhase >= 1) { beatPhase -= 1; beat = 1 }

  wantBlanks = 1 + floor(blanksCtrl * (cellCount * 0.25))
  if (wantBlanks != nBlanks) buildBoard(wantBlanks)

  if (beat) {
    beatCount++
    snapAll()
    startSlides()
    slideProgress = 0
    sliding = 1
  }

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
    // Set even on the frame that finishes the slide, so the resting layout is
    // drawn rather than the second-to-last frame of the slide.
    rasterNow = 1
  }

  if (rasterNow) {
    rasterize()
    needRaster = 0
  }
}

export function render3D(index, x, y, z) {
  if (phase == 0) {
    // Pass 1: record which quantised positions this display actually has. Every
    // pixel votes; duplicates are harmless.
    colSeen[round(x * (QMAX - 1))] = 1
    rowSeen[round((1 - z) * (QMAX - 1))] = 1
    hsv(0, 0, 0)
  } else if (phase == 1) {
    // Pass 2: the ranks are built, so this pixel can be given its dense cell.
    c = colSeen[round(x * (QMAX - 1))]
    r = rowSeen[round((1 - z) * (QMAX - 1))]
    if (c < 0) c = 0
    if (r < 0) r = 0
    pxCell[index] = r * nCols + c
    hsv(0, 0, 0)
  } else {
    // Forward lookup: this pixel reads the cell it occupies. A cell no pixel
    // claims is simply never read.
    p = pxCell[index]
    v = gridVal[p]
    if (v > 0) hsv(gridHue[p], 1, min(v, 1))
    else hsv(0, 0, 0)
  }
}

export function render2D(index, x, y) {
  // On a flattened map y is height across the unrolled surface.
  render3D(index, x, 0, y)
}

export function render(index) {
  // No map at all: fall back to a plain 1D run of the box colours.
  hsv(boxHue[floor(index / pixelCount * max(nBoxes, 1))], 1, 1)
}
