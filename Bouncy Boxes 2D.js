/*
  4 smooth 4x4 squares with impulse collisions (no slowdown)
  - X wraps (cylinder 32)
  - Y bounces (height 8)
  - Squares collide and bounce, no intersections
  - Rainbow base hue per square + radial hue from center outward
  - Deterministic index mapping (matches your mapper)
  - Background glitches:
      * random sparkles
      * occasional short horizontal streaks
      * NEW: "digital tearing" bands (random rows shifted in X for a moment)
*/

W = 8
A = 32
S = 4
maxH = W - S

// ----- initial conditions -----
ax0 =  2.0; hy0 = 1.0; vx0 =  4.2; vy0 =  2.1
ax1 = 10.0; hy1 = 3.0; vx1 = -3.6; vy1 =  1.8
ax2 = 18.0; hy2 = 0.5; vx2 =  3.0; vy2 = -2.4
ax3 = 26.0; hy3 = 2.0; vx3 = -4.8; vy3 = -1.6

// physics knobs
restitution = 1.0
posSlop = 0.01
posPercent = 0.70
solverIters = 6
targetSpeed = 5.0

// hues (animated in beforeRender)
h0 = 0; h1 = 0.25; h2 = 0.5; h3 = 0.75

// background glitch controls
var glitchRate = 0.00          // sparkle chance per pixel per glitch frame
var glitchVal  = 0.00           // sparkle brightness
var glitchSat  = 1.0

export function sliderGlitchRate(v) {
  // v is automatically passed as a value between 0.0 and 1.0
  glitchRate = v;
}
export function sliderGlitchBrightness(v) {
  // v is automatically passed as a value between 0.0 and 1.0
  glitchVal = v;
}
export function sliderGlitchSat(v) {
  // v is automatically passed as a value between 0.0 and 1.0
  glitchSat = v;
}

// tearing controls
var tearChance = 0.06       // chance each tear band is active per tear frame (0..1)
var tearFramesPerSec = 10    // how often tear config changes
var tearMaxShift = 10         // max pixel shift in X for tearing
var tearVal = 0.5          // brightness for tear pixels
var tearSat = .05 

export function sliderTearChance(v) {
  // v is automatically passed as a value between 0.0 and 1.0
  tearChance = v;
}
export function sliderTearFramesPerSec(v) {
  // v is automatically passed as a value between 0.0 and 1.0
  tearFramesPerSec = v * 10;
}
export function sliderTearBrightness(v) {
  // v is automatically passed as a value between 0.0 and 1.0
  tearVal = v;
}
export function sliderTearSat(v) {
  // v is automatically passed as a value between 0.0 and 1.0
  tearSat = v;
}

function wrap01(v) { return v - floor(v) }
function wrapA(v) { return (v % A + A) % A }

function wrapDxSigned(a, b) {
  d = a - b
  if (d >  A * 0.5) d -= A
  if (d < -A * 0.5) d += A
  return d
}

function clampYAndBounce(i) {
  if (i == 0) {
    if (hy0 < 0)    { hy0 = -hy0;         vy0 = -vy0 }
    if (hy0 > maxH) { hy0 = 2*maxH-hy0;   vy0 = -vy0 }
  } else if (i == 1) {
    if (hy1 < 0)    { hy1 = -hy1;         vy1 = -vy1 }
    if (hy1 > maxH) { hy1 = 2*maxH-hy1;   vy1 = -vy1 }
  } else if (i == 2) {
    if (hy2 < 0)    { hy2 = -hy2;         vy2 = -vy2 }
    if (hy2 > maxH) { hy2 = 2*maxH-hy2;   vy2 = -vy2 }
  } else {
    if (hy3 < 0)    { hy3 = -hy3;         vy3 = -vy3 }
    if (hy3 > maxH) { hy3 = 2*maxH-hy3;   vy3 = -vy3 }
  }
}

function get(i) {
  if (i == 0) { ax=ax0; hy=hy0; vx=vx0; vy=vy0 }
  else if (i == 1) { ax=ax1; hy=hy1; vx=vx1; vy=vy1 }
  else if (i == 2) { ax=ax2; hy=hy2; vx=vx2; vy=vy2 }
  else { ax=ax3; hy=hy3; vx=vx3; vy=vy3 }
}

function set(i, ax, hy, vx, vy) {
  if (i == 0) { ax0=ax; hy0=hy; vx0=vx; vy0=vy }
  else if (i == 1) { ax1=ax; hy1=hy; vx1=vx; vy1=vy }
  else if (i == 2) { ax2=ax; hy2=hy; vx2=vx; vy2=vy }
  else { ax3=ax; hy3=hy; vx3=vx; vy3=vy }
}

function keepSpeed(i, target) {
  get(i)
  sp = sqrt(vx*vx + vy*vy)
  if (sp > 0.0001) {
    s = target / sp
    set(i, ax, hy, vx * s, vy * s)
  }
}

function collide(i, j) {
  get(i); axi=ax; hyi=hy; vxi=vx; vyi=vy
  get(j); axj=ax; hyj=hy; vxj=vx; vyj=vy

  cxi = axi + S*0.5
  cyi = hyi + S*0.5
  cxj = axj + S*0.5
  cyj = hyj + S*0.5

  dx = wrapDxSigned(cxi, cxj)
  dy = cyi - cyj
  adx = abs(dx)
  ady = abs(dy)

  if (adx < S && ady < S) {
    px = S - adx
    py = S - ady

    if (px < py) { nx = (dx >= 0) ? 1 : -1; ny = 0; pen = px }
    else         { nx = 0; ny = (dy >= 0) ? 1 : -1; pen = py }

    corr = posPercent * max(pen - posSlop, 0) * 0.5
    axi += nx * corr; hyi += ny * corr
    axj -= nx * corr; hyj -= ny * corr

    axi = wrapA(axi); axj = wrapA(axj)

    set(i, axi, hyi, vxi, vyi); clampYAndBounce(i); get(i); axi=ax; hyi=hy; vxi=vx; vyi=vy
    set(j, axj, hyj, vxj, vyj); clampYAndBounce(j); get(j); axj=ax; hyj=hy; vxj=vx; vyj=vy

    rvx = vxi - vxj
    rvy = vyi - vyj
    velN = rvx*nx + rvy*ny

    if (velN < 0) {
      jImp = -(1 + restitution) * velN / 2
      ix = jImp * nx
      iy = jImp * ny
      vxi += ix; vyi += iy
      vxj -= ix; vyj -= iy
    }

    set(i, axi, hyi, vxi, vyi)
    set(j, axj, hyj, vxj, vyj)
  }
}

function softBox(px, py, ax, hy) {
  dx = wrapDxSigned(px, ax)
  pxx = ax + dx

  left   = pxx - ax
  right  = (ax + S) - pxx
  top    = py - hy
  bottom = (hy + S) - py

  d = min(min(left, right), min(top, bottom))
  soft = 0.8
  return clamp((d + soft) / soft, 0, 1)
}

function radialHue(px, py, ax, hy, baseHue) {
  dxw = wrapDxSigned(px, ax)
  pxx = ax + dxw

  cx = ax + S*0.5
  cy = hy + S*0.5

  dx = pxx - cx
  dy = py - cy

  rMax = (S * 0.5) * 1.4142
  dist = sqrt(dx*dx + dy*dy) / rMax
  dist = clamp(dist, 0, 1)

  return wrap01(baseHue + dist * 0.60)
}

// stable pseudo-random 0..1
function hash01(n) {
  return wrap01(sin(n * 12.9898 + 78.233) * 43758.5453)
}

// ----- tearing state (computed in beforeRender) -----
tearRow0 = 0; tearRow1 = 0; tearRow2 = 0
tearShift0 = 0; tearShift1 = 0; tearShift2 = 0
tearOn0 = 0; tearOn1 = 0; tearOn2 = 0
tearHue0 = 0; tearHue1 = 0; tearHue2 = 0
gFrame = 0

export function beforeRender(delta) {
  dt = min(delta / 1000, 0.05)

  // integrate
  ax0 = wrapA(ax0 + vx0 * dt); hy0 += vy0 * dt
  ax1 = wrapA(ax1 + vx1 * dt); hy1 += vy1 * dt
  ax2 = wrapA(ax2 + vx2 * dt); hy2 += vy2 * dt
  ax3 = wrapA(ax3 + vx3 * dt); hy3 += vy3 * dt

  clampYAndBounce(0)
  clampYAndBounce(1)
  clampYAndBounce(2)
  clampYAndBounce(3)

  // collisions
  for (k = 0; k < solverIters; k++) {
    collide(0,1); collide(0,2); collide(0,3)
    collide(1,2); collide(1,3)
    collide(2,3)
  }

  keepSpeed(0, targetSpeed)
  keepSpeed(1, targetSpeed)
  keepSpeed(2, targetSpeed)
  keepSpeed(3, targetSpeed)

  // rainbow base hues per square
  t = time(12 / 65.536)
  h0 = wrap01(t + 0.00)
  h1 = wrap01(t + 0.25)
  h2 = wrap01(t + 0.50)
  h3 = wrap01(t + 0.75)

  // glitch frame counters
  gFrame = floor(time(18 / 65.536) * 1000)

  // tearing "frame" (changes a few times per second)
  tearFrame = floor(time((1 / tearFramesPerSec) / 65.536) * 100000)

  // pick up to 3 tear bands (random rows), each with its own shift
  // each band sometimes off
  r0 = hash01(tearFrame * 31.0 + 1.0)
  r1 = hash01(tearFrame * 37.0 + 2.0)
  r2 = hash01(tearFrame * 41.0 + 3.0)

  tearRow0 = floor(hash01(tearFrame * 53.0 + 4.0) * W)
  tearRow1 = floor(hash01(tearFrame * 59.0 + 5.0) * W)
  tearRow2 = floor(hash01(tearFrame * 61.0 + 6.0) * W)

  // shifts in [-tearMaxShift, +tearMaxShift]
  tearShift0 = floor((hash01(tearFrame * 67.0 + 7.0) * 2 - 1) * tearMaxShift)
  tearShift1 = floor((hash01(tearFrame * 71.0 + 8.0) * 2 - 1) * tearMaxShift)
  tearShift2 = floor((hash01(tearFrame * 73.0 + 9.0) * 2 - 1) * tearMaxShift)

  tearOn0 = (r0 < tearChance) ? 1 : 0
  tearOn1 = (r1 < tearChance) ? 1 : 0
  tearOn2 = (r2 < tearChance) ? 1 : 0

  tearHue0 = hash01(tearFrame * 79.0 + 10.0)
  tearHue1 = hash01(tearFrame * 83.0 + 11.0)
  tearHue2 = hash01(tearFrame * 89.0 + 12.0)
}

export function render2D(index, x, y) {
  // exact mapper-derived coords
  around = floor(index / W)   // 0..31
  h = index % W               // 0..7
  if (around % 2 == 1) h = (W - 1) - h

  // ---- digital tearing: shift some rows in X (background only + affects square sampling) ----
  // We'll apply tearing to the "pixel sample position" px, but NOT to the physical mapping.
  // This creates a tearing illusion without changing your actual topology.
  pxBase = around + 0.5
  py = h + 0.5

  tearShift = 0
  tearHue = 0
  tearOn = 0

  if (tearOn0 && h == tearRow0) { tearShift = tearShift0; tearHue = tearHue0; tearOn = 1 }
  if (tearOn1 && h == tearRow1) { tearShift = tearShift1; tearHue = tearHue1; tearOn = 1 }
  if (tearOn2 && h == tearRow2) { tearShift = tearShift2; tearHue = tearHue2; tearOn = 1 }

  // apply shift with wrap for sampling
  px = wrapA(pxBase + tearShift)

  // squares sampled at torn px (so the whole scene appears torn)
  v0 = softBox(px, py, ax0, hy0)
  v1 = softBox(px, py, ax1, hy1)
  v2 = softBox(px, py, ax2, hy2)
  v3 = softBox(px, py, ax3, hy3)

  total = v0 + v1 + v2 + v3

  // ---- background glitches (only visible where squares aren't) ----
  r = hash01(index + gFrame * 97.0)
  sparkle = (r < glitchRate) ? 1 : 0
  gAmt = sparkle ? glitchVal : 0

  // occasional short horizontal streak (rare)
  rr = hash01(gFrame * 13.0 + 5.0)
  rs = hash01(gFrame * 17.0 + 11.0)
  rowPick = floor(rr * W)
  streakCenter = floor(rs * A)
  dxs = abs(wrapDxSigned(around, streakCenter))
  streak = (h == rowPick && dxs <= 2) ? 1 : 0
  if (streak) gAmt = max(gAmt, glitchVal * 0.7)

  // tearing band brightness (background feel)
  tearAmt = tearOn ? tearVal : 0

  // ---- choose final pixel ----
  if (total > 0) {
    // dominant square hue, radiating outward
    bestV = v0; hue = radialHue(px, py, ax0, hy0, h0)
    if (v1 > bestV) { bestV = v1; hue = radialHue(px, py, ax1, hy1, h1) }
    if (v2 > bestV) { bestV = v2; hue = radialHue(px, py, ax2, hy2, h2) }
    if (v3 > bestV) { bestV = v3; hue = radialHue(px, py, ax3, hy3, h3) }

    // subtle tear overlay on squares
    val = clamp(total + tearAmt * 0.15, 0, 1)
    hsv(hue, 1, val)
  } else if (tearAmt > 0) {
    // tearing-only band
    hsv(tearHue, tearSat, tearAmt)
  } else if (gAmt > 0) {
    // sparkles/streaks
    gHue = hash01(index * 3.0 + gFrame * 19.0)
    hsv(gHue, glitchSat, gAmt)
  } else {
    hsv(0, 0, 0)
  }
}

export function render(index) {
  hsv(0, 0, 0)
}
