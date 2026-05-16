/*
  FireBox1 3D

  A volumetric flame burning inside the cube. Each surface pixel samples
  the 3D fire field at its own (x, y, z), so the five panels read like
  windows looking in at a fireball suspended in the middle of the box.

  Bright white embers spawn near the core, shoot off in random (mostly
  upward) directions, arc under "gravity", and leave a short fading
  trail of single-pixel dots behind them. The falloff radius is small
  enough that each trail point only lights the closest pixel or two.
*/

export var sliderIntensity  = 0.7
export var sliderSize       = 0.5   // radius of the central flame
export var sliderTurbulence = 0.5   // amount of internal flame structure
export var sliderRise       = 0.5   // upward flow rate
export var sliderSparks     = 0.5   // ember frequency

// Cube convention from the pixel map: z = 1 is the bottom panel, z = 0 is
// the (missing) top, so "rising" means moving toward smaller z and
// gravity pulls toward larger z.

t1 = 0
t2 = 0
rise = 0
size2 = 0.16
coreScale = 1
turbAmt = 0.7
turbBase = 0.3
flick = 1

// ---- Embers ----
// Each spark keeps a history of the last `trailLen` positions so the
// rendered ember has a fading tail. Stored as flat parallel arrays so
// the inner loop in render3D is one straight pass over `slots` entries.
maxSparks = 6
trailLen  = 6
slots     = maxSparks * trailLen   // 36

trailX   = array(slots)
trailY   = array(slots)
trailZ   = array(slots)
trailAmp = array(slots)            // 0 = dark; 1+ = visible

sparkVx     = array(maxSparks)
sparkVy     = array(maxSparks)
sparkVz     = array(maxSparks)
sparkAge    = array(maxSparks)
sparkLife   = array(maxSparks)
sparkActive = array(maxSparks)

for (i = 0; i < slots; i++) trailAmp[i] = 0
for (i = 0; i < maxSparks; i++) sparkActive[i] = 0

shiftAccum = 0
shiftPeriod = 0.06   // seconds between trail snapshots — sets trail spacing
trailDecay  = 0.65   // brightness multiplier each shift

function spawnSpark(i) {
  base = i * trailLen

  // Born near the cube core with a small jitter.
  trailX[base] = 0.5 + (random(1) - 0.5) * 0.15
  trailY[base] = 0.5 + (random(1) - 0.5) * 0.15
  trailZ[base] = 0.6

  // Collapse the rest of the trail onto the spawn point so it can grow
  // out behind the spark cleanly as shifts happen.
  for (j = 1; j < trailLen; j++) {
    trailX[base + j]   = trailX[base]
    trailY[base + j]   = trailY[base]
    trailZ[base + j]   = trailZ[base]
    trailAmp[base + j] = 0
  }
  trailAmp[base] = 1.8

  // Random direction in the upper hemisphere with a cosine-weighted bias
  // toward straight up. Some embers will fly nearly horizontal.
  phi    = random(1) * 2 * PI
  upDot  = 0.2 + random(0.8)             // 0.2..1.0; 1 = straight up
  horR   = sqrt(1 - upDot * upDot)
  speed  = 0.7 + random(0.7)

  sparkVx[i] = cos(phi) * horR * speed
  sparkVy[i] = sin(phi) * horR * speed
  sparkVz[i] = -upDot * speed            // negative z is "up"

  sparkAge[i]    = 0
  sparkLife[i]   = 0.8 + random(0.8)
  sparkActive[i] = 1
}

// Push every spark's recent positions back one slot. Slot 0 stays put
// (the head keeps moving each frame), slot 1 captures where the head
// was at the previous shift, slot 2 the shift before that, etc.
function shiftTrails() {
  for (i = 0; i < maxSparks; i++) {
    base = i * trailLen
    for (j = trailLen - 1; j > 0; j--) {
      trailX[base + j]   = trailX[base + j - 1]
      trailY[base + j]   = trailY[base + j - 1]
      trailZ[base + j]   = trailZ[base + j - 1]
      trailAmp[base + j] = trailAmp[base + j - 1] * trailDecay
    }
  }
}

export function beforeRender(delta) {
  dt = delta * 0.001

  // ---- Fire field ----
  t1   = time(0.07)
  t2   = time(0.083)
  rise = time(0.04 / (0.3 + sliderRise * 1.7))

  size  = 0.22 + sliderSize * 0.32
  size2 = size * size
  flick     = 0.85 + 0.15 * wave(time(0.013))
  coreScale = sliderIntensity * 5.5 * flick * size2

  turbAmt  = sliderTurbulence
  turbBase = 1 - turbAmt

  // ---- Trail snapshot ----
  shiftAccum += dt
  if (shiftAccum >= shiftPeriod) {
    shiftAccum -= shiftPeriod
    shiftTrails()
  }

  // ---- Spark physics + lifecycle ----
  spawnChance = dt * (0.5 + sliderSparks * 12)
  gravity     = 0.6   // pulls velocity toward +z (down)

  for (i = 0; i < maxSparks; i++) {
    base = i * trailLen
    if (!sparkActive[i]) {
      if (random(1) < spawnChance) spawnSpark(i)
      else trailAmp[base] = 0
    } else {
      sparkAge[i] += dt
      sparkVz[i] += gravity * dt
      trailX[base] += sparkVx[i] * dt
      trailY[base] += sparkVy[i] * dt
      trailZ[base] += sparkVz[i] * dt

      // Linear fade over life — gives the head a clear extinction.
      p = sparkAge[i] / sparkLife[i]
      trailAmp[base] = (1 - p) * 1.8

      // Retire if it has flown out the top, fallen back through the
      // bottom panel into the floor, or wandered past a side wall.
      if (sparkAge[i] >= sparkLife[i] ||
          trailZ[base] < -0.05 || trailZ[base] > 1.15 ||
          trailX[base] < -0.15 || trailX[base] > 1.15 ||
          trailY[base] < -0.15 || trailY[base] > 1.15) {
        sparkActive[i] = 0
        trailAmp[base] = 0
        // Older slots are left to dim through subsequent shifts so the
        // tail finishes trailing off after the head goes dark.
      }
    }
  }
}

export function render3D(index, x, y, z) {
  // ---- fire field ----
  cx = x - 0.5
  cy = y - 0.5
  cz = z - 0.5
  r2 = cx * cx + cy * cy + cz * cz
  core = coreScale / (size2 + r2 * 4)
  turb = wave(x * 3 + t1) * wave(y * 3 + t2) * wave(z * 3 + rise)
  heat = core * (turbBase + turb * 2 * turbAmt)

  // ---- trail contribution ----
  // Polynomial falloff with hard cutoff: at d² · 220 ≥ 1 (≈ d > 0.067,
  // which is one pixel spacing on a 16-cell face) the contribution is 0.
  // So each trail point essentially lights one pixel — its nearest.
  white = 0
  for (i = 0; i < slots; i++) {
    dx = x - trailX[i]
    dy = y - trailY[i]
    dz = z - trailZ[i]
    d2 = dx * dx + dy * dy + dz * dz
    white += trailAmp[i] * clamp(1 - d2 * 220, 0, 1)
  }

  // ---- palette ----
  h   = 0.02 + clamp(heat, 0, 1) * 0.10
  sat = clamp(1.3 - heat * 1.0 - white * 2.5, 0, 1)
  val = clamp(heat * heat * 1.5 + white * 1.5, 0, 1)
  hsv(h, sat, val)
}

export function render2D(index, x, y) {
  render3D(index, x, y, 0.5)
}

export function render(index) {
  render3D(index, index / pixelCount, 0.5, 0.5)
}
