/*
  FireBox1 3D

  A volumetric flame burning inside the cube. Each surface pixel samples
  the 3D fire field at its own (x, y, z), so the five panels read like
  windows looking in at a fireball suspended in the middle of the box.

  The field is a soft radial core (hottest at the cube center) modulated by
  three multiplied wave fields — same trick as the stock "Cube fire 3D"
  example — but with one of the time offsets coupled to z so the internal
  structure flows continuously upward.

  Bright white sparks spawn near the core and shoot upward (toward z = 0,
  the open top). Each spark is a moving 3D point; pixels light up white
  when they fall inside its 3D falloff radius — so as a spark drifts from
  the center toward a wall, it focuses into a sharp moving dot on that
  wall before exiting through the top.
*/

export var sliderIntensity  = 0.7
export var sliderSize       = 0.5   // radius of the central flame
export var sliderTurbulence = 0.5   // amount of internal flame structure
export var sliderRise       = 0.5   // upward flow rate
export var sliderSparks     = 0.5   // spark frequency

// Cube convention from the pixel map: z = 1 is the bottom panel, z = 0 is
// the (missing) top, so "rising" means moving toward smaller z.

t1 = 0
t2 = 0
rise = 0
size2 = 0.16
coreScale = 1
turbAmt = 0.7
turbBase = 0.3
flick = 1

// Spark state — flat parallel arrays for fast indexing in render3D.
maxSparks = 8
spx     = array(maxSparks)
spy     = array(maxSparks)
spz     = array(maxSparks)
spvx    = array(maxSparks)
spvy    = array(maxSparks)
spvz    = array(maxSparks)
spage   = array(maxSparks)
splife  = array(maxSparks)
spamp   = array(maxSparks)   // current brightness (0 = inactive, contributes nothing)

for (i = 0; i < maxSparks; i++) {
  spamp[i]  = 0
  spage[i]  = 1
  splife[i] = 1
}

function spawnSpark(i) {
  // Born near the cube center with a small jitter so they don't all
  // emerge from exactly the same point.
  spx[i] = 0.5 + (random(1) - 0.5) * 0.15
  spy[i] = 0.5 + (random(1) - 0.5) * 0.15
  spz[i] = 0.6  // a touch below center so they have farther to climb

  // Strong upward velocity (negative vz), modest random horizontal drift.
  // The drift is what eventually carries the spark close enough to a side
  // wall to be visible as a sharp point.
  ang = random(1) * 2 * PI
  spd = 0.25 + random(0.35)
  spvx[i] = cos(ang) * spd
  spvy[i] = sin(ang) * spd
  spvz[i] = -(0.7 + random(0.7))

  splife[i] = 0.9 + random(0.9)
  spage[i]  = 0
}

export function beforeRender(delta) {
  dt = delta * 0.001

  // Three time bases for the wave field; rise is on its own axis so the
  // flame appears to flow up rather than just shimmer in place.
  t1   = time(0.07)
  t2   = time(0.083)
  rise = time(0.04 / (0.3 + sliderRise * 1.7))

  size  = 0.22 + sliderSize * 0.32
  size2 = size * size
  flick     = 0.85 + 0.15 * wave(time(0.013))
  coreScale = sliderIntensity * 5.5 * flick * size2

  turbAmt  = sliderTurbulence
  turbBase = 1 - turbAmt

  // Spark lifecycle: each idle slot has a per-frame chance to spawn,
  // scaled so sliderSparks ≈ 1 keeps roughly all 8 slots in flight.
  spawnChance = dt * (0.5 + sliderSparks * 6)

  for (i = 0; i < maxSparks; i++) {
    if (spage[i] >= splife[i]) {
      // Idle slot.
      if (random(1) < spawnChance) {
        spawnSpark(i)
      } else {
        spamp[i] = 0
      }
    } else {
      spage[i] += dt
      spx[i] += spvx[i] * dt
      spy[i] += spvy[i] * dt
      spz[i] += spvz[i] * dt

      // Fade in fast, then a long taper. Peak amplitude tuned so a spark
      // overpowers the fire palette where it lands.
      p = spage[i] / splife[i]
      spamp[i] = (p < 0.1 ? p * 10 : 1 - (p - 0.1) / 0.9) * 1.6

      // Retire if the spark has shot through the top or wandered past
      // the cube on a side. Setting age to life marks the slot idle.
      if (spz[i] < -0.05 ||
          spx[i] < -0.15 || spx[i] > 1.15 ||
          spy[i] < -0.15 || spy[i] > 1.15) {
        spage[i] = splife[i]
        spamp[i] = 0
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

  // ---- spark contribution ----
  // Tight 3D falloff: at d = 0 → 1, at d ≈ 0.1 → ~0.5, at d ≈ 0.3 → ~0.1.
  // Sparks deep in the interior glow softly across nearby walls; once
  // they drift close to a wall they crisp up into a single pixel.
  white = 0
  for (i = 0; i < maxSparks; i++) {
    dx = x - spx[i]
    dy = y - spy[i]
    dz = z - spz[i]
    d2 = dx * dx + dy * dy + dz * dz
    white += spamp[i] / (1 + d2 * 120)
  }

  // ---- palette ----
  // White overlay bleaches saturation and pushes value to 1 wherever a
  // spark lands; elsewhere the fire palette is unchanged.
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
