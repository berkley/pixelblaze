/*
  FireBox1 3D

  A volumetric flame burning inside the cube. Each surface pixel samples
  the 3D fire field at its own (x, y, z), so the five panels read like
  windows looking in at a fireball suspended in the middle of the box.

  The field is a soft radial core (hottest at the cube center) modulated by
  three multiplied wave fields — same trick as the stock "Cube fire 3D"
  example — but with one of the time offsets coupled to z so the internal
  structure flows continuously upward.
*/

export var sliderIntensity  = 0.7
export var sliderSize       = 0.5   // radius of the central flame
export var sliderTurbulence = 0.5   // amount of internal flame structure
export var sliderRise       = 0.5   // upward flow rate

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

export function beforeRender(delta) {
  // Three time bases for the wave field; rise is on its own axis so the
  // flame appears to flow up rather than just shimmer in place.
  t1   = time(0.07)
  t2   = time(0.083)
  rise = time(0.04 / (0.3 + sliderRise * 1.7))

  size  = 0.22 + sliderSize * 0.32
  size2 = size * size
  // Pre-fold size² and intensity into one constant so render3D doesn't
  // have to multiply them out per pixel.
  flick     = 0.85 + 0.15 * wave(time(0.013))
  coreScale = sliderIntensity * 5.5 * flick * size2

  turbAmt  = sliderTurbulence
  turbBase = 1 - turbAmt
}

export function render3D(index, x, y, z) {
  // Distance² from the cube center.
  cx = x - 0.5
  cy = y - 0.5
  cz = z - 0.5
  r2 = cx * cx + cy * cy + cz * cz

  // Soft inverse-square core: hottest at center, gentle falloff to edges.
  core = coreScale / (size2 + r2 * 4)

  // 3D turbulence — three multiplied waves. Coupling z to `rise` makes the
  // structure crawl upward over time.
  turb = wave(x * 3 + t1) *
         wave(y * 3 + t2) *
         wave(z * 3 + rise)

  // Map turb [0..1] -> [turbBase..turbBase+2*turbAmt], so at turbAmt = 0
  // the field is uniform and at turbAmt = 1 it's fully modulated.
  heat = core * (turbBase + turb * 2 * turbAmt)

  // Fire palette: dark red -> orange -> yellow -> white at the core.
  h   = 0.02 + clamp(heat, 0, 1) * 0.10
  sat = clamp(1.3 - heat * 1.0, 0, 1)
  val = clamp(heat * heat * 1.5, 0, 1)
  hsv(h, sat, val)
}

export function render2D(index, x, y) {
  render3D(index, x, y, 0.5)
}

export function render(index) {
  render3D(index, index / pixelCount, 0.5, 0.5)
}
