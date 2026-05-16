/*
  Cube Fire 3D

  Random hotspots ignite on the surface of a 5-sided LED cube, glow with
  flickering heat, and fade away. Rendering in 3D lets the Pixelblaze pixel
  map carry heat continuously across panel seams without any special wiring.

  Designed for a 5-panel cube (no top), but the heat field is fully 3D so it
  will work on any 3D-mapped project.
*/

export var sliderIntensity = 0.6   // overall brightness of the fire
export var sliderActivity  = 0.5   // hotspot lifetime / respawn rate
export var sliderReach     = 0.5   // how far each hotspot spreads
export var sliderFlicker   = 0.5   // amount of turbulence

numSources = 8
sources = array(numSources)
for (i = 0; i < numSources; i++) sources[i] = array(5)  // [x, y, z, age, life]

// Pick a random point on one of the 5 cube faces (bottom + 4 sides, no top).
function spawn(idx, initialAge) {
  s = sources[idx]
  face = floor(random(5))
  u = random(1)
  v = random(1)
  if      (face == 0) { s[0] = u; s[1] = v; s[2] = 1 }   // bottom
  else if (face == 1) { s[0] = 0; s[1] = u; s[2] = v }   // left
  else if (face == 2) { s[0] = u; s[1] = 1; s[2] = v }   // back
  else if (face == 3) { s[0] = 1; s[1] = u; s[2] = v }   // right
  else                { s[0] = u; s[1] = 0; s[2] = v }   // front
  life = 1.2 + random(3.0)
  s[3] = initialAge * life
  s[4] = life
}

// Stagger initial ages so the cube isn't dark at startup.
for (i = 0; i < numSources; i++) spawn(i, random(1))

export function beforeRender(delta) {
  dt = delta * 0.001

  // Activity slider scales the apparent lifetime: high activity = short-lived,
  // fast-respawning sparks; low activity = slow, lingering glow.
  ageRate = 0.4 + sliderActivity * 1.8

  t1 = time(0.05)
  t2 = time(0.061)
  t3 = time(0.043)

  for (i = 0; i < numSources; i++) {
    s = sources[i]
    s[3] += dt * ageRate
    if (s[3] >= s[4]) spawn(i, 0)
  }

  reach = 0.10 + sliderReach * 0.30   // radius in normalized cube units
  reach2 = reach * reach
  flickAmp = sliderFlicker * 0.12
}

// Attack-then-decay envelope. 15% rise, 85% fall, peak = 1.
function envelope(age, life) {
  p = age / life
  if (p < 0.15) return p / 0.15
  return 1 - (p - 0.15) / 0.85
}

export function render3D(index, x, y, z) {
  // Per-pixel turbulence makes the field shimmer like flame.
  fx = x + flickAmp * (wave(x * 3 + t1) - 0.5)
  fy = y + flickAmp * (wave(y * 3 + t2) - 0.5)
  fz = z + flickAmp * (wave(z * 3 + t3) - 0.5)

  heat = 0
  for (i = 0; i < numSources; i++) {
    s = sources[i]
    dx = fx - s[0]
    dy = fy - s[1]
    dz = fz - s[2]
    d2 = dx * dx + dy * dy + dz * dz
    // Inverse-square falloff, soft-clamped near the source.
    heat += reach2 / (reach2 + d2 * 3) * envelope(s[3], s[4])
  }

  heat = heat * sliderIntensity * 2.2

  // Fire palette: dark red -> orange -> yellow -> white at the core.
  h = 0.02 + clamp(heat, 0, 1) * 0.10
  sat = clamp(1.3 - heat * 1.1, 0, 1)
  val = clamp(heat * heat * 1.8, 0, 1)

  hsv(h, sat, val)
}

// Sensible 2D / 1D fallbacks so the preview works at any dimensionality.
export function render2D(index, x, y) {
  render3D(index, x, y, 0.5)
}

export function render(index) {
  render3D(index, index / pixelCount, 0.5, 0.5)
}
