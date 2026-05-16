/*
  Cube Fire 3D

  Random hotspots ignite on the surface of a 5-sided LED cube, glow with
  flickering heat, and fade away. Rendering in 3D lets the Pixelblaze pixel
  map carry heat continuously across panel seams without any special wiring.

  Performance: heavy work (envelope, jitter, palette inputs) happens once per
  source per frame in beforeRender. The per-pixel render3D loop is just
  distance-squared + one divide per source, so it scales well on 1280 pixels.
*/

export var sliderIntensity = 0.6
export var sliderActivity  = 0.5   // hotspot lifetime / respawn rate
export var sliderReach     = 0.5   // how far each hotspot spreads

numSources = 5

// Flat parallel arrays — faster to index than an array of small arrays.
sx     = array(numSources)
sy     = array(numSources)
sz     = array(numSources)
sbx    = array(numSources)  // base (un-jittered) position
sby    = array(numSources)
sbz    = array(numSources)
sage   = array(numSources)
slife  = array(numSources)
samp   = array(numSources)  // precomputed per-frame amplitude
sphase = array(numSources)  // per-source jitter phase

function spawn(idx, initialAge) {
  face = floor(random(5))
  u = random(1)
  v = random(1)
  if      (face == 0) { sbx[idx] = u; sby[idx] = v; sbz[idx] = 1 }  // bottom
  else if (face == 1) { sbx[idx] = 0; sby[idx] = u; sbz[idx] = v }  // left
  else if (face == 2) { sbx[idx] = u; sby[idx] = 1; sbz[idx] = v }  // back
  else if (face == 3) { sbx[idx] = 1; sby[idx] = u; sbz[idx] = v }  // right
  else                { sbx[idx] = u; sby[idx] = 0; sbz[idx] = v }  // front
  life = 1.5 + random(2.5)
  slife[idx] = life
  sage[idx]  = initialAge * life
  sphase[idx] = random(1)
}

// Stagger initial ages so the cube isn't dark or all-bright at startup.
for (i = 0; i < numSources; i++) spawn(i, random(1))

reach2 = 0.04
ampScale = 1
jitterPhase = 0

export function beforeRender(delta) {
  dt = delta * 0.001
  ageRate = 0.4 + sliderActivity * 1.6
  reach = 0.12 + sliderReach * 0.28
  reach2 = reach * reach
  // Pre-fold reach2 into the amplitude so the divide in render3D matches it.
  ampScale = sliderIntensity * 2.4 * reach2
  jitterPhase += dt

  for (i = 0; i < numSources; i++) {
    sage[i] += dt * ageRate
    if (sage[i] >= slife[i]) spawn(i, 0)

    // wave() of p in [0..1] is a smooth 0 -> 1 -> 0 sinusoidal envelope.
    // No hard breakpoints, so brightness can't pop when a source respawns.
    p = sage[i] / slife[i]
    samp[i] = wave(p) * ampScale

    // Slow wander around the spawn point gives the field a lapping motion
    // without any per-pixel turbulence cost.
    ph = sphase[i] + jitterPhase * 0.6
    sx[i] = sbx[i] + 0.05 * (wave(ph) - 0.5)
    sy[i] = sby[i] + 0.05 * (wave(ph + 0.33) - 0.5)
    sz[i] = sbz[i] + 0.05 * (wave(ph + 0.67) - 0.5)
  }
}

export function render3D(index, x, y, z) {
  heat = 0
  for (i = 0; i < numSources; i++) {
    dx = x - sx[i]
    dy = y - sy[i]
    dz = z - sz[i]
    d2 = dx * dx + dy * dy + dz * dz
    heat += samp[i] / (reach2 + d2 * 3)
  }

  // Fire palette: dark red -> orange -> yellow -> white at the core.
  h   = 0.02 + clamp(heat, 0, 1) * 0.10
  sat = clamp(1.3 - heat * 1.1, 0, 1)
  val = clamp(heat * heat * 1.8, 0, 1)
  hsv(h, sat, val)
}

export function render2D(index, x, y) {
  render3D(index, x, y, 0.5)
}

export function render(index) {
  render3D(index, index / pixelCount, 0.5, 0.5)
}
