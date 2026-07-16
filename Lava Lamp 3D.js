/*
  Lava Lamp 3D -- heated-wax blobs for the LED cuboid

  The cuboid is the lamp: a warm heater glow sits at the base, and colorful
  blobs of "wax" form there, heat up, rise with a wobble, flex and stretch,
  stall as they cool near the top, then sink back down to reheat. The four
  walls are the glass -- each pixel shows the 3D metaball field evaluated at
  its position on the glass, so blobs brighten as they drift near a wall and
  glow softly through the liquid when they're mid-tank. Overlapping blobs
  merge organically (metaball field sum).

  Uses the hardware-validated cuboid map (z=0 bottom, z=1 top; x,y span the
  square footprint with the walls at the 0/1 edges).

  Physics runs per-blob in beforeRender (cheap); render3D does one distance
  eval per blob per pixel. The blob field uses a division-free Wyvill-style
  kernel, (1 - d^2/R^2)^2 with the reciprocal precomputed per blob -- the
  classic r^2/d^2 metaball needs a divide per blob per pixel, which drops a
  1024-pixel v3 to ~7 fps.
*/

var MAXB = 6

// ---- control values (driven by the slider handlers below) ----
ctlSpeed  = 0.5   // how fast the lamp runs (heating, rise, wobble)
ctlSize   = 0.5   // blob radius
ctlCount  = 0.8   // number of blobs, 2..6
ctlHeight = 0.85  // how high the blobs rise before stalling and sinking
ctlHue    = 0.0   // base hue of the first blob
ctlSpread = 1.0   // how far the other blobs' hues fan out around the wheel
ctlGlow   = 0.5   // brightness of the heater glow at the base

export function sliderSpeed(v)       { ctlSpeed  = v }
export function sliderBlobSize(v)    { ctlSize   = v }
export function sliderBlobs(v)       { ctlCount  = v }
export function sliderRiseHeight(v)  { ctlHeight = v }
export function sliderHue(v)         { ctlHue    = v }
export function sliderColorSpread(v) { ctlSpread = v }
export function sliderGlow(v)        { ctlGlow   = v }

// ---- tuning constants ----
var zAspect = 0.35  // vertical shape of a blob: walls are 8px wide x 32px
                    // tall, so normalized z is ~4x denser in pixels; 0.25
                    // would be physically spherical, a bit more looks lava-y
var THRESH  = 0.5   // field level where a blob surface starts
var EDGE    = 1.6   // how hard the blob edge ramps from the threshold
var HALO    = 2     // kernel support radius, in blob radii (soft halo size)

// ---- blob state ----
bx    = array(MAXB)  // position
by    = array(MAXB)
bz    = array(MAXB)
bvz   = array(MAXB)  // vertical velocity
btemp = array(MAXB)  // temperature: >0.55 rises, <0.55 sinks
bang  = array(MAXB)  // home angle in the footprint
bph   = array(MAXB)  // wobble phase offset
bir2  = array(MAXB)  // per-frame: 1 / (support radius)^2
bzi   = array(MAXB)  // per-frame: 1 / (zAspect * stretch)
bspan = array(MAXB)  // per-frame: vertical reach in z units, for cheap culling
bhue  = array(MAXB)

// stagger the blobs through the rise/sink cycle so they don't move in unison
for (i = 0; i < MAXB; i++) {
  bang[i]  = i / MAXB
  bph[i]   = i * 2.6
  bz[i]    = 0.06 + (i % 3) * 0.31
  btemp[i] = 0.25 + (i * 0.13) % 0.55
  bvz[i]   = 0
}

wclock = 0
orb = 0
nBlobs = 4
glowLevel = 0.5

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  sdt = dt * (0.25 + ctlSpeed * 1.75)   // speed slider scales sim time

  nBlobs = floor(2 + ctlCount * 4.99)   // 2..6
  baseR = 0.13 + ctlSize * 0.22
  glowLevel = ctlGlow
  // liquid temperature gradient: buoyancy fades with altitude, so this sets
  // where a fully heated blob stalls -- ~1/4 height at 0, the top at 1
  grad = 1.5 - ctlHeight * 1.3

  // wobble clock: wave() coefficients below are multiples of 1/16 so the
  // wrap at 16 lands on a whole period (no visual jump)
  wclock += sdt * 0.0625
  if (wclock >= 16) wclock -= 16
  orb += sdt * 0.02                     // slow orbit of the home positions
  if (orb >= 1) orb -= 1

  for (i = 0; i < nBlobs; i++) {
    // thermal: the heater pumps heat in near the base; ambient cooling is
    // slow -- it drives the sink-and-reheat cycle, not the stall height
    if (bz[i] < 0.16) btemp[i] += sdt * 0.17
    btemp[i] = clamp(btemp[i] - sdt * 0.02, 0, 1)

    // buoyancy vs. drag: hot wax rises until the liquid's temperature
    // gradient neutralizes it (RiseHeight slider), cool wax sinks
    bvz[i] += (btemp[i] - 0.55 - bz[i] * grad) * 0.07 * sdt
    bvz[i] -= bvz[i] * 0.5 * sdt
    bz[i] += bvz[i] * sdt
    if (bz[i] < 0.06) { bz[i] = 0.06; if (bvz[i] < 0) bvz[i] = 0 }
    if (bz[i] > 0.94) { bz[i] = 0.94; if (bvz[i] > 0) bvz[i] = 0 }

    // home point orbits the center slowly; wobble meanders around it
    ang = (bang[i] + orb) * PI2
    w = wclock + bph[i]
    bx[i] = 0.5 + cos(ang) * 0.17 + (wave(w * 0.6875) - 0.5) * 0.26
    by[i] = 0.5 + sin(ang) * 0.17 + (wave(w * 0.9375) - 0.5) * 0.26

    // flexing: radius slowly pulses; rising/sinking blobs stretch vertically
    r = HALO * baseR * (0.85 + 0.3 * wave(w * 1.1875))
    bir2[i] = 1 / (r * r)
    stretch = zAspect * (1 + min(abs(bvz[i]) * 8, 1.2))
    bzi[i] = 1 / stretch
    bspan[i] = r * stretch   // beyond this |z - bz|, the kernel is zero

    bhue[i] = ctlHue + i * ctlSpread / MAXB
  }
}

export function render3D(index, x, y, z) {
  f = 0
  best = 0
  hue = 0
  for (i = 0; i < nBlobs; i++) {
    // cheap vertical-band cull before the full distance math
    dz = z - bz[i]
    if (dz < bspan[i] && -dz < bspan[i]) {
      dx = x - bx[i]
      dy = y - by[i]
      dz *= bzi[i]
      q = 1 - (dx * dx + dy * dy + dz * dz) * bir2[i]
      if (q > 0) {
        c = q * q
        f += c
        if (c > best) { best = c; hue = bhue[i] }
      }
    }
  }

  v = clamp((f - THRESH) * EDGE, 0, 1)
  v = v * v * (3 - 2 * v)   // smooth the blob edge
  v = max(v, f * 0.2)       // blobs deep in the tank glow faintly through

  if (v > 0.02) {
    // glossy hot core: desaturate a touch where the field is strongest
    sat = 1 - clamp((f - 1.1) * 0.5, 0, 0.35)
    hsv(hue, sat, v)
  } else {
    // the liquid: warm heater glow at the base fading to dark above
    g = clamp(1 - z * 3.3, 0, 1)
    hsv(0.02, 1, g * g * glowLevel * 0.55)
  }
}

export function render2D(index, x, y) { render3D(index, x, 0.5, y) }
export function render(index) { render3D(index, 0.5, 0.5, index / pixelCount) }
