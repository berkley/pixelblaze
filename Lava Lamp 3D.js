/*
  Lava Lamp 3D -- heated-wax blobs for the LED cuboid

  The cuboid is the lamp: a warm heater glow sits at the base, and colorful
  blobs of "wax" form there, heat up, rise with a wobble, flex and stretch,
  stall as they cool near the top, then sink back down to reheat. The four
  walls are the glass, and the liquid is transparent: each wall renders every
  blob by orthographic projection straight through the tank (horizontal +
  height position only, depth ignored except for a mild haze), so the same
  blob is visible from all four sides at its true world position, just like
  looking through a real lamp. Blobs physically collide -- they push apart,
  ooze around each other, and exchange heat, mutating both trajectories --
  and their metaball fields merge organically while they're in contact.

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
var HAZE    = 0.25  // how much the liquid dims a blob seen from the far wall
var wallStep = 1 / MAXB  // turns a proj/att block offset back into a wall number

// ---- blob state ----
bx    = array(MAXB)  // position
by    = array(MAXB)
bz    = array(MAXB)
bvx   = array(MAXB)  // velocity (horizontal motion is physical so collision
bvy   = array(MAXB)  //   impulses persist instead of being overwritten)
bvz   = array(MAXB)
btemp = array(MAXB)  // temperature: >0.55 rises, <0.55 sinks
bang  = array(MAXB)  // home angle in the footprint
bph   = array(MAXB)  // wobble phase offset
bir2  = array(MAXB)  // per-frame: 1 / (support radius)^2
bzi   = array(MAXB)  // per-frame: 1 / (zAspect * stretch)
bspan = array(MAXB)  // per-frame: vertical reach in z units, for cheap culling
bhue  = array(MAXB)
// per-frame, per wall (front/back/left/right blocks of MAXB):
proj  = array(4 * MAXB)  // blob's horizontal coordinate projected onto the wall
att   = array(4 * MAXB)  // haze dimming by distance behind that wall

// stagger the blobs through the rise/sink cycle so they don't move in unison
for (i = 0; i < MAXB; i++) {
  bang[i]  = i / MAXB
  bph[i]   = i * 2.6
  bx[i]    = 0.5 + cos(bang[i] * PI2) * 0.2
  by[i]    = 0.5 + sin(bang[i] * PI2) * 0.2
  bz[i]    = 0.06 + (i % 3) * 0.31
  btemp[i] = 0.25 + (i * 0.13) % 0.55
  bvx[i]   = 0
  bvy[i]   = 0
  bvz[i]   = 0
}

wclock = 0
orb = 0
nBlobs = 4
glowLevel = 0.5
fl1 = 0
fl2 = 0
ft = 0
gfl = 1

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

  // fire clocks for the heater flames: real-time rates (not the Speed
  // slider) so the fire dances fast even when the lava is glacial; wave()
  // coefficients on fl1/fl2 are 1, so wrapping at 16 lands on a whole period
  fl1 += dt * 0.9
  if (fl1 >= 16) fl1 -= 16
  fl2 += dt * 1.4
  if (fl2 >= 16) fl2 -= 16
  ft += dt * 2.2
  if (ft >= 16) ft -= 16
  gfl = 0.75 + 0.25 * wave(ft)          // global fast flicker

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

    // horizontal wander: a meandering force plus a gentle spring toward a
    // slowly orbiting home point; velocity-based so collisions can shove
    ang = (bang[i] + orb) * PI2
    w = wclock + bph[i]
    hx = 0.5 + cos(ang) * 0.15
    hy = 0.5 + sin(ang) * 0.15
    bvx[i] += ((wave(w * 0.6875) - 0.5) * 0.06 + (hx - bx[i]) * 0.08) * sdt
    bvy[i] += ((wave(w * 0.9375) - 0.5) * 0.06 + (hy - by[i]) * 0.08) * sdt
    bvx[i] -= bvx[i] * 0.8 * sdt
    bvy[i] -= bvy[i] * 0.8 * sdt
    bx[i] += bvx[i] * sdt
    by[i] += bvy[i] * sdt
    if (bx[i] < 0.12) { bx[i] = 0.12; if (bvx[i] < 0) bvx[i] = 0 }
    if (bx[i] > 0.88) { bx[i] = 0.88; if (bvx[i] > 0) bvx[i] = 0 }
    if (by[i] < 0.12) { by[i] = 0.12; if (bvy[i] < 0) bvy[i] = 0 }
    if (by[i] > 0.88) { by[i] = 0.88; if (bvy[i] > 0) bvy[i] = 0 }

    // flexing: radius slowly pulses; rising/sinking blobs stretch vertically
    r = HALO * baseR * (0.85 + 0.3 * wave(w * 1.1875))
    bir2[i] = 1 / (r * r)
    stretch = zAspect * (1 + min(abs(bvz[i]) * 8, 1.2))
    bzi[i] = 1 / stretch
    bspan[i] = r * stretch   // beyond this |z - bz|, the kernel is zero

    bhue[i] = ctlHue + i * ctlSpread / MAXB

    // projection tables for the see-through render: front/back walls see the
    // blob at its x, side walls at its y; haze dims with distance behind
    proj[i]            = bx[i]
    proj[MAXB + i]     = bx[i]
    proj[2 * MAXB + i] = by[i]
    proj[3 * MAXB + i] = by[i]
    att[i]             = 1 - by[i] * HAZE        // front wall (y=0)
    att[MAXB + i]      = 1 - (1 - by[i]) * HAZE  // back  (y=1)
    att[2 * MAXB + i]  = 1 - bx[i] * HAZE        // left  (x=0)
    att[3 * MAXB + i]  = 1 - (1 - bx[i]) * HAZE  // right (x=1)
  }

  // collisions: overlapping blobs push apart like immiscible oil, and
  // exchange heat so a collision mutates both blobs' rise/sink cycles
  minD = baseR * 1.9
  for (i = 0; i < nBlobs - 1; i++) {
    for (j = i + 1; j < nBlobs; j++) {
      ddx = bx[j] - bx[i]
      ddy = by[j] - by[i]
      ddz = (bz[j] - bz[i]) / zAspect   // compare in round-blob space
      d2 = ddx * ddx + ddy * ddy + ddz * ddz
      if (d2 < minD * minD) {
        d = sqrt(d2) + 0.001
        push = (minD - d) * 0.8 * sdt / d   // soft spring, so they ooze apart
        bvx[i] -= ddx * push; bvx[j] += ddx * push
        bvy[i] -= ddy * push; bvy[j] += ddy * push
        pz = ddz * push * zAspect
        bvz[i] -= pz; bvz[j] += pz
        mt = (btemp[i] + btemp[j]) * 0.5
        btemp[i] += (mt - btemp[i]) * sdt
        btemp[j] += (mt - btemp[j]) * sdt
      }
    }
  }
}

export function render3D(index, x, y, z) {
  // which pane of glass is this pixel on? wb indexes that wall's block in
  // the proj/att tables; u is the pixel's horizontal position on the pane
  if (y < 0.01)      { wb = 0;        u = x }
  else if (y > 0.99) { wb = MAXB;     u = x }
  else if (x < 0.01) { wb = 2 * MAXB; u = y }
  else               { wb = 3 * MAXB; u = y }

  f = 0
  best = 0
  hue = 0
  for (i = 0; i < nBlobs; i++) {
    // cheap vertical-band cull before the full distance math
    dz = z - bz[i]
    if (dz < bspan[i] && -dz < bspan[i]) {
      du = u - proj[wb + i]   // see-through: depth doesn't move or shrink it
      dz *= bzi[i]
      q = 1 - (du * du + dz * dz) * bir2[i]
      if (q > 0) {
        c = q * q * att[wb + i]
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
  } else if (z < 0.45) {
    // the heater is a fire: two drifting waves along the perimeter make
    // flame tongues that vary in height and brightness, red at the roots
    // shifting toward yellow in the hottest spots
    s = u + wb * wallStep             // rough perimeter coordinate
    n = wave(s * 3 + fl1) * 0.6 + wave(s * 5 - fl2) * 0.4
    g = clamp(1 - z * (4.6 - 2.3 * n), 0, 1)
    hsv(0.015 + 0.045 * n * g, 1, g * g * (0.35 + 0.55 * n) * gfl * glowLevel * 1.3)
  } else {
    hsv(0, 0, 0)
  }
}

export function render2D(index, x, y) { render3D(index, x, 0, y) }
export function render(index) { render3D(index, 0.5, 0, index / pixelCount) }
