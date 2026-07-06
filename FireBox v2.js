/*
  FireBox v2  -- rising cuboid fire

  A hot core sits at the bottom of the shape and turbulent flames lick
  upward and around the walls, cooling to dark at the tips. Uses the
  hardware-validated cuboid map (z=0 bottom, z=1 top; x,y wrap the four
  walls), so 3D noise sampled at each surface pixel is seamless across the
  corner seams.

  Cheap by design: one turbulent-noise sample per pixel that scrolls
  upward over time -- no per-pixel ember/trail loop.

  Sliders use the function-handler form so they bind reliably to the
  running pattern.
*/

// ---- control values (driven by the slider handlers below) ----
ctlIntensity   = 0.75  // overall heat / brightness
ctlFlameHeight = 0.6   // how far up the flames reach
ctlTurbulence  = 0.55  // ragged tongues vs. smooth glow
ctlSpeed       = 0.5   // how fast flames rise
ctlColor       = 0.05  // base flame hue (0=red, .08=orange, .15=yellow, .6=blue ...)
ctlColorSpread = 0.5   // how far hue shifts from cool base to hot tips
ctlColorCycle  = 0     // drift the base hue over time (0 = static color)
ctlWhiteHot    = 0.25  // how much the hottest flame washes to white (0 = none)

export function sliderIntensity(v)   { ctlIntensity   = v }
export function sliderFlameHeight(v) { ctlFlameHeight = v }
export function sliderTurbulence(v)  { ctlTurbulence  = v }
export function sliderSpeed(v)       { ctlSpeed       = v }
export function sliderColor(v)       { ctlColor       = v }
export function sliderColorSpread(v) { ctlColorSpread = v }
export function sliderColorCycle(v)  { ctlColorCycle  = v }
export function sliderWhiteHot(v)    { ctlWhiteHot    = v }

scroll = 0
swirl  = 0
flick  = 1
flameH = 1
turbAmt = 1
intensity = 1
hueBase = 0.05
hueSpread = 0.08
whiteHot = 0.25
hueDrift = 0

export function beforeRender(delta) {
  dt = delta * 0.001
  scroll += dt * (0.5 + ctlSpeed * 2.5)      // upward flow
  if (scroll > 256) scroll -= 256            // keep noise coords bounded
  swirl += dt * 0.35                         // slow drift so flames lick sideways
  if (swirl > 256) swirl -= 256
  flick = 0.86 + 0.14 * wave(time(0.015))    // global flicker
  flameH = 0.35 + ctlFlameHeight * 0.95      // flame reach in z units
  turbAmt = ctlTurbulence
  intensity = 0.7 + ctlIntensity * 1.3

  // ---- color ----
  hueDrift += dt * ctlColorCycle * 0.066     // 0 = static; full spectrum ~15s at max
  if (hueDrift > 1) hueDrift -= 1
  hueBase   = ctlColor + hueDrift             // base hue (wraps in hsv); full spectrum
  hueSpread = ctlColorSpread * 0.16          // hue rise from base (cool) to core (hot)
  whiteHot  = ctlWhiteHot                     // 0 = stay saturated, 1 = white-hot core
}

export function render3D(index, x, y, z) {
  // Turbulent noise on the box surface, scrolling upward, with a gentle
  // horizontal drift so the tongues weave from side to side.
  n = perlinTurbulence(x * 2.1 + wave(swirl + y) * 0.15,
                       y * 2.1,
                       z * 3.0 - scroll,
                       2, 0.5, 3)            // ~0..1, turbulent
  turb = (1 - turbAmt) + n * (turbAmt * 1.9)

  // Height gradient: 1 at the base, 0 at the top of the flame region.
  h = z / flameH

  // Fire trick: heat = turbulence - height. Tall flickering tongues where
  // the noise is high; always hot at the base (h ~ 0).
  heat = (turb - h) * intensity
  heat = clamp(heat * flick, 0, 1)

  // Extra near-white core in the very bottom rows.
  heat = clamp(heat + clamp((0.12 - z) * 6, 0, 1) * 0.45, 0, 1)

  // Palette: dark base color -> hotter hue toward the core, only washing
  // to white where whiteHot allows.
  hue = hueBase + heat * hueSpread
  sat = clamp(1 - heat * heat * whiteHot * 1.4, 0, 1)
  val = clamp(heat * heat * 1.35, 0, 1)
  hsv(hue, sat, val)
}

export function render2D(index, x, y) { render3D(index, x, y, 0.2) }
export function render(index) { render3D(index, index / pixelCount, 0.5, 0.15) }
