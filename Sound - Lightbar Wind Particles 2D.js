/*
  Sound - Lightbar Wind Particles 2D

  For the DJ Booth Lightbar (pixelmaps/halfcuboid-bar.js): coloured particles
  blowing along the 160x16 banner like leaves in a gusty wind, leaving fading
  tails, with their motion and colour driven by the music.

  WIND is gusty and reverses. Two wave() oscillators at incommensurate rates sum
  to a signed drift that swings back and forth along the bar every 10-16s,
  scaled by how loud the music currently is. On top of that sits a perlin flow
  field sampled per particle, which is what makes the paths weave rather than
  run straight.

  MOTION is why this reads as leaves rather than sprites: particles never get
  assigned a velocity, they CHASE the wind velocity with drag. The momentum that
  leaves behind is the whole effect -- a particle keeps coasting when the gust
  drops, and banks into a turn instead of snapping to it.

  A BEAT does two things: it jumps the noise field's z coordinate, so the whole
  flow re-shuffles and particles visibly change heading; and it spawns a burst at
  whichever edge is currently upwind, so density pulses with the music.

  COLOUR comes from the spectrum. Each particle is bound to one of the 32 FFT
  bins, chosen at spawn by sampling three bins and keeping the loudest, so the
  swarm's palette tracks whatever is actually playing. Hue runs bass=red through
  treble=violet, and a particle flares when its own bin is loud.

  Beat bursts are the exception: `BassBoost` of them take their colour from the
  loudest sub-bass bin instead, so a kick throws a visible slug of red. Sampling
  three bins out of thirty-two only lands on a bass bin about a third of the
  time, which was not nearly often enough to read the kick off the bar.

  TAILS reuse the decay idiom from "Bouncing Box Trails 2D": the trail buffer is
  decayed lazily inside render rather than in a separate pass, so each pixel is
  touched exactly once per frame and the tails cost almost nothing. beforeRender
  splats particles into that same buffer with max-wins, which also latches each
  trail's colour to whatever laid it down.

  One deliberate change from that original: its decay is a flat per-frame
  multiply, so tail length varies with framerate. This bar swings between 10 and
  21 fps, so decay is computed as pow(halfLife) against real elapsed time and
  tails last a fixed number of SECONDS instead.

  Particles are splatted bilinearly across the four pixels around their
  fractional position. Sub-pixel placement is not optional -- snapping a small
  sprite to whole pixels turns smooth motion into visible stepping, which is
  exactly the failure the sliding-boxes pattern ran into.

  perlin(x, y, z, seed) measured -0.82..0.83 with mean ~0 on this controller
  (v3.51), so it is used directly as a signed field with no re-centring.

  Works against either variant of the map: render3D uses x and z, and render2D
  reconstructs z from the flattened 160x16 plane.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var particlesCtrl  = 0.55   // how many particles to hold in the air
export function sliderParticles(v) { particlesCtrl = v }

var tailCtrl = 0.35         // tail half-life, 0.05s .. 1.2s
export function sliderTail(v) { tailCtrl = v }

var windCtrl = 0.5          // gust strength
export function sliderWind(v) { windCtrl = v }

var turbCtrl = 0.5          // how much the flow field deflects particles
export function sliderTurbulence(v) { turbCtrl = v }

var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

// What fraction of each beat burst is coloured from the sub-bass bins (i.e.
// comes out red/orange) rather than from the spectrum at large. At 0 the burst
// is coloured like everything else and the kick is hard to pick out.
var bassBoostCtrl = 0.75
export function sliderBassBoost(v) { bassBoostCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm      = 120
export var bassSpike        = 0
export var beatCount        = 0
export var activeParticles  = 0
export var windX            = 0
export var energyNorm       = 0

// ---- Geometry ----
barW = 160
barH = 16
gridSize = barW * barH        // 2560
maxParticles = 160
bins = 32

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0

// ---- Wind state ----
wclock   = 0    // gust oscillator clock, wraps at 256 on whole periods
nScroll  = 0    // noise field drift
turbSeed = 0    // jumped by each beat to re-shuffle the field

// ---- Spectrum AGC ----
// Raw frequencyData on this device runs about 0.0003..0.003, well below the
// 0.006..0.02 the repo's fixed-gain patterns assume, so every band is
// normalised against its own rolling mean and decaying peak instead of scaled
// by a constant.
bandLvl   = array(bins)
bandPeak  = array(bins)
bandSpike = array(bins)
totAvg  = 0
totPeak = 0

// ---- Particles (SoA, per repo convention) ----
px    = array(maxParticles)
py    = array(maxParticles)
pvx   = array(maxParticles)
pvy   = array(maxParticles)
pbin  = array(maxParticles)
plife = array(maxParticles)   // remaining seconds; <= 0 means the slot is free
pmax  = array(maxParticles)   // total lifespan, for the fade envelope

// ---- Trail framebuffer, indexed by pixel index ----
pixHue = array(gridSize)
pixVal = array(gridSize)
idxOf  = array(gridSize)      // grid position (row * barW + col) -> pixel index

spawnAccum = 0
decay = 0.5
frames = 0
mapped = 0

for (initIdx = 0; initIdx < maxParticles; initIdx++) plife[initIdx] = 0

function randRange(a, b) { return a + (b - a) * random(1) }

// Pick a bin by sampling three and keeping the loudest, so spawn colour tracks
// whatever is actually playing without a full scan of all 32 every spawn.
function pickBin() {
  c1 = floor(random(bins))
  c2 = floor(random(bins))
  c3 = floor(random(bins))
  best = c1
  if (bandSpike[c2] > bandSpike[best]) best = c2
  if (bandSpike[c3] > bandSpike[best]) best = c3
  return best
}

// Loudest of the sub-bass bins -- the same 0..3 range the beat detector itself
// watches, so a burst is coloured by whatever actually triggered it. These map
// to hue 0 .. 0.07, i.e. red through orange.
function pickBassBin() {
  best = 0
  for (c = 1; c < 4; c++) if (bandSpike[c] > bandSpike[best]) best = c
  return best
}

// Spawn at whichever end is upwind, nudged a random distance inboard so
// arrivals don't line up along the edge.
function spawnOne(useBass) {
  for (s = 0; s < maxParticles; s++) {
    if (plife[s] <= 0) {
      if (windX >= 0) px[s] = random(6)
      else px[s] = barW - 1 - random(6)
      py[s] = random(barH - 1)
      pvx[s] = windX
      pvy[s] = 0
      pbin[s] = useBass ? pickBassBin() : pickBin()
      pmax[s] = randRange(3, 8)
      plife[s] = pmax[s]
      return 1
    }
  }
  return 0
}

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05   // physics patterns clamp this; a stall must not teleport anything

  elapsed += dt

  // One full render pass has completed by the second beforeRender, so idxOf[]
  // is populated and particles can be splatted into the framebuffer.
  frames++
  if (frames >= 2) mapped = 1

  // ---- Per-bin AGC ----
  dw = delta / 2000
  totalNow = 0
  for (i = 0; i < bins; i++) {
    lvl = frequencyData[i]
    totalNow += lvl
    bandLvl[i] = bandLvl[i] * (1 - dw) + lvl * dw
    bandPeak[i] = bandPeak[i] * 0.997
    if (lvl > bandPeak[i]) bandPeak[i] = lvl
    dyn = max(max(bandPeak[i] - bandLvl[i], bandPeak[i] * 0.2), 0.00005)
    bandSpike[i] = min(max(0, lvl - bandLvl[i]) / dyn, 1)
  }
  totalNow /= bins

  // Sustained loudness, not a transient: wind strength should follow how loud
  // the music is, not spike on every hit.
  totAvg = totAvg * (1 - dw) + totalNow * dw
  totPeak = totPeak * 0.997
  if (totalNow > totPeak) totPeak = totalNow
  energyNorm = min(totalNow / max(totPeak, 0.00005), 1)

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

  // ---- Wind ----
  // Both coefficients are exact binary fractions, so wrapping wclock at 256
  // lands on a whole number of periods for each and the gust never jumps.
  wclock += dt
  if (wclock >= 256) wclock -= 256
  gust = (wave(wclock * 0.0625) - 0.5) + 0.6 * (wave(wclock * 0.09375) - 0.5)
  windX = (20 + windCtrl * 70) * gust * (0.35 + energyNorm * 1.5)

  nScroll += dt * 0.25
  if (nScroll >= 256) nScroll -= 256

  // ---- Population ----
  // The slider sets the PEAK population. The trickle only holds a fraction of
  // it, and beat bursts fill the rest -- that gap is what makes density pulse
  // with the music. Letting bursts spawn freely instead would swamp the slider:
  // at two beats a second they alone settle the population near 75, so the low
  // end of the control did nothing at all.
  target = 16 + floor(particlesCtrl * (maxParticles - 16))
  alive = 0
  for (i = 0; i < maxParticles; i++) if (plife[i] > 0) alive++

  if (beat) {
    beatCount++
    turbSeed += 7 + random(40)      // re-shuffle the flow field
    if (turbSeed >= 4096) turbSeed -= 4096
    // Burst size grows with the gap left to fill, so a high Particles setting
    // actually reaches its target. With a fixed burst size the top half of the
    // slider did almost nothing -- 0.55, 0.8 and 1.0 all settled near 80.
    headroom = target - alive
    burst = 3 + floor(min(bassSpike, 1) * 9 + max(0, headroom) * 0.3)
    if (burst > headroom) burst = headroom
    for (bq = 0; bq < burst; bq++) {
      if (!spawnOne(random(1) < bassBoostCtrl)) break
      alive++
    }
  }

  // ---- Trickle spawning toward the resting population ----
  restTarget = target * 0.55
  if (alive < restTarget) {
    spawnAccum += (restTarget - alive) * 1.5 * dt
    while (spawnAccum >= 1) {
      spawnAccum -= 1
      if (!spawnOne(0)) break
      alive++
    }
  } else {
    spawnAccum = 0
  }

  // ---- Integrate + splat ----
  turbAmp = 15 + turbCtrl * 55
  resp = 2.5                     // low, so particles carry momentum
  zc = nScroll + turbSeed
  tailHalfLife = 0.05 + tailCtrl * 1.15
  decay = pow(0.5, dt / tailHalfLife)

  alive = 0
  for (i = 0; i < maxParticles; i++) {
    if (plife[i] <= 0) continue
    plife[i] -= dt
    if (plife[i] <= 0) continue
    alive++

    nx = px[i] * 0.035
    ny = py[i] * 0.09
    tvx = perlin(nx, ny, zc, 0)
    tvy = perlin(nx + 31.7, ny + 17.3, zc, 0)

    // Vertical room is only 16px against 160 horizontal, so the vertical
    // component is scaled back or particles just slam between the edges.
    vx = pvx[i] + (windX + tvx * turbAmp - pvx[i]) * resp * dt
    vy = pvy[i] + (tvy * turbAmp * 0.45 - pvy[i]) * resp * dt
    pvx[i] = vx
    pvy[i] = vy

    nxp = px[i] + vx * dt
    nyp = py[i] + vy * dt

    nxp = nxp % barW
    if (nxp < 0) nxp += barW

    if (nyp < 0) { nyp = -nyp; pvy[i] = -pvy[i] * 0.6 }
    else if (nyp > barH - 1) { nyp = 2 * (barH - 1) - nyp; pvy[i] = -pvy[i] * 0.6 }

    px[i] = nxp
    py[i] = nyp

    if (mapped) {
      // Fast attack, slow release. A symmetric wave() envelope is smoother but
      // is ZERO at birth and only peaks at mid-life, so a particle spawned on a
      // kick took seconds to become visible -- which made the bass impossible
      // to see, since the burst is the whole point of it. 0.15s of attack is
      // still 2-3 frames here, enough not to pop.
      age = pmax[i] - plife[i]
      att = age * 6.67
      if (att > 1) att = 1
      rel = plife[i]
      if (rel > 1) rel = 1
      bri = att * rel * (0.25 + 0.75 * bandSpike[pbin[i]])
      if (bri > 0.01) {
        hu = pbin[i] * (0.72 / 31)   // bin 0 red -> bin 31 violet

        ix = floor(nxp); fx = nxp - ix
        iy = floor(nyp); fy = nyp - iy
        ix1 = ix + 1
        if (ix1 >= barW) ix1 -= barW   // x wraps; y does not

        w00 = (1 - fx) * (1 - fy) * bri
        w10 = fx * (1 - fy) * bri
        w01 = (1 - fx) * fy * bri
        w11 = fx * fy * bri

        rb = iy * barW
        q = idxOf[rb + ix];  if (w00 > pixVal[q]) { pixVal[q] = w00; pixHue[q] = hu }
        q = idxOf[rb + ix1]; if (w10 > pixVal[q]) { pixVal[q] = w10; pixHue[q] = hu }
        if (iy + 1 < barH) {
          rb = (iy + 1) * barW
          q = idxOf[rb + ix];  if (w01 > pixVal[q]) { pixVal[q] = w01; pixHue[q] = hu }
          q = idxOf[rb + ix1]; if (w11 > pixVal[q]) { pixVal[q] = w11; pixHue[q] = hu }
        }
      }
    }
  }
  activeParticles = alive
}

export function render3D(index, x, y, z) {
  if (!mapped) {
    // First pass: learn where this pixel sits on the 160x16 grid. Rounding to
    // the nearest of the known positions rather than binning a continuous
    // coordinate -- the mapper places pixels at exactly col/159 and (15-row)/15,
    // and floor(x * 160) would drop the last column into a 161st bin at x = 1.
    col = round(x * (barW - 1))
    row = round((1 - z) * (barH - 1))
    idxOf[row * barW + col] = index
    hsv(0, 0, 0)
  } else {
    // Trail decay happens here rather than in a separate sweep, so every pixel
    // is touched exactly once per frame.
    v = pixVal[index] * decay
    pixVal[index] = v
    if (v > 0.004) hsv(pixHue[index], 1, v)
    else hsv(0, 0, 0)
  }
}

export function render2D(index, x, y) {
  // On the flattened map y is height across the unrolled banner.
  render3D(index, x, 0, y)
}

export function render(index) {
  // No map at all: no grid to blow across, so just show the spectrum.
  hsv(index / pixelCount * 0.72, 1, bandSpike[floor(index / pixelCount * bins)])
}
