/*
  Sound - Sparks Center

  Sound-reactive variant of the stock Sparks Center, saved alongside it rather
  than replacing it.

  The original spawns sparks at the middle of the strip with random energy and
  lets them fly outward, decelerating under friction and depositing heat as they
  go, respawning once they fizzle out or run off an end. Here the kick spawns
  them: a beat launches a burst whose energy scales with how hard the bass hit,
  so the bar throws sparks on the downbeat instead of bubbling continuously.

  RESTAGED TO 2D, which is a real departure and worth being explicit about. On
  this bar the pixel index snakes: within each half cuboid it runs left to right
  along the upper face one 8px column at a time, then back right to left along
  the lower face. Sparks travelling in index space would therefore zigzag across
  faces rather than flying outward along the bar, which is the entire effect. So
  they travel along x on the 160x16 grid, each depositing heat into a full-height
  column. "Sound - Edgeburst" kept its 1D form because a symmetric expanding
  field survives the snake; discrete moving objects do not.

  Spark count is scaled to the 160 columns rather than the 2560 pixels -- the
  stock pixelCount/6 would be 426 sparks fighting over 160 columns, which just
  reads as a solid glow.

  Heat decay is a half-life in seconds, not the stock per-frame multiply, since
  the framerate here swings 10..21 and a per-frame decay changes the look with
  it.

  With no sensor board attached (`light` stays -1) it free-runs at 120 BPM.
*/

// ---- UI controls ----
// Pixelblaze builds sliders from exported `slider*` FUNCTIONS, not vars.
var sensitivityCtrl = 0.5   // beat detector threshold
export function sliderSensitivity(v) { sensitivityCtrl = v }

// How many sparks each kick launches, as a fraction of the pool.
var burstCtrl = 0.5
export function sliderBurstSize(v) { burstCtrl = v }

// How far they get before friction stops them.
var reachCtrl = 0.55
export function sliderReach(v) { reachCtrl = v }

// How long the heat lingers behind them.
var coolCtrl = 0.4
export function sliderCoolDown(v) { coolCtrl = v }

// ---- Sensor board inputs ----
export var light         = -1
export var frequencyData = array(32)
export var energyAverage = 0

// ---- Diagnostics ----
export var detectedBpm  = 120
export var bassSpike    = 0
export var beatCount    = 0
export var liveSparks   = 0

// ---- Beat detection state ----
bassAvg      = 0
bassPeak     = 0
lastBassBeat = -10
elapsed      = 0
beatPhase    = 0

// ---- Geometry ----
barW = 160
barH = 16
gridSize = barW * barH
midX = barW / 2

numSparks = 32          // scaled to the 160 columns, not the 2560 pixels
sparkE = array(numSparks)   // signed energy: sign is the direction of travel
sparkX = array(numSparks)   // position along the bar, 0..159

// Sparks light a full-height column, so only the column index is needed per
// pixel -- one 2560-element array instead of two.
colOf   = array(gridSize)
colHeat = array(barW)

frames = 0
mapped = 0

for (initIdx = 0; initIdx < numSparks; initIdx++) sparkE[initIdx] = 0

// Launch one spark from the middle. `power` scales the initial energy, so a
// harder kick throws them further.
function launch(s, power) {
  sparkX[s] = midX
  e = (0.35 + random(0.5)) * power
  sparkE[s] = random(1) > 0.5 ? e : -e
}

export function beforeRender(delta) {
  dt = delta * 0.001
  if (dt > 0.05) dt = 0.05
  elapsed += dt

  frames++
  if (frames >= 2) mapped = 1

  dw = delta / 2000

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

  // ---- Launch on the kick ----
  if (beat) {
    beatCount++
    power = 0.6 + min(bassSpike, 1) * 0.8
    want = 2 + floor(burstCtrl * (numSparks - 2))
    launched = 0
    for (s = 0; s < numSparks && launched < want; s++) {
      if (sparkE[s] == 0) { launch(s, power); launched++ }
    }
  }

  // ---- Cool the heat already deposited ----
  halfLife = 0.06 + coolCtrl * 0.5
  fade = pow(0.5, dt / halfLife)
  for (c = 0; c < barW; c++) colHeat[c] *= fade

  // ---- Move the sparks ----
  // Friction is per-second here, not per-frame as in the stock pattern, so
  // reach does not change with framerate.
  friction = (0.9 - reachCtrl * 0.75) * 60
  live = 0
  for (s = 0; s < numSparks; s++) {
    e = sparkE[s]
    if (e == 0) continue

    // Bleed energy, preserving the sign, and stop once it is spent.
    e -= friction * dt * (e > 0 ? 1 : -1) * 0.01
    if ((e > 0) != (sparkE[s] > 0) || abs(e) < 0.002) {
      sparkE[s] = 0
      continue
    }

    prevX = sparkX[s]
    x = prevX + e * 60 * dt
    if (x >= barW || x < 0) { sparkE[s] = 0; continue }

    sparkX[s] = x
    sparkE[s] = e
    live++

    // Deposit heat along every column the spark crossed this frame, a fixed
    // amount per column rather than a per-frame or per-dt lump at its landing
    // point. Heat then tracks distance travelled, which is what makes it
    // framerate-independent -- and a fast spark no longer skips columns and
    // leaves a dotted trail at low framerates. Sign is direction, so drop it.
    // 0.09 puts peak column heat near 1.2 in simulation. The stock render does
    // v = v*v then hsv(.63, 1-v, v), so anything much above 1 saturates to white
    // over a wide area instead of staying blue with hot white cores.
    heat = abs(e) * 0.09
    c0 = floor(min(prevX, x))
    c1 = floor(max(prevX, x))
    while (c0 <= c1) {
      if (c0 >= 0 && c0 < barW) colHeat[c0] += heat
      c0++
    }
  }
  liveSparks = live
}

export function render3D(index, x, y, z) {
  if (!mapped) {
    // First pass: learn this pixel's column. Rounding to the nearest of the 160
    // known positions rather than binning -- the mapper places pixels at exactly
    // col/159, and floor(x * 160) would drop the last column into a 161st bin.
    colOf[index] = round(x * (barW - 1))
    hsv(0, 0, 0)
  } else {
    v = colHeat[colOf[index]]
    v = v * v          // stock gamma
    // Stock colouring: hue 0.63 so sparks cool to blue, and hot ones wash out
    // to white as v approaches 1.
    hsv(.63, 1 - v, v)
  }
}

export function render2D(index, x, y) {
  render3D(index, x, 0, y)
}

export function render(index) {
  // No map: fall back to 1D over the raw index.
  v = colHeat[floor(index / pixelCount * barW)]
  v = v * v
  hsv(.63, 1 - v, v)
}
