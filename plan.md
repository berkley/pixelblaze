# Plan: Motion - Centrifugal Slide

## Concept
A pattern where spinning the ring causes rainbow colors to "slide" outward from the center pivot (pixel 29) toward both ends (pixel 0 and pixel 58), like centrifugal force pushing paint outward.

## How the accelerometer detects spin
- The ring is mapped with `cos/sin` in 2D, so spinning it around its center axis changes the accelerometer readings on axes 0 and 1 (X and Y).
- We compute **angular velocity** from the cross-product of smoothed vs. raw acceleration vectors (same approach as detecting rotation from a gyro-like signal), OR more simply: track the rate of change of the acceleration angle using `atan2(accel[1], accel[0])`.
- The magnitude of the angular velocity = spin speed.

## Algorithm

### Spin Detection
1. Each frame, compute the angle of the accelerometer vector: `angle = atan2(accelerometer[1], accelerometer[0])`
2. Compute `deltaAngle` (change since last frame), handling wraparound.
3. Smooth `deltaAngle` to get `spinSpeed` — a signed value representing how fast the ring is spinning.
4. Compute `spinForce = abs(spinSpeed)` — this drives the outward push.

### Pixel Sliding (Centrifugal Effect)
- Maintain an array of 59 hue values, one per pixel.
- Each frame, "push" hues outward from center (index 29) toward both ends, proportional to `spinForce * speedMultiplier`.
- New hues are injected at the center (pixel 29) using a cycling rainbow.
- The push is implemented as fractional shifting: each pixel blends toward its outward neighbor by `spinForce * dt * speed`.
- Faster spin = faster outward flow. When spin stops, the pattern freezes in place.

### Color
- Rainbow hue cycles continuously at the center injection point (using `time()`).
- Full saturation by default, controllable via slider.
- Brightness uses a gentle falloff toward the ends to add depth, or stays full — kept simple.

### Sliders
1. **Saturation** (`sliderSaturation`): Controls color saturation (0 = white, 1 = full color). Default: 1.0
2. **Speed** (`sliderSpeed`): Multiplier on how fast pixels slide outward for a given spin rate. Default: mid-range.

## File
- Create: `Motion - Centrifugal Slide.js`

## Implementation Details

### Data structures
- `hues[59]` — array storing the hue at each pixel position (0..1)
- `vals[59]` — array storing brightness at each pixel (0..1)

### beforeRender(delta)
1. Compute current accel angle via `atan2`.
2. Compute `deltaAngle`, unwrap, smooth it.
3. Derive `spinForce`.
4. Shift hues outward from center:
   - For left half (indices 28→0): each pixel pulls from its right neighbor (toward center).
   - For right half (indices 30→58): each pixel pulls from its left neighbor (toward center).
   - Blend amount = `clamp(spinForce * speedMul * dt, 0, 1)`.
5. Inject new rainbow hue at index 29.
6. Fade brightness slightly at the ends to give a trailing-off look.

### render(index)
- Simply output `hsv(hues[index], saturation, vals[index])`.

## Notes
- Uses `render(index)` (1D), not `render2D`, since the sliding logic is index-based along the strip.
- The pixel map still applies for the Pixelblaze's spatial preview, but the pattern logic treats it as a linear array with center at 29.
- Pixelblaze language: no `let`/`const`/`var` in loops, uses `export function`, `array(n)` to allocate arrays, and built-in functions like `time()`, `wave()`, `triangle()`, `mod()`, `clamp()`, `mix()`, `atan2()`, `abs()`, `hsv()`.
