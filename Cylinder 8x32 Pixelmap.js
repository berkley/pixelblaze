// Pixelmap for an 8x32 LED matrix wrapped around a cylinder
//   - 32 pixels around the circumference (columns)
//   - 8 pixels tall (rows)
//   - Serpentine wiring: even rows left→right, odd rows right→left

function (pixelCount) {
  var map = []
  var cols = 32
  var rows = 8

  for (var i = 0; i < pixelCount; i++) {
    var row = floor(i / cols)
    var col = (row % 2 == 0) ? (i % cols) : (cols - 1 - i % cols)

    var angle = col / cols * PI2
    var x = cos(angle)
    var y = sin(angle)
    var z = row / (rows - 1)   // 0 at bottom, 1 at top

    map.push([x, y, z])
  }
  return map
}
