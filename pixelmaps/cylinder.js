//pixel map for the CG Mini Beacon.  8x32 array wrapped around a 6" cylinder.

function (pixelCount) {
  width = 8
  var rows = pixelCount / width  // 32
  var map = []
  for (i = 0; i < pixelCount; i++) {
    row = Math.floor(i / width)
    col = i % width
    col = row % 2 == 1 ? width - 1 - col : col  // zigzag

    var angle = row / rows * Math.PI * 2
    var x = Math.cos(angle)
    var y = Math.sin(angle)
    var z = col / (width - 1)  // 0 at bottom, 1 at top

    map.push([x, y, z])
  }
  return map
}
