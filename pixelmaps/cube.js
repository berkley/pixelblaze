function (pixelCount) {
  // Set to true to flip the map vertically (up ↔ down).
  var flipVertical = false

  width = 16
  function side(targets, offsets) {
    var matrix = [], coords, row, col
    for (i = 0; i < width * width; i++) {
      row = Math.floor(i / width)
      col = i % width
      col = row % 2 == 1 ? width - 1 - col : col  // zigzag: serpentine columns
      coords = [0, row, col]
      matrix.push(targets.map(function (target, index) {
        var coord = coords[Math.abs(target)]
        if (target < 0) coord = width - 1 - coord
        return coord + offsets[index]
      }))
    }
    return matrix
  }
  var map = []
  var gap = .75
  map = map.concat(side([2, 1, 0], [0, 0, width - 1 + gap]))
  map = map.concat(side([0, 1, -2], [-gap, 0, 0]))
  map = map.concat(side([1, 0, -2], [0, width - 1 + gap, 0]))
  map = map.concat(side([0, -1, -2], [width - 1 + gap, 0, 0]))
  map = map.concat(side([-1, 0, -2], [0, -gap, 0]))

  if (flipVertical) {
    // z spans 0 .. (width - 1 + gap). Reflect each pixel's z about the midpoint.
    var maxZ = width - 1 + gap
    for (var i = 0; i < map.length; i++) {
      map[i][2] = maxZ - map[i][2]
    }
  }

  return map
}
