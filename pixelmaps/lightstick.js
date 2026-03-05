function (pixelCount) {
  var map = [];
  for (i = 0; i < pixelCount; i++) {
    c = i / pixelCount * Math.PI * 2
    map.push([Math.cos(c), Math.sin(c)])
  }
  return map
}
