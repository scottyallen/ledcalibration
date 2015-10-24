
positions = []
for light in lights:
  light.on()
  x, y = getLightPosition()
  positions.append( (x, y, light) )
  light.off()

for x, y, light in sorted(positions, key=lambda x: x[0]):
  light.on()
  time.sleep(100)
  light.off()
