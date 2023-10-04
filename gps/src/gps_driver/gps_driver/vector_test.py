import math


pos1 = [1, 0]
pos0 = [0, 0]

vector = [pos1[0] - pos0[0], pos1[1] - pos0[1]]
magnitude = math.sqrt(vector[0] ** 2 + vector[1] ** 2)

unit_vector = [vector[0] / magnitude, vector[1] / magnitude]

angle = math.degrees(math.atan2(unit_vector[1], unit_vector[0]))

# convert to 0 to 360
if angle < 0:
    angle += 360

print("Angle: ", angle)
