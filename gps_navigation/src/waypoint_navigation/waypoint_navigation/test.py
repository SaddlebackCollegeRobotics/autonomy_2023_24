from math import degrees, atan2

current_pos = (0.0, 0.0)
target_pos = (1.0, 1.0)

x = target_pos[0] - current_pos[0]
y = target_pos[1] - current_pos[1]

target_heading = degrees(atan2(y, x))
# print(target_heading)

if target_heading < 0:
    target_heading += 360

print(target_heading)