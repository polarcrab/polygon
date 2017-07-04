import json
import sys
sys.path.append('./')

import matplotlib.pyplot as plt
from libcavehull import concave_hull, convex_hull

points = json.load(open('example/points.json'))
points = [tuple(each) for each in points]
plt.scatter([_[0] for _ in points], [_[1] for _ in points], marker='.')


concave = concave_hull(points, 2)
convex = convex_hull(points)

for each in concave:
    plt.plot([each[0][0], each[1][0]], [each[0][1], each[1][1]], color='C2')
for each in convex:
    plt.plot([each[0][0], each[1][0]], [each[0][1], each[1][1]], color='C0')

plt.show()
