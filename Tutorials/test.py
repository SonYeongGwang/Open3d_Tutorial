import numpy as np
import matplotlib.pyplot as plt

bounding_polygon = np.array([
		[ -0.1, -0.035, 0.0 ],
		[ 0.1, -0.035, 0.0 ],
		[ 0.1, 0.155, 0.0 ],
		[ -0.1, 0.155, 0.0 ],
        [ -0.1, -0.035, 0.0 ]
	])

bounding_polygon = bounding_polygon[:, [0, 1]]
print(bounding_polygon)

plt.figure()
plt.plot(bounding_polygon[:,0], bounding_polygon[:,1])
plt.show()