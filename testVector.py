import numpy as np
import matplotlib.pyplot as plt
import math
ax = plt.axes()
v = np.array([2.1,4.5])
vu = v / np.linalg.norm(v)

def angle(vector_1, vector_2):
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    return angle

print("Vector original: ", v)
print("Vector unitario: ", vu)
print("Vector perpendicular", np.array([-vu[1],vu[0]]))

ax.arrow(0,0,vu[0],vu[1],head_width=0.05, head_length=0.1, fc='k', ec='k')
ax.arrow(0,0,-vu[1],vu[0],head_width=0.05, head_length=0.1, fc='k', ec='k')

plt.ylim(-3,3)
plt.xlim(-3,3)
print(angle(vu,[-vu[1],vu[0]]))
plt.show()