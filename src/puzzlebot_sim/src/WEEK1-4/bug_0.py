import pdb
from math import cos, sin
import numpy as np

def unit_vector(vector):
  """ Returns the unit vector of the vector.  """
  return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
  """ Returns the angle in radians between vectors 'v1' and 'v2'::
          >>> angle_between((1, 0, 0), (0, 1, 0))
          1.5707963267948966
          >>> angle_between((1, 0, 0), (1, 0, 0))
          0.0
          >>> angle_between((1, 0, 0), (-1, 0, 0))
          3.141592653589793
  """
  v1_u = unit_vector(v1)
  v2_u = unit_vector(v2)
  return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def rotate_vec(vec, deg):
  theta = np.deg2rad(deg)
  rotation = np.array([
    [cos(theta), -sin(theta)],
    [sin(theta), cos(theta)]
  ])
  return np.dot(rotation, vec)


def linear_eq(point_a, point_b):
  y, x = point_a[1], point_a[0]
  dx = point_b[0] - x
  if(dx == 0):
    return (float('inf'), x) 
  m = (point_b[1] - y) / dx 
  b = y - m*x
  return (m, b)


class BugZero:
  def __init__(self, radius):
    self.radius = radius
    self.colliding = False
    self.circumnavigating = False
    self.goal = None
  

  def _goal_on_sight(self, position, boundaries):
    v = self.radius*unit_vector(self.goal - position)
    v_left, v_right = rotate_vec(v, 90), rotate_vec(v, -90)
    m_l, b_l = linear_eq(position + v_left, self.goal + v_left)
    m_r, b_r = linear_eq(position + v_right, self.goal + v_right)
    for boundary in boundaries:
      global_boundary = position + boundary
      x, y = global_boundary[0], global_boundary[1]
      if(m_l == float('inf') or m_r == float('inf')):
        if(x > b_l and x < b_r):
          return False
      below_left = (m_l*x + b_l) > y
      above_right = (m_r*x + b_r) < y
      # pdb.set_trace()
      if below_left and above_right:
        return False
    return True


  def _closest_boundary(self, boundaries):
    distances = np.linalg.norm(boundaries, axis = 1)
    idx = list(distances).index(min(distances))
    return idx, boundaries[idx]


  def _is_colliding(self, boundaries):
    distances = np.linalg.norm(boundaries, axis = 1)
    return min(distances) <= self.radius


  def _get_boundary_step(self, position, boundaries):
    idx, closest = self._closest_boundary(boundaries)
    left_closest = boundaries[idx - 1]
    # pdb.set_trace()
    dist_to_closest = np.linalg.norm(closest)
    D = left_closest - closest
    R = closest
    RD_angle = angle_between(R, D)
    if(idx == 0):
      # pdb.set_trace()
      direction = self.radius*unit_vector(closest)
      next_step = rotate_vec(direction, 90)
      return next_step
    if dist_to_closest > self.radius:
      return D
    if RD_angle == np.pi/2:
      return D
    no_intersect = self.radius*cos(RD_angle)
    return np.array([no_intersect, 0])


  def _get_goal_step(self, position):
    return unit_vector(self.goal - position)


  def set_goal(self, position):
    self.goal = position


  def next_step(self, position, boundaries):
    if self.goal is None:
      raise Exception(
        'goal is not set in bug object. '
        + 'Please set it with set_goal')
    if self._is_colliding(boundaries) or self.circumnavigating:
      if self._goal_on_sight(position, boundaries):
        self.circumnavigating = False
      else:
        self.circumnavigating = True
        return (self._get_boundary_step(position, boundaries),
          self.circumnavigating)
    return (self._get_goal_step(position), self.circumnavigating)
