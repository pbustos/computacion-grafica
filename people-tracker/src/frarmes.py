import numpy as np
import matplotlib.pyplot as plt
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager

object2cam = pt.transform_from(
    pr.active_matrix_from_intrinsic_euler_xyz(np.array([1.25, -0.028, -0.06])),
    np.array([381, 761, -3173]))

object2cam2 = pt.transform_from(
    pr.active_matrix_from_intrinsic_euler_xyz(np.array([-2.04, -3.13, 0.09])),
    np.array([-192, 254, -3408]))


tm = TransformManager()
tm.add_transform("object", "camera", object2cam)
tm.add_transform("object","camera2",object2cam2)

p = pt.transform(tm.get_transform("camera", "object"), np.array([0,0,0,1]))

ax = tm.plot_frames_in("object", s=500)
ax.scatter(p[0],p[1],p[2])
ax.set_xlim((-1000, 3000))
ax.set_ylim((-3000, 3000))
ax.set_zlim((-2000.0, 4000))
plt.show()