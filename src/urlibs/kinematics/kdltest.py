import numpy as np
import PyKDL as kdl
# When taking the rotation vector directly from KDL, there are
# numeric problems when the rotation matrix is diagonal, for example
# 180 degree rotation around any or the axes. In this case the vector
# is set to zero even if it should be for example (0, 0, pi) for a 
# 180 degree rotation around z


# frames.cpp line 325 GetRot


rot = kdl.Rotation()
rot.DoRotZ(np.pi)
print rot.GetRot()

rot = kdl.Rotation()
rot.DoRotZ(np.pi + 0.001)
print rot.GetRot()


rot3 = kdl.Rotation()
rot3.DoRotX(np.pi)
print rot3.GetRotAngle()

rot2 = kdl.Rotation()
rot2.DoRotY(np.pi)
print rot2.GetRotAngle()

rot4 = kdl.Rotation()
rot4.DoRotZ(np.pi)
print rot4.GetRotAngle()
