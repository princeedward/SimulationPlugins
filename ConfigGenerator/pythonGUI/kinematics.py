from numpy import matrix, deg2rad, rad2deg, cos, sin, hstack, vstack, eye, pi, arcsin, arccos
c = cos
s = sin


def rotz(theta):
	''' Returns a rotation matrix about z-axis by angle theta, in radians. '''
	return matrix([[c(theta), -s(theta), 0],
			 	[s(theta),  c(theta), 0],
			 	[		 0, 	   0, 1],
			   ])

def rotx(theta):
	''' Returns a rotation matrix about x-axis by angle theta, in radians. '''
	return matrix([[1, 		  0, 		 0],
				   [0, c(theta), -s(theta)],
				   [0, s(theta),  c(theta)],
				  ])

def roty(theta):
	''' Returns a rotatino matrix about y-axis by angle theta, in radians. '''
	return matrix([[c(theta),	0,	-s(theta)],
				   [	   0,	1,			0],
				   [s(theta),	0,	 c(theta)]
				  ])

def se3(r, t):
	''' Returns a 4x4 se3 matrix using r as rotation and t as translation. '''
	return vstack( [hstack([r, t]), matrix([0, 0, 0, 1])] )

def rotZYX2rpy(R):
	''' Returns a touple (r,p,y) which are the tait-bryan angles for R, which
	is a ZYX rotation matrix. '''
	t2 = arcsin(-R[2,0])
	t1 = arccos(R[0,0]/cos(t2))
	t3 = arccos(R[2,2]/cos(t2))
	return (t1, t2, t3)

#########
L = 1

def get_new_position(parent_module, new_module_angles, parent_face, new_face):
        """ Returns the position and orientation of the new module as (x,y,z,r,p,y) """
        new_wrt_old = kinematics.get_xform(parent_module.JointAngle, new_module_angles,
        								   parent_face, new_face)
        p_pos = matrix(parent_module.position[0:3]).T
        p_rpy = parent_module.position[4:6]
        p_rot = rotz(p_rpy[3])*roty(p_rpy[2])*rotx(p_rpy[1])
        new_wrt_world = se3(p_rot, p_pos)*new_wrt_old
        n_pos = tuple(new_wrt_world[0:3,3])
        n_rpy = rotZYX2rpy( new_wrt_world[0:3,0:3] )
        return n_pos + n_rpy

def get_xform(angles1, angles2, face1, face2):
	T1 = get_xform1(angles1, face1)
	T2 = get_xform2(angles2, face2)
	return T1*T2

def get_xform1(a, face1):
	if face1 == 1:
		t0 = matrix([0, 0, 0]).T
		r0 = rotx( a[3] )
		t1 = matrix([L, 0, 0]).T
		r1 = rotx( a[1] )
		t2 = matrix([0, 0, 0]).T
		r2 = matrix(eye(3))
		return se3(r0,t0)*se3(r1,t1)*se3(r2,t2)

def get_xform2(a, face2):
	if face2 == 2:
		t0 = matrix([0, 0, 0]).T
		r0 = rotz( pi )
		t1 = matrix([0, 0, 0]).T
		r1 = rotx( a[2] ) # joint
		t2 = matrix([0, 0, 0]).T
		r2 = rotz( -pi )
		t3 = matrix([L, 0, 0]).T
		r3 = matrix(eye(3))
		return se3(r0,t0)*se3(r1,t1)*se3(r2,t2)*se3(r3,t3)

if __name__ == "__main__":
	angles1 = [0, 0, 0, pi/4]
	angles2 = [0, 0, 0, 0]
	face1 = 1
	face2 = 2
	print get_xform(angles1, angles2, face1, face2)