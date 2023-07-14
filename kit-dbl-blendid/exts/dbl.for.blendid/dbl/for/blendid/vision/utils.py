import numpy as np
import cv2
from pxr import Gf

BOX_SIZE = [0.071, 0.0965, 0.1198] # in cm

def find_bottom_point(points):
    """
    Find the bottom point from a list of points
    """
    bottom_point = points[0]
    bottom_idx = 0
    for c, point in enumerate(points):
        if point[1] > bottom_point[1]:
            bottom_point = point
            bottom_idx = c
    return bottom_point, bottom_idx

def find_left_point(points):
    """
    Find the left point from a list of points
    """
    left_point = points[0]
    for point in points:
        if point[0] < left_point[0]:
            left_point = point
    return left_point

def get_projection(point, direction, z):
    """
    Get projection
    """
    t = (z - point[2]) / direction[2]
    x = point[0] + direction[0] * t
    y = point[1] + direction[1] * t
    return np.array((x, y, z))

def get_box_transform_from_point(camera_position, bottom_direction, left_direction, affordance_z = 0):
    """
    Get box points
    """
    bottom_point = get_projection(camera_position, bottom_direction, affordance_z)
    left_point = get_projection(camera_position, left_direction, affordance_z)

    distance =  np.linalg.norm(bottom_point - left_point)
    
    closest_value = min(BOX_SIZE,key=lambda x:abs(x-distance))
    print("distance: ", distance, bottom_point, left_point, "\n close to: ", closest_value)

    direction = left_point - bottom_point
    direction = direction / np.linalg.norm(direction)
    direction = Gf.Vec3d(direction[0], direction[1], direction[2])
    print("direction: ", direction)
    
    # determine the box rotation
    if closest_value == BOX_SIZE[0]:  
        direction_r = np.array([direction[1], -direction[0], 0])
        right_point = bottom_point + direction_r * BOX_SIZE[1]
        center_point = (left_point + right_point) / 2
        rotation = Gf.Rotation(Gf.Vec3d(0, -1, 0), direction)
    elif closest_value == BOX_SIZE[1]:
        direction_r = np.array([direction[1], -direction[0], 0])
        right_point = bottom_point + direction_r * BOX_SIZE[0]
        center_point = (left_point + right_point) / 2
        rotation = Gf.Rotation(Gf.Vec3d(-1, 0, 0), direction)
    else:
        
        center_point = (left_point + bottom_point) / 2
        from_direction = Gf.Vec3d([BOX_SIZE[1], -BOX_SIZE[1], 0]).GetNormalized()
        rotation = Gf.Rotation(from_direction, direction)

    return center_point, rotation.GetQuat()

def get_box_transform_from_point2(camera_position, bottom_direction, left_direction, right_direction, affordance_z = 0):
    """
    Get box points
    """
    bottom_point = get_projection(camera_position, bottom_direction, affordance_z)
    left_point = get_projection(camera_position, left_direction, affordance_z)
    right_point = get_projection(camera_position, right_direction, affordance_z)

    distance_lb =  np.linalg.norm(bottom_point - left_point)
    distance_rb =  np.linalg.norm(bottom_point - right_point)
    print("distance: ", distance_lb, distance_rb, bottom_point, left_point, right_point)
    
    center_point = (left_point + left_point) / 2
    center_point = Gf.Vec3d(center_point[0], center_point[1], center_point[2])

    direction = left_point - bottom_point
    direction = direction / np.linalg.norm(direction)
    direction = Gf.Vec3d(direction[0], direction[1], direction[2])
    rotation = Gf.Rotation(Gf.Vec3d(-1, 0, 0), direction)

    return center_point, rotation.GetQuat()

def get_affine_mat(source_points, target_points):
    src_points = np.array(source_points)
    dst_points = np.array(target_points)
    # src_homogeneous = np.hstack((src_points, np.ones((3, 1))))
    # dst_homogeneous = np.hstack((dst_points, np.ones((3, 1))))
    # affine_transform = np.linalg.solve(src_homogeneous, dst_homogeneous)
    # Pad the data with ones, so that our transformation can do translations too
    n = src_points.shape[0]
    pad = lambda x: np.hstack([x, np.ones((x.shape[0], 1))])
    unpad = lambda x: x[:,:-1]
    X = pad(src_points)
    Y = pad(dst_points)

    # Solve the least squares problem X * A = Y
    # to find our transformation matrix A
    A, res, rank, s = np.linalg.lstsq(X, Y, rcond=None)
    transform = lambda x: unpad(np.dot(pad(x), A))
    print("transform", transform(src_points), dst_points)

    # ret = np.linalg.lstsq(src_points, dst_points, rcond=None)
    # affine_transform = ret[0]
    # residual = ret[1]
    # # affine_transform = affine_transform
    # s0 = np.array([source_points[0][0], source_points[0][1]]) @ affine_transform + residual
    # s1 = np.array([source_points[1][0], source_points[1][1]]) @ affine_transform + residual
    # s2 = np.array([source_points[2][0], source_points[2][1]]) @ affine_transform + residual
    # print("s0: ", s0, "s1: ", s1, "s2: ", s2)
    # print("t0: ", target_points[0], "t1: ", target_points[1], "t2: ", target_points[2])
     

    return A