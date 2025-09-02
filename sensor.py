import math
import numpy as np

neutral_quat = None

def quaternion_inverse(x, y, z, w):
    # The inverse of a unit quaternion is its conjugate
    return -x, -y, -z, w

def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return x, y, z, w

def quaternion_to_euler(x, y, z, w):
    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    # Convert to degrees
    return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z)

def onValueChange(channel, sampleIndex, val, prev):
    global neutral_quat
    input_chop = channel.owner
    
    # Get quaternion values from input CHOP
    x = input_chop['quat1'].eval()
    y = input_chop['quat2'].eval()
    z = input_chop['quat3'].eval()
    w = input_chop['quat4'].eval()
    button = input_chop['button'].eval()
    
    current_quat = (x, y, z, w)
    
    # current_quat = (z, x, y, w)
    
    if neutral_quat is None:
        neutral_quat = current_quat
        print("Neutral position saved. The cube will not move until the sensor is rotated.")
        return
    
    if button == 1:
        # Reset cube rotation to start from zero
        geo_comp = op('geo2')
        geo_comp.par.rx = 0
        geo_comp.par.ry = 0
        geo_comp.par.rz = 0
        
        null_chop = op('constantRotation1')
        null_chop.par.value0 = 0
        null_chop.par.value1 = 0
        null_chop.par.value2 = 0
        
        # Also update neutral_quat to reset reference point
        neutral_quat = None
        print("Rotation reset to zero")
        return

    # Calculate differential rotation (relative quaternion)
    inv_neutral = quaternion_inverse(*neutral_quat)
    rel_quat = quaternion_multiply(current_quat, inv_neutral)
    
    # Convert RELATIVE quaternion to Euler angles
    rx, ry, rz = quaternion_to_euler(*rel_quat)
    
    # Assign rotation to Geometry COMP
    # No gimbal lock but movements don’t correspond visually
    geo_comp = op('geo2')
    geo_comp.par.rx = rx
    geo_comp.par.ry = ry
    geo_comp.par.rz = rz
    
    # Movements correspond, but gimbal lock happens
    # and when rotating around ry, other axes start flipping
    # geo_comp = op('geo2')
    # geo_comp.par.ry = -rx
    # geo_comp.par.rx = -rz
    # geo_comp.par.rz = -ry

    # Assign rotation to Null CHOP
    null_chop = op('constantRotation1')
    null_chop.par.value0 = rx
    null_chop.par.value1 = ry
    null_chop.par.value2 = rz
    
    return