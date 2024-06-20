import numpy as np

def cal_1307e ( new_kp, new_kd):
    gw = 21 * 7 /360
    old_kd = new_kd /(gw * 0.255 * 7 * 180/np.pi )
    old_kp = new_kp /(old_kd * 7 * 0.255 * 7  * 180/np.pi)
    return old_kp, old_kd


def cal_13014e ( new_kp, new_kd):
    gw = 21 * 14 /360
    old_kd = new_kd /(gw * 0.262 * 7 * 180/np.pi )
    old_kp = new_kp /(old_kd * 14 * 0.262 * 14  * 180/np.pi)
    return old_kp, old_kd

def cal_601750 ( new_kp, new_kd):
    gw = 10 * 51 /360
    old_kd = new_kd /(gw * 0.1 * 51 * 180/np.pi )
    old_kp = new_kp /(old_kd * 51 * 0.1 * 51  * 180/np.pi)
    return old_kp, old_kd

def cal_802030 ( new_kp, new_kd):
    gw = 21 * 30 / 360
    old_kd = new_kd /(gw * 0.11 * 30 * 180/np.pi )
    old_kp = new_kp /(old_kd * 30 * 0.11 * 30  * 180/np.pi)
    return old_kp, old_kd

def cal_802029 ( new_kp, new_kd):
    gw = 21 * 28.79 / 360
    old_kd = new_kd /(gw * 0.07 * 28.79 * 180/np.pi )
    old_kp = new_kp /(old_kd * 28.79 * 0.07 * 28.79  * 180/np.pi)
    return old_kp, old_kd

def cal_361480 ( new_kp, new_kd):
    gw = 10 * 80 / 360
    old_kd = new_kd /(gw * 0.067 * 80 * 180/np.pi )
    old_kp = new_kp /(old_kd * 80 * 0.067 * 80  * 180/np.pi)
    return old_kp, old_kd

def cal_3611100 ( new_kp, new_kd):
    gw = 10 * 100 / 360
    old_kd = new_kd /(gw * 0.05 * 100 * 180/np.pi )
    old_kp = new_kp /(old_kd * 100 * 0.05 * 100  * 180/np.pi)
    return old_kp, old_kd


### use ankle pitch sim pd is ok

def cal_36b36e ( new_kp, new_kd):
    
    gw = 10 * 36 / 360
    old_kd = new_kd /(gw * 0.0688 * 36 * 180/np.pi )
    old_kp = new_kp /(old_kd * 36 * 0.0688 * 36  * 180/np.pi)
    return old_kp, old_kd

# print(cal_802030(251.625,14.72))
print(cal_601750(600,50))
# print(cal_1307e(200,11))
# print(cal_36b36e(10.98,0.6))
# print(cal_361480(92.85,2.575))
# print(cal_3611100(112.06,3.1))



# print(cal_802029(251.625,14.72))


# 0 'l_hip_roll',             251.625      14.72        802030        0.9972   0.0445    192.168.137.70
# 1 'l_hip_yaw',              362.52       10.0833      601750        1.0229   0.0300    192.168.137.71
# 2 'l_hip_pitch',            200          11           1307e         1.0606   0.2634    192.168.137.72
# 3 'l_knee_pitch',           200          11           1307e         1.0606   0.2634    192.168.137.73
# 4 'l_ankle_pitch',          10.98        0.6          36b36e        0.5083   0.0042    192.168.137.74 
# 5 'l_ankle_roll',           0.0          0.1          36b36e        0.5083   0.0042    192.168.137.75
# 6 'r_hip_roll',             251.625      14.72        802030        0.9972   0.0445    192.168.137.50
# 7 'r_hip_yaw',              362.52       10.0833      601750        1.0229   0.0300    192.168.137.51
# 8 'r_hip_pitch',            200          11           1307e         1.0606   0.2634    192.168.137.52
# 9 'r_knee_pitch',           200          11           1307e         1.0606   0.2634    192.168.137.53
# 10 'r_ankle_pitch',         10.98        0.6          36b36e        0.5083   0.0042    192.168.137.54
# 11 'r_ankle_roll',          0.0          0.1          36b36e        0.5083   0.0042    192.168.137.55
# 12 'waist_yaw',             362.52       10.0833      601750        1.0229   0.0300    192.168.137.90
# 13 'waist_pitch',           362.52       10.0833      601750        1.0229   0.0300    192.168.137.91
# 14 'waist_roll',            362.52       10.0833      601750        1.0229   0.0300    192.168.137.92
# 15 'l_shoulder_pitch',      92.85        2.575        361480        1.0016   0.0038    192.168.137.10
# 16 'l_shoulder_roll',       92.85        2.575        361480        1.0016   0.0038    192.168.137.11
# 17 'l_shoulder_yaw',        112.06       3.1          3611100       1.0041   0.0039    192.168.137.12
# 18 'l_elbow_pitch',         112.06       3.1          3611100       1.0041   0.0039    192.168.137.13
# 19 'r_shoulder_pitch',      92.85        2.575        361480        1.0016   0.0038    192.168.137.30
# 20 'r_shoulder_roll',       92.85        2.575        361480        1.0016   0.0038    192.168.137.31
# 21 'r_shoulder_yaw',        112.06       3.1          3611100       1.0041   0.0039    192.168.137.32
# 22 'r_elbow_pitch'          112.06       3.1          3611100       1.0041   0.0039    192.168.137.33