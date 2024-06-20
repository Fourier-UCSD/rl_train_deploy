from math import floor
import torch

"""@class a set of the functions for clock gait phases changing between swing and stance
"""


def trapezoidclockgait(num_env, sim_deviece, phi, r, deltT):
    I_frc = torch.zeros(num_env, dtype=torch.float, device=sim_deviece, requires_grad=False)
    swing_flag = (phi >= (deltT)) & (phi <= (r - deltT))
    stand_flag = (phi >= (r + deltT)) & (phi <= (1 - deltT))
    # trans_flag = ~swing_flag & ~stand_flag
    trans_flag1 = (phi < (deltT))
    trans_flag2 = (phi > (r - deltT)) & (phi < (r + deltT))
    trans_flag3 = (phi > (1 - deltT))
    I_frc = 1.0 * swing_flag + (0.5 + phi / (2 * deltT)) * trans_flag1 - (phi - r - deltT) / (2.0 * deltT) * trans_flag2 + 0.0 * stand_flag + (phi - 1 + deltT) / (2 * deltT) * trans_flag3
    # print(phi)
    # print("I_frc")
    # print(I_frc)
    I_spd = 1.0 - I_frc
    return I_frc, I_spd


def trapezoidclockgait_v1(num_env, sim_deviece, phi, r, deltT):
    I_frc = torch.zeros(num_env, dtype=torch.float, device=sim_deviece, requires_grad=False)
    I_spd = torch.zeros(num_env, dtype=torch.float, device=sim_deviece, requires_grad=False)
    swing_ini_flag = (phi < (2 * deltT))
    swing_med_flag = (phi >= (2 * deltT)) & (phi < (r - 2 * deltT))
    swing_end_flag = (phi >= (r - 2 * deltT)) & (phi < r)
    stand_ini_flag = (phi >= r) & (phi < (r + 2 * deltT))
    stand_med_flag = (phi >= (r + 2 * deltT)) & (phi < (1 - 2 * deltT))
    stand_end_flag = (phi >= (1 - 2 * deltT))

    I_frc = 1.0 * (swing_ini_flag + swing_med_flag + swing_end_flag) - (phi - r - 2.0 * deltT) / (2.0 * deltT) * stand_ini_flag + 0.0 * stand_med_flag + (phi - 1 + 2 * deltT) / (2 * deltT) * stand_end_flag
    I_spd = (2 * deltT - phi) / (2.0 * deltT) * swing_ini_flag + 0.0 * swing_med_flag + (phi - r + 2 * deltT) / (2 * deltT) * swing_end_flag + 1.0 * (stand_ini_flag + stand_med_flag + stand_end_flag)
    # print(phi)
    # print("I_spd")
    # print(I_spd)

    return I_frc, I_spd
