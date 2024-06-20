# Author: Ren Xiaoyu
# Date: 2023-02-08
# function: 1. read from absAngle.json, check if the abs number is right, if the abs ip is right
#           2. write the abs value into motorlist.json
# using to calculate motor zero position
import json
# for LITE
# abs_IP_list = ['192.168.137.150','192.168.137.151','192.168.137.152','192.168.137.153','192.168.137.154','192.168.137.155',
#                '192.168.137.170','192.168.137.171','192.168.137.172','192.168.137.173','192.168.137.174','192.168.137.175']
# motor_IP_list = ['192.168.137.50','192.168.137.51','192.168.137.52','192.168.137.53','192.168.137.54','192.168.137.55',
#                 '192.168.137.70','192.168.137.71','192.168.137.72','192.168.137.73','192.168.137.74','192.168.137.75']
# #for WHOLE
abs_IP_list = ['192.168.137.150', '192.168.137.151', '192.168.137.152', '192.168.137.153', '192.168.137.154', '192.168.137.155',
               '192.168.137.170', '192.168.137.171', '192.168.137.172', '192.168.137.173', '192.168.137.174', '192.168.137.175',
                  '192.168.137.190',
               '192.168.137.191', '192.168.137.192']
motor_IP_list = ['192.168.137.50', '192.168.137.51', '192.168.137.52', '192.168.137.53', '192.168.137.54', '192.168.137.55',
                 '192.168.137.70', '192.168.137.71', '192.168.137.72', '192.168.137.73', '192.168.137.74', '192.168.137.75',
                 '192.168.137.90',
                 '192.168.137.91', '192.168.137.92']


def check_legality():
    legality = True
    # read absAngle.json
    # check if the absAngle is null
    try:
        with open("absAngle.json", 'r', encoding='utf-8') as absAngle:
            absAngle_dict = json.load(absAngle)
        # print(absAngle_dict)
        # print(type(absAngle_dict))
        # check if the number of the absAngle is right
        for ip in abs_IP_list:
            exsit_flag = ip in absAngle_dict
            if exsit_flag == False:
                legality = False
                print("abs IP: ", ip, " does not exsit!")
                return legality
    except json.decoder.JSONDecodeError:
        print("absAngle.json is null!")
        legality = False
        return legality
    return legality


def setMotorZeroPos():
    ready_flag = True
    motorlist_path = "../MotorList/sources/motorlist.json"
    # check abs absAngle legality
    if not check_legality():
        print("set Motor Zero Position failed!")
        ready_flag = False
        return ready_flag
    # load absAngle
    with open("absAngle.json", 'r', encoding='utf-8') as absAngle:
        absAngle_dict = json.load(absAngle)
    # load motor list
    try:
        with open(motorlist_path, 'r', encoding='utf-8') as motorlist:
            motorlist_dict = json.load(motorlist)
    except json.decoder.JSONDecodeError:
        print("motorlist.json is null!")
        ready_flag = False
        return ready_flag
    # set zero pos
    try:
        for ip_num in range(len(abs_IP_list)):
            # print(motor_IP_list[ip_num])
            # print(abs_IP_list[ip_num])
            motorlist_dict[motor_IP_list[ip_num]]["absolute_pos_zero"] = absAngle_dict[abs_IP_list[ip_num]]["radian"]
    except:
        print("Set Motor Zero Failed!")
        ready_flag = False
        return ready_flag
    # write into motorlist
    try:
        with open(motorlist_path, 'w', encoding='utf-8') as motorlist_w:
            json.dump(motorlist_dict, motorlist_w, indent=4, ensure_ascii=False)
    except:
        print("write into motorlist.json Failed!")
        ready_flag = False
        return ready_flag
    print("Set Motor Zero Position Success!")
    return ready_flag


def main():
    legality = check_legality()
    print(legality)
    setMotorZeroPos()


if __name__ == '__main__':
    main()
