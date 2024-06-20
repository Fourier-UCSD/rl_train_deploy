#!/usr/bin/env Python
# -*- coding:UTF-8 -*-
# author：作者    date:2020/1/8 time:21:45
# 代码自动补齐----------------------------
# p: parameter 参数
# m: method 方法
# f: function 函数
# v: variable 变量
# c: class 类
# 快捷键---------------------------------
# 复制上一行：crtl + D# 删除这一行：crtl + Y
# 增加/删除注释：Ctrl + /
# 折叠代码：crtl + -       全部：crtl + shift + -
# 展开代码：crtl + +       全部：Ctrl + shift + +
# 回车换行：shift + Enter# 查找：Ctrl + F
# 替换：Ctrl + R# 自动排版：Ctrl + Alt + L
# 缩进：Tab# 反缩进：Shift + Tab# 找寻变量\函数\参数定义的位置：Ctrl + 鼠标单击
# 逐步选定相邻的代码：Ctrl + W
# 同时选定多行并且编辑：Alt + 鼠标左击，退出：Esc
# 变成指定代码块：Ctrl + Alt + T

import time
import socket
import json

udp_sorket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sorket.settimeout(1)
udp_sorket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 20)


network = '192.168.137.255'
# network = '255.255.255.255'

remote_port = 2334

ctrl_port = 2333



DeviceModel = 'ABS_Encoder'



Server_IP_list = ["192.168.31.224"]
abs_angle_dict = {}
json_dirt = {}

# 广播获取设备信息，ip地址等
def brosdcast_func():
    data = {
        "id": 0,
        "method": "Device.Info",
        "params": "",
    }

    json_str = json.dumps(data)
    #print(json_str)
    found_abs = False
    address_list = []

    for i in range(3): # Wi-Fi 连接时，广播一次不一定能全部接收到所有设备的回复 ，需要多广播几次
        udp_sorket.sendto(json_str.encode('utf-8'), (network, remote_port)) # udp 广播
        #print('send broadcast')
        while True:
            try:
                r_data, addr = udp_sorket.recvfrom(1024) #接收局域网中回复数据，每次接收最大1024
                #print('udp received from {}:{}'.format(addr, r_data.decode('utf-8')))
                device_json_obj = json.loads(r_data.decode('utf-8'))
                if "result" in device_json_obj:
                    if address_list.count(addr[0]) == 0 : #如果列表中没有此值，则添加上
                        if device_json_obj['result']["dev_model"] == DeviceModel:  # 查看设备回复的设备类型是否是abs
                            address_list.append(addr[0])  # 加入到列表中
                            json_dirt[addr[0]] = device_json_obj['result']['serial_number']
                found_abs = True
                #break
            except socket.timeout:
                if found_abs:
                    break
                else:
                    #print('do not receive any abs')
                    break
    return address_list

# 获取每个设备的 abs角度
# addrList  设备ip地址列表
# angle_dict abs 角度字典 ： {ip : angle}
def get_angle(addrList,angle_dict):
    data = {
        "id": 1,
        "method": "Encoder.Angle",
        "params": "",
    }
    json_str = json.dumps(data)
    for i in addrList: 
        tcp_sorket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_sorket_client.connect((i,remote_port))
        tcp_sorket_client.send(json_str.encode('utf-8'))
        r_data = tcp_sorket_client.recv(1024)
        
        json_obj = json.loads(r_data.decode('utf-8'))  # 解析json数据包
        if json_obj['id'] == 1:
            #d_abs = [json_obj['result']['angle'],json_obj['result']['radian'] ] # 获取abs角度值 ，弧度值
            #d_abs = {"ip":i,"angle":json_obj['result']['angle'],"radian":json_obj['result']['radian'] }
            d_abs = {"angle":json_obj['result']['angle'],"radian":json_obj['result']['radian'] }
            # print(d_abs)
            angle_dict[i] = d_abs  # 将abs值放入到 键为IP地址的 字典中
            # print(abs_angle_dict)
        tcp_sorket_client.close();        

# 广播获取设备信息，ip地址等
def brosdcast_func_new():
    data = {
        "id": 0,
        "method": "Device.Info",
        "params": "",
    }

    json_str = json.dumps(data)
    #print(json_str)
    found_abs = False
    address_list = []

    for i in range(5): # Wi-Fi 连接时，广播一次不一定能全部接收到所有设备的回复 ，需要多广播几次
        udp_sorket.sendto(json_str.encode('utf-8'), (network, remote_port)) # udp 广播
        #print('send broadcast')
        while True:
            try:
                r_data, addr = udp_sorket.recvfrom(1024) #接收局域网中回复数据，每次接收最大1024
                # print('udp received from {}:{}'.format(addr, r_data.decode('utf-8')))
                
                if addr[0] in address_list:
                    pass
                else:                
                    json_obj = json.loads(r_data.decode('utf-8'))
                    if json_obj.get("type") == "AbsEncoder":
                    	address_list.append(addr[0])

                found_abs = True
                #break
            except socket.timeout:
                if found_abs:
                    break
                else:
                    #print('do not receive any abs')
                    break

    return address_list

def get_angle_new(addrList, angle_dict):
    data = {
        "method": "GET",
        "reqTarget": "/measured",
        "property": "/"
    }

    json_str = json.dumps(data)
    
    for target_address in addrList: 
        # print("target_address = ", target_address)
    
        udp_sorket.sendto(json_str.encode('utf-8'), (target_address, ctrl_port))
        r_data, addr = udp_sorket.recvfrom(1024)
        # print('udp received from {}:{}'.format(addr, r_data.decode('utf-8'))) 
        # print(json_obj)
        # d_abs = {"ip": addr[0], "angle":json_obj['angle'],"radian":json_obj['radian'] }
        json_obj = json.loads(r_data.decode("utf-8"))
        d_abs = {"angle":json_obj.get("angle"),"radian":json_obj.get("radian")}
  
        print(d_abs)
        angle_dict[target_address] = d_abs  # 将abs值放入到 键为IP地址的 字典中
        
        time.sleep(0.01)

    # print(abs_angle_dict)    


def main():
    abs_file = open("absAngle.json", mode="w+", encoding="utf-8")

    Server_IP_list = brosdcast_func_new()# 广播查询局域网下的所有abs
    print(Server_IP_list)

    if len(Server_IP_list) == 0:
        print('there is no abs')
        return
    
    get_angle_new(Server_IP_list, abs_angle_dict)

    json_str = json.dumps(abs_angle_dict, indent=4)
    abs_file.write(json_str)
    time.sleep(3)
    print("get abs angle complete!") 
      





if __name__ == '__main__':
    main()
