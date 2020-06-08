######################################################
#                                                    #
#             微软峙雄                               #
#                                                    #
######################################################

#! /usr/bin/env python
# coding=utf-8
import os
# import math
import socket
import robotcontrol
import threading
import inspect
import ctypes
# import Queue
# 机械臂末端最大加速度
EndMaxAcc = 0.1
#机械臂末端最大线速度
EndMacVelc = 0.1
#机械臂碰撞等级
CollisionLevel = 7
#机械臂初始位置，工作完成后要回到初始位置
init_jiont_radian = (0, 0, 0, 0, 0, 0)
#dianwei shuju
z_pos=10.0
#socket
#robot = None

#t1 = None
#t2 = None
#a = None
def SochetServer():
    global robot
    global bExite
    global data
    global c
    #global qDataQueue
    while 1:
          print("waiting for connection....")
          coin, addr = tcpSerSock.accept()
          print("....connected from:", addr)
          try:
              data= coin.recv(1024)  # 接收1024个字节
              if len(data) == 0:
                  bExite = 1
                  robotcontrol.logger.error("socket.recv empty,service close")
                  coin.close()
                  break
              else:
                  print('客户端的数据', data)
                  #台达软件解析数据点注意机械手默认位置单位是m所以要做好单位转换
                  tr1 =data.decode('gbk')
                  aa = tr1.split('/')
                  robot.remove_all_waypoint()
                  # 初始化全局配置文件
                  robot.init_profile()
                  # 设置关节最大加速度
                  robot.set_joint_maxacc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))
                  # 设置关节最大加速度
                  robot.set_joint_maxvelc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))
                  ret = robot.get_current_waypoint()
                  ret['pos'][0] = -0.214
                  ret['pos'][1] = -0.341
                  ret['pos'][2] = 0.42
                  ik_result = robot.inverse_kin(ret['joint'], ret['pos'], ret['ori'])
                  joint_radian = ik_result['joint']
                  robotcontrol.logger.info("move joint to {0}".format(joint_radian))
                  robot.move_joint(joint_radian)
                  robotcontrol.time.sleep(3)
                  ret1=robot.get_current_waypoint()
                  robotcontrol.logger.info("ret1:{0}".format(ret1))
                  # 初始化全局配置
                  robot.init_profile()
                  # 设置防撞等级
                  robot.set_collision_class(CollisionLevel)
                  # 设置直线末端最大速度
                  robot.set_end_max_line_acc(EndMaxAcc)
                  robot.set_end_max_line_velc(EndMacVelc)
                  robot.set_blend_radius(0.01)
                  for cout in range(3):
                      bb = aa[cout].split(',')
                      x = float(bb[0])
                      y = float(bb[1])
                      print('x', x)
                      print('y', y)
                      ret['pos'][0] = x/1000
                      ret['pos'][1] = y/1000
                      ret['pos'][2] =0.42
                      ik_result=robot.inverse_kin(ret['joint'], ret['pos'], ret['ori'])
                      robotcontrol.logger.info(ik_result)
                      robotcontrol.time.sleep(0.5)
                      joint=ik_result['joint']
                      robot.add_waypoint(joint)
                      robotcontrol.time.sleep(0.5)
                  res=robot.get_current_waypoint()
                  print('currrent waypoint is {0}'.format(res))
                  robot.move_track(robotcontrol.RobotMoveTrackType.CARTESIAN_MOVEP)
                  #robot.remove_all_waypoint()
          except:
              print('error')
              break
          coin.close()
def robot_login():
    global robot
    global bExite
    # 系统初始化
    robotcontrol.Auboi5Robot.initialize()
    # 初始化logger
    robotcontrol.logger_init()
    # 创建机械臂控制类
    robot = robotcontrol.Auboi5Robot()
    # 启动测试
    robotcontrol.logger.info("{0} test beginning...".format(robotcontrol.Auboi5Robot.get_local_time()))
    # 创建上下文
    handle = robot.create_context()
    # 打印上下文
    robotcontrol.logger.info("robot.rshd={0}".format(handle))
    try:
        # 链接服务器
        ip = '192.168.1.113'
        port = 8899
        result = robot.connect(ip, port)
        if result !=robotcontrol.RobotErrorType.RobotError_SUCC:
            robotcontrol.logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            #初始化全局配置
            robot.init_profile()
            #设置防撞等级
            robot.set_collision_class(CollisionLevel)
            #设置直线末端最大速度
            robot.set_end_max_line_acc(EndMaxAcc)
            robot.set_end_max_line_velc(EndMacVelc)
            print('初始化成功')
    except robotcontrol.RobotError as e:
        robotcontrol.logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))
    # finally:
    #     # 断开服务器链接
    #     if robot.connected:
    #         # 关闭机械臂
    #         robot.robot_shutdown()
    #         # 断开机械臂链接
    #         robot.disconnect()
    #     # 释放库资源
    #     robotcontrol.Auboi5Robot.uninitialize()
    #     robotcontrol.logger.info("{0} test completed.".format(robotcontrol.Auboi5Robot.get_local_time()))
    #     bExite =True
if __name__ == '__main__':
    robot_login()
    bExite = False
    HOST='192.168.1.111'
    PORT=8899
    BUFSIZ=1024
    ADDR=(HOST,PORT)
    tcpSerSock =socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcpSerSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 重用地址端口
    tcpSerSock.bind(ADDR)
    tcpSerSock.listen(1)
    t=threading.Thread(target=SochetServer, args=())
    t.start()
    t.join()
    # 断开服务器链接
    if robot.connected:
       robot.disconnect()
    robotcontrol.Auboi5Robot.initialize()


