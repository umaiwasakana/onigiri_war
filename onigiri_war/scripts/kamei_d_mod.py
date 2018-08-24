#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from cvtest.msg import cv_msg     #←これを追加

'''
変えたところ
　速度を統一、はやくなり過ぎないように+
  あといろいろ
'''

class move():
    def __init__(self, bot_name):
        self.name = bot_name
        self.i = 0
        self.counter = 0
        self.tmp_counter = 0
        self.Atack_counter = 0
        self.stuck_counter = 0
        self.counter_flag = False          #めんどくせ
        self.cant_move_flag = False
        self.wheel_pre_rot_r = 0.0         #前回の値
        self.wheel_pre_rot_l = 0.0         #前回の値
        self.wheel_ini_rot_r = 0.0         #初期値
        self.wheel_ini_rot_l = 0.0         #初期値
        self.wheel_rot_r = 0.0             #今回の値
        self.wheel_rot_l = 0.0             #今回の値
        self.wheel_rot_temp = 0.0          #前回の仮保存値
        self.tmp_use = 0.0
        self.tmp_timer = rospy.Time.now()
        self.x = 0
        self.th = 0
        self.linear_speed = 0.4
        self.angular_speed = 0.5          #この辺が限界？
        self.cant_move_counter = 0
        # # 超音波センサ検知 (超音波センサ 使うとき有効)
        self.us_left_detect = False
        self.us_right_detect = False
        # # 敵の確認状況
        self.enemyfind_flag = False
        self.enemy_place = 0.0
        self.enemy_vel = 0.0
        self.enemy_dis = 0.0

        # # Velocity
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # # 車輪回転情報
        self.rot_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)
        # # 赤外線センサ
        self.opt_left_sub = rospy.Subscriber('opt_left', LaserScan, self.optLeftCallback)
        # # 超音波センサ
        self.sub_us_left = rospy.Subscriber('us_left', LaserScan, self.us_left_callback, queue_size=1)
        self.sub_us_right = rospy.Subscriber('us_right', LaserScan, self.us_right_callback, queue_size=1)
        # # 画像センサ
        self.cvtest_sub = rospy.Subscriber('cvtest', cv_msg, self.Callback)     #←これを追加

    # ### 画像センサTopic Subscribe時のCallback関数
    def Callback(self, data):
#        print('test')
#        print(data.status)
#        print(data.isdoubleget)
#        print(data.place)
#        print(data.enemy_vel)
#        print(data.enemy_dis)
        self.enemyfind_flag = data.status
        self.enemy_place = data.place
        self.enemy_vel = data.enemy_vel
        self.enemy_dis = data.enemy_dis


    # ### 赤外線センサTopic Subscribe時のCallback関数(左)
    def optLeftCallback(self, data):
        self.ranges0 = data.ranges[0]     #壁からの距離(m)
        self.ranges1 = data.ranges[1]
        self.ranges2 = data.ranges[2]
        self.mintoWall = min(data.ranges)

    # ### 超音波センサTopic Subscribe時のCallback関数(左) (超音波センサ 使うとき有効)
    def us_left_callback(self, sens):
        self.us_left_ranges = sens.ranges[0]
        if self.us_left_ranges < 0.25:           # [unit :m]
            self.us_left_detect = True
        else:
            self.us_left_detect = False
#        print('us_left = ',self.us_left_detect,',  ',self.us_left_ranges)

    # ### 超音波センサTopic Subscribe時のCallback関数(右) (超音波センサ 使うとき有効)
    def us_right_callback(self, sens):
        self.us_right_ranges = sens.ranges[0]
        if self.us_right_ranges < 0.25:           # [unit :m]
            self.us_right_detect = True
        else:
            self.us_right_detect = False
#        print('us_right = ',self.us_right_detect,',  ',self.us_right_ranges)

    def jointstateCallback(self, data):     #謎ムーブ
        if self.wheel_rot_temp != data.position[1]: #前回の[0]と今回の[1]が異なる時に値を更新する
            self.wheel_pre_rot_l = self.wheel_rot_l   #前回の値の保存
            self.wheel_pre_rot_r = self.wheel_rot_r   #前回の値の保存
            if self.i < 3:          #おまじないの待機時間
                self.wheel_ini_rot_l = data.position[0]
                self.wheel_ini_rot_r = data.position[1]
                self.wheel_rot_l = data.position[0] - self.wheel_ini_rot_l
                self.wheel_rot_r = data.position[1] - self.wheel_ini_rot_r
                self.i += 1
            else:
                self.wheel_rot_l = data.position[0] - self.wheel_ini_rot_l
                self.wheel_rot_r = data.position[1] - self.wheel_ini_rot_r
#            print('Jointstate_Callback_d ' + str(self.i) + ',' + str(data.position[0]) + ',' + str(data.position[1]))
#            print('Jointstate_Callback_n ' + str(self.i) + ',' + str(self.wheel_rot_r) + ',' + str(self.wheel_rot_l))
#            print('Jointstate_Callback_p ' + str(self.i) + ',' + str(self.wheel_pre_rot_r) + ',' + str(self.wheel_pre_rot_l))
        self.wheel_rot_temp = data.position[0] # [0]の値を保存

    def calcTwist(self):      #とりあえず回る
        print(self.counter)
        #司令通りに移動できなくなっているかの判定
        if (self.x >= 0.1 or self.x <= -0.1) and self.th== 0.0:
            wheel_ave = (abs(self.wheel_rot_r - self.wheel_pre_rot_r) + abs(self.wheel_rot_l - self.wheel_pre_rot_l) + 0.1)/2 #左右ホイールの移動距離の平均
#            print('wheel_ave x = ' + str(wheel_ave))
            #x=0.1の時、ホイールの移動距離は約0.33/1周期
            if wheel_ave < 0.3 and self.wheel_pre_rot_r != 0.0:
#                print('Can not move x now ' + str(self.wheel_rot_r) + ',' + str(self.wheel_rot_l))
#                print('Can not move x pre ' + str(self.wheel_pre_rot_r) + ',' + str(self.wheel_pre_rot_l))
                self.cant_move_counter += 1
            else:
                self.cant_move_counter = 0
        elif self.th >= 0.1 or self.th <= -0.1:
            wheel_ave = (abs(self.wheel_rot_r - self.wheel_pre_rot_r) + abs(self.wheel_rot_l - self.wheel_pre_rot_l) + 0.01)/2 #左右ホイールの移動距離の平均
#            print('wheel_ave th = ' + str(wheel_ave))
            #th=0.1の時、ホイールの移動距離は約0.045/1周期　360度で約30.0
            if wheel_ave < 0.04 and self.wheel_pre_rot_r != 0.0:
#                print('Can not move th now ' + str(self.wheel_rot_r) + ',' + str(self.wheel_rot_l))
#                print('Can not move th pre ' + str(self.wheel_pre_rot_r) + ',' + str(self.wheel_pre_rot_l))
                self.cant_move_counter += 1
            else:
                self.cant_move_counter = 0
        if self.cant_move_counter >= 3:
            self.cant_move_counter = 0
            self.cant_move_flag = True
            print('Can not move, change the counter!! ')

        #左に回転し壁に向かう
        if self.counter == 0 and abs(self.wheel_rot_r - self.wheel_rot_l) < 4.9:
            x = 0.0
            th = self.angular_speed
        elif self.counter == 0:
            self.counter_flag = True

        #左に回転し壁に向かう直進
        if self.counter == 1 and self.wheel_rot_l < 55: #45
            x = self.linear_speed
#            print(str(self.counter) + ': OP ' + str(self.mintoWall) + ',' + str(self.ranges0) + ',' + str(self.ranges1) + ',' + str(self.ranges2))
#            print(str(self.counter) + ': US ' + str(self.us_left_detect) + ',' + str(self.us_left_ranges) + ',' + str(self.us_right_ranges))
            #赤外線センサが距離を検知し、壁に向かっている時
            if self.mintoWall < 0.2 and self.us_left_detect == True:
                if self.mintoWall < 0.08:
                    x = 0.0
                    th = -0.2
                elif self.mintoWall < 0.09:
                    x = 0.1
                    th = -0.2
                elif self.mintoWall > 0.10:
                    x = self.linear_speed
                    th = 0.2
                else:
                    th = 0.0
            #赤外線センサが距離を検知し、壁に向かっていない時
            elif self.mintoWall < 0.2:
                if self.mintoWall < 0.05:
                    x = 0.0
                    th = -0.3
                elif self.mintoWall < 0.08:
                    x = 0.1
                    th = -0.2
                elif self.mintoWall > 0.10:
                    x = self.linear_speed
                    th = 0.2
                else:
                    th = 0.0
            else :
                x = self.linear_speed
                th = 0.2

        elif self.counter == 1:
            self.counter_flag = True
            self.tmp_use = abs(self.wheel_rot_r - self.wheel_rot_l)


        if self.counter == 2 and self.tmp_counter < 2:
            dir_symbol = -1 + 2 * (self.tmp_counter % 2)  #0→-1　1→1
            if abs(abs(self.wheel_rot_r - self.wheel_rot_l) - self.tmp_use) < 4.5:
                x = 0
                th = dir_symbol * self.angular_speed
            else:
                x = 0
                th = 0
                self.tmp_counter += 1
                self.tmp_use = abs(self.wheel_rot_r - self.wheel_rot_l)
        elif self.counter == 2:
            self.counter_flag = True
            self.tmp_counter = 0

        if self.counter == 3 and not (self.us_right_detect): #45
            x = self.linear_speed
#            print(str(self.counter) + ': OP ' + str(self.mintoWall) + ',' + str(self.ranges0) + ',' + str(self.ranges1) + ',' + str(self.ranges2))
#            print(str(self.counter) + ': US ' + str(self.us_left_detect) + ',' + str(self.us_left_ranges) + ',' + str(self.us_right_ranges))
            #赤外線センサが距離を検知し、壁に向かっている時
            if self.mintoWall < 0.2 and self.us_left_detect == True:
                if self.mintoWall < 0.08:
                    x = 0.0      #なんか全力で走るとthの設定が効いてないっぽい
                    th = -0.2
                elif self.mintoWall < 0.09:
                    x = 0.1      #なんか力で走るとthの設定が効いてないっぽい
                    th = -0.2
                elif self.mintoWall > 0.10:
                    x = self.linear_speed      #なんか力で走るとthの設定が効いてないっぽい
                    th = 0.2
                else:
                    th = 0.0
            #赤外線センサが距離を検知し、壁に向かっていない時
            elif self.mintoWall < 0.2:
                if self.mintoWall < 0.05:
                    x = 0.0      #なんか全力で走るとthの設定が効いてないっぽい
                    th = -0.3
                elif self.mintoWall < 0.08:
                    x = 0.1      #なんか力で走るとthの設定が効いてないっぽい
                    th = -0.2
                elif self.mintoWall > 0.10:
                    x = self.linear_speed      #なんか力で走るとthの設定が効いてないっぽい
                    th = 0.2
                else:
                    th = 0.0
            else :
                x = self.linear_speed      #なんか力で走るとthの設定が効いてないっぽい
                th = 0.2

        elif self.counter == 3:
            self.counter_flag = True

        #少しバック
        if self.counter == 4 and self.wheel_rot_l > -1.5:
            x = -self.linear_speed
            th = 0.0
#            print(str(self.counter) + ': OP ' + str(self.mintoWall) + ',' + str(self.ranges0) + ',' + str(self.ranges1) + ',' + str(self.ranges2))
#            print(str(self.counter) + ': US ' + str(self.us_left_detect) + ',' + str(self.us_left_ranges) + ',' + str(self.us_right_ranges))
        elif self.counter == 4:
            self.counter_flag = True

        #右回転約90度
        if self.counter == 5 and abs(self.wheel_rot_r - self.wheel_rot_l) < 11:
            x = 0
            th = -1 * self.angular_speed
#            print(str(self.counter) + ': OP ' + str(self.mintoWall) + ',' + str(self.ranges0) + ',' + str(self.ranges1) + ',' + str(self.ranges2))
#            print(str(self.counter) + ': US ' + str(self.us_left_detect) + ',' + str(self.us_left_ranges) + ',' + str(self.us_right_ranges))
        elif self.counter == 5:
            self.counter_flag = True
            self.tmp_timer = rospy.Time.now()

        if self.counter == 6 and self.tmp_counter < 2:
            dir_symbol = -1 + 2 * (self.tmp_counter % 2)  #0→-1　1→1
            if rospy.Time.now() - self.tmp_timer < rospy.Duration(3):
                x = 0
                th = dir_symbol * self.angular_speed
            else:
                x = 0
                th = 0
                self.tmp_counter += 1
                self.tmp_timer = rospy.Time.now()
        elif self.counter == 6:
            self.counter_flag = True
            self.tmp_counter = 0


        #敵を追跡
        if self.counter == 10:   # and abs(self.wheel_rot_r - self.wheel_rot_l) < 11:
            if self.enemyfind_flag == False:
                self.stuck_counter += 1
            #確実に枠内に捉える
            if self.enemy_place < -0.6 or self.enemy_place > 0.6:
                x = 0.0
                th = -0.3 * self.enemy_place / abs(self.enemy_place)
            else:
                #敵の速度から攻撃方向を見定める
                if self.Atack_counter < 3:
                    x = 0.0
                    th = 0.0
                    self.Atack_Direction = self.enemy_vel
                    self.Atack_counter += 1
                #攻撃開始！
#                elif self.Atack_counter >= 3:
                else:
                    #敵が右に動いている時 敵が画面左側(-0.3)の位置にいるようにしながら接近
                    if self.Atack_Direction >= 0:
                        x = 0.3
                        th = -1 * (self.enemy_place + 0.3)
                    #敵が左に動いている時 敵が画面左側(0.3)の位置にいるようにしながら接近
                    else:
                        x = 0.3
                        th = -1 * (self.enemy_place - 0.3)



        if self.counter_flag:
            self.counter_flag = False
            self.i = 0
            x = 0.0
            th = 0.0
            self.cant_move_counter = 0    #動かないかの判定カウンタもリセット
            print("counter ",self.counter," end")
            if self.counter == 6:
                self.counter = 1
            elif self.counter == 10 and self.stuck_counter > 10:
                self.counter = 1
                self.Atack_counter = 0
            else:
                self.counter += 1

        if self.cant_move_flag:
            self.cant_move_flag = False
            self.cant_move_counter = 0    #動かないかの判定カウンタもリセット
            print("counter ",self.counter," end")
            self.i = 0
            x = 0.0
            th = 0.0
            self.counter = 4    #バックを設定　その後、右回転

        if self.enemyfind_flag == True and self.enemy_dis < 1.2:    #敵を発見！
            self.i = 0
#            x = 0.0
#            th = 0.0
            self.counter = 10    #敵追跡モード

        twist = Twist()
        twist.linear.x = x
        twist.angular.z = th
        self.x = x        #値の保存
        self.th = th        #値の保存
#        print(twist)
        return twist

    def strategy(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            twist = self.calcTwist()
            self.vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('runtest')
    bot = move('runtest')
    bot.strategy()
