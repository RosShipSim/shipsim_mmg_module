#! /usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from shipsim_msgs_module.msg import MMGControl


class MmgModelNode(Node):
    """ShipMMGModelNode."""

    cmd_vel_Twist = Twist()

    rps = 0.0
    rudder_angle_degree = 0.0

    def __init__(
        self,
        # 船体情報L7-model
        L = 7.00,  # 船長Lpp[m]
        B = 1.27,  # 船幅[m]
        d = 0.46,  # 喫水[m]
        nabla = 3.27,  # 排水量[m^3]
        xG = 0.25,  # 重心位置[-]
        Cb = 0.810, # 方形係数[-] 
        Dp = 0.216,  # プロペラ直径[m] 
        AR = 0.0539,  # 舵面積比[-]
        HR = 0.345,  #  舵高さ[m]
        mx_ = 0.022,  # 付加質量x(無次元)
        my_ = 0.223,  # 付加質量y(無次元)
        Jzz_ = 0.011,  # 付加質量Izz(無次元)
        fa = 2.747,  # 直圧力勾配係数
        ϵ = 1.09,  # プロペラ・舵位置伴流係数比
        tR = 0.387,  # 操縦抵抗減少率
        aH = 0.312,  # 舵力増加係数
        xH_ = -0.464,  # 舵力増分作用位置
        γR = 0.395,  # 整流係数
        lr_ = -0.710,  # 船長に対する舵位置
        κ = 0.50,  # 修正係数
        tP = 0.220,  # 推力減少率
        wpo = 0.40,  # 有効伴流率
        
        # 操縦性微係数
        k0 = 0.2931,
        k1 = -0.2753,
        k2 = -0.1385,
        
        R_0 = 0.022,

        X_vv = -0.040,
        X_vr = 0.002,
        X_rr = 0.011,
        X_vvvv = 0.771,

        Y_v = -0.315,
        Y_r = 0.083,
        Y_vvr = 0.379,
        Y_vrr = -0.391,
        Y_vvv = -1.607,
        Y_rrr = 0.008,

        N_v = -0.137,
        N_r = -0.049,
        N_vvr = -0.294,
        N_vrr = 0.055,
        N_vvv = -0.030,
        N_rrr = -0.013,

        publish_address="/ship1/cmd_vel",
        timer_period=0.1,
    ):
        """init."""
        super().__init__("ship_model")
        self.delta_time = timer_period

        self.L = L
        self.B = B
        self.d = d
        self.nabla  = nabla
        self.xG = xG
        self.Dp = Dp
        self.Cb = Cb
        self.AR = AR
        self.HR = HR
        self.mx_ = mx_
        self.my_ = my_
        self.Jzz_ = Jzz_
        self.fa = fa
        self.ϵ = ϵ
        self.tR = tR
        self.aH = aH
        self.xH_ = xH_
        self.γR = γR
        self.lr_ = lr_
        self.κ = κ
        self.tP = tP
        self.wpo = wpo

        self.k0 = k0
        self.k1 = k1
        self.k2 = k2

        self.R_0 = R_0

        self.X_vv = X_vv
        self.X_vr = X_vr
        self.X_rr = X_rr
        self.X_vvvv = X_vvvv

        self.Y_v = Y_v
        self.Y_r = Y_r
        self.Y_vvr = Y_vvr
        self.Y_vrr = Y_vrr
        self.Y_vvv = Y_vvv
        self.Y_rrr = Y_rrr

        self.N_v = N_v
        self.N_r = N_r
        self.N_vvr = N_vvr
        self.N_vrr = N_vrr
        self.N_vvv = N_vvv
        self.N_rrr = N_rrr

        self.pub_cmd_vel = self.create_publisher(Twist, publish_address, 10)

        self.subscription = self.create_subscription(
            MMGControl, "ship1/actuator", self.listener_callback, 10
        )

        self.timer = self.create_timer(timer_period, self.sender_callback)

    def sender_callback(self):
        """sender_callback."""
        u_now = self.cmd_vel_Twist.linear.x
        v_now = self.cmd_vel_Twist.linear.y
        r_now = self.cmd_vel_Twist.angular.z
        self.cmd_vel_Twist = self.get_twist_from_MMG(
            u_now,
            v_now,
            r_now,
            self.rps,
            self.rudder_angle_degree,
            self.delta_time,
        )
        self.pub_cmd_vel.publish(self.cmd_vel_Twist)
        self.get_logger().info('Publishing: "%s"' % self.cmd_vel_Twist)

    def get_twist_from_MMG(self, u_now, v_now, r_now, rps, rudder_angle_degree, delta_time):
        twist = Twist()
        rudder_angle = rudder_angle_degree * np.pi / 180.0
        ρ = 1.025

        # MMG計算のためのパラメータ
        m = self.nabla*1.025 # 質量
        m_ = self.nabla/(0.5*self.L**2*self.d) # 質量(無次元化)
        mx = self.mx_*(0.5*ρ*self.L**2*self.d) # 付加質量x
        my = self.my_*(0.5*ρ*self.L**2*self.d) # 付加質量y
        Jzz = self.Jzz_*(0.5*ρ*self.L**4*self.d)
        xG_ = self.xG/self.L # 無次元化
        xH = self.xH_*self.L
        kx = 0.25*self.L # 慣動半径
        Izz = m*(0.25*self.L)**2 # 慣性モーメント[-]
        η = self.Dp/self.HR
        #U = np.sqrt(X[0]**2+(X[1]-X[2]*self.xG)**2) #合速度(重心周りで考える際にはこちら)
        #β = 0.0 if U==0.0 else np.arcsin(-(X[1]-X[2]*self.xG)/U) #斜航角(重心周りで考える際にはこちら)
        U = np.sqrt(u_now**2+v_now**2) #合速度
        β = 0.0 if U==0.0 else np.arcsin(-v_now/U) #斜航角
        
        v_dash = 0.0 if U==0.0 else v_now/U #無次元化された横方向速度
        r_dash = 0.0 if U==0.0 else r_now*self.L/U #無次元化された回頭角速度
        J = 0.0 if rps==0.0 else (1-self.wpo)*u_now/(rps*self.Dp) #前進常数
        K_T = self.k0+self.k1*J+self.k2*J**2 #スラスト係数kx
        v_R = U*self.γR*(β-self.lr_*r_dash) #舵に流入する横方向速度成分
        #v_R_ = self.γR*(β-self.lr_*r_dash) #無次元化
        u_R = np.sqrt(η*(self.κ*self.ϵ*8.0*self.k0*rps**2*self.Dp**4/np.pi)**2) if J==0.0 else u_now*(1-self.wpo)*self.ϵ*np.sqrt(η*(1.0+self.κ*(np.sqrt(1.0+8.0*K_T/(np.pi*J**2))-1))**2+(1-η)) #舵に流入する前後方向速度成分
        #u_R_ = (1-self.wpo)**2*self.ϵ**2*(η*(1.0+self.κ*(np.sqrt(1.0+8.0*K_T/(np.pi*J**2))-1))**2+(1-η)) #無次元化
        U_R = np.sqrt(u_R**2+v_R**2) #舵への流入速度
        α_R = rudder_angle - np.arctan2(v_R,u_R) #舵への流入角度
        #α_R_ = X[6] - np.arctan2(v_R_,u_R_) #無次元化
        F_N = 0.5*self.AR*ρ*self.fa*(U_R**2)*np.sin(α_R)
        #F_N_ = self.AR/(self.L*self.d)*self.fa*(u_R_**2+v_R_**2)*np.sin(α_R_) #無次元化

        # 力の成分
        X_H = 0.5*ρ*self.L*self.d*(U**2)*(-(self.R_0)+self.X_vv*v_dash**2+self.X_vr*v_dash*r_dash+self.X_rr*r_dash**2+self.X_vvvv*v_dash**4)
        #X_H = 0.5*ρ*self.L*self.d*(U**2)*(-R_0+X_vv*v_dash**2+(X_vr+m_+self.my_)*v_dash*r_dash+(X_rr+xG_*m_)*r_dash**2+X_vvvv*v_dash**4) #付加質量で調整
        X_R = -(1-self.tR)*F_N*np.sin(rudder_angle)
        X_P = (1-self.tP)*ρ*K_T*rps**2*self.Dp**4
        
        Y_H = 0.5*ρ*self.L*self.d*(U**2)*(self.Y_v*v_dash+self.Y_r*r_dash+self.Y_vvr*(v_dash**2)*r_dash+self.Y_vrr*v_dash*(r_dash**2)+self.Y_vvv*(v_dash**3)+self.Y_rrr*(r_dash**3))
        #Y_H = 0.5*ρ*self.L*self.d*(U**2)*(Y_v*v_dash+(Y_r-m_-self.mx_)*r_dash+Y_vvr*(v_dash**2)*r_dash+Y_vrr*v_dash*(r_dash**2)+Y_vvv*(v_dash**3)+Y_rrr*(r_dash**3)) #付加質量で調整
        Y_R = -(1+self.aH)*F_N*np.cos(rudder_angle)
        
        N_H = 0.5*ρ*(self.L**2)*self.d*(U**2)*(self.N_v*v_dash+self.N_r*r_dash+self.N_vvr*(v_dash**2)*r_dash+self.N_vrr*v_dash*(r_dash**2)+self.N_vvv*(v_dash**3)+self.N_rrr*(r_dash**3))
        #N_H = 0.5*ρ*(self.L**2)*self.d*(U**2)*(N_v*v_dash+(N_r-xG_*m_)*r_dash+N_vvr*(v_dash**2)*r_dash+N_vrr*v_dash*(r_dash**2)+N_vvv*(v_dash**3)+N_rrr*(r_dash**3)) #付加質量で調整
        N_R = -(-0.5+self.aH*xH)*F_N*np.cos(rudder_angle)
        
        # 状態計算
        u_dot = ((X_H+X_R+X_P)+(m+my)*v_now*r_now+self.xG*m*r_now**2)/(m+mx)
        twist.linear.x = u_now + u_dot * delta_time

        #d_v =(Y_H+Y_R-(m+mx)*X[0]*X[2]-self.xG*m*d_r)/(m+my)
        v_dot = (self.xG**2*m**2*u_now*r_now-(N_H+N_R)*self.xG*m+((Y_H+Y_R)-(m+mx)*u_now*r_now)*(Izz+Jzz+self.xG**2*m))/((Izz+Jzz+self.xG**2*m)*(m+my)-self.xG**2*m**2)
        twist.linear.y = v_now + v_dot * delta_time

        #d_r = (N_H+N_R-self.xG*m*(d_v+X[0]*X[2]))/(Izz+Jzz+self.xG**2*m)
        r_dot = ((N_H+N_R)*(m+my)-self.xG*m*(Y_H+Y_R-(m+mx)*u_now*r_now)-self.xG*m*u_now*r_now*(m+my))/((Izz+Jzz+self.xG**2*m)*(m+my)-self.xG**2*m**2)
        twist.angular.z = r_now + r_dot * delta_time

        return twist

    def listener_callback(self, msg):
        """listener_callback."""
        self.get_logger().info(
            'MMG_model_node heard: rps="%s", \
                rudder_angle="%s"'
            % (msg.rps, msg.rudder_angle_degree)
        )
        self.rps = msg.rps
        self.rudder_angle_degree = msg.rudder_angle_degree


def main(args=None):
    """Run main."""
    rclpy.init(args=args)

    node = MmgModelNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()