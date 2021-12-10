#! /usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from shipsim_msgs_module.msg import MMGControl


class MmgModelNode(Node):
    """ShipMMGModelNode."""

    cmd_vel_Twist = Twist()

    n_p = 0.0
    rudder_angle_degree = 0.0

    def __init__(self):
        """init."""
        super().__init__("model", namespace="ship1")
        self.declare_parameter("ρ", 1.025)

        # KVLCC2 L7-model
        self.declare_parameter("L", 7.00)
        self.declare_parameter("B", 1.27)
        self.declare_parameter("d", 0.46)
        self.declare_parameter("nabla", 3.27)
        self.declare_parameter("xG", 0.25)
        self.declare_parameter("Cb", 0.810)
        self.declare_parameter("Dp", 0.216)
        self.declare_parameter("AR", 0.0539)
        self.declare_parameter("HR", 0.345)
        self.declare_parameter("mx_", 0.022)
        self.declare_parameter("my_", 0.223)
        self.declare_parameter("Jzz_", 0.011)
        self.declare_parameter("fa", 2.747)
        self.declare_parameter("ϵ", 1.09)
        self.declare_parameter("tR", 0.387)
        self.declare_parameter("aH", 0.312)
        self.declare_parameter("xH_", -0.464)
        self.declare_parameter("γR", 0.395)
        self.declare_parameter("lr_", -0.710)
        self.declare_parameter("κ", 0.50)
        self.declare_parameter("tP", 0.220)
        self.declare_parameter("wpo", 0.40)

        # Maneuvering coefficients
        self.declare_parameter("k0", 0.2931)
        self.declare_parameter("k1", -0.2753)
        self.declare_parameter("k2", -0.1385)
        self.declare_parameter("R_0", 0.022)
        self.declare_parameter("X_vv", -0.040)
        self.declare_parameter("X_vr", 0.002)
        self.declare_parameter("X_rr", 0.011)
        self.declare_parameter("X_vvvv", 0.771)

        self.declare_parameter("Y_v", -0.315)
        self.declare_parameter("Y_r", 0.083)
        self.declare_parameter("Y_vvr", 0.379)
        self.declare_parameter("Y_vrr", -0.391)
        self.declare_parameter("Y_vvv", -1.607)
        self.declare_parameter("Y_rrr", 0.008)

        self.declare_parameter("N_v", -0.137)
        self.declare_parameter("N_r", -0.049)
        self.declare_parameter("N_vvr", -0.294)
        self.declare_parameter("N_vrr", 0.055)
        self.declare_parameter("N_vvv", -0.030)
        self.declare_parameter("N_rrr", -0.013)

        self.declare_parameter("publish_address", "/ship1/cmd_vel")
        self.declare_parameter("subscribe_address", "/ship1/control_input")
        self.declare_parameter("delta_time", 0.01)

        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.pub_cmd_vel = self.create_publisher(Twist, publish_address, 10)

        subscribe_address = (
            self.get_parameter("subscribe_address").get_parameter_value().string_value
        )
        self.subscription = self.create_subscription(
            MMGControl, subscribe_address, self.listener_callback, 10
        )

        delta_time = self.get_parameter("delta_time").value
        self.timer = self.create_timer(delta_time, self.sender_callback)

    def sender_callback(self):
        """sender_callback."""
        delta_time = self.get_parameter("delta_time").value
        u_now = self.cmd_vel_Twist.linear.x
        v_now = self.cmd_vel_Twist.linear.y
        r_now = self.cmd_vel_Twist.angular.z
        self.cmd_vel_Twist = self.get_twist_from_MMG(
            u_now,
            v_now,
            r_now,
            self.n_p,
            self.rudder_angle_degree,
            delta_time,
        )
        self.pub_cmd_vel.publish(self.cmd_vel_Twist)
        self.get_logger().info('Publishing: "%s"' % self.cmd_vel_Twist)

    def get_twist_from_MMG(self, u_now, v_now, r_now, n_p, rudder_angle_degree, delta_time):
        twist = Twist()
        rudder_angle = rudder_angle_degree * np.pi / 180.0

        ρ = self.get_parameter("ρ").value
        L = self.get_parameter("L").value
        B = self.get_parameter("B").value
        d = self.get_parameter("d").value
        nabla = self.get_parameter("nabla").value
        xG = self.get_parameter("xG").value
        Cb = self.get_parameter("Cb").value
        Dp = self.get_parameter("Dp").value
        AR = self.get_parameter("AR").value
        HR = self.get_parameter("HR").value
        mx_ = self.get_parameter("mx_").value
        my_ = self.get_parameter("my_").value
        Jzz_ = self.get_parameter("Jzz_").value
        fa = self.get_parameter("fa").value
        ϵ = self.get_parameter("ϵ").value
        tR = self.get_parameter("tR").value
        aH = self.get_parameter("aH").value
        xH_ = self.get_parameter("xH_").value
        γR = self.get_parameter("γR").value
        lr_ = self.get_parameter("lr_").value
        κ = self.get_parameter("κ").value
        tP = self.get_parameter("tP").value
        wpo = self.get_parameter("wpo").value

        # 操縦性微係数
        k0 = self.get_parameter("k0").value
        k1 = self.get_parameter("k1").value
        k2 = self.get_parameter("k2").value
        R_0 = self.get_parameter("R_0").value
        X_vv = self.get_parameter("X_vv").value
        X_vr = self.get_parameter("X_vr").value
        X_rr = self.get_parameter("X_rr").value
        X_vvvv = self.get_parameter("X_vvvv").value
        Y_v = self.get_parameter("Y_v").value
        Y_r = self.get_parameter("Y_r").value
        Y_vvr = self.get_parameter("Y_vvr").value
        Y_vrr = self.get_parameter("Y_vrr").value
        Y_vvv = self.get_parameter("Y_vvv").value
        Y_rrr = self.get_parameter("Y_rrr").value
        N_v = self.get_parameter("N_v").value
        N_r = self.get_parameter("N_r").value
        N_vvr = self.get_parameter("N_vvr").value
        N_vrr = self.get_parameter("N_vrr").value
        N_vvv = self.get_parameter("N_vvv").value
        N_rrr = self.get_parameter("N_rrr").value

        # MMG計算のためのパラメータ
        m = nabla*1.025 # 質量
        m_ = nabla/(0.5*L**2*d) # 質量(無次元化)
        mx = mx_*(0.5*ρ*L**2*d) # 付加質量x
        my = my_*(0.5*ρ*L**2*d) # 付加質量y
        Jzz = Jzz_*(0.5*ρ*L**4*d)
        xG_ = xG/L # 無次元化
        xH = xH_*L
        kx = 0.25*L # 慣動半径
        Izz = m*(0.25*L)**2 # 慣性モーメント[-]
        η = Dp/HR
        #U = np.sqrt(X[0]**2+(X[1]-X[2]*xG)**2) #合速度(重心周りで考える際にはこちら)
        #β = 0.0 if U==0.0 else np.arcsin(-(X[1]-X[2]*xG)/U) #斜航角(重心周りで考える際にはこちら)
        U = np.sqrt(u_now**2+v_now**2) #合速度
        β = 0.0 if U==0.0 else np.arcsin(-v_now/U) #斜航角
        
        v_dash = 0.0 if U==0.0 else v_now/U #無次元化された横方向速度
        r_dash = 0.0 if U==0.0 else r_now*L/U #無次元化された回頭角速度
        J = 0.0 if n_p==0.0 else (1-wpo)*u_now/(n_p*Dp) #前進常数
        K_T = k0+k1*J+k2*J**2 #スラスト係数kx
        v_R = U*γR*(β-lr_*r_dash) #舵に流入する横方向速度成分
        #v_R_ = γR*(β-lr_*r_dash) #無次元化
        u_R = np.sqrt(η*(κ*ϵ*8.0*k0*n_p**2*Dp**4/np.pi)**2) if J==0.0 else u_now*(1-wpo)*ϵ*np.sqrt(η*(1.0+κ*(np.sqrt(1.0+8.0*K_T/(np.pi*J**2))-1))**2+(1-η)) #舵に流入する前後方向速度成分
        #u_R_ = (1-wpo)**2*ϵ**2*(η*(1.0+κ*(np.sqrt(1.0+8.0*K_T/(np.pi*J**2))-1))**2+(1-η)) #無次元化
        U_R = np.sqrt(u_R**2+v_R**2) #舵への流入速度
        α_R = rudder_angle - np.arctan2(v_R,u_R) #舵への流入角度
        #α_R_ = X[6] - np.arctan2(v_R_,u_R_) #無次元化
        F_N = 0.5*AR*ρ*fa*(U_R**2)*np.sin(α_R)
        #F_N_ = AR/(L*d)*fa*(u_R_**2+v_R_**2)*np.sin(α_R_) #無次元化

        # 力の成分
        X_H = 0.5*ρ*L*d*(U**2)*(-(R_0)+X_vv*v_dash**2+X_vr*v_dash*r_dash+X_rr*r_dash**2+X_vvvv*v_dash**4)
        #X_H = 0.5*ρ*L*d*(U**2)*(-R_0+X_vv*v_dash**2+(X_vr+m_+my_)*v_dash*r_dash+(X_rr+xG_*m_)*r_dash**2+X_vvvv*v_dash**4) #付加質量で調整
        X_R = -(1-tR)*F_N*np.sin(rudder_angle)
        X_P = (1-tP)*ρ*K_T*n_p**2*Dp**4
        
        Y_H = 0.5*ρ*L*d*(U**2)*(Y_v*v_dash+Y_r*r_dash+Y_vvr*(v_dash**2)*r_dash+Y_vrr*v_dash*(r_dash**2)+Y_vvv*(v_dash**3)+Y_rrr*(r_dash**3))
        #Y_H = 0.5*ρ*L*d*(U**2)*(Y_v*v_dash+(Y_r-m_-mx_)*r_dash+Y_vvr*(v_dash**2)*r_dash+Y_vrr*v_dash*(r_dash**2)+Y_vvv*(v_dash**3)+Y_rrr*(r_dash**3)) #付加質量で調整
        Y_R = -(1+aH)*F_N*np.cos(rudder_angle)
        
        N_H = 0.5*ρ*(L**2)*d*(U**2)*(N_v*v_dash+N_r*r_dash+N_vvr*(v_dash**2)*r_dash+N_vrr*v_dash*(r_dash**2)+N_vvv*(v_dash**3)+N_rrr*(r_dash**3))
        #N_H = 0.5*ρ*(L**2)*d*(U**2)*(N_v*v_dash+(N_r-xG_*m_)*r_dash+N_vvr*(v_dash**2)*r_dash+N_vrr*v_dash*(r_dash**2)+N_vvv*(v_dash**3)+N_rrr*(r_dash**3)) #付加質量で調整
        N_R = -(-0.5+aH*xH)*F_N*np.cos(rudder_angle)
        
        # 状態計算
        u_dot = ((X_H+X_R+X_P)+(m+my)*v_now*r_now+xG*m*r_now**2)/(m+mx)
        twist.linear.x = u_now + u_dot * delta_time

        #d_v =(Y_H+Y_R-(m+mx)*X[0]*X[2]-xG*m*d_r)/(m+my)
        v_dot = (xG**2*m**2*u_now*r_now-(N_H+N_R)*xG*m+((Y_H+Y_R)-(m+mx)*u_now*r_now)*(Izz+Jzz+xG**2*m))/((Izz+Jzz+xG**2*m)*(m+my)-xG**2*m**2)
        twist.linear.y = v_now + v_dot * delta_time

        #d_r = (N_H+N_R-xG*m*(d_v+X[0]*X[2]))/(Izz+Jzz+xG**2*m)
        r_dot = ((N_H+N_R)*(m+my)-xG*m*(Y_H+Y_R-(m+mx)*u_now*r_now)-xG*m*u_now*r_now*(m+my))/((Izz+Jzz+xG**2*m)*(m+my)-xG**2*m**2)
        twist.angular.z = r_now + r_dot * delta_time

        return twist

    def listener_callback(self, msg):
        """listener_callback."""
        self.get_logger().info(
            'MMG_model_node heard: n_p="%s", \
                rudder_angle="%s"'
            % (msg.n_p, msg.rudder_angle_degree)
        )
        self.n_p = msg.n_p
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