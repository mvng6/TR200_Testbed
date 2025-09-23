#!/usr/bin/env python3
import rospy
import asyncio
import threading
import time
from geometry_msgs.msg import Twist
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT
from woosh.proto.robot.robot_pack_pb2 import Twist as WooshTwist

class TR200VelocityBridge:
    def __init__(self):
        rospy.init_node('tr200_velocity_bridge')
        
        # TR200 연결 설정
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        
        # ROS 구독자 설정
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Woosh Robot SDK 초기화
        self.robot = None
        self.robot_connected = False
        self.setup_robot()
        
        rospy.loginfo("TR200 Velocity Bridge 시작됨")
        
    def setup_robot(self):
        """로봇 연결 설정"""
        try:
            settings = CommuSettings(
                addr=self.robot_ip,
                port=self.robot_port,
                identity="ros_velocity_bridge"
            )
            self.robot = WooshRobot(settings)
            
            # 별도 스레드에서 연결 시도
            self.connect_thread = threading.Thread(target=self.connect_robot_sync)
            self.connect_thread.daemon = True
            self.connect_thread.start()
            
        except Exception as e:
            rospy.logerr(f"로봇 연결 설정 실패: {e}")
    
    def connect_robot_sync(self):
        """동기적으로 로봇 연결"""
        try:
            # 새로운 이벤트 루프 생성
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            # 연결 시도
            connected = loop.run_until_complete(self.robot.run())
            
            if connected:
                self.robot_connected = True
                rospy.loginfo("TR200 로봇 연결 성공")
            else:
                rospy.logerr("TR200 로봇 연결 실패")
                
        except Exception as e:
            rospy.logerr(f"로봇 연결 오류: {e}")
    
    def cmd_vel_callback(self, msg):
        """ROS cmd_vel 토픽 콜백"""
        if not self.robot_connected:
            rospy.logwarn("로봇이 연결되지 않음")
            return
            
        # 별도 스레드에서 속도 명령 전송
        cmd_thread = threading.Thread(
            target=self.send_velocity_command_sync,
            args=(msg.linear.x, msg.angular.z)
        )
        cmd_thread.daemon = True
        cmd_thread.start()
    
    def send_velocity_command_sync(self, linear, angular):
        """동기적으로 속도 명령 전송"""
        try:
            # 새로운 이벤트 루프 생성
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            # 속도 명령 생성 및 전송
            twist_cmd = WooshTwist(linear=linear, angular=angular)
            _, ok, msg = loop.run_until_complete(
                self.robot.twist_req(twist_cmd, NO_PRINT, NO_PRINT)
            )
            
            if ok:
                rospy.logdebug(f"속도 명령 전송 성공: linear={linear:.2f}, angular={angular:.2f}")
            else:
                rospy.logwarn(f"속도 명령 전송 실패: {msg}")
                
        except Exception as e:
            rospy.logerr(f"속도 명령 전송 오류: {e}")

if __name__ == '__main__':
    try:
        bridge = TR200VelocityBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
