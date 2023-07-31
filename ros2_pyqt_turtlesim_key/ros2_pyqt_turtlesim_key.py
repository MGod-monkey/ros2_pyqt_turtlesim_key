import re
import sys
import rclpy
import random
from . ui_key import *
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QMainWindow
from turtlesim.srv import Spawn, Kill
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
# from composition_interfaces.srv import LoadNode
from rclpy.node import Node    
import math
import subprocess                  

# pyuic5 -o key_ui.py key.ui

move_step = 1.0     # m/s
rotation_step = math.pi / 4       # rad/s

class objectClient(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("node launch success：%s!" % name)
        self.client_add = self.create_client(Spawn,"/spawn")
        self.client_kill = self.create_client(Kill,"/kill")
        self.client_clear = self.create_client(Empty,"/clear")
        self.publisher_move = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Using subprocess to directly start turtlesim nodes
        process = subprocess.Popen(['ros2', 'run', 'turtlesim', 'turtlesim_node'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        # stdout, stderr = process.communicate()
        # if process.poll == None:
        #     self.get_logger().info(f"{stdout}")
        # else:
        #     self.get_logger().info(f"turtlesim node start failed! error message: {result.stderr}")

    def result_callback_add(self, result_future):
        response = result_future.result()
        self.get_logger().info(f"add a turtlesim, name：{response.name}")
    
    def result_callback_kill(self, result_future):
        self.get_logger().info(f"kill a turtlesim!")

    def result_callback_clear(self, result_future):
        self.get_logger().info(f"clear the background and line!")
    
    def add_turtlesim(self):
        while rclpy.ok() and self.client_add.wait_for_service(1)==False:
            self.get_logger().info(f"Wait for the server to go online....")
        
        request = Spawn.Request()
        request.x = random.uniform(0, 9)   # position x
        request.y = random.uniform(0, 9)   # position y
        request.theta = random.uniform(0, math.pi)   # angle t

        self.client_add.call_async(request).add_done_callback(self.result_callback_add)
        
    def kill_turtlesim(self, name):
        while rclpy.ok() and self.client_kill.wait_for_service(1)==False:
            self.get_logger().info(f"Wait for the server to go online....")
        
        request = Kill.Request()
        request.name = name.replace('/', '')

        self.client_kill.call_async(request).add_done_callback(self.result_callback_kill)

    def clear_line(self):
        while rclpy.ok() and self.client_clear.wait_for_service(1)==False:
            self.get_logger().info(f"Wait for the server to go online....")
        
        request = Empty.Request()
        self.client_clear.call_async(request).add_done_callback(self.result_callback_clear)
    
    def move(self, control_key):
        twist = Twist()
        
        if control_key == "forward":
            twist.linear.x = move_step
            self.get_logger().info(f"Moving forward at a speed: {move_step}m/s")
        elif control_key == "backward":
            twist.linear.x = -move_step
            self.get_logger().info(f"Moving backward at a speed: {move_step}m/s")
        elif control_key == "turn_left":
            twist.angular.z = rotation_step
            self.get_logger().info(f"Turning left at a speed: {rotation_step}rad/s")
        elif control_key == "turn_right":
            twist.angular.z = -rotation_step
            self.get_logger().info(f"Turning right at a speed: {rotation_step}rad/s")
        elif control_key == "stop":
            twist.linear.x = 0
            twist.angular.z = 0
            self.get_logger().info("Stop moving!")
        
        self.publisher_move.publish(twist)
        # future = rclpy.spin_once_future(self.node, timeout_sec=0.1)
        # rclpy.spin_until_future_complete(self.node, future)
        
            
    def change_turtlesim(self, topic_name):
        self.get_logger().info("now control turtle: %s" % topic_name)
        self.publisher_move = self.create_publisher(Twist, topic_name+'/cmd_vel', 10)
    
    def get_turtlesim_topic_list(self):
        service_names_and_types = self.get_service_names_and_types()
        result = re.findall(r'/turtle\d+/set_pen', str(service_names_and_types))  # Get topics such as "turtle */cmd_vel" through regular expressions
        result = [r.replace('/set_pen', '') for r in result]
        
        return result

        # print('Available services:')
        # for service_name, service_type in get_topic_names_and_types:
        #     print(service_name, '(', service_type, ')')
        # return get_topic_names_and_types[0][6:-1]

    

class MyApp(QMainWindow):
    def __init__(self, node):
        QMainWindow.__init__(self)
        self.node = node

        # UI
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        # 绑定按键事件
        self.ui.btn_new.clicked.connect(self.node.add_turtlesim)             # Send a request to add a turtle node
        self.ui.btn_kill.clicked.connect(lambda: self.node.kill_turtlesim(self.ui.cbox_topic.currentText()))    # Send a request to kill now turtle
        self.ui.btn_clear.clicked.connect(self.node.clear_line)             # Send a request to clear the background and line
        self.ui.btn_update.clicked.connect(self.update_topic_list)             # Send a request to clear the background and line
        self.ui.cbox_topic.currentTextChanged.connect(lambda: self.node.change_turtlesim(self.ui.cbox_topic.currentText()))
        self.ui.btn_forward.clicked.connect(lambda: self.node.move("forward"))
        self.ui.btn_backward.clicked.connect(lambda: self.node.move("backward"))
        self.ui.btn_left.clicked.connect(lambda: self.node.move("turn_left"))
        self.ui.btn_right.clicked.connect(lambda: self.node.move("turn_right"))
    
    # 刷新订阅的topic列表
    def update_topic_list(self):
        self.ui.cbox_topic.currentTextChanged.disconnect()         # Unbind slot function to prevent slot function from triggering after clear
        self.ui.cbox_topic.clear()
        self.ui.cbox_topic.addItems([])                             # Clear all internal options
        self.ui.cbox_topic.addItems(self.node.get_turtlesim_topic_list())
        self.ui.cbox_topic.currentTextChanged.connect(lambda: self.node.change_turtlesim(self.ui.cbox_topic.currentText()))
    
    # 重写关闭事件
    def closeEvent(self, event):
        self.node.destroy_node() # close node
        rclpy.shutdown() # close rclpy
        

def main(args=None):
    rclpy.init(args=args)                                 # ROS2 Python init
    node = objectClient("pyqt_turtlesim_key")       # Create and initialize the ROS2 node object

    # UI init
    app = QApplication(sys.argv)
    myapp = MyApp(node)
    myapp.show()
    
    sys.exit(app.exec_())
    # rclpy.spin(node) # Keep the node running and check whether the exit instruction is received（Ctrl+C）
    # rclpy.shutdown() # close rclpy


if __name__ == '__main__':
    main()
