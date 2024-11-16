import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # Segundos
        self.timer = self.create_timer(timer_period, self.move_drone)

    def move_drone(self):
        msg = Twist()
        msg.linear.x = 0.5  # Movimento para frente
        msg.angular.z = 0.0  # Sem rotação
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
