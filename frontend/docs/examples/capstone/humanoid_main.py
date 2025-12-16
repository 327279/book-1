import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.subscription = self.create_subscription(
            String,
            'voice_commands',
            self.command_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'motor_commands', 10)

    def command_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        # Logic to convert voice command to motor control
        if "walk" in msg.data.lower():
            self.walk_forward()
    
    def walk_forward(self):
        msg = String()
        msg.data = "EXECUTE_GAIT_WALK"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
