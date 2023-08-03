import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.publisher_ = self.create_publisher(Joy, 'joystick', 10)
        self.get_joystick_input()

    def get_joystick_input(self):
        pygame.init()

        pygame.joystick.init()

        # Assuming that you have only one joystick connected
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        clock = pygame.time.Clock()  # Add this line

        while True:
            for event in pygame.event.get(): 
                if event.type == pygame.QUIT: 
                    done=True
                
                elif event.type == pygame.JOYBUTTONDOWN:
                    print("Joystick button pressed.")
                elif event.type == pygame.JOYBUTTONUP:
                    print("Joystick button released.")
            
            axes = []
            for i in range(joystick.get_numaxes()):
                axes.append(float(joystick.get_axis(i)))
                print(str(i) + ": " + str(joystick.get_axis(i)))

            buttons = []
            for i in range(joystick.get_numbuttons()):
                buttons.append(int(joystick.get_button(i)))

            for i in range(joystick.get_numhats()):
                hat = joystick.get_hat(i)
                axes.append(float(hat[0]))
                axes.append(float(hat[1]))
                print(str(i + joystick.get_numaxes()) + ": " + str(joystick.get_hat(i)))

            joy = Joy()
            joy.axes = axes
            joy.buttons = buttons
            self.publisher_.publish(joy)

            clock.tick(100)  # And this line

def main(args=None):
    rclpy.init(args=args)

    joystick_node = JoystickNode()
    rclpy.spin(joystick_node)

    # Destroy the node explicitly
    joystick_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
