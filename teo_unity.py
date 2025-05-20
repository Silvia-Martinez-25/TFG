import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from yarp_control_msgs.msg import PositionDirect
import time

class TEO_Unity(Node):
    def __init__(self):
        super().__init__('teo_Unity')

        # Mapeo de extremidades a joints
        self.joint_groups = {
            "leftArm": [
                "FrontalLeftShoulder",
                "SagittalLeftShoulder",
                "AxialLeftShoulder",
                "FrontalLeftElbow",
                "AxialLeftWrist",
                "FrontalLeftWrist"
            ],
            "rightArm": [
                "FrontalRightShoulder",
                "SagittalRightShoulder",
                "AxialRightShoulder",
                "FrontalRightElbow",
                "AxialRightWrist",
                "FrontalRightWrist"
            ],
            "leftLeg": [
                "AxialLeftHip",
                "SagittalLeftHip",
                "FrontalLeftHip",
                "FrontalLeftKnee",
                "FrontalLeftAnkle",
                "SagittalLeftAnkle"
            ],
            "rightLeg": [
                "AxialRightHip",
                "SagittalRightHip",
                "FrontalRightHip",
                "FrontalRightKnee",
                "FrontalRightAnkle",
                "SagittalRightAnkle"
            ],
            "head": [
                "AxialNeck",
                "FrontalNeck"
            ],
            "trunk": [
                "AxialTrunk",
                "FrontalTrunk"
            ]
        }

        # Definir los enlaces finales para cada extremidad
        self.link_targets = {
            "leftArm":  "FrontalLeftWrist", 
            "rightArm": "FrontalRightWrist",
            "leftLeg":  "SagittalLeftAnkle", 
            "rightLeg": "SagittalRightAnkle"
        }

        # Frame de referencia
        self.reference_frame = "waist"

        # Suscripción a los estados de las articulaciones
        self.current_joint_positions = {}
        self.current_poses = {}

        self.joint_subscriptions = {}
        self.pose_subscriptions = {}
        self.command_publishers = {}

        for group, joints in self.joint_groups.items():
            self.current_joint_positions[group] = {}
            
            self.joint_subscriptions[group] = self.create_subscription(
                JointState,
                f'/teoUnity/{group}/state', 
                lambda msg, g=group: self.joint_state_callback(msg, g),
                10)

            self.get_logger().info(f'/teoUnity/{group}/state')

            self.pose_subscriptions[group] = self.create_subscription(
                Pose,
                f'/teoUnity/{group}/pose', 
                lambda msg, g=group: self.pose_callback(msg, g),
                10)
            
            self.get_logger().info(f'/teoUnity/{group}/pose')

            self.command_publishers[group] = self.create_publisher(
                PositionDirect,
                f'/teoUnity/{group}/position_direct', 
                10)
            
            self.get_logger().info(f'/teoUnity/{group}/position_direct')

    def joint_state_callback(self, msg, group):
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_positions[group][name] = pos

    def pose_callback(self ,msg, group):
        self.current_poses[group] = msg

    def print_joint_states(self):
        self.get_logger().info(" ESTADO DE ARTICULACIONES POR EXTREMIDAD ")
        for group, joints in self.joint_groups.items():
            self.get_logger().info(f"[{group}]")
            for j in joints:
                angle = self.current_joint_positions[group].get(j)
                if angle is not None:
                    self.get_logger().info(f"  {j}: {angle:.3f} rad")
                else:
                    self.get_logger().warn(f"  {j}: no disponible")

    def print_link_poses(self):
        self.get_logger().info(" POSES POR EXTREMIDAD ")
        for group, p in self.link_targets.items():
            self.get_logger().info(f"[{group}]")
            pose = self.current_poses[group]
            if pose:
                pos = pose.position
                rot = pose.orientation
                self.get_logger().info(f"  Pos: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
                self.get_logger().info(f"  Ori: x={rot.x:.2f}, y={rot.y:.2f}, z={rot.z:.2f}, w={rot.w:.2f}")

    def send_joint_command(self, group, positions):
        msg = PositionDirect()
        msg.names = list(positions.keys())
        msg.positions = list(positions.values())
        self.command_publishers[group].publish(msg)
        self.get_logger().info(f"Enviado comando a {group}:{positions}")

def main(args=None):
    rclpy.init(args=args)
    node = TEO_Unity()

    try:
        start = time.time()

        while time.time() - start < 2:
            # Esperar datos iniciales
            rclpy.spin_once(node)
            time.sleep(0.01)
        
        # Imprimir los estados de las articulaciones
        node.print_joint_states()
        node.print_link_poses()

        positions = {}
        positions['AxialNeck'] = -0.4;
        positions['FrontalNeck'] = -0.4;
        node.send_joint_command('head', positions)
       
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

