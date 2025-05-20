import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState		#para enviar o recibir las posiciones articulares
from yarp_control_msgs.msg import Position, PositionDirect
from yarp_control_msgs.srv import SetControlModes
import time
import argparse

EPSILON = 0.00175  # Aprox. 0.1 grados en radianes

class TEO_Gazebo_Unity(Node):
    def __init__(self, mode):
        super().__init__('teo_gazebo_unity')
        self.mode = mode

        if mode == 'unity_to_gazebo':
            self.prefix_in = 'teoUnity'
            self.prefix_out = 'teoSim'
        elif mode == 'gazebo_to_unity':
            self.prefix_in = 'teoSim'
            self.prefix_out = 'teoUnity'
        else:
            raise ValueError()

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

        #guarda las posiciones actuales
        self.latest_joint_states_in = {}
        self.latest_joint_states_out = {}

        self.joint_subscribers_in = {}
        self.joint_subscribers_out = {}

	    #publica las posiciones actuales
        self.command_publishers_pos = {}
        self.command_publishers_posd = {}

        for group, joints in self.joint_groups.items():
            self.joint_subscribers_in[group] = self.create_subscription(
                JointState,
                f'/{self.prefix_in}/{group}/state',
                lambda msg, g=group, states=self.latest_joint_states_in: self.joint_state_callback(msg, g, states),
                10)

            if mode == 'unity_to_gazebo':
                self.joint_subscribers_out[group] = self.create_subscription(
                    JointState,
                    f'/{self.prefix_out}/{group}/state',
                    lambda msg, g=group, states=self.latest_joint_states_out: self.joint_state_callback(msg, g, states),
                    10)

                self.command_publishers_pos[group] = self.create_publisher(
                    Position,
                    f'/{self.prefix_out}/{group}/position',
                    10)

            self.command_publishers_posd[group] = self.create_publisher(
                PositionDirect,
                f'/{self.prefix_out}/{group}/position_direct',
                10)
     
    def joint_state_callback(self, msg, group, latest_joint_states):
        for name, pos in zip(msg.name, msg.position):
            if group not in latest_joint_states:
                latest_joint_states[group] = {}
            latest_joint_states[group][name] = pos
        
    def send_joint_command_pos(self, group, joints):
        msg = Position()
        msg.names = joints.keys()
        msg.positions = joints.values()
        self.command_publishers_pos[group].publish(msg)

    def send_joint_command_posd(self, group, joints):
        msg = PositionDirect()
        msg.names = joints.keys()
        msg.positions = joints.values()
        self.command_publishers_posd[group].publish(msg)

    def set_control_mode(self, group, mode):
        client = self.create_client(SetControlModes, f"/teoSim/{group}/set_modes")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"No se encontró el servicio para {group}")
            return False

        request = SetControlModes.Request()
        request.modes = [mode] * len(self.joint_groups[group])

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f"[FALLO] {group} modo {mode}")
            return False

        self.get_logger().info(f"[OK] {group} modo {mode}")
        return True

    def joints_synchronized(self, expected):
        for group_out, joints_out in self.latest_joint_states_out.items():
            if not group_out in expected:
                return False

            joints_in = expected[group_out]

            for name, target in joints_out.items():
                if name not in joints_in:
                    return False
                if abs(joints_in[name] - target) > EPSILON:
                    return False
        return True

    def configure_and_sync(self):
        #  Poner en modo POSITION
        if self.mode == 'unity_to_gazebo':
            for group, joints in self.joint_groups.items():
                self.set_control_mode(group, 'POSITION')
        
        # Espera a recibir datos de Unity
        self.get_logger().info("Esperando estado articular...")
        while len(self.latest_joint_states_in) < len(self.joint_groups.items()):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Publicar la posicion   
        for group, joints in self.latest_joint_states_in.items():
            if self.mode == 'unity_to_gazebo':
                self.send_joint_command_pos(group, joints)
            else:
                self.send_joint_command_posd(group, joints)

        if self.mode == 'unity_to_gazebo':
            self.get_logger().info("Sincronizando articulaciones...")

            #se compara la posicion con la actual
            while not self.joints_synchronized(self.latest_joint_states_in):
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)

            self.get_logger().info("sincronizacion hecha")

            # Cambiar a modo POSITION_DIRECT
            for group, joints in self.joint_groups.items():
                self.set_control_mode(group, 'POSITION_DIRECT')

    def continuous_sync(self):
        self.get_logger().info("Iniciando sincronizacion continua...")
        while rclpy.ok():
            rclpy.spin_once(self)
            for group, joints in self.latest_joint_states_in.items():
                self.send_joint_command_posd(group, joints)
            time.sleep(0.02)

def main(args=None):
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices =[ 'unity_to_gazebo', 'gazebo_to_unity'], required=True)
    args = parser.parse_args()
    node = TEO_Gazebo_Unity(mode=args.mode)

    try:
        node.configure_and_sync()
        node.continuous_sync()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
