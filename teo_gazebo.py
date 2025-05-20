import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from yarp_control_msgs.msg import Position, PositionDirect
from yarp_control_msgs.srv import GetJointsNames, SetControlModes
import time

class TEO_Gazebo(Node):
    def __init__(self):
        super().__init__('teo_Gazebo')

        # Definir los enlaces finales de cada extremidad para publicar la info
        self.joint_groups = {
            "leftArm": [], 
            "rightArm": [],
            "leftLeg": [], 
            "rightLeg": [],
            "head": [],
            "trunk": []
        }

        # Frame de referencia
        self.reference_frame = "waist"

        #Grupos de articulaciones por extremidad
        self.current_joint_positions = {}
        self.joint_subscriptions = {}
        self.command_publishers_pos = {}
        self.command_publishers_posd = {}

        #LLama al servicio de los joints de cada grupo   
        self.get_joint_names_from_services()

        # Crear suscriptores y publicadores por cada grupo
                 
        for group, joints in self.joint_groups.items():
            self.current_joint_positions[group] = {}
            
            self.joint_subscriptions[group] = self.create_subscription(
                JointState,
                f'/teoSim/{group}/state', 
                lambda msg, g=group: self.joint_state_callback(msg, g),
                10)
            self.get_logger().info(f'/teoSim/{group}/state')

            self.command_publishers_pos[group] = self.create_publisher(
                Position,
                f'/teoSim/{group}/position', 
                10)
            self.get_logger().info(f'/teoSim/{group}/position')
                   
            self.command_publishers_posd[group] = self.create_publisher(
                PositionDirect,
                f'/teoSim/{group}/position_direct', 
                10)
            self.get_logger().info(f'/teoSim/{group}/position_direct')
    
    def get_joint_names_from_services(self):
        for group, link in self.joint_groups.items():
            service_name = f'/teoSim/{group}/get_joints_names'
            client = self.create_client(GetJointsNames, service_name)
            client.wait_for_service()

            request = GetJointsNames.Request()
            future = client.call_async(request)

            rclpy.spin_until_future_complete(self, future)

            if future.done() and future.result() is not None:
                joint_names = list(future.result().names)
                self.joint_groups[group] = joint_names
                self.get_logger().info(f"[{group} Articulaciones: {joint_names}]")
            else:
                self.get_logger().error(f"No se pudo obtener las articulaciones de {group}")

    def joint_state_callback(self, msg, group):
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_positions[group][name] = pos

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
   
    def send_joint_command_pos(self, group, positions):
        msg = Position()
        msg.names = self.joint_groups[group]
        msg.positions = positions
        self.command_publishers_pos[group].publish(msg)
        self.get_logger().info(f"Enviado comando a {group}:{positions}")

    def send_joint_command_posd(self, group, positions):
        msg = PositionDirect()
        msg.names = self.joint_groups[group]
        msg.positions = list(positions)
        self.command_publishers_posd[group].publish(msg)
        self.get_logger().info(f"Enviado comando a {group}:{positions}")

    def set_control_mode(self, group, mode):
        service_name = f'/teoSim/{group}/set_modes'
        client = self.create_client(SetControlModes, service_name)
        client.wait_for_service()

        request = SetControlModes.Request()
        request.modes = [mode] * len(self.joint_groups[group])
        request.names = self.joint_groups[group]

        future = client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"[{group} Modo cambiado a {mode}]")
        else:
            self.get_logger().error(f"[{group}] Error al cambiar de modo")

    def move_to_position(self, group, target_positions):
        self.set_control_mode(group, "POSITION")
        self.send_joint_command_pos(group, target_positions)
      
    def move_to_position_direct(self, group, target_positions):
        self.set_control_mode(group, "POSITION_DIRECT")
        self.send_joint_command_posd(group, target_positions)
   


#Inicia el nodo y entra en un bucle en el que el temporizador se ejecuta periódicamente.
def main(args=None):
    rclpy.init(args=args)
    node = TEO_Gazebo()

    try:
        start = time.time()

        while time.time() - start < 2:
            # Esperar datos iniciales
            rclpy.spin_once(node)
            time.sleep(0.01)

        node.print_joint_states()
        node.move_to_position('head', [-0.4, -0.4])
        time.sleep(10)
        node.move_to_position_direct('head', [0.0, 0.0])
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
