from typing import Dict, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

NODO: str = "recoleccion_datos"
ESTADO_ARTICULACIONES: str = "/joint_states"
ARTICULACIONES: List[str] = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TAMANNO_COLA: int = 10
INTERVALO_SEG_LOG: float = 1.0
NANOSEGUNDOS_AL_SEGUNDO: float = 1e9


class RecolectarDatos(Node):
    def __init__(self) -> None:
        super().__init__(NODO)
        self.create_subscription(JointState, ESTADO_ARTICULACIONES, self._estado_recibido, TAMANNO_COLA)
        self.get_logger().info(f"{ESTADO_ARTICULACIONES}...")

    def _estado_recibido(self, mensaje: JointState) -> None:
        """Procesa el mensaje que recibe de '/joint_states'.
        
        Argumentos
            mensaje: estado de las articulaciones."""
        posiciones = self._extraer_posiciones(mensaje)
        if posiciones is None:
            return
        tiempo = self._tiempo_a_segundos(mensaje)
        posiciones_texto = ", ".join(f"{valor: .3f}" for valor in posiciones.values())
        self.get_logger().info(f"tiempo:{tiempo: .3f}s / posiciones: {posiciones_texto}", throttle_duration_sec = INTERVALO_SEG_LOG)

    @staticmethod
    def _tiempo_a_segundos(mensaje: JointState) -> float:
        """Junta el tiempo de segundos y nanosegundos y los convierte en segundos"""
        tiempo = mensaje.header.stamp
        return tiempo.sec + tiempo.nanosec / NANOSEGUNDOS_AL_SEGUNDO

    @staticmethod
    def _extraer_posiciones(mensaje: JointState) -> Optional[Dict[str, float]]:
        """Extrae las posiciones de las articulaciones del brazo. Se emparejan por nombre.

        Devuelve
            Diccionario[articulacion, posicion] o None"""
        posicion = dict(zip(mensaje.name, mensaje.position))
        if not all(articulacion in posicion for articulacion in ARTICULACIONES):
            return None
        return {articulacion: posicion[articulacion] for articulacion in ARTICULACIONES}

def main() -> None:
    """Inicio del programa"""
    rclpy.init()
    nodo = RecolectarDatos()
    try: 
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()

if __name__=='__main__':
    main()
