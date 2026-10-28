"""
Trayectorias.py

Ejecución de trayectorias Joint, lineal y circular del UR3e.
"""

from math import cos, sin, pi
from copy import deepcopy
from typing import Callable, Dict

import rclpy
from std_msgs.msg import String

from practica_1.control_robot import ControlRobot

# Configuración de las trayectorias. 
# Posición inicial, en radianes.
POSICION_INICIAL: list[float] = [0.0, -pi / 2, -pi / 2, -pi / 2, pi / 2, 0.0]

# Destino del destino, en radianes.
POSICION_JOINT_DESTINO: list[float] = [pi / 4, -pi / 2, -pi / 2, -pi / 2, pi / 2, 0.0]

# Desplazamiento lineal de la trayectoria, en metros
DESPLAZAMIENTO_LINEAL: float = 0.1

# Parámetros de la trayectoria circular
# Radio, en metros
RADIO_CIRCULO: float = 0.05
PUNTOS_CIRCULO: int = 24

# Comunicación con el nodo recolector
TOPIC_TRAYECTORIA_ACTIVA: str = "/practica_1/trayectoria_activa"
NOMBRE_JOINT: str = "joint"
NOMBRE_LINEAL: str = "lineal"
NOMBRE_CIRCULAR: str = "circular"

# Indicación de que no hay ninguna trayectoria en curso
SIN_TRAYECTORIA: str = ""

# Guarda en cola antes de descartar los más antiguos
TAMANO_COLA_PUBLICADOR: int = 10


class EjecutorTrayectorias:
    """Ejecuta las trayectorias del robot."""

    def __init__(self) -> None:
        """Inicializa el controlador y el publicador de trayectorias."""

        self.control = ControlRobot()
        self.publicador = self.control.create_publisher(String, TOPIC_TRAYECTORIA_ACTIVA, TAMANO_COLA_PUBLICADOR)

    def trayectoria_joint(self) -> bool:
        """Ejecuta un movimiento articular hasta el objetivo.

        Returns:
            True si el movimiento se completa correctamente.
        """
        return self.control.mover_articulaciones(POSICION_JOINT_DESTINO)

    def trayectoria_lineal(self) -> bool:
        """Desplaza el brazo en línea recta a lo largo del eje X.

        Returns:
            True si el movimiento se completó con éxito.
        """

        pose_destino = self.control.pose_actual()
        pose_destino.position.x += DESPLAZAMIENTO_LINEAL
        return self.control.mover_trayectoria([pose_destino])

    def trayectoria_circular(self) -> bool:
        """Mueve el brazo en línea recta desde la pose actual

        Returns:
            True si el movimiento se completó con éxito.
        """

        pose_centro = self.control.pose_actual()
        poses = []
        for indice in range(PUNTOS_CIRCULO + 1):
            angulo = 2 * pi * indice / PUNTOS_CIRCULO
            pose = deepcopy(pose_centro)
            pose.position.x += RADIO_CIRCULO * (cos(angulo) - 1)
            pose.position.y += RADIO_CIRCULO * sin(angulo)
            poses.append(pose)
        return self.control.mover_trayectoria(poses)

    def _anunciar(self, nombre: str) -> None:
        """Publica el identificador de la trayectoria.
        
        Args:
            nombre: Nombre del mensaje
        """
        mensaje = String()
        mensaje.data = nombre
        self.publicador.publish(mensaje)

    def _ejecutar_anunciando(self, nombre: str, trayectoria: Callable[[], bool]) -> bool:
        """Prepara, anuncia y ejecuta una trayectoria.

        Args:
            nombre: Nombre del nodo recolector.
            trayectoria: Método que mueve el robot y devuelve si tuvo éxito.

        Returns:
            True si la trayectoria se completó con éxito.
        """

        if not self._ir_a_posicion_inicial():
            return False
        self._anunciar(nombre)
        exito = trayectoria()
        self._anunciar(SIN_TRAYECTORIA)
        return exito

    def ejecutar_todas(self) -> Dict[str, bool]:
        """Ejecuta las tres trayectorias, una tras otra.

        Returns:
            Diccionario con el nombre de cada trayectoria y si tuvo éxito.
        """
        return {
            NOMBRE_JOINT: self._ejecutar_anunciando(NOMBRE_JOINT, self.trayectoria_joint),
            NOMBRE_LINEAL: self._ejecutar_anunciando(NOMBRE_LINEAL, self.trayectoria_lineal),
            NOMBRE_CIRCULAR: self._ejecutar_anunciando(NOMBRE_CIRCULAR, self.trayectoria_circular),
        }

    def _ir_a_posicion_inicial(self) -> bool:
        """Coloca el robot en la configuración articular de partida."""
        return self.control.mover_articulaciones(POSICION_INICIAL)

    def cerrar(self) -> None:
        """Libera los recursos de ROS 2."""
        self.control.ejecutor.shutdown()
        self.control.destroy_node()


def main() -> None:
    """Ejecuta las tres trayectorias y muestra si cada una tuvo éxito."""
    rclpy.init()
    ejecutor = EjecutorTrayectorias()
    print(ejecutor.ejecutar_todas())
    ejecutor.cerrar()
    rclpy.shutdown()


if __name__ == '__main__':
    main()