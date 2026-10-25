"""Ejecución de trayectorias Joint, lineal y circular del UR3e (Práctica 1)."""

from math import cos, sin, pi
from copy import deepcopy
from typing import Callable, Dict

import rclpy
from std_msgs.msg import String
from practica_1.control_robot import ControlRobot

# Postura de partida común a todas las trayectorias, para que sean repetibles
POSICION_INICIAL = [0.0, -pi / 2, -pi / 2, -pi / 2, pi / 2, 0.0]
# Destino de la trayectoria Joint: gira la base 45° respecto a la posición inicial
POSICION_JOINT_DESTINO = [pi / 4, -pi / 2, -pi / 2, -pi / 2, pi / 2, 0.0]
# Desplazamiento de la trayectoria lineal, en metros
DESPLAZAMIENTO_LINEAL = 0.1
# Parámetros de la trayectoria circular: radio en metros y puntos que la aproximan
RADIO_CIRCULO: float = 0.05
PUNTOS_CIRCULO: int = 24

# Contrato con el nodo recolector
TOPIC_TRAYECTORIA_ACTIVA: str = "/practica_1/trayectoria_activa"
NOMBRE_JOINT: str = "joint"
NOMBRE_LINEAL: str = "lineal"
NOMBRE_CIRCULAR: str = "circular"
# Mensaje que indica que no hay ninguna trayectoria en curso
SIN_TRAYECTORIA: str = ""
# Mensajes que se guardan en cola antes de descartar los más antiguos
TAMANO_COLA_PUBLICADOR: int = 10


class EjecutorTrayectorias:
    """Ejecuta las trayectorias del robot usando ControlRobot (MoveIt 2)."""

    def __init__(self) -> None:
        """Crea el ControlRobot, que espera a que MoveIt esté disponible."""
        self.control = ControlRobot()
        self.publicador = self.control.create_publisher(String, TOPIC_TRAYECTORIA_ACTIVA, TAMANO_COLA_PUBLICADOR)

    def trayectoria_joint(self) -> bool:
        """Mueve el robot en el espacio articular hasta la posición de destino.

        Requiere que el robot esté en POSICION_INICIAL.

        Returns:
            True si el movimiento se completó con éxito.
        """
        return self.control.mover_articulaciones(POSICION_JOINT_DESTINO)

    def trayectoria_lineal(self) -> bool:
        """Mueve el efector final en línea recta desde la pose actual.

        Requiere que el robot esté en POSICION_INICIAL.

        Returns:
            True si el movimiento se completó con éxito.
        """
        pose_destino = self.control.pose_actual()
        pose_destino.position.x += DESPLAZAMIENTO_LINEAL
        return self.control.mover_trayectoria([pose_destino])

    def trayectoria_circular(self) -> bool:
        """Dibuja una circunferencia con el efector final en el plano horizontal.

        Requiere que el robot esté en POSICION_INICIAL.

        Returns:
            True si el movimiento se completó con éxito.
        """
        pose_centro = self.control.pose_actual()
        poses = []
        for indice in range(PUNTOS_CIRCULO + 1):
            angulo = 2 * pi * indice / PUNTOS_CIRCULO
            pose = deepcopy(pose_centro)
            # Se resta RADIO en x para que el círculo pase por la pose actual
            pose.position.x += RADIO_CIRCULO * (cos(angulo) - 1)
            pose.position.y += RADIO_CIRCULO * sin(angulo)
            poses.append(pose)
        return self.control.mover_trayectoria(poses)

    def _anunciar(self, nombre: str) -> None:
        """Publica la trayectoria activa; un texto vacío indica que no hay ninguna."""
        mensaje = String()
        mensaje.data = nombre
        self.publicador.publish(mensaje)

    def _ejecutar_anunciando(self, nombre: str, trayectoria: Callable[[], bool]) -> bool:
        """Anuncia el inicio y el fin de una trayectoria mientras la ejecuta.

        Args:
            nombre: identificador que recibirá el nodo recolector.
            trayectoria: método que mueve el robot y devuelve si tuvo éxito.

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
        """Ejecuta las tres trayectorias, una tras otra, anunciando cada una.

        Returns:
            Diccionario con el nombre de cada trayectoria y si tuvo éxito.
        """
        return {
            NOMBRE_JOINT: self._ejecutar_anunciando(NOMBRE_JOINT, self.trayectoria_joint),
            NOMBRE_LINEAL: self._ejecutar_anunciando(NOMBRE_LINEAL, self.trayectoria_lineal),
            NOMBRE_CIRCULAR: self._ejecutar_anunciando(NOMBRE_CIRCULAR, self.trayectoria_circular),
        }

    def _ir_a_posicion_inicial(self) -> bool:
        """Lleva el robot a la posición de partida común a todas las trayectorias."""
        return self.control.mover_articulaciones(POSICION_INICIAL)

    def cerrar(self) -> None:
        """Libera los recursos de ROS 2 (ejecutor y nodo)."""
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