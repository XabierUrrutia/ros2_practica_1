"""
MonitorEstado.py

Muestra por pantalla el estado cinemático publicado por el recolector
con el mensaje personalizado EstadoCinematico.
"""

import time
from typing import Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from practica_1_mensajes.msg import EstadoCinematico

# Configuración del nodo
NODO: str = "monitor_estado"
TOPIC_ESTADO_CINEMATICO: str = "/practica_1/estado_cinematico"

# Capacidad de la cola de suscripción
TAMANNO_COLA: int = 10

# Intervalo mínimo entre mensajes mostrados, en segundos.
# El recolector publica a ~480 Hz: mostrar todos haría ilegible la terminal.
INTERVALO_SEG_LOG: float = 0.5

# Conversión de nanosegundos a segundos
NANOSEGUNDOS_AL_SEGUNDO: float = 1e9

# Texto que se muestra cuando no hay ninguna trayectoria en curso
TEXTO_SIN_TRAYECTORIA: str = "ninguna"

# Formato de cada valor numérico: signo, 8 caracteres y 3 decimales
FORMATO_VALOR: str = " 8.3f"
SEPARADOR_VALORES: str = ", "


class MonitorEstado(Node):
    """Nodo que se suscribe al mensaje EstadoCinematico y lo muestra por pantalla."""

    def __init__(self) -> None:
        """Inicializa el nodo y la suscripción al estado cinemático."""
        super().__init__(NODO)
        self._instante_ultimo_log: float = 0.0
        self.create_subscription(EstadoCinematico, TOPIC_ESTADO_CINEMATICO,
                                 self._estado_recibido, TAMANNO_COLA)
        self.get_logger().info(f"Escuchando {TOPIC_ESTADO_CINEMATICO}")

    def _estado_recibido(self, mensaje: EstadoCinematico) -> None:
        """Muestra el mensaje recibido, como máximo una vez por INTERVALO_SEG_LOG.

        Args:
            mensaje: Estado cinemático de las articulaciones.
        """
        ahora = time.monotonic()
        if ahora - self._instante_ultimo_log < INTERVALO_SEG_LOG:
            return
        self._instante_ultimo_log = ahora

        tiempo = mensaje.header.stamp.sec + mensaje.header.stamp.nanosec / NANOSEGUNDOS_AL_SEGUNDO
        trayectoria = mensaje.trayectoria or TEXTO_SIN_TRAYECTORIA

        texto = (f"[EstadoCinematico] tiempo: {tiempo:.3f} s | trayectoria: {trayectoria}\n"
                 f"  articulaciones:         {SEPARADOR_VALORES.join(mensaje.articulaciones)}\n"
                 f"  posiciones (rad):       {self._formatear(mensaje.posiciones)}\n"
                 f"  velocidades (rad/s):    {self._formatear(mensaje.velocidades)}\n"
                 f"  aceleraciones (rad/s²): {self._formatear(mensaje.aceleraciones)}")
        self.get_logger().info(texto)

    @staticmethod
    def _formatear(valores: Sequence[float]) -> str:
        """Convierte una lista de valores en texto.

        Args:
            valores: Valores numéricos de cada articulación.

        Returns:
            Valores separados por SEPARADOR_VALORES, con el formato FORMATO_VALOR.
        """
        return SEPARADOR_VALORES.join(f"{valor:{FORMATO_VALOR}}" for valor in valores)


def main() -> None:
    """Inicializa ROS 2, ejecuta el nodo y gestiona su cierre."""
    rclpy.init()
    nodo = MonitorEstado()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()