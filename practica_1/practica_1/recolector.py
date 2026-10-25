import time
from typing import Dict, List, Optional, Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from practica_1.cinematica import CalculadorCinematica, EstadoCinematico
from practica_1.guardado_csv import GuardadorCsv, MuestraCinematica
from practica_1.trayectorias import SIN_TRAYECTORIA, TOPIC_TRAYECTORIA_ACTIVA

NODO: str = "recoleccion_datos"
ESTADO_ARTICULACIONES: str = "/joint_states"
ARTICULACIONES: List[str] = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TAMANNO_COLA: int = 500
INTERVALO_SEG_LOG: float = 1.0
NANOSEGUNDOS_AL_SEGUNDO: float = 1e9


class RecolectarDatos(Node):

    def __init__(self) -> None:
        super().__init__(NODO)
        self._calculador = CalculadorCinematica()
        self._guardador = GuardadorCsv()
        self._trayectoria_activa: str = SIN_TRAYECTORIA
        self._tiempo_inicio: Optional[float] = None
        self._muestras: List[MuestraCinematica] = []
        self._instante_ultimo_log: float = 0.0

        self.create_subscription(JointState, ESTADO_ARTICULACIONES, self._estado_recibido, TAMANNO_COLA)
        self.create_subscription(String, TOPIC_TRAYECTORIA_ACTIVA, self._trayectoria_recibida, TAMANNO_COLA)
        self.get_logger().info(f"Escuchando {ESTADO_ARTICULACIONES} y {TOPIC_TRAYECTORIA_ACTIVA}...")

    def _trayectoria_recibida(self, mensaje: String) -> None:
        self.guardar_trayectoria_activa()
        if mensaje.data != SIN_TRAYECTORIA:
            self._empezar_trayectoria(mensaje.data)

    def _empezar_trayectoria(self, nombre: str) -> None:
        self._trayectoria_activa = nombre
        self._tiempo_inicio = None
        self._muestras = []
        self.get_logger().info(f"Empieza la trayectoria '{nombre}'")

    def guardar_trayectoria_activa(self) -> None:
        if self._trayectoria_activa == SIN_TRAYECTORIA:
            return
        if self._muestras:
            ruta = self._guardador.guardar(self._trayectoria_activa, self._muestras)
            self.get_logger().info(f"Trayectoria '{self._trayectoria_activa}': "
                                   f"{len(self._muestras)} muestras guardadas en {ruta}")
        else:
            self.get_logger().warning(f"Trayectoria '{self._trayectoria_activa}' sin muestras")
        self._trayectoria_activa = SIN_TRAYECTORIA
        self._muestras = []

    def _estado_recibido(self, mensaje: JointState) -> None:
        posiciones = self._valores_por_articulacion(mensaje.name, mensaje.position)
        if posiciones is None:
            return  # El mensaje no contiene las articulaciones del brazo
        tiempo = self._tiempo_a_segundos(mensaje)

        estados = self._calculador.actualizar(tiempo, posiciones)
        if estados is None:
            return

        if self._trayectoria_activa != SIN_TRAYECTORIA:
            self._annadir_muestras(tiempo, estados)

        self._mostrar(tiempo, estados, mensaje)

    def _annadir_muestras(self, tiempo: float, estados: Dict[str, EstadoCinematico]) -> None:
        if self._tiempo_inicio is None:
            self._tiempo_inicio = tiempo
        tiempo_relativo = tiempo - self._tiempo_inicio
        for articulacion, estado in estados.items():
            self._muestras.append(MuestraCinematica(
                trayectoria=self._trayectoria_activa, tiempo=tiempo_relativo,
                articulacion=articulacion, posicion=estado.posicion,
                velocidad=estado.velocidad, aceleracion=estado.aceleracion))

    def _mostrar(self, tiempo: float, estados: Dict[str, EstadoCinematico], mensaje: JointState) -> None:
        ahora = time.monotonic()
        if ahora - self._instante_ultimo_log < INTERVALO_SEG_LOG:
            return
        self._instante_ultimo_log = ahora

        def formatear(valores: Sequence[float]) -> str:
            return ", ".join(f"{valor: 8.3f}" for valor in valores)

        trayectoria = self._trayectoria_activa or "ninguna"
        texto = (f"tiempo:{tiempo: .3f}s | trayectoria: {trayectoria}\n"
                 f"  posiciones (rad):       {formatear([e.posicion for e in estados.values()])}\n"
                 f"  velocidades (rad/s):    {formatear([e.velocidad for e in estados.values()])}\n"
                 f"  aceleraciones (rad/s²): {formatear([e.aceleracion for e in estados.values()])}")
        velocidades_mensaje = self._valores_por_articulacion(mensaje.name, mensaje.velocity)
        if velocidades_mensaje is not None:
            texto += f"\n  vel. mensaje (rad/s): {formatear(list(velocidades_mensaje.values()))}"
        self.get_logger().info(texto)

    @staticmethod
    def _tiempo_a_segundos(mensaje: JointState) -> float:
        tiempo = mensaje.header.stamp
        return tiempo.sec + tiempo.nanosec / NANOSEGUNDOS_AL_SEGUNDO

    @staticmethod
    def _valores_por_articulacion(nombres: Sequence[str], valores: Sequence[float]) -> Optional[Dict[str, float]]:
        valor = dict(zip(nombres, valores))
        if not all(articulacion in valor for articulacion in ARTICULACIONES):
            return None
        return {articulacion: valor[articulacion] for articulacion in ARTICULACIONES}


def main() -> None:
    rclpy.init()
    nodo = RecolectarDatos()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.guardar_trayectoria_activa()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()