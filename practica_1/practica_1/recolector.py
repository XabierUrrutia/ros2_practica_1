"""
Recolector.py

Recoge datos cinemáticos del robot. 
Calcula la posición, velocidad y aceleración de las articulaciones,
publica el estado cinemático y guarda muestras .csv durante el movimiento.
"""

import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from practica_1.cinematica import CalculadorCinematica, EstadoCinematico
from practica_1.detector_movimiento import DetectorMovimiento
from practica_1.guardado_cinematico import GuardadorCsv, MuestraCinematica
from practica_1.trayectorias import SIN_TRAYECTORIA, TOPIC_TRAYECTORIA_ACTIVA
from practica_1_mensajes.msg import EstadoCinematico as MensajeEstadoCinematico

# Configuración del nodo
NODO: str = "recoleccion_datos"
ESTADO_ARTICULACIONES: str = "/joint_states"
TOPIC_ESTADO_CINEMATICO: str = "/practica_1/estado_cinematico"

# Articulaciones del brazo, se ignoran las de la pinza
ARTICULACIONES: Tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# Capacidad de las colas de suscripción y publicación.
# A 480 Hz, 500 mensajes representan aproximadamente un segundo de datos.
TAMANNO_COLA: int = 500

# Intervalo mínimo entre mensajes, en segundos
INTERVALO_SEG_LOG: float = 1.0

# Conversión de nanosegundos a segundos.
NANOSEGUNDOS_AL_SEGUNDO: float = 1e9

TEXTO_SIN_TRAYECTORIA: str = "ninguna"


class RecolectarDatos(Node):
    """Nodo que calcula el estado cinemático de las articulaciones y lo guarda por trayectoria."""

    def __init__(self) -> None:
        """Inicializa los componentes y las comunicaciones ROS 2."""
        
        super().__init__(NODO)
        self._calculador = CalculadorCinematica()
        self._detector = DetectorMovimiento()
        self._guardador = GuardadorCsv()
        self._trayectoria_activa: str = SIN_TRAYECTORIA
        self._tiempo_inicio: Optional[float] = None
        self._muestras: List[MuestraCinematica] = []
        self._instantes_en_reposo: int = 0
        self._instante_ultimo_log: float = 0.0

        self.create_subscription(JointState, ESTADO_ARTICULACIONES, self._estado_recibido, TAMANNO_COLA)
        self.create_subscription(String, TOPIC_TRAYECTORIA_ACTIVA, self._trayectoria_recibida, TAMANNO_COLA)
        self._publicador_estado = self.create_publisher(MensajeEstadoCinematico, TOPIC_ESTADO_CINEMATICO, TAMANNO_COLA)
        
    def _trayectoria_recibida(self, mensaje: String) -> None:
        """Gestiona el inicio y el fin de cada trayectoria.

        Args:
            mensaje: Nombre de la trayectoria.
        """

        self.guardar_trayectoria_activa()
        if mensaje.data != SIN_TRAYECTORIA:
            self._empezar_trayectoria(mensaje.data)

    def _empezar_trayectoria(self, nombre: str) -> None:
        """Inicia el estado de recogida de una trayectoria.

        Args:
            nombre: Nombre de la trayectoria.
        """

        self._trayectoria_activa = nombre
        self._tiempo_inicio = None
        self._muestras = []
        self._instantes_en_reposo = 0

    def guardar_trayectoria_activa(self) -> None:
        """Guarda las trayectorias acumuladas."""
        
        if self._muestras:
            self._guardador.guardar(self._trayectoria_activa, self._muestras)
        
        self._trayectoria_activa = SIN_TRAYECTORIA
        self._muestras = []

    def _estado_recibido(self, mensaje: JointState) -> None:
        """Procesa un mensaje recibido.

        Args:
            mensaje: Estado de las articulaciones.
        """

        posiciones = self._valores_por_articulacion(mensaje.name, mensaje.position)
        if posiciones is None:
            return
        tiempo = self._tiempo_a_segundos(mensaje)
        estados = self._calculador.actualizar(tiempo, posiciones)
        if estados is None:
            return
        self._publicar_estado(mensaje, estados)
        
        if self._trayectoria_activa != SIN_TRAYECTORIA:
            self._registrar_si_hay_movimiento(tiempo, estados)

        self._mostrar(tiempo, estados, mensaje)

    def _publicar_estado(self, mensaje: JointState, estados: Dict[str, EstadoCinematico]) -> None:
        """Publica el estado cinemático calculado con la marca original.

        Args:
            mensaje: Mensaje de /joint_states.
            estados: Posición, velocidad y aceleración de cada articulación.
        """

        salida = MensajeEstadoCinematico()
        salida.header = mensaje.header
        salida.trayectoria = self._trayectoria_activa
        salida.articulaciones = list(estados.keys())
        salida.posiciones = [estado.posicion for estado in estados.values()]
        salida.velocidades = [estado.velocidad for estado in estados.values()]
        salida.aceleraciones = [estado.aceleracion for estado in estados.values()]
        self._publicador_estado.publish(salida)

    def _registrar_si_hay_movimiento(self, tiempo: float, estados: Dict[str, EstadoCinematico]) -> None:
        """Registra una muestra solo si se detecta movimiento del robot.

        Argumentos
            tiempo: Tiempo, en segundos.
            estados: Posición, velocidad y aceleración de cada articulación.
        """

        velocidades = [estado.velocidad for estado in estados.values()]
        if self._detector.esta_en_movimiento(velocidades):
            self._annadir_muestras(tiempo, estados)
        else:
            self._instantes_en_reposo += 1

    def _annadir_muestras(self, tiempo: float, estados: Dict[str, EstadoCinematico]) -> None:
        """Añade una muestra para cada articulación. 

        Argumentos
            tiempo: Tiempo, en segundo.
            estados: Posición, velocidad y aceleración de cada articulación.
        """

        if self._tiempo_inicio is None:
            self._tiempo_inicio = tiempo
        tiempo_relativo = tiempo - self._tiempo_inicio
        for articulacion, estado in estados.items():
            self._muestras.append(MuestraCinematica(
                trayectoria=self._trayectoria_activa, tiempo=tiempo_relativo,
                articulacion=articulacion, posicion=estado.posicion,
                velocidad=estado.velocidad, aceleracion=estado.aceleracion))

    def _mostrar(self, tiempo: float, estados: Dict[str, EstadoCinematico], mensaje: JointState) -> None:
        """Muestra información una vez por intervalo.

        Argumentos
            tiempo: Tiempo, en segundos.
            estados: Posición, velocidad y aceleración de cada articulación.
            mensaje: Mensaje, para comparar con la velocidad que publica el robot.
        """

        ahora = time.monotonic()
        if ahora - self._instante_ultimo_log < INTERVALO_SEG_LOG:
            return
        self._instante_ultimo_log = ahora

        posiciones = [estado.posicion for estado in estados.values()]
        velocidades = [estado.velocidad for estado in estados.values()]
        aceleraciones = [estado.aceleracion for estado in estados.values()]
        trayectoria = self._trayectoria_activa or TEXTO_SIN_TRAYECTORIA

        texto = (f"tiempo: {tiempo:.3f} s | trayectoria: {trayectoria}\n"
                 f"  posiciones (rad):       {self._formatear(posiciones)}\n"
                 f"  velocidades (rad/s):    {self._formatear(velocidades)}\n"
                 f"  aceleraciones (rad/s²): {self._formatear(aceleraciones)}")

        velocidades_mensaje = self._valores_por_articulacion(mensaje.name, mensaje.velocity)
        if velocidades_mensaje is not None:
            texto += (f"\n  vel. mensaje (rad/s):   "
                      f"{self._formatear(list(velocidades_mensaje.values()))}")
        self.get_logger().info(texto)

    @staticmethod
    def _formatear(valores: Sequence[float]) -> str:
        """Convierte una lista de valores en texto.

        Args:
            valores: valores numéricos de cada articulación.

        Returns:
            Valores separados por comas, con el formato FORMATO_VALOR_LOG.
        """
        return ", ".join(f"{valor: 8.3f}" for valor in valores)
   
    @staticmethod
    def _tiempo_a_segundos(mensaje: JointState) -> float:
        """Junta el tiempo de segundos y nanosegundos y los convierte en segundos.
        
        Args:
            mensaje: mensaje de /joint_states.

        Returns:
            Tiempo en segundos.
        """
        tiempo = mensaje.header.stamp
        return tiempo.sec + tiempo.nanosec / NANOSEGUNDOS_AL_SEGUNDO

    @staticmethod
    def _valores_por_articulacion(nombres: Sequence[str], valores: Sequence[float]) -> Optional[Dict[str, float]]:
        """Asocia los valores recibidos a las articulaciones por su nombre.

        Args:
            nombres: Nombres de las articulaciones.
            valores: Valor de cada articulación.

        Returns:
            Diccionario[articulacion, valor] en el orden de ARTICULACIONES o 
            None si falta alguna.
        """

        valor = dict(zip(nombres, valores))
        if not all(articulacion in valor for articulacion in ARTICULACIONES):
            return None
        return {articulacion: valor[articulacion] for articulacion in ARTICULACIONES}


def main() -> None:
    """Inicializa ROS 2, ejecuta el nodo y gestiona su cierre."""

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