"""Nodo recolector de datos cinemáticos del UR3e (Práctica 1)."""

import time
from typing import Dict, List, Optional, Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from practica_1.cinematica import CalculadorCinematica, EstadoCinematico
from practica_1.detector_movimiento import DetectorMovimiento
from practica_1.guardado_csv import GuardadorCsv, MuestraCinematica
from practica_1.trayectorias import SIN_TRAYECTORIA, TOPIC_TRAYECTORIA_ACTIVA
from practica_1_mensajes.msg import EstadoCinematico as MensajeEstadoCinematico

NODO: str = "recoleccion_datos"
ESTADO_ARTICULACIONES: str = "/joint_states"
# Topic donde se publica el estado cinemático calculado (mensaje propio)
TOPIC_ESTADO_CINEMATICO: str = "/practica_1/estado_cinematico"
# Articulaciones del brazo; se ignoran otras (por ejemplo, las de la pinza RG2,
# que publica en el mismo topic desde rg2_control_node)
ARTICULACIONES: List[str] = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
# Mensajes que se guardan en cola antes de descartar los más antiguos. El brazo publica
# a ~480 Hz: con 500 mensajes se absorben parones del nodo de hasta ~1 s sin perder datos
TAMANNO_COLA: int = 500
# Intervalo mínimo entre mensajes de log (s)
INTERVALO_SEG_LOG: float = 1.0
NANOSEGUNDOS_AL_SEGUNDO: float = 1e9


class RecolectarDatos(Node):
    """Nodo que calcula el estado cinemático de las articulaciones y lo guarda por trayectoria.

    Dentro de cada trayectoria solo guarda los instantes en los que el robot se mueve
    (ingesta inteligente), usando DetectorMovimiento. Además, publica el estado calculado
    en cada instante con un mensaje propio (EstadoCinematico).
    """

    def __init__(self) -> None:
        """Crea el nodo, el calculador, el detector, el guardador, las suscripciones y el publicador."""
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
        self._publicador_estado = self.create_publisher(MensajeEstadoCinematico,
                                                        TOPIC_ESTADO_CINEMATICO, TAMANNO_COLA)
        self.get_logger().info(f"Escuchando {ESTADO_ARTICULACIONES} y {TOPIC_TRAYECTORIA_ACTIVA}, "
                               f"publicando en {TOPIC_ESTADO_CINEMATICO}...")

    def _trayectoria_recibida(self, mensaje: String) -> None:
        """Gestiona el inicio y el fin de cada trayectoria.

        Un nombre ("joint", "lineal", "circular") indica que empieza una trayectoria,
        y un texto vacío, que ha terminado.

        Argumentos
            mensaje: nombre de la trayectoria activa, o texto vacío."""
        # Si había una trayectoria en curso, se cierra. Así también se guarda si
        # llegara un nombre nuevo sin haber recibido antes el texto vacío.
        self.guardar_trayectoria_activa()
        if mensaje.data != SIN_TRAYECTORIA:
            self._empezar_trayectoria(mensaje.data)

    def _empezar_trayectoria(self, nombre: str) -> None:
        """Prepara la recogida de datos de una trayectoria nueva.

        El calculador no se reinicia: antes de cada trayectoria el robot está parado en
        la posición inicial, así que derivar sobre muestras previas al inicio es correcto
        y permite tener velocidad y aceleración desde el primer instante.

        Argumentos
            nombre: identificador de la trayectoria."""
        self._trayectoria_activa = nombre
        self._tiempo_inicio = None
        self._muestras = []
        self._instantes_en_reposo = 0
        self.get_logger().info(f"Empieza la trayectoria '{nombre}'")

    def guardar_trayectoria_activa(self) -> None:
        """Guarda en CSV las muestras de la trayectoria en curso, si la hay, y la da por cerrada."""
        if self._trayectoria_activa == SIN_TRAYECTORIA:
            return
        if self._muestras:
            ruta = self._guardador.guardar(self._trayectoria_activa, self._muestras)
            self.get_logger().info(f"Trayectoria '{self._trayectoria_activa}': "
                                   f"{len(self._muestras)} muestras guardadas en {ruta} "
                                   f"({self._instantes_en_reposo} instantes en reposo descartados)")
        else:
            self.get_logger().warning(f"Trayectoria '{self._trayectoria_activa}' sin muestras")
        self._trayectoria_activa = SIN_TRAYECTORIA
        self._muestras = []

    def _estado_recibido(self, mensaje: JointState) -> None:
        """Procesa el mensaje que recibe de '/joint_states' y calcula el estado cinemático.

        Se ejecuta unas 480 veces por segundo, así que solo hace el trabajo imprescindible.

        Argumentos
            mensaje: estado de las articulaciones."""
        posiciones = self._valores_por_articulacion(mensaje.name, mensaje.position)
        if posiciones is None:
            return  # El mensaje no contiene las articulaciones del brazo
        tiempo = self._tiempo_a_segundos(mensaje)

        estados = self._calculador.actualizar(tiempo, posiciones)
        if estados is None:
            return  # Aún no hay muestras suficientes para derivar

        self._publicar_estado(mensaje, estados)
        if self._trayectoria_activa != SIN_TRAYECTORIA:
            self._registrar_si_hay_movimiento(tiempo, estados)

        self._mostrar(tiempo, estados, mensaje)

    def _publicar_estado(self, mensaje: JointState,
                         estados: Dict[str, EstadoCinematico]) -> None:
        """Publica el estado cinemático del instante con el mensaje propio EstadoCinematico.

        Se reutiliza la cabecera del mensaje de /joint_states, para que la marca de tiempo
        publicada sea la del instante medido y no la del momento del cálculo.

        Argumentos
            mensaje: mensaje original de /joint_states.
            estados: posición, velocidad y aceleración de cada articulación."""
        salida = MensajeEstadoCinematico()
        salida.header = mensaje.header
        salida.trayectoria = self._trayectoria_activa
        salida.articulaciones = list(estados.keys())
        salida.posiciones = [estado.posicion for estado in estados.values()]
        salida.velocidades = [estado.velocidad for estado in estados.values()]
        salida.aceleraciones = [estado.aceleracion for estado in estados.values()]
        self._publicador_estado.publish(salida)

    def _registrar_si_hay_movimiento(self, tiempo: float,
                                     estados: Dict[str, EstadoCinematico]) -> None:
        """Ingesta inteligente: guarda el instante solo si el robot se está moviendo.

        Así se descartan los periodos estáticos dentro de la trayectoria, como el tiempo
        que tarda MoveIt en planificar antes de arrancar o el reposo final.

        Argumentos
            tiempo: marca de tiempo de la muestra (s).
            estados: posición, velocidad y aceleración de cada articulación."""
        velocidades = [estado.velocidad for estado in estados.values()]
        if self._detector.esta_en_movimiento(velocidades):
            self._annadir_muestras(tiempo, estados)
        else:
            self._instantes_en_reposo += 1

    def _annadir_muestras(self, tiempo: float, estados: Dict[str, EstadoCinematico]) -> None:
        """Añade una muestra por articulación a la trayectoria en curso.

        El tiempo se guarda relativo a la primera muestra guardada (el inicio del movimiento),
        para que las gráficas empiecen en 0 en lugar de en la fecha actual en segundos.

        Argumentos
            tiempo: marca de tiempo de la muestra (s).
            estados: posición, velocidad y aceleración de cada articulación."""
        if self._tiempo_inicio is None:
            self._tiempo_inicio = tiempo
        tiempo_relativo = tiempo - self._tiempo_inicio
        for articulacion, estado in estados.items():
            self._muestras.append(MuestraCinematica(
                trayectoria=self._trayectoria_activa, tiempo=tiempo_relativo,
                articulacion=articulacion, posicion=estado.posicion,
                velocidad=estado.velocidad, aceleracion=estado.aceleracion))

    def _mostrar(self, tiempo: float, estados: Dict[str, EstadoCinematico],
                 mensaje: JointState) -> None:
        """Muestra por pantalla, como máximo una vez por intervalo, el estado calculado.

        Comprueba el intervalo antes de construir el texto: el logger de rclpy hace trabajo
        en cada llamada aunque esté limitado, y a ~480 Hz eso hacía perder mensajes.

        Argumentos
            tiempo: marca de tiempo de la muestra (s).
            estados: posición, velocidad y aceleración de cada articulación.
            mensaje: mensaje original, para comparar con la velocidad que publica el robot."""
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
        # Solo para comparar: la velocidad que publica el robot (puede venir vacía)
        velocidades_mensaje = self._valores_por_articulacion(mensaje.name, mensaje.velocity)
        if velocidades_mensaje is not None:
            texto += f"\n  vel. mensaje (rad/s):   {formatear(list(velocidades_mensaje.values()))}"
        self.get_logger().info(texto)

    @staticmethod
    def _tiempo_a_segundos(mensaje: JointState) -> float:
        """Junta el tiempo de segundos y nanosegundos y los convierte en segundos."""
        tiempo = mensaje.header.stamp
        return tiempo.sec + tiempo.nanosec / NANOSEGUNDOS_AL_SEGUNDO

    @staticmethod
    def _valores_por_articulacion(nombres: Sequence[str],
                                  valores: Sequence[float]) -> Optional[Dict[str, float]]:
        """Asocia cada articulación del brazo con su valor. Se emparejan por nombre,
        porque el orden de los nombres en el mensaje no está garantizado.

        Argumentos
            nombres: nombres de las articulaciones del mensaje.
            valores: valor de cada articulación (posición o velocidad), en el mismo orden.

        Devuelve
            Diccionario[articulacion, valor] en el orden de ARTICULACIONES, o None si falta
            alguna (mensaje de la pinza, o campo vacío)."""
        valor = dict(zip(nombres, valores))
        if not all(articulacion in valor for articulacion in ARTICULACIONES):
            return None
        return {articulacion: valor[articulacion] for articulacion in ARTICULACIONES}


def main() -> None:
    """Inicio del programa: crea el nodo y lo mantiene escuchando hasta Ctrl+C."""
    rclpy.init()
    nodo = RecolectarDatos()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Cierre normal con Ctrl+C
    finally:
        # Por si el aviso de fin de la última trayectoria no llegó antes de cerrar
        nodo.guardar_trayectoria_activa()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()