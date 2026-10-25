from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple

VENTANA_DERIVADA_S: float = 0.1


@dataclass
class EstadoCinematico:
    """Posición (rad), velocidad (rad/s) y aceleración (rad/s²) de una articulación."""

    posicion: float
    velocidad: float
    aceleracion: float


class DerivadorVentana:

    def __init__(self, ventana_s: float = VENTANA_DERIVADA_S) -> None:
        self.ventana_s = ventana_s
        self.reiniciar()

    def reiniciar(self) -> None:
        self._historial: Deque[Tuple[float, Dict[str, float]]] = deque()

    def derivar(self, tiempo: float, valores: Dict[str, float]) -> Optional[Dict[str, float]]:
        if self._historial and tiempo <= self._historial[-1][0]:
            return None 
        self._historial.append((tiempo, dict(valores)))

        while len(self._historial) >= 2 and tiempo - self._historial[1][0] >= self.ventana_s:
            self._historial.popleft()

        tiempo_referencia, valores_referencia = self._historial[0]
        intervalo = tiempo - tiempo_referencia
        if intervalo < self.ventana_s:
            return None
        return {nombre: (valor - valores_referencia[nombre]) / intervalo
                for nombre, valor in valores.items()}


class CalculadorCinematica:

    def __init__(self, ventana_s: float = VENTANA_DERIVADA_S) -> None:
        self._derivador_velocidad = DerivadorVentana(ventana_s)
        self._derivador_aceleracion = DerivadorVentana(ventana_s)

    def reiniciar(self) -> None:
        self._derivador_velocidad.reiniciar()
        self._derivador_aceleracion.reiniciar()

    def actualizar(self, tiempo: float, posiciones: Dict[str, float]) -> Optional[Dict[str, EstadoCinematico]]:
        velocidades = self._derivador_velocidad.derivar(tiempo, posiciones)
        if velocidades is None:
            return None
        aceleraciones = self._derivador_aceleracion.derivar(tiempo, velocidades)
        if aceleraciones is None:
            return None
        return {
            articulacion: EstadoCinematico(
                posicion=posiciones[articulacion],
                velocidad=velocidades[articulacion],
                aceleracion=aceleraciones[articulacion],
            )
            for articulacion in posiciones
        }


if __name__ == '__main__':
    PASO_PRUEBA_S = 0.0025
    MUESTRAS_PRUEBA = 81
    MOSTRAR_CADA = 20
    calculador = CalculadorCinematica()
    for indice in range(MUESTRAS_PRUEBA):
        tiempo_prueba = indice * PASO_PRUEBA_S
        estado = calculador.actualizar(tiempo_prueba, {"prueba": tiempo_prueba ** 2})
        if indice % MOSTRAR_CADA == 0:
            print(f"t={tiempo_prueba:.3f}", estado)