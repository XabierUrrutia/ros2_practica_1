from dataclasses import dataclass
from typing import Dict, Optional

INTERVALO_MINIMO: float = 1e-6

@dataclass
class EstadoCinematico:
    """Posicion, velocidad y aceleracion de una articulacion, en radianes"""
    posicion: float
    velocidad: float
    aceleracion: float

class CalculadorCinematica:
    """Calcula la velocidad"""
    
    def __init__(self) -> None:
        self.reiniciar()

    def reiniciar(self) -> None:
        self._tiempo_pasado: Optional[float] = None
        self._posicion_pasada: Dict[str, float] = {}
        self._velocidad_pasada: Dict[str, float] = {}

    def actualizar(self, tiempo: float, posiciones: Dict[str, float]) -> Optional[Dict[str, EstadoCinematico]]:
        if self._tiempo_pasado is None:
            self._guardar(tiempo, posiciones, {})
            return None

        intervalo = tiempo - self._tiempo_pasado
        if intervalo < INTERVALO_MINIMO:
            return None

        velocidades = {articulacion: (posicion - self._posiciones_anteriores[articulacion]) / intervalo for articulacion, posicion in posiciones.items()}

        if not self._velocidades_anteriores:
            self._guardar(tiempo, posiciones, velocidades)
            return None

        estados = {
            articulacion: EstadoCinematico(
                posicion=posiciones[articulacion],
                velocidad=velocidades[articulacion],
                aceleracion=(velocidades[articulacion] - self._velocidades_anteriores[articulacion]) / intervalo,
            )
            for articulacion in posiciones
        }
        self._guardar(tiempo, posiciones, velocidades)
        return estados

    def _guardar(self, tiempo: float, posiciones: Dict[str, float], velocidades: Dict[str, float]) -> None:
        """Guarda la muestra actual como referencia para la siguiente."""
        self._tiempo_anterior = tiempo
        self._posiciones_anteriores = dict(posiciones)
        self._velocidades_anteriores = dict(velocidades)


if __name__ == '__main__':
    PASO_PRUEBA_S = 0.01
    MUESTRAS_PRUEBA = 5
    calculador = CalculadorCinematica()
    for indice in range(MUESTRAS_PRUEBA):
        tiempo_prueba = indice * PASO_PRUEBA_S
        print(f"t={tiempo_prueba:.2f}", calculador.actualizar(tiempo_prueba, {"prueba": tiempo_prueba ** 2}))