"""
Cinematica.py

Calcula la posición, velocidad y aceleración de las articulaciones.
"""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple

#Tiempo mínimo entre registros para calcular la derivada.
INTERVALO_DERIVADA_SEGUNDOS: float = 0.1

@dataclass
class EstadoCinematico:
    """Establece la posición (rad), velocidad (rad/s) y aceleración (rad/s²) de una articulación."""

    posicion: float
    velocidad: float
    aceleracion: float

class DerivadorVentana:
    """Calcula cuanto cambia la velocidad y los cambios de velocidad 
    a partir de las posiciones de las articulaciones a lo largo del tiempo."""

    def __init__(self, intervalo_derivada_segundos: float = INTERVALO_DERIVADA_SEGUNDOS) -> None:
        """Inicia el derivador. 
        
        Args:
            intervalo_derivada_segundos: Tiempo mínimo entre registros. 
        """
        
        self.intervalo_derivada_segundos = intervalo_derivada_segundos
        self.reiniciar()

    def reiniciar(self) -> None:
        """Elimina los registros anteriores"""

        self._historial: Deque[Tuple[float, Dict[str, float]]] = deque()

    def derivar(self, tiempo: float, valores: Dict[str, float]) -> Optional[Dict[str, float]]:
        """Calcula la derivada de los valores introducidos

        Args:
            tiempo: Tiempo actual, en segundos. 
            valores: Valores actuales. 

        Returns:
            Un diccionario con las derivadas calculadas o 
            None si todavía no hay suficientes registros.
        """

        if self._historial and tiempo <= self._historial[-1][0]:
            return None 
        self._historial.append((tiempo, dict(valores)))

        #Conservar el registro más antiguo. 
        while len(self._historial) >= 2 and tiempo - self._historial[1][0] >= self.intervalo_derivada_segundos:
            self._historial.popleft()

        tiempo_referencia, valores_referencia = self._historial[0]
        intervalo = tiempo - tiempo_referencia
        if intervalo < self.intervalo_derivada_segundos:
            return None
        
        return {
            nombre: (valor - valores_referencia[nombre]) / intervalo
            for nombre, valor in valores.items()
        }


class CalculadorCinematica:
    """Calcula la velocidad y la aceleración de las articulaciones."""

    def __init__(self, intervalo_derivada_segundos: float = INTERVALO_DERIVADA_SEGUNDOS) -> None:
        """Incia los derivadores de velocidad y aceleración. 

        Args:
            intervalo_derivada_segundos: Tiempo mínimo entre registros.
        """

        self._derivador_velocidad = DerivadorVentana(intervalo_derivada_segundos)
        self._derivador_aceleracion = DerivadorVentana(intervalo_derivada_segundos)

    def reiniciar(self) -> None:
        """Elimina los registros anteriores"""

        self._derivador_velocidad.reiniciar()
        self._derivador_aceleracion.reiniciar()

    def actualizar(self, tiempo: float, posiciones: Dict[str, float]) -> Optional[Dict[str, EstadoCinematico]]:
        """Calcula el estado cinenmático de cada articulación. 

        Args: 
            tiempo: Tiempo, en segundos. 
            posiciones: Posición de cada articulación, en radianes. 

        Returns:
            Un diccionario con el estado de cada articulación o 
            None si todavía no hay un historial
        """

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