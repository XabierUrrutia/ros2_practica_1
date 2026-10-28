"""
DetectorMovimiento.py

Determina si el robot se está moviendo en base a la velocidad de sus articulaciones.
"""

from typing import Sequence

# Velocidad mínima para considerar que una articulación se está moviendo. 
#TODO El valor inicial se debe ajustar según el ruido del robot en reposo. 
UMBRAL_VELOCIDAD_RAD_S: float = 0.001

class DetectorMovimiento:
    """Detecta si alguna articulación está en movimiento."""

    def __init__(self, umbral_velocidad: float = UMBRAL_VELOCIDAD_RAD_S) -> None:
        """Inicia el detector para determinar si el robot se mueve.
        
        Args:
            umbral_velocidad: Velocidad mínima que se considera que la articulación está en movimiento, en rad/s
        """
        self.umbral_velocidad = umbral_velocidad

    def esta_en_movimiento(self, velocidades: Sequence[float]) -> bool:
        """Comprueba si una articulación supera el umbral de velocidad, en valor absoluto. 

        Args:
            velocidades: Velocidad las articulaciones, rad/s.

        Returns:
            True si alguna articulación se mueve o
            False en caso contrario.
        """
        return any(abs(velocidad) > self.umbral_velocidad
        for velocidad in velocidades)