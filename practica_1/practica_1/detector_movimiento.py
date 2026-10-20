"""Detección de si el robot se mueve, a partir de la velocidad de sus articulaciones."""

from typing import Sequence

# Velocidad (rad/s) por debajo de la cual una articulación se considera parada.
# Valor inicial: hay que ajustarlo observando el ruido de /joint_states con el robot en reposo.
UMBRAL_VELOCIDAD: float = 0.001


class DetectorMovimiento:
    """Decide si el robot se mueve comparando la velocidad de sus articulaciones con un umbral."""

    def __init__(self, umbral_velocidad: float = UMBRAL_VELOCIDAD) -> None:
        """Guarda el umbral de velocidad (rad/s) a partir del cual hay movimiento."""
        self.umbral_velocidad = umbral_velocidad

    def esta_en_movimiento(self, velocidades: Sequence[float]) -> bool:
        """Indica si alguna articulación supera el umbral de velocidad.

        Se usa el valor absoluto porque una articulación que gira en sentido
        negativo también se está moviendo.

        Args:
            velocidades: velocidad de cada articulación en un instante (rad/s).

        Returns:
            True si al menos una articulación se mueve más rápido que el umbral.
        """
        return any(abs(velocidad) > self.umbral_velocidad for velocidad in velocidades)


if __name__ == '__main__':
    detector = DetectorMovimiento()
    print(detector.esta_en_movimiento([0.0] * 6))                              # Esperado: False
    print(detector.esta_en_movimiento([0.0005, -0.0002, 0.0, 0.0, 0.0, 0.0]))  # Esperado: False (ruido)
    print(detector.esta_en_movimiento([0.0, 0.5, 0.0, 0.0, 0.0, 0.0]))         # Esperado: True
    print(detector.esta_en_movimiento([0.0, 0.0, -0.3, 0.0, 0.0, 0.0]))        # Esperado: True (sentido negativo)
