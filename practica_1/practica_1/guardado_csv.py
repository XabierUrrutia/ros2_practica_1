"""Escritura en disco de los datos cinemáticos de cada trayectoria."""

import csv
from dataclasses import astuple, dataclass
from pathlib import Path
from typing import Iterable

CARPETA_DATOS = Path("datos")
EXTENSION_CSV = ".csv"
CABECERA_CSV = ["trayectoria", "tiempo_s", "articulacion",
                "posicion_rad", "velocidad_rad_s", "aceleracion_rad_s2"]


@dataclass
class MuestraCinematica:
    """Datos cinemáticos de una articulación en un instante concreto."""

    trayectoria: str
    tiempo: float
    articulacion: str
    posicion: float
    velocidad: float
    aceleracion: float


class GuardadorCsv:
    """Escribe en un fichero CSV las muestras de cada trayectoria."""

    def __init__(self, carpeta: Path = CARPETA_DATOS) -> None:
        """Crea la carpeta de destino si no existe."""
        self.carpeta = carpeta
        self.carpeta.mkdir(parents=True, exist_ok=True)

    def guardar(self, nombre_trayectoria: str, muestras: Iterable[MuestraCinematica]) -> Path:
        """Guarda las muestras de una trayectoria en `<carpeta>/<nombre>.csv`.

        Returns:
            Ruta del fichero escrito.
        """
        ruta = self.carpeta / f"{nombre_trayectoria}{EXTENSION_CSV}"
        with ruta.open("w", newline="", encoding="utf-8") as fichero:
            escritor = csv.writer(fichero)
            escritor.writerow(CABECERA_CSV)
            escritor.writerows(astuple(muestra) for muestra in muestras)
        return ruta


if __name__ == '__main__':
    prueba = [MuestraCinematica("prueba", 0.0, "shoulder_pan_joint", 0.1, 0.0, 0.0),
              MuestraCinematica("prueba", 0.1, "shoulder_pan_joint", 0.2, 1.0, 10.0)]
    print(GuardadorCsv().guardar("prueba", prueba))