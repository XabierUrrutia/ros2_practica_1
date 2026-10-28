"""
GuardadoCinematico.py

Guarda en .csv los datos cinemáticos de las articulaciones.
"""

import csv
from dataclasses import astuple, dataclass
from pathlib import Path
from typing import Iterable, List

# Carpeta de los .csv
CARPETA_DATOS: Path = Path("datos")
EXTENSION_CSV: str = ".csv"
CODIFICACION: str = "utf-8"

CABECERA_CSV: List[str] = ["trayectoria", "tiempo_segundos", "articulacion", "posicion_rad", "velocidad_rad_segundos", "aceleracion_rad_segundos2"]

@dataclass
class MuestraCinematica:
    """Datos cinemáticos de una articulación."""

    trayectoria: str
    tiempo: float
    articulacion: str
    posicion: float
    velocidad: float
    aceleracion: float

class GuardadorCsv:
    """Escribe en un .csv las datos de cada trayectoria."""

    def __init__(self, carpeta: Path = CARPETA_DATOS) -> None:
        """Crea la carpeta de destino si no existe.
        
        Args: 
            carpeta: Ruta donde se guardan los .csv.
        """

        self._carpeta = carpeta
        self._carpeta.mkdir(parents=True, exist_ok=True)

    def guardar(self, nombre_trayectoria: str, muestras: Iterable[MuestraCinematica]) -> Path:
        """Guarda las muestras de una trayectoria en `<carpeta>/<nombre>.csv`.

        Args:
            nombre_trayectoria: Nombre de la trayectoria del fichero.
            muestras: Una muestra por cada articualción.

        Returns:
            Ruta del fichero escrito.
        """

        ruta = self._carpeta / f"{nombre_trayectoria}{EXTENSION_CSV}"
        with ruta.open("w", newline="", encoding=CODIFICACION) as fichero:
            escritor = csv.writer(fichero)
            escritor.writerow(CABECERA_CSV)
            escritor.writerows(astuple(muestra) for muestra in muestras)
        return ruta