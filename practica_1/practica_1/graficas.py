"""Dibuja posición, velocidad y aceleración de cada trayectoria a partir de sus CSV."""

import csv
from collections import defaultdict
from pathlib import Path
from typing import Dict, List

import matplotlib
matplotlib.use("Agg")  # Backend sin ventana
import matplotlib.pyplot as plt

CARPETA_DATOS = Path("datos")
EXTENSION_CSV = ".csv"
EXTENSION_IMAGEN = ".png"
COLUMNA_TIEMPO = "tiempo_s"
COLUMNA_ARTICULACION = "articulacion"
# Cada magnitud: (columna del CSV, etiqueta del eje Y)
MAGNITUDES = [
    ("posicion_rad", "Posición (rad)"),
    ("velocidad_rad_s", "Velocidad (rad/s)"),
    ("aceleracion_rad_s2", "Aceleración (rad/s²)"),
]
TAMANO_FIGURA = (10, 8)  # pulgadas (ancho, alto)

class GraficadorTrayectorias:
    """Genera una imagen con tres gráficas por cada CSV de trayectoria."""

    def __init__(self, carpeta: Path = CARPETA_DATOS) -> None:
        """Define la carpeta donde están los CSV y donde se guardarán las imágenes."""
        self.carpeta = carpeta

    def _leer_filas(self, ruta: Path) -> Dict[str, List[Dict[str, str]]]:
        """Lee un CSV y agrupa sus filas por articulación."""
        filas_por_actualizacion = defaultdict(list)
        with ruta.open(newline="", encoding="UTF8") as fichero:
            for fila in csv.DictReader(fichero):
                filas_por_actualizacion[fila[COLUMNA_ARTICULACION]].append(fila)
        return filas_por_actualizacion
    
    def graficar(self, nombre_trayectoria: str) -> Path:
        """Dibuja posición, velocidad y aceleración frente al tiempo de una trayectoria.

        Returns:
            Ruta de la imagen generada.
        """
        ruta_csv = self.carpeta / f"{nombre_trayectoria}{EXTENSION_CSV}"
        filas_por_articulacion = self._leer_filas(ruta_csv)

        figura, ejes = plt.subplots(len(MAGNITUDES), 1, sharex=True, figsize=TAMANO_FIGURA)
        for articulacion, filas in filas_por_articulacion.items():
            tiempos = [float(fila[COLUMNA_TIEMPO]) for fila in filas]
            for eje, (columna, etiqueta) in zip(ejes, MAGNITUDES):
                eje.plot(tiempos, [float(fila[columna]) for fila in filas], label=articulacion)
                eje.set_ylabel(etiqueta)

        ejes[0].legend(loc="upper right", fontsize="small")
        ejes[-1].set_xlabel("Tiempo (s)")
        figura.suptitle(f"Trayectoria {nombre_trayectoria}")

        ruta_imagen = self.carpeta / f"{nombre_trayectoria}{EXTENSION_IMAGEN}"
        figura.savefig(ruta_imagen)
        plt.close(figura)  # Libera la memoria de la figura
        return ruta_imagen
    
    def graficar_todas(self) -> List[Path]:
        """Genera la imagen de cada CSV que haya en la carpeta de datos."""
        return [self.graficar(ruta.stem) for ruta in sorted(self.carpeta.glob(f"*{EXTENSION_CSV}"))]
    
if __name__ == '__main__':
    for imagen in GraficadorTrayectorias().graficar_todas():
        print(imagen)