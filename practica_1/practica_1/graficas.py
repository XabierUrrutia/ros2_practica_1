"""
Graficas.py

A partir de los datos guardados en los .csv, genera un .png por cada trayectoria."""

import csv
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib
# El backend se elige antes de importar pyplot: "Agg" genera una imagen sin abrir una ventana
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from practica_1.guardado_cinematico import CARPETA_DATOS, CODIFICACION, EXTENSION_CSV

# Configuración de los archivos 
EXTENSION_IMAGEN: str = ".png"

# Nombre de las columans del .csv
COLUMNA_TIEMPO: str = "tiempo_segundos"
COLUMNA_ARTICULACION: str = "articulacion"

# Cada columna del .csv se etiqueta en el eje Y
CONFIGURACION_GRAFICAS: List[Tuple[str, str]]  = [
    ("posicion_rad", "Posición (rad)"),
    ("velocidad_rad_segundos", "Velocidad (rad/s)"),
    ("aceleracion_rad_segundos2", "Aceleración (rad/s²)"),
]

# Tamaño de la imagen (ancho, alto)
TAMANO_FIGURA: Tuple[float, float] = (10.0, 8.0)

# Configuración de la gráfica
POSICION_LEYENDA: str = "upper right"
TAMANNO_LEYENDA: str = "small"
ETIQUETA_TIEMPO: str = "Tiempo (s)"

class GraficadorTrayectorias:
    """Genera una imagen con tres gráficas por cada .csv."""

    def __init__(self, carpeta: Path = CARPETA_DATOS) -> None:
        """Inicia el graficador con la carpeta incluida.
        
        Args:
            carpeta: Ruta en la que se guardan las imágenes.
        """
        self._carpeta = carpeta
    
    def graficar(self, nombre_trayectoria: str) -> Path:
        """Genera las gráficas de una trayectoria. 

        Args:
            nombre_trayectoria: Nombre del .csv sin su extensión. 

        Returns:
            Ruta de la imagen generada.
        """
        ruta_csv = self._carpeta / f"{nombre_trayectoria}{EXTENSION_CSV}"
        datos_por_articulacion = self._leer_datos_por_articulacion(ruta_csv)

        figura, ejes = plt.subplots(len(CONFIGURACION_GRAFICAS), 1, sharex=True, figsize=TAMANO_FIGURA)
        for articulacion, filas in datos_por_articulacion.items():
            tiempos = [float(fila[COLUMNA_TIEMPO]) for fila in filas]
            for eje, (columna, etiqueta) in zip(ejes, CONFIGURACION_GRAFICAS):
                eje.plot(tiempos, [float(fila[columna]) for fila in filas], label=articulacion)
                eje.set_ylabel(etiqueta)

        ejes[0].legend(loc=POSICION_LEYENDA, fontsize=TAMANNO_LEYENDA)
        ejes[-1].set_xlabel(ETIQUETA_TIEMPO)
        figura.suptitle(f"Trayectoria {nombre_trayectoria}")

        ruta_imagen = self._carpeta / f"{nombre_trayectoria}{EXTENSION_IMAGEN}"
        figura.savefig(ruta_imagen)
        plt.close(figura)
        return ruta_imagen
    
    def graficar_todas(self) -> List[Path]:
        """Genera las gráficas de todos los .csv que haya en la carpeta de datos.
        
        Returns:
            Lista de las rutas de las imágenes .png o
            una Lista vacía si no hay nigún archivo.
        """

        rutas_csv = sorted(self._carpeta.glob(f"*{EXTENSION_CSV}"))
        return [self.graficar(ruta.stem) for ruta in rutas_csv]

    @staticmethod
    def _leer_datos_por_articulacion(ruta: Path) -> Dict[str, List[Dict[str, str]]]:
        """Lee un .csv y agrupa sus datos por articulación.

        Args:
            ruta: Ruta del .csv

        Returns:
            Diccionario de articulaciones con sus datos del .csv de cada articulación.
        """

        datos_por_articulacion: Dict[str, List[Dict[str, str]]] = defaultdict(list)
        with ruta.open(newline = "", encoding = CODIFICACION) as fichero:
            for fila in csv.DictReader(fichero):
                datos_por_articulacion[fila[COLUMNA_ARTICULACION]].append(fila)
        return datos_por_articulacion


if __name__ == '__main__':
    for imagen in GraficadorTrayectorias().graficar_todas():
        print(imagen)