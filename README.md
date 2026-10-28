# ros2_practica_1

**Práctica 1 — Extracción y análisis de datos cinemáticos del robot UR3e con ROS 2 Jazzy**
Robótica y Automatización Inteligente · Máster Universitario en Computación y Sistemas Inteligentes (Universidad de Deusto)

| Autores | Rol principal |
|---|---|
| Hugo Rey | Paquete ROS 2, nodo recolector, cálculo cinemático, mensaje personalizado y monitor |
| Xabier Urrutia | Trayectorias del robot, guardado en CSV, gráficas y detector de movimiento |

---

## Objetivo

Desarrollar un nodo en Python que se suscribe al estado de las articulaciones del UR3e (`/joint_states`) mientras el robot ejecuta varias trayectorias, y extrae la evolución temporal de la **posición, velocidad y aceleración** de cada articulación, **por separado para cada trayectoria**.

## Requisitos cubiertos

| Requisito del enunciado | Implementación |
|---|---|
| Suscriptor en tiempo real al estado de las articulaciones | `recolector.py` → nodo `recoleccion_datos`, suscrito a `/joint_states` (~480 Hz) |
| Ejecución de trayectorias Joint, Lineal y Circular | `trayectorias.py` → `EjecutorTrayectorias` con MoveIt 2 |
| Posición, velocidad y aceleración con marca de tiempo, separadas por trayectoria | `cinematica.py` + `guardado_cinematico.py` → `datos/joint.csv`, `datos/lineal.csv`, `datos/circular.csv` |
| **Opcional:** mensaje ROS 2 personalizado | Paquete `practica_1_mensajes` con `EstadoCinematico.msg`, publicado en `/practica_1/estado_cinematico` y recibido por `monitor_estado.py` |
| **Opcional:** ingesta inteligente (solo cuando el robot se mueve) | `detector_movimiento.py` → `DetectorMovimiento`, integrado en el recolector |
| Calidad de código | POO, docstrings, type hints, PEP-8 y constantes con nombre (sin valores mágicos) |

---

## Estructura del repositorio

```
ros2_practica_1/
├── README.md
├── datos/                          # Resultados generados (CSV + gráficas)
│   ├── joint.csv      joint.png
│   ├── lineal.csv     lineal.png
│   └── circular.csv   circular.png
├── practica_1/                     # Paquete ROS 2 (ament_python)
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/practica_1
│   └── practica_1/
│       ├── recolector.py           # Nodo suscriptor: calcula, publica y guarda
│       ├── cinematica.py           # Derivación numérica de velocidad y aceleración
│       ├── detector_movimiento.py  # Decide si el robot se está moviendo
│       ├── guardado_cinematico.py  # Escritura de los CSV por trayectoria
│       ├── trayectorias.py         # Ejecuta las trayectorias Joint, Lineal y Circular
│       ├── control_robot.py        # Control del robot con MoveIt 2 (base de los ejemplos)
│       ├── monitor_estado.py       # Nodo que muestra el mensaje personalizado
│       └── graficas.py             # Genera las gráficas a partir de los CSV
└── practica_1_mensajes/            # Paquete de interfaces (ament_cmake)
    ├── CMakeLists.txt
    ├── package.xml
    └── msg/EstadoCinematico.msg
```

---

## Arquitectura

```
                    /joint_states (sensor_msgs/JointState, ~480 Hz)
  Simulación UR3e ─────────────────────────────────────┐
        ▲                                               ▼
        │ MoveIt 2                          ┌───────────────────────┐   /practica_1/estado_cinematico    ┌──────────────────┐
        │                                   │   recoleccion_datos   │ ─────────────────────────────────► │  monitor_estado  │
┌───────────────────┐ /practica_1/          │     (recolector)      │  (practica_1_mensajes/             └──────────────────┘
│   trayectorias    │ trayectoria_activa    │                       │   EstadoCinematico)
│ (control_robot)   │ ─────────────────────►│ cinemática + detector │
└───────────────────┘ (std_msgs/String)     └───────────┬───────────┘
                                                        │ al terminar cada trayectoria
                                                        ▼
                                              datos/<trayectoria>.csv ──► graficas.py ──► datos/<trayectoria>.png
```

### Comunicación entre nodos

| Topic | Tipo | Publica | Se suscribe | Contenido |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Simulación | `recoleccion_datos` | Posición de cada articulación con su marca de tiempo |
| `/practica_1/trayectoria_activa` | `std_msgs/String` | `trayectorias` | `recoleccion_datos` | `"joint"`, `"lineal"` o `"circular"` al empezar; `""` al terminar |
| `/practica_1/estado_cinematico` | `practica_1_mensajes/EstadoCinematico` | `recoleccion_datos` | `monitor_estado` | Posición, velocidad y aceleración de las 6 articulaciones en un único mensaje |

---

## Funcionamiento

### 1. Trayectorias (`trayectorias.py`)

Antes de cada trayectoria el robot vuelve a una **posición inicial común** (`[0, -π/2, -π/2, -π/2, π/2, 0]` rad), para que todas sean repetibles y comparables.

| Trayectoria | Tipo de movimiento | Descripción |
|---|---|---|
| **Joint** | Espacio articular (`mover_articulaciones`) | Gira la base 45° (π/4 rad); el resto de articulaciones no se mueve |
| **Lineal** | Cartesiano en línea recta (`mover_trayectoria`) | Desplaza el efector final 0,1 m en el eje X |
| **Circular** | Cartesiano por waypoints (`mover_trayectoria`) | Circunferencia de 0,05 m de radio en el plano horizontal, aproximada con 24 puntos |

Cada trayectoria se **anuncia** en `/practica_1/trayectoria_activa` al empezar y al terminar. Antes de cada anuncio se espera `PAUSA_ESTABILIZACION_S = 0.5 s`, para que el robot esté completamente parado: así los datos de una trayectoria no incluyen restos del movimiento anterior ni se cortan antes de que el robot frene.

### 2. Cálculo cinemático (`cinematica.py`)

`JointState` solo garantiza la **posición**; la velocidad puede venir vacía y la aceleración no viene. Por eso ambas se obtienen por **derivación numérica respecto al tiempo**:

- `DerivadorVentana` calcula la derivada como `(valor_actual − valor_referencia) / Δt`, donde la referencia es la muestra más reciente que tiene al menos `INTERVALO_DERIVADA_SEGUNDOS = 0.1 s` de antigüedad.
- `CalculadorCinematica` encadena dos derivadores: posición → **velocidad** → **aceleración**.

Usar una ventana de 0,1 s en lugar de dos muestras consecutivas (2 ms) reduce mucho el ruido, que de otro modo se amplifica en cada derivación.

La marca de tiempo es la del propio mensaje (`header.stamp`), no la hora de procesado, de modo que los retrasos del nodo no falsean las derivadas.

### 3. Ingesta inteligente (`detector_movimiento.py`)

`DetectorMovimiento` considera que el robot se mueve si **alguna articulación supera** `UMBRAL_VELOCIDAD_RAD_S = 0.001 rad/s` en valor absoluto. El recolector solo guarda muestras cuando:

1. hay una trayectoria activa, **y**
2. el detector indica que el robot se está moviendo.

Los instantes de reposo se descartan (por ejemplo, el tiempo que MoveIt tarda en planificar antes de arrancar).

### 4. Recolector (`recolector.py`)

Por cada mensaje de `/joint_states`:

1. Empareja cada valor con su articulación **por nombre** (el orden no está garantizado) e ignora las articulaciones de la pinza RG2, que publica en el mismo topic.
2. Calcula el estado cinemático con `CalculadorCinematica`.
3. Publica el mensaje personalizado `EstadoCinematico`, reutilizando la cabecera del mensaje original.
4. Si hay trayectoria activa y el robot se mueve, añade una muestra por articulación con el tiempo **relativo al inicio del movimiento**, para que las gráficas empiecen en 0.
5. Muestra el estado por pantalla, como máximo una vez por segundo para no saturar la terminal.

Al recibir el aviso de fin de una trayectoria (o al cerrar con Ctrl+C), guarda sus muestras en `datos/<trayectoria>.csv`.

### 5. Mensaje personalizado (`practica_1_mensajes/msg/EstadoCinematico.msg`)

```
std_msgs/Header header       # Marca de tiempo del /joint_states del que se ha calculado
string trayectoria           # "joint", "lineal", "circular" o vacío
string[] articulaciones
float64[] posiciones         # rad
float64[] velocidades        # rad/s
float64[] aceleraciones      # rad/s²
```

Los arrays van en el mismo orden que `articulaciones`. Los mensajes solo se pueden generar desde un paquete `ament_cmake`, por eso el `.msg` está en un paquete aparte (`practica_1_mensajes`), del que depende `practica_1`.

`monitor_estado.py` se suscribe a este topic y muestra su contenido dos veces por segundo, demostrando que el mensaje se envía y se recibe correctamente.

### 6. Guardado y gráficas

- `guardado_cinematico.py` escribe un CSV por trayectoria con las columnas:

  ```
  trayectoria, tiempo_segundos, articulacion, posicion_rad, velocidad_rad_segundos, aceleracion_rad_segundos2
  ```

- `graficas.py` lee todos los CSV de `datos/` y genera, para cada trayectoria, una imagen con tres gráficas (posición, velocidad y aceleración frente al tiempo) con una línea por articulación.

---

## Cómo ejecutar

### Requisitos

Entorno Docker de la asignatura (ROS 2 Jazzy + simulación del UR3e + MoveIt 2), con este repositorio en `~/ros_workspace/src/ros2_practica_1`.

### 1. Arrancar el entorno y la simulación

```bash
lab.bat iniciar        # Windows  (Linux: ./lab.sh iniciar)
lab.bat sim 203        # Windows  (Linux: ./lab.sh sim 203)
```

Esperar a que aparezca `Successfully switched controllers!` y arranque `move_group`.

### 2. Compilar

En una terminal del contenedor (`docker exec -it -u laboratorio ros2_mucsi_26_27-desktop-1 bash`):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros_workspace
colcon build --symlink-install --packages-select practica_1_mensajes practica_1
source install/setup.bash
```

> Si aparece `failed to create symbolic link ... Is a directory`, el paquete se compiló antes sin `--symlink-install`. Borrar `build/` e `install/` de esos paquetes y volver a compilar.

### 3. Lanzar los nodos (una terminal para cada uno, todas con `source ~/ros_workspace/install/setup.bash`)

```bash
# Terminal 1 — recolector (desde la raíz del repo, para que datos/ se cree aquí)
cd ~/ros_workspace/src/ros2_practica_1
ros2 run practica_1 recolector

# Terminal 2 — monitor del mensaje personalizado
ros2 run practica_1 monitor_estado

# Terminal 3 — trayectorias
ros2 run practica_1 trayectorias
```

Al terminar, `trayectorias` imprime `{'joint': True, 'lineal': True, 'circular': True}`.

### 4. Generar las gráficas

Detener el recolector con **Ctrl+C** y, desde la raíz del repo:

```bash
python3 -m practica_1.graficas
```

### Comprobaciones útiles

```bash
ros2 interface show practica_1_mensajes/msg/EstadoCinematico   # definición del mensaje
ros2 topic echo /practica_1/estado_cinematico --once            # un mensaje personalizado
ros2 topic hz /practica_1/estado_cinematico                     # frecuencia de publicación
ros2 run rqt_graph rqt_graph                                    # grafo de nodos y topics
```

---

## Resultados

| | Joint | Lineal | Circular |
|---|---|---|---|
| Duración del movimiento | 3,19 s | 2,41 s | 5,67 s |
| Muestras por articulación | 1 241 | 905 | 2 070 |
| Velocidad máxima al inicio / final | ≤ 0,0011 / ≤ 0,0010 rad/s | ≤ 0,0012 / ≤ 0,0017 rad/s | ≤ 0,0010 / ≤ 0,0011 rad/s |

- **Joint:** solo se mueve `shoulder_pan_joint`, de 0,000 a 0,785 rad (π/4). La velocidad tiene un perfil trapezoidal (≈ 0,31 rad/s en crucero) y la aceleración es ≈ +0,5 rad/s² al arrancar, 0 en crucero y ≈ −0,5 rad/s² al frenar.
- **Lineal:** varias articulaciones se mueven de forma coordinada para mantener el efector en línea recta; todas empiezan y terminan en reposo.
- **Circular:** las seis articulaciones vuelven exactamente a su posición inicial, como corresponde a una trayectoria cerrada.

Las gráficas están en `datos/joint.png`, `datos/lineal.png` y `datos/circular.png`.

### Limitaciones conocidas

- **Picos aislados en la aceleración.** El simulador publica nominalmente a 500 Hz, pero dentro de Docker el `controller_manager` sufre *overruns* (avisos `Overrun detected!` en su log) y llegan entre 365 y 390 muestras por segundo, con huecos de hasta ~30 ms. Al derivar dos veces, esas irregularidades se amplifican y producen picos puntuales. Podrían suavizarse ampliando la ventana de derivación o aplicando un filtro, a costa de introducir retraso.
- **Ruta de `datos/`.** Es relativa al directorio desde el que se lanza el recolector; por eso debe ejecutarse desde la raíz del repositorio.

---

## Flujo de trabajo con Git

- Rama `main` siempre funcional.
- Commits pequeños, con mensaje descriptivo y con el usuario de GitHub de cada autor (la evaluación es individual y por commits).
- Ningún colaborador externo al grupo.