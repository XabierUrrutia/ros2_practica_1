# Practica 1

**Práctica 1 — Extracción y análisis de datos cinemáticos del robot UR3e con ROS 2 Jazzy**

| Autor | Rol principal |
|---|---|
| Xabier Urrutia | Trayectorias del robot, guardado en CSV, gráficas y detector de movimiento |
| Hugo Rey | Paquete ROS 2, nodo recolector, cálculo cinemático, mensaje personalizado y monitor |

---

## Índice

1. [¿Qué hace la herramienta?](#1-qué-hace-la-herramienta)
2. [Estructura del repositorio](#2-estructura-del-repositorio)
3. [Cómo funciona](#3-cómo-funciona)
4. [Cómo arrancarla](#4-cómo-arrancarla)
5. [Resultados esperados](#5-resultados-esperados)

---

## 1. ¿Qué hace la herramienta?

Mientras el UR3e simulado ejecuta tres trayectorias (**Joint**, **Lineal** y **Circular**), un nodo de ROS 2 escucha el estado de sus articulaciones y obtiene, para cada una de las seis articulaciones del brazo, la evolución temporal de:

- **Posición** (rad)
- **Velocidad** (rad/s)
- **Aceleración** (rad/s²)

Los datos se guardan **por separado para cada trayectoria** (un CSV cada una) y, a partir de ellos, se generan las gráficas.

| Requisito del enunciado | Implementación |
|---|---|
| Suscriptor en tiempo real al estado de las articulaciones | `recolector.py` → nodo `recoleccion_datos`, suscrito a `/joint_states` |
| Trayectorias Joint, Lineal y Circular | `trayectorias.py` → `EjecutorTrayectorias` (MoveIt 2) |
| Posición, velocidad y aceleración con marca de tiempo, por trayectoria | `cinematica.py` + `guardado_cinematico.py` → `datos/<trayectoria>.csv` |
| **Opcional:** mensaje ROS 2 personalizado | Paquete `practica_1_mensajes` → `EstadoCinematico.msg` |
| **Opcional:** ingesta inteligente (solo con el robot en movimiento) | `detector_movimiento.py` → `DetectorMovimiento` |

---

## 2. Estructura del repositorio

```
ros2_practica_1/
├── README.md
├── datos/                          # Resultados (se generan al ejecutar)
│   ├── joint.csv      joint.png
│   ├── lineal.csv     lineal.png
│   └── circular.csv   circular.png
├── practica_1/                     # Paquete ROS 2 (ament_python)
│   ├── package.xml
│   ├── setup.py / setup.cfg
│   ├── resource/practica_1
│   └── practica_1/
│       ├── recolector.py           # Nodo suscriptor: calcula, publica y guarda
│       ├── cinematica.py           # Derivación numérica de velocidad y aceleración
│       ├── detector_movimiento.py  # Decide si el robot se está moviendo
│       ├── guardado_cinematico.py  # Escritura de un CSV por trayectoria
│       ├── trayectorias.py         # Ejecuta las trayectorias Joint, Lineal y Circular
│       ├── control_robot.py        # Interfaz con MoveIt 2 (acciones y servicios)
│       ├── monitor_estado.py       # Nodo que muestra el mensaje personalizado
│       └── graficas.py             # Genera las gráficas a partir de los CSV
└── practica_1_mensajes/            # Paquete de interfaces (ament_cmake)
    ├── CMakeLists.txt
    ├── package.xml
    └── msg/EstadoCinematico.msg
```

El mensaje personalizado está en un paquete aparte porque los `.msg` solo pueden generarse desde un paquete `ament_cmake`; `practica_1` depende de él.

---

## 3. ¿Cómo funciona?

### 3.1 Nodos y topics


| Topic | Tipo | Publica | Se suscribe | Contenido |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Simulación | `recoleccion_datos` | Posición de cada articulación con su marca de tiempo |
| `/practica_1/trayectoria_activa` | `std_msgs/String` | `trayectorias` | `recoleccion_datos` | `"joint"`, `"lineal"` o `"circular"` al empezar; `""` al terminar |
| `/practica_1/estado_cinematico` | `practica_1_mensajes/EstadoCinematico` | `recoleccion_datos` | `monitor_estado` | Posición, velocidad y aceleración de las 6 articulaciones en un único mensaje |

### 3.2 Trayectorias (`trayectorias.py`)

Antes de cada trayectoria el robot vuelve a una **posición inicial común**, `[0, -π/2, -π/2, -π/2, π/2, 0]` rad, para que las tres sean repetibles y comparables. Todas se ejecutan al 10 % de la velocidad y aceleración máximas del robot.

| Trayectoria | Tipo | Qué hace |
|---|---|---|
| **Joint** | Espacio articular (`mover_articulaciones`) | Gira la base (`shoulder_pan_joint`) 45° (π/4 rad); el resto de articulaciones no se mueve |
| **Lineal** | Cartesiana en línea recta (`mover_trayectoria`) | Desplaza el efector final 0,1 m en el eje X |
| **Circular** | Cartesiana por waypoints (`mover_trayectoria`) | Recorre una circunferencia de 0,05 m de radio en el plano horizontal, aproximada con 24 puntos, y vuelve al punto de partida |

Cada trayectoria se **anuncia** en `/practica_1/trayectoria_activa` al empezar y al terminar. Antes de cada anuncio se espera 0,5 s para que el robot esté parado; así los datos de una trayectoria no incluyen restos de la anterior.

### 3.3 Cálculo cinemático (`cinematica.py`)

`JointState` solo garantiza la posición, así que velocidad y aceleración se obtienen por **derivación numérica respecto al tiempo**:

- `DerivadorVentana` calcula `(valor_actual − valor_referencia) / Δt`, donde la referencia es la muestra más reciente con al menos **0,1 s** de antigüedad. Derivar entre muestras consecutivas (~2 ms) amplificaría demasiado el ruido.
- `CalculadorCinematica` encadena dos derivadores: posición → velocidad → aceleración.
- Se usa la marca de tiempo del mensaje (`header.stamp`), no la hora de procesado, para que los retrasos del nodo no falseen las derivadas.

### 3.4 Ingesta inteligente (`detector_movimiento.py`)

`DetectorMovimiento` considera que el robot se mueve si **alguna articulación supera 0,001 rad/s** en valor absoluto. El recolector solo guarda una muestra si:

1. hay una trayectoria activa, **y**
2. el detector indica movimiento.

Los periodos de reposo (por ejemplo, mientras MoveIt planifica) se descartan.

### 3.5 Recolector (`recolector.py`)

Por cada mensaje de `/joint_states`:

1. Empareja cada valor con su articulación **por nombre** e ignora las de la pinza RG2.
2. Calcula el estado cinemático.
3. Publica `EstadoCinematico` reutilizando la cabecera del mensaje original.
4. Si hay trayectoria activa y movimiento, guarda una muestra por articulación con el tiempo **relativo al inicio del movimiento** (las gráficas empiezan en 0).
5. Muestra el estado por pantalla como máximo una vez por segundo.

Al recibir el fin de una trayectoria (o al cerrar con Ctrl+C) escribe `datos/<trayectoria>.csv`.

### 3.6 Mensaje personalizado (`EstadoCinematico.msg`)

```
std_msgs/Header header       # Marca de tiempo del /joint_states de origen
string trayectoria           # "joint", "lineal", "circular" o vacío
string[] articulaciones
float64[] posiciones         # rad
float64[] velocidades        # rad/s
float64[] aceleraciones      # rad/s²
```

Los arrays siguen el orden de `articulaciones`.

---

## 4. Cómo arrancarla

### 4.1 Requisitos

- Entorno Docker de la asignatura (ROS 2 Jazzy + simulación del UR3e + MoveIt 2), con sus requisitos cumplidos según el README del repositorio base.
- Este repositorio clonado en `~/ros_workspace/src/ros2_practica_1` dentro del contenedor.

### 4.2 Arrancar el entorno y la simulación

Desde el repositorio base del entorno:

```bash
lab.bat iniciar
lab.bat sim 203
```

Esperar a que aparezca `Successfully switched controllers!` y a que arranque `move_group`.

### 4.3 Compilar

En una terminal dentro del contenedor (`docker exec -it -u laboratorio ros2_mucsi_26_27-desktop-1 bash`):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros_workspace
colcon build --symlink-install --packages-select practica_1_mensajes practica_1
source install/setup.bash
```

Comprobar que el mensaje se ha generado:

```bash
ros2 interface show practica_1_mensajes/msg/EstadoCinematico
```

### 4.4 Lanzar los nodos

Abrir **tres terminales** en el contenedor y, en cada una, ejecutar antes `source ~/ros_workspace/install/setup.bash`. El orden importa: el recolector debe estar escuchando antes de que empiecen las trayectorias.

```bash
# Terminal 1 — recolector (desde la raíz del repo, para que datos/ se cree aquí)
cd ~/ros_workspace/src/ros2_practica_1
ros2 run practica_1 recolector

# Terminal 2 — monitor del mensaje personalizado (opcional)
ros2 run practica_1 monitor_estado

# Terminal 3 — trayectorias
ros2 run practica_1 trayectorias
```

Cuando terminan las tres trayectorias, la terminal 3 imprime:

```
{'joint': True, 'lineal': True, 'circular': True}
```

### 4.5 Generar las gráficas

Detener el recolector con **Ctrl+C** y, desde la raíz del repo:

```bash
cd ~/ros_workspace/src/ros2_practica_1
python3 -m practica_1.graficas
```

Se imprimen las rutas de las imágenes generadas en `datos/`.

### 4.6 Comprobaciones útiles

```bash
ros2 topic echo /practica_1/estado_cinematico --once   # un mensaje personalizado
ros2 topic hz /practica_1/estado_cinematico            # frecuencia de publicación
ros2 run rqt_graph rqt_graph                           # grafo de nodos y topics
```

---

## 5. Resultados esperados

### 5.1 Ficheros

Tras una ejecución completa, `datos/` contiene:

| Fichero | Contenido |
|---|---|
| `joint.csv`, `lineal.csv`, `circular.csv` | Una fila por articulación y por instante |
| `joint.png`, `lineal.png`, `circular.png` | Tres gráficas (posición, velocidad, aceleración) frente al tiempo, una línea por articulación |

Columnas del CSV:

```
trayectoria, tiempo_segundos, articulacion, posicion_rad, velocidad_rad_segundos, aceleracion_rad_segundos2
```

Ejemplo de las primeras filas de `joint.csv`:

```
trayectoria,tiempo_segundos,articulacion,posicion_rad,velocidad_rad_segundos,aceleracion_rad_segundos2
joint,0.0,shoulder_pan_joint,-0.000108,0.001081,0.010788
joint,0.0,shoulder_lift_joint,-1.570321,-0.000000,-0.000002
...
```

### 5.2 Valores obtenidos en nuestra ejecución

Los valores concretos variarán ligeramente entre ejecuciones (planificación de MoveIt y carga del contenedor), pero el comportamiento debe ser el mismo.

| | Joint | Lineal | Circular |
|---|---|---|---|
| Duración del movimiento registrado | 3,19 s | 2,41 s | 5,67 s |
| Muestras por articulación | 1 241 | 905 | 2 070 |
| Articulaciones que se mueven | 1 (`shoulder_pan_joint`) | 5 (todas salvo `wrist_2_joint`) | 5 (todas salvo `wrist_2_joint`) |
| Velocidad máxima (valor absoluto) | 0,37 rad/s | 0,36 rad/s (`elbow_joint`) | 0,36 rad/s |
| ¿Vuelve a la posición inicial? | No (termina en π/4) | No | Sí, en todas las articulaciones |

### 5.3 Qué debe verse en las gráficas

- **Joint:** solo `shoulder_pan_joint` pasa de 0 a 0,785 rad (π/4). La velocidad tiene **perfil trapezoidal**: sube, se mantiene en crucero (~0,3 rad/s) y baja. La aceleración es positiva al arrancar, ~0 en crucero y negativa al frenar.
- **Lineal:** varias articulaciones se mueven de forma coordinada para mantener el efector en línea recta. Todas empiezan y terminan con velocidad ≈ 0.
- **Circular:** las posiciones describen curvas suaves y **vuelven a su valor inicial**, como corresponde a una trayectoria cerrada. Las velocidades cambian de signo a mitad del recorrido.
- **En las tres:** la primera y la última muestra tienen velocidad cercana a 0,001 rad/s (el umbral del detector), lo que confirma que el reposo previo y posterior se ha descartado.

### 5.4 Limitaciones conocidas

- **Picos en la aceleración** (hasta ~1,5 rad/s² en la circular). La simulación publica nominalmente a 500 Hz, pero dentro de Docker el `controller_manager` sufre *overruns* y llegan unas 365–390 muestras por segundo, con huecos de hasta ~30 ms. Al derivar dos veces, esas irregularidades se amplifican. Se podrían suavizar ampliando la ventana o filtrando, a costa de más retraso.
- **Retraso de la derivada.** Por la ventana de 0,1 s, velocidad y aceleración van ligeramente retrasadas respecto a la posición; por eso la aceleración no llega exactamente a 0 al final del registro.
- **Ruta de `datos/`.** Es relativa al directorio desde el que se lanza el recolector: hay que lanzarlo desde la raíz del repositorio.
