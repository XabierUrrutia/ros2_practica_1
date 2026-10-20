# ros2_practica_1

Práctica 1 — Extracción y análisis de datos cinemáticos del robot UR3e con ROS 2 Jazzy.

Objetivo: un nodo en Python que se suscribe a `/joint_states` mientras el robot ejecuta
varias trayectorias, y extrae la evolución temporal de **posición, velocidad y aceleración**
de cada articulación, por separado para cada trayectoria.

- **Pareja:** Persona A = **Hugo** · Persona B = **Xabier** (reparto sorteado al azar y reequilibrado después por carga: el CSV y las gráficas pasaron de A a B)
- **Fecha límite de entrega:** _(rellenar)_
- **Entrega:** solo la URL de este repositorio.

> La evaluación es **individual y por commits**. Cada miembro debe tener commits propios
> con trabajo real. Sin commits, la nota es 0. No puede haber colaboradores externos.

---

## Reparto de tareas

Leyenda: `[ ]` pendiente · `[x]` hecho.

### Fase 0 — Preparación (juntos)

- [x] Ambos: clonar el repo y arrancar el entorno (`lab.bat iniciar`, `lab.bat sim 203`).
- [x] Ambos: comprobar que se ejecutan los ejemplos de `ejemplos/` (publicador, suscriptor, `control_robot.py`).
- [ ] **A (Hugo):** crear el paquete ROS 2 `practica_1` (`package.xml`, `setup.py`, `setup.cfg`, carpeta del módulo).
- [ ] **B (Xabier):** crear la rama de trabajo y comprobar que el paquete compila (`colcon build --symlink-install`).
- [ ] Ambos: acordar la **interfaz entre los dos nodos** (ver "Contrato entre nodos") antes de programar.

### Persona A (Hugo) — Recolección y procesado de datos

Responsable del suscriptor y de todo lo que pasa con los datos.

- [ ] Nodo suscriptor a `/joint_states` (clase propia, POO).
- [ ] Cálculo de **velocidad y aceleración**. `JointState.velocity` puede venir vacío o ser poco fiable, y la aceleración no viene en el mensaje: hay que derivar numéricamente respecto al tiempo.
- [ ] Almacenar `(tiempo, posición, velocidad, aceleración)` por articulación y **separado por trayectoria**, en una estructura que B pueda leer (ver "Contrato entre nodos").
- [ ] **Opcional:** mensaje `.msg` propio que empaquete posición, velocidad y aceleración en un solo mensaje (paquete de interfaces aparte, `practica_1_interfaces`).

### Persona B (Xabier) — Trayectorias del robot

Responsable de mover el robot y de avisar de cuándo empieza y acaba cada trayectoria.

- [x] Nodo que usa `ControlRobot` (de `ejemplos/control_robot.py`) para ejecutar las trayectorias.
- [x] Trayectoria 1: **Joint** (`mover_articulaciones`).
- [x] Trayectoria 2: **Lineal** (`mover_a_pose` / `mover_trayectoria` con dos puntos).
- [x] Trayectoria 3: **Circular** (varios waypoints sobre una circunferencia con `mover_trayectoria`).
- [x] Publicar el inicio y el fin de cada trayectoria para que A sepa dónde cortar los datos.
- [x] Guardar los resultados en disco (un CSV por trayectoria).
- [x] Script o notebook que dibuje las gráficas posición/velocidad/aceleración vs. tiempo.
- [ ] **Opcional:** ingesta inteligente. Una clase `DetectorMovimiento` que decide si el robot se está moviendo (umbral de velocidad con nombre de constante) y que A integra en el suscriptor.

### Calidad del código (los dos, 10 % de la nota)

- [ ] POO, sin valores mágicos (constantes con nombre).
- [ ] Docstrings en todas las clases y funciones, y comentarios no triviales.
- [ ] Type hints en todo.
- [ ] PEP-8: `variables_y_funciones`, `NombresDeClases`, `_funciones_privadas`.
- [ ] Revisión cruzada: cada uno revisa y entiende el código del otro (el profesor puede preguntar).

---

## Contrato entre nodos

Hay que acordarlo antes de programar, para trabajar en paralelo. Propuesta inicial:

| Elemento | Detalle |
|---|---|
| Topic `/practica_1/trayectoria_activa` | `std_msgs/String`. B publica el nombre de la trayectoria (`"joint"`, `"lineal"`, `"circular"`) al empezar y `""` al terminar. |
| Quién guarda | A: el suscriptor abre un buffer nuevo al recibir un nombre y lo cierra al recibir `""`. |
| Nombres de ficheros | `datos/<trayectoria>.csv` |
| Quién escribe el CSV | B: recibe de A los datos ya calculados y los escribe en disco. |
| Datos que pasa A a B | Por cada muestra: `trayectoria`, `tiempo` (s), `articulacion`, `posicion` (rad), `velocidad` (rad/s), `aceleracion` (rad/s²). Formato exacto pendiente de acordar. |

---

## Flujo de trabajo con Git

- Rama `main` siempre funcional. Cada uno trabaja en su rama (`feature/recolector`, `feature/trayectorias`) y se une a `main` con Pull Request revisado por el otro.
- Commits pequeños y con mensaje claro. Cada commit con el usuario propio de GitHub.
- Ningún commit después de la fecha límite (se considera entrega tardía).
- `git pull` antes de empezar a trabajar y `git push` al terminar.

## Estructura prevista

```
ros2_practica_1/
├── README.md
├── practica_1/                  # paquete ROS 2 (Python)
│   ├── package.xml
│   ├── setup.py
│   └── practica_1/
│       ├── recolector.py        # A
│       ├── trayectorias.py      # B
│       └── control_robot.py     # copiado de ejemplos/
├── practica_1_interfaces/       # opcional: .msg propio (A)
└── datos/                       # CSV generados
```

## Cómo ejecutar

_(Se rellenará cuando haya código.)_
