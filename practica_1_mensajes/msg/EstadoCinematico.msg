# Estado cinemático de las articulaciones del brazo en un instante.
# Los arrays van en el mismo orden que 'articulaciones'.

# Marca de tiempo del mensaje de /joint_states del que se ha calculado
std_msgs/Header header
# Trayectoria en curso ("joint", "lineal", "circular"), o vacío si no hay ninguna
string trayectoria
string[] articulaciones
float64[] posiciones       # rad
float64[] velocidades      # rad/s
float64[] aceleraciones    # rad/s²