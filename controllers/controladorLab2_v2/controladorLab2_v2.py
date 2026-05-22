from controller import Robot
import math
import csv
import os

# Dimensiones físicas del robot e-puck
RADIO_RUEDA = 0.0205
MAX_VELOCIDAD = 6.28

# Configuración de la simulación (en milisegundos)
TIME_STEP = 50

# Reglas de decisión para el movimiento
UMBRAL_OBSTACULO = 0.6  # Metros de distancia mínima al frente
UMBRAL_PARED_LATERAL = 78.0  # Lectura cruda de los sensores de los lados
VELOCIDAD_GIRO = 2.5
VELOCIDAD_AVANCE = MAX_SPEED = 6.28 * 0.5

# Parámetros matemáticos del Filtro de Kalman
Q_PROCESO = 0.01
R_MEDICION = 0.4
ALPHA_EMA = 0.3

# Variables globales para guardar el estado del Filtro de Kalman
kalman_distancia = 1.0  
kalman_incertidumbre = 1.0  
kalman_prediccion = 1.0  

def sensor_a_metros(valor_crudo):
    valor_crudo = max(valor_crudo, 1.0)
    return (0.05 * 1024.0) / valor_crudo


def encoder_a_lineal(angulo_delta):
    return RADIO_RUEDA * angulo_delta


def filtro_promedio_movil(valor_nuevo, valor_anterior):
    return ALPHA_EMA * valor_nuevo + (1 - ALPHA_EMA) * valor_anterior


def ejecutar_filtro_kalman(avance_robot, medicion_sensor):
    global kalman_distancia, kalman_incertidumbre, kalman_prediccion

    # 1. Predicción
    kalman_prediccion = kalman_distancia - avance_robot
    kalman_incertidumbre = kalman_incertidumbre + Q_PROCESO

    # 2. Corrección
    ganancia_bucle = kalman_incertidumbre / (kalman_incertidumbre + R_MEDICION)
    kalman_distancia = kalman_prediccion + ganancia_bucle * (medicion_sensor - kalman_prediccion)
    kalman_incertidumbre = (1 - ganancia_bucle) * kalman_incertidumbre

    return kalman_distancia, ganancia_bucle


# Se crea un archivo csv para analizarlo luego
def crear_archivo_csv(nombre_archivo="sensor_log.csv"):
    ruta = os.path.join(os.path.dirname(__file__), nombre_archivo)
    archivo = open(ruta, "w", newline="")
    escritor = csv.writer(archivo)
    escritor.writerow([
        "step", "time_s", "ps0_raw", "ps7_raw", "ps1_raw", "ps2_raw",
        "ps5_raw", "ps6_raw", "front_m_raw", "front_filtered",
        "left_enc", "right_enc", "delta_d", "kalman_prediccion",
        "kalman_estimate", "kalman_gain", "action"
    ])
    return archivo, escritor


def guardar_fila_csv(escritor, paso, tiempo, ps0, ps7, ps1, ps2, ps5, ps6, dist_cruda, dist_filtrada, 
                     enc_izq, enc_der, avance, pred_k, est_k, ganancia_k, accion):
    #Se escriben los resultados con los decimales redondeados
    escritor.writerow([
        paso, round(tiempo, 3), round(ps0, 2), round(ps7, 2),
        round(ps1, 2), round(ps2, 2), round(ps5, 2), round(ps6, 2),
        round(dist_cruda, 4), round(dist_filtrada, 4),
        round(enc_izq, 4), round(enc_der, 4), round(avance, 6),
        round(pred_k, 4), round(est_k, 4), round(ganancia_k, 4), accion
    ])



# ----------- MAIN -----------
robot = Robot()

# Configurar motores para que giren libres por velocidad infinita
motor_izquierdo = robot.getDevice("left wheel motor")
motor_derecho = robot.getDevice("right wheel motor")
motor_izquierdo.setPosition(float("inf"))
motor_derecho.setPosition(float("inf"))
motor_izquierdo.setVelocity(0.0)
motor_derecho.setVelocity(0.0)

encoder_izquierdo = robot.getDevice("left wheel sensor")
encoder_derecho = robot.getDevice("right wheel sensor")
encoder_izquierdo.enable(TIME_STEP)
encoder_derecho.enable(TIME_STEP)

nombres_sensores = ["ps0", "ps1", "ps2", "ps5", "ps6", "ps7"]
sensores = {}
for nombre in nombres_sensores:
    dispositivo = robot.getDevice(nombre)
    dispositivo.enable(TIME_STEP)
    sensores[nombre] = dispositivo

# Preparar archivo de registro de datos
archivo_csv, escritor_csv = crear_archivo_csv()

anterior_enc_izq = 0.0
anterior_enc_der = 0.0
distancia_frontal_filtrada = None
paso_actual = 0

# Primer paso para capturar valores iniciales
robot.step(TIME_STEP)
anterior_enc_izq = encoder_izquierdo.getValue()
anterior_enc_der = encoder_derecho.getValue()

while robot.step(TIME_STEP) != -1:
    tiempo_segundos = paso_actual * (TIME_STEP / 1000.0)

    # 1. Leer los valores de los sensores del robot
    sensor_ps0 = sensores["ps0"].getValue()
    sensor_ps7 = sensores["ps7"].getValue()
    sensor_ps1 = sensores["ps1"].getValue()
    sensor_ps2 = sensores["ps2"].getValue()
    sensor_ps5 = sensores["ps5"].getValue()
    sensor_ps6 = sensores["ps6"].getValue()

    # 2. Convertir sensores frontales a metros y promediarlos
    distancia_frontal_cruda = (sensor_a_metros(sensor_ps0) + sensor_a_metros(sensor_ps7)) / 2.0

    # Guardar las peores lecturas laterales (el objeto más cercano)
    pared_izquierda = max(sensor_ps5, sensor_ps6)
    pared_derecha = max(sensor_ps1, sensor_ps2)

    # 3. Aplicar Filtro Suavizado EMA
    if distancia_frontal_filtrada is None:
        distancia_frontal_filtrada = distancia_frontal_cruda
    distancia_frontal_filtrada = filtro_promedio_movil(distancia_frontal_cruda, distancia_frontal_filtrada)

    # 4. Calcular el avance del robot usando las ruedas
    actual_enc_izq = encoder_izquierdo.getValue()
    actual_enc_der = encoder_derecho.getValue()

    giro_izquierdo = actual_enc_izq - anterior_enc_izq
    giro_derecho = actual_enc_der - anterior_enc_der

    # Corregir errores de lectura iniciales si el sensor da valores extraños (infinitos o vacíos)
    if math.isnan(giro_izquierdo) or math.isinf(giro_izquierdo): giro_izquierdo = 0.0
    if math.isnan(giro_derecho) or math.isinf(giro_derecho):   giro_derecho = 0.0

    distancia_avanzada = (encoder_a_lineal(giro_izquierdo) + encoder_a_lineal(giro_derecho)) / 2.0

    # Actualizar memoria para el próximo ciclo
    anterior_enc_izq = actual_enc_izq
    anterior_enc_der = actual_enc_der

    # 5. Ejecutar Filtro de Kalman simplificado
    estimacion_kalman, ganancia_kalman = ejecutar_filtro_kalman(distancia_avanzada, distancia_frontal_cruda)

    # 6. Tomar decisiones de navegación reactiva
    if estimacion_kalman <= UMBRAL_OBSTACULO:
        # Obstáculo detectado al frente -> Escapar hacia el lado con más espacio libre
        if pared_izquierda > pared_derecha:
            vel_izq = VELOCIDAD_GIRO
            vel_der = -VELOCIDAD_GIRO
            accion_actual = "GIRAR_DERECHA"
        else:
            vel_izq = -VELOCIDAD_GIRO
            vel_der = VELOCIDAD_GIRO
            accion_actual = "GIRAR_IZQUIERDA"

    elif pared_izquierda > UMBRAL_PARED_LATERAL:
        # Demasiado cerca de la pared izquierda -> Corrección suave a la derecha
        vel_izq = VELOCIDAD_AVANCE
        vel_der = VELOCIDAD_AVANCE * 0.3
        accion_actual = "CURVA_DERECHA"

    elif pared_derecha > UMBRAL_PARED_LATERAL:
        # Demasiado cerca de la pared derecha -> Corrección suave a la izquierda
        vel_izq = VELOCIDAD_AVANCE * 0.3
        vel_der = VELOCIDAD_AVANCE
        accion_actual = "CURVA_IZQUIERDA"

    else:
        # El camino está despejado -> Avanzar en línea recta
        vel_izq = VELOCIDAD_AVANCE
        vel_der = VELOCIDAD_AVANCE
        accion_actual = "AVANZAR"

    # Asegurar que las velocidades no sobrepasen los límites físicos del e-puck
    vel_izq = max(min(vel_izq, 6.28), -6.28)
    vel_der = max(min(vel_der, 6.28), -6.28)

    motor_izquierdo.setVelocity(vel_izq)
    motor_derecho.setVelocity(vel_der)

    # 7. Guardar telemetría en el archivo externo
    guardar_fila_csv(escritor_csv, paso_actual, tiempo_segundos, sensor_ps0, sensor_ps7, 
                     sensor_ps1, sensor_ps2, sensor_ps5, sensor_ps6, distancia_frontal_cruda, 
                     distancia_frontal_filtrada, actual_enc_izq, actual_enc_der, distancia_avanzada, 
                     kalman_prediccion, estimacion_kalman, ganancia_kalman, accion_actual)

    paso_actual += 1

# Cerrar el documento al terminar la simulación de Webots
archivo_csv.close()
print("Se exportó el archivo con los datos")