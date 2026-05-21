from controller import Robot
import math
import csv
import os

# Parámetros del robot
WHEEL_RADIUS   = 0.0205   # Esto es el radio de las ruedas en metros
WHEEL_DISTANCE = 0.052    # Esto es al distancia entre ruedas en metros 
MAX_SPEED      = 6.28     # rad/s

# Parámetros de simulación 
TIME_STEP = 50 # Está medida en ms, como el ejemplo del lab que sale Ts = 0.05 s,  fs = 20 Hz

# Umbrales de navegación
OBSTACLE_THRESHOLD = 0.10  # metros: si distancia estimada < umbral, entonces debe girar
TURN_SPEED = 3.0   # rad/s para girar

# Parámetros del filtro de Kalman 
# Q: varianza del proceso, que es la incertidumbre en la predicción por encoders
# R: varianza de la medición, que es el ruido del sensor de distancia
Q_PROCESS = 1e-4
R_MEASURE = 5e-3

# Filtro paso-bajo simple (media móvil exponencial) 
ALPHA = 0.3   # 0 < alpha < 1 ; valores más bajos, entonces hay más suavizado

#Filtro de Kalman escalar para estimar distancia frontal.
class KalmanFilter1D: 

    def __init__(self, initial_distance: float, Q: float, R: float):
        self.d_hat = initial_distance # estimación actual
        self.P = 1.0 # covarianza del error inicial
        self.Q = Q  # varianza del proceso
        self.R = R # varianza de la medición

    def predict(self, delta_d: float):
        """
        Etapa de predicción.
        delta_d: avance estimado del robot (positivo = avanzar → distancia disminuye)
        """
        self.d_hat = self.d_hat - delta_d   # el robot avanza → distancia frontal decrece
        self.P = self.P + self.Q        # la incertidumbre crece al predecir

    def correct(self, z: float):
        """
        Etapa de corrección.
        z: medición del sensor frontal (en metros)
        """
        K = self.P / (self.P + self.R)   # ganancia de Kalman
        self.d_hat = self.d_hat + K * (z - self.d_hat)
        self.P = (1 - K) * self.P
        return self.d_hat, K


def sensor_to_meters(raw_value: float, max_range: float = 0.15) -> float:
    """
    Convierte el valor crudo de un sensor de distancia del e-puck a metros.
    El e-puck devuelve valores entre 0 (lejos) y ~4096 (muy cerca).
    Se usa una curva logarítmica aproximada.
    """
    # Valor mínimo para evitar log(0)
    raw_value = max(raw_value, 1.0)
    # Aproximación empírica para el e-puck
    distance = max_range - (max_range / 4096.0) * raw_value
    return max(0.005, distance)   # mínimo 5 mm


def encoder_to_linear(delta_angle: float) -> float:
    """Convierte variación angular del encoder (rad) a desplazamiento lineal (m)."""
    return WHEEL_RADIUS * delta_angle


def exponential_moving_average(new_val: float, prev_filtered: float, alpha: float) -> float:
    """Filtro paso-bajo de primer orden."""
    return alpha * new_val + (1 - alpha) * prev_filtered


# MAIN

def main():
    robot = Robot()

    # Motores 
    left_motor  = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float("inf"))   # modo velocidad
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # Encoders (posición de rueda) 
    left_encoder  = robot.getDevice("left wheel sensor")
    right_encoder = robot.getDevice("right wheel sensor")
    left_encoder.enable(TIME_STEP)
    right_encoder.enable(TIME_STEP)

    # Sensores de distancia (e-puck tiene 8: ps0…ps7)
    # ps0, ps7 son los sensores frontales derecho e izquierdo
    # ps5, ps6 son los sensores del lateral izquierdo
    # ps1, ps2 son los sensores del lateral derecho
    sensor_names = ["ps0", "ps1", "ps2", "ps5", "ps6", "ps7"]
    sensors = {}
    for name in sensor_names:
        s = robot.getDevice(name)
        s.enable(TIME_STEP)
        sensors[name] = s

    # archivo CSV  
    log_path = os.path.join(os.path.dirname(__file__), "sensor_log.csv")
    csv_file  = open(log_path, "w", newline="")
    writer    = csv.writer(csv_file)
    writer.writerow([
        "step", "time_s",
        "ps0_raw", "ps7_raw",         # frontales crudos
        "ps1_raw", "ps2_raw",         # lateral derecho crudo
        "ps5_raw", "ps6_raw",         # lateral izquierdo crudo
        "front_m_raw",                # distancia frontal cruda (media ps0+ps7) en metros
        "front_filtered",             # distancia frontal filtrada (EMA)
        "left_enc", "right_enc",      # encoders crudos (rad)
        "delta_d",                    # avance estimado (m)
        "kalman_estimate",            # estimación Kalman (m)
        "kalman_gain",                # ganancia Kalman
        "action"                      # acción tomada
    ])

    # Estado inicial
    prev_left_enc  = 0.0
    prev_right_enc = 0.0
    front_filtered = 0.10 # valor inicial del filtro EMA
    kf = KalmanFilter1D(initial_distance=0.10, Q=Q_PROCESS, R=R_MEASURE)

    step = 0

    # Esperar primer tick para leer encoders iniciales
    robot.step(TIME_STEP)
    prev_left_enc  = left_encoder.getValue()
    prev_right_enc = right_encoder.getValue()

    print("[Lab2] Iniciando navegación reactiva con filtro de Kalman...")
    print(f"  Ts = {TIME_STEP/1000} s  |  fs = {1000/TIME_STEP} Hz")
    print(f"  Umbral de obstáculo: {OBSTACLE_THRESHOLD} m")
    print(f"  Log: {log_path}")

    while robot.step(TIME_STEP) != -1:
        time_s = step * (TIME_STEP / 1000.0)

        # 1. Leer sensores crudos 
        ps0_raw = sensors["ps0"].getValue()
        ps7_raw = sensors["ps7"].getValue()
        ps1_raw = sensors["ps1"].getValue()
        ps2_raw = sensors["ps2"].getValue()
        ps5_raw = sensors["ps5"].getValue()
        ps6_raw = sensors["ps6"].getValue()

        # 2. Convertir sensores frontales a metros
        dist_ps0 = sensor_to_meters(ps0_raw)
        dist_ps7 = sensor_to_meters(ps7_raw)
        front_m_raw = (dist_ps0 + dist_ps7) / 2.0   # media de los dos frontales

        # Sensores laterales (usados solo para decidir dirección de giro)
        dist_left  = sensor_to_meters((ps5_raw + ps6_raw) / 2.0)
        dist_right = sensor_to_meters((ps1_raw + ps2_raw) / 2.0)

        # 3. Filtro EMA sobre distancia frontal 
        front_filtered = exponential_moving_average(front_m_raw, front_filtered, ALPHA)

        # 4. Leer encoders y calcular avance 
        curr_left_enc  = left_encoder.getValue()
        curr_right_enc = right_encoder.getValue()

        delta_left_rad  = curr_left_enc  - prev_left_enc
        delta_right_rad = curr_right_enc - prev_right_enc

        delta_left_m  = encoder_to_linear(delta_left_rad)
        delta_right_m = encoder_to_linear(delta_right_rad)
        delta_d       = (delta_left_m + delta_right_m) / 2.0   # avance lineal

        prev_left_enc  = curr_left_enc
        prev_right_enc = curr_right_enc

        # 5. Filtro de Kalman
        kf.predict(delta_d) # etapa de predicción
        kalman_est, kalman_gain = kf.correct(front_m_raw) # etapa de corrección
        # Clamp para evitar distancias negativas
        kf.d_hat = max(0.005, kf.d_hat)

        # 6. Navegación reactiva 
        if kalman_est > OBSTACLE_THRESHOLD:
            # Avanzar
            left_speed  = MAX_SPEED
            right_speed = MAX_SPEED
            action = "AVANZAR"
        else:
            # Girar: usar sensores laterales para decidir dirección
            if dist_left >= dist_right:
                # Más espacio a la izquierda -> girar a la izquierda
                left_speed  = -TURN_SPEED
                right_speed =  TURN_SPEED
                action = "GIRAR_IZQUIERDA"
            else:
                # Más espacio a la derecha → girar a la derecha
                left_speed  =  TURN_SPEED
                right_speed = -TURN_SPEED
                action = "GIRAR_DERECHA"

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        # 7. Registrar datos
        writer.writerow([
            step, round(time_s, 3),
            round(ps0_raw, 2), round(ps7_raw, 2),
            round(ps1_raw, 2), round(ps2_raw, 2),
            round(ps5_raw, 2), round(ps6_raw, 2),
            round(front_m_raw, 4),
            round(front_filtered, 4),
            round(curr_left_enc, 4), round(curr_right_enc, 4),
            round(delta_d, 6),
            round(kalman_est, 4),
            round(kalman_gain, 4),
            action
        ])

        step += 1

    csv_file.close()
    print("[Lab2] Simulación terminada. Datos guardados en sensor_log.csv")


if __name__ == "__main__":
    main()