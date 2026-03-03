import motor, hub, math, runloop, utime

# Setup
PORT_IZQ, PORT_DER = hub.port.A, hub.port.E

# Variables globales
R_RUEDA = 2.78
T = 0.015
x, y_pos = 0.0, 0.0
i_k = 0.0
d_k_prev = 0.0
y_meas_prev = 0.0

async def corregir_orientacion(target_angle, i_limit=150):
    global i_k, d_k_prev, y_meas_prev
    Kp=4.5
    Ti=0.29
    Td=0.08
    i_k = 0.0
    d_k_prev = 0.0
    y_meas_prev = hub.motion_sensor.tilt_angles()[0] / 10.0

    while True:
        # 1. Obtener ángulo actual y calcular error circular
        y_k = hub.motion_sensor.tilt_angles()[0] / 10.0
        e_k = (target_angle - y_k + 180) % 360 - 180

        # 2. Condición de parada (umbral de precisión)
        if abs(e_k) < 1.0: # Margen de 1 grado
            motor.stop(PORT_IZQ, stop=motor.HOLD)
            motor.stop(PORT_DER, stop=motor.HOLD)
            break

        # 3. Cálculo del PID (Términos P, I y D con filtro N)
        p_k = Kp * e_k
        i_k_fut = i_k + (Kp * T / Ti) * e_k
        i_k_fut = max(-i_limit, min(i_limit, i_k_fut))

        N = 10.0 # Factor de filtrado
        denom = (N * T) + Td
        d_k = (Td / denom) * d_k_prev + ((Kp * Td * N) / denom) * (y_k - y_meas_prev)

        u_unsat = p_k + i_k - d_k
        u_k = max(-600, min(600, u_unsat)) # Limitar velocidad de giro

        if abs(u_unsat) < 600: i_k = i_k_fut

        # 4. Actuación: Giro sobre su propio eje (v_lineal = 0)
        # Un motor avanza y el otro retrocede con la misma magnitud
        motor.run(PORT_IZQ, int(u_k))
        motor.run(PORT_DER, int(u_k))

        # 5. Actualización de variables de estado
        d_k_prev, y_meas_prev = d_k, y_k
        await runloop.sleep_ms(int(T * 1000))

async def navegar_pid_avanzado(target_x, target_y, v_max=400, Kp=5.0, Ti=0.5, Td=0.05, N=10.0, i_limit=150, dist_frenado = 20.0):
    global x, y_pos, i_k, d_k_prev, y_meas_prev

    y_meas_prev = hub.motion_sensor.tilt_angles()[0] / 10.0
    i_k = 0.0
    d_k_prev = 0.0

    while True:
        dx, dy = target_x - x, target_y - y_pos
        distancia = math.sqrt(dx**2 + dy**2)
        angle_obj = math.degrees(math.atan2(dy, dx))
        y_k = hub.motion_sensor.tilt_angles()[0] / 10.0
        e_k = (angle_obj - y_k + 180) % 360 - 180

        # Validación de llegada con limites fijados por inercia
        if distancia < 3.5 and abs(e_k) < 7:
            motor.stop(PORT_IZQ, stop=motor.HOLD)
            motor.stop(PORT_DER, stop=motor.HOLD)
            break

        # PID de velocidad
        p_k = Kp * e_k
        i_k_fut = i_k + (Kp * T / Ti) * e_k
        i_k_fut = max(-i_limit, min(i_limit, i_k_fut))

        denom = (N * T) + Td
        d_k = (Td / denom) * d_k_prev + ((Kp * Td * N) / denom) * (y_k - y_meas_prev)

        u_unsat = p_k + i_k - d_k

        # Desaturación de PID e integrador
        u_k = max(-800, min(800, u_unsat))
        if abs(u_unsat) < 800: i_k = i_k_fut

        # Calculo condiciones de control de velocidad
        rampa = min(1.0, distancia / dist_frenado)
        v_lineal = v_max * rampa * math.cos(math.radians(e_k)) if abs(e_k) < 45 else 0

        motor.run(PORT_IZQ, -int(v_lineal - u_k))
        motor.run(PORT_DER, int(v_lineal + u_k))

        last_l, last_r = motor.relative_position(PORT_IZQ), motor.relative_position(PORT_DER)
        d_k_prev, y_meas_prev = d_k, y_k
        await runloop.sleep_ms(int(T * 1000))

        # Jazar equetion (kinematic)
        dl = (motor.relative_position(PORT_IZQ) - last_l) * -1
        dr = (motor.relative_position(PORT_DER) - last_r)
        dS = (R_RUEDA / 2.0) * (math.radians(dr) + math.radians(dl))
        phi = math.radians(hub.motion_sensor.tilt_angles()[0] / 10.0)
        x += dS * math.cos(phi)
        y_pos += dS * math.sin(phi)

    motor.stop(PORT_IZQ); motor.stop(PORT_DER)

"""
Tabla de datos estimativos para las constantes del PID

Tipo    | Kp            | Ti        | Td    | N        | dist_fenado| Notas        |
Movimiento|Agresividad    | Precisión | Amort | Filtro    |            | Imoportantes|
----------------------------------------------------------------------------------------------
 Recta    |            |        |    |        |            |Estabilidad    |
 lata /|3.5 - 4.5| 1.2 - 1.5 |0.04 |12 - 15|    15 cm    |lineal, evita|
 veloz    |            |        |    |        |            |oscilaciones a|
__________|_______________|___________|_______|___________|_______________|_alta_velocidad_|
Curva    |            |        |    |        |            | Alta fuerza de |
cerrada|6.0 - 7.5| 0.4 - 0.6 | 0.12|8 - 10|    30 cm    | giro, evita    |
(90°)    |            |        |    |        |            | coleo en curva |
----------------------------------------------------------------------------------------------
Maniobra|            |        |    |        |            | Elimina error| |
precisa|    5.0    |    0.3    | 0.08|5 - 8|    10 cm    | error de fric__|
        |            |        |    |        |            | ción, preciso|
----------------------------------------------------------------------------------------------
"""

async def mover_relativo(D, angulo_rel_deg, **kwargs):
    global x, y_pos

    heading = hub.motion_sensor.tilt_angles()[2] / 10.0
    angulo_global = heading + angulo_rel_deg

    dx = D * math.cos(math.radians(angulo_global))
    dy = D * math.sin(math.radians(angulo_global))

    target_x = x + dx
    target_y = y_pos + dy

    await navegar_pid_avanzado(target_x, target_y, **kwargs)

def calcular_rumbo(x_actual, y_actual, x_destino, y_destino):
    dx = x_destino - x_actual
    dy = y_destino - y_actual
    radianes = math.atan2(dy, dx)
    return math.degrees(radianes)

async def main():
    global x, y_pos
    hub.motion_sensor.reset_yaw(0)

    # ruta = [(50, 0, 400), (50, 50, 400), (0, 50, 400), (0, 0, 300)]
    ruta = [(80, 80, 390), (70, 120, 400), (20, 170, 400)]

    for punto in ruta:
        tx, ty, vmax = punto
        await navegar_pid_avanzado(tx, ty, v_max=vmax, Kp=4.5, Ti=0.29, Td=0.08, dist_frenado=12)
        angulo_necesario = calcular_rumbo(x, y_pos, tx, ty)
        await corregir_orientacion(angulo_necesario)
        await runloop.sleep_ms(50)

    await corregir_orientacion(45)

runloop.run(main())
