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

Tipo    | Kp        | Ti        | Td    | N        | dist    | Notas        |
Movimiento|Agresividad| Precisión | Amort | Filtro|frenado| Imoportantes|
----------------------------------------------------------------------------------------------
 Recta    |            |        |    |        |            |Estabilidad    |
 lata /| 3.5 - 4.5| 1.2 - 1.5 |0.04 |12 - 15 |    15 cm|lineal, evita|
 veloz    |            |        |    |        |            |oscilaciones a|
__________|_____________|___________|_______|__________|____________|_alta_velocidad_|
Curva    |            |            |    |        |            | Alta fuerza de |
cerrada| 6.0 - 7.5| 0.4 - 0.6| 0.12| 8 - 10|    30 cm| giro, evita    |
(90°)    |            |            |    |        |            | coleo en curva |
----------------------------------------------------------------------------------------------
Maniobra|            |            |    |        |            | Elimina error|
precisa|    5.0    |    0.3    | 0.08|5 - 8|    10 cm| error de fric_ |
        |            |            |    |        |            | ción, preciso|
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

async def sensado():
    print("Posición actual:", x, y_pos)
    angulo = hub.motion_sensor.tilt_angles()[0]
    print("Ángulo actual:", angulo)
    angulo2 =hub.motion_sensor.reset_yaw(0)
    print("reset °: ", angulo2)

async def main():

    hub.motion_sensor.reset_yaw(0)

    await motor.run_for_degrees(hub.port.F, 56, 300)

    await runloop.sleep_ms(200)

    # Ejecutar solo el lanzamiento que quieras probar
    await lanzamiento_1()

    # await lanzamiento_2()
    # await lanzamiento_3()

    runloop.run(main())

async def lanzamiento_1():
    global x, y_pos
    hub.motion_sensor.reset_yaw(0)

    ruta = [(34, 12, 600), (63.5, 12.8, 600), (77.5, -11, 600), (90, -10, 600)]

    for i, punto in enumerate(ruta):

        if i == 0:
            tx, ty, vmax = punto
            await navegar_pid_avanzado(tx,-ty, v_max=vmax, Kp=4.5, Ti=1.5, Td=0.08, dist_frenado=15)
            await corregir_orientacion(-1)
            await runloop.sleep_ms(1000)
            await sensado()
         
        if i == 1:
            tx, ty, vmax = punto
            await navegar_pid_avanzado(tx,-ty, v_max=vmax, Kp=4.5, Ti=0.29, Td=0.08, dist_frenado=12)
            await runloop.sleep_ms(1000)
            await reto_1()
            await sensado()

        if i == 2:
            tx, ty, vmax = punto
            await navegar_pid_avanzado(tx, ty, v_max=vmax, Kp=4.5, Ti=0.29, Td=0.08, dist_frenado=12)
            await runloop.sleep_ms(50)
            await reto_2()
            await sensado()

        if i == 3:
            tx, ty, vmax = punto
            await navegar_pid_avanzado(tx, ty, v_max=vmax, Kp=4.5, Ti=1.5, Td=0.04, dist_frenado=15)
            await runloop.sleep_ms(50)
            await reto_3()
            await sensado()
        
       
        
async def reto_1(dist_cm=5, grados_motor=180, velocidad=900):
    global x, y_pos

    # 1. Bloquear el ángulo actual como "Rumbo Objetivo" para NO reorientarse
    target_angle = hub.motion_sensor.tilt_angles()[0] / 10.0

    # Guardar las posiciones iniciales de los motores
    last_l = motor.relative_position(PORT_IZQ)
    last_r = motor.relative_position(PORT_DER)

    distancia_recorrida = 0.0
    Kp_estabilizacion = 4.5# Mismo valor de tu P para corregir micro-desvíos

    # Velocidad lineal negativa para obligar al robot a ir marcha atrás
    v_lineal = -velocidad

    # 2. Bucle de control dinámico: Se ejecuta hasta cumplir los cm deseados
    while distancia_recorrida < dist_cm:

        # --- ODOMETRÍA ACTIVA (Rastreo de posición real) ---
        current_l = motor.relative_position(PORT_IZQ)
        current_r = motor.relative_position(PORT_DER)

        dl = (current_l - last_l) * -1
        dr = (current_r - last_r)

        # Cálculo del diferencial de distancia en cm
        dS = (R_RUEDA / 2.0) * (math.radians(dr) + math.radians(dl))
        phi = math.radians(hub.motion_sensor.tilt_angles()[0] / 10.0)

        # Actualizamos la posición global del sistema en tiempo real
        x += dS * math.cos(phi)
        y_pos += dS * math.sin(phi)

        # Acumulamos el valor absoluto de la distancia recorrida
        distancia_recorrida += abs(dS)

        # Actualizar lecturas previas
        last_l, last_r = current_l, current_r

        # --- CONTROL DE ESTABILIDAD (Mantener la orientación fija) ---
        y_k = hub.motion_sensor.tilt_angles()[0] / 10.0
        e_k = (target_angle - y_k + 180) % 360 - 180
        u_k = Kp_estabilizacion * e_k# Corrección de giro si el robot se enchueca

        # Actuación de motores: Mantiene la marcha atrás aplicando el diferencial de corrección
        motor.run(PORT_IZQ, -int(v_lineal - u_k))
        motor.run(PORT_DER, int(v_lineal + u_k))

        # Esperar el tiempo de muestreo del sistema (T = 0.015s -> 15ms)
        await runloop.sleep_ms(int(T * 1000))

    # 3. Frenado inmediato al cumplir la distancia
    motor.stop(PORT_IZQ, stop=motor.HOLD)
    motor.stop(PORT_DER, stop=motor.HOLD)
    await runloop.sleep_ms(200)

    # --- Continuación de tus retos con el motor accesorio F ---
    await motor.run_for_degrees(hub.port.F, grados_motor, 250)
    await runloop.sleep_ms(200)

    await corregir_orientacion(-1)
    await runloop.sleep_ms(800)

    await motor.run_for_degrees(hub.port.F, -180, 700)
    await runloop.sleep_ms(200)

    await motor.run_for_degrees(hub.port.F, 100, 250)
    await runloop.sleep_ms(200)


async def reto_2(dist_cm=16, grados_motor=180, velocidad=400):
    
    grados = int((dist_cm / (2 * math.pi * R_RUEDA)) * 360) #dist_avanzado
    grados2 = int((11 / (2 * math.pi * R_RUEDA)) * 360) #dist_retroceso 

    await motor.run_for_degrees(hub.port.F, -50, 250)
    await runloop.sleep_ms(500)

    await corregir_orientacion(45)
    await runloop.sleep_ms(1000)

    await motor.run_for_degrees(hub.port.D, -115, 100)
    await runloop.sleep_ms(100)

    #adelante
    motor.run_for_degrees(PORT_IZQ, -grados, velocidad)
    motor.run_for_degrees(PORT_DER, grados, velocidad)
    await runloop.sleep_ms(1000)

    await motor.run_for_degrees(hub.port.D, 28, 250)

    #atras
    motor.run_for_degrees(PORT_IZQ, grados2, velocidad)
    motor.run_for_degrees(PORT_DER, -grados2, velocidad)

    await motor.run_for_degrees(hub.port.F, -100, 250)
    await runloop.sleep_ms(1000)
    
    await corregir_orientacion(-60)
    await runloop.sleep_ms(1000)
    
async def reto_3():
    int = 1

runloop.run(main())
