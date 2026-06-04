import motor
import hub
import runloop

async def main():
    # Diccionario de traducción: entero nativo a nomenclatura física del Hub
    MAPA_PUERTOS = {
        0: "A",
        1: "B",
        2: "C",
        3: "D",
        4: "E",
        5: "F"
    }

    # Lista de puertos (evaluados internamente como enteros: 0, 1, 2, 3, 4, 5)
    puertos = [hub.port.A, hub.port.B, hub.port.C, hub.port.D, hub.port.E, hub.port.F]
    motores_detectados = []

    print("Iniciando escaneo de puertos...")

    # Fase 1: Identificación de motores vinculados
    for puerto in puertos:
        try:
            posicion_actual = motor.absolute_position(puerto)
            motores_detectados.append(puerto)
            letra = MAPA_PUERTOS.get(puerto, str(puerto))
            print("Motor detectado en el Puerto {} | Angulo inicial: {} grados".format(letra, posicion_actual))
        except Exception:
            # Si el puerto no contiene un motor, se ignora la excepción
            pass

    if not motores_detectados:
        print("Escaneo finalizado. No se detectaron motores en el Hub.")
        return

    print("\nMotores listos para calibracion: {}. Iniciando secuencia...".format(len(motores_detectados)))

    # Fase 2: Alineación asíncrona con la firma de 3 argumentos exactos
    for puerto in motores_detectados:
        letra = MAPA_PUERTOS.get(puerto, str(puerto))

        try:
            pos_antes = motor.absolute_position(puerto)
            print("Moviendo motor del Puerto {} desde {} grados hacia 0...".format(letra, pos_antes))

            # SOLUCIÓN: Se reducen los parámetros a los 3 requeridos por el firmware:
            # 1. puerto (int) -> Identificador del canal del motor.
            # 2. posicion (int) -> 0 (Objetivo absoluto).
            # 3. velocidad (int) -> 360 (Grados por segundo).
            # La dirección (camino más corto) la gestiona el Hub de forma nativa.
            await motor.run_to_absolute_position(puerto, 0, 360)

            pos_despues = motor.absolute_position(puerto)
            print("Calibracion exitosa en Puerto {}. Posicion actual: {} grados".format(letra, pos_despues))

        except Exception as e:
            print("Imposible calibrar el motor en el Puerto {}. Razon: {}".format(letra, e))

    print("\nTodos los procesos de alineacion han concluido.")

# Ejecución del bucle asíncrono
runloop.run(main())
