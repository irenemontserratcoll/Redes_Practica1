import socket
import struct
import time
import math
from UR3e_rg2.dynamic_ur3e import UR3eRG2
# ============== CIERRE DE VENTANA (NUEVO) ==============
import pybullet as p        # solo para saber si la ventana sigue abierta
# ============ FIN CIERRE DE VENTANA (NUEVO) ============

ROBOT_IP = "10.172.18.90"
# ============ B1 (DIFERENTE de primaria.py) ============
# Puerto de la interfaz de tiempo real (en primaria.py: 30001)
PUERTO = 30003
# ========== FIN B1 (DIFERENTE de primaria.py) ==========
# ============ B4 (DIFERENTE de primaria.py) ============
# Buffer de recv() más grande (en primaria.py: 4096): a 500 Hz llegan
# ~600 kB/s, y si el dibujo tarda un poco se acumulan bytes en el socket.
# Con recv() más grandes se vacía ese atraso en menos vueltas del bucle.
TAM_RECV = 65536            # bytes máximos por cada recv()
# ========== FIN B4 (DIFERENTE de primaria.py) ==========
# ============ B1 (DIFERENTE de primaria.py) ============
# Cabecera de 4 B: solo la longitud (int). No hay campo de tipo porque
# solo existe un tipo de paquete (en primaria.py: 5 B, longitud + tipo).
TAM_CABECERA = 4
# ========== FIN B1 (DIFERENTE de primaria.py) ==========
MAX_LONGITUD = 64 * 1024    # más de 64 kB -> buffer desalineado
# ============== CIERRE DE URSIM (NUEVO) ==============
# Igual que en primaria.py. Antes 5 s: a 500 Hz, 2 s sin ningún paquete ya
# es una conexión perdida, y así se detecta antes que se ha cerrado URSim.
TIMEOUT = 2                 # s sin recibir nada -> se da la conexión por perdida
# Si pasan más de estos segundos sin datos (reintentos incluidos), se da por
# hecho que URSim se ha cerrado y el programa termina. Un reinicio de URSim
# más rápido que esto se reengancha solo, como pide la defensa.
TIEMPO_MAX_SIN_DATOS = 15
# ============ FIN CIERRE DE URSIM (NUEVO) ============
ESPERA_RECONEXION = 2       # s entre intentos de reconexión
N_ARTICULACIONES = 6
ID_ROBOT_SIM = 0            # robot que crea UR3eRG2() en el origen
# Convenio de ángulos: si al comparar con URSim alguna articulación sale
# girada o con el signo cambiado, se corrige AQUÍ (nunca en el módulo).
# q_sim[j] = SIGNOS[j] * q_real[j] + OFFSETS[j]
SIGNOS = [1, 1, 1, 1, 1, 1]
OFFSETS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # rad
# ============ B3 (DIFERENTE de primaria.py) ============
# q_actual está en una posición FIJA del paquete: 6 doubles seguidos a
# partir del byte 252 (en primaria.py había que buscar el subpaquete 1 y
# saltar bloques de 41 B por articulación).
OFFSET_Q_ACTUAL = 252
FMT_Q_ACTUAL = ">6d"                                    # 6 doubles big-endian
TAM_MIN_PAQUETE = OFFSET_Q_ACTUAL + struct.calcsize(FMT_Q_ACTUAL)  # 300 B
# ========== FIN B3 (DIFERENTE de primaria.py) ==========
# ============== ESTADO DEL ROBOT (NUEVO) ==============
# Igual que en primaria.py: solo se visualiza con el robot en RUNNING.
# Diferencia: aquí no hay subpaquete 0; el modo viaja como un double en una
# posición FIJA (comprobadlo en la hoja Client Interfaces, pestaña RealTime).
OFFSET_ROBOT_MODE = 756
FMT_ROBOT_MODE = ">d"
TAM_MIN_ROBOT_MODE = OFFSET_ROBOT_MODE + struct.calcsize(FMT_ROBOT_MODE)  # 764 B
ROBOT_MODE_RUNNING = 7
NOMBRES_MODO = {0: "DISCONNECTED", 1: "CONFIRM_SAFETY", 2: "BOOTING",
                3: "POWER_OFF", 4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE",
                7: "RUNNING", 8: "UPDATING_FIRMWARE", -1: "NO_CONTROLLER"}
# ============ FIN ESTADO DEL ROBOT (NUEVO) ============
# ============ B4 (DIFERENTE de primaria.py) ============
# Ritmo de dibujo: se separa la RECEPCIÓN (500 Hz, todos los paquetes) del
# DIBUJO (30 Hz, solo el último q). En primaria.py se dibujaba con cada
# mensaje porque solo llegan 10 por segundo.
FREC_DIBUJO = 30                     # actualizaciones de PyBullet por segundo
PERIODO_DIBUJO = 1.0 / FREC_DIBUJO   # s entre dos move_robot()
# ========== FIN B4 (DIFERENTE de primaria.py) ==========


# ============ B3 (DIFERENTE de primaria.py) ============
# Equivale a leer_q_actual() de primaria.py, pero sin recorrer_subpaquetes():
# basta con saltar directamente al desplazamiento de q_actual.
def leer_q_actual(paquete):
    """Devuelve los 6 q_actual (rad) de un paquete de tiempo real."""
    if len(paquete) < TAM_MIN_PAQUETE:
        raise ValueError(f"Paquete de {len(paquete)} B: demasiado corto para "
                         f"contener q_actual (mínimo {TAM_MIN_PAQUETE} B)")
    return list(struct.unpack_from(FMT_Q_ACTUAL, paquete, OFFSET_Q_ACTUAL))
# ========== FIN B3 (DIFERENTE de primaria.py) ==========


# ============== ESTADO DEL ROBOT (NUEVO) ==============
# Equivale a leer_robot_mode() de primaria.py, pero leyendo de posición fija
def leer_robot_mode(paquete):
    """Devuelve el robotMode (entero) de un paquete de tiempo real."""
    if len(paquete) < TAM_MIN_ROBOT_MODE:
        raise ValueError(f"Paquete de {len(paquete)} B: demasiado corto para "
                         f"contener Robot Mode (mínimo {TAM_MIN_ROBOT_MODE} B)")
    # Viaja como double aunque sea un entero: se convierte con int()
    return int(struct.unpack_from(FMT_ROBOT_MODE, paquete, OFFSET_ROBOT_MODE)[0])


def nombre_modo(modo):
    return NOMBRES_MODO.get(modo, f"desconocido ({modo})")
# ============ FIN ESTADO DEL ROBOT (NUEVO) ============


# ============== CIERRE DE VENTANA (NUEVO) ==============
# Igual que en primaria.py
class VentanaCerrada(Exception):
    """Se ha cerrado la ventana de PyBullet: el programa debe terminar."""


def comprobar_ventana():
    """Lanza VentanaCerrada si el usuario ha cerrado la ventana de PyBullet.
    Sin esto, el bucle de recepción seguiría para siempre aunque ya no
    haya nada que dibujar."""
    if not p.isConnected():
        raise VentanaCerrada()


def mover_robot_sim(robot_sim, id_robot, q):
    """move_robot() comprobando antes que la ventana sigue abierta."""
    comprobar_ventana()
    try:
        robot_sim.move_robot(id_robot, q)
    except p.error:                         # la ventana se cerró justo ahora
        raise VentanaCerrada()


def esperar(segundos):
    """time.sleep() en trozos de 0,1 s, vigilando la ventana mientras tanto
    (para que al cerrarla durante una espera de reconexión se detecte ya)."""
    fin = time.time() + segundos
    while time.time() < fin:
        comprobar_ventana()
        time.sleep(0.1)
# ============ FIN CIERRE DE VENTANA (NUEVO) ============


def a_convenio_simulador(q):
    """Aplica signo y offset de cada articulación (ver SIGNOS y OFFSETS)."""
    return [SIGNOS[j] * q[j] + OFFSETS[j] for j in range(N_ARTICULACIONES)]


def visualizar(robot_sim):
    """Abre una conexión con el robot y mueve el robot simulado con el
    último estado recibido. Cuando la conexión se pierde, avisa del motivo y
    devuelve el instante del último paquete recibido (None si no llegó
    ninguno), para que main() decida si reconectar o terminar.
    """
    # ============== SIN VARIABLES GLOBALES ==============
    # Igual que en primaria.py: antes era una variable global; ahora es local
    # y se devuelve con return. Los except que estaban en main() pasan aquí
    # para poder devolverla también cuando la conexión se corta con una
    # excepción.
    t_ultimo_dato = None
    try:
    # ============ FIN SIN VARIABLES GLOBALES ============
        print(f"Conectando a {ROBOT_IP}:{PUERTO} ...")
        with socket.create_connection((ROBOT_IP, PUERTO), timeout=TIMEOUT) as sock:
            print("Conexión TCP establecida (saludo de tres vías completado).")

            # Los bytes se acumulan en un buffer y se cortan los paquetes usando
            # la longitud L que cada uno lleva en sus 4 primeros bytes.
            buffer = bytearray()
            # ============ B1 (DIFERENTE de primaria.py) ============
            # Para comprobar el B1: longitud del primer paquete y nº de paquetes
            longitud_esperada = None
            n_paquetes = 0
            # ========== FIN B1 (DIFERENTE de primaria.py) ==========
            # ============ B2 (DIFERENTE de primaria.py) ============
            # Contador de paquetes del segundo en curso
            paquetes_segundo = 0
            t_segundo = time.time()
            # ========== FIN B2 (DIFERENTE de primaria.py) ==========
            # ============ B3 (DIFERENTE de primaria.py) ============
            q = None                    # último q_actual decodificado
            # ========== FIN B3 (DIFERENTE de primaria.py) ==========
            # ============== ESTADO DEL ROBOT (NUEVO) ==============
            modo = None                 # último robotMode decodificado
            modo_anterior = None        # para avisar solo cuando cambia el modo
            # ============ FIN ESTADO DEL ROBOT (NUEVO) ============
            # ============ B4 (DIFERENTE de primaria.py) ============
            t_ultimo_dibujo = 0.0       # instante del último move_robot()
            dibujos_segundo = 0         # move_robot() hechos en el segundo en curso
            # ========== FIN B4 (DIFERENTE de primaria.py) ==========

            # Se lee indefinidamente hasta perder la conexión o hasta Ctrl+C.
            while True:
                # Paso 1: recv() y añadir al final del buffer
                datos = sock.recv(TAM_RECV)
                if not datos:                   # recv() == b"" -> el robot cerró la conexión
                    print("El robot ha cerrado la conexión.")
                    return t_ultimo_dato
                buffer += datos

                # Paso 2: mientras haya al menos una cabecera completa...
                while len(buffer) >= TAM_CABECERA:
                    # ============ B1 (DIFERENTE de primaria.py) ============
                    # ...leer solo L SIN quitarla del buffer (no hay tipo T;
                    # en primaria.py: L, T = struct.unpack_from(">iB", ...))
                    (L,) = struct.unpack_from(">i", buffer, 0)
                    # ========== FIN B1 (DIFERENTE de primaria.py) ==========

                    # Validación: longitud imposible -> buffer desalineado
                    if L < TAM_CABECERA or L > MAX_LONGITUD:
                        raise ValueError(f"Longitud imposible ({L} B): buffer desalineado")

                    # Paso 3: el paquete aún no ha llegado entero -> más recv()
                    if len(buffer) < L:
                        break

                    # Paso 4: cortar el paquete y quitarlo del buffer
                    paquete = bytes(buffer[:L])
                    del buffer[:L]
                    # ============== CIERRE DE URSIM (NUEVO) ==============
                    # Ha llegado un paquete completo: el robot sigue vivo
                    t_ultimo_dato = time.time()
                    # ============ FIN CIERRE DE URSIM (NUEVO) ============

                    # ============ B1 (DIFERENTE de primaria.py) ============
                    # Paso 5: no hay tipos que filtrar (en primaria.py: if T == 16).
                    # A 500 Hz no se puede imprimir cada paquete: la consola no da
                    # abasto y el programa se quedaría atrás. Solo se imprime la
                    # longitud del primero y se avisa si alguno es distinto.
                    n_paquetes += 1
                    if longitud_esperada is None:
                        longitud_esperada = L
                        print(f"Primer paquete: longitud = {L} B")
                    elif L != longitud_esperada:
                        print(f"AVISO: paquete #{n_paquetes} mide {L} B "
                              f"y se esperaban {longitud_esperada} B")
                    # ========== FIN B1 (DIFERENTE de primaria.py) ==========
                    # ============ B2 (DIFERENTE de primaria.py) ============
                    paquetes_segundo += 1
                    # ========== FIN B2 (DIFERENTE de primaria.py) ==========
                    # ============ B3 (DIFERENTE de primaria.py) ============
                    # Se decodifica cada paquete, pero solo se guarda el último q
                    q = leer_q_actual(paquete)
                    # ========== FIN B3 (DIFERENTE de primaria.py) ==========
                    # ============== ESTADO DEL ROBOT (NUEVO) ==============
                    modo = leer_robot_mode(paquete)
                    if modo != modo_anterior:
                        modo_anterior = modo
                        print(f"Modo del robot: {nombre_modo(modo)}")
                        if modo != ROBOT_MODE_RUNNING:
                            print("  -> esperando a que se encienda el robot "
                                  "(ON + START en PolyScope)")
                    # ============ FIN ESTADO DEL ROBOT (NUEVO) ============

                # ============ B4 (DIFERENTE de primaria.py) ============
                # Dibujo a ritmo fijo, FUERA del bucle de paquetes: ya se han
                # consumido todos los paquetes completos del buffer y q es el más
                # reciente. Los intermedios se descartan, así que el robot
                # simulado nunca acumula retraso (en primaria.py, move_robot()
                # se llamaba dentro del bucle, con cada mensaje).
                ahora = time.time()
                # ============== CIERRE DE VENTANA (NUEVO) ==============
                # Se comprueba en cada vuelta, no solo al dibujar
                comprobar_ventana()
                # ============ FIN CIERRE DE VENTANA (NUEVO) ============
                # ============== ESTADO DEL ROBOT (NUEVO) ==============
                # Solo se visualiza con el robot en RUNNING
                robot_listo = modo == ROBOT_MODE_RUNNING
                # ============ FIN ESTADO DEL ROBOT (NUEVO) ============
                # (ESTADO DEL ROBOT: se añade "robot_listo and")
                if robot_listo and q is not None and ahora - t_ultimo_dibujo >= PERIODO_DIBUJO:
                    # move_robot() necesita una LISTA en radianes y la
                    # modifica (le añade 6 valores): se le pasa una copia.
                    # (CIERRE DE VENTANA: antes robot_sim.move_robot(...))
                    mover_robot_sim(robot_sim, ID_ROBOT_SIM, list(a_convenio_simulador(q)))
                    t_ultimo_dibujo = ahora
                    dibujos_segundo += 1
                # ========== FIN B4 (DIFERENTE de primaria.py) ==========

                # ============ B2 (DIFERENTE de primaria.py) ============
                # Una vez por segundo se imprime cuántos paquetes han llegado
                # (sustituye al "cada 500 paquetes" del B1). Se divide entre el
                # tiempo real transcurrido porque nunca es exactamente 1,000 s.
                if ahora - t_segundo >= 1.0:
                    frecuencia = paquetes_segundo / (ahora - t_segundo)
                    print(f"{paquetes_segundo:4d} paquetes en el último segundo "
                          f"(~{frecuencia:.0f} Hz) | total: {n_paquetes} | "
                          f"longitud: {longitud_esperada} B")
                    # ============ B4 (DIFERENTE de primaria.py) ============
                    # Comprobación del B4: cuántas veces se ha dibujado y cuántos
                    # bytes esperan en el buffer (si crece, vamos con retraso)
                    print(f"     dibujos: {dibujos_segundo} | "
                          f"bytes pendientes en el buffer: {len(buffer)}")
                    dibujos_segundo = 0
                    # ========== FIN B4 (DIFERENTE de primaria.py) ==========
                    # ============ B3 (DIFERENTE de primaria.py) ============
                    # Mismo print que primaria.py, pero 1 vez por segundo (no
                    # 500): el robot envía q_actual en RADIANES; los grados solo
                    # se calculan para compararlos con PolyScope / primaria.py
                    # (ESTADO DEL ROBOT: se añade "robot_listo and")
                    if robot_listo and q is not None:
                        grados = "  ".join(f"{math.degrees(a):8.2f}" for a in q)
                        print(f"q_actual (º): {grados}")
                    # ========== FIN B3 (DIFERENTE de primaria.py) ==========
                    paquetes_segundo = 0
                    t_segundo = ahora
                # ========== FIN B2 (DIFERENTE de primaria.py) ==========

    # ============== SIN VARIABLES GLOBALES ==============
    # Los mismos except que antes estaban en main()
    except ConnectionRefusedError:
        print("Conexión rechazada (RST): el equipo responde pero nadie escucha en ese puerto.")
    except socket.timeout:
        print(f"Tiempo agotado ({TIMEOUT} s sin datos): IP errónea, VM apagada o cortafuegos.")
    except OSError as e:
        print(f"Error de red: {e}")
    except ValueError as e:
        print(f"Error de protocolo: {e} -> se cierra la conexión y se reabre")
    return t_ultimo_dato
    # ============ FIN SIN VARIABLES GLOBALES ============


def main():
    # PyBullet admite una única ventana por proceso: se crea UNA vez, fuera
    # del bucle de reconexión, y se reutiliza en cada conexión nueva.
    robot_sim = UR3eRG2()
    # ============== CIERRE DE URSIM (NUEVO) ==============
    # Se cuenta desde el arranque: si URSim no está encendido al lanzar el
    # programa, también termina tras TIEMPO_MAX_SIN_DATOS.
    # (SIN VARIABLES GLOBALES: ahora es una variable local de main())
    t_ultimo_dato = time.time()
    # ============ FIN CIERRE DE URSIM (NUEVO) ============

    # Bucle de reconexión: si URSim se reinicia o se cae la red, el programa
    # no se cuelga; avisa y vuelve a intentarlo (hasta TIEMPO_MAX_SIN_DATOS).
    while True:
        # ============== SIN VARIABLES GLOBALES ==============
        # visualizar() ya avisa del motivo de la desconexión y devuelve el
        # instante del último paquete (None si en esta conexión no llegó nada)
        t = visualizar(robot_sim)
        if t is not None:
            t_ultimo_dato = t
        # ============ FIN SIN VARIABLES GLOBALES ============

        # ============== CIERRE DE URSIM (NUEVO) ==============
        sin_datos = time.time() - t_ultimo_dato
        if sin_datos > TIEMPO_MAX_SIN_DATOS:
            print(f"{sin_datos:.0f} s sin datos del robot: se da por hecho que "
                  f"URSim se ha cerrado. Programa terminado.")
            return
        # ============ FIN CIERRE DE URSIM (NUEVO) ============
        print(f"Reintentando en {ESPERA_RECONEXION} s ... (Ctrl+C para salir)")
        # ============== CIERRE DE VENTANA (NUEVO) ==============
        esperar(ESPERA_RECONEXION)          # antes: time.sleep(ESPERA_RECONEXION)
        # ============ FIN CIERRE DE VENTANA (NUEVO) ============


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nPrograma terminado por el usuario.")
    # ============== CIERRE DE VENTANA (NUEVO) ==============
    except VentanaCerrada:
        print("\nVentana de PyBullet cerrada: programa terminado.")
    # ============ FIN CIERRE DE VENTANA (NUEVO) ============
