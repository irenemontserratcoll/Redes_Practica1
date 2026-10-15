import socket
import struct
import time
import math
# ===================== A5 (NUEVO) =====================
from UR3e_rg2.dynamic_ur3e import UR3eRG2
# =================== FIN A5 (NUEVO) ===================
# ============== CIERRE DE VENTANA (NUEVO) ==============
import pybullet as p        # solo para saber si la ventana sigue abierta
# ============ FIN CIERRE DE VENTANA (NUEVO) ============

ROBOT_IP = "10.172.18.90"
PUERTO = 30001
TAM_RECV = 4096             # bytes máximos por cada recv()
TAM_CABECERA = 5            # longitud (int, 4 B) + tipo (unsigned char, 1 B)
MAX_LONGITUD = 64 * 1024    # más de 64 kB -> buffer desalineado
TIPO_ESTADO = 16            # Robot State Message
TIPO_JOINT_DATA = 1         # subpaquete con los datos articulares
N_ARTICULACIONES = 6
FMT_ARTICULACION = ">dddffffB"  # q_actual, q_target, qd_actual, I, V, T_motor, T_micro, jointMode
TAM_ARTICULACION = struct.calcsize(FMT_ARTICULACION)              # 41 B
TAM_JOINT_DATA = TAM_CABECERA + N_ARTICULACIONES * TAM_ARTICULACION  # 251 B
# ============== ESTADO DEL ROBOT (NUEVO) ==============
# El controlador de URSim acepta conexiones y envía posiciones en cuanto
# arranca, ANTES de que se encienda el robot desde PolyScope (ON + START).
# Para no visualizar nada hasta entonces se lee el subpaquete 0
# (Robot mode data) y solo se mueve el robot simulado en modo RUNNING.
TIPO_ROBOT_MODE = 0
# Tras la cabecera (5 B): timestamp (uint64), isRealRobotConnected,
# isRealRobotEnabled, isRobotPowerOn, isEmergencyStopped,
# isProtectiveStopped, isProgramRunning, isProgramPaused (7 bool) y
# robotMode (unsigned char)
FMT_ROBOT_MODE = ">Q???????B"
ROBOT_MODE_RUNNING = 7
NOMBRES_MODO = {0: "DISCONNECTED", 1: "CONFIRM_SAFETY", 2: "BOOTING",
                3: "POWER_OFF", 4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE",
                7: "RUNNING", 8: "UPDATING_FIRMWARE", 255: "NO_CONTROLLER"}
# ============ FIN ESTADO DEL ROBOT (NUEVO) ============
# ===================== A5 (NUEVO) =====================
# ============== CIERRE DE URSIM (NUEVO) ==============
# Antes 5 s: a 10 Hz, 2 s sin ningún mensaje ya es una conexión perdida, y
# así se detecta antes que se ha cerrado URSim.
TIMEOUT = 2                 # s sin recibir nada -> se da la conexión por perdida
# Si pasan más de estos segundos sin datos (reintentos incluidos), se da por
# hecho que URSim se ha cerrado y el programa termina. Un reinicio de URSim
# más rápido que esto se reengancha solo, como pide la defensa.
TIEMPO_MAX_SIN_DATOS = 45
# ============ FIN CIERRE DE URSIM (NUEVO) ============
ESPERA_RECONEXION = 2       # s entre intentos de reconexión
ID_ROBOT_SIM = 0            # robot que crea UR3eRG2() en el origen
# Convenio de ángulos: si al comparar con URSim alguna articulación sale
# girada o con el signo cambiado, se corrige AQUÍ (nunca en el módulo).
# q_sim[j] = SIGNOS[j] * q_real[j] + OFFSETS[j]
SIGNOS = [1, 1, 1, 1, 1, 1]
OFFSETS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # rad
# =================== FIN A5 (NUEVO) ===================


def recorrer_subpaquetes(mensaje):
    """Recorre los subpaquetes de un mensaje de estado (tipo 16).

    Devuelve una lista de tuplas (tipo, tamaño, posición), una por subpaquete.
    Cada subpaquete empieza con la misma cabecera de 5 bytes que el mensaje
    (longitud + tipo), y la longitud incluye esa cabecera: así se puede
    saltar al siguiente sin saber qué contiene.
    """
    L = len(mensaje)
    subpaquetes = []
    posicion = TAM_CABECERA                 # saltar la cabecera del mensaje

    while posicion < L:
        # Hace falta al menos una cabecera completa de subpaquete
        if posicion + TAM_CABECERA > L:
            raise ValueError(f"Cabecera de subpaquete cortada en la posición {posicion}")

        tam, tipo = struct.unpack_from(">iB", mensaje, posicion)

        # Validaciones: tamaño imposible o subpaquete que se sale del mensaje
        if tam < TAM_CABECERA:
            raise ValueError(f"Subpaquete tipo {tipo} con tamaño imposible ({tam} B)")
        if posicion + tam > L:
            raise ValueError(f"Subpaquete tipo {tipo} ({tam} B) se sale del mensaje "
                             f"(posición {posicion}, mensaje de {L} B)")

        subpaquetes.append((tipo, tam, posicion))
        posicion += tam                     # saltar al siguiente subpaquete

    return subpaquetes


def leer_q_actual(mensaje, posicion, tam):
    """Decodifica el subpaquete Joint data y devuelve los 6 q_actual (rad).

    Después de la cabecera de 5 bytes vienen 6 bloques de 41 bytes, uno por
    articulación; q_actual es el primer double (8 B) de cada bloque.
    """
    if tam != TAM_JOINT_DATA:
        print(f"AVISO: Joint data mide {tam} B y se esperaban {TAM_JOINT_DATA} B "
              f"(¿otra versión de software?)")

    q = []
    for j in range(N_ARTICULACIONES):
        inicio_art = posicion + TAM_CABECERA + TAM_ARTICULACION * j
        q_actual = struct.unpack_from(FMT_ARTICULACION, mensaje, inicio_art)[0]
        q.append(q_actual)
    return q


# ============== ESTADO DEL ROBOT (NUEVO) ==============
def leer_robot_mode(mensaje, posicion, tam):
    """Decodifica el subpaquete Robot mode data (tipo 0) y devuelve un dict
    con los campos útiles (también servirán en el bloque D)."""
    if tam < TAM_CABECERA + struct.calcsize(FMT_ROBOT_MODE):
        raise ValueError(f"Robot mode data de {tam} B: demasiado corto")
    (_, conectado, habilitado, encendido, emergencia, parada_proteccion,
     programa_en_marcha, programa_pausado, modo) = struct.unpack_from(
        FMT_ROBOT_MODE, mensaje, posicion + TAM_CABECERA)
    return {"isRobotPowerOn": encendido,
            "isEmergencyStopped": emergencia,
            "isProtectiveStopped": parada_proteccion,
            "isProgramRunning": programa_en_marcha,
            "robotMode": modo}


def nombre_modo(modo):
    return NOMBRES_MODO.get(modo, f"desconocido ({modo})")
# ============ FIN ESTADO DEL ROBOT (NUEVO) ============


# ============== CIERRE DE VENTANA (NUEVO) ==============
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


# ===================== A5 (NUEVO) =====================
def a_convenio_simulador(q):
    """Aplica signo y offset de cada articulación (ver SIGNOS y OFFSETS)."""
    return [SIGNOS[j] * q[j] + OFFSETS[j] for j in range(N_ARTICULACIONES)]


def visualizar(robot_sim):
    """Abre una conexión con el robot y mueve el robot simulado con cada
    mensaje de estado. Cuando la conexión se pierde, avisa del motivo y
    devuelve el instante del último mensaje recibido (None si no llegó
    ninguno), para que main() decida si reconectar o terminar.
    """
    # ============== SIN VARIABLES GLOBALES ==============
    # Antes era una variable global; ahora es local y se devuelve con return.
    # Los except que estaban en main() pasan aquí para poder devolverla
    # también cuando la conexión se corta con una excepción.
    t_ultimo_dato = None
    try:
    # ============ FIN SIN VARIABLES GLOBALES ============
        print(f"Conectando a {ROBOT_IP}:{PUERTO} ...")
        with socket.create_connection((ROBOT_IP, PUERTO), timeout=TIMEOUT) as sock:
            print("Conexión TCP establecida (saludo de tres vías completado).")
# =================== FIN A5 (NUEVO) ===================

            # Los bytes se acumulan en un buffer y se cortan los mensajes usando
            # la longitud L que cada uno lleva en sus 4 primeros bytes.
            buffer = bytearray()
            # ============== ESTADO DEL ROBOT (NUEVO) ==============
            modo_anterior = None        # para avisar solo cuando cambia el modo
            # ============ FIN ESTADO DEL ROBOT (NUEVO) ============

            # ===================== A5 (NUEVO) =====================
            # Ya no hay DURACION: se lee indefinidamente hasta perder la conexión
            # o hasta que el usuario pulse Ctrl+C.
            while True:
            # =================== FIN A5 (NUEVO) ===================
                # Paso 1: recv() y añadir al final del buffer
                datos = sock.recv(TAM_RECV)
                if not datos:                   # recv() == b"" -> el robot cerró la conexión
                    print("El robot ha cerrado la conexión.")
                    return t_ultimo_dato
                buffer += datos

                # Paso 2: mientras haya al menos una cabecera completa...
                while len(buffer) >= TAM_CABECERA:
                    # ...leer L y T SIN quitarlos del buffer
                    L, T = struct.unpack_from(">iB", buffer, 0)

                    # Validación: longitud imposible -> buffer desalineado
                    if L < TAM_CABECERA or L > MAX_LONGITUD:
                        raise ValueError(f"Longitud imposible ({L} B): buffer desalineado")

                    # Paso 3: el mensaje aún no ha llegado entero -> más recv()
                    if len(buffer) < L:
                        break

                    # Paso 4: cortar el mensaje y quitarlo del buffer
                    mensaje = bytes(buffer[:L])
                    del buffer[:L]
                    # ============== CIERRE DE URSIM (NUEVO) ==============
                    # Ha llegado un mensaje completo: el robot sigue vivo
                    t_ultimo_dato = time.time()
                    # ============ FIN CIERRE DE URSIM (NUEVO) ============

                    # Paso 5: si es un mensaje de estado, recorrer sus subpaquetes;
                    # cualquier otro tipo se ignora (ya está fuera del buffer).
                    if T == TIPO_ESTADO:
                        # ============== ESTADO DEL ROBOT (NUEVO) ==============
                        # Primero se recorren TODOS los subpaquetes guardando lo
                        # que interesa, y después se decide qué hacer (antes se
                        # movía el robot directamente al encontrar el tipo 1).
                        modo, q = None, None
                        for tipo_sp, tam_sp, pos_sp in recorrer_subpaquetes(mensaje):
                            if tipo_sp == TIPO_ROBOT_MODE:
                                modo = leer_robot_mode(mensaje, pos_sp, tam_sp)
                            elif tipo_sp == TIPO_JOINT_DATA:
                                q = leer_q_actual(mensaje, pos_sp, tam_sp)

                        # Avisar por consola solo cuando cambia el modo del robot
                        if modo is not None and modo["robotMode"] != modo_anterior:
                            modo_anterior = modo["robotMode"]
                            print(f"Modo del robot: {nombre_modo(modo_anterior)}")
                            if modo_anterior != ROBOT_MODE_RUNNING:
                                print("  -> esperando a que se encienda el robot "
                                      "(ON + START en PolyScope)")

                        # Solo se visualiza con el robot en RUNNING
                        robot_listo = modo is not None and modo["robotMode"] == ROBOT_MODE_RUNNING
                        if q is not None and robot_listo:
                        # ============ FIN ESTADO DEL ROBOT (NUEVO) ============

                            # ===================== A5 (NUEVO) =====================
                            # move_robot() necesita una LISTA en radianes y la
                            # modifica (le añade 6 valores): se le pasa una copia.
                            # (CIERRE DE VENTANA: antes robot_sim.move_robot(...);
                            # ahora mover_robot_sim() comprueba la ventana)
                            mover_robot_sim(robot_sim, ID_ROBOT_SIM, list(a_convenio_simulador(q)))
                            # =================== FIN A5 (NUEVO) ===================

                            # El robot envía q_actual en RADIANES; los grados
                            # solo se calculan para compararlos con PolyScope
                            grados = "  ".join(f"{math.degrees(a):8.2f}" for a in q)
                            print(f"q_actual (º): {grados}")

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


# ===================== A5 (NUEVO) =====================
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
        # instante del último mensaje (None si en esta conexión no llegó nada)
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
# =================== FIN A5 (NUEVO) ===================
