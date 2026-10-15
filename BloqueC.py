import socket
import struct
import time
from UR3e_rg2.dynamic_ur3e import UR3eRG2
# ============ C (DIFERENTE de primaria.py) ============
import sys
import os
import json
import threading
import pybullet as p        # solo para escribir el estado encima de cada robot

# Se REUTILIZAN tal cual las constantes y funciones de primaria.py: el
# formato de los mensajes es el mismo, solo cambia que hay varios robots.
from primaria import (TAM_RECV, TAM_CABECERA, MAX_LONGITUD, TIPO_ESTADO,
                      TIPO_JOINT_DATA, TIMEOUT, ESPERA_RECONEXION,
                      recorrer_subpaquetes, leer_q_actual, a_convenio_simulador,
                      # CIERRE DE VENTANA (NUEVO): también se reutilizan
                      VentanaCerrada, comprobar_ventana, mover_robot_sim,
                      # CIERRE DE URSIM (NUEVO)
                      TIEMPO_MAX_SIN_DATOS,
                      # ESTADO DEL ROBOT (NUEVO)
                      TIPO_ROBOT_MODE, ROBOT_MODE_RUNNING, leer_robot_mode, nombre_modo)

# La lista de robots (IP, puerto, posición en la escena) se lee de un fichero
# de configuración, no está en el código (en primaria.py: ROBOT_IP y PUERTO).
# Uso:  python BloqueC.py [fichero.json]   (por defecto robots.json)
FICHERO_CONFIG = "robots.json"
FREC_DIBUJO = 30                     # actualizaciones de PyBullet por segundo
PERIODO_DIBUJO = 1.0 / FREC_DIBUJO
SIN_DATOS = 1.0                      # s sin q nuevo -> robot marcado como desconectado
ALTURA_ETIQUETA = 0.9                # m por encima de la base del robot
VERDE = [0, 0.6, 0]
ROJO = [0.9, 0, 0]
# ============== ESTADO DEL ROBOT (NUEVO) ==============
NARANJA = [1.0, 0.5, 0]
# Situaciones de cada robot: texto de la etiqueta y su color
OK, APAGADO, SIN_DATOS_TXT = "OK", "ROBOT NO ARRANCADO", "SIN DATOS"
COLORES = {OK: VERDE, APAGADO: NARANJA, SIN_DATOS_TXT: ROJO}
# ============ FIN ESTADO DEL ROBOT (NUEVO) ============


def cargar_config(ruta):
    """Lee la lista de robots del fichero JSON y comprueba sus campos."""
    with open(ruta, encoding="utf-8") as f:
        robots = json.load(f)
    if not isinstance(robots, list) or not robots:
        raise ValueError("El fichero debe contener una lista con al menos un robot")
    for r in robots:
        for campo in ("nombre", "ip", "puerto", "pose"):
            if campo not in r:
                raise ValueError(f"Falta el campo '{campo}' en {r}")
        if len(r["pose"]) != 6:
            raise ValueError(f"La pose de {r['nombre']} debe ser [x, y, z, roll, pitch, yaw]")
    return robots
# ========== FIN C (DIFERENTE de primaria.py) ==========


# ============ C (DIFERENTE de primaria.py) ============
# Antes visualizar(robot_sim): ahora recibe la configuración de UN robot y,
# en vez de llamar a move_robot(), guarda el último q en el estado
# compartido. PyBullet NO se toca desde aquí: solo desde el bucle principal.
# (SIN VARIABLES GLOBALES: estado y lock llegan como argumentos desde main())
def recibir(robot, estado, lock):
    """Abre una conexión con un robot y guarda su último q_actual en estado
    (protegido con lock). Vuelve (o lanza una excepción) cuando la conexión
    se pierde, para que hilo_receptor() pueda reconectar.
    """
    nombre, ip, puerto = robot["nombre"], robot["ip"], robot["puerto"]
    print(f"[{nombre}] Conectando a {ip}:{puerto} ...")
    with socket.create_connection((ip, puerto), timeout=TIMEOUT) as sock:
        print(f"[{nombre}] Conexión TCP establecida.")
# ========== FIN C (DIFERENTE de primaria.py) ==========

        # Los bytes se acumulan en un buffer y se cortan los mensajes usando
        # la longitud L que cada uno lleva en sus 4 primeros bytes.
        # (Cada hilo tiene su propio socket y su propio buffer.)
        buffer = bytearray()
        # ============== ESTADO DEL ROBOT (NUEVO) ==============
        modo_anterior = None        # para avisar solo cuando cambia el modo
        # ============ FIN ESTADO DEL ROBOT (NUEVO) ============

        # Se lee indefinidamente hasta perder la conexión o hasta Ctrl+C.
        while True:
            # Paso 1: recv() y añadir al final del buffer
            datos = sock.recv(TAM_RECV)
            if not datos:                   # recv() == b"" -> el robot cerró la conexión
                print(f"[{nombre}] El robot ha cerrado la conexión.")
                return
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

                # Paso 5: si es un mensaje de estado, recorrer sus subpaquetes;
                # cualquier otro tipo se ignora (ya está fuera del buffer).
                if T == TIPO_ESTADO:
                    # ============== ESTADO DEL ROBOT (NUEVO) ==============
                    # Igual que en primaria.py: se recorren todos los
                    # subpaquetes y después se decide qué hacer.
                    modo, q = None, None
                    for tipo_sp, tam_sp, pos_sp in recorrer_subpaquetes(mensaje):
                        if tipo_sp == TIPO_ROBOT_MODE:
                            modo = leer_robot_mode(mensaje, pos_sp, tam_sp)
                        elif tipo_sp == TIPO_JOINT_DATA:
                            q = leer_q_actual(mensaje, pos_sp, tam_sp)

                    if modo is not None and modo["robotMode"] != modo_anterior:
                        modo_anterior = modo["robotMode"]
                        print(f"[{nombre}] Modo del robot: {nombre_modo(modo_anterior)}")

                    robot_listo = modo is not None and modo["robotMode"] == ROBOT_MODE_RUNNING
                    # ============ FIN ESTADO DEL ROBOT (NUEVO) ============

                    if q is not None:
                        # ============ C (DIFERENTE de primaria.py) ============
                        # En vez de move_robot() y print(): se escribe el
                        # último q en el estado compartido, con el Lock.
                        # ("listo" es de ESTADO DEL ROBOT: el bucle de dibujo
                        # solo mueve el robot simulado si está en RUNNING)
                        with lock:
                            estado[nombre] = {"q": q, "t": time.time(), "listo": robot_listo}
                        # ========== FIN C (DIFERENTE de primaria.py) ==========


# ============ C (DIFERENTE de primaria.py) ============
# Antes era main(): el bucle de reconexión es el mismo, pero ahora se ejecuta
# en un HILO por robot. Si un robot se cae, solo se bloquea su hilo y los
# demás siguen recibiendo.
# (SIN VARIABLES GLOBALES: estado y lock llegan como argumentos desde main())
def hilo_receptor(robot, estado, lock):
    nombre = robot["nombre"]
# ========== FIN C (DIFERENTE de primaria.py) ==========
    # Bucle de reconexión: si URSim se reinicia o se cae la red, el programa
    # no se cuelga ni termina; avisa y vuelve a intentarlo.
    while True:
        try:
            recibir(robot, estado, lock)
        except ConnectionRefusedError:
            print(f"[{nombre}] Conexión rechazada (RST): el equipo responde pero nadie escucha en ese puerto.")
        except socket.timeout:
            print(f"[{nombre}] Tiempo agotado ({TIMEOUT} s sin datos): IP errónea, VM apagada o cortafuegos.")
        except OSError as e:
            print(f"[{nombre}] Error de red: {e}")
        except ValueError as e:
            print(f"[{nombre}] Error de protocolo: {e} -> se cierra la conexión y se reabre")

        print(f"[{nombre}] Reintentando en {ESPERA_RECONEXION} s ...")
        time.sleep(ESPERA_RECONEXION)


# ============ C (DIFERENTE de primaria.py) ============
def crear_robots_sim(robot_sim, robots):
    """Crea un robot simulado por cada robot real y devuelve sus IDs.

    UR3eRG2() ya carga el robot 0 en el origen: se aprovecha para el primer
    robot cuya pose sea el origen. El resto se crean con spawn_robot(),
    usando el ID que devuelve (no es consecutivo: el suelo también ocupa uno).
    """
    ids = {}
    id0_usado = False
    for r in robots:
        if not id0_usado and all(v == 0 for v in r["pose"]):
            ids[r["nombre"]] = 0
            id0_usado = True
        else:
            ids[r["nombre"]] = robot_sim.spawn_robot(list(r["pose"]))
        print(f"Robot simulado '{r['nombre']}' -> ID {ids[r['nombre']]}, pose {r['pose']}")
    return ids


def poner_etiqueta(robot, situacion, id_etiqueta):
    """Escribe encima del robot su nombre y su situación (verde/naranja/rojo)."""
    x, y, z = robot["pose"][:3]
    # ============== ESTADO DEL ROBOT (NUEVO) ==============
    # Antes solo había dos casos (OK / SIN DATOS); ahora tres
    texto = f"{robot['nombre']}: {situacion}"
    color = COLORES[situacion]
    # ============ FIN ESTADO DEL ROBOT (NUEVO) ============
    if id_etiqueta is None:
        return p.addUserDebugText(texto, [x, y, z + ALTURA_ETIQUETA], color, textSize=1.5)
    return p.addUserDebugText(texto, [x, y, z + ALTURA_ETIQUETA], color, textSize=1.5,
                              replaceItemUniqueId=id_etiqueta)


def main():
    ruta = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), FICHERO_CONFIG)
    robots = cargar_config(ruta)
    print(f"Configuración: {ruta} ({len(robots)} robots)")

    # PyBullet admite una única ventana por proceso: un solo UR3eRG2 y un
    # robot simulado por cada robot real.
    robot_sim = UR3eRG2()
    ids = crear_robots_sim(robot_sim, robots)

    # ============== SIN VARIABLES GLOBALES ==============
    # Estado compartido entre los hilos receptores (escriben) y el bucle de
    # dibujo (lee). Una entrada por robot: {"q": [...], "t": instante,
    # "listo": robot en RUNNING}. Se protege con un Lock para no leer un
    # vector a medio escribir. Antes eran variables globales del módulo;
    # ahora se crean aquí y se pasan a cada hilo como argumentos.
    estado = {}
    lock = threading.Lock()
    # ============ FIN SIN VARIABLES GLOBALES ============

    # Un hilo receptor por robot. daemon=True: al pulsar Ctrl+C el programa
    # termina aunque los hilos sigan bloqueados en recv().
    for r in robots:
        # (SIN VARIABLES GLOBALES: se añaden estado y lock a args)
        threading.Thread(target=hilo_receptor, args=(r, estado, lock), daemon=True,
                         name=r["nombre"]).start()

    # Bucle principal de dibujo: lee el estado a ritmo fijo y actualiza cada
    # robot simulado. Es el ÚNICO sitio desde el que se llama a PyBullet.
    t_dibujado = {r["nombre"]: 0.0 for r in robots}     # último q ya dibujado
    situacion = {r["nombre"]: None for r in robots}     # None = aún sin decidir
    etiquetas = {r["nombre"]: None for r in robots}
    # ============== CIERRE DE URSIM (NUEVO) ==============
    t_arranque = time.time()
    # ============ FIN CIERRE DE URSIM (NUEVO) ============

    while True:
        inicio = time.time()
        # ============== CIERRE DE VENTANA (NUEVO) ==============
        # Si se cierra la ventana, se sale del bucle de dibujo; los hilos
        # receptores son daemon y terminan solos con el programa.
        comprobar_ventana()
        # ============ FIN CIERRE DE VENTANA (NUEVO) ============

        # Copia del estado con el Lock, para tenerlo el menor tiempo posible
        with lock:
            copia = {nombre: dict(e) for nombre, e in estado.items()}

        # ============== CIERRE DE URSIM (NUEVO) ==============
        # Un robot caído NO detiene a los demás (requisito del notable): el
        # programa solo termina cuando TODOS llevan más de TIEMPO_MAX_SIN_DATOS
        # sin enviar nada (por ejemplo, porque se han cerrado todos los URSim).
        ultimo_dato = max([e["t"] for e in copia.values()] + [t_arranque])
        if inicio - ultimo_dato > TIEMPO_MAX_SIN_DATOS:
            print(f"{inicio - ultimo_dato:.0f} s sin datos de ningún robot: se da por "
                  f"hecho que se han cerrado todos los URSim. Programa terminado.")
            return
        # ============ FIN CIERRE DE URSIM (NUEVO) ============

        for r in robots:
            nombre = r["nombre"]
            e = copia.get(nombre)
            hay_datos = e is not None and inicio - e["t"] < SIN_DATOS
            # ============== ESTADO DEL ROBOT (NUEVO) ==============
            # Llegan datos pero el robot no está en RUNNING (por ejemplo,
            # URSim arrancando y aún sin ON + START) -> no se visualiza
            if not hay_datos:
                nueva = SIN_DATOS_TXT
            elif e["listo"]:
                nueva = OK
            else:
                nueva = APAGADO
            # ============ FIN ESTADO DEL ROBOT (NUEVO) ============

            # Cambio de estado: se avisa por consola y se cambia la etiqueta
            if nueva != situacion[nombre]:
                situacion[nombre] = nueva
                # (CIERRE DE VENTANA: si falla porque la ventana se acaba de cerrar)
                try:
                    etiquetas[nombre] = poner_etiqueta(r, nueva, etiquetas[nombre])
                except p.error:
                    raise VentanaCerrada()
                if nueva == OK:
                    print(f"[{nombre}] Recibiendo datos: el robot simulado le sigue.")
                elif nueva == APAGADO:
                    print(f"[{nombre}] Conectado, pero el robot no está arrancado "
                          f"(ON + START en PolyScope): no se visualiza.")
                else:
                    print(f"[{nombre}] AVISO: sin datos; el robot simulado queda congelado.")

            # Solo se mueve si hay un q nuevo desde el último dibujo y el robot
            # está en RUNNING; si no, el robot simulado simplemente no se mueve.
            if nueva == OK and e["t"] > t_dibujado[nombre]:
                # (CIERRE DE VENTANA: antes robot_sim.move_robot(...))
                mover_robot_sim(robot_sim, ids[nombre], list(a_convenio_simulador(e["q"])))
                t_dibujado[nombre] = e["t"]

        # Esperar lo que falte hasta completar el periodo de dibujo
        time.sleep(max(0.0, PERIODO_DIBUJO - (time.time() - inicio)))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nPrograma terminado por el usuario.")
    # ============== CIERRE DE VENTANA (NUEVO) ==============
    except VentanaCerrada:
        print("\nVentana de PyBullet cerrada: programa terminado.")
    # ============ FIN CIERRE DE VENTANA (NUEVO) ============
    except (OSError, ValueError) as e:
        print(f"Error en la configuración: {e}")
# ========== FIN C (DIFERENTE de primaria.py) ==========
