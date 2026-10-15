import socket
import struct
import time
import math
import sys
import os
import json
import queue
import threading
import pybullet as p
from UR3e_rg2.dynamic_ur3e import UR3eRG2

# Se REUTILIZAN tal cual las constantes y funciones de primaria.py (formato
# de los mensajes) y la lectura de la configuración de BloqueC.py.
from primaria import (TAM_RECV, TAM_CABECERA, MAX_LONGITUD, TIPO_ESTADO,
                      TIPO_JOINT_DATA, TIPO_ROBOT_MODE, TIMEOUT, ESPERA_RECONEXION,
                      TIEMPO_MAX_SIN_DATOS, ROBOT_MODE_RUNNING, N_ARTICULACIONES,
                      SIGNOS, OFFSETS, ID_ROBOT_SIM,
                      recorrer_subpaquetes, leer_q_actual, leer_robot_mode, nombre_modo,
                      a_convenio_simulador, VentanaCerrada, comprobar_ventana,
                      mover_robot_sim)
from BloqueC import cargar_config, FICHERO_CONFIG, VERDE, ROJO, NARANJA

# ============ D (NUEVO) ============
# Uso:  python BloqueD.py [fichero.json] [índice del robot]
#       (por defecto robots.json y el robot 0, el primero de la lista)
PUERTO_ORDENES = 30001      # acepta URScript y es el que sabemos decodificar

# Límites OBLIGATORIOS en el robot real (apartado 11): no subirlos
V_MOVEJ = 0.3               # rad/s
A_MOVEJ = 0.5               # rad/s²
LIMITE_ANGULO = 2 * math.pi  # rad: un valor mayor huele a grados
TOLERANCIA = 0.01           # rad: q_actual "cerca" del objetivo
T_MAX_INICIO = 2.0          # s para que isProgramRunning pase a True

TIPO_MENSAJE_ROBOT = 20
SUBTIPO_TEXTO, SUBTIPO_VERSION, SUBTIPO_SEGURIDAD = 0, 3, 5
SUBTIPO_ERROR, SUBTIPO_CLAVE, SUBTIPO_EXCEPCION = 6, 7, 10

FICHERO_SECUENCIA = "secuencia.json"
POSE_PREVIEW = [0.0, 0.7, 0.0, 0.0, 0.0, 0.0]   # robot de previsualización
VEL_PREVIEW = 1.0           # rad/s de la animación (solo en pantalla)
FREC_DIBUJO = 30
PERIODO_DIBUJO = 1.0 / FREC_DIBUJO
SIN_DATOS = 1.0             # s sin estado nuevo -> sin datos
ALTURA_ETIQUETA = 0.9
AZUL = [0.1, 0.3, 0.9]

AYUDA = """
Comandos (escribe la letra y pulsa Enter):
  g            guardar la posición ACTUAL del robot como punto
  d            guardar la posición de los DESLIZADORES como punto
  b            borrar el último punto
  v            ver la lista de puntos
  p            previsualizar la secuencia (robot azul)
  e            ejecutar la secuencia en el robot, punto a punto
  r            reanudar tras una pausa (repite el punto en que se paró)
  c            cancelar la ejecución
  x            parar el robot YA (stopj)
  s [fichero]  guardar la secuencia (por defecto secuencia.json)
  l [fichero]  cargar una secuencia
  h            ver esta ayuda
  q            salir
"""


# ---------------------------------------------------------------------------
# Mensajes de tipo 20 (Robot Messages): formato del documento
# Primary/Secondary Client Interface, apartado 2.2
# ---------------------------------------------------------------------------
def texto_de(mensaje, desde):
    """Texto (charArray) que ocupa desde 'desde' hasta el final del mensaje."""
    return mensaje[desde:].decode("utf-8", errors="replace").strip("\x00").strip()


def leer_mensaje_robot(mensaje):
    """Decodifica un mensaje de tipo 20 y devuelve (nivel, texto), con nivel
    "ERROR", "AVISO" o "INFO"; None si no interesa.

    Cabecera común: messageSize (int) + messageType (uchar) + timestamp
    (uint64) + source (char) + robotMessageType (char) -> 15 bytes.
    """
    if len(mensaje) < 15:
        return None
    subtipo = mensaje[14]
    try:
        if subtipo == SUBTIPO_ERROR:
            # robotMessageCode, Argument, ReportLevel (int) + DataType, Data (uint32)
            codigo, argumento, _, _, _ = struct.unpack_from(">iiiII", mensaje, 15)
            return "ERROR", f"C{codigo}A{argumento}: {texto_de(mensaje, 35)}"
        if subtipo == SUBTIPO_EXCEPCION:
            linea, columna = struct.unpack_from(">ii", mensaje, 15)
            return "ERROR", (f"excepción del programa (línea {linea}, columna "
                             f"{columna}): {texto_de(mensaje, 23)}")
        if subtipo == SUBTIPO_SEGURIDAD:
            codigo, argumento, modo_seg = struct.unpack_from(">iiB", mensaje, 15)
            return "AVISO", f"modo de seguridad {modo_seg} (C{codigo}A{argumento})"
        if subtipo == SUBTIPO_CLAVE:
            codigo, argumento, tam_titulo = struct.unpack_from(">iiB", mensaje, 15)
            titulo = texto_de(mensaje[:24 + tam_titulo], 24)
            return "INFO", f"{titulo} {texto_de(mensaje, 24 + tam_titulo)}".strip()
        if subtipo == SUBTIPO_TEXTO:
            return "INFO", texto_de(mensaje, 15)
        if subtipo == SUBTIPO_VERSION:
            tam_nombre = mensaje[15]
            mayor, menor = struct.unpack_from(">BB", mensaje, 16 + tam_nombre)
            return "INFO", f"versión del controlador {mayor}.{menor}"
    except struct.error:
        return "AVISO", f"mensaje de tipo 20/{subtipo} con formato inesperado"
    return None                     # otros subtipos: se ignoran


# ---------------------------------------------------------------------------
# Estado compartido entre el hilo receptor y el bucle principal
# ---------------------------------------------------------------------------
class ConexionRobot:
    """Último estado del robot y socket abierto, protegidos con un Lock.

    El hilo receptor escribe; el bucle principal lee con foto() y envía
    órdenes con enviar() por el MISMO socket (TCP es bidireccional).
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.sock = None            # None mientras no hay conexión
        self.q = None
        self.modo = None            # dict de leer_robot_mode()
        self.t = 0.0                # instante del último mensaje de estado
        self.n_estados = 0          # nº de mensajes de estado recibidos
        self.eventos = []           # mensajes de tipo 20 aún sin mostrar

    def foto(self):
        """Copia del estado actual; vacía la lista de eventos."""
        with self.lock:
            f = {"q": list(self.q) if self.q else None,
                 "modo": dict(self.modo) if self.modo else None,
                 "t": self.t, "n": self.n_estados,
                 "conectado": self.sock is not None,
                 "eventos": self.eventos}
            self.eventos = []
        return f

    def enviar(self, orden):
        """Envía una orden URScript terminada en salto de línea."""
        with self.lock:
            sock = self.sock
        if sock is None:
            raise ConnectionError("no hay conexión con el robot")
        # Sin el '\n' el controlador sigue esperando y no ejecuta nada;
        # sendall() garantiza que se escriben todos los bytes
        sock.sendall((orden.strip() + "\n").encode("utf-8"))


# ---------------------------------------------------------------------------
# Hilo receptor: copia de recibir()/hilo_receptor() de BloqueC.py
# ---------------------------------------------------------------------------
def recibir(robot, con):
    nombre, ip, puerto = robot["nombre"], robot["ip"], robot["puerto"]
    print(f"[{nombre}] Conectando a {ip}:{puerto} ...")
    with socket.create_connection((ip, puerto), timeout=TIMEOUT) as sock:
        print(f"[{nombre}] Conexión TCP establecida.")
        # (D) Se publica el socket para que el bucle principal pueda enviar
        with con.lock:
            con.sock = sock
        try:
            buffer = bytearray()
            while True:
                # Paso 1: recv() y añadir al final del buffer
                datos = sock.recv(TAM_RECV)
                if not datos:
                    print(f"[{nombre}] El robot ha cerrado la conexión.")
                    return
                buffer += datos

                # Paso 2: mientras haya al menos una cabecera completa...
                while len(buffer) >= TAM_CABECERA:
                    L, T = struct.unpack_from(">iB", buffer, 0)
                    if L < TAM_CABECERA or L > MAX_LONGITUD:
                        raise ValueError(f"Longitud imposible ({L} B): buffer desalineado")
                    # Paso 3: el mensaje aún no ha llegado entero
                    if len(buffer) < L:
                        break
                    # Paso 4: cortar el mensaje y quitarlo del buffer
                    mensaje = bytes(buffer[:L])
                    del buffer[:L]

                    # Paso 5: mensaje de estado -> modo del robot y q_actual
                    if T == TIPO_ESTADO:
                        modo, q = None, None
                        for tipo_sp, tam_sp, pos_sp in recorrer_subpaquetes(mensaje):
                            if tipo_sp == TIPO_ROBOT_MODE:
                                modo = leer_robot_mode(mensaje, pos_sp, tam_sp)
                            elif tipo_sp == TIPO_JOINT_DATA:
                                q = leer_q_actual(mensaje, pos_sp, tam_sp)
                        if q is not None and modo is not None:
                            with con.lock:
                                con.q, con.modo = q, modo
                                con.t = time.time()
                                con.n_estados += 1

                    # (D) Mensajes del robot (tipo 20): errores, excepciones...
                    elif T == TIPO_MENSAJE_ROBOT:
                        evento = leer_mensaje_robot(mensaje)
                        if evento is not None:
                            with con.lock:
                                con.eventos.append(evento)
        finally:
            with con.lock:
                con.sock = None


def hilo_receptor(robot, con):
    nombre = robot["nombre"]
    while True:
        try:
            recibir(robot, con)
        except ConnectionRefusedError:
            print(f"[{nombre}] Conexión rechazada (RST): nadie escucha en ese puerto.")
        except socket.timeout:
            print(f"[{nombre}] Tiempo agotado ({TIMEOUT} s sin datos).")
        except OSError as e:
            print(f"[{nombre}] Error de red: {e}")
        except ValueError as e:
            print(f"[{nombre}] Error de protocolo: {e} -> se reabre la conexión")
        print(f"[{nombre}] Reintentando en {ESPERA_RECONEXION} s ...")
        time.sleep(ESPERA_RECONEXION)


def hilo_teclado(comandos):
    """Lee comandos de la consola sin bloquear el bucle de PyBullet."""
    while True:
        try:
            comandos.put(input().strip())
        except EOFError:
            return


# ---------------------------------------------------------------------------
# Utilidades
# ---------------------------------------------------------------------------
def desde_convenio_simulador(q_sim):
    """Inversa de a_convenio_simulador(): de ángulos de PyBullet a ángulos
    del robot real (para los puntos definidos con los deslizadores)."""
    return [(q_sim[j] - OFFSETS[j]) / SIGNOS[j] for j in range(N_ARTICULACIONES)]


def validar_punto(q):
    """Lanza ValueError si el punto no es seguro para enviarlo con movej."""
    if len(q) != N_ARTICULACIONES:
        raise ValueError(f"el punto tiene {len(q)} valores y deben ser {N_ARTICULACIONES}")
    for j, a in enumerate(q):
        if not isinstance(a, (int, float)) or math.isnan(a) or abs(a) > LIMITE_ANGULO:
            raise ValueError(f"articulación {j} = {a}: fuera de ±2π rad (¿grados?)")


def cerca(q, objetivo):
    return all(abs(a - b) < TOLERANCIA for a, b in zip(q, objetivo))


def en_grados(q):
    return "  ".join(f"{math.degrees(a):8.2f}" for a in q)


def pose_animacion(puntos, t):
    """Pose de la animación de previsualización en el instante t: recorre los
    puntos interpolando en el espacio articular (como movej). None al acabar."""
    for a, b in zip(puntos, puntos[1:]):
        duracion = max(max(abs(x - y) for x, y in zip(a, b)) / VEL_PREVIEW, 0.2)
        if t <= duracion:
            s = t / duracion
            return [x + (y - x) * s for x, y in zip(a, b)]
        t -= duracion
    return None


# ---------------------------------------------------------------------------
# Mini-aplicación Enseñar y reproducir
# ---------------------------------------------------------------------------
class EnsenarReproducir:
    INACTIVA, ESPERANDO_INICIO, EN_MOVIMIENTO, PAUSADA = (
        "INACTIVA", "ESPERANDO_INICIO", "EN_MOVIMIENTO", "PAUSADA")

    def __init__(self, robot_sim, con, robot_cfg):
        self.robot_sim = robot_sim
        self.con = con
        self.cfg = robot_cfg
        self.id_preview = robot_sim.spawn_robot(list(POSE_PREVIEW))
        self.puntos = []                    # en radianes, convenio del robot real
        self.estado = self.INACTIVA
        self.indice = 0                     # punto que se está ejecutando
        self.objetivo = None
        self.t_envio = 0.0
        self.n_envio = 0                    # nº de estados al enviar
        self.errores_desde_envio = []
        self.motivo_pausa = ""
        self.t_preview = None               # None = no hay animación
        self.puntos_preview = []
        self.etiquetas = {}                 # nombre -> (id, texto)
        self.salir = False

    # ----- avisos -----
    def etiqueta(self, clave, pose, texto, color):
        """Texto encima de un robot; solo se redibuja si cambia."""
        anterior = self.etiquetas.get(clave)
        if anterior and anterior[1] == texto:
            return
        x, y, z = pose[:3]
        argumentos = dict(textSize=1.3)
        if anterior:
            argumentos["replaceItemUniqueId"] = anterior[0]
        try:
            id_texto = p.addUserDebugText(texto, [x, y, z + ALTURA_ETIQUETA],
                                          color, **argumentos)
        except p.error:
            raise VentanaCerrada()
        self.etiquetas[clave] = (id_texto, texto)

    # ----- comandos -----
    def comando(self, linea, f):
        if not linea:
            return
        partes = linea.split(maxsplit=1)
        orden = partes[0].lower()
        argumento = partes[1] if len(partes) > 1 else None
        ejecutando = self.estado in (self.ESPERANDO_INICIO, self.EN_MOVIMIENTO)

        if orden == "h":
            print(AYUDA)
        elif orden == "q":
            if ejecutando:
                self.parar("salida del programa")
            self.salir = True
        elif orden == "x":
            self.parar("parada pedida por el operador")
        elif ejecutando and orden in ("g", "d", "b", "p", "e", "l"):
            print("Hay una secuencia en marcha: espera a que acabe o cancélala (c).")
        elif orden == "g":
            if not self.robot_listo(f):
                print("No hay datos del robot en RUNNING: no se puede guardar su posición.")
            else:
                self.anadir_punto(f["q"], "robot")
        elif orden == "d":
            self.anadir_punto(desde_convenio_simulador(self.deslizadores()), "deslizadores")
        elif orden == "b":
            if self.puntos:
                self.puntos.pop()
                print(f"Último punto borrado. Quedan {len(self.puntos)}.")
            else:
                print("No hay puntos.")
        elif orden == "v":
            self.listar()
        elif orden == "p":
            self.previsualizar(f)
        elif orden == "e":
            self.ejecutar(f)
        elif orden == "r":
            if self.estado != self.PAUSADA:
                print("No hay ninguna secuencia pausada.")
            else:
                print(f"Reanudando desde el punto {self.indice + 1}.")
                self.enviar_punto(f)
        elif orden == "c":
            if self.estado == self.INACTIVA:
                print("No hay nada que cancelar.")
            else:
                if ejecutando:
                    self.enviar_orden(f"stopj({A_MOVEJ})")
                self.estado = self.INACTIVA
                print("Secuencia cancelada.")
        elif orden == "s":
            self.guardar(argumento or FICHERO_SECUENCIA)
        elif orden == "l":
            self.cargar(argumento or FICHERO_SECUENCIA)
        else:
            print(f"Comando desconocido: '{linea}'. Escribe h para ver la ayuda.")

    def deslizadores(self):
        """Valores de los 6 primeros deslizadores de la ventana (rad)."""
        valores = {}
        for nombre, valor in self.robot_sim.get_param_values().items():
            nombre = nombre.strip()
            if nombre.startswith("j") and nombre[1:].isdigit():
                valores[int(nombre[1:])] = valor
        return [valores.get(j, 0.0) for j in range(N_ARTICULACIONES)]

    def anadir_punto(self, q, origen):
        try:
            validar_punto(q)
        except ValueError as e:
            print(f"Punto rechazado: {e}")
            return
        self.puntos.append(list(q))
        print(f"Punto {len(self.puntos)} guardado ({origen}): {en_grados(q)} º")

    def listar(self):
        if not self.puntos:
            print("La secuencia está vacía.")
        for i, q in enumerate(self.puntos, 1):
            print(f"  {i:2d}: {en_grados(q)} º")

    def guardar(self, ruta):
        try:
            with open(ruta, "w", encoding="utf-8") as fich:
                json.dump({"puntos_rad": self.puntos}, fich, indent=2)
            print(f"Secuencia de {len(self.puntos)} puntos guardada en {ruta}")
        except OSError as e:
            print(f"No se pudo guardar: {e}")

    def cargar(self, ruta):
        try:
            with open(ruta, encoding="utf-8") as fich:
                puntos = json.load(fich)["puntos_rad"]
            for q in puntos:
                validar_punto(q)
        except (OSError, KeyError, TypeError, ValueError) as e:
            print(f"No se pudo cargar {ruta}: {e}")
            return
        self.puntos = [list(q) for q in puntos]
        print(f"Cargados {len(self.puntos)} puntos de {ruta}")
        self.listar()

    def previsualizar(self, f):
        if not self.puntos:
            print("La secuencia está vacía.")
            return
        # Empieza en la posición actual del robot, igual que hará el movej real
        inicio = [f["q"]] if f["q"] else []
        self.puntos_preview = [a_convenio_simulador(q) for q in inicio + self.puntos]
        if len(self.puntos_preview) == 1:
            self.puntos_preview.append(self.puntos_preview[0])
        self.t_preview = time.time()
        print("Previsualizando la secuencia en el robot azul ...")

    # ----- ejecución -----
    @staticmethod
    def robot_listo(f):
        return (f["conectado"] and f["modo"] is not None and f["q"] is not None
                and time.time() - f["t"] < SIN_DATOS
                and f["modo"]["robotMode"] == ROBOT_MODE_RUNNING)

    def ejecutar(self, f):
        if not self.puntos:
            print("La secuencia está vacía.")
            return
        self.indice = 0
        print(f"Ejecutando {len(self.puntos)} puntos (v={V_MOVEJ} rad/s, a={A_MOVEJ} rad/s²)")
        self.enviar_punto(f)

    def enviar_orden(self, orden):
        try:
            self.con.enviar(orden)
            return True
        except OSError as e:            # ConnectionError también es OSError
            print(f"No se pudo enviar '{orden}': {e}")
            return False

    def enviar_punto(self, f):
        """Comprueba las condiciones previas y envía el movej del punto actual."""
        if not self.robot_listo(f):
            modo = nombre_modo(f["modo"]["robotMode"]) if f["modo"] else "sin datos"
            self.pausar(f"el robot no está listo (modo: {modo}); "
                        f"enciéndelo con ON + START")
            return
        if f["modo"]["isProtectiveStopped"]:
            self.pausar("parada de protección activa: desbloquéala en PolyScope")
            return
        if f["modo"]["isProgramRunning"]:
            self.pausar("ya hay un programa en marcha en el robot; un programa "
                        "nuevo lo abortaría")
            return
        q = self.puntos[self.indice]
        try:
            validar_punto(q)
        except ValueError as e:
            self.pausar(f"punto {self.indice + 1} no válido: {e}")
            return
        orden = (f"movej([{', '.join(f'{a:.6f}' for a in q)}], "
                 f"a={A_MOVEJ}, v={V_MOVEJ})")
        if not self.enviar_orden(orden):
            self.pausar("no se pudo enviar la orden")
            return
        print(f"-> Punto {self.indice + 1}/{len(self.puntos)}: {orden}")
        self.objetivo = q
        self.t_envio = time.time()
        self.n_envio = f["n"]
        self.errores_desde_envio = []
        self.estado = self.ESPERANDO_INICIO

    def punto_cumplido(self):
        print(f"   Punto {self.indice + 1} alcanzado.")
        self.indice += 1
        if self.indice >= len(self.puntos):
            print("Secuencia completada.")
            self.estado = self.INACTIVA
            return True
        return False

    def pausar(self, motivo):
        self.estado = self.PAUSADA
        self.motivo_pausa = motivo
        print("\n" + "!" * 60)
        print(f"SECUENCIA DETENIDA en el punto {self.indice + 1}: {motivo}")
        print("r = reanudar (repite este punto)   c = cancelar")
        print("!" * 60)

    def parar(self, motivo):
        if self.enviar_orden(f"stopj({A_MOVEJ})"):
            print("Enviado stopj.")
        if self.estado in (self.ESPERANDO_INICIO, self.EN_MOVIMIENTO):
            self.pausar(motivo)

    def paso(self, f):
        """Avanza la máquina de estados con el último estado del robot. La
        orden solo se da por cumplida con lo que el propio robot publica:
        isProgramRunning pasa a True, luego a False, y q_actual está cerca."""
        for nivel, texto in f["eventos"]:
            print(f"[ROBOT] {nivel}: {texto}")
            if nivel == "ERROR":
                self.errores_desde_envio.append(texto)

        if self.estado not in (self.ESPERANDO_INICIO, self.EN_MOVIMIENTO):
            return

        # Supervisión: cualquiera de estas condiciones detiene la secuencia
        if self.errores_desde_envio:
            self.pausar(f"error del robot: {self.errores_desde_envio[-1]}")
            return
        if not f["conectado"] or time.time() - f["t"] > SIN_DATOS:
            self.pausar("se ha perdido la conexión con el robot")
            return
        if f["modo"]["isProtectiveStopped"]:
            self.pausar("parada de protección (isProtectiveStopped)")
            return
        if f["modo"]["robotMode"] != ROBOT_MODE_RUNNING:
            self.pausar(f"el robot ha salido de RUNNING "
                        f"({nombre_modo(f['modo']['robotMode'])})")
            return

        # Solo valen estados recibidos DESPUÉS de enviar la orden
        if f["n"] <= self.n_envio:
            return
        en_marcha = f["modo"]["isProgramRunning"]

        if self.estado == self.ESPERANDO_INICIO:
            if en_marcha:
                self.estado = self.EN_MOVIMIENTO
            elif time.time() - self.t_envio > T_MAX_INICIO:
                if cerca(f["q"], self.objetivo):
                    # Ya estaba ahí o el movimiento duró menos de un mensaje
                    if not self.punto_cumplido():
                        self.enviar_punto(f)
                else:
                    self.pausar(f"orden rechazada: el programa no arrancó en "
                                f"{T_MAX_INICIO:.0f} s. ¿Está PolyScope en modo "
                                f"Remoto y la interfaz habilitada?")
        elif self.estado == self.EN_MOVIMIENTO and not en_marcha:
            if cerca(f["q"], self.objetivo):
                if not self.punto_cumplido():
                    self.enviar_punto(f)
            else:
                self.pausar("el programa terminó pero el robot no está en el "
                            "objetivo (error > 0,01 rad)")

    # ----- dibujo -----
    def dibujar(self, f):
        pose_real = self.cfg["pose"]
        # Robot que sigue al real (solo en RUNNING, como en primaria.py)
        if self.robot_listo(f):
            mover_robot_sim(self.robot_sim, ID_ROBOT_SIM, list(a_convenio_simulador(f["q"])))
        if not f["conectado"] or time.time() - f["t"] > SIN_DATOS:
            self.etiqueta("real", pose_real, "REAL: SIN DATOS", ROJO)
        elif not self.robot_listo(f):
            self.etiqueta("real", pose_real, "REAL: ROBOT NO ARRANCADO", NARANJA)
        elif self.estado == self.PAUSADA:
            self.etiqueta("real", pose_real, f"REAL: DETENIDA - {self.motivo_pausa[:40]}", ROJO)
        elif self.estado != self.INACTIVA:
            self.etiqueta("real", pose_real,
                          f"REAL: punto {self.indice + 1}/{len(self.puntos)}", VERDE)
        else:
            self.etiqueta("real", pose_real, "REAL: OK", VERDE)

        # Robot de previsualización: animación o, si no, los deslizadores
        pose = None
        if self.t_preview is not None:
            pose = pose_animacion(self.puntos_preview, time.time() - self.t_preview)
            if pose is None:
                self.t_preview = None
                print("Previsualización terminada.")
        if pose is not None:
            self.etiqueta("preview", POSE_PREVIEW, "PREVISUALIZACIÓN", AZUL)
        else:
            pose = self.deslizadores()
            self.etiqueta("preview", POSE_PREVIEW, "DESLIZADORES (d = guardar)", AZUL)
        mover_robot_sim(self.robot_sim, self.id_preview, list(pose))


def main():
    ruta = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), FICHERO_CONFIG)
    indice = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    robot_cfg = cargar_config(ruta)[indice]
    if robot_cfg["puerto"] != PUERTO_ORDENES:
        raise ValueError(f"El bloque D usa la interfaz primaria (puerto {PUERTO_ORDENES}); "
                         f"'{robot_cfg['nombre']}' tiene el puerto {robot_cfg['puerto']}")
    print(f"Robot: {robot_cfg['nombre']} ({robot_cfg['ip']}:{robot_cfg['puerto']})")

    robot_sim = UR3eRG2()           # robot 0 en el origen: sigue al real
    con = ConexionRobot()
    app = EnsenarReproducir(robot_sim, con, robot_cfg)
    comandos = queue.Queue()
    threading.Thread(target=hilo_receptor, args=(robot_cfg, con), daemon=True).start()
    threading.Thread(target=hilo_teclado, args=(comandos,), daemon=True).start()
    print(AYUDA)

    t_ultimo_dato = time.time()
    while not app.salir:
        inicio = time.time()
        comprobar_ventana()
        f = con.foto()

        # Como en BloqueC.py: termina si el robot lleva mucho sin enviar nada
        if f["t"] > 0:
            t_ultimo_dato = max(t_ultimo_dato, f["t"])
        if inicio - t_ultimo_dato > TIEMPO_MAX_SIN_DATOS:
            print(f"{inicio - t_ultimo_dato:.0f} s sin datos del robot: "
                  f"programa terminado.")
            return

        while not comandos.empty():
            app.comando(comandos.get(), f)
        app.paso(f)
        app.dibujar(f)
        time.sleep(max(0.0, PERIODO_DIBUJO - (time.time() - inicio)))
    print("Programa terminado.")
# ========== FIN D (NUEVO) ==========


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nPrograma terminado por el usuario.")
    except VentanaCerrada:
        print("\nVentana de PyBullet cerrada: programa terminado.")
    except (OSError, ValueError, IndexError) as e:
        print(f"Error en la configuración: {e}")
