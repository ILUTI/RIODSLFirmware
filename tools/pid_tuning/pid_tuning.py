#!/usr/bin/env python3
"""
pid_tuning.py -- identificacion y simulacion del PID de RIO-DSL, sin MATLAB.

Ver README.md seccion 9 del repo para el procedimiento completo y las
formulas usadas aqui. Resumen del flujo:

  1. capture   -- (opcional) grabar el log del puerto serie a un archivo.
  2. identify  -- a partir de UN escalon real de SET_RPM (en lazo cerrado,
                  con Kp conocido) calcula el modelo de planta FOPDT
                  (K, tau, L) del motor+servo real.
     identify-abierto -- lo mismo pero del escalon del PULSO en lazo
                  abierto que hace la autosintonia (log de CALIB=6/13);
                  compara contra lo que calculo la placa.
  3. tune     -- sobre ESE MODELO SIMULADO (no el motor real), busca la
                  ganancia ultima (Ku/Tu, metodo Ziegler-Nichols en lazo
                  cerrado) y da las ganancias P/PI/PID sugeridas.
  4. simulate  -- previsualiza la respuesta de una combinacion de
                  ganancias sobre el modelo, antes de cargarla al motor.

Las ganancias que salgan de "tune"/"simulate" son un punto de partida,
no un resultado final -- SIEMPRE hay que validarlas en el motor real
repitiendo el mismo escalon de SET_RPM (ver README seccion 9).
"""

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PID_TEST_PREFIX = "PID_TEST,"
PRESION_PID_TEST_PREFIX = "PRESION_PID_TEST,"
REMOTO_PID_TEST_PREFIX = "REMOTO_PID_TEST,"
GANANCIA_CAL_PASO_PREFIX = "GANANCIA_CAL,PASO,"
PRESION_GANANCIA_CAL_PASO_PREFIX = "PRESION_GANANCIA_CAL,PASO,"


# ============================================================================
# 1. Captura / parseo del log del puerto serie
# ============================================================================

@dataclass
class LogData:
    t_s: np.ndarray        # segundos relativos al primer dato (desde HAL_GetTick() ms)
    set_rpm: np.ndarray
    rpm: np.ndarray
    pulso_us: np.ndarray


def parse_pid_test_log(path):
    """Extrae las lineas 'PID_TEST,<ms>,<SET_RPM>,<RPM_filtrada>,<pulso_us>'
    de un log capturado, ignorando cualquier otra linea intercalada
    (RAK3172, GPS, eco del mando manual, etc.) -- ver README seccion 9."""
    t_ms, set_rpm, rpm, pulso = [], [], [], []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for linea in f:
            linea = linea.strip()
            if not linea.startswith(PID_TEST_PREFIX):
                continue
            partes = linea.split(",")
            if len(partes) != 4 + 1:
                continue
            try:
                t_ms.append(int(partes[1]))
                set_rpm.append(float(partes[2]))
                rpm.append(float(partes[3]))
                pulso.append(float(partes[4]))
            except ValueError:
                continue

    if not t_ms:
        raise ValueError(f"No se encontraron lineas '{PID_TEST_PREFIX}' validas en {path}")

    t_ms_arr = np.asarray(t_ms, dtype=float)
    return LogData(
        t_s=(t_ms_arr - t_ms_arr[0]) / 1000.0,
        set_rpm=np.asarray(set_rpm, dtype=float),
        rpm=np.asarray(rpm, dtype=float),
        pulso_us=np.asarray(pulso, dtype=float),
    )


@dataclass
class PresionLogData:
    t_s: np.ndarray         # segundos relativos al primer dato
    objetivo: np.ndarray    # PRESION_OBJETIVO_LOCAL vigente en cada muestra (psi)
    presion: np.ndarray     # presion medida (psi)
    setpoint_rpm: np.ndarray  # salida del lazo externo (RPM, ya con piso RPM_MIN)
    rpm: np.ndarray         # RPM filtrada real (lazo interno)


def parse_presion_pid_test_log(path):
    """Extrae las lineas 'PRESION_PID_TEST,<ms>,<PRESION_OBJETIVO_LOCAL>,
    <presion_medida>,<setpoint_rpm_crudo>,<rpm_filtrada>' -- mismo
    formato/cadencia (200ms) que PID_TEST pero del lazo EXTERNO
    (presion -> RPM, ver presion_pid.c), gateado en CALIB=10 o =11
    && MODO=CALIB_MODO_PRESION_LOCAL (ver main.c)."""
    t_ms, objetivo, presion, setpoint_rpm, rpm = [], [], [], [], []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for linea in f:
            linea = linea.strip()
            if not linea.startswith(PRESION_PID_TEST_PREFIX):
                continue
            partes = linea.split(",")
            if len(partes) != 5 + 1:
                continue
            try:
                t_ms.append(int(partes[1]))
                objetivo.append(float(partes[2]))
                presion.append(float(partes[3]))
                setpoint_rpm.append(float(partes[4]))
                rpm.append(float(partes[5]))
            except ValueError:
                continue

    if not t_ms:
        raise ValueError(f"No se encontraron lineas '{PRESION_PID_TEST_PREFIX}' validas en {path}")

    t_ms_arr = np.asarray(t_ms, dtype=float)
    return PresionLogData(
        t_s=(t_ms_arr - t_ms_arr[0]) / 1000.0,
        objetivo=np.asarray(objetivo, dtype=float),
        presion=np.asarray(presion, dtype=float),
        setpoint_rpm=np.asarray(setpoint_rpm, dtype=float),
        rpm=np.asarray(rpm, dtype=float),
    )


@dataclass
class RemotoLogData:
    t_s: np.ndarray        # segundos relativos al primer dato (desde HAL_GetTick() ms)
    set_rpm: np.ndarray    # SET_RPM mandado a mano (MODO=3, lazo abierto)
    presion_remoto: np.ndarray    # PRESION_REMOTO reportada por el aspersor remoto (psi)


def parse_remoto_pid_test_log(path):
    """Extrae las lineas 'REMOTO_PID_TEST,<ms>,<SET_RPM>,<presion_remota>'
    -- a diferencia de PID_TEST/PRESION_PID_TEST, estas lineas NO salen
    cada 200ms sino solo cuando llega un reporte nuevo del aspersor
    remoto (ver main.c) -- por eso el espaciado entre 't_s' consecutivos
    de este log es MUY irregular (segundos a mas de 20 minutos), a
    diferencia de los otros dos parsers de este archivo."""
    t_ms, set_rpm, presion_remoto = [], [], []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for linea in f:
            linea = linea.strip()
            if not linea.startswith(REMOTO_PID_TEST_PREFIX):
                continue
            partes = linea.split(",")
            if len(partes) != 3 + 1:
                continue
            try:
                t_ms.append(int(partes[1]))
                set_rpm.append(float(partes[2]))
                presion_remoto.append(float(partes[3]))
            except ValueError:
                continue

    if not t_ms:
        raise ValueError(f"No se encontraron lineas '{REMOTO_PID_TEST_PREFIX}' validas en {path}")

    t_ms_arr = np.asarray(t_ms, dtype=float)
    return RemotoLogData(
        t_s=(t_ms_arr - t_ms_arr[0]) / 1000.0,
        set_rpm=np.asarray(set_rpm, dtype=float),
        presion_remoto=np.asarray(presion_remoto, dtype=float),
    )


def parse_ganancia_cal_log(path):
    """Extrae las lineas 'GANANCIA_CAL,PASO,pulso=<us>,rpm=<rpm>,salto=<rpm>'
    del barrido en lazo abierto de CALIB=5 (ver main.c) --
    devuelve (pulso_us, rpm) ordenados por pulso, listos para
    peor_caso_K_ganancia_cal()."""
    pulso, rpm = [], []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for linea in f:
            linea = linea.strip()
            if not linea.startswith(GANANCIA_CAL_PASO_PREFIX):
                continue
            campos = {}
            for parte in linea.split(",")[2:]:
                if "=" not in parte:
                    continue
                clave, _, valor = parte.partition("=")
                campos[clave] = valor
            try:
                pulso.append(float(campos["pulso"]))
                rpm.append(float(campos["rpm"]))
            except (KeyError, ValueError):
                continue

    if not pulso:
        raise ValueError(f"No se encontraron lineas '{GANANCIA_CAL_PASO_PREFIX}' validas en {path}")

    orden = np.argsort(pulso)
    return np.asarray(pulso)[orden], np.asarray(rpm)[orden]


def peor_caso_K_ganancia_cal(pulso_us, rpm):
    """Ganancia LOCAL (RPM por microsegundo) entre cada par de pasos
    consecutivos del barrido, y el peor caso (maximo) -- mismo analisis
    que se hizo a mano en README seccion 9 ('el K real se tomo del
    mapeo de curva de ganancia... K=6.3 peor caso') para no repetir un
    Kp/Ki que funciona en una zona del rango pero oscila en otra."""
    if len(pulso_us) < 2:
        raise ValueError("Hacen falta al menos 2 pasos del barrido para calcular una ganancia local.")
    d_pulso = np.diff(pulso_us)
    d_rpm = np.diff(rpm)
    validos = d_pulso > 0
    if not np.any(validos):
        raise ValueError("El barrido no tiene pasos de pulso crecientes validos.")
    k_local = d_rpm[validos] / d_pulso[validos]
    idx_peor = int(np.argmax(k_local))
    # pulso_us tiene 1 elemento mas que d_pulso/d_rpm/validos (son diferencias
    # entre pasos consecutivos) -- pulso_us[:-1] es el pulso de INICIO de cada
    # paso, alineado 1 a 1 con 'validos' para poder indexar con la misma mascara.
    pulso_inicio_paso = pulso_us[:-1]
    return {
        "k_worst": float(k_local[idx_peor]),
        "k_min": float(np.min(k_local)),
        "k_max": float(np.max(k_local)),
        "pulso_en_peor": float(pulso_inicio_paso[validos][idx_peor]),
        "n_pasos": len(pulso_us),
    }


def parse_presion_ganancia_cal_log(path):
    """Extrae las lineas 'PRESION_GANANCIA_CAL,PASO,rpm=<rpm>,psi=<psi>,
    salto=<psi>' del barrido en lazo abierto de CALIB=9 (ver
    main.c) -- devuelve (rpm, psi) ordenados por rpm, listos para
    peor_caso_K_presion_ganancia_cal(). Mismo formato/proposito que
    parse_ganancia_cal_log() de arriba, pero para el PID#2 (presion local)
    en vez del PID#1."""
    rpm, psi = [], []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for linea in f:
            linea = linea.strip()
            if not linea.startswith(PRESION_GANANCIA_CAL_PASO_PREFIX):
                continue
            campos = {}
            for parte in linea.split(",")[2:]:
                if "=" not in parte:
                    continue
                clave, _, valor = parte.partition("=")
                campos[clave] = valor
            try:
                rpm.append(float(campos["rpm"]))
                psi.append(float(campos["psi"]))
            except (KeyError, ValueError):
                continue

    if not rpm:
        raise ValueError(f"No se encontraron lineas '{PRESION_GANANCIA_CAL_PASO_PREFIX}' validas en {path}")

    orden = np.argsort(rpm)
    return np.asarray(rpm)[orden], np.asarray(psi)[orden]


def peor_caso_K_presion_ganancia_cal(rpm, psi):
    """Ganancia LOCAL (PSI por RPM) entre cada par de pasos consecutivos del
    barrido de CALIB=9, y el peor caso (maximo) -- mismo
    analisis que peor_caso_K_ganancia_cal() de arriba, pero para la relacion
    RPM->presion del PID#2 en vez de pulso->RPM del PID#1."""
    if len(rpm) < 2:
        raise ValueError("Hacen falta al menos 2 pasos del barrido para calcular una ganancia local.")
    d_rpm = np.diff(rpm)
    d_psi = np.diff(psi)
    validos = d_rpm > 0
    if not np.any(validos):
        raise ValueError("El barrido no tiene pasos de RPM crecientes validos.")
    k_local = d_psi[validos] / d_rpm[validos]
    idx_peor = int(np.argmax(k_local))
    # rpm tiene 1 elemento mas que d_rpm/d_psi/validos (son diferencias entre
    # pasos consecutivos) -- rpm[:-1] es el RPM de INICIO de cada paso,
    # alineado 1 a 1 con 'validos' para poder indexar con la misma mascara.
    rpm_inicio_paso = rpm[:-1]
    return {
        "k_worst": float(k_local[idx_peor]),
        "k_min": float(np.min(k_local)),
        "k_max": float(np.max(k_local)),
        "rpm_en_peor": float(rpm_inicio_paso[validos][idx_peor]),
        "n_pasos": len(rpm),
    }


def _g_cl_con_kp(k, kp):
    """Ganancia en lazo cerrado de un P puro sobre una planta de ganancia
    K: G_cl = K*Kp / (1 + K*Kp) -- misma relacion que usa
    _identify_step_generico() para volver de lazo cerrado a lazo abierto,
    pero aca en la direccion inversa (dado K y Kp, que G_cl da)."""
    return (k * kp) / (1.0 + k * kp)


def _kp_para_g_cl_objetivo(k, g_cl_objetivo):
    """Despeje de la formula de arriba: que Kp hace falta para que un
    P puro sobre una planta de ganancia K llegue a un G_cl dado."""
    if not (0.0 < g_cl_objetivo < 1.0):
        raise ValueError("--g-cl-objetivo debe estar estrictamente entre 0 y 1.")
    return g_cl_objetivo / (k * (1.0 - g_cl_objetivo))


def cmd_sugerir_kp_presion(args):
    """Calcula un Kp de PRUEBA sugerido para el escalon cerrado de CALIB=10,
    a partir del barrido en lazo ABIERTO de CALIB=9 -- reemplaza el
    "probar un valor a ojo y mandarlo a revisar" por un numero derivado
    de la ganancia real ya medida en esa instalacion. Usa el K MINIMO
    (peor caso para señal, no para golpe de ariete) del barrido para
    garantizar una señal suficiente en TODA la zona operativa, no solo
    donde la ganancia es mas fuerte -- ver README seccion 9."""
    rpm, psi = parse_presion_ganancia_cal_log(args.ganancia_log)
    peor_caso = peor_caso_K_presion_ganancia_cal(rpm, psi)
    k_min = peor_caso["k_min"]
    k_max = peor_caso["k_max"]

    kp_sugerido = _kp_para_g_cl_objetivo(k_min, args.g_cl_objetivo)
    g_cl_en_k_min = _g_cl_con_kp(k_min, kp_sugerido)
    g_cl_en_k_max = _g_cl_con_kp(k_max, kp_sugerido)

    print(f"Barrido CALIB=9 ({peor_caso['n_pasos']} pasos): "
          f"K entre {k_min:.4f} y {k_max:.4f} PSI/RPM "
          f"({k_max/max(k_min, 1e-9):.1f}x de variacion)")
    print()
    print(f"Kp sugerido para el escalon de CALIB=10: {kp_sugerido:.4f}")
    print(f"  (calculado con el K MINIMO del barrido, para asegurar señal suficiente")
    print(f"  en toda la zona -- objetivo G_cl={args.g_cl_objetivo:.2f} en esa zona)")
    print(f"  G_cl esperado en la zona de menor ganancia: {g_cl_en_k_min:.3f}")
    print(f"  G_cl esperado en la zona de mayor ganancia: {g_cl_en_k_max:.3f}")
    if g_cl_en_k_max > 0.85:
        print()
        print("  ADVERTENCIA: el barrido muestra bastante variacion de ganancia -- con este")
        print("  Kp, la zona de mayor ganancia da un G_cl alto, cerca del limite donde la")
        print("  identificacion (K = G_cl/(Kp*(1-G_cl))) se vuelve numericamente inestable.")
        print("  Si el escalon de CALIB=10 termina cayendo en esa zona y falla, reintentar con")
        print("  --g-cl-objetivo mas bajo (ej. 0.2) para un Kp mas conservador.")
    print()
    print("Siguiente paso: bajar por downlink PID_PSI_KP="
          f"{kp_sugerido:.4f} y PID_PSI_KI=0, despues mandar CALIB=10.")


def cmd_capture(args):
    import serial  # import local -- pyserial solo hace falta para este subcomando

    print(f"Capturando {args.port} @ {args.baud} baud -> {args.out}  (Ctrl+C para terminar)")
    contador_pid_test = 0
    with serial.Serial(args.port, args.baud, timeout=1) as ser, \
         open(args.out, "w", encoding="utf-8") as f:
        try:
            while True:
                linea = ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")
                if not linea:
                    continue
                print(linea)
                f.write(linea + "\n")
                f.flush()
                if linea.startswith(PID_TEST_PREFIX):
                    contador_pid_test += 1
        except KeyboardInterrupt:
            print(f"\nCapturado {contador_pid_test} lineas PID_TEST en: {args.out}")


# ============================================================================
# 2. Identificacion del modelo de planta (FOPDT) a partir de un escalon
#    en LAZO CERRADO -- ver README seccion 9 para la derivacion completa.
# ============================================================================

@dataclass
class PlantModel:
    K: float    # ganancia de la planta: RPM en estado estable por microsegundo de correccion
    tau: float  # constante de tiempo de la planta, segundos
    L: float    # tiempo muerto de la planta, segundos


@dataclass
class StepIdentification:
    step_time_s: float
    delta_set_rpm: float
    baseline_rpm: float
    final_rpm: float
    delta_rpm_ss: float
    G_cl: float
    L_cl: float
    tau_cl: float
    kp_used: float
    plant: PlantModel


def _suavizar(resp, t_s, ventana_s):
    """Promedio movil centrado sobre 'resp', con ventana en segundos
    convertida a muestras via el dt tipico del log (mediana de np.diff,
    robusta a algun hueco/linea intercalada perdida). Se usa SOLO para
    decidir 'cuando se movio'/'cuando llego al 63%' en
    _identify_step_generico() -- la deteccion de escalon en lazo cerrado
    compara una lectura contra un umbral chico (ej. 1 PSI), y con un
    sensor ruidoso (ver README hardware, rizado del alternador acoplado
    al GND) una sola muestra cruda puede cruzar ese umbral por ruido
    puro, mucho antes o despues del movimiento real -- encontrado en
    campo 2026-09-21 con el lazo de presion (tau salio ~0.001s, L salio
    14.2s, ninguno de los dos creible). baseline/final NO se suavizan
    aca porque ya salen de un promedio sobre pre_window_s/settle_window_s
    (ver _identify_step_generico), que es igual de robusto."""
    if len(t_s) < 3:
        return resp
    dt_tipico = float(np.median(np.diff(t_s)))
    if dt_tipico <= 0:
        return resp
    n = max(1, int(round(ventana_s / dt_tipico)))
    if n <= 1:
        return resp
    kernel = np.ones(n) / n
    # 'same' + padding por reflexion en los bordes para no perder muestras
    # ni sesgar el arranque/final con el borde implicito de np.convolve.
    pad = n // 2
    resp_pad = np.pad(resp, pad, mode="edge")
    suavizado = np.convolve(resp_pad, kernel, mode="same")
    return suavizado[pad:pad + len(resp)]


def _detectar_escalon_generico(t_s, cmd, salto_minimo, nombre_cmd="setpoint"):
    diffs = np.diff(cmd)
    if len(diffs) == 0:
        raise ValueError("El log no tiene suficientes muestras para detectar un escalon.")
    idx = int(np.argmax(np.abs(diffs)))
    if abs(diffs[idx]) < salto_minimo:
        raise ValueError(
            f"No se detecto un escalon claro en {nombre_cmd} (mayor salto encontrado: "
            f"{diffs[idx]:.2f}, minimo esperado {salto_minimo}) -- "
            "especificar --step-time-s a mano si el escalon es mas chico."
        )
    return idx + 1  # indice de la primera muestra DESPUES del escalon


def _identify_step_generico(t_s, cmd, resp, kp_used, step_time_s,
                             pre_window_s, settle_window_s, umbral_movimiento,
                             salto_minimo, nombre_cmd, suavizado_s=0.0,
                             umbral_movimiento_frac=0.2):
    """Nucleo compartido de identificacion FOPDT en lazo cerrado (Kp puro,
    Ki=Kd=0 durante la prueba) -- usado tanto por identify_step() (lazo
    interno RPM<-servo) como por identify_step_presion() (lazo externo
    presion<-RPM). 'cmd' es el setpoint mandado (SET_RPM u OBJETIVO),
    'resp' es la variable controlada real (RPM o presion).

    'suavizado_s' (segundos, 0 = sin suavizar) aplica un promedio movil
    a 'resp' (ver _suavizar()) ANTES de decidir 'cuando se movio' y
    'cuando cruzo el 63%' -- baseline_resp/final_resp siguen calculados
    sobre 'resp' crudo (ya son un promedio sobre toda la ventana, no
    hace falta suavizarlos de nuevo)."""
    if kp_used <= 0:
        raise ValueError("kp_used debe ser > 0 (es la Kp usada durante la prueba real).")

    if step_time_s is None:
        step_idx = _detectar_escalon_generico(t_s, cmd, salto_minimo, nombre_cmd)
        step_time_s = float(t_s[step_idx])

    pre_mask = (t_s >= step_time_s - pre_window_s) & (t_s < step_time_s)
    if not np.any(pre_mask):
        raise ValueError(
            "No hay suficientes datos ANTES del escalon -- capturar mas tiempo "
            f"en reposo antes de mandar el escalon (pre_window_s={pre_window_s}s)."
        )
    baseline_resp = float(np.mean(resp[pre_mask]))
    baseline_cmd = float(np.mean(cmd[pre_mask]))

    post_mask = t_s >= step_time_s
    settle_mask = post_mask & (t_s >= t_s[-1] - settle_window_s)
    if not np.any(settle_mask):
        raise ValueError(
            "No hay suficientes datos AL FINAL del log -- capturar hasta que "
            f"la respuesta se estabilice (settle_window_s={settle_window_s}s)."
        )
    final_resp = float(np.mean(resp[settle_mask]))
    final_cmd = float(np.mean(cmd[settle_mask]))

    delta_cmd = final_cmd - baseline_cmd
    delta_resp_ss = final_resp - baseline_resp

    if delta_cmd == 0:
        raise ValueError(f"delta_{nombre_cmd} salio 0 -- no se detecto un cambio real de setpoint.")

    G_cl = delta_resp_ss / delta_cmd
    if not (0.0 < G_cl < 0.999):
        raise ValueError(
            f"G_cl={G_cl:.3f} fuera del rango esperado (0,1) para un lazo P puro "
            "estable -- revisar los datos (¿la respuesta realmente se movio hacia "
            "el nuevo setpoint, sin haber saturado, y con Ki=0 durante la prueba?)."
        )

    resp_suave = _suavizar(resp, t_s, suavizado_s) if suavizado_s > 0.0 else resp

    # Umbral EFECTIVO = el mayor entre el piso absoluto (umbral_movimiento,
    # para no disparar con puro ruido cuando delta_resp_ss es chico) y una
    # fraccion del cambio real medido (umbral_movimiento_frac) -- un piso
    # fijo solo tiene sentido para lazos con delta grande (RPM); en el
    # lazo de presion, con control P puro, delta_resp_ss suele ser chico
    # (ej. 1.36 PSI de un escalon de 5 PSI en OBJETIVO, G_cl<1) y un piso
    # fijo de 1.0 PSI terminaba siendo ~75% del cambio total -- "se movio"
    # se detectaba casi al final del transitorio real, no al principio,
    # y eso indefinia L/tau (encontrado en campo 2026-09-21).
    umbral_efectivo = max(umbral_movimiento, umbral_movimiento_frac * abs(delta_resp_ss))

    moved = post_mask & (np.abs(resp_suave - baseline_resp) > umbral_efectivo)
    if not np.any(moved):
        raise ValueError(
            f"La respuesta nunca se movio mas de {umbral_efectivo:.3f} tras el escalon "
            "-- revisar los datos, bajar --umbral-movimiento/--umbral-movimiento-frac, "
            "o bajar --suavizado-s si esta de mas (un suavizado muy largo puede diluir "
            "un cambio chico)."
        )
    t_movio = float(t_s[moved][0])
    L_cl = max(t_movio - step_time_s, 0.0)

    objetivo_63 = baseline_resp + 0.632 * delta_resp_ss
    despues_de_moverse = t_s >= t_movio
    if delta_resp_ss > 0:
        cruzo = despues_de_moverse & (resp_suave >= objetivo_63)
    else:
        cruzo = despues_de_moverse & (resp_suave <= objetivo_63)
    if not np.any(cruzo):
        raise ValueError(
            "La respuesta nunca llego al 63.2% del cambio total -- ¿la captura "
            "termino antes de que se estabilizara del todo?"
        )
    t_63 = float(t_s[cruzo][0])
    tau_cl = max(t_63 - t_movio, 1e-3)

    K = G_cl / (kp_used * (1.0 - G_cl))
    tau = tau_cl / (1.0 - G_cl)
    L = L_cl

    return StepIdentification(
        step_time_s=step_time_s, delta_set_rpm=delta_cmd,
        baseline_rpm=baseline_resp, final_rpm=final_resp,
        delta_rpm_ss=delta_resp_ss, G_cl=G_cl, L_cl=L_cl, tau_cl=tau_cl,
        kp_used=kp_used, plant=PlantModel(K=K, tau=tau, L=L),
    )


def identify_step(data: LogData, kp_used, step_time_s=None,
                   pre_window_s=2.0, settle_window_s=3.0,
                   umbral_movimiento_rpm=5.0, suavizado_s=0.0,
                   umbral_movimiento_frac=0.2):
    """Identifica (K, tau, L) del lazo INTERNO (motor+servo) a partir de un
    escalon de SET_RPM capturado en lazo cerrado con ganancia Kp=kp_used
    conocida (Ki=Kd=0 en la prueba, ver README seccion 9). El tacometro
    es bastante menos ruidoso que el sensor de presion, por eso
    suavizado_s por default es 0 (sin suavizar) -- subirlo si un log en
    particular sale con L/tau poco creibles (ver identify_step_presion)."""
    return _identify_step_generico(
        data.t_s, data.set_rpm, data.rpm, kp_used, step_time_s,
        pre_window_s, settle_window_s, umbral_movimiento_rpm,
        salto_minimo=20.0, nombre_cmd="SET_RPM", suavizado_s=suavizado_s,
        umbral_movimiento_frac=umbral_movimiento_frac)


def identify_step_presion(data: PresionLogData, kp_used, step_time_s=None,
                           pre_window_s=2.0, settle_window_s=3.0,
                           umbral_movimiento_psi=0.3, suavizado_s=3.0,
                           umbral_movimiento_frac=0.2):
    """Identifica (K, tau, L) del lazo EXTERNO (presion -> RPM) a partir de
    un escalon de PRESION_OBJETIVO_LOCAL capturado en lazo cerrado con ganancia
    PID_PSI_KP=kp_used conocida (PID_PSI_KI=0 en la prueba --
    mismo motivo que el lazo interno: con Ki activo el error de estado
    estable se borra y la formula de K se indefine, ver README seccion 9).
    K sale en PSI por RPM de correccion (delta sobre RPM_MIN).

    suavizado_s=3.0 por default (a diferencia de identify_step): el
    sensor de presion (PresionV, GPT203 sin acondicionar) tiene bastante
    mas ruido que el tacometro, y la constante de tiempo real de este
    lazo (~60-120s, confirmado en campo 2026-09-21) deja de sobra 3s de
    margen para suavizar sin perder informacion real de la dinamica.

    umbral_movimiento_psi bajado de 1.0 a 0.3 (2026-09-21): con control
    P puro, delta_resp_ss suele ser chico (G_cl<1), y el piso absoluto
    solo debe evitar disparar con puro ruido -- el umbral EFECTIVO real
    (ver _identify_step_generico) termina dominado por
    umbral_movimiento_frac de todas formas si el escalon es chico."""
    return _identify_step_generico(
        data.t_s, data.objetivo, data.presion, kp_used, step_time_s,
        pre_window_s, settle_window_s, umbral_movimiento_psi,
        salto_minimo=1.0, nombre_cmd="PRESION_OBJETIVO_LOCAL", suavizado_s=suavizado_s,
        umbral_movimiento_frac=umbral_movimiento_frac)


def _identify_step_abierto_generico(t_s, cmd, resp, step_time_s,
                                      pre_window_s, settle_window_s, umbral_movimiento,
                                      salto_minimo, nombre_cmd, suavizado_s=0.0,
                                      umbral_movimiento_frac=0.2):
    """Identificacion FOPDT en LAZO ABIERTO -- a diferencia de
    _identify_step_generico() (usada por identify_step/identify_step_presion),
    esta NO asume que hay un PID ya cerrado con una Kp conocida durante la
    prueba: el 'cmd' mandado es directamente lo que se aplica a la planta
    (ej. SET_RPM en MODO=3/MANUAL_BANCO), asi que la ganancia sale directo
    de delta_resp_ss/delta_cmd, sin la transformacion lazo-cerrado->
    lazo-abierto que usa la version de arriba (esa exige 0<G_cl<1, que
    solo tiene sentido con realimentacion negativa ya activa).

    Pensada para el lazo REMOTO (MODO=2): 'cmd'=SET_RPM, 'resp'=PRESION_REMOTO,
    con pre_window_s/settle_window_s tipicamente en MINUTOS (no
    segundos) porque el aspersor reporta con cadencia muy espaciada e
    irregular (ver REMOTO_PID_TEST en main.c) -- cada muestra ya es un
    reporte discreto y deliberado, no una lectura ADC ruidosa, por eso
    suavizado_s default es 0 (sin suavizar), a diferencia de
    identify_step_presion()."""
    if step_time_s is None:
        step_idx = _detectar_escalon_generico(t_s, cmd, salto_minimo, nombre_cmd)
        step_time_s = float(t_s[step_idx])

    pre_mask = (t_s >= step_time_s - pre_window_s) & (t_s < step_time_s)
    if not np.any(pre_mask):
        raise ValueError(
            "No hay suficientes datos ANTES del escalon -- capturar mas tiempo "
            f"en reposo antes de mandar el escalon (pre_window_s={pre_window_s}s), "
            "o esperar a que llegue al menos un reporte remoto mas antes de probar."
        )
    baseline_resp = float(np.mean(resp[pre_mask]))
    baseline_cmd = float(np.mean(cmd[pre_mask]))

    post_mask = t_s >= step_time_s
    settle_mask = post_mask & (t_s >= t_s[-1] - settle_window_s)
    if not np.any(settle_mask):
        raise ValueError(
            "No hay suficientes datos AL FINAL del log -- capturar hasta que "
            f"la presion remota se estabilice (settle_window_s={settle_window_s}s)."
        )
    final_resp = float(np.mean(resp[settle_mask]))
    final_cmd = float(np.mean(cmd[settle_mask]))

    delta_cmd = final_cmd - baseline_cmd
    delta_resp_ss = final_resp - baseline_resp

    if delta_cmd == 0:
        raise ValueError(f"delta_{nombre_cmd} salio 0 -- no se detecto un cambio real de setpoint.")

    K = delta_resp_ss / delta_cmd  # ganancia de planta EN LAZO ABIERTO, directa (sin transformar)

    resp_suave = _suavizar(resp, t_s, suavizado_s) if suavizado_s > 0.0 else resp

    umbral_efectivo = max(umbral_movimiento, umbral_movimiento_frac * abs(delta_resp_ss))

    moved = post_mask & (np.abs(resp_suave - baseline_resp) > umbral_efectivo)
    if not np.any(moved):
        raise ValueError(
            f"La presion remota nunca se movio mas de {umbral_efectivo:.3f} tras el "
            "escalon -- revisar los datos, o bajar --umbral-movimiento/--umbral-movimiento-frac."
        )
    t_movio = float(t_s[moved][0])
    L = max(t_movio - step_time_s, 0.0)

    objetivo_63 = baseline_resp + 0.632 * delta_resp_ss
    despues_de_moverse = t_s >= t_movio
    if delta_resp_ss > 0:
        cruzo = despues_de_moverse & (resp_suave >= objetivo_63)
    else:
        cruzo = despues_de_moverse & (resp_suave <= objetivo_63)
    if not np.any(cruzo):
        raise ValueError(
            "La presion remota nunca llego al 63.2% del cambio total -- ¿la captura "
            "termino antes de que se estabilizara del todo? con la cadencia tan lenta "
            "del aspersor, puede hacer falta capturar mucho mas tiempo."
        )
    t_63 = float(t_s[cruzo][0])
    tau = max(t_63 - t_movio, 1e-3)

    return StepIdentification(
        step_time_s=step_time_s, delta_set_rpm=delta_cmd,
        baseline_rpm=baseline_resp, final_rpm=final_resp,
        delta_rpm_ss=delta_resp_ss, G_cl=float("nan"), L_cl=L, tau_cl=tau,
        kp_used=float("nan"), plant=PlantModel(K=K, tau=tau, L=L),
    )


def identify_step_remoto(data: RemotoLogData, step_time_s=None,
                          pre_window_s=1800.0, settle_window_s=600.0,
                          umbral_movimiento_psi=0.3, suavizado_s=0.0,
                          umbral_movimiento_frac=0.2):
    """Identifica (K, tau, L) del lazo REMOTO (SET_RPM -> presion del
    aspersor) a partir de un escalon real de SET_RPM en MODO=3
    (MANUAL_BANCO, lazo ABIERTO -- ver _identify_step_abierto_generico()).
    K sale en PSI (remotos) por RPM.

    pre_window_s/settle_window_s son GRANDES por default (30min/10min)
    porque el aspersor puede tardar 20-25min en reportar de nuevo si esta
    en 0 PSI -- con ventanas chicas (como las de identify/identify-presion)
    es facil que no haya NINGUN dato adentro y la identificacion falle."""
    return _identify_step_abierto_generico(
        data.t_s, data.set_rpm, data.presion_remoto, step_time_s,
        pre_window_s, settle_window_s, umbral_movimiento_psi,
        salto_minimo=20.0, nombre_cmd="SET_RPM", suavizado_s=suavizado_s,
        umbral_movimiento_frac=umbral_movimiento_frac)


def _imprimir_identificacion(resultado, unidad_cmd, unidad_resp, unidad_K, n_muestras):
    print(f"Escalon detectado en t={resultado.step_time_s:.2f}s (log tiene {n_muestras} muestras)")
    print(f"  Setpoint ({unidad_cmd}): delta={resultado.delta_set_rpm:.2f}")
    print(f"  Respuesta ({unidad_resp}): {resultado.baseline_rpm:.2f} -> {resultado.final_rpm:.2f} "
          f"(delta_ss={resultado.delta_rpm_ss:.2f})")
    print(f"  G_cl (lazo cerrado) = {resultado.G_cl:.4f}")
    print(f"  L_cl (tiempo muerto observado)   = {resultado.L_cl:.3f} s")
    print(f"  tau_cl (constante de tiempo, lazo cerrado) = {resultado.tau_cl:.3f} s")
    print()
    print(f"Modelo de planta identificado (con Kp={resultado.kp_used} durante la prueba):")
    print(f"  K   = {resultado.plant.K:.6f}  {unidad_K}")
    print(f"  tau = {resultado.plant.tau:.3f} s")
    print(f"  L   = {resultado.plant.L:.3f} s")


def _imprimir_identificacion_abierto(resultado, unidad_cmd, unidad_resp, unidad_K, n_muestras):
    print(f"Escalon detectado en t={resultado.step_time_s:.2f}s (log tiene {n_muestras} muestras)")
    print(f"  Setpoint ({unidad_cmd}): delta={resultado.delta_set_rpm:.2f}")
    print(f"  Respuesta ({unidad_resp}): {resultado.baseline_rpm:.2f} -> {resultado.final_rpm:.2f} "
          f"(delta_ss={resultado.delta_rpm_ss:.2f})")
    print()
    print("Modelo de planta identificado (lazo ABIERTO, sin PID activo durante la prueba):")
    print(f"  K   = {resultado.plant.K:.6f}  {unidad_K}")
    print(f"  tau = {resultado.plant.tau:.3f} s")
    print(f"  L   = {resultado.plant.L:.3f} s")


def cmd_identify(args):
    data = parse_pid_test_log(args.log)
    resultado = identify_step(data, kp_used=args.kp, step_time_s=args.step_time_s,
                               suavizado_s=args.suavizado_s,
                               umbral_movimiento_frac=args.umbral_movimiento_frac,
                               umbral_movimiento_rpm=args.umbral_movimiento_rpm)
    _imprimir_identificacion(resultado, "RPM", "RPM", "RPM por microsegundo de correccion", len(data.t_s))
    print()
    print("Siguiente paso:")
    print(f"  python pid_tuning.py tune --K {resultado.plant.K:.6f} "
          f"--tau {resultado.plant.tau:.3f} --L {resultado.plant.L:.3f}")


def cmd_identify_presion(args):
    data = parse_presion_pid_test_log(args.log)
    resultado = identify_step_presion(data, kp_used=args.kp, step_time_s=args.step_time_s,
                                       suavizado_s=args.suavizado_s,
                                       umbral_movimiento_frac=args.umbral_movimiento_frac,
                                       umbral_movimiento_psi=args.umbral_movimiento_psi)
    _imprimir_identificacion(resultado, "PSI (PRESION_OBJETIVO_LOCAL)", "PSI", "PSI por RPM de correccion", len(data.t_s))
    print()
    print("Siguiente paso (SIMC, no hace falta buscar Ku/Tu con tiempo muerto tan chico):")
    print(f"  python pid_tuning.py simc --K {resultado.plant.K:.6f} "
          f"--tau {resultado.plant.tau:.3f} --L {resultado.plant.L:.3f}")


def cmd_identify_remoto(args):
    data = parse_remoto_pid_test_log(args.log)
    resultado = identify_step_remoto(data, step_time_s=args.step_time_s,
                                      pre_window_s=args.pre_window_s,
                                      settle_window_s=args.settle_window_s,
                                      suavizado_s=args.suavizado_s,
                                      umbral_movimiento_frac=args.umbral_movimiento_frac,
                                      umbral_movimiento_psi=args.umbral_movimiento_psi)
    _imprimir_identificacion_abierto(resultado, "RPM (SET_RPM)", "PSI (remota)", "PSI por RPM", len(data.t_s))
    print()
    print("Siguiente paso (SIMC):")
    print(f"  python pid_tuning.py simc --K {resultado.plant.K:.6f} "
          f"--tau {resultado.plant.tau:.3f} --L {resultado.plant.L:.3f}")


# ----------------------------------------------------------------------------
# identify-abierto: escalon del PULSO en lazo abierto de la autosintonia PID#1
# (CALIB=6 o 13, autotune_rpm.c). Replica la identificacion de la placa para
# poder revisarla en la PC con el mismo log.
# ----------------------------------------------------------------------------

AUTOTUNE1_ESCALON_PREFIX = "AUTOTUNE_PID1,ESCALON_ABIERTO,"
AUTOTUNE1_ID_PREFIX = "AUTOTUNE_PID1,ID,"
AUTOTUNE1_ID_COMPLETO_PREFIX = "AUTOTUNE_PID1,ID_COMPLETO,"
AUTOTUNE1_CANDIDATA_PREFIX = "AUTOTUNE_PID1,CANDIDATA,"
AUTOTUNE1_CURVA_REUSADA_PREFIX = "AUTOTUNE_PID1,CURVA_REUSADA,"

# Mismas constantes que autotune_rpm.c -- si cambian alla, cambiarlas aca.
AUTOTUNE1_L_MIN_S = 0.15
AUTOTUNE1_TAU_C_FACTOR_L = 2.0
AUTOTUNE1_TAU_C_FACTOR_TAU = 3.0
AUTOTUNE1_TAU_C_MIN_S = 0.30


def _campos_clave_valor(linea, desde):
    campos = {}
    for parte in linea.split(",")[desde:]:
        clave, sep, valor = parte.partition("=")
        if sep:
            campos[clave] = valor
    return campos


def parse_escalon_abierto_autotune(path):
    """Lee el log de una corrida de CALIB=6/13 y devuelve las muestras
    PID_TEST, el indice de la primera muestra DESPUES de la ultima linea
    'AUTOTUNE_PID1,ESCALON_ABIERTO,...' (el escalon no se autodetecta: en
    el mismo log estan la curva y las vueltas a la base), los campos de esa
    linea y lo que imprimio la placa despues (ID / candidata)."""
    t_ms, rpm, pulso = [], [], []
    marca = None
    placa = {}
    k_peor_reusada = None  # CALIB=6 usa la curva de CALIB=5 y solo imprime su K peor caso
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for linea in f:
            linea = linea.strip()
            if linea.startswith(PID_TEST_PREFIX):
                partes = linea.split(",")
                if len(partes) != 5:
                    continue
                try:
                    t_ms.append(int(partes[1]))
                    rpm.append(float(partes[3]))
                    pulso.append(float(partes[4]))
                except ValueError:
                    continue
            elif linea.startswith(AUTOTUNE1_CURVA_REUSADA_PREFIX):
                k_peor_reusada = _fnum(_campos_clave_valor(linea, 2), "k_peor")
            elif linea.startswith(AUTOTUNE1_ESCALON_PREFIX):
                marca = {"indice": len(t_ms), "campos": _campos_clave_valor(linea, 2)}
                placa = {}  # nos quedamos con la ULTIMA corrida del log
            elif marca is not None and linea.startswith(AUTOTUNE1_ID_PREFIX):
                placa["id"] = _campos_clave_valor(linea, 2)
            elif marca is not None and (linea.startswith(AUTOTUNE1_ID_COMPLETO_PREFIX)
                                        or linea.startswith(AUTOTUNE1_CANDIDATA_PREFIX)):
                placa.setdefault("candidata", _campos_clave_valor(linea, 2))

    if marca is None:
        raise ValueError(
            f"No se encontro la linea '{AUTOTUNE1_ESCALON_PREFIX}...' en {path} -- este "
            "comando es para logs de CALIB=6 o CALIB=13 (escalon en lazo abierto del autotune).")
    if marca["indice"] >= len(t_ms):
        raise ValueError("No hay lineas PID_TEST despues del escalon -- ¿el log se corto?")
    t_arr = np.asarray(t_ms, dtype=float)
    return {
        "t_s": (t_arr - t_arr[0]) / 1000.0,
        "rpm": np.asarray(rpm, dtype=float),
        "pulso_us": np.asarray(pulso, dtype=float),
        "i_escalon": marca["indice"],
        "marca": marca["campos"],
        "placa": placa,
        "k_peor_reusada": k_peor_reusada,
    }


def identify_escalon_abierto(t_s, rpm, i_escalon, delta_pulso, duracion_s,
                             pre_window_s=2.0, ventana_final_s=5.0, metodo="dos-puntos",
                             umbral_movimiento_rpm=5.0, umbral_movimiento_frac=0.2,
                             suavizado_s=0.0):
    """Mismo calculo que Autotune_IdentificarLazoAbierto() (autotune.c):
    K = delta_RPM_ss / delta_pulso; tau/L por dos puntos (28.3%/63.2%, Smith,
    lo que usa la placa para PID#1) o por umbral (como identify-remoto)."""
    t0 = float(t_s[i_escalon])
    pre = (t_s >= t0 - pre_window_s) & (t_s < t0)
    if not np.any(pre):
        raise ValueError("No hay muestras PID_TEST antes del escalon para la base.")
    post = (t_s >= t0) & (t_s <= t0 + duracion_s)
    tt = t_s[post] - t0
    y = rpm[post]
    if len(y) < 10:
        raise ValueError("Muy pocas muestras despues del escalon.")
    y0 = float(np.mean(rpm[pre]))

    fin = tt >= tt[-1] - ventana_final_s
    ant = (tt >= tt[-1] - 2 * ventana_final_s) & ~fin
    yf = float(np.mean(y[fin]))
    aviso = None
    if np.any(ant):
        deriva = abs(yf - float(np.mean(y[ant])))
        tol = max(0.10 * abs(yf - y0), 2.5 * umbral_movimiento_rpm)
        if deriva > tol:
            aviso = (f"la RPM seguia moviendose al final ({deriva:.1f} RPM entre las dos ultimas "
                     f"ventanas de {ventana_final_s:.0f} s; la placa lo rechazaria como no_asento)")

    delta_ss = yf - y0
    K = delta_ss / delta_pulso
    if not K > 0:
        raise ValueError(f"K={K:.4f} no es positiva -- la RPM no subio con el pulso.")

    ys = _suavizar(y, tt, suavizado_s) if suavizado_s > 0.0 else y
    sube = delta_ss > 0

    def primer_cruce(objetivo, desde=0.0):
        # Interpolado entre la muestra anterior y la que cruza: PID_TEST va
        # cada 200 ms y tau de este lazo es ~0.4 s, sin interpolar tau sale
        # cuantizado a multiplos de 0.2 s.
        m = (tt >= desde) & ((ys >= objetivo) if sube else (ys <= objetivo))
        if not np.any(m):
            return None
        i = int(np.argmax(m))
        if i == 0 or ys[i] == ys[i - 1]:
            return float(tt[i])
        frac = (objetivo - ys[i - 1]) / (ys[i] - ys[i - 1])
        return float(tt[i - 1] + frac * (tt[i] - tt[i - 1]))

    t63 = primer_cruce(y0 + 0.632 * delta_ss)
    if t63 is None:
        raise ValueError("La respuesta nunca llego al 63.2% del cambio total.")
    if metodo == "dos-puntos":
        t28 = primer_cruce(y0 + 0.283 * delta_ss)
        tau = max(1.5 * (t63 - t28), 1e-3)
        L = max(t63 - tau, 0.0)
    else:
        umbral = max(umbral_movimiento_rpm, umbral_movimiento_frac * abs(delta_ss))
        movio = np.abs(ys - y0) > umbral
        if not np.any(movio):
            raise ValueError("La respuesta nunca supero el umbral de movimiento.")
        t_movio = float(tt[movio][0])
        L = t_movio
        t63 = primer_cruce(y0 + 0.632 * delta_ss, desde=t_movio)
        tau = max(t63 - t_movio, 1e-3)

    return {"plant": PlantModel(K=K, tau=tau, L=L), "y0": y0, "yf": yf,
            "delta_ss": delta_ss, "n": int(len(y)), "aviso": aviso}


def _fnum(d, clave):
    try:
        return float(d[clave])
    except (KeyError, TypeError, ValueError):
        return None


def cmd_identify_abierto(args):
    datos = parse_escalon_abierto_autotune(args.log)
    marca = datos["marca"]
    pulso_min = _fnum(marca, "pulso_min")
    pulso_esc = _fnum(marca, "pulso_escalon")
    if pulso_min is None or pulso_esc is None or pulso_esc <= pulso_min:
        raise ValueError(f"Linea ESCALON_ABIERTO sin pulso_min/pulso_escalon validos: {marca}")
    duracion_s = _fnum(marca, "duracion_s") or 30.0
    delta_pulso = pulso_esc - pulso_min  # el mismo delta que usa la placa (comando, no el pulso rampeado)

    r = identify_escalon_abierto(datos["t_s"], datos["rpm"], datos["i_escalon"], delta_pulso, duracion_s,
                                 metodo=args.metodo, umbral_movimiento_rpm=args.umbral_movimiento_rpm,
                                 umbral_movimiento_frac=args.umbral_movimiento_frac,
                                 suavizado_s=args.suavizado_s)
    p = r["plant"]
    print(f"Escalon del pulso: {pulso_min:.0f} -> {pulso_esc:.0f} us (delta {delta_pulso:.0f} us), "
          f"{r['n']} muestras PID_TEST en {duracion_s:.0f} s")
    print(f"  RPM: {r['y0']:.1f} -> {r['yf']:.1f} (delta_ss={r['delta_ss']:.1f})")
    if r["aviso"]:
        print(f"  ⚠️ {r['aviso']}")
    print()
    print(f"Modelo (lazo abierto, metodo {args.metodo}):")
    print(f"  K   = {p.K:.6f}  RPM por us")
    print(f"  tau = {p.tau:.3f} s")
    print(f"  L   = {p.L:.3f} s")

    # K peor caso de la curva: del mismo log (CALIB=6/13 la imprime) o de --ganancia-log.
    k_usada = p.K
    ruta_curva = args.ganancia_log or args.log
    try:
        pulso_c, rpm_c = parse_ganancia_cal_log(ruta_curva)
        peor = peor_caso_K_ganancia_cal(pulso_c, rpm_c)
        k_usada = max(p.K, peor["k_worst"])
        print(f"  K peor caso de la curva = {peor['k_worst']:.6f} ({peor['n_pasos']} pasos) "
              f"-> K usada = {k_usada:.6f}")
    except ValueError:
        if datos["k_peor_reusada"]:
            k_usada = max(p.K, datos["k_peor_reusada"])
            print(f"  K peor caso de la curva de CALIB=5 (linea CURVA_REUSADA) = {datos['k_peor_reusada']:.6f} "
                  f"-> K usada = {k_usada:.6f}")
        else:
            print("  (sin curva en el log: se usa la K del escalon; pasar --ganancia-log con el log de CALIB=5)")

    L_usada = max(p.L, AUTOTUNE1_L_MIN_S)
    tau_c = max(AUTOTUNE1_TAU_C_FACTOR_L * L_usada, AUTOTUNE1_TAU_C_FACTOR_TAU * p.tau, AUTOTUNE1_TAU_C_MIN_S)
    kp, ki = simc_pi_gains(k_usada, p.tau, L_usada, tau_c)
    print()
    print(f"Candidata con la receta de la placa (L piso {AUTOTUNE1_L_MIN_S} s -> L={L_usada:.3f}, "
          f"tau_c = max(2L, 3*tau) = {tau_c:.3f} s):")
    print(f"  Kp = {kp:.4f}   Ki = {ki:.4f}")

    placa = datos["placa"]
    if placa:
        print()
        print("Lo que calculo la placa en esa corrida:")
        idp = placa.get("id", {})
        if idp:
            print(f"  K_escalon={idp.get('K_escalon')}  K_usada={idp.get('K_usada')}  "
                  f"tau={idp.get('tau')}  L={idp.get('L')}")
        cand = placa.get("candidata", {})
        if cand:
            print(f"  Kp={cand.get('kp')}  Ki={cand.get('ki')}")
        print("  (diferencias chicas son normales: la placa muestrea cada 50 ms, el log PID_TEST cada 200 ms)")
    print()
    print("Para ver otras filas SIMC o simular:")
    print(f"  python pid_tuning.py simc --K {k_usada:.6f} --tau {p.tau:.3f} --L {L_usada:.3f}")


# ============================================================================
# 3. Replica exacta de PID_CalcularSalidaUs() (pid.c) -- si pid.c cambia,
#    actualizar esto tambien para que la simulacion siga siendo fiel.
# ============================================================================

@dataclass
class PidState:
    integral: float = 0.0
    prev_error: float = 0.0
    first_call: bool = True


def pid_calcular_salida_us(setpoint_rpm, rpm_medida, dt_s, kp, ki, kd,
                            minimo_us, maximo_us, state: PidState):
    error = setpoint_rpm - rpm_medida
    rango = maximo_us - minimo_us

    derivada = 0.0
    if not state.first_call and dt_s > 0.0:
        derivada = (error - state.prev_error) / dt_s
    state.prev_error = error
    state.first_call = False

    if ki > 0.0:
        integral_tentativa = state.integral + error * dt_s
        salida_tentativa = kp * error + ki * integral_tentativa + kd * derivada
        satura_alto = salida_tentativa > rango
        satura_bajo = salida_tentativa < 0.0
        if not (satura_alto and error > 0.0) and not (satura_bajo and error < 0.0):
            state.integral = integral_tentativa
    else:
        state.integral = 0.0

    salida = kp * error + ki * state.integral + kd * derivada
    salida = max(0.0, min(rango, salida))

    return minimo_us + salida


# ============================================================================
# 4. Simulacion del lazo cerrado sobre el modelo FOPDT identificado
# ============================================================================

def simulate_closed_loop(plant: PlantModel, kp, ki, kd, servo_min_us, servo_max_us,
                          sp_before, sp_after, step_time_s=None,
                          sim_time_s=None, dt_s=None):
    """Simula PID_CalcularSalidaUs() (replicado arriba) cerrando el lazo
    sobre un modelo de planta de primer orden con tiempo muerto (FOPDT):
        d(delta_rpm)/dt = (K * correccion(t-L) - delta_rpm) / tau
    donde 'correccion' es el pulso aplicado por encima de servo_min_us.
    """
    if dt_s is None:
        dt_s = max(1e-3, min(0.05, plant.tau / 20.0, max(plant.L, 0.01) / 5.0))
    if step_time_s is None:
        step_time_s = max(10.0, 5.0 * plant.tau)
    if sim_time_s is None:
        sim_time_s = step_time_s + max(40.0, 10.0 * plant.tau)

    n = int(sim_time_s / dt_s)
    retraso_muestras = max(1, int(round(plant.L / dt_s)))

    t = np.arange(n) * dt_s
    setpoint = np.where(t < step_time_s, sp_before, sp_after)

    rpm = np.zeros(n)
    pulso = np.zeros(n)
    correccion_hist = np.zeros(n + retraso_muestras)

    delta_rpm = 0.0
    rpm_actual = sp_before
    state = PidState()

    for i in range(n):
        pulso_us = pid_calcular_salida_us(
            setpoint[i], rpm_actual, dt_s, kp, ki, kd, servo_min_us, servo_max_us, state)
        correccion = pulso_us - servo_min_us
        correccion_hist[i + retraso_muestras] = correccion
        correccion_retrasada = correccion_hist[i]

        delta_rpm += dt_s * (plant.K * correccion_retrasada - delta_rpm) / plant.tau
        rpm_actual = sp_before + delta_rpm

        rpm[i] = rpm_actual
        pulso[i] = pulso_us

    return t, rpm, pulso, setpoint


def cmd_simulate(args):
    plant = PlantModel(K=args.K, tau=args.tau, L=args.L)
    t, rpm, pulso, setpoint = simulate_closed_loop(
        plant, args.kp, args.ki, args.kd, args.servo_min, args.servo_max,
        args.sp_before, args.sp_before + args.step_size)

    paso = max(1, len(t) // 40)
    for i in range(0, len(t), paso):
        print(f"t={t[i]:7.2f}s  SET_RPM={setpoint[i]:7.1f}  RPM={rpm[i]:7.1f}  pulso={pulso[i]:7.1f}us")

    if args.plot_out:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
        ax1.plot(t, setpoint, "--", label="SET_RPM")
        ax1.plot(t, rpm, label="RPM simulada")
        ax1.set_ylabel("RPM")
        ax1.legend()
        ax1.grid(True)
        ax2.plot(t, pulso, label="pulso servo (us)")
        ax2.set_ylabel("us")
        ax2.set_xlabel("tiempo (s)")
        ax2.legend()
        ax2.grid(True)
        fig.tight_layout()
        fig.savefig(args.plot_out, dpi=120)
        print(f"\nGrafico guardado en {args.plot_out}")


# ============================================================================
# 5. Ziegler-Nichols en lazo cerrado, SOBRE EL MODELO SIMULADO (no el motor
#    real) -- ver README seccion 9. Heuristica de "relacion de decaimiento"
#    entre los dos primeros sobre-impulsos tras un escalon, para ubicar la
#    ganancia a partir de la cual la oscilacion deja de amortiguarse.
# ============================================================================

def _picos_locales(y):
    picos = []
    for i in range(2, len(y) - 2):
        if y[i] > y[i - 1] and y[i] >= y[i + 1] and y[i] > y[i - 2] and y[i] >= y[i + 2]:
            picos.append(i)
    return picos


def _relacion_decaimiento(plant, kp, servo_min_us, servo_max_us, sp_before, step_size):
    t, rpm, _, _ = simulate_closed_loop(
        plant, kp, 0.0, 0.0, servo_min_us, servo_max_us, sp_before, sp_before + step_size)

    idx_escalon = int(np.searchsorted(t, max(10.0, 5.0 * plant.tau)))
    post = rpm[idx_escalon:]
    post_t = t[idx_escalon:]
    valor_final = post[-1]

    picos = _picos_locales(post)
    if len(picos) < 2:
        return None, None

    desviaciones = [abs(post[p] - valor_final) for p in picos]
    if desviaciones[0] < 1e-6:
        return None, None

    relacion = desviaciones[1] / desviaciones[0]
    periodo = post_t[picos[1]] - post_t[picos[0]]
    return relacion, periodo


def find_ultimate_gain(plant: PlantModel, servo_min_us, servo_max_us,
                        sp_before=1000.0, step_size=200.0,
                        kp_min=0.01, kp_max=1000.0, tol=1e-3, max_iter=60,
                        umbral_sostenida=0.9):
    """Busqueda binaria (en escala log) de la ganancia ultima Ku sobre el
    modelo SIMULADO: el Kp mas chico a partir del cual dos sobre-impulsos
    consecutivos ya no decaen (relacion >= umbral_sostenida)."""
    relacion_hi, periodo_hi = _relacion_decaimiento(plant, kp_max, servo_min_us, servo_max_us, sp_before, step_size)
    if relacion_hi is None or relacion_hi < umbral_sostenida:
        raise ValueError(
            f"Ni con Kp={kp_max} se observa oscilacion sostenida en la simulacion "
            "-- el modelo de planta puede tener demasiado retraso/ganancia baja, "
            "probar con --kp-max mas alto."
        )

    lo, hi = kp_min, kp_max
    for _ in range(max_iter):
        mid = math.sqrt(lo * hi)
        relacion, periodo = _relacion_decaimiento(plant, mid, servo_min_us, servo_max_us, sp_before, step_size)
        if relacion is not None and relacion >= umbral_sostenida:
            hi, periodo_hi = mid, periodo
        else:
            lo = mid
        if hi / lo - 1.0 < tol:
            break

    return hi, periodo_hi


def ziegler_nichols_gains(ku, tu):
    return {
        "P":   {"Kp": 0.50 * ku},
        "PI":  {"Kp": 0.45 * ku, "Ki": 0.45 * ku / (tu / 1.2)},
        "PID": {"Kp": 0.60 * ku, "Ki": 0.60 * ku / (tu / 2.0), "Kd": 0.60 * ku * (tu / 8.0)},
    }


def cmd_tune(args):
    plant = PlantModel(K=args.K, tau=args.tau, L=args.L)
    ku, tu = find_ultimate_gain(
        plant, args.servo_min, args.servo_max,
        sp_before=args.sp_before, step_size=args.step_size, kp_max=args.kp_max)

    print(f"Ganancia ultima (simulada, NO en el motor real): Ku = {ku:.4f}   Tu = {tu:.3f}s")
    print()
    for tipo, ganancias in ziegler_nichols_gains(ku, tu).items():
        partes = "  ".join(f"{nombre}={valor:.4f}" for nombre, valor in ganancias.items())
        print(f"  {tipo:4s}: {partes}")
    print()
    print("Recomendado para este acelerador (ver README seccion 9): empezar por PI")
    print("(el ruido de medicion conocido en ralenti hace que Kd amplifique ruido).")
    print()
    print("Estas ganancias son un PUNTO DE PARTIDA de la simulacion -- validar")
    print("SIEMPRE en el motor real repitiendo el mismo escalon de SET_RPM antes")
    print("de confiar en ellas.")


# ============================================================================
# 6. SIMC (Skogestad, 2003) -- alternativa a Ziegler-Nichols pensada para
#    plantas con tiempo muerto significativo respecto a su constante de
#    tiempo (L/tau alto), como esta. A diferencia de Z-N (que persigue la
#    respuesta mas rapida posible, y por eso tiende a dar ganancias con
#    sobre-impulso justo en este tipo de planta), SIMC deja elegir a
#    proposito que tan conservador ser via tau_c (constante de tiempo
#    deseada del lazo cerrado) -- no hace falta oscilar nada para usarla,
#    solo el modelo (K, tau, L) ya identificado con `identify`.
# ============================================================================

def simc_pi_gains(K, tau, L, tau_c):
    """Formulas SIMC para PI (Skogestad 2003):
        Kc  = (1/K) * tau / (tau_c + L)
        Ti  = min(tau, 4*(tau_c + L))
    tau_c mas chico (tipicamente >= L) da un lazo mas rapido pero con
    menos margen; mas grande da un lazo mas lento pero mas robusto ante
    un modelo de planta impreciso -- justo el control que Z-N no da."""
    kc = (1.0 / K) * (tau / (tau_c + L))
    ti = min(tau, 4.0 * (tau_c + L))
    ki = kc / ti
    return kc, ki


def cmd_simc(args):
    plant = PlantModel(K=args.K, tau=args.tau, L=args.L)

    # Tres puntos sobre el espectro agresivo <-> conservador, todos
    # centrados en el tiempo muerto identificado (tau_c >= L siempre,
    # regla practica de Skogestad).
    opciones = [
        ("Agresivo (tau_c = L)",        plant.L),
        ("Medio (tau_c = 2L)",          2.0 * plant.L),
        ("Conservador (tau_c = tau)",   plant.tau),
    ]

    print(f"Modelo: K={plant.K:.6f}  tau={plant.tau:.3f}s  L={plant.L:.3f}s")
    print()
    for etiqueta, tau_c in opciones:
        kc, ki = simc_pi_gains(plant.K, plant.tau, plant.L, tau_c)
        print(f"  {etiqueta:24s} (tau_c={tau_c:.3f}s): Kp={kc:.4f}  Ki={ki:.4f}")
    print()
    print("A diferencia de `tune` (Ziegler-Nichols), estas ganancias NO buscan")
    print("la respuesta mas rapida posible -- estan pensadas para no oscilar")
    print("ni siquiera cerca del setpoint, en plantas con bastante tiempo")
    print("muerto como esta. Empezar por la fila 'Medio' y validar con")
    print("`simulate` antes de probar en el motor real; si simulate ya se ve")
    print("con sobre-impulso, probar la fila 'Conservador'.")


# ============================================================================
# 7. Pipeline de un solo comando: identify -> simc -> simulate encadenados,
#    terminando en una recomendacion lista para llevar a campo. Reduce las
#    4 corridas manuales separadas a 1 -- pensado para el flujo real
#    (capturar log en campo -> correr esto -> probar la ganancia sugerida
#    -> confirmar visualmente en el motor, o volver a correr esto con
#    --fila para la siguiente opcion si oscilo). NO aplica nada al motor
#    ni decide por si solo -- sigue siendo el operador quien confirma en
#    campo antes de cargar cualquier ganancia (ver README seccion 9).
# ============================================================================

def _resumen_respuesta_simulada(t, rpm, setpoint, step_time_s):
    """Metricas simples sobre la simulacion (sobre-impulso, asentamiento)
    para que la recomendacion final sea cuantitativa, no solo 'probablemente
    esta bien' -- asentamiento definido como +-2% del cambio total, mismo
    criterio informal que se usa a ojo en los logs de campo de este
    proyecto (ver README seccion 9)."""
    post_mask = t >= step_time_s
    sp_before = float(setpoint[~post_mask][-1]) if np.any(~post_mask) else float(setpoint[0])
    sp_after = float(setpoint[post_mask][0])
    delta = sp_after - sp_before
    if delta == 0:
        return {"sobre_impulso_pct": 0.0, "tiempo_asentamiento_s": 0.0}

    post_t = t[post_mask]
    post_rpm = rpm[post_mask]

    if delta > 0:
        pico = float(np.max(post_rpm))
        sobre_impulso_pct = max(0.0, (pico - sp_after) / delta * 100.0)
    else:
        pico = float(np.min(post_rpm))
        sobre_impulso_pct = max(0.0, (sp_after - pico) / delta * -100.0)

    banda = 0.02 * abs(delta)
    dentro_banda = np.abs(post_rpm - sp_after) <= banda
    tiempo_asentamiento_s = None
    for i in range(len(dentro_banda)):
        if np.all(dentro_banda[i:]):
            tiempo_asentamiento_s = float(post_t[i] - step_time_s)
            break
    if tiempo_asentamiento_s is None:
        tiempo_asentamiento_s = float(post_t[-1] - step_time_s)  # nunca asento dentro del log simulado

    return {"sobre_impulso_pct": sobre_impulso_pct, "tiempo_asentamiento_s": tiempo_asentamiento_s}


def cmd_auto(args):
    # --loop interno: --kp default 1.0 (agregado 2026-09-25, decision
    # explicita del usuario) -- a diferencia de --loop presion (donde la
    # Kp real usada durante el escalon de CALIB=10 varia segun que tanto ya
    # se haya sintonizado ese lazo en esa instalacion en particular), el
    # escalon de CALIB=6 SIEMPRE se hace con PID_RPM_KP=1/PID_RPM_KI=0 fijo (README
    # seccion 9) -- es una prueba de identificacion de una sola vez, no
    # tiene sentido reusar el Kp ya validado de una calibracion anterior
    # (da una señal mas debil, no mas precisa) ni pedirselo al operador
    # cada vez. Se puede seguir pasando --kp explicito si alguna vez se
    # usa un valor distinto de 1.0 para el escalon.
    if args.loop == "interno" and args.kp is None:
        args.kp = 1.0
        print("(--kp no especificado para --loop interno -- usando el default 1.0, "
              "el valor con el que siempre se hace el escalon de CALIB=6, "
              "ver README seccion 9)")
    if args.loop == "presion" and args.kp is None:
        raise ValueError(f"--kp es obligatorio para --loop {args.loop} (identificacion en lazo cerrado -- "
                          "es la Kp que estaba activa durante la prueba real, varia segun la "
                          "instalacion -- a diferencia de --loop interno, no hay un valor fijo).")
    if args.ganancia_log and args.loop == "remoto":
        raise ValueError("--ganancia-log no aplica a --loop remoto (ese lazo ya se identifica "
                          "en lazo abierto directo con identify-remoto, CALIB=12 -- "
                          "no tiene el problema de K poco confiable que --ganancia-log corrige "
                          "para interno/presion, ver README seccion 9).")

    if args.loop == "interno":
        data = parse_pid_test_log(args.log)
        resultado = identify_step(data, kp_used=args.kp, step_time_s=args.step_time_s,
                                   suavizado_s=args.suavizado_s if args.suavizado_s is not None else 0.0,
                                   umbral_movimiento_frac=args.umbral_movimiento_frac,
                                   umbral_movimiento_rpm=args.umbral_movimiento_rpm)
        _imprimir_identificacion(resultado, "RPM", "RPM", "RPM por microsegundo de correccion", len(data.t_s))

        if args.ganancia_log:
            # Reemplaza la K del escalon cerrado por el PEOR CASO del barrido
            # en lazo abierto (CALIB=5) -- mismo criterio usado
            # a mano en README seccion 9: un solo escalon da una K de UN
            # punto del rango (y puede salir poco confiable si la señal fue
            # debil con esa Kp), el barrido cubre TODO el rango y expone si
            # la ganancia varia -- un Kp/Ki que solo se probo en la zona de
            # menor ganancia puede oscilar en la de mayor ganancia, como
            # paso de verdad en el motor real (ver README).
            pulso_ganancia, rpm_ganancia = parse_ganancia_cal_log(args.ganancia_log)
            peor_caso = peor_caso_K_ganancia_cal(pulso_ganancia, rpm_ganancia)
            print()
            print(f"--- Barrido CALIB=5 ({peor_caso['n_pasos']} pasos): "
                  f"K entre {peor_caso['k_min']:.4f} y {peor_caso['k_max']:.4f} RPM/us "
                  f"({peor_caso['k_max']/max(peor_caso['k_min'], 1e-9):.1f}x de variacion) ---")
            print(f"  Usando el PEOR CASO K={peor_caso['k_worst']:.6f} (en pulso={peor_caso['pulso_en_peor']:.0f}us) "
                  f"en vez de la K={resultado.plant.K:.6f} del escalon cerrado -- mas conservador,")
            print("  cubre toda la zona del rango, no solo donde se hizo el escalon.")
            resultado.plant.K = peor_caso["k_worst"]
    elif args.loop == "presion":
        data = parse_presion_pid_test_log(args.log)
        resultado = identify_step_presion(data, kp_used=args.kp, step_time_s=args.step_time_s,
                                           suavizado_s=args.suavizado_s if args.suavizado_s is not None else 3.0,
                                           umbral_movimiento_frac=args.umbral_movimiento_frac,
                                           umbral_movimiento_psi=args.umbral_movimiento_rpm
                                           if args.umbral_movimiento_rpm != 5.0 else 0.3)
        _imprimir_identificacion(resultado, "PSI (PRESION_OBJETIVO_LOCAL)", "PSI", "PSI por RPM de correccion", len(data.t_s))

        if args.ganancia_log:
            # Mismo criterio que --loop interno de arriba (ver ese bloque):
            # reemplaza la K del escalon CERRADO (CALIB=10/
            # PRESION_CAL, presion_pid.c corriendo) por el PEOR CASO del
            # barrido en lazo ABIERTO (CALIB=9/
            # PRESION_GANANCIA_CAL, presion_pid.c NO corre en ese modo --
            # ver README seccion 9 y main.c).
            rpm_ganancia, psi_ganancia = parse_presion_ganancia_cal_log(args.ganancia_log)
            peor_caso = peor_caso_K_presion_ganancia_cal(rpm_ganancia, psi_ganancia)
            print()
            print(f"--- Barrido CALIB=9 ({peor_caso['n_pasos']} pasos): "
                  f"K entre {peor_caso['k_min']:.4f} y {peor_caso['k_max']:.4f} PSI/RPM "
                  f"({peor_caso['k_max']/max(peor_caso['k_min'], 1e-9):.1f}x de variacion) ---")
            print(f"  Usando el PEOR CASO K={peor_caso['k_worst']:.6f} (en rpm={peor_caso['rpm_en_peor']:.0f}) "
                  f"en vez de la K={resultado.plant.K:.6f} del escalon cerrado -- mas conservador,")
            print("  cubre toda la zona del rango, no solo donde se hizo el escalon.")
            resultado.plant.K = peor_caso["k_worst"]
    else:  # remoto
        data = parse_remoto_pid_test_log(args.log)
        resultado = identify_step_remoto(data, step_time_s=args.step_time_s,
                                          suavizado_s=args.suavizado_s if args.suavizado_s is not None else 0.0,
                                          umbral_movimiento_frac=args.umbral_movimiento_frac,
                                          umbral_movimiento_psi=args.umbral_movimiento_rpm
                                          if args.umbral_movimiento_rpm != 5.0 else 0.3)
        _imprimir_identificacion_abierto(resultado, "RPM (SET_RPM)", "PSI (remota)", "PSI por RPM", len(data.t_s))

    plant = resultado.plant
    print()
    print(f"--- SIMC sobre el modelo identificado (K={plant.K:.6f}  tau={plant.tau:.3f}s  L={plant.L:.3f}s) ---")
    opciones = [
        ("Agresivo", plant.L),
        ("Medio", 2.0 * plant.L),
        ("Conservador", plant.tau),
    ]
    filas = {}
    for etiqueta, tau_c in opciones:
        kc, ki = simc_pi_gains(plant.K, plant.tau, plant.L, tau_c)
        filas[etiqueta] = (kc, ki, tau_c)
        marca = " <--" if etiqueta.lower() == args.fila else ""
        print(f"  {etiqueta:12s} (tau_c={tau_c:6.3f}s): Kp={kc:.4f}  Ki={ki:.4f}{marca}")

    etiqueta_elegida = {"agresivo": "Agresivo", "medio": "Medio", "conservador": "Conservador"}[args.fila]
    kp_elegida, ki_elegida, _ = filas[etiqueta_elegida]

    servo_min = args.servo_min
    servo_max = args.servo_max
    sp_before = args.sp_before
    step_size = args.step_size

    t, rpm, _, setpoint = simulate_closed_loop(
        plant, kp_elegida, ki_elegida, 0.0, servo_min, servo_max,
        sp_before, sp_before + step_size)
    step_time_sim = max(10.0, 5.0 * plant.tau)
    resumen = _resumen_respuesta_simulada(t, rpm, setpoint, step_time_sim)

    print()
    print(f"--- Recomendacion (fila '{etiqueta_elegida}') ---")
    print(f"  Kp = {kp_elegida:.4f}   Ki = {ki_elegida:.4f}")
    print(f"  Simulado (sobre el modelo, NO el motor real): sobre-impulso ~{resumen['sobre_impulso_pct']:.1f}%, "
          f"asienta en ~{resumen['tiempo_asentamiento_s']:.1f}s tras el escalon.")
    if resumen["sobre_impulso_pct"] > 15.0:
        print("  ADVERTENCIA: la simulacion muestra sobre-impulso notable -- considerar la fila")
        print("  'Conservador' antes de probar esta en el motor real (--fila conservador).")
    print()
    print("  Probar esta ganancia EN CAMPO y confirmar visualmente:")
    print("    - Si converge sin oscilar como lo de arriba: dejarla fija (downlink de las ganancias).")
    print(f"    - Si oscila/se pasa de largo: volver a correr con --fila conservador"
          if args.fila != "conservador" else
          "    - Si TODAVIA oscila en 'Conservador': el modelo identificado puede no ser confiable,"
          " revisar el log fuente antes de seguir.")
    print("  Esto NO se aplica solo -- es una sugerencia para que confirmes en campo, no una decision automatica.")


# ============================================================================
# 8. compare -- verificar cuanto se parecio la respuesta REAL del motor
#    (con Kp/Ki YA aplicados y corriendo, log normal PID_TEST/
#    PRESION_PID_TEST/REMOTO_PID_TEST) a la respuesta que predice la
#    SIMULACION con el mismo modelo (K, tau, L) y las mismas ganancias.
#    A diferencia de identify_step*/auto (que exigen Ki=0 durante la
#    prueba, es un requisito matematico para poder despejar K), aca el
#    log YA tiene el PID completo (Kp+Ki) corriendo -- no se identifica
#    nada nuevo, solo se compara lo medido contra lo esperado.
# ============================================================================

def _extraer_baseline_final(t_s, cmd, resp, step_time_s, pre_window_s, settle_window_s):
    """Mismo promedio de ventana que usan los identify_step* (ver arriba),
    pero sin ninguna de las suposiciones de lazo abierto/cerrado puro --
    aca el 'cmd' y 'resp' pueden venir de un log con Ki activo."""
    pre_mask = (t_s >= step_time_s - pre_window_s) & (t_s < step_time_s)
    if not np.any(pre_mask):
        raise ValueError(
            "No hay suficientes datos ANTES del escalon -- capturar mas tiempo "
            f"en reposo antes del escalon (pre_window_s={pre_window_s}s)."
        )
    baseline_cmd = float(np.mean(cmd[pre_mask]))
    baseline_resp = float(np.mean(resp[pre_mask]))

    post_mask = t_s >= step_time_s
    settle_mask = post_mask & (t_s >= t_s[-1] - settle_window_s)
    if not np.any(settle_mask):
        raise ValueError(
            "No hay suficientes datos AL FINAL del log -- capturar hasta que "
            f"la respuesta se estabilice (settle_window_s={settle_window_s}s)."
        )
    final_cmd = float(np.mean(cmd[settle_mask]))
    final_resp = float(np.mean(resp[settle_mask]))
    return baseline_cmd, final_cmd, baseline_resp, final_resp


def _metricas_respuesta(t, resp, sp_before, sp_after, step_time_s):
    """Sobre-impulso, tiempo de asentamiento (banda +-2%, mismo criterio que
    _resumen_respuesta_simulada) y error de estado estacionario -- version
    generica que sirve tanto para la curva REAL como para la SIMULADA
    (a diferencia de _resumen_respuesta_simulada, no asume un array
    'setpoint' escalon, recibe sp_before/sp_after ya conocidos)."""
    delta = sp_after - sp_before
    if delta == 0:
        return {"sobre_impulso_pct": 0.0, "tiempo_asentamiento_s": 0.0,
                "valor_final": float(resp[-1]) if len(resp) else float("nan"),
                "error_estacionario": 0.0, "error_estacionario_pct": 0.0}

    post_mask = t >= step_time_s
    post_t = t[post_mask]
    post_resp = resp[post_mask]
    if len(post_resp) == 0:
        raise ValueError("No hay datos despues del escalon para calcular metricas.")

    if delta > 0:
        pico = float(np.max(post_resp))
        sobre_impulso_pct = max(0.0, (pico - sp_after) / delta * 100.0)
    else:
        pico = float(np.min(post_resp))
        sobre_impulso_pct = max(0.0, (sp_after - pico) / delta * -100.0)

    banda = 0.02 * abs(delta)
    dentro_banda = np.abs(post_resp - sp_after) <= banda
    tiempo_asentamiento_s = None
    for i in range(len(dentro_banda)):
        if np.all(dentro_banda[i:]):
            tiempo_asentamiento_s = float(post_t[i] - step_time_s)
            break
    if tiempo_asentamiento_s is None:
        tiempo_asentamiento_s = float(post_t[-1] - step_time_s)  # nunca asento dentro del log

    # valor_final = promedio del ultimo 10% de las muestras post-escalon, no
    # solo la ultima muestra -- mas robusto ante ruido de la ultima lectura.
    n_final = max(1, len(post_resp) // 10)
    valor_final = float(np.mean(post_resp[-n_final:]))
    error_estacionario = valor_final - sp_after
    error_estacionario_pct = (error_estacionario / delta * 100.0) if delta != 0 else 0.0

    return {
        "sobre_impulso_pct": sobre_impulso_pct,
        "tiempo_asentamiento_s": tiempo_asentamiento_s,
        "valor_final": valor_final,
        "error_estacionario": error_estacionario,
        "error_estacionario_pct": error_estacionario_pct,
    }


def _cargar_log_para_comparar(loop, log_path):
    """Devuelve (t_s, cmd, resp, salto_minimo, nombre_cmd, unidad_resp,
    pre_window_default, settle_window_default) segun el lazo -- mismos
    parsers/umbrales que ya usan identify/identify-presion/identify-remoto,
    reusados aca sin la parte de identificacion FOPDT (ver cabecera de
    esta seccion)."""
    if loop == "interno":
        data = parse_pid_test_log(log_path)
        return (data.t_s, data.set_rpm, data.rpm, 20.0, "SET_RPM", "RPM", 2.0, 3.0)
    elif loop == "presion":
        data = parse_presion_pid_test_log(log_path)
        return (data.t_s, data.objetivo, data.presion, 1.0, "PRESION_OBJETIVO_LOCAL", "PSI", 5.0, 15.0)
    else:  # remoto
        data = parse_remoto_pid_test_log(log_path)
        return (data.t_s, data.set_rpm, data.presion_remoto, 20.0, "SET_RPM", "PSI (remota)", 1800.0, 600.0)


def cmd_compare(args):
    t_s, cmd, resp, salto_minimo, nombre_cmd, unidad_resp, pre_def, settle_def = \
        _cargar_log_para_comparar(args.loop, args.log)
    pre_window_s = args.pre_window_s if args.pre_window_s is not None else pre_def
    settle_window_s = args.settle_window_s if args.settle_window_s is not None else settle_def

    if args.step_time_s is not None:
        step_time_s = args.step_time_s
    else:
        idx = _detectar_escalon_generico(t_s, cmd, salto_minimo, nombre_cmd)
        step_time_s = float(t_s[idx])

    baseline_cmd, final_cmd, baseline_resp, final_resp = _extraer_baseline_final(
        t_s, cmd, resp, step_time_s, pre_window_s, settle_window_s)

    print(f"Escalon real en t={step_time_s:.2f}s: {nombre_cmd} {baseline_cmd:.2f} -> {final_cmd:.2f}")
    print(f"  Respuesta real ({unidad_resp}): {baseline_resp:.2f} -> {final_resp:.2f}")

    plant = PlantModel(K=args.K, tau=args.tau, L=args.L)
    t_sim, resp_sim, _, _ = simulate_closed_loop(
        plant, args.kp, args.ki, args.kd, args.servo_min, args.servo_max,
        baseline_cmd, final_cmd, step_time_s=step_time_s, sim_time_s=float(t_s[-1]))

    metricas_real = _metricas_respuesta(t_s, resp, baseline_cmd, final_cmd, step_time_s)
    metricas_sim = _metricas_respuesta(t_sim, resp_sim, baseline_cmd, final_cmd, step_time_s)

    resp_real_en_grilla_sim = np.interp(t_sim, t_s, resp)
    post_mask_sim = t_sim >= step_time_s
    residuo = resp_real_en_grilla_sim[post_mask_sim] - resp_sim[post_mask_sim]

    delta_cmd = final_cmd - baseline_cmd
    if delta_cmd == 0:
        raise ValueError("delta de setpoint salio 0 -- no se detecto un escalon real valido.")
    rmse = float(np.sqrt(np.mean(residuo ** 2)))
    rmse_norm = min(rmse / abs(delta_cmd), 1.0)
    pct_parecido = 100.0 * (1.0 - rmse_norm)

    # Ventana de asentamiento para la deteccion de oscilacion: se salta el
    # tiempo muerto + una constante de tiempo (L+tau) del transitorio
    # esperado, para no contar como "oscilacion" el cruce normal del
    # residuo mientras ambas curvas todavia estan subiendo -- si despues
    # de ese punto el residuo sigue cambiando de signo varias veces, es
    # que el real se quedo oscilando alrededor de lo que predice el
    # modelo (que ya deberia estar asentado o asentandose monotonamente).
    t_sim_post = t_sim[post_mask_sim]
    inicio_asentamiento = step_time_s + plant.L + plant.tau
    mask_asentamiento = t_sim_post >= inicio_asentamiento
    residuo_asentamiento = residuo[mask_asentamiento] if np.any(mask_asentamiento) else residuo

    signos = np.sign(residuo_asentamiento)
    signos_no_cero = signos[signos != 0]
    cruces = int(np.sum(np.diff(signos_no_cero) != 0)) if len(signos_no_cero) > 1 else 0

    print()
    print(f"--- Comparacion contra el modelo simulado (K={plant.K:.6f}  tau={plant.tau:.3f}s  "
          f"L={plant.L:.3f}s  Kp={args.kp:.4f}  Ki={args.ki:.4f}) ---")
    print(f"  Parecido (100% - error cuadratico medio normalizado): {pct_parecido:.1f}%")
    print()
    print(f"  {'':20s}{'Real':>12s}{'Simulado':>12s}")
    print(f"  {'Sobre-impulso (%)':20s}{metricas_real['sobre_impulso_pct']:>12.1f}"
          f"{metricas_sim['sobre_impulso_pct']:>12.1f}")
    print(f"  {'Asentamiento (s)':20s}{metricas_real['tiempo_asentamiento_s']:>12.1f}"
          f"{metricas_sim['tiempo_asentamiento_s']:>12.1f}")
    print(f"  {'Error estacionario':20s}{metricas_real['error_estacionario']:>12.2f}"
          f"{metricas_sim['error_estacionario']:>12.2f}")
    print()
    if cruces >= 3:
        print(f"  ADVERTENCIA: posible oscilacion real no prevista por el modelo "
              f"({cruces} cruces de signo del residuo en la ventana de asentamiento).")
        print("  El modelo simulado no predice este comportamiento -- revisar visualmente")
        print("  el grafico (--plot-out) antes de confirmar esta ganancia en campo.")
    else:
        print(f"  Sin indicios de oscilacion no prevista ({cruces} cruces de signo del "
              "residuo en la ventana de asentamiento).")

    if args.plot_out:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
        ax1.plot(t_s, resp, label="Real", alpha=0.8)
        ax1.plot(t_sim, resp_sim, "--", label="Simulado")
        ax1.axhline(final_cmd, color="gray", linestyle=":", linewidth=1, label="Setpoint final")
        ax1.set_ylabel(unidad_resp)
        ax1.legend()
        ax1.grid(True)
        ax2.plot(t_sim[post_mask_sim], residuo, label="Residuo (real - simulado)")
        ax2.axhline(0.0, color="gray", linestyle=":", linewidth=1)
        ax2.set_ylabel(f"Residuo ({unidad_resp})")
        ax2.set_xlabel("tiempo (s)")
        ax2.legend()
        ax2.grid(True)
        fig.tight_layout()
        fig.savefig(args.plot_out, dpi=120)
        print(f"\nGrafico guardado en {args.plot_out}")


# ============================================================================
# CLI
# ============================================================================

def build_parser():
    p = argparse.ArgumentParser(
        description="Identificacion y simulacion del PID de RIO-DSL (ver README seccion 9).")
    sub = p.add_subparsers(dest="comando", required=True)

    p_cap = sub.add_parser("capture", help="Capturar el log del puerto serie a un archivo (requiere pyserial).")
    p_cap.add_argument("--port", required=True, help="Puerto serie, ej. COM5")
    p_cap.add_argument("--baud", type=int, default=115200)
    p_cap.add_argument("--out", required=True, help="Archivo donde guardar el log capturado")
    p_cap.set_defaults(func=cmd_capture)

    p_id = sub.add_parser("identify", help="Identificar el modelo de planta (K, tau, L) a partir de un escalon real.")
    p_id.add_argument("--log", required=True, help="Archivo de log capturado (o su recorte)")
    p_id.add_argument("--kp", type=float, required=True, help="PID_RPM_KP que estaba activo durante la prueba")
    p_id.add_argument("--suavizado-s", type=float, default=0.0,
                       help="Promedio movil (segundos) sobre la respuesta antes de detectar L/tau -- 0 = sin suavizar (default, el tacometro es poco ruidoso). Subir si L/tau salen poco creibles.")
    p_id.add_argument("--umbral-movimiento-frac", type=float, default=0.2,
                       help="Umbral de 'se movio' como fraccion del cambio real medido (default 0.2 = 20%%) -- el umbral EFECTIVO es el mayor entre este y --umbral-movimiento-rpm (ese ultimo solo actua como piso para no disparar con puro ruido).")
    p_id.add_argument("--umbral-movimiento-rpm", type=float, default=5.0,
                       help="Piso absoluto (RPM) del umbral de 'se movio', ver --umbral-movimiento-frac.")
    p_id.add_argument("--step-time-s", type=float, default=None,
                       help="Instante del escalon en segundos relativos al log (autodetectado si se omite)")
    p_id.set_defaults(func=cmd_identify)

    p_id_p = sub.add_parser("identify-presion",
                             help="Identificar el modelo de planta (K, tau, L) del lazo de PRESION a partir de un escalon real de PRESION_OBJETIVO_LOCAL.")
    p_id_p.add_argument("--log", required=True, help="Archivo de log capturado (o su recorte)")
    p_id_p.add_argument("--kp", type=float, required=True, help="PID_PSI_KP que estaba activo durante la prueba (PID_PSI_KI debe haber sido 0)")
    p_id_p.add_argument("--suavizado-s", type=float, default=3.0,
                         help="Promedio movil (segundos) sobre la presion antes de detectar L/tau -- default 3.0 (el sensor de presion es bastante mas ruidoso que el tacometro). Subir mas si L/tau siguen saliendo poco creibles, bajar si diluye un cambio real chico.")
    p_id_p.add_argument("--umbral-movimiento-frac", type=float, default=0.2,
                         help="Umbral de 'se movio' como fraccion del cambio real medido (default 0.2 = 20%%) -- el umbral EFECTIVO es el mayor entre este y --umbral-movimiento-psi (ese ultimo solo actua como piso para no disparar con puro ruido).")
    p_id_p.add_argument("--umbral-movimiento-psi", type=float, default=0.3,
                         help="Piso absoluto (PSI) del umbral de 'se movio', ver --umbral-movimiento-frac.")
    p_id_p.add_argument("--step-time-s", type=float, default=None,
                         help="Instante del escalon en segundos relativos al log (autodetectado si se omite)")
    p_id_p.set_defaults(func=cmd_identify_presion)

    p_id_a = sub.add_parser("identify-abierto",
                             help="Identificar (K, tau, L) del lazo INTERNO a partir del escalon del pulso EN LAZO ABIERTO de la autosintonia (log de CALIB=6 o CALIB=13), y comparar con lo que calculo la placa.")
    p_id_a.add_argument("--log", required=True, help="Log capturado de una corrida de CALIB=6 o 13 (debe tener la linea AUTOTUNE_PID1,ESCALON_ABIERTO)")
    p_id_a.add_argument("--ganancia-log", default=None,
                         help="Log con la curva GANANCIA_CAL (default: el mismo --log, que ya la trae)")
    p_id_a.add_argument("--metodo", default="dos-puntos", choices=["dos-puntos", "umbral"],
                         help="dos-puntos = el de la placa (28%%/63%%, sin sesgo en tau); umbral = el de identify/identify-remoto")
    p_id_a.add_argument("--suavizado-s", type=float, default=0.0,
                         help="Promedio movil (segundos) antes de buscar los cruces -- default 0")
    p_id_a.add_argument("--umbral-movimiento-frac", type=float, default=0.2,
                         help="Solo --metodo umbral: fraccion del cambio total para 'se movio'")
    p_id_a.add_argument("--umbral-movimiento-rpm", type=float, default=5.0,
                         help="Piso absoluto (RPM) del umbral de movimiento, y del chequeo de 'ya asento'")
    p_id_a.set_defaults(func=cmd_identify_abierto)

    p_id_r = sub.add_parser("identify-remoto",
                             help="Identificar el modelo de planta (K, tau, L) del lazo REMOTO (MODO=2) EN LAZO ABIERTO, a partir de un escalon real de SET_RPM en MODO=3.")
    p_id_r.add_argument("--log", required=True, help="Archivo de log capturado (o su recorte)")
    p_id_r.add_argument("--pre-window-s", type=float, default=1800.0,
                         help="Ventana ANTES del escalon para promediar el baseline, en segundos (default 1800s=30min -- el aspersor puede tardar 20-25min en reportar de nuevo en 0 PSI).")
    p_id_r.add_argument("--settle-window-s", type=float, default=600.0,
                         help="Ventana AL FINAL del log para promediar el valor estabilizado, en segundos (default 600s=10min).")
    p_id_r.add_argument("--suavizado-s", type=float, default=0.0,
                         help="Promedio movil (segundos) antes de detectar L/tau -- default 0 (sin suavizar, cada reporte remoto ya es una lectura discreta/deliberada, no ruido de ADC).")
    p_id_r.add_argument("--umbral-movimiento-frac", type=float, default=0.2,
                         help="Umbral de 'se movio' como fraccion del cambio real medido (default 0.2 = 20%%).")
    p_id_r.add_argument("--umbral-movimiento-psi", type=float, default=0.3,
                         help="Piso absoluto (PSI) del umbral de 'se movio'.")
    p_id_r.add_argument("--step-time-s", type=float, default=None,
                         help="Instante del escalon en segundos relativos al log (autodetectado si se omite)")
    p_id_r.set_defaults(func=cmd_identify_remoto)

    p_sug = sub.add_parser("sugerir-kp-presion",
                            help="A partir del barrido de CALIB=9 (lazo abierto), sugerir un PID_PSI_KP de prueba para el escalon cerrado de CALIB=10 -- reemplaza el 'probar un valor a ojo' por un numero derivado de la ganancia real ya medida en esa instalacion.")
    p_sug.add_argument("--ganancia-log", required=True,
                        help="Log del barrido CALIB=9 ('PRESION_GANANCIA_CAL,PASO,...')")
    p_sug.add_argument("--g-cl-objetivo", type=float, default=0.3,
                        help="Ganancia en lazo cerrado (G_cl, entre 0 y 1) que se busca en la zona de MENOR ganancia del barrido -- default 0.3, un punto medio con buena señal sin acercarse a la inestabilidad numerica de la identificacion. Bajar (ej. 0.2) si el barrido muestra mucha variacion de ganancia y el Kp sugerido da una advertencia.")
    p_sug.set_defaults(func=cmd_sugerir_kp_presion)

    p_tune = sub.add_parser("tune", help="Buscar Ku/Tu (Ziegler-Nichols) sobre el modelo simulado y sugerir ganancias.")
    p_tune.add_argument("--K", type=float, required=True)
    p_tune.add_argument("--tau", type=float, required=True)
    p_tune.add_argument("--L", type=float, required=True)
    p_tune.add_argument("--servo-min", type=float, default=1000.0, help="SERVO_PULSO_MIN actual (us)")
    p_tune.add_argument("--servo-max", type=float, default=2000.0, help="SERVO_PULSO_MAX actual (us)")
    p_tune.add_argument("--sp-before", type=float, default=1000.0, help="RPM base simulada antes del escalon")
    p_tune.add_argument("--step-size", type=float, default=200.0, help="Tamano del escalon de SET_RPM simulado")
    p_tune.add_argument("--kp-max", type=float, default=1000.0, help="Techo de busqueda de Ku")
    p_tune.set_defaults(func=cmd_tune)

    p_simc = sub.add_parser("simc", help="Ganancias PI via SIMC (Skogestad) -- alternativa a Ziegler-Nichols para plantas con bastante tiempo muerto.")
    p_simc.add_argument("--K", type=float, required=True)
    p_simc.add_argument("--tau", type=float, required=True)
    p_simc.add_argument("--L", type=float, required=True)
    p_simc.set_defaults(func=cmd_simc)

    p_sim = sub.add_parser("simulate", help="Previsualizar la respuesta de una combinacion de ganancias sobre el modelo.")
    p_sim.add_argument("--K", type=float, required=True)
    p_sim.add_argument("--tau", type=float, required=True)
    p_sim.add_argument("--L", type=float, required=True)
    p_sim.add_argument("--kp", type=float, required=True)
    p_sim.add_argument("--ki", type=float, default=0.0)
    p_sim.add_argument("--kd", type=float, default=0.0)
    p_sim.add_argument("--servo-min", type=float, default=1000.0)
    p_sim.add_argument("--servo-max", type=float, default=2000.0)
    p_sim.add_argument("--sp-before", type=float, default=1000.0)
    p_sim.add_argument("--step-size", type=float, default=200.0)
    p_sim.add_argument("--plot-out", default=None, help="Si se da, guarda un PNG con la respuesta (requiere matplotlib)")
    p_sim.set_defaults(func=cmd_simulate)

    p_auto = sub.add_parser("auto",
                             help="Pipeline de un solo comando: identify -> simc -> simulate, terminando en una ganancia sugerida lista para probar en campo (ver README seccion 9). NO aplica nada -- la confirmacion final sigue siendo del operador en el motor real.")
    p_auto.add_argument("--loop", required=True, choices=["interno", "presion", "remoto"],
                         help="Cual de los 3 lazos: 'interno' (RPM->servo, PID_TEST), 'presion' (MODO=1, PRESION_PID_TEST), 'remoto' (MODO=2, REMOTO_PID_TEST, lazo abierto).")
    p_auto.add_argument("--log", required=True, help="Archivo de log capturado")
    p_auto.add_argument("--kp", type=float, default=None,
                         help="Kp usada durante la prueba (lazo cerrado). Para --loop interno, default 1.0 si se omite (escalon cerrado viejo con Kp=1/Ki=0; para logs de CALIB=6 desde 2026-10-01, lazo abierto, usar identify-abierto). REQUERIDO para --loop presion (varia segun la instalacion). Se ignora para --loop remoto, lazo abierto.")
    p_auto.add_argument("--ganancia-log", default=None,
                         help="Para --loop interno o --loop presion (no aplica a remoto): archivo con el barrido en lazo abierto correspondiente -- CALIB=5 ('GANANCIA_CAL,PASO,...') para interno, CALIB=9 ('PRESION_GANANCIA_CAL,PASO,...') para presion. Si se da, se usa el PEOR CASO de K de ese barrido en vez de la K del escalon cerrado -- mas robusto si la ganancia varia con el punto de operacion (ver README seccion 9).")
    p_auto.add_argument("--fila", default="medio", choices=["agresivo", "medio", "conservador"],
                         help="Cual de las 3 filas SIMC recomendar por default (default 'medio').")
    p_auto.add_argument("--step-time-s", type=float, default=None,
                         help="Instante del escalon en segundos relativos al log (autodetectado si se omite -- para --loop remoto, mejor pasarlo a mano con el tick del downlink real, ver identify-remoto).")
    p_auto.add_argument("--suavizado-s", type=float, default=None,
                         help="Promedio movil (segundos) antes de detectar L/tau -- default por lazo (0 para interno/remoto, 3.0 para presion).")
    p_auto.add_argument("--umbral-movimiento-frac", type=float, default=0.2,
                         help="Umbral de 'se movio' como fraccion del cambio real medido (default 0.2 = 20%%).")
    p_auto.add_argument("--umbral-movimiento-rpm", type=float, default=5.0,
                         help="Piso absoluto del umbral de 'se movio' -- en RPM para --loop interno, en PSI para presion/remoto (default 0.3 PSI en esos dos si no se pasa explicito).")
    p_auto.add_argument("--servo-min", type=float, default=1000.0,
                         help="Piso de la salida simulada: SERVO_PULSO_MIN (us) para --loop interno, o RPM_MIN para presion/remoto (esos dos lazos usan RPM_MIN como piso feedforward, ver presion_pid.c/presion_pid_remoto.c).")
    p_auto.add_argument("--servo-max", type=float, default=2000.0,
                         help="Techo de la salida simulada: SERVO_PULSO_MAX (us) para --loop interno, o RPM_MAX para presion/remoto.")
    p_auto.add_argument("--sp-before", type=float, default=1000.0, help="Setpoint base simulado antes del escalon")
    p_auto.add_argument("--step-size", type=float, default=200.0, help="Tamano del escalon simulado")
    p_auto.set_defaults(func=cmd_auto)

    p_cmp = sub.add_parser("compare",
                            help="Comparar la respuesta REAL del motor (log con Kp/Ki YA aplicados) "
                                 "contra lo que predice la simulacion con el mismo modelo y ganancias -- "
                                 "da un %% de parecido y avisa si el real oscilo sin que el modelo lo previera.")
    p_cmp.add_argument("--loop", required=True, choices=["interno", "presion", "remoto"],
                        help="Cual de los 3 lazos: 'interno' (PID_TEST), 'presion' (PRESION_PID_TEST), 'remoto' (REMOTO_PID_TEST).")
    p_cmp.add_argument("--log", required=True,
                        help="Log capturado con el motor corriendo YA con las ganancias --kp/--ki cargadas (no un log de identificacion con Ki=0).")
    p_cmp.add_argument("--K", type=float, required=True)
    p_cmp.add_argument("--tau", type=float, required=True)
    p_cmp.add_argument("--L", type=float, required=True)
    p_cmp.add_argument("--kp", type=float, required=True, help="Kp que estaba activa durante la captura de --log")
    p_cmp.add_argument("--ki", type=float, default=0.0, help="Ki que estaba activa durante la captura de --log")
    p_cmp.add_argument("--kd", type=float, default=0.0)
    p_cmp.add_argument("--servo-min", type=float, default=1000.0,
                        help="Piso de la salida simulada: SERVO_PULSO_MIN (us) para --loop interno, o RPM_MIN para presion/remoto.")
    p_cmp.add_argument("--servo-max", type=float, default=2000.0,
                        help="Techo de la salida simulada: SERVO_PULSO_MAX (us) para --loop interno, o RPM_MAX para presion/remoto.")
    p_cmp.add_argument("--step-time-s", type=float, default=None,
                        help="Instante del escalon en segundos relativos al log (autodetectado si se omite)")
    p_cmp.add_argument("--pre-window-s", type=float, default=None,
                        help="Ventana ANTES del escalon para promediar el baseline -- default por lazo (2s interno, 5s presion, 1800s remoto).")
    p_cmp.add_argument("--settle-window-s", type=float, default=None,
                        help="Ventana AL FINAL del log para promediar el valor estabilizado -- default por lazo (3s interno, 15s presion, 600s remoto).")
    p_cmp.add_argument("--plot-out", default=None,
                        help="Si se da, guarda un PNG con la curva real vs simulada y el residuo (requiere matplotlib)")
    p_cmp.set_defaults(func=cmd_compare)

    return p


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        args.func(args)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
