# pid_tuning.py

Herramienta de identificación/simulación del PID de RIO-DSL, sin MATLAB.
Ver la sección 9 del `README.md` del repo para el procedimiento completo
y la derivación de las fórmulas usadas acá — esto es la implementación,
no un documento aparte.

```
python -m venv venv
venv\Scripts\activate          (o source venv/bin/activate en Linux/Mac)
pip install -r requirements.txt
```

## Flujo típico

1. **Capturar un escalón real** en el motor (motor estable, `PID_RPM_KP=1`,
   `PID_RPM_KI=0`, `PID_KD=0`, mandar un `SET_RPM` moderado y esperar a que
   se estabilice — ver README sección 9):
   ```
   python pid_tuning.py capture --port COM5 --out captura_2026-08-31.log
   ```
   (o simplemente guardá el log desde PuTTY/Tera Term a mano, el formato
   de archivo es el mismo: texto plano con las líneas `PID_TEST,...`
   intercaladas con el resto del log).

2. **Identificar el modelo de planta** (K, tau, L) a partir de ese
   escalón:
   ```
   python pid_tuning.py identify --log captura_2026-08-31.log --kp 1.0
   ```

3. **Buscar ganancias**. Dos métodos disponibles:
   - `tune` (Ziegler-Nichols en lazo cerrado, sobre el modelo simulado —
     NO oscila el motor real). **Confirmado en campo (2026-09-01): en
     plantas con tiempo muerto significativo respecto a `tau`, esto
     tiende a dar ganancias que oscilan cerca del setpoint, sin importar
     qué tan chico se ponga `Ki`** — probar primero `simc` si el modelo
     identificado tiene `L` grande respecto a `tau`.
     ```
     python pid_tuning.py tune --K <K> --tau <tau> --L <L> --servo-min 925 --servo-max 2000
     ```
   - `simc` (Skogestad 2003 — alternativa pensada justo para tiempo
     muerto alto, deja elegir qué tan conservador ser en vez de
     perseguir la respuesta más rápida):
     ```
     python pid_tuning.py simc --K <K> --tau <tau> --L <L>
     ```

4. **Previsualizar** una combinación de ganancias antes de cargarla al
   motor real:
   ```
   python pid_tuning.py simulate --K <K> --tau <tau> --L <L> --kp <Kp> --ki <Ki> --kd <Kd> --plot-out respuesta.png
   ```

5. Cargar la combinación elegida por downlink o por el mando manual de
   serial (`comando_serial.c/h`, ver README sección 2.7) y **validar
   siempre en el motor real** repitiendo el escalón del paso 1.

## Revisar en la PC lo que hizo el autotune de la placa (2026-10-01)

Desde 2026-10-01 las calibraciones `CALIB` 5-11 son etapas de los autotunes
del firmware (README del repo, sección 9) y el escalón del lazo interno ya
no es cerrado con Kp=1 sino **en lazo abierto** (escalón del pulso):

- **PID#1, log de `CALIB=6` o `13`** — mismo método que la placa (dos
  puntos 28 %/63 %, cruces interpolados porque `PID_TEST` va cada 200 ms),
  misma receta SIMC, y compara contra lo que imprimió la placa. Se ancla a
  la línea `AUTOTUNE_PID1,ESCALON_ABIERTO`; la K peor caso la toma de la
  curva del mismo log, de la línea `CURVA_REUSADA` o de `--ganancia-log`:
  ```
  python pid_tuning.py identify-abierto --log captura_calib6.log
  ```
- **PID#2, log de `CALIB=10` o `14`** — el escalón sigue siendo cerrado
  (+5 PSI con Ki=0), con la Kp de prueba que calculó `CALIB=9` (aparece en
  la línea `AUTOTUNE_PID2,CURVA,...,kp_prueba=`):
  ```
  python pid_tuning.py identify-presion --log captura_calib10.log --kp <kp_prueba>
  ```

## Limitaciones conocidas

- El modelo identificado (paso 2) es una aproximación lineal FOPDT
  (primer orden + tiempo muerto) alrededor del punto de operación donde
  se hizo la prueba — no es válido en todo el rango del motor sin
  repetir la identificación con más escalones.
- Cuando el tiempo muerto `L` es una fracción grande de `tau` (dead
  time relativo alto), las fórmulas algebraicas de este método
  (realimentación unitaria proporcional) subestiman algo `tau` —
  limitación conocida del método, no un bug. Para mayor precisión en
  ese caso, ajustar una curva al escalón capturado con
  `scipy.optimize.curve_fit` contra un modelo FOPDT explícito (ver
  README sección 9, "Dónde sí ayuda un script de Python").
- Las ganancias de `tune` pueden salir agresivas/con sobre-impulso
  notorio en `simulate` — **confirmado en el motor real (2026-09-01)**:
  con tiempo muerto relativo alto (`L` grande respecto a `tau`, como en
  este motor), Ziegler-Nichols oscila cerca del setpoint sin importar
  qué tan chico se ponga `Ki`. No es un error de la herramienta ni de
  la ganancia elegida — es una limitación del método en sí para este
  tipo de planta. Usar `simc` en su lugar en ese caso (ver sección 3
  del flujo arriba).
- `pid_calcular_salida_us()` en este script es una réplica manual de
  `PID_CalcularSalidaUs()` en `Core/Src/pid.c` — si ese archivo cambia,
  actualizar esta copia también para que la simulación siga siendo
  fiel al firmware real.
