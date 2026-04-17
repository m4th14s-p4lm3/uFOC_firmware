# Refactoring návrh + CAN protokol pro uFOC

Autor: analýza stávajícího prototypu (`Core/Src/main.c`, `uFOC_lib/*`). Nic se v kódu nemění – tento dokument slouží jako plán.

---

## 1. Analýza současného stavu

Prototyp funguje – má:

- **Current loop (PI, d & q)** v ISR od `HAL_ADCEx_InjectedConvCpltCallback` (ADC injected trigger z TIM1 CC4). Běží na plné spínací frekvenci (~5 kHz při ARR=1800, centeraligned).
- **Velocity loop (PID, `pid_w`)** v `HAL_TIM_PeriodElapsedCallback` od TIM6 (~1125 Hz, dle `const float dt = 1.0f/1125.0f;`).
- **Position loop (PID, `pid_pos`)** ve stejné TIM6 ISR, vydává `target_w`.
- **Encoder** MT6835 přes SPI3 + CRC-8 kontrola.
- **Driver** DRV8316 + ADC injected pro fázové proudy.
- **CAN** – `communication.c` s jednoduchou mailbox strukturou `can_message_t`, parser v `main()` přes `switch(msg.id)`.

### 1.1 Hlavní slabá místa

1. **Bohové v `main.c`.** Globální proměnné `target_w`, `target_position`, `id_ref`, `iq_ref`, `pi_d`, `pi_q`, `pid_w`, `pid_pos`, `foc_enabled`, `foc_enable_positional_control`, `va/vb/vc`, `ia/ib/ic`. Modifikují je 3 různé ISR + hlavní smyčka. Žádná atomicita, žádná jednotka vlastnictví.
2. **Tři kopie stejného `switch(msg.id)` handleru** (current / velocity / position – dvě zakomentované). Každá přepisuje jiné PID struktury stejnými zprávami (SET_P → jednou `pi_d/pi_q`, podruhé `pid_w`, potřetí `pid_pos`). Není vidět, kterou smyčku zrovna ladíš, hrozí chybné přepsání PID za běhu.
3. **"HACK – REMOVE LATER!"** v TIM6 ISR (`if (target_w < -1000) return;`) – nouzová brzda roztroušená po kódu.
4. **Magická čísla v ISR:** `1.0f/1125.0f`, `0.0002f`, `9.55741f` (rad/s → RPM), `V_BUS_NOMINAL=11.5f`. Mají být na jednom místě a pocházet z konfigurace HW/timerů.
5. **SVPWM + duty clamp** napsané inline v ADC ISR. Patří do `foc.c`/`svpwm.c` jako testovatelná funkce.
6. **`torque_control()`** zapisuje do globálních `va,vb,vc` – skrytý vedlejší efekt; těžko testovatelné.
7. **`calibrate_electrical_offset()`** v `main.c` míchá aplikační logiku s matematikou FOC i s řízením PWM.
8. **`communication_read()` zakazuje IRQ** (`__disable_irq()`) pokaždé – mohlo by stačit jen CAN RX NVIC disable, globální `cli` je v ISR režimu citlivé k FOC.
9. **Protokol CAN je ploché ID mapování.** Neřeší direction (set/get), cílové ID zařízení (vícero motorů na sběrnici), žádný ACK, žádná konzistence (SET_P se vztahuje k jaké smyčce?), žádné stavové/telemetrické zprávy.
10. **Enkodér** mísí hw přístup (SPI, CRC) a kinematiku (pozice, rychlost, el. úhel). `update_encoder()` je tak těžko testovatelný bez HW.
11. **`start foc_enabled` a `start foc_enable_positional_control`** bootují přímo v `main()` bez stavového automatu – neumíš přepínat režimy (torque-only, velocity-only, position).

### 1.2 Co je v pořádku (nebourat)

- `pi_controller.c` – PI/PID s back-calc anti-windup je čistý modul, ponechat.
- `foc.c` (Clarke/Park/invClarke/invPark + sin/cos LUT) – dobrý modul, jen vyhodit zakomentovaný duplikát.
- `driver.h`/`driver.c` (DRV8316 SPI, ADC → proud) – dobrá abstrakce.
- `communication.c` (transport) – jen nad ním potřebuje vyšší vrstva.

---

## 2. Návrh architektury po refaktoru

Cílem je **tři jasně oddělené vrstvy**: HW → FOC core → Aplikační řídicí smyčka → Protocol/CAN. Každá se dá testovat izolovaně (i na PC, mimo STM32), a každá má jasného vlastníka dat.

```
  ┌───────────────────────────────────────────────────────────────┐
  │                        app/ (cooperative state machine)      │
  │     motor_ctrl_t { mode, set_point, status, fault_flags }    │
  └────────┬──────────────────────────┬───────────────────────────┘
           │ cmd (target, mode)       │ telem (pos, vel, iq, ...)
  ┌────────▼──────────┐      ┌────────▼──────────┐
  │   control_loops/   │      │   protocol/       │  <- nad communication.c
  │ current_loop.c     │      │ can_protocol.c    │
  │ velocity_loop.c    │      │ frame_ids.h        │
  │ position_loop.c    │      └────────┬──────────┘
  └────────┬──────────┘               │
           │ vd,vq           ┌────────▼──────────┐
  ┌────────▼──────────┐      │ communication.c  │ (transport, už máš)
  │   foc_core/        │      └────────┬──────────┘
  │ clarke/park/svpwm  │               │
  └────────┬──────────┘      ┌────────▼──────────┐
           │                 │     HAL CAN      │
  ┌────────▼──────────┐      └───────────────────┘
  │   hw/              │
  │ driver.c (DRV8316) │
  │ encoder.c (MT6835) │
  │ adc.c / pwm.c      │
  └────────────────────┘
```

### 2.1 Navrhovaná adresářová struktura

```
uFOC_lib/
├── hw/                        # vše, co sahá na HAL / periferii
│   ├── Inc/ hw_pwm.h          # pwm_init, pwm_set
│   ├── Inc/ hw_adc.h          # injected start, raw readers, offset calib
│   ├── Inc/ hw_drv8316.h      # (z driver.h)
│   ├── Inc/ hw_encoder.h      # jen SPI transakce + raw21 + CRC
│   └── Src/ ...
│
├── foc_core/
│   ├── Inc/ clarke_park.h
│   ├── Inc/ svpwm.h           # svpwm_compute_duties(va,vb,vc,vbus) -> du,dv,dw
│   ├── Inc/ pi_controller.h
│   └── Src/ ...
│
├── control/
│   ├── Inc/ current_loop.h    # FOC torque; state = {pi_d,pi_q,id_ref,iq_ref,vbus}
│   ├── Inc/ velocity_loop.h   # PID; state = {pid_w, target_w, gear/ratio}
│   ├── Inc/ position_loop.h   # PID; state = {pid_pos, target_pos}
│   ├── Inc/ motor_ctrl.h      # state machine + režimy (IDLE/TORQUE/VEL/POS/FAULT)
│   └── Src/ ...
│
├── sensors/
│   ├── Inc/ encoder_kinematics.h  # turns, vel EWMA, vel MA, electrical_angle
│   └── Src/ encoder_kinematics.c
│
├── calibration/
│   └── Src/ calib_current_offsets.c, calib_electrical_offset.c
│
├── protocol/
│   ├── Inc/ can_protocol.h    # frame IDs, packing, dispatcher
│   └── Src/ can_protocol.c
│
└── transport/
    ├── Inc/ communication.h   # (ponechat – transport)
    └── Src/ communication.c
```

### 2.2 Konkrétní refactoring kroky (pořadí)

1. **Vytěžit globály do struct `motor_ctrl_t`.** Jedna statická instance v souboru `motor_ctrl.c`, všechno ostatní přistupuje přes API (`motor_set_mode`, `motor_set_target`, `motor_get_telemetry`). Přístup z ISR jen přes `volatile` pole uvnitř struktury + zásadní: **signálové toky mezi smyčkami jdou přes atomické float/int32 zápisy** (single-writer pravidlo viz §2.3).
2. **Rozdělit `torque_control()`** na čisté funkce bez vedlejších efektů:
   - `foc_compute_voltages(currents, theta_e, id_ref, iq_ref, &pi_d, &pi_q) -> {vd, vq, va, vb, vc}` – bez zápisu do globálu.
   - `svpwm_compute_duties(va, vb, vc, vbus) -> {du, dv, dw}` (čistý výpočet s min/max injekcí).
   - `pwm_set()` zůstane HW vrstva.
3. **Zavést stavový automat `motor_mode_t`:**
   ```
   MOTOR_IDLE       → výstupy 0.5/0.5/0.5, všechny PID reset
   MOTOR_CALIBRATE  → offsety proudu + el. úhlu (nyní v main())
   MOTOR_TORQUE     → pouze current loop (iq_ref z CAN)
   MOTOR_VELOCITY   → velocity → iq_ref, current loop dole
   MOTOR_POSITION   → position → target_w → iq_ref
   MOTOR_FAULT      → latched, brake/coast dle config
   ```
   Přechody ovládá CAN (`MOTOR_CMD_SET_MODE`) a interní fault logika. Jedno místo rozhodne, zda ISR výpočty běží ("HACK – REMOVE LATER" zmizí).
4. **ISR politika:** ADC-injected ISR **jen volá** `current_loop_tick()`, která dostane handle na `motor_ctrl_t`. TIM6 ISR volá `cascade_tick()`, která interně podle mode zavolá position/velocity.
5. **`dt` jako konstanta odvozená z HW konfigurace** v `timing.h` (`DT_CURRENT = 1.0f/5000.0f`, `DT_OUTER = 1.0f/1125.0f`). Žádné magické konstanty v ISR.
6. **Duty limit a modulační index** udělat funkcí `svpwm_compute_duties()` – clamping na [0,1] a varování / fault při přepadu.
7. **Protokol (CAN) vytáhnout z `main()`:** nová funkce `can_protocol_handle(const can_message_t *m, motor_ctrl_t *mc)` nahradí všechny tři zakomentované `switch` varianty. Podrobnosti v §3.
8. **Encoder rozdělit:** `hw_encoder.c` (jen `mt6835_read_raw21`, CRC, SPI setup); `encoder_kinematics.c` (position_ticks, EWMA, moving-avg, electrical_angle) – jednotkově testovatelné na PC.
9. **Kalibrace** do samostatných funkcí v `calibration/` – netancujou v `main()` s globály.
10. **Logging** – sprintf + UART v hlavní smyčce nechat, ale data brát přes `motor_get_telemetry(&snap)` (atomicky, single read).

### 2.3 Pravidlo vlastnictví dat (single-writer)

Abys nemusel řešit zámky v ISR, stačí dodržet:

- `id, iq, ia, ib, ic` – **píše jen** ADC ISR; čtou ostatní.
- `theta_e, position_ticks, angular_velocity_ewma` – **píše jen** ADC ISR (přes `update_encoder()`); čtou ostatní.
- `target_w` – **píše** TIM6 ISR (position→velocity kaskáda) i CAN (přímý override v MOTOR_VELOCITY mode). Toto je jediné skutečně sdílené místo → zaveď `float target_w_from_can` a `float target_w_from_position_loop` zvlášť a v cascade_tick rozhodni podle mode, které se použije. Žádný race.
- `iq_ref` – **píše** TIM6 ISR (velocity→current kaskáda) nebo CAN (v MOTOR_TORQUE mode). Stejný trik.
- `pi_d, pi_q, pid_w, pid_pos, config` – **píše jen** hlavní smyčka (parametry z CAN). ISR jen čte. Zápis ze CAN → krátká sekvence `pid_init()` je bezpečná i když ISR mezitím zavolá `pid_update()`, protože se jen přepisují koeficienty – nejhorší případ je 1 tick s smíšenými koeficienty. Pro preciznost: double-buffer + indikátor `active_bank`.

### 2.4 Testovatelnost

Rozdělením `foc_core/`, `control/`, `sensors/` do čistých funkcí (žádné HAL includy) můžeš:

- stáhnout složky na PC a napsat unit-testy (Unity / Ceedling) pro `clarke/park`, `svpwm`, `pi_update`, `pid_update` (saturace + antiwindup), `encoder_kinematics_update` (delta přes hranici modula).
- vytvořit plant model (discretizovaný PMSM) a regresní testy na ladění PID.

---

## 3. Návrh CAN protokolu

### 3.1 Cíle

- Podporovat **více motorů** na jedné sběrnici (node ID 0..31).
- Rozlišit **SET vs. GET vs. ACK vs. TELEMETRY**.
- Deterministicky adresovat, **které smyčky** se parametr týká (current/velocity/position) — vyřeší současný chaos `SET_P`.
- Periodická **telemetrie** bez RTR.
- Ponechat 11-bit standardní identifikátor (STM32F302 CAN classic bxCAN s Standard ID je levný; extended by fungoval, ale 11 bit úplně stačí na 32 uzlů × dostatečný prostor na typy zpráv).

### 3.2 Layout 11-bit CAN ID

```
 bits:    10 9 8 7 6 | 5 4 3 2 1 0
          ---------- | -----------
          |   MSG_TYPE  |   NODE_ID
          |   (5 bits)  |   (6 bits, 0–63)
```

Node ID 0 = broadcast (master), 1..62 = motory, 63 = rezerva.
Message type dává 32 různých typů – dost.

> Pokud chceš zůstat kompatibilní s tím, co máš teď, je alternativou použít **extended 29-bit ID** a dát tam `priority(3) | msg_type(8) | dest_node(6) | src_node(6) | idx(6)`. Výhodnější pro budoucí rozšiřování, ale STM32 bxCAN to zvládá bez problému a filtr se nastaví stejně snadno.

### 3.3 Přehled zpráv (11-bit, doporučený minimalista)

| MSG_TYPE (hex) | Směr            | Význam                                        | Payload                        |
|----------------|------------------|-----------------------------------------------|--------------------------------|
| `0x01` CMD_SET_MODE         | host → motor    | Přepni režim                                  | `u8 mode` (IDLE/TORQUE/VEL/POS/CAL/FAULT_CLR) |
| `0x02` CMD_SET_TARGET_IQ    | host → motor    | Jen v MOTOR_TORQUE                            | `f32 iq_ref`                  |
| `0x03` CMD_SET_TARGET_ID    | host → motor    | Obvykle 0, pro field weakening               | `f32 id_ref`                  |
| `0x04` CMD_SET_TARGET_VEL   | host → motor    | `target_w` (rad/s nebo RPM — dohodni)        | `f32 target_w`                |
| `0x05` CMD_SET_TARGET_POS   | host → motor    | `target_position` (turns nebo rad)           | `f32 target_pos`              |
| `0x06` CMD_SET_PID          | host → motor    | **Univerzální** – nahradí SET_P/I/D/MIN/MAX  | `u8 loop, u8 field, f32 val`  |
| `0x07` CMD_CALIBRATE        | host → motor    | Spusť kalibraci                              | `u8 calib_type`               |
| `0x08` CMD_SAVE_CONFIG      | host → motor    | Flash                                         | `u8 magic=0xC0`               |
| `0x09` CMD_RESET            | host → motor    | Software reset                                | –                             |
| `0x10` TEL_STATE            | motor → host    | Periodicky (~100 Hz)                         | `u8 mode, u8 faults, u16 vbus_mv, f32 temp` |
| `0x11` TEL_POSITION         | motor → host    | Periodicky (~500 Hz)                         | `f32 pos_turns, f32 vel_rpm` |
| `0x12` TEL_CURRENTS         | motor → host    | Periodicky (~200 Hz)                         | `f32 id, f32 iq`              |
| `0x13` TEL_PHASE_CURRENTS   | motor → host    | Na vyžádání / debug                          | `f32 ia, f32 ib` (ic dopočet) |
| `0x14` TEL_DEBUG            | motor → host    | Libovolné debug float                        | `u16 slot, f32 value`         |
| `0x1A` ACK                  | motor → host    | Potvrzení SET příkazu                        | `u16 echo_msg_type, u8 status`|
| `0x1B` NACK                 | motor → host    | Chyba zpracování                              | `u16 echo_msg_type, u8 err`   |
| `0x1E` GET_PARAM            | host → motor    | Přečti parametr                              | `u8 loop, u8 field`           |
| `0x1F` GET_PARAM_REPLY      | motor → host    | Odpověď na GET                               | `u8 loop, u8 field, f32 val`  |

### 3.4 Univerzální `CMD_SET_PID`

Toto je hlavní úklid proti současnému rozhazování `SET_P/SET_I/SET_MIN/SET_MAX` po třech různých smyčkách:

```c
// Byte 0: který loop
enum pid_loop {
  LOOP_CURRENT_D = 0,
  LOOP_CURRENT_Q = 1,
  LOOP_VELOCITY  = 2,
  LOOP_POSITION  = 3,
};

// Byte 1: které pole
enum pid_field {
  FIELD_KP      = 0,
  FIELD_KI      = 1,
  FIELD_KD      = 2,
  FIELD_OUT_MAX = 3,
  FIELD_OUT_MIN = 4,  // dnes symetrické s OUT_MAX, ponecháno pro budoucnost
  FIELD_ALPHA   = 5,  // filtr (EWMA) pokud bude potřeba
  FIELD_RESET   = 6,  // reset integrátoru
};

// Bytes 2..5: IEEE-754 float, little-endian
```

Host chce nastavit `Kp` velocity smyčky:
```
ID   = 0x06 << 6 | node
data = [ LOOP_VELOCITY, FIELD_KP, <f32 0.00232> ]   // DLC = 6
```
Motor odpoví `ACK` s echem. Jeden handler, jedna logika, nulová dvojznačnost.

### 3.5 Rámec periodické telemetrie

Místo `sprintf` přes UART, co teď máš v hlavní smyčce, navrhnu:

- `TEL_POSITION` každých 2 ms (500 Hz) – pozice a rychlost; to potřebuješ na ladění position loopu v pythonu.
- `TEL_CURRENTS` každých 5 ms (200 Hz) – id/iq dostatečné pro torque loop.
- `TEL_STATE` každých 10 ms (100 Hz) – mode, faults, Vbus, teplota.

Odesílání z **hlavní smyčky** (ne z ISR), protože `HAL_CAN_AddTxMessage` není vhodný v ISR. Timer/soft-scheduler v main loopu se stará o šedulování (counter na `HAL_GetTick()`).

### 3.6 Pack/Unpack pomocné funkce

Navrhuji centralizovat v `can_protocol.c`:

```c
static inline void pack_f32(uint8_t *dst, float v) {
    memcpy(dst, &v, 4);    // little-endian, host musí mít stejný endian
}
static inline float unpack_f32(const uint8_t *src) {
    float v; memcpy(&v, src, 4); return v;
}
```

Na master (Python) straně `struct.pack('<f', val)`. Pozor: všichni v projektu byste se měli na endianu dohodnout – doporučuji **little-endian** (STM32 native).

### 3.7 Filtry a FIFO

Aktuálně `communication_filter_init` přijímá vše. Po zavedení node ID doporučuji:

- Filtr 0 (mask mode): přijímat zprávy, kde nižších 6 bitů = vlastní node ID NEBO 0 (broadcast).
  - Mask: `0b11111000000`, ID: `(own_node)` – klasický trik.
  - To redukuje zátěž RX ISR, když je na sběrnici víc uzlů.

### 3.8 Fault/ACK politika

- Každý `CMD_SET_*` vrací `ACK` (status=0) nebo `NACK` (status=kód chyby) **do 1 ms** od přijetí. Host tak může mít jednoduchý request/response.
- `GET_PARAM` vrací `GET_PARAM_REPLY` s aktuální hodnotou (včetně uložené v flash).
- Fault flags v `TEL_STATE.faults` jako bitfield: `OVERCURRENT=1<<0`, `OVERVOLTAGE=1<<1`, `DRV_FAULT=1<<2`, `ENCODER_CRC=1<<3`, `OVERTEMP=1<<4`, `FOLLOWING_ERROR=1<<5`, `BUS_OFF=1<<6`. Master vidí, proč motor spadl do MOTOR_FAULT.

### 3.9 Bitrate & timing

Tvůj `hcan.Init`: prescaler 4, BS1=15, BS2=2, SJW=1. Předpokládám PCLK1 = 36 MHz → bit_time = 4 × (1+15+2) TQ = 72 TQ_per_4_HCLK → **500 kbit/s** s sample point ~88 %. To je dobrá volba pro krátké motorové stojany (<10 m sběrnice). Ponech.

Odhadovaná propustnost: při DLC=8 + overhead ~111 bit/rámec × 500 kbit/s ≈ **4500 rámců/s**. Tvá cílová telemetrie (500 + 200 + 100 + libovolné debug) zabere ~900 rámců/s → **20 % bus utilization**, zbytek volný na SET příkazy. V pořádku.

---

## 4. Roadmapa implementace (bez dopadu na funkcionalitu)

Doporučené pořadí (každý krok je refactor, ne rewrite – prototyp by měl stále točit):

1. **Vytáhnout `switch(msg.id)` z `main()`** do `can_protocol_handle()`. Zatím stejné ID i chování, jen přesun. Tímto rozbiješ všechny tři zakomentované bloky a sjednotíš na jeden.
2. **Zavést `motor_ctrl_t`** jako obal nad existujícími globály (žádné nové featury). Globály zůstávají, ale `motor_get_iq_ref()` atd. budou jediné přístupové body.
3. **Rozdělit ISR obsahy do modulárních funkcí** (`current_loop_tick`, `cascade_tick`). Nepřesouvat logiku, jen extrahovat.
4. **Rozšířit CAN protokol na nové rámce** (CMD_SET_MODE, CMD_SET_PID, TEL_POSITION). Starý prototypový `SET_P/SET_I/…` dočasně **mapuj** na `LOOP_POSITION` (dnes je tím zabráněno mixování – definuj explicitně). Později deprecate.
5. **Zavést stavový automat** – dnes máš `foc_enabled` a `foc_enable_positional_control`; nahraď `mode` a odstraň "HACK REMOVE LATER".
6. **Telemetrie** jako soft-scheduler v main loopu – nahrazuje `sprintf/print` (UART je ok nechat jako fallback / debug pipe).
7. **Encoder split** (hw ↔ kinematics). Umožní PC-side unit testy.
8. **Flash persistence** pro PID + electrical_offset + pole_pairs (EEPROM emulation nebo poslední sektor flash). Slot 1 = current, slot 2 = velocity, slot 3 = position, + motor config.
9. **(volitelné) Přepnout na 29-bit extended ID**, pokud chceš víc než ~30 typů zpráv nebo více uzlů.

---

## 5. Shrnutí

Co dostaneš:

- **Jeden vlastník dat**, žádný race v ISR – bez FreeRTOS, jen single-writer pravidla.
- **Jeden handler CAN** místo tří zakomentovaných `switch` bloků; neambivalentní, ke kterému PID se parametr váže.
- **Jasný režim provozu** místo dvou booleanů a "HACK".
- **Testovatelné FOC jádro** bez HAL, **testovatelné PID** izolovaně.
- **Rozšířený CAN protokol** s telemetry, ACK/NACK, multi-node, GET – zároveň zpětně kompatibilní s tvým dnešním `main.py` minimálním tooligem (stačí mapovat starý `SET_P` na nové `CMD_SET_PID(LOOP_POSITION, FIELD_KP)`).

Pokud chceš, v dalším kroku ti můžu navrhnout konkrétní API hlavičkové soubory (`motor_ctrl.h`, `can_protocol.h`, `current_loop.h` …) jako skeletony – stále bez změny funkční logiky.
