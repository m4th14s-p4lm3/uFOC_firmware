import math
import os
import struct
import threading
import time
from contextlib import suppress

import can
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel


CAN_CHANNEL = os.getenv("CAN_CHANNEL", "/dev/cu.usbmodem206F327555481")
CAN_BITRATE = int(os.getenv("CAN_BITRATE", "500000"))

SET_P = 0x100
SET_K = 0x101
SET_D = 0x107

SET_MIN = 0x102
SET_MAX = 0x103

SET_ELECTRICAL_OFFSET = 0x104
SET_ID_REF = 0x105
SET_IQ_REF = 0x106

COMMAND_IDS = {
    "p": SET_P,
    "i": SET_K,
    "d": SET_D,
    "min": SET_MIN,
    "max": SET_MAX,
    "electrical_offset": SET_ELECTRICAL_OFFSET,
    "id_ref": SET_ID_REF,
    "iq_ref": SET_IQ_REF,
}

SLIDER_CONFIG = {
    "p": {"label": "P", "min": 0.0, "max": 15.0, "step": 0.0001, "value": 0.2},
    "i": {"label": "I", "min": 0.0, "max": 20.0, "step": 0.001, "value": 0.0},
    "d": {"label": "D", "min": -2.0, "max": 2.0, "step": 0.1, "value": 0.0},
    "min": {"label": "Min", "min": -2.0, "max": 0.0, "step": 0.01, "value": -0.3},
    "max": {"label": "Max", "min": 0.0, "max": 0.8, "step": 0.01, "value": 0.3},
    "electrical_offset": {"label": "Velocity", "min": -1200.0, "max": 1000.0, "step": 1, "value": 0.0},
    "id_ref": {"label": "id_ref", "min": -2.0, "max": 2.0, "step": 0.01, "value": 0.0},
    "iq_ref": {"label": "iq_ref", "min": -3.0, "max": 3.0, "step": 0.01, "value": 0.0},
}


def float_to_bytes(value: float) -> bytes:
    return struct.pack("f", value)


class SliderUpdate(BaseModel):
    value: float


class SineConfig(BaseModel):
    minimum: float
    maximum: float
    frequency: float


class CanController:
    def __init__(self, channel: str, bitrate: int) -> None:
        self.channel = channel
        self.bitrate = bitrate
        self._bus: can.BusABC | None = None
        self._lock = threading.Lock()

    def _ensure_bus(self) -> can.BusABC:
        if self._bus is None:
            self._bus = can.Bus(
                interface="slcan",
                channel=self.channel,
                bitrate=self.bitrate,
                receive_own_messages=True,
            )
        return self._bus

    def send_value(self, name: str, value: float) -> dict[str, object]:
        arbitration_id = COMMAND_IDS.get(name)
        if arbitration_id is None:
            raise ValueError(f"Unknown parameter '{name}'")

        msg = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=False,
            data=float_to_bytes(value),
        )

        with self._lock:
            bus = self._ensure_bus()
            bus.send(msg)

        return {
            "parameter": name,
            "value": value,
            "arbitration_id": arbitration_id,
            "data_hex": msg.data.hex(" "),
        }

    def close(self) -> None:
        with self._lock:
            if self._bus is not None:
                with suppress(Exception):
                    self._bus.shutdown()
                self._bus = None


class SineWaveSender:
    def __init__(self, controller: CanController, parameter_name: str = "iq_ref") -> None:
        self.controller = controller
        self.parameter_name = parameter_name

        self.minimum = 0.0
        self.maximum = 1.0
        self.frequency = 1.0
        self.sample_time = 0.01

        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._running = False
        self._started_at = 0.0
        self._last_value = 0.0

    def configure(self, minimum: float, maximum: float, frequency: float) -> None:
        if maximum < minimum:
            raise ValueError("Maximum must be >= minimum")
        if frequency < 0:
            raise ValueError("Frequency must be >= 0")

        with self._lock:
            self.minimum = minimum
            self.maximum = maximum
            self.frequency = frequency

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._stop_event.clear()
            self._started_at = time.perf_counter()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._running = True
            self._thread.start()

    def stop(self) -> None:
        thread = None
        with self._lock:
            if not self._running:
                return
            self._stop_event.set()
            thread = self._thread

        if thread is not None:
            thread.join(timeout=1.0)

        with self._lock:
            self._running = False
            self._thread = None

    def status(self) -> dict[str, object]:
        with self._lock:
            return {
                "running": self._running,
                "parameter": self.parameter_name,
                "minimum": self.minimum,
                "maximum": self.maximum,
                "frequency": self.frequency,
                "sample_time": self.sample_time,
                "last_value": self._last_value,
            }

    def shutdown(self) -> None:
        self.stop()

    def _run(self) -> None:
        next_tick = time.perf_counter()

        while not self._stop_event.is_set():
            now = time.perf_counter()

            with self._lock:
                minimum = self.minimum
                maximum = self.maximum
                frequency = self.frequency
                started_at = self._started_at

            t = now - started_at

            mid = (maximum + minimum) / 2.0
            amp = (maximum - minimum) / 2.0
            value = mid + amp * math.sin(2.0 * math.pi * frequency * t)

            try:
                self.controller.send_value(self.parameter_name, value)
            except Exception as exc:
                print(f"[SINE] send failed: {exc}")

            with self._lock:
                self._last_value = value

            next_tick += self.sample_time
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()


def build_page() -> str:
    slider_rows = []
    for key, config in SLIDER_CONFIG.items():
        slider_rows.append(
            f"""
            <section class="slider-card">
              <div class="slider-head">
                <label for="{key}">{config["label"]}</label>
                <output id="{key}-value">{config["value"]:.2f}</output>
              </div>
              <input
                id="{key}"
                type="range"
                min="{config["min"]}"
                max="{config["max"]}"
                step="{config["step"]}"
                value="{config["value"]}"
                data-name="{key}"
              />
            </section>
            """
        )

    sliders_html = "\n".join(slider_rows)

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>CAN Slider Control</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f4efe6;
      --panel: rgba(255, 251, 245, 0.92);
      --ink: #1f2933;
      --muted: #5f6c7b;
      --accent: #bc412b;
      --line: rgba(31, 41, 51, 0.12);
      --shadow: 0 24px 60px rgba(60, 38, 20, 0.14);
    }}

    * {{
      box-sizing: border-box;
    }}

    body {{
      margin: 0;
      min-height: 100vh;
      font-family: "Avenir Next", "Segoe UI", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(188, 65, 43, 0.18), transparent 28%),
        radial-gradient(circle at bottom right, rgba(28, 126, 214, 0.12), transparent 30%),
        linear-gradient(135deg, #efe2cc 0%, var(--bg) 46%, #f8f5ef 100%);
      display: grid;
      place-items: center;
      padding: 24px;
    }}

    main {{
      width: min(980px, 100%);
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 28px;
      box-shadow: var(--shadow);
      padding: 28px;
      backdrop-filter: blur(12px);
    }}

    h1 {{
      margin: 0 0 8px;
      font-size: clamp(2rem, 4vw, 3.4rem);
      line-height: 0.95;
      letter-spacing: -0.04em;
    }}

    p {{
      margin: 0;
      color: var(--muted);
      font-size: 1rem;
    }}

    h2 {{
      margin: 0 0 12px;
      font-size: 1.2rem;
    }}

    .grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 18px;
      margin-top: 28px;
    }}

    .slider-card {{
      background: rgba(255, 255, 255, 0.66);
      border: 1px solid rgba(31, 41, 51, 0.08);
      border-radius: 22px;
      padding: 18px;
    }}

    .slider-head {{
      display: flex;
      justify-content: space-between;
      align-items: baseline;
      margin-bottom: 12px;
      gap: 12px;
    }}

    label {{
      font-size: 1.15rem;
      font-weight: 700;
      letter-spacing: 0.02em;
    }}

    output {{
      min-width: 84px;
      text-align: right;
      font-variant-numeric: tabular-nums;
      color: var(--accent);
      font-size: 1.1rem;
      font-weight: 700;
    }}

    input[type="range"] {{
      width: 100%;
      accent-color: var(--accent);
      cursor: pointer;
    }}

    .panel {{
      margin-top: 24px;
      background: rgba(255, 255, 255, 0.66);
      border: 1px solid rgba(31, 41, 51, 0.08);
      border-radius: 22px;
      padding: 18px;
    }}

    .row {{
      display: grid;
      grid-template-columns: 140px 1fr 100px;
      gap: 12px;
      align-items: center;
      margin-bottom: 12px;
    }}

    .buttons {{
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
      margin-top: 12px;
    }}

    button {{
      border: none;
      border-radius: 12px;
      padding: 12px 18px;
      font-size: 1rem;
      cursor: pointer;
      background: var(--accent);
      color: white;
      font-weight: 700;
    }}

    button.secondary {{
      background: #46515c;
    }}

    .status {{
      margin-top: 22px;
      padding: 14px 16px;
      border-radius: 16px;
      background: rgba(31, 41, 51, 0.05);
      color: var(--muted);
      font-size: 0.95rem;
      min-height: 54px;
      white-space: pre-wrap;
    }}

    @media (max-width: 720px) {{
      main {{
        padding: 20px;
        border-radius: 22px;
      }}

      .grid {{
        grid-template-columns: 1fr;
      }}

      .row {{
        grid-template-columns: 1fr;
      }}
    }}
  </style>
</head>
<body>
  <main>
    

    <div class="grid">
      {sliders_html}
    </div>

    <section class="panel">
      <h2>Sinus generator for iq_ref</h2>

      <div class="row">
        <label for="sine-min">Minimum</label>
        <input id="sine-min" type="range" min="-1500.0" max="1500.0" step="0.01" value="0.0" />
        <output id="sine-min-value">0.00</output>
      </div>

      <div class="row">
        <label for="sine-max">Maximum</label>
        <input id="sine-max" type="range" min="-1500.0" max="1500.0" step="0.01" value="1.0" />
        <output id="sine-max-value">1.00</output>
      </div>

      <div class="row">
        <label for="sine-frequency">Frequency [Hz]</label>
        <input id="sine-frequency" type="range" min="0.0" max="20.0" step="0.01" value="1.0" />
        <output id="sine-frequency-value">1.00</output>
      </div>

      <div class="buttons">
        <button id="start-sine">Start sine</button>
        <button id="stop-sine" class="secondary">Stop sine</button>
      </div>
    </section>

    <div id="status" class="status">Ready.</div>
  </main>

  <script>
    const statusNode = document.getElementById("status");
    const debounceTimers = new Map();

    async function sendValue(name, value) {{
      try {{
        const response = await fetch(`/api/set/${{name}}`, {{
          method: "POST",
          headers: {{
            "Content-Type": "application/json"
          }},
          body: JSON.stringify({{ value: Number(value) }})
        }});

        const payload = await response.json();
        if (!response.ok) {{
          throw new Error(payload.detail || "Send failed");
        }}

        statusNode.textContent =
          `Sent ${{payload.parameter}} = ${{payload.value}} | ID=0x${{payload.arbitration_id.toString(16).toUpperCase()}} | DATA=${{payload.data_hex}}`;
      }} catch (error) {{
        statusNode.textContent = `Error: ${{error.message}}`;
      }}
    }}

    document.querySelectorAll('input[type="range"][data-name]').forEach((slider) => {{
      const output = document.getElementById(`${{slider.dataset.name}}-value`);
      const formatValue = () => Number(slider.value).toFixed(2);

      slider.addEventListener("input", () => {{
        output.textContent = formatValue();
        clearTimeout(debounceTimers.get(slider.dataset.name));
        debounceTimers.set(
          slider.dataset.name,
          setTimeout(() => sendValue(slider.dataset.name, slider.value), 80)
        );
      }});
    }});

    const sineMin = document.getElementById("sine-min");
    const sineMax = document.getElementById("sine-max");
    const sineFrequency = document.getElementById("sine-frequency");

    const sineMinValue = document.getElementById("sine-min-value");
    const sineMaxValue = document.getElementById("sine-max-value");
    const sineFrequencyValue = document.getElementById("sine-frequency-value");

    function updateSineLabels() {{
      sineMinValue.textContent = Number(sineMin.value).toFixed(2);
      sineMaxValue.textContent = Number(sineMax.value).toFixed(2);
      sineFrequencyValue.textContent = Number(sineFrequency.value).toFixed(2);
    }}

    sineMin.addEventListener("input", updateSineLabels);
    sineMax.addEventListener("input", updateSineLabels);
    sineFrequency.addEventListener("input", updateSineLabels);
    updateSineLabels();

    async function startSine() {{
      try {{
        const response = await fetch("/api/sine/start", {{
          method: "POST",
          headers: {{
            "Content-Type": "application/json"
          }},
          body: JSON.stringify({{
            minimum: Number(sineMin.value),
            maximum: Number(sineMax.value),
            frequency: Number(sineFrequency.value)
          }})
        }});

        const payload = await response.json();
        if (!response.ok) {{
          throw new Error(payload.detail || "Sine start failed");
        }}

        statusNode.textContent =
          `Sine running: iq_ref from ${{payload.min}} to ${{payload.max}} at ${{payload.frequency}} Hz`;
      }} catch (error) {{
        statusNode.textContent = `Error: ${{error.message}}`;
      }}
    }}

    async function stopSine() {{
      try {{
        const response = await fetch("/api/sine/stop", {{
          method: "POST"
        }});

        const payload = await response.json();
        if (!response.ok) {{
          throw new Error(payload.detail || "Sine stop failed");
        }}

        statusNode.textContent = payload.detail;
      }} catch (error) {{
        statusNode.textContent = `Error: ${{error.message}}`;
      }}
    }}

    document.getElementById("start-sine").addEventListener("click", startSine);
    document.getElementById("stop-sine").addEventListener("click", stopSine);
  </script>
</body>
</html>"""


controller = CanController(CAN_CHANNEL, CAN_BITRATE)
sine_sender = SineWaveSender(controller=controller, parameter_name="electrical_offset")

app = FastAPI(title="CAN Slider Control")


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    return build_page()


@app.get("/api/config")
def get_config() -> dict[str, object]:
    return {
        "channel": CAN_CHANNEL,
        "bitrate": CAN_BITRATE,
        "sliders": SLIDER_CONFIG,
    }


@app.get("/api/sine/status")
def get_sine_status() -> dict[str, object]:
    return sine_sender.status()


@app.post("/api/sine/start")
def start_sine(config: SineConfig) -> dict[str, object]:
    try:
        sine_sender.configure(
            minimum=config.minimum,
            maximum=config.maximum,
            frequency=config.frequency,
        )
        sine_sender.start()
        return {
            "detail": "Sine started",
            "running": True,
            "parameter": "iq_ref",
            "min": config.minimum,
            "max": config.maximum,
            "frequency": config.frequency,
        }
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.post("/api/sine/stop")
def stop_sine() -> dict[str, object]:
    sine_sender.stop()
    return {
        "detail": "Sine stopped",
        "running": False,
    }


@app.post("/api/set/{name}")
def set_parameter(name: str, update: SliderUpdate) -> dict[str, object]:
    try:
        return controller.send_value(name, update.value)
    except ValueError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except can.CanError as exc:
        raise HTTPException(status_code=500, detail=f"CAN send failed: {exc}") from exc
    except OSError as exc:
        raise HTTPException(status_code=500, detail=f"CAN interface open failed: {exc}") from exc


@app.on_event("shutdown")
def shutdown() -> None:
    sine_sender.shutdown()
    controller.close()