import os
import struct
import threading
from contextlib import suppress

import can
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel


CAN_CHANNEL = os.getenv("CAN_CHANNEL", "/dev/cu.usbmodem206F327555481")
CAN_BITRATE = int(os.getenv("CAN_BITRATE", "500000"))

SET_P = 0x100
SET_K = 0x101
SET_MIN = 0x102
SET_MAX = 0x103

SET_ELECTRICAL_OFFSET = 0x104
SET_ID_REF = 0x105
SET_IQ_REF = 0x106

COMMAND_IDS = {
    "p": SET_P,
    "k": SET_K,
    "min": SET_MIN,
    "max": SET_MAX,
    "electrical_offset" : SET_ELECTRICAL_OFFSET,
    "id_ref": SET_ID_REF,
    "iq_ref": SET_IQ_REF
}

SLIDER_CONFIG = { 
    "p": {"label": "P", "min": 0.0, "max": 10.0, "step": 0.001, "value": 0.2},
    "k": {"label": "K", "min": 0.0, "max": 5000.0, "step": 0.001, "value": 0.0},
    "min": {"label": "Min", "min": -2.0, "max": 0, "step": 0.01, "value": -0.3},
    "max": {"label": "Max", "min": 0, "max": 5.0, "step": 0.01, "value": 0.3},
    "electrical_offset": {"label": "electrical_offset", "min": 0, "max": 10.0, "step": 0.01, "value": 0.3},
    "id_ref": {"label": "id_ref", "min": -2.0, "max": 2.0, "step": 0.0001, "value": 0.0},
    "iq_ref": {"label": "iq_ref", "min": -1.0, "max": 1.0, "step": 0.0001, "value": 0.0}
}


def float_to_bytes(value: float) -> bytes:
    return struct.pack("f", value)


class SliderUpdate(BaseModel):
    value: float


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
      --accent-soft: #f2c3a8;
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
      width: min(920px, 100%);
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
    }}
  </style>
</head>
<body>
  <main>
    <div class="grid">
      {sliders_html}
    </div>

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

    document.querySelectorAll('input[type="range"]').forEach((slider) => {{
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
  </script>
</body>
</html>"""


controller = CanController(CAN_CHANNEL, CAN_BITRATE)
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
    controller.close()
