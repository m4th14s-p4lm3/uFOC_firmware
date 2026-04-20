"""
uFOC Controller – FastAPI UI
Spuštění: uvicorn ui:app --reload
Pak otevři http://127.0.0.1:8000
"""

import time
import threading
from typing import Optional

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from ufoc_client import uFOCclient, AngleUnits, ControlState

# ---------------------------------------------------------------------------
# App & stav
# ---------------------------------------------------------------------------

app = FastAPI(title="uFOC Controller")

_client: Optional[uFOCclient] = None
_lock = threading.Lock()
_last_cmd_time: float = 0.0
MIN_INTERVAL = 0.001  # 1 ms

# ---------------------------------------------------------------------------
# Rate limiter
# ---------------------------------------------------------------------------

def _check_rate() -> None:
    global _last_cmd_time
    with _lock:
        now = time.monotonic()
        elapsed = now - _last_cmd_time
        if elapsed < MIN_INTERVAL:
            raise HTTPException(
                status_code=429,
                detail=f"Příliš rychlé volání – počkej ještě {(MIN_INTERVAL - elapsed) * 1000:.2f} ms."
            )
        _last_cmd_time = now


def _get_client() -> uFOCclient:
    if _client is None:
        raise HTTPException(status_code=503, detail="Nejprve se připoj k zařízení.")
    return _client


# ---------------------------------------------------------------------------
# Pydantic modely
# ---------------------------------------------------------------------------

class ConnectRequest(BaseModel):
    channel: str = "/dev/tty.usbmodem206F327555481"
    arbitration_id: int = 0x123
    bitrate: int = 500000
    interface: str = "slcan"
    device_id: int = 0x00


class FloatValue(BaseModel):
    value: float


class IntValue(BaseModel):
    value: int


# ---------------------------------------------------------------------------
# Endpointy – Connection
# ---------------------------------------------------------------------------

@app.post("/connect")
def connect(req: ConnectRequest):
    global _client
    if _client is not None:
        try:
            _client.close()
        except Exception:
            pass
    _client = uFOCclient(
        channel=req.channel,
        arbitration_id=req.arbitration_id,
        bitrate=req.bitrate,
        interface=req.interface,
        device_id=req.device_id,
    )
    return {"status": "connected"}


@app.post("/disconnect")
def disconnect():
    global _client
    if _client is not None:
        _client.close()
        _client = None
    return {"status": "disconnected"}


@app.get("/status")
def status():
    return {"connected": _client is not None}


# ---------------------------------------------------------------------------
# Endpointy – General
# ---------------------------------------------------------------------------

@app.post("/general/control_state")
def set_control_state(body: IntValue):
    _check_rate()
    _get_client().set_control_state(body.value)
    return {"ok": True}


@app.post("/general/angle_units")
def set_angle_units(body: IntValue):
    _check_rate()
    _get_client().set_user_angle_units(body.value)
    return {"ok": True}


@app.get("/general/angle_units")
def get_angle_units():
    _check_rate()
    val = _get_client().get_user_angle_units()
    if val is None:
        return {"value": None, "name": "timeout"}
    return {"value": int(val), "name": val.name}


# ---------------------------------------------------------------------------
# Endpointy – Current control
# ---------------------------------------------------------------------------

@app.post("/current/torque_target")
def set_torque_target(body: FloatValue):
    _check_rate()
    _get_client().set_torque_current_target(body.value)
    return {"ok": True}


@app.post("/current/soft_limit")
def set_torque_soft_limit(body: FloatValue):
    _check_rate()
    _get_client().set_torque_current_soft_limit(body.value)
    return {"ok": True}


@app.get("/current/soft_limit")
def get_torque_soft_limit():
    _check_rate()
    val = _get_client().get_torque_current_soft_limit()
    return {"value": val}


@app.post("/current/kp")
def set_current_kp(body: FloatValue):
    _check_rate()
    _get_client().set_current_kp(body.value)
    return {"ok": True}


@app.post("/current/ki")
def set_current_ki(body: FloatValue):
    _check_rate()
    _get_client().set_current_ki(body.value)
    return {"ok": True}


# ---------------------------------------------------------------------------
# Endpointy – Velocity control
# ---------------------------------------------------------------------------

@app.post("/velocity/target")
def set_velocity_target(body: FloatValue):
    _check_rate()
    _get_client().set_angular_velocity_target(body.value)
    return {"ok": True}


@app.post("/velocity/soft_limit")
def set_velocity_soft_limit(body: FloatValue):
    _check_rate()
    _get_client().set_angular_velocity_soft_limit(body.value)
    return {"ok": True}


@app.get("/velocity/soft_limit")
def get_velocity_soft_limit():
    _check_rate()
    val = _get_client().get_angular_velocity_soft_limit()
    return {"value": val}


@app.get("/velocity/current")
def get_velocity():
    _check_rate()
    val = _get_client().get_angular_velocity()
    return {"value": val}


@app.post("/velocity/kp")
def set_velocity_kp(body: FloatValue):
    _check_rate()
    _get_client().set_angular_velocity_kp(body.value)
    return {"ok": True}


@app.post("/velocity/ki")
def set_velocity_ki(body: FloatValue):
    _check_rate()
    _get_client().set_angular_velocity_ki(body.value)
    return {"ok": True}


# ---------------------------------------------------------------------------
# Endpointy – Position control
# ---------------------------------------------------------------------------

@app.post("/position/target")
def set_position_target(body: FloatValue):
    _check_rate()
    _get_client().set_position_target(body.value)
    return {"ok": True}


@app.get("/position/target")
def get_position_target():
    _check_rate()
    val = _get_client().get_position_target()
    return {"value": val}


@app.get("/position/current")
def get_position():
    _check_rate()
    val = _get_client().get_position()
    return {"value": val}


@app.get("/position/relative")
def get_relative_position():
    _check_rate()
    val = _get_client().get_relative_position()
    return {"value": val}


@app.post("/position/kp")
def set_position_kp(body: FloatValue):
    _check_rate()
    _get_client().set_position_kp(body.value)
    return {"ok": True}


@app.post("/position/ki")
def set_position_ki(body: FloatValue):
    _check_rate()
    _get_client().set_position_ki(body.value)
    return {"ok": True}


@app.post("/position/kd")
def set_position_kd(body: FloatValue):
    _check_rate()
    _get_client().set_position_kd(body.value)
    return {"ok": True}


# ---------------------------------------------------------------------------
# Endpointy – Encoder
# ---------------------------------------------------------------------------

@app.get("/encoder/offset")
def get_electrical_offset():
    _check_rate()
    val = _get_client().get_electrical_offset()
    return {"value": val}


@app.post("/encoder/offset")
def set_electrical_offset(body: FloatValue):
    _check_rate()
    _get_client().set_electrical_offset(body.value)
    return {"ok": True}


@app.post("/encoder/calibrate")
def calibrate_offset():
    _check_rate()
    _get_client().calibrate_electrical_offset()
    return {"ok": True}


# ---------------------------------------------------------------------------
# HTML UI
# ---------------------------------------------------------------------------

HTML = """<!DOCTYPE html>
<html lang="cs">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>uFOC Controller</title>
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  :root {
    --bg: #0f1117;
    --surface: #1a1d27;
    --surface2: #22263a;
    --accent: #4f8ef7;
    --ok: #3ecf8e;
    --warn: #f7c94f;
    --err: #f76f6f;
    --text: #e0e4f0;
    --muted: #6b7499;
    --radius: 10px;
    --gap: 16px;
  }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Segoe UI', system-ui, sans-serif;
    font-size: 14px;
    line-height: 1.5;
    padding: 24px;
  }

  h1  { font-size: 22px; font-weight: 700; margin-bottom: 4px; }
  h2  { font-size: 13px; font-weight: 600; text-transform: uppercase;
        letter-spacing: .08em; color: var(--muted); margin-bottom: 12px; }
  h3  { font-size: 12px; font-weight: 600; text-transform: uppercase;
        letter-spacing: .06em; color: var(--muted); margin: 14px 0 8px; }

  #header { display: flex; align-items: center; gap: 12px; margin-bottom: 24px; }
  #conn-badge {
    padding: 3px 10px; border-radius: 20px; font-size: 12px; font-weight: 600;
    background: var(--err); color: #fff; transition: background .3s;
  }
  #conn-badge.ok { background: var(--ok); }

  .grid { display: grid; gap: var(--gap);
          grid-template-columns: repeat(auto-fill, minmax(340px, 1fr)); }

  .card {
    background: var(--surface);
    border: 1px solid #2a2f45;
    border-radius: var(--radius);
    padding: 18px 20px;
  }

  label { display: block; font-size: 12px; color: var(--muted); margin-bottom: 4px; }

  input[type=text], input[type=number], select {
    width: 100%; background: var(--surface2); border: 1px solid #323757;
    border-radius: 6px; color: var(--text); padding: 7px 10px; font-size: 14px;
    outline: none; transition: border-color .2s;
  }
  input:focus, select:focus { border-color: var(--accent); }

  .row { display: flex; gap: 8px; align-items: flex-end; margin-bottom: 10px; }
  .row > * { flex: 1; }
  .row button { flex: 0 0 auto; }

  button {
    background: var(--accent); color: #fff; border: none; border-radius: 6px;
    padding: 8px 14px; font-size: 13px; font-weight: 600; cursor: pointer;
    transition: opacity .15s, transform .1s; white-space: nowrap;
  }
  button:hover  { opacity: .85; }
  button:active { transform: scale(.97); }
  button.danger  { background: var(--err); }
  button.neutral { background: var(--surface2); border: 1px solid #323757; }
  button.warn    { background: var(--warn); color: #111; }

  .read-row { display: flex; gap: 8px; align-items: center; margin-bottom: 10px; }
  .read-val {
    flex: 1; background: var(--surface2); border: 1px solid #323757;
    border-radius: 6px; padding: 7px 10px; font-family: monospace; font-size: 13px;
    color: var(--ok); min-height: 34px;
  }

  .toast-area {
    position: fixed; bottom: 20px; right: 20px;
    display: flex; flex-direction: column; gap: 8px; z-index: 999;
  }
  .toast {
    background: var(--surface2); border-left: 4px solid var(--ok);
    border-radius: 6px; padding: 10px 16px; font-size: 13px;
    box-shadow: 0 4px 20px #0008; max-width: 320px;
    animation: slideIn .2s ease;
  }
  .toast.err  { border-color: var(--err); }
  .toast.warn { border-color: var(--warn); }
  @keyframes slideIn {
    from { opacity: 0; transform: translateX(30px); }
    to   { opacity: 1; transform: none; }
  }

  .divider { border: none; border-top: 1px solid #2a2f45; margin: 14px 0; }
  .full { grid-column: 1 / -1; }

  .unit { font-size: 11px; color: var(--muted); margin-left: 4px; }
</style>
</head>
<body>

<div id="header">
  <h1>⚡ uFOC Controller</h1>
  <span id="conn-badge">Odpojeno</span>
</div>

<div class="grid">

  <!-- ===== CONNECTION ===== -->
  <div class="card full">
    <h2>🔌 Připojení</h2>
    <div class="grid" style="grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:10px;margin-bottom:14px;">
      <div>
        <label>Kanál (channel)</label>
        <input id="c_channel" type="text" value="/dev/tty.usbmodem206F327555481">
      </div>
      <div>
        <label>Arbitration ID (hex)</label>
        <input id="c_arb_id" type="text" value="0x123">
      </div>
      <div>
        <label>Bitrate</label>
        <input id="c_bitrate" type="number" value="500000" step="1">
      </div>
      <div>
        <label>Interface</label>
        <select id="c_interface">
          <option value="slcan">slcan</option>
          <option value="socketcan">socketcan</option>
          <option value="pcan">pcan</option>
          <option value="kvaser">kvaser</option>
        </select>
      </div>
      <div>
        <label>Device ID (hex)</label>
        <input id="c_device_id" type="text" value="0x00">
      </div>
    </div>
    <div style="display:flex;gap:10px;">
      <button onclick="doConnect()">Připojit</button>
      <button class="danger" onclick="doDisconnect()">Odpojit</button>
    </div>
  </div>

  <!-- ===== GENERAL ===== -->
  <div class="card">
    <h2>⚙️ Obecné</h2>

    <h3>Control State</h3>
    <div class="row">
      <div>
        <label>Stav regulátoru</label>
        <select id="g_state">
          <option value="0">0 – NO_CONTROL</option>
          <option value="1">1 – CURRENT_CONTROL</option>
          <option value="2">2 – VELOCITY_CONTROL</option>
          <option value="3">3 – POSITION_CONTROL</option>
        </select>
      </div>
      <button onclick="setControlState()">Set</button>
    </div>

    <hr class="divider">

    <h3>User Angle Units</h3>
    <div class="row">
      <div>
        <label>Jednotky</label>
        <select id="g_units">
          <option value="0">0 – RADIANS</option>
          <option value="1" selected>1 – ROTATIONS</option>
          <option value="2">2 – DEGREES</option>
        </select>
      </div>
      <button onclick="setAngleUnits()">Set</button>
    </div>
    <div class="read-row">
      <button class="neutral" onclick="getAngleUnits()">Get</button>
      <div class="read-val" id="g_units_r">—</div>
    </div>
  </div>

  <!-- ===== CURRENT CONTROL ===== -->
  <div class="card">
    <h2>⚡ Proudová regulace</h2>

    <div class="row">
      <div>
        <label>Torque current target <span class="unit">A</span></label>
        <input id="cc_torque" type="number" value="0" step="0.1">
      </div>
      <button onclick="post('/current/torque_target', {value: numVal('cc_torque')})">Set</button>
    </div>

    <hr class="divider">
    <h3>Soft limit</h3>
    <div class="row">
      <div>
        <label>Soft limit <span class="unit">A</span></label>
        <input id="cc_soft" type="number" value="0" step="0.1">
      </div>
      <button onclick="post('/current/soft_limit', {value: numVal('cc_soft')})">Set</button>
    </div>
    <div class="read-row">
      <button class="neutral" onclick="getVal('/current/soft_limit', 'cc_soft_r')">Get</button>
      <div class="read-val" id="cc_soft_r">—</div>
    </div>

    <hr class="divider">
    <h3>PID — d + q osa</h3>
    <div class="row">
      <div><label>Kp</label><input id="cc_kp" type="number" value="0" step="0.001"></div>
      <button onclick="post('/current/kp', {value: numVal('cc_kp')})">Set Kp</button>
    </div>
    <div class="row">
      <div><label>Ki</label><input id="cc_ki" type="number" value="0" step="0.001"></div>
      <button onclick="post('/current/ki', {value: numVal('cc_ki')})">Set Ki</button>
    </div>
  </div>

  <!-- ===== VELOCITY CONTROL ===== -->
  <div class="card">
    <h2>🌀 Rychlostní regulace</h2>

    <div class="row">
      <div>
        <label>Angular velocity target <span class="unit">rot/s</span></label>
        <input id="vc_target" type="number" value="0" step="0.5">
      </div>
      <button onclick="post('/velocity/target', {value: numVal('vc_target')})">Set</button>
    </div>

    <hr class="divider">
    <h3>Aktuální rychlost</h3>
    <div class="read-row">
      <button class="neutral" onclick="getVal('/velocity/current', 'vc_curr_r')">Get</button>
      <div class="read-val" id="vc_curr_r">—</div>
    </div>

    <hr class="divider">
    <h3>Soft limit</h3>
    <div class="row">
      <div>
        <label>Soft limit <span class="unit">rot/s</span></label>
        <input id="vc_soft" type="number" value="0" step="0.5">
      </div>
      <button onclick="post('/velocity/soft_limit', {value: numVal('vc_soft')})">Set</button>
    </div>
    <div class="read-row">
      <button class="neutral" onclick="getVal('/velocity/soft_limit', 'vc_soft_r')">Get</button>
      <div class="read-val" id="vc_soft_r">—</div>
    </div>

    <hr class="divider">
    <h3>PID</h3>
    <div class="row">
      <div><label>Kp</label><input id="vc_kp" type="number" value="0" step="0.001"></div>
      <button onclick="post('/velocity/kp', {value: numVal('vc_kp')})">Set Kp</button>
    </div>
    <div class="row">
      <div><label>Ki</label><input id="vc_ki" type="number" value="0" step="0.001"></div>
      <button onclick="post('/velocity/ki', {value: numVal('vc_ki')})">Set Ki</button>
    </div>
  </div>

  <!-- ===== POSITION CONTROL ===== -->
  <div class="card">
    <h2>📍 Polohová regulace</h2>

    <div class="row">
      <div>
        <label>Position target <span class="unit">rot</span></label>
        <input id="pc_target" type="number" value="0" step="0.01">
      </div>
      <button onclick="post('/position/target', {value: numVal('pc_target')})">Set</button>
    </div>
    <div class="read-row">
      <button class="neutral" onclick="getVal('/position/target', 'pc_target_r')">Get target</button>
      <div class="read-val" id="pc_target_r">—</div>
    </div>

    <hr class="divider">
    <h3>Aktuální poloha</h3>
    <div class="read-row">
      <button class="neutral" onclick="getVal('/position/current', 'pc_curr_r')">Get absolute</button>
      <div class="read-val" id="pc_curr_r">—</div>
    </div>
    <div class="read-row">
      <button class="neutral" onclick="getVal('/position/relative', 'pc_rel_r')">Get relative</button>
      <div class="read-val" id="pc_rel_r">—</div>
    </div>

    <hr class="divider">
    <h3>PID</h3>
    <div class="row">
      <div><label>Kp</label><input id="pc_kp" type="number" value="0" step="0.001"></div>
      <button onclick="post('/position/kp', {value: numVal('pc_kp')})">Set Kp</button>
    </div>
    <div class="row">
      <div><label>Ki</label><input id="pc_ki" type="number" value="0" step="0.001"></div>
      <button onclick="post('/position/ki', {value: numVal('pc_ki')})">Set Ki</button>
    </div>
    <div class="row">
      <div><label>Kd</label><input id="pc_kd" type="number" value="0" step="0.001"></div>
      <button onclick="post('/position/kd', {value: numVal('pc_kd')})">Set Kd</button>
    </div>
  </div>

  <!-- ===== ENCODER ===== -->
  <div class="card">
    <h2>🔁 Enkodér</h2>

    <h3>Elektrický offset <span class="unit">(float)</span></h3>
    <div class="read-row">
      <button class="neutral" onclick="getVal('/encoder/offset', 'enc_off_r')">Get</button>
      <div class="read-val" id="enc_off_r">—</div>
    </div>
    <div class="row">
      <div><label>Offset</label><input id="enc_off" type="number" value="118107" step="1"></div>
      <button onclick="post('/encoder/offset', {value: numVal('enc_off')})">Set</button>
    </div>

    <hr class="divider">
    <h3>Kalibrace</h3>
    <button class="warn" onclick="calibrate()">🔧 Kalibrovat offset</button>
  </div>

</div>

<div class="toast-area" id="toasts"></div>

<script>
/* ---- rate limiter – 1 ms ---- */
let _lastSent = 0;
function guardRate() {
  const now = performance.now();
  if (now - _lastSent < 1) {
    toast('Příliš rychlé volání – počkej 1 ms.', 'warn');
    return false;
  }
  _lastSent = now;
  return true;
}

/* ---- helpers ---- */
function numVal(id) { return parseFloat(document.getElementById(id).value); }

function toast(msg, type = 'ok') {
  const area = document.getElementById('toasts');
  const el = document.createElement('div');
  el.className = 'toast' + (type !== 'ok' ? ' ' + type : '');
  el.textContent = msg;
  area.appendChild(el);
  setTimeout(() => el.remove(), 3500);
}

async function api(method, url, body) {
  const opts = { method, headers: { 'Content-Type': 'application/json' } };
  if (body !== undefined) opts.body = JSON.stringify(body);
  const r = await fetch(url, opts);
  const json = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(json.detail || r.statusText);
  return json;
}

async function post(url, body) {
  if (!guardRate()) return;
  try {
    await api('POST', url, body);
    toast('✓ ' + url);
  } catch(e) { toast('✗ ' + e.message, 'err'); }
}

async function getVal(url, targetId) {
  if (!guardRate()) return;
  try {
    const d = await api('GET', url);
    const el = document.getElementById(targetId);
    el.textContent = (d.value !== null && d.value !== undefined)
      ? d.value.toFixed(6)
      : 'null (timeout)';
  } catch(e) { toast('✗ ' + e.message, 'err'); }
}

/* ---- connection ---- */
function parseHex(id) {
  const v = document.getElementById(id).value.trim();
  return (v.startsWith('0x') || v.startsWith('0X')) ? parseInt(v, 16) : parseInt(v, 10);
}

async function doConnect() {
  try {
    await api('POST', '/connect', {
      channel:        document.getElementById('c_channel').value,
      arbitration_id: parseHex('c_arb_id'),
      bitrate:        parseInt(document.getElementById('c_bitrate').value),
      interface:      document.getElementById('c_interface').value,
      device_id:      parseHex('c_device_id'),
    });
    setBadge(true);
    toast('Připojeno ✓');
  } catch(e) { toast('Chyba připojení: ' + e.message, 'err'); }
}

async function doDisconnect() {
  try {
    await api('POST', '/disconnect');
    setBadge(false);
    toast('Odpojeno.');
  } catch(e) { toast('Chyba: ' + e.message, 'err'); }
}

function setBadge(ok) {
  const b = document.getElementById('conn-badge');
  b.textContent = ok ? 'Připojeno' : 'Odpojeno';
  b.className   = ok ? 'ok' : '';
}

/* ---- general ---- */
async function setControlState() {
  if (!guardRate()) return;
  const v = parseInt(document.getElementById('g_state').value);
  const labels = ['NO_CONTROL', 'CURRENT_CONTROL', 'VELOCITY_CONTROL', 'POSITION_CONTROL'];
  try {
    await api('POST', '/general/control_state', { value: v });
    toast('Control state → ' + labels[v]);
  } catch(e) { toast('✗ ' + e.message, 'err'); }
}

async function setAngleUnits() {
  if (!guardRate()) return;
  const v = parseInt(document.getElementById('g_units').value);
  try {
    await api('POST', '/general/angle_units', { value: v });
    toast('Angle units → ' + ['RADIANS', 'ROTATIONS', 'DEGREES'][v]);
  } catch(e) { toast('✗ ' + e.message, 'err'); }
}

async function getAngleUnits() {
  if (!guardRate()) return;
  try {
    const d = await api('GET', '/general/angle_units');
    document.getElementById('g_units_r').textContent =
      d.value !== null ? d.value + ' – ' + d.name : 'null (timeout)';
  } catch(e) { toast('✗ ' + e.message, 'err'); }
}

/* ---- encoder ---- */
async function calibrate() {
  if (!guardRate()) return;
  if (!confirm('Opravdu spustit kalibraci elektrického offsetu?\\nMotor přejde do NO_CONTROL.')) return;
  try {
    await api('POST', '/encoder/calibrate');
    toast('Kalibrace spuštěna ✓');
  } catch(e) { toast('✗ ' + e.message, 'err'); }
}

/* ---- poll connection status ---- */
setInterval(async () => {
  try {
    const d = await fetch('/status').then(r => r.json());
    setBadge(d.connected);
  } catch {}
}, 3000);
</script>
</body>
</html>
"""


@app.get("/", response_class=HTMLResponse)
def index():
    return HTML