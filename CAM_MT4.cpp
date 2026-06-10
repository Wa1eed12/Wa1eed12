#include "CAM_MT4.h"
#include <FS.h>
#include <SPIFFS.h>
#include <Preferences.h>

// ============================================================
// HTML PAGES — STORED IN PROGMEM
// ============================================================

// ---- PAGE 1: Live Monitor ----
static const char DASH_PAGE_MONITOR[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Waiter Robot — Live Monitor</title>
<style>
  body{margin:0;font-family:'Segoe UI',sans-serif;background:#111;color:#eee;}
  nav{display:flex;gap:4px;padding:8px 12px;background:#1a1a2e;}
  nav a{padding:8px 18px;border-radius:6px;text-decoration:none;color:#aaa;font-weight:600;font-size:14px;}
  nav a.active,nav a:hover{background:#0f3460;color:#e94560;}
  h2{margin:18px 20px 8px;color:#e94560;font-size:18px;}
  .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:14px;padding:0 20px 20px;}
  .card{background:#1c1c2e;border-radius:10px;padding:16px;box-shadow:0 2px 8px #0006;}
  .card label{font-size:11px;color:#888;text-transform:uppercase;letter-spacing:1px;}
  .card .val{font-size:28px;font-weight:700;margin-top:4px;color:#fff;}
  .card .val.green{color:#2ecc71;} .card .val.yellow{color:#f1c40f;} .card .val.red{color:#e74c3c;}
  .bar-bg{height:10px;border-radius:5px;background:#333;margin-top:8px;overflow:hidden;}
  .bar-fill{height:100%;border-radius:5px;background:linear-gradient(90deg,#e94560,#f1c40f);transition:width .5s;}
  .stream-wrap{padding:0 20px 20px;}
  .stream-wrap img{border-radius:10px;border:2px solid #0f3460;max-width:100%;display:block;}
  .status-badge{display:inline-block;padding:4px 14px;border-radius:20px;font-size:13px;font-weight:700;background:#0f3460;color:#e94560;margin-top:6px;}
  .footer{padding:8px 20px;font-size:11px;color:#555;}
</style>
</head>
<body>
<nav>
  <a href="/" class="active">&#128250; Live Monitor</a>
  <a href="/config">&#9881; Calibration</a>
  <a href="/logs">&#128190; SD Logs</a>
</nav>
<h2>&#128250; Live Robot Monitor</h2>
<div class="stream-wrap">
  <img id="camStream" src="" alt="Camera Stream" onerror="this.alt='Stream unavailable'">
</div>
<div class="grid">
  <div class="card">
    <label>Robot State</label>
    <div class="val" id="stateVal">—</div>
    <div class="status-badge" id="stateBadge">UNKNOWN</div>
  </div>
  <div class="card">
    <label>Battery Voltage</label>
    <div class="val" id="battVal">—</div>
    <div class="bar-bg"><div class="bar-fill" id="battBar" style="width:0%"></div></div>
  </div>
  <div class="card">
    <label>SD Frames Logged</label>
    <div class="val green" id="framesVal">—</div>
  </div>
  <div class="card">
    <label>SD Free Space</label>
    <div class="val" id="sdFreeVal">—</div>
  </div>
  <div class="card">
    <label>Uptime</label>
    <div class="val" id="uptimeVal">—</div>
  </div>
  <div class="card">
    <label>Corrupt Frames</label>
    <div class="val" id="corruptVal" style="color:#e74c3c">—</div>
  </div>
</div>
<div class="footer">Waiter Robot ESP32-CAM Dashboard &mdash; Auto-refresh 1s</div>
<script>
var ip = location.hostname;
document.getElementById('camStream').src = 'http://' + ip + ':81/stream';
function fmt(s){var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;return (h?h+'h ':'')+m+'m '+sec+'s';}
function poll(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    document.getElementById('stateVal').textContent = d.state||'—';
    document.getElementById('stateBadge').textContent = d.state||'—';
    var bv = parseFloat(d.battery)||0;
    document.getElementById('battVal').textContent = bv.toFixed(2)+'V';
    var pct = Math.min(100,Math.max(0,((bv-6.0)/3.6)*100));
    document.getElementById('battBar').style.width = pct+'%';
    document.getElementById('framesVal').textContent = (d.frames||0).toLocaleString();
    document.getElementById('sdFreeVal').textContent = (d.sd_free_mb||0).toFixed(1)+' MB';
    document.getElementById('uptimeVal').textContent = fmt(d.uptime||0);
    document.getElementById('corruptVal').textContent = (d.corrupt||0).toLocaleString();
  }).catch(e=>{});
}
poll(); setInterval(poll,1000);
</script>
</body>
</html>
)rawhtml";

// ---- PAGE 2: Calibration Panel ----
static const char DASH_PAGE_CONFIG[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Waiter Robot — Calibration</title>
<style>
  body{margin:0;font-family:'Segoe UI',sans-serif;background:#111;color:#eee;}
  nav{display:flex;gap:4px;padding:8px 12px;background:#1a1a2e;}
  nav a{padding:8px 18px;border-radius:6px;text-decoration:none;color:#aaa;font-weight:600;font-size:14px;}
  nav a.active,nav a:hover{background:#0f3460;color:#e94560;}
  h2{margin:18px 20px 8px;color:#e94560;font-size:18px;}
  .form-wrap{max-width:600px;margin:0 auto;padding:0 20px 40px;}
  .section{background:#1c1c2e;border-radius:10px;padding:20px;margin-bottom:16px;box-shadow:0 2px 8px #0006;}
  .section h3{margin:0 0 16px;font-size:14px;color:#e94560;text-transform:uppercase;letter-spacing:1px;}
  label{display:block;font-size:12px;color:#888;margin-bottom:4px;margin-top:12px;}
  input[type=number],input[type=range],select{width:100%;padding:8px 10px;border-radius:6px;border:1px solid #333;background:#12122a;color:#fff;font-size:15px;box-sizing:border-box;}
  input[type=range]{padding:4px 0;accent-color:#e94560;}
  .radio-group{display:flex;gap:16px;margin-top:6px;}
  .radio-group label{display:flex;align-items:center;gap:6px;color:#ccc;cursor:pointer;font-size:14px;margin:0;}
  .speed-display{font-size:22px;font-weight:700;color:#e94560;text-align:center;margin-top:4px;}
  button{width:100%;padding:14px;margin-top:20px;border:none;border-radius:8px;background:#e94560;color:#fff;font-size:16px;font-weight:700;cursor:pointer;letter-spacing:1px;}
  button:hover{background:#c0392b;}
  .msg{text-align:center;padding:10px;border-radius:6px;margin-top:10px;display:none;font-weight:600;}
  .msg.ok{background:#1a4a2e;color:#2ecc71;}
  .msg.err{background:#4a1a1a;color:#e74c3c;}
</style>
</head>
<body>
<nav>
  <a href="/">&#128250; Live Monitor</a>
  <a href="/config" class="active">&#9881; Calibration</a>
  <a href="/logs">&#128190; SD Logs</a>
</nav>
<h2>&#9881; Calibration Panel</h2>
<div class="form-wrap">
  <div class="section">
    <h3>PID Gains</h3>
    <label>KP (Proportional) — Range: 0.00 – 10.00</label>
    <input type="number" id="pid_kp" min="0" max="10" step="0.01" value="1.5">
    <label>KI (Integral) — Range: 0.00 – 10.00</label>
    <input type="number" id="pid_ki" min="0" max="10" step="0.001" value="0.05">
    <label>KD (Derivative) — Range: 0.00 – 10.00</label>
    <input type="number" id="pid_kd" min="0" max="10" step="0.01" value="0.8">
  </div>
  <div class="section">
    <h3>Motion Settings</h3>
    <label>Base Speed (0–255)</label>
    <input type="range" id="base_speed" min="0" max="255" value="160" oninput="document.getElementById('speedNum').textContent=this.value">
    <div class="speed-display" id="speedNum">160</div>
  </div>
  <div class="section">
    <h3>Line Sensor</h3>
    <label>Line Color Target</label>
    <select id="line_color">
      <option value="0">BLACK line on white surface</option>
      <option value="1">WHITE line on black surface</option>
    </select>
    <label>Navigation Mode</label>
    <div class="radio-group">
      <label><input type="radio" name="nav_mode" value="0" checked> Simple</label>
      <label><input type="radio" name="nav_mode" value="1"> Grid</label>
    </div>
  </div>
  <div class="section">
    <h3>Battery Thresholds</h3>
    <label>Battery LOW threshold (V)</label>
    <input type="number" id="batt_low" min="5.0" max="9.0" step="0.1" value="7.2">
    <label>Battery VERY LOW threshold (V)</label>
    <input type="number" id="batt_vlow" min="5.0" max="9.0" step="0.1" value="6.6">
  </div>
  <button onclick="saveConfig()">&#128190; SAVE &amp; APPLY TO ROBOT</button>
  <div class="msg ok" id="msgOk">&#10003; Configuration saved and sent to robot!</div>
  <div class="msg err" id="msgErr">&#10007; Error saving configuration.</div>
</div>
<script>
function loadConfig(){
  fetch('/api/config').then(r=>r.json()).then(d=>{
    document.getElementById('pid_kp').value = d.pid_kp||1.5;
    document.getElementById('pid_ki').value = d.pid_ki||0.05;
    document.getElementById('pid_kd').value = d.pid_kd||0.8;
    document.getElementById('base_speed').value = d.base_speed||160;
    document.getElementById('speedNum').textContent = d.base_speed||160;
    document.getElementById('line_color').value = d.line_color_target||0;
    document.getElementById('batt_low').value = d.battery_low_v||7.2;
    document.getElementById('batt_vlow').value = d.battery_verylow_v||6.6;
    var nm = d.nav_mode||0;
    document.querySelectorAll('input[name=nav_mode]').forEach(r=>{r.checked=(parseInt(r.value)===nm);});
  }).catch(e=>{});
}
function saveConfig(){
  var nav = 0;
  document.querySelectorAll('input[name=nav_mode]').forEach(r=>{if(r.checked)nav=parseInt(r.value);});
  var cfg={
    pid_kp: parseFloat(document.getElementById('pid_kp').value),
    pid_ki: parseFloat(document.getElementById('pid_ki').value),
    pid_kd: parseFloat(document.getElementById('pid_kd').value),
    base_speed: parseInt(document.getElementById('base_speed').value),
    line_color_target: parseInt(document.getElementById('line_color').value),
    nav_mode: nav,
    battery_low_v: parseFloat(document.getElementById('batt_low').value),
    battery_verylow_v: parseFloat(document.getElementById('batt_vlow').value)
  };
  fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(cfg)})
  .then(r=>{
    if(r.ok){document.getElementById('msgOk').style.display='block';document.getElementById('msgErr').style.display='none';}
    else{document.getElementById('msgErr').style.display='block';document.getElementById('msgOk').style.display='none';}
  }).catch(e=>{document.getElementById('msgErr').style.display='block';});
  setTimeout(()=>{document.getElementById('msgOk').style.display='none';document.getElementById('msgErr').style.display='none';},4000);
}
loadConfig();
</script>
</body>
</html>
)rawhtml";

// ---- PAGE 3: SD Log Viewer ----
static const char DASH_PAGE_LOGS[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Waiter Robot — SD Logs</title>
<style>
  body{margin:0;font-family:'Segoe UI',sans-serif;background:#111;color:#eee;}
  nav{display:flex;gap:4px;padding:8px 12px;background:#1a1a2e;}
  nav a{padding:8px 18px;border-radius:6px;text-decoration:none;color:#aaa;font-weight:600;font-size:14px;}
  nav a.active,nav a:hover{background:#0f3460;color:#e94560;}
  h2{margin:18px 20px 8px;color:#e94560;font-size:18px;}
  .wrap{padding:0 20px 40px;}
  .usage-bar{background:#1c1c2e;border-radius:10px;padding:16px;margin-bottom:16px;}
  .usage-bar label{font-size:11px;color:#888;text-transform:uppercase;letter-spacing:1px;}
  .bar-bg{height:12px;border-radius:6px;background:#333;margin-top:8px;overflow:hidden;}
  .bar-fill{height:100%;border-radius:6px;background:linear-gradient(90deg,#2ecc71,#e94560);transition:width .5s;}
  .usage-text{font-size:13px;color:#aaa;margin-top:6px;}
  table{width:100%;border-collapse:collapse;background:#1c1c2e;border-radius:10px;overflow:hidden;}
  th{background:#0f3460;padding:12px 14px;font-size:12px;text-transform:uppercase;letter-spacing:1px;color:#e94560;text-align:left;}
  td{padding:11px 14px;border-top:1px solid #222;font-size:13px;}
  tr:hover td{background:#22223a;}
  .btn-dl{padding:5px 12px;border-radius:5px;background:#0f3460;color:#e94560;text-decoration:none;font-size:12px;font-weight:600;}
  .btn-del{padding:5px 12px;border-radius:5px;background:#4a1a1a;color:#e74c3c;border:none;cursor:pointer;font-size:12px;font-weight:600;}
  .btn-del:hover{background:#e74c3c;color:#fff;}
  .empty{text-align:center;padding:40px;color:#555;}
</style>
</head>
<body>
<nav>
  <a href="/">&#128250; Live Monitor</a>
  <a href="/config">&#9881; Calibration</a>
  <a href="/logs" class="active">&#128190; SD Logs</a>
</nav>
<h2>&#128190; SD Log Viewer</h2>
<div class="wrap">
  <div class="usage-bar">
    <label>SD Card Usage</label>
    <div class="bar-bg"><div class="bar-fill" id="sdBar" style="width:0%"></div></div>
    <div class="usage-text" id="sdUsageText">Loading…</div>
  </div>
  <table>
    <thead><tr><th>Filename</th><th>Size</th><th>Actions</th></tr></thead>
    <tbody id="logTable"><tr><td colspan="3" class="empty">Loading…</td></tr></tbody>
  </table>
</div>
<script>
function humanSize(b){if(b<1024)return b+'B';if(b<1048576)return (b/1024).toFixed(1)+'KB';return (b/1048576).toFixed(2)+'MB';}
function loadLogs(){
  fetch('/api/logs').then(r=>r.json()).then(d=>{
    var total=d.total_kb||0,free=d.free_kb||0,used=total-free;
    var pct=total?Math.min(100,(used/total*100)):0;
    document.getElementById('sdBar').style.width=pct.toFixed(1)+'%';
    document.getElementById('sdUsageText').textContent='Used: '+humanSize(used*1024)+' / '+humanSize(total*1024)+' ('+pct.toFixed(1)+'%)';
    var tbody=document.getElementById('logTable');
    if(!d.files||!d.files.length){tbody.innerHTML='<tr><td colspan="3" class="empty">No log files found.</td></tr>';return;}
    tbody.innerHTML=d.files.map(f=>`<tr>
      <td>${f.name}</td>
      <td>${humanSize(f.size)}</td>
      <td>
        <a class="btn-dl" href="/download?file=${encodeURIComponent(f.name)}">&#11015; Download</a>
        &nbsp;
        <button class="btn-del" onclick="delFile('${f.name}')">&#128465; Delete</button>
      </td>
    </tr>`).join('');
  }).catch(e=>{document.getElementById('logTable').innerHTML='<tr><td colspan="3" class="empty">Error loading logs.</td></tr>';});
}
function delFile(name){
  if(!confirm('Delete '+name+'?'))return;
  fetch('/api/logs?file='+encodeURIComponent(name),{method:'DELETE'}).then(r=>{if(r.ok)loadLogs();});
}
loadLogs();
</script>
</body>
</html>
)rawhtml";

// ============================================================
// MODULE: HAL UART CAM
// ============================================================
// Uses UART0 (Serial) — GPIO1(TX)/GPIO3(RX)
// NOTE: Disconnect UART wires when flashing firmware!

static uint8_t _uart_rx_buf[256];
static uint16_t _uart_rx_head = 0;
static uint16_t _uart_rx_tail = 0;
static const uint16_t UART_RX_BUF_MASK = 255;

void HAL_UART_CAM_Init(void) {
    // Serial already started in APP_CAM_Init
    // Flush any stale bytes
    while (Serial.available()) Serial.read();
}

// FIX [W1 / C3-header]: len widened to uint16_t; const-correct pointer
void HAL_UART_CAM_SendBytes(const uint8_t* data, uint16_t len) {
    Serial.write(data, len);
    Serial.flush();
}

bool HAL_UART_CAM_Available(void) {
    return (Serial.available() > 0);
}

uint8_t HAL_UART_CAM_ReadByte(void) {
    return (uint8_t)Serial.read();
}

// ============================================================
// MODULE: HAL SD
// ============================================================
static File _sd_log_file;
bool _sd_mounted = false;
static bool _sd_session_open = false;
static char _sd_session_path[64];
static uint32_t _sd_last_flush_ms = 0;

SD_Status HAL_SD_Init(void) {
    // SD_MMC 1-bit mode: pass true for 1-bit mode
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("[SD] Mount FAILED");
        _sd_mounted = false;
        return SD_NOT_MOUNTED;
    }
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("[SD] No card inserted");
        _sd_mounted = false;
        return SD_NOT_MOUNTED;
    }
    // Create log directory if absent
    if (!SD_MMC.exists(SD_LOG_DIR)) {
        SD_MMC.mkdir(SD_LOG_DIR);
    }
    _sd_mounted = true;
    Serial.printf("[SD] Mounted OK — Free: %u KB\n", HAL_SD_GetFreeKB());
    return SD_OK;
}

bool HAL_SD_CreateSession(uint32_t session_id) {
    if (!_sd_mounted) return false;
    if (_sd_session_open) HAL_SD_CloseSession();
    snprintf(_sd_session_path, sizeof(_sd_session_path),
             "%s/session_%06lu.bin", SD_LOG_DIR, (unsigned long)session_id);
    _sd_log_file = SD_MMC.open(_sd_session_path, FILE_WRITE);
    if (!_sd_log_file) {
        Serial.printf("[SD] Cannot create session: %s\n", _sd_session_path);
        return false;
    }
    _sd_session_open = true;
    Serial.printf("[SD] Session started: %s\n", _sd_session_path);
    return true;
}

void HAL_SD_CloseSession(void) {
    if (_sd_session_open && _sd_log_file) {
        _sd_log_file.flush();
        _sd_log_file.close();
    }
    _sd_session_open = false;
}

uint32_t HAL_SD_GetFreeKB(void) {
    if (!_sd_mounted) return 0;
    uint64_t total = SD_MMC.totalBytes();
    uint64_t used  = SD_MMC.usedBytes();
    if (total <= used) return 0;
    return (uint32_t)((total - used) / 1024ULL);
}

// BUG FIX [B8]: _sd_freekb_cache and _sd_freekb_last_check added to rate-limit
// the expensive SD_MMC.totalBytes()/usedBytes() FAT calls. These calls take
// several ms on real hardware and were previously invoked on every 20-byte write,
// introducing jitter into the main loop at up to 10 Hz.
// Free space is now re-checked at most once per SD_FLUSH_INTERVAL_MS.
static uint32_t _sd_freekb_cache      = UINT32_MAX;
static uint32_t _sd_freekb_last_check = 0;

SD_Status HAL_SD_WriteRecord(const LogRecord* rec) {
    if (!_sd_mounted)      return SD_NOT_MOUNTED;
    if (!_sd_session_open) return SD_WRITE_FAIL;

    // BUG FIX [B8]: Rate-limited free-space check — only query FAT once per flush interval.
    uint32_t now = millis();
    if ((now - _sd_freekb_last_check) >= SD_FLUSH_INTERVAL_MS || _sd_freekb_cache == UINT32_MAX) {
        _sd_freekb_cache      = HAL_SD_GetFreeKB();
        _sd_freekb_last_check = now;
    }
    if (_sd_freekb_cache < SD_MIN_FREE_KB) {
        return SD_FULL;
    }

    size_t written = _sd_log_file.write((const uint8_t*)rec, sizeof(LogRecord));
    if (written != sizeof(LogRecord)) {
        return SD_WRITE_FAIL;
    }

    // Periodic flush — reuse 'now' computed above
    if ((now - _sd_last_flush_ms) >= SD_FLUSH_INTERVAL_MS) {
        _sd_log_file.flush();
        _sd_last_flush_ms = now;
        _sd_freekb_cache  = UINT32_MAX;  // invalidate cache; re-check after flush
    }
    return SD_OK;
}

SD_Status HAL_SD_Flush(void) {
    if (!_sd_mounted || !_sd_session_open) return SD_NOT_MOUNTED;
    _sd_log_file.flush();
    _sd_last_flush_ms = millis();
    return SD_OK;
}

// ============================================================
// MODULE: RING BUFFER (PSRAM)
// ============================================================
static uint8_t*  _rb_buf     = nullptr;
static uint32_t  _rb_head    = 0;   // write pointer
static uint32_t  _rb_tail    = 0;   // read pointer
static uint32_t  _rb_used    = 0;   // bytes used
static uint32_t  _rb_count   = 0;   // frames stored
static const uint32_t RB_CAP = RINGBUF_CAPACITY_BYTES;

void RINGBUF_Init(void) {
    if (psramFound()) {
        _rb_buf = (uint8_t*)ps_malloc(RB_CAP);
        if (_rb_buf) {
            Serial.printf("[RINGBUF] Allocated %u bytes from PSRAM\n", RB_CAP);
        } else {
            Serial.println("[RINGBUF] PSRAM alloc FAILED, falling back to heap");
            _rb_buf = (uint8_t*)malloc(RB_CAP);
        }
    } else {
        Serial.println("[RINGBUF] No PSRAM found — using heap (reduced capacity)");
        _rb_buf = (uint8_t*)malloc(RB_CAP);
    }
    if (!_rb_buf) {
        Serial.println("[RINGBUF] FATAL: Cannot allocate ring buffer");
    }
    _rb_head = 0; _rb_tail = 0; _rb_used = 0; _rb_count = 0;
}

// BUG FIX [B2]: const qualifier added — Push never writes to the input frame.
// Non-const was a silent contract violation against callers passing const data.
bool RINGBUF_Push(const uint8_t* frame, uint16_t len) {
    if (!_rb_buf) return false;
    if (len == 0 || len > RINGBUF_FRAME_MAX_SIZE) return false;
    uint32_t needed = 2 + len; // 2-byte length prefix + data
    if (_rb_used + needed > RB_CAP) return false; // full

    // Write length prefix
    _rb_buf[_rb_head] = (uint8_t)(len >> 8);
    _rb_head = (_rb_head + 1) % RB_CAP;
    _rb_buf[_rb_head] = (uint8_t)(len & 0xFF);
    _rb_head = (_rb_head + 1) % RB_CAP;
    // Write data
    for (uint16_t i = 0; i < len; i++) {
        _rb_buf[_rb_head] = frame[i];
        _rb_head = (_rb_head + 1) % RB_CAP;
    }
    _rb_used  += needed;
    _rb_count++;
    return true;
}

bool RINGBUF_Pop(uint8_t* out, uint16_t* out_len) {
    if (!_rb_buf || _rb_used < 2) return false;

    uint16_t len = ((uint16_t)_rb_buf[_rb_tail] << 8);
    _rb_tail = (_rb_tail + 1) % RB_CAP;
    len |= _rb_buf[_rb_tail];
    _rb_tail = (_rb_tail + 1) % RB_CAP;

    if (len == 0 || len > RINGBUF_FRAME_MAX_SIZE || _rb_used < (uint32_t)(2 + len)) {
        // Corrupt entry — reset buffer
        RINGBUF_Flush();
        *out_len = 0;
        return false;
    }
    for (uint16_t i = 0; i < len; i++) {
        out[i] = _rb_buf[_rb_tail];
        _rb_tail = (_rb_tail + 1) % RB_CAP;
    }
    *out_len = len;
    _rb_used -= (2 + len);
    if (_rb_count > 0) _rb_count--;
    return true;
}

bool RINGBUF_IsEmpty(void) {
    return (_rb_used == 0);
}

bool RINGBUF_IsNearFull(void) {
    if (!_rb_buf) return true;
    // BUG FIX [B5]: cast to uint64_t before multiply — prevents overflow if capacity
    // or _rb_used ever approaches UINT32_MAX / 100.
    return (((uint64_t)_rb_used * 100ULL / RB_CAP) >= RINGBUF_NEAR_FULL_PERCENT);
}

void RINGBUF_Flush(void) {
    _rb_head = 0; _rb_tail = 0; _rb_used = 0; _rb_count = 0;
}

// FIX [W4 / S4-header]: narrowed to uint16_t — max slots = 65536/64 = 1024
uint16_t RINGBUF_GetCount(void) {
    return (uint16_t)_rb_count;
}

// ============================================================
// MODULE: FRAME PARSER
// ============================================================
typedef enum {
    FP_WAIT_START = 0,
    FP_WAIT_TYPE,
    FP_WAIT_LEN,
    FP_READING_PAYLOAD,
    FP_WAIT_CRC,
    FP_WAIT_END
} FP_State;

static FP_State      _fp_state         = FP_WAIT_START;
static uint8_t       _fp_type          = 0;
static uint8_t       _fp_len           = 0;
static uint8_t       _fp_payload[128]  = {0};
static uint8_t       _fp_payload_idx   = 0;
static uint8_t       _fp_crc_received  = 0;
static TelemetryFrame _fp_latest_frame = {0};
static uint32_t      _fp_corrupt_count = 0;
// BUG FIX [B10]: _fp_raw_frame removed — it was declared here but never read.
// _fp_decode_telemetry correctly uses a local stack buffer instead.

void FRAMEPARSER_Init(void) {
    _fp_state         = FP_WAIT_START;
    _fp_corrupt_count = 0;
    memset(&_fp_latest_frame, 0, sizeof(_fp_latest_frame));
    _fp_latest_frame.valid = false;
}

static void _fp_decode_telemetry(void) {
    // Payload layout (matches ESP32 telemetry packet):
    // [0..1]  enc_left   int16
    // [2..3]  enc_right  int16
    // [4..5]  heading_x10 int16
    // [6..13] state_str  8 bytes
    // [14..17] timestamp_ms uint32
    if (_fp_len < 18) return;
    TelemetryFrame f;
    memcpy(&f.enc_left,      &_fp_payload[0],  2);
    memcpy(&f.enc_right,     &_fp_payload[2],  2);
    memcpy(&f.heading_x10,   &_fp_payload[4],  2);
    memcpy(f.state_str,       &_fp_payload[6],  8);
    f.state_str[8] = '\0';
    memcpy(&f.timestamp_ms,  &_fp_payload[14], 4);
    f.valid = true;
    _fp_latest_frame = f;

    // Build raw frame for ring buffer (start, type, len, payload, crc, end)
    uint8_t raw[140];
    raw[0] = PACKET_START_BYTE;
    raw[1] = _fp_type;
    raw[2] = _fp_len;
    memcpy(&raw[3], _fp_payload, _fp_len);
    raw[3 + _fp_len] = _fp_crc_received;
    raw[4 + _fp_len] = PACKET_END_BYTE;
    // BUG FIX [B3]: raw_len typed as uint16_t — uint8_t would overflow for any
    // packet where 5 + _fp_len > 255. _fp_len is uint8_t (max 255), so max total
    // = 260, which silently wraps in uint8_t.
    uint16_t raw_len = (uint16_t)(5U + _fp_len);
    RINGBUF_Push(raw, raw_len < RINGBUF_FRAME_MAX_SIZE ? raw_len : RINGBUF_FRAME_MAX_SIZE);
}

void FRAMEPARSER_Update(void) {
    while (HAL_UART_CAM_Available()) {
        uint8_t b = HAL_UART_CAM_ReadByte();
        switch (_fp_state) {
            case FP_WAIT_START:
                if (b == PACKET_START_BYTE) _fp_state = FP_WAIT_TYPE;
                break;
            case FP_WAIT_TYPE:
                _fp_type = b;
                _fp_state = FP_WAIT_LEN;
                break;
            case FP_WAIT_LEN:
                _fp_len = b;
                _fp_payload_idx = 0;
                if (_fp_len == 0) {
                    _fp_state = FP_WAIT_CRC;
                } else if (_fp_len <= sizeof(_fp_payload)) {
                    _fp_state = FP_READING_PAYLOAD;
                } else {
                    _fp_corrupt_count++;
                    _fp_state = FP_WAIT_START;
                }
                break;
            case FP_READING_PAYLOAD:
                _fp_payload[_fp_payload_idx++] = b;
                if (_fp_payload_idx >= _fp_len) _fp_state = FP_WAIT_CRC;
                break;
            case FP_WAIT_CRC:
                _fp_crc_received = b;
                _fp_state = FP_WAIT_END;
                break;
            case FP_WAIT_END:
                if (b == PACKET_END_BYTE) {
                    // Validate CRC
                    uint8_t computed = CAM_PACKET_CRC8(_fp_payload, _fp_len);
                    if (computed == _fp_crc_received) {
                        if (_fp_type == MSG_TYPE_TELEMETRY) {
                            _fp_decode_telemetry();
                        }
                    } else {
                        _fp_corrupt_count++;
                    }
                } else {
                    _fp_corrupt_count++;
                }
                _fp_state = FP_WAIT_START;
                break;
        }
    }
}

TelemetryFrame FRAMEPARSER_GetLatest(void) {
    return _fp_latest_frame;
}

uint32_t FRAMEPARSER_GetCorruptCount(void) {
    return _fp_corrupt_count;
}

// ============================================================
// MODULE: PACKET BUILDER
// ============================================================
// CRC-8/SMBUS — polynomial 0x07, init 0x00, no reflect.
// FIX [W5]: The header incorrectly documented polynomial 0x31 (CRC-8/MAXIM).
// The actual implementation uses 0x07. Comment and header corrected to match.
// IMPORTANT: The ESP32 main board MUST use the same algorithm.
uint8_t CAM_PACKET_CRC8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else            crc <<= 1;
        }
    }
    return crc;
}

uint8_t CAM_PACKET_Build(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* out) {
    out[0] = PACKET_START_BYTE;
    out[1] = type;
    out[2] = len;
    if (len > 0) memcpy(&out[3], payload, len);
    out[3 + len] = CAM_PACKET_CRC8(payload, len);
    out[4 + len] = PACKET_END_BYTE;
    return (uint8_t)(5 + len);
}

bool CAM_PACKET_Validate(const uint8_t* frame, uint8_t len) {
    if (len < 5) return false;
    if (frame[0] != PACKET_START_BYTE) return false;
    if (frame[len-1] != PACKET_END_BYTE) return false;
    uint8_t payload_len = frame[2];
    if ((uint8_t)(5 + payload_len) != len) return false;
    uint8_t computed = CAM_PACKET_CRC8(&frame[3], payload_len);
    return (computed == frame[3 + payload_len]);
}

// ============================================================
// MODULE: SD LOGGER
// ============================================================
static uint32_t  _sdlog_frame_count = 0;
static SD_Status _sdlog_status      = SD_OK;

void SDLOGGER_Init(void) {
    _sdlog_frame_count = 0;
    _sdlog_status      = SD_OK;
}

void SDLOGGER_Update(void) {
    if (RINGBUF_IsEmpty()) return;

    uint8_t  raw[RINGBUF_FRAME_MAX_SIZE];
    uint16_t raw_len = 0;

    while (!RINGBUF_IsEmpty()) {
        if (!RINGBUF_Pop(raw, &raw_len)) break;
        if (raw_len < 5) continue; // too short to be valid

        // Validate and extract telemetry from raw packet
        // raw layout: [START][TYPE][LEN][PAYLOAD...][CRC][END]
        if (!CAM_PACKET_Validate(raw, raw_len)) continue;
        if (raw[1] != MSG_TYPE_TELEMETRY) continue;
        uint8_t plen = raw[2];
        if (plen < 18) continue;
        uint8_t* p = &raw[3];

        LogRecord rec;
        memcpy(&rec.enc_left,     &p[0],  2);
        memcpy(&rec.enc_right,    &p[2],  2);
        memcpy(&rec.heading_x10,  &p[4],  2);
        memcpy(rec.state_str,      &p[6],  8);
        memcpy(&rec.timestamp_ms, &p[14], 4);

        SD_Status result = HAL_SD_WriteRecord(&rec);
        if (result == SD_OK) {
            _sdlog_frame_count++;
            _sdlog_status = SD_OK;
        } else if (result == SD_FULL) {
            _sdlog_status = SD_FULL;
            ERRREPORTER_Report(CAM_ERR_SD_FULL);
            break;
        } else {
            _sdlog_status = SD_WRITE_FAIL;
            ERRREPORTER_Report(CAM_ERR_SD_FAIL);
            break;
        }
    }
}

uint32_t SDLOGGER_GetFrameCount(void) { return _sdlog_frame_count; }
SD_Status SDLOGGER_GetStatus(void)    { return _sdlog_status; }

// ============================================================
// MODULE: ERROR REPORTER
// ============================================================
static uint32_t _err_last_time[4] = {0, 0, 0, 0};

void ERRREPORTER_Init(void) {
    memset(_err_last_time, 0, sizeof(_err_last_time));
}

void ERRREPORTER_Report(CAM_ErrorCode code) {
    uint32_t now = millis();
    uint8_t idx = (uint8_t)code;
    if (idx >= 4) idx = 0;

    if ((now - _err_last_time[idx]) < ERR_FRAME_INTERVAL_MS) return;
    _err_last_time[idx] = now;

    uint8_t err_code_byte = 0;
    switch (code) {
        case CAM_ERR_SD_FAIL:  err_code_byte = ERR_CODE_SD_FAIL;  break;
        case CAM_ERR_SD_FULL:  err_code_byte = ERR_CODE_SD_FULL;  break;
        case CAM_ERR_BUF_FULL: err_code_byte = ERR_CODE_BUF_FULL; break;
        default: return;
    }

    uint8_t payload[2] = { err_code_byte, 0x00 };
    uint8_t pkt[16];
    uint8_t pkt_len = CAM_PACKET_Build(MSG_TYPE_ERROR, payload, 2, pkt);
    HAL_UART_CAM_SendBytes(pkt, pkt_len);
}

// ============================================================
// MODULE: WIFI STREAM
// FIX [C3]: The original handler used a blocking while(client.connected()) loop
// registered as a WebServer callback. This permanently stalled loop(), halting
// FRAMEPARSER_Update(), SDLOGGER_Update(), and WEBDASH_Handle().
// Fix: stream runs in a dedicated FreeRTOS task. The WebServer callback only
// sets a flag; the task owns the client socket independently.
// ============================================================
// BUG FIX [B6]: volatile — _stream_active is written by the mjpeg_stream FreeRTOS
// task and read by loop() core. Plain bool is a data race; volatile prevents the
// compiler from caching the value in a register across the cores.
// BUG FIX [B7]: _stream_task_handle likewise volatile for same reason.
static volatile bool       _stream_active      = false;
static volatile TaskHandle_t _stream_task_handle = NULL;

// FreeRTOS stream task — runs independently, does NOT block loop()
static void _stream_task(void* param) {
    WiFiClient client = *((WiFiClient*)param);
    delete (WiFiClient*)param;   // free the heap copy

    _stream_active = true;
    client.print("HTTP/1.1 200 OK\r\n"
                 "Content-Type: multipart/x-mixed-replace; boundary=--ESP32CAMFrame\r\n"
                 "Cache-Control: no-cache\r\n"
                 "Connection: keep-alive\r\n\r\n");

    const uint32_t frame_delay_ms = 1000 / STREAM_FPS_TARGET;
    while (client.connected()) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        client.print("--ESP32CAMFrame\r\n"
                     "Content-Type: image/jpeg\r\n"
                     "Content-Length: ");
        client.print(fb->len);
        client.print("\r\n\r\n");
        client.write(fb->buf, fb->len);
        client.print("\r\n");
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(frame_delay_ms));
    }
    _stream_active = false;
    _stream_task_handle = NULL;
    vTaskDelete(NULL);  // self-delete
}

// WebServer callback — returns immediately, spawns task for the actual stream
static WebServer* _stream_server = nullptr;
static void _stream_handle_stream(void) {
    if (!_stream_server) return;

    // Only allow one concurrent stream client
    if (_stream_active) {
        _stream_server->send(503, "text/plain", "Stream already in use");
        return;
    }

    // Heap-copy the client so the task owns it independently
    WiFiClient* client_copy = new WiFiClient(_stream_server->client());
    if (!client_copy) {
        _stream_server->send(500, "text/plain", "Out of memory");
        return;
    }

    // BUG FIX [B7 cont]: xTaskCreate requires a non-volatile TaskHandle_t*.
    // Use a local, then assign to the volatile handle atomically.
    TaskHandle_t new_handle = NULL;
    BaseType_t ret = xTaskCreate(
        _stream_task,
        "mjpeg_stream",
        4096,
        (void*)client_copy,
        1,                      // low priority — below loop() core tasks
        &new_handle
    );
    if (ret != pdPASS) {
        delete client_copy;
        _stream_server->send(500, "text/plain", "Task creation failed");
    } else {
        _stream_task_handle = new_handle;
    }
    // Do NOT call _stream_server->send() here — the task sends the HTTP response
}

// FIX [W2 from header]: returns bool — false if WiFi not connected
bool WIFISTREAM_Init(void) {
    if (!WiFi.isConnected()) {
        Serial.println("[STREAM] WiFi not connected — stream disabled");
        return false;
    }
    _stream_server = new WebServer(WEB_STREAM_PORT);
    _stream_server->on("/stream", HTTP_GET, _stream_handle_stream);
    _stream_server->begin();
    Serial.printf("[STREAM] MJPEG stream at http://%s:%d/stream\n",
                  WiFi.localIP().toString().c_str(), WEB_STREAM_PORT);
    return true;
}

void WIFISTREAM_StartStream(void) {
    if (_stream_server) _stream_server->handleClient();
}

bool WIFISTREAM_IsActive(void) { return _stream_active; }

// ============================================================
// MODULE: CONFIG SPIFFS
// FIX [C2]: Real CRC32 integrity check replaces the fake "OK\0\0" sentinel.
// FIX [C5]: SPIFFS.begin() removed from CONFIG_Init — APP_CAM_Init owns lifecycle.
// ============================================================
static RobotConfig _current_cfg;

// CRC32 over all fields in RobotConfig except the crc32 field itself.
// Computed on save, verified on load.
static uint32_t _config_crc32(const RobotConfig* cfg) {
    const uint8_t* data = (const uint8_t*)cfg;
    uint32_t crc = 0xFFFFFFFFUL;
    // Cover all bytes except the last 4 (the crc32 field)
    size_t len = sizeof(RobotConfig) - sizeof(uint32_t);
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320UL;
            else         crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

RobotConfig CONFIG_GetDefaults(void) {
    RobotConfig d;
    d.pid_kp             = 1.5f;
    d.pid_ki             = 0.05f;
    d.pid_kd             = 0.80f;
    d.base_speed         = 160;
    d.line_color_target  = 0;    // black
    d.nav_mode           = 0;    // simple
    d.battery_low_v      = 7.2f;
    d.battery_verylow_v  = 6.6f;
    d.crc32              = _config_crc32(&d);
    return d;
}

// FIX [C5]: SPIFFS.begin() removed — caller (APP_CAM_Init) already mounted SPIFFS.
// CONFIG_Init simply loads config from the already-mounted filesystem.
void CONFIG_Init(void) {
    _current_cfg = CONFIG_Load();
}

RobotConfig CONFIG_Load(void) {
    File f = SPIFFS.open("/config.bin", "r");
    if (!f || f.size() != sizeof(RobotConfig)) {
        if (f) f.close();
        Serial.println("[CFG] No valid config — defaults loaded");
        return CONFIG_GetDefaults();
    }
    RobotConfig cfg;
    f.read((uint8_t*)&cfg, sizeof(RobotConfig));
    f.close();
    // FIX [C2]: Verify real CRC32 — not a sentinel string
    uint32_t expected = _config_crc32(&cfg);
    if (cfg.crc32 != expected) {
        Serial.printf("[CFG] CRC32 FAIL (got 0x%08X, expected 0x%08X) — defaults loaded\n",
                      cfg.crc32, expected);
        return CONFIG_GetDefaults();
    }
    Serial.println("[CFG] Config loaded from SPIFFS — CRC32 OK");
    return cfg;
}

// FIX [W7 from header]: const correctness
void CONFIG_Save(const RobotConfig* cfg) {
    RobotConfig to_save = *cfg;
    to_save.crc32 = _config_crc32(&to_save);  // compute CRC over all other fields
    File f = SPIFFS.open("/config.bin", "w");
    if (!f) {
        Serial.println("[CFG] Cannot open /config.bin for write");
        return;
    }
    f.write((uint8_t*)&to_save, sizeof(RobotConfig));
    f.close();
    _current_cfg = to_save;
    Serial.println("[CFG] Config saved to SPIFFS — CRC32 written");
}

// ============================================================
// MODULE: WEB DASHBOARD
// ============================================================
static WebServer* _web_server = nullptr;
static char _web_user[32] = {0};
static char _web_pass[32] = {0};

static void _load_web_credentials(void) {
    Preferences prefs;
    prefs.begin("cam_creds", true);  // read-only
    String u = prefs.getString("web_user", "admin");    // fallback only for first boot
    String p = prefs.getString("web_pass", "changeme"); // force change on first use
    prefs.end();
    strncpy(_web_user, u.c_str(), sizeof(_web_user) - 1);
    strncpy(_web_pass, p.c_str(), sizeof(_web_pass) - 1);
}

void CONFIG_ProvisionCredentials(const char* wifi_ssid, const char* wifi_pass,
                                  const char* web_user,  const char* web_pass) {
    Preferences prefs;
    prefs.begin("cam_creds", false);  // read-write
    prefs.putString("wifi_ssid",  wifi_ssid);
    prefs.putString("wifi_pass",  wifi_pass);
    prefs.putString("web_user",   web_user);
    prefs.putString("web_pass",   web_pass);
    prefs.end();
    Serial.println("[CFG] Credentials provisioned to NVS");
}

// Authentication helper — uses NVS-loaded credentials
static bool _web_auth_check(void) {
    if (!_web_server) return false;
    if (!_web_server->authenticate(_web_user, _web_pass)) {
        _web_server->requestAuthentication(BASIC_AUTH, "WaiterRobot",
                                           "Authentication required");
        return false;
    }
    return true;
}

// -- GET / — Live Monitor
static void _web_handle_root(void) {
    if (!_web_auth_check()) return;
    _web_server->send_P(200, "text/html", DASH_PAGE_MONITOR);
}

// -- GET /config — Calibration Panel (HTML)
static void _web_handle_config_page(void) {
    if (!_web_auth_check()) return;
    _web_server->send_P(200, "text/html", DASH_PAGE_CONFIG);
}

// -- GET /logs — SD Log Viewer (HTML)
static void _web_handle_logs_page(void) {
    if (!_web_auth_check()) return;
    _web_server->send_P(200, "text/html", DASH_PAGE_LOGS);
}

// -- GET /api/status
static void _web_handle_api_status(void) {
    if (!_web_auth_check()) return;

    TelemetryFrame tf = FRAMEPARSER_GetLatest();
    uint32_t uptime_s  = millis() / 1000;
    uint32_t free_kb   = HAL_SD_GetFreeKB();
    float    free_mb   = free_kb / 1024.0f;

    char json[256];
    snprintf(json, sizeof(json),
        "{\"state\":\"%s\","
        "\"battery\":%.2f,"
        "\"frames\":%lu,"
        "\"uptime\":%lu,"
        "\"sd_free_mb\":%.1f,"
        "\"corrupt\":%lu,"
        // BUG FIX [B4]: RINGBUF_GetCount() returns uint16_t — use %u not %lu.
        // %lu on ESP32 expects unsigned long (4 bytes); uint16_t is 2 bytes.
        // Mismatch is undefined behavior and produces garbage JSON output.
        "\"buf_count\":%u}",
        tf.valid ? tf.state_str : "UNKNOWN",
        0.0f,   // battery sent from ESP32 in future extension
        (unsigned long)SDLOGGER_GetFrameCount(),
        (unsigned long)uptime_s,
        free_mb,
        (unsigned long)FRAMEPARSER_GetCorruptCount(),
        (unsigned int)RINGBUF_GetCount()
    );
    _web_server->send(200, "application/json", json);
}

// -- GET /api/config — return current config as JSON
static void _web_handle_api_config_get(void) {
    if (!_web_auth_check()) return;
    char json[256];
    snprintf(json, sizeof(json),
        "{\"pid_kp\":%.3f,"
        "\"pid_ki\":%.4f,"
        "\"pid_kd\":%.3f,"
        "\"base_speed\":%d,"
        "\"line_color_target\":%d,"
        "\"nav_mode\":%d,"
        "\"battery_low_v\":%.2f,"
        "\"battery_verylow_v\":%.2f}",
        _current_cfg.pid_kp, _current_cfg.pid_ki, _current_cfg.pid_kd,
        _current_cfg.base_speed, _current_cfg.line_color_target,
        _current_cfg.nav_mode,
        _current_cfg.battery_low_v, _current_cfg.battery_verylow_v
    );
    _web_server->send(200, "application/json", json);
}

// Simple JSON field extractor
static float _json_get_float(const String& json, const char* key, float def) {
    String search = String("\"") + key + "\":";
    int pos = json.indexOf(search);
    if (pos < 0) return def;
    pos += search.length();
    return json.substring(pos).toFloat();
}
static int _json_get_int(const String& json, const char* key, int def) {
    String search = String("\"") + key + "\":";
    int pos = json.indexOf(search);
    if (pos < 0) return def;
    pos += search.length();
    return (int)json.substring(pos).toInt();
}

// -- POST /api/config — receive JSON, validate, save, apply
static void _web_handle_api_config_post(void) {
    if (!_web_auth_check()) return;

    if (!_web_server->hasArg("plain")) {
        _web_server->send(400, "application/json", "{\"error\":\"No body\"}");
        return;
    }
    String body = _web_server->arg("plain");

    // Extract and validate fields
    float kp  = _json_get_float(body, "pid_kp",            _current_cfg.pid_kp);
    float ki  = _json_get_float(body, "pid_ki",            _current_cfg.pid_ki);
    float kd  = _json_get_float(body, "pid_kd",            _current_cfg.pid_kd);
    int   spd = _json_get_int  (body, "base_speed",        _current_cfg.base_speed);
    int   lct = _json_get_int  (body, "line_color_target", _current_cfg.line_color_target);
    int   nm  = _json_get_int  (body, "nav_mode",          _current_cfg.nav_mode);
    float blv = _json_get_float(body, "battery_low_v",     _current_cfg.battery_low_v);
    float bvl = _json_get_float(body, "battery_verylow_v", _current_cfg.battery_verylow_v);

    // Validate ranges
    if (kp  < 0.0f || kp  > 10.0f) { _web_server->send(400,"application/json","{\"error\":\"KP out of range\"}"); return; }
    if (ki  < 0.0f || ki  > 10.0f) { _web_server->send(400,"application/json","{\"error\":\"KI out of range\"}"); return; }
    if (kd  < 0.0f || kd  > 10.0f) { _web_server->send(400,"application/json","{\"error\":\"KD out of range\"}"); return; }
    if (spd < 0    || spd > 255)    { _web_server->send(400,"application/json","{\"error\":\"Speed out of range\"}"); return; }
    if (lct < 0    || lct > 1)      { _web_server->send(400,"application/json","{\"error\":\"Invalid color target\"}"); return; }
    if (nm  < 0    || nm  > 1)      { _web_server->send(400,"application/json","{\"error\":\"Invalid nav mode\"}"); return; }
    if (blv < 5.0f || blv > 9.0f)  { _web_server->send(400,"application/json","{\"error\":\"Battery low V out of range\"}"); return; }
    if (bvl < 5.0f || bvl > 9.0f)  { _web_server->send(400,"application/json","{\"error\":\"Battery vlow V out of range\"}"); return; }
    // BUG FIX [B9]: Cross-field validation — very-low threshold MUST be strictly
    // less than low threshold. Inverted values would break the ESP32 battery state
    // machine (the VERY_LOW state could never trigger).
    if (bvl >= blv) { _web_server->send(400,"application/json","{\"error\":\"battery_verylow_v must be < battery_low_v\"}"); return; }

    // Apply
    RobotConfig cfg;
    cfg.pid_kp            = kp;
    cfg.pid_ki            = ki;
    cfg.pid_kd            = kd;
    cfg.base_speed        = (int16_t)spd;
    cfg.line_color_target = (uint8_t)lct;
    cfg.nav_mode          = (uint8_t)nm;
    cfg.battery_low_v     = blv;
    cfg.battery_verylow_v = bvl;

    CONFIG_Save(&cfg);
    WEBDASH_SendConfigToESP32(&cfg);
    _web_server->send(200, "application/json", "{\"status\":\"ok\"}");
}

// -- GET /api/logs — list all log files
static void _web_handle_api_logs(void) {
    if (!_web_auth_check()) return;

    // FIX [W6]: Guard against SD not mounted
    if (!_sd_mounted) {
        _web_server->send(503, "application/json",
                          "{\"error\":\"SD not mounted\",\"files\":[]}");
        return;
    }

    uint64_t total_bytes = SD_MMC.totalBytes();
    uint64_t used_bytes  = SD_MMC.usedBytes();
    // FIX [C4]: Guard against underflow if filesystem reports used > total
    uint64_t free_bytes  = (used_bytes < total_bytes) ? (total_bytes - used_bytes) : 0ULL;
    uint32_t total_kb    = (uint32_t)(total_bytes / 1024ULL);
    uint32_t free_kb     = (uint32_t)(free_bytes  / 1024ULL);

    String json = "{\"total_kb\":";
    json += total_kb;
    json += ",\"free_kb\":";
    json += free_kb;
    json += ",\"files\":[";

    File dir = SD_MMC.open(SD_LOG_DIR);
    bool first = true;
    if (dir && dir.isDirectory()) {
        File entry = dir.openNextFile();
        while (entry) {
            if (!entry.isDirectory()) {
                if (!first) json += ",";
                first = false;
                json += "{\"name\":\"";
                // Strip /sdcard prefix if present
                String nm = String(entry.name());
                // entry.name() on SD_MMC returns full path like /logs/session_xxx.bin
                int slash = nm.lastIndexOf('/');
                if (slash >= 0) nm = nm.substring(slash + 1);
                json += nm;
                json += "\",\"size\":";
                json += (uint32_t)entry.size();
                json += "}";
            }
            entry.close();
            entry = dir.openNextFile();
        }
        dir.close();
    }
    json += "]}";
    _web_server->send(200, "application/json", json);
}

// -- GET /download?file=xxx
static void _web_handle_download(void) {
    if (!_web_auth_check()) return;
    if (!_web_server->hasArg("file")) {
        _web_server->send(400, "text/plain", "Missing file param");
        return;
    }
    String filename = _web_server->arg("file");
    // Sanitize — prevent path traversal
    if (filename.indexOf("..") >= 0 || filename.indexOf("/") >= 0) {
        _web_server->send(400, "text/plain", "Invalid filename");
        return;
    }
    String fullpath = String(SD_LOG_DIR) + "/" + filename;
    File f = SD_MMC.open(fullpath.c_str(), "r");
    if (!f) {
        _web_server->send(404, "text/plain", "File not found");
        return;
    }
    String cd = "attachment; filename=\"" + filename + "\"";
    _web_server->sendHeader("Content-Disposition", cd);
    _web_server->streamFile(f, "application/octet-stream");
    f.close();
}

// -- DELETE /api/logs?file=xxx
static void _web_handle_delete_log(void) {
    if (!_web_auth_check()) return;
    if (!_web_server->hasArg("file")) {
        _web_server->send(400, "application/json", "{\"error\":\"Missing file\"}");
        return;
    }
    String filename = _web_server->arg("file");
    if (filename.indexOf("..") >= 0 || filename.indexOf("/") >= 0) {
        _web_server->send(400, "application/json", "{\"error\":\"Invalid filename\"}");
        return;
    }
    String fullpath = String(SD_LOG_DIR) + "/" + filename;
    if (SD_MMC.remove(fullpath.c_str())) {
        _web_server->send(200, "application/json", "{\"status\":\"deleted\"}");
    } else {
        _web_server->send(404, "application/json", "{\"error\":\"Not found\"}");
    }
}

// -- POST /api/order
static void _web_handle_api_order(void) {
    if (!_web_auth_check()) return;
    uint8_t payload[1] = { 0x01 };   // order_confirmed flag
    uint8_t pkt[16];
    uint8_t pkt_len = CAM_PACKET_Build(MSG_TYPE_CAM_CONFIG, payload, 1, pkt);
    HAL_UART_CAM_SendBytes(pkt, pkt_len);
    _web_server->send(200, "application/json", "{\"status\":\"order_confirmed\"}");
}

void WEBDASH_Init(void) {
    if (!WiFi.isConnected()) {
        Serial.println("[WEB] WiFi not connected — dashboard disabled");
        return;
    }
    _web_server = new WebServer(WEB_SERVER_PORT);

    // Pages
    _web_server->on("/",        HTTP_GET,    _web_handle_root);
    _web_server->on("/config",  HTTP_GET,    _web_handle_config_page);
    _web_server->on("/logs",    HTTP_GET,    _web_handle_logs_page);

    // API
    _web_server->on("/api/status", HTTP_GET,    _web_handle_api_status);
    _web_server->on("/api/config", HTTP_GET,    _web_handle_api_config_get);
    _web_server->on("/api/config", HTTP_POST,   _web_handle_api_config_post);
    _web_server->on("/api/logs",   HTTP_GET,    _web_handle_api_logs);
    _web_server->on("/api/logs",   HTTP_DELETE, _web_handle_delete_log);
    _web_server->on("/download",   HTTP_GET,    _web_handle_download);
    _web_server->on("/api/order",  HTTP_POST,   _web_handle_api_order);

    _web_server->begin();
    Serial.printf("[WEB] Dashboard at http://%s:%d/\n",
                  WiFi.localIP().toString().c_str(), WEB_SERVER_PORT);
}

void WEBDASH_Handle(void) {
    if (_web_server) _web_server->handleClient();
}

// FIX [W6-header / W3]: const-correct; pkt buffer explicitly sized to 5 + payload
void WEBDASH_SendConfigToESP32(const RobotConfig* cfg) {
    // Pack config into payload for MSG_TYPE_CAM_CONFIG
    // Layout: [kp_f32][ki_f32][kd_f32][speed_i16][color_u8][nav_u8][batt_lo_f32][batt_vlo_f32]
    // Total payload: 4+4+4+2+1+1+4+4 = 24 bytes  →  packet = 5+24 = 29 bytes
    uint8_t payload[24];
    uint8_t idx = 0;
    memcpy(&payload[idx], &cfg->pid_kp,            4); idx += 4;
    memcpy(&payload[idx], &cfg->pid_ki,            4); idx += 4;
    memcpy(&payload[idx], &cfg->pid_kd,            4); idx += 4;
    memcpy(&payload[idx], &cfg->base_speed,        2); idx += 2;
    payload[idx++] = cfg->line_color_target;
    payload[idx++] = cfg->nav_mode;
    memcpy(&payload[idx], &cfg->battery_low_v,     4); idx += 4;
    memcpy(&payload[idx], &cfg->battery_verylow_v, 4); idx += 4;

    // FIX [W7]: buffer sized to exactly 5 + sizeof(payload) = 29 bytes
    uint8_t pkt[5 + 24];
    uint8_t pkt_len = CAM_PACKET_Build(MSG_TYPE_CAM_CONFIG, payload, idx, pkt);
    HAL_UART_CAM_SendBytes(pkt, pkt_len);
    Serial.println("[WEB] Config packet sent to ESP32");
}

// ============================================================
// MODULE: CAMERA INIT
// ============================================================
bool camera_init(void) {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = CAM_PIN_D0;
    config.pin_d1       = CAM_PIN_D1;
    config.pin_d2       = CAM_PIN_D2;
    config.pin_d3       = CAM_PIN_D3;
    config.pin_d4       = CAM_PIN_D4;
    config.pin_d5       = CAM_PIN_D5;
    config.pin_d6       = CAM_PIN_D6;
    config.pin_d7       = CAM_PIN_D7;
    config.pin_xclk     = CAM_PIN_XCLK;
    config.pin_pclk     = CAM_PIN_PCLK;
    config.pin_vsync    = CAM_PIN_VSYNC;
    config.pin_href     = CAM_PIN_HREF;
    config.pin_sscb_sda = CAM_PIN_SIOD;
    config.pin_sscb_scl = CAM_PIN_SIOC;
    config.pin_pwdn     = CAM_PIN_PWDN;
    config.pin_reset    = CAM_PIN_RESET;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if (psramFound()) {
        config.frame_size   = STREAM_FRAME_SIZE;
        config.jpeg_quality = STREAM_QUALITY;
        config.fb_count     = 2;
    } else {
        config.frame_size   = FRAMESIZE_CIF;
        config.jpeg_quality = 12;
        config.fb_count     = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed: 0x%x\n", err);
        return false;
    }

    // Fine-tune sensor
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, 0);
        s->set_hmirror(s, 0);
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_sharpness(s, 0);
        s->set_quality(s, STREAM_QUALITY);
    }
    Serial.println("[CAM] OV2640 initialized OK");
    return true;
}

// ============================================================
// MODULE: APP MAIN CAM
// ============================================================
void APP_CAM_Init(void) {
    Serial.begin(CAM_UART_BAUD);
    delay(200);

    Serial.println("[APP] ESP32-CAM WAITER LOGGER STARTING");

    // FIX [C5]: SPIFFS.begin() called ONCE here — CONFIG_Init no longer calls it
    if (!SPIFFS.begin(true)) {
        Serial.println("[APP] SPIFFS init failed — config will use defaults");
    }

    CONFIG_Init();

    // FIX [C1]: Load web credentials from NVS into module-level buffers
    _load_web_credentials();

    // SD
    SD_Status sd_result = HAL_SD_Init();
    if (sd_result != SD_OK) {
        Serial.println("[APP] SD init failed — logging disabled");
    }

    // Camera
    if (!camera_init()) {
        Serial.println("[APP] Camera init failed — stream disabled");
    }

    // Ring buffer
    RINGBUF_Init();

    // Frame parser
    FRAMEPARSER_Init();

    // SD logger
    SDLOGGER_Init();

    // Error reporter
    ERRREPORTER_Init();

    // HAL UART (uses Serial which is already started)
    HAL_UART_CAM_Init();

    // FIX [C1]: Load WiFi credentials from NVS — never from compile-time macros
    Preferences prefs;
    prefs.begin("cam_creds", true);
    String ssid = prefs.getString("wifi_ssid", "");
    String pass = prefs.getString("wifi_pass", "");
    prefs.end();

    if (ssid.length() > 0) {
        WiFi.begin(ssid.c_str(), pass.c_str());
        uint32_t t0 = millis();
        while (!WiFi.isConnected() && (millis() - t0) < WIFI_CONNECT_TIMEOUT_MS) {
            delay(200);
            Serial.print(".");
        }
        Serial.println();
        if (WiFi.isConnected()) {
            Serial.printf("[APP] WiFi connected: %s\n", WiFi.localIP().toString().c_str());
        } else {
            Serial.println("[APP] WiFi connect timeout — offline mode");
        }
    } else {
        Serial.println("[APP] No WiFi credentials in NVS — offline mode");
    }

    // Web dashboard + stream (only if WiFi connected)
    WEBDASH_Init();
    WIFISTREAM_Init();

    // Create initial SD session
    HAL_SD_CreateSession((uint32_t)(millis() / 1000));

    Serial.println("[APP] Init complete");
}

void APP_CAM_Run(void) {
    // Parse incoming UART telemetry from ESP32
    FRAMEPARSER_Update();

    // Drain ring buffer to SD card
    SDLOGGER_Update();

    // Handle web dashboard requests
    WEBDASH_Handle();

    // Handle stream server
    WIFISTREAM_StartStream();

    // Report ring buffer near-full warning
    if (RINGBUF_IsNearFull()) {
        ERRREPORTER_Report(CAM_ERR_BUF_FULL);
    }
}
