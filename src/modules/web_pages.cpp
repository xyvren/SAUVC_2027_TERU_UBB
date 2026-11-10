#include "web_pages.h"

namespace ROVWebPages {

const char CONTROL_PAGE[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="id">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AUV SAUVAC 2027 | Web Terminal &amp; Flasher</title>
<style>
:root{
  --bg:#f8fafc;
  --surface:#ffffff;
  --surface-muted:#f1f5f9;
  --surface-alt:#f8fafc;
  --border:#e2e8f0;
  --border-strong:#cbd5e1;
  --text:#0f172a;
  --text-muted:#64748b;
  --primary:#0284c7;
  --primary-hover:#0369a1;
  --primary-light:#e0f2fe;
  --emerald:#059669;
  --emerald-hover:#047857;
  --emerald-light:#d1fae5;
  --amber:#d97706;
  --amber-light:#fef3c7;
  --crimson:#dc2626;
  --crimson-hover:#b91c1c;
  --crimson-light:#fee2e2;
  --purple:#7c3aed;
  --purple-light:#ede9fe;
  --mono:'JetBrains Mono','Cascadia Code','Fira Code','SF Mono',Consolas,monospace;
  --shadow-sm:0 1px 2px 0 rgba(0,0,0,0.05);
  --shadow:0 1px 3px 0 rgba(0,0,0,0.1),0 1px 2px -1px rgba(0,0,0,0.1);
  --shadow-lg:0 10px 25px -5px rgba(15,23,42,0.15),0 8px 10px -6px rgba(15,23,42,0.1);
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font:14px/1.5 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;-webkit-font-smoothing:antialiased}
.container{max-width:1280px;margin:0 auto;padding:14px;display:flex;flex-direction:column;gap:12px}

/* HEADER */
.header{display:flex;align-items:center;justify-content:space-between;padding:12px 18px;background:var(--surface);border:1px solid var(--border);border-radius:12px;box-shadow:var(--shadow-sm)}
.brand{display:flex;align-items:center;gap:14px}
.brand-icon{width:40px;height:40px;border-radius:10px;background:var(--primary-light);border:1px solid #bae6fd;display:grid;place-items:center;color:var(--primary);font-size:1.3rem;font-weight:900}
.brand-title{font-size:1.2rem;font-weight:800;letter-spacing:0.02em;color:var(--text)}
.brand-sub{font-size:0.75rem;color:var(--primary);letter-spacing:0.08em;font-family:var(--mono);font-weight:700}
.status-pill{display:flex;align-items:center;gap:8px;padding:6px 14px;background:var(--surface-muted);border:1px solid var(--border);border-radius:20px;font-size:0.8rem;font-family:var(--mono);color:var(--text-muted)}
.dot{width:9px;height:9px;border-radius:50%;background:var(--text-muted)}
.dot.online{background:var(--emerald);box-shadow:0 0 8px var(--emerald)}
.dot.offline{background:var(--crimson);box-shadow:0 0 8px var(--crimson)}

/* TELEMETRY BAR */
.tele-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:10px}
.tele-card{background:var(--surface);border:1px solid var(--border);border-radius:10px;padding:10px 14px;display:flex;flex-direction:column;gap:4px;box-shadow:var(--shadow-sm);transition:border-color 0.15s ease}
.tele-card:hover{border-color:var(--border-strong)}
.tele-label{font-size:0.7rem;font-weight:700;letter-spacing:0.06em;color:var(--text-muted);text-transform:uppercase}
.tele-val{font-family:var(--mono);font-size:1.1rem;font-weight:700;color:var(--text);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.badge{display:inline-block;padding:2px 10px;border-radius:6px;font-size:0.75rem;font-weight:800;text-align:center;letter-spacing:0.04em}
.state-idle{background:#f1f5f9;color:#475569;border:1px solid #cbd5e1}
.state-calibrating{background:var(--amber-light);color:#92400e;border:1px solid #fde68a}
.state-running{background:var(--emerald-light);color:#065f46;border:1px solid #a7f3d0}
.state-completed{background:var(--primary-light);color:#0369a1;border:1px solid #bae6fd}
.state-abort{background:var(--crimson-light);color:#991b1b;border:1px solid #fca5a5}
.armed-yes{background:var(--crimson-light);color:#991b1b;border:1px solid #ef4444}
.armed-no{background:var(--emerald-light);color:#065f46;border:1px solid #10b981}

/* QUICK ACTIONS BAR */
.action-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px}
.btn{cursor:pointer;font-family:var(--mono);font-size:0.85rem;font-weight:700;padding:10px 16px;border-radius:8px;border:1px solid transparent;display:flex;align-items:center;justify-content:center;gap:6px;transition:all 0.15s ease;text-decoration:none;user-select:none}
.btn:hover{filter:brightness(0.95);box-shadow:var(--shadow-sm)}
.btn:active{transform:scale(0.98)}
.btn-run{background:var(--emerald);color:#ffffff}
.btn-run:hover{background:var(--emerald-hover)}
.btn-stop{background:var(--crimson);color:#ffffff}
.btn-stop:hover{background:var(--crimson-hover)}
.btn-calib{background:#ffffff;border:1px solid var(--primary);color:var(--primary)}
.btn-calib:hover{background:var(--primary-light)}
.btn-status{background:#ffffff;border:1px solid var(--border-strong);color:var(--text)}
.btn-status:hover{background:var(--surface-muted)}

/* NAVIGATION & CONTROLS */
.nav-bar{display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:10px;padding:6px;background:var(--surface);border:1px solid var(--border);border-radius:10px;box-shadow:var(--shadow-sm)}
.tab-group{display:flex;gap:6px;flex-wrap:wrap}
.tab-btn{cursor:pointer;font-family:inherit;font-size:0.85rem;font-weight:700;padding:8px 16px;border-radius:6px;border:1px solid transparent;background:transparent;color:var(--text-muted);display:flex;align-items:center;gap:6px;transition:all 0.15s ease}
.tab-btn:hover{color:var(--text);background:var(--surface-muted)}
.tab-btn.active{background:var(--primary-light);color:var(--primary);border-color:#bae6fd}
.btn-split{cursor:pointer;font-family:var(--mono);font-size:0.8rem;font-weight:700;padding:8px 14px;border-radius:6px;border:1px solid var(--border-strong);background:var(--surface);color:var(--text);display:flex;align-items:center;gap:6px;transition:all 0.15s ease}
.btn-split:hover{background:var(--surface-muted)}
.btn-split.active-split{background:var(--primary);color:#ffffff;border-color:var(--primary)}

/* PANELS CONTAINER */
.panels-wrap{display:flex;flex-direction:column;gap:12px}
.panels-wrap.split-layout{display:grid;grid-template-columns:1fr 1fr;gap:12px;align-items:stretch}
@media(max-width:960px){
  .panels-wrap.split-layout{grid-template-columns:1fr}
}

/* CARDS COMMON */
.panel-card{background:var(--surface);border:1px solid var(--border);border-radius:12px;display:flex;flex-direction:column;overflow:hidden;box-shadow:var(--shadow-sm)}
.panel-header{display:flex;align-items:center;justify-content:space-between;padding:10px 16px;background:var(--surface);border-bottom:1px solid var(--border);font-family:var(--mono);font-size:0.82rem;font-weight:700;color:var(--text)}
.panel-header-left{display:flex;align-items:center;gap:8px}
.panel-header-actions{display:flex;align-items:center;gap:10px}
.panel-header-actions label{display:flex;align-items:center;gap:4px;cursor:pointer;font-size:0.78rem;color:var(--text-muted);user-select:none}
.btn-mini{cursor:pointer;padding:4px 10px;background:#ffffff;border:1px solid var(--border-strong);border-radius:6px;color:var(--text);font-family:var(--mono);font-size:0.75rem;font-weight:600;display:flex;align-items:center;gap:4px;text-decoration:none}
.btn-mini:hover{background:var(--surface-muted);border-color:var(--text-muted)}

/* TERMINAL / LOG BODIES */
.console-body{background:var(--surface-alt);border-top:1px solid transparent;border-bottom:1px solid var(--border);height:clamp(320px,50vh,520px);overflow-y:auto;padding:14px;font-family:var(--mono);font-size:0.82rem;line-height:1.55;color:var(--text);word-break:break-word;scroll-behavior:smooth}
.term-line{padding:2px 0}

/* TERMINAL INPUT & PROMPT */
.prompt-bar-wrap{position:relative;background:var(--surface);padding:10px 14px}
.prompt-form{display:flex;align-items:center;background:#ffffff;border:1px solid var(--border-strong);border-radius:8px;padding:4px 10px;gap:8px;transition:border-color 0.15s ease,box-shadow 0.15s ease}
.prompt-form:focus-within{border-color:var(--primary);box-shadow:0 0 0 3px rgba(2,132,199,0.15)}
.prompt-tag{color:var(--primary);font-family:var(--mono);font-weight:800;font-size:0.95rem;user-select:none}
.prompt-input{flex:1;background:transparent;border:none;outline:none;color:var(--text);font-family:var(--mono);font-size:0.88rem}
.prompt-input::placeholder{color:#94a3b8}
.btn-send{cursor:pointer;background:var(--primary);color:#ffffff;border:none;border-radius:6px;font-family:var(--mono);font-size:0.8rem;font-weight:700;padding:8px 16px;transition:background 0.15s ease}
.btn-send:hover{background:var(--primary-hover)}

/* LINUX-STYLE SLASH COMMAND PALETTE */
.slash-palette{position:absolute;bottom:calc(100% + 4px);left:14px;right:14px;background:#ffffff;border:1px solid var(--border-strong);border-radius:10px;box-shadow:var(--shadow-lg);max-height:270px;overflow-y:auto;z-index:99;display:none;flex-direction:column}
.slash-palette-head{display:flex;align-items:center;justify-content:space-between;padding:6px 12px;background:var(--surface-muted);border-bottom:1px solid var(--border);font-size:0.72rem;color:var(--text-muted);font-weight:700}
.slash-item{padding:7px 12px;display:flex;align-items:baseline;gap:12px;cursor:pointer;border-bottom:1px solid var(--surface-muted);transition:background 0.1s}
.slash-item:last-child{border-bottom:none}
.slash-item:hover{background:#f0f9ff}
.slash-item.active{background:var(--primary-light);border-left:4px solid var(--primary);padding-left:8px}
.slash-cmd{font-family:var(--mono);font-size:0.82rem;font-weight:800;color:var(--primary);min-width:140px}
.slash-desc{font-size:0.78rem;color:var(--text-muted);flex:1}

/* HIGHLIGHTING IN CONSOLES */
.c-prompt{color:var(--primary);font-weight:800}
.c-cmdtext{color:var(--text);font-weight:700}
.c-ok{color:var(--emerald);font-weight:700}
.c-err{color:var(--crimson);font-weight:700}
.c-warn{color:var(--amber);font-weight:700}
.c-resp{color:#334155}

/* LOG SYSTEM BADGES */
.log-time{color:#64748b;margin-right:4px}
.log-tag{color:#0f172a;font-weight:700;margin:0 4px}
.badge-info{background:var(--primary-light);color:#0369a1;border:1px solid #bae6fd}
.badge-state{background:var(--emerald-light);color:#065f46;border:1px solid #a7f3d0}
.badge-warn{background:var(--amber-light);color:#92400e;border:1px solid #fde68a}
.badge-error{background:var(--crimson-light);color:#991b1b;border:1px solid #fca5a5}
.badge-cmd{background:var(--purple-light);color:#5b21b6;border:1px solid #ddd6fe}

/* OTA FLASHER */
.ota-box{padding:20px;display:flex;flex-direction:column;gap:14px}
.ota-desc{font-size:0.88rem;color:var(--text-muted);line-height:1.5}
.ota-desc code{background:var(--surface-muted);padding:2px 6px;border-radius:4px;font-family:var(--mono);color:var(--primary);border:1px solid var(--border)}
.ota-row{display:flex;flex-wrap:wrap;gap:12px;align-items:center}
.ota-file-wrap{flex:1;min-width:240px}
.ota-input{width:100%;padding:10px 12px;background:#ffffff;border:1px solid var(--border-strong);border-radius:8px;color:var(--text);font-family:var(--mono);font-size:0.85rem}
.ota-btn{cursor:pointer;background:var(--primary);border:none;border-radius:8px;color:#ffffff;font-family:var(--mono);font-weight:700;font-size:0.88rem;padding:11px 22px;transition:background 0.15s ease}
.ota-btn:hover{background:var(--primary-hover)}
.ota-btn:disabled{opacity:0.5;cursor:not-allowed}
.ota-progress-wrap{width:100%;height:12px;background:var(--surface-muted);border:1px solid var(--border);border-radius:6px;overflow:hidden}
.ota-progress-bar{width:0%;height:100%;background:linear-gradient(90deg,var(--primary),var(--emerald));transition:width 0.2s ease}
.ota-status{font-family:var(--mono);font-size:0.82rem;color:var(--text-muted)}

@media(max-width:680px){
  .header{flex-direction:column;align-items:flex-start;gap:10px}
  .status-pill{align-self:stretch;justify-content:center}
  .tele-grid{grid-template-columns:repeat(2,1fr)}
  .action-grid{grid-template-columns:repeat(2,1fr)}
  .nav-bar{flex-direction:column;align-items:stretch}
  .tab-group{justify-content:center}
  .btn-split{justify-content:center}
  .slash-cmd{min-width:110px}
}
</style>
</head>
<body>
<div class="container">
  <!-- TOPBAR -->
  <header class="header">
    <div class="brand">
      <div class="brand-icon">&#9875;</div>
      <div>
        <div class="brand-title">AUV SAUVAC 2027</div>
        <div class="brand-sub">UNIVERSITAS BANGKA BELITUNG &middot; PICO W DUAL-CORE &middot; FIRMWARE V1.0</div>
      </div>
    </div>
    <div class="status-pill">
      <div id="connDot" class="dot offline"></div>
      <span id="connText">CONNECTING</span> &middot; <span id="wifiAp">AUV_UBB_JUARA (192.168.4.1)</span>
    </div>
  </header>

  <!-- TELEMETRY BAR -->
  <div class="tele-grid">
    <div class="tele-card">
      <div class="tele-label">Mission State</div>
      <div class="tele-val"><span id="teleState" class="badge state-idle">IDLE</span></div>
    </div>
    <div class="tele-card">
      <div class="tele-label">Armed Status</div>
      <div class="tele-val"><span id="teleArmed" class="badge armed-no">DISARMED</span></div>
    </div>
    <div class="tele-card">
      <div class="tele-label">Depth</div>
      <div id="teleDepth" class="tele-val">0.00 m</div>
    </div>
    <div class="tele-card">
      <div class="tele-label">Pitch / Roll</div>
      <div id="teleAttitude" class="tele-val">P: 0.0&deg; &middot; R: 0.0&deg;</div>
    </div>
    <div class="tele-card">
      <div class="tele-label">Heading</div>
      <div id="teleHeading" class="tele-val">000.0&deg;</div>
    </div>
    <div class="tele-card">
      <div class="tele-label">Temperature</div>
      <div id="teleTemp" class="tele-val">0.0 &deg;C</div>
    </div>
    <div class="tele-card">
      <div class="tele-label">Power Limit</div>
      <div id="telePower" class="tele-val">25%</div>
    </div>
    <div class="tele-card">
      <div class="tele-label">Odometry (X, Y)</div>
      <div id="telePos" class="tele-val">X:0.00 Y:0.00 m</div>
    </div>
    <div class="tele-card" style="align-items:center;padding:6px 8px;">
      <div class="tele-label" style="align-self:flex-start;">Horizon (Attitude)</div>
      <svg id="horizonSvg" width="56" height="56" viewBox="-28 -28 56 56" style="border-radius:50%;background:#0284c7;border:2px solid var(--border-strong);overflow:hidden;">
        <g id="horizonGroup">
          <rect x="-40" y="0" width="80" height="50" fill="#0f766e" />
          <line x1="-40" y1="0" x2="40" y2="0" stroke="#ffffff" stroke-width="1.8" />
          <line x1="-10" y1="-8" x2="10" y2="-8" stroke="rgba(255,255,255,0.7)" stroke-width="1.2" />
          <line x1="-14" y1="-16" x2="14" y2="-16" stroke="rgba(255,255,255,0.7)" stroke-width="1.2" />
          <line x1="-10" y1="8" x2="10" y2="8" stroke="rgba(255,255,255,0.7)" stroke-width="1.2" />
          <line x1="-14" y1="16" x2="14" y2="16" stroke="rgba(255,255,255,0.7)" stroke-width="1.2" />
        </g>
        <line x1="-16" y1="0" x2="-6" y2="0" stroke="#f59e0b" stroke-width="2" />
        <line x1="6" y1="0" x2="16" y2="0" stroke="#f59e0b" stroke-width="2" />
        <circle cx="0" cy="0" r="2" fill="#f59e0b" />
      </svg>
    </div>
    <div class="tele-card" style="align-items:center;padding:6px 8px;">
      <div class="tele-label" style="align-self:flex-start;">Kompas (Heading)</div>
      <svg id="compassSvg" width="56" height="56" viewBox="-28 -28 56 56" style="border-radius:50%;background:#ffffff;border:2px solid var(--border-strong);overflow:hidden;">
        <circle cx="0" cy="0" r="25" fill="none" stroke="#e2e8f0" stroke-width="1" />
        <text x="0" y="-17" font-size="7" font-weight="900" fill="#dc2626" text-anchor="middle" dominant-baseline="central">N</text>
        <text x="18" y="0" font-size="6" font-weight="700" fill="#64748b" text-anchor="middle" dominant-baseline="central">E</text>
        <text x="0" y="18" font-size="6" font-weight="700" fill="#64748b" text-anchor="middle" dominant-baseline="central">S</text>
        <text x="-18" y="0" font-size="6" font-weight="700" fill="#64748b" text-anchor="middle" dominant-baseline="central">W</text>
        <g id="compassNeedle">
          <polygon points="0,-18 3.5,-4 0,-6 -3.5,-4" fill="#dc2626" />
          <polygon points="0,18 3.5,4 0,6 -3.5,4" fill="#64748b" />
          <circle cx="0" cy="0" r="2.5" fill="#0f172a" />
        </g>
      </svg>
    </div>
  </div>

  <!-- QUICK ACTION BUTTONS -->
  <div class="action-grid">
    <button class="btn btn-run" id="btnRun" onclick="sendCmd('/run')">&#9654; RUN</button>
    <button class="btn btn-stop" id="btnStop" onclick="sendCmd('/stop')">&#9209; STOP DARURAT</button>
    <button class="btn btn-calib" id="btnCalib" onclick="sendCmd('/calib zero')">&#10227; CALIB ZERO</button>
    <button class="btn btn-status" id="btnStatus" onclick="sendCmd('/status')">&#8505; STATUS</button>
  </div>

  <!-- NAVIGATION TABS & SPLIT VIEW TOGGLE -->
  <nav class="nav-bar">
    <div class="tab-group">
      <button class="tab-btn active" id="tabTerminal" onclick="setTab('terminal')">&#128187; Terminal Perintah</button>
      <button class="tab-btn" id="tabSysLog" onclick="setTab('log')">&#128203; Log Sistem &amp; Audit</button>
      <button class="tab-btn" id="tabOta" onclick="setTab('ota')">&#128230; OTA Firmware Update</button>
    </div>
    <button class="btn-split" id="btnToggleSplit" onclick="toggleSplitView()">&#9707; Tampilan Berdampingan</button>
  </nav>

  <!-- MAIN PANELS WRAPPER -->
  <div class="panels-wrap" id="panelsContainer">

    <!-- PANEL 1: TERMINAL PERINTAH (CLI INTERAKTIF) -->
    <section class="panel-card" id="panelTerminal">
      <div class="panel-header">
        <div class="panel-header-left">
          <span>&#128187; TERMINAL PERINTAH (CLI INTERAKTIF)</span>
        </div>
        <div class="panel-header-actions">
          <button class="btn-mini" onclick="clearCmdTerm()">&#129529; Bersihkan</button>
        </div>
      </div>
      <div class="console-body" id="cmdOutput"></div>
      <div class="prompt-bar-wrap">
        <!-- Floating Linux-style Slash Command Palette -->
        <div class="slash-palette" id="slashPalette">
          <div class="slash-palette-head">
            <span>&#9889; MENU PERINTAH CEPAT (PILIH / NAVIGASI)</span>
            <span>&uarr; &darr; Pilih &middot; Tab/Enter Masukkan &middot; Esc Tutup</span>
          </div>
          <div id="slashList"></div>
        </div>
        <form class="prompt-form" id="termForm" onsubmit="onFormSubmit(event)">
          <span class="prompt-tag">AUV&gt;</span>
          <input type="text" id="termInput" class="prompt-input" placeholder="Ketik / untuk memunculkan perintah (/run, /depth 50, /status, /help)..." autocomplete="off" spellcheck="false">
          <button type="submit" class="btn-send">Kirim</button>
        </form>
      </div>
    </section>

    <!-- PANEL 2: LOG SISTEM & AUDIT BLACK BOX -->
    <section class="panel-card" id="panelSysLog" style="display:none">
      <div class="panel-header">
        <div class="panel-header-left">
          <span>&#128203; LOG SISTEM &amp; AUDIT BLACK BOX</span>
        </div>
        <div class="panel-header-actions">
          <label><input type="checkbox" id="chkAutoScrollLog" checked> Auto-Scroll</label>
          <button class="btn-mini" onclick="clearSysLogServer()">&#129529; Bersihkan Log</button>
          <a class="btn-mini" href="/api/log/download" download="auv_audit_log.txt">&#11015; Unduh auv_audit_log.txt</a>
          <a class="btn-mini" href="/api/blackbox/download" download="auv_blackbox.csv">&#11015; Unduh CSV Blackbox</a>
        </div>
      </div>
      <div class="console-body" id="sysLogOutput"></div>
    </section>

    <!-- PANEL 3: WIRELESS OTA FIRMWARE UPDATE -->
    <section class="panel-card" id="panelOta" style="display:none">
      <div class="panel-header">
        <div class="panel-header-left">
          <span>&#128230; WIRELESS OTA FIRMWARE UPDATE</span>
        </div>
      </div>
      <div class="ota-box">
        <div class="ota-desc">
          Unggah berkas biner PlatformIO (<code>firmware.bin</code> dari direktori <code>.pio/build/picow/firmware.bin</code>) secara nirkabel melalui SoftAP Pico W. Flashing berlangsung cepat tanpa membuka hull kedap air AUV.
        </div>
        <div class="ota-row">
          <div class="ota-file-wrap">
            <input type="file" id="otaFile" class="ota-input" accept=".bin">
          </div>
          <button id="otaBtn" class="ota-btn" onclick="startOtaUpload()">Flash Firmware Nirkabel</button>
        </div>
        <div class="ota-progress-wrap">
          <div id="otaBar" class="ota-progress-bar"></div>
        </div>
        <div id="otaStatus" class="ota-status">Siaga. Pilih berkas firmware.bin untuk memulai flashing.</div>
      </div>
    </section>

  </div>
</div>

<script>
const SLASH_COMMANDS = [
  { cmd: '/run', label: '/run', desc: 'Mulai peluncuran misi otonom SAUVC' },
  { cmd: '/stop', label: '/stop', desc: 'Stop darurat (netralkan semua motor)' },
  { cmd: '/status', label: '/status', desc: 'Cek laporan komprehensif sensor & telemetri' },
  { cmd: '/depth ', label: '/depth <cm>', desc: 'Set target kedalaman selam (cm)' },
  { cmd: '/swim ', label: '/swim <sec>', desc: 'Set durasi berenang maju (detik)' },
  { cmd: '/power ', label: '/power <%>', desc: 'Set batas daya motor keseluruhan (%)' },
  { cmd: '/autorun ', label: '/autorun <on/off>', desc: 'Aktifkan/nonaktifkan auto-run saat boot' },
  { cmd: '/rev_boost ', label: '/rev_boost <val>', desc: 'Set pengali daya dorong mundur (1.0 - 2.0)' },
  { cmd: '/set ssid ', label: '/set ssid <nama>', desc: 'Ubah nama hotspot Wi-Fi' },
  { cmd: '/set wifi_pass ', label: '/set wifi_pass <pass>', desc: 'Ubah password Wi-Fi WPA2 min 8 karakter' },
  { cmd: '/calib zero', label: '/calib zero', desc: 'Kalibrasi nol sensor kedalaman di permukaan air' },
  { cmd: '/calib ballast', label: '/calib ballast', desc: 'Ukur daya apung netral dan heave bias' },
  { cmd: '/calib compass', label: '/calib compass', desc: 'Kalibrasi kompas digital magnetometer' },
  { cmd: '/calib level', label: '/calib level', desc: 'Kalibrasi posisi datar sensor IMU MPU-6050' },
  { cmd: '/rl status', label: '/rl status', desc: 'Status gain adaptif real-time RL-PID vs baseline' },
  { cmd: '/rl on', label: '/rl on', desc: 'Aktifkan adaptif learning RL-PID online tuning' },
  { cmd: '/rl off', label: '/rl off', desc: 'Nonaktifkan adaptif learning (kembali ke baseline PID)' },
  { cmd: '/set rl_rate ', label: '/set rl_rate <val>', desc: 'Set learning rate adaptif RL (0.0001 - 0.1)' },
  { cmd: '/set rl_max_kp ', label: '/set rl_max_kp <val>', desc: 'Set batas atas adaptasi gain Kp (0.01 - 2.0)' },
  { cmd: '/sim ', label: '/sim <on/off>', desc: 'Mode simulasi meja (daya motor aman 3%)' },
  { cmd: '/thrusters', label: '/thrusters', desc: 'Tampilkan tabel matriks konfigurasi 6 thruster' },
  { cmd: '/set thruster ', label: '/set thruster <id> <prop> <val>', desc: 'Konfigurasi thruster (reverse, rev_boost, dll)' },
  { cmd: '/test motor ', label: '/test motor <id> <pwm/%>', desc: 'Uji putaran motor T1-T6 selama 2 detik' },
  { cmd: '/tes motor ', label: '/tes motor <id> <pwm/%>', desc: 'Alias uji putaran motor T1-T6 selama 2 detik' },
  { cmd: '/selftest', label: '/selftest', desc: 'Pre-flight self-test hardware, sensor, & thruster' },
  { cmd: '/stream ', label: '/stream <on/off>', desc: 'Stream telemetri JSON 20Hz ke Raspberry Pi 5 via USB Serial' },
  { cmd: '/blackbox ', label: '/blackbox <dump/clear>', desc: 'Kelola file rekaman underwater flight recorder CSV' },
  { cmd: '/esc_calib ', label: '/esc_calib <start/cancel>', desc: 'Kalibrasi rentang throttle ESC PWM' },
  { cmd: '/get all', label: '/get all', desc: 'Tampilkan seluruh konfigurasi parameter aktif' },
  { cmd: '/save', label: '/save', desc: 'Simpan konfigurasi RAM saat ini ke EEPROM' },
  { cmd: '/log clear', label: '/log clear', desc: 'Bersihkan seluruh riwayat log di ring-buffer' },
  { cmd: '/log dump', label: '/log dump', desc: 'Tampilkan seluruh riwayat log ring-buffer' },
  { cmd: '/reboot', label: '/reboot', desc: 'Reboot Raspberry Pi Pico W' },
  { cmd: '/help', label: '/help', desc: 'Tampilkan panduan perintah lengkap sistem' }
];

let seenLogLines = new Set();
let cmdHistory = [];
let histIdx = -1;
let histDraft = '';

let splitMode = false;
let currentTab = 'terminal';

let paletteVisible = false;
let paletteItems = [];
let paletteIdx = 0;

function escapeHtml(s){
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

/* TABS & SPLIT VIEW */
function setTab(tab){
  currentTab = tab;
  const pTerm = document.getElementById('panelTerminal');
  const pLog = document.getElementById('panelSysLog');
  const pOta = document.getElementById('panelOta');
  const tTerm = document.getElementById('tabTerminal');
  const tLog = document.getElementById('tabSysLog');
  const tOta = document.getElementById('tabOta');

  if (splitMode){
    if (tab === 'ota'){
      pTerm.style.display = 'none';
      pLog.style.display = 'none';
      pOta.style.display = 'block';
      tTerm.classList.remove('active');
      tLog.classList.remove('active');
      tOta.classList.add('active');
    } else {
      pTerm.style.display = 'flex';
      pLog.style.display = 'flex';
      pOta.style.display = 'none';
      tTerm.classList.toggle('active', tab === 'terminal');
      tLog.classList.toggle('active', tab === 'log');
      tOta.classList.remove('active');
    }
    return;
  }

  tTerm.classList.toggle('active', tab === 'terminal');
  tLog.classList.toggle('active', tab === 'log');
  tOta.classList.toggle('active', tab === 'ota');

  pTerm.style.display = (tab === 'terminal') ? 'flex' : 'none';
  pLog.style.display = (tab === 'log') ? 'flex' : 'none';
  pOta.style.display = (tab === 'ota') ? 'block' : 'none';
}

function toggleSplitView(){
  splitMode = !splitMode;
  const btn = document.getElementById('btnToggleSplit');
  const container = document.getElementById('panelsContainer');

  if (splitMode){
    btn.classList.add('active-split');
    btn.innerHTML = '&#9638; Mode Tab Tunggal';
    container.classList.add('split-layout');
    document.getElementById('panelTerminal').style.display = 'flex';
    document.getElementById('panelSysLog').style.display = 'flex';
    document.getElementById('panelOta').style.display = 'none';
    document.getElementById('tabTerminal').classList.add('active');
    document.getElementById('tabSysLog').classList.add('active');
    document.getElementById('tabOta').classList.remove('active');
  } else {
    btn.classList.remove('active-split');
    btn.innerHTML = '&#9707; Tampilan Berdampingan';
    container.classList.remove('split-layout');
    setTab(currentTab === 'ota' ? 'ota' : 'terminal');
  }
}

/* TERMINAL OUTPUT & LOGIC */
function appendCmdLine(html){
  const el = document.getElementById('cmdOutput');
  const d = document.createElement('div');
  d.className = 'term-line';
  d.innerHTML = html;
  el.appendChild(d);
  el.scrollTop = el.scrollHeight;
}

function clearCmdTerm(){
  document.getElementById('cmdOutput').innerHTML = '<div class="term-line" style="color:var(--text-muted)">Terminal dibersihkan. Ketik / untuk membuka daftar perintah.</div>';
}

function formatReply(raw){
  const lines = raw.split(/\r?\n/);
  for (const l of lines){
    if (!l.trim()) continue;
    let s = escapeHtml(l);
    if (/^(OK|SUCCESS)/i.test(l.trim())){
      s = '<span class="c-ok">' + s + '</span>';
    } else if (/^(ERR|ERROR|FAIL)/i.test(l.trim())){
      s = '<span class="c-err">' + s + '</span>';
    } else if (/^(WARN)/i.test(l.trim())){
      s = '<span class="c-warn">' + s + '</span>';
    } else {
      s = '<span class="c-resp">' + s + '</span>';
    }
    appendCmdLine(s);
  }
}

async function sendCmd(cmd){
  if (!cmd || !cmd.trim()) return;
  cmd = cmd.trim();
  hidePalette();
  appendCmdLine('<span class="c-prompt">AUV&gt;</span> <span class="c-cmdtext">' + escapeHtml(cmd) + '</span>');
  try {
    const res = await fetch('/api/terminal', {
      method: 'POST',
      headers: {'Content-Type': 'application/x-www-form-urlencoded'},
      body: 'cmd=' + encodeURIComponent(cmd)
    });
    const txt = await res.text();
    if (txt) formatReply(txt);
  } catch(err){
    appendCmdLine('<span class="c-err">[ERROR] Gagal mengirim: ' + escapeHtml(err.message) + '</span>');
  }
}

function onFormSubmit(e){
  e.preventDefault();
  const input = document.getElementById('termInput');
  const val = input.value.trim();
  if (!val) return;
  cmdHistory.push(val);
  histIdx = -1;
  histDraft = '';
  input.value = '';
  sendCmd(val);
}

/* SLASH COMMAND PALETTE */
function renderPalette(query){
  const listEl = document.getElementById('slashList');
  const palEl = document.getElementById('slashPalette');
  const q = query.toLowerCase().trim();
  const search = q.startsWith('/') ? q.substring(1) : q;

  paletteItems = SLASH_COMMANDS.filter(item => {
    if (!search) return true;
    const c = item.cmd.toLowerCase().replace('/', '');
    const l = item.label.toLowerCase();
    const d = item.desc.toLowerCase();
    return c.includes(search) || l.includes(search) || d.includes(search);
  });

  if (paletteItems.length === 0){
    palEl.style.display = 'none';
    paletteVisible = false;
    return;
  }

  paletteIdx = 0;
  listEl.innerHTML = '';
  paletteItems.forEach((item, idx) => {
    const div = document.createElement('div');
    div.className = 'slash-item' + (idx === 0 ? ' active' : '');
    div.innerHTML = '<span class="slash-cmd">' + escapeHtml(item.label) + '</span><span class="slash-desc">' + escapeHtml(item.desc) + '</span>';
    div.onmousedown = (e) => {
      e.preventDefault();
      applySlashCommand(item.cmd);
    };
    listEl.appendChild(div);
  });

  palEl.style.display = 'flex';
  paletteVisible = true;
}

function hidePalette(){
  const palEl = document.getElementById('slashPalette');
  palEl.style.display = 'none';
  paletteVisible = false;
}

function updatePaletteHighlight(){
  const listEl = document.getElementById('slashList');
  const children = listEl.children;
  for (let i = 0; i < children.length; i++){
    if (i === paletteIdx){
      children[i].classList.add('active');
      children[i].scrollIntoView({ block: 'nearest' });
    } else {
      children[i].classList.remove('active');
    }
  }
}

function applySlashCommand(cmd){
  const input = document.getElementById('termInput');
  input.value = cmd;
  hidePalette();
  input.focus();
}

const termInput = document.getElementById('termInput');
termInput.addEventListener('input', () => {
  const val = termInput.value;
  if (val.startsWith('/')){
    renderPalette(val);
  } else {
    hidePalette();
  }
});

termInput.addEventListener('keydown', (e) => {
  if (paletteVisible && paletteItems.length > 0){
    if (e.key === 'ArrowDown'){
      e.preventDefault();
      paletteIdx = (paletteIdx + 1) % paletteItems.length;
      updatePaletteHighlight();
      return;
    } else if (e.key === 'ArrowUp'){
      e.preventDefault();
      paletteIdx = (paletteIdx - 1 + paletteItems.length) % paletteItems.length;
      updatePaletteHighlight();
      return;
    } else if (e.key === 'Tab' || e.key === 'Enter'){
      e.preventDefault();
      if (paletteIdx >= 0 && paletteItems[paletteIdx]){
        applySlashCommand(paletteItems[paletteIdx].cmd);
      }
      return;
    } else if (e.key === 'Escape'){
      e.preventDefault();
      hidePalette();
      return;
    }
  }

  // History browsing when palette is closed
  if (!paletteVisible){
    if (e.key === 'ArrowUp'){
      e.preventDefault();
      if (!cmdHistory.length) return;
      if (histIdx === -1){
        histDraft = termInput.value;
        histIdx = cmdHistory.length - 1;
      } else if (histIdx > 0){
        histIdx--;
      }
      termInput.value = cmdHistory[histIdx];
    } else if (e.key === 'ArrowDown'){
      e.preventDefault();
      if (histIdx === -1) return;
      if (histIdx < cmdHistory.length - 1){
        histIdx++;
        termInput.value = cmdHistory[histIdx];
      } else {
        histIdx = -1;
        termInput.value = histDraft;
      }
    }
  }
});

/* LOG SISTEM & AUDIT */
function formatLogLine(raw){
  let s = escapeHtml(raw);
  s = s.replace(/^\[(\d{2}:\d{2}:\d{2}\.\d{3})\]/, '<span class="log-time">[$1]</span>');
  s = s.replace(/\[INFO\]/g, '<span class="badge badge-info">INFO</span>')
       .replace(/\[WARN\]/g, '<span class="badge badge-warn">WARN</span>')
       .replace(/\[ERROR\]/g, '<span class="badge badge-error">ERROR</span>')
       .replace(/\[STATE\]/g, '<span class="badge badge-state">STATE</span>')
       .replace(/\[CMD\]/g, '<span class="badge badge-cmd">CMD</span>');
  s = s.replace(/\[([A-Z0-9_-]+)\]/g, '<span class="log-tag">[$1]</span>');
  return s;
}

function appendSysLogLine(html){
  const el = document.getElementById('sysLogOutput');
  const d = document.createElement('div');
  d.className = 'term-line';
  d.innerHTML = html;
  el.appendChild(d);
  if (document.getElementById('chkAutoScrollLog').checked){
    el.scrollTop = el.scrollHeight;
  }
}

async function pollLogs(){
  try {
    const res = await fetch('/api/terminal/poll');
    if (!res.ok) return;
    const txt = await res.text();
    if (!txt) return;
    const lines = txt.split(/\r?\n/).filter(l => l.trim().length > 0);
    for (const l of lines){
      if (!seenLogLines.has(l)){
        seenLogLines.add(l);
        appendSysLogLine(formatLogLine(l));
      }
    }
    if (seenLogLines.size > 300){
      seenLogLines = new Set(Array.from(seenLogLines).slice(-150));
    }
  } catch(e){}
}

async function clearSysLogServer(){
  try {
    await fetch('/api/cli', {
      method: 'POST',
      headers: {'Content-Type': 'application/x-www-form-urlencoded'},
      body: 'cmd=log+clear'
    });
    document.getElementById('sysLogOutput').innerHTML = '';
    seenLogLines.clear();
    appendSysLogLine('<span class="badge badge-state">STATE</span> <span class="log-tag">[SYS]</span> Log sistem & audit berhasil dibersihkan.');
  } catch(e){
    alert('Gagal membersihkan log: ' + e.message);
  }
}

let consecutiveMisses = 0;
let pollingBusy = false;

/* TELEMETRY UPDATES */
async function updateTelemetry(){
  try {
    const res = await fetch('/api/telemetry');
    if (!res.ok) throw new Error();
    const d = await res.json();
    consecutiveMisses = 0;

    const st = d.missionState || (d.auto && d.auto.enabled ? 'RUNNING' : 'IDLE');
    const stEl = document.getElementById('teleState');
    stEl.textContent = st;
    stEl.className = 'badge state-' + st.toLowerCase();

    const armEl = document.getElementById('teleArmed');
    armEl.textContent = d.armed ? 'ARMED' : 'DISARMED';
    armEl.className = 'badge ' + (d.armed ? 'armed-yes' : 'armed-no');

    const depth = (d.depthM !== undefined) ? d.depthM : (d.pressure ? d.pressure.depthM : 0);
    document.getElementById('teleDepth').textContent = Number(depth).toFixed(2) + ' m';

    const p = (d.pitch !== undefined) ? d.pitch : (d.imu ? d.imu.pitch : 0);
    const r = (d.roll !== undefined) ? d.roll : (d.imu ? d.imu.roll : 0);
    document.getElementById('teleAttitude').textContent = 'P: ' + Number(p).toFixed(1) + '° · R: ' + Number(r).toFixed(1) + '°';

    const h = (d.heading !== undefined) ? d.heading : (d.compass ? d.compass.heading : 0);
    document.getElementById('teleHeading').textContent = Number(h).toFixed(1) + '°';

    const temp = (d.temperatureC !== undefined) ? d.temperatureC : (d.pressure ? d.pressure.temperatureC : 0);
    document.getElementById('teleTemp').textContent = Number(temp).toFixed(1) + ' °C';

    const pwr = (d.powerLimit !== undefined) ? d.powerLimit : 25;
    document.getElementById('telePower').textContent = pwr + '%';

    // Odometry Pool Coordinates
    const x = (d.posX !== undefined) ? Number(d.posX).toFixed(2) : '0.00';
    const y = (d.posY !== undefined) ? Number(d.posY).toFixed(2) : '0.00';
    const elPos = document.getElementById('telePos');
    if (elPos) elPos.textContent = 'X:' + x + ' Y:' + y + ' m';

    // Mini Artificial Horizon widget
    const hGroup = document.getElementById('horizonGroup');
    if (hGroup) {
      const clampedP = Math.max(-25, Math.min(25, Number(p)));
      hGroup.setAttribute('transform', 'rotate(' + (-Number(r)) + ') translate(0,' + (clampedP * 0.7) + ')');
    }

    // Mini Compass needle widget
    const cNeedle = document.getElementById('compassNeedle');
    if (cNeedle) {
      cNeedle.setAttribute('transform', 'rotate(' + Number(h) + ')');
    }

    document.getElementById('connDot').className = 'dot online';
    document.getElementById('connText').textContent = 'ONLINE';
  } catch(e){
    consecutiveMisses++;
    if (consecutiveMisses >= 4) {
      document.getElementById('connDot').className = 'dot offline';
      document.getElementById('connText').textContent = 'OFFLINE';
    }
  }
}

/* WIRELESS OTA FIRMWARE FLASHER */
function startOtaUpload(){
  const fileInput = document.getElementById('otaFile');
  const file = fileInput.files[0];
  if (!file){
    alert('Pilih berkas firmware.bin terlebih dahulu!');
    return;
  }
  if (!file.name.toLowerCase().endsWith('.bin')){
    if (!confirm('Peringatan: Berkas bukan bereksistensi .bin! Tetap lanjutkan?')) return;
  }
  if (!confirm('Flash firmware ' + file.name + ' (' + (file.size/1024).toFixed(1) + ' KB) ke AUV SAUVAC? AUV akan reboot otomatis setelah selesai.')) return;

  const btn = document.getElementById('otaBtn');
  const bar = document.getElementById('otaBar');
  const stat = document.getElementById('otaStatus');

  btn.disabled = true;
  fileInput.disabled = true;
  bar.style.width = '0%';
  stat.textContent = 'Menghubungkan dan mengunggah firmware...';
  stat.style.color = 'var(--primary)';

  const formData = new FormData();
  formData.append('firmware', file, file.name);

  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/update', true);

  xhr.upload.onprogress = (e) => {
    if (e.lengthComputable){
      const pct = Math.round((e.loaded / e.total) * 100);
      bar.style.width = pct + '%';
      stat.textContent = 'Mengunggah firmware: ' + pct + '% (' + (e.loaded/1024).toFixed(0) + ' / ' + (e.total/1024).toFixed(0) + ' KB)...';
    }
  };

  xhr.onload = () => {
    if (xhr.status === 200){
      bar.style.width = '100%';
      stat.textContent = 'Sukses! Firmware terpasang. AUV sedang me-reboot...';
      stat.style.color = 'var(--emerald)';
      appendCmdLine('<span class="c-ok">[OTA] Firmware flashing sukses! AUV me-reboot otomatis...</span>');
      let sec = 8;
      const t = setInterval(() => {
        sec--;
        if (sec <= 0){
          clearInterval(t);
          window.location.reload();
        } else {
          stat.textContent = 'Sukses! AUV me-reboot. Memuat ulang halaman dalam ' + sec + ' detik...';
        }
      }, 1000);
    } else {
      bar.style.width = '0%';
      stat.textContent = 'Gagal flashing firmware: HTTP ' + xhr.status + ' ' + xhr.responseText;
      stat.style.color = 'var(--crimson)';
      btn.disabled = false;
      fileInput.disabled = false;
    }
  };

  xhr.onerror = () => {
    bar.style.width = '0%';
    stat.textContent = 'Koneksi terputus saat mengunggah firmware.';
    stat.style.color = 'var(--crimson)';
    btn.disabled = false;
    fileInput.disabled = false;
  };

  xhr.send(formData);
}

// Initial Banner in Interactive Terminal
appendCmdLine('<span class="c-prompt">=== AUV SAUVAC 2027 INTERACTIVE CLI (Firmware v1.0) ===</span>');
appendCmdLine('<span class="c-resp">Ketik <b>/</b> untuk membuka menu perintah cepat (autocomplete).</span>');
appendCmdLine('<span class="c-resp">Gunakan <b>&uarr; &darr;</b> untuk navigasi menu atau riwayat perintah.</span>');
appendCmdLine('<span class="c-resp">Contoh perintah: <b>/status</b>, <b>/depth 50</b>, <b>/run</b>, <b>/stop</b>, <b>/help</b></span>');

async function consolidatedPoll(){
  if (pollingBusy) return;
  pollingBusy = true;
  try {
    await updateTelemetry();
    await pollLogs();
  } finally {
    pollingBusy = false;
  }
}

setInterval(consolidatedPoll, 1000);
consolidatedPoll();
</script>
</body>
</html>)rawliteral";

const char SETUP_PAGE[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="id">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Setup AUV SAUVAC 2027</title>
<style>
:root{
  --bg:#f8fafc;
  --surface:#ffffff;
  --border:#e2e8f0;
  --border-strong:#cbd5e1;
  --text:#0f172a;
  --text-muted:#64748b;
  --primary:#0284c7;
  --primary-hover:#0369a1;
  --primary-light:#e0f2fe;
}
*{box-sizing:border-box;margin:0;padding:0}
body{font:15px/1.5 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;background:var(--bg);color:var(--text);display:grid;min-height:100vh;place-items:center;padding:16px}
.card{max-width:480px;width:100%;background:var(--surface);border:1px solid var(--border);border-radius:12px;padding:28px;box-shadow:0 10px 25px -5px rgba(15,23,42,0.08),0 8px 10px -6px rgba(15,23,42,0.04)}
.card-header{display:flex;align-items:center;gap:12px;margin-bottom:12px}
.icon-badge{width:40px;height:40px;border-radius:10px;background:var(--primary-light);border:1px solid #bae6fd;display:grid;place-items:center;color:var(--primary);font-size:1.3rem}
h1{font-size:1.25rem;font-weight:800;color:var(--text)}
p{color:var(--text-muted);font-size:0.88rem;margin-bottom:20px;line-height:1.5}
label{display:block;margin:14px 0 6px;color:var(--text);font-size:0.85rem;font-weight:700}
input{width:100%;padding:10px 12px;background:#ffffff;border:1px solid var(--border-strong);border-radius:8px;color:var(--text);font:inherit}
input:focus{outline:none;border-color:var(--primary);box-shadow:0 0 0 3px rgba(2,132,199,0.15)}
button{cursor:pointer;width:100%;margin-top:24px;padding:12px;background:var(--primary);border:none;border-radius:8px;color:#ffffff;font:inherit;font-weight:700;transition:background 0.15s ease}
button:hover{background:var(--primary-hover)}
</style>
</head>
<body>
<div class="card">
  <div class="card-header">
    <div class="icon-badge">&#9875;</div>
    <h1>Setup Hotspot AUV SAUVAC</h1>
  </div>
  <p>Pico W membuat hotspot darurat di <b>192.168.4.1</b>. Buat password web kontrol (minimal 6 karakter) untuk menyelesaikan inisialisasi konfigurasi sistem.</p>
  <form method="post" action="/setup">
    <label>Nama Hotspot Wi-Fi (SSID)</label>
    <input name="ssid" value="AUV_UBB_JUARA" maxlength="32" required>
    <label>Password Web Kontrol</label>
    <input name="webPass" type="password" minlength="6" maxlength="32" autocomplete="new-password" placeholder="Minimal 6 karakter" required>
    <button type="submit">Simpan &amp; Reboot AUV</button>
  </form>
</div>
</body>
</html>)rawliteral";

}  // namespace ROVWebPages
