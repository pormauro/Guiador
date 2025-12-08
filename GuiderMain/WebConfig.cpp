// File: WebConfig.cpp
#include "WebConfig.h"
#include <WiFi.h>
#include <WebServer.h>

#include "Config.h"
#include "Control.h"
#include "Log.h"

// --------- CONFIG WIFI AP ---------
static const char *AP_SSID = "DEPROS-GUIDER";
static const char *AP_PASS = "depros1234";

static WebServer server(80);

// --------- HTML PRINCIPAL ---------
static const char MAIN_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<title>DEPROS Guider ESP32</title>
<style>
body{font-family:Arial,Helvetica,sans-serif;margin:0;background:#f4f4f4;}
#wrap{max-width:1100px;margin:10px auto;background:#fff;border-radius:8px;
      box-shadow:0 0 10px rgba(0,0,0,0.15);padding:16px;}
h1{margin-top:0;}
h2,h3{margin-bottom:6px;}
.status{background:#eee;padding:8px;border-radius:6px;font-size:14px;margin-bottom:10px;}
.grid{display:flex;gap:16px;flex-wrap:wrap;}
.col{flex:1;min-width:320px;}
label{display:block;font-size:12px;margin-top:6px;}
input{width:100%;padding:4px;margin-top:2px;font-size:12px;box-sizing:border-box;}
button{margin-top:6px;padding:6px 12px;background:#0077cc;color:#fff;border:none;
       border-radius:4px;cursor:pointer;font-size:12px;}
button:hover{background:#005fa3;}
canvas{width:100%;height:250px;border:1px solid #ccc;border-radius:4px;}
.badge{display:inline-block;padding:2px 6px;border-radius:4px;font-size:11px;margin-left:4px;}
.badge-ok{background:#c8e6c9;color:#256029;}
.badge-warn{background:#fff9c4;color:#827717;}
.badge-err{background:#ffcdd2;color:#b71c1c;}
.flex-row{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin-top:4px;}
.io-pill{display:inline-block;padding:3px 8px;border-radius:12px;font-size:11px;
         border:1px solid #ccc;margin:2px 4px 0 0;}
.io-on{background:#c8e6c9;border-color:#2e7d32;color:#1b5e20;}
.io-off{background:#ffcdd2;border-color:#c62828;color:#b71c1c;}
.switch-row{display:flex;align-items:center;gap:6px;margin-top:4px;}
#logBox{
  width:100%;
  min-height:200px;
  max-height:300px;
  border:1px solid #ccc;
  border-radius:4px;
  padding:6px;
  font-size:11px;
  font-family:monospace;
  background:#111;
  color:#0f0;
  overflow-y:auto;
  white-space:pre-wrap;
}
</style>
</head>
<body>
<div id="wrap">
<h1>DEPROS GUIADOR ESP32</h1>

<div class="status">
  <div>Posición: <span id="pos">0</span> °</div>
  <div>Target: <span id="tgt">0</span> °</div>
  <div>Corriente: <span id="cur">0</span> A</div>
  <div>Estado: <span id="flt">OK</span></div>
  <div>Modo: <span id="modeLabel">AUTO</span>
    <span id="modeBadge" class="badge badge-ok">AUTO</span>
  </div>
  <div>Válvula: <span id="valveLabel">OFF</span></div>
</div>

<div class="grid">
  <div class="col">
    <h2>Gráfico en vivo</h2>
    <canvas id="plot"></canvas>

    <h2>Mantenimiento / Control manual</h2>
    <div class="switch-row">
      <input type="checkbox" id="manual_mode" onchange="onManualModeChange(this)">
      <label for="manual_mode">Modo manual</label>
    </div>

    <div class="flex-row">
      <button type="button" onclick="sendManualCmd('left')">◀ Izquierda</button>
      <button type="button" onclick="sendManualCmd('stop')">■ Stop</button>
      <button type="button" onclick="sendManualCmd('right')">Derecha ▶</button>
    </div>

    <h3>Entradas</h3>
    <div id="io_status">
      <span id="io_optL" class="io-pill io-off">Opt L</span>
      <span id="io_optR" class="io-pill io-off">Opt R</span>
      <span id="io_limit" class="io-pill io-off">Limit Mag</span>
      <span id="io_button" class="io-pill io-off">Botón</span>
    </div>

    <h3>Válvula / Relé de salida</h3>
    <div class="flex-row">
      <button type="button" onclick="toggleValve()">Toggle válvula</button>
    </div>

    <h2>Log de eventos</h2>
    <div id="logBox"></div>
    <button type="button" onclick="clearLog()">Limpiar log</button>

  </div>

  <div class="col">
    <h2>Configuración</h2>
    <form id="cfgForm">
      <label>Home (°)
        <input name="home_position_deg" id="home_position_deg"/>
      </label>
      <label>Offset máximo (°)
        <input name="edge_max_deg" id="edge_max_deg"/>
      </label>
      <label>K_EDGE (°/step)
        <input name="k_edge_deg_per_step" id="k_edge_deg_per_step"/>
      </label>
      <label>Periodo control borde (ms)
        <input name="edge_control_period_ms" id="edge_control_period_ms"/>
      </label>
      <label>Debounce (ms)
        <input name="edge_debounce_ms" id="edge_debounce_ms"/>
      </label>
      <label>No paper timeout (ms)
        <input name="no_paper_timeout_ms" id="no_paper_timeout_ms"/>
      </label>
      <label>Saturación timeout (ms)
        <input name="edge_saturation_timeout_ms" id="edge_saturation_timeout_ms"/>
      </label>

      <h3>Manual</h3>
      <label>Escala velocidad manual (0-1)
        <input name="manual_speed_scale" id="manual_speed_scale"/>
      </label>

      <h3>PID</h3>
      <label>Kp <input name="pid_kp" id="pid_kp"/></label>
      <label>Ki <input name="pid_ki" id="pid_ki"/></label>
      <label>Kd <input name="pid_kd" id="pid_kd"/></label>
      <label>Counts por grado <input name="counts_per_degree" id="counts_per_degree"/></label>

      <h3>Corriente</h3>
      <label>Soft limit (A) <input name="soft_current_limitA" id="soft_current_limitA"/></label>
      <label>Hard limit (A) <input name="hard_current_limitA" id="hard_current_limitA"/></label>
      <label>ADC offset <input name="current_adc_offset" id="current_adc_offset"/></label>
      <label>ADC scale (A/count) <input name="current_adc_scale" id="current_adc_scale"/></label>

      <button type="button" onclick="sendConfig()">Guardar config</button>
    </form>
  </div>
</div>

</div>
<script>
let posEl          = document.getElementById('pos');
let tgtEl          = document.getElementById('tgt');
let curEl          = document.getElementById('cur');
let fltEl          = document.getElementById('flt');
let modeLabel      = document.getElementById('modeLabel');
let modeBadge      = document.getElementById('modeBadge');
let valveLabel     = document.getElementById('valveLabel');
let manualCheckbox = document.getElementById('manual_mode');

let ioOptL   = document.getElementById('io_optL');
let ioOptR   = document.getElementById('io_optR');
let ioLimit  = document.getElementById('io_limit');
let ioButton = document.getElementById('io_button');

let logBox   = document.getElementById('logBox');

let valveState = 0;

const MAX_POINTS=300;
let dataTime=[], dataPos=[], dataTgt=[];
let t0=null;

// -----------------------
// CONFIG
// -----------------------
function fetchConfig(){
  fetch('/config')
    .then(r=>r.json())
    .then(cfg=>{
      for(let k in cfg){
        let el=document.getElementById(k);
        if(el) el.value=cfg[k];
      }
    })
    .catch(e=>console.log('Config error',e));
}

function sendConfig(){
  let form=document.getElementById('cfgForm');
  let params=new URLSearchParams();
  for(let i=0;i<form.elements.length;i++){
    let e=form.elements[i];
    if(e.name) params.append(e.name, e.value);
  }
  fetch('/config',{method:'POST',body:params})
    .then(r=>r.text())
    .then(t=>console.log('cfg resp',t))
    .catch(e=>console.log('cfg post error',e));
}

// -----------------------
// STATUS / PLOT
// -----------------------
function fetchStatus(){
  fetch('/status')
    .then(r=>r.json())
    .then(st=>{
      posEl.textContent = (+st.pos).toFixed(2);
      tgtEl.textContent = (+st.tgt).toFixed(2);
      curEl.textContent = (+st.cur).toFixed(2);
      fltEl.textContent = st.flt;

      manualCheckbox.checked = (st.manual === 1);
      valveState = st.valve ? 1 : 0;
      valveLabel.textContent = valveState ? 'ON' : 'OFF';

      if(st.manual === 1){
        modeLabel.textContent='MANUAL';
        modeBadge.textContent='MANUAL';
        modeBadge.className='badge badge-warn';
      } else {
        modeLabel.textContent='AUTO';
        modeBadge.textContent='AUTO';
        modeBadge.className='badge badge-ok';
      }

      let t = st.t;
      if(t0 === null) t0 = t;
      let tt = (t - t0)/1000.0;

      dataTime.push(tt);
      dataPos.push(st.pos);
      dataTgt.push(st.tgt);
      if(dataTime.length>MAX_POINTS){
        dataTime.shift(); dataPos.shift(); dataTgt.shift();
      }
      drawPlot();
    })
    .catch(e=>console.log('Status error',e));
}

function drawPlot(){
  let canvas=document.getElementById('plot');
  let ctx=canvas.getContext('2d');
  let w=canvas.width, h=canvas.height;
  ctx.clearRect(0,0,w,h);
  if(dataTime.length<2) return;

  let tmin=dataTime[0], tmax=dataTime[dataTime.length-1];
  let ymin=Math.min(...dataPos, ...dataTgt);
  let ymax=Math.max(...dataPos, ...dataTgt);
  if(ymax-ymin<1){ ymax+=0.5; ymin-=0.5; }

  function tx(t){ return 10+(t-tmin)/(tmax-tmin)*(w-20); }
  function ty(y){ return h-10-(y-ymin)/(ymax-ymin)*(h-20); }

  ctx.strokeStyle='#ccc';
  ctx.beginPath();
  ctx.moveTo(10,10); ctx.lineTo(10,h-10); ctx.lineTo(w-10,h-10);
  ctx.stroke();

  ctx.strokeStyle='red';
  ctx.beginPath();
  ctx.moveTo(tx(dataTime[0]), ty(dataTgt[0]));
  for(let i=1;i<dataTime.length;i++) ctx.lineTo(tx(dataTime[i]), ty(dataTgt[i]));
  ctx.stroke();

  ctx.strokeStyle='blue';
  ctx.beginPath();
  ctx.moveTo(tx(dataTime[0]), ty(dataPos[0]));
  for(let i=1;i<dataTime.length;i++) ctx.lineTo(tx(dataTime[i]), ty(dataPos[i]));
  ctx.stroke();
}

// -----------------------
// IO STATUS / VÁLVULA
// -----------------------
function setIoPill(elem, on){
  if(on){
    elem.classList.remove('io-off');
    elem.classList.add('io-on');
  } else {
    elem.classList.remove('io-on');
    elem.classList.add('io-off');
  }
}

function fetchIOStatus(){
  fetch('/io_status')
    .then(r=>r.json())
    .then(io=>{
      setIoPill(ioOptL,  io.optL);
      setIoPill(ioOptR,  io.optR);
      setIoPill(ioLimit, io.limitMag);
      setIoPill(ioButton,io.button);
      valveState = io.valve ? 1 : 0;
      valveLabel.textContent = valveState ? 'ON' : 'OFF';
      manualCheckbox.checked = (io.manual === 1);
    })
    .catch(e=>console.log('io_status error',e));
}

function toggleValve(){
  let newState = valveState ? 0 : 1;
  let params=new URLSearchParams();
  params.append('state', newState.toString());
  fetch('/valve',{method:'POST',body:params})
    .then(r=>r.text())
    .then(t=>{
      console.log('valve resp',t);
      valveState = newState;
      valveLabel.textContent = valveState ? 'ON' : 'OFF';
    })
    .catch(e=>console.log('valve error',e));
}

// -----------------------
// MANUAL MODE / CMD
// -----------------------
function onManualModeChange(chk){
  let mode = chk.checked ? '1' : '0';
  let params=new URLSearchParams();
  params.append('mode', mode);
  fetch('/manual',{method:'POST',body:params})
    .then(r=>r.text())
    .then(t=>console.log('manual mode resp',t))
    .catch(e=>console.log('manual mode error',e));
}

function sendManualCmd(cmd){
  let params=new URLSearchParams();
  params.append('cmd', cmd);
  fetch('/manual',{method:'POST',body:params})
    .then(r=>r.text())
    .then(t=>console.log('manual cmd resp',t))
    .catch(e=>console.log('manual cmd error',e));
}

// -----------------------
// LOG
// -----------------------
function fetchLog(){
  fetch('/log')
    .then(r=>r.json())
    .then(data=>{
      if(!data || !data.log) return;
      logBox.textContent = data.log.join('\n');
      logBox.scrollTop = logBox.scrollHeight;
    })
    .catch(e=>console.log('log error',e));
}

function clearLog(){
  fetch('/clear_log',{method:'POST'})
    .then(r=>r.text())
    .then(t=>{
      console.log('clear_log resp',t);
      logBox.textContent="";
    })
    .catch(e=>console.log('clear_log error',e));
}

// -----------------------
// ON LOAD
// -----------------------
window.onload=function(){
  fetchConfig();
  setInterval(fetchStatus,   200);
  setInterval(fetchIOStatus, 300);
  setInterval(fetchLog,     1000);
};
</script>
</body>
</html>
)HTML";

// --------- HELPERS PARAMS ---------

static float getArgFloat(const String &name, float currentVal) {
  if (!server.hasArg(name)) return currentVal;
  return server.arg(name).toFloat();
}
static uint32_t getArgU32(const String &name, uint32_t currentVal) {
  if (!server.hasArg(name)) return currentVal;
  return (uint32_t)server.arg(name).toInt();
}
static uint16_t getArgU16(const String &name, uint16_t currentVal) {
  if (!server.hasArg(name)) return currentVal;
  return (uint16_t)server.arg(name).toInt();
}

// --------- HANDLERS ---------

static void handleRoot() {
  server.send_P(200, "text/html", MAIN_PAGE);
}

static void handleStatus() {
  String json = "{";
  json += "\"t\":"   + String(millis()) + ",";
  json += "\"pos\":" + String(getServoPositionDeg(),3) + ",";
  json += "\"tgt\":" + String(getServoTargetDeg(),3) + ",";
  json += "\"cur\":" + String(getCurrentA(),3) + ",";
  json += "\"flt\":\"" + getFaultString() + "\",";
  json += "\"manual\":" + String(getManualMode() ? 1 : 0) + ",";
  json += "\"valve\":"  + String(getValveOutput() ? 1 : 0);
  json += "}";
  server.send(200, "application/json", json);
}

static void handleConfigGet() {
  String json = "{";
  json += "\"home_position_deg\":"          + String(gConfig.home_position_deg)          + ",";
  json += "\"edge_max_deg\":"               + String(gConfig.edge_max_deg)               + ",";
  json += "\"k_edge_deg_per_step\":"        + String(gConfig.k_edge_deg_per_step)        + ",";
  json += "\"edge_control_period_ms\":"     + String(gConfig.edge_control_period_ms)     + ",";
  json += "\"edge_debounce_ms\":"           + String(gConfig.edge_debounce_ms)           + ",";
  json += "\"no_paper_timeout_ms\":"        + String(gConfig.no_paper_timeout_ms)        + ",";
  json += "\"edge_saturation_timeout_ms\":" + String(gConfig.edge_saturation_timeout_ms) + ",";
  json += "\"manual_speed_scale\":"         + String(gConfig.manual_speed_scale)         + ",";
  json += "\"pid_kp\":"                     + String(gConfig.pid_kp)                     + ",";
  json += "\"pid_ki\":"                     + String(gConfig.pid_ki)                     + ",";
  json += "\"pid_kd\":"                     + String(gConfig.pid_kd)                     + ",";
  json += "\"counts_per_degree\":"          + String(gConfig.counts_per_degree)          + ",";
  json += "\"soft_current_limitA\":"        + String(gConfig.soft_current_limitA)        + ",";
  json += "\"hard_current_limitA\":"        + String(gConfig.hard_current_limitA)        + ",";
  json += "\"current_adc_offset\":"         + String(gConfig.current_adc_offset)         + ",";
  json += "\"current_adc_scale\":"          + String(gConfig.current_adc_scale);
  json += "}";
  server.send(200, "application/json", json);
}

static void handleConfigPost() {
  gConfig.home_position_deg          = getArgFloat("home_position_deg",          gConfig.home_position_deg);
  gConfig.edge_max_deg               = getArgFloat("edge_max_deg",               gConfig.edge_max_deg);
  gConfig.k_edge_deg_per_step        = getArgFloat("k_edge_deg_per_step",        gConfig.k_edge_deg_per_step);
  gConfig.edge_control_period_ms     = getArgU32 ("edge_control_period_ms",      gConfig.edge_control_period_ms);
  gConfig.edge_debounce_ms           = getArgU32 ("edge_debounce_ms",            gConfig.edge_debounce_ms);
  gConfig.no_paper_timeout_ms        = getArgU32 ("no_paper_timeout_ms",         gConfig.no_paper_timeout_ms);
  gConfig.edge_saturation_timeout_ms = getArgU32 ("edge_saturation_timeout_ms",  gConfig.edge_saturation_timeout_ms);
  gConfig.manual_speed_scale         = getArgFloat("manual_speed_scale",         gConfig.manual_speed_scale);

  gConfig.pid_kp                     = getArgFloat("pid_kp",                     gConfig.pid_kp);
  gConfig.pid_ki                     = getArgFloat("pid_ki",                     gConfig.pid_ki);
  gConfig.pid_kd                     = getArgFloat("pid_kd",                     gConfig.pid_kd);
  gConfig.counts_per_degree          = getArgFloat("counts_per_degree",          gConfig.counts_per_degree);

  gConfig.soft_current_limitA        = getArgFloat("soft_current_limitA",        gConfig.soft_current_limitA);
  gConfig.hard_current_limitA        = getArgFloat("hard_current_limitA",        gConfig.hard_current_limitA);
  gConfig.current_adc_offset         = getArgU16 ("current_adc_offset",          gConfig.current_adc_offset);
  gConfig.current_adc_scale          = getArgFloat("current_adc_scale",          gConfig.current_adc_scale);

  saveConfig();
  logEvent("Config guardada via /config");
  server.send(200, "text/plain", "OK");
}

static void handleIOStatus() {
  bool optL, optR, limitMag, button;
  getInputsStatus(optL, optR, limitMag, button);
  bool valve  = getValveOutput();
  bool manual = getManualMode();

  String json = "{";
  json += "\"optL\":"     + String(optL ? 1 : 0) + ",";
  json += "\"optR\":"     + String(optR ? 1 : 0) + ",";
  json += "\"limitMag\":" + String(limitMag ? 1 : 0) + ",";
  json += "\"button\":"   + String(button ? 1 : 0) + ",";
  json += "\"valve\":"    + String(valve ? 1 : 0) + ",";
  json += "\"manual\":"   + String(manual ? 1 : 0);
  json += "}";
  server.send(200, "application/json", json);
}

static void handleManual() {
  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    bool enabled = (m == "1" || m == "true" || m == "on");
    setManualMode(enabled);
    logEvent(String("Manual MODE via /manual = ") + (enabled ? "ON" : "OFF"));
  }

  if (server.hasArg("cmd")) {
    String c = server.arg("cmd");
    float v = 0.0f;
    if (c == "left")      v = -1.0f;
    else if (c == "right")v =  1.0f;
    else if (c == "stop") v =  0.0f;
    else                  v = c.toFloat();
    setManualCommand(v);
    logEvent("Manual CMD via /manual: " + c + " (v=" + String(v,3) + ")");
  }

  server.send(200, "text/plain", "OK");
}

static void handleValve() {
  if (server.hasArg("state")) {
    String s = server.arg("state");
    bool on = (s == "1" || s == "true" || s == "on");
    setValveOutput(on);
    logEvent(String("Valve via /valve = ") + (on ? "ON" : "OFF"));
  }
  server.send(200, "text/plain", "OK");
}

static void handleLog() {
  String arr;
  getLogJson(arr);
  String json = "{\"log\":" + arr + "}";
  server.send(200, "application/json", json);
}

static void handleClearLog() {
  clearLog();
  logEvent("LOG limpiado via /clear_log");
  server.send(200, "text/plain", "OK");
}

static void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// --------- INICIALIZACIÓN WIFI + WEB ---------

void initWiFiAndWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(ip);
  logEvent("WiFi AP iniciado, IP=" + ip.toString());

  server.on("/",           HTTP_GET,  handleRoot);
  server.on("/status",     HTTP_GET,  handleStatus);
  server.on("/config",     HTTP_GET,  handleConfigGet);
  server.on("/config",     HTTP_POST, handleConfigPost);
  server.on("/io_status",  HTTP_GET,  handleIOStatus);
  server.on("/manual",     HTTP_POST, handleManual);
  server.on("/valve",      HTTP_POST, handleValve);
  server.on("/log",        HTTP_GET,  handleLog);
  server.on("/clear_log",  HTTP_POST, handleClearLog);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
  logEvent("HTTP server started");
}

void webLoop() {
  server.handleClient();
}
