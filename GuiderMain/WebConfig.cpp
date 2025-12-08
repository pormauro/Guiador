// File: WebConfig.cpp
#include "WebConfig.h"
#include <WiFi.h>
#include <WebServer.h>

#include "Config.h"
#include "Control.h"

static const char *AP_SSID = "DEPROS-GUIDER";
static const char *AP_PASS = "depros1234";

static WebServer server(80);

// HTML principal en PROGMEM
static const char MAIN_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<title>DEPROS Guider ESP32</title>
<style>
body{font-family:Arial;margin:0;background:#f4f4f4;}
#wrap{max-width:1000px;margin:10px auto;background:#fff;border-radius:8px;
      box-shadow:0 0 10px rgba(0,0,0,0.15);padding:16px;}
h1{margin-top:0;}
.status{background:#eee;padding:8px;border-radius:6px;font-size:14px;margin-bottom:10px;}
.grid{display:flex;gap:16px;flex-wrap:wrap;}
.col{flex:1;min-width:280px;}
label{display:block;font-size:12px;margin-top:6px;}
input{width:100%;padding:4px;margin-top:2px;font-size:12px;}
button{margin-top:10px;padding:8px 16px;background:#0077cc;color:#fff;border:none;
       border-radius:4px;cursor:pointer;font-size:13px;}
button:hover{background:#005fa3;}
canvas{width:100%;height:250px;border:1px solid #ccc;border-radius:4px;}
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
</div>

<div class="grid">
  <div class="col">
    <h2>Gráfico en vivo</h2>
    <canvas id="plot"></canvas>
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

      <button type="button" onclick="sendConfig()">Guardar</button>
    </form>
  </div>
</div>

</div>
<script>
let posEl=document.getElementById('pos');
let tgtEl=document.getElementById('tgt');
let curEl=document.getElementById('cur');
let fltEl=document.getElementById('flt');

const MAX_POINTS=300;
let dataTime=[], dataPos=[], dataTgt=[];
let t0=null;

function fetchConfig(){
  fetch('/config').then(r=>r.json()).then(cfg=>{
    for(let k in cfg){
      let el=document.getElementById(k);
      if(el) el.value=cfg[k];
    }
  }).catch(e=>console.log(e));
}

function fetchStatus(){
  fetch('/status').then(r=>r.json()).then(st=>{
    posEl.textContent=st.pos.toFixed(2);
    tgtEl.textContent=st.tgt.toFixed(2);
    curEl.textContent=st.cur.toFixed(2);
    fltEl.textContent=st.flt;

    let t=st.t;
    if(t0===null) t0=t;
    let tt=(t-t0)/1000.0;

    dataTime.push(tt);
    dataPos.push(st.pos);
    dataTgt.push(st.tgt);
    if(dataTime.length>MAX_POINTS){
      dataTime.shift(); dataPos.shift(); dataTgt.shift();
    }
    drawPlot();
  }).catch(e=>console.log(e));
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

function sendConfig(){
  let form=document.getElementById('cfgForm');
  let params=new URLSearchParams();
  for(let i=0;i<form.elements.length;i++){
    let e=form.elements[i];
    if(e.name) params.append(e.name, e.value);
  }
  fetch('/setConfig',{method:'POST',body:params})
    .then(r=>r.text()).then(t=>console.log('cfg resp',t))
    .catch(e=>console.log(e));
}

window.onload=function(){
  fetchConfig();
  setInterval(fetchStatus,200);
};
</script>
</body>
</html>
)HTML";

// helpers de parseo
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

// handlers
static void handleRoot() {
  server.send_P(200, "text/html", MAIN_PAGE);
}

static void handleStatus() {
  String json = "{";
  json += "\"t\":"   + String(millis()) + ",";
  json += "\"pos\":" + String(getServoPositionDeg(),3) + ",";
  json += "\"tgt\":" + String(getServoTargetDeg(),3) + ",";
  json += "\"cur\":" + String(getCurrentA(),3) + ",";
  json += "\"flt\":\"" + getFaultString() + "\"";
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

  gConfig.pid_kp                     = getArgFloat("pid_kp",                     gConfig.pid_kp);
  gConfig.pid_ki                     = getArgFloat("pid_ki",                     gConfig.pid_ki);
  gConfig.pid_kd                     = getArgFloat("pid_kd",                     gConfig.pid_kd);
  gConfig.counts_per_degree          = getArgFloat("counts_per_degree",          gConfig.counts_per_degree);

  gConfig.soft_current_limitA        = getArgFloat("soft_current_limitA",        gConfig.soft_current_limitA);
  gConfig.hard_current_limitA        = getArgFloat("hard_current_limitA",        gConfig.hard_current_limitA);
  gConfig.current_adc_offset         = getArgU16 ("current_adc_offset",          gConfig.current_adc_offset);
  gConfig.current_adc_scale          = getArgFloat("current_adc_scale",          gConfig.current_adc_scale);

  saveConfig();
  server.send(200, "text/plain", "OK");
}

static void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void initWiFiAndWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(ip);

  server.on("/",         HTTP_GET,  handleRoot);
  server.on("/status",   HTTP_GET,  handleStatus);
  server.on("/config",   HTTP_GET,  handleConfigGet);
  server.on("/setConfig",HTTP_POST, handleConfigPost);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void webLoop() {
  server.handleClient();
}
