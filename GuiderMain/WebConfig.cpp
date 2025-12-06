// File: WebConfig.cpp
#include "WebConfig.h"

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "Config.h"
#include "Control.h"

static const char *AP_SSID = "DEPROS-GUIDER";
static const char *AP_PASS = "depros1234";

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

// Página HTML (igual a la que te generé antes)…
static const char MAIN_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<title>DEPROS Guider Live</title>
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
button{margin-top:10px;padding:8px 16px;background:#0077cc;color:white;border:none;
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
        <input name="home_position_deg" id="home_position_deg" />
      </label>
      <label>Offset máximo (°)
        <input name="edge_max_deg" id="edge_max_deg" />
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
let ws;
let posEl = document.getElementById('pos');
let tgtEl = document.getElementById('tgt');
let curEl = document.getElementById('cur');
let fltEl = document.getElementById('flt');

const MAX_POINTS = 300;
let dataPos = [];
let dataTgt = [];
let dataTime = [];
let t0 = null;

function connectWS(){
  let proto = (location.protocol === 'https:') ? 'wss://' : 'ws://';
  ws = new WebSocket(proto + location.host + '/ws');
  ws.onopen = () => console.log('WS conectado');
  ws.onclose = () => setTimeout(connectWS, 1000);
  ws.onmessage = (ev) => {
    let j = JSON.parse(ev.data);
    if(j.type === 'status'){
      posEl.textContent = j.pos.toFixed(2);
      tgtEl.textContent = j.tgt.toFixed(2);
      curEl.textContent = j.cur.toFixed(2);
      fltEl.textContent = j.flt;

      if(t0 === null) t0 = j.t;
      let tt = j.t - t0;

      dataTime.push(tt);
      dataPos.push(j.pos);
      dataTgt.push(j.tgt);

      if(dataTime.length > MAX_POINTS){
        dataTime.shift();
        dataPos.shift();
        dataTgt.shift();
      }
      drawPlot();
    } else if(j.type === 'config'){
      for(let k in j.cfg){
        let el = document.getElementById(k);
        if(el) el.value = j.cfg[k];
      }
    }
  };
}

function drawPlot(){
  let canvas = document.getElementById('plot');
  let ctx = canvas.getContext('2d');
  let w = canvas.width;
  let h = canvas.height;

  ctx.clearRect(0,0,w,h);
  if(dataTime.length < 2) return;

  let tmin = dataTime[0];
  let tmax = dataTime[dataTime.length-1];
  let ymin = Math.min(...dataPos, ...dataTgt);
  let ymax = Math.max(...dataPos, ...dataTgt);
  if(ymax - ymin < 1){ ymax += 0.5; ymin -= 0.5; }

  function tx(t){ return (t - tmin)/(tmax - tmin) * (w-20) + 10; }
  function ty(y){ return h - ((y - ymin)/(ymax - ymin)*(h-20)+10); }

  ctx.strokeStyle = '#ccc';
  ctx.beginPath();
  ctx.moveTo(10,10);
  ctx.lineTo(10,h-10);
  ctx.lineTo(w-10,h-10);
  ctx.stroke();

  ctx.strokeStyle = 'red';
  ctx.beginPath();
  ctx.moveTo(tx(dataTime[0]), ty(dataTgt[0]));
  for(let i=1;i<dataTime.length;i++) ctx.lineTo(tx(dataTime[i]), ty(dataTgt[i]));
  ctx.stroke();

  ctx.strokeStyle = 'blue';
  ctx.beginPath();
  ctx.moveTo(tx(dataTime[0]), ty(dataPos[0]));
  for(let i=1;i<dataTime.length;i++) ctx.lineTo(tx(dataTime[i]), ty(dataPos[i]));
  ctx.stroke();
}

function sendConfig(){
  if(!ws || ws.readyState !== WebSocket.OPEN) return alert("WS desconectado");

  let form = document.getElementById("cfgForm");
  let data = {};
  for(let i=0; i<form.elements.length; i++){
    let e = form.elements[i];
    if(e.name) data[e.name] = e.value;
  }
  ws.send(JSON.stringify({type:"set_config", cfg:data}));
}

connectWS();
</script>
</body>
</html>
)HTML";

// WS events
static void wsEvent(AsyncWebSocket *server,
                    AsyncWebSocketClient *client,
                    AwsEventType type,
                    void *arg,
                    uint8_t *data,
                    size_t len);

// Task de broadcast
static void broadcastTask(void *pv);

void initWiFiAndWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  ws.onEvent(wsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send_P(200, "text/html", MAIN_PAGE);
  });

  server.begin();

  xTaskCreatePinnedToCore(broadcastTask, "WSBroadcast", 4096, NULL, 1, NULL, 1);
}

void webLoop() {
  ws.cleanupClients();
}

static void wsEvent(AsyncWebSocket *server,
                    AsyncWebSocketClient *client,
                    AwsEventType type,
                    void *arg,
                    uint8_t *data,
                    size_t len)
{
  if(type == WS_EVT_CONNECT){
    String msg = "{\"type\":\"config\",\"cfg\":{";
    msg += "\"home_position_deg\":" + String(gConfig.home_position_deg) + ",";
    msg += "\"edge_max_deg\":" + String(gConfig.edge_max_deg) + ",";
    msg += "\"k_edge_deg_per_step\":" + String(gConfig.k_edge_deg_per_step) + ",";
    msg += "\"edge_control_period_ms\":" + String(gConfig.edge_control_period_ms) + ",";
    msg += "\"edge_debounce_ms\":" + String(gConfig.edge_debounce_ms) + ",";
    msg += "\"no_paper_timeout_ms\":" + String(gConfig.no_paper_timeout_ms) + ",";
    msg += "\"edge_saturation_timeout_ms\":" + String(gConfig.edge_saturation_timeout_ms) + ",";
    msg += "\"pid_kp\":" + String(gConfig.pid_kp) + ",";
    msg += "\"pid_ki\":" + String(gConfig.pid_ki) + ",";
    msg += "\"pid_kd\":" + String(gConfig.pid_kd) + ",";
    msg += "\"counts_per_degree\":" + String(gConfig.counts_per_degree) + ",";
    msg += "\"soft_current_limitA\":" + String(gConfig.soft_current_limitA) + ",";
    msg += "\"hard_current_limitA\":" + String(gConfig.hard_current_limitA) + ",";
    msg += "\"current_adc_offset\":" + String(gConfig.current_adc_offset) + ",";
    msg += "\"current_adc_scale\":" + String(gConfig.current_adc_scale);
    msg += "}}";
    client->text(msg);
  }
  else if(type == WS_EVT_DATA){
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT){
      String s = "";
      s.reserve(len);
      for(size_t i=0;i<len;i++) s += (char)data[i];

      if(s.indexOf("\"type\":\"set_config\"") >= 0){
        auto getVal = [&](const char* key, String cur)->String {
          int p = s.indexOf(String("\"") + key + "\":");
          if(p<0) return cur;
          p += strlen(key)+3;
          int e = s.indexOf(",", p);
          if(e<0) e = s.indexOf("}", p);
          return s.substring(p,e);
        };

        gConfig.home_position_deg = getVal("home_position_deg", String(gConfig.home_position_deg)).toFloat();
        gConfig.edge_max_deg = getVal("edge_max_deg", String(gConfig.edge_max_deg)).toFloat();
        gConfig.k_edge_deg_per_step = getVal("k_edge_deg_per_step", String(gConfig.k_edge_deg_per_step)).toFloat();

        gConfig.edge_control_period_ms = getVal("edge_control_period_ms", String(gConfig.edge_control_period_ms)).toInt();
        gConfig.edge_debounce_ms = getVal("edge_debounce_ms", String(gConfig.edge_debounce_ms)).toInt();
        gConfig.no_paper_timeout_ms = getVal("no_paper_timeout_ms", String(gConfig.no_paper_timeout_ms)).toInt();
        gConfig.edge_saturation_timeout_ms = getVal("edge_saturation_timeout_ms", String(gConfig.edge_saturation_timeout_ms)).toInt();

        gConfig.pid_kp = getVal("pid_kp", String(gConfig.pid_kp)).toFloat();
        gConfig.pid_ki = getVal("pid_ki", String(gConfig.pid_ki)).toFloat();
        gConfig.pid_kd = getVal("pid_kd", String(gConfig.pid_kd)).toFloat();
        gConfig.counts_per_degree = getVal("counts_per_degree", String(gConfig.counts_per_degree)).toFloat();

        gConfig.soft_current_limitA = getVal("soft_current_limitA", String(gConfig.soft_current_limitA)).toFloat();
        gConfig.hard_current_limitA = getVal("hard_current_limitA", String(gConfig.hard_current_limitA)).toFloat();
        gConfig.current_adc_offset = getVal("current_adc_offset", String(gConfig.current_adc_offset)).toInt();
        gConfig.current_adc_scale = getVal("current_adc_scale", String(gConfig.current_adc_scale)).toFloat();

        saveConfig();
      }
    }
  }
}

static void broadcastTask(void *pv) {
  TickType_t last = xTaskGetTickCount();
  TickType_t dt = pdMS_TO_TICKS(100);

  while(1) {
    float pos = getServoPositionDeg();
    float tgt = getServoTargetDeg();
    float cur = getCurrentA();
    String flt = getFaultString();
    uint32_t t = millis();

    String msg = "{";
    msg += "\"type\":\"status\",";
    msg += "\"t\":" + String(t) + ",";
    msg += "\"pos\":" + String(pos,3) + ",";
    msg += "\"tgt\":" + String(tgt,3) + ",";
    msg += "\"cur\":" + String(cur,3) + ",";
    msg += "\"flt\":\"" + flt + "\"";
    msg += "}";

    ws.textAll(msg);

    vTaskDelayUntil(&last, dt);
  }
}
