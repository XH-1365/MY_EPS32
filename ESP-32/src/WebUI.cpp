#include "WebUI.h"
#include <WiFi.h>
#include <WebServer.h>
#include "Config.h"
#include "Commands.h"
#include "AIClient.h"

static WebServer server(80);
#define WIFI_SSID "111"
#define WIFI_PASS "00000000"

static const char* HTML = R"rawliteral(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Braille XY</title>
<style>
body{font-family:Arial;margin:16px;max-width:720px}
button{padding:10px 14px;margin:6px}
input{padding:10px;width:120px}
#log{white-space:pre-wrap;background:#111;color:#0f0;padding:10px;border-radius:8px;min-height:120px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
.small{opacity:.7;font-size:12px}
</style>
</head><body>
<h2>Braille XY Control</h2>
<div class="row">
  <button onclick="sendCmd('home')">Home</button>
  <button onclick="sendCmd('stop')">Stop</button>
  <button onclick="sendCmd('punch')">Punch</button>
  <button onclick="sendCmd('help')">Help</button>
</div>

<h3>Move</h3>
<div class="row">
  X(mm): <input id="mx" value="10">
  <button onclick="sendCmd('mx '+val('mx'))">Move X</button>
  Y(mm): <input id="my" value="10">
  <button onclick="sendCmd('my '+val('my'))">Move Y</button>
</div>
<div class="row">
  dX: <input id="dx" value="10">
  dY: <input id="dy" value="10">
  <button onclick="sendCmd('mxy '+val('dx')+' '+val('dy'))">Move XY</button>
</div>

<h3>Braille Print</h3>
<div class="row">
  <input id="txt" style="width:360px" value="hello world">
  <button onclick="sendCmd('print '+val('txt'))">Print</button>
</div>

<h3>Status</h3>
<div class="row">
  <button onclick="refreshPos()">Refresh</button>
  <span class="small" id="pos">-</span>
</div>

<h3>Log</h3>
<div id="log"></div>

<script>
function val(id){ return document.getElementById(id).value; }
function log(t){
  const el=document.getElementById('log');
  el.textContent = (new Date().toLocaleTimeString())+"  "+t+"\n"+el.textContent;
}
async function sendCmd(c){
  log(">> "+c);
  const r=await fetch("/cmd?c="+encodeURIComponent(c));
  const t=await r.text();
  log(t);
  refreshPos();
}
async function refreshPos(){
  const r=await fetch("/pos");
  const j=await r.json();
  document.getElementById('pos').textContent =
    `X=${j.xmm.toFixed(3)}mm (enc ${j.encX}) | Y=${j.ymm.toFixed(3)}mm (enc ${j.encY}) | printing=${j.printing} ${j.dotI}/${j.dotN}`;
}
setInterval(refreshPos, 800);
</script>
</body></html>

<script>
async function askAI(){
  const text = document.getElementById('aiText').value;
  const r = await fetch("/ai?text=" + encodeURIComponent(text));
  const j = await r.json();
  document.getElementById('aiReply').textContent = j.reply || JSON.stringify(j);
}
</script>

<h3>AI Assistant</h3>
<div class="row">
  <input id="aiText" style="width:360px" value="你好">
  <button onclick="askAI()">Ask AI</button>
</div>
<div id="aiReply" style="white-space:pre-wrap;border:1px solid #ccc;padding:10px;margin-top:10px;"></div>

)rawliteral";

static void handleRoot(){ server.send(200,"text/html",HTML); }
static void handleCmd(){
  String c = server.hasArg("c") ? server.arg("c") : "";
  server.send(200,"text/plain", execCommand(c));
}
static void handlePos(){ server.send(200,"application/json", posJson()); }

static void handleAI() {
  String text = server.hasArg("text") ? server.arg("text") : "";
  String result = askAI(text);
  server.send(200, "application/json", result);
}
void webInit(){

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED)
  {
      delay(500);
      Serial.print(".");
  }

  server.on("/ai", handleAI);
  Serial.println();
  Serial.println("WiFi connected!");

  IPAddress ip = WiFi.localIP();
  Serial.print("IP address: ");
  Serial.println(ip);
  
  // Serial.printf("AP: %s  PASS: %s\n", AP_SSID, AP_PASS);
  // Serial.print("Open: http://"); Serial.println(ip);

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/pos", handlePos);
  server.begin();
}
void webUpdate(){
  server.handleClient();
}