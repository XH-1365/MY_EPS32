#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>

WebServer server(80);

// 外部变量来自 main.cpp
extern float currentX;
extern float currentY;
extern float currentTheta;
extern bool moving;
// 向 main.c 中请求打印文本的接口
extern void scheduleBraillePrint(const char* utf8text);
// 轴控制接口
extern void scheduleAxisMove(bool isLeftAxis, float pos_mm, int pwm);
extern void stopAll();
// 新增外部接口
extern void resetHome();
extern void setThetaDeg(float deg);
// 右轴反转控制（来自 main.cpp）
extern void toggleInvertRight();
extern bool getInvertRight();
// 校准接口
extern void startCalibration(bool isLeft, float mm, int pwm);
extern float stopCalibration();
// 请求回 home
extern void requestGoHome();

void handleRoot() {
  const char* page = R"rawliteral(
  <!doctype html>
  <html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>Braille Printer</title>
    <style>body{font-family:Arial; padding:10px;} .val{font-weight:bold} textarea{width:100%;height:80px}</style>
  </head>
  <body>
    <h3>Braille Printer - Status & Print</h3>
    <div>X: <span id="x">0</span> mm</div>
    <div>Y: <span id="y">0</span> mm</div>
    <div>Theta: <span id="t">0</span> rad</div>
    <div>Moving: <span id="m">0</span></div>

    <h4>Print Text (ASCII a-z)</h4>
    <textarea id="txt" placeholder="输入要打印的文本（仅 a-z 空格）"></textarea>
    <div style="margin-top:8px">
        <button id="btn">Print</button>
      <span id="status" style="margin-left:10px"></span>
    </div>

    <h4>Axis Control</h4>
    <div>
      X (mm): <input id="xpos" type="number" step="0.1" value="0" />
      <button id="movex">Move X</button>
    </div>
    <div style="margin-top:6px">
      Y (mm): <input id="ypos" type="number" step="0.1" value="0" />
      <button id="movey">Move Y</button>
      <button id="invertY" style="margin-left:8px">Invert Y</button>
      <span id="invertState" style="margin-left:6px"></span>
    </div>
    <h4>Calibration</h4>
    <div>
      Cal mm: <input id="calmm" type="number" step="1" value="100" />
      PWM: <input id="calpwm" type="number" step="1" value="100" />
      <button id="calStartX">Cal X Start</button>
      <button id="calStartY">Cal Y Start</button>
      <button id="calStop">Cal Stop</button>
      <span id="calres" style="margin-left:10px"></span>
    </div>
    <div style="margin-top:6px">
      PWM: <input id="axispwm" type="number" step="1" value="120" />
      <button id="stopbtn" style="margin-left:12px">Stop All</button>
    </div>

    <h4>Quick Jog (step 1 mm)</h4>
    <div style="font-size:18px; margin-top:6px">
      <button id="xminus" style="width:64px">X -</button>
      <button id="xplus" style="width:64px; margin-left:6px">X +</button>
      <button id="yminus" style="width:64px; margin-left:12px">Y -</button>
      <button id="yplus" style="width:64px; margin-left:6px">Y +</button>
      <button id="homebtn" style="width:80px; margin-left:12px">Home</button>
    </div>

      <h4>Controls</h4>
      <div>
        Angle (deg): <input id="angle" type="number" step="0.1" value="0" />
        <button id="setang">Set Angle</button>
      </div>
      <div style="margin-top:6px">
        <button id="resethome">Reset Home</button>
        <span id="hstatus" style="margin-left:10px"></span>
      </div>

    <script>
      function update() {
        fetch('/pos').then(r=>r.json()).then(j=>{
          document.getElementById('x').innerText = j.x.toFixed(2);
          document.getElementById('y').innerText = j.y.toFixed(2);
          document.getElementById('t').innerText = j.theta.toFixed(2);
          document.getElementById('m').innerText = j.moving? '1':'0';
        }).catch(e=>{});
      }
      setInterval(update,500);
      update();

      document.getElementById('btn').addEventListener('click', function(){
        var text = document.getElementById('txt').value || '';
        if (!text) { document.getElementById('status').innerText='empty'; return; }
        document.getElementById('status').innerText='sending...';
        fetch('/printtext?text='+encodeURIComponent(text)).then(r=>r.text()).then(t=>{
          document.getElementById('status').innerText = t;
        }).catch(e=>{ document.getElementById('status').innerText='error'; });
      });
      document.getElementById('movex').addEventListener('click', function(){
        var pos = parseFloat(document.getElementById('xpos').value||'0');
        var pwm = parseInt(document.getElementById('axispwm').value||'120');
        document.getElementById('status').innerText='scheduling X...';
        fetch('/moveaxis?axis=X&pos='+encodeURIComponent(pos)+'&pwm='+encodeURIComponent(pwm)).then(r=>r.text()).then(t=>{ document.getElementById('status').innerText=t; }).catch(e=>{ document.getElementById('status').innerText='error'; });
      });
      document.getElementById('movey').addEventListener('click', function(){
        var pos = parseFloat(document.getElementById('ypos').value||'0');
        var pwm = parseInt(document.getElementById('axispwm').value||'120');
        document.getElementById('status').innerText='scheduling Y...';
        fetch('/moveaxis?axis=Y&pos='+encodeURIComponent(pos)+'&pwm='+encodeURIComponent(pwm)).then(r=>r.text()).then(t=>{ document.getElementById('status').innerText=t; }).catch(e=>{ document.getElementById('status').innerText='error'; });
      });
      // Invert Y handler
      document.getElementById('invertY').addEventListener('click', function(){
        fetch('/invertY?op=toggle').then(r=>r.text()).then(t=>{ document.getElementById('invertState').innerText = t; }).catch(e=>{ document.getElementById('invertState').innerText='err'; });
      });
      // 获取初始反转状态
      fetch('/invertY?op=get').then(r=>r.text()).then(t=>{ document.getElementById('invertState').innerText = t; }).catch(e=>{});
      
      document.getElementById('stopbtn').addEventListener('click', function(){
        fetch('/stop').then(r=>r.text()).then(t=>{ document.getElementById('status').innerText=t; }).catch(e=>{ document.getElementById('status').innerText='error'; });
      });
      document.getElementById('setang').addEventListener('click', function(){
        var a = parseFloat(document.getElementById('angle').value || '0');
        fetch('/settheta?deg='+encodeURIComponent(a)).then(r=>r.text()).then(t=>{
          document.getElementById('hstatus').innerText = t;
        }).catch(e=>{ document.getElementById('hstatus').innerText='error'; });
      });

      document.getElementById('movey').addEventListener('click', function(){
        var pos = parseFloat(document.getElementById('ypos').value||'0');
        var pwm = parseInt(document.getElementById('axispwm').value||'120');
        document.getElementById('status').innerText='scheduling Y...';
        fetch('/moveaxis?axis=Y&pos='+encodeURIComponent(pos)+'&pwm='+encodeURIComponent(pwm)).then(r=>r.text()).then(t=>{ document.getElementById('status').innerText=t; }).catch(e=>{ document.getElementById('status').innerText='error'; });
      });
      // Quick jog handlers (step = 1mm)
      document.getElementById('xminus').addEventListener('click', function(){
        var pwm = parseInt(document.getElementById('axispwm').value||'120');
        fetch('/jog?axis=X&step='+encodeURIComponent(-1)+'&pwm='+encodeURIComponent(pwm)).then(r=>r.text()).then(t=>{ document.getElementById('status').innerText=t; });
      });
      document.getElementById('xplus').addEventListener('click', function(){
        var pwm = parseInt(document.getElementById('axispwm').value||'120');
        fetch('/jog?axis=X&step='+encodeURIComponent(1)+'&pwm='+encodeURIComponent(pwm)).then(r=>r.text()).then(t=>{ document.getElementById('status').innerText=t; });
      });
      document.getElementById('yminus').addEventListener('click', function(){
        var pwm = parseInt(document.getElementById('axispwm').value||'120');
        fetch('/jog?axis=Y&step='+encodeURIComponent(-1)+'&pwm='+encodeURIComponent(pwm)).then(r=>r.text()).then(t=>{ document.getElementById('status').innerText=t; });
      });
      document.getElementById('yplus').addEventListener('click', function(){
        var pwm = parseInt(document.getElementById('axispwm').value||'120');
        fetch('/jog?axis=Y&step='+encodeURIComponent(1)+'&pwm='+encodeURIComponent(pwm)).then(r=>r.text()).then(t=>{ document.getElementById('status').innerText=t; });
      });
      document.getElementById('homebtn').addEventListener('click', function(){
        fetch('/gohome').then(r=>r.text()).then(t=>{ document.getElementById('status').innerText=t; });
      });
      // Calibration handlers
      document.getElementById('calStartX').addEventListener('click', function(){
        var mm = parseFloat(document.getElementById('calmm').value||'100');
        var pwm = parseInt(document.getElementById('calpwm').value||'100');
        fetch('/calib_start?axis=X&mm='+encodeURIComponent(mm)+'&pwm='+encodeURIComponent(pwm)).then(r=>r.text()).then(t=>{ document.getElementById('calres').innerText=t; });
      });
      document.getElementById('calStartY').addEventListener('click', function(){
        var mm = parseFloat(document.getElementById('calmm').value||'100');
        var pwm = parseInt(document.getElementById('calpwm').value||'100');
        fetch('/calib_start?axis=Y&mm='+encodeURIComponent(mm)+'&pwm='+encodeURIComponent(pwm)).then(r=>r.text()).then(t=>{ document.getElementById('calres').innerText=t; });
      });
      document.getElementById('calStop').addEventListener('click', function(){
        fetch('/calib_stop').then(r=>r.text()).then(t=>{ document.getElementById('calres').innerText=t; });
      });
    </script>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", page);
}

// Use AP+STA mode to be more robust, set explicit channel and allow multiple connections
void setupWiFi() {
  WiFi.mode(WIFI_AP_STA);
  const char* ssid = "Braille_ESP";
  const char* pass = "12345678"; // must be >=8 for WPA2
  int channel = 1;
  bool hidden = false;
  int maxConn = 4;
  bool ok = WiFi.softAP(ssid, pass, channel, hidden, maxConn);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("WiFi.softAP returned: %d\n", ok ? 1 : 0);
  Serial.printf("WiFi AP %s started, IP: %s\n", ssid, ip.toString().c_str());
  Serial.printf("AP MAC: %s\n", WiFi.softAPmacAddress().c_str());
  server.on("/", handleRoot);
  server.on("/pos", [](){
    String s = "{";
    s += "\"x\":" + String(currentX, 2) + ",";
    s += "\"y\":" + String(currentY, 2) + ",";
    s += "\"theta\":" + String(currentTheta, 3) + ",";
    s += "\"moving\":" + String(moving?1:0);
    s += "}";
    server.send(200, "application/json", s);
  });
  // 返回已连接客户端数（简单接口）
  server.on("/clients", [](){
    int n = WiFi.softAPgetStationNum();
    String s = "{" + String("\"clients\":") + String(n) + String("}");
    server.send(200, "application/json", s);
  });
  server.on("/printtext", [](){
    if (server.hasArg("text")) {
      String t = server.arg("text");
      scheduleBraillePrint(t.c_str());
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "missing text");
    }
  });
  // 设置角度（度）
  server.on("/settheta", [](){
    if (server.hasArg("deg")) {
      float d = server.arg("deg").toFloat();
      setThetaDeg(d);
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "missing deg");
    }
  });
  // 重置 home
  server.on("/resethome", [](){
    resetHome();
    server.send(200, "text/plain", "OK");
  });
  // 单轴移动请求（不在 handler 中直接移动，而是安排到主循环执行）
  server.on("/moveaxis", [](){
    if (server.hasArg("axis") && server.hasArg("pos")) {
      String a = server.arg("axis");
      float p = server.arg("pos").toFloat();
      int pwm = 120;
      if (server.hasArg("pwm")) pwm = server.arg("pwm").toInt();
      bool isLeft = (a == "X");
      scheduleAxisMove(isLeft, p, pwm);
      server.send(200, "text/plain", "scheduled");
    } else {
      server.send(400, "text/plain", "missing axis or pos");
    }
  });
  // 快捷 Jog 接口：按固定步长移动（服务器端计算目标并安排）
  server.on("/jog", [](){
    if (server.hasArg("axis") && server.hasArg("step")) {
      String a = server.arg("axis");
      float step = server.arg("step").toFloat();
      int pwm = 120;
      if (server.hasArg("pwm")) pwm = server.arg("pwm").toInt();
      bool isLeft = (a == "X");
      float target = isLeft ? (currentX + step) : (currentY + step);
      scheduleAxisMove(isLeft, target, pwm);
      server.send(200, "text/plain", "scheduled_jog");
    } else {
      server.send(400, "text/plain", "missing axis or step");
    }
  });
  // 半自动校准：开始
  server.on("/calib_start", [](){
    if (server.hasArg("axis") && server.hasArg("mm")) {
      String a = server.arg("axis");
      float mm = server.arg("mm").toFloat();
      int pwm = 100;
      if (server.hasArg("pwm")) pwm = server.arg("pwm").toInt();
      bool isLeft = (a == "X");
      startCalibration(isLeft, mm, pwm);
      server.send(200, "text/plain", "calib_started");
    } else {
      server.send(400, "text/plain", "missing axis or mm");
    }
  });
  server.on("/calib_stop", [](){
    float v = stopCalibration();
    if (v > 0.0f) {
      char buf[64];
      sprintf(buf, "counts_per_mm=%.3f", v);
      server.send(200, "text/plain", String(buf));
    } else {
      server.send(400, "text/plain", "not_running");
    }
  });
  // 网页请求去 home（回到坐标 0,0，由主循环执行）
  server.on("/gohome", [](){
    requestGoHome();
    server.send(200, "text/plain", "gohome_requested");
  });
  // 读取或切换右轴反向（op=get|toggle）
  server.on("/invertY", [](){
    String op = "";
    if (server.hasArg("op")) op = server.arg("op");
    if (op == "toggle") {
      toggleInvertRight();
    }
    bool st = getInvertRight();
    server.send(200, "text/plain", st?"ON":"OFF");
  });
  // 紧急停止
  server.on("/stop", [](){
    stopAll();
    server.send(200, "text/plain", "stopped");
  });
  server.begin();
  Serial.println("Web server started");

  // 注册事件，记录客户端连接与断开（便于串口调试）
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t){
    if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
      Serial.printf("Station connected, total=%d\n", WiFi.softAPgetStationNum());
    } else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
      Serial.printf("Station disconnected, total=%d\n", WiFi.softAPgetStationNum());
    }
  });
}

void handleServer() {
  server.handleClient();
}
