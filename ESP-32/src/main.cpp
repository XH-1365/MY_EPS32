#include "Config.h"
#include "Motion.h"
#include "ServoPunch.h"
#include "Braille.h"
#include "WebUI.h"
#include "Commands.h"
#include "OledUI.h"

static String uartLine;

void setup(){
  Serial.begin(115200);
  delay(200);

  motionInit();
  servoInit();
  webInit();
  oledInit();

  Serial.println("READY. type 'help' in Serial.");
}

void loop(){
  motionUpdate();
  brailleProcess();
  webUpdate();
  oledUpdate();

  // 串口命令
  while(Serial.available()){
    char c=(char)Serial.read();
    if(c=='\n'||c=='\r'){
      uartLine.trim();
      if(uartLine.length()){
        Serial.print(">> "); Serial.println(uartLine);
        Serial.println(execCommand(uartLine));
      }
      uartLine="";
    }else{
      uartLine+=c;
      if(uartLine.length()>200) uartLine="";
    }
  }

  delay(1);
}