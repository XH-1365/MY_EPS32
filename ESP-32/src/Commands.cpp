#include "Commands.h"
#include "Motion.h"
#include "ServoPunch.h"
#include "Braille.h"
#include "Config.h"

String posJson(){
  long x = curEncX();
  long y = curEncY();
  float xmm = (float)x / PULSES_PER_MM_X;
  float ymm = (float)y / PULSES_PER_MM_Y;

  String s="{";
  s += "\"encX\":"+String(x)+",";
  s += "\"encY\":"+String(y)+",";
  s += "\"xmm\":"+String(xmm,3)+",";
  s += "\"ymm\":"+String(ymm,3)+",";
  s += "\"mx_running\":"+String(isMovingX()?"true":"false")+",";
  s += "\"my_running\":"+String(isMovingY()?"true":"false")+",";
  s += "\"printing\":"+String(brailleIsPrinting()?"true":"false")+",";
  s += "\"dotI\":"+String(brailleDotI())+",";
  s += "\"dotN\":"+String(brailleDotN());
  s += "}";
  return s;
}

String execCommand(String cmd){
  cmd.trim();
  if(cmd.length()==0) return "EMPTY";

  if(cmd=="help"){
    return "CMD: home | pos | mx <mm> | my <mm> | mxy <dx> <dy> | punch | print <text> | stop";
  }
  if(cmd=="pos"){
    return posJson();
  }
  if(cmd=="home"){
    brailleStop();
    stopAllBrake();
    zeroEncoders();
    servoUp();
    return "OK home";
  }
  if(cmd=="stop"){
    brailleStop();
    stopAllBrake();
    return "OK stop";
  }
  if(cmd=="punch"){
    servoPunchOnce();
    return "OK punch";
  }
  if(cmd.startsWith("mx ")){
    float mm = cmd.substring(3).toFloat();
    startMoveX(mm);
    return "OK mx " + String(mm,3);
  }
  if(cmd.startsWith("my ")){
    float mm = cmd.substring(3).toFloat();
    startMoveY(mm);
    return "OK my " + String(mm,3);
  }
  if(cmd.startsWith("mxy ")){
    String rest=cmd.substring(4); rest.trim();
    int sp=rest.indexOf(' ');
    if(sp<=0) return "ERR usage: mxy <dx> <dy>";
    float dx=rest.substring(0,sp).toFloat();
    float dy=rest.substring(sp+1).toFloat();
    startMoveX(dx);
    while(isMovingX()){ motionUpdate(); delay(1); }
    startMoveY(dy);
    return "OK mxy";
  }
  if(cmd.startsWith("print ")){
    String text=cmd.substring(6); text.trim();
    if(text.length()==0) return "ERR usage: print <text>";
    brailleStop();
    brailleQueueText(text);
    brailleStart();
    return "OK print queued";
  }
  return "UNKNOWN";
}