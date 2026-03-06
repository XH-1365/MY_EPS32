#include "Braille.h"
#include "Config.h"
#include "Motion.h"
#include "ServoPunch.h"

struct Dot{ float x,y; };
static Dot dots[5000];
static int dotN=0, dotI=0;
static bool printing=false;

static uint8_t braillePattern(char c){
  if(c>='A'&&c<='Z') c=c-'A'+'a';
  switch(c){
    case 'a': return 0b000001; case 'b': return 0b000011; case 'c': return 0b000101;
    case 'd': return 0b001101; case 'e': return 0b001001; case 'f': return 0b000111;
    case 'g': return 0b001111; case 'h': return 0b001011; case 'i': return 0b000110;
    case 'j': return 0b001110; case 'k': return 0b010001; case 'l': return 0b010011;
    case 'm': return 0b010101; case 'n': return 0b011101; case 'o': return 0b011001;
    case 'p': return 0b010111; case 'q': return 0b011111; case 'r': return 0b011011;
    case 's': return 0b010110; case 't': return 0b011110; case 'u': return 0b100001;
    case 'v': return 0b100011; case 'w': return 0b101110; case 'x': return 0b100101;
    case 'y': return 0b101101; case 'z': return 0b101001;
    case ' ': return 0;
    default: return 0;
  }
}

static void addCell(float baseX,float baseY,uint8_t pat){
  for(int d=0; d<6; d++){
    if(pat & (1<<d)){
      int col = (d>=3)?1:0;
      int row = d%3;
      float x = baseX + (col?DOT_DX_MM:0.0f);
      float y = baseY + row*DOT_DY_MM;
      if(dotN < (int)(sizeof(dots)/sizeof(dots[0]))) dots[dotN++] = {x,y};
    }
  }
}

void brailleQueueText(const String& text){
  dotN=0; dotI=0;
  int col=0,row=0;
  for(int i=0;i<(int)text.length();i++){
    if(row>=PAGE_ROWS) break;
    uint8_t pat = braillePattern(text[i]);
    float cellX = PAGE_ORIGIN_X_MM + col*CELL_DX_MM;
    float cellY = PAGE_ORIGIN_Y_MM + row*LINE_DY_MM;
    addCell(cellX,cellY,pat);
    col++;
    if(col>=PAGE_COLS){ col=0; row++; }
  }
}

void brailleStart(){
  // 每次开始都先回原点
  moveToAbs(0.0f,0.0f);
  printing = true;
}
void brailleStop(){
  printing=false; dotN=0; dotI=0;
}

bool brailleIsPrinting(){ return printing; }
int  brailleDotI(){ return dotI; }
int  brailleDotN(){ return dotN; }

void brailleProcess(){
  if(!printing) return;

  if(dotI >= dotN){
    moveToAbs(0.0f,0.0f);
    printing=false;
    return;
  }
  Dot d = dots[dotI++];
  moveToAbs(d.x, d.y);
  servoPunchOnce();
}