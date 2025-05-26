#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SD.h>

// TFT pins
#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  -1    // tied to 3.3V

// Backlight pin (PWM)
#define TFT_BL   15    // connected to backlight +

// Battery voltage divider pin
#define BAT_ADC_PIN 34 // ADC1 channel 6

// Battery calibration
#define BAT_FULL_VOLTAGE 4.2f
#define BAT_EMPTY_VOLTAGE 3.0f
#define BAT_CAL_FACTOR   (3.68f/3.17f)

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Button pins (INPUT_PULLUP, active LOW)
const int BTN_UP     = 32; // rotate
const int BTN_DOWN   = 33; // soft drop
const int BTN_LEFT   = 25; // move left
const int BTN_RIGHT  = 26; // move right
const int BTN_A      = 27; // brightness +
const int BTN_B      = 14; // brightness -
const int BTN_START  = 22; // start/restart
const int BTN_SELECT = 13; // pause
const int BTN_POWER  = 21;
int buttonPins[] = {BTN_UP,BTN_DOWN,BTN_LEFT,BTN_RIGHT,BTN_A,BTN_B,BTN_START,BTN_SELECT,BTN_POWER};
volatile int brightness = 128;

// Tetris settings
#define GRID_COLS 10
#define GRID_ROWS 20
const int CELL_W = 24;
const int CELL_H = 15;

typedef uint16_t Color;
typedef struct { Color c; bool filled; } Cell;
Cell grid[GRID_ROWS][GRID_COLS];
int currentPiece, rotation, pieceX, pieceY;
Color pieceColor;
const uint16_t tetromino[7][4] = {
  {0x0F00,0x4444,0x0F00,0x4444},
  {0x0660,0x0660,0x0660,0x0660},
  {0x0710,0x3220,0x0470,0x2260},
  {0x0622,0x0740,0x0622,0x0740},
  {0x0264,0x0470,0x0264,0x0470},
  {0x0170,0x0446,0x0710,0x0644},
  {0x0710,0x0462,0x0170,0x0624}
};

unsigned long lastDrop, lastMove;
int dropInterval = 500;
int moveInterval = 300; // ms between lateral/rotate moves (adjusted slower) // ms between lateral/rotate moves
bool gameOver;

// Prototypes
void newPiece();
bool collide(int x,int y,int rot);
void placePiece();
void clearLines();
void drawGrid();
void drawPiece(int x,int y,int id,int rot,bool erase=false);
void drawBattery();

// Brightness ISRs
void IRAM_ATTR onButtonA(){ brightness = min(brightness+16,255); analogWrite(TFT_BL,brightness); }
void IRAM_ATTR onButtonB(){ brightness = max(brightness-16,0);   analogWrite(TFT_BL,brightness); }

void setup(){
  Serial.begin(115200);
  pinMode(TFT_BL, OUTPUT); analogWrite(TFT_BL, brightness);
  analogReadResolution(12); analogSetAttenuation(ADC_11db); pinMode(BAT_ADC_PIN, INPUT);
  for(int i=0;i<9;i++) pinMode(buttonPins[i], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_A), onButtonA, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_B), onButtonB, FALLING);
  SPI.begin(18,19,23);
  tft.begin(); tft.fillScreen(ILI9341_BLACK);
  // init grid
  for(int r=0;r<GRID_ROWS;r++) for(int c=0;c<GRID_COLS;c++) grid[r][c] = {0,false};
  gameOver = false;
  newPiece();
  lastDrop = millis();
  lastMove = 0;
}

void loop(){
  unsigned long now = millis();
  drawBattery();
  if(!gameOver){
    // lateral and rotate with interval
    if(now - lastMove > moveInterval) {
      if(digitalRead(BTN_LEFT)==LOW && !collide(pieceX-1,pieceY,rotation)){
        drawPiece(pieceX,pieceY,currentPiece,rotation,true);
        pieceX--; drawPiece(pieceX,pieceY,currentPiece,rotation);
        lastMove = now;
      }
      else if(digitalRead(BTN_RIGHT)==LOW && !collide(pieceX+1,pieceY,rotation)){
        drawPiece(pieceX,pieceY,currentPiece,rotation,true);
        pieceX++; drawPiece(pieceX,pieceY,currentPiece,rotation);
        lastMove = now;
      }
      else if(digitalRead(BTN_UP)==LOW && !collide(pieceX,pieceY,(rotation+1)&3)){
        drawPiece(pieceX,pieceY,currentPiece,rotation,true);
        rotation = (rotation+1)&3; drawPiece(pieceX,pieceY,currentPiece,rotation);
        lastMove = now;
      }
    }
    // drop speed control
    dropInterval = digitalRead(BTN_DOWN)==LOW ? 50 : 500;
    if(now - lastDrop > dropInterval) {
      lastDrop = now;
      if(!collide(pieceX,pieceY+1,rotation)){
        drawPiece(pieceX,pieceY,currentPiece,rotation,true);
        pieceY++; drawPiece(pieceX,pieceY,currentPiece,rotation);
      } else {
        placePiece(); clearLines(); newPiece();
      }
    }
  }
}

void newPiece(){
  currentPiece = random(0,7);
  rotation = 0; pieceX = (GRID_COLS/2)-2; pieceY = 0;
  uint16_t colors[] = {ILI9341_RED,ILI9341_GREEN,ILI9341_BLUE,ILI9341_YELLOW,ILI9341_CYAN,ILI9341_MAGENTA,ILI9341_ORANGE};
  pieceColor = colors[random(0,7)];
  if(collide(pieceX,pieceY,rotation)) gameOver = true;
}

bool collide(int x,int y,int rot){
  uint16_t shape = tetromino[currentPiece][rot];
  for(int i=0;i<4;i++) for(int j=0;j<4;j++) if(shape & (0x8000>>(i*4+j))) {
    int gx = x+j, gy = y+i;
    if(gx<0||gx>=GRID_COLS||gy<0||gy>=GRID_ROWS|| grid[gy][gx].filled) return true;
  }
  return false;
}

void placePiece(){
  uint16_t shape = tetromino[currentPiece][rotation];
  for(int i=0;i<4;i++) for(int j=0;j<4;j++) if(shape & (0x8000>>(i*4+j))) {
    grid[pieceY+i][pieceX+j] = {pieceColor,true}; }
  drawGrid();
}

void clearLines(){
  for(int i=0;i<GRID_ROWS;i++){
    bool full = true;
    for(int j=0;j<GRID_COLS;j++) if(!grid[i][j].filled) { full = false; break; }
    if(full){
      for(int k=i;k>0;k--) memcpy(grid[k],grid[k-1],GRID_COLS*sizeof(Cell));
      for(int c=0;c<GRID_COLS;c++) grid[0][c] = {0,false};
      drawGrid();
    }
  }
}

void drawGrid(){
  tft.fillRect(0,20,GRID_COLS*CELL_W,GRID_ROWS*CELL_H,ILI9341_BLACK);
  for(int r=0;r<GRID_ROWS;r++) for(int c=0;c<GRID_COLS;c++) if(grid[r][c].filled) {
    tft.fillRect(c*CELL_W, r*CELL_H+20, CELL_W, CELL_H, grid[r][c].c);
  }
}

void drawPiece(int x,int y,int id,int rot,bool erase){
  uint16_t shape = tetromino[id][rot];
  for(int i=0;i<4;i++) for(int j=0;j<4;j++) if(shape & (0x8000>>(i*4+j))) {
    int px = (x+j)*CELL_W, py = (y+i)*CELL_H+20;
    tft.fillRect(px,py,CELL_W,CELL_H, erase?ILI9341_BLACK:pieceColor);
  }
}

void drawBattery(){
  int raw = analogRead(BAT_ADC_PIN);
  float measured = (raw/4095.0f)*3.3f;
  float voltage = measured*2.0f*BAT_CAL_FACTOR;
  int pct = constrain(int((voltage-BAT_EMPTY_VOLTAGE)/(BAT_FULL_VOLTAGE-BAT_EMPTY_VOLTAGE)*100+0.5),0,100);
  tft.fillRect(0,0,tft.width(),20,ILI9341_BLACK);
  tft.setCursor(2,2); tft.setTextSize(2); tft.print(voltage,2); tft.print('V');
  tft.setCursor(tft.width()-50,2); tft.print(pct); tft.print('%');
}
