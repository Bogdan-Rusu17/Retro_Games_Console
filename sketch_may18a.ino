#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SD.h>
#include <esp_sleep.h>
#include <driver/ledc.h> 
#include "esp32-hal-ledc.h"

// --- Configuration ---
#define NOTE_A4      440
#define NOTE_B4      494
#define NOTE_C5      523
#define NOTE_D5      587
#define NOTE_E5      659

#define TFT_CS       5
// tft_dc face separatie intre bytes de comanda si bytes de date
// aici date inseamna pixeli
// high - date
// low - comanda
#define TFT_DC       2
#define TFT_RST     -1  // pus la 3v3
#define TFT_BL      15  // pwm pt luminozitate ecran
#define SD_CS        4   // select la card

// instantiaza obiectul tft cu care am acces la functiile ecranului
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// constante pentru masurarea bateriei
// limitele intre 3v si 4.2v ca in 
// https://www.ufinebattery.com/blog/everything-you-should-know-about-18650-battery-voltage/
// am pus 3v ca sa nu ma duc chiar pana la limita inferioara

#define BAT_ADC_PIN      34
#define BAT_FULL_VOLTAGE 4.2f
#define BAT_EMPTY_VOLTAGE 3.0f

// din cauza imperfectiunii firelor si cum le-am lipit
// se citeste mai putin pe pinul de ADC si masurat cu voltmetru
// cam asta e raportul
#define BAT_CAL_FACTOR   (3.68f / 3.17f)

// Button pins
const int BTN_UP    = 32;
const int BTN_DOWN  = 33;
const int BTN_LEFT  = 25;
const int BTN_RIGHT = 26;
const int BTN_A     = 27;
const int BTN_B     = 14;
const int BTN_MENU  = 22; // menu/select
const int BTN_SOUND = 21; // toggle la sunet
const int BTN_POWER = 12; // pin RTC-capable ca sa pot sa retrezesc consola

#define BUZZER_PIN 13

// Grid calculat sa acopere tot width si ~ tot height ul din arena de joc
// pt ca avem doua benzi cu bateria si scorul una sub alta
#define GRID_COLS 15
#define GRID_ROWS 17
const int CELL_W = 16;
const int CELL_H = 16;

// variabile si definitii care ne ajuta sa vedem cum coloram celulele de pe harta
typedef uint16_t Color;
typedef struct {
  Color c;
  bool  filled;
} Cell;

Cell grid[GRID_ROWS][GRID_COLS];

// definitii tetromino

// formele in coordonate carteziene pentru piesele de tetris
// se porneste de la o forma fixa pentru fiecare piesa
// si cel mai simplu mod de a obtine coordonatele pentru o rotatie
// e sa inmultim cu matricea
// {
// {cos(pi / 2), sin(pi / 2)},
// {cos(pi / 2), -sin(pi / 2)}}

const int8_t shapes[7][4][4][2] = {
  // I-piece
  { // 0°        90°         180°       270°
    {{-2,0},{-1,0},{0,0},{1,0}},    //orizontal
    {{0,-2},{0,-1},{0,0},{0,1}},    // vertical
    {{-2,-1},{-1,-1},{0,-1},{1,-1}},// orizontal
    {{-1,-2},{-1,-1},{-1,0},{-1,1}} // vertical 
  },

  // O-piece (patrat, invariant la rotatie)
  {
    {{0,0},{1,0},{0,1},{1,1}},
    {{0,0},{1,0},{0,1},{1,1}},
    {{0,0},{1,0},{0,1},{1,1}},
    {{0,0},{1,0},{0,1},{1,1}}
  },

  // T-piece
  {
    {{-1,0},{0,0},{1,0},{0,1}},   // T-down
    {{0,-1},{0,0},{0,1},{1,0}},   // T-right
    {{-1,0},{0,0},{1,0},{0,-1}},  // T-up
    {{0,-1},{0,0},{0,1},{-1,0}}   // T-left
  },

  // S-piece
  {
    {{0,0},{1,0},{-1,1},{0,1}},   // S-flat
    {{0,-1},{0,0},{1,0},{1,1}},   // S-tall
    {{0,0},{1,0},{-1,1},{0,1}},   // S-flat
    {{0,-1},{0,0},{1,0},{1,1}}    // S-tall
  },

  // Z-piece
  {
    {{-1,0},{0,0},{0,1},{1,1}},    // Z-flat
    {{1,-1},{1,0},{0,0},{0,1}},    // Z-tall
    {{-1,0},{0,0},{0,1},{1,1}},    // Z-flat
    {{1,-1},{1,0},{0,0},{0,1}}     // Z-tall
  },

  // J-piece
  {
    {{-1,0},{0,0},{1,0},{1,1}},    //
    {{0,-1},{0,0},{0,1},{1,-1}},   // 
    {{-1,-1},{-1,0},{0,0},{1,0}},  //
    {{-1,1},{0,-1},{0,0},{0,1}}    //
  },

  // L-piece
  {
    {{-1,0},{0,0},{1,0},{-1,1}},   // 
    {{0,-1},{0,0},{0,1},{1,1}},    //
    {{1,-1},{-1,0},{0,0},{1,0}},   //
    {{-1,-1},{0,-1},{0,0},{0,1}}   //
  }
};

// indexul piesei curente, rotatia aplicata si coordonatele sale relative
int currentPiece, rotation, pieceX, pieceY;
Color pieceColor;

// scorul acumulat
int score = 0;

#define MAX_SNAKE 100 // cat de lung poate fi snake
int sx[MAX_SNAKE], sy[MAX_SNAKE]; // coordonatele patratelor care alcatuiesc sarpele
int tail;                          // lungime_curenta - 1
int dirX, dirY;                    // in ce directie ma duc ca versor pe grid
int appleX, appleY; // coordonata marului

enum State { SHOW_SPLASH, MENU, TETRIS, SNAKE } state;
const char* menuItems[] = { "Tetris", "Snake" };
const int MENU_COUNT = 2;

int menuIndex = 0;

// Melodia de tetris cu ordinea frecventelor si durate ale notelor
const int melody[] = {
  NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4,
  NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_A4, NOTE_A4
};
const int beatDurations[] = {
  250, 125, 125, 250, 250, 250, 250, 500,
  250, 250, 250, 250, 250, 250, 250, 500,
  250, 250, 500
};
const int notesCount = sizeof(melody)/sizeof(melody[0]);
unsigned long themeNoteStart = 0;
int themeIndex = 0;

// Timing pentru cat de des cade piesa cu o unitate pe grid
unsigned long lastDrop = 0;
int dropInterval = 500;

// Brightness
volatile int brightness = 128;
unsigned long lastBright = 0;
const unsigned long BRIGHT_DEBOUNCE_MS = 100;

// Flaguri pentru detectia apasarii butoanelor si rate de debounce
// si valorile anterioare ale butoanelor
// volatile ca sa fie vizibile si in bucla loop

volatile bool btnUpFlag=false, btnDownFlag=false, btnLeftFlag=false,
              btnRightFlag=false, btnMenuFlag=false,
              btnSoundFlag=false, btnPowerFlag=false;
const uint32_t DEB_SOUND=200000, DEB_POWER=1000000, DEB_GAME=20000, DEB_FAST = 8000;
volatile uint32_t lastUp, lastDown, lastLeft, lastRight;
volatile uint32_t lastMenu, lastSound, lastPower;


// flag operat doar in main ca sa vad daca opresc muzica
bool soundOff = false;

// functii prototip ca sa le pot apela in loop()
void drawSplash();
void drawMenu();
void startTetris();
void startSnake();
void drawGrid();
void drawPiece(int x, int y, int id, int rot, bool erase=false);
bool collide(int x, int y, int rot);
void placePiece();
void clearLines();
void newPiece();
void drawBattery();
void drawScore();
void displayGameOver();
void resetGrid();
void placeApple();

// helper de ISR pt butoane ca sa nu am cod duplicat
// verific la debounce sa fi trecut un numar de secunde ca sa iau apasarea in considerare

void IRAM_ATTR onBtn(volatile uint32_t &lastTime, uint32_t debounce, volatile bool &flag) {
  uint32_t now = micros();
  if (now - lastTime > debounce) flag = true;
  lastTime = now;
}

// interuperi pentru butoane cu rate custom de debounce dependent de status consola
// de exemplu la butonul UP cand suntem in jocul de tetris cu el rotim piesa si nu
// vrem sa se roteasca foarte repede ca sa controlam precizia
// cand suntem pe meniu vrem cam tot acelasi debounce rate
// dar cand suntem pe snake vrem sa avem rapiditate

void IRAM_ATTR onUp() {
  if (state == TETRIS) {
    onBtn(lastUp, DEB_SOUND, btnUpFlag);
  } else if (state == SNAKE) {
    onBtn(lastUp, DEB_FAST, btnUpFlag);
  }

  onBtn(lastUp, DEB_SOUND, btnUpFlag);
}
void IRAM_ATTR onDown() { 
  if (state == TETRIS) {
    onBtn(lastDown, DEB_FAST, btnDownFlag);
  } else if (state == SNAKE) {
    onBtn(lastDown, DEB_FAST, btnDownFlag);
  }

  onBtn(lastDown, DEB_SOUND, btnDownFlag);
}
void IRAM_ATTR onLeft() {
  if (state == TETRIS) {
    onBtn(lastLeft, DEB_GAME, btnLeftFlag); 
  }
  onBtn(lastLeft, DEB_FAST, btnLeftFlag);

}
void IRAM_ATTR onRight() {
  if (state == TETRIS) {
    onBtn(lastRight, DEB_GAME, btnRightFlag); 
  }
  onBtn(lastRight, DEB_FAST, btnRightFlag);
}

// rutine de tratare intreruperi simple pentru butoanele cu o singura functie

void IRAM_ATTR onMenu()  { onBtn(lastMenu, DEB_SOUND, btnMenuFlag); }
void IRAM_ATTR onSound() { onBtn(lastSound,DEB_SOUND, btnSoundFlag); }
void IRAM_ATTR onPower() { onBtn(lastPower,DEB_POWER, btnPowerFlag); }

void setup() {
  // initializez PWM-ul pe pinul capabil PWM TFT_BL pe canalul 0 
  pinMode(TFT_BL, OUTPUT);
  analogWrite(TFT_BL, brightness);

  // initializez buzzer ul pe canalul 2 de PWM ca sa nu intre in conflict cu backlight-ul
  ledcAttachChannel(BUZZER_PIN, 2000, 8, 2);
  ledcWriteTone(BUZZER_PIN, 0);

  // ADC pentru baterie battery, 12 biti rezolutie, 11 db pt zgomot atenuare
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(BAT_ADC_PIN, INPUT);

  // initializam protocolul SPI specificand in ordine
  // SCLK, MISO si MOSI
  SPI.begin(18, 19, 23);
  tft.begin();
  tft.setRotation(0);

  Serial.begin(115200);

  // initializare si verificare pe serial ca merge cardul SD
  pinMode(SD_CS, OUTPUT);
  if (!SD.begin(SD_CS)) {
  Serial.println("SD init failed!");
  } else {
    Serial.println("SD init OK");
    // quick Read/Write test
    const char* testPath = "/sd_test.txt";
    if (SD.exists(testPath)) {
      File t = SD.open(testPath, FILE_READ);
      Serial.print("Read back: ");
      Serial.println(t.readStringUntil('\n'));
      t.close();
    } else {
      File t = SD.open(testPath, FILE_WRITE);
      t.println("hello from ESP32");
      t.close();
      Serial.println("Created /sd_test.txt");
    }
  }

  // atasarea intreruperilor
  pinMode(BTN_UP,    INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(BTN_UP),   onUp,    FALLING);
  pinMode(BTN_DOWN,  INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(BTN_DOWN), onDown,  FALLING);
  pinMode(BTN_LEFT,  INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(BTN_LEFT), onLeft,  FALLING);
  pinMode(BTN_RIGHT, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(BTN_RIGHT),onRight, FALLING);
  pinMode(BTN_MENU,  INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(BTN_MENU), onMenu,  FALLING);
  pinMode(BTN_SOUND, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(BTN_SOUND),onSound, FALLING);
  pinMode(BTN_POWER, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(BTN_POWER),onPower, FALLING);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);

  // atasam butonului de power care e RTC-Capable privilegiul de a trezi
  // controllerul din sleep
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_POWER, 0);

  // curata arena de joc
  resetGrid();

  // incepe cu state ul de splash art
  state = SHOW_SPLASH;
  themeNoteStart = millis();
  themeIndex = 0;
  drawSplash();
}

void loop() {
  // cand am apasat pe butonul de power
  // flag ul se pune pe true si il resetam imediat
  // apoi facem un delay ca sa nu detectez aceeasi apasare fizica, umana
  // ca power up
  // puteam sa fac si cu variabila dar am incercat varianta mai safe prima data
  // si am vazut ca merge si am lasat asa
  // asteptam pana se elibereaza butonul
  // facem ecranul negru
  // teoretic il si inchidem
  // si incepem deep sleep

  if (btnPowerFlag) {
    btnPowerFlag = false;
    delay(50);
    while (digitalRead(BTN_POWER) == LOW) delay(10);
    tft.fillScreen(ILI9341_BLACK);
    analogWrite(TFT_BL, 0);
    esp_deep_sleep_start();
    return;
  }

  unsigned long now = millis();

  // creste sau descreste luminozitatea cu debouncesi
  if (digitalRead(BTN_A)==LOW && now - lastBright > BRIGHT_DEBOUNCE_MS) {
    brightness = min(brightness + 16, 255);
    analogWrite(TFT_BL, brightness);
    lastBright = now;
  }
  if (digitalRead(BTN_B)==LOW && now - lastBright > BRIGHT_DEBOUNCE_MS) {
    brightness = max(brightness - 16, 0);
    analogWrite(TFT_BL, brightness);
    lastBright = now;
  }

  switch (state) {
    case SHOW_SPLASH:
      if (btnMenuFlag) {
        btnMenuFlag = false;
        state = MENU;
        drawMenu();
      }
      break;

    case MENU:
      // butoanele UP si DOWN cicleaza printre cele 2 jocuri
      if (btnUpFlag || btnDownFlag) {
        btnUpFlag = btnDownFlag = false;
        if (btnUpFlag) menuIndex = (menuIndex + MENU_COUNT - 1) % MENU_COUNT;
        else            menuIndex = (menuIndex + 1) % MENU_COUNT;
        drawMenu();
      }
      if (btnMenuFlag) {
        btnMenuFlag = false;
        if (menuIndex == 0) {
          startTetris();
        } else {
          startSnake();
        }
      }
      break;

    case TETRIS:
      // verifica daca am apasat menu ca sa ne intoarcem la selectare jocuri
      // si opreste muzica
      if (btnMenuFlag) {
        btnMenuFlag = false;
        ledcWriteTone(BUZZER_PIN, 0);
        state = MENU;
        drawMenu();
        break;
      }

      // afisam benzile cu bateria si scorul
      drawBattery();
      drawScore();

      // opreste melodia daca am apasat butonul de sound off/on
      if (btnSoundFlag) {
        ledcWriteTone(BUZZER_PIN,0);
        btnSoundFlag = false;
        soundOff = !soundOff;
      }

      // daca nu avem muzica oprita si putem trece la urmatoarea nota trecem la ea
      // ciclic, reluand de la inceput melodia
      if (!soundOff && now - themeNoteStart >= beatDurations[themeIndex] + 50) {
        themeIndex = (themeIndex + 1) % notesCount;
        ledcWriteTone(BUZZER_PIN, melody[themeIndex]);
        themeNoteStart = now;
      }
      

      // daca mutandu-ne la dreapta sau stanga nu intram in coliziune sau iesim din arena
      // atunci facem miscarea pe butonul corespunzator si stergem piesa de la pozitia veche
      // si o redesam la cea noua

      if (btnLeftFlag && !collide(pieceX - 1, pieceY, rotation)) {
        drawPiece(pieceX, pieceY, currentPiece, rotation, true);
        pieceX--;
        drawPiece(pieceX, pieceY, currentPiece, rotation);
        btnLeftFlag = false;
      }
      if (btnRightFlag && !collide(pieceX + 1, pieceY, rotation)) {
        drawPiece(pieceX, pieceY, currentPiece, rotation, true);
        pieceX++;
        drawPiece(pieceX, pieceY, currentPiece, rotation);
        btnRightFlag = false;
      }

      // daca apasam pe down coboram mai repede
      dropInterval = btnDownFlag? 50: 500;
      btnDownFlag = false;
      // daca a trecut suficient timp ca sa dam drop
      if (now - lastDrop >= dropInterval) {
        lastDrop = now;

        // facem aceeasi verificare ca mai sus la deplasarea stanga dreapta
        // daca intra in coliziune fixam piesa, aici poate avea coliziune cu alta piesa
        // sau ground
        // si apoi verificam daca putem elimina o linie sau mai multe completate full

        if (!collide(pieceX, pieceY + 1, rotation)) {
          drawPiece(pieceX, pieceY, currentPiece, rotation, true);
          pieceY++;
          drawPiece(pieceX, pieceY, currentPiece, rotation);
        } else {
          placePiece();
          clearLines();
          newPiece();
        }
      }

      // aici verificam ciclic daca e posibil a roti piesa cu 90 de grade
      // & 3 e fancy way de modulo 4 
      if (btnUpFlag && !collide(pieceX, pieceY, (rotation + 1) & 3)) {
        drawPiece(pieceX, pieceY, currentPiece, rotation, true);
        rotation = (rotation+1) & 3;
        drawPiece(pieceX, pieceY, currentPiece, rotation);
        btnUpFlag = false;
      }
      break;
    
    case SNAKE: {
      static unsigned long lastMove = 0;
      const unsigned long MOVE_INTERVAL = 200;
      
      if (btnMenuFlag) {
        btnMenuFlag = false;
        ledcWriteTone(BUZZER_PIN, 0);
        state = MENU;
        drawMenu();
        break;
      }

      unsigned long now = millis();
      if (now - lastMove >= MOVE_INTERVAL) {
        lastMove = now;

        // sterge vechea pozitie a cozii sarpelui
        int tx = sx[tail], ty = sy[tail];
        
        drawBattery();
        drawScore();
        grid[ty][tx] = { 0, false };

        // misca bucatile din corpul sarpelui shiftand la dreapta cu o pozitie 
        for (int i = tail; i > 0; i--) {
          sx[i] = sx[i-1];
          sy[i] = sy[i-1];
        }

        // capul sarpelui avanseaza in directia deplasarii 
        sx[0] += dirX;
        sy[0] += dirY;

        // verificam daca ne lovim de noi insine sau de pereti pierdem
        if (sx[0] < 0 || sx[0] >= GRID_COLS ||
            sy[0] < 0 || sy[0] >= GRID_ROWS ||
            grid[sy[0]][sx[0]].filled) {
          // game over si apoi la meniu
          resetGrid();
          tail = 0;
          dirX = dirY = 0;
          appleX = appleY = -1;
          for (int i = 0; i < MAX_SNAKE; i++) {
            sx[i] = sy[i] = 0;
          }
          displayGameOver();

          break;
        }

        // Daca ajungem cu capul la marul de pe tabla atunci crestem scorul si crestem cu o unitate
        bool grow = (sx[0] == appleX && sy[0] == appleY);
        if (grow) {
          score += 10;
        } else {
          // dam remove la vechea coada daca nu am gasit mar 
          grid[sy[tail] ][sx[tail]] = {0,false};
        }

        // desenam noul cap al sarpelui
        grid[sy[0]][sx[0]] = { ILI9341_GREEN, true };
        drawGrid();

        // daca crescusem trebuie sa crestem lungimea si sa actualizam pozitia cozii
        // si sa mai punem un alt mar pe tabla
        if (grow) { 
          tail = min(tail + 1, MAX_SNAKE-1);
          placeApple();
        }

        // desenam marul curent de pe tabla de joc la pozitiile specificate
        tft.fillRect((240 - GRID_COLS * CELL_W) / 2 + appleX*CELL_W,
               40 + appleY*CELL_H,
               CELL_W, CELL_H,
               ILI9341_RED);
      }
      

      // modificam versorul directie in functie de ce apasam
      // dar avem grija sa putem sa ne ducem in directia aia
      if (btnUpFlag && dirY==0)    { dirX = 0; dirY = -1; btnUpFlag = false; }
      if (btnDownFlag && dirY==0)  { dirX = 0; dirY = 1; btnDownFlag = false; }
      if (btnLeftFlag && dirX==0)  { dirX = -1; dirY = 0; btnLeftFlag = false; }
      if (btnRightFlag && dirX==0) { dirX = 1; dirY = 0; btnRightFlag = false; }

      break;
    }
  }
}

void drawSplash() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(4); tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20,100); tft.print("RetroPlay");
  tft.setTextSize(2); tft.setCursor(60,150); tft.print("RBO Industries");
}

void drawMenu() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);

  // deseneaza 
  for (int i = 0; i < MENU_COUNT; i++) {
    int x = 20;
    int y = 80 + i * 60;
    int w = 200;
    int h = 40;
    // outline pentru dreptunghiurile de selectie
    tft.drawRect(x, y, w, h, ILI9341_WHITE);
    // daca jocul selectat e i, atunci da i alta culoare
    if (i == menuIndex) {
      tft.fillRect(x + 2, y + 2, w - 4, h - 4, ILI9341_WHITE);
      tft.setTextColor(ILI9341_BLACK);
    } else {
      tft.setTextColor(ILI9341_YELLOW);
    }
    tft.setCursor(x + 10, y + 10);
    tft.print(menuItems[i]);
  }

}

void startTetris() {
  state = TETRIS;
  score = 0;
  resetGrid();
  tft.fillScreen(ILI9341_BLACK);
  drawBattery();
  drawScore();
  newPiece();
  drawGrid();
  drawPiece(pieceX, pieceY, currentPiece, rotation);
  lastDrop = millis();
  themeNoteStart = millis();
  // incepe melodia
  ledcWriteTone(BUZZER_PIN, melody[0]);
}

void resetGrid() {
  for(int r = 0; r < GRID_ROWS; r++)
    for(int c = 0; c < GRID_COLS; c++)
      grid[r][c] = {0, false};
}

void newPiece() {
  // alege random forma si culoarea noii piese
  currentPiece = random(0, 7);
  rotation=0;
  pieceX = (GRID_COLS / 2) - 2;
  pieceY = 0;
  Color cols[]={ILI9341_RED, ILI9341_GREEN, ILI9341_BLUE,
                ILI9341_YELLOW, ILI9341_CYAN,
                ILI9341_MAGENTA, ILI9341_ORANGE};
  pieceColor = cols[random(0, 7)];

  // game over daca spawn ul e blocat
  if (collide(pieceX, pieceY, rotation)) {
    displayGameOver();
  }
}

bool collide(int x, int y, int rot){
  // suntem in coliziune daca pozitia world + pozitia in spatiul obiect ar fi
  // ori in afara hartii ori intr-o alta piesa
  for (int b = 0; b < 4; b++) {
    int gx = x + shapes[currentPiece][rot][b][0];
    int gy = y + shapes[currentPiece][rot][b][1];
    if (gx < 0 || gx >= GRID_COLS || gy < 0 || gy >= GRID_ROWS ||
        grid[gy][gx].filled) {
      return true;
    }
  }
  return false;
}

void placePiece() {
  // cand ne oprim marcam cu culoarea aferenta pozitia din grid si aratam ca e filled
  for (int b = 0; b < 4; b++) {
    int gx = pieceX + shapes[currentPiece][rotation][b][0];
    int gy = pieceY + shapes[currentPiece][rotation][b][1];
    grid[gy][gx] = { pieceColor, true };
  }
  drawGrid();
}

void clearLines() {
  // elimina liniile care sunt deja full
  for(int i = 0; i < GRID_ROWS; i++) {
    bool full = true;
    for(int j = 0; j < GRID_COLS; j++) if (!grid[i][j].filled) {
       full = false;
       break; 
    }
    if (full) {
      score += 100;
      // copiaza mai jos bucatile din piese de pe randul de mai sus
      for (int k = i; k > 0; k--)
        memcpy(grid[k], grid[k - 1], GRID_COLS * sizeof(Cell));
      for (int c = 0; c < GRID_COLS; c++)
        grid[0][c]={0, false};
      drawGrid();
      drawScore();
    }
  }
}

void drawGrid() {
  // coloreaza pe ecran in functie de culorile care sunt in grid
  for(int r = 0; r < GRID_ROWS; r++) {
    for(int c = 0; c < GRID_COLS; c++) {
      Color col = grid[r][c].filled ? grid[r][c].c : ILI9341_BLACK;
      tft.fillRect(c * CELL_W, r * CELL_H + 40, CELL_W, CELL_H, col);
    }
  }
}

void drawPiece(int x, int y, int id, int rot, bool erase) {
  // coloreaza pe ecran o piesa in miscare dupa culoarea aferenta
  // sau daca parametrul erase e true, coloreaza cu negru adica sterge practic
  for (int b = 0; b < 4; b++) {
    int dx = shapes[id][rot][b][0];
    int dy = shapes[id][rot][b][1];
    int px = (x + dx) * CELL_W;
    int py = (y + dy) * CELL_H + 40;
    tft.fillRect(px, py, CELL_W, CELL_H,
                 erase ? ILI9341_BLACK : pieceColor);
  }
}

void placeApple() {
  // genereaza pozitii aleatoare pentru mar atata timp cat nu e ocupata patratica respectiva
  // si deseneaza marul
  do {
    appleX = random(0, GRID_COLS);
    appleY = random(0, GRID_ROWS);
  } while (grid[appleY][appleX].filled);
  tft.fillRect((240 - GRID_COLS * CELL_W) / 2 + appleX * CELL_W,
               40 + appleY * CELL_H,
               CELL_W, CELL_H,
               ILI9341_RED);
}

void startSnake() {
  state = SNAKE;
  score = 0;
  resetGrid();
  tft.fillScreen(ILI9341_BLACK);
  drawBattery();
  drawScore();

  // porneste din mijlocul hartii cu 3 patratele lungime
  tail = 3;
  int mx = GRID_COLS/2, my = GRID_ROWS / 2;
  for (int i = 0; i <= tail; i++) {
    sx[i] = mx - i; sy[i] = my;
    grid[my][mx - i] = { ILI9341_GREEN, true };
  }
  dirX = 1; dirY = 0; // seteaza directia spre dreapta initial
  drawGrid();

  // plaseaza primul mar
  placeApple();
  score = 0;
}

int lastBatUpdate = -1;

void drawBattery() {
  // updateaza bateria doar odata la 1s ca sa nu fie flicker
  int now = millis();
  if (now - lastBatUpdate < 1000) {
    return;
  }

  lastBatUpdate = now;
  // citeste valoarea pe 12 biti de pe pinul de ADC
  // obtinem tensiunea la pinul ADC inmultind cu valoarea de referinta
  // apoi obtinem valoarea reala a tensiunii din BAT+ a TP4056 inmultind cu 2
  // pt ca avem divizor de tensiune cu 2 rezistenta egale
  // calculam procentul de baterie rotunjit la cel mai apropiat int
  int raw = analogRead(BAT_ADC_PIN);
  float m = (raw / 4095.0f) * 3.3f, v = m * 2.0f * BAT_CAL_FACTOR;
  int pct = constrain(int((v - BAT_EMPTY_VOLTAGE)
            / (BAT_FULL_VOLTAGE - BAT_EMPTY_VOLTAGE) * 100 + 0.5), 0, 100);
  tft.fillRect(0, 0, tft.width(), 20, ILI9341_BLACK);
  tft.setCursor(2,2);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.print(v,2);
  tft.print("V");
  tft.setCursor(tft.width()-50, 2);
  tft.print(pct);
  tft.print("%");
}

int lastScoreUpdate = -1;

void drawScore() {
  // la fel ca la baterie, updateaza scorul doar odata la 1s ca sa nu fie flicker
  int now = millis();
  if (now - lastScoreUpdate < 1000) {
    return;
  }

  lastScoreUpdate = now;

  tft.fillRect(0,20,tft.width(),20,ILI9341_BLACK);
  tft.setCursor(2,22); tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
  tft.print("Score: "); tft.print(score);
}

void displayGameOver(){
  // pentru fiecare din jocuri citim din fisierul aferent de pe cardul microSD
  // daca exista acel fisier atunci comparam cu ce a obtinut jucatorul
  // si aratam pe ecran
  // daca scorul obtinut e mai bun decat ceea ce stiam,
  // atunci salvam noul best score
  
  int oldHigh = 0;
  if (state == TETRIS) {
    const char * path = "/tetris_score.txt";
    if (SD.exists(path)) {
      File f = SD.open(path, FILE_READ);
      if (f) { oldHigh = f.parseInt(); f.close(); 
          Serial.println(oldHigh);}
    }
    if (!SD.exists(path) || score > oldHigh) {
      SD.remove(path);
      File f = SD.open(path, FILE_WRITE);
      if (f) { f.print(score); f.close(); }
      oldHigh = score;
    }
  } else {
    const char * path = "/snake_score.txt";
    if (SD.exists(path)) {
      File f = SD.open(path, FILE_READ);
      if (f) { oldHigh = f.parseInt(); f.close(); 
          Serial.println(oldHigh);}
    }
    if (!SD.exists(path) || score > oldHigh) {
      SD.remove(path);
      File f = SD.open(path, FILE_WRITE);
      if (f) { f.print(score); f.close(); }
      oldHigh = score;
    }
  }

  // arata cele 2 valori
  tft.fillScreen(ILI9341_BLACK);
  drawBattery();
  drawScore();
  tft.setTextSize(4); tft.setTextColor(ILI9341_RED);
  tft.setCursor(40, tft.height()/2 - 20); tft.print("You lost");
  tft.setTextSize(3); tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(40, tft.height()/2 + 20); tft.print("Score: "); tft.print(score);

  tft.setCursor(40, tft.height()/2 + 60); tft.print("Best: "); tft.print(oldHigh);
  // sunet trist...
  ledcWriteTone(BUZZER_PIN, NOTE_C5);
  delay(1000);
  ledcWriteTone(BUZZER_PIN, 0);

  state = MENU;
}
