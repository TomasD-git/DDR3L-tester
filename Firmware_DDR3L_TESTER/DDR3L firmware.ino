#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#define PIN_ROT_A      14   
#define PIN_ROT_B      15  
#define PIN_ROT_SW     18 
#define PIN_BTN_SEL    19   
#define PIN_BTN_ABORT  20   
#define PIN_SLIDE1     22  
#define PIN_SLIDE2     23   
#define PIN_WE         21   

#define SDA_LCD  6
#define SCL_LCD  7
#define SDA_SPD  4
#define SCL_SPD  5

TwoWire spdBus(1);   

U8G2_SSD1306_128X64_NONAME_F_HW_I2C lcd(
    U8G2_R0, U8X8_PIN_NONE, SCL_LCD, SDA_LCD);

#define SPD_EEPROM_ADDR 0x50
#define SPD_SIZE        256

#define SPD_DRAM_TYPE   2   
#define SPD_MODULE_TYPE 3   
#define SPD_DENSITY     4   
#define SPD_ADDRESSING  5   
#define SPD_VOLTAGE     6   
#define SPD_ORG         7   
#define SPD_BUS_WIDTH   8   
#define SPD_TCK_MIN     12 
#define SPD_CL_LSB      14 
#define SPD_CL_MSB      15  
#define SPD_TAA_MIN     16  
#define SPD_TWR_MIN     17 
#define SPD_TRCD_MIN    18 
#define SPD_TRRD_MIN    19 
#define SPD_TRP_MIN     20  
#define SPD_TRAS_UPP    21 
#define SPD_TRAS_LSB    22 
#define SPD_TRC_LSB     23 
#define SPD_MFR_LSB     117
#define SPD_MFR_MSB     118
#define SPD_PART_NUM    128 

uint8_t spdOrig[SPD_SIZE];   
uint8_t spdEdit[SPD_SIZE];  

struct Param {
  const char *label;   
  uint8_t     idx;     
  uint8_t     minV;
  uint8_t     maxV;
  uint8_t     step;
};

static const Param PARAMS[] = {
  { "tCK (spd)",  SPD_TCK_MIN,   8,  30,  1 }, 
  { "tAA  (CL)",  SPD_TAA_MIN,   5,  30,  1 },
  { "tRCD",       SPD_TRCD_MIN,  5,  30,  1 },
  { "tRP",        SPD_TRP_MIN,   5,  30,  1 },
  { "tRAS",       SPD_TRAS_LSB, 20, 200,  2 },
  { "tRC",        SPD_TRC_LSB,  20, 255,  2 },
  { "tRRD",       SPD_TRRD_MIN,  4,  30,  1 },
  { "tWR",        SPD_TWR_MIN,   5,  30,  1 },
};
static const int N_PARAMS = (int)(sizeof(PARAMS) / sizeof(PARAMS[0]));

volatile int rotPos      = 0;
volatile int lastEncoded = 0;

void IRAM_ATTR rotISR() {
  int a   = digitalRead(PIN_ROT_A);
  int b   = digitalRead(PIN_ROT_B);
  int enc = (a << 1) | b;
  int sum = (lastEncoded << 2) | enc;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) rotPos++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) rotPos--;
  lastEncoded = enc;
}

struct Button {
  uint8_t pin;
  bool    lastState    = true;  
  bool    wasPressed   = false;  
  unsigned long holdStart = 0;
  void update() {
    bool cur = (digitalRead(pin) == LOW);
    wasPressed = cur && !lastState;
    if (wasPressed) holdStart = millis();
    lastState  = cur;
  }
  bool heldFor(uint32_t ms) const {
    return (digitalRead(pin) == LOW) && holdStart && (millis() - holdStart >= ms);
  }
  bool isDown() const { return digitalRead(pin) == LOW; }
};

Button btnSel   = { PIN_BTN_SEL   };
Button btnAbort = { PIN_BTN_ABORT };
Button btnRot   = { PIN_ROT_SW   };

static int lastRotPos = 0;
int rotDelta() {
  int d = rotPos - lastRotPos;
  lastRotPos = rotPos;
  return d;
}

uint16_t tckToMTps(uint8_t tck) {
  return tck ? (uint16_t)(16000u / tck) : 0;
}

uint16_t calcCapMB(const uint8_t *s) {
  static const uint16_t dieMb[] = { 256, 512, 1024, 2048, 4096, 8192 };
  uint8_t dieIdx = s[SPD_DENSITY] & 0x0F;
  if (dieIdx < 1 || dieIdx > 6) return 0;
  uint16_t dieCap = dieMb[dieIdx - 1];
  uint8_t devW  = 4u << (s[SPD_ORG]       & 0x07);  
  uint8_t ranks = 1u + ((s[SPD_ORG] >> 3) & 0x07);
  uint8_t busW  = 8u << (s[SPD_BUS_WIDTH] & 0x07);   
  return (uint32_t)dieCap * busW / devW * ranks / 8;
}

void getPartNum(const uint8_t *s, char *buf, size_t bufLen) {
  size_t n = min((size_t)18, bufLen - 1);
  for (size_t i = 0; i < n; i++) {
    char c = (char)s[SPD_PART_NUM + i];
    buf[i] = (c >= 0x20 && c < 0x7F) ? c : ' ';
  }
  int end = (int)n - 1;
  while (end >= 0 && buf[end] == ' ') end--;
  buf[end + 1] = '\0';
}

bool spdDetect() {
  spdBus.beginTransmission(SPD_EEPROM_ADDR);
  return spdBus.endTransmission() == 0;
}

bool spdRead(uint8_t *buf) {
  for (int pg = 0; pg < 2; pg++) {
    spdBus.beginTransmission(0x36 + pg);
    if (spdBus.endTransmission() != 0) return false;
    delayMicroseconds(500);
    spdBus.beginTransmission(SPD_EEPROM_ADDR);
    spdBus.write((uint8_t)0x00);
    if (spdBus.endTransmission(false) != 0) return false;
    uint8_t got = spdBus.requestFrom((uint8_t)SPD_EEPROM_ADDR, (uint8_t)128);
    if (got < 128) return false;
    for (int i = 0; i < 128; i++) buf[pg * 128 + i] = spdBus.read();
  }
  spdBus.beginTransmission(0x36);
  spdBus.endTransmission();
  return (buf[SPD_DRAM_TYPE] == 0x0B);   
}

bool spdWriteChanges(const uint8_t *orig, const uint8_t *edit) {
  spdBus.beginTransmission(0x36);
  spdBus.endTransmission();
  delay(2);
  for (int i = 0; i < 128; i++) {
    if (edit[i] == orig[i]) continue;
    spdBus.beginTransmission(SPD_EEPROM_ADDR);
    spdBus.write((uint8_t)i);
    spdBus.write(edit[i]);
    if (spdBus.endTransmission() != 0) return false;
    delay(6);   
  }
  return true;
}
#define FONT_MD  u8g2_font_6x10_tf    
#define FONT_LG  u8g2_font_10x20_tf  
#define FONT_XS  u8g2_font_5x7_tf    
void drawHeader(const char *title) {
  lcd.setFont(FONT_MD);
  lcd.drawStr(0, 10, title);
  lcd.drawHLine(0, 12, 128);
}

void screenNoDDR() {
  lcd.clearBuffer();
  drawHeader("DDR3L SPD Tool");
  lcd.setFont(FONT_MD);
  lcd.drawStr(10, 32, "No module found.");
  lcd.drawStr(8,  46, "Insert DDR3L...");
  lcd.sendBuffer();
}
void screenReading() {
  lcd.clearBuffer();
  drawHeader("DDR3L SPD Tool");
  lcd.setFont(FONT_MD);
  lcd.drawStr(18, 40, "Reading SPD...");
  lcd.sendBuffer();
}
void screenInfo(const uint8_t *s) {
  lcd.clearBuffer();
  drawHeader("DDR3L Module Info");
  char buf[24];
  lcd.setFont(FONT_MD);
  snprintf(buf, sizeof(buf), "Speed: %5u MT/s", tckToMTps(s[SPD_TCK_MIN]));
  lcd.drawStr(0, 25, buf);
  snprintf(buf, sizeof(buf), "Size : %5u MB",   calcCapMB(s));
  lcd.drawStr(0, 36, buf);
  snprintf(buf, sizeof(buf), "CL %2u  tRCD %2u  tRP %2u",
           s[SPD_TAA_MIN], s[SPD_TRCD_MIN], s[SPD_TRP_MIN]);
  lcd.drawStr(0, 47, buf);
  lcd.setFont(FONT_XS);
  const char *vlt = (s[SPD_VOLTAGE] & 0x02) ? "DDR3L 1.35V" : "DDR3  1.5V ";
  snprintf(buf, sizeof(buf), "%s  Slide>edit", vlt);
  lcd.drawStr(0, 62, buf);
  lcd.sendBuffer();
}
void screenEdit(int selParam) {
  lcd.clearBuffer();
  drawHeader("Edit  [ROT=val BTN1=param]");
  lcd.setFont(FONT_MD);
  int startIdx = constrain(selParam - 1, 0, N_PARAMS - 4);
  for (int i = startIdx; i < min(startIdx + 4, N_PARAMS); i++) {
    int  y   = 24 + (i - startIdx) * 11;
    bool sel = (i == selParam);
    if (sel) {
      lcd.drawBox(0, y - 9, 128, 11);
      lcd.setDrawColor(0);
    }
    char line[24];
    bool changed = (spdEdit[PARAMS[i].idx] != spdOrig[PARAMS[i].idx]);
    snprintf(line, sizeof(line), "%-11s %3d%s",
             PARAMS[i].label,
             spdEdit[PARAMS[i].idx],
             changed ? "*" : " ");
    lcd.drawStr(1, y, line);
    lcd.setDrawColor(1);
  }
  lcd.setFont(FONT_XS);
  lcd.drawStr(0, 63, "PRESS:edit val  HOLD 3s:write");
  lcd.sendBuffer();
}
void screenValue(int selParam) {
  lcd.clearBuffer();
  lcd.setFont(FONT_MD);
  lcd.drawStr(0, 10, PARAMS[selParam].label);
  lcd.drawHLine(0, 12, 128);
  uint8_t v = spdEdit[PARAMS[selParam].idx];
  char    buf[16];
  lcd.setFont(FONT_LG);
  snprintf(buf, sizeof(buf), "%3d", v);
  lcd.drawStr(34, 46, buf);
  lcd.setFont(FONT_XS);
  if (PARAMS[selParam].idx == SPD_TCK_MIN) {
    snprintf(buf, sizeof(buf), "= %u MT/s", tckToMTps(v));
    lcd.drawStr(36, 60, buf);
  } else {
    snprintf(buf, sizeof(buf), "orig: %d", spdOrig[PARAMS[selParam].idx]);
    lcd.drawStr(36, 60, buf);
  }
  lcd.drawStr(0, 63, "ROT:chg  PRESS:ok  BTN1:nxt");
  lcd.sendBuffer();
}
void screenConfirm() {
  lcd.clearBuffer();
  drawHeader("Confirm Write");
  lcd.setFont(FONT_MD);
  lcd.drawStr(4, 28, "Hold ROT 3s to write.");
  lcd.drawStr(4, 42, "Hold BTN2   to abort.");
  lcd.setFont(FONT_XS);
  lcd.drawStr(0, 63, "Changes marked with * will save");
  lcd.sendBuffer();
}
void screenWriting() {
  lcd.clearBuffer();
  drawHeader("Writing SPD...");
  lcd.setFont(FONT_MD);
  lcd.drawStr(10, 36, "Do not remove");
  lcd.drawStr(18, 50, "the module!");
  lcd.sendBuffer();
}
void screenDone(bool ok) {
  lcd.clearBuffer();
  drawHeader(ok ? "Write OK!" : "Write Failed!");
  lcd.setFont(FONT_MD);
  if (ok) {
    lcd.drawStr(10, 34, "SPD updated.");
    lcd.drawStr(4,  50, "Reinsert module.");
  } else {
    lcd.drawStr(4,  34, "Check connection");
    lcd.drawStr(4,  50, "and try again.");
  }
  lcd.sendBuffer();
}
void screenError() {
  lcd.clearBuffer();
  drawHeader("Read Error");
  lcd.setFont(FONT_MD);
  lcd.drawStr(8, 34, "SPD read failed!");
  lcd.drawStr(4, 50, "Check module.");
  lcd.sendBuffer();
}
enum State {
  S_NO_DDR,
  S_READING,
  S_INFO,
  S_EDIT,
  S_VALUE,
  S_WRITE_CONFIRM,
  S_WRITING,
  S_ERROR
};
State state     = S_NO_DDR;
int   curParam  = 0;

//     SETUP
void setup() {
  Serial.begin(115200);
  pinMode(PIN_ROT_A,     INPUT_PULLUP);
  pinMode(PIN_ROT_B,     INPUT_PULLUP);
  pinMode(PIN_ROT_SW,    INPUT_PULLUP);
  pinMode(PIN_BTN_SEL,   INPUT_PULLUP);
  pinMode(PIN_BTN_ABORT, INPUT_PULLUP);
  pinMode(PIN_SLIDE1,    INPUT_PULLUP);
  pinMode(PIN_SLIDE2,    INPUT_PULLUP);
  pinMode(PIN_WE,        OUTPUT);
  digitalWrite(PIN_WE,   HIGH);  
  attachInterrupt(digitalPinToInterrupt(PIN_ROT_A), rotISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ROT_B), rotISR, CHANGE);
  spdBus.begin(SDA_SPD, SCL_SPD, 100000UL);
  lcd.begin();
  lcd.setContrast(200);
  screenNoDDR();
}

//            LOOP

void loop() {
  btnSel.update();
  btnAbort.update();
  btnRot.update();
  int rd = rotDelta();
  static State        prevState    = (State)255;
  static unsigned long pollTimer   = 0;
  bool redraw = (state != prevState);
  prevState   = state;
  auto editUnlocked = []() { return digitalRead(PIN_SLIDE2) == LOW; };
  switch (state) {
    case S_NO_DDR:
      if (redraw) screenNoDDR();
      if (millis() - pollTimer > 500) {
        pollTimer = millis();
        if (spdDetect()) state = S_READING;
      }
      break;
    case S_READING:
      screenReading();
      delay(30);
      if (spdRead(spdOrig)) {
        memcpy(spdEdit, spdOrig, SPD_SIZE);
        curParam = 0;
        state    = S_INFO;
      } else {
        state = S_ERROR;
      }
      break;
    case S_INFO:
      if (redraw) screenInfo(spdOrig);
      if (millis() - pollTimer > 1000) {
        pollTimer = millis();
        if (!spdDetect()) { state = S_NO_DDR; break; }
      }
      if (editUnlocked() && btnRot.wasPressed) {
        memcpy(spdEdit, spdOrig, SPD_SIZE);   
        curParam = 0;
        state    = S_EDIT;
      }
      break;
    case S_EDIT:
      if (!editUnlocked()) { state = S_INFO; break; }   
      if (rd != 0) {
        curParam = constrain(curParam + (rd > 0 ? 1 : -1), 0, N_PARAMS - 1);
        redraw   = true;
      }
      if (btnSel.wasPressed) {      
        curParam = (curParam + 1) % N_PARAMS;
        redraw   = true;
      }
      if (btnRot.wasPressed) {        
        state = S_VALUE;
        break;
      }
      if (btnRot.heldFor(3000)) {     
        state = S_WRITE_CONFIRM;
        break;
      }
      if (redraw) screenEdit(curParam);
      break;
    case S_VALUE:
      if (!editUnlocked()) { state = S_INFO; break; }
      if (rd != 0) {
        int v = (int)spdEdit[PARAMS[curParam].idx] + rd * (int)PARAMS[curParam].step;
        spdEdit[PARAMS[curParam].idx] =
            (uint8_t)constrain(v, PARAMS[curParam].minV, PARAMS[curParam].maxV);
        redraw = true;
      }
      if (btnSel.wasPressed) {       
        curParam = (curParam + 1) % N_PARAMS;
        redraw   = true;
      }
      if (btnRot.wasPressed) {       
        state = S_EDIT;
        break;
      }
      if (btnRot.heldFor(3000)) {   
        state = S_WRITE_CONFIRM;
        break;
      }
      if (redraw) screenValue(curParam);
      break;
    case S_WRITE_CONFIRM: {
      if (redraw) screenConfirm();
      if (btnAbort.heldFor(800)) {
        state = S_EDIT;
        break;
      }
      static unsigned long freshHoldStart = 0;
      static bool          waitRelease    = true;
      if (redraw) { freshHoldStart = 0; waitRelease = true; }
      bool rotDown = btnRot.isDown();
      if (waitRelease) {
        if (!rotDown) waitRelease = false;   
      } else {
        if (rotDown && freshHoldStart == 0)  freshHoldStart = millis();
        if (!rotDown)                        freshHoldStart = 0;
        if (freshHoldStart && millis() - freshHoldStart >= 3000) {
          freshHoldStart = 0;
          state          = S_WRITING;
        }
      }
      break;
    }
    case S_WRITING: {
      screenWriting();
      digitalWrite(PIN_WE, LOW);   
      delay(10);
      bool ok = spdWriteChanges(spdOrig, spdEdit);
      delay(10);
      digitalWrite(PIN_WE, HIGH);   
      if (ok) memcpy(spdOrig, spdEdit, SPD_SIZE); 
      screenDone(ok);
      delay(3500);
      state = S_INFO;
      break;
    }
    case S_ERROR:
      if (redraw) screenError();
      if (millis() - pollTimer > 2500) {
        pollTimer = millis();
        state     = S_NO_DDR;
      }
      break;
  }
  delay(8);   
}
