#include <Keypad.h>
#include <Wire.h> 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <algorithm>

// =====================================================
// 1. HARDWARE CONFIGURATION
// =====================================================
#define BIT(i) (1U << (i))

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int ledColPins[3] = {15, 2, 4}; 
const int ledGreenPins[3] = {13, 12, 14}; 
const int ledRedPins[3]   = {5, 18, 19}; 
const byte ROWS = 4; 
const byte COLS = 3; 
char hexaKeys[ROWS][COLS] = {{'7','4','1'},{'8','5','2'},{'9','6','3'},{'*','0','#'}};
byte rowPins[ROWS] = {27, 26, 25, 33}; 
byte colPins[COLS] = {32, 16, 17};      
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// =====================================================
// 2. GLOBALS & DATA
// =====================================================
uint16_t currentGreenMask = 0, currentRedMask = 0;
uint16_t patternMask[9], trapMasks[4], fragMask[3][2];
int patterns[9][9] = {
  {0,1,0, 1,0,1, 0,1,0}, {0,0,1, 0,1,0, 1,0,1}, {0,0,1, 1,1,0, 0,1,0},
  {1,1,0, 1,1,0, 0,0,0}, {0,1,1, 1,0,0, 1,0,0}, {0,1,0, 0,1,0, 1,0,1},
  {0,1,0, 1,1,0, 1,0,0}, {0,1,0, 0,1,1, 0,1,0}, {1,0,0, 1,0,1, 0,1,0}
};
uint16_t mem3Frag[6][2];
char mem3FragKey[6];
struct MemStep { uint16_t mask; char expectedKey; };
float rt_std_dev = 0.0f;
float finalStressPercentage = 0.0f;
static int  streak = 0;

// =====================================================
// 3. CORE ENGINES & HARDWARE HELPERS
// =====================================================

int pop9(uint16_t m){ return __builtin_popcount(m & 0x1FF); }
int ham(uint16_t a,uint16_t b){ return pop9((a^b) & 0x1FF); }

int manh(int i,int j){
  int r1=i/3,c1=i%3,r2=j/3,c2=j%3;
  int dr=r1-r2; if(dr<0) dr=-dr;
  int dc=c1-c2; if(dc<0) dc=-dc;
  return dr+dc;
}

uint16_t mirrorLR(uint16_t x){
  uint16_t o=0;
  for(int r=0;r<3;r++) for(int c=0;c<3;c++)
    if(x & (1U<<(r*3+c))) o |= (1U<<(r*3 + (2-c)));
  return o;
}
uint16_t mirrorTB(uint16_t x){
  uint16_t o=0;
  for(int r=0;r<3;r++) for(int c=0;c<3;c++)
    if(x & (1U<<(r*3+c))) o |= (1U<<((2-r)*3 + c));
  return o;
}
uint16_t rot180(uint16_t x){
  uint16_t o=0;
  for(int r=0;r<3;r++) for(int c=0;c<3;c++)
    if(x & (1U<<(r*3+c))) o |= (1U<<((2-r)*3 + (2-c)));
  return o;
}

bool isSym(uint16_t x){ return x==mirrorLR(x) || x==mirrorTB(x) || x==rot180(x); }

bool isConn(uint16_t x){
  int t = pop9(x);
  if(!t) return false;
  bool v[9]={0}; int st[9], sp=0;
  for(int i=0;i<9;i++) if(x&(1U<<i)){ v[i]=1; st[sp++]=i; break; }
  while(sp){
    int i = st[--sp], r=i/3, c=i%3;
    if(r>0){ int n=(r-1)*3+c; if((x&(1U<<n))&&!v[n]) v[n]=1, st[sp++]=n; }
    if(r<2){ int n=(r+1)*3+c; if((x&(1U<<n))&&!v[n]) v[n]=1, st[sp++]=n; }
    if(c>0){ int n=r*3+c-1; if((x&(1U<<n))&&!v[n]) v[n]=1, st[sp++]=n; }
    if(c<2){ int n=r*3+c+1; if((x&(1U<<n))&&!v[n]) v[n]=1, st[sp++]=n; }
  }
  int s=0; for(int i=0;i<9;i++) if(v[i]) s++;
  return s==t;
}

bool isEasy(uint16_t x){ return isConn(x) || isSym(x); }
bool isTough(uint16_t x){ return (!isEasy(x)) && pop9(x)>=5; }

void refreshMatrix() {
  for (int c = 0; c < 3; c++) {
    digitalWrite(ledColPins[c], LOW);
    for (int r = 0; r < 3; r++) {
      int idx = (r * 3) + c;
      digitalWrite(ledGreenPins[r], (currentGreenMask & BIT(idx)) ? HIGH : LOW);
      digitalWrite(ledRedPins[r], (currentRedMask & BIT(idx)) ? HIGH : LOW);
    }
    delayMicroseconds(500);
    digitalWrite(ledColPins[c], HIGH);
    for(int r=0; r<3; r++) { digitalWrite(ledGreenPins[r], LOW); digitalWrite(ledRedPins[r], LOW); }
  }
}

bool waitAndRefresh(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (customKeypad.getKey() == '*') return true;
    refreshMatrix();
  }
  return false;
}

void turnOffAllLEDs() { currentGreenMask = 0; currentRedMask = 0; }
void showMaskGreen(uint16_t m) { currentGreenMask = m; }
void showMaskRed(uint16_t m)   { currentRedMask = m; }

char getKeyWithRefresh() { char k = customKeypad.getKey(); refreshMatrix(); return k; }

// =====================================================
// 4. TRANSFORMATIONS
// =====================================================
uint16_t rotateCW(uint16_t m)  { uint16_t o=0; for(int r=0;r<3;r++) for(int c=0;c<3;c++) if(m&BIT(r*3+c)) o|=BIT(c*3+(2-r)); return o; }
uint16_t rotateCCW(uint16_t m) { uint16_t o=0; for(int r=0;r<3;r++) for(int c=0;c<3;c++) if(m&BIT(r*3+c)) o|=BIT((2-c)*3+r); return o; }
uint16_t mirrorH(uint16_t m)   { uint16_t o=0; for(int r=0;r<3;r++){ if(m&BIT(r*3)) o|=BIT(r*3+2); if(m&BIT(r*3+1)) o|=BIT(r*3+1); if(m&BIT(r*3+2)) o|=BIT(r*3); } return o; }
uint16_t mirrorV(uint16_t m)   { uint16_t o=0; for(int c=0;c<3;c++){ if(m&BIT(c)) o|=BIT(6+c); if(m&BIT(3+c)) o|=BIT(3+c); if(m&BIT(6+c)) o|=BIT(c); } return o; }
uint16_t applyTransform(uint16_t m, int t) { switch(t){ case 0: return mirrorH(m); case 1: return mirrorV(m); case 2: return rotateCW(m); case 3: return rotateCCW(m); default: return m; } }
const char* getCmdName(int t) { 
  static const char* n[] = { "V Mirror", "H Mirror", "Rot 90 CW", "Rot 90 CCW" }; 
  return n[t]; 
}

// =====================================================
// 5. MEMORY EVALUATION ENGINE (Relational Cognitive)
// =====================================================
// Edit operation type codes (uint8_t) — avoids Arduino IDE enum/prototype ordering bug
#define OP_MATCH     ((uint8_t)0)
#define OP_MINOR_SUB ((uint8_t)1)
#define OP_MAJOR_SUB ((uint8_t)2)
#define OP_DELETE    ((uint8_t)3)
#define OP_INSERT    ((uint8_t)4)
#define OP_TRANSPOSE ((uint8_t)5)
typedef uint8_t EditOp;

// Integer costs (scaled x10) to keep DP table in ints (avoids float DP errors)
#define DL_MINOR 5
#define DL_MAJOR 10
#define DL_DEL   8
#define DL_INS   9
#define DL_TRANS 6

// Restricted Damerau-Levenshtein with full traceback.
// tgt/tLen = target (correct) sequence; inp/iLen = player input sequence.
// script[] is filled with the edit operations in forward order.
// Returns the number of operations written into script[].
int memDL(char* tgt, int tLen, char* inp, int iLen, uint8_t* script) {
  static int dp[26][26];
  for (int i = 0; i <= tLen; i++) dp[i][0] = i * DL_DEL;
  for (int j = 0; j <= iLen; j++) dp[0][j] = j * DL_INS;
  for (int i = 1; i <= tLen; i++) {
    for (int j = 1; j <= iLen; j++) {
      int d  = abs(tgt[i-1] - inp[j-1]);
      int sc = (d == 0) ? 0 : (d == 1) ? DL_MINOR : DL_MAJOR;
      dp[i][j] = dp[i-1][j-1] + sc;
      if (dp[i-1][j] + DL_DEL < dp[i][j]) dp[i][j] = dp[i-1][j] + DL_DEL;
      if (dp[i][j-1] + DL_INS < dp[i][j]) dp[i][j] = dp[i][j-1] + DL_INS;
      if (i > 1 && j > 1 && tgt[i-1] == inp[j-2] && tgt[i-2] == inp[j-1])
        if (dp[i-2][j-2] + DL_TRANS < dp[i][j]) dp[i][j] = dp[i-2][j-2] + DL_TRANS;
    }
  }
  // Traceback: build reverse edit script in tmp[], then flip into script[]
  uint8_t tmp[52]; int tl = 0;
  int i = tLen, j = iLen;
  while (i > 0 || j > 0) {
    // Transposition has highest check priority
    if (i > 1 && j > 1 && tgt[i-1] == inp[j-2] && tgt[i-2] == inp[j-1]
        && dp[i][j] == dp[i-2][j-2] + DL_TRANS) {
      tmp[tl++] = OP_TRANSPOSE; tmp[tl++] = OP_TRANSPOSE; // 2 entries per pair
      i -= 2; j -= 2; continue;
    }
    if (i > 0 && j > 0) {
      int d  = abs(tgt[i-1] - inp[j-1]);
      int sc = (d == 0) ? 0 : (d == 1) ? DL_MINOR : DL_MAJOR;
      if (dp[i][j] == dp[i-1][j-1] + sc) {
        tmp[tl++] = (d == 0) ? OP_MATCH : (d == 1) ? OP_MINOR_SUB : OP_MAJOR_SUB;
        i--; j--; continue;
      }
    }
    if (i > 0 && dp[i][j] == dp[i-1][j] + DL_DEL) {
      tmp[tl++] = OP_DELETE; i--; continue;
    }
    if (j > 0) { tmp[tl++] = OP_INSERT; j--; continue; }
    if (i > 0) { tmp[tl++] = OP_DELETE; i--; } // safety fallback
  }
  for (int k = 0; k < tl; k++) script[k] = tmp[tl - 1 - k];
  return tl;
}

// Classify the dominant cognitive error type for an error zone.
// (mat=matches, mnr=minor subs, maj=major subs, del=deletions, ins=insertions, trn=transposition pairs)
const char* memClassifyZone(int mat, int mnr, int maj, int del, int ins, int trn) {
  if (mat + mnr + maj + del + ins + trn == 0) return "";
  if (trn > 0  && del == 0 && ins == 0 && maj == 0)    return "Index Swap";
  if (del == 1 && (mat + mnr) > (maj + ins))            return "Chain Shift";
  if ((maj + ins) > (mat + mnr + trn))                  return "Collapse";
  if (mnr == 1 && maj == 0 && del == 0 && ins == 0 && trn == 0) return "Minor Alt";
  if (mat > 0)                                          return "Partial";
  return "Collapse";
}

// Core evaluator: aligns target vs input using D-L, counts edit operations,
// then computes Storage Strength and Order Integrity percentages.
// Also writes a zone label (caller must supply >= 16 bytes for zoneLabelOut).
void evalMemSeq(char* tgt, int tLen, char* inp, int iLen,
                float& ss, float& oi, char* zoneLabelOut) {
  const char* src;
  int lbl = 0;
  if (tLen == 0) {
    ss = 0; oi = 0; zoneLabelOut[0] = '\0'; return;
  }
  if (iLen == 0) {
    ss = 0; oi = 0; src = "Collapse";
    while (src[lbl]) { zoneLabelOut[lbl] = src[lbl]; lbl++; }
    zoneLabelOut[lbl] = '\0'; return;
  }
  uint8_t script[52];
  int sLen = memDL(tgt, tLen, inp, iLen, script);
  int mat = 0, mnr = 0, maj = 0, del = 0, ins = 0, trn = 0;
  for (int k = 0; k < sLen; k++) {
    switch (script[k]) {
      case OP_MATCH:      mat++; break;
      case OP_MINOR_SUB:  mnr++; break;
      case OP_MAJOR_SUB:  maj++; break;
      case OP_DELETE:     del++; break;
      case OP_INSERT:     ins++; break;
      case OP_TRANSPOSE:  trn++; break;
    }
  }
  trn /= 2; // Each transposition pair emits 2 OP_TRANSPOSE entries

  // Storage Strength: measures content identity recall.
  // Transpositions do NOT reduce storage (content exists, just reordered).
  float ssCredit = (mat + 0.5f * mnr + 0.9f * trn) / (float)tLen;
  float ssDebit  = (maj * 1.0f + del * 0.8f)        / (float)tLen;
  ss = constrain((ssCredit - ssDebit) * 100.0f, 0.0f, 100.0f);

  // Order Integrity: measures positional correctness of the sequence.
  // Only exact matches contribute positively; swaps and shifts penalize.
  float oiNum = (float)mat - trn * 0.7f - (del + ins) * 0.3f - mnr * 0.1f;
  oi = constrain((oiNum / (float)tLen) * 100.0f, 0.0f, 100.0f);

  // Dominant error label (overall, not per-zone — suitable for OLED display)
  src = memClassifyZone(mat, mnr, maj, del, ins, trn);
  lbl = 0;
  while (src[lbl]) { zoneLabelOut[lbl] = src[lbl]; lbl++; }
  zoneLabelOut[lbl] = '\0';
}

// =====================================================
// 5b. MEMORY MODE (Relational Cognitive Evaluation)
// =====================================================
void runMemoryMode(int stage) {
  display.clearDisplay(); display.setTextSize(2); display.setCursor(15, 20);
  display.print("MEM STAGE "); display.print(stage); display.display();
  if (waitAndRefresh(1500)) return;

  MemStep sequence[25]; char targets[25]; char enteredChars[25];
  int rounds = (stage <= 3) ? 3 : 1;

  // Welford online variance for Stability computation across rounds
  float meanSS = 0.0f, m2SS = 0.0f, meanOI = 0.0f, m2OI = 0.0f;
  int nRounds = 0;

  for (int round = 1; round <= rounds; round++) {
    display.clearDisplay(); display.setTextSize(2); display.setCursor(20, 20);
    display.print("ROUND "); display.print(round); display.display();
    if (waitAndRefresh(1200)) return;

    int seqLen = 0; int tIdx = 0;
    for (int i = 0; i < 25; i++) enteredChars[i] = 0;

    if (stage == 0) {
      seqLen = 5 + round; // 6, 7, 8
      for (int i = 0; i < seqLen; i++) {
        int p = random(0, 6); sequence[i] = { patternMask[p], (char)('1' + p) };
      }
    } else if (stage == 1) {
      seqLen = 6 + round; // 7, 8, 9
      for (int i = 0; i < seqLen; i++) {
        int p = random(0, 9); sequence[i] = { patternMask[p], (char)('1' + p) };
      }
    } else if (stage == 2) {
      seqLen = 9 + round; // 10, 11, 12
      int traps = random(2, 4); int greens = seqLen - traps; int idx = 0;
      for (int i = 0; i < greens; i++) {
        int p = random(0, 9); sequence[idx++] = { patternMask[p], (char)('1' + p) };
      }
      for (int i = 0; i < traps; i++) sequence[idx++] = { trapMasks[random(0, 4)], 0 };
      for (int i = 0; i < seqLen; i++) {
        int n = random(0, seqLen); MemStep t = sequence[i]; sequence[i] = sequence[n]; sequence[n] = t;
      }
    } else if (stage == 3) {
      seqLen = 11 + round; // 12, 13, 14
      int trapCount  = random(2, 4);
      int fragCount  = random(2, 5);
      int normalCount = seqLen - trapCount - (2 * fragCount);
      if (normalCount < 0) normalCount = 0;
      int idx = 0;
      for (int i = 0; i < normalCount; i++) {
        int p = random(0, 9); sequence[idx++] = { patternMask[p], (char)('1' + p) };
      }
      for (int i = 0; i < trapCount; i++) sequence[idx++] = { trapMasks[random(0, 4)], 0 };
      int fragId[25]; for (int i = 0; i < 25; i++) fragId[i] = -1;
      for (int f = 0; f < fragCount; f++) {
        int pick = random(0, 6); char k = mem3FragKey[pick];
        sequence[idx] = { mem3Frag[pick][0], k }; fragId[idx] = f; idx++;
        sequence[idx] = { mem3Frag[pick][1], 0 }; fragId[idx] = f; idx++;
      }
      seqLen = idx;
      for (int attempt = 0; attempt < 60; attempt++) {
        for (int i = 0; i < seqLen; i++) {
          int n = random(0, seqLen);
          MemStep ts = sequence[i]; sequence[i] = sequence[n]; sequence[n] = ts;
          int tf = fragId[i];   fragId[i]   = fragId[n];   fragId[n]   = tf;
        }
        bool ok = true;
        for (int i = 0; i < seqLen - 1; i++)
          if (fragId[i] != -1 && fragId[i] == fragId[i+1]) ok = false;
        if (ok) break;
      }
    }

    for (int i = 0; i < seqLen; i++)
      if (sequence[i].expectedKey != 0) targets[tIdx++] = sequence[i].expectedKey;

    // Show sequence on LED matrix
    for (int i = 0; i < seqLen; i++) {
      turnOffAllLEDs(); if (waitAndRefresh(200)) return;
      showMaskGreen(sequence[i].mask); if (waitAndRefresh(800)) return;
      turnOffAllLEDs();
    }

    // Timed recall input phase
    unsigned long recallDurations[] = {5000, 7000, 8000, 10000};
    unsigned long totalTime = recallDurations[stage];
    unsigned long startTime = millis();
    int currentInputIdx = 0; int lastBarPixels = -1;

    display.clearDisplay(); display.setTextSize(1);
    display.setCursor(0, 0); display.print("Recall Sequence:");
    display.drawRect(0, 50, 128, 12, SSD1306_WHITE);

    while (currentInputIdx < tIdx) {
      unsigned long elapsed = millis() - startTime;
      if (elapsed >= totalTime) break;
      int barPixels = (int)(124.0 * (totalTime - elapsed) / totalTime);
      if (abs(barPixels - lastBarPixels) >= 2) {
        display.fillRect(2, 52, 124, 8, SSD1306_BLACK);
        display.fillRect(2, 52, barPixels, 8, SSD1306_WHITE);
        display.fillRect(0, 15, 128, 30, SSD1306_BLACK);
        display.setCursor(0, 20); display.setTextSize((tIdx <= 10) ? 2 : 1);
        for (int i = 0; i < tIdx; i++) {
          if (i < currentInputIdx) display.print(enteredChars[i]); else display.print('_');
        }
        display.display(); lastBarPixels = barPixels;
      }
      char k = customKeypad.getKey();
      if (k) {
        if (k == '*') return;
        if (k >= '1' && k <= '9') {
          int ledIdx = k - '1';
          currentGreenMask |= BIT(ledIdx);
          enteredChars[currentInputIdx] = k;
          currentInputIdx++;
          unsigned long flashStart = millis();
          while (millis() - flashStart < 100) refreshMatrix();
          currentGreenMask &= ~BIT(ledIdx);
          lastBarPixels = -1;
        }
      }
      refreshMatrix();
    }

    // Relational cognitive evaluation
    float ss = 0.0f, oi = 0.0f; char zoneLabel[16];
    evalMemSeq(targets, tIdx, enteredChars, currentInputIdx, ss, oi, zoneLabel);

    // Welford online update for Stability
    nRounds++;
    float dSS = ss - meanSS; meanSS += dSS / nRounds; m2SS += dSS * (ss - meanSS);
    float dOI = oi - meanOI; meanOI += dOI / nRounds; m2OI += dOI * (oi - meanOI);

    // Per-round result screen
    display.clearDisplay(); display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(currentInputIdx < tIdx ? "TIMEOUT!" : "ROUND DONE");
    display.setCursor(0, 16); display.print("Storage: ");
    display.setCursor(60, 16); display.print((int)(ss + 0.5f)); display.print("%");
    display.setCursor(0, 30); display.print("Order:   ");
    display.setCursor(60, 30); display.print((int)(oi + 0.5f)); display.print("%");
    display.setCursor(0, 48); display.print(zoneLabel);
    display.display(); if (waitAndRefresh(2500)) return;
  }

  // Stability: based on Welford variance across all rounds
  float sdSS  = sqrt((nRounds > 1) ? m2SS / (nRounds - 1) : 0.0f);
  float sdOI  = sqrt((nRounds > 1) ? m2OI / (nRounds - 1) : 0.0f);
  float stabSS = constrain(100.0f * (1.0f - sdSS / 30.0f), 0.0f, 100.0f);
  float stabOI = constrain(100.0f * (1.0f - sdOI / 30.0f), 0.0f, 100.0f);
  float stability = 0.5f * stabSS + 0.5f * stabOI;
  float overall   = (meanSS + meanOI + stability) / 3.0f;

  // Stage-end summary screen
  display.clearDisplay(); display.setTextSize(1);
  display.setCursor(0, 0);  display.print("STAGE COMPLETE");
  display.setCursor(0, 12); display.print("Storage: ");
  display.setCursor(60, 12); display.print((int)(meanSS   + 0.5f)); display.print("%");
  display.setCursor(0, 24); display.print("Order:   ");
  display.setCursor(60, 24); display.print((int)(meanOI   + 0.5f)); display.print("%");
  display.setCursor(0, 36); display.print("Stability:");
  display.setCursor(60, 36); display.print((int)(stability + 0.5f)); display.print("%");
  display.setCursor(0, 52);
  if      (overall >= 90) display.print("PERFECT MEMORY!");
  else if (overall >= 75) display.print("EXCELLENT WORK!");
  else if (overall >= 55) display.print("GOOD JOB!");
  else                    display.print("TRAIN MORE!");
  display.display(); if (waitAndRefresh(6000)) return;
}

uint16_t makeRandomPatternWithBits(int bits) {
  uint16_t m; do { m = random(1, 512); } while (__builtin_popcount(m) != bits); return m;
}

// =====================================================
// ARENA OS SPATIAL COST ALGORITHM & TIER HELPERS
// =====================================================
const int INF = 9999;
int getBaseCost(int u, int v) {
    if (u == v) return 0; int r1 = u/3, c1 = u%3; int r2 = v/3, c2 = v%3;
    int dr = abs(r1 - r2); int dc = abs(c1 - c2);
    if (dr == 0 && dc == 1) return 1; if (dr == 1 && dc == 0) return 1; 
    if (dr == 1 && dc == 1) return 2; if (dr == 0 && dc == 2) return 2; 
    if (dr == 2 && dc == 0) return 3; if ((dr == 1 && dc == 2) || (dr == 2 && dc == 1)) return 4; 
    if (dr == 2 && dc == 2) return 5; return INF;
}
bool isPathBlocked(int u, int v, const bool walls[9]) {
    int r1 = u/3, c1 = u%3; int r2 = v/3, c2 = v%3; int dr = abs(r1 - r2), dc = abs(c1 - c2);
    if (dr == 0 && dc == 2) return walls[r1 * 3 + 1]; if (dr == 2 && dc == 0) return walls[1 * 3 + c1]; 
    if (dr == 2 && dc == 2) return walls[4]; if (dr == 1 && dc == 2) return walls[r1 * 3 + 1] && walls[r2 * 3 + 1];
    if (dr == 2 && dc == 1) return walls[1 * 3 + c1] && walls[1 * 3 + c2]; return false;
}
int getShortestPath(int start, int target, const bool walls[9]) {
    int dist[9]; bool visited[9];
    for (int i=0; i<9; i++) { dist[i] = INF; visited[i] = false; } dist[start] = 0;
    for (int i=0; i<9; i++) {
        int u = -1; for (int j=0; j<9; j++) { if (!visited[j] && (u == -1 || dist[j] < dist[u])) u = j; }
        if (dist[u] == INF) break; visited[u] = true;
        for (int v=0; v<9; v++) {
            if (u == v || (walls[v] && v != target) || isPathBlocked(u, v, walls)) continue;
            int alt = dist[u] + getBaseCost(u, v); if (alt < dist[v]) dist[v] = alt;
        }
    }
    return (dist[target] == INF) ? (getBaseCost(start, target) + 2) : dist[target];
}
int calculateTotalSpatialCost(uint16_t expected, uint16_t input) {
    bool walls[9] = {false}; std::vector<int> leftoverTargets, leftoverInputs;
    for (int i=0; i<9; i++) {
        bool inTarget = expected & (1 << i); bool inInput = input & (1 << i);
        if (inTarget && inInput) walls[i] = true;
        else if (inTarget) leftoverTargets.push_back(i); else if (inInput) leftoverInputs.push_back(i);
    }
    if (leftoverTargets.empty() || leftoverInputs.empty()) return 0;
    int minTotalCost = INF; std::sort(leftoverInputs.begin(), leftoverInputs.end()); 
    do {
        int currentPermutationCost = 0;
        for (size_t i = 0; i < leftoverTargets.size(); i++) currentPermutationCost += getShortestPath(leftoverInputs[i], leftoverTargets[i], walls);
        if (currentPermutationCost < minTotalCost) minTotalCost = currentPermutationCost;
    } while (std::next_permutation(leftoverInputs.begin(), leftoverInputs.end()));
    return minTotalCost;
}
bool isOppositeCommand(uint16_t F, uint16_t Y, uint16_t X) {
    if (Y == mirrorV(F) && X == mirrorH(F)) return true; if (Y == mirrorH(F) && X == mirrorV(F)) return true;
    if (Y == rotateCW(F) && X == rotateCCW(F)) return true; if (Y == rotateCCW(F) && X == rotateCW(F)) return true; return false;
}
bool isRigidShift(uint16_t expected, uint16_t input) {
    for (int dr = -2; dr <= 2; dr++) {
        for (int dc = -2; dc <= 2; dc++) {
            if (dr == 0 && dc == 0) continue; if (dr != 0 && dc != 0) continue; 
            uint16_t shifted = 0; bool outOfBounds = false;
            for (int i = 0; i < 9; i++) {
                if (expected & (1 << i)) {
                    int nr = (i / 3) + dr; int nc = (i % 3) + dc;
                    if (nr < 0 || nr > 2 || nc < 0 || nc > 2) { outOfBounds = true; break; } shifted |= (1 << (nr * 3 + nc));
                }
            }
            if (!outOfBounds && shifted == input) return true;
        }
    }
    return false;
}

// 6. COGNITION MODE
// =====================================================
void runCognitionMode(int stage) {
  float sumPer = 0.0f; float sumPro = 0.0f; int maxStreak = 0; 
  display.clearDisplay(); display.setTextSize(2); display.setCursor(15, 20); display.print("COG STAGE "); display.print(stage); display.display();
  if (waitAndRefresh(1500)) return;
  float meanPer=0.0f, m2Per=0.0f, meanPro=0.0f, m2Pro=0.0f; int nPP = 0; float meanE=0.0f, m2E=0.0f; int nE = 0;
  
  for (int r = 1; r <= 5; r++) {
    int bitCount = (stage == 0) ? 4 : (stage == 1) ? 5 : 6;
    uint16_t base = makeRandomPatternWithBits(bitCount); uint16_t trap = 0;
    
    display.clearDisplay(); display.setTextSize(2); display.setCursor(25, 20); display.print("ROUND "); display.print(r); display.display();
    showMaskGreen(base); if (waitAndRefresh(1200)) return; turnOffAllLEDs(); if (waitAndRefresh(400)) return;
    
    if (stage >= 2) {
      int trapSize = (stage == 2) ? random(2, 4) : random(3, 5);
      do { trap = base & random(1, 512); } while (__builtin_popcount(trap) != trapSize);
      showMaskRed(trap); if (waitAndRefresh(1000)) return; turnOffAllLEDs(); if (waitAndRefresh(400)) return;
    }
    uint16_t filtered = base & ~trap; int cmd1 = 0; uint16_t expected = 0;
    
    display.clearDisplay();
    if (stage == 3) {
      int mirrorType = random(0, 2); int rotType = random(0, 2); int order = random(0, 2); uint16_t t1 = filtered;
      if (order == 0) {
        t1 = (mirrorType == 0) ? mirrorH(t1) : mirrorV(t1); expected = (rotType == 0) ? rotateCW(t1) : rotateCCW(t1);
        display.setTextSize(2); display.setCursor(0, 5); display.print(mirrorType == 0 ? "V Mirror" : "H Mirror");
        display.setTextSize(1); display.setCursor(0, 25); display.print("then...");
        display.setTextSize(2); display.setCursor(0, 40); display.print(rotType == 0 ? "Rot 90 CW" : "Rot 90 CCW");
      } else {
        t1 = (rotType == 0) ? rotateCW(t1) : rotateCCW(t1); expected = (mirrorType == 0) ? mirrorH(t1) : mirrorV(t1);
        display.setTextSize(2); display.setCursor(0, 5); display.print(rotType == 0 ? "Rot 90 CW" : "Rot 90 CCW");
        display.setTextSize(1); display.setCursor(0, 25); display.print("then...");
        display.setTextSize(2); display.setCursor(0, 40); display.print(mirrorType == 0 ? "V Mirror" : "H Mirror");
      }
      display.display(); unsigned long showT = millis(); while (millis() - showT < 900) { if (customKeypad.getKey() == '*') return; refreshMatrix(); }
    } else {
      cmd1 = (stage == 0) ? random(0, 2) : random(0, 4); expected = applyTransform(filtered, cmd1);
      display.setTextSize(2); display.setCursor(0, 25); display.print(getCmdName(cmd1)); display.display();
      unsigned long showCmd = millis(); while (millis() - showCmd < 1500) { if (customKeypad.getKey() == '*') return; refreshMatrix(); }
    }

    uint16_t input = 0; int targetCount = __builtin_popcount(expected);
    int maxIn = targetCount + 1; if (maxIn > 9) maxIn = 9;     
    int current = 0; unsigned long timeLimit = (stage == 3) ? 10000 : 7000;
    unsigned long start = millis(); int lastBarPixels = -1;

    display.clearDisplay(); display.setTextSize(2); display.setCursor(15, 20); display.print("INPUT..."); display.drawRect(0, 50, 128, 12, SSD1306_WHITE);
    
    while (current < maxIn && (millis() - start < timeLimit)) {
      unsigned long elapsed = millis() - start; int barPixels = (int)(124.0 * (timeLimit - elapsed) / timeLimit);
      if (abs(barPixels - lastBarPixels) >= 2) {
        display.fillRect(2, 52, 124, 8, SSD1306_BLACK); display.fillRect(2, 52, barPixels, 8, SSD1306_WHITE); display.display(); lastBarPixels = barPixels;
      }
      char k = getKeyWithRefresh();
      if (k == '*') return; else if (k == '#') break;
      if (k >= '1' && k <= '9') {
        int idx = k - '1'; if (!(input & BIT(idx))) { input |= BIT(idx); current++; currentGreenMask |= BIT(idx); if (waitAndRefresh(100)) return; currentGreenMask &= ~BIT(idx); }
      }
    }
    
    if ((millis() - start) >= timeLimit && current < targetCount) {
      display.clearDisplay(); display.setTextSize(2); display.setCursor(15,20); display.print("TIMEOUT!"); display.display(); if (waitAndRefresh(1200)) return;
    }
    
    uint16_t F = filtered, Y = expected, X = input; float Perception = 0.0f, Processing = 0.0f;
    bool showRoundScores = true; bool perfect = (X == Y);
    if (X == Y) streak++; else streak = 0; if (streak > maxStreak) maxStreak = streak;

    if (X == 0) { Perception = 0.0f; Processing = 0.0f; showRoundScores = false; } 
    else if (X == Y && Y == F) { Perception = 100.0f; Processing = 100.0f; } 
    else if (X == F && Y != F) { Perception = 85.0f; Processing = 0.0f; } 
    else if (isOppositeCommand(F, Y, X)) { Perception = 85.0f; Processing = 60.0f; } 
    else if (pop9(X) == pop9(Y) && isRigidShift(Y, X)) { Perception = 90.0f; Processing = 60.0f; } 
    else {
        int D = calculateTotalSpatialCost(Y, X); int FP = pop9(X & ~Y); int FN = pop9(Y & ~X); int H = ham(X, Y);
        float stagePenP[] = {1.0f, 1.15f, 1.35f, 1.6f}; float symF = isSym(F) ? 0.85f : 1.0f; float divisor = 8.5f * symF * stagePenP[stage];
        int m = pop9(X); int n = pop9(Y); float K = 7.5f + (float)stage; float sizeRatioPen = (n > 0) ? abs(1.0f - ((float)m / (float)n)) * K : 0.0f;
        float deduct = ((0.5f * (float)D) + (0.40f * (float)FP) + (0.30f * (float)FN) + sizeRatioPen) / divisor;
        Perception = max(0.0f, 100.0f - (deduct * 100.0f));
        int cmdForProc = (stage == 3) ? 4 : cmd1; float Wcmd = 1.0f;
        if (cmdForProc == 0 || cmdForProc == 1) Wcmd = 1.05f; else if (cmdForProc == 2 || cmdForProc == 3) Wcmd = 1.30f; else if (cmdForProc == 4) Wcmd = 1.60f; 
        if (isTough(F)) Wcmd *= 1.10f; 
        float Dn = (D > 12) ? 12.0f : (float)D; float qH = max(0.0f, 1.0f - ((float)H / 9.0f)); float qD = max(0.0f, 1.0f - (Dn / 12.0f));
        float qFN = max(0.0f, 1.0f - ((float)FN / 9.0f)); float qFP = max(0.0f, 1.0f - ((float)FP / 9.0f));
        float BaseQuality = 0.45f*qH + 0.30f*qD + 0.15f*qFN + 0.10f*qFP; float procK = 0.40f + ((float)stage * 0.10f); 
        float procSizeRatioPen = (n > 0) ? abs(1.0f - ((float)m / (float)n)) * procK : 0.0f;
        float Quality = constrain(BaseQuality - procSizeRatioPen, 0.0f, 1.0f);
        float Wp = ((stage == 0) ? 1.0f : (stage == 1) ? 1.15f : (stage == 2) ? 1.35f : 1.6f); if (isSym(F)) Wp *= 0.85f;
        float stagePen = (stage == 0) ? 1.60f : (stage == 1) ? 1.35f : (stage == 2) ? 1.15f : 1.00f;
        Processing = (100.0f * Quality * Wcmd) / (Wp * stagePen); if (perfect) Processing = 100.0f; Processing = constrain(Processing, 0.0f, 100.0f);
    }
    sumPer += Perception; sumPro += Processing;
    nPP++; float dP = Perception - meanPer; meanPer += dP / nPP; m2Per += dP * (Perception - meanPer);
    float dR = Processing - meanPro; meanPro += dR / nPP; m2Pro += dR * (Processing - meanPro);
    int FP_e = pop9(X & ~Y); int FN_e = pop9(Y & ~X); int H_e = ham(X, Y); int D_e = calculateTotalSpatialCost(Y, X);
    float E = 6.0f*FP_e + 3.0f*FN_e + 1.0f*D_e + 2.0f*H_e;
    nE++; float dE = E - meanE; meanE += dE / nE; m2E += dE * (E - meanE);
    
    if (showRoundScores) {
        display.clearDisplay(); display.setTextSize(1);
        display.setCursor(0, 10); display.print("Perception:"); display.setCursor(0, 35); display.print("Processing:");
        display.setTextSize(2);
        display.setCursor(75, 5); display.print((int)(Perception + 0.5f)); display.print("%");
        display.setCursor(75, 30); display.print((int)(Processing + 0.5f)); display.print("%"); display.display();
    }
    showMaskGreen(expected); if (waitAndRefresh(2000)) return; turnOffAllLEDs();
  }
  
  float sdPer = sqrt((nPP > 1) ? (m2Per / (nPP - 1)) : 0.0f); float sdPro = sqrt((nPP > 1) ? (m2Pro / (nPP - 1)) : 0.0f); float sdE = sqrt((nE > 1) ? (m2E / (nE - 1)) : 0.0f);
  float Stability = 0.60f * (0.40f*max(0.0f, 100.0f*(1.0f-sdPer/22.0f)) + 0.60f*max(0.0f, 100.0f*(1.0f-sdPro/22.0f))) + 0.40f*max(0.0f, 100.0f*(1.0f-sdE/18.0f));
  Stability = min(100.0f, Stability + ((maxStreak >= 2) ? (maxStreak - 1) * 4.0f : 0.0f));
  
  display.clearDisplay(); display.setTextSize(1);
  display.setCursor(0, 5); display.print("Perception:"); display.setCursor(0, 25); display.print("Processing:"); display.setCursor(0, 45); display.print("Stability:");
  display.setTextSize(2);
  display.setCursor(75, 0); display.print((int)(sumPer/5.0f + 0.5f)); display.print("%");
  display.setCursor(75, 20); display.print((int)(sumPro/5.0f + 0.5f)); display.print("%");
  display.setCursor(75, 40); display.print((int)(Stability + 0.5f)); display.print("%"); display.display();
  if (waitAndRefresh(4000)) return;
}

// =====================================================
// 7. REFLEX MODE (RHYTHM ENGINE WITH STRESS METRICS)
// =====================================================
void runReflexMode(int stage) {
  float reflexScore = 0;
  float reflexMultiplier = 1.0;
  float perfectScore = 0;
  float perfectMultiplier = 1.0;
  
  int remainingNormalGreens = (stage==0)? 12 : (stage==1)? 18 : (stage==2)? 24 : 40;
  int remainingRapidGreens  = (stage==0)?  0 : (stage==1)?  0 : (stage==2)? 12 : 20;
  
  unsigned long baseBeatIntervalMs = 1000 - (stage * 120);
  
  bool isRapidFire = false;
  bool rapidFirePlayed = false;
  int rapidFireTriggerThreshold = (remainingNormalGreens > 5) ? random(5, remainingNormalGreens - 5) : 0;

  int redTracker = 0;
  bool punishmentState = false;

  // Stress Tracking Variables
  int max_punishment_duration = 5;
  int current_punishment_duration = 0;
  int max_punishment_lapses = 0;
  int current_punishment_lapses = 0;
  int max_mash_per_beat = 0;
  int error_history = 0; 
  int max_error_density = 0;
  int hyper_hits = 0;
  int total_green_hits = 0;
  float rt_mean = 0.0f;
  float rt_m2 = 0.0f;

  display.clearDisplay(); 
  display.setTextSize(2); display.setCursor(25, 10); display.print("REFLEX"); 
  display.setCursor(15, 35); display.print("STAGE "); display.print(stage);
  display.display();
  waitAndRefresh(1500);

  while (remainingNormalGreens > 0 || remainingRapidGreens > 0) {
    if (!isRapidFire && !rapidFirePlayed && remainingRapidGreens > 0) {
        if (remainingNormalGreens <= rapidFireTriggerThreshold || remainingNormalGreens == 0) {
            isRapidFire = true; rapidFirePlayed = true;
        }
    }
    if (isRapidFire && remainingRapidGreens == 0) isRapidFire = false;

    int targets = 1;
    if (stage == 3) targets = random(1, 4); 
    int* pool = isRapidFire ? &remainingRapidGreens : &remainingNormalGreens;
    if (targets > *pool) targets = *pool;

    uint16_t localGreenMask = 0;
    uint16_t localRedMask = 0;

    for (int i=0; i<targets; i++) {
        int bit; do { bit = random(0, 9); } while (localGreenMask & BIT(bit));
        localGreenMask |= BIT(bit);
    }

    if (stage >= 1 && random(0, 5) == 0) {
        int bit = random(0, 9); localRedMask |= BIT(bit);
        if (localGreenMask & BIT(bit)) localGreenMask &= ~BIT(bit); 
    }

    int actual_greens_spawned = pop9(localGreenMask);
    *pool -= actual_greens_spawned;

    float ai_slot_val = actual_greens_spawned;
    float ai_bonus = (actual_greens_spawned > 1) ? (isRapidFire ? 1.5 : 1.0) : 0.0;
    perfectScore += (ai_slot_val + ai_bonus) * perfectMultiplier;
    for (int i=0; i<actual_greens_spawned; i++) perfectMultiplier += (isRapidFire ? 0.4 : 0.2);

    showMaskGreen(localGreenMask);
    showMaskRed(localRedMask);

    unsigned long beatDuration = isRapidFire ? (baseBeatIntervalMs / 1.75) : baseBeatIntervalMs;
    unsigned long startBeat = millis();
    char inputs[15];
    unsigned long input_times[15];
    int input_cnt = 0;

    while(millis() - startBeat < beatDuration) {
        char k = customKeypad.getKey();
        unsigned long now = millis();
        if (k == '*') return;
        if (k && k != '#') {
            if (input_cnt == 0 || inputs[input_cnt-1] != k) { 
                if (input_cnt < 15) {
                    inputs[input_cnt] = k;
                    input_times[input_cnt] = now;
                    input_cnt++;
                }
            }
        }
        refreshMatrix(); 
    }
    turnOffAllLEDs();

    int limit = (stage <= 1) ? 3 : 2;
    if (stage == 3 && actual_greens_spawned > 1) limit = 4;
    bool spammed = (input_cnt > limit);

    int green_hits = 0;
    int red_hits = 0;
    uint16_t hit_greens_mask = 0;
    
    int errors_this_beat = 0;
    int neutral_red_this_beat = 0;

    for (int i=0; i<input_cnt; i++) {
        int idx = inputs[i] - '1';
        unsigned long rt = input_times[i] - startBeat;
        if (idx < 0 || idx > 8) continue;

        if (localRedMask & BIT(idx)) {
            red_hits++; neutral_red_this_beat++; errors_this_beat++;
            if (isRapidFire) { isRapidFire = false; remainingRapidGreens = 0; } 
            
            if (punishmentState) { 
                reflexMultiplier = 0.0; 
                current_punishment_lapses++;
                if (current_punishment_lapses > max_punishment_lapses) max_punishment_lapses = current_punishment_lapses;
            } else {
                redTracker++;
                if (redTracker >= 3) { 
                    punishmentState = true; reflexMultiplier = 0.0; 
                    current_punishment_duration = 0; current_punishment_lapses = 0; 
                }
            }
        } else if (localGreenMask & BIT(idx)) {
            if (!(hit_greens_mask & BIT(idx))) { 
                hit_greens_mask |= BIT(idx);
                green_hits++; total_green_hits++;
                
                if (rt < 100) hyper_hits++;
                float delta = rt - rt_mean; rt_mean += delta / total_green_hits; rt_m2 += delta * (rt - rt_mean);
                if (punishmentState) current_punishment_duration++;
                
                reflexMultiplier += (isRapidFire ? 0.4 : 0.2);
            } else { 
                neutral_red_this_beat++; errors_this_beat++;
                if (punishmentState) current_punishment_duration++;
                if (!punishmentState) reflexMultiplier -= (isRapidFire ? 0.4 : 0.2);
            }
        } else { 
            neutral_red_this_beat++; errors_this_beat++;
            if (punishmentState) current_punishment_duration++;
            if (!punishmentState) reflexMultiplier -= (isRapidFire ? 0.4 : 0.2);
        }

        if (!punishmentState && reflexMultiplier < 1.0) reflexMultiplier = 1.0;
        if (punishmentState && reflexMultiplier >= 1.0) { 
            punishmentState = false; redTracker = 0; reflexMultiplier = 1.0; 
            if (current_punishment_duration > max_punishment_duration) max_punishment_duration = current_punishment_duration;
        }
    }

    if (input_cnt == 0 && actual_greens_spawned > 0 && !punishmentState) {
        reflexMultiplier -= 0.1;
        if (reflexMultiplier < 1.0) reflexMultiplier = 1.0;
        errors_this_beat++;
    }
    
    if (spammed) errors_this_beat++;
    if (neutral_red_this_beat > max_mash_per_beat) max_mash_per_beat = neutral_red_this_beat;

    error_history = (error_history << 1) & 0x3FF;
    if (errors_this_beat > 0) error_history |= 1;
    int current_density = 0;
    for(int b=0; b<10; b++) if(error_history & (1<<b)) current_density++;
    if (current_density > max_error_density) max_error_density = current_density;

    float slot_val = 0;
    if (!spammed) slot_val += green_hits;
    slot_val -= red_hits; 

    float comp_bonus = 0;
    if (!spammed && actual_greens_spawned > 1 && green_hits == actual_greens_spawned) {
        comp_bonus = isRapidFire ? 1.5 : 1.0;
    }

    reflexScore += (slot_val + comp_bonus) * reflexMultiplier;
  }
  
  if (punishmentState && current_punishment_duration > max_punishment_duration) {
      max_punishment_duration = current_punishment_duration;
  }

  // End Game Calculation - Performance
  float ratio = reflexMultiplier / perfectMultiplier;
  int milestoneBonus = 0;

  if (ratio >= 0.90) {
      if (stage == 0) milestoneBonus = 3; else if (stage == 1) milestoneBonus = 6; else if (stage == 2) milestoneBonus = 15; else if (stage == 3) milestoneBonus = 54;
  } else if (ratio >= 0.80) {
      if (stage == 0) milestoneBonus = 2; else if (stage == 1) milestoneBonus = 4; else if (stage == 2) milestoneBonus = 11; else if (stage == 3) milestoneBonus = 38;
  } else if (ratio >= 0.70) {
      if (stage == 0) milestoneBonus = 1; else if (stage == 1) milestoneBonus = 2; else if (stage == 2) milestoneBonus = 5; else if (stage == 3) milestoneBonus = 16;
  }

  float finalScore = reflexScore + milestoneBonus;
  float performancePercent = (perfectScore > 0) ? (finalScore / perfectScore * 100.0) : 0;
  if (performancePercent < 0) performancePercent = 0;
  if (performancePercent > 100) performancePercent = 100;

  // End Game Calculation - Stress Evaluation
  float S_rec_eff = min(1.0f, max(0.0f, (max_punishment_duration - 5) / 15.0f));
  float S_lapse   = min(1.0f, max_punishment_lapses / 3.0f);
  float std_dev   = (total_green_hits > 1) ? sqrt(rt_m2 / (total_green_hits - 1)) : 0.0f;
  float S_var     = min(1.0f, max(0.0f, (std_dev - 80.0f) / 370.0f));
  float S_dens    = min(1.0f, max(0.0f, (max_error_density - 1) / 4.0f));
  float S_hyper   = min(0.2f, (total_green_hits > 0) ? ((float)hyper_hits / total_green_hits) : 0.0f);
  float S_mash    = min(1.0f, max_mash_per_beat / 5.0f);

  float s_array[6] = {S_rec_eff, S_lapse, S_var, S_dens, S_hyper, S_mash};
  float s_max = 0.0f;
  float s_sum = 0.0f;
  for (int i=0; i<6; i++) {
      if (s_array[i] > s_max) s_max = s_array[i];
      s_sum += s_array[i];
  }
  float s_mean = (s_sum - s_max) / 5.0f;
  // Calculate the raw ratio first (0.0 to 1.0)
  float rawStressRatio = (0.85f * s_max) + (0.15f * s_mean);
  
  // Apply Quadratic Damping: Stress = (Raw^2) * 100
  float stressPercentage = pow(rawStressRatio, 2.0f) * 100.0f;
  
  // Final bounds check
  stressPercentage = constrain(stressPercentage, 0.0f, 100.0f);
  // Bug fix: assign computed value to the global used by the display block
  finalStressPercentage = stressPercentage;

  display.clearDisplay(); 
  display.setTextSize(1); 
  display.setCursor(0, 0); display.print("PERFORMANCE:");
  display.setTextSize(2); 
  display.setCursor(0, 10); display.print(performancePercent, 1); display.print("%");
  
  display.setTextSize(1); 
  display.setCursor(0, 35); display.print("STRESS LEVEL:");
  display.setTextSize(2); 
  display.setCursor(0, 45);
    display.setCursor(0, 45); 
    display.print((int)(finalStressPercentage + 0.5f)); 
    display.print("%");

  display.display();
  waitAndRefresh(6000);
}

// =====================================================
// 8. SETUP & LOOP
// =====================================================
void setup() {
  randomSeed(analogRead(0));
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for(;;); 
  }
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.display();

  for (int i=0; i<3; i++) { pinMode(ledColPins[i], OUTPUT); digitalWrite(ledColPins[i], HIGH); pinMode(ledGreenPins[i], OUTPUT); pinMode(ledRedPins[i], OUTPUT); }
  for(int p=0; p<9; p++) { uint16_t m=0; for(int i=0; i<9; i++) if(patterns[p][i]) m|=BIT(i); patternMask[p] = m; }
  trapMasks[0] = 0x155; trapMasks[1] = 0x0B4; trapMasks[2] = 0x072; trapMasks[3] = 0x0AC;
  mem3Frag[0][0]=0x00A; mem3Frag[0][1]=0x0A0; mem3FragKey[0]='1';
  mem3Frag[1][0]=0x014; mem3Frag[1][1]=0x140; mem3FragKey[1]='2';
  mem3Frag[2][0]=0x012; mem3Frag[2][1]=0x140; mem3FragKey[2]='6';
  mem3Frag[3][0]=0x048; mem3Frag[3][1]=0x012; mem3FragKey[3]='7';
  mem3Frag[4][0]=0x012; mem3Frag[4][1]=0x0A0; mem3FragKey[4]='8';
  mem3Frag[5][0]=0x009; mem3Frag[5][1]=0x0A0; mem3FragKey[5]='9';
}

void loop() {
  turnOffAllLEDs(); 
  display.clearDisplay(); 
  display.setTextSize(2); display.setCursor(16, 5); display.print("ARENA OS");
  display.setTextSize(1); display.setCursor(52, 25); display.print("v3.3");
  display.setCursor(0, 45); display.print("1:Mem  2:Cog  3:Ref");
  display.display();
  
  char mode = 0; while(!mode) { mode = customKeypad.getKey(); refreshMatrix(); }
  
  display.clearDisplay(); 
  display.setTextSize(2); display.setCursor(25, 10); display.print("SELECT"); display.setCursor(30, 35); display.print("STAGE");
  display.setTextSize(1); display.setCursor(50, 55); display.print("(0-3)");
  display.display();
  
  char s = 0; while(!s) { s = customKeypad.getKey(); refreshMatrix(); }
  int stg = s - '0';
  
  if (mode == '1') runMemoryMode(stg);
  else if (mode == '2') runCognitionMode(stg);
  else if (mode == '3') runReflexMode(stg);
}