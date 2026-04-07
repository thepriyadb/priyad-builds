// ==========================================================
// BRAILLE TO TEXT CONVERTER (LOOKUP TABLE + MULTI-MODE VERSION)
// ==========================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd_27(0x27, 16, 2);
LiquidCrystal_I2C lcd_3F(0x3F, 16, 2);
LiquidCrystal_I2C *lcd = nullptr;

// --- Pin Definitions ---
const int braillePins[6] = {2, 3, 4, 5, 6, 7};
const int buzzerPin = 8;
const int spaceButton = A0;
const int modeButton = A3;

String buffer = "";
int mode = 0; // 0 = Letters, 1 = Numbers, 2 = Symbols

// --------------------------
// Character sets
// --------------------------
const char letters[] = {
  'A','B','C','D','E','F','G','H','I','J',
  'K','L','M','N','O','P','Q','R','S','T',
  'U','V','W','X','Y','Z'
};
const int brailleLetters[] = {
  0b100000,0b110000,0b100100,0b100110,0b100010,
  0b110100,0b110110,0b110010,0b010100,0b010110,
  0b101000,0b111000,0b101100,0b101110,0b101010,
  0b111100,0b111110,0b111010,0b011100,0b011110,
  0b101001,0b111001,0b010111,0b101101,0b101111,0b101011
};

// Numbers (A–J patterns)
const char numbers[] = {'1','2','3','4','5','6','7','8','9','0'};
const int brailleNumbers[] = {
  0b100000,0b110000,0b100100,0b100110,0b100010,
  0b110100,0b110110,0b110010,0b010100,0b010110
};

// Symbols
const char symbols[] = {'.', ',', '?', '!', '-', '"', '\'', '(', ')'};
const int brailleSymbols[] = {
  0b010011,0b010000,0b011001,0b011010,0b001001,
  0b011011,0b001000,0b001110,0b001111
};

// Morse codes
const String morseLetters[] = {
  ".-","-...","-.-.","-..",".","..-.", "--.","....","..",".---",
  "-.-",".-..","--","-.","---",".--.","--.-",".-.","...","-",
  "..-","...-", ".--","-..-","-.--","--.."
};
const String morseNumbers[] = {
  ".----","..---","...--","....-",".....","-....","--...","---..","----.","-----"
};
const String morseSymbols[] = {
  ".-.-.-","--..--","..--..","-.-.--","-....-",
  ".-..-.",".----.","-.--.","-.--.-"
};

// --------------------------
// Helper Functions
// --------------------------
void beep(int duration) {
  digitalWrite(buzzerPin, HIGH);
  delay(duration);
  digitalWrite(buzzerPin, LOW);
  delay(120);
}

void playMorse(String code) {
  const int dot = 150, dash = 450;
  for (int i = 0; i < code.length(); i++) {
    if (code[i] == '.') beep(dot);
    else if (code[i] == '-') beep(dash);
    delay(150);
  }
  delay(300);
}

char decodeBraille(int pattern) {
  if (mode == 0) {
    for (int i = 0; i < 26; i++) if (brailleLetters[i] == pattern) return letters[i];
  } else if (mode == 1) {
    for (int i = 0; i < 10; i++) if (brailleNumbers[i] == pattern) return numbers[i];
  } else if (mode == 2) {
    for (int i = 0; i < 9; i++) if (brailleSymbols[i] == pattern) return symbols[i];
  }
  return '?';
}

String getMorse(char c) {
  if (mode == 0 && c >= 'A' && c <= 'Z') return morseLetters[c - 'A'];
  if (mode == 1 && c >= '0' && c <= '9') return morseNumbers[c == '0' ? 9 : c - '1'];
  if (mode == 2) {
    for (int i = 0; i < 9; i++) if (symbols[i] == c) return morseSymbols[i];
  }
  return "";
}

// --------------------------
// Setup
// --------------------------
void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 6; i++) pinMode(braillePins[i], INPUT_PULLUP);
  pinMode(spaceButton, INPUT_PULLUP);
  pinMode(modeButton, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);

  // Auto-detect LCD address
  Wire.begin();
  Wire.beginTransmission(0x27);
  if (Wire.endTransmission() == 0) lcd = &lcd_27;
  else {
    Wire.beginTransmission(0x3F);
    if (Wire.endTransmission() == 0) lcd = &lcd_3F;
  }

  if (lcd == nullptr) {
    Serial.println("❌ No I2C LCD detected! Check wiring (A4 SDA, A5 SCL).");
    while (true);
  }

  lcd->init();
  lcd->backlight();
  lcd->print("Braille Ready");
  delay(1500);
  lcd->clear();

  Serial.println("✅ Braille to Text Converter Ready");
  Serial.println("Mode button (A3): Switch between Letters, Numbers, Symbols");
  Serial.println("----------------------------------------------------------");
}

// --------------------------
// Main Loop
// --------------------------
void loop() {
  checkModeSwitch();

  if (!digitalRead(spaceButton)) {
    buffer += ' ';
    lcd->clear();
    lcd->print(buffer);
    beep(200);
    Serial.println("Space Added");
    waitForRelease();
  }

  int pattern = readStablePatternMSB();
  if (pattern > 0) {
    char symbol = decodeBraille(pattern);
    String morse = getMorse(symbol);
    String binary = "";

    for (int i = 5; i >= 0; i--) binary += String((pattern >> i) & 1);

    Serial.print(binary);
    Serial.print(" | ");
    Serial.print(symbol);
    Serial.print(" | ");
    Serial.println(morse);

    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("C:");
    lcd->print(symbol);
    lcd->print(" B:");
    lcd->print(binary);
    lcd->setCursor(0, 1);
    lcd->print("M:");
    lcd->print(morse);

    if (symbol != '?' && morse.length() > 0) playMorse(morse);

    waitForRelease();
  }
}

// --------------------------
// Mode Switching
// --------------------------
void checkModeSwitch() {
  static unsigned long lastPress = 0;
  if (!digitalRead(modeButton)) {
    if (millis() - lastPress > 500) {
      mode = (mode + 1) % 3;
      lcd->clear();
      if (mode == 0) lcd->print("Mode: Letters");
      else if (mode == 1) lcd->print("Mode: Numbers");
      else lcd->print("Mode: Symbols");

      Serial.print("Switched to ");
      if (mode == 0) Serial.println("Letters Mode");
      else if (mode == 1) Serial.println("Numbers Mode");
      else Serial.println("Symbols Mode");

      delay(800);
      lcd->clear();
    }
    lastPress = millis();
  }
}

// --------------------------
// Braille Input Handling
// --------------------------
int readStablePatternMSB() {
  int prev = 0, curr = 0, stableCount = 0;
  unsigned long start = millis();
  while (millis() - start < 400) {
    curr = 0;
    for (int i = 0; i < 6; i++) {
      int bitValue = !digitalRead(braillePins[i]);
      curr |= (bitValue << (5 - i));
    }
    if (curr == prev && curr != 0) stableCount++;
    else stableCount = 0;
    prev = curr;
    if (stableCount > 4) break;
    delay(30);
  }
  return curr;
}

void waitForRelease() {
  unsigned long start = millis();
  bool released = false;
  while (!released && (millis() - start < 2000)) {
    released = true;
    for (int i = 0; i < 6; i++)
      if (!digitalRead(braillePins[i])) released = false;
    if (!digitalRead(spaceButton)) released = false;
    if (!digitalRead(modeButton)) released = false;
    delay(25);
  }
  delay(250);
}
