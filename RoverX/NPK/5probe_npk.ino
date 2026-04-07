#include <ModbusMaster.h>
#include <SoftwareSerial.h>

/* ------------------ PIN DEFINITIONS ------------------ */
#define RO_PIN 2   // MAX485 RO
#define DI_PIN 3   // MAX485 DI
#define DE_PIN 6   // MAX485 DE
#define RE_PIN 7   // MAX485 RE

SoftwareSerial swSerial(RO_PIN, DI_PIN);
ModbusMaster node;

/* ------------------ RS485 DIRECTION CONTROL ------------------ */
void preTransmission() {
  digitalWrite(RE_PIN, HIGH);
  digitalWrite(DE_PIN, HIGH);
}

void postTransmission() {
  digitalWrite(RE_PIN, LOW);
  digitalWrite(DE_PIN, LOW);
}

/* ------------------ SETUP ------------------ */
void setup() {
  Serial.begin(9600);

  pinMode(RE_PIN, OUTPUT);
  pinMode(DE_PIN, OUTPUT);

  digitalWrite(RE_PIN, LOW);   // Receive mode
  digitalWrite(DE_PIN, LOW);   // Receive mode

  swSerial.begin(9600);        // Sensor default baud rate

  node.begin(1, swSerial);     // Slave ID = 1
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  delay(1000);
  Serial.println("NPK Sensor Initialized");
}

/* ------------------ LOOP ------------------ */
void loop() {
  uint8_t result;

  /* ---------- Soil Moisture + Temperature ---------- */
  result = node.readHoldingRegisters(0x0012, 2);
  if (result == node.ku8MBSuccess) {
    float moisture = node.getResponseBuffer(0) / 10.0;
    float temperature = node.getResponseBuffer(1) / 10.0;

    Serial.print("Moisture: ");
    Serial.print(moisture);
    Serial.println(" %");

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
  } else {
    printModbusError(result);
  }

  delay(200);

  /* ------------------ pH ------------------ */
  result = node.readHoldingRegisters(0x0006, 1);
  if (result == node.ku8MBSuccess) {
    float ph = node.getResponseBuffer(0) / 100.0;
    Serial.print("pH: ");
    Serial.println(ph);
  } else {
    printModbusError(result);
  }

  delay(200);

  /* ------------------ EC ------------------ */
  result = node.readHoldingRegisters(0x0015, 1);
  if (result == node.ku8MBSuccess) {
    uint16_t ec = node.getResponseBuffer(0);
    Serial.print("EC: ");
    Serial.print(ec);
    Serial.println(" uS/cm");
  } else {
    printModbusError(result);
  }

  delay(200);

  /* ------------------ NPK ------------------ */
  result = node.readHoldingRegisters(0x001E, 3);
  if (result == node.ku8MBSuccess) {
    uint16_t N = node.getResponseBuffer(0);
    uint16_t P = node.getResponseBuffer(1);
    uint16_t K = node.getResponseBuffer(2);

    Serial.print("Nitrogen: ");
    Serial.print(N);
    Serial.println(" mg/kg");

    Serial.print("Phosphorus: ");
    Serial.print(P);
    Serial.println(" mg/kg");

    Serial.print("Potassium: ");
    Serial.print(K);
    Serial.println(" mg/kg");
  } else {
    printModbusError(result);
  }

  Serial.println("--------------------------------");
  delay(10000);
}

/* ------------------ ERROR HANDLER ------------------ */
void printModbusError(uint8_t errNum) {
  switch (errNum) {
    case node.ku8MBSuccess:
      Serial.println("Success");
      break;
    case node.ku8MBIllegalFunction:
      Serial.println("Illegal Function");
      break;
    case node.ku8MBIllegalDataAddress:
      Serial.println("Illegal Data Address");
      break;
    case node.ku8MBIllegalDataValue:
      Serial.println("Illegal Data Value");
      break;
    case node.ku8MBSlaveDeviceFailure:
      Serial.println("Slave Device Failure");
      break;
    case node.ku8MBInvalidSlaveID:
      Serial.println("Invalid Slave ID");
      break;
    case node.ku8MBInvalidFunction:
      Serial.println("Invalid Function");
      break;
    case node.ku8MBResponseTimedOut:
      Serial.println("Response Timed Out");
      break;
    case node.ku8MBInvalidCRC:
      Serial.println("Invalid CRC");
      break;
    default:
      Serial.println("Unknown Error");
      break;
  }
}