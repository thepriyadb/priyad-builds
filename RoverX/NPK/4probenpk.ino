#include <ModbusMaster.h>
#include <SoftwareSerial.h>

const int RO_PIN = 5;
const int DI_PIN = 6;
const int RE_PIN = 8;
const int DE_PIN = 7;

SoftwareSerial swSerial(RO_PIN, DI_PIN); // Receive (RO), Transmit (DI)
ModbusMaster node;

// Put the MAX485 into transmit mode
void preTransmission()
{
  digitalWrite(RE_PIN, 1);
  digitalWrite(DE_PIN, 1);
}

// Put the MAX485 into receive mode
void postTransmission()
{
  digitalWrite(RE_PIN, 0);
  digitalWrite(DE_PIN, 0);
}

void setup()
{
  Serial.begin(9600);

  // configure the MAX485 RE & DE control signals
  pinMode(RE_PIN, OUTPUT);
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, 0);
  digitalWrite(RE_PIN, 0);

  // Modbus communication runs at 9600 baud
  swSerial.begin(9600);

  // Modbus slave ID of NPK sensor is 1
  node.begin(1, swSerial);

  // RS485 direction callbacks
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  delay(1000);
}

void loop()
{
  uint8_t result;

  // ask for 7x 16-bit words starting at register address 0x0000
  result = node.readHoldingRegisters(0x0000, 7);

  if (result == node.ku8MBSuccess)
  {
    Serial.println("Reply:");

    Serial.print("Temp: ");
    Serial.print(node.getResponseBuffer(0) / 10.0);
    Serial.println(" F");

    Serial.print("Moisture: ");
    Serial.print(node.getResponseBuffer(1) / 10.0);
    Serial.println("%");

    Serial.print("EC: ");
    Serial.print(node.getResponseBuffer(2));
    Serial.println(" uS/cm");

    Serial.print("pH: ");
    Serial.print(node.getResponseBuffer(3) / 100.0);
    Serial.println();

    Serial.print("N: ");
    Serial.print(node.getResponseBuffer(4) * 10);
    Serial.println(" mg/kg");

    Serial.print("P: ");
    Serial.print(node.getResponseBuffer(5) * 10);
    Serial.println(" mg/kg");

    Serial.print("K: ");
    Serial.print(node.getResponseBuffer(6) * 10);
    Serial.println(" mg/kg");
  }
  else
  {
    printModbusError(result);
  }

  delay(10000);
}

void printModbusError(uint8_t errNum)
{
  switch (errNum)
  {
    case node.ku8MBSuccess:
      Serial.println("Success");
      break;

    case node.ku8MBIllegalFunction:
      Serial.println("Illegal Function Exception");
      break;

    case node.ku8MBIllegalDataAddress:
      Serial.println("Illegal Data Address Exception");
      break;

    case node.ku8MBIllegalDataValue:
      Serial.println("Illegal Data Value Exception");
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
