#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD 115200       // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND 1              // [ms] Sending time interval
#define SPEED_MAX_TEST 50        // [-] Maximum speed for testing
#define SPEED_STEP 1             // [-] Speed step
#define LED_BUILTIN 2            // [-] Built-in LED pin
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

unsigned long StartTime = millis();

uint16_t fps = 0;
uint16_t cmdCount = 0;
uint16_t lastCmdCount = 0;


// Global variables
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct
{
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct
{
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

typedef struct
{
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback2;
SerialFeedback Feedback2;
SerialFeedback NewFeedback2;

#include <HardwareSerial.h>
#define RX_PIN 16
#define TX_PIN 17

// Set up a new SoftwareSerial object
// HardwareSerial Serial1(1);  // RX, TX

// ########################## SETUP ##########################
void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.44");
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(LED_BUILTIN, OUTPUT);
}

// ########################## SEND ##########################
void SendFront(int16_t uSteer, int16_t uSpeed)
{
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = 0;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
  // if (Serial1.available()) {
  //   // Serial1.write((uint8_t *)&Command, sizeof(Command));
  // }
  Serial1.write((uint8_t *)&Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
void Receive()
{
  // Check for new data availability in the Serial buffer
  if (Serial1.available())
  {
    incomingByte = Serial1.read();                                      // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame
  }
  else
  {
    return;
  }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME)
  { // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  }
  else if (idx >= 2 && idx < sizeof(SerialFeedback))
  { // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback))
  {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
    {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      Serial.print("cmd1: ");
      Serial.print(Feedback.cmd1);
      Serial.print(" speedL_meas: ");
      Serial.print(Feedback.speedL_meas);
      Serial.print(" vol: ");
      Serial.print(Feedback.batVoltage);
      Serial.print(" lastCmdCount: ");
      Serial.print(lastCmdCount);
      Serial.print(" fps: ");
      Serial.println(fps);
    }
    else
    {
      Serial.println("Non-valid data skipped");
    }
    idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}


// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;


unsigned long previousMicros = 0;
unsigned long interval = 1000000; // 1 second interval in microseconds
unsigned int loopCounter = 0;

void loop(void)
{
  unsigned long timeNow = micros();

  loopCounter++;
  if (timeNow - previousMicros >= interval)
  {
    fps = loopCounter;
    loopCounter = 0;
    previousMicros = timeNow;
    lastCmdCount = cmdCount;
    cmdCount = 0;
  }

  // Check for new received data
  Receive();
  // delay(50);
  // SendFront(30, 0);
  // Send commands
  if (iTimeSend > timeNow)
    return;
  iTimeSend = timeNow + 1000; // Set TIME_SEND to 200 microseconds
  SendFront(8, 0);
  cmdCount++;

  // Calculate test command signal
  iTest += iStep;

  // invert step if reaching limit
  if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
    iStep = -iStep;

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);  
}

// ########################## END ##########################
