#define HOVER_SERIAL_BAUD 115200  // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD 115200        // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xABCD        // [-] Start frme definition for reliable serial communication
#define START_FRAME2 0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND 100             // [ms] Sending time interval
#define SPEED_MAX_TEST 1000       // [-] Maximum speed for testing
#define SPEED_STEP 30             // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#include <U8glib.h>
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_FAST);

#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(2, 3);  // RX, TX
unsigned long StartTime = millis();


int speedR;
int speedL;
int speedR2;
int speedL2;
int bat;
int bat2;

// Global variables
uint8_t idx = 0;         // Index for new data pointer
uint16_t bufStartFrame;  // Buffer Start Frame
byte *p;                 // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

uint8_t idx2 = 0;         // Index for new data pointer
uint16_t bufStartFrame2;  // Buffer Start Frame
byte *p2;                 // Pointer declaration for the new received data
byte incomingByte2;
byte incomingBytePrev2;

typedef struct {
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct {
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



typedef struct {
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



// ########################## SETUP ##########################
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  //HoverSerial.begin(HOVER_SERIAL_BAUD);
  Serial1.begin(HOVER_SERIAL_BAUD);
  Serial2.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);

  if (u8g.getMode() == U8G_MODE_R3G3B2) {
    u8g.setColorIndex(255);  // white
  } else if (u8g.getMode() == U8G_MODE_GRAY2BIT) {
    u8g.setColorIndex(3);  // max intensity
  } else if (u8g.getMode() == U8G_MODE_BW) {
    u8g.setColorIndex(1);  // pixel on
  } else if (u8g.getMode() == U8G_MODE_HICOLOR) {
    u8g.setHiColorByRGB(255, 255, 255);
  }

  //RunTest();
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed) {
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  Serial1.write((uint8_t *)&Command, sizeof(Command));
  Serial2.write((uint8_t *)&Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
void Receive() {
  // Check for new data availability in the Serial buffer
  if (Serial1.available()) {
    incomingByte = Serial1.read();                                       // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;  // Construct the start frame
  }

  if (Serial2.available()) {
    incomingByte2 = Serial2.read();                                         // Read the incoming byte
    bufStartFrame2 = ((uint16_t)(incomingByte2) << 8) | incomingBytePrev2;  // Construct the start frame
  }



// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME) {  // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Copy received data
  if (bufStartFrame2 == START_FRAME2) {  // Initialize if new data is detected
    p2 = (byte *)&NewFeedback2;
    *p2++ = incomingBytePrev2;
    *p2++ = incomingByte2;
    idx2 = 2;
  } else if (idx2 >= 2 && idx2 < sizeof(SerialFeedback2)) {  // Save the new received data
    *p2++ = incomingByte2;
    idx2++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      // Serial.print("cmd1: ");
      // Serial.print(Feedback.cmd1);
      // Serial.print(" cmd2: ");
      // Serial.print(Feedback.cmd2);
      Serial.print(" speedR: ");
      Serial.print(Feedback.speedR_meas);
      Serial.print(" speedL: ");
      Serial.print(Feedback.speedL_meas);
      // Serial.print(" batVoltage: ");
      // Serial.print(Feedback.batVoltage);
      // Serial.print(" boardTemp: ");
      // Serial.println(Feedback.boardTemp);
      // Serial.print(" 7: ");
      // Serial.println(Feedback.cmdLed);
      Serial.println();

      speedR = Feedback.speedR_meas;
      speedL = Feedback.speedL_meas;
      bat = Feedback.batVoltage;

    } else {
      //Serial.println("Non-valid data skipped");
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;


  // Check if we reached the end of the package
  if (idx2 == sizeof(SerialFeedback2)) {
    uint16_t checksum2;
    checksum2 = (uint16_t)(NewFeedback2.start ^ NewFeedback2.cmd1 ^ NewFeedback2.cmd2 ^ NewFeedback2.speedR_meas ^ NewFeedback2.speedL_meas
                           ^ NewFeedback2.batVoltage ^ NewFeedback2.boardTemp ^ NewFeedback2.cmdLed);

    // Check validity of the new data
    if (NewFeedback2.start == START_FRAME2 && checksum2 == NewFeedback2.checksum) {
      // Copy the new data
      memcpy(&Feedback2, &NewFeedback2, sizeof(SerialFeedback2));

      // Print data to built-in Serial
      // Serial.print("cmd1: ");
      // Serial.print(Feedback.cmd1);
      // Serial.print(" cmd2: ");
      // Serial.print(Feedback.cmd2);
      Serial.print("                                            speedR: ");
      Serial.print(Feedback2.speedR_meas);
      Serial.print(" speedL: ");
      Serial.print(Feedback2.speedL_meas);
      // Serial.print(" batVoltage: ");
      // Serial.print(Feedback.batVoltage);
      // Serial.print(" boardTemp: ");
      // Serial.println(Feedback.boardTemp);
      // Serial.print(" 7: ");
      // Serial.println(Feedback.cmdLed);
      Serial.println();

      speedR2 = Feedback2.speedR_meas;
      speedL2 = Feedback2.speedL_meas;
      bat2 = Feedback2.batVoltage;

    } else {
      //Serial.println("                                           xx Non-valid data skipped");
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev2 = incomingByte2;
}



// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;

void draw(void) {
  // graphic commands to redraw the complete screen should be placed here
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, 20);
  u8g.print(speedR);
  u8g.setPrintPos(30, 20);
  u8g.print(speedL);
  u8g.setPrintPos(0, 50);
  u8g.print(bat);

  u8g.setPrintPos(60, 20);
  u8g.print(speedR2);
  u8g.setPrintPos(90, 20);
  u8g.print(speedL2);
  u8g.setPrintPos(60, 50);
  u8g.print(bat2);
}

void loop(void) {
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  if (timeNow - StartTime > 15000) {
    iTest = 0;
  }
  Send(0, 0);

  // Calculate test command signal
  iTest += iStep;
  if (iTest >= SPEED_MAX_TEST) {
    iTest = SPEED_MAX_TEST;
  }

  // // invert step if reaching limit
  // if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
  //   iStep = -iStep;

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);

  u8g.firstPage();
  do {
    draw();
  } while (u8g.nextPage());
}

// ########################## END ##########################
