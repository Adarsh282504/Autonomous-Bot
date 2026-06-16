// --- Ultrasonic Sensor Pins ---
#define TRIG_FRONT 26
#define ECHO_FRONT 27
#define TRIG_LEFT 28
#define ECHO_LEFT 29
#define TRIG_RIGHT 30
#define ECHO_RIGHT 31

// --- Motor Driver Pins (L298N) ---
#define IN1_LEFT 51
#define IN2_LEFT 50
#define ENA_LEFT 7

#define IN1_RIGHT 43
#define IN2_RIGHT 42
#define ENB_RIGHT 6

// --- Encoder Pins ---
#define ENCODER_LEFT_A 20
#define ENCODER_LEFT_B 21
#define ENCODER_RIGHT_A 19
#define ENCODER_RIGHT_B 18

// --- Bump Sensors (2 Interrupt Pins) ---
#define BUMP_LEFT_PIN 2     // Left bump sensor (Interrupt 0)
#define BUMP_RIGHT_PIN 3    // Right bump sensor (Interrupt 1)

// --- Constants ---
#define OBSTACLE_DISTANCE 30
#define MOTOR_SPEED 110         
#define TURN_SPEED 100         
#define ENCODER_TICKS_PER_CM 10  
#define WHEEL_BASE_CM 13
#define TURN_90_TICKS (3.1416 * WHEEL_BASE_CM * ENCODER_TICKS_PER_CM / 2)
#define OPEN_SPACE_THRESHOLD 100
#define OBSTACLE_DISTANCE1 40 // for side wall
#define LEFT_CORRECTION 0.85   // ⚙️ correction factor for left motor (reduce by 15%) // cjevk it once


// --- Global Variables ---
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile bool bumpTriggered = false;
bool toMotion = true;
int axischeck = 0;
int largerwall=0;

// --- Encoder ISR ---
void countLeftEncoder() { leftEncoderCount++; }
void countRightEncoder() { rightEncoderCount++; }

// --- Bump ISR ---
void bumpISR() {
  static unsigned long lastTrigger = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastTrigger > 300) {
    bumpTriggered = true;
    Serial.println("⚡ Interrupt Triggered: BUMP DETECTED (Left or Right)!");
  }

  lastTrigger = currentTime;
}

// --- Ultrasonic Distance Function ---
long getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000);
  long distance = duration * 0.0343 / 2;
  if (distance == 0) distance = 400;
  return distance;
}

// --- Stop Function ---
void stopBot() {
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, LOW);
  analogWrite(ENA_LEFT, 0);
  analogWrite(ENB_RIGHT, 0);
  Serial.println("🛑 Bot Stopped!");
}

// --- Motion Functions ---
void moveForwardToMotion() {
  int leftSpeed = MOTOR_SPEED * LEFT_CORRECTION;
  int rightSpeed = MOTOR_SPEED;

  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);
  analogWrite(ENA_LEFT, leftSpeed);
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
  analogWrite(ENB_RIGHT, rightSpeed);
  Serial.println("🚗 Moving Forward (To Motion)");
}

void moveForwardByDistance(int distanceCM) {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  long targetTicks = distanceCM * ENCODER_TICKS_PER_CM;

  int leftSpeed = MOTOR_SPEED * LEFT_CORRECTION;
  int rightSpeed = MOTOR_SPEED;

  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);
  analogWrite(ENA_LEFT, leftSpeed);
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
  analogWrite(ENB_RIGHT, rightSpeed);

  while ((leftEncoderCount + rightEncoderCount) / 2 < targetTicks) {
    if (bumpTriggered) return;
  }
  stopBot();
}

void moveBackwardByDistance(int distanceCM) {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  long targetTicks = distanceCM * ENCODER_TICKS_PER_CM;

  int leftSpeed = MOTOR_SPEED * LEFT_CORRECTION;
  int rightSpeed = MOTOR_SPEED;

  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, HIGH);
  analogWrite(ENA_LEFT, leftSpeed);
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, HIGH);
  analogWrite(ENB_RIGHT, rightSpeed);

  while ((leftEncoderCount + rightEncoderCount) / 2 < targetTicks) {
    if (bumpTriggered) return;
  }
  stopBot();
}

// --- Turns ---
void rightTurn90Encoder() {
  Serial.println("↩ Right 90° Turn");
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  int leftSpeed = TURN_SPEED * LEFT_CORRECTION;
  int rightSpeed = TURN_SPEED;

  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);
  analogWrite(ENA_LEFT, leftSpeed);
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, HIGH);
  analogWrite(ENB_RIGHT, rightSpeed);

  while ((leftEncoderCount + rightEncoderCount) / 2 < TURN_90_TICKS) {
    if (bumpTriggered) return;
  }
  stopBot();
}

void leftTurn90Encoder() {
  Serial.println("↪ Left 90° Turn");
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  int leftSpeed = TURN_SPEED * LEFT_CORRECTION;
  int rightSpeed = TURN_SPEED;

  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, HIGH);
  analogWrite(ENA_LEFT, leftSpeed);
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
  analogWrite(ENB_RIGHT, rightSpeed);

  while ((leftEncoderCount + rightEncoderCount) / 2 < TURN_90_TICKS) {
    if (bumpTriggered) return;
  }
  stopBot();
}

// --- Wall Side Detection ---
bool detectWallSide() {
  long leftDist = 0, rightDist = 0;

  for (int i = 0; i < 30; i++) {
    leftDist += getDistanceCM(TRIG_LEFT, ECHO_LEFT);
    rightDist += getDistanceCM(TRIG_RIGHT, ECHO_RIGHT);
    delay(100);
  }

  leftDist /= 30;
  rightDist /= 30;

  Serial.print("📏 Left Wall Avg Distance: "); Serial.println(leftDist);
  Serial.print("📏 Right Wall Avg Distance: "); Serial.println(rightDist);

  if (leftDist < rightDist) {
    Serial.println("🧱 Wall Closer on LEFT → Start with FRO Motion");
    return false;
  } else {
    Serial.println("🧱 Wall Closer on RIGHT → Start with TO Motion");
    return true;
  }
}

// --- Handle Bump ---
void handleBump() {
  Serial.println("⚠️ Bump Detected! Executing Evasive Action...");
  stopBot();
  delay(1000);

  moveBackwardByDistance(30);
  delay(1000);

  if (toMotion) {
    rightTurn90Encoder();
    delay(800);
    moveForwardByDistance(25);
    delay(800);
    rightTurn90Encoder();
    toMotion = false;
  } else {
    leftTurn90Encoder();
    delay(800);
    moveForwardByDistance(30);
    delay(800);
    leftTurn90Encoder();
    toMotion = true;
  }

  bumpTriggered = false;
  Serial.println("✅ Bump Recovered, Resuming Path!");
}

// ---Initial Wall Detection for x axis---
void walldet() {
  moveForwardByDistance(40);

  if (detectWallSide() == false) {
    while (getDistanceCM(TRIG_FRONT, ECHO_FRONT) > OBSTACLE_DISTANCE) moveForwardToMotion();
    stopBot();
    delay(800);
    rightTurn90Encoder();
    delay(800);
    moveForwardByDistance(30);
    delay(800);
    rightTurn90Encoder();
    toMotion = false;
  } else {
    while (getDistanceCM(TRIG_FRONT, ECHO_FRONT) > OBSTACLE_DISTANCE) moveForwardToMotion();
    stopBot();
    delay(800);
    leftTurn90Encoder();
    delay(800);
    moveForwardByDistance(30);
    delay(800);
    leftTurn90Encoder();
    toMotion = true;
  }

  delay(1000);
  Serial.println("🧭 Zig-Zag Mission Started!");
}

// --- Time Limit ---
unsigned long startTime;
const unsigned long maxRunTime = 270000; // 4 min 30 sec

// --- Setup ---
void setup() {
  Serial.begin(9600);
  Serial.println("🚀 BOT INITIALIZING...");

  pinMode(TRIG_FRONT, OUTPUT); 
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT); 
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); 
  pinMode(ECHO_RIGHT, INPUT);

  pinMode(IN1_LEFT, OUTPUT); 
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(ENA_LEFT, OUTPUT);
  pinMode(IN1_RIGHT, OUTPUT); 
  pinMode(IN2_RIGHT, OUTPUT);
  pinMode(ENB_RIGHT, OUTPUT);

  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(BUMP_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUMP_RIGHT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BUMP_LEFT_PIN), bumpISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUMP_RIGHT_PIN), bumpISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), countLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), countRightEncoder, RISING);

  Serial.println("✅ Initialization Complete! Detecting Wall Side...");

  moveForwardByDistance(40);

  if (detectWallSide() == false) {
    while (getDistanceCM(TRIG_FRONT, ECHO_FRONT) > OBSTACLE_DISTANCE)
    moveForwardToMotion();
    stopBot(); 
    delay(500);
    rightTurn90Encoder(); 
    delay(500);
    moveForwardByDistance(30); 
    delay(500);
    rightTurn90Encoder(); 
    toMotion = false;
  } else {
    while (getDistanceCM(TRIG_FRONT, ECHO_FRONT) > OBSTACLE_DISTANCE) moveForwardToMotion();
    stopBot(); 
    delay(500);
    leftTurn90Encoder(); 
    delay(500);
    moveForwardByDistance(30); delay(500);
    leftTurn90Encoder(); toMotion = true;
  }

  delay(1000);
  Serial.println("🧭 Zig-Zag Mission Started!");
  startTime = millis();
}

// --- Main Loop ---
void loop() {
  if (millis() - startTime >= maxRunTime) {
    stopBot();
    Serial.println("⏱️ Time up! Stopping bot after 4 min 30 sec.");
    while (true);
  }

   if (axischeck==0){
  if (bumpTriggered) {
    handleBump();
    return;
  }

  long frontDist = getDistanceCM(TRIG_FRONT, ECHO_FRONT);
  long leftDist = getDistanceCM(TRIG_LEFT, ECHO_LEFT);
  long rightDist = getDistanceCM(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("Front: "); 
  Serial.print(frontDist);
  Serial.print(" | Left: ");
  Serial.print(leftDist);
  Serial.print(" | Right: "); 
  Serial.println(rightDist);

  // --- WALL DETECTION LOGIC (STABLE VERSION) ---
  const int STABLE_COUNT = 60;                 // number of consecutive confirmations (yaha par badhana hai for real case scenario for end wall detection)
  static int leftCount = 0;
  static int rightCount = 0;

  if (leftDist < OBSTACLE_DISTANCE1 + 5) {
    if (leftCount < STABLE_COUNT) leftCount++;
  } else {
    if (leftCount > 0) leftCount--;
  }

  if (rightDist < OBSTACLE_DISTANCE1 + 5) {
    if (rightCount < STABLE_COUNT) rightCount++;
  } else {
    if (rightCount > 0) rightCount--;
  }

  bool leftDetected = (leftCount >= STABLE_COUNT);
  bool rightDetected = (rightCount >= STABLE_COUNT);

  static bool prevLeft = false;
  static bool prevRight = false;

  if (leftDetected != prevLeft) {
    if (leftDetected) Serial.println("✅ LEFT WALL CONFIRMED");
    else Serial.println("❌ LEFT WALL LOST");
    prevLeft = leftDetected;
  }

  if (rightDetected != prevRight) {
    if (rightDetected) Serial.println("✅ RIGHT WALL CONFIRMED");
    else Serial.println("❌ RIGHT WALL LOST");
    prevRight = rightDetected;
  }
  

  //---End Wall Detection
  if (frontDist <= OBSTACLE_DISTANCE) {
    bool isLeftEnd = leftDist < OBSTACLE_DISTANCE1 && (rightDist > OPEN_SPACE_THRESHOLD);
    bool isRightEnd = rightDist < OBSTACLE_DISTANCE1 && (leftDist > OPEN_SPACE_THRESHOLD);

    if (isLeftEnd) {
      Serial.println("🎯 LEFT WALL END DETECTED!");
      rightTurn90Encoder();
      moveForwardByDistance(30);
      stopBot();
      Serial.println("✅ MISSION COMPLETE!");
      axischeck=1;  // for x axis
      Serial.println("🔄 Switching to X-axis cleaning...");
      return;
    } else if (isRightEnd) {
      Serial.println("🎯 RIGHT WALL END DETECTED!");
      leftTurn90Encoder();
      moveForwardByDistance(30);
      stopBot();
      Serial.println("✅ MISSION COMPLETE!");
      axischeck=1;  // for x axis
      Serial.println("🔄 Switching to X-axis cleaning...");
      return; 
    }
  }


  //--  Zig Zag
  if (frontDist > OBSTACLE_DISTANCE) {
    moveForwardToMotion();
  } else {
    stopBot();
    delay(500);

    if (toMotion) {
      rightTurn90Encoder();
      delay(500);
      moveForwardByDistance(30);
      delay(500);
      rightTurn90Encoder();
      toMotion = false;
    } else {
      leftTurn90Encoder();
      delay(500);
      moveForwardByDistance(30);
      delay(500);
      leftTurn90Encoder();
      toMotion = true;
    }
  }
  }
  delay(300);

  if (axischeck==1){
    if(largerwall=0){
    walldet();
    largerwall++;
    }
  if (bumpTriggered) {
    handleBump();
    return;
  }
  long frontDist = getDistanceCM(TRIG_FRONT, ECHO_FRONT);
  long leftDist = getDistanceCM(TRIG_LEFT, ECHO_LEFT);
  long rightDist = getDistanceCM(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("Front: "); 
  Serial.print(frontDist);
  Serial.print(" | Left: ");
  Serial.print(leftDist);
  Serial.print(" | Right: "); 
  Serial.println(rightDist);

  // --- WALL DETECTION LOGIC (STABLE VERSION) ---
  const int STABLE_COUNT = 60;                 // number of consecutive confirmations (yaha par badhana hai for real case scenario for end wall detection)
  static int leftCount = 0;
  static int rightCount = 0;

  if (leftDist < OBSTACLE_DISTANCE1 + 5) {
    if (leftCount < STABLE_COUNT) leftCount++;
  } else {
    if (leftCount > 0) leftCount--;
  }

  if (rightDist < OBSTACLE_DISTANCE1 + 5) {
    if (rightCount < STABLE_COUNT) rightCount++;
  } else {
    if (rightCount > 0) rightCount--;
  }

  bool leftDetected = (leftCount >= STABLE_COUNT);
  bool rightDetected = (rightCount >= STABLE_COUNT);

  static bool prevLeft = false;
  static bool prevRight = false;

  if (leftDetected != prevLeft) {
    if (leftDetected) Serial.println("✅ LEFT WALL CONFIRMED");
    else Serial.println("❌ LEFT WALL LOST");
    prevLeft = leftDetected;
  }

  if (rightDetected != prevRight) {
    if (rightDetected) Serial.println("✅ RIGHT WALL CONFIRMED");
    else Serial.println("❌ RIGHT WALL LOST");
    prevRight = rightDetected;
  }
  

  //---End Wall Detection
  if (frontDist <= OBSTACLE_DISTANCE) {
    bool isLeftEnd = leftDist < OBSTACLE_DISTANCE1 && (rightDist > OPEN_SPACE_THRESHOLD);
    bool isRightEnd = rightDist < OBSTACLE_DISTANCE1 && (leftDist > OPEN_SPACE_THRESHOLD);

    if (isLeftEnd) {
      Serial.println("🎯 LEFT WALL END DETECTED!");
      stopBot();
      Serial.println("✅ MISSION COMPLETE!");
      while (true){
        stopBot();
      }
    } else if (isRightEnd) {
      Serial.println("🎯 RIGHT WALL END DETECTED!");
      stopBot();
      Serial.println("✅ MISSION COMPLETE!");
      while (true){
        stopBot();
      }
    }
  }


  //--  Zig Zag
  if (frontDist > OBSTACLE_DISTANCE) {
    moveForwardToMotion();
  } else {
    stopBot();
    delay(500);

    if (toMotion) {
      rightTurn90Encoder();
      delay(500);
      moveForwardByDistance(30);
      delay(500);
      rightTurn90Encoder();
      toMotion = false;
    } else {
      leftTurn90Encoder();
      delay(500);
      moveForwardByDistance(30);
      delay(500);
      leftTurn90Encoder();
      toMotion = true;
    }
  }
  }
}

