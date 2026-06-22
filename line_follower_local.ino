// =============================================================================
//  STANDALONE AUTONOMOUS LINE FOLLOWER — LOCAL SENSOR LOGIC ONLY
//  Surface: Low-Friction Smooth Board | Target: White Line on Black Background
//  PWM API: ESP32 Arduino Core 3.0+ (ledcAttach / ledcWrite)
// =============================================================================

// ── 5-BIT LINE Array SENSOR PINS (MD0482) ───────────────────────────────────
constexpr uint8_t S1 = 23; // Far Left
constexpr uint8_t S2 = 22; // Left
constexpr uint8_t S3 = 21; // Center
constexpr uint8_t S4 = 19; // Right
constexpr uint8_t S5 = 18; // Far Right

// ── MOTOR DRIVER OUTPUT PINS (GOLDEN WORKING CONFIGURATION) ──────────────────
constexpr uint8_t PWMA_PIN = 12; // Speed Left
constexpr uint8_t AIN1_PIN = 13; // Direction Left 1   
constexpr uint8_t AIN2_PIN = 14; // Direction Left 2   

constexpr uint8_t PWMB_PIN = 25; // Speed Right
constexpr uint8_t BIN1_PIN = 26; // Direction Right 1   
constexpr uint8_t BIN2_PIN = 27; // Direction Right 2   

// ── PWM FREQUENCY SETUP ──────────────────────────────────────────────────────
constexpr uint32_t PWM_FREQ       = 20000; // 20kHz Ultrasound Frequency
constexpr uint8_t  PWM_RESOLUTION = 8;     // 0-255 Speed Range

// ── SENSOR POLARITY ──────────────────────────────────────────────────────────
constexpr uint8_t LINE_ACTIVE = HIGH; // Highly reflective white line = HIGH [cite: 1788]

// ── CALIBRATED SPEED BOOST PROFILES ──────────────────────────────────────────
constexpr int CRUISE_SPEED  = 135; // Controlled velocity on straight line segments [cite: 1893]
constexpr int STEER_SPEED   = 175; // Outside wheel turning power for gentle curves [cite: 1893]
constexpr int PIVOT_SPEED   = 75;  // Inner wheel turning power for gentle curves [cite: 1893]

// Edge Sensor Parameters (Snap-Pivot Controls for Outer S1 & S5) [cite: 1893]
constexpr int SHARP_STEER   = 170; // Outside wheel push force [cite: 1893]
constexpr int SHARP_REVERSE = -110;// Aggressive inside wheel reverse braking anchor [cite: 1893, 1895]

// Timeout Search Radar Parameters (For recovery if track is completely lost) [cite: 1893]
constexpr int SCAN_SPEED = 105;    // Slow rotation speed to safely spot the line [cite: 1893, 1885]
constexpr int SCAN_TIMEOUT = 950;  // Milliseconds to sweep one way before flipping [cite: 1893, 1884]

// ── STATE VARIABLES ──────────────────────────────────────────────────────────
int lastSeenSide = 0;   // Memory: -1 for Left, 1 for Right, 0 for Center [cite: 1870, 1871]
unsigned long lostTime = 0;
bool isScanning = false;
int scanDirection = 1;  // 1 = Scan Right, -1 = Scan Left

void setup() {
    Serial.begin(115200); [cite: 1526]
    Serial.println(F("\n=== Standalone Autonomous Line Follower Ready ==="));

    // Configure all 5 sensor pins as inputs
    pinMode(S1, INPUT); pinMode(S2, INPUT); pinMode(S3, INPUT); pinMode(S4, INPUT); pinMode(S5, INPUT);

    // Configure H-Bridge Direction pins as outputs
    const uint8_t dirPins[] = { AIN1_PIN, AIN2_PIN, BIN1_PIN, BIN2_PIN };
    for (uint8_t p : dirPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

    // Connect hardware PWM pins via ESP32 Board Core v3.0+ library
    ledcAttach(PWMA_PIN, PWM_FREQ, PWM_RESOLUTION); [cite: 1514, 1516]
    ledcAttach(PWMB_PIN, PWM_FREQ, PWM_RESOLUTION); [cite: 1514, 1516]

    stopMotors();
    Serial.println(F("Place car on white track line. Starting in 3 seconds..."));
    delay(3000);
}

void loop() {
    // Read current digital states from your 5-bit array
    int r1 = digitalRead(S1);
    int r2 = digitalRead(S2);
    int r3 = digitalRead(S3);
    int r4 = digitalRead(S4);
    int r5 = digitalRead(S5);

    // Check if any sensor at all is currently capturing the line
    bool lineDetected = (r1 == LINE_ACTIVE || r2 == LINE_ACTIVE || r3 == LINE_ACTIVE || r4 == LINE_ACTIVE || r5 == LINE_ACTIVE);

    if (lineDetected) {
        // Line found! Disengage recovery mode
        isScanning = false;

        // Active tracking memory updates
        if (r1 == LINE_ACTIVE || r2 == LINE_ACTIVE) {
            lastSeenSide = -1; // Line drifted to the Left side history [cite: 1870]
        } else if (r4 == LINE_ACTIVE || r5 == LINE_ACTIVE) {
            lastSeenSide = 1;  // Line drifted to the Right side history [cite: 1871]
        } else if (r3 == LINE_ACTIVE) {
            lastSeenSide = 0;  // Line is perfectly Centered [cite: 1874]
        }
    }

    // ── ACTIVE TRICK LOGIC CASE MATRIX ───────────────────────────────────────
    
    // CASE 1: PERFECTLY CENTERED
    if (r3 == LINE_ACTIVE) {
        setLeftMotor(CRUISE_SPEED);
        setRightMotor(CRUISE_SPEED);
    }
    
    // CASE 2: DRIFTED SLIGHTLY RIGHT (Inner Left sensor S2 hits white)
    else if (r2 == LINE_ACTIVE) {
        setLeftMotor(PIVOT_SPEED);
        setRightMotor(STEER_SPEED);
    }
    
    // CASE 3: DRIFTED SLIGHTLY LEFT (Inner Right sensor S4 hits white)
    else if (r4 == LINE_ACTIVE) {
        setLeftMotor(STEER_SPEED);
        setRightMotor(PIVOT_SPEED);
    }
    
    // CASE 4: HARD RIGHT DRIFT / 90° LEFT BEND (Outer Left sensor S1 hits white)
    else if (r1 == LINE_ACTIVE) {
        setLeftMotor(SHARP_REVERSE); // Reverse inside wheel into a mechanical anchor [cite: 1895]
        setRightMotor(SHARP_STEER);  // Push outside wheel hard to rotate chassis
    }
    
    // CASE 5: HARD LEFT DRIFT / 90° RIGHT BEND (Outer Right sensor S5 hits white)
    else if (r5 == LINE_ACTIVE) {
        setLeftMotor(SHARP_STEER);   // Push outside wheel hard to rotate chassis
        setRightMotor(SHARP_REVERSE); // Reverse inside wheel into a mechanical anchor [cite: 1895]
    }
    
    // CASE 6: OVERSHOT BEND RECOVERY — ACTIVE RADAR STATE MACHINE
    else {
        // If the car has just crossed into black space, lock current time frame
        if (!isScanning) {
            isScanning = true;
            lostTime = millis();
            // Turn towards the last seen historical path direction
            scanDirection = (lastSeenSide != 0) ? lastSeenSide : 1; 
        }

        unsigned long currentLostDuration = millis() - lostTime;

        // If sweep timer has crossed our limit threshold, flip rotation direction to look wider [cite: 1884]
        if (currentLostDuration > SCAN_TIMEOUT) {
            scanDirection = -scanDirection; // Invert scan sweep
            lostTime = millis();            // Reset window timer boundary
            Serial.println(F("[RADAR SCAN] Inverting sweep window..."));
        }

        // Execute physical sweep commands based on logic direction flags
        if (scanDirection == -1) {
            setLeftMotor(-SCAN_SPEED); // Spin Left in place
            setRightMotor(SCAN_SPEED);
            Serial.println(F("[LOST] Radar sweeping LEFT"));
        } else {
            setLeftMotor(SCAN_SPEED);  // Spin Right in place
            setRightMotor(-SCAN_SPEED);
            Serial.println(F("[LOST] Radar sweeping RIGHT"));
        }
    }

    delay(10); // Super-fast sampling interval
}

// ── REVERSIBLE ANALOG MOTOR ROUTINES ─────────────────────────────────────────
static void setLeftMotor(int speed) {
    if (speed >= 0) {
        digitalWrite(AIN1_PIN, HIGH);
        digitalWrite(AIN2_PIN, LOW);
        ledcWrite(PWMA_PIN, speed); // Core v3.0+ directly accepts the pin variable [cite: 1517]
    } else {
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, HIGH);
        ledcWrite(PWMA_PIN, -speed); 
    }
}

static void setRightMotor(int speed) {
    if (speed >= 0) {
        digitalWrite(BIN1_PIN, HIGH);
        digitalWrite(BIN2_PIN, LOW);
        ledcWrite(PWMB_PIN, speed);
    } else {
        digitalWrite(BIN1_PIN, LOW);
        digitalWrite(BIN2_PIN, HIGH);
        ledcWrite(PWMB_PIN, -speed); 
    }
}

void stopMotors() {
    digitalWrite(AIN1_PIN, LOW); digitalWrite(AIN2_PIN, LOW); ledcWrite(PWMA_PIN, 0);
    digitalWrite(BIN1_PIN, LOW); digitalWrite(BIN2_PIN, LOW); ledcWrite(PWMB_PIN, 0);
}
