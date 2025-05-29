#include <PID_v1.h>

double setpoint = 8.0; // Desired output voltage
double input, output;  
double Kp = 0.1, Ki = 0.09, Kd = 0.001; 

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int duty_cycle = 40; 
const int DUTY_MIN = 0;
const int DUTY_MAX = 70;

void setup() {
    pinMode(9, OUTPUT);
    
    TCCR1A = _BV(COM1A1) | _BV(WGM11); // Fast PWM, clear on compare match
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // No prescaling
    ICR1 = 319; // 50 kHz frequency (16 MHz / (50 kHz * 1) - 1)
    OCR1A = map(duty_cycle, 0, 100, 0, 319);

    Serial.begin(9600);

    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-100, 100);
}

void loop() {
    int adc_value = analogRead(A0);
    float voltage_at_pin = adc_value * (5.0 / 1023.0);
    input = voltage_at_pin * (20.0 + 10.0) / 10.0;

    myPID.Compute();

    duty_cycle += output;
    if (duty_cycle > DUTY_MAX) duty_cycle = DUTY_MAX;
    if (duty_cycle < DUTY_MIN) duty_cycle = DUTY_MIN;

    OCR1A = map(duty_cycle, 0, 100, 0, 319);

    readSerialInput();

    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" V | Actual Voltage: ");
    Serial.print(input);
    Serial.print(" V | Duty Cycle: ");
    Serial.print(duty_cycle);
    Serial.print(" % | PID Output: ");
    Serial.println(output);

    delay(100);
}

void readSerialInput() {
  static bool newData = false;
  static String inputString = "";
  while (Serial.available() > 0) {  
    char c = Serial.read();  
    if (c == '\n') {  
      newData = true;
      break;  
    } else if (isDigit(c) || c == '.' || c == '-') {  
      inputString += c;  
    }
  }

  if (newData) {  
    setpoint = inputString.toFloat();  
    inputString = "";  
    newData = false;  
  }
}