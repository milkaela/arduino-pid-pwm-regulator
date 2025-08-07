// Closed-loop control using PWM (Pulse Width Modulation)
int adc_target = 0;
float P = 0, I = 0, D = 0;

float integral = 0;
float previous_error = 0;
unsigned long last_time = 0;

// Clear the Serial input buffer
void clearSerialBuffer() {
  while (Serial.available() > 0) Serial.read();
}

// Initialization
void setup() {
  pinMode(A5, INPUT);
  pinMode(4, OUTPUT);
  Serial.begin(9600);  // Adjust depending on your board version

  float target_voltage = 0;
  const float Vref = 5.0;

  // Read the desired voltage from the user (e.g., 2.5V)
  Serial.println("Enter desired voltage (e.g., 2.5 for 2.5V):");
  while (Serial.available() == 0);
  target_voltage = Serial.parseFloat();
  clearSerialBuffer();

  // Convert voltage to ADC value (0–1023)
  adc_target = (int)(target_voltage * 1023.0 / Vref);
  adc_target = constrain(adc_target, 0, 1023);  // Protection
  Serial.print("adc_target (converted) = "); Serial.println(adc_target);

  // PID control formula:
  // output = P ⋅ error + I ⋅ ∫error⋅dt + D ⋅ d(error)/dt

  // Read P coefficient
  Serial.println("Enter P (e.g., 1.0):");
  while (Serial.available() == 0);
  P = Serial.parseFloat();
  clearSerialBuffer();
  Serial.print("P = "); Serial.println(P);

  // Read I coefficient
  Serial.println("Enter I (e.g., 0.1):");
  while (Serial.available() == 0);
  I = Serial.parseFloat();
  clearSerialBuffer();
  Serial.print("I = "); Serial.println(I);

  // Read D coefficient
  Serial.println("Enter D (e.g., 0.01):");
  while (Serial.available() == 0);
  D = Serial.parseFloat();
  clearSerialBuffer();
  Serial.print("D = "); Serial.println(D);

  last_time = millis();
}

void loop() {
  int adc = analogRead(A5);
  float error = adc_target - adc;

  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  if (dt <= 0) dt = 0.001; // Avoid division by zero

  // Integral term calculation
  integral += error * dt;
  integral = constrain(integral, -300, 300); // Anti-windup

  // Derivative term calculation
  float derivative = (error - previous_error) / dt;
  // Calculated as the difference between current and previous error over time

  // PID Control Output
  float output = P * error + I * integral + D * derivative;
  int pwmVal = constrain(output + 128, 0, 255); // Offset + PWM limit (128 = neutral)

  analogWrite(4, pwmVal);

  // Display values in Serial Monitor / Plotter
  Serial.print("ADC="); Serial.print(adc);
  Serial.print(" Target="); Serial.print(adc_target);
  Serial.print(" PWM="); Serial.println(pwmVal);

  previous_error = error;
  last_time = now;

  delay(100); // Sampling at 10Hz
}
