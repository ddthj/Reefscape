// Util functions for Reefbot
// In no particular order...


double wrap(double a) {
  a = fmod(a + PI, TWO_PI);
  if (a < 0){ a += TWO_PI; }
  return a - PI;
}


void configureMotor(NoU_Motor* motor) {
  // Configures the motor curves for better drive handling
  motor->setMinimumOutput(0.25); // Require more power to start moving
  motor->setDeadband(0.01);
  motor->setExponent(0.5); // Rapidly add power with diminishing return.
}

void drive(double dir, float pwr, float r) {
  float raw1 = pwr * cos(dir - rad1) * (1-fabs(r));
  float raw2 = pwr * cos(dir - rad2) * (1-fabs(r));
  float raw3 = pwr * cos(dir - rad3) * (1-fabs(r));
  float raw4 = pwr * cos(dir - rad4) * (1-fabs(r));

  float s = fmax(fmax(fabs(raw1), fabs(raw2)), fmax(fabs(raw3), fabs(raw4))) / pwr;

  float pwr1 = (raw1 + r) / s;
  float pwr2 = (raw2 + r) / s;
  float pwr3 = (raw3 + r) / s;
  float pwr4 = (raw4 + r) / s;

  one.set(pwr1);
  two.set(pwr2);
  three.set(pwr3);
  four.set(pwr4);
}

void quaternionToEuler(sh2_SensorValue_t* quat, euler_t* ypr) {
  float qr = quat->un.gameRotationVector.real;
  float qi = quat->un.gameRotationVector.i;
  float qj = quat->un.gameRotationVector.j;
  float qk = quat->un.gameRotationVector.k;
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = (double)atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = (double)asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = (double)atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
}

void updateTag() {
  if (Serial2.available() > 0){
    String newline = Serial2.readStringUntil('\n');
    int commaCount = 0;
    for (int i = 0; i < newline.length(); i++) {if (newline.charAt(i) == ',') {commaCount++;}}
    // No crashing! Bad!
    if (commaCount != 3) {return;}

    char buffer[newline.length() + 1];
    newline.toCharArray(buffer, newline.length() + 1);

    char* token = strtok(buffer, ",");
    tag_id = atoi(token);
    token = strtok(NULL, ",");
    tag_x = atof(token);
    token = strtok(NULL, ",");
    tag_z = atof(token);
    token = strtok(NULL, ",");
    tag_r = atof(token);
    //Serial.print("Detected Tag ");
    //Serial.print(tag_id);
    //Serial.print(": ");
    //Serial.print(tag_x);
    //Serial.print("x, ");
    //Serial.print(tag_z);
    //Serial.print("z, ");
    //Serial.println(tag_r);
    heartbeat = millis();
    // TAG DATA IS GIVEN IN THE TAG'S COORDINATE SYSTEM
    // positive x is left
    // negative z is forward. Tags can be detected as close as 0.05m but 0.10m is much more reliable.
    // positive r is counterclockwise. This is the difference between the robot's angle and the tag's angle
    // where 0 means the bot is normal to the tag's plane

  }
}

void updateHeartbeat() {
  if (heartbeat == 0){
    digitalWrite(tag_led, HIGH);
    return;
  }
  long delay = 10;
  if (tag_id != -1) {delay = 250;}
  if (millis() < heartbeat + delay){
    digitalWrite(tag_led, HIGH);
  } else {
    digitalWrite(tag_led, LOW);
  }
}