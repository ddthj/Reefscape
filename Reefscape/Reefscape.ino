#include <Alfredo_NoU2.h>
#include <PestoLink-Receive.h>
#include <HardwareSerial.h>
#include <Adafruit_BNO08x.h>

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

Adafruit_BNO08x gyro(-1);
sh2_SensorValue_t quat;    //https://github.com/adafruit/Adafruit_BNO08x/blob/67b91b809da04a08fccb8793770343f872daaf43/src/sh2_SensorValue.h#L186

struct euler_t {
  double yaw;
  double pitch;
  double roll;
} rot;

bool use_gyro = true;

NoU_Motor one(1);
double one_rad = DEG_TO_RAD * 45;
//double one_rad = DEG_TO_RAD * 30;
NoU_Motor two(2);
double two_rad = DEG_TO_RAD * 135;
//double two_rad = DEG_TO_RAD * 150;
NoU_Motor three(3);
double three_rad = DEG_TO_RAD * 225;
//double three_rad = DEG_TO_RAD * 210;
NoU_Motor four(4);
double four_rad = DEG_TO_RAD * 315;
//double four_rad = DEG_TO_RAD * 330;

float joy_x, joy_y, joy_rot = 0.0;
float friction = 0.15;
float rot_strength = 0.5;


void setup() {

  PestoLink.begin("Werbenjagermanjensen");
  Serial.begin(115200);
  Serial.println("Hello!");
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  if (use_gyro){
      //Gyro Connection
    if (!gyro.begin_I2C()) {
      delay(1);
    }
    gyro.enableReport(SH2_GAME_ROTATION_VECTOR);
  }
  
  RSL::initialize();
  RSL::setState(RSL_DISABLED);

}


void loop() {
  if (PestoLink.update()) {
    joy_x = -PestoLink.getAxis(0);
    joy_y = -PestoLink.getAxis(1);
    joy_rot = -PestoLink.getAxis(2);

    RSL::setState(RSL_ENABLED);
  }
  else {
    joy_x = 0.0;
    joy_y = 0.0;
    joy_rot = 0.0;
    RSL::setState(RSL_DISABLED);
  }

  if (use_gyro && gyro.getSensorEvent(&quat)){
    quaternionToEuler(&quat, &rot);
    Serial.println(rot.yaw);
  }

  drive(joy_x, joy_y, joy_rot);
  RSL::update();
}


void drive(float x, float y, float z) {
  float magnitude = abs(x) + abs(y);
  if (magnitude != 0.0){
    x /= magnitude;
    y /= magnitude;
  }

  double desired_direction = atan2(x, y);

  one.set(get_motor_power(desired_direction, one_rad, magnitude, z));
  two.set(get_motor_power(desired_direction, two_rad, magnitude, z));
  three.set(get_motor_power(desired_direction, three_rad, magnitude, z));
  four.set(get_motor_power(desired_direction, four_rad, magnitude, z));
}

float get_motor_power(float d, float a, float s, float z){
  double c = cos(a-d);
  return friction * sgn(c) + z * rot_strength + (1 - friction) * (1 - abs(z * rot_strength)) * c * s;
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
