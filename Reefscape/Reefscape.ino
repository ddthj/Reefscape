#include <Alfredo_NoU2.h>
#include <PestoLink-Receive.h>
#include <HardwareSerial.h>
#include <Adafruit_BNO08x.h>

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

Adafruit_BNO08x gyro(-1);
sh2_SensorValue_t quat;    //https://github.com/adafruit/Adafruit_BNO08x/blob/67b91b809da04a08fccb8793770343f872daaf43/src/sh2_SensorValue.h#L186
struct euler_t {double yaw; double pitch; double roll;} rot;




NoU_Motor one(2);
//double one_rad = DEG_TO_RAD * 45.0;
double one_rad = DEG_TO_RAD * 30.0;
NoU_Motor two(3);
//double two_rad = DEG_TO_RAD * 135.0;
double two_rad = DEG_TO_RAD * 150.0;
NoU_Motor three(4);
//double three_rad = DEG_TO_RAD * 225.0;
double three_rad = DEG_TO_RAD * 210.0;
NoU_Motor four(1);
//double four_rad = DEG_TO_RAD * 315.0;
double four_rad = DEG_TO_RAD * 330.0;

float joy_x, joy_y, joy_rot = 0.0;

int target_tag = -1;
int tag_id = -1;
float tag_x = 0.0;
float tag_z = 0.0;
float tag_r = 0.0;
u_long tag_heartbeat = 0;
const int tag_led = 2;



void setup() {

  PestoLink.begin("Werbenjagermanjensen");
  Serial.begin(115200);
  Serial.println("Hello!");
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  //Gyro Connection
  if (!gyro.begin_I2C()) {
    delay(1);
  }
  gyro.enableReport(SH2_GAME_ROTATION_VECTOR);
  
  RSL::initialize();
  RSL::setState(RSL_DISABLED);
  pinMode(tag_led, OUTPUT);

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

  if (gyro.getSensorEvent(&quat)){
    quaternionToEuler(&quat, &rot);
  }

  updateTag();
  updateHeartbeat();



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

  one.set(get_power(desired_direction, one_rad, rot.yaw, z, magnitude, 0.15));
  two.set(get_power(desired_direction, two_rad, rot.yaw, z, magnitude, 0.15));
  three.set(get_power(desired_direction, three_rad, rot.yaw, z, magnitude, 0.15));
  four.set(get_power(desired_direction, four_rad, rot.yaw, z, magnitude, 0.15));
}


