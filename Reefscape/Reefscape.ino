#include <Alfredo_NoU2.h>
#include <PestoLink-Receive.h>
#include <HardwareSerial.h>
#include <Adafruit_BNO08x.h>

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

Adafruit_BNO08x gyro(-1);
sh2_SensorValue_t quat;    //https://github.com/adafruit/Adafruit_BNO08x/blob/67b91b809da04a08fccb8793770343f872daaf43/src/sh2_SensorValue.h#L186
struct euler_t {double yaw; double pitch; double roll;} rot;

bool gyro_good = false;


NoU_Motor one(2);
//double rad1 = DEG_TO_RAD * 45.0;
double rad1 = DEG_TO_RAD * 30.0;
NoU_Motor two(3);
//double rad2 = DEG_TO_RAD * 135.0;
double rad2 = DEG_TO_RAD * 150.0;
NoU_Motor three(4);
//double rad3 = DEG_TO_RAD * 225.0;
double rad3 = DEG_TO_RAD * 210.0;
NoU_Motor four(1);
//double rad4 = DEG_TO_RAD * 315.0;
double rad4 = DEG_TO_RAD * 330.0;

float joy_x, joy_y, joy_rot = 0.0;

int target_tag = -1;
int tag_id = -1;
float tag_x = 0.0;
float tag_z = 0.0;
float tag_r = 0.0;
u_long heartbeat = 0;
const int tag_led = 26;


//Drive modes:
// 0 = Field-oriented control
// 1 = Robot-oriented control
// 2 = Tag-oriented control
bool toggle_drive = false;
int drive_mode = 0;
double target_heading = 0.0;
double drive_angle = 0.0;
float drive_rotation = 0;


void setup() {
  PestoLink.begin("Werbenjagermanjensen");
  Serial.begin(115200);
  Serial.println("Hello!");
  Serial2.begin(115200, SERIAL_8N1, 5, 4);
  if (gyro.begin_I2C()) {
    gyro_good = true;
      delay(1);
      gyro.enableReport(SH2_GAME_ROTATION_VECTOR);
  }
  configureMotor(&one);
  configureMotor(&two);
  configureMotor(&three);
  configureMotor(&four);
  pinMode(tag_led, OUTPUT);
  RSL::initialize();
  RSL::setState(RSL_DISABLED);
}


void loop() {
  if (PestoLink.update()) {
    joy_x = -PestoLink.getAxis(0);
    joy_y = -PestoLink.getAxis(1);
    joy_rot = -PestoLink.getAxis(2);

    if (toggle_drive == false){
      toggle_drive = PestoLink.buttonHeld(3);
      if (toggle_drive == true){
        if (drive_mode == 0) {drive_mode = 1;}
        else {drive_mode = 0;}
      }
    } else{
      toggle_drive = PestoLink.buttonHeld(3);
    }

    RSL::setState(RSL_ENABLED);
  }
  else {
    joy_x = 0.0;
    joy_y = 0.0;
    joy_rot = 0.0;
    RSL::setState(RSL_DISABLED);
  }

  if (gyro_good && gyro.getSensorEvent(&quat)){
    quaternionToEuler(&quat, &rot);
  }

  if (drive_mode == 0){
    // Field-oriented control
    drive_angle = atan2(joy_x, joy_y) - rot.yaw; // Rotate to field north
  } else if (drive_mode == 1){
    // Robot-oriented control
    drive_angle = atan2(joy_x, joy_y);
  }
  if (fabs(joy_rot) > 0.1){
    target_heading = rot.yaw;
    drive_rotation = joy_rot * 0.5;
  } else {
    double error = wrap(target_heading - rot.yaw);
    drive_rotation = error * 0.5;
  }

  float pwr = fmin(fabs(joy_x) + fabs(joy_y) + fabs(drive_rotation), 1.0);
  if (pwr > 0.0) {
    drive(drive_angle, pwr, drive_rotation);
  }


  updateTag();
  updateHeartbeat();
  
  RSL::update();
}





