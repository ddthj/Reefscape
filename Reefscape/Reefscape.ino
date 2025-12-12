#include <Alfredo_NoU2.h>
#include <PestoLink-Receive.h>
#include <HardwareSerial.h>
#include <Adafruit_BNO08x.h>

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

Adafruit_BNO08x gyro(-1);
sh2_SensorValue_t quat;    //https://github.com/adafruit/Adafruit_BNO08x/blob/67b91b809da04a08fccb8793770343f872daaf43/src/sh2_SensorValue.h#L186
struct euler_t {double yaw; double pitch; double roll;} rot;

bool gyro_good = false;


NoU_Motor one(1);
double rad1 = DEG_TO_RAD * 35.0;
NoU_Motor two(2);
double rad2 = DEG_TO_RAD * 125.0;
NoU_Motor three(3);
double rad3 = DEG_TO_RAD * 215.0;
NoU_Motor four(4);
double rad4 = DEG_TO_RAD * 305.0;

float joy_x, joy_y, joy_rot = 0.0;

//Drive modes:
// 0 = Field-oriented control
// 1 = Robot-oriented control
// 2 = Tag-oriented control
bool toggle_drive = false;
int drive_mode = 0;
double target_heading = 0.0;
double drive_angle = 0.0;
float drive_rotation = 0;
float drive_power = 0;

const int tag_led = 26;
int tag_id = -1;
float tag_x = 0.0;
float tag_z = 0.0;
float tag_r = 0.0;
u_long heartbeat = 0;

int target_tag_id = -1;
float target_tag_x = 0.0;
float target_tag_z = 0.0;

float target_tag_r = 0.0;
u_long target_heartbeat = 0;

int auto_align = 0;
float align_z1 = -0.3;
float align_z2 = -0.1;
float align_x1 = 0.0;
float align_gain = 5;
float align_avg_speed = 0.25; // m/s
u_long dead_time = 0;

NoU_Servo ele_one(1);
int ele_pos[] = {0, 90, 180};
int ele_len = 2;
int ele_index = 0;
bool ele_input = false;




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
  two.setInverted(true);
  three.setInverted(true);

  ele_one.write(90);

  pinMode(tag_led, OUTPUT);
  RSL::initialize();
  RSL::setState(RSL_DISABLED);
}


void loop() {
  if (PestoLink.isConnected()) {
    joy_x = -PestoLink.getAxis(0);
    joy_y = -PestoLink.getAxis(1);
    joy_rot = -PestoLink.getAxis(2);
    
    
    if (PestoLink.buttonHeld(14)){
      auto_align = 14;
    } else if (PestoLink.buttonHeld(15)){
      auto_align = 15;
    } else {
      auto_align = 0;
    }

    if (!ele_input) {
      if (PestoLink.buttonHeld(12) && ele_index < ele_len){
        ele_input = true;
        ele_index++;
        ele_one.write(ele_pos[ele_index]);
      } else if (PestoLink.buttonHeld(13) && ele_index > 0){
        ele_input = true;
        ele_index--;
        ele_one.write(ele_pos[ele_index]);
      }
    } else if (!PestoLink.buttonHeld(12) && !PestoLink.buttonHeld(13)){
      ele_input = false;
    }  
    

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

  if (auto_align == 0){
    target_tag_id = -1;
    if (drive_mode == 0){
      // Field-oriented control
      drive_angle = atan2(joy_x, joy_y) - rot.yaw; // Rotate to field north
    } else if (drive_mode == 1){
      // Robot-oriented control
      drive_angle = atan2(joy_x, joy_y);
    }

    if (fabs(joy_rot) > 0.05){
      target_heading = rot.yaw;
      drive_rotation = fmin(joy_rot, 0.5);
    } else {
      double error = wrap(target_heading - rot.yaw);
      drive_rotation = error * 0.5;
    }

    drive_power = fmin(fabs(joy_x) + fabs(joy_y) + fabs(drive_rotation), 1.0);
  } else{
    if (target_tag_id == -1){
      target_tag_id = tag_id; // Select most recently seen tag to lock on to
    } else if (target_tag_id == tag_id){
      target_tag_x = tag_x;
      target_tag_z = tag_z;
      target_tag_r = tag_r;
      target_heartbeat = heartbeat;
      target_heading = wrap(target_tag_r + rot.yaw);
    }

    //TODO - multi-step alignment
    float x_error = align_x1 - target_tag_x;
    float z_error = align_z1 - target_tag_z;

    float error_distance = sqrt(x_error * x_error + z_error * z_error);
    //1000 ms/s * m * m/s
    int error_time = (int)(1000 * error_distance / align_avg_speed);

    //Serial.println("Rotation Stuff!");
    //Serial.print(target_heading); Serial.print(" "); Serial.println(rot.yaw);
    //Serial.println("Updated Tag Tracking!");
    //Serial.print("X: "); Serial.print(x_error); Serial.print(" Z: "); Serial.println(z_error);
    //Serial.print("Distance: "); Serial.print(error_distance); Serial.print(" time: "); Serial.println(error_time);
    //Serial.println(millis() - target_heartbeat);
    if (millis() - target_heartbeat < error_time){
      drive_angle = wrap(atan2(x_error, z_error) + target_tag_r);
      drive_power = fmin(align_gain * error_distance, 0.66);
      drive_rotation = wrap(target_heading - rot.yaw)  * 0.33;
    } else {
      drive_power = 0.0;
      drive_rotation = 0.0;
    }
  }

  drive(drive_angle, drive_power, drive_rotation);


  updateTag();
  updateHeartbeat();
  RSL::update();
}





