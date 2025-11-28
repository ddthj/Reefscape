
float get_power(double drive_angle, double motor_angle, double robot_angle, float rot, float power, float friction){
  // Determines the power output to drive a motor at to move the robot in the desired direction
  float local_angle = drive_angle - robot_angle;
  float translation = power * cos(local_angle - motor_angle);
  float raw_output = rot + (1-fabs(rot)) * translation;
  return friction * sgn(translation) + (1-friction) * raw_output;
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