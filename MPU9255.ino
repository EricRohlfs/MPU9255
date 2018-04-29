/*
Raw data example
This example reads raw readings from the magnetometer gyroscope and the accelerometer and then
displays them in serial monitor.
*/

#include <MPU9255.h>//include MPU9255 library


#define g 9.81 // 1g ~ 9.81 m/s^2
#define magnetometer_cal 0.06 //magnetometer calibration
#define acc_div 16384.0
#define gyr_div 131.0
#define alpha_a  0.8
#define alpha_g  0.5 
#define alpha_m  0.7
#define magnetometer_cal 0.06 //magnetometer calibration
#define yaw_axis_correc 90
#define toDeg 57.3
#define toRad 0.01745200698

#define magCalx 9.81
#define magCaly 24.725
#define magCalz -27.395

MPU9255 mpu;

double ax,ax_F,ax_F_prev,ay,ay_F,ay_F_prev,az,az_F,az_F_prev;
double gx,gx_F,gx_prev,gx_prev_F,gy,gy_F,gy_prev,gy_prev_F,gz,gz_F,gz_prev,gz_prev_F;
double roll, pitch, yaw, yaw_F;
double mx,my,mz;
double mx_h, my_h;
double mx_max,my_max,mz_max = -100000;
double mx_min,my_min,mz_min = 100000;


double low_pass_filter(double val, double prev_filtered_val, double alpha){
  double filtered_val;
  filtered_val = (1-alpha)*val+alpha*prev_filtered_val;
  return filtered_val;     
}
  
double high_pass_filter(double val,double prev_val, double prev_filtered_val, double alpha){
  double filtered_val;
  filtered_val = (1-alpha)*prev_filtered_val+(1-alpha)*(val-prev_val);
  return filtered_val;
}

double process_magnetic_flux(int16_t input, double sensitivity)
{
  return (input*magnetometer_cal*sensitivity)/0.6;
}

void calibrate_mag();

void setup() {
  Serial.begin(115200);//initialize Serial port
  mpu.init();//initialize MPU9255 chip

  mpu.set_acc_scale(scale_2g);//set accelerometer scale
  mpu.set_gyro_scale(scale_250dps);//set gyroscope scale

  // get initial values for complementary filter
  mpu.read_acc();//get data from the accelerometer
  mpu.read_gyro();//get data from the gyroscope
  mpu.read_mag();//get data from the magnetometer

  ax = mpu.ax;
  ax_F_prev = low_pass_filter(ax, ax, alpha_a);
  ay = mpu.ay;
  ay_F_prev = low_pass_filter(ay, ay, alpha_a);
  az = mpu.az;
  az_F_prev = low_pass_filter(az, az, alpha_a);

  gx = mpu.gx;
  gx_prev = mpu.gx;
  gx_prev_F = high_pass_filter(gx,gx_prev, gx, alpha_g);
  gy = mpu.gy;
  gy_prev = mpu.gy;
  gy_prev_F = high_pass_filter(gy,gy_prev, gy, alpha_g);
  gz = mpu.gz;
  gz_prev = mpu.gz;
  gz_prev_F = high_pass_filter(gz,gz_prev, gz, alpha_g);


  mx = process_magnetic_flux(mpu.mx,mpu.mx_sensitivity) - magCalx;
  my = process_magnetic_flux(mpu.my,mpu.my_sensitivity) - magCaly;
  mz = process_magnetic_flux(mpu.mz,mpu.mz_sensitivity) - magCalz;

  
  yaw = atan2(my,mx)*57.3;
  
  
}

void loop() {
  
  mpu.read_acc();//get data from the accelerometer
  mpu.read_gyro();//get data from the gyroscope
  mpu.read_mag();//get data from the magnetometer

  //print all data in serial monitor
  ax = mpu.ax;
  ay = mpu.ay;
  az = mpu.az;

  // Accelorometer
  ax_F = low_pass_filter(ax, ax_F_prev, alpha_a);
  ay_F = low_pass_filter(ay, ay_F_prev, alpha_a);
  az_F = low_pass_filter(az, az_F_prev, alpha_a);


  //roll = atan2(ay_F , az_F) * toDeg; // deg
  //pitch = atan2((- ax_F) , sqrt(ay_F * ay_F + az_F * az_F)) * toDeg; // deg

  pitch = atan2(ax_F,az_F)*toDeg;
  roll = atan2(ay_F,az_F)*toDeg;

  mx = low_pass_filter(mx, process_magnetic_flux(mpu.mx,mpu.mx_sensitivity) - magCalx, alpha_m);
  my = low_pass_filter(mx, process_magnetic_flux(mpu.my,mpu.my_sensitivity) - magCaly, alpha_m);
  mz = low_pass_filter(mx, process_magnetic_flux(mpu.mz,mpu.mz_sensitivity) - magCalz, alpha_m);


  mx_h = my*cos(pitch*toRad) + mx*sin(pitch*toRad)*sin(roll*toRad) + mz*sin(pitch*toRad)*cos(roll*toRad);
  my_h = mx*cos(roll*toRad) + mz*sin(roll*toRad);

  
  yaw = atan2(-my_h, mx_h) * toDeg;

  
   

  ax_F_prev = ax_F;
  ay_F_prev = ay_F;
  az_F_prev = az_F;


/***********************************************/
  // Gyroscope
  gx = mpu.gx;
  gy = mpu.gy;
  gz = mpu.gz;


  
  gx_F = high_pass_filter(gx,gx_prev, gx_prev_F, alpha_g);
  gy_F = high_pass_filter(gy,gy_prev, gy_prev_F, alpha_g);
  gz_F = high_pass_filter(gz,gz_prev, gz_prev_F, alpha_g);

  gx_prev = gx;
  gy_prev = gy;
  gz_prev = gz;

  gx_prev_F = gx_F;
  gy_prev_F = gy_F;
  gz_prev_F = gz_F;




  Serial.print(" Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Yaw: ");
  Serial.print(yaw);
  Serial.println();
 
  delay(100);
}


void calibrate_mag(){
  // move in figure 8 
  for (int i = 0;i<10000;i++){
    // Max Mag
  if (mx > mx_max){
    mx_max = mx;
    }
  if (my > my_max){
    my_max = my;
    }
  if (mz > mz_max){
    mz_max = mz;
    }

    
  // min Mag
  if (mx < mx_min){
    mx_min = mx;
    }
  if (my < my_min){
    my_min = my;
    }
  if (mz < mz_min){
    mz_min = mz;
    }
  
  Serial.print("Min [ ");
  Serial.print(mx_min);
  Serial.print(", ");
  Serial.print(my_min);
  Serial.print(", ");
  Serial.print(mz_min);
  Serial.print(" ] . ");
  Serial.print("Max [ ");
  Serial.print(mx_max);
  Serial.print(", ");
  Serial.print(my_max);
  Serial.print(", ");
  Serial.print(mz_max);
  Serial.print(" ] . ");
  Serial.println();
  }
  
  }
