#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include "car.h"
#include <webots/distance_sensor.h>

//仿真时间步长
float TIME_STEP= 10.;
//组件定义

WbDeviceTag flw;
WbDeviceTag frw;;
WbDeviceTag ps;
WbDeviceTag imu;
WbDeviceTag camera;


//雷达，IMU，相机 电机位置 数据变量
double *imu_val = NULL;
const unsigned char *img;
double dis_val= 0;

double ang_add= 0;
//流程控制参数
short process_ =0 ;
double ang_move = 0.;
double SPEED_move = 6;
int time_count= 0;
short temp = 0;
double temp2 = 0;

void Set_Speed(SPEED *spe);


void mainfunc(void){
    
    TAR tar;//目标点1
    SPEED spe;//速度类
 
    imu_val = wb_inertial_unit_get_roll_pitch_yaw(imu);
    img = wb_camera_get_image(camera);
    dis_val = wb_distance_sensor_get_value(ps);
    // printf("%d\n",detect_count);

    tar = tar_get(img);
    
    ang_add = (acos(tar.x /tar.len) - PI_/2);
    
    
    ang_move = imu_val[2] + ang_add;
    
    
    //避障部分
    if (dis_val < 800 && temp == 0){
      temp = 1;
      temp2 = imu_val[2] + PI_/3;
    }
    if (temp == 1){
      if (fabs(imu_val[2] - temp2) < 0.1){
          temp =2;
      }
      
      ang_move = imu_val[2] + 0.17;
      
    }else if (temp == 2){
      ang_move = imu_val[2] - 0.28;
      
      if (detect_count > 500){
          temp =0;
      }
    }
    
    // 识别结点标志
    if (YELLOW_COUNT > 200){
      printf("%d\n",detect_count);
      imgsplit = 2;
    }else {
      imgsplit = 1;
    }

    if (GREEN_COUNT > 1000){
      SPEED_move = 0;
    }else {
      SPEED_move = 4;
    }
    
    //速度转化
    spe = angkeep(imu_val[2],ang_move,1);

    spe.w_l +=SPEED_move;
    spe.w_r +=SPEED_move;
    Set_Speed(&spe);
    
}


void Set_Speed(SPEED *spe){
  //限制最大速度 该车子最大速度为6.4
  spe->w_l = val_limit(spe->w_l,-10,10);
  spe->w_r = val_limit(spe->w_r,-10,10);
  wb_motor_set_velocity(flw, spe->w_l);
  wb_motor_set_velocity(frw, spe->w_r);

}


int main() {

  wb_robot_init();

  imu = wb_robot_get_device("imu");
  camera = wb_robot_get_device("camera");

  
  flw = wb_robot_get_device("l_wheel");
  frw = wb_robot_get_device("r_wheel");

  //设置电机角度为无限制
  wb_motor_set_position(flw, INFINITY);
  wb_motor_set_position(frw, INFINITY);

  //使能 雷达, IMU, 相机 指定工作频率
  wb_inertial_unit_enable(imu, TIME_STEP);
  wb_camera_enable(camera, TIME_STEP);
  
  ps = wb_robot_get_device("diss"); 
  wb_distance_sensor_enable(ps, TIME_STEP);
    
  while (wb_robot_step(TIME_STEP) != -1) {
    mainfunc();
    
  };

  wb_robot_cleanup();


  return 0;
}




