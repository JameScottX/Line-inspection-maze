#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define PI_ 3.1415926
#define W 80
#define H 80

#define YELLOW 0x01

#define GREEN 0x03


//定义速度结构体
typedef  struct {
    float w_l ;
    float w_r ;
} SPEED;
//定义点结构体
typedef struct {
    float x ;
    float y ;
    float len;
} TAR;
//定义图片结构体
typedef struct {
    unsigned char r[W][H] ;
    unsigned char g[W][H] ;
    unsigned char b[W][H] ;
} IMG;
//定义图片全局变量
IMG img1;
IMG img2;

const short IMG_W  = W;
const short IMG_H  = H;
unsigned int detect_count = 0;
short stop_count = 0;
short YELLOW_COUNT = 0;
short GREEN_COUNT = 0;
short imgsplit= 1;

//雷达数据有效参数
short laser_eff[512];
int lidar_num = 512 ;
float lidar_max_range = 3;
//pd控制器 临时参数
double err_yaw_last =0.;
//定义最大速度 和一般前进速度
float max_speed = 6.3;

double pd_control(double dis,double k, double d);
IMG img_process(IMG img);

//长度计算函数
__inline double len_get(double x, double y){
    double len = sqrt(x*x+ y *y);
    return len;
}

__inline bool black_get(unsigned char col[3]){
          
  short max_ = 30;
  if (col[0] < max_ && col[1] < max_ && col[2] < max_){return true;}
  else {return false;}
}  
//多种颜色线判断依据
__inline bool color_who(short color, unsigned char col[3]){
    short max_ = 190,min_ = 90;
    if (color == YELLOW && col[0] > max_ && col[1] > max_ && col[2] < min_){return true;}

    max_ = 190,min_ = 90;
    if (color == GREEN && col[0] < min_ && col[1] > max_ && col[2]< min_){return true;}
    
    return false;
}

//巡线图像处理，获得期望点
TAR tar_get(const unsigned char *img){

    TAR xy_c;
    IMG img_temp;
    int  NUM = IMG_W * IMG_H ;
    int t_r = 0, t_c = 0;
    const unsigned char *_img_ = img;//取首个地址
    int r_add =0,c_add=0;
    unsigned char pix[3] = {0};
    //对webots相机数据进行获取
    for (int i=0 ; i< NUM ; i +=1, _img_ +=4){
        t_c = i % IMG_W;
        t_r = (int)i / IMG_W;
        img1.b[t_r][t_c] = (unsigned char )*_img_;
        img1.g[t_r][t_c] = (unsigned char )*(_img_ +1);
        img1.r[t_r][t_c] = (unsigned char )*(_img_+2);  
    }
    
    img_temp = img_process(img1);
    
    detect_count =0;
    stop_count = 0;
    YELLOW_COUNT = 0;

    GREEN_COUNT = 0;
    for (short r = 1; r < IMG_W  ;r+=1){
            for (short c = 1; c < (short)(IMG_H / imgsplit) ;c+=1){
                pix[0] = img_temp.r[r][c];
                pix[1] = img_temp.g[r][c];
                pix[2] = img_temp.b[r][c];
                
                if (black_get(pix)){
                    r_add+=r;
                    c_add+=c;
                    detect_count+=1;
                }
                
                if (color_who(YELLOW,pix)){
                    YELLOW_COUNT+=1;
                }
                if (color_who(GREEN,pix)){
                    GREEN_COUNT+=1;
                }
                
                
            }
    }
    

    if (detect_count ==0){
        detect_count =1;
    }

    xy_c.y = IMG_H - r_add / detect_count;
    xy_c.x = c_add / detect_count - 0.5* IMG_W;
    xy_c.len = len_get(xy_c.x ,xy_c.y);
    

    
    return xy_c;
}

//图片处理函数，以下为 3*3 卷积核的均值滤波操作
IMG img_process(IMG img){

    IMG temp_;
    float addr= 0,addg= 0,addb= 0;
    
    
    for (short r = 1; r < IMG_W-1 ;r+=1){
            for (short c = 1; c < IMG_H-1 ;c+=1){

                    addr= 0.;addg= 0.;addb= 0.;
                    
                    for (short i= 0;i<3;i+=1){
                        for (short j= 0;j<3;j+=1){
                        
                            addr += img.r[r-1 +i][c-1 + j] ;
                            addg += img.g[r-1 +i][c-1 + j] ;
                            addb += img.b[r-1 +i][c-1 + j] ;
                        }
                        
                    }
                    temp_.r[r][c]= (unsigned char) (addr/9);
                    temp_.g[r][c]= (unsigned char) (addg/9);
                    temp_.b[r][c]= (unsigned char) (addb/9);
            }
        }
      
      return temp_;
}


//简单的PD控制器
double pd_control(double dis,double k, double d){
    double temp_ = 0., dis_2 = 0.;
    dis_2 = dis - err_yaw_last;
    temp_ = k * dis + d*(0-dis_2);
    err_yaw_last = dis;
    return temp_;
}

//限制幅度函数
double val_limit(double val , double min , double max){
    double temp_ = 0.;
    temp_ = val;
    if (temp_ < min)temp_ = min;
    if (temp_ > max)temp_ = max;
    return temp_;
}

//PD控制器保持姿态
SPEED angkeep(double yaw,double yaw_d, double L){
    SPEED temp_;
    double dis =0, yaw_w =0,speed=0;
    dis = yaw_d - yaw;
    yaw_w = pd_control(dis, 12., 4. );
    speed = val_limit(yaw_w * L/2,-max_speed,max_speed);
    temp_.w_l = -speed;
    temp_.w_r = speed;
    return temp_;
}




