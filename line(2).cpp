#include "serialPort/SerialPort.h"
#include <math.h>
#include <stdio.h>

#define TIME_STEP 0.01       // 控制周期
#define PI 3.1415926
#define deg_2_rad(X) ( X / 180.0 * PI )
#define rad_2_deg(X) ( X / PI * 180.0 )

// 机械臂参数
#define LINK1 0.115            // 第一段连杆长度（单位：米）
#define LINK2 0.220            // 第二段连杆长度（单位：米）
#define LINK3 0.367           // 第三段连杆长度（单位：米）

// 电机参数
#define MOTOR_COUNT 3        // 电机数量
double motor_offset[MOTOR_COUNT] = {0.0, 0.0, 0.0}; // 电机初始位置偏移

// 五次多项式规划
double S5_Plan(double S_Pos, double all_t, double t) {
    return (10 * pow(t, 3) / pow(all_t, 3) - 15 * pow(t, 4) / pow(all_t, 4) + 6 * pow(t, 5) / pow(all_t, 5)) * S_Pos;
}

// 假设逆解函数（需替换为你的实际函数），逆解求出来的是转化矩阵
void inverse_kinematics(double x, double y, double z, double x0, double y0, double z0, double *theta1, double *theta2, double *theta3) {
    double c3, s3, z_;
    x = x - x0;
    y = y - y0;
    z_ = z - z0;
    double theta1_deg, theta2_deg, theta3_deg;
    c3 = (pow(x, 2) + pow(y, 2) + pow(z_, 2) - pow(LINK2, 2) - pow(LINK3, 2)) / (2 * LINK2 * LINK3); // cos(theta3)
    s3 = sqrt(1 - pow(c3, 2)); // sin(theta3)

    *theta1 = atan2(y, x);  // 计算第一关节角度
    *theta2 = atan2(z_ * (LINK2 + LINK3 * c3) - LINK3 * s3 * sqrt(pow(x, 2) + pow(y, 2)), (LINK2 + LINK3 * c3) * sqrt(pow(x, 2) + pow(y, 2)) + z_ * LINK3 * s3); // 计算第二关节角度
    *theta3 = atan2(s3, c3); // 计算第三关节角度
    theta1_deg = rad_2_deg(*theta1);
    theta2_deg = rad_2_deg(*theta2);
    theta3_deg = rad_2_deg(*theta3);
    printf("theta1:%lf    ", theta1_deg);
    printf("theta2:%lf ", theta2_deg);
    printf("theta3:%lf\n ", theta3_deg);
}

// 封装直线运动函数
void linear_motion(SerialPort &serial, MotorCmd &cmd, MotorData &data, double start_x, double start_y, double start_z, double stop_x, double stop_y, double stop_z, double z0, double *init_Pos) {
    double step = 100; // 步数
    for (double p = 1; p <= step; p++) {
        // 分割点
        double x2 = start_x + p / step * (stop_x - start_x);
        double y2 = start_y + p / step * (stop_y - start_y);
        double z2 = start_z + p / step * (stop_z - start_z);
        printf("x2 = %lf, y2 = %lf, z2 = %lf \n\n", x2, y2, z2);

        // 运动学逆解
        double theta1, theta2, theta3;
        inverse_kinematics(x2, y2, z2, 0, 0, z0, &theta1, &theta2, &theta3);

        // 控制电机运动
        double target_pos[MOTOR_COUNT] = {theta1, theta2+ 0.194* PI, theta3 - 0.921* PI};
        for (int i = 0; i < MOTOR_COUNT; i++) {
            cmd.motorType = MotorType::GO_M8010_6;
            cmd.id = i;
            cmd.mode = 1; // 位置控制模式
            cmd.K_P = 1.5;
            cmd.K_W = 0.1;
            cmd.Pos = 6.33 * target_pos[i] + init_Pos[i];
            serial.sendRecv(&cmd, &data);
        }
        usleep(10000); // 每步间隔 10ms
    }
}

int main() {
    SerialPort serial("/dev/ttyUSB0"); // 串口初始化
    MotorCmd cmd;
    MotorData data;

    // 记录上电时电机初始位置
    double init_Pos[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++) {
        cmd.motorType = MotorType::GO_M8010_6;
        cmd.id = i;
        cmd.mode = 0; // 读取模式
        serial.sendRecv(&cmd, &data);
        init_Pos[i] = data.Pos; // 保存初始位置
        printf("Motor %d Init Pos: %f rad\n", i, data.Pos);
        usleep(200);
    }


    // 定义路径点（全局坐标）
    double x0 = 0, y0 = 0.0, z0 = 0.115; // 坐标系零点  0° -30° 166° 
    double start_x = 0.220, start_y = 0.0, start_z = 0.482; // 直线出发点
    double stop_x = 0.220, stop_y = 0.35, stop_z = 0.482; // 第一段终点
    double stop2_x = 0.220, stop2_y = -0.35, stop2_z = 0.482; // 第二段终点
    double stop3_x = -0.220, stop3_y = -0.35, stop3_z = 0.482; // 第二段终点
    double stop4_x = -0.068983, stop4_y = 0, stop4_z = 0.264508; // 第二段终点

    double t_total = 3; // 每段运动的时间
    double t = 0;

    // 从点0到出发点的运动
    while (t <= t_total) {
        // 目标点2（全局坐标系）
        double theta1, theta2, theta3;

        // 控制电机运动
        double target_pos[MOTOR_COUNT] = {0,  0.194* PI,  - 0.421* PI};
        for (int i = 0; i < MOTOR_COUNT; i++) {
            cmd.motorType = MotorType::GO_M8010_6;
            cmd.id = i;
            cmd.mode = 1; // 位置控制模式
            cmd.K_P = 1.5;
            cmd.K_W = 0.1;
            cmd.Pos = S5_Plan(6.33 * target_pos[i], t_total, t) + init_Pos[i];
            serial.sendRecv(&cmd, &data);
        }
        t += TIME_STEP;
        usleep(TIME_STEP * 1e6); // 延时
    }

    //到达出发点后，修正初始位置
//    init_Pos[1] = init_Pos[1] + 0.194* PI;
//    init_Pos[2] = init_Pos[2]  - 0.921* PI;


    usleep(1000000);
    // 执行直线运动
    printf("Moving to first line...\n");
    linear_motion(serial, cmd, data, start_x, start_y, start_z, stop_x, stop_y, stop_z, z0, init_Pos);

   usleep(1000000); // 延时 1s

    printf("Moving to second line...\n");
    linear_motion(serial, cmd, data, stop_x, stop_y, stop_z, stop2_x, stop2_y, stop2_z, z0, init_Pos);
    usleep(1000000); // 延时 1s
    printf("Moving to third line...\n");
    linear_motion(serial, cmd, data, stop2_x, stop2_y, stop2_z, stop3_x, stop3_y, stop3_z, z0, init_Pos);
   
    usleep(1000000); // 延时 1s

    double init_Pos_2[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++) {
        cmd.motorType = MotorType::GO_M8010_6;
        cmd.id = i;
        cmd.mode = 0; // 读取模式
        serial.sendRecv(&cmd, &data);
        init_Pos_2[i] = data.Pos; // 保存初始位置
        printf("Motor %d Init Pos: %f rad\n", i, data.Pos);
        usleep(200);
    }
    //回到终止点 
    double step = 200; // 步数
    for (double p = 1; p <= step; p++) 
    {  
        
     //回到终止点
        cmd.motorType = MotorType::GO_M8010_6;
        cmd.id    = 0;
        cmd.mode  = 1;
        cmd.K_P   = 0.5;//位置刚度P
        cmd.K_W   = 0.2;//速度刚度D
        cmd.Pos = (6.33*0- init_Pos_2[0]+init_Pos[0]) * p / step + init_Pos_2[0];
        cmd.W     = 0 ;// 期望关节速度(电机转子转速 rad/s)         ±804.00
        cmd.T     = 0.0;// 期望关节的输出力矩(电机转子转矩 N.m) 范围: ±127.99
        serial.sendRecv(&cmd,&data);
        

        cmd.motorType = MotorType::GO_M8010_6;
        cmd.id    = 1;
        cmd.mode  = 1;
        cmd.K_P   = 1.5;//位置刚度P
        cmd.K_W   = 0.2;//速度刚度D
        //cmd.Pos   = (6.33*PI/2 +init_Pos[1])* p / step;  //对目标值进行分隔
        cmd.Pos = (6.33*PI/2- init_Pos_2[1]+init_Pos[1] + 6.33*0.194* PI) * p / step + init_Pos_2[1];
        cmd.W     = 0 ;// 期望关节速度(电机转子转速 rad/s)         ±804.00
        cmd.T     = 0.0;// 期望关节的输出力矩(电机转子转矩 N.m) 范围: ±127.99
        serial.sendRecv(&cmd,&data);


        cmd.motorType = MotorType::GO_M8010_6;
        cmd.id    = 2;
        cmd.mode  = 1;
        cmd.K_P   = 1.5;//位置刚度P
        cmd.K_W   = 0.2;//速度刚度D
        cmd.Pos   = (-6.33*PI/2 ) * p / step + init_Pos_2[2];
        cmd.Pos = (6.33*0- init_Pos_2[2]+init_Pos[2] - 6.33  * 0.921* PI) * p / step + init_Pos_2[2];
        cmd.W     = 0 ;// 期望关节速度(电机转子转速 rad/s)         ±804.00
        cmd.T     = 0.0;// 期望关节的输出力矩(电机转子转矩 N.m) 范围: ±127.99
        serial.sendRecv(&cmd,&data);

        usleep(10000); 
    
    }

    return 0;
}
