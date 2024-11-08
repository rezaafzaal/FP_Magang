#include <ros/ros.h>
#include <FP_Magang/PC2BS.h>
#include <FP_Magang/BS2PC.h>
#include <termios.h>
#include <iostream>
#include <cmath>

FP_Magang::PC2BS pc2bs_msg;
ros::Publisher pub;

int getch() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

// Fungsi untuk menghitung kecepatan translasi vx, vy, dan rotasi vth
void calculateVelocity(double motor1, double motor2, double motor3, double& vx, double& vy, double& vth) {
    vx = (2.0 / 3.0) * motor1 - (1.0 / 3.0) * motor2 - (1.0 / 3.0) * motor3;
    vy = (std::sqrt(3) / 3.0) * (motor2 - motor3);
    vth = (1.0 / 3.0) * (motor1 + motor2 + motor3);
}

void robot(double motor1, double motor2, double motor3) {
    pc2bs_msg.motor1 = motor1;
    pc2bs_msg.motor2 = motor2;
    pc2bs_msg.motor3 = motor3;
    pub.publish(pc2bs_msg);
    
    ros::Duration(1).sleep();

    pc2bs_msg.motor1 = pc2bs_msg.motor2 = pc2bs_msg.motor3 = 0;
    pub.publish(pc2bs_msg);
}

void basestationCallback(const FP_Magang::BS2PC& msg) {
    if (msg.status == 1) {
        ROS_INFO("Status 1: Manual control mode. Use WASD keys to move the robot.");
    } else if (msg.status == 2) {
        ROS_INFO("Status 2: Following the ball at (%f, %f)", pc2bs_msg.bola_x, pc2bs_msg.bola_y);
    } else if (msg.status == 3) {
        ROS_INFO("Status 3: Moving to target position (%f, %f)", msg.tujuan_x, msg.tujuan_y);
    } else if (msg.status == 4) {
        ROS_INFO("Status 4: Approaching the ball and circling it twice.");
    }
}

void moveRobot(char command) {
    double vx = 0.0, vy = 0.0, omega = 0.0;

    switch (command) {
        case 'w': vy = 1.0; break;
        case 's': vy = -1.0; break;
        case 'a': vx = -1.0; break;
        case 'd': vx = 1.0; break;
        case 'q': omega = 1.0; break;
        case 'e': omega = -1.0; break;
        // case 'x': pc2bs_msg.bola_pegang = false; ROS_INFO("Dropping the ball"); break;
        // case 'z': pc2bs_msg.bola_pegang = true; ROS_INFO("Picking up the ball"); break;
        default: break;
    }

    // Menggunakan koefisien untuk menghitung kecepatan motor berdasarkan vx, vy, dan omega
    pc2bs_msg.motor1 = vx + omega;
    pc2bs_msg.motor2 = -0.5 * vx + 0.866 * vy + omega;
    pc2bs_msg.motor3 = -0.5 * vx - 0.866 * vy + omega;

    pub.publish(pc2bs_msg);
}

void handleStatus2() {
    if (pc2bs_msg.bola_x != -1 && pc2bs_msg.bola_y != -1) {
        ROS_INFO("Following the ball at (%f, %f)", pc2bs_msg.bola_x, pc2bs_msg.bola_y);
        // Logika untuk mendekati bola
    }
}

void handleStatus3(const FP_Magang::BS2PC& msg) {
    ROS_INFO("Moving to target position (%f, %f)", msg.tujuan_x, msg.tujuan_y);
    // Logika untuk menuju target
}

void handleStatus4() {
    if (pc2bs_msg.bola_x != -1 && pc2bs_msg.bola_y != -1) {
        ROS_INFO("Circling around the ball.");
        // Logika mengelilingi bola 2 kali
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle nh;

    pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs", 10);
    ros::Subscriber sub = nh.subscribe("/bs2pc", 10, basestationCallback);

    FP_Magang::BS2PC msg;

    while (ros::ok()) {
        ros::spinOnce();
        
        if (msg.status == 1) {
            // Status 1: Kontrol manual
            char command = getch();
            moveRobot(command);

        } else if (msg.status == 2) {
            // Status 2: Mengikuti bola
            handleStatus2();

        } else if (msg.status == 3) {
            // Status 3: Bergerak menuju target
            handleStatus3(msg);

        } else if (msg.status == 4) {
            // Status 4: Mendekati bola dan mengelilinginya
            handleStatus4();
        }
    }

    return 0;
}
