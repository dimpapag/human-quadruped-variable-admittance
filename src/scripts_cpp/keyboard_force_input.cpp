#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#define KEY_W 119
#define KEY_S 115
#define KEY_A 97
#define KEY_D 100
#define KEY_R 114
#define KEY_F 102
#define KEY_O 111
#define KEY_P 112
#define KEY_K 107
#define KEY_L 108
#define KEY_N 110
#define KEY_M 109
#define KEY_T 116

// Define maximum force values
float FMAX_X = 20.0;
float FMAX_Y = 20.0;
float FMAX_Z = 20.0;

float current_f_x = 0.0;
float current_f_y = 0.0;
float current_f_z = 0.0;

bool key_held = false;

int getKey() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF) {
        return ch;
    }

    return 0;
}

void sendForce(float f_x, float f_y, float f_z, ros::Publisher& pub) {
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(f_x);
    msg.data.push_back(f_y);
    msg.data.push_back(f_z);
    pub.publish(msg);
}

void updateForce(int key, float& current_f, float FMAX) {
    float increment = FMAX / 20.0;  // Adjust increment based on FMAX

    if (key == 0) {
        if (current_f > 0.0) {
            current_f -= increment;
            if (current_f < 0.0)
                current_f = 0.0;
        } else if (current_f < 0.0) {
            current_f += increment;
            if (current_f > 0.0)
                current_f = 0.0;
        }
    } else if (key == KEY_W || key == KEY_A || key == KEY_R) {
        if (current_f < FMAX) {
            current_f += increment;
            if (current_f > FMAX)
                current_f = FMAX;
        }
    } else if (key == KEY_S || key == KEY_D || key == KEY_F) {
        if (current_f > -FMAX) {
            current_f -= increment;
            if (current_f < -FMAX)
                current_f = -FMAX;
        }
    }
}

void updateFMAX(int key, float& FMAX_X, float& FMAX_Y, float& FMAX_Z) {
    if (key == KEY_O && FMAX_X > 0.0) {
        FMAX_X -= 0.5;
    } else if (key == KEY_P && FMAX_X < 10.0) {
        FMAX_X += 0.5;
    } else if (key == KEY_K && FMAX_Y > 0.0) {
        FMAX_Y -= 0.5;
    } else if (key == KEY_L && FMAX_Y < 10.0) {
        FMAX_Y += 0.5;
    } else if (key == KEY_N && FMAX_Z > 0.0) {
        FMAX_Z -= 0.5;
    } else if (key == KEY_M && FMAX_Z < 10.0) {
        FMAX_Z += 0.5;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_force_input");
    ros::NodeHandle nh;

    ros::Publisher force_pub = nh.advertise<std_msgs::Float32MultiArray>("external_force", 10);

    int key;

    ros::Rate rate(30);  // 30Hz

    while(ros::ok()) {
        key = getKey();

        updateFMAX(key, FMAX_X, FMAX_Y, FMAX_Z);

        switch(key) {
            case KEY_W:
            case KEY_S:
                key_held = true;
                updateForce(key, current_f_x, FMAX_X);
                break;
            case KEY_A:
            case KEY_D:
                key_held = true;
                updateForce(key, current_f_y, FMAX_Y);
                break;
            case KEY_R:
            case KEY_F:
                key_held = true;
                updateForce(key, current_f_z, FMAX_Z);
                break;
            case KEY_T:  // Emergency key to reset all forces to zero
                key_held = false;
                current_f_x = 0.0;
                current_f_y = 0.0;
                current_f_z = 0.0;
                break;
            default:
                key_held = false;
                updateForce(0, current_f_x, FMAX_X);
                updateForce(0, current_f_y, FMAX_Y);
                updateForce(0, current_f_z, FMAX_Z);
                break;
        }

        // Publish forces
        sendForce(current_f_x, current_f_y, current_f_z, force_pub);

        // Display current sending force and FMAX values
        std::cout << "Current sending force: F_x = " << current_f_x << ", F_y = " << current_f_y << ", F_z = " << current_f_z << std::endl;
        std::cout << "FMAX values: FMAX_X = " << FMAX_X << ", FMAX_Y = " << FMAX_Y << ", FMAX_Z = " << FMAX_Z << std::endl;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}