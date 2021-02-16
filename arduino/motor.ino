#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

#define STOP -1
#define WAIT 0
#define SCAN 1

ros::NodeHandle nh;

std_msgs::Int16 theta_msg;
ros::Publisher theta_pub("theta_msg", &theta_msg);


SoftwareSerial motor(2, 3);

unsigned char header = 255;
unsigned char id = 1;
unsigned char length = 5;
unsigned char instruction = 3;
unsigned char goal = 30;

int min_r = 512;
int max_r = 827;
int r = 827;

int mode = WAIT;


void modeCb(const std_msgs::Int16MultiArray& mode_msg)
{
    mode = mode_msg.data[0];
    if (mode == SCAN) {
        min_r = mode_msg.data[1];
        max_r = mode_msg.data[2];
        while (r > min_r) {
            motorWrite(r--);
            delay(10);
        }
    }
}

ros::Subscriber<std_msgs::Int16MultiArray> mode_sub("mode_msg", &modeCb);

void movePub(int position){
    motorWrite(position);

    theta_msg.data = position;
    theta_pub.publish(&theta_msg);

    delay(100);
}

void motorWrite(int position){
    unsigned char param1 = position & 0xFF;
    unsigned char param2 = (position & 0xFF00) >> 8;

    unsigned char checksum = (0xFF - (id + length + instruction + goal + param1 + param2)) % 256;

    motor.write(header);
    motor.write(header);
    motor.write(id);
    motor.write(length);
    motor.write(instruction);
    motor.write(goal);
    motor.write(param1);
    motor.write(param2);
    motor.write(checksum);
}

void setup(){
    motor.begin(115200);
    

    nh.initNode();
    nh.advertise(theta_pub);
    nh.subscribe(mode_sub);

    delay(100);
}

void loop(){
    if (mode != STOP){
        if (mode == SCAN){
            while (r < max_r){
                movePub(r++);
                nh.spinOnce();
            }
            movePub(r);
            nh.spinOnce();
            while (mode == SCAN){
                nh.spinOnce();
            }
        } else {
            theta_msg.data = WAIT;
            theta_pub.publish(&theta_msg);
            delay(100);
            nh.spinOnce();
        }
    }
}