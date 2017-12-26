#include "create/create.h"

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <thread>
#include <chrono>

struct JoystickCommand
{
    float left_wheel_vel_mps;
    float right_wheel_vel_mps;
    float accel_mpss;
    float p_gain;
    float i_gain;
};

struct WheelStatus
{
    float current_vel_mps;
    float accum_error;
};

static create::Create* robot;

static JoystickCommand s_current_command;

// Variables to coordinate termination of threads
static bool s_signal_threads_exit;
static bool s_fail;

static void network_recv_execute(void *args)
{
    (void)args;

    static const float NETWORK_RECV_UPDATE_INTERVAL_S = 0.1f;

    int sock_fd;
    struct sockaddr_in in_addr;
    if ((sock_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0)) < 0)
    {
        s_fail = true;
        return;
    }
    memset((char*)&in_addr, 0, sizeof(in_addr));
    in_addr.sin_family = AF_INET;
    in_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    in_addr.sin_port = htons(9999);
    if (bind(sock_fd, (struct sockaddr *)&in_addr, sizeof(in_addr)) < 0)
    {
        std::cout << "Failed to bind to socket" << std::endl;
        s_fail = true;
    }

    while (!s_signal_threads_exit)
    {
        recvfrom(sock_fd, (uint8_t*)&s_current_command, sizeof(s_current_command), 0, NULL, 0);
        usleep(1000000 * NETWORK_RECV_UPDATE_INTERVAL_S);
    }

    close(sock_fd);
}

static void ramp_vel(float target_vel_mps, float accel_mpscy, float &current_vel_mps)
{
    float vel_delta = target_vel_mps - current_vel_mps;

    if (fabsf(vel_delta) < accel_mpscy)
    {
        current_vel_mps = target_vel_mps;
    }
    else if (vel_delta > 0)
    {
        current_vel_mps += accel_mpscy;
    }
    else
    {
        current_vel_mps -= accel_mpscy;
    }
}

static void pid_update(float target_vel_mps, float measured_vel_mps, float p_gain, float i_gain, float &accum_error, float &output)
{
    static const float MAX_OUTPUT = 1.0f;
    static const float MIN_OUTPUT = -1.0f;

    //s_current_command.p_gain = 1.0f;
    //s_current_command.i_gain = 0.2f;

    if (target_vel_mps != 0)
    {
        float error = target_vel_mps - measured_vel_mps;

        if (abs(output) < MAX_OUTPUT)
        {
            accum_error += error;
        }
        output = p_gain * error;
        output += i_gain * accum_error;

        output = std::min(output, MAX_OUTPUT);
        output = std::max(output, MIN_OUTPUT);
    }
    else
    {
        accum_error = 0;
        output = 0;
    }
}

static void control_loop_execute(void *args)
{
    (void)args;

    static const float CONTROL_LOOP_UPDATE_INTERVAL_S = 0.016f;

    WheelStatus s_left_status;
    WheelStatus s_right_status;

    while(!s_signal_threads_exit)
    {
        ramp_vel(s_current_command.left_wheel_vel_mps,
                 s_current_command.accel_mpss * CONTROL_LOOP_UPDATE_INTERVAL_S,
                 s_left_status.current_vel_mps);
        ramp_vel(s_current_command.right_wheel_vel_mps,
                 s_current_command.accel_mpss * CONTROL_LOOP_UPDATE_INTERVAL_S,
                 s_right_status.current_vel_mps);

        static float last_left_wheel_dist = 0;
        static float last_right_wheel_dist = 0;

        float left_measured_vel_mps = robot->getMeasuredLeftWheelVel();
        float right_measured_vel_mps = robot->getMeasuredRightWheelVel();

        if (s_current_command.p_gain == 0 && s_current_command.i_gain == 0)
        {
            robot->driveWheels(s_left_status.current_vel_mps, s_right_status.current_vel_mps);
        }
        else
        {
            static float left_duty;
            static float right_duty;

            pid_update(s_left_status.current_vel_mps,
                       left_measured_vel_mps,
                       s_current_command.p_gain,
                       s_current_command.i_gain,
                       s_left_status.accum_error,
                       left_duty);
            pid_update(s_right_status.current_vel_mps,
                       right_measured_vel_mps,
                       s_current_command.p_gain,
                       s_current_command.i_gain,
                       s_right_status.accum_error,
                       right_duty);

            robot->driveWheelsPwm(left_duty, right_duty);
        }

        usleep(1000000 * CONTROL_LOOP_UPDATE_INTERVAL_S); //66hz
    }
}

int main(int argc, char** argv)
{
    std::string port;
    if (argc > 1)
    {
        port = std::string(argv[1]);
    }
    else
    {
        port = "/dev/ttyUSB0";
    }
    int baud = 115200;
    create::RobotModel model = create::RobotModel::CREATE_2;

    robot = new create::Create(model);

    s_fail = false;

    // Attempt to connect to Create
    if (robot->connect(port, baud))
    {
        std::cout << "Successfully connected to Create" << std::endl;
    }
    else
    {
        std::cout << "Failed to connect to Create on port " << port.c_str() << std::endl;
        return 1;
    }

    robot->setMode(create::MODE_FULL);

    usleep(1000000);

    s_signal_threads_exit = false;

    std::thread network_recv_thread(network_recv_execute, nullptr);
    std::thread control_loop_thread(control_loop_execute, nullptr);

    std::cout << "Entering control loop" << std::endl;

    // Quit when center "Clean" button pressed, or we get a failure
    while (!robot->isCleanButtonPressed() && !s_fail);

    // Signal to the other threads to end
    s_signal_threads_exit = true;

    std::cout << "Stopping Create." << std::endl;

    // Wait for threads to end
    network_recv_thread.join();
    control_loop_thread.join();

    // Turn the robot off
    robot->setMode(create::MODE_OFF);

    robot->disconnect();
    delete robot;

    return s_fail;
}
