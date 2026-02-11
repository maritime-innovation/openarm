#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }
}

int main(int argc, char** argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const double amp = (argc > 1) ? std::atof(argv[1]) : 0.7;
    const double hz = (argc > 2) ? std::atof(argv[2]) : 0.30;
    const int seconds = (argc > 3) ? std::atoi(argv[3]) : 20;

    std::cout << "=== OpenArm can1 ID7,8 Demo ===\n";
    std::cout << "amp=" << amp << " rad, freq=" << hz << " Hz, duration=" << seconds << " s\n";

    try {
        openarm::can::socket::OpenArm arm("can1", true);

        std::vector<openarm::damiao_motor::MotorType> motor_types = {
            openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310,
        };
        std::vector<uint32_t> send_ids = {0x07, 0x08};
        std::vector<uint32_t> recv_ids = {0x17, 0x18};

        arm.init_arm_motors(motor_types, send_ids, recv_ids);
        arm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);
        arm.enable_all();
        arm.recv_all(2000);

        arm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        const auto t0 = std::chrono::steady_clock::now();
        const auto tend = t0 + std::chrono::seconds(seconds);
        auto next_print = t0;

        while (!g_stop && std::chrono::steady_clock::now() < tend) {
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            const double q7 = amp * std::sin(2.0 * M_PI * hz * t);
            const double q8 = 0.5 * amp * std::sin(2.0 * M_PI * hz * t + M_PI);

            arm.get_arm().mit_control_all({
                openarm::damiao_motor::MITParam{6.0, 1.2, q7, 0.0, 0.0},
                openarm::damiao_motor::MITParam{6.0, 1.2, q8, 0.0, 0.0},
            });
            arm.recv_all(600);

            if (std::chrono::steady_clock::now() >= next_print) {
                const auto motors = arm.get_arm().get_motors();
                if (motors.size() == 2) {
                    std::cout << "t=" << t
                              << " id7=" << motors[0].get_position()
                              << " id8=" << motors[1].get_position() << "\n";
                }
                next_print = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        arm.get_arm().mit_control_all({
            openarm::damiao_motor::MITParam{6.0, 1.2, 0.0, 0.0, 0.0},
            openarm::damiao_motor::MITParam{6.0, 1.2, 0.0, 0.0, 0.0},
        });
        arm.recv_all(700);

        arm.disable_all();
        arm.recv_all(1200);
        std::cout << "Done.\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
