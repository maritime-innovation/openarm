#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

namespace {
volatile std::sig_atomic_t g_stop = 0;

void on_signal(int) { g_stop = 1; }

openarm::damiao_motor::MotorType motor_type_for_id(int id) {
    // Map 1..8 as one arm and 9..15 as the second arm subset.
    int local = ((id - 1) % 8) + 1;
    if (local <= 2) return openarm::damiao_motor::MotorType::DM8009;
    if (local <= 4) return openarm::damiao_motor::MotorType::DM4340;
    return openarm::damiao_motor::MotorType::DM4310;
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const std::string can_if = (argc > 1) ? argv[1] : "can0";
    const bool use_fd = true;
    const double amplitude = 0.6;   // rad
    const double frequency = 0.35;  // Hz
    const double kp = 4.0;
    const double kd = 0.4;

    std::cout << "=== OpenArm Full Sweep Demo (IDs 1..15) ===\n";
    std::cout << "CAN interface: " << can_if << "  FD: " << (use_fd ? "on" : "off") << "\n";
    std::cout << "Press Ctrl+C to stop safely.\n";

    for (int id = 1; id <= 15 && !g_stop; ++id) {
        const uint32_t send_id = static_cast<uint32_t>(id);
        const uint32_t recv_id = static_cast<uint32_t>(id + 16);

        std::cout << "\n--- ID " << id << " (recv " << recv_id << ") ---\n";

        try {
            openarm::can::socket::OpenArm arm(can_if, use_fd);
            arm.init_arm_motors({motor_type_for_id(id)}, {send_id}, {recv_id});

            arm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);
            arm.enable_all();
            arm.recv_all(2000);

            arm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

            auto t0 = std::chrono::steady_clock::now();
            constexpr int kSteps = 140;  // ~7 seconds (50ms interval)
            for (int step = 0; step < kSteps && !g_stop; ++step) {
                const auto now = std::chrono::steady_clock::now();
                const double t = std::chrono::duration<double>(now - t0).count();
                const double q_des = amplitude * std::sin(2.0 * M_PI * frequency * t);

                arm.get_arm().mit_control_all({openarm::damiao_motor::MITParam{q_des, kp, kd, 0.0, 0.0}});
                arm.recv_all(500);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            arm.get_arm().mit_control_all({openarm::damiao_motor::MITParam{0.0, kp, kd, 0.0, 0.0}});
            arm.recv_all(500);
            arm.disable_all();
            arm.recv_all(1000);

            std::cout << "Done ID " << id << "\n";
        } catch (const std::exception& e) {
            std::cerr << "Skip ID " << id << ": " << e.what() << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    std::cout << "\nFinished.\n";
    return 0;
}
