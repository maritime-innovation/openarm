#include <chrono>
#include <cmath>
#include <csignal>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>

namespace {
std::atomic<bool> g_stop{false};

void on_signal(int) { g_stop = true; }

openarm::damiao_motor::MotorType motor_type_for_id(int id) {
    // Repeat OpenArm v1 layout every 8 IDs: [1,2]=DM8009, [3,4]=DM4340, others=DM4310.
    const int local = ((id - 1) % 8) + 1;
    if (local <= 2) return openarm::damiao_motor::MotorType::DM8009;
    if (local <= 4) return openarm::damiao_motor::MotorType::DM4340;
    return openarm::damiao_motor::MotorType::DM4310;
}

double per_joint_amp(int local_joint_1to8) {
    // Keep proximal joints moderate to avoid self-collision with body.
    switch (local_joint_1to8) {
        case 1: return 0.45;
        case 2: return 0.30;
        case 3: return 0.40;
        case 4: return 0.35;
        case 5: return 0.30;
        case 6: return 0.25;
        case 7: return 0.25;
        case 8: return 0.02;  // gripper
        default: return 0.20;
    }
}

double outward_bias_can0(int local_joint_1to8) {
    // Outward/open posture bias for right side.
    switch (local_joint_1to8) {
        case 2: return 0.90;
        case 4: return 0.65;
        case 8: return 0.01;
        default: return 0.0;
    }
}

double outward_bias_can1(int local_joint_1to8) {
    // Mirror shoulder spread for left side.
    switch (local_joint_1to8) {
        case 2: return -0.90;
        case 4: return 0.65;
        case 8: return 0.01;
        default: return 0.0;
    }
}

std::vector<openarm::damiao_motor::MotorType> build_motor_types_for_ids(
    const std::vector<uint32_t>& ids) {
    std::vector<openarm::damiao_motor::MotorType> out;
    out.reserve(ids.size());
    for (const auto id : ids) {
        out.push_back(motor_type_for_id(static_cast<int>(id)));
    }
    return out;
}

void safe_disable(openarm::can::socket::OpenArm& arm) {
    try {
        arm.disable_all();
        arm.recv_all(1500);
    } catch (...) {
    }
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const double amp = (argc > 1) ? std::atof(argv[1]) : 1.2;  // amplitude scale
    const double hz = (argc > 2) ? std::atof(argv[2]) : 0.35;  // Hz
    const int seconds = (argc > 3) ? std::atoi(argv[3]) : 30;  // seconds

    std::cout << "=== OpenArm Dual Full Demo ===\n";
    std::cout << "can0 fixed IDs: 2,4,6,8\n";
    std::cout << "can1 fixed IDs: 1,3,5,7\n";
    std::cout << "amp=" << amp << " rad, freq=" << hz << " Hz, duration=" << seconds << " s\n";
    std::cout << "Ctrl+C to stop safely\n";

    const std::vector<uint32_t> send_ids_can0{2, 4, 6, 8};
    const std::vector<uint32_t> recv_ids_can0{18, 20, 22, 24};
    const auto motor_types_can0 = build_motor_types_for_ids(send_ids_can0);

    const std::vector<uint32_t> send_ids_can1{1, 3, 5, 7};
    const std::vector<uint32_t> recv_ids_can1{17, 19, 21, 23};
    const auto motor_types_can1 = build_motor_types_for_ids(send_ids_can1);

    try {
        // Initialize left side first to test IF-order sensitivity.
        openarm::can::socket::OpenArm arm1("can1", true);
        openarm::can::socket::OpenArm arm0("can0", true);

        arm0.init_arm_motors(motor_types_can0, send_ids_can0, recv_ids_can0);
        arm1.init_arm_motors(motor_types_can1, send_ids_can1, recv_ids_can1);
        arm0.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);
        arm1.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);

        arm0.enable_all();
        arm1.enable_all();
        arm0.recv_all(2000);
        arm1.recv_all(2000);

        arm0.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        arm1.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        const auto start = std::chrono::steady_clock::now();
        const auto end_time = start + std::chrono::seconds(seconds);

        auto next_print = start;
        while (!g_stop && std::chrono::steady_clock::now() < end_time) {
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

            std::vector<openarm::damiao_motor::MITParam> cmd0;
            std::vector<openarm::damiao_motor::MITParam> cmd1;
            cmd0.reserve(send_ids_can0.size());
            cmd1.reserve(send_ids_can1.size());

            for (size_t i = 0; i < send_ids_can0.size(); ++i) {
                const int id = static_cast<int>(send_ids_can0[i]);
                const int local = ((id - 1) % 8) + 1;
                const double phase = (i % 2 == 0) ? 0.0 : M_PI;  // neighboring joints move opposite
                const double swing = amp * per_joint_amp(local) * std::sin(2.0 * M_PI * hz * t + phase);
                const double q0 = outward_bias_can0(local) + swing;
                cmd0.push_back(openarm::damiao_motor::MITParam{6.0, 1.2, q0, 0.0, 0.0});
            }

            for (size_t i = 0; i < send_ids_can1.size(); ++i) {
                const int id = static_cast<int>(send_ids_can1[i]);
                const int local = ((id - 1) % 8) + 1;
                const double phase = (i % 2 == 0) ? 0.0 : M_PI;
                const double swing = amp * per_joint_amp(local) * std::sin(2.0 * M_PI * hz * t + phase);
                const double q1 = outward_bias_can1(local) + swing;
                cmd1.push_back(openarm::damiao_motor::MITParam{6.0, 1.2, q1, 0.0, 0.0});
            }

            arm0.get_arm().mit_control_all(cmd0);
            arm1.get_arm().mit_control_all(cmd1);

            arm0.recv_all(600);
            arm1.recv_all(600);

            const auto now = std::chrono::steady_clock::now();
            if (now >= next_print) {
                const auto motors0 = arm0.get_arm().get_motors();
                const auto motors1 = arm1.get_arm().get_motors();
                if (!motors0.empty() && !motors1.empty()) {
                    std::cout << "t=" << t
                              << " can0:id2 q=" << motors0.front().get_position()
                              << " can1:id1 q=" << motors1.front().get_position() << "\n";
                } else {
                    std::cout << "t=" << t << " no arm motor state available yet\n";
                }
                next_print = now + std::chrono::seconds(1);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        // Return to near zero before disabling.
        std::vector<openarm::damiao_motor::MITParam> zero_cmd(
            send_ids_can0.size(), openarm::damiao_motor::MITParam{6.0, 1.2, 0.0, 0.0, 0.0});
        std::vector<openarm::damiao_motor::MITParam> zero_cmd1(
            send_ids_can1.size(), openarm::damiao_motor::MITParam{6.0, 1.2, 0.0, 0.0, 0.0});
        arm0.get_arm().mit_control_all(zero_cmd);
        arm1.get_arm().mit_control_all(zero_cmd1);
        arm0.recv_all(700);
        arm1.recv_all(700);

        safe_disable(arm0);
        safe_disable(arm1);
        std::cout << "Done.\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
