#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }

openarm::damiao_motor::MotorType motor_type_for_id(int id) {
    if (id <= 2) return openarm::damiao_motor::MotorType::DM8009;
    if (id <= 4) return openarm::damiao_motor::MotorType::DM4340;
    return openarm::damiao_motor::MotorType::DM4310;
}

std::vector<openarm::damiao_motor::MotorType> build_types_even_arm() {
    std::vector<openarm::damiao_motor::MotorType> v;
    for (int id : {2, 4, 6}) v.push_back(motor_type_for_id(id));
    return v;
}

std::vector<uint32_t> build_send_even_arm() { return {0x02, 0x04, 0x06}; }
std::vector<uint32_t> build_recv_even_arm() { return {0x12, 0x14, 0x16}; }

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

    const double arm_amp = (argc > 1) ? std::atof(argv[1]) : 1.0;
    const double arm_hz = (argc > 2) ? std::atof(argv[2]) : 0.30;
    const int seconds = (argc > 3) ? std::atoi(argv[3]) : 30;

    std::cout << "=== Combo Demo: Even IDs on Both Arms ===\n";
    std::cout << "right arm: can0 IDs2/4/6 + gripper8, left arm: can1 IDs2/4/6 + gripper8\n";
    std::cout << "amp=" << arm_amp << " rad, freq=" << arm_hz << " Hz, duration=" << seconds << " s\n";

    try {
        // Right arm (even IDs only: 2,4,6)
        openarm::can::socket::OpenArm right("can0", true);
        right.init_arm_motors(build_types_even_arm(), build_send_even_arm(), build_recv_even_arm());
        right.init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18,
                                 openarm::damiao_motor::ControlMode::POS_FORCE);
        right.get_gripper().set_limit(6.0, 0.6);

        // Left arm (even IDs only: 2,4,6)
        openarm::can::socket::OpenArm left("can1", true);
        left.init_arm_motors(build_types_even_arm(), build_send_even_arm(), build_recv_even_arm());
        left.init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18,
                                openarm::damiao_motor::ControlMode::POS_FORCE);
        left.get_gripper().set_limit(6.0, 0.6);

        right.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);
        left.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);

        right.enable_all();
        left.enable_all();
        right.recv_all(2000);
        left.recv_all(2000);

        right.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        left.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        const auto start = std::chrono::steady_clock::now();
        const auto end = start + std::chrono::seconds(seconds);
        auto next_print = start;

        while (!g_stop && std::chrono::steady_clock::now() < end) {
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

            std::vector<openarm::damiao_motor::MITParam> arm_cmd;
            arm_cmd.reserve(3);
            arm_cmd.push_back(
                openarm::damiao_motor::MITParam{6.0, 1.2,
                                                0.75 + arm_amp * 0.35 * std::sin(2.0 * M_PI * arm_hz * t),
                                                0.0, 0.0});  // ID2
            arm_cmd.push_back(
                openarm::damiao_motor::MITParam{6.0, 1.2,
                                                0.55 + arm_amp * 0.35 * std::sin(2.0 * M_PI * arm_hz * t + M_PI),
                                                0.0, 0.0});  // ID4
            arm_cmd.push_back(
                openarm::damiao_motor::MITParam{6.0, 1.2,
                                                arm_amp * 0.25 * std::sin(2.0 * M_PI * arm_hz * t + M_PI / 2.0),
                                                0.0, 0.0});  // ID6
            right.get_arm().mit_control_all(arm_cmd);

            // Left arm mirrors right arm.
            left.get_arm().mit_control_all(arm_cmd);

            // Gripper ID8 open/close on both sides.
            const double g = 0.5 * (std::sin(2.0 * M_PI * 0.2 * t) + 1.0);
            right.get_gripper().set_position(g);
            left.get_gripper().set_position(g);

            right.recv_all(600);
            left.recv_all(600);

            const auto now = std::chrono::steady_clock::now();
            if (now >= next_print) {
                auto motors = right.get_arm().get_motors();
                std::cout << "t=" << t;
                if (!motors.empty()) std::cout << " right_id2=" << motors.front().get_position();
                auto lm = left.get_arm().get_motors();
                if (!lm.empty()) std::cout << " left_id2=" << lm.front().get_position();
                auto rg = right.get_gripper().get_motors();
                auto lg = left.get_gripper().get_motors();
                if (!rg.empty()) std::cout << " right_id8=" << rg.front().get_position();
                if (!lg.empty()) std::cout << " left_id8=" << lg.front().get_position();
                std::cout << "\n";
                next_print = now + std::chrono::seconds(1);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        std::vector<openarm::damiao_motor::MITParam> zero(
            3, openarm::damiao_motor::MITParam{6.0, 1.2, 0.0, 0.0, 0.0});
        right.get_arm().mit_control_all(zero);
        left.get_arm().mit_control_all(zero);
        right.get_gripper().set_position(0.0);
        left.get_gripper().set_position(0.0);
        right.recv_all(700);
        left.recv_all(700);

        safe_disable(right);
        safe_disable(left);
        std::cout << "Done.\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
