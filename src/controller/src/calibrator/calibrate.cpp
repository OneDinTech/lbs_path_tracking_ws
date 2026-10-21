// =====================================================================
// calibrate.cpp : semi automatic calibration for the 3 wheel swerve.
// Sensors used: HFI_A9 gyro, the two passive dead wheels, and (new)
// the REAL steering angle feedback that every module broadcasts on
// CAN id = command id + 100, confirmed by candump:
//   drive feedback    0x65 0x67 0x69  (motors 1, 3, 5)
//   steering feedback 0x66 0x68 0x6A  (motors 2, 4, 6)
//   frame layout: position bytes 0..3, speed bytes 4..7, float32 LE
//
// CONTROLS (keyboard ONLY, typed in this terminal):
//   Enter = start / finish       q = abort (STOPS THE ROBOT)
//   s = skip                     b = back one test
// No PS4 controller is used at all. Between runs every motor is
// STOPPED, so you reposition the robot by pushing it by hand.
// SAFETY: for the two runs where the robot translates (tests 4 and
// 5, both slow and about a meter long) stand next to it with your
// hand on the keyboard; q + Enter stops every motor immediately.
//
// Run these (calibrate in its OWN terminal so the keyboard works; do
// not start it through ros2 launch):
//   terminal 1:  ros2 run hfi_a9 hfi_a9_node
//   terminal 2:  ros2 run controller calibrate
//
// FLOW OF EVERY TEST: the terminal prints simple numbered steps for
// the NEXT test; you press Enter; a 3 second countdown runs; the test
// runs; motors stop; the result prints; the next test is announced.
//
// NOTHING NEEDS RESETTING BETWEEN RUNS: dead wheels are used as
// VELOCITY sensors, the gyro is a rate sensor, steering feedback is
// an absolute angle in the motor's own frame (the same frame your
// commands use), and every test zeroes its own accumulators when
// recording starts. A stationary robot accumulates exactly nothing.
//
// TEST LIST:
//   0  gyro bias        : robot untouched, node averages the gyro
//   1  dead wheel X cal : push the robot exactly PUSH_DISTANCE (+X)
//   2  dead wheel Y cal : same along body +Y
//   3  spin             : rotate in place exactly SPIN_TURNS turns;
//                         gyro scale + pod coupling K_X / K_Y
//   4  drive sign + r   : robot drives itself slowly forward then
//                         backward (~0.75 m each way) and compares
//                         dead wheel speed to the command (sign + r_eff)
//   5  step response    : clean velocity step, rise time ->
//                         acceleration limit
//   6  steering speed   : wheels twist in place 0/90/0 while
//                         the node times each wheel using the REAL
//                         measured angles -> per wheel settle time and
//                         STEER_RATE_EST. The robot itself stays put.
//                         (This replaces the old 8 launch sweep: with
//                         real angle feedback the settle time is
//                         measured directly, no launches needed.)
//
// Results: printed after each test and written to
// calibration_results.txt in the directory the node started from.
// =====================================================================

#include "Swervedrive_Kinematic.hpp"
#include "can.hpp"
#include "driver.hpp"
#include "packer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cmath>
#include <cstdio>
#include <cctype>
#include <fstream>
#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <algorithm>

// ==== hardware constants (keep identical to path_tracker.cpp) ====
static constexpr uint16_t SPEED_ID[NUM_MODULES_3] = {1, 3, 5};
static constexpr uint16_t STEER_ID[NUM_MODULES_3] = {2, 4, 6};
static constexpr uint16_t FEEDBACK_OFFSET = 100;   // feedback id = cmd id + 100
static constexpr uint16_t DEADWHEEL_X_ID = 162;
static constexpr uint16_t DEADWHEEL_Y_ID = 161;
static constexpr float    DEADWHEEL_R     = 0.028f;
static constexpr float    DEADWHEEL_CAL_X = 1.0696132f;
static constexpr float    DEADWHEEL_CAL_Y = 1.0661124f;
static constexpr float    WHEEL_R = 0.045f;    // current best guess (m)
static constexpr float    WHEEL_L = 0.24647f;  // centre to wheel (m)

static constexpr double   CONTROL_FREQ = 100.0;
static constexpr double   Ts           = 1.0 / CONTROL_FREQ;

// ==== test parameters (edit here if your space differs) ====
static constexpr double COUNTDOWN_S      = 3.0;    // wait after Enter
static constexpr double BIAS_DURATION    = 5.0;    // s of averaging
static constexpr float  PUSH_DISTANCE    = 2.0f;   // m, mark it with tape
static constexpr int    SPIN_TURNS       = 5;      // exact full turns
static constexpr float  DRIVE_TEST_V     = 0.3f;   // m/s for the sign/radius run
static constexpr double DRIVE_SEG        = 2.5;    // s per direction (~0.75 m)
static constexpr double DRIVE_PAUSE      = 0.8;    // s stop between directions
static constexpr float  STEP_TEST_V      = 0.5f;   // m/s, needs ~3 m clear
static constexpr double STEP_DURATION    = 2.5;    // s at STEP_TEST_V
static constexpr double STEER_HOLD       = 1.5;    // s pre-steer before step
// test 6 (steering speed, measured with the real angle feedback):
static constexpr float  STEER_SETTLE_TOL = 0.035f; // rad (~2 deg) = "arrived"
static constexpr int    STEER_MOVES      = 6;      // 0->90->0->... measured moves
static constexpr double STEER_TIMEOUT    = 3.0;    // s per move before giving up

class CalibrateNode : public rclcpp::Node
{
public:
    CalibrateNode()
    : Node("calibrate"),
      can_("can0", 1'000'000, 1000),
      driver_(can_),
      rx_running_(true)
    {
        if (!can_.enable())
            RCLCPP_ERROR(get_logger(), "Failed to open CAN, motors will not move");

        swerve_init3(&sr_, WHEEL_L, WHEEL_R);   // kept for geometry reference
        for (int i = 0; i < NUM_MODULES_3; i++) {
            phi_meas_[i] = 0.0f;
            settle_t_[i] = -1.0;
            settle_worst_[i] = 0.0;
        }

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", rclcpp::QoS(10),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                omega_ = float(msg->angular_velocity.z);
            });

        rx_thread_    = std::thread(&CalibrateNode::rx_loop, this);
        stdin_thread_ = std::thread(&CalibrateNode::stdin_loop, this);
        stdin_thread_.detach();   // std::getline cannot be interrupted;
                                  // the thread dies with the process

        timer_ = create_wall_timer(
            std::chrono::duration<double>(Ts),
            std::bind(&CalibrateNode::tick, this));

        RCLCPP_INFO(get_logger(),
            "\n============================================================"
            "\n CALIBRATION NODE (keyboard only, no PS4)"
            "\n Enter = start / finish     q = abort (STOPS THE ROBOT)"
            "\n s = skip                   b = back one test"
            "\n Motors are STOPPED between runs; push the robot by hand"
            "\n to reposition it."
            "\n============================================================");
        stop_all();
        enter_menu();
    }

    ~CalibrateNode() override
    {
        rx_running_.store(false);
        if (rx_thread_.joinable())
            rx_thread_.join();
        stop_all();
    }

private:
    // ================= keyboard input thread =================
    void stdin_loop()
    {
        std::string line;
        while (std::getline(std::cin, line)) {
            if (line.empty()) { p_enter_.store(true); continue; }
            switch (std::tolower(static_cast<unsigned char>(line[0]))) {
                case 'q': p_abort_.store(true); break;
                case 's': p_skip_.store(true);  break;
                case 'b': p_back_.store(true);  break;
                default:  p_enter_.store(true); break;
            }
        }
    }
    bool take(std::atomic<bool>& f) { return f.exchange(false); }

    // wrap an angle difference into (-pi, pi]; needed because the
    // commanded phi is a continuous multi turn angle while the
    // feedback may sit in a different turn of the circle
    static float wrapd(float a)
    {
        while (a >  float(M_PI)) a -= 2.0f * float(M_PI);
        while (a < -float(M_PI)) a += 2.0f * float(M_PI);
        return a;
    }

    // ============ CAN RX: dead wheels + module feedback dispatcher ============
    void rx_loop()
    {
        while (rx_running_.load()) {
            uint16_t id {}; uint8_t data[8] {}; uint8_t len {};
            if (can_.receive_frame(id, data, len, 50) != CANError::OK)
                continue;
            if (len < 8)
                continue;

            // frame layout everywhere: position@0..3, speed@4..7, f32 LE
            if (id == DEADWHEEL_X_ID) {
                float spd = Packer::unpack_f32(data + 4);
                std::lock_guard<std::mutex> lock(fb_mutex_);
                dw_vx_raw_ = spd * DEADWHEEL_R * DEADWHEEL_CAL_X;
                continue;
            }
            if (id == DEADWHEEL_Y_ID) {
                float spd = Packer::unpack_f32(data + 4);
                std::lock_guard<std::mutex> lock(fb_mutex_);
                dw_vy_raw_ = spd * DEADWHEEL_R * DEADWHEEL_CAL_Y;
                continue;
            }
            for (int i = 0; i < NUM_MODULES_3; i++) {
                if (id == uint16_t(STEER_ID[i] + FEEDBACK_OFFSET)) {
                    // REAL steering angle of wheel i (motor frame, rad)
                    float pos = Packer::unpack_f32(data + 0);
                    std::lock_guard<std::mutex> lock(fb_mutex_);
                    phi_meas_[i] = pos;
                    steer_fb_count_++;
                    break;
                }
                if (id == uint16_t(SPEED_ID[i] + FEEDBACK_OFFSET)) {
                    // real wheel speed (rad/s); stored for future use
                    float spd = Packer::unpack_f32(data + 4);
                    std::lock_guard<std::mutex> lock(fb_mutex_);
                    wheel_speed_meas_[i] = spd;
                    break;
                }
            }
        }
    }

    // ================= motor helpers =================
    float effective_r() const { return (r_eff_ > 0.0f) ? r_eff_ : WHEEL_R; }

    void steer_all(float angle)
    {
        for (int i = 0; i < NUM_MODULES_3; i++)
            driver_.send_position(STEER_ID[i], angle);
    }
    // drive_sign_ starts at +1 and is corrected by test 4, so every
    // later powered command automatically drives the right way
    void drive_all(float wheel_speed)
    {
        for (int i = 0; i < NUM_MODULES_3; i++)
            driver_.send_speed(SPEED_ID[i], drive_sign_ * wheel_speed);
    }
    // full stop: motors release, robot can be pushed by hand freely
    void stop_all()
    {
        for (int i = 0; i < NUM_MODULES_3; i++) {
            driver_.send_stop(SPEED_ID[i]);
            driver_.send_stop(STEER_ID[i]);
        }
    }

    // raw drive used only by test 4, which measures the sign itself
    void drive_raw(float wheel_speed)
    {
        for (int i = 0; i < NUM_MODULES_3; i++)
            driver_.send_speed(SPEED_ID[i], wheel_speed);
    }

    // ================= sensor snapshot with corrections so far =================
    // Applies every correction already measured (cal factors, K slopes,
    // gyro bias and scale) so each test benefits from the earlier ones.
    void read_sensors(float& vx, float& vy, float& w)
    {
        float rvx, rvy;
        {
            std::lock_guard<std::mutex> lock(fb_mutex_);
            rvx = dw_vx_raw_;
            rvy = dw_vy_raw_;
        }
        w  = (omega_ - bias_) * gyro_scale_;
        vx = rvx * fx_ - k_x_ * w;
        vy = rvy * fy_ - k_y_ * w;
    }

    void read_steer_meas(float pm[NUM_MODULES_3], long& count)
    {
        std::lock_guard<std::mutex> lock(fb_mutex_);
        for (int i = 0; i < NUM_MODULES_3; i++) pm[i] = phi_meas_[i];
        count = steer_fb_count_;
    }

    // ================= state machine =================
    enum class Phase { MENU, COUNTDOWN, BIAS, PUSH, SPIN, DRIVE, STEP, STEER, DONE };

    void enter_menu()
    {
        phase_ = Phase::MENU;
        t_ = 0.0; stage_ = 0;
        stop_all();

        if (test_idx_ > 6) { finish(); return; }

        const char* dm = "To STOP the robot anytime: type q then Enter.";

        RCLCPP_INFO(get_logger(), "\n==========================================");
        switch (test_idx_) {
        case 0:
            RCLCPP_INFO(get_logger(),
                ">>> NEXT: TEST 0 of 6, GYRO"
                "\n  Robot does: nothing. You do: nothing."
                "\n  Just do not touch it for %.0f seconds."
                "\n  Press ENTER to start. (s = skip)", BIAS_DURATION);
            break;
        case 1: case 2:
            RCLCPP_INFO(get_logger(),
                ">>> NEXT: TEST %d of 6, DEAD WHEEL %s"
                "\n  1. Put robot at the START mark, body +%s roughly along"
                "\n     the tape (perfect alignment is NOT needed)."
                "\n  2. Press ENTER."
                "\n  3. Push it %.3f m to the END mark."
                "\n  4. Come back and press ENTER again.",
                test_idx_, test_idx_ == 1 ? "X" : "Y",
                test_idx_ == 1 ? "X" : "Y", double(PUSH_DISTANCE));
            break;
        case 3:
            RCLCPP_INFO(get_logger(),
                ">>> NEXT: TEST 3 of 6, SPIN"
                "\n  1. Line up the tape arrows (robot and floor)."
                "\n  2. Press ENTER."
                "\n  3. Turn the robot in place, slowly, exactly %d full turns."
                "\n  4. Press ENTER again when the arrows meet.", SPIN_TURNS);
            break;
        case 4:
            RCLCPP_INFO(get_logger(),
                ">>> NEXT: TEST 4 of 6, DRIVE SIGN + WHEEL SIZE (ROBOT DRIVES ITSELF)"
                "\n  It will drive slowly (%.1f m/s) about %.1f m one way, pause,"
                "\n  then drive back to where it started."
                "\n  It MAY go the wrong way first, that is the measurement."
                "\n  1. Clear about 1.5 m in FRONT and BEHIND the robot."
                "\n  2. Stand next to it. %s"
                "\n  3. Press ENTER.",
                double(DRIVE_TEST_V), DRIVE_TEST_V * DRIVE_SEG, dm);
            break;
        case 5:
            RCLCPP_INFO(get_logger(),
                ">>> NEXT: TEST 5 of 6, SPEED STEP (ROBOT DRIVES ITSELF)"
                "\n  It will drive straight about %.1f m at %.1f m/s, then stop."
                "\n  1. Clear about 3 m in front of the robot."
                "\n  2. Stand next to it. %s"
                "\n  3. Press ENTER.",
                STEP_TEST_V * STEP_DURATION, double(STEP_TEST_V), dm);
            break;
        case 6:
            RCLCPP_INFO(get_logger(),
                ">>> NEXT: TEST 6 of 6, STEERING SPEED (wheels twist in place)"
                "\n  The robot stays where it is; only the wheels turn"
                "\n  0 / 90 / 0 degrees, %d moves, while the node times each"
                "\n  wheel using its REAL measured angle."
                "\n  1. Keep hands and cables away from the wheels."
                "\n  2. %s"
                "\n  3. Press ENTER.",
                STEER_MOVES, dm);
            break;
        }
    }

    void enter_countdown()
    {
        phase_ = Phase::COUNTDOWN;
        t_ = 0.0;
        count_next_ = 0;
        RCLCPP_INFO(get_logger(), "Starting in...");
    }

    void start_test()
    {
        t_ = 0.0; stage_ = 0;
        switch (test_idx_) {
        case 0:
            phase_ = Phase::BIAS;
            sum_ = 0.0; n_ = 0;
            RCLCPP_INFO(get_logger(), "Measuring... DO NOT TOUCH the robot.");
            break;
        case 1: case 2:
            phase_ = Phase::PUSH;
            push_axis_ = test_idx_ - 1;
            dx_ = 0.0; dy_ = 0.0;
            push_theta_ = 0.0;
            next_print_ = 0.5;
            RCLCPP_INFO(get_logger(),
                "RECORDING. Push now. ENTER here when the robot sits at the END mark.");
            break;
        case 3:
            phase_ = Phase::SPIN;
            sww_ = swx_ = swy_ = 0.0;
            theta_int_ = 0.0;
            next_print_ = 0.5;
            RCLCPP_INFO(get_logger(),
                "RECORDING. Spin now, %d turns. ENTER here when the arrows meet.",
                SPIN_TURNS);
            break;
        case 4:
            phase_ = Phase::DRIVE;
            sum_ = 0.0; n_ = 0;
            mean_fwd_ = 0.0; mean_bwd_ = 0.0;
            drive_sign_ = +1.0f;   // commands go out RAW on purpose
            RCLCPP_INFO(get_logger(), "RUNNING drive test (forward, then back).");
            break;
        case 5:
            phase_ = Phase::STEP;
            step_buf_.clear();
            step_buf_.reserve(size_t(STEP_DURATION / Ts) + 10);
            RCLCPP_INFO(get_logger(), "RUNNING step test.");
            break;
        case 6: {
            phase_ = Phase::STEER;
            steer_target_ = 0.0f;
            moves_done_ = 0;
            for (int i = 0; i < NUM_MODULES_3; i++) {
                settle_t_[i] = -1.0;
                settle_worst_[i] = 0.0;
            }
            float pm[NUM_MODULES_3]; long c;
            read_steer_meas(pm, c);
            fb_count_start_ = c;
            RCLCPP_INFO(get_logger(), "RUNNING steering speed test.");
            break;
        }
        default:
            finish();
            break;
        }
    }

    void tick()
    {
        if (phase_ != Phase::MENU && phase_ != Phase::DONE && take(p_abort_)) {
            stop_all();
            RCLCPP_WARN(get_logger(), "ABORTED, nothing recorded, motors stopped.");
            enter_menu();
            return;
        }
        if (phase_ != Phase::MENU) t_ += Ts;

        float vx, vy, w;
        read_sensors(vx, vy, w);

        switch (phase_) {

        case Phase::MENU:
            if (take(p_enter_)) { enter_countdown(); return; }
            if (take(p_skip_))  { test_idx_++; enter_menu(); return; }
            if (take(p_back_) && test_idx_ > 0) {
                test_idx_--; enter_menu(); return;
            }
            break;

        case Phase::DONE:
            break;

        case Phase::COUNTDOWN:
            if (t_ >= double(count_next_)) {
                int remain = int(COUNTDOWN_S) - count_next_;
                if (remain > 0)
                    RCLCPP_INFO(get_logger(), "  %d...", remain);
                count_next_++;
            }
            if (t_ >= COUNTDOWN_S)
                start_test();
            break;

        case Phase::BIAS:
            sum_ += double(omega_); n_++;          // raw gyro, no corrections
            if (t_ >= BIAS_DURATION) {
                bias_ = float(sum_ / std::max(n_, 1));
                RCLCPP_INFO(get_logger(),
                    "RESULT gyro bias = %+.6f rad/s (uncorrected drift would be"
                    " %+.2f deg/min)", double(bias_),
                    double(bias_) * 180.0 / M_PI * 60.0);
                test_idx_ = 1;
                enter_menu();
            }
            break;

        case Phase::PUSH: {
            // Placement tolerant measurement:
            // 1. integrate in the START orientation frame using the gyro
            //    (bias from test 0), so small rotations WHILE pushing do
            //    not corrupt the distance;
            // 2. calibrate against the straight line length
            //    sqrt(dx^2 + dy^2) between the marks, which is the same
            //    number no matter how tilted the robot sits on the line.
            //    Placing it 100 percent straight is NOT required.
            push_theta_ += double(omega_ - bias_) * Ts;
            double cth = std::cos(push_theta_), sth = std::sin(push_theta_);
            dx_ += (cth * double(vx) - sth * double(vy)) * Ts;
            dy_ += (sth * double(vx) + cth * double(vy)) * Ts;
            double dist = std::sqrt(dx_ * dx_ + dy_ * dy_);

            if (t_ >= next_print_) {
                next_print_ += 0.5;
                RCLCPP_INFO(get_logger(),
                    "  dead wheels:  X %+7.3f m   Y %+7.3f m   distance %6.3f m"
                    "   (target %.3f m)",
                    dx_, dy_, dist, double(PUSH_DISTANCE));
            }
            if (take(p_enter_)) {
                double along = (push_axis_ == 0) ? dx_ : dy_;
                double cross = (push_axis_ == 0) ? dy_ : dx_;
                if (dist < 0.2) {
                    RCLCPP_WARN(get_logger(),
                        "Measured only %.3f m, that cannot be right (pod not"
                        " touching the floor? wrong axis?). Redoing this test.", dist);
                    enter_menu();
                    break;
                }
                if (std::fabs(cross) > std::fabs(along)) {
                    RCLCPP_WARN(get_logger(),
                        "The OTHER axis measured more than the pushed one"
                        " (X %.3f vs Y %.3f). Wrong push direction for this"
                        " test? Redoing.", dx_, dy_);
                    enter_menu();
                    break;
                }
                double corr = double(PUSH_DISTANCE) / dist;
                double tilt = std::atan2(std::fabs(cross), std::fabs(along))
                              * 180.0 / M_PI;
                if (push_axis_ == 0) {
                    fx_ *= float(corr);
                    RCLCPP_INFO(get_logger(),
                        "RESULT distance %.3f m  ->  DEADWHEEL_CAL_X = %.7f"
                        "\n  robot sat about %.2f deg tilted on the line;"
                        " harmless, already accounted for.",
                        dist, double(DEADWHEEL_CAL_X) * double(fx_), tilt);
                } else {
                    fy_ *= float(corr);
                    RCLCPP_INFO(get_logger(),
                        "RESULT distance %.3f m  ->  DEADWHEEL_CAL_Y = %.7f"
                        "\n  robot sat about %.2f deg tilted on the line;"
                        " harmless, already accounted for.",
                        dist, double(DEADWHEEL_CAL_Y) * double(fy_), tilt);
                }
                test_idx_++;
                enter_menu();
            }
            break;
        }

        case Phase::SPIN: {
            // bias corrected but UNSCALED omega on purpose: this very
            // test is the one that measures the scale
            float wb = omega_ - bias_;
            theta_int_ += double(wb) * Ts;
            if (std::fabs(wb) > 0.2f) {
                sww_ += double(wb) * double(wb);
                swx_ += double(wb) * double(vx);
                swy_ += double(wb) * double(vy);
            }
            // live readout: turn counter (so you do not have to count in
            // your head) plus what the dead wheels see while spinning
            // (nonzero values here ARE the pod coupling being measured)
            if (t_ >= next_print_) {
                next_print_ += 0.5;
                RCLCPP_INFO(get_logger(),
                    "  spin: %5.2f / %d turns   dead wheels see X %+6.3f  Y %+6.3f m/s",
                    std::fabs(theta_int_) / (2.0 * M_PI), SPIN_TURNS,
                    double(vx), double(vy));
            }
            if (take(p_enter_)) {
                double truth = double(SPIN_TURNS) * 2.0 * M_PI;
                double meas  = std::fabs(theta_int_);
                if (meas < 0.5 * truth) {
                    RCLCPP_WARN(get_logger(),
                        "Only %.1f rad integrated, expected about %.1f"
                        " (miscounted turns?). Redoing this test.", meas, truth);
                    enter_menu();
                    break;
                }
                gyro_scale_ = float(truth / meas);
                if (sww_ > 1.0) {
                    k_x_ = float(swx_ / sww_);
                    k_y_ = float(swy_ / sww_);
                }
                RCLCPP_INFO(get_logger(),
                    "RESULT gyro scale = %.5f (%.2f%% error)"
                    "\n  rotation coupling  K_X = %+.5f m   K_Y = %+.5f m"
                    "\n  (fake translation per rad/s of spin; compensated"
                    " automatically for the remaining tests)",
                    double(gyro_scale_), (double(gyro_scale_) - 1.0) * 100.0,
                    double(k_x_), double(k_y_));
                test_idx_ = 4;
                enter_menu();
            }
            break;
        }

        case Phase::DRIVE: {
            // scripted: steer forward, drive +RAW for DRIVE_SEG, pause,
            // drive the opposite raw direction for DRIVE_SEG. The sign
            // of the first segment's measured speed is the drive sign;
            // both segments together give the wheel radius. Only counts
            // samples while every wheel's MEASURED angle sits on 0.
            float pm[NUM_MODULES_3]; long c;
            read_steer_meas(pm, c);
            bool steer_aligned = true;
            if (c > 0)
                for (int i = 0; i < NUM_MODULES_3; i++)
                    if (std::fabs(wrapd(pm[i])) > 0.15f)
                        steer_aligned = false;

            if (stage_ == 0) {
                steer_all(0.0f);
                if (t_ >= STEER_HOLD) {
                    stage_ = 1; t_ = 0.0; sum_ = 0.0; n_ = 0;
                }
            } else if (stage_ == 1) {
                steer_all(0.0f);
                drive_raw(+DRIVE_TEST_V / WHEEL_R);
                if (t_ > 0.8 && steer_aligned) { sum_ += double(vx); n_++; }
                if (t_ >= DRIVE_SEG) {
                    mean_fwd_ = sum_ / std::max(n_, 1);
                    drive_raw(0.0f);
                    stage_ = 2; t_ = 0.0;
                }
            } else if (stage_ == 2) {
                drive_raw(0.0f);
                if (t_ >= DRIVE_PAUSE) {
                    stage_ = 3; t_ = 0.0; sum_ = 0.0; n_ = 0;
                }
            } else {
                steer_all(0.0f);
                drive_raw(-DRIVE_TEST_V / WHEEL_R);
                if (t_ > 0.8 && steer_aligned) { sum_ += double(vx); n_++; }
                if (t_ >= DRIVE_SEG) {
                    mean_bwd_ = sum_ / std::max(n_, 1);
                    stop_all();
                    if (std::fabs(mean_fwd_) < 0.05) {
                        RCLCPP_WARN(get_logger(),
                            "Robot barely moved (%.3f m/s measured). Check CAN"
                            " and motors. Redoing this test.", mean_fwd_);
                        enter_menu();
                        break;
                    }
                    drive_sign_measured_ = true;
                    drive_sign_ = (mean_fwd_ >= 0.0) ? +1.0f : -1.0f;
                    double ratio = (std::fabs(mean_fwd_) + std::fabs(mean_bwd_))
                                   / (2.0 * double(DRIVE_TEST_V));
                    r_eff_ = WHEEL_R * float(ratio);
                    if (mean_fwd_ * mean_bwd_ > 0.0)
                        RCLCPP_WARN(get_logger(),
                            "Both directions measured the SAME sign, something"
                            " is off; trust this result less and consider redoing.");
                    RCLCPP_INFO(get_logger(),
                        "RESULT forward %+.3f m/s, backward %+.3f m/s for raw"
                        " commands of %.1f m/s"
                        "\n  DRIVE SIGN = %+d  %s"
                        "\n  effective wheel radius r_eff = %.5f m (was %.5f)",
                        mean_fwd_, mean_bwd_, double(DRIVE_TEST_V),
                        int(drive_sign_),
                        drive_sign_ > 0
                          ? "(positive moves forward: tx_step must NOT negate)"
                          : "(INVERTED: tx_step MUST negate wheel_speed, Fix 1)",
                        double(r_eff_), double(WHEEL_R));
                    test_idx_ = 5;
                    enter_menu();
                }
            }
            break;
        }

        case Phase::STEP:
            if (stage_ == 0) {
                steer_all(0.0f);
                if (t_ >= STEER_HOLD) {
                    stage_ = 1; t_ = 0.0;
                    drive_all(STEP_TEST_V / effective_r());
                }
            } else {
                drive_all(STEP_TEST_V / effective_r());
                step_buf_.push_back({t_, vx});
                if (t_ >= STEP_DURATION) {
                    stop_all();
                    analyze_step();
                    test_idx_ = 6;
                    enter_menu();
                }
            }
            break;

        case Phase::STEER: {
            // measure the REAL settle time of every steering motor by
            // commanding 0 / 90 / 0 and watching the measured angles
            steer_all(steer_target_);
            float pm[NUM_MODULES_3]; long c;
            read_steer_meas(pm, c);

            if (stage_ == 0) {
                // move to the start angle 0 and confirm feedback is alive
                bool all = true;
                for (int i = 0; i < NUM_MODULES_3; i++)
                    if (std::fabs(wrapd(pm[i] - steer_target_)) > STEER_SETTLE_TOL)
                        all = false;
                bool fb_alive = (c > fb_count_start_ + 10);
                if (all && fb_alive && t_ > 0.5) {
                    stage_ = 1; t_ = 0.0;
                    steer_target_ = float(M_PI) / 2.0f;
                    for (int i = 0; i < NUM_MODULES_3; i++) settle_t_[i] = -1.0;
                } else if (t_ > STEER_TIMEOUT) {
                    stop_all();
                    RCLCPP_WARN(get_logger(),
                        fb_alive
                          ? "Wheels never reached the start angle, run aborted."
                          : "NO steering feedback frames received (ids 0x66/0x68/0x6A)."
                            " Check candump. Run aborted.");
                    enter_menu();
                }
                break;
            }

            // timing one move: note when each wheel first arrives
            for (int i = 0; i < NUM_MODULES_3; i++)
                if (settle_t_[i] < 0.0
                    && std::fabs(wrapd(pm[i] - steer_target_)) < STEER_SETTLE_TOL)
                    settle_t_[i] = t_;

            bool all_arrived = true;
            for (int i = 0; i < NUM_MODULES_3; i++)
                if (settle_t_[i] < 0.0) all_arrived = false;

            if (all_arrived || t_ > STEER_TIMEOUT) {
                for (int i = 0; i < NUM_MODULES_3; i++) {
                    double v = (settle_t_[i] < 0.0) ? STEER_TIMEOUT : settle_t_[i];
                    settle_worst_[i] = std::max(settle_worst_[i], v);
                }
                moves_done_++;
                RCLCPP_INFO(get_logger(),
                    "  move %d/%d: wheel settle times %.0f / %.0f / %.0f ms",
                    moves_done_, STEER_MOVES,
                    (settle_t_[0] < 0 ? STEER_TIMEOUT : settle_t_[0]) * 1000.0,
                    (settle_t_[1] < 0 ? STEER_TIMEOUT : settle_t_[1]) * 1000.0,
                    (settle_t_[2] < 0 ? STEER_TIMEOUT : settle_t_[2]) * 1000.0);
                if (moves_done_ >= STEER_MOVES) {
                    stop_all();
                    analyze_steer();
                    test_idx_ = 7;
                    enter_menu();
                } else {
                    steer_target_ = (steer_target_ == 0.0f)
                                    ? float(M_PI) / 2.0f : 0.0f;
                    for (int i = 0; i < NUM_MODULES_3; i++) settle_t_[i] = -1.0;
                    t_ = 0.0;
                }
            }
            break;
        }
        }
    }

    // ================= analysis =================
    void analyze_step()
    {
        if (step_buf_.size() < 20) {
            RCLCPP_WARN(get_logger(), "Step test: not enough data, redo it (b).");
            return;
        }
        double steady = 0.0; int ns = 0;
        for (auto& s : step_buf_)
            if (s.first > STEP_DURATION - 0.5) { steady += double(s.second); ns++; }
        steady /= std::max(ns, 1);

        double t10 = -1.0, t90 = -1.0;
        for (auto& s : step_buf_) {
            if (t10 < 0.0 && double(s.second) >= 0.1 * steady) t10 = s.first;
            if (t90 < 0.0 && double(s.second) >= 0.9 * steady) { t90 = s.first; break; }
        }
        if (t10 < 0.0 || t90 <= t10) {
            RCLCPP_WARN(get_logger(), "Step test: no clean rise found, redo it (b).");
            return;
        }
        rise_ = float(t90 - t10);
        float a_demo = float(0.8 * steady / double(rise_));   // 10 to 90 percent
        a_suggest_   = 0.5f * a_demo;
        RCLCPP_INFO(get_logger(),
            "RESULT step to %.2f m/s: steady %.3f m/s, rise time (10 to 90%%)"
            " = %.0f ms\n  drivetrain demonstrated about %.2f m/s^2;"
            " suggested MAXIMUM_LINEAR_ACCELERATION = %.2f m/s^2",
            double(STEP_TEST_V), steady, double(rise_) * 1000.0,
            double(a_demo), double(a_suggest_));
        if (steady < 0.9 * double(STEP_TEST_V))
            RCLCPP_WARN(get_logger(),
                "Steady speed is well below the commanded %.2f m/s: the drive"
                " motors may be SATURATING already. Check headroom before"
                " targeting high speeds.", double(STEP_TEST_V));
    }

    void analyze_steer()
    {
        double worst = 0.0; int slow = 0;
        for (int i = 0; i < NUM_MODULES_3; i++)
            if (settle_worst_[i] > worst) { worst = settle_worst_[i]; slow = i; }
        t_settle_   = std::max(worst, 0.05);
        steer_rate_ = 0.9f * float(M_PI / 2.0 / t_settle_);   // 10 percent margin
        RCLCPP_INFO(get_logger(),
            "RESULT steering (measured with real angles):"
            "\n  worst 90 deg settle per wheel: motor %d = %.0f ms,"
            " motor %d = %.0f ms, motor %d = %.0f ms"
            "\n  SLOWEST wheel: steering motor id %d (this one causes the"
            " launch yaw the most)"
            "\n  STEER_RATE_EST = %.2f rad/s",
            STEER_ID[0], settle_worst_[0] * 1000.0,
            STEER_ID[1], settle_worst_[1] * 1000.0,
            STEER_ID[2], settle_worst_[2] * 1000.0,
            STEER_ID[slow], double(steer_rate_));
        if (worst >= STEER_TIMEOUT)
            RCLCPP_WARN(get_logger(),
                "At least one wheel never reached its angle within %.0f s:"
                " check that motor or increase STEER_TIMEOUT.", STEER_TIMEOUT);
    }

    void finish()
    {
        if (phase_ == Phase::DONE) return;
        phase_ = Phase::DONE;
        stop_all();

        std::ofstream f("calibration_results.txt");
        char line[256];
        auto out = [&](const char* s) {
            f << s << "\n";
            RCLCPP_INFO(get_logger(), "%s", s);
        };

        out("==================== CALIBRATION RESULTS ====================");
        out("Paste these into the code as indicated. Values of 0 / 1 mean");
        out("the corresponding test was skipped.");
        out("");
        out("in path_tracker.cpp:");
        std::snprintf(line, sizeof(line), "  GYRO_BIAS       = %.6ff;", double(bias_));                         out(line);
        std::snprintf(line, sizeof(line), "  GYRO_SCALE      = %.5ff;", double(gyro_scale_));                   out(line);
        std::snprintf(line, sizeof(line), "  DEADWHEEL_CAL_X = %.7ff;", double(DEADWHEEL_CAL_X) * double(fx_)); out(line);
        std::snprintf(line, sizeof(line), "  DEADWHEEL_CAL_Y = %.7ff;", double(DEADWHEEL_CAL_Y) * double(fy_)); out(line);
        std::snprintf(line, sizeof(line), "  DEADWHEEL_K_X   = %.5ff;", double(k_x_));                          out(line);
        std::snprintf(line, sizeof(line), "  DEADWHEEL_K_Y   = %.5ff;", double(k_y_));                          out(line);
        std::snprintf(line, sizeof(line), "  STEER_RATE_EST  = %.2ff;", double(steer_rate_));                   out(line);
        out("");
        out("in swerve_init3(...) (path_tracker.cpp constructor):");
        std::snprintf(line, sizeof(line), "  wheel radius r  = %.5ff;  (second argument)", double(effective_r())); out(line);
        out("");
        out("in the path tracker parameters:");
        std::snprintf(line, sizeof(line), "  MAXIMUM_LINEAR_ACCELERATION = %.2f;", double(a_suggest_));         out(line);
        out("");
        out("steering settle per wheel (worst, ms):");
        std::snprintf(line, sizeof(line), "  motor %d: %.0f   motor %d: %.0f   motor %d: %.0f",
                      STEER_ID[0], settle_worst_[0] * 1000.0,
                      STEER_ID[1], settle_worst_[1] * 1000.0,
                      STEER_ID[2], settle_worst_[2] * 1000.0);                                                  out(line);
        out("");
        out("tx_step drive sign (Fix 1):");
        if (!drive_sign_measured_)
            out("  NOT MEASURED (test 4 skipped)");
        else if (drive_sign_ > 0)
            out("  positive command moved the robot FORWARD: do NOT negate in tx_step");
        else
            out("  positive command moved the robot BACKWARD: NEGATE wheel_speed in tx_step");
        out("=============================================================");
        RCLCPP_INFO(get_logger(),
            "Saved to calibration_results.txt. Motors are stopped;"
            " Ctrl+C when done.");
    }

    // ================= members =================
    CAN    can_;
    Driver driver_;

    std::thread       rx_thread_;
    std::thread       stdin_thread_;
    std::atomic<bool> rx_running_;
    std::mutex        fb_mutex_;
    float             dw_vx_raw_ = 0.0f, dw_vy_raw_ = 0.0f;
    float             phi_meas_[NUM_MODULES_3];          // REAL steering angles
    float             wheel_speed_meas_[NUM_MODULES_3] = {0.0f, 0.0f, 0.0f};
    long              steer_fb_count_ = 0;

    float omega_ = 0.0f;                       // latest raw gyro z (rad/s)
    std::atomic<bool> p_enter_{false}, p_abort_{false}, p_skip_{false}, p_back_{false};

    Swerve_Robot3 sr_;

    Phase  phase_ = Phase::MENU;
    int    test_idx_ = 0;
    int    stage_ = 0;
    double t_ = 0.0;
    int    count_next_ = 0;
    int    push_axis_ = 0;

    // scratch (each test zeroes what it uses at recording start)
    double sum_ = 0.0; int n_ = 0;
    double dx_ = 0.0, dy_ = 0.0;
    double push_theta_ = 0.0;
    double next_print_ = 0.0;
    double sww_ = 0.0, swx_ = 0.0, swy_ = 0.0, theta_int_ = 0.0;
    double mean_fwd_ = 0.0, mean_bwd_ = 0.0;
    std::vector<std::pair<double, float>> step_buf_;
    float  steer_target_ = 0.0f;
    int    moves_done_ = 0;
    long   fb_count_start_ = 0;
    double settle_t_[NUM_MODULES_3];
    double settle_worst_[NUM_MODULES_3];

    // results (neutral defaults = "not measured yet")
    float bias_ = 0.0f;
    float gyro_scale_ = 1.0f;
    float fx_ = 1.0f, fy_ = 1.0f;              // multiplicative cal corrections
    float k_x_ = 0.0f, k_y_ = 0.0f;            // rotation coupling (m)
    float drive_sign_ = +1.0f;
    bool  drive_sign_measured_ = false;
    float r_eff_ = 0.0f;
    float rise_ = 0.0f, a_suggest_ = 0.0f;
    double t_settle_ = 0.0;
    float steer_rate_ = 0.0f;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalibrateNode>());
    rclcpp::shutdown();
    return 0;
}