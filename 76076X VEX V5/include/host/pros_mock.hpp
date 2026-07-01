#pragma once

#include <vector>
#include <initializer_list>
#include <chrono>
#include <thread>
#include <atomic>
#include <functional>
#include <string>
#include <iostream>
#include <map>

namespace pros {

using std::string;

inline long long millis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

inline void delay(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// LCD mock
namespace lcd {
    inline void initialize() {}
    inline void set_text(int line, const char *text) { std::cout << "LCD L"<<line<<": "<<text<<"\n"; }
    inline void clear_line(int line) {}
    inline void register_btn1_cb(std::function<void()> cb) { /* ignore */ }
    inline void print(int row, const char *fmt, int a, int b, int c) { std::cout << "LCD:" << row << " " << a << " " << b << " " << c << "\n"; }
    inline int read_buttons() { return 0; }
}

// Controller constants
constexpr int E_CONTROLLER_MASTER = 0;
constexpr int ANALOG_LEFT_X = 1;
constexpr int ANALOG_LEFT_Y = 3;

// Digital controller buttons
constexpr int E_CONTROLLER_DIGITAL_L1 = 6;
constexpr int E_CONTROLLER_DIGITAL_L2 = 7;
constexpr int E_CONTROLLER_DIGITAL_R1 = 5;
constexpr int E_CONTROLLER_DIGITAL_R2 = 8;
constexpr int E_CONTROLLER_DIGITAL_A = 1;
constexpr int E_CONTROLLER_DIGITAL_X = 3;

// Button masks
constexpr int LCD_BTN_LEFT = 1 << 2;
constexpr int LCD_BTN_CENTER = 1 << 1;
constexpr int LCD_BTN_RIGHT = 1 << 0;

// Host simulation helpers will be defined after Motor/Controller classes
static std::map<int, int> __host_controller_analogs; // controller analog values
static std::map<int, bool> __host_controller_digital; // controller button values
// Prototypes needed by classes
inline bool has_motor(int port);
inline void create_motor(int port);
inline void host_set_motor_position(int port, double pos);
inline void host_set_motor_voltage(int port, int v);
inline void host_increment_motor_position(int port, double delta);
inline double host_get_motor_position(int port);
inline void host_set_controller_analog(int channel, int value);
inline int host_get_controller_analog(int channel);
inline void host_set_controller_digital(int button, bool pressed);
inline bool host_get_controller_digital(int button);

class Motor {
    public:
        Motor() : port(-1), position(0), voltage(0) {}
        Motor(int port) : port(port), position(0), voltage(0) {}
        void move(int v) { voltage = v; position += v * 0.1; }
        void move_voltage(int v) { move(v); }
        double get_position() const { return position; }
        void tare_position() { position = 0; }
        void set_position(double p) { position = p; }
        int get_voltage() const { return voltage; }
    private:
        int port;
        double position;
        int voltage;
};

class MotorGroup {
    public:
        MotorGroup(std::initializer_list<std::int8_t> ports) {
            for (auto p : ports) {
                ports_.push_back(static_cast<int>(p));
                if (!has_motor(static_cast<int>(p))) create_motor(static_cast<int>(p));
            }
        }
        MotorGroup(const std::vector<std::int8_t>& ports) {
            for (auto p : ports) {
                ports_.push_back(static_cast<int>(p));
                if (!has_motor(static_cast<int>(p))) create_motor(static_cast<int>(p));
            }
        }
        MotorGroup(Motor &m) { ports_.push_back(0); if (!has_motor(0)) create_motor(0); }
        void move(int v) const {
            for (int p : ports_) {
                host_set_motor_voltage(p, v);
                host_increment_motor_position(p, v * 0.1);
            }
        }
        void move_velocity(int v) const { move(v); }
        void tare_position_all() const {
            for (int p : ports_) host_set_motor_position(p, 0.0);
        }
        std::vector<double> get_position_all() const {
            std::vector<double> out;
            for (int p : ports_) out.push_back(host_get_motor_position(p));
            return out;
        }
    private:
        std::vector<int> ports_;
};

class Imu {
    public:
        Imu(int port) : angle(0.0) {}
        double get_rotation() const { return angle; }
        void reset() { angle = 0; }
        void set_rotation(double a) { angle = a; }
    private:
        double angle;
};

class Controller {
    public:
        Controller(int type) {}
        int get_analog(int channel) { return host_get_controller_analog(channel); }
        void set_analog(int channel, int value) { host_set_controller_analog(channel, value); }
        bool get_digital(int button) const { auto it = __host_controller_digital.find(button); return it != __host_controller_digital.end() && it->second; }
        void set_digital(int button, bool pressed) { __host_controller_digital[button] = pressed; }
};

// Implement host simulation helpers inside namespace pros
static std::map<int, Motor> __host_motors; // map must be declared after Motor is defined
inline bool has_motor(int port) { return __host_motors.find(port) != __host_motors.end(); }
inline void create_motor(int port) { __host_motors.emplace(port, Motor(port)); }
inline void host_set_motor_position(int port, double pos) { if (!has_motor(port)) create_motor(port); __host_motors[port].set_position(pos); }
inline void host_set_motor_voltage(int port, int v) { if (!has_motor(port)) create_motor(port); /* voltage stored via move */ }
inline void host_increment_motor_position(int port, double delta) { if (!has_motor(port)) create_motor(port); double p = __host_motors[port].get_position(); __host_motors[port].set_position(p + delta); }
inline double host_get_motor_position(int port) { if (!has_motor(port)) create_motor(port); return __host_motors[port].get_position(); }

inline void host_set_controller_analog(int channel, int value) { __host_controller_analogs[channel] = value; }
inline int host_get_controller_analog(int channel) { auto it = __host_controller_analogs.find(channel); return it == __host_controller_analogs.end() ? 0 : it->second; }
inline void host_set_controller_digital(int button, bool pressed) { __host_controller_digital[button] = pressed; }
inline bool host_get_controller_digital(int button) { auto it = __host_controller_digital.find(button); return it != __host_controller_digital.end() && it->second; }

} // namespace pros
