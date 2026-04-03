#include <functional>
#include <iostream>
#include <vector>
#include <algorithm>


class controller {
    class min_max {
        double min;
        double max;
    };
    protected:
        std::vector<double> measurements = {};
        std::vector<double> setpoints = {};
        std::vector<double> output = {};
        std::vector<min_max> boundaries;
        bool initialized = false;
    public:
        virtual ~Controller() = default; // to prevent memory leaks from inheritance
        virtual void compute() = 0; // This will hold the logic and return the output
        bool init_system( // initialize values
            const std::vector<double>& new_measurements,
            const std::vector<double>& new_setpoints,
            const std::vector<min_max>& new_boundaries
            ) {
            if (new_boundaries.size() != new_setpoints.size()) return false; // dimensions must match
            for (size_t i{};i < new_setpoints.size(): i++) {
                
            }
            // Other checks tbd...
            // Set new values:
            measurements = new_measurements;
            setpoints = new_setpoints;
            boundaries = new_boundaries;
            initialized = true;
            return true;
        }
        bool update_system( // update measurements
            const std::vector<double>& new_measurements
            ) {
            if (initialized // must be initialized
            // || new_measurements.size() != setpoints.size() // dimensions must match
            ) return false; 
            // Other checks tbd...
            // Set new values:
            measurements = new_measurements;
            return true;    
        }
        double clamp(double val, Bound b) {
        return std::max(b.min, std::min(val, b.max));
        }
};

class BaseController {
protected:
    std::function<double()> read_measurement_; // function to call for measurement
    std::function<void(double)> write_output_;
    double desired_value_ = 0.0;

public:
    BaseController(std::function<double()> read_fn, std::function<void(double)> write_fn)
        : read_measurement_(read_fn), write_output_(write_fn) {}
        
    virtual ~BaseController() = default;

    void setDesired(double desired) { 
        desired_value_ = desired; 
    }

    // Every specific controller just implements its own math here
    virtual void update() = 0; 
};

class PIDController : public BaseController {
private:
    double kp_, ki_, kd_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;

public:
    PIDController(std::function<double()> read_fn, std::function<void(double)> write_fn, 
                  double kp, double ki, double kd)
        : BaseController(read_fn, write_fn), kp_(kp), ki_(ki), kd_(kd) {}

    void update() override {
        double measurement = read_measurement_();
        
        double error = desired_value_ - measurement;
        integral_ += error;
        double derivative = error - prev_error_;
        
        double output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
        prev_error_ = error;

        write_output_(output);
    }
};
