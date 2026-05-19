// Keyboard teleop for RelativeMove velocity commands.
//
// Publishing is decoupled from key events: a fast reader latches the current
// velocity from the most recent keypress, and a steady timer publishes that
// latched velocity at a fixed rate (default 3 Hz, matching the world-model
// rollout). This gives the velocity controller a regular command stream it
// can integrate smoothly, instead of the bursty autorepeat-driven stream the
// old event-publish produced. Raw terminals give no key-up event, so a key
// "release" is inferred from the time since the last keypress.

#include <rclcpp/rclcpp.hpp>
#include <realtime_servo/msg/relative_move.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop()
    : Node("keyboard_relmove")
    {
        scale_by_ = declare_parameter("scale_by", 0.1);
        // Match hardware_env's control_freq (10 Hz) so teleop drives the
        // exact same command cadence the world-model rollout produces.
        publish_rate_hz_ = declare_parameter("publish_rate_hz", 10.0);
        // Must exceed the OS keyboard autorepeat delay (~0.25 s) so a held
        // key doesn't momentarily zero during the pre-repeat gap; kept just
        // above it for a snappy stop on release.
        key_hold_timeout_ = declare_parameter("key_hold_timeout", 0.35);

        pub_ = create_publisher<realtime_servo::msg::RelativeMove>(
            "/velocity_pub/vel_command", 10);

        setupTerminal();

        // Fast reader: faster than both the publish rate and OS autorepeat.
        read_timer_ = create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&KeyboardTeleop::readKeys, this));

        const auto period =
            std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
        publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&KeyboardTeleop::publishLatched, this));

        last_key_time_ = now();
        RCLCPP_INFO(get_logger(), "Keyboard teleop started at %.1f Hz", publish_rate_hz_);
        RCLCPP_INFO(get_logger(),
            "w:+z s:-z a:+x d:-x z:-y x:+y c:+dtheta v:-dtheta e:zero q:quit");
    }

    ~KeyboardTeleop()
    {
        restoreTerminal();
    }

private:
    rclcpp::Publisher<realtime_servo::msg::RelativeMove>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    double scale_by_;
    double publish_rate_hz_;
    double key_hold_timeout_;

    double dx_{0.0}, dy_{0.0}, dz_{0.0}, dtheta_{0.0};
    rclcpp::Time last_key_time_;

    struct termios orig_termios_;

    void setupTerminal()
    {
        tcgetattr(STDIN_FILENO, &orig_termios_);
        struct termios raw = orig_termios_;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }

    void setVel(double dx, double dy, double dz, double dtheta)
    {
        dx_ = dx;
        dy_ = dy;
        dz_ = dz;
        dtheta_ = dtheta;
    }

    void readKeys()
    {
        // Drain every byte queued since the last tick (autorepeat can stack
        // several); the last key wins.
        char c;
        bool got_key = false;
        while (read(STDIN_FILENO, &c, 1) > 0) {
            got_key = true;
            if (c == 'q') {
                rclcpp::shutdown();
                return;
            }
            switch (c) {
                case 'w': setVel(0.0, 0.0,  scale_by_, 0.0); break;
                case 's': setVel(0.0, 0.0, -scale_by_, 0.0); break;
                case 'a': setVel( scale_by_, 0.0, 0.0, 0.0); break;
                case 'd': setVel(-scale_by_, 0.0, 0.0, 0.0); break;
                case 'z': setVel(0.0, -scale_by_, 0.0, 0.0); break;
                case 'x': setVel(0.0,  scale_by_, 0.0, 0.0); break;
                case 'c': setVel(0.0, 0.0, 0.0,  scale_by_ * 5.0); break;
                case 'v': setVel(0.0, 0.0, 0.0, -scale_by_ * 5.0); break;
                case 'e': setVel(0.0, 0.0, 0.0, 0.0); break;
                default: break;
            }
        }
        if (got_key) {
            last_key_time_ = now();
        }
    }

    void publishLatched()
    {
        // Infer key release from the gap since the last keypress.
        if ((now() - last_key_time_).seconds() > key_hold_timeout_) {
            setVel(0.0, 0.0, 0.0, 0.0);
        }
        auto msg = realtime_servo::msg::RelativeMove();
        msg.dx = dx_;
        msg.dy = dy_;
        msg.dz = dz_;
        msg.dtheta = dtheta_;
        pub_->publish(msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
    rclcpp::shutdown();
}
