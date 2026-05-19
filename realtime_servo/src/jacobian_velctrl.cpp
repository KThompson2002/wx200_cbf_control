#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <realtime_servo/msg/relative_move.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <interbotix_xs_msgs/msg/joint_group_command.hpp>
#include <interbotix_xs_msgs/srv/robot_info.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <yaml-cpp/yaml.h>

class JacobianVelCtrlNode : public rclcpp::Node
{
public:
	struct JointLimits
	{
		std::string name;
        bool has_position_limits{false};
		double min_position{-std::numeric_limits<double>::infinity()};
		double max_position{std::numeric_limits<double>::infinity()};
		bool has_velocity_limits{false};
		double max_velocity{std::numeric_limits<double>::infinity()};
		bool has_acceleration_limits{false};
		double max_acceleration{std::numeric_limits<double>::infinity()};
	};

	JacobianVelCtrlNode()
	: Node("jacobian_velctrl_node"),
		filtered_dx_(0.0),
		filtered_dy_(0.0),
		filtered_dz_(0.0),
		filtered_dtheta_(0.0)
	{
		declare_parameter<std::string>("robot_description", "");
		declare_parameter<std::string>("urdf_path", "");
		declare_parameter<std::string>("base_link", "base_link");
		declare_parameter<std::string>("ee_link", "link6");
		declare_parameter<std::string>("joint_states_topic", "/joint_states");
		declare_parameter<std::string>("velocity_command_topic", "/velocity_pub/vel_command");
		declare_parameter<std::string>("joint_velocity_command_topic", "/arm_velocity_controller/commands");
		declare_parameter<std::string>("joint_position_command_topic", "/arm_controller/joint_trajectory");
		// output_mode: "position" (JointTrajectory via JTC), "velocity" (Float64MultiArray
		// via forward velocity controller), or "direct_position" (JointGroupCommand straight
		// to the XS driver, skipping JTC).
		declare_parameter<std::string>("output_mode", "position");
		declare_parameter<std::string>("xs_group_command_topic", "/wx200/commands/joint_group");
		declare_parameter<std::string>("xs_group_name", "arm");
		declare_parameter<std::string>("xs_robot_info_service", "/wx200/get_robot_info");
		// Fallback ordering used if the RobotInfo service does not respond within
		// xs_robot_info_timeout_sec; matches the standard wx200 motor_config yaml.
		declare_parameter<std::vector<std::string>>(
			"xs_joint_order_fallback",
			std::vector<std::string>{"waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"});
		declare_parameter<double>("xs_robot_info_timeout_sec", 5.0);
		declare_parameter<double>("position_command_time_from_start", 0.1);
		declare_parameter<double>("position_lead_clamp", 0.05);
		// Auto-reseed the integrator only when it has diverged from the
		// measured arm by more than this many radians on any joint. Sized
		// to be bigger than realistic gravity droop / tracking error but
		// smaller than any MoveIt move the user might initiate while
		// velocity control is idle.
		declare_parameter<double>("reseed_divergence_threshold", 0.3);
		// Joint-limit CBF: smoothly scale the whole qdot vector down as any
		// joint nears its position limit so the arm decelerates into the
		// bound instead of being hard-frozen. joint_cbf_alpha is the decay
		// rate (higher = brake later / allow more speed near the limit);
		// the soft slowdown zone for a joint moving at q̇ is roughly
		// q̇ / alpha radians wide.
		declare_parameter<bool>("enable_joint_cbf", true);
		declare_parameter<double>("joint_cbf_alpha", 2.0);
		declare_parameter<std::string>("joint_limits_yaml", "");
		declare_parameter<double>("position_limit_margin", 0.02);
		declare_parameter<bool>("use_damped_pseudoinverse", false);
		declare_parameter<double>("damping_lambda", 0.02);
		declare_parameter<bool>("use_adaptive_damping", true);
		declare_parameter<double>("singularity_threshold", 0.05);
		declare_parameter<double>("alpha", 0.5);
		declare_parameter<double>("control_rate_hz", 100.0);
		declare_parameter<bool>("publish_ee_state", true);
		declare_parameter<std::string>("ee_state_topic", "ee_state");
		declare_parameter<double>("command_timeout_sec", 0.0);

		std::string robot_description;
		std::string urdf_path;
		std::string base_link;
		std::string ee_link;
		std::string joint_states_topic;
		std::string velocity_command_topic;
		std::string joint_velocity_command_topic;
		std::string joint_position_command_topic;
		std::string output_mode;
		std::string xs_group_command_topic;
		std::string xs_group_name;
		std::string xs_robot_info_service;
		std::vector<std::string> xs_joint_order_fallback;
		double xs_robot_info_timeout_sec;
		std::string joint_limits_yaml;
		std::string ee_state_topic;
		double position_limit_margin;
		double position_command_time_from_start;
		double position_lead_clamp;
		double damping_lambda;
		double singularity_threshold;
		double alpha;
		double control_rate_hz;
		bool use_damped_pseudoinverse;
		bool use_adaptive_damping;
		bool publish_ee_state;
		double command_timeout_sec;

		get_parameter("robot_description", robot_description);
		get_parameter("urdf_path", urdf_path);
		get_parameter("base_link", base_link);
		get_parameter("ee_link", ee_link);
		get_parameter("joint_states_topic", joint_states_topic);
		get_parameter("velocity_command_topic", velocity_command_topic);
		get_parameter("joint_velocity_command_topic", joint_velocity_command_topic);
		get_parameter("joint_position_command_topic", joint_position_command_topic);
		get_parameter("output_mode", output_mode);
		get_parameter("xs_group_command_topic", xs_group_command_topic);
		get_parameter("xs_group_name", xs_group_name);
		get_parameter("xs_robot_info_service", xs_robot_info_service);
		get_parameter("xs_joint_order_fallback", xs_joint_order_fallback);
		get_parameter("xs_robot_info_timeout_sec", xs_robot_info_timeout_sec);
		get_parameter("position_command_time_from_start", position_command_time_from_start);
		get_parameter("position_lead_clamp", position_lead_clamp);
		get_parameter("reseed_divergence_threshold", reseed_divergence_threshold_);
		get_parameter("enable_joint_cbf", enable_joint_cbf_);
		get_parameter("joint_cbf_alpha", joint_cbf_alpha_);
		get_parameter("joint_limits_yaml", joint_limits_yaml);
		get_parameter("position_limit_margin", position_limit_margin);
		get_parameter("use_damped_pseudoinverse", use_damped_pseudoinverse);
		get_parameter("damping_lambda", damping_lambda);
		get_parameter("use_adaptive_damping", use_adaptive_damping);
		get_parameter("singularity_threshold", singularity_threshold);
		get_parameter("alpha", alpha);
		get_parameter("control_rate_hz", control_rate_hz);
		get_parameter("publish_ee_state", publish_ee_state);
		get_parameter("ee_state_topic", ee_state_topic);
		get_parameter("command_timeout_sec", command_timeout_sec);

		alpha_ = std::min(1.0, std::max(0.0, alpha));
		base_link_ = base_link;
		ee_link_ = ee_link;
		publish_ee_state_ = publish_ee_state;
		position_limit_margin_ = std::max(0.0, position_limit_margin);
		use_damped_pseudoinverse_ = use_damped_pseudoinverse;
		damping_lambda_ = std::max(0.0, damping_lambda);
		use_adaptive_damping_ = use_adaptive_damping;
		singularity_threshold_ = std::max(0.0, singularity_threshold);
		last_lambda_used_ = damping_lambda_;
		joint_position_command_topic_ = joint_position_command_topic;

		if (output_mode == "position") {
			output_mode_ = OutputMode::Position;
		} else if (output_mode == "velocity") {
			output_mode_ = OutputMode::Velocity;
		} else if (output_mode == "direct_position") {
			output_mode_ = OutputMode::DirectPosition;
		} else {
			RCLCPP_WARN(
				get_logger(),
				"Unknown output_mode '%s'; falling back to 'position'.",
				output_mode.c_str());
			output_mode_ = OutputMode::Position;
		}

		xs_group_name_ = xs_group_name;
		position_command_time_from_start_ = std::max(0.01, position_command_time_from_start);
		position_lead_clamp_ = std::max(0.0, position_lead_clamp);
		command_timeout_sec_ = std::max(0.0, command_timeout_sec);

		if (!initialize_kdl(robot_description, urdf_path, base_link, ee_link)) {
			throw std::runtime_error("Failed to initialize KDL chain/solver.");
		}

		load_joint_limits(joint_limits_yaml);

		joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
			joint_states_topic,
			20,
			std::bind(&JacobianVelCtrlNode::joint_state_callback, this, std::placeholders::_1));

		vel_cmd_sub_ = create_subscription<realtime_servo::msg::RelativeMove>(
			velocity_command_topic,
			20,
			std::bind(&JacobianVelCtrlNode::vel_cmd_callback, this, std::placeholders::_1));

		joint_velocity_cmd_pub_ =
			create_publisher<std_msgs::msg::Float64MultiArray>(joint_velocity_command_topic, 10);
		joint_position_cmd_pub_ =
			create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_position_command_topic, 10);
		xs_group_cmd_pub_ =
			create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(xs_group_command_topic, 10);
		jacobian_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/jacobian", 10);
		jacobian_pinv_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/jacobian_pinv", 10);

		if (output_mode_ == OutputMode::DirectPosition) {
			xs_robot_info_client_ =
				create_client<interbotix_xs_msgs::srv::RobotInfo>(xs_robot_info_service);
			resolve_xs_joint_order(xs_robot_info_service, xs_joint_order_fallback,
				xs_robot_info_timeout_sec);
		}

		if (publish_ee_state_) {
			ee_state_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(ee_state_topic, 10);
			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
			tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		}

		const auto timer_period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz, 1.0));
		control_period_sec_ = timer_period.count();
		control_timer_ = create_wall_timer(
			std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
			std::bind(&JacobianVelCtrlNode::control_cycle, this));

		RCLCPP_INFO(
			get_logger(),
			"Jacobian velocity controller ready. command_topic='%s', output_mode='%s'",
			velocity_command_topic.c_str(), output_mode.c_str());
	}

	void load_joint_limits(const std::string & joint_limits_yaml_path)
	{
		joint_limits_.clear();
		joint_limits_.resize(chain_joint_names_.size());

		if (joint_limits_yaml_path.empty()) {
			RCLCPP_WARN(get_logger(), "Parameter 'joint_limits_yaml' is empty. Joint limit saturation disabled.");
			has_joint_limits_ = false;
			return;
		}

		try {
			const YAML::Node root = YAML::LoadFile(joint_limits_yaml_path);
			const YAML::Node limits_root = root["joint_limits"];
			if (!limits_root) {
				RCLCPP_WARN(
					get_logger(),
					"File '%s' does not contain 'joint_limits' root key. Joint saturation disabled.",
					joint_limits_yaml_path.c_str());
				has_joint_limits_ = false;
				return;
			}

			for (size_t index = 0; index < chain_joint_names_.size(); ++index) {
				const auto & joint_name = chain_joint_names_[index];
				const YAML::Node joint_node = limits_root[joint_name];
				if (!joint_node) {
					continue;
				}

				JointLimits limits;
                limits.name = joint_name;
				if (joint_node["min_position"] && joint_node["max_position"]) {
					limits.has_position_limits = true;
					limits.min_position = joint_node["min_position"].as<double>();
					limits.max_position = joint_node["max_position"].as<double>();
				}

				limits.has_velocity_limits =
					joint_node["has_velocity_limits"] && joint_node["has_velocity_limits"].as<bool>();
				if (limits.has_velocity_limits && joint_node["max_velocity"]) {
					limits.max_velocity = std::abs(joint_node["max_velocity"].as<double>());
				}

				limits.has_acceleration_limits =
					joint_node["has_acceleration_limits"] && joint_node["has_acceleration_limits"].as<bool>();
				if (limits.has_acceleration_limits && joint_node["max_acceleration"]) {
					limits.max_acceleration = std::abs(joint_node["max_acceleration"].as<double>());
				}

				joint_limits_[index] = limits;
			}

			has_joint_limits_ = true;
			RCLCPP_INFO(
				get_logger(),
				"Loaded joint limits from '%s' for %zu chain joints.",
				joint_limits_yaml_path.c_str(),
				joint_limits_.size());
		} catch (const std::exception & ex) {
			RCLCPP_WARN(
				get_logger(),
				"Could not load joint limits from '%s': %s. Saturation disabled.",
				joint_limits_yaml_path.c_str(),
				ex.what());
			has_joint_limits_ = false;
		}
	}

private:
	bool initialize_kdl(
		const std::string & robot_description,
		const std::string & urdf_path,
		const std::string & base_link,
		const std::string & ee_link)
	{
		// Design decision: prefer robot_description over file path so this node can reuse
		// the launch-time xacro-expanded model without relying on package file lookup.
		const bool parsed_from_description =
			!robot_description.empty() && kdl_parser::treeFromString(robot_description, kdl_tree_);
		const bool parsed_from_file =
			parsed_from_description ? true : (!urdf_path.empty() && kdl_parser::treeFromFile(urdf_path, kdl_tree_));

		if (!parsed_from_description && !parsed_from_file) {
			RCLCPP_ERROR(get_logger(), "Could not parse URDF. Set 'robot_description' or valid 'urdf_path'.");
			return false;
		}

		if (!kdl_tree_.getChain(base_link, ee_link, kdl_chain_)) {
			RCLCPP_ERROR(
				get_logger(), "Failed to create KDL chain from '%s' to '%s'.", base_link.c_str(), ee_link.c_str());
			return false;
		}

		jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
		chain_joint_names_ = extract_chain_joint_names(kdl_chain_);

		if (chain_joint_names_.empty()) {
			RCLCPP_ERROR(get_logger(), "No non-fixed joints found in selected KDL chain.");
			return false;
		}

		// Log the chain joint order so we can compare against the JTC's
		// configured joint order. A mismatch — or a chain that picks up
		// an unexpected joint (e.g., a roll joint that doesn't exist on
		// the wx200 but does on the wx250s) — silently routes Jacobian
		// columns to the wrong motors and produces "X command also drops Z"
		// type symptoms.
		std::string chain_str;
		for (size_t i = 0; i < chain_joint_names_.size(); ++i) {
			chain_str += chain_joint_names_[i];
			if (i + 1 < chain_joint_names_.size()) chain_str += ", ";
		}
		RCLCPP_INFO(
			get_logger(),
			"KDL chain '%s' -> '%s' has %zu non-fixed joints: [%s]",
			base_link.c_str(), ee_link.c_str(),
			chain_joint_names_.size(), chain_str.c_str());
		return true;
	}

	static std::vector<std::string> extract_chain_joint_names(const KDL::Chain & chain)
	{
		std::vector<std::string> names;
		names.reserve(chain.getNrOfJoints());
		for (const auto & segment : chain.segments) {
			const auto & joint = segment.getJoint();
			if (joint.getType() != KDL::Joint::None) {
				names.push_back(joint.getName());
			}
		}
		return names;
	}

	void vel_cmd_callback(const realtime_servo::msg::RelativeMove::SharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(command_mutex_);
		latest_command_ = *msg;
		has_command_ = true;
		last_command_time_ = get_clock()->now();
	}

	void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(joint_state_mutex_);
		latest_joint_state_ = msg;
	}

	void update_velocities(const realtime_servo::msg::RelativeMove & latest)
	{
		filtered_dx_ = alpha_ * filtered_dx_ + (1.0 - alpha_) * latest.dx;
		filtered_dy_ = alpha_ * filtered_dy_ + (1.0 - alpha_) * latest.dy;
		filtered_dz_ = alpha_ * filtered_dz_ + (1.0 - alpha_) * latest.dz;
		filtered_dtheta_ = alpha_ * filtered_dtheta_ + (1.0 - alpha_) * latest.dtheta;
	}

	void control_cycle()
	{
		realtime_servo::msg::RelativeMove command_snapshot;
		sensor_msgs::msg::JointState::SharedPtr joint_state_snapshot;

		{
			std::lock_guard<std::mutex> lock(command_mutex_);
			if (has_command_ && command_timeout_sec_ > 0.0) {
				const double elapsed = (get_clock()->now() - last_command_time_).seconds();
				if (elapsed >= command_timeout_sec_) {
					has_command_ = false;
					filtered_dx_ = 0.0;
					filtered_dy_ = 0.0;
					filtered_dz_ = 0.0;
					filtered_dtheta_ = 0.0;
				}
			}
			if (!has_command_) {
				RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for /velocity_pub/vel_command.");
				publish_zero_joint_velocity();
				maybe_publish_ee_state();
				return;
			}
			command_snapshot = latest_command_;
		}

		{
			std::lock_guard<std::mutex> lock(joint_state_mutex_);
			joint_state_snapshot = latest_joint_state_;
		}

		if (!joint_state_snapshot || joint_state_snapshot->position.empty()) {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for valid /joint_states.");
			publish_zero_joint_velocity();
			maybe_publish_ee_state();
			return;
		}

		update_velocities(command_snapshot);

		std::unordered_map<std::string, double> joint_positions;
		const size_t count = std::min(joint_state_snapshot->name.size(), joint_state_snapshot->position.size());
		joint_positions.reserve(count);
		for (size_t i = 0; i < count; ++i) {
			joint_positions[joint_state_snapshot->name[i]] = joint_state_snapshot->position[i];
		}

		KDL::JntArray q(kdl_chain_.getNrOfJoints());
		for (size_t i = 0; i < chain_joint_names_.size(); ++i) {
			const auto found = joint_positions.find(chain_joint_names_[i]);
			if (found == joint_positions.end()) {
				RCLCPP_WARN_THROTTLE(
					get_logger(), *get_clock(), 2000, "Joint '%s' missing from /joint_states.", chain_joint_names_[i].c_str());
				publish_zero_joint_velocity();
				maybe_publish_ee_state();
				return;
			}
			q(i) = found->second;
		}

		KDL::Jacobian kdl_jacobian(kdl_chain_.getNrOfJoints());
		const int status = jacobian_solver_->JntToJac(q, kdl_jacobian);
		if (status < 0) {
			RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "KDL Jacobian solver failed: %d", status);
			publish_zero_joint_velocity();
			maybe_publish_ee_state();
			return;
		}

		const Eigen::MatrixXd jacobian = kdl_jacobian.data;

		// The wx200 is 5-DOF. Asking the pinv to satisfy a 6D twist (with
		// the angular rows zeroed) overconstrains the problem and produces
		// a least-squares solution that trades translation accuracy for
		// rotation suppression — notably, it prefers tilting the wrist
		// over lifting the whole arm because wrist tilt is "cheap" in the
		// joint-norm cost. Use a translation-only 3×N Jacobian so the
		// problem becomes underconstrained (3 task constraints, 5 joints)
		// and the pinv gives the minimum-norm qdot that *exactly* achieves
		// the commanded translation, leaving orientation free.
		const Eigen::MatrixXd jacobian_translation = jacobian.topRows(3);
		// Published for debugging only; the actual qdot comes from the
		// limit-aware solver below.
		const Eigen::MatrixXd jacobian_pinv = compute_pseudoinverse(jacobian_translation);

		Eigen::Vector3d xdot;
		xdot << filtered_dx_, filtered_dy_, filtered_dz_;

		// Solve with column-removal joint-limit handling: any joint at its
		// position limit is frozen and the remaining joints redistribute to
		// still produce the commanded EE velocity.
		std::vector<bool> frozen;
		Eigen::VectorXd qdot = solve_qdot_with_position_limits(q, jacobian_translation, xdot, frozen);

		// Joint-limit CBF: scale the whole vector down (direction-preserving)
		// as any joint nears its bound, so the arm eases into the limit
		// rather than slamming into the column-removal freeze.
		const double cbf_scale = joint_limit_cbf_scale(q, qdot);
		qdot *= cbf_scale;

		// recovered = what the EE will actually do given this qdot. With
		// column removal this should still match xdot unless the unfrozen
		// joints no longer span the task (then it's a least-squares fit and
		// some axis is sacrificed — which the log makes visible).
		const Eigen::Vector3d xdot_recovered = jacobian_translation * qdot;
		std::string qdot_str;
		std::string frozen_str;
		for (Eigen::Index i = 0; i < qdot.size(); ++i) {
			char buf[40];
			std::snprintf(buf, sizeof(buf), "%s=%+.3f",
				chain_joint_names_[static_cast<size_t>(i)].c_str(), qdot(i));
			qdot_str += buf;
			if (i + 1 < qdot.size()) qdot_str += " ";
			if (frozen[static_cast<size_t>(i)]) {
				if (!frozen_str.empty()) frozen_str += ", ";
				frozen_str += chain_joint_names_[static_cast<size_t>(i)];
			}
		}
		RCLCPP_INFO_THROTTLE(
			get_logger(), *get_clock(), 500,
			"cmd dx=%.3f dy=%.3f dz=%.3f | qdot {%s} | recovered=[%.3f, %.3f, %.3f] | frozen={%s} | cbf_scale=%.2f",
			filtered_dx_, filtered_dy_, filtered_dz_, qdot_str.c_str(),
			xdot_recovered.x(), xdot_recovered.y(), xdot_recovered.z(),
			frozen_str.empty() ? "none" : frozen_str.c_str(), cbf_scale);

		// Warn only when the redistribution can no longer achieve the
		// command (task no longer spanned) — that's the case where the EE
		// direction actually deviates from what was requested.
		const double recovery_err = (xdot_recovered - xdot).norm();
		if (!frozen_str.empty() && recovery_err > 1e-3) {
			RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 500,
				"Frozen joints {%s} leave the task underdetermined: EE will move [%.3f, %.3f, %.3f] vs commanded [%.3f, %.3f, %.3f] (err=%.3f).",
				frozen_str.c_str(),
				xdot_recovered.x(), xdot_recovered.y(), xdot_recovered.z(),
				filtered_dx_, filtered_dy_, filtered_dz_, recovery_err);
		}

		rclcpp::Time now = get_clock()->now();
		double dt = control_period_sec_;
		if (has_prev_control_time_) {
			dt = (now - prev_control_time_).seconds();
			if (dt <= 0.0) {
				dt = control_period_sec_;
			}
		}

		apply_velocity_acceleration_limits(qdot, dt);

		prev_control_time_ = now;
		has_prev_control_time_ = true;
		prev_qdot_ = qdot;
		has_prev_qdot_ = true;

		jacobian_pub_->publish(to_multi_array(jacobian));
		jacobian_pinv_pub_->publish(to_multi_array(jacobian_pinv));
		switch (output_mode_) {
			case OutputMode::Position:
				publish_joint_position_command(q, qdot, dt);
				break;
			case OutputMode::DirectPosition:
				publish_direct_position_command(q, qdot, dt);
				break;
			case OutputMode::Velocity:
				publish_joint_velocity_command(qdot);
				break;
		}
		maybe_publish_ee_state();
	}

	static constexpr double kQdotClampDeadband = 1e-3;

	Eigen::VectorXd advance_integrated_setpoint(
		const KDL::JntArray & q, const Eigen::VectorXd & qdot, double dt)
	{
		const Eigen::Index n = static_cast<Eigen::Index>(chain_joint_names_.size());

		Eigen::VectorXd q_meas(n);
		for (Eigen::Index i = 0; i < n; ++i) {
			q_meas(i) = q(static_cast<unsigned int>(i));
		}

		// Seed on first use or after a shape change.
		if (!integrated_position_initialized_ || integrated_positions_.size() != n) {
			integrated_positions_ = q_meas;
			integrated_position_initialized_ = true;
		} else {
			// Auto-reseed only when the integrator has lost touch with the
			// arm — e.g., MoveIt or a teach pendant moved the joints while
			// velocity control was idle. Gravity droop produces a few
			// centidegrees of divergence and must *not* trip this.
			for (Eigen::Index i = 0; i < n; ++i) {
				if (std::abs(integrated_positions_(i) - q_meas(i)) > reseed_divergence_threshold_) {
					RCLCPP_INFO(
						get_logger(),
						"Integrator diverged on joint %ld (|Δ|=%.3f > %.3f); reseeding from measured.",
						static_cast<long>(i),
						std::abs(integrated_positions_(i) - q_meas(i)),
						reseed_divergence_threshold_);
					integrated_positions_ = q_meas;
					break;
				}
			}
		}

		// Advance the setpoint at the true commanded speed: step over the real
		// elapsed dt, not the time_from_start window. Fixes the ~1/3-speed
		// scaling error from integrating over dt but executing over 0.1s.
		const double integration_dt = std::max(dt, 1e-3);
		integrated_positions_ += qdot * integration_dt;

		// Anti-windup: only clamp on the side we're actively commanding
		// in this cycle. The clamp's job is to prevent the integrator from
		// running away from the motor when the motor lags an active push;
		// it is *not* a "tracking error" clamp.
		//
		// If qdot ≈ 0 (idle hold), the integrator is frozen at the last
		// held setpoint. A symmetric clamp would pull that setpoint with
		// any measured drift — including gravity droop — and the motor
		// would stop fighting to hold position. By gating each side on
		// qdot's sign we let gravity-induced tracking error pile up
		// against the held setpoint, which is exactly the load the
		// motor's PID is supposed to resist. Sustained droop should be
		// caught by the effort watchdog, not papered over here.
		for (Eigen::Index i = 0; i < n; ++i) {
			if (qdot(i) > kQdotClampDeadband) {
				const double upper = q_meas(i) + position_lead_clamp_;
				if (integrated_positions_(i) > upper) {
					integrated_positions_(i) = upper;
				}
			} else if (qdot(i) < -kQdotClampDeadband) {
				const double lower = q_meas(i) - position_lead_clamp_;
				if (integrated_positions_(i) < lower) {
					integrated_positions_(i) = lower;
				}
			}
		}

		return q_meas;
	}

	Eigen::VectorXd apply_position_limits(const Eigen::VectorXd & q_cmd_in)
	{
		Eigen::VectorXd q_cmd = q_cmd_in;
		for (Eigen::Index i = 0; i < q_cmd.size(); ++i) {
			const JointLimits & lim = joint_limits_[static_cast<size_t>(i)];
			if (lim.has_position_limits) {
				q_cmd(i) = std::max(lim.min_position, std::min(lim.max_position, q_cmd(i)));
			}
		}
		return q_cmd;
	}

	void publish_joint_position_command(const KDL::JntArray & q, const Eigen::VectorXd & qdot, double dt)
	{
		advance_integrated_setpoint(q, qdot, dt);
		// Publish a point one time_from_start window ahead so the JTC
		// interpolates toward it at qdot.
		const Eigen::VectorXd q_cmd =
			apply_position_limits(integrated_positions_ + qdot * position_command_time_from_start_);
		publish_position_target(q_cmd);
	}

	void publish_direct_position_command(
		const KDL::JntArray & q, const Eigen::VectorXd & qdot, double dt)
	{
		if (!xs_joint_order_resolved_) {
			RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 2000,
				"xs joint order not resolved yet — skipping direct_position publish.");
			return;
		}

		advance_integrated_setpoint(q, qdot, dt);

		// In direct mode the DXL motors run their own profile per the
		// xs_sdk modes.yaml. We still lead by position_command_time_from_start
		// so the motor's profile has a forward setpoint to ramp toward;
		// position_lead_clamp keeps that from running away under stalls.
		const Eigen::VectorXd q_cmd =
			apply_position_limits(integrated_positions_ + qdot * position_command_time_from_start_);

		interbotix_xs_msgs::msg::JointGroupCommand msg;
		msg.name = xs_group_name_;
		msg.cmd.resize(xs_joint_order_.size());
		for (size_t xs_i = 0; xs_i < xs_joint_order_.size(); ++xs_i) {
			const size_t chain_i = xs_to_chain_index_[xs_i];
			msg.cmd[xs_i] = static_cast<float>(q_cmd(static_cast<Eigen::Index>(chain_i)));
		}
		xs_group_cmd_pub_->publish(msg);
	}

	void resolve_xs_joint_order(
		const std::string & service_name,
		const std::vector<std::string> & fallback_order,
		double timeout_sec)
	{
		std::vector<std::string> xs_order = fallback_order;
		bool from_service = false;

		if (xs_robot_info_client_->wait_for_service(
				std::chrono::milliseconds(static_cast<int64_t>(timeout_sec * 1000.0))))
		{
			auto request = std::make_shared<interbotix_xs_msgs::srv::RobotInfo::Request>();
			request->cmd_type = "group";
			request->name = xs_group_name_;
			auto future = xs_robot_info_client_->async_send_request(request);
			// We're still inside the constructor, so the executor isn't spinning
			// yet — spin_until_future_complete on the node directly.
			const auto status = rclcpp::spin_until_future_complete(
				get_node_base_interface(), future,
				std::chrono::milliseconds(static_cast<int64_t>(timeout_sec * 1000.0)));
			if (status == rclcpp::FutureReturnCode::SUCCESS) {
				const auto response = future.get();
				if (!response->joint_names.empty()) {
					xs_order = response->joint_names;
					from_service = true;
				}
			}
		}

		if (!from_service) {
			RCLCPP_WARN(
				get_logger(),
				"Could not query '%s' for xs_group '%s'; using fallback joint order.",
				service_name.c_str(), xs_group_name_.c_str());
		}

		// Build the index map: for each XS slot, find the matching chain joint.
		xs_to_chain_index_.clear();
		xs_to_chain_index_.reserve(xs_order.size());
		for (const auto & xs_name : xs_order) {
			const auto it = std::find(
				chain_joint_names_.begin(), chain_joint_names_.end(), xs_name);
			if (it == chain_joint_names_.end()) {
				RCLCPP_ERROR(
					get_logger(),
					"XS group joint '%s' is not in the KDL chain. direct_position will refuse to publish.",
					xs_name.c_str());
				xs_to_chain_index_.clear();
				xs_joint_order_.clear();
				xs_joint_order_resolved_ = false;
				return;
			}
			xs_to_chain_index_.push_back(
				static_cast<size_t>(std::distance(chain_joint_names_.begin(), it)));
		}

		xs_joint_order_ = xs_order;
		xs_joint_order_resolved_ = true;

		std::string order_str;
		for (size_t i = 0; i < xs_joint_order_.size(); ++i) {
			order_str += xs_joint_order_[i];
			if (i + 1 < xs_joint_order_.size()) order_str += ", ";
		}
		RCLCPP_INFO(
			get_logger(),
			"direct_position xs joint order [%s] (%s)",
			order_str.c_str(), from_service ? "from RobotInfo" : "fallback");
	}

	void publish_position_target(const Eigen::VectorXd & q_target)
	{
		trajectory_msgs::msg::JointTrajectory traj_msg;
		traj_msg.header.stamp = get_clock()->now();
		traj_msg.joint_names = chain_joint_names_;

		trajectory_msgs::msg::JointTrajectoryPoint point;
		point.positions.resize(static_cast<size_t>(q_target.size()));
		for (Eigen::Index i = 0; i < q_target.size(); ++i) {
			point.positions[static_cast<size_t>(i)] = q_target(i);
		}
        // for (size_t i = 0; i < command_to_chain_index_.size(); ++i) {
		// 	const auto chain_index = static_cast<Eigen::Index>(command_to_chain_index_[i]);
		// 	point.positions[i] = q_target(chain_index);
		// }
		point.time_from_start.sec = static_cast<int32_t>(position_command_time_from_start_);
		point.time_from_start.nanosec =
			static_cast<uint32_t>((position_command_time_from_start_ - point.time_from_start.sec) * 1e9);

		traj_msg.points.push_back(point);
		joint_position_cmd_pub_->publish(traj_msg);
	}

	Eigen::MatrixXd compute_pseudoinverse(const Eigen::MatrixXd & jacobian)
	{
		if (!use_damped_pseudoinverse_) {
			last_lambda_used_ = 0.0;
			return jacobian.completeOrthogonalDecomposition().pseudoInverse();
		}

		// Design decision: use an adaptive damped least-squares pseudoinverse near singularities.
		// Formula: J^T (J J^T + lambda^2 I)^-1
		double lambda = damping_lambda_;
		if (use_adaptive_damping_) {
			const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
			double sigma_min = 0.0;
			if (svd.singularValues().size() > 0) {
				sigma_min = svd.singularValues().minCoeff();
			}
			if (sigma_min >= singularity_threshold_) {
				lambda = 0.0;
			}
		}

		last_lambda_used_ = lambda;
		const Eigen::MatrixXd jj_t = jacobian * jacobian.transpose();
		const Eigen::MatrixXd regularized =
			jj_t + (lambda * lambda) * Eigen::MatrixXd::Identity(jj_t.rows(), jj_t.cols());

		return jacobian.transpose() * regularized.inverse();
	}

	// Solve for qdot, freezing any joint that the solution would push past
	// its position limit and redistributing the task onto the remaining
	// joints. Unlike post-hoc zeroing of qdot — which breaks the identity
	// J·qdot = xdot and so corrupts the commanded EE direction — removing
	// the joint's *column* before re-solving keeps the remaining joints
	// producing the commanded EE velocity exactly (as long as they still
	// span the task). Iterates because freezing one joint can push another
	// into its limit. Reports which joints ended up frozen.
	Eigen::VectorXd solve_qdot_with_position_limits(
		const KDL::JntArray & q,
		const Eigen::MatrixXd & jacobian_translation,
		const Eigen::Vector3d & xdot_desired,
		std::vector<bool> & frozen_out)
	{
		const Eigen::Index n = jacobian_translation.cols();
		std::vector<bool> active(static_cast<size_t>(n), true);
		Eigen::VectorXd qdot = Eigen::VectorXd::Zero(n);

		const bool limits_usable =
			has_joint_limits_ && joint_limits_.size() == static_cast<size_t>(n);

		// At most n iterations: each iteration freezes ≥1 more joint or stops.
		for (Eigen::Index iter = 0; iter < n; ++iter) {
			std::vector<Eigen::Index> active_idx;
			active_idx.reserve(static_cast<size_t>(n));
			for (Eigen::Index i = 0; i < n; ++i) {
				if (active[static_cast<size_t>(i)]) active_idx.push_back(i);
			}
			if (active_idx.empty()) {
				qdot.setZero();
				break;
			}

			Eigen::MatrixXd J_reduced(
				jacobian_translation.rows(), static_cast<Eigen::Index>(active_idx.size()));
			for (size_t k = 0; k < active_idx.size(); ++k) {
				J_reduced.col(static_cast<Eigen::Index>(k)) =
					jacobian_translation.col(active_idx[k]);
			}

			const Eigen::MatrixXd J_pinv = compute_pseudoinverse(J_reduced);
			const Eigen::VectorXd qdot_reduced = J_pinv * xdot_desired;

			qdot.setZero();
			for (size_t k = 0; k < active_idx.size(); ++k) {
				qdot(active_idx[k]) = qdot_reduced(static_cast<Eigen::Index>(k));
			}

			if (!limits_usable) break;

			bool froze_any = false;
			for (Eigen::Index i = 0; i < n; ++i) {
				if (!active[static_cast<size_t>(i)]) continue;
				const JointLimits & lim = joint_limits_[static_cast<size_t>(i)];
				if (!lim.has_position_limits) continue;
				const double q_i = q(static_cast<unsigned int>(i));
				const double upper_guard = lim.max_position - position_limit_margin_;
				const double lower_guard = lim.min_position + position_limit_margin_;
				if ((q_i >= upper_guard && qdot(i) > 0.0) ||
					(q_i <= lower_guard && qdot(i) < 0.0)) {
					active[static_cast<size_t>(i)] = false;
					froze_any = true;
				}
			}
			if (!froze_any) break;
		}

		frozen_out.assign(static_cast<size_t>(n), false);
		for (Eigen::Index i = 0; i < n; ++i) {
			frozen_out[static_cast<size_t>(i)] = !active[static_cast<size_t>(i)];
		}
		return qdot;
	}

	// Joint-limit CBF. Returns a single scalar in [0, 1] to scale the whole
	// qdot vector by, so the arm decelerates along the commanded Cartesian
	// direction as any joint approaches its bound. For each joint moving
	// toward a limit, the barrier h is the (margin-shifted) distance to that
	// limit and the admissible speed is alpha·h; the binding joint sets the
	// global scale. As h → 0 the scale → 0, braking the motion smoothly
	// before the column-removal freeze ever has to act.
	double joint_limit_cbf_scale(const KDL::JntArray & q, const Eigen::VectorXd & qdot)
	{
		if (!enable_joint_cbf_ || !has_joint_limits_ ||
			joint_limits_.size() != static_cast<size_t>(qdot.size())) {
			return 1.0;
		}

		double scale = 1.0;
		for (Eigen::Index i = 0; i < qdot.size(); ++i) {
			const JointLimits & lim = joint_limits_[static_cast<size_t>(i)];
			if (!lim.has_position_limits) continue;
			const double q_i = q(static_cast<unsigned int>(i));

			if (qdot(i) > 0.0) {
				const double h = (lim.max_position - position_limit_margin_) - q_i;
				const double cap = joint_cbf_alpha_ * std::max(0.0, h);
				if (qdot(i) > cap) {
					scale = std::min(scale, cap / qdot(i));
				}
			} else if (qdot(i) < 0.0) {
				const double h = q_i - (lim.min_position + position_limit_margin_);
				const double cap = joint_cbf_alpha_ * std::max(0.0, h);
				if (-qdot(i) > cap) {
					scale = std::min(scale, cap / (-qdot(i)));
				}
			}
		}
		return std::max(0.0, std::min(1.0, scale));
	}

	// Velocity and acceleration saturation. Position limits are handled
	// upstream in solve_qdot_with_position_limits (column removal), so they
	// are intentionally not repeated here. Velocity/accel clipping can still
	// perturb the EE direction slightly, but unlike a hard position-limit
	// zero it only scales magnitude / rate, so the direction error is small.
	void apply_velocity_acceleration_limits(Eigen::VectorXd & qdot, double dt)
	{
		if (!has_joint_limits_ || joint_limits_.size() != static_cast<size_t>(qdot.size())) {
			return;
		}

		for (Eigen::Index i = 0; i < qdot.size(); ++i) {
			const JointLimits & lim = joint_limits_[static_cast<size_t>(i)];

			if (lim.has_velocity_limits) {
				qdot(i) = std::max(-lim.max_velocity, std::min(lim.max_velocity, qdot(i)));
			}

			if (lim.has_acceleration_limits && has_prev_qdot_ && dt > 1e-6) {
				const double max_delta = lim.max_acceleration * dt;
				const double lower = prev_qdot_(i) - max_delta;
				const double upper = prev_qdot_(i) + max_delta;
				qdot(i) = std::max(lower, std::min(upper, qdot(i)));
			}
		}
	}

	void publish_joint_velocity_command(const Eigen::VectorXd & qdot)
	{
		std_msgs::msg::Float64MultiArray msg;
		msg.layout.dim.resize(1);
		msg.layout.dim[0].label = "joint_velocities";
		msg.layout.dim[0].size = static_cast<size_t>(qdot.size());
		msg.layout.dim[0].stride = static_cast<size_t>(qdot.size());
		msg.data.reserve(static_cast<size_t>(qdot.size()));

        for (Eigen::Index i = 0; i < qdot.size(); ++i) {
			msg.data.push_back(qdot(i));
		}
		// if (msg.data.size() > 2) {
		// 	msg.data[2] = -0.1; // temp test to see robot move
		// }

		// Design decision: command is published as Float64MultiArray to integrate with
		// a forward command velocity controller topic such as /arm_velocity_controller/commands.
		joint_velocity_cmd_pub_->publish(msg);
	}

	void publish_zero_joint_velocity()
	{
		const Eigen::VectorXd zero = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(chain_joint_names_.size()));
		switch (output_mode_) {
			case OutputMode::Position:
			case OutputMode::DirectPosition:
				// Idle in either position mode: stop publishing commands so
				// that other producers (MoveIt's follow_joint_trajectory
				// action, the XS driver's own services, etc.) can drive
				// the arm without being preempted at every control tick.
				// The integrator persists across the idle gap; if MoveIt
				// (or anything else) moves the arm meanwhile, the
				// reseed_divergence_threshold check in
				// advance_integrated_setpoint catches the jump and
				// reseeds. This intentionally does NOT reseed on every
				// short command gap, which would track gravity droop
				// downward each session.
				break;
			case OutputMode::Velocity:
				publish_joint_velocity_command(zero);
				break;
		}
		prev_qdot_ = zero;
		has_prev_qdot_ = true;
		prev_control_time_ = get_clock()->now();
		has_prev_control_time_ = true;
	}

	void maybe_publish_ee_state()
	{
		if (!publish_ee_state_ || !tf_buffer_ || !ee_state_pub_) {
			return;
		}

		geometry_msgs::msg::TransformStamped transform;
		try {
			transform = tf_buffer_->lookupTransform(base_link_, ee_link_, tf2::TimePointZero);
		} catch (const tf2::TransformException & ex) {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed for ee_state: %s", ex.what());
			return;
		}

		geometry_msgs::msg::PoseStamped ee_pose;
		ee_pose.header = transform.header;
        ee_pose.header.frame_id = ee_link_; // Note this is relative to base_link
		ee_pose.pose.position.x = transform.transform.translation.x;
		ee_pose.pose.position.y = transform.transform.translation.y;
		ee_pose.pose.position.z = transform.transform.translation.z;
		ee_pose.pose.orientation = transform.transform.rotation;
		ee_state_pub_->publish(ee_pose);
	}

	static std_msgs::msg::Float64MultiArray to_multi_array(const Eigen::MatrixXd & matrix)
	{
		std_msgs::msg::Float64MultiArray msg;
		msg.layout.dim.resize(2);
		msg.layout.dim[0].label = "rows";
		msg.layout.dim[0].size = matrix.rows();
		msg.layout.dim[0].stride = matrix.rows() * matrix.cols();
		msg.layout.dim[1].label = "cols";
		msg.layout.dim[1].size = matrix.cols();
		msg.layout.dim[1].stride = matrix.cols();
		msg.data.reserve(static_cast<size_t>(matrix.rows() * matrix.cols()));

		for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
			for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
				msg.data.push_back(matrix(row, col));
			}
		}
		return msg;
	}

	enum class OutputMode { Position, Velocity, DirectPosition };

	KDL::Tree kdl_tree_;
	KDL::Chain kdl_chain_;
	std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
	std::vector<std::string> chain_joint_names_;
	std::vector<std::string> command_joint_names_;
	std::vector<size_t> command_to_chain_index_;
	// XS group joint order (from RobotInfo, or fallback) and the lookup
	// from xs slot index -> chain joint index used in direct_position mode.
	std::vector<std::string> xs_joint_order_;
	std::vector<size_t> xs_to_chain_index_;
	bool xs_joint_order_resolved_{false};
	std::string xs_group_name_;

	std::mutex command_mutex_;
	std::mutex joint_state_mutex_;
	realtime_servo::msg::RelativeMove latest_command_;
	bool has_command_{false};
	sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;

	double alpha_;
	double filtered_dx_;
	double filtered_dy_;
	double filtered_dz_;
	double filtered_dtheta_;
	std::string base_link_;
	std::string ee_link_;
	bool publish_ee_state_{true};
	bool has_joint_limits_{false};
	double position_limit_margin_{0.02};
	std::vector<JointLimits> joint_limits_;
	OutputMode output_mode_{OutputMode::Position};
	std::string joint_position_command_topic_;
	double position_command_time_from_start_{0.1};
	double position_lead_clamp_{0.05};
	bool integrated_position_initialized_{false};
	Eigen::VectorXd integrated_positions_;
	double reseed_divergence_threshold_{0.3};
	bool enable_joint_cbf_{true};
	double joint_cbf_alpha_{2.0};

	bool use_damped_pseudoinverse_{true};
	double damping_lambda_{0.02};
	bool use_adaptive_damping_{true};
	double singularity_threshold_{0.05};
	double last_lambda_used_{0.02};

	double control_period_sec_{0.01};
	bool has_prev_control_time_{false};
	rclcpp::Time prev_control_time_{0, 0, RCL_ROS_TIME};
	double command_timeout_sec_{0.0};
	rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
	bool has_prev_qdot_{false};
	Eigen::VectorXd prev_qdot_;

	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
	rclcpp::Subscription<realtime_servo::msg::RelativeMove>::SharedPtr vel_cmd_sub_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_cmd_pub_;
	rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_position_cmd_pub_;
	rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr xs_group_cmd_pub_;
	rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>::SharedPtr xs_robot_info_client_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_pub_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_pinv_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_state_pub_;
	rclcpp::TimerBase::SharedPtr control_timer_;

	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<JacobianVelCtrlNode>());
	rclcpp::shutdown();
	return 0;
}