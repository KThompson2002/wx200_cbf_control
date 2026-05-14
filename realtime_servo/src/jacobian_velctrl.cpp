#include <algorithm>
#include <cmath>
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
		declare_parameter<std::string>("output_mode", "position");
		declare_parameter<double>("position_command_time_from_start", 0.1);
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
		std::string joint_limits_yaml;
		std::string ee_state_topic;
		double position_limit_margin;
		double position_command_time_from_start;
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
		get_parameter("position_command_time_from_start", position_command_time_from_start);
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
		use_position_output_ = (output_mode == "position");
		position_command_time_from_start_ = std::max(0.01, position_command_time_from_start);
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
		jacobian_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/jacobian", 10);
		jacobian_pinv_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/jacobian_pinv", 10);

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
			velocity_command_topic.c_str(), use_position_output_ ? "position" : "velocity");
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
		// std::vector<size_t> command_to_chain_index_ = {2, 1, 0, 3, 4, 5}; // hard code mapping for now since we know the joint order in the URDF and the command convention we want to use. This will need to be more flexible if we want to support different robots and/or command conventions.

		if (chain_joint_names_.empty()) {
			RCLCPP_ERROR(get_logger(), "No non-fixed joints found in selected KDL chain.");
			return false;
		}
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
		const Eigen::MatrixXd jacobian_pinv = compute_pseudoinverse(jacobian);

		Eigen::VectorXd xdot = Eigen::VectorXd::Zero(6);
		RCLCPP_INFO_THROTTLE(
			get_logger(), *get_clock(), 500,
			"Filtered command: dx=%.3f dy=%.3f dz=%.3f dtheta=%.3f",
			filtered_dx_, filtered_dy_, filtered_dz_, filtered_dtheta_);
		xdot(0) = filtered_dx_;
		xdot(1) = filtered_dy_;
		xdot(2) = filtered_dz_;
		// Design decision: map RelativeMove.dtheta to end-effector angular z velocity only.
		// Extend this to roll/pitch/yaw fields later if full orientation-rate control is needed.
		// xdot(5) = filtered_dtheta_;

		Eigen::VectorXd qdot = jacobian_pinv * xdot;

		rclcpp::Time now = get_clock()->now();
		double dt = control_period_sec_;
		if (has_prev_control_time_) {
			dt = (now - prev_control_time_).seconds();
			if (dt <= 0.0) {
				dt = control_period_sec_;
			}
		}

		apply_joint_limits(q, qdot, dt);
		prev_control_time_ = now;
		has_prev_control_time_ = true;
		prev_qdot_ = qdot;
		has_prev_qdot_ = true;

		jacobian_pub_->publish(to_multi_array(jacobian));
		jacobian_pinv_pub_->publish(to_multi_array(jacobian_pinv));
		if (use_position_output_) {
			publish_joint_position_command(q, qdot, dt);
		} else {
			publish_joint_velocity_command(qdot);
		}
		maybe_publish_ee_state();
	}

	void publish_joint_position_command(const KDL::JntArray & q, const Eigen::VectorXd & qdot, double dt)
	{
		Eigen::VectorXd q_cmd(static_cast<Eigen::Index>(chain_joint_names_.size()));
		if (!integrated_position_initialized_ || integrated_positions_.size() != q_cmd.size()) {
			for (Eigen::Index i = 0; i < q_cmd.size(); ++i) {
				q_cmd(i) = q(static_cast<unsigned int>(i));
			}
			integrated_positions_ = q_cmd;
			integrated_position_initialized_ = true;
		}

		const double integration_dt = std::max(dt, 1e-3);
		q_cmd = integrated_positions_ + qdot * integration_dt;

		for (Eigen::Index i = 0; i < q_cmd.size(); ++i) {
			const JointLimits & lim = joint_limits_[static_cast<size_t>(i)];
			if (lim.has_position_limits) {
				q_cmd(i) = std::max(lim.min_position, std::min(lim.max_position, q_cmd(i)));
			}
		}

		publish_position_target(q_cmd);
		integrated_positions_ = q_cmd;

		// publish_position_target(q_cmd);
		// Eigen::VectorXd q_cmd(static_cast<Eigen::Index>(chain_joint_names_.size()));
		// for (Eigen::Index i = 0; i < q_cmd.size(); ++i) {
		// 	q_cmd(i) = q(static_cast<unsigned int>(i)) + qdot(i) * position_command_time_from_start_;
		// }

		// for (Eigen::Index i = 0; i < q_cmd.size(); ++i) {
		// 	const JointLimits & lim = joint_limits_[static_cast<size_t>(i)];
		// 	if (lim.has_position_limits) {
		// 		q_cmd(i) = std::max(lim.min_position, std::min(lim.max_position, q_cmd(i)));
		// 	}
		// }
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

	void apply_joint_limits(const KDL::JntArray & q, Eigen::VectorXd & qdot, double dt)
	{
		if (!has_joint_limits_ || joint_limits_.size() != static_cast<size_t>(qdot.size())) {
			return;
		}

		for (Eigen::Index i = 0; i < qdot.size(); ++i) {
			const JointLimits & lim = joint_limits_[static_cast<size_t>(i)];

			if (lim.has_position_limits) {
				const double q_i = q(static_cast<unsigned int>(i));
				const double upper_guard = lim.max_position - position_limit_margin_;
				const double lower_guard = lim.min_position + position_limit_margin_;
				if (q_i >= upper_guard && qdot(i) > 0.0) {
					qdot(i) = 0.0;
				}
				if (q_i <= lower_guard && qdot(i) < 0.0) {
					qdot(i) = 0.0;
				}
			}

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
		if (use_position_output_) {
			// Idle in position mode: stop publishing JointTrajectory commands so that
			// other producers (MoveIt's follow_joint_trajectory action, etc.) can drive
			// the JTC without being preempted at every control tick. Mark the integrated
			// position as uninitialized so the next active cycle re-seeds it from the
			// measured joint positions — otherwise the arm would snap back to its
			// pre-MoveIt pose on the first new velocity command.
			integrated_position_initialized_ = false;
		} else {
			publish_joint_velocity_command(zero);
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

	KDL::Tree kdl_tree_;
	KDL::Chain kdl_chain_;
	std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
	std::vector<std::string> chain_joint_names_;
	std::vector<std::string> command_joint_names_;
	std::vector<size_t> command_to_chain_index_;

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
	bool use_position_output_{true};
	std::string joint_position_command_topic_;
	double position_command_time_from_start_{0.1};
	bool integrated_position_initialized_{false};
	Eigen::VectorXd integrated_positions_;

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