// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{
  namespace
  {
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points,
                               double start_z, double target_z)
    {
      if (points.empty())
        return;
      if (points.size() == 1)
      {
        points.front().z() = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i].z() = start_z + ratio * (target_z - start_z);
      }

      points.front().z() = start_z;
      points.back().z() = target_z;
    }

    bool planarYaw(const Eigen::Vector3d &from, const Eigen::Vector3d &to, double &yaw)
    {
      const Eigen::Vector2d direction = (to - from).head<2>();
      if (!direction.allFinite())
        return false;
      if (direction.squaredNorm() < 1e-8)
      {
        yaw = 0.0;
        return true;
      }
      yaw = std::atan2(direction.y(), direction.x());
      return true;
    }

    bool controlPointYaw(const Eigen::MatrixXd &points, int index, double &yaw)
    {
      if (points.cols() <= 1 || index < 0 || index >= points.cols())
        return false;
      const int previous = std::max(0, index - 1);
      const int next = std::min(static_cast<int>(points.cols()) - 1, index + 1);
      return planarYaw(points.col(previous), points.col(next), yaw);
    }

    std::string formatPointState(const GridMap::Ptr &map, const Eigen::Vector3d &pt, double yaw)
    {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2)
          << "[" << pt.x() << ", " << pt.y() << ", " << pt.z() << "]";

      if (map == nullptr)
      {
        oss << " map=null";
        return oss.str();
      }

      const auto collision = map->checkFootprint(pt, yaw);
      oss << " footprint_status=" << static_cast<int>(collision.status)
          << " cylinder=" << collision.cylinder_index;
      return oss.str();
    }
  } // namespace

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis)
  {
    node->declare_parameter("manager/max_vel", -1.0);
    node->declare_parameter("manager/max_acc", -1.0);
    node->declare_parameter("manager/max_jerk", -1.0);
    node->declare_parameter("optimization/vel_tolerance", 1.0);
    node->declare_parameter("optimization/acc_tolerance", 1.0);
    node->declare_parameter("manager/feasibility_tolerance", 0.0);
    node->declare_parameter("manager/control_points_distance", -1.0);
    node->declare_parameter("manager/planning_horizon", 5.0);
    node->declare_parameter("manager/use_distinctive_trajs", false);
    node->declare_parameter("manager/drone_id", -1);
    node->declare_parameter("manager/verbose_replan_log", false);

    node->get_parameter("manager/max_vel", pp_.max_vel_);
    node->get_parameter("manager/max_acc", pp_.max_acc_);
    node->get_parameter("manager/max_jerk", pp_.max_jerk_);
    node->get_parameter("optimization/vel_tolerance", pp_.vel_tolerance_);
    node->get_parameter("optimization/acc_tolerance", pp_.acc_tolerance_);
    node->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
    node->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
    node->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
    node->get_parameter("manager/use_distinctive_trajs", pp_.use_distinctive_trajs);
    node->get_parameter("manager/drone_id", pp_.drone_id);
    node->get_parameter("manager/verbose_replan_log", verbose_replan_log_);

    if (!std::isfinite(pp_.max_vel_) || pp_.max_vel_ <= 0.0 ||
        !std::isfinite(pp_.max_acc_) || pp_.max_acc_ <= 0.0 ||
        !std::isfinite(pp_.vel_tolerance_) || pp_.vel_tolerance_ < 0.0 ||
        !std::isfinite(pp_.acc_tolerance_) || pp_.acc_tolerance_ < 0.0 ||
        !std::isfinite(pp_.ctrl_pt_dist) || pp_.ctrl_pt_dist <= 0.0 ||
        !std::isfinite(pp_.planning_horizen_) || pp_.planning_horizen_ <= 0.0)
      throw std::runtime_error("manager motion, horizon and control-point limits must be positive");
    if (pp_.use_distinctive_trajs)
      throw std::runtime_error(
          "manager/use_distinctive_trajs must be false for deterministic real-robot planning");

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);

    bspline_optimizer_.reset(new BsplineOptimizer);
    // bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setParam(node);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel,
                                        bool use_polynomial_seed, bool random_polynomial_seed)
  {
    static int count = 0;
    if (verbose_replan_log_)
    {
      printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count);
    }
    count++;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      if (continous_failures_count_ % 20 == 0)
      {
        cout << "Close to goal" << endl;
      }
      continous_failures_count_++;
      return false;
    }

    double route_yaw = 0.0;
    planarYaw(start_pt, local_target_pt, route_yaw);

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    rclcpp::Time t_start = rclcpp::Clock().now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);

    /*** STEP 1: INIT
    根据起始点和目标点的距离计算首个时间步长ts,向量的模大于0.1则用1.5倍否则用5倍
    ***/
    // Keep the initial polynomial closer to the original 3d-navi setup so the
    // terminal control points are less likely to start inside inflated voxels.
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    point_set.clear();
    start_end_derivatives.clear();

    if (!use_polynomial_seed)
    {
      if (local_data_.duration_ <= 1e-3 || local_data_.start_time_.seconds() <= 1e-5)
        return false;

      const double t_cur = std::clamp(
          (rclcpp::Clock(RCL_SYSTEM_TIME).now() - local_data_.start_time_).seconds(),
          0.0, local_data_.duration_);

      std::vector<double> pseudo_arc_length(1, 0.0);
      std::vector<Eigen::Vector3d> segment_points;
      double t = t_cur;
      for (; t < local_data_.duration_ + 1e-3; t += ts)
      {
        segment_points.push_back(local_data_.position_traj_.evaluateDeBoorT(
            std::min(t, local_data_.duration_)));
        if (segment_points.size() > 1)
          pseudo_arc_length.push_back(
              pseudo_arc_length.back() +
              (segment_points.back() - segment_points[segment_points.size() - 2]).norm());
      }
      t -= ts;

      if (segment_points.size() < 2 || pseudo_arc_length.back() <= 1e-6)
        return false;

      const Eigen::Vector3d tail = local_data_.position_traj_.evaluateDeBoorT(t);
      const double polynomial_time = (tail - local_target_pt).norm() / pp_.max_vel_ * 2.0;
      if (polynomial_time > ts)
      {
        const PolynomialTraj connector = PolynomialTraj::one_segment_traj_gen(
            tail, local_data_.velocity_traj_.evaluateDeBoorT(t),
            local_data_.acceleration_traj_.evaluateDeBoorT(t), local_target_pt,
            local_target_vel, Eigen::Vector3d::Zero(), polynomial_time);
        for (double connector_t = ts; connector_t < polynomial_time; connector_t += ts)
        {
          segment_points.push_back(connector.evaluate(connector_t));
          pseudo_arc_length.push_back(
              pseudo_arc_length.back() +
              (segment_points.back() - segment_points[segment_points.size() - 2]).norm());
        }
      }

      if (pseudo_arc_length.size() < 2 || pseudo_arc_length.back() <= 1e-6)
        return false;

      double control_point_spacing = pp_.ctrl_pt_dist * 1.5;
      do
      {
        control_point_spacing /= 1.5;
        point_set.clear();
        double sample_length = 0.0;
        size_t segment_index = 0;
        while (segment_index + 1 < pseudo_arc_length.size() &&
               sample_length <= pseudo_arc_length.back())
        {
          if (sample_length >= pseudo_arc_length[segment_index] &&
              sample_length < pseudo_arc_length[segment_index + 1])
          {
            const double segment_length =
                pseudo_arc_length[segment_index + 1] - pseudo_arc_length[segment_index];
            if (segment_length <= 1e-9)
              return false;
            const double ratio =
                (sample_length - pseudo_arc_length[segment_index]) / segment_length;
            point_set.push_back(
                (1.0 - ratio) * segment_points[segment_index] +
                ratio * segment_points[segment_index + 1]);
            sample_length += control_point_spacing;
          }
          else
          {
            ++segment_index;
          }
        }
        point_set.push_back(local_target_pt);
      } while (point_set.size() < 7);

      start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
      start_end_derivatives.push_back(local_target_vel);
      start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
      start_end_derivatives.push_back(Eigen::Vector3d::Zero());
    }
    else
    // A fresh target uses a direct/lateral polynomial seed. Continuous replanning
    // reuses the unexecuted trajectory tail, matching SCAN's continuity model.
    {
        // 用于存储生成的轨迹
        PolynomialTraj gl_traj;
        double dist = (start_pt - local_target_pt).norm();
        // 判断 速度的平方/加速度 是否大于dist，并决定如何计算时间
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!random_polynomial_seed)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(
              start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
              Eigen::Vector3d::Zero(), time);
        }
        else
        {
          const Eigen::Vector3d horizon_dir =
              ((start_pt - local_target_pt).cross(Eigen::Vector3d(0.0, 0.0, 1.0))).normalized();
          const Eigen::Vector3d vertical_dir =
              ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          const double failure_scale =
              -0.978 / (continous_failures_count_ + 0.989) + 0.989;
          const Eigen::Vector3d random_inserted_pt =
              0.5 * (start_pt + local_target_pt) +
              (static_cast<double>(rand()) / RAND_MAX - 0.5) * dist *
                  horizon_dir * 0.8 * failure_scale +
              (static_cast<double>(rand()) / RAND_MAX - 0.5) * dist *
                  vertical_dir * 0.4 * failure_scale;
          Eigen::MatrixXd seed_points(3, 3);
          seed_points.col(0) = start_pt;
          seed_points.col(1) = random_inserted_pt;
          seed_points.col(2) = local_target_pt;
          Eigen::VectorXd seed_times(2);
          seed_times << 0.5 * time, 0.5 * time;
          gl_traj = PolynomialTraj::minSnapTraj(
              seed_points, start_vel, local_target_vel, start_acc,
              Eigen::Vector3d::Zero(), seed_times);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
    }
    // 将轨迹变为B样条轨迹
    applyLinearZReference(point_set, start_pt.z(), local_target_pt.z());

    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    const int debug_ctrl_pt_num = std::min(3, static_cast<int>(ctrl_pts.cols()));
    if (visualization_ != nullptr)
    {
      const Eigen::Vector4d ctrl_pt_colors[3] = {
          Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
          Eigen::Vector4d(1.0, 0.4, 0.0, 1.0),
          Eigen::Vector4d(1.0, 0.8, 0.0, 1.0)};

      for (int i = 0; i < debug_ctrl_pt_num; ++i)
      {
        visualization_->displayGoalPoint(ctrl_pts.col(i), ctrl_pt_colors[i], 0.16, 9100 + i);
      }
    }

    if (grid_map_ != nullptr)
    {
      bool ctrl_pt_problem = !grid_map_->isFootprintFree(start_pt, route_yaw);
      std::ostringstream ctrl_pt_state;

      ctrl_pt_state << "start=" << formatPointState(grid_map_, start_pt, route_yaw)
                    << " | local_target=" << formatPointState(grid_map_, local_target_pt, route_yaw);

      for (int i = 0; i < debug_ctrl_pt_num; ++i)
      {
        const Eigen::Vector3d ctrl_pt = ctrl_pts.col(i);
        double ctrl_pt_yaw = 0.0;
        const bool valid_yaw = controlPointYaw(ctrl_pts, i, ctrl_pt_yaw);
        const bool ctrl_pt_free = valid_yaw && grid_map_->isFootprintFree(ctrl_pt, ctrl_pt_yaw);
        ctrl_pt_problem = ctrl_pt_problem || !ctrl_pt_free;
        ctrl_pt_state << " | cp" << i << "="
                      << (valid_yaw ? formatPointState(grid_map_, ctrl_pt, ctrl_pt_yaw)
                                    : std::string("invalid_yaw"));
      }

      if (ctrl_pt_problem)
      {
        RCLCPP_WARN(rclcpp::get_logger("planner_manager"), "[EGO][debug] %s", ctrl_pt_state.str().c_str());
      }
    }

    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);
    // Match SCAN: if colliding seed cannot be repaired by A*, abort before optimizing
    // a wall-crossing polynomial without rebound guides (and without flashing it in RViz).
    if (bspline_optimizer_->initControlPointsFailed())
    {
      continous_failures_count_++;
      return false;
    }
    // 计算时间差并更新时间
    auto now = rclcpp::Clock().now();
    t_init = now - t_start;
    t_start = now;

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    if (pp_.use_distinctive_trajs)
    {
      // cout << "enter" << endl;
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      if (verbose_replan_log_)
      {
        cout << "\033[1;33m"
             << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;
      }

      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {

          if (verbose_replan_log_)
            cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          if (verbose_replan_log_)
            cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = rclcpp::Clock().now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2);
    }
    else
    {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = rclcpp::Clock().now() - t_start;
      // static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    if (verbose_replan_log_)
      cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // Match SCAN: do not publish/display a failed/colliding control-point set.
      continous_failures_count_++;
      return false;
    }

    t_start = rclcpp::Clock().now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time profile.
    if (pp_.drone_id <= 0)
    {

      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        if (verbose_replan_log_)
          cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success || !checkDynamicFeasibility(pos))
      {
        printf("\033[34mThis refined trajectory is dynamically infeasible. Skip publishing it.\n\033[0m");
        continous_failures_count_++;
        return false;
      }
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "IN SWARM MODE, REFINE DISABLED!");
      }
    }

    // t_refine = ros::Time::now() - t_start;
    t_refine = rclcpp::Clock().now() - t_start;

    // save planned results
    updateTrajInfo(pos, rclcpp::Clock().now());

    static double sum_time = 0;
    static int count_success = 0;

    sum_time += (t_init + t_opt + t_refine).seconds();

    count_success++;

    // cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    if (verbose_replan_log_)
    {
      cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << ",avg_time=" << sum_time / count_success << endl;
    }

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::checkDynamicFeasibility(
      UniformBspline position_traj) const
  {
    UniformBspline velocity_traj = position_traj.getDerivative();
    UniformBspline acceleration_traj = velocity_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double velocity_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acceleration_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double sample_time = std::min(t, duration);
      const Eigen::Vector3d velocity = velocity_traj.evaluateDeBoorT(sample_time);
      if (velocity.norm() > velocity_limit)
      {
        RCLCPP_WARN(rclcpp::get_logger("planner_manager"),
                    "Dynamic feasibility failed at t=%.3f: |v|=%.3f > %.3f",
                    sample_time, velocity.norm(), velocity_limit);
        return false;
      }

      const Eigen::Vector3d acceleration =
          acceleration_traj.evaluateDeBoorT(sample_time);
      if (acceleration.norm() > acceleration_limit)
      {
        RCLCPP_WARN(rclcpp::get_logger("planner_manager"),
                    "Dynamic feasibility failed at t=%.3f: |a|=%.3f > %.3f",
                    sample_time, acceleration.norm(), acceleration_limit);
        return false;
      }
    }
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), rclcpp::Clock().now());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    // if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
    if (local_data_.start_time_.seconds() < 1e9)
      return false;

    // double my_traj_start_time = local_data_.start_time_.toSec();
    // double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();
    double my_traj_start_time = local_data_.start_time_.seconds();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.seconds();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    if (waypoints.empty())
      return false;

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = rclcpp::Clock().now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    /*挨个读取点并计算点距判断是否需要插点，随后计算插点并写入矩阵，最后根据插点数量生成全局轨迹
      最终返回值为是否规划成功的布尔值 */
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = rclcpp::Clock().now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
//这段代码主要实现了EGOPlannerManager类的成员函数，包括初始化规划模块、重新规划、紧急停止、碰撞检测、全局轨迹规划、轨迹优化等功能。
//通过调用BsplineOptimizer类中的方法来优化B样条轨迹，并使用PlanningVisualization类进行可视化展示。
