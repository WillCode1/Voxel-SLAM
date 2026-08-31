#include "voxelslam.hpp"

using namespace std;

class Initialization
{
public:
  static Initialization &instance()
  {
    static Initialization inst;
    return inst;
  }

  void align_gravity(vector<IMUST> &xs)
  {
    Eigen::Vector3d g0 = xs[0].g;
    Eigen::Vector3d n0 = g0 / g0.norm();
    Eigen::Vector3d n1(0, 0, 1);
    if (n0[2] < 0)
      n1[2] = -1;

    Eigen::Vector3d rotvec = n0.cross(n1);
    double rnorm = rotvec.norm();
    rotvec = rotvec / rnorm;

    Eigen::AngleAxisd angaxis(asin(rnorm), rotvec);
    Eigen::Matrix3d rot = angaxis.matrix();
    g0 = rot * g0;

    Eigen::Vector3d p0 = xs[0].p;
    for (int i = 0; i < xs.size(); i++)
    {
      xs[i].p = rot * (xs[i].p - p0) + p0;
      xs[i].R = rot * xs[i].R;
      xs[i].v = rot * xs[i].v;
      xs[i].g = g0;
    }
  }

#ifdef ROS1
  void motion_blur(pcl::PointCloud<PointType> &pl, PVec &pvec, IMUST xc, IMUST xl, deque<sensor_msgs::Imu::Ptr> &imus, double pcl_beg_time, IMUST &extrin_para)
#else
  void motion_blur(pcl::PointCloud<PointType> &pl, PVec &pvec, IMUST xc, IMUST xl, deque<sensor_msgs::msg::Imu::SharedPtr> &imus, double pcl_beg_time, IMUST &extrin_para)
#endif
  {
    xc.bg = xl.bg;
    xc.ba = xl.ba;
    Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(xc.v), pos_imu(xc.p);
    Eigen::Matrix3d R_imu(xc.R);
    vector<IMUST> imu_poses;

    for (auto it_imu = imus.end() - 1; it_imu != imus.begin(); it_imu--)
    {
#ifdef ROS1
      sensor_msgs::Imu &head = **(it_imu - 1);
      sensor_msgs::Imu &tail = **(it_imu);
#else
      sensor_msgs::msg::Imu &head = **(it_imu - 1);
      sensor_msgs::msg::Imu &tail = **(it_imu);
#endif

      angvel_avr << 0.5 * (head.angular_velocity.x + tail.angular_velocity.x),
          0.5 * (head.angular_velocity.y + tail.angular_velocity.y),
          0.5 * (head.angular_velocity.z + tail.angular_velocity.z);
      acc_avr << 0.5 * (head.linear_acceleration.x + tail.linear_acceleration.x),
          0.5 * (head.linear_acceleration.y + tail.linear_acceleration.y),
          0.5 * (head.linear_acceleration.z + tail.linear_acceleration.z);

      angvel_avr -= xc.bg;
      acc_avr = acc_avr * imupre_scale_gravity - xc.ba;

#ifdef ROS1
      double dt = head.header.stamp.toSec() - tail.header.stamp.toSec();
#else
      double dt = to_seconds(head.header.stamp) - to_seconds(tail.header.stamp);
#endif
      // Eigen::Matrix3d acc_avr_skew = hat(acc_avr);
      Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);

      acc_imu = R_imu * acc_avr + xc.g;
      pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
      vel_imu = vel_imu + acc_imu * dt;
      R_imu = R_imu * Exp_f;

#ifdef ROS1
      double offt = head.header.stamp.toSec() - pcl_beg_time;
#else
      double offt = to_seconds(head.header.stamp) - pcl_beg_time;
#endif
      imu_poses.emplace_back(offt, R_imu, pos_imu, vel_imu, angvel_avr, acc_imu);
    }

    pointVar pv;
    pv.var.setIdentity();
    if (point_notime)
    {
      for (PointType &ap : pl.points)
      {
        pv.pnt << ap.x, ap.y, ap.z;
        pv.pnt = extrin_para.R * pv.pnt + extrin_para.p;
        pvec.push_back(pv);
      }
      return;
    }
    auto it_pcl = pl.end() - 1;
    // for(auto it_kp=imu_poses.end(); it_kp!=imu_poses.begin(); it_kp--)
    for (auto it_kp = imu_poses.begin(); it_kp != imu_poses.end(); it_kp++)
    {
      // IMUST &head = *(it_kp - 1);
      IMUST &head = *it_kp;
      R_imu = head.R;
      acc_imu = head.ba;
      vel_imu = head.v;
      pos_imu = head.p;
      angvel_avr = head.bg;

      for (; it_pcl->curvature > head.t; it_pcl--)
      {
        double dt = it_pcl->curvature - head.t;
        Eigen::Matrix3d R_i = R_imu * Exp(angvel_avr, dt);
        Eigen::Vector3d T_ei = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - xc.p;

        Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        Eigen::Vector3d P_compensate = xc.R.transpose() * (R_i * (extrin_para.R * P_i + extrin_para.p) + T_ei);

        pv.pnt = P_compensate;
        pvec.push_back(pv);
        if (it_pcl == pl.begin())
          break;
      }
    }
  }

#ifdef ROS1
  int motion_init(vector<pcl::PointCloud<PointType>::Ptr> &pl_origs, vector<deque<sensor_msgs::Imu::Ptr>> &vec_imus, vector<double> &beg_times, Eigen::MatrixXd *hess, LidarFactor &voxhess, vector<IMUST> &x_buf, unordered_map<VOXEL_LOC, OctoTree *> &surf_map, unordered_map<VOXEL_LOC, OctoTree *> &surf_map_slide, vector<PVecPtr> &pvec_buf, int win_size, vector<vector<SlideWindow *>> &sws, IMUST &x_curr, deque<IMU_PRE *> &imu_pre_buf, IMUST &extrin_para, double degrade_eigval)
#else
  int motion_init(vector<pcl::PointCloud<PointType>::Ptr> &pl_origs, vector<deque<sensor_msgs::msg::Imu::SharedPtr>> &vec_imus, vector<double> &beg_times, Eigen::MatrixXd *hess, LidarFactor &voxhess, vector<IMUST> &x_buf, unordered_map<VOXEL_LOC, OctoTree *> &surf_map, unordered_map<VOXEL_LOC, OctoTree *> &surf_map_slide, vector<PVecPtr> &pvec_buf, int win_size, vector<vector<SlideWindow *>> &sws, IMUST &x_curr, deque<IMU_PRE *> &imu_pre_buf, IMUST &extrin_para, double degrade_eigval)
#endif
  {
    PLV(3) pwld;
    // double last_g_norm = x_buf[0].g.norm();
    int converge_flag = 0;

    double min_eigen_value_orig = min_eigen_value;
    vector<double> eigen_value_array_orig = plane_eigen_value_thre;

    min_eigen_value = 0.02;
    for (double &iter : plane_eigen_value_thre)
      iter = 1.0 / 4;

#ifdef ROS1
    double t0 = ros::Time::now().toSec();
#else
    double t0 = rclcpp::Clock().now().seconds();
#endif
    double converge_thre = 0.05;
    // int converge_times = 0;
    bool is_degrade = true;
    Eigen::Vector3d eigvalue;
    eigvalue.setZero();
    for (int iterCnt = 0; iterCnt < 10; iterCnt++)
    {
      if (converge_flag == 1)
      {
        min_eigen_value = min_eigen_value_orig;
        plane_eigen_value_thre = eigen_value_array_orig;
      }

      vector<OctoTree *> octos;
      for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
      {
        iter->second->tras_ptr(octos);
        iter->second->clear_slwd(sws[0]);
        delete iter->second;
      }
      for (int i = 0; i < octos.size(); i++)
        delete octos[i];
      surf_map.clear();
      octos.clear();
      surf_map_slide.clear();

      for (int i = 0; i < win_size; i++)
      {
        pwld.clear();
        pvec_buf[i]->clear();
        int l = i == 0 ? i : i - 1;
        motion_blur(*pl_origs[i], *pvec_buf[i], x_buf[i], x_buf[l], vec_imus[i], beg_times[i], extrin_para);

        if (converge_flag == 1)
        {
          for (pointVar &pv : *pvec_buf[i])
            calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);
          pvec_update(pvec_buf[i], x_buf[i], pwld);
        }
        else
        {
          for (pointVar &pv : *pvec_buf[i])
            pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
        }

        cut_voxel(surf_map, pvec_buf[i], i, surf_map_slide, win_size, pwld, sws[0]);
      }

      // LidarFactor voxhess(win_size);
      voxhess.clear();
      voxhess.win_size = win_size;
      for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
      {
        iter->second->recut(win_size, x_buf, sws[0]);
        iter->second->tras_opt(voxhess);
      }

      if (voxhess.plvec_voxels.size() < 10)
        break;
      LI_BA_OptimizerGravity opt_lsv;
      vector<double> resis;
      opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, hess, 3);
      Eigen::Matrix3d nnt;
      nnt.setZero();

      printf("%d: %lf %lf %lf: %lf %lf\n", iterCnt, x_buf[0].g[0], x_buf[0].g[1], x_buf[0].g[2], x_buf[0].g.norm(), fabs(resis[0] - resis[1]) / resis[0]);

      for (int i = 0; i < win_size - 1; i++)
        delete imu_pre_buf[i];
      imu_pre_buf.clear();

      for (int i = 1; i < win_size; i++)
      {
        imu_pre_buf.push_back(new IMU_PRE(x_buf[i - 1].bg, x_buf[i - 1].ba));
        imu_pre_buf.back()->push_imu(vec_imus[i]);
      }

      if (fabs(resis[0] - resis[1]) / resis[0] < converge_thre && iterCnt >= 2)
      {
        for (Eigen::Matrix3d &iter : voxhess.eig_vectors)
        {
          Eigen::Vector3d v3 = iter.col(0);
          nnt += v3 * v3.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
        eigvalue = saes.eigenvalues();
        is_degrade = eigvalue[0] < degrade_eigval ? true : false;

        converge_thre = 0.01;
        if (converge_flag == 0)
        {
          align_gravity(x_buf);
          converge_flag = 1;
          continue;
        }
        else
          break;
      }
    }

    x_curr = x_buf[win_size - 1];
    double gnm = x_curr.g.norm();
    if (is_degrade || gnm < 9.6 || gnm > 10.0)
    {
      converge_flag = 0;
    }
    if (converge_flag == 0)
    {
      vector<OctoTree *> octos;
      for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
      {
        iter->second->tras_ptr(octos);
        iter->second->clear_slwd(sws[0]);
        delete iter->second;
      }
      for (int i = 0; i < octos.size(); i++)
        delete octos[i];
      surf_map.clear();
      octos.clear();
      surf_map_slide.clear();
    }

    printf("mn: %lf %lf %lf\n", eigvalue[0], eigvalue[1], eigvalue[2]);
    // Eigen::Vector3d angv(vec_imus[0][0]->angular_velocity.x, vec_imus[0][0]->angular_velocity.y, vec_imus[0][0]->angular_velocity.z);
    // Eigen::Vector3d acc(vec_imus[0][0]->linear_acceleration.x, vec_imus[0][0]->linear_acceleration.y, vec_imus[0][0]->linear_acceleration.z);
    // acc *= 9.8;

    pl_origs.clear();
    vec_imus.clear();
    beg_times.clear();
#ifdef ROS1
    double t1 = ros::Time::now().toSec();
#else
    double t1 = rclcpp::Clock().now().seconds();
#endif
    printf("init time: %lf\n", t1 - t0);

    // align_gravity(x_buf);
    pcl::PointCloud<PointType> pcl_send;
    PointType pt;
    for (int i = 0; i < win_size; i++)
      for (pointVar &pv : *pvec_buf[i])
      {
        Eigen::Vector3d vv = x_buf[i].R * pv.pnt + x_buf[i].p;
        pt.x = vv[0];
        pt.y = vv[1];
        pt.z = vv[2];
        pcl_send.push_back(pt);
      }
    pub_pl_func(pcl_send, pub_init);

    return converge_flag;
  }
};

class VOXEL_SLAM
{
public:
  pcl::PointCloud<PointType> pcl_path;
  IMUST x_curr, extrin_para;
  IMUEKF odom_ekf;
  unordered_map<VOXEL_LOC, OctoTree *> surf_map, surf_map_slide;
  double down_size;

  int win_size;
  vector<IMUST> x_buf;
  vector<PVecPtr> pvec_buf;
  deque<IMU_PRE *> imu_pre_buf;
  int win_count = 0, win_base = 0;
  vector<vector<SlideWindow *>> sws;

  vector<ScanPose *> *scanPoses;
  mutex mtx_loop;
  deque<ScanPose *> buf_lba2loop, buf_lba2loop_tem;
  vector<Keyframe *> *keyframes;
  int loop_detect = 0;
  unordered_map<VOXEL_LOC, OctoTree *> map_loop;
  IMUST dx;
  pcl::PointCloud<PointType>::Ptr pl_kdmap;
  pcl::KdTreeFLANN<PointType> kd_keyframes;
  int history_kfsize = 0;
  vector<OctoTree *> octos_release;
  int reset_flag = 0;
  int g_update = 0; // gravity update flag: 0 = not update, 1 = update in motion_init, 2 = update after loop closure
  int thread_num = 5;
  double degrade_eigval = 14;
  int degrade_bound = 10;

  vector<vector<ScanPose *> *> multimap_scanPoses;
  vector<vector<Keyframe *> *> multimap_keyframes;
  volatile int gba_flag = 0;
  int gba_size = 0;
  vector<int> cnct_map;
  mutex mtx_keyframe;
  PGO_Edges gba_edges1, gba_edges2;
  bool is_finish = false;

  vector<string> sessionNames;
  string bagname, savepath;
  int is_save_map;
  bool motion_init_en = false;
  bool gravity_align_en = true;
  Eigen::Vector3d preset_gravity;

#ifdef ROS1
  VOXEL_SLAM(ros::NodeHandle &n)
#else
  VOXEL_SLAM(rclcpp::Node::SharedPtr &node)
#endif
  {
    double cov_gyr, cov_acc, rand_walk_gyr, rand_walk_acc;
    vector<double> vecR(9), vecT(3), gravity_init(3);
    scanPoses = new vector<ScanPose *>();
    keyframes = new vector<Keyframe *>();

    string lid_topic, imu_topic;
#ifdef ROS1
    n.param<string>("General/lid_topic", lid_topic, "/livox/lidar");
    n.param<string>("General/imu_topic", imu_topic, "/livox/imu");
    n.param<string>("General/bagname", bagname, "site3_handheld_4");
    n.param<string>("General/save_path", savepath, "");
    n.param<int>("General/lidar_type", feat.lidar_type, 0);
    n.param<double>("General/blind", feat.blind, 0.1);
    n.param<int>("General/point_filter_num", feat.point_filter_num, 3);
    n.param<vector<double>>("General/extrinsic_tran", vecT, vector<double>());
    n.param<vector<double>>("General/extrinsic_rota", vecR, vector<double>());
    n.param<bool>("General/motion_init_en", motion_init_en, false);
    n.param<bool>("General/gravity_align_en", gravity_align_en, true);
    n.param<vector<double>>("General/gravity_init", gravity_init, vector<double>());
    n.param<int>("General/is_save_map", is_save_map, 0);

    sub_imu = n.subscribe(imu_topic, 80000, imu_handler);
    if (feat.lidar_type == LIVOX)
      sub_pcl = n.subscribe<livox_ros_driver::CustomMsg>(lid_topic, 1000, pcl_handler);
    else
      sub_pcl = n.subscribe<sensor_msgs::PointCloud2>(lid_topic, 1000, pcl_handler);

    n.param<double>("Odometry/cov_gyr", cov_gyr, 0.1);
    n.param<double>("Odometry/cov_acc", cov_acc, 0.1);
    n.param<double>("Odometry/rdw_gyr", rand_walk_gyr, 1e-4);
    n.param<double>("Odometry/rdw_acc", rand_walk_acc, 1e-4);
    n.param<double>("Odometry/down_size", down_size, 0.1);
    n.param<double>("Odometry/dept_err", dept_err, 0.02);
    n.param<double>("Odometry/beam_err", beam_err, 0.05);
    n.param<double>("Odometry/voxel_size", voxel_size, 1);
    n.param<double>("Odometry/min_eigen_value", min_eigen_value, 0.0025);
    n.param<double>("Odometry/degrade_eigval", degrade_eigval, 14);
    n.param<int>("Odometry/degrade_bound", degrade_bound, 10);
    n.param<int>("Odometry/point_notime", point_notime, 0);
#else
    node->declare_parameter("lid_topic", "/livox/lidar");
    node->declare_parameter("imu_topic", "/livox/imu");
    node->declare_parameter("bagname", "site3_handheld_4");
    node->declare_parameter("save_path", "");
    node->declare_parameter("lidar_type", 0);
    node->declare_parameter("blind", 0.1);
    node->declare_parameter("point_filter_num", 3);
    node->declare_parameter("extrinsic_tran", vector<double>());
    node->declare_parameter("extrinsic_rota", vector<double>());
    node->declare_parameter("motion_init_en", false);
    node->declare_parameter("gravity_align_en", true);
    node->declare_parameter("gravity_init", vector<double>());
    node->declare_parameter("is_save_map", 0);

    node->get_parameter("lid_topic", lid_topic);
    node->get_parameter("imu_topic", imu_topic);
    node->get_parameter("bagname", bagname);
    node->get_parameter("save_path", savepath);
    node->get_parameter("lidar_type", feat.lidar_type);
    node->get_parameter("blind", feat.blind);
    node->get_parameter("point_filter_num", feat.point_filter_num);
    node->get_parameter("extrinsic_tran", vecT);
    node->get_parameter("extrinsic_rota", vecR);
    node->get_parameter("motion_init_en", motion_init_en);
    node->get_parameter("gravity_align_en", gravity_align_en);
    node->get_parameter("gravity_init", gravity_init);
    node->get_parameter("is_save_map", is_save_map);

    auto qos = rclcpp::SensorDataQoS();

    sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200000, imu_handler);
    if (feat.lidar_type == LIVOX)
      sub_pcl1 = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, qos, livox_pcl_cbk);
    else
      sub_pcl2 = node->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, qos, standard_pcl_cbk);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*node);

    node->declare_parameter("odom_cov_gyr", 0.1);
    node->declare_parameter("odom_cov_acc", 0.1);
    node->declare_parameter("odom_rdw_gyr", 1e-4);
    node->declare_parameter("odom_rdw_acc", 1e-4);
    node->declare_parameter("odom_down_size", 0.1);
    node->declare_parameter("odom_dept_err", 0.02);
    node->declare_parameter("odom_beam_err", 0.05);
    node->declare_parameter("odom_voxel_size", 1.0);
    node->declare_parameter("odom_min_eigen_value", 0.0025);
    node->declare_parameter("odom_degrade_eigval", 14.);
    node->declare_parameter("odom_degrade_bound", 10);
    node->declare_parameter("odom_point_notime", 0);

    node->get_parameter("odom_cov_gyr", cov_gyr);
    node->get_parameter("odom_cov_acc", cov_acc);
    node->get_parameter("odom_rdw_gyr", rand_walk_gyr);
    node->get_parameter("odom_rdw_acc", rand_walk_acc);
    node->get_parameter("odom_down_size", down_size);
    node->get_parameter("odom_dept_err", dept_err);
    node->get_parameter("odom_beam_err", beam_err);
    node->get_parameter("odom_voxel_size", voxel_size);
    node->get_parameter("odom_min_eigen_value", min_eigen_value);
    node->get_parameter("odom_degrade_eigval", degrade_eigval);
    node->get_parameter("odom_degrade_bound", degrade_bound);
    node->get_parameter("odom_point_notime", point_notime);
#endif
    odom_ekf.imu_topic = imu_topic;
    odom_ekf.point_notime = point_notime;

    feat.blind = feat.blind * feat.blind;
    odom_ekf.cov_gyr << cov_gyr, cov_gyr, cov_gyr;
    odom_ekf.cov_acc << cov_acc, cov_acc, cov_acc;
    odom_ekf.cov_bias_gyr << rand_walk_gyr, rand_walk_gyr, rand_walk_gyr;
    odom_ekf.cov_bias_acc << rand_walk_acc, rand_walk_acc, rand_walk_acc;
    odom_ekf.Lid_offset_to_IMU << vecT[0], vecT[1], vecT[2];
    odom_ekf.Lid_rot_to_IMU << vecR[0], vecR[1], vecR[2],
        vecR[3], vecR[4], vecR[5],
        vecR[6], vecR[7], vecR[8];
    preset_gravity << gravity_init[0], gravity_init[1], gravity_init[2];
    extrin_para.R = odom_ekf.Lid_rot_to_IMU;
    extrin_para.p = odom_ekf.Lid_offset_to_IMU;
    min_point << 5, 5, 5, 5;

#ifdef ROS1
    n.param<int>("LocalBA/win_size", win_size, 10);
    n.param<int>("LocalBA/max_layer", max_layer, 2);
    n.param<double>("LocalBA/cov_gyr", cov_gyr, 0.1);
    n.param<double>("LocalBA/cov_acc", cov_acc, 0.1);
    n.param<double>("LocalBA/rdw_gyr", rand_walk_gyr, 1e-4);
    n.param<double>("LocalBA/rdw_acc", rand_walk_acc, 1e-4);
    n.param<int>("LocalBA/min_ba_point", min_ba_point, 20);
    n.param<vector<double>>("LocalBA/plane_eigen_value_thre", plane_eigen_value_thre, vector<double>({1, 1, 1, 1}));
    n.param<double>("LocalBA/imu_coef", imu_coef, 1e-4);
    n.param<int>("LocalBA/thread_num", thread_num, 5);
#else
    node->declare_parameter("localba_win_size", 10);
    node->declare_parameter("localba_max_layer", 2);
    node->declare_parameter("localba_cov_gyr", 0.1);
    node->declare_parameter("localba_cov_acc", 0.1);
    node->declare_parameter("localba_rdw_gyr", 1e-4);
    node->declare_parameter("localba_rdw_acc", 1e-4);
    node->declare_parameter("localba_min_ba_point", 20);
    node->declare_parameter("localba_plane_eigen_value_thre", vector<double>({1, 1, 1, 1}));
    node->declare_parameter("localba_imu_coef", 1e-4);
    node->declare_parameter("localba_thread_num", 5);

    node->get_parameter("localba_win_size", win_size);
    node->get_parameter("localba_max_layer", max_layer);
    node->get_parameter("localba_cov_gyr", cov_gyr);
    node->get_parameter("localba_cov_acc", cov_acc);
    node->get_parameter("localba_rdw_gyr", rand_walk_gyr);
    node->get_parameter("localba_rdw_acc", rand_walk_acc);
    node->get_parameter("localba_min_ba_point", min_ba_point);
    node->get_parameter("localba_plane_eigen_value_thre", plane_eigen_value_thre);
    node->get_parameter("localba_imu_coef", imu_coef);
    node->get_parameter("localba_thread_num", thread_num);
#endif

    for (double &iter : plane_eigen_value_thre)
      iter = 1.0 / iter;
    // for(double &iter: plane_eigen_value_thre) iter = 1.0 / iter;

    noiseMeas.setZero();
    noiseWalk.setZero();
    noiseMeas.diagonal() << cov_gyr, cov_gyr, cov_gyr,
        cov_acc, cov_acc, cov_acc;
    noiseWalk.diagonal() << rand_walk_gyr, rand_walk_gyr, rand_walk_gyr,
        rand_walk_acc, rand_walk_acc, rand_walk_acc;

    int ss = 0;
    if (access((savepath + bagname + "/").c_str(), X_OK) == -1)
    {
      string cmd = "mkdir " + savepath + bagname + "/";
      ss = system(cmd.c_str());
    }
    else
      ss = -1;

    if (ss != 0 && is_save_map == 1)
    {
      printf("The pointcloud will be saved in this run.\n");
      printf("So please clear or rename the existed folder.\n");
      exit(0);
    }

    sws.resize(thread_num);
    cout << "bagname: " << bagname << endl;
  }

  // The point-to-plane alignment for odometry
  bool lio_state_estimation(PVecPtr pptr)
  {
    IMUST x_prop = x_curr;

    const int num_max_iter = 4;
    bool EKF_stop_flg = 0, flg_EKF_converged = 0;
    Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();
    int rematch_num = 0;
    int match_num = 0;

    int psize = pptr->size();
    vector<OctoTree *> octos;
    octos.resize(psize, nullptr);

    Eigen::Matrix3d nnt;
    Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();
    for (int iterCount = 0; iterCount < num_max_iter; iterCount++)
    {
      Eigen::Matrix<double, 6, 6> HTH;
      HTH.setZero();
      Eigen::Matrix<double, 6, 1> HTz;
      HTz.setZero();
      Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
      Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);
      match_num = 0;
      nnt.setZero();

      for (int i = 0; i < psize; i++)
      {
        pointVar &pv = pptr->at(i);
        Eigen::Matrix3d phat = hat(pv.pnt);
        // Eigen::Matrix3d var_world = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
        Eigen::Matrix3d var_world = x_curr.R * pv.var * x_curr.R.transpose() + x_curr.R * phat * rot_var * phat.transpose() * x_curr.R.transpose() + tsl_var;
        Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

        double sigma_d = 0;
        Plane *pla = nullptr;
        int flag = 0;
        if (octos[i] != nullptr && octos[i]->inside(wld))
        {
          double max_prob = 0;
          flag = octos[i]->match(wld, pla, max_prob, var_world, sigma_d, octos[i]);
        }
        else
        {
          flag = match(surf_map, wld, pla, var_world, sigma_d, octos[i]);
        }

        if (flag)
        {
          Plane &pp = *pla;
          double R_inv = 1.0 / (0.0005 + sigma_d);
          double resi = pp.normal.dot(wld - pp.center);

          Eigen::Matrix<double, 6, 1> jac;
          jac.head(3) = phat * x_curr.R.transpose() * pp.normal;
          jac.tail(3) = pp.normal;
          HTH += R_inv * jac * jac.transpose();
          HTz -= R_inv * jac * resi;
          nnt += pp.normal * pp.normal.transpose();
          match_num++;
        }
      }

      H_T_H.block<6, 6>(0, 0) = HTH;
      Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv).inverse();
      G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;
      Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
      Eigen::Matrix<double, DIM, 1> solution = K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

      x_curr += solution;
      Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
      Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

      EKF_stop_flg = false;
      flg_EKF_converged = false;

      if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015))
        flg_EKF_converged = true;

      if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == num_max_iter - 2)))
      {
        rematch_num++;
      }

      if (rematch_num >= 2 || (iterCount == num_max_iter - 1))
      {
        x_curr.cov = (I_STATE - G) * x_curr.cov;
        EKF_stop_flg = true;
      }

      if (EKF_stop_flg)
        break;
    }

    // Degeneration detection: eigenvalue threshold = 14
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
    Eigen::Vector3d evalue = saes.eigenvalues();
    // printf("eva %d: %lf\n", match_num, evalue[0]);

    if (evalue[0] < degrade_eigval)
      return false;
    else
      return true;
  }

  // The point-to-plane alignment for initialization
  pcl::PointCloud<PointType>::Ptr pl_tree;
  void lio_state_estimation_kdtree(PVecPtr pptr)
  {
    static pcl::KdTreeFLANN<PointType> kd_map;
    if (pl_tree->size() < 100)
    {
      for (pointVar pv : *pptr)
      {
        PointType pp;
        pv.pnt = x_curr.R * pv.pnt + x_curr.p;
        pp.x = pv.pnt[0];
        pp.y = pv.pnt[1];
        pp.z = pv.pnt[2];
        pl_tree->push_back(pp);
      }
      kd_map.setInputCloud(pl_tree);
      return;
    }

    const int num_max_iter = 4;
    IMUST x_prop = x_curr;
    int psize = pptr->size();
    bool EKF_stop_flg = 0, flg_EKF_converged = 0;
    Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    // double max_dis = 2 * 2;
    vector<float> sqdis(NMATCH);
    vector<int> nearInd(NMATCH);
    PLV(3) vecs(NMATCH);
    int rematch_num = 0;
    Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();

    Eigen::Matrix<double, NMATCH, 1> b;
    b.setOnes();
    b *= -1.0f;

    vector<double> ds(psize, -1);
    PLV(3) directs(psize);
    bool refind = true;

    for (int iterCount = 0; iterCount < num_max_iter; iterCount++)
    {
      Eigen::Matrix<double, 6, 6> HTH;
      HTH.setZero();
      Eigen::Matrix<double, 6, 1> HTz;
      HTz.setZero();
      int valid = 0;
      for (int i = 0; i < psize; i++)
      {
        pointVar &pv = pptr->at(i);
        Eigen::Matrix3d phat = hat(pv.pnt);
        Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

        if (refind)
        {
          PointType apx;
          apx.x = wld[0];
          apx.y = wld[1];
          apx.z = wld[2];
          kd_map.nearestKSearch(apx, NMATCH, nearInd, sqdis);

          Eigen::Matrix<double, NMATCH, 3> A;
          for (int i = 0; i < NMATCH; i++)
          {
            PointType &pp = pl_tree->points[nearInd[i]];
            A.row(i) << pp.x, pp.y, pp.z;
          }
          Eigen::Vector3d direct = A.colPivHouseholderQr().solve(b);
          bool check_flag = false;
          for (int i = 0; i < NMATCH; i++)
          {
            if (fabs(direct.dot(A.row(i)) + 1.0) > 0.1)
              check_flag = true;
          }

          if (check_flag)
          {
            ds[i] = -1;
            continue;
          }

          double d = 1.0 / direct.norm();
          // direct *= d;
          ds[i] = d;
          directs[i] = direct * d;
        }

        if (ds[i] >= 0)
        {
          double pd2 = directs[i].dot(wld) + ds[i];
          Eigen::Matrix<double, 6, 1> jac_s;
          jac_s.head(3) = phat * x_curr.R.transpose() * directs[i];
          jac_s.tail(3) = directs[i];

          HTH += jac_s * jac_s.transpose();
          HTz += jac_s * (-pd2);
          valid++;
        }
      }

      H_T_H.block<6, 6>(0, 0) = HTH;
      Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv / 1000).inverse();
      G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;
      Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
      Eigen::Matrix<double, DIM, 1> solution = K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

      x_curr += solution;
      Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
      Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

      refind = false;
      if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015))
      {
        refind = true;
        flg_EKF_converged = true;
        rematch_num++;
      }

      if (iterCount == num_max_iter - 2 && !flg_EKF_converged)
      {
        refind = true;
      }

      if (rematch_num >= 2 || (iterCount == num_max_iter - 1))
      {
        x_curr.cov = (I_STATE - G) * x_curr.cov;
        EKF_stop_flg = true;
      }

      if (EKF_stop_flg)
        break;
    }

    // double tt1 = ros::Time::now().toSec();
    for (pointVar pv : *pptr)
    {
      pv.pnt = x_curr.R * pv.pnt + x_curr.p;
      PointType ap;
      ap.x = pv.pnt[0];
      ap.y = pv.pnt[1];
      ap.z = pv.pnt[2];
      pl_tree->push_back(ap);
    }
    down_sampling_voxel(*pl_tree, 0.5);
    kd_map.setInputCloud(pl_tree);
    // double tt2 = ros::Time::now().toSec();
  }

  // After detecting loop closure, refine current map and states
  void loop_update()
  {
    printf("loop update: %zu\n", sws[0].size());
#ifdef ROS1
    double t1 = ros::Time::now().toSec();
#else
    double t1 = rclcpp::Clock().now().seconds();
#endif
    for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
    {
      // octos_release.push_back(iter->second);
      iter->second->tras_ptr(octos_release);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
      iter->second = nullptr;
    }
    surf_map.clear();
    surf_map_slide.clear();
    surf_map = map_loop;
    map_loop.clear();

    printf("scanPoses: %zu %zu %zu %d %d %zu\n", scanPoses->size(), buf_lba2loop.size(), x_buf.size(), win_base, win_count, sws[0].size());
    int blsize = scanPoses->size();
    PointType ap = pcl_path[0];
    pcl_path.clear();

    for (int i = 0; i < blsize; i++)
    {
      ap.x = scanPoses->at(i)->x.p[0];
      ap.y = scanPoses->at(i)->x.p[1];
      ap.z = scanPoses->at(i)->x.p[2];
      pcl_path.push_back(ap);
    }

    for (ScanPose *bl : buf_lba2loop)
    {
      bl->update(dx);
      ap.x = bl->x.p[0];
      ap.y = bl->x.p[1];
      ap.z = bl->x.p[2];
      pcl_path.push_back(ap);
    }

    for (int i = 0; i < win_count; i++)
    {
      IMUST &x = x_buf[i];
      x.v = dx.R * x.v;
      x.p = dx.R * x.p + dx.p;
      x.R = dx.R * x.R;
      if (g_update == 1)
        x.g = dx.R * x.g;
      // PointType ap;
      ap.x = x.p[0];
      ap.y = x.p[1];
      ap.z = x.p[2];
      pcl_path.push_back(ap);
    }

    pub_pl_func(pcl_path, pub_curr_path);

    x_curr.R = x_buf[win_count - 1].R;
    x_curr.p = x_buf[win_count - 1].p;
    x_curr.v = dx.R * x_curr.v;
    x_curr.g = x_buf[win_count - 1].g;

    for (int i = 0; i < win_size; i++)
      mp[i] = i;

    for (ScanPose *bl : buf_lba2loop)
    {
      IMUST xx = bl->x;
      PVec pvec_tem = *(bl->pvec);
      for (pointVar &pv : pvec_tem)
        pv.pnt = xx.R * pv.pnt + xx.p;
      cut_voxel(surf_map, pvec_tem, win_size, 0);
    }

    PLV(3) pwld;
    for (int i = 0; i < win_count; i++)
    {
      pwld.clear();
      for (pointVar &pv : *pvec_buf[i])
        pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
      cut_voxel(surf_map, pvec_buf[i], i, surf_map_slide, win_size, pwld, sws[0]);
    }

    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
      iter->second->recut(win_count, x_buf, sws[0]);

    if (g_update == 1)
      g_update = 2;
    loop_detect = 0;
#ifdef ROS1
    double t2 = ros::Time::now().toSec();
#else
    double t2 = rclcpp::Clock().now().seconds();
#endif
    printf("loop head: %lf %zu\n", t2 - t1, sws[0].size());
  }

  // load the previous keyframe in the local voxel map
  void keyframe_loading(double jour)
  {
    if (history_kfsize <= 0)
      return;
    // double tt1 = ros::Time::now().toSec();
    PointType ap_curr;
    ap_curr.x = x_curr.p[0];
    ap_curr.y = x_curr.p[1];
    ap_curr.z = x_curr.p[2];
    vector<int> vec_idx;
    vector<float> vec_dis;
    kd_keyframes.radiusSearch(ap_curr, 10, vec_idx, vec_dis);

    for (int id : vec_idx)
    {
      // int ord_kf = pl_kdmap->points[id].curvature;
      if (keyframes->at(id)->exist)
      {
        Keyframe &kf = *(keyframes->at(id));
        IMUST &xx = kf.x0;
        PVec pvec;
        pvec.reserve(kf.plptr->size());

        pointVar pv;
        pv.var.setZero();
        int plsize = kf.plptr->size();
        // for(int j=0; j<plsize; j+=2)
        for (int j = 0; j < plsize; j++)
        {
          PointType ap = kf.plptr->points[j];
          pv.pnt << ap.x, ap.y, ap.z;
          pv.pnt = xx.R * pv.pnt + xx.p;
          pvec.push_back(pv);
        }

        cut_voxel(surf_map, pvec, win_size, jour);
        kf.exist = 0;
        history_kfsize--;
        break;
      }
    }
  }

  template <typename T>
  static Eigen::Matrix<T, 3, 1> RotationMatrix2RPY(const Eigen::Matrix<T, 3, 3> &rotation)
  {
    // return rotation_matrix.eulerAngles(0, 1, 2);

    // fix eigen bug: https://blog.csdn.net/qq_36594547/article/details/119218807
    const Eigen::Matrix<T, 3, 1> &n = rotation.col(0);
    const Eigen::Matrix<T, 3, 1> &o = rotation.col(1);
    const Eigen::Matrix<T, 3, 1> &a = rotation.col(2);

    Eigen::Matrix<T, 3, 1> rpy(3);
    const double &y = atan2(n(1), n(0));
    const double &p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    const double &r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    rpy(0) = r;
    rpy(1) = p;
    rpy(2) = y;
    return rpy;
  }

  template <typename T>
  static Eigen::Matrix<T, 3, 3> RPY2RotationMatrix(const Eigen::Matrix<T, 3, 1> &eulerAngles)
  {
    Eigen::AngleAxis<T> rollAngle(Eigen::AngleAxis<T>(eulerAngles(0), Eigen::Matrix<T, 3, 1>::UnitX()));
    Eigen::AngleAxis<T> pitchAngle(Eigen::AngleAxis<T>(eulerAngles(1), Eigen::Matrix<T, 3, 1>::UnitY()));
    Eigen::AngleAxis<T> yawAngle(Eigen::AngleAxis<T>(eulerAngles(2), Eigen::Matrix<T, 3, 1>::UnitZ()));
    Eigen::Matrix<T, 3, 3> rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    return rotation_matrix;
  }

  /**
   * @brief 将预设重力方向和测量到的重力方向对比，将imu初始姿态对齐到地图
   * @param preset_gravity 预设重力方向，也就是map方向
   * @param meas_gravity 测量到的重力
   * @param rot_init 返回的imu初始姿态
   */
  void get_imu_init_rot(const Eigen::Vector3d &preset_gravity, const Eigen::Vector3d &meas_gravity, Eigen::Matrix3d &rot_init)
  {
    Eigen::Matrix3d hat_grav = hat(-preset_gravity);
    // sin(theta) = |a^b|/(|a|*|b|) = |axb|/(|a|*|b|)
    double align_sin = (hat_grav * meas_gravity).norm() / meas_gravity.norm() / preset_gravity.norm();
    // cos(theta) = a*b/(|a|*|b|)
    double align_cos = preset_gravity.transpose() * meas_gravity;
    align_cos = align_cos / preset_gravity.norm() / meas_gravity.norm();

    Eigen::Matrix3d rot_mat_init;

    if (align_sin < 1e-6)
    {
      if (align_cos > 1e-6)
        rot_mat_init = Eigen::Matrix3d::Identity();
      else
        rot_mat_init = -Eigen::Matrix3d::Identity();
    }
    else
    {
      // 沿着axb方向旋转对应夹角，得到imu初始姿态
      Eigen::Vector3d align_angle = hat_grav * meas_gravity / (hat_grav * meas_gravity).norm() * acos(align_cos);
      rot_mat_init = Exp(align_angle);
    }

    Eigen::Vector3d rpy = RotationMatrix2RPY(rot_mat_init);
    rpy.z() = 0;
    rot_init = RPY2RotationMatrix(rpy);
  }

#ifdef ROS1
  int initialization(deque<sensor_msgs::Imu::Ptr> &imus, Eigen::MatrixXd &hess, LidarFactor &voxhess, PLV(3) & pwld, pcl::PointCloud<PointType>::Ptr pcl_curr)
  {
    static vector<deque<sensor_msgs::Imu::Ptr>> vec_imus;
#else
  int initialization(deque<sensor_msgs::msg::Imu::SharedPtr> &imus, Eigen::MatrixXd &hess, LidarFactor &voxhess, PLV(3) & pwld, pcl::PointCloud<PointType>::Ptr pcl_curr)
  {
    static vector<deque<sensor_msgs::msg::Imu::SharedPtr>> vec_imus;
#endif
    static vector<pcl::PointCloud<PointType>::Ptr> pl_origs;
    static vector<double> beg_times;

    pcl::PointCloud<PointType>::Ptr orig(new pcl::PointCloud<PointType>(*pcl_curr));
    if (odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
      return 0;

    static bool gravity_align = false;
    if (gravity_align_en && !gravity_align)
    {
      // 1.gravity aligns the imu direction
      get_imu_init_rot(preset_gravity, x_curr.g, x_curr.R);
      // 2.fix gravity vec
      x_curr.g = x_curr.R * x_curr.g;
      gravity_align = true;

      auto tmp = RotationMatrix2RPY(x_curr.R);
      printf("gravity_align: align rpy = (%.3f, %.3f, %.3f), the final gravity = (%.3f, %.3f, %.3f)!\n",
             RAD2DEG(tmp.x()), RAD2DEG(tmp.y()), RAD2DEG(tmp.z()), x_curr.g.x(), x_curr.g.y(), x_curr.g.z());
    }

    if (win_count == 0)
      imupre_scale_gravity = odom_ekf.scale_gravity;

    PVecPtr pptr(new PVec);
    double downkd = down_size >= 0.5 ? down_size : 0.5;
    down_sampling_voxel(*pcl_curr, downkd);
    var_init(extrin_para, *pcl_curr, pptr, dept_err, beam_err);
    if (motion_init_en)
      lio_state_estimation_kdtree(pptr);
    else
      lio_state_estimation(pptr);

    pwld.clear();
    pvec_update(pptr, x_curr, pwld);

    win_count++;
    x_buf.push_back(x_curr);
    pvec_buf.push_back(pptr);
    ResultOutput::instance().pub_localtraj(pwld, 0, x_curr, sessionNames.size() - 1, pcl_path);

    if (win_count > 1)
    {
      imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count - 2].bg, x_buf[win_count - 2].ba));
      imu_pre_buf[win_count - 2]->push_imu(imus);
    }

    pcl::PointCloud<PointType> pl_mid = *orig;
    down_sampling_close(*orig, down_size);
    if (orig->size() < 1000)
    {
      *orig = pl_mid;
      down_sampling_close(*orig, down_size / 2);
    }

    sort(orig->begin(), orig->end(), [](PointType &x, PointType &y)
         { return x.curvature < y.curvature; });

    pl_origs.push_back(orig);
    beg_times.push_back(odom_ekf.pcl_beg_time);
    vec_imus.push_back(imus);

    int is_success = 0;
    if (win_count >= win_size)
    {
      if (motion_init_en)
      {
        is_success = Initialization::instance().motion_init(pl_origs, vec_imus, beg_times, &hess, voxhess, x_buf, surf_map, surf_map_slide, pvec_buf, win_size, sws, x_curr, imu_pre_buf, extrin_para, degrade_eigval);

        if (is_success == 0)
          return -1;
      }
      return 1;
    }
    return 0;
  }

#ifdef ROS1
  void system_reset(deque<sensor_msgs::Imu::Ptr> &imus)
#else
  void system_reset(deque<sensor_msgs::msg::Imu::SharedPtr> &imus)
#endif
  {
    for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
    {
      iter->second->tras_ptr(octos_release);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }
    surf_map.clear();
    surf_map_slide.clear();

    x_curr.setZero();
    x_curr.p = Eigen::Vector3d(0, 0, 30);
    odom_ekf.mean_acc.setZero();
    odom_ekf.init_num = 0;
    odom_ekf.IMU_init(imus);
    x_curr.g = -odom_ekf.mean_acc * imupre_scale_gravity;

    for (int i = 0; i < imu_pre_buf.size(); i++)
      delete imu_pre_buf[i];
    x_buf.clear();
    pvec_buf.clear();
    imu_pre_buf.clear();
    pl_tree->clear();

    for (int i = 0; i < win_size; i++)
      mp[i] = i;
    win_base = 0;
    win_count = 0;
    pcl_path.clear();
    pub_pl_func(pcl_path, pub_cmap);
    printf("Reset\n");
  }

  // After local BA, update the map and marginalize the points of oldest scan
  // multi means multiple thread
  void multi_margi(unordered_map<VOXEL_LOC, OctoTree *> &feat_map, double jour, int win_count, vector<IMUST> &xs, LidarFactor &voxopt, vector<SlideWindow *> &sw)
  {
    // for(auto iter=feat_map.begin(); iter!=feat_map.end();)
    // {
    //   iter->second->jour = jour;
    //   iter->second->margi(win_count, 1, xs, voxopt);
    //   if(iter->second->isexist)
    //     iter++;
    //   else
    //   {
    //     iter->second->clear_slwd(sw);
    //     feat_map.erase(iter++);
    //   }
    // }
    // return;

    int thd_num = thread_num;
    vector<vector<OctoTree *> *> octs;
    for (int i = 0; i < thd_num; i++)
      octs.push_back(new vector<OctoTree *>());

    int g_size = feat_map.size();
    if (g_size < thd_num)
      return;
    vector<thread *> mthreads(thd_num);
    double part = 1.0 * g_size / thd_num;
    int cnt = 0;
    for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
    {
      iter->second->jour = jour;
      octs[cnt]->push_back(iter->second);
      if (octs[cnt]->size() >= part && cnt < thd_num - 1)
        cnt++;
    }

    auto margi_func = [](int win_cnt, vector<OctoTree *> *oct, vector<IMUST> xxs, LidarFactor &voxhess)
    {
      for (OctoTree *oc : *oct)
      {
        oc->margi(win_cnt, 1, xxs, voxhess);
      }
    };

    for (int i = 1; i < thd_num; i++)
    {
      mthreads[i] = new thread(margi_func, win_count, octs[i], xs, ref(voxopt));
    }

    for (int i = 0; i < thd_num; i++)
    {
      if (i == 0)
      {
        margi_func(win_count, octs[i], xs, voxopt);
      }
      else
      {
        mthreads[i]->join();
        delete mthreads[i];
      }
    }

    for (auto iter = feat_map.begin(); iter != feat_map.end();)
    {
      if (iter->second->isexist)
        iter++;
      else
      {
        iter->second->clear_slwd(sw);
        feat_map.erase(iter++);
      }
    }

    for (int i = 0; i < thd_num; i++)
      delete octs[i];
  }

  // Determine the plane and recut the voxel map in octo-tree
  void multi_recut(unordered_map<VOXEL_LOC, OctoTree *> &feat_map, int win_count, vector<IMUST> &xs, LidarFactor &voxopt, vector<vector<SlideWindow *>> &sws)
  {
    // for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    // {
    //   iter->second->recut(win_count, xs, sws[0]);
    //   iter->second->tras_opt(voxopt);
    // }

    int thd_num = thread_num;
    vector<vector<OctoTree *>> octss(thd_num);
    int g_size = feat_map.size();
    if (g_size < thd_num)
      return;
    vector<thread *> mthreads(thd_num);
    double part = 1.0 * g_size / thd_num;
    int cnt = 0;
    for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
    {
      octss[cnt].push_back(iter->second);
      if (octss[cnt].size() >= part && cnt < thd_num - 1)
        cnt++;
    }

    auto recut_func = [](int win_count, vector<OctoTree *> &oct, vector<IMUST> xxs, vector<SlideWindow *> &sw)
    {
      for (OctoTree *oc : oct)
        oc->recut(win_count, xxs, sw);
    };

    for (int i = 1; i < thd_num; i++)
    {
      mthreads[i] = new thread(recut_func, win_count, ref(octss[i]), xs, ref(sws[i]));
    }

    for (int i = 0; i < thd_num; i++)
    {
      if (i == 0)
      {
        recut_func(win_count, octss[i], xs, sws[i]);
      }
      else
      {
        mthreads[i]->join();
        delete mthreads[i];
      }
    }

    for (int i = 1; i < sws.size(); i++)
    {
      sws[0].insert(sws[0].end(), sws[i].begin(), sws[i].end());
      sws[i].clear();
    }

    for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
      iter->second->tras_opt(voxopt);
  }

  // The main thread of odometry and local mapping
#ifdef ROS1
  void thd_odometry_localmapping(ros::NodeHandle &n)
#else
  void thd_odometry_localmapping(rclcpp::Node::SharedPtr &node)
#endif
  {
    PLV(3) pwld;
    // double down_sizes[3] = {0.1, 0.2, 0.4};
    Eigen::Vector3d last_pos(0, 0, 0);
    double jour = 0;
    // int counter = 0;

    pcl::PointCloud<PointType>::Ptr pcl_curr(new pcl::PointCloud<PointType>());
    int motion_init_flag = 1;
    pl_tree.reset(new pcl::PointCloud<PointType>());
    vector<pcl::PointCloud<PointType>::Ptr> pl_origs;
    vector<double> beg_times;
#ifdef ROS1
    vector<deque<sensor_msgs::Imu::Ptr>> vec_imus;
#else
    vector<deque<sensor_msgs::msg::Imu::SharedPtr>> vec_imus;
#endif
    bool release_flag = false;
    int degrade_cnt = 0;
    LidarFactor voxhess(win_size);
    const int mgsize = 1;
    Eigen::MatrixXd hess;
#ifdef ROS1
    while (n.ok())
    {
      ros::spinOnce();
#else
    node->declare_parameter("finish", false);
    while (rclcpp::ok())
    {
      rclcpp::spin_some(node);
#endif
      if (loop_detect == 1)
      {
        loop_update();
        last_pos = x_curr.p;
        jour = 0;
      }

#ifdef ROS1
      n.param<bool>("finish", is_finish, false);
#else
      node->get_parameter("finish", is_finish);
#endif
      if (is_finish)
      {
        break;
      }

#ifdef ROS1
      deque<sensor_msgs::Imu::Ptr> imus;
#else
      deque<sensor_msgs::msg::Imu::SharedPtr> imus;
#endif
      if (!sync_packages(pcl_curr, imus, odom_ekf))
      {
        if (octos_release.size() != 0)
        {
          int msize = octos_release.size();
          if (msize > 1000)
            msize = 1000;
          for (int i = 0; i < msize; i++)
          {
            delete octos_release.back();
            octos_release.pop_back();
          }
          malloc_trim(0);
        }
        else if (release_flag)
        {
          release_flag = false;
          vector<OctoTree *> octos;
          for (auto iter = surf_map.begin(); iter != surf_map.end();)
          {
            int dis = jour - iter->second->jour;
            if (dis < 700)
            // if(dis < 200)
            {
              iter++;
            }
            else
            {
              octos.push_back(iter->second);
              iter->second->tras_ptr(octos);
              surf_map.erase(iter++);
            }
          }
          int ocsize = octos.size();
          for (int i = 0; i < ocsize; i++)
            delete octos[i];
          octos.clear();
          malloc_trim(0);
        }
        else if (sws[0].size() > 10000)
        {
          for (int i = 0; i < 500; i++)
          {
            delete sws[0].back();
            sws[0].pop_back();
          }
          malloc_trim(0);
        }

        sleep(0.001);
        continue;
      }

      static int first_flag = 1;
      if (first_flag)
      {
        pcl::PointCloud<PointType> pl;
        pub_pl_func(pl, pub_pmap);
        pub_pl_func(pl, pub_prev_path);
        first_flag = 0;
      }

      // double t0 = ros::Time::now().toSec();
      // double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0, t7 = 0, t8 = 0;

      if (motion_init_flag)
      {
        int init = initialization(imus, hess, voxhess, pwld, pcl_curr);

        if (init == 1)
        {
          motion_init_flag = 0;
        }
        else
        {
          if (init == -1)
            system_reset(imus);
          continue;
        }
      }
      else
      {
        if (odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
          continue;

        pcl::PointCloud<PointType> pl_down = *pcl_curr;
        down_sampling_voxel(pl_down, down_size);

        if (pl_down.size() < 500)
        {
          pl_down = *pcl_curr;
          down_sampling_voxel(pl_down, down_size / 2);
        }

        PVecPtr pptr(new PVec);
        var_init(extrin_para, pl_down, pptr, dept_err, beam_err);

        if (lio_state_estimation(pptr))
        {
          if (degrade_cnt > 0)
            degrade_cnt--;
        }
        else
          degrade_cnt++;

        pwld.clear();
        pvec_update(pptr, x_curr, pwld);
        ResultOutput::instance().pub_localtraj(pwld, jour, x_curr, sessionNames.size() - 1, pcl_path);

        // t1 = ros::Time::now().toSec();

        win_count++;
        x_buf.push_back(x_curr);
        pvec_buf.push_back(pptr);
        if (win_count > 1)
        {
          imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count - 2].bg, x_buf[win_count - 2].ba));
          imu_pre_buf[win_count - 2]->push_imu(imus);
        }

        keyframe_loading(jour);
        voxhess.clear();
        voxhess.win_size = win_size;

        // cut_voxel(surf_map, pvec_buf[win_count-1], win_count-1, surf_map_slide, win_size, pwld, sws[0]);
        cut_voxel_multi(surf_map, pvec_buf[win_count - 1], win_count - 1, surf_map_slide, win_size, pwld, sws);
        // t2 = ros::Time::now().toSec();

        multi_recut(surf_map_slide, win_count, x_buf, voxhess, sws);
        // t3 = ros::Time::now().toSec();

        if (degrade_cnt > degrade_bound)
        {
          degrade_cnt = 0;
          system_reset(imus);

          last_pos = x_curr.p;
          jour = 0;

          mtx_loop.lock();
          buf_lba2loop_tem.swap(buf_lba2loop);
          mtx_loop.unlock();
          reset_flag = 1;

          motion_init_flag = 1;
          history_kfsize = 0;

          continue;
        }
      }

      // Local BA: Sliding Window BA Optimization and Marginalization
      // https://blog.csdn.net/love20102011/article/details/160621052
      if (win_count >= win_size)
      {
        // t4 = ros::Time::now().toSec();

        if (g_update == 2)
        {
          LI_BA_OptimizerGravity opt_lsv;
          vector<double> resis;
          opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, &hess, 5);
          printf("g update: %lf %lf %lf: %lf\n", x_buf[0].g[0], x_buf[0].g[1], x_buf[0].g[2], x_buf[0].g.norm());
          g_update = 0;
          x_curr.g = x_buf[win_count - 1].g;
        }
        else
        {
          LI_BA_Optimizer opt_lsv;
          opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, &hess);
        }

        ScanPose *bl = new ScanPose(x_buf[0], pvec_buf[0]);
        bl->v6 = hess.block<6, 6>(0, DIM).diagonal();
        for (int i = 0; i < 6; i++)
          bl->v6[i] = 1.0 / fabs(bl->v6[i]);
        mtx_loop.lock();
        buf_lba2loop.push_back(bl);
        mtx_loop.unlock();

        x_curr.R = x_buf[win_count - 1].R;
        x_curr.p = x_buf[win_count - 1].p;
        // t5 = ros::Time::now().toSec();

        ResultOutput::instance().pub_localmap(mgsize, sessionNames.size() - 1, pvec_buf, x_buf, pcl_path, win_base, win_count);

        multi_margi(surf_map_slide, jour, win_count, x_buf, voxhess, sws[0]);
        // t6 = ros::Time::now().toSec();

        if ((win_base + win_count) % 10 == 0)
        {
          double spat = (x_curr.p - last_pos).norm();
          if (spat > 0.5)
          {
            jour += spat;
            last_pos = x_curr.p;
            release_flag = true;
          }
        }

        if (is_save_map)
        {
          for (int i = 0; i < mgsize; i++)
            FileReaderWriter::instance().save_pcd(pvec_buf[i], x_buf[i], win_base + i, savepath + bagname);
        }

        for (int i = 0; i < win_size; i++)
        {
          mp[i] += mgsize;
          if (mp[i] >= win_size)
            mp[i] -= win_size;
        }

        for (int i = mgsize; i < win_count; i++)
        {
          x_buf[i - mgsize] = x_buf[i];
          PVecPtr pvec_tem = pvec_buf[i - mgsize];
          pvec_buf[i - mgsize] = pvec_buf[i];
          pvec_buf[i] = pvec_tem;
        }

        for (int i = win_count - mgsize; i < win_count; i++)
        {
          x_buf.pop_back();
          pvec_buf.pop_back();

          delete imu_pre_buf.front();
          imu_pre_buf.pop_front();
        }

        win_base += mgsize;
        win_count -= mgsize;
      }

      // double t_end = ros::Time::now().toSec();
      // double mem = get_memory();
      // printf("%d: %.4lf: %.4lf %.4lf %.4lf %.4lf %.4lf %.2lfGb %.1lf\n", win_base+win_count, t_end-t0, t1-t0, t2-t1, t3-t2, t5-t4, t6-t5, mem, jour);

      // printf("%d: %lf %lf %lf\n", win_base + win_count, x_curr.p[0], x_curr.p[1], x_curr.p[2]);
    }

    vector<OctoTree *> octos;
    for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
    {
      iter->second->tras_ptr(octos);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }

    for (int i = 0; i < octos.size(); i++)
      delete octos[i];
    octos.clear();

    for (int i = 0; i < sws[0].size(); i++)
      delete sws[0][i];
    sws[0].clear();
    malloc_trim(0);
  }

  // Build the pose graph in loop closure
  void build_graph(gtsam::Values &initial, gtsam::NonlinearFactorGraph &graph, int cur_id, PGO_Edges &lp_edges, gtsam::noiseModel::Diagonal::shared_ptr default_noise, vector<int> &ids, vector<int> &stepsizes, int lpedge_enable)
  {
    initial.clear();
    graph = gtsam::NonlinearFactorGraph();
    ids.clear();
    lp_edges.connect(cur_id, ids);

    stepsizes.clear();
    stepsizes.push_back(0);
    for (int i = 0; i < ids.size(); i++)
      stepsizes.push_back(stepsizes.back() + multimap_scanPoses[ids[i]]->size());

    for (int ii = 0; ii < ids.size(); ii++)
    {
      int bsize = stepsizes[ii], id = ids[ii];
      for (int j = bsize; j < stepsizes[ii + 1]; j++)
      {
        IMUST &xc = multimap_scanPoses[id]->at(j - bsize)->x;
        gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
        initial.insert(j, pose3);
        if (j > bsize)
        {
          gtsam::Vector samv6(6);
          samv6 = multimap_scanPoses[ids[ii]]->at(j - 1 - bsize)->v6;
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(samv6);
          add_edge(j - 1, j, multimap_scanPoses[id]->at(j - 1 - bsize)->x, multimap_scanPoses[id]->at(j - bsize)->x, graph, v6_noise);
          // add_edge(j-1, j, multimap_scanPoses[id]->at(j-1-bsize)->x, multimap_scanPoses[id]->at(j-bsize)->x, graph, default_noise);
        }
      }
    }

    if (multimap_scanPoses[ids[0]]->size() != 0)
    {
      int ceil = multimap_scanPoses[ids[0]]->size();
      // if(ceil > 10) ceil = 10;
      ceil = 1;
      for (int i = 0; i < ceil; i++)
      {
        Eigen::Matrix<double, 6, 1> v6_fixd;
        v6_fixd << 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9;
        gtsam::noiseModel::Diagonal::shared_ptr fixd_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_fixd));
        IMUST xf = multimap_scanPoses[ids[0]]->at(i)->x;
        gtsam::Pose3 pose3 = gtsam::Pose3(gtsam::Rot3(xf.R), gtsam::Point3(xf.p));
        graph.addPrior(i, pose3, fixd_noise);
      }
    }

    if (lpedge_enable == 1)
      for (PGO_Edge &edge : lp_edges.edges)
      {
        vector<int> step(2);
        if (edge.is_adapt(ids, step))
        {
          int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
          for (int i = 0; i < edge.rots.size(); i++)
          {
            int id1 = mp[0] + edge.ids1[i];
            int id2 = mp[1] + edge.ids2[i];
            add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, default_noise);
          }
        }
      }
  }

  // The main thread of loop clousre
  // The topDownProcess of HBA is also run here
#ifdef ROS1
  void thd_loop_closure(ros::NodeHandle &n)
#else
  void thd_loop_closure(rclcpp::Node::SharedPtr &node)
#endif
  {
    pl_kdmap.reset(new pcl::PointCloud<PointType>);
    vector<STDescManager *> std_managers;
    PGO_Edges lp_edges;

    double jud_default = 0.45, icp_eigval = 14;
    double ratio_drift = 0.05, loop_search_radius = 5.0;
    int curr_halt = 10, prev_halt = 30;
    int isHighFly = 0;
#ifdef ROS1
    n.param<double>("Loop/jud_default", jud_default, 0.45);
    n.param<double>("Loop/icp_eigval", icp_eigval, 14);
    n.param<double>("Loop/ratio_drift", ratio_drift, 0.05);
    n.param<int>("Loop/curr_halt", curr_halt, 10);
    n.param<int>("Loop/prev_halt", prev_halt, 30);
    n.param<int>("Loop/isHighFly", isHighFly, 0);
    n.param<double>("Loop/search_radius", loop_search_radius, 5.0);
#else
    node->declare_parameter("loop_jud_default", 0.45);
    node->declare_parameter("loop_icp_eigval", 14.0);
    node->declare_parameter("loop_ratio_drift", 0.05);
    node->declare_parameter("loop_curr_halt", 10);
    node->declare_parameter("loop_prev_halt", 30);
    node->declare_parameter("loop_isHighFly", 0);
    node->declare_parameter("loop_search_radius", 5.0);
    node->get_parameter("loop_jud_default", jud_default);
    node->get_parameter("loop_icp_eigval", icp_eigval);
    node->get_parameter("loop_ratio_drift", ratio_drift);
    node->get_parameter("loop_curr_halt", curr_halt);
    node->get_parameter("loop_prev_halt", prev_halt);
    node->get_parameter("loop_isHighFly", isHighFly);
    node->get_parameter("loop_search_radius", loop_search_radius);
#endif
    ConfigSetting config_setting;
    read_parameters(config_setting, isHighFly);

    vector<double> juds;
#ifdef ROS1
    FileReaderWriter::instance().previous_map_names(n, sessionNames, juds);
    FileReaderWriter::instance().pgo_edges_io(lp_edges, sessionNames, 0, savepath, bagname);
    FileReaderWriter::instance().previous_map_read(std_managers, multimap_scanPoses, multimap_keyframes, config_setting, lp_edges, n, sessionNames, juds, savepath, win_size);
#else
    FileReaderWriter::instance().previous_map_names(node, sessionNames, juds);
    FileReaderWriter::instance().pgo_edges_io(lp_edges, sessionNames, 0, savepath, bagname);
    FileReaderWriter::instance().previous_map_read(std_managers, multimap_scanPoses, multimap_keyframes, config_setting, lp_edges, node, sessionNames, juds, savepath, win_size);
#endif

    STDescManager *std_manager = new STDescManager(config_setting);
    sessionNames.push_back(bagname);
    std_managers.push_back(std_manager);
    multimap_scanPoses.push_back(scanPoses);
    multimap_keyframes.push_back(keyframes);
    juds.push_back(jud_default);
    vector<double> jours(std_managers.size(), 0);

    vector<int> relc_counts(std_managers.size(), prev_halt);

    deque<ScanPose *> bl_local;
    Eigen::Matrix<double, 6, 1> v6_init, v6_fixd;
    v6_init << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    v6_fixd << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_init));
    gtsam::noiseModel::Diagonal::shared_ptr fixd_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_fixd));
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;

    vector<int> ids(1, std_managers.size() - 1), stepsizes(2, 0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plbtc(new pcl::PointCloud<pcl::PointXYZI>);
    IMUST x_key;
    int buf_base = 0;

#ifdef ROS1
    while (n.ok())
#else
    while (rclcpp::ok())
#endif
    {
      if (reset_flag == 1)
      {
        reset_flag = 0;
        scanPoses->insert(scanPoses->end(), buf_lba2loop_tem.begin(), buf_lba2loop_tem.end());
        for (ScanPose *bl : buf_lba2loop_tem)
          bl->pvec = nullptr;
        buf_lba2loop_tem.clear();

        keyframes = new vector<Keyframe *>();
        multimap_keyframes.push_back(keyframes);
        scanPoses = new vector<ScanPose *>();
        multimap_scanPoses.push_back(scanPoses);

        bl_local.clear();
        buf_base = 0;
        std_manager->config_setting_.skip_near_num_ = -(std_manager->plane_cloud_vec_.size() + 10);
        std_manager = new STDescManager(config_setting);
        std_managers.push_back(std_manager);
        relc_counts.push_back(prev_halt);
        sessionNames.push_back(bagname + to_string(sessionNames.size()));
        juds.push_back(jud_default);
        jours.push_back(0);

        bagname = sessionNames.back();
        string cmd = "mkdir " + savepath + bagname + "/";
        // int ss = system(cmd.c_str());

        ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids);
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids, pub_pmap);

        initial.clear();
        graph = gtsam::NonlinearFactorGraph();
        ids.clear();
        ids.push_back(std_managers.size() - 1);
        stepsizes.clear();
        stepsizes.push_back(0);
        stepsizes.push_back(0);
      }

      if (is_finish && buf_lba2loop.empty())
      {
        break;
      }

      if (buf_lba2loop.empty() || loop_detect == 1)
      {
        sleep(0.01);
        continue;
      }
      ScanPose *bl_head = nullptr;
      mtx_loop.lock();
      if (!buf_lba2loop.empty())
      {
        bl_head = buf_lba2loop.front();
        buf_lba2loop.pop_front();
      }
      mtx_loop.unlock();
      if (bl_head == nullptr)
        continue;

      int cur_id = std_managers.size() - 1;
      scanPoses->push_back(bl_head);
      bl_local.push_back(bl_head);
      IMUST xc = bl_head->x;
      gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
      int g_pos = stepsizes.back();
      initial.insert(g_pos, pose3);

      if (g_pos > 0)
      {
        gtsam::Vector samv6(scanPoses->at(buf_base - 1)->v6);
        gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(samv6);
        add_edge(g_pos - 1, g_pos, scanPoses->at(buf_base - 1)->x, xc, graph, v6_noise);
      }
      else
      {
        gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
        graph.addPrior(0, pose3, fixd_noise);
      }

      if (buf_base == 0)
        x_key = xc;
      buf_base++;
      stepsizes.back() += 1;

      if (bl_local.size() < win_size)
        continue;
      double ang = Log(x_key.R.transpose() * xc.R).norm() * 57.3;
      double len = (xc.p - x_key.p).norm();
      if (ang < 5 && len < 0.1 && buf_base > win_size)
      {
        bl_local.front()->pvec = nullptr;
        bl_local.pop_front();
        continue;
      }
      for (double &jour : jours)
        jour += len;
      x_key = xc;

      // Keyframe Aggregation
      PVecPtr pptr(new PVec);
      for (int i = 0; i < win_size; i++)
      {
        ScanPose &bl = *bl_local[i];
        Eigen::Vector3d delta_p = xc.R.transpose() * (bl.x.p - xc.p);
        Eigen::Matrix3d delta_R = xc.R.transpose() * bl.x.R;
        for (pointVar pv : *(bl.pvec))
        {
          pv.pnt = delta_R * pv.pnt + delta_p;
          pptr->push_back(pv);
        }
      }
      for (int i = 0; i < win_size; i++)
      {
        bl_local.front()->pvec = nullptr;
        bl_local.pop_front();
      }

      Keyframe *smp = new Keyframe(xc);
      smp->id = buf_base - 1;
      smp->jour = jours[cur_id];
      down_sampling_pvec(*pptr, voxel_size / 10, *(smp->plptr));

      plbtc->clear();
      pcl::PointXYZI ap;
      for (pointVar &pv : *pptr)
      {
        Eigen::Vector3d &wld = pv.pnt;
        ap.x = wld[0];
        ap.y = wld[1];
        ap.z = wld[2];
        plbtc->push_back(ap);
      }
      mtx_keyframe.lock();
      keyframes->push_back(smp);
      mtx_keyframe.unlock();

      vector<STD> stds_vec;
      std_manager->GenerateSTDescs(plbtc, stds_vec, buf_base - 1);
      pair<int, double> search_result(-1, 0);
      pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
      vector<pair<STD, STD>> loop_std_pair;

      bool isGraph = false, isOpt = false;
      int match_num = 0;
      for (int id = 0; id <= cur_id; id++)
      {
        std_managers[id]->SearchLoop(stds_vec, search_result, loop_transform, loop_std_pair, std_manager->plane_cloud_vec_.back());

        if (search_result.first >= 0)
        {
          printf("Find Loop in session%d: %d %d\n", id, buf_base, search_result.first);
          printf("score: %lf\n", search_result.second);
        }

        if (search_result.first >= 0 && search_result.second > juds[id])
        {
          if (icp_normal(*(std_manager->plane_cloud_vec_.back()), *(std_managers[id]->plane_cloud_vec_[search_result.first]), loop_transform, icp_eigval))
          {
            int ord_bl = std_managers[id]->plane_cloud_vec_[search_result.first]->header.seq;

            IMUST &xx = multimap_scanPoses[id]->at(ord_bl)->x;
            double drift_p = (xx.R * loop_transform.first + xx.p - xc.p).norm();
            double dis_p = (xx.p - xc.p).norm();

            bool isPush = false;
            int step = -1;
            // same session
            if (id == cur_id)
            {
              double span = smp->jour - keyframes->at(search_result.first)->jour;
              printf("drift: %lf %lf %lf\n", drift_p, dis_p, span);

              if (drift_p / span < ratio_drift && dis_p < loop_search_radius)
              {
                isPush = true;
                step = stepsizes.size() - 2;

                if (relc_counts[id] > curr_halt && drift_p > 0.10)
                {
                  isOpt = true;
                  printf("\033[32;43misOpt!\033[0m\n");
                  for (int &cnt : relc_counts)
                    cnt = 0;
                }
              }
            }
            else
            {
              for (int i = 0; i < ids.size(); i++)
                if (id == ids[i])
                  step = i;

              printf("drift: %lf %lf\n", drift_p, jours[id]);

              if (step == -1)
              {
                isGraph = true;
                isOpt = true;
                relc_counts[id] = 0;
                g_update = 1;
                isPush = true;
                jours[id] = 0;
              }
              else
              {
                if (drift_p / jours[id] < 0.05)
                {
                  jours[id] = 1e-6; // set to 0
                  isPush = true;
                  if (relc_counts[id] > prev_halt && drift_p > 0.25)
                  {
                    isOpt = true;
                    for (int &cnt : relc_counts)
                      cnt = 0;
                  }
                }
              }
            }

            if (isPush)
            {
              match_num++;
              lp_edges.push(id, cur_id, ord_bl, buf_base - 1, loop_transform.second, loop_transform.first, v6_init);
              if (step > -1)
              {
                int id1 = stepsizes[step] + ord_bl;
                int id2 = stepsizes.back() - 1;
                add_edge(id1, id2, loop_transform.second, loop_transform.first, graph, odom_noise);
                printf("\033[32;43maddedge: (%d %d) (%d %d)\033[0m\n", id, cur_id, ord_bl, buf_base - 1);
              }
            }

            // if(isPush)
            // {
            //   icp_check(*(smp->plptr), *(std_managers[id]->plane_cloud_vec_[search_result.first]), pub_test, pub_init, loop_transform, multimap_scanPoses[id]->at(ord_bl)->x);
            // }
          }
        }
      }
      for (int &it : relc_counts)
        it++;
      std_manager->AddSTDescs(stds_vec);

      if (isGraph)
      {
        build_graph(initial, graph, cur_id, lp_edges, odom_noise, ids, stepsizes, 1);
      }

      if (isOpt)
      {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        gtsam::ISAM2 isam(parameters);
        isam.update(graph, initial);

        for (int i = 0; i < 5; i++)
          isam.update();
        gtsam::Values results = isam.calculateEstimate();
        int resultsize = results.size();

        IMUST x1 = scanPoses->at(buf_base - 1)->x;
        int idsize = ids.size();

        history_kfsize = 0;
        for (int ii = 0; ii < idsize; ii++)
        {
          int tip = ids[ii];
          for (int j = stepsizes[ii]; j < stepsizes[ii + 1]; j++)
          {
            int ord = j - stepsizes[ii];
            multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
          }
        }
        mtx_keyframe.lock();
        for (int ii = 0; ii < idsize; ii++)
        {
          int tip = ids[ii];
          for (Keyframe *kf : *multimap_keyframes[tip])
            kf->x0 = multimap_scanPoses[tip]->at(kf->id)->x;
        }
        mtx_keyframe.unlock();

        initial.clear();
        for (int i = 0; i < resultsize; i++)
          initial.insert(i, results.at(i).cast<gtsam::Pose3>());

        IMUST x3 = scanPoses->at(buf_base - 1)->x;
        dx.p = x3.p - x3.R * x1.R.transpose() * x1.p;
        dx.R = x3.R * x1.R.transpose();
        x_key = x3;

        PVec pvec_tem;
        int subsize = keyframes->size();
        int init_num = 5;
        // build corrected local voxel map (map_loop) from recent keyframes
        for (int i = subsize - init_num; i < subsize; i++)
        {
          if (i < 0)
            continue;
          Keyframe &sp = *(keyframes->at(i));
          sp.exist = 0;
          pvec_tem.reserve(sp.plptr->size());
          pointVar pv;
          pv.var.setZero();
          for (PointType &ap : sp.plptr->points)
          {
            pv.pnt << ap.x, ap.y, ap.z;
            pv.pnt = sp.x0.R * pv.pnt + sp.x0.p;
            for (int j = 0; j < 3; j++)
              pv.var(j, j) = ap.normal[j];
            pvec_tem.push_back(pv);
          }
          cut_voxel(map_loop, pvec_tem, win_size, 0);
        }

        if (subsize > init_num)
        {
          pl_kdmap->clear();
          for (int i = 0; i < subsize - init_num; i++)
          {
            Keyframe &kf = *(keyframes->at(i));
            kf.exist = 1;
            PointType pp;
            pp.x = kf.x0.p[0];
            pp.y = kf.x0.p[1];
            pp.z = kf.x0.p[2];
            pp.intensity = cur_id;
            pp.curvature = i;
            pl_kdmap->push_back(pp);
          }

          kd_keyframes.setInputCloud(pl_kdmap);
          history_kfsize = pl_kdmap->size();
        }
        loop_detect = 1;

        vector<int> ids2 = ids;
        ids2.pop_back();
        ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids2);
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_pmap);
        ids2.clear();
        ids2.push_back(ids.back());
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_cmap);
      }
    }

    for (int i = 0; i < std_managers.size(); i++)
      delete std_managers[i];
    malloc_trim(0);

    if (is_finish)
    {
      if (keyframes->empty())
      {
        sessionNames.pop_back();
        std_managers.pop_back();
        multimap_scanPoses.pop_back();
        multimap_keyframes.pop_back();
        juds.pop_back();
        jours.pop_back();
        relc_counts.pop_back();
      }

      if (multimap_keyframes.empty())
      {
        printf("no data\n");
        return;
      }

      int cur_id = std_managers.size() - 1;
      build_graph(initial, graph, cur_id, lp_edges, odom_noise, ids, stepsizes, 0);

      topDownProcess(initial, graph, ids, stepsizes);
    }

    if (is_save_map)
    {
      for (int i = 0; i < ids.size(); i++)
        FileReaderWriter::instance().save_pose(*(multimap_scanPoses[ids[i]]), sessionNames[ids[i]], "/alidarState.txt", savepath);

      for (int i = 0; i < ids.size(); i++)
        FileReaderWriter::instance().save_global_pcd(*(multimap_scanPoses[ids[i]]), savepath + bagname);

      FileReaderWriter::instance().pgo_edges_io(lp_edges, sessionNames, 1, savepath, bagname);
    }

    for (int i = 0; i < multimap_scanPoses.size(); i++)
    {
      for (int j = 0; j < multimap_scanPoses[i]->size(); j++)
        delete multimap_scanPoses[i]->at(j);
    }
    for (int i = 0; i < multimap_keyframes.size(); i++)
    {
      for (int j = 0; j < multimap_keyframes[i]->size(); j++)
        delete multimap_keyframes[i]->at(j);
    }

    malloc_trim(0);
  }

  // The top down process of HBA
  void topDownProcess(gtsam::Values &initial, gtsam::NonlinearFactorGraph &graph, vector<int> &ids, vector<int> &stepsizes)
  {
    cnct_map = ids;
    gba_size = multimap_keyframes.back()->size();
    gba_flag = 1;

    pcl::PointCloud<PointType> pl0;
    pub_pl_func(pl0, pub_pmap);
    pub_pl_func(pl0, pub_cmap);
    pub_pl_func(pl0, pub_curr_path);
    pub_pl_func(pl0, pub_prev_path);
    pub_pl_func(pl0, pub_scan);

#ifdef ROS1
    double t0 = ros::Time::now().toSec();
#else
    double t0 = rclcpp::Clock().now().seconds();
#endif
    while (gba_flag)
      sleep(0.1);

    for (PGO_Edge &edge : gba_edges1.edges)
    {
      vector<int> step(2);
      if (edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for (int i = 0; i < edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(edge.covs[i]));
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, v6_noise);
        }
      }
    }

    for (PGO_Edge &edge : gba_edges2.edges)
    {
      vector<int> step(2);
      if (edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for (int i = 0; i < edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(edge.covs[i]));
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, v6_noise);
        }
      }
    }

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);

    for (int i = 0; i < 5; i++)
      isam.update();
    gtsam::Values results = isam.calculateEstimate();
    // int resultsize = results.size();

    int idsize = ids.size();
    for (int ii = 0; ii < idsize; ii++)
    {
      int tip = ids[ii];
      for (int j = stepsizes[ii]; j < stepsizes[ii + 1]; j++)
      {
        int ord = j - stepsizes[ii];
        multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
      }
    }

    Eigen::Quaterniond qq(multimap_scanPoses[0]->at(0)->x.R);

#ifdef ROS1
    double t1 = ros::Time::now().toSec();
#else
    double t1 = rclcpp::Clock().now().seconds();
#endif
    printf("GBA opt: %lfs\n", t1 - t0);

    for (int ii = 0; ii < idsize; ii++)
    {
      int tip = ids[ii];
      for (Keyframe *smp : *multimap_keyframes[tip])
        smp->x0 = multimap_scanPoses[tip]->at(smp->id)->x;
    }

    ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids);
    vector<int> ids2 = ids;
    ids2.pop_back();
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_pmap);
    ids2.clear();
    ids2.push_back(ids.back());
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_cmap);
  }

  // The bottom up to add edge in HBA
  void HBA_add_edge(vector<IMUST> &p_xs, vector<Keyframe *> &p_smps, PGO_Edges &gba_edges, vector<int> &maps, int max_iter, int thread_num, pcl::PointCloud<PointType>::Ptr plptr = nullptr)
  {
    bool is_display = false;
    if (plptr == nullptr)
      is_display = true;

    // double t0 = ros::Time::now().toSec();
    vector<Keyframe *> smps;
    vector<IMUST> xs;
    int last_mp = -1, isCnct = 0;
    for (int i = 0; i < p_smps.size(); i++)
    {
      Keyframe *smp = p_smps[i];
      if (smp->mp != last_mp)
      {
        isCnct = 0;
        for (int &m : maps)
          if (smp->mp == m)
          {
            isCnct = 1;
            break;
          }
        last_mp = smp->mp;
      }

      if (isCnct)
      {
        smps.push_back(smp);
        xs.push_back(p_xs[i]);
      }
    }

    int wdsize = smps.size();
    Eigen::MatrixXd hess;
    vector<double> gba_eigen_value_array_orig = gba_eigen_value_array;
    double gba_min_eigen_value_orig = gba_min_eigen_value;
    double gba_voxel_size_orig = gba_voxel_size;

    int up = 4;
    int converge_flag = 0;
    double converge_thre = 0.05;

    for (int iterCnt = 0; iterCnt < max_iter; iterCnt++)
    {
      if (converge_flag == 1 || iterCnt == max_iter - 1)
      {
        // if(plptr == nullptr)
        // {
        //   break;
        // }

        gba_voxel_size = voxel_size;
        gba_eigen_value_array = plane_eigen_value_thre;
        gba_min_eigen_value = min_eigen_value;
      }

      unordered_map<VOXEL_LOC, OctreeGBA *> oct_map;
      for (int i = 0; i < wdsize; i++)
        OctreeGBA::cut_voxel(oct_map, xs[i], smps[i]->plptr, i, wdsize);

      LidarFactor voxhess(wdsize);
      OctreeGBA_multi_recut(oct_map, voxhess, thread_num);

      Lidar_BA_Optimizer opt_lsv;
      opt_lsv.thd_num = thread_num;
      vector<double> resis;
      bool is_converge = opt_lsv.damping_iter(xs, voxhess, &hess, resis, up, is_display);
      if (is_display)
        printf("%lf\n", fabs(resis[0] - resis[1]) / resis[0]);
      if ((fabs(resis[0] - resis[1]) / resis[0] < converge_thre && is_converge) || (iterCnt == max_iter - 2 && converge_flag == 0))
      {
        converge_thre = 0.01;
        if (converge_flag == 0)
        {
          converge_flag = 1;
        }
        else if (converge_flag == 1)
        {
          break;
        }
      }
    }

    gba_eigen_value_array = gba_eigen_value_array_orig;
    gba_min_eigen_value = gba_min_eigen_value_orig;
    gba_voxel_size = gba_voxel_size_orig;

    for (int i = 0; i < wdsize - 1; i++)
      for (int j = i + 1; j < wdsize; j++)
      {
        bool isAdd = true;
        Eigen::Matrix<double, 6, 1> v6;
        for (int k = 0; k < 6; k++)
        {
          double hc = fabs(hess(6 * i + k, 6 * j + k));
          if (hc < 1e-6) // 1e-6
          {
            isAdd = false;
            break;
          }
          v6[k] = 1.0 / hc;
        }

        if (isAdd)
        {
          Keyframe &s1 = *smps[i];
          Keyframe &s2 = *smps[j];
          Eigen::Vector3d tra = xs[i].R.transpose() * (xs[j].p - xs[i].p);
          Eigen::Matrix3d rot = xs[i].R.transpose() * xs[j].R;
          gba_edges.push(s1.mp, s2.mp, s1.id, s2.id, rot, tra, v6);
        }
      }

    if (plptr != nullptr)
    {
      pcl::PointCloud<PointType> pl;
      IMUST xc = xs[0];
      for (int i = 0; i < wdsize; i++)
      {
        Eigen::Vector3d dp = xc.R.transpose() * (xs[i].p - xc.p);
        Eigen::Matrix3d dR = xc.R.transpose() * xs[i].R;
        for (PointType ap : smps[i]->plptr->points)
        {
          Eigen::Vector3d v3(ap.x, ap.y, ap.z);
          v3 = dR * v3 + dp;
          ap.x = v3[0];
          ap.y = v3[1];
          ap.z = v3[2];
          ap.intensity = smps[i]->mp;
          pl.push_back(ap);
        }
      }

      down_sampling_voxel(pl, voxel_size / 8);
      plptr->clear();
      plptr->reserve(pl.size());
      for (PointType &ap : pl.points)
        plptr->push_back(ap);
    }
    else
    {
      // pcl::PointCloud<PointType> pl, path;
      // pub_pl_func(pl, pub_test);
      // for(int i=0; i<wdsize; i++)
      // {
      //   PointType pt;
      //   pt.x = xs[i].p[0]; pt.y = xs[i].p[1]; pt.z = xs[i].p[2];
      //   path.push_back(pt);
      //   for(int j=1; j<smps[i]->plptr->size(); j+=2)
      //   {
      //     PointType ap = smps[i]->plptr->points[j];
      //     Eigen::Vector3d v3(ap.x, ap.y, ap.z);
      //     v3 = xs[i].R * v3 + xs[i].p;
      //     ap.x = v3[0]; ap.y = v3[1]; ap.z = v3[2];
      //     ap.intensity = smps[i]->mp;
      //     pl.push_back(ap);

      //     if(pl.size() > 1e7)
      //     {
      //       pub_pl_func(pl, pub_test);
      //       pl.clear();
      //       sleep(0.05);
      //     }
      //   }
      // }
      // pub_pl_func(pl, pub_test);
      // return;
    }
  }

  // The main thread of bottom up in global mapping
  // HBA: https://blog.csdn.net/love20102011/article/details/160825783
#ifdef ROS1
  void thd_globalmapping(ros::NodeHandle &n)
  {
    int total_max_iter = 1;
    n.param<double>("GBA/voxel_size", gba_voxel_size, 1.0);
    n.param<double>("GBA/min_eigen_value", gba_min_eigen_value, 0.01);
    n.param<vector<double>>("GBA/eigen_value_array", gba_eigen_value_array, vector<double>());
    n.param<int>("GBA/total_max_iter", total_max_iter, 1);
#else
  void thd_globalmapping(rclcpp::Node::SharedPtr &node)
  {
    int total_max_iter = 1;
    node->declare_parameter("gba_voxel_size", 1.0);
    node->declare_parameter("gba_min_eigen_value", 0.01);
    node->declare_parameter("gba_eigen_value_array", vector<double>());
    node->declare_parameter("gba_total_max_iter", 1);

    node->get_parameter("gba_voxel_size", gba_voxel_size);
    node->get_parameter("gba_min_eigen_value", gba_min_eigen_value);
    node->get_parameter("gba_eigen_value_array", gba_eigen_value_array);
    node->get_parameter("gba_total_max_iter", total_max_iter);
#endif
    for (double &iter : gba_eigen_value_array)
      iter = 1.0 / iter;

    vector<Keyframe *> gba_submaps;
    deque<int> localID;

    int smp_mp = 0;
    int buf_base = 0;
    int wdsize = 10; // HBA param, window size of submap
    int mgsize = 5;  // HBA param, step size to update submap
    int thread_num = 5;

#ifdef ROS1
    while (n.ok())
#else
    while (rclcpp::ok())
#endif
    {
      if (multimap_keyframes.empty())
      {
        sleep(0.1);
        continue;
      }

      int smp_flag = 0;
      if (smp_mp + 1 < multimap_keyframes.size() && !multimap_keyframes.back()->empty())
        smp_flag = 1;

      vector<Keyframe *> &smps = *multimap_keyframes[smp_mp];
      int total_ba = 0;
      if (gba_flag == 1 && smp_mp >= cnct_map.back() && gba_size <= buf_base)
      {
        printf("gba_flag enter: %d\n", gba_flag);
        total_ba = 1;
      }
      else if (smps.size() <= buf_base)
      {
        if (smp_flag == 0)
        {
          sleep(0.1);
          continue;
        }
      }
      else
      {
        smps[buf_base]->mp = smp_mp;
        localID.push_back(buf_base);

        buf_base++;
        if (localID.size() < wdsize)
        {
          sleep(0.1);
          continue;
        }
      }

      vector<IMUST> xs;
      vector<Keyframe *> smp_local;
      mtx_keyframe.lock();
      for (int i : localID)
      {
        xs.push_back(multimap_keyframes[smp_mp]->at(i)->x0);
        smp_local.push_back(multimap_keyframes[smp_mp]->at(i));
      }
      mtx_keyframe.unlock();

      // double tg1 = ros::Time::now().toSec();

      Keyframe *gba_smp = new Keyframe(smp_local[0]->x0);
      vector<int> mps{smp_mp};
      // extract intra-submap constraints for layer 1
      HBA_add_edge(xs, smp_local, gba_edges1, mps, 1, 2, gba_smp->plptr);
      gba_smp->id = smp_local[0]->id;
      gba_smp->mp = smp_mp;
      gba_submaps.push_back(gba_smp);

      if (total_ba == 1)
      {
        printf("GBAsize: %d\n", gba_size);
        vector<IMUST> xs;
        mtx_keyframe.lock();
        for (Keyframe *smp : gba_submaps)
        {
          xs.push_back(multimap_scanPoses[smp->mp]->at(smp->id)->x);
        }
        mtx_keyframe.unlock();
        gba_edges2.edges.clear();
        gba_edges2.mates.clear();
        // extract inter-submap constraints for layer 2
        HBA_add_edge(xs, gba_submaps, gba_edges2, cnct_map, total_max_iter, thread_num);

        if (is_finish)
        {
          for (int i = 0; i < gba_submaps.size(); i++)
            delete gba_submaps[i];
        }
        gba_submaps.clear();

        malloc_trim(0);
        gba_flag = 0;
      }
      else if (smp_flag == 1 && multimap_keyframes[smp_mp]->size() <= buf_base)
      {
        smp_mp++;
        buf_base = 0;
        localID.clear();
        // printf("switch: %d\n", smp_mp);
      }
      else
      {
        for (int i = 0; i < mgsize; i++)
          localID.pop_front();
      }
    }
  }
};

/*
+===================================================================+
|                     SENSOR INPUT                                   |
|  IMU Topic ----> imu_handler() ----> imu_buf (deque)              |
|  LiDAR Topic --> pcl_handler() ----> pcl_buf + time_buf (deque)   |
+========================|==========================================+
                         |
                         v
+===================================================================+
|  THREAD 1: thd_odometry_localmapping()                            |
|                                                                    |
|  sync_packages() ------> pcl_curr + imus                          |
|       |                                                            |
|       v                                                            |
|  IMUEKF::process()                                                 |
|    |-- IMU_init() (accumulate gravity)                             |
|    |-- motion_blur() (forward propagation + de-distortion)         |
|       |                                                            |
|       v                                                            |
|  [Initialization Path]        [Normal Path]                        |
|  var_init()                   down_sampling_voxel()                |
|  lio_state_estimation_        var_init()                           |
|    kdtree()                   lio_state_estimation()               |
|  motion_init()                  |                                  |
|    (gravity estimation)         v                                  |
|       |                   pvec_update() --> pwld                    |
|       |                   cut_voxel_multi() --> OctoTree           |
|       |                   multi_recut() --> LidarFactor            |
|       |                   LI_BA::damping_iter()                    |
|       |                   multi_margi() --> marginalize            |
|       |                         |                                  |
|       +-------------------------+                                  |
|                                 |                                  |
|                                 v                                  |
|              ScanPose ---> buf_lba2loop (mutex protected)          |
|                                 |                                  |
|              <-- loop_detect -- | -- dx (correction) <--           |
|              <-- map_loop ------+                       |          |
+===================================================================+
                                  |
          +-----------------------+---------------------+
          |                                             |
          v                                             v
+================================+  +===================================+
| THREAD 2: thd_loop_closure()   |  | THREAD 3: thd_globalmapping()     |
|                                |  |                                   |
| Receive ScanPose from          |  | Receive Keyframes from            |
|   buf_lba2loop                 |  |   multimap_keyframes              |
|       |                        |  |       |                           |
|       v                        |  |       v                           |
| Aggregate win_size scans       |  | Accumulate wdsize keyframes      |
| Create Keyframe + plbtc        |  |       |                           |
|       |                        |  |       v                           |
|       v                        |  | HBA bottom-up:                    |
| GenerateSTDescs()              |  |   OctreeGBA::cut_voxel()          |
|   (BTC descriptors)            |  |   OctreeGBA_multi_recut()         |
|       |                        |  |   Lidar_BA_Optimizer              |
|       v                        |  |     ::damping_iter()              |
| SearchLoop() x all sessions    |  |       |                           |
|       |                        |  |       v                           |
|       v                        |  | HBA_add_edge()                    |
| icp_normal()                   |  |   --> gba_edges1 (intra-submap)   |
|   (ICP refinement)             |  |       |                           |
|       |                        |  |       v                           |
|       v                        |  | (when gba_flag==1)                |
| Drift check + add edge         |  | Top-level BA across submaps      |
| lp_edges.push()               |  |   --> gba_edges2 (inter-submap)   |
|       |                        |  |   gba_flag = 0                    |
|       v                        |  +===================================+
| GTSAM ISAM2 optimize          |                  |
|       |                        |                  v
|       v                        |     topDownProcess():
| Update all ScanPose poses     |       Add gba_edges1 + gba_edges2
| Compute dx (correction)       |       ISAM2 optimize
| Build map_loop                 |       Update all poses
| loop_detect = 1               |
|       |                        |
|       v                        |
| (at finish) topDownProcess()   |
+================================+
 */

int main(int argc, char **argv)
{
#ifdef ROS1
  ros::init(argc, argv, "voxel_slam");
  ros::NodeHandle n;

  pub_cmap = n.advertise<sensor_msgs::PointCloud2>("/map_cmap", 100);
  pub_pmap = n.advertise<sensor_msgs::PointCloud2>("/map_pmap", 100);
  pub_scan = n.advertise<sensor_msgs::PointCloud2>("/map_scan", 100);
  pub_init = n.advertise<sensor_msgs::PointCloud2>("/map_init", 100);
  pub_test = n.advertise<sensor_msgs::PointCloud2>("/map_test", 100);
  pub_curr_path = n.advertise<sensor_msgs::PointCloud2>("/map_path", 100);
  pub_prev_path = n.advertise<sensor_msgs::PointCloud2>("/map_true", 100);

  VOXEL_SLAM vs(n);
#else
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("voxel_slam");

  pub_cmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cmap", 100);
  pub_pmap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_pmap", 100);
  pub_scan = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_scan", 100);
  pub_init = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_init", 100);
  pub_test = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_test", 100);
  pub_curr_path = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_path", 100);
  pub_prev_path = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_true", 100);

  VOXEL_SLAM vs(node);
#endif

  mp = new int[vs.win_size];
  for (int i = 0; i < vs.win_size; i++)
    mp[i] = i;

#ifdef ROS1
  thread thread_loop(&VOXEL_SLAM::thd_loop_closure, &vs, ref(n));
  thread thread_gba(&VOXEL_SLAM::thd_globalmapping, &vs, ref(n));
  vs.thd_odometry_localmapping(n);
#else
  thread thread_loop(&VOXEL_SLAM::thd_loop_closure, &vs, ref(node));
  thread thread_gba(&VOXEL_SLAM::thd_globalmapping, &vs, ref(node));
  vs.thd_odometry_localmapping(node);
#endif

  thread_loop.join();
  thread_gba.join();
#ifdef ROS1
  ros::spin();
#else
  rclcpp::spin(node);
#endif
  return 0;
}
