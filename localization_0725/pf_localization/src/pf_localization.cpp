#include "pf_localization/pf_localization.h"

namespace localization
{
pf_localization::~pf_localization()
{
}

bool pf_localization::init()
{
    ROS_INFO("Init start");
    if (!loadParams())
    {
        ROS_ERROR("Load parameters failed!");
        return false;
    }

    sub_initpose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, boost::bind(&pf_localization::initalposeCB, this, _1));
    sub_clicked_ = nh_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, boost::bind(&pf_localization::clickedPointCB, this, _1));
    sub_scan_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 50, boost::bind(&pf_localization::laserCB, this, _1));
    sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 50, boost::bind(&pf_localization::odomCB, this, _1));
    getmap_call_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
    if (param_publish_odom_)
    {
        pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/pf/odom", 5);
    }
    pub_particle_ = nh_.advertise<geometry_msgs::PoseArray>("/pf/particles", 2);

    first_odom_ = true;
    first_sensor_update_ = true;
    odom_data_ = pf_vector_zero();
    last_pose_ = pf_vector_zero();
    pose_mean_ = pf_vector_zero();
    pose_cov_ = pf_matrix_zero();

    map_initialized_ = false;
    odom_initialized_ = false;
    lidar_initialized_ = false;
    pf_initialized_ = false;

    gen_ = std::mt19937(rd_());
    uniform_ = std::uniform_real_distribution<double>(0., 1.);

    getMap();
    precomputeSensorModel();
    initializeGlobal();

    ROS_INFO("Init ok");
    return true;
}

bool pf_localization::loadParams()
{
    ROS_INFO("Starting loading params...");
    pnh_.param<int>("max_particles", param_max_particles_, 1000);
    pnh_.param<int>("min_particles", param_min_particles_, 100);
    pnh_.param<double>("update_dist", param_update_dist_, 0.1);
    pnh_.param<double>("update_angle", param_update_angle_, 0.1);
    pnh_.param<double>("max_range", param_max_range_, 10.);
    pnh_.param<bool>("publish_odom", param_publish_odom_, true);
    pnh_.param<int>("max_viz_particles", param_max_viz_particles_, 100);
    pnh_.param<int>("resample_interval", param_resample_interval_, 2);

    pnh_.param<std::string>("range_method", param_range_method_, std::string("cddt"));
    pnh_.param<int>("theta_discretization", param_td_, 112);
    pnh_.param<int>("rangelib_variant", param_rangelib_var_, 2);

    pnh_.param<double>("z_short", param_z_short_, 0.01);
    pnh_.param<double>("z_max", param_z_max_, 0.07);
    pnh_.param<double>("z_rand", param_z_rand_, 0.12);
    pnh_.param<double>("sigma_hit", param_sigma_hit_, 8.0);
    pnh_.param<double>("z_hit", param_z_hit_, 0.75);
    pnh_.param<double>("c_r", param_c_r_, 0.01);
    pnh_.param<int>("downsample_size", param_downsample_size_, 18);
    pnh_.param<double>("motion_dev_x", param_motion_dev_x_, 0.05);
    pnh_.param<double>("motion_dev_y", param_motion_dev_y_, 0.025);
    pnh_.param<double>("motion_dev_theta", param_motion_dev_theta_, 0.25);
    pnh_.param<double>("squash_fac", param_squash_fac_, 2.2);

    pnh_.param<std::string>("laser_frame_id", param_laser_frame_id_, std::string("/laser"));
    pnh_.param<std::string>("odom_frame_id", param_odom_frame_id_, std::string("/odom"));
    pnh_.param<std::string>("base_link_frame_id", param_base_link_frame_id_, std::string("/base_link"));
    pnh_.param<std::string>("map_frame_id", param_map_frame_id_, std::string("/map"));

    ROS_INFO("End loading params.");
    return true;
}

void pf_localization::getMap()
{
    ROS_INFO("Waiting for map data from static_map service...");
    getmap_call_.waitForExistence((ros::Duration)(-1));
    getmap_call_.call(srv_getmap_);
    static_map_ = srv_getmap_.response.map;
    ROS_INFO("Map data received: width %d, height %d, resolution: %.7f", static_map_.info.width, static_map_.info.height, static_map_.info.resolution);
    // TODO: 注意源码可能有歧义，OMap(width, height) 生成的 grid 是 w*h 的数组，但应该是 h*w
    omap_ = new ranges::OMap(static_map_.info.height, static_map_.info.width);

    for (int i = 0; i < static_map_.info.height; i++)
    {
        for (int j = 0; j < static_map_.info.width; j++)
        {
            if (static_map_.data[i * static_map_.info.width + j] > 10)
            {
                omap_->grid[i][j] = true;
            }
        }
    }
    double yaw, pitch, roll;
    tf::Matrix3x3(tf::Quaternion(static_map_.info.origin.orientation.x, static_map_.info.origin.orientation.y, static_map_.info.origin.orientation.z, static_map_.info.origin.orientation.w)).getEulerYPR(yaw, pitch, roll);
    omap_->world_scale = static_map_.info.resolution;
    omap_->world_angle = -1. * yaw;
    omap_->world_origin_x = static_map_.info.origin.position.x;
    omap_->world_origin_y = static_map_.info.origin.position.y;
    omap_->world_sin_angle = sin(omap_->world_angle);
    omap_->world_cos_angle = cos(omap_->world_angle);

    max_range_px_ = (int)(param_max_range_ / static_map_.info.resolution);

    if (std::string("bl").compare(param_range_method_) == 0)
    {
        rm_ = static_cast<ranges::BresenhamsLine *>(new ranges::BresenhamsLine(*omap_, max_range_px_));
    }
    else if (std::string("cddt").compare(param_range_method_) == 0)
    {
        rm_ = static_cast<ranges::CDDTCast *>(new ranges::CDDTCast(*omap_, max_range_px_, param_td_));
    }
    else if (std::string("rm").compare(param_range_method_) == 0)
    {
        rm_ = static_cast<ranges::RayMarching *>(new ranges::RayMarching(*omap_, max_range_px_));
    }
    else if (std::string("rmgpu").compare(param_range_method_) == 0)
    {
        rm_ = static_cast<ranges::RayMarchingGPU *>(new ranges::RayMarchingGPU(*omap_, max_range_px_));
    }
    else if (std::string("glt").compare(param_range_method_) == 0)
    {
        rm_ = static_cast<ranges::GiantLUTCast *>(new ranges::GiantLUTCast(*omap_, max_range_px_, param_td_));
    }
    else
    {
        ROS_ERROR("Failed load range method: %s !", param_range_method_.c_str());
        return;
    }

    // permissible_region_ = new uint8_t[static_map_.info.height * static_map_.info.width];
    // for (int y = 0; y < static_map_.info.height; y++)
    // {
    //     for (int x = 0; x < static_map_.info.width; x++)
    //     {
    //         // static_map_.data: 0: permissible, -1: unmapped, 100: blocked
    //         permissible_region_[y * static_map_.info.width + x] = static_map_.data[y * static_map_.info.width + x] == 0 ? 1 : 0;
    //     }
    // }

    map_initialized_ = true;
    ROS_INFO("Map initialized.");
}

void pf_localization::precomputeSensorModel()
{
    ROS_INFO("Generating sensor model and store it into RangeMethod for later use...");
    table_width_ = max_range_px_ + 1;
    sensor_model_table_ = new double[table_width_ * table_width_];

    // d is computed range from RangeLibc
    for (int d = 0; d < table_width_; d++)
    {
        float norm = 0., sum_unkown = 0.;
        // r is observed range from the lidar
        for (int r = 0; r < table_width_; r++)
        {
            float prob = 0.;
            float z = r - d;

            prob += param_z_hit_ * exp(-pow(z, 2) / (2. * pow(param_sigma_hit_, 2))) / (param_sigma_hit_ * sqrt(2 * PI));

            if (r < d)
            {
                prob += 2. * param_z_short_ * (d - r) / d;
            }
            if (r == max_range_px_)
            {
                prob += param_z_max_;
            }
            else
            {
                // r < max_range_px_
                prob += param_z_rand_ / max_range_px_;
            }
            norm += prob;
            sensor_model_table_[r * table_width_ + d] = prob;
        }
        for (int r = 0; r < table_width_; r++)
        {
            sensor_model_table_[r * table_width_ + d] /= norm;
        }
    }

    if (param_rangelib_var_ > 0)
    {
        ROS_INFO("Setting sensor model . table width: %d", table_width_);
        rm_->set_sensor_model(sensor_model_table_, table_width_);
        for (int i = 0; i < table_width_; i++)
        {
            for (int j = 0; j < table_width_; j++)
            {
                printf("%.2f ", sensor_model_table_[i * table_width_ + j]);
            }
            printf("\n");
        }
    }

    ROS_INFO("Generating successed.");

    // TODO: visualize
}

void pf_localization::initializeGlobal()
{
}

void pf_localization::initalposeCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    double yaw, pitch, roll;
    tf::Matrix3x3(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z, msg->pose.pose.orientation.w))
        .getEulerYPR(yaw, pitch, roll);
    ROS_INFO("Initial pose received: (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
             msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, roll, pitch, yaw);
    boost::recursive_mutex::scoped_lock initpose(config_mutex_);

    ROS_INFO("Start init particle filter sample set");
    ros::Time start = ros::Time::now();
    bool tmp_flag = pf_initialized_;
    pf_initialized_ = false;

    if (!tmp_flag)
    {
        pf_ = (pf_t *)calloc(1, sizeof(pf_t));
        // 初始化粒子集
        pf_->max_samples = param_max_particles_;
        pf_->min_samples = param_min_particles_;
    }

    double avg_weight = 1. / param_max_particles_;

    for (int i = 0; i < 2; i++)
    {
        guassian_[i] = std::normal_distribution<double>(0., msg->pose.covariance[i * 6 + i]);
    }
    guassian_[2] = std::normal_distribution<double>(0., msg->pose.covariance[35]);

    for (int i = 0; i < 2; i++)
    {
        pf_sample_set_t *set;
        set = pf_->sets + i;

        if (!tmp_flag)
        {
            set->sample_count = param_max_particles_;
            set->samples = (pf_sample_t *)calloc(param_max_particles_, sizeof(pf_sample_t));
        }

        set->mean = pf_vector_zero();
        set->cov = pf_matrix_zero();

        for (int j = 0; j < param_max_particles_; j++)
        {
            set->samples[j].pose.v[0] = msg->pose.pose.position.x + guassian_[0](generator_[0]);
            set->samples[j].pose.v[1] = msg->pose.pose.position.y + guassian_[0](generator_[0]);
            // TODO: 注意此处 yaw 的正负
            set->samples[j].pose.v[2] = yaw + guassian_[2](generator_[2]);
            set->samples[j].weight = avg_weight;
            // pf_kdtree_insert(set->kdtree, set->samples[j].pose, set->samples[j].weight);
        }

        set->mean.v[0] = msg->pose.pose.position.x;
        set->mean.v[1] = msg->pose.pose.position.y;
        set->mean.v[2] = yaw;

        set->cov = pf_matrix_zero();
        set->cov.m[0][0] = msg->pose.covariance[0];
        set->cov.m[1][1] = msg->pose.covariance[1 * 6 + 1];
        set->cov.m[2][2] = msg->pose.covariance[5 * 6 + 5];
    }
    pf_->current_set = 0;
    // last_pose_ = pf_->sets[0].mean;
    pf_initialized_ = true;
    resample_count = 0;

    ROS_INFO("Particle filter sample set initialized! \ntime elapsed: %.3f s", (ros::Time::now() - start).toSec());
}

void pf_localization::clickedPointCB(const geometry_msgs::PointStampedConstPtr &msg)
{
    ROS_INFO("Clicked point: (%.3f, %.3f, %.3f) received.", msg->point.x, msg->point.y, msg->point.z);
    boost::recursive_mutex::scoped_lock initpose(config_mutex_);

    ROS_INFO("Starting initializing particles...");

    ros::Time start = ros::Time::now();
    bool tmp_flag = pf_initialized_;
    pf_initialized_ = false;

    if (!tmp_flag)
    {
        pf_ = (pf_t *)calloc(1, sizeof(pf_t));
        // 初始化粒子集
        pf_->max_samples = param_max_particles_;
        pf_->min_samples = param_min_particles_;
    }

    double avg_weight = 1. / param_max_particles_;

    guassian_[0] = std::normal_distribution<double>(0., 1);
    guassian_[2] = std::normal_distribution<double>(0., 2 * M_PI);

    for (int i = 0; i < 2; i++)
    {
        pf_sample_set_t *set;
        set = pf_->sets + i;

        if (!tmp_flag)
        {
            set->sample_count = param_max_particles_;
            set->samples = (pf_sample_t *)calloc(param_max_particles_, sizeof(pf_sample_t));
        }

        set->mean = pf_vector_zero();
        set->cov = pf_matrix_zero();

        for (int j = 0; j < param_max_particles_; j++)
        {
            set->samples[j].pose.v[0] = msg->point.x + guassian_[0](generator_[0]);
            set->samples[j].pose.v[1] = msg->point.y + guassian_[0](generator_[0]);
            // TODO: 注意此处 yaw 的正负
            set->samples[j].pose.v[2] = 0 + guassian_[2](generator_[2]);
            set->samples[j].weight = avg_weight;
        }

        set->mean.v[0] = msg->point.x;
        set->mean.v[1] = msg->point.y;
        set->mean.v[2] = 0;

        set->cov = pf_matrix_zero();
    }
    pf_->current_set = 0;
    // last_pose_ = pf_->sets[0].mean;
    pf_initialized_ = true;
    resample_count = 0;

    ROS_INFO("End initializing particles.");
}

void pf_localization::odomCB(const nav_msgs::OdometryConstPtr &msg)
{
    boost::recursive_mutex::scoped_lock odom(config_mutex_);

    pf_vector_t pose;
    pose.v[0] = msg->pose.pose.position.x;
    pose.v[1] = msg->pose.pose.position.y;
    double pitch, roll, yaw;
    tf::Matrix3x3(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w))
        .getEulerYPR(yaw, roll, pitch);
    pose.v[2] = yaw;

    if (first_odom_)
    {
        last_pose_ = pose;
        first_odom_ = false;
        return;
    }

    pf_vector_t delta = pf_vector_sub(pose, last_pose_);
    // odom_data_.v[0] = delta.v[0] * cos(-last_pose_.v[2]) - delta.v[1] * sin(-last_pose_.v[2]);
    // odom_data_.v[1] = delta.v[0] * sin(-last_pose_.v[2]) + delta.v[1] * cos(-last_pose_.v[2]);
    odom_data_.v[0] = delta.v[0];
    // TODO: loam 生成的 /odom 数据左右需要修改一下
    odom_data_.v[1] = delta.v[1];
    odom_data_.v[2] = delta.v[2];
    last_pose_ = pose;
    last_stamp_ = msg->header.stamp;
    odom_initialized_ = true;

    // ROS_INFO("Received odometry. delta: [%.3f, %.3f, %.3f]", odom_data_.v[0], odom_data_.v[1], odom_data_.v[2]);

    // update();
}

void pf_localization::laserCB(const sensor_msgs::LaserScanConstPtr &msg)
{
    boost::recursive_mutex::scoped_lock pc(config_mutex_);

    // ROS_INFO("Received laser scan...");
    if (msg->ranges.size() < 1)
    {
        return;
    }
    num_rays_ = (int)((msg->ranges.size() - 1) / param_downsample_size_) + 1;
    if (!lidar_initialized_)
    {
        downsample_angles_ = new float[num_rays_];

        for (int i = 0; i < num_rays_; i++)
        {
            downsample_angles_[i] = msg->angle_min + msg->angle_increment * i * param_downsample_size_;
        }
    }

    downsample_ranges_ = new float[num_rays_];
    for (int i = 0; i < num_rays_; i++)
    {
        downsample_ranges_[i] = msg->ranges[i * param_downsample_size_];
    }
    lidar_initialized_ = true;
    // ROS_INFO("Downsample ranges length: %d .", num_rays_);
    update();
}

void pf_localization::update()
{
    boost::recursive_mutex::scoped_lock odom(config_mutex_);
    ROS_INFO("Starting update particles...");

    if (map_initialized_ && odom_initialized_ && lidar_initialized_ && pf_initialized_)
    {
        ros::Time start = ros::Time::now();
        pf_vector_t action = odom_data_;
        odom_data_ = pf_vector_zero();

        motionModel(action);
        sensorModel();
        normalize();
        publishTF(pose_mean_);
        visualize();
        resampleParticles();

        ros::Time end = ros::Time::now();
        ROS_INFO("Particles updated. duration: %.4f", (end - start).toSec());
    }
    ROS_INFO("End updating particles...");
}

void pf_localization::motionModel(const pf_vector_t &action)
{

    ROS_INFO("Start predicting particle position via motion model...");

    // TODO: transform every pose to local coords

    ros::Time start = ros::Time::now();

    pf_sample_set_t *set = pf_->sets + pf_->current_set;
    guassian_[0] = std::normal_distribution<double>(0., param_motion_dev_x_);
    guassian_[1] = std::normal_distribution<double>(0., param_motion_dev_y_);
    guassian_[2] = std::normal_distribution<double>(0., param_motion_dev_theta_);
    double max_dev = 0;
    for (int i = 0; i < param_max_particles_; i++)
    {
        // TODO: 检查一下这里的角度正负
        // set->samples[i].pose.v[0] += action.v[0] * cos(-set->samples[i].pose.v[2]) - action.v[1] * sin(-set->samples[i].pose.v[2]) + guassian_[0](generator_[0]);
        // set->samples[i].pose.v[1] += action.v[0] * sin(-set->samples[i].pose.v[2]) + action.v[1] * sin(-set->samples[i].pose.v[2]) + guassian_[1](generator_[1]);
        // max_dev = action.v[0] > action.v[1] ? action.v[0] : action.v[1];
        // guassian_[0] = std::normal_distribution<double>(0., param_motion_dev_x_ * (1 + action.v[0] / max_dev));
        // guassian_[1] = std::normal_distribution<double>(0., param_motion_dev_y_ * (1 + action.v[1] / max_dev));
        // guassian_[2] = std::normal_distribution<double>(0., param_motion_dev_theta_ * (1 + action.v[2] / M_PI * 2));
        set->samples[i].pose.v[0] += action.v[0] + guassian_[0](generator_[0]);
        set->samples[i].pose.v[1] += action.v[1] + guassian_[1](generator_[1]);

        set->samples[i].pose.v[2] += action.v[2] + guassian_[2](generator_[2]);
    }

    ROS_INFO("End predicting. duration: %.4f", (ros::Time::now() - start).toSec());
}

void pf_localization::sensorModel()
{
    ROS_INFO("Start updating particles position via sensor model...");

    ros::Time start = ros::Time::now();
    if (param_rangelib_var_ == 2)
    {
        // num_rays_ = sizeof(downsample_angles_) / sizeof(float);
        if (first_sensor_update_)
        {
            first_sensor_update_ = false;
            queries_ = new float[param_max_particles_ * 3];
            ranges_ = new float[param_max_particles_ * num_rays_];
            weights_ = new double[param_max_particles_];
            // tiled_angles_ = new float[param_max_particles_ * num_rays_];
            // for (int p = 0; p < param_max_particles_; p++)
            // {
            //     for (int r = 0; r < num_rays_; r++)
            //     {
            //         tiled_angles_[p * num_rays_ + r] = downsample_angles_[r];
            //     }
            // }
        }

        for (int p = 0; p < param_max_particles_; p++)
        {
            for (int i = 0; i < 3; i++)
            {
                queries_[p * 3 + i] = pf_->sets[pf_->current_set].samples[p].pose.v[i];
            }
            // weights_[p] = pf_->sets[pf_->current_set].samples[p].weight;
        }
        // ROS_INFO("test sensor model. queries: %d, downsampe: %d, ranges: %d, max_p: %d, num_rays: %d", sizeof(queries_) / sizeof(float), sizeof(downsample_angles_) / sizeof(float), sizeof(ranges_) / sizeof(float), param_max_particles_, num_rays_);
        rm_->numpy_calc_range_angles(queries_, downsample_angles_, ranges_, param_max_particles_, num_rays_);

        rm_->eval_sensor_model(downsample_ranges_, ranges_, weights_, num_rays_, param_max_particles_);

        pf_sample_set_t *set = pf_->sets + pf_->current_set;
        int value = -1, x = 0, y = 0;
        for (int p = 0; p < param_max_particles_; p++)
        {
            weights_[p] = pow(weights_[p], 1. / param_squash_fac_);
            // weights_[p] *= 10000;
            x = (set->samples[p].pose.v[0] - static_map_.info.origin.position.x) / static_map_.info.resolution;
            y = (set->samples[p].pose.v[1] - static_map_.info.origin.position.y) / static_map_.info.resolution;

            if (x < 0 || x >= static_map_.info.width || y < 0 || y >= static_map_.info.height)
            {
                weights_[p] = 0;
            }
            else
            {
                int value = static_map_.data[y * static_map_.info.width + x];
                if (value == -1)
                {
                    weights_[p] /= 10;
                }
                else if (value > 0)
                {
                    weights_[p] /= value;
                }
            }
        }
    }
    else
    {
        ROS_ERROR("No such ranglib method! SET rangelib_variant PARAM to 2");
    }

    ROS_INFO("End updating. duration: %.4f", (ros::Time::now() - start).toSec());
}

void pf_localization::normalize()
{
    ROS_INFO("Start normalizing particles weight...");
    ros::Time start = ros::Time::now();
    double sum_w = 0.;
    for (int i = 0; i < param_max_particles_; i++)
    {
        sum_w += weights_[i];
    }
    if (sum_w < 0.01)
    {
        ROS_WARN("Total weight: %.8f is less than 0.01 !! cannot normalize", sum_w);
        // return;
    }
    else
    {
        ROS_WARN("Total weight: %.8f!!!!!!!!!!!!!!!!", sum_w);
    }

    pose_mean_ = pf_vector_zero();

    for (int i = 0; i < param_max_particles_; i++)
    {
        weights_[i] /= sum_w;
        pose_mean_.v[0] += queries_[i * 3] * weights_[i];
        pose_mean_.v[1] += queries_[i * 3 + 1] * weights_[i];
        pose_mean_.v[2] += queries_[i * 3 + 2] * weights_[i];

        pf_->sets[pf_->current_set].samples[i].weight = weights_[i];
    }
    pf_->sets[pf_->current_set].mean = pose_mean_;

    ROS_INFO("End normalizing. duration: %.4f", (ros::Time::now() - start).toSec());
}

void pf_localization::publishTF(const pf_vector_t &mean)
{

    transform_.setOrigin(tf::Vector3(mean.v[0], mean.v[1], 0));
    transform_.setRotation(tf::createQuaternionFromYaw(mean.v[2]));

    tf::Stamped<tf::Pose> tmp_tf_pose(transform_.inverse(), last_stamp_, param_laser_frame_id_);
    tf::Stamped<tf::Pose> odom_map;
    try
    {
        tf_listener_.transformPose(param_odom_frame_id_, tmp_tf_pose, odom_map);
    }
    catch (tf::TransformException e)
    {
        ROS_WARN("Failed to subtract base frame to odom");
        return;
    }
    tf::Transform tmp = tf::Transform(tf::Quaternion(odom_map.getRotation()), tf::Vector3(odom_map.getOrigin()));
    tf_broadcaster_.sendTransform(tf::StampedTransform(tmp.inverse(), last_stamp_, param_map_frame_id_, param_odom_frame_id_));

    // tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, last_stamp_, param_map_frame_id_, param_laser_frame_id_));

    if (param_publish_odom_)
    {
        nav_msgs::Odometry odom;
        odom.header.frame_id = param_map_frame_id_;
        odom.header.stamp = last_stamp_;
        odom.pose.pose.position.x = mean.v[0];
        odom.pose.pose.position.y = mean.v[1];
        tf::Quaternion q = tf::createQuaternionFromYaw(mean.v[2]);
        odom.pose.pose.orientation.x = q.getX();
        odom.pose.pose.orientation.y = q.getY();
        odom.pose.pose.orientation.z = q.getZ();
        odom.pose.pose.orientation.w = q.getW();

        pub_odom_.publish(odom);
    }
}

void pf_localization::visualize()
{
    // if (count_++ > 0)
    // {
    //     return;
    // }
    if (pub_particle_.getNumSubscribers() > 0)
    {
        int size = param_max_viz_particles_ < param_max_particles_ ? param_max_viz_particles_ : param_max_particles_;
        geometry_msgs::PoseArray pa;
        pa.header.frame_id = param_map_frame_id_;
        pa.header.stamp = last_stamp_;
        pa.poses.resize(size);
        geometry_msgs::Pose pose;
        tf::Quaternion q;
        for (int p = 0; p < size; p++)
        {
            pose.position.x = queries_[p * 3];
            pose.position.y = queries_[p * 3 + 1];
            q = tf::createQuaternionFromYaw(queries_[p * 3 + 2]);
            pose.orientation.x = q.getX();
            pose.orientation.y = q.getY();
            pose.orientation.z = q.getZ();
            pose.orientation.w = q.getW();
            pa.poses.push_back(pose);
        }
        pub_particle_.publish(pa);
    }
}

void pf_localization::resampleParticles()
{

    if (!(++resample_count % param_resample_interval_))
    {
        return;
    }
    // TODO: does not need resample particles every update
    ROS_INFO("Start resampling particles...");

    ros::Time start = ros::Time::now();
    pf_sample_set_t *set = pf_->sets + pf_->current_set;
    double max_w = 0.;
    for (int p = 0; p < param_max_particles_; p++)
    {
        // set->samples[p].pose.v[0] = ranges_[p * 3];
        // set->samples[p].pose.v[1] = ranges_[p * 3 + 1];
        // set->samples[p].pose.v[2] = ranges_[p * 3 + 2];
        set->samples[p].weight = weights_[p];
        if (max_w < weights_[p])
        {
            max_w = weights_[p];
        }
    }
    pf_sample_set_t *set_b = pf_->sets + ((pf_->current_set + 1) % 2);

    int index = (int)(uniform_(gen_) * param_max_particles_);
    double beta = 0.;
    max_w *= 2;
    for (int p = 0; p < param_max_particles_; p++)
    {
        beta += uniform_(gen_) * max_w;
        while (beta > weights_[index])
        {
            beta -= weights_[index];
            index = (index + 1) % param_max_particles_;
        }
        set_b->samples[p].pose.v[0] = set->samples[index].pose.v[0];
        set_b->samples[p].pose.v[1] = set->samples[index].pose.v[1];
        set_b->samples[p].pose.v[2] = set->samples[index].pose.v[2];
        set_b->samples[p].weight = set->samples[index].weight;
    }
    pf_->current_set = (pf_->current_set + 1) % 2;

    ROS_INFO("End resampling particles. duration: %.4f", (ros::Time::now() - start).toSec());
}

} // namespace localization