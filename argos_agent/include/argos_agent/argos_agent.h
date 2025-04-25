class ArgosAgent {
  public:
    ArgosAgent();
    ~ArgosAgent();

    int run();

  private:
    // Functions
    // Callbacks
    void lookaheadCallback();
    void odometryCallback();
    // Member Functions
    double pidStep();
    double getSteeringAngle();

    // Parameters
    // Car parameters
    std::string agent_name_;
    std::string global_frame_;

    // Simulation parameters
    double d_;
    double p_;
    double d_;
    double desired_velocity_;
    double lookahead_distance_;
    double prev_error_;
    ros::Duration prev_timestamp_;
    double theta_;

    // Member variables
    geometry_msgs::Pose lookahead_point_; // TODO just temporary until proper path received
    nav_msgs::Odometry odometry_;

    // Defaulted parameters
    bool control_mode_;

    // TF parameters
    std::string rear_axis_frame_;
    std::string base_link_frame_;
    tf2::Stamped<tf2::Transform> rear_axis_pose_;
    tf2::Stamped<tf2::Transform> base_link_pose_;

    // ROS infrastructure
    ros::NodeHandle nh_, nh_private_; 
    ros::Publisher control_pub_, control_mode_pub_; 
    ros::Subscriber lookahead_point_sub_, odometry_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener;
};
