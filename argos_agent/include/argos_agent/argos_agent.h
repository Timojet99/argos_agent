class ArgosAgent {
  public:
    ArgosAgent();
    ~ArgosAgent();

    int Run();

  private:
    // Functions
    void LookaheadCallback();
    void OdometryCallback();

    // Parameters
    // Car parameters
    std::string agent_name_;
    std::string global_frame_;
    std::string cmd_topic_;
    std::string agent_state_topic_;

    // Simulation parameters
    float integral_;
    float proportional_;
    float differential_;
    float desired_velocity_;
    float lookahead_distance_;

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
