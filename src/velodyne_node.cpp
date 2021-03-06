#include <ros/ros.h>
#include <dynamic_obstacle_tracking/dynamic_obstacle_tracking.hpp>
#include <dynamic_obstacle_tracking/velodyneConfig.h>



using namespace datmo;



    class Velodyne: public cloud_segmentation{

    public:
	    Velodyne();
	    ~Velodyne();
	    void init(ros::NodeHandle &nh, ros::NodeHandle &private_nh);

	protected:

        dynamic_reconfigure::Server<dynamic_obstacle_tracking::velodyneConfig> server;
        dynamic_reconfigure::Server<dynamic_obstacle_tracking::velodyneConfig>::CallbackType cb_type;

	private:
        void dynamic_reconfigure_cb(dynamic_obstacle_tracking::velodyneConfig &config, uint32_t level);
    };



    Velodyne::Velodyne(){};
    Velodyne::~Velodyne(){};

    //dynamic reconfigure callback
    void Velodyne::dynamic_reconfigure_cb(dynamic_obstacle_tracking::velodyneConfig &config, uint32_t level)
    {
        ENABLE_OCCLUSION_DETECTION = config.ENABLE_OCCLUSION_DETECTION;
        PASS_X_MIN = config.PASS_X_MIN;
        PASS_X_MAX = config.PASS_X_MAX;
        PASS_Y_MIN = config.PASS_Y_MIN;
        PASS_Y_MAX = config.PASS_Y_MAX;
        PASS_Z_MIN = config.PASS_Z_MIN;
        PASS_Z_MAX = config.PASS_Z_MAX;
        OCTREE_WINDOW = config.OCTREE_WINDOW;
        OCTREE_RESOLUTION = config.OCTREE_RESOLUTION;
        SEG_CLUSTER_TOLERANCE = config.SEG_CLUSTER_TOLERANCE;
        SEG_MIN_CLUSTER_SIZE = config.SEG_MIN_CLUSTER_SIZE;
        SEG_MAX_CLUSTER_SIZE = config.SEG_MAX_CLUSTER_SIZE;
    }

    void Velodyne::init(ros::NodeHandle &nh, ros::NodeHandle &private_nh){

        PASS_X_MIN = 0.0; /**< minimum value of x */
        PASS_X_MAX = 20.0; ///< maximum value of x
        PASS_Y_MIN = -15.0;
        PASS_Y_MAX = 15.0;
        PASS_Z_MIN = -0.3;
        PASS_Z_MAX = 4.0;
        //plane segmentation params
        OCTREE_WINDOW = 5;
        OCTREE_RESOLUTION = 0.4;
        //segmentation params
        SEG_CLUSTER_TOLERANCE = 1.5;
        SEG_MIN_CLUSTER_SIZE = 50;
        SEG_MAX_CLUSTER_SIZE = 2500;

        ENABLE_ICP = false;
        ENABLE_GROUND_REMOVAL  = true;
        ENABLE_VOXELISE = false;
        ENABLE_SOR = false;
        ENABLE_OCCLUSION_DETECTION  = false;

        private_nh.param("frame_id", frame_id,  std::string("/odom"));
        private_nh.param("base_frame_id", base_frame_id,  std::string("/base_link"));
        private_nh.param("sensor_frame_id", sensor_frame_id,  std::string("/vlp16"));


        cb_type = boost::bind(&Velodyne::dynamic_reconfigure_cb, this, _1, _2);
        server.setCallback(cb_type);
        cloud_segmentation::init(nh,private_nh);
    }









//.................


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_segmentation");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  Velodyne velodyne;
  velodyne.init(nh,private_nh);


  // cloud_segmentation seg;
  // seg.init(nh,private_nh);

  ros::spin();
  return 0;

}
