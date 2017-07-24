#include <ros/ros.h>
#include <dynamic_obstacle_tracking/dynamic_obstacle_tracking.hpp>
#include <dynamic_obstacle_tracking/zr300Config.h>



using namespace datmo;



    class Sensor: public cloud_segmentation{

    public:
	    Sensor();
	    ~Sensor();
	    void init(ros::NodeHandle &nh, ros::NodeHandle &private_nh);

	protected:

        dynamic_reconfigure::Server<dynamic_obstacle_tracking::zr300Config> server;
        dynamic_reconfigure::Server<dynamic_obstacle_tracking::zr300Config>::CallbackType cb_type;

	private:
        void dynamic_reconfigure_cb(dynamic_obstacle_tracking::zr300Config &config, uint32_t level);
    };



    Sensor::Sensor(){};
    Sensor::~Sensor(){};

    //dynamic reconfigure callback
    void Sensor::dynamic_reconfigure_cb(dynamic_obstacle_tracking::zr300Config &config, uint32_t level)
    {
        VOXEL_LEAF_SIZE = config.VOXEL_LEAF_SIZE;
        ENABLE_SOR = config.ENABLE_SOR;
        ENABLE_OCCLUSION_DETECTION = config.ENABLE_OCCLUSION_DETECTION;
        SOR_MEAN_K = config.SOR_MEAN_K;
        SOR_STD_DEV_MUL_THRESH = config.SOR_STD_DEV_MUL_THRESH;
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

    void Sensor::init(ros::NodeHandle &nh, ros::NodeHandle &private_nh){

        VOXEL_LEAF_SIZE = 0.15;
        //Statistical outlier parameters
        ENABLE_SOR = true;
        SOR_MEAN_K = 25;              //number of neighbors to analyze for each point
        SOR_STD_DEV_MUL_THRESH = 1.0; //distance in multiples of std dev after which point is considered an outlier
        //Passthrough filter parameters
        PASS_X_MIN = 6.0;
        PASS_X_MAX = 0.0;
        PASS_Y_MIN = -3.0;
        PASS_Y_MAX = 3.0;
        PASS_Z_MIN = 0.2;
        PASS_Z_MAX = 4.0;
        //plane segmentation params
        OCTREE_WINDOW = 15;
        OCTREE_RESOLUTION = 0.3;
        //segmentation params
        SEG_CLUSTER_TOLERANCE = 0.5;
        SEG_MIN_CLUSTER_SIZE = 40;
        SEG_MAX_CLUSTER_SIZE = 2500;

        ENABLE_OCCLUSION_DETECTION  = true;

        cb_type = boost::bind(&Sensor::dynamic_reconfigure_cb, this, _1, _2);
        server.setCallback(cb_type);
        cloud_segmentation::init(nh,private_nh);
    }









//.................


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_segmentation");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  Sensor zr300;
  zr300.init(nh,private_nh);


  // cloud_segmentation seg;
  // seg.init(nh,private_nh);

  ros::spin();
  return 0;

}
