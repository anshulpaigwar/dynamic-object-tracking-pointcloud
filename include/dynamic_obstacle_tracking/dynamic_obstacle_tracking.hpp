//#ifndef __DYNAMIC_OBSTACLE_TRACKING_HPP__
//#define __DYNAMIC_OBSTACLE_TRACKING_HPP__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <dynamic_reconfigure/server.h>


// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
//#include <pcl/filters/voxel_grid.h>
//#
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>




#define PI 3.14159265
#define SQR(a) ((a)*(a))

using namespace Eigen;
using namespace std;

namespace datmo
{

struct Float3{
   float x;
   float y;
   float z;
};


    class cloud_segmentation{

	public:
	    cloud_segmentation();
	    ~cloud_segmentation();
	    void init(ros::NodeHandle &nh, ros::NodeHandle &private_nh);

	protected:
	    ros::Publisher pub; /**< publish point cloud after transforming it to base_link*/
	    ros::Publisher pub_2;
	    ros::Publisher marker_pub;
        ros::Publisher centroids_pub;

        ros::Subscriber sub;

	    visualization_msgs::MarkerArray marker_array;
	    nav_msgs::Odometry odom;


	    tf::TransformListener listener;
        tf::TransformBroadcaster br;


	    tf::StampedTransform prev_transform;
        tf::StampedTransform current_transform;
        tf::StampedTransform last_transform;


	    pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_transformed;
	    pcl::PointCloud<pcl::PointXYZ>:: Ptr prev_obj_cloud;
	    pcl::PointCloud<pcl::PointXYZRGB> cloud_classified;


	    vector<tf::StampedTransform> sourceTransforms;
	    vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud<pcl::PointXYZ>::Ptr > > sourceClouds;

	    pcl::PointCloud<pcl::PointXYZ> last_centroids;
	    Eigen::Affine3d odom_transform_matrix;
	    Eigen::Affine3f icp_transform_matrix;



	    int counter;
	    //pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_filter;
	    bool ENABLE_ICP;
        bool ENABLE_GROUND_REMOVAL;
        bool ENABLE_VOXELISE;
        bool ENABLE_OCCLUSION_DETECTION;

        float VOXEL_LEAF_SIZE;
        //Statistical outlier parameters
        bool ENABLE_SOR;
        float SOR_MEAN_K;              //number of neighbors to analyze for each point
        float SOR_STD_DEV_MUL_THRESH; //distance in multiples of std dev after which point is considered an outlier
        //Passthrough filter parameters

        //Passthrough filter parameters
        float PASS_X_MIN; /**< minimum value of x */
        float PASS_X_MAX ; ///< maximum value of x
        float PASS_Y_MIN;
        float PASS_Y_MAX;
        float PASS_Z_MIN;
        float PASS_Z_MAX;
        //plane segmentation params
        int OCTREE_WINDOW;
        float OCTREE_RESOLUTION;
        //segmentation params
        float SEG_CLUSTER_TOLERANCE;
        int SEG_MIN_CLUSTER_SIZE;
        int SEG_MAX_CLUSTER_SIZE;





	private:
        /**
         * Callback function get called every time a new point cloud is recieved
         * @param[in] cloud_msg constant pointer to sensor_msgs PointCloud2
         */
	    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);



	    void cloud_transform (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);


	    void cloud_filter (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, const string field, float min, float max);

        void Voxel_filter ( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                            float VOXEL_LEAF_SIZE);

        void SOR_filter (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                         float SOR_MEAN_K,
                         float SOR_STD_DEV_MUL_THRESH );

        void Ransac_plane (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
	    //void euclideancluster (const pcl::PointCloud<pcl::PointXYZ>::Ptr& obj_cloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster_cloud, std::vector<Float3> centroids);

        void euclideancluster  (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                float SEG_CLUSTER_TOLERANCE,
                                int SEG_MIN_CLUSTER_SIZE,
                                int SEG_MAX_CLUSTER_SIZE);

        void detect_spatial_change (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_now,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_to_compare,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& dynamic_cloud,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& occuluded_cloud,
                                    float OCTREE_RESOLUTION,
                                    bool ENABLE_OCCLUSION_DETECTION);
        /**
         * [is_in_bounding_area description]
         * @param  coord [description]
         * @param  pt    [description]
         * @return       [description]
         */
        bool is_in_bounding_area(const pcl::PointCloud<pcl::PointXYZ>::Ptr& coord, pcl::PointXYZRGB pt );

        
	    void mark_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b);
	    void integrateOdom_ICP (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& obj_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& prev_obj_cloud );
	    void integrateOdom ( pcl::PointCloud<pcl::PointXYZ> &last_centroids);


        //void integrateOdom (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
	    //void integrateOdom (std::vector<Float3>& last_centroids);
        //void dynamic_reconfigure_cb(datmo::datmoConfig &config, uint32_t level);
	    //void odom_cb (const nav_msgs::Odometry odom_msg);
    };

}
//#endif // __DYNAMIC_OBSTACLE_TRACKING_HPP__
