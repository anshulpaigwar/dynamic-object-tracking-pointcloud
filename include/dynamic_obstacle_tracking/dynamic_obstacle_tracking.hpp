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
	    ros::Publisher pub; ///< publish point cloud after transforming it to base_link on topic /transformed_points
	    ros::Publisher pub_2; ///< Publish pointcloud with dynamic obstacles on topic /dynamic_points
	    //ros::Publisher marker_pub;
        ros::Publisher centroids_pub; ///< Publish geometry_msgs::PoseArray centroids of dynamic obstacles  on topic /object_centroids

        ros::Subscriber sub;  ///< subscribes sensor_msgs::PointCloud2 from topic /cloud

	    //visualization_msgs::MarkerArray marker_array;
	    //nav_msgs::Odometry odom;


	    tf::TransformListener listener; ///< A TF listener
        tf::TransformBroadcaster br; ///< TF brodcaster to broadcast relative transfom between current pointcloud and prev PointCloud

        tf::StampedTransform current_cloud_transform; ///< Container to store transform of latest pointcloud wrt /odom
	    tf::StampedTransform ref_cloud_transform; ///< Container to store transform of reference pointcloud wrt /odom
        tf::StampedTransform prev_cloud_transform; ///< Container to store transform of previous pointcloud wrt /odom



        pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_transformed; ///< container to store current pointcloud after transforming it to /base_link frame from /sensor frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud; ///< container to store pointcloud having both static and dynamic objects
	    pcl::PointCloud<pcl::PointXYZ>:: Ptr ref_obj_cloud; ///< container to store object cloud to be used as reference for comparing with object cloud at current time step for octree based spatial change detection
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr occuluded_cloud; ///< container to store the points at current timestep that are in the areas which were occuluded in reference object cloud. (Green in colour)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_discovered_cloud; ///< Container to store points at current timestep that are in the areas which are newly discovered. (Blue in colour)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_dynamic_cloud; ///< Container to store filtered dynamic cloud (Red in colour) = dynamic_cloud - occuluded_cloud - new_discovered_cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr classified_cloud; ///< Container to store final classified cloud  = filtered_dynamic_cloud(Red) + occuluded_cloud(Green) + new_discovered_cloud(Blue)






	    vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud<pcl::PointXYZ>::Ptr > > sourceClouds; ///< vector to store the "OCTREE_WINDOW" number of obj_cloud in the memory
	    vector<tf::StampedTransform> sourceTransforms; ///< Vector to store corresponding Transforms of each PointCloud in sourceClouds /odom to /base_link
	    //pcl::PointCloud<pcl::PointXYZ> last_centroids;
	    Eigen::Affine3d odom_transform_matrix; ///< Container to store relative transform between obj_cloud and ref_obj_cloud
	    Eigen::Affine3f icp_transform_matrix;



	    int counter;
        string frame_id;
        string base_frame_id;
        string sensor_frame_id;
	    //pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_filter;
	    bool ENABLE_ICP; ///< Enable use of ICP to better match obj_cloud with ref_obj_cloud : Default it is disabled bcoz of bad results
        bool ENABLE_GROUND_REMOVAL; ///< Enable use of ransac plane fitting to remove ground points : Default it is disabled for ZR300 and enabled for Velodyne
        bool ENABLE_VOXELISE; ///< Enable voxel filter: Default it is enable for ZR300 and disabled for Velodyne
        bool ENABLE_OCCLUSION_DETECTION; ///< Enable Occlusion detection so as to prevent false dynamic obstacles detection due to shadow : Default it is enabled for ZR300 and enabled for Velodyne

        float VOXEL_LEAF_SIZE;
        //Statistical outlier parameters
        bool ENABLE_SOR; ///< Enable Statistical Outlier removal filter: Default it is enable for ZR300 and disabled for Velodyne
        float SOR_MEAN_K;              ///<number of neighbors to analyze for each point
        float SOR_STD_DEV_MUL_THRESH; ///<distance in multiples of std dev after which point is considered an outlier
        //Passthrough filter parameters

        float PASS_X_MIN;  ///< minimum value in x direction
        float PASS_X_MAX;  ///< maximum value in x direction
        float PASS_Y_MIN;  ///< minimum value in y direction
        float PASS_Y_MAX;  ///< maximum value in y direction
        float PASS_Z_MIN;  ///< minimum value in z direction
        float PASS_Z_MAX;  ///< maximum value in z direction


        int OCTREE_WINDOW; ///< This parameter defines that current pointcloud will be compared with  "OCTREE_WINDOW"  times previous pointcloud for spatial change detection
        float OCTREE_RESOLUTION; ///< smallest dimension of octree leaf node

        float SEG_CLUSTER_TOLERANCE;
        int SEG_MIN_CLUSTER_SIZE;
        int SEG_MAX_CLUSTER_SIZE;





	private:
        /**
         * Callback function get called every time a new point cloud is recieved
         * @param[in] cloud_msg constant pointer to sensor_msgs PointCloud2
         */
	    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);


        /**
         * Transforms the point cloud from sensor frame to base_link
         * @param[in] cloud_in  [description]
         * @param[out] cloud_out [description]
         */
	    void cloud_transform (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

        /**
         * filters the pointcloud wiithin given dimensions and field
         * @param[in,out] cloud_in [escription]
         * @param[in] field    [description]
         * @param[in] min      [description]
         * @param[in] max      [description]
         */
	    void cloud_filter (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, const string field, float min, float max);

        /**
         * [Voxel_filter  description]
         * @param[in] cloud_in        [description]
         * @param[out] cloud_out       [description]
         * @param[in] VOXEL_LEAF_SIZE [description]
         */

        void Voxel_filter ( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                            float VOXEL_LEAF_SIZE);
        /**
         * SOR filter is required for ZR300 pointcloud data and is disables for Velodyne pointcloud
         * @param cloud_in               [description]
         * @param cloud_out              [description]
         * @param SOR_MEAN_K             [description]
         * @param SOR_STD_DEV_MUL_THRESH [description]
         */
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
	    void integrateOdom_ICP (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& obj_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& ref_obj_cloud );
	    void integrateOdom ( pcl::PointCloud<pcl::PointXYZ> &last_centroids);


        //void integrateOdom (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
	    //void integrateOdom (std::vector<Float3>& last_centroids);
        //void dynamic_reconfigure_cb(datmo::datmoConfig &config, uint32_t level);
	    //void odom_cb (const nav_msgs::Odometry odom_msg);
    };

}
//#endif // __DYNAMIC_OBSTACLE_TRACKING_HPP__
