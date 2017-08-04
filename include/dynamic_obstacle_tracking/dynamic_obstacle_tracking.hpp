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
#include <pcl/octree/octree_key.h>
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
	    ros::Publisher pub_3; ///< Publish pointcloud with dynamic obstacles on topic /dynamic_points
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

        float SEG_CLUSTER_TOLERANCE; ///< Maximum distance between two points to be included in same cluster
        int SEG_MIN_CLUSTER_SIZE; ///< Minimum number of points that should be present within a cluster
        int SEG_MAX_CLUSTER_SIZE; ///< Maximum number points to be included in same cluster





	private:
        /**
         * Callback function get called every time a new point cloud is recieved
         * @param[in] cloud_msg constant pointer to sensor_msgs PointCloud2
         */
	    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);


        /**
         * Transforms the point cloud from sensor_frame_id to base_link
         * This function contains a TF listener
         * @param[in] cloud_in
         * @param[out] cloud_out
         */
	    void cloud_transform (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

        /**
         * filters the pointcloud wiithin given dimensions and field
         * @param[in,out] cloud_in ponitcloud to be filtered
         * @param[in] field    axis name ("x" or "y" or "z")
         * @param[in] min      minimum value of points in given field
         * @param[in] max      maximum value of points in given field
         */
	    void cloud_filter (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, const string field, float min, float max);

        /**
         * This function downsamples a PointCloud that is, reduce the number of points – a point cloud dataset, using a voxelized grid approach
         * For more information follow the link http://pointclouds.org/documentation/tutorials/voxel_grid.php
         * Also it removes any nan values in pointcloud
         * This filter is neccessary for the dense PointCloud generated by stereo camera like ZED stereo camera and RGBD camera like ZR300
         * PointCloud generated by 3D Lidars like Velodyne and Quanergy are very sparse and hence is disable for them by default.
         * @param[in] cloud_in
         * @param[out] cloud_out
         * @param[in] VOXEL_LEAF_SIZE
         */

        void Voxel_filter ( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                            float VOXEL_LEAF_SIZE);
        /**
         * StatisticalOutlierRemoval filter is used to remove sparse points generated due to noise
         * for more information follow the link: http://pointclouds.org/documentation/tutorials/statistical_outlier.php
         * SOR filter is required for ZR300 pointcloud data and is disables for Velodyne pointcloud
         * @param[in] cloud_in
         * @param[out] cloud_out
         * @param[in] SOR_MEAN_K
         * @param[in] SOR_STD_DEV_MUL_THRESH
         */
        void SOR_filter (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                         float SOR_MEAN_K,
                         float SOR_STD_DEV_MUL_THRESH );


        /**
         * This function uses Ransac plane fitting algorithm to find all the points within a point cloud that support a plane model
         * in our case we aim to find a Ground plane and remove all the points belonging to Ground so that we are only left with points
         * belonging to obstacles. For more information follow link: http://pointclouds.org/documentation/tutorials/planar_segmentation.php
         *
         * In this function we filter out the point cloud above certain height so that the most dominating plane to be fit by Ransac is Ground Plane.
         * Since the we have large number of data points using Ransac to fit all of them would be computationally expensive so we only try fit plane
         * to point in surrounding area of our vehicle here x(0m to 30m) y(-15m to 15m) and z(less than 0.5m). Accuracy and computational time can be changed
         * by tweaking these parameters.
         *
         * Also for further reducing computational time we randomly choose 500 points and fit Ransac plane to them only....!
         *
         * For ZR300 point cloud generated do not contain much points on ground hence Ransac_plane fitting is not neccessary. Putting a simple cap on height
         * and neglecting all the point below that suffice our purpose. So Ground plane estimation is disabled by default.
         *
         * For Pointclouds generated by Velodyne Lidars significant number of point s fall on ground plane and hence properly removing them is
         * neccessary hence Ground Plane estimation is enabled by default.
         *
         * @param cloud_in  PointCloud in which ground plane is to be estimated
         * @param cloud_out PointCloud containing only obstacles (after removing ground points from original PointCloud)
         */

        void Ransac_plane (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
	    //void euclideancluster (const pcl::PointCloud<pcl::PointXYZ>::Ptr& obj_cloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster_cloud, std::vector<Float3> centroids);

        /**
         * This function uses euclidean clustering algorithm to remove outliers. i.e remove random points or very small set of clusters.
         * for more information please follow the link:http://pointclouds.org/documentation/tutorials/cluster_extraction.php
         *
         * @param[in,out] cloud                 The input PointCloud is replaced by filtered PointCloud
         * @param[in] SEG_CLUSTER_TOLERANCE
         * @param[in] SEG_MIN_CLUSTER_SIZE
         * @param[in] SEG_MAX_CLUSTER_SIZE
         */
        void euclideancluster  (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                float SEG_CLUSTER_TOLERANCE,
                                int SEG_MIN_CLUSTER_SIZE,
                                int SEG_MAX_CLUSTER_SIZE);

        /**
         * This function is core for classifying between dynamic and static objects.
         * We used Octree based spatial change detection to find dynamic points more information follow link:  http://pointclouds.org/documentation/tutorials/octree_change.php
         * We compare the object cloud(obj_cloud) at current time step with object cloud "octree_window" times previous object cloud (ref_obj_cloud)
         * We are trying to find the new octree leaf that are created in current obj_cloud that were not present previously.
         *
         * Since the vehicle is moving Previous object cloud first has to be transformed to the frame of object cloud at current time step before pluging into this function
         *
         * OCTREE_WINDOW and OCTREE_RESOLUTION parameter can be used for tweaking dynamic object detection
         * For Example:
         * Velodyne data is published at 10 HZ, Keeping OCTREE_WINDOW = 5 and OCTREE_RESOLUTION = 0.4 means that and object would be detected as dynamic if it has moved minimum of 0.4m distancein half sec.
         *
         * zr300 data is published at 30 Hz , Keeping OCTREE_WINDOW = 15 and OCTREE_RESOLUTION = 0.4 means that and object would be detected as dynamic if it has moved minimum of 0.4m distancein half sec.
         *
         *
         *
         *
         * @param cloud_now                  [description]
         * @param cloud_to_compare           [description]
         * @param dynamic_cloud              [description]
         * @param occuluded_cloud            [description]
         * @param OCTREE_RESOLUTION          [description]
         * @param ENABLE_OCCLUSION_DETECTION Enables occlusion detection
         */

        void detect_spatial_change (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_now,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_to_compare,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& dynamic_cloud,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& occuluded_cloud,
                                    float OCTREE_RESOLUTION,
                                    bool ENABLE_OCCLUSION_DETECTION);
        /**
         * Since our vehicle is moving we keep exploring new areas. The points those detected in these new areas are classified as dynamic
         * by Spatial change detection algorithm because these points did not existed in previous time step i.e in reference cloud.
         *
         * Lets say coordinates of our bounding area is (PASS_X_MIN, PASS_Y_MAX) (PASS_X_MAX, PASS_Y_MAX) (PASS_X_MAX, PASS_Y_MIN) (PASS_X_MIN, PASS_Y_MIN)
         * as the vehicle moves forward we tranform this bounding area backwards accoridingly.
         *
         * Now every point that is detected as dynamic is checked if it lies in the bounding area. If not then it is classified as new detected point.
         *
         * To check whether the point lies within the polygon a ray casting algorithim is used.
         *
         * for more information to know about the algorithm follow link: http://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
         * @param[in]  coord PointCloud containing 4 points extreme co-ordinates of bounding area.
         * @param[in]  pt    point to be checked if it lies within bounding area
         * @return       true/false
         */
        bool is_in_bounding_area(const pcl::PointCloud<pcl::PointXYZ>::Ptr& coord, pcl::PointXYZRGB pt );


	    //void mark_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b);
	    void integrateOdom_ICP (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& obj_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& ref_obj_cloud );
	    //void integrateOdom ( pcl::PointCloud<pcl::PointXYZ> &last_centroids);


        //void integrateOdom (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
	    //void integrateOdom (std::vector<Float3>& last_centroids);
        //void dynamic_reconfigure_cb(datmo::datmoConfig &config, uint32_t level);
	    //void odom_cb (const nav_msgs::Odometry odom_msg);
    };

}
//#endif // __DYNAMIC_OBSTACLE_TRACKING_HPP__
