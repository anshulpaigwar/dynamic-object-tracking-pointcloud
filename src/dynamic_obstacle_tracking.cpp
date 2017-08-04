
#include <dynamic_obstacle_tracking/dynamic_obstacle_tracking.hpp>

using namespace datmo;
using namespace pcl::octree;



// template< typename PointT >
// class OcclusionDetection: public pcl::octree::OctreePointCloudAdjacency<PointT>{

template< typename PointT,
        typename LeafContainerT = OctreePointCloudAdjacencyContainer <PointT>,
        typename BranchContainerT = OctreeContainerEmpty >
class OcclusionDetection: public pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>{

public:

    OcclusionDetection(const double resolution_arg);
    ~OcclusionDetection(){};

    //
    //template< typename LeafContainerT, typename BranchContainerT>
    float testForOcclusion2 (const PointT& point_arg, const pcl::PointXYZ& camera_pos)
    //bool testForOcclusion2 (const PointT& point_arg, const pcl::PointXYZ& camera_pos)
       {
         OctreeKey key;
         this->genOctreeKeyforPoint (point_arg, key);
         // This code follows the method in Octree::PointCloud
         Eigen::Vector3f sensor(camera_pos.x,
                                camera_pos.y,
                                camera_pos.z);

        Eigen::Vector3f leaf_centroid(static_cast<float> ((static_cast<double> (key.x) + 0.5f) * this->resolution_ + this->min_x_),
                                       static_cast<float> ((static_cast<double> (key.y) + 0.5f) * this->resolution_ + this->min_y_),
                                       static_cast<float> ((static_cast<double> (key.z) + 0.5f) * this->resolution_ + this->min_z_));
         Eigen::Vector3f direction = sensor - leaf_centroid;

        float norm = direction.norm ();
        direction.normalize ();
         float precision = 1.0f;
         const float step_size = static_cast<const float> (resolution_) * precision;
         const int nsteps = std::max (1, static_cast<int> (norm / step_size));

         OctreeKey prev_key = key;
         // Walk along the line segment with small steps.
         Eigen::Vector3f p = leaf_centroid;
         PointT octree_p;
         for (int i = 0; i < nsteps; ++i)
         {
           //Start at the leaf voxel, and move back towards sensor.
           p += (direction * step_size);

           octree_p.x = p.x ();
           octree_p.y = p.y ();
           octree_p.z = p.z ();
           //  std::cout << octree_p<< "\n";
           OctreeKey key;
           this->genOctreeKeyforPoint (octree_p, key);

           // Not a new key, still the same voxel (starts at self).
           if ((key == prev_key))
             continue;

           prev_key = key;

           LeafContainerT *leaf = this->findLeaf (key);
           //If the voxel is occupied, there is a possible occlusion
           if (leaf)
           {
            float occlusion_distance = sqrt(SQR(leaf_centroid.x()-p.x()) + SQR(leaf_centroid.y()-p.y()) + SQR(leaf_centroid.z()-p.z()));
            //return true;
            return occlusion_distance;

           }
         }

         //If we didn't run into a voxel on the way to this camera, it can't be occluded.
         //return false;
         return -1;

       }

private:
    float resolution_;

};


template< typename PointT,
        typename LeafContainerT,
        typename BranchContainerT  >
OcclusionDetection<PointT, LeafContainerT, BranchContainerT>::OcclusionDetection (const double resolution_arg)
: OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT > (resolution_arg)
 {
     resolution_ = resolution_arg;
 }




















cloud_segmentation::cloud_segmentation(){};
cloud_segmentation::~cloud_segmentation(){};



void cloud_segmentation::init(ros::NodeHandle &nh, ros::NodeHandle &private_nh){

    cloud_transformed = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    ref_obj_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    filtered_dynamic_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
    classified_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
    occuluded_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
    new_discovered_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());

    odom_transform_matrix = Eigen::Affine3d::Identity();
    icp_transform_matrix = Eigen::Affine3f::Identity();



    sub = nh.subscribe ("/cloud", 1, &cloud_segmentation::cloud_cb,this);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/transformed_points", 1);
    pub_2 = nh.advertise<sensor_msgs::PointCloud2> ("/dynamic_points", 1);
    pub_3 = nh.advertise<sensor_msgs::PointCloud2> ("/obj_cloud", 1);
    //marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    centroids_pub = nh.advertise< geometry_msgs::PoseArray >("/object_centroids", 1, true);
    counter = 0;

}




void cloud_segmentation::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    //pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    obj_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());



    //std::vector<Float3> centroids;
    pcl::PointCloud<pcl::PointXYZ> centroids;
    pcl::PointXYZRGB point;
    pcl::PointXYZ pt;

    // important ---> Convert to PCL data type from ros msg type    to pcl/PointCloud<T>  and not the pcl/PointCloud2
    pcl::fromROSMsg (*cloud_msg, *cloud);
    //std::cout << cloud->header.frame_id << '\n';
    // transform the point clould
    cloud_transform(cloud, cloud_transformed);

    cloud->clear();
    obj_cloud->clear();
    ref_obj_cloud->clear();
    filtered_dynamic_cloud->clear();
    classified_cloud->clear();
    occuluded_cloud->clear();
    new_discovered_cloud->clear();

    // filter the cloud to specific dimensions in x y and z directions
    cloud_filter(cloud_transformed,"y",PASS_Y_MIN,PASS_Y_MAX);
    cloud_filter(cloud_transformed,"x",PASS_X_MIN,PASS_X_MAX);
    cloud_filter(cloud_transformed,"z",PASS_Z_MIN,PASS_Z_MAX);

    // this will store the extreme coordinates (4 points) of our working area so as to transform them
    pcl::PointCloud<pcl::PointXYZ>::Ptr bounding_area_coord (new pcl::PointCloud<pcl::PointXYZ> ());
    pt.x = PASS_X_MIN; pt.y = PASS_Y_MAX; pt.z = 0;
    bounding_area_coord->push_back(pt);
    pt.x = PASS_X_MAX; pt.y = PASS_Y_MAX; pt.z = 0;
    bounding_area_coord->push_back(pt);
    pt.x = PASS_X_MAX; pt.y = PASS_Y_MIN; pt.z = 0;
    bounding_area_coord->push_back(pt);
    pt.x = PASS_X_MIN; pt.y = PASS_Y_MIN; pt.z = 0;
    bounding_area_coord->push_back(pt);




    if(ENABLE_VOXELISE){
        Voxel_filter(cloud_transformed, cloud_transformed, VOXEL_LEAF_SIZE);
    }




    if(ENABLE_SOR){
        SOR_filter(cloud_transformed, obj_cloud, SOR_MEAN_K, SOR_STD_DEV_MUL_THRESH);
        //ROS_INFO_STREAM("Time After SOR: " << (ros::Time::now().toSec()- start_time));
    }



    // ...........................Ground plane extraction using Ransac_plane fitting................................//


    if(ENABLE_GROUND_REMOVAL){
        Ransac_plane(cloud_transformed,obj_cloud); // we try to fit a ransac plane to data point and get the coefficients of plane.
        //ROS_INFO_STREAM("Time After Voxelgrid: " << (ros::Time::now().toSec()- start_time));
    }



    // euclideanclustering just to remove outliers i.e very small group of obstacles
    euclideancluster(obj_cloud, SEG_CLUSTER_TOLERANCE, SEG_MIN_CLUSTER_SIZE, SEG_MAX_CLUSTER_SIZE);





     //find the current transfom wrt world frame (/odom)
    try{
      listener.waitForTransform(frame_id, base_frame_id,
                                ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform(frame_id, base_frame_id,
                               ros::Time(0), current_cloud_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }


     //std::cout << "obj_Clouds" <<obj_cloud->size()<< '\n';

    if(counter < OCTREE_WINDOW){
          // save all the object clouds in the vector
          sourceClouds.push_back(obj_cloud);
          // save the corresponding transform in the vector
  	      sourceTransforms.push_back(current_cloud_transform);
          counter++;
    }

    else if(obj_cloud->size()!= 0){

        //std::cout << "sourceClouds" <<sourceClouds.size()<< '\n';

            //............................................intergrate the odometry..........................................//

            ref_obj_cloud = sourceClouds.front();
            ref_cloud_transform = sourceTransforms.front();//transform of pointcloud octree_window times ago
            prev_cloud_transform = sourceTransforms.back();// transform of pointcloud  just one previous time step


            //std::cout << "obj_Clouds" <<obj_cloud->size()<< '\n';
            //std::cout << "reference obj_Clouds" <<ref_obj_cloud->size()<< '\n';

        	//relative pose between pointcloud at current time step and octree_window times previous pointcloud
        	tf::Transform twist = current_cloud_transform.inverseTimes(ref_cloud_transform);
            //convert TF to Eigen transform matrix
        	tf::transformTFToEigen(twist,odom_transform_matrix );

            // std::cout << "" << '\n';
            // std::cout << "relative _transform" << '\n';
            // cout<< odom_transform_matrix.matrix() <<endl;

            //transform previous times object cloud to the current frame of reference
            pcl::transformPointCloud (*ref_obj_cloud, *ref_obj_cloud, odom_transform_matrix);
            // find the new bounding_area polygon coordinates


            pcl::transformPointCloud (*bounding_area_coord, *bounding_area_coord, odom_transform_matrix);



            if(ENABLE_ICP){
                integrateOdom_ICP(obj_cloud, ref_obj_cloud);
            }


            // transform between pointcloud at current time step and just one previous time step
            tf::Transform twist2 = current_cloud_transform.inverseTimes(prev_cloud_transform);
            br.sendTransform(tf::StampedTransform(twist2, ros::Time::now(), "/latest_centroids", "/prev_centroids"));





            //std::cout << "here1" << '\n';

            //........................................Spatial change detection......................................................//
            // concept to understand is for spatial change detection we are comparing current pointcloud with maybe
            // 4-5 timestep previous pointcloud but this is happening at every time we get new data (10hz)so centroids are
            // are being published at every time step so for kalman filter we only need transform current and last time step
            //
            //std::cout << "ENABLE_OCCLUSION_DETECTION"<< ENABLE_OCCLUSION_DETECTION << '\n';
            //std::cout << "ENABLE_VOXELISE"<< ENABLE_VOXELISE << '\n';

            detect_spatial_change (obj_cloud, ref_obj_cloud, dynamic_cloud, occuluded_cloud, OCTREE_RESOLUTION, ENABLE_OCCLUSION_DETECTION);









            //.......................................eucledian cluster dynamic cloud to remove further outliers.....................//


            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB> ());

            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (dynamic_cloud);
            //pcl::PointXYZRGB point;

            // Float3 cg;
            pcl::PointXYZ cg;

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (SEG_CLUSTER_TOLERANCE); // 2cm
            ec.setMinClusterSize (SEG_MIN_CLUSTER_SIZE);
            ec.setMaxClusterSize (SEG_MAX_CLUSTER_SIZE);
            ec.setSearchMethod (tree);
            ec.setInputCloud (dynamic_cloud);
            ec.extract (cluster_indices);

            int num_clusters = cluster_indices.size();

            for (std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
        	    int index = std::distance( cluster_indices.begin(), it );
        	    cluster->clear();
                Eigen::Vector4f centroid;
        	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){

            		point.x = dynamic_cloud->points[*pit].x ;
            		point.y = dynamic_cloud->points[*pit].y ;
            		point.z = dynamic_cloud->points[*pit].z ;

                    // check if the dynamic points that detected actualy lie inside the bounding area means it is not in newly found area
                    if (is_in_bounding_area(bounding_area_coord,point)){

                                point.r = 255;
                                point.g = 0;
                                point.b = 0;
                                cluster->push_back(point);
                    }
                    else
                    {
                        point.r = 0;
                        point.g = 0;
                        point.b = 255;
                        new_discovered_cloud->push_back(point);
                    }


        	    }

                pcl::compute3DCentroid (*cluster, centroid);
                cg.x = centroid[0];
                cg.y = centroid[1];
                cg.z = centroid[2];
                centroids.push_back(cg);

        	    //cluster->header.frame_id = "/Sensor1";
        	    //mark_cluster(cluster, "dynamic", index, 0.f, 255, 0.f);
        	    *filtered_dynamic_cloud += *cluster;

            }


            // this is for visualistaion
            *classified_cloud += *occuluded_cloud + *new_discovered_cloud;
            *classified_cloud += *filtered_dynamic_cloud;


            // std::cout << classified_cloud->size() << '\n';


            // Convert to ROS data type
            sensor_msgs::PointCloud2 output;
            sensor_msgs::PointCloud2 output_2;
            //sensor_msgs::PointCloud2 output_3;
            pcl::toROSMsg(*cloud_transformed, output);
            pcl::toROSMsg(*classified_cloud, output_2);
            //pcl::toROSMsg(*obj_cloud, output_3);
            // pcl::toROSMsg(*obj_cloud, output);
            // pcl::toROSMsg(*ref_obj_cloud, output_2);

            output.header.frame_id = base_frame_id;
            output_2.header.frame_id = base_frame_id;
            //output_3.header.frame_id = base_frame_id;


            // Publish the data
            pub.publish (output);
            pub_2.publish (output_2);
            //pub_3.publish (output_3);

            geometry_msgs::PoseArray obj_poses;
            geometry_msgs::Pose obj_pose;

            // Go ahead and fill out rviz arrow message
            obj_poses.header.stamp = ros::Time::now();
            obj_poses.header.frame_id = base_frame_id;


            // std::cout << "new centroids" << '\n';
            for(pcl::PointCloud<pcl::PointXYZ>::iterator it = centroids.begin(); it != centroids.end(); ++it ){
                obj_pose.position.x = it->x;
                obj_pose.position.y = it->y;
                obj_pose.position.z = it->z;
                obj_poses.poses.push_back(obj_pose);
                // std::cout << it->x <<" "<<it->y<< '\n';
            }


            centroids_pub.publish(obj_poses);




        // while (marker_pub.getNumSubscribers() < 1)
        // {
        //   if (!ros::ok())
        //   {
        //     //return 0;
        //   }
        //   ROS_WARN_ONCE("Please create a subscriber to the marker");
        //   sleep(1);
        // }


        // marker_pub.publish(marker_array);
        // marker_array.markers.clear();



        // if(obj_poses.poses.size() > 0)
        // {
        //   block_pose_pub_.publish(obj_poses);
        //   ROS_INFO_STREAM("Found " << obj_poses.poses.size() << " blocks this iteration");
        // }
        // else
        // {
        //   ROS_INFO("Couldn't find any blocks this iteration!");
        // }


        //first_time = false;



            sourceClouds.erase(sourceClouds.begin());
            sourceClouds.push_back(obj_cloud);
            sourceTransforms.erase(sourceTransforms.begin());
            sourceTransforms.push_back(current_cloud_transform);


    }

}






















//....................Function definations.........................//


void cloud_segmentation::cloud_transform (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out){

    tf::StampedTransform TF;
    ros::Time now = ros::Time::now();
    //ros::Time(0)
    try{
      listener.waitForTransform(    base_frame_id, sensor_frame_id,
                              ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform(    base_frame_id, sensor_frame_id,
                               ros::Time(0) , TF);
    //   listener.waitForTransform(   sensor_frame_id, base_frame_id,
    //                           ros::Time(0), ros::Duration(3.0));
    //   listener.lookupTransform(   sensor_frame_id, base_frame_id,
    //                            ros::Time(0) , TF);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      std::cout << "here is the error" << '\n';
      ros::Duration(1.0).sleep();
    //   return;
    }

    Eigen::Affine3d transform_matrix;

    tf::transformTFToEigen(TF,transform_matrix);

    //cout<< transform_matrix.matrix() <<endl;

    pcl::transformPointCloud(*cloud_in, *cloud_out,transform_matrix);

}







void cloud_segmentation::cloud_filter (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, const string field, float min, float max){

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName (field);
    pass.setFilterLimits (min, max);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_in);

}






void cloud_segmentation::euclideancluster  (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                            float SEG_CLUSTER_TOLERANCE,
                                            int SEG_MIN_CLUSTER_SIZE,
                                            int SEG_MAX_CLUSTER_SIZE){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ> ());
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    pcl::PointXYZ point;


    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.4); // 2cm
    ec.setMinClusterSize (20);
    ec.setMaxClusterSize (4000);
    // ec.setClusterTolerance (SEG_CLUSTER_TOLERANCE); // 2cm
    // ec.setMinClusterSize (SEG_MIN_CLUSTER_SIZE);
    // ec.setMaxClusterSize (SEG_MAX_CLUSTER_SIZE);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int num_clusters = cluster_indices.size();


    srand (time(NULL));

    for (std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){

    	    point.x = cloud->points[*pit].x ;
    	    point.y = cloud->points[*pit].y ;
    	    point.z = cloud->points[*pit].z ;
    	    cluster->push_back(point);
    	}
    }

    *cloud = *cluster;
}




void cloud_segmentation::Voxel_filter ( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                                        float VOXEL_LEAF_SIZE){

    //removing NaN values and converting organized cloud to unorganized
    std::vector<int> nan_filter_indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, nan_filter_indices);

    // Perform the Voxelization filtering
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> vog;
    vog.setInputCloud (cloud_out);
    vog.setLeafSize (VOXEL_LEAF_SIZE,VOXEL_LEAF_SIZE,VOXEL_LEAF_SIZE);
    vog.filter (*cloud_out);

}


void cloud_segmentation::SOR_filter (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                                     float SOR_MEAN_K,
                                     float SOR_STD_DEV_MUL_THRESH ){

    // Applying the statistical outlier removal filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_in);
    sor.setMeanK (SOR_MEAN_K);
    sor.setStddevMulThresh (SOR_STD_DEV_MUL_THRESH);
    sor.filter (*cloud_out);

}



void cloud_segmentation::
Ransac_plane (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out){

    // coefficients for ground plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p2 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointXYZ pt;

    // std::cout << "ransac1" << '\n';

    // filter the cloud again to just encorporate ground mostly
    for(int i = 0; i<cloud_in->size(); i++ ){
    	if(cloud_in->points[i].y < 15
    	&& cloud_in->points[i].y > -15
    	&& cloud_in->points[i].x < 30
    	&& cloud_in->points[i].x > 0
    	&& cloud_in->points[i].z < 0.5){
    	    cloud_p->push_back(cloud_in->points[i]);
    	}
    }

    // cout<<cloud_p->size()<<endl;

    srand (time(NULL));

    for (int i = 0; i<500; ++i){
	int num  = rand() % (cloud_p->size() -1);
	cloud_p2->push_back(cloud_p->points[num]);
    }

    // cout<<cloud_p2->size()<<endl;



    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.2);
    seg.setMaxIterations (15);
    seg.setInputCloud (cloud_p2);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }


    //here we classify points that belong to ground and non ground
    //cloud_classified.clear()

    float dist; /// distance from ransac plane
    float normaliser;
    normaliser = sqrt(pow(coefficients->values[0],2) + pow(coefficients->values[1],2) + pow(coefficients->values[2],2));


    for(int i = 0; i<cloud_in->size(); i++ ){

        pt.x = cloud_in->points[i].x;
        pt.y = cloud_in->points[i].y;
        pt.z = cloud_in->points[i].z;
        //dist of point from plane
        dist = abs((coefficients->values[0] * pt.x + coefficients->values[1] * pt.y + coefficients->values[2] * pt.z +coefficients->values[3])/(float)normaliser);

        // all points below zero height are considered to be ground in our case which is wrong but works for our case
        // as we assume that there are no slopes in indoor warehouse enviornment

        if (dist>0.2 && pt.z > 0){

            cloud_out->push_back(pt); // obj_cloud contains only objects (static and dynamic both)
        }
    }


}











void cloud_segmentation:: detect_spatial_change (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_now,
                                                const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_to_compare,
                                                const pcl::PointCloud<pcl::PointXYZ>::Ptr& dynamic_cloud,
                                                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& occuluded_cloud,
                                                float OCTREE_RESOLUTION,
                                                bool ENABLE_OCCLUSION_DETECTION){

    // Octree resolution - side length of octree voxels
    //float resolution = 0.4f;


    pcl::PointXYZRGB point;
    pcl::PointXYZ pt;
    pcl::PointXYZ camera_pos;

    camera_pos.x = 0;
    camera_pos.y = 0;
    camera_pos.z = 0;

    // Instantiate octree-based point cloud change detection class
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (OCTREE_RESOLUTION);

    // Add points from cloudA to octree
    octree.setInputCloud (cloud_to_compare); //compare the current pointcloud octree with pointcloud octree_window times ago
    octree.addPointsFromInputCloud ();

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree.switchBuffers ();

    // Add points from currentcloud to octree
    octree.setInputCloud (cloud_now);
    octree.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;

    // Get vector of point indices from octree voxels which did not exist in previous buffer
    octree.getPointIndicesFromNewVoxels (newPointIdxVector);


    //std::cout <<"cloud_to_compare"<< cloud_to_compare->size() << '\n';
    //std::cout << "cloud_now"<<cloud_now->size() << '\n';

    //....................Check if the new dectected points(dynamic points) were occuluded in in previous time step.............//

    // resolution = 0.2f;

    if(ENABLE_OCCLUSION_DETECTION){

        // // Instantiate octree-based point cloud adjacency class
        // pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZ> octree_a (OCTREE_RESOLUTION);
        //
        // // Add points from prev cloud to octreeobj_cloud
        // octree_a.setInputCloud (cloud_to_compare);
        // octree_a.addPointsFromInputCloud ();
        //
        // // check if dynamic points were occuluded
        //
        // for (size_t i = 0; i < newPointIdxVector.size (); ++i){
        //
        //     pt = cloud_now->points[newPointIdxVector[i]];
        //     //filtered_dynamic_cloud->push_back(pt);
        //     // if ((pt.y < 4) && (pt.y > -4) &&  (pt.x < 10)){
        //     //     dynamic_cloud->push_back(pt);
        //     // }
        //     // else
        //     if(!(octree_a.testForOcclusion(pt))){
        //         dynamic_cloud->push_back(pt);
        //     }
        //     else{
        //         point.x = pt.x;
        //         point.y = pt.y;
        //         point.z = pt.z;
        //         point.r = 0;
        //         point.g = 255;
        //         point.b = 0;
        //         occuluded_cloud->push_back(point);
        //     }
        // }


        // Instantiate octree-based point cloud adjacency class
        OcclusionDetection<pcl::PointXYZ> octree_a (OCTREE_RESOLUTION);

        // Add points from prev cloud to octreeobj_cloud
        octree_a.setInputCloud (cloud_to_compare);
        octree_a.addPointsFromInputCloud ();

        // check if dynamic points were occuluded

        for (size_t i = 0; i < newPointIdxVector.size (); ++i){

            pt = cloud_now->points[newPointIdxVector[i]];
            //filtered_dynamic_cloud->push_back(pt);
            // if ((pt.y < 4) && (pt.y > -4) &&  (pt.x < 10)){
            //     dynamic_cloud->push_back(pt);
            // }
            if((octree_a.testForOcclusion2(pt,camera_pos) < 2)){
                dynamic_cloud->push_back(pt);
            }
            else{
                point.x = pt.x;
                point.y = pt.y;
                point.z = pt.z;
                point.r = 0;
                point.g = 255;
                point.b = 0;
                occuluded_cloud->push_back(point);
            }
        }


    }

    else{

        for (size_t i = 0; i < newPointIdxVector.size (); ++i){

            pt = cloud_now->points[newPointIdxVector[i]];
            dynamic_cloud->push_back(pt);
        }
    }



}









// bool cloud_segmentation::is_in_bounding_area(const pcl::PointCloud<pcl::PointXYZ>::Ptr& coord, pcl::PointXYZRGB pt ){
//     float p21[] = {coord->points[2].x - coord->points[1].x, coord->points[2].y -  coord->points[1].y};
//     float p41[] = {coord->points[4].x - coord->points[1].x, coord->points[4].y -  coord->points[1].y};
//
//     float p21magnitude_squared = SQR(p21[0]) + SQR(p21[1]);
//     float p41magnitude_squared = SQR(p41[0]) + SQR(p41[1]);
//     float p[] = {pt.x - coord->points[1].x, pt.y - coord->points[1].y};
//     float dot_product21 = p[0] * p21[0] + p[1] * p21[1];
//     float dot_product41 = p[0] * p41[0] + p[1] * p41[1];
//         if( (0 <= dot_product21) && (dot_product21 <= p21magnitude_squared)){
//
//             if( (0 <= dot_product41) &&  (dot_product21 <= p41magnitude_squared)){
//                 return true;
//             }
//             else{
//                 return false;
//             }
//         }
//         else{
//             return false;
//         }
//
// }


bool cloud_segmentation::is_in_bounding_area(const pcl::PointCloud<pcl::PointXYZ>::Ptr& coord, pcl::PointXYZRGB pt ){
    // vector<Point> points = polygon.getPoints();
    int i, j, nvert = coord->size();
    bool c = false;

    for(i = 0, j = nvert - 1; i < nvert; j = i++) {
      if( ( (coord->points[i].y > pt.y ) != (coord->points[j].y > pt.y) ) &&
          (pt.x <= (coord->points[j].x - coord->points[i].x) * (pt.y - coord->points[i].y) / (coord->points[j].y - coord->points[i].y) + coord->points[i].x)
        )
        c = !c;
    }

    return c;
}







void cloud_segmentation::integrateOdom_ICP (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& obj_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& ref_obj_cloud ){

    pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(ref_obj_cloud);
    icp.setInputTarget(obj_cloud);




    //Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.5);
    //
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (10);
    // Set the transformation epsilon (criterion 2)
    // icp.setTransformationEpsilon (1e-8);

    // Set the euclidean distance difference epsilon (criterion 3)
    //icp.setEuclideanFitnessEpsilon (1);

    icp.align(*Final);
    //std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
    *ref_obj_cloud = *Final;

    // Obtain the transformation that aligned cloud_source to cloud_source_registered and store it into affine3f transform object
    // icp_transform_matrix.matrix() = icp.getFinalTransformation ();
    // std::cout << icp.getFinalTransformation ()<< '\n';
}






/*

void cloud_segmentation::integrateOdom (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud){


    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/odom_tf", "/base_link",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }


    Eigen::Affine3d transform_matrix;

    tf::transformTFToEigen(transform,transform_matrix);

    //cout<< transform_matrix.matrix() <<endl;


    pcl::transformPointCloud(*cloud, *cloud_transformed,transform_matrix);
}




void cloud_segmentation::integrateOdom (std::vector<Float3>& prev_centroids){

    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/odom_tf", "/base_link",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    Float3 odom_twist;
    odom_twist.x = transform.getOrigin().x();
    odom_twist.y = transform.getOrigin().y();
    odom_twist.z = tf::getYaw(transform.getRotation());

    cout<<odom_twist.x<<endl;

    float st = sin(odom_twist.z);
    float ct = cos(odom_twist.z);

    for (int i = 0; i < prev_centroids.size(); i++){
	prev_centroids[i].x = ct * prev_centroids[i].x - st * prev_centroids[i].y + odom_twist.x;
	prev_centroids[i].y = st * prev_centroids[i].x + ct * prev_centroids[i].y + odom_twist.y;
    }

}











void cloud_segmentation::integrateOdom (pcl::PointCloud<pcl::PointXYZ> &last_centroids){

    tf::Transform twist = current_cloud_transform.inverseTimes(prev_cloud_transform);

    br.sendTransform(tf::StampedTransform(twist, ros::Time::now(), "/latest_centroids", "/prev_centroids"));

    Eigen::Affine3d transform_matrix;

    tf::transformTFToEigen(twist,transform_matrix);

    //cout<< transform_matrix.matrix() <<endl;
    pcl::transformPointCloud (last_centroids, last_centroids, transform_matrix);
}







*/
















/*

void cloud_segmentation::euclideancluster (const pcl::PointCloud<pcl::PointXYZ>::Ptr& obj_cloud,
					    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster_cloud,
					    std::vector<Float3> centroids){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB> ());

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (obj_cloud);
    pcl::PointXYZRGB point;

    Float3 cg;


    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.4); // 2cm
    ec.setMinClusterSize (40);
    ec.setMaxClusterSize (4000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (obj_cloud);
    ec.extract (cluster_indices);

    int num_clusters = cluster_indices.size();


    srand (time(NULL));

    for (std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
	int index = std::distance( cluster_indices.begin(), it );
	int num = rand() % 255;
	cluster->clear();


	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){

	    point.x = obj_cloud->points[*pit].x ;
	    point.y = obj_cloud->points[*pit].y ;
	    point.z = obj_cloud->points[*pit].z ;
	    point.r = 255;
	    point.g = 0;
	    point.b = 0;
	    cluster->push_back(point);

	    cg.x += point.x;
	    cg.y += point.y;
	    cg.z += point.z;
	}

	//cluster->header.frame_id = "/velodyne";
	//mark_cluster(cluster, "dynamic", index, 0.f, 255, 0.f);
	*cluster_cloud += *cluster;
	cg.x = cg.x / it->indices.size();
	cg.y = cg.y / it->indices.size();
	cg.z = cg.z / it->indices.size();
	centroids.push_back(cg);
    }

}

*/









/*

void cloud_segmentation::mark_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, std::string ns ,int id, float r, float g, float b){
  Eigen::Vector4f centroid;
  pcl::PointXYZRGB min_p, max_p;

  pcl::compute3DCentroid (*cluster, centroid);
  pcl::getMinMax3D (*cluster, min_p, max_p);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = cluster->header.frame_id;
  //marker.header.frame_id = "/velodyne";
  marker.header.stamp = ros::Time::now();

  marker.ns = ns;
  marker.id = id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = centroid[0];
  marker.pose.position.y = centroid[1];
  marker.pose.position.z = centroid[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = (max_p.x - min_p.x);
  marker.scale.y = (max_p.y - min_p.y);
  marker.scale.z = (max_p.z - min_p.z);

  if (marker.scale.x ==0)
      marker.scale.x=0.1;

  if (marker.scale.y ==0)
    marker.scale.y=0.1;

  if (marker.scale.z ==0)
    marker.scale.z=0.1;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration(1);
  // marker.lifetime = ros::Duration(0.5);

  marker_array.markers.push_back(marker);


}
*/
