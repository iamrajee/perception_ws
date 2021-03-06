#include "tf_conversions/tf_eigen.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <fstream>
#include <iostream>
#include <limits>
#include <list>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/tracking.hpp>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <signal.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#define LEAF_SIZE 0.5
#define MAX_NUM_BBOX 20
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, sensor_msgs::Image,
    darknet_ros_msgs::BoundingBoxes>
    MySyncPolicy;
struct BBox {
  cv::MatND hist_bbox;
  cv::Mat bbox_image;
  ros::Time detection_time;
  int detection_index;
};
class DrawBbox {
public:
  DrawBbox() {
    frame_num = 0;
    nh_ = new ros::NodeHandle("~");
    // initialize all the parameters

    r = {0.015453, 0.010533, -0.999825, 0.863324, -0.504586,
         0.008027, 0.504413, 0.863297,  0.016891};
    t = {-0.098300, -0.024300, 0.024600};
    k = {424.256690, 0.000000, 0.000000, 0.0000, 407.246190,
         0.000,      0.000,    0.000,    1.0000};
    c = {239.500000, 319.500000};
    d = {-0.295620, 0.082190, 0.000540, -0.011990, 0.000000};

    nh_->param("in_cloud_topic", in_cloud_topic,
               std::string("/velodyne_points"));
    nh_->param("enable_tracking", enable_tracking, true);
    nh_->param("out_cloud_topic", out_cloud_topic, std::string("/bbox_cloud"));
    nh_->param("in_image_topic", in_image_topic,
               std::string("/camera0/image_raw"));
    nh_->param("label_file_path", label_file_path,
               std::string("/home/akhil/labels.txt"));
    nh_->param("depth_image_topic", depth_image_topic,
               std::string("/velodyne_depth_image"));
    nh_->param("lidar_frame_id", lidar_frame_id, std::string("/velodyne"));
    nh_->param("bbox2d_topic", bbox2d_topic,
               std::string("/darknet_ros/bounding_boxes"));
    nh_->param("marker_array_topic", marker_array_topic,
               std::string("/bbox3d_marker_array"));
    nh_->param("cloud_zmax_thresh", cloud_zmax_thresh, float(0));
    nh_->param("cloud_zmin_thresh", cloud_zmin_thresh, float(-0.9));
    nh_->getParam("R", r);
    nh_->getParam("T", t);
    nh_->getParam("K", k);
    nh_->getParam("C", c);
    nh_->getParam("D", d);
    R = cv::Mat(3, 3, CV_64FC1, r.data());
    T = cv::Mat(3, 1, CV_64FC1, t.data());
    K = cv::Mat(3, 3, CV_64FC1, k.data());
    C = cv::Mat(2, 1, CV_64FC1, c.data());
    D = cv::Mat(5, 1, CV_64FC1, d.data());
    label_file.open(label_file_path);

    std::cout << "using rotation matrix R = " << std::endl
              << " " << R << std::endl
              << std::endl;
    std::cout << "using translation vector = " << std::endl
              << " " << T << std::endl
              << std::endl;
    std::cout << "using Intrinsic matrix K = " << std::endl
              << " " << R << std::endl
              << std::endl;
    std::cout << "using  principal points vector C = " << std::endl
              << " " << C << std::endl
              << std::endl;
    std::cout << "using distortion coefficients D = " << std::endl
              << " " << D << std::endl
              << std::endl;
    // create pointcloud and bbox3d marker array publishers
    cloud_pub = nh_->advertise<sensor_msgs::PointCloud2>(out_cloud_topic, 1);
    bbox3d_pub =
        nh_->advertise<visualization_msgs::MarkerArray>(marker_array_topic, 1);

    // create image, bbox2d and cloud subscribers
    image_sub = new message_filters::Subscriber<sensor_msgs::Image>(
        *nh_, in_image_topic, 1);
    cloud_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(
        *nh_, in_cloud_topic, 1);
    bbox_sub = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(
        *nh_, bbox2d_topic, 1);
    sync = new message_filters::Synchronizer<MySyncPolicy>(
        MySyncPolicy(10), *cloud_sub, *image_sub, *bbox_sub);
    // bind the callbacks
    sync->registerCallback(boost::bind(&DrawBbox::SyncCB, this, _1, _2, _3));
    current_bboxes.bounding_boxes.resize(0);
    track_id.resize(0);
    current_track_id = 0;
    max_track_list_size = 30;
  }
  std::vector<int> GetImageProjection(cv::Mat &point3d, bool &is_valid) {
    std::vector<int> proj_point;
    proj_point.push_back(0);
    proj_point.push_back(0);
    cv::Mat point2d(3, 1, CV_64FC1);
    // get the corresponding image pixel
    point2d = (R * point3d + T);
    double w = point2d.at<double>(2, 0);
    if (w > 0) {
      point2d = point2d / w;
      float x_1 = point2d.at<double>(0, 0);
      float y_1 = point2d.at<double>(1, 0);
      float r = x_1 * x_1 + y_1 * y_1;
      x_1 = (x_1 * (1.0 + d[0] * r * r + d[1] * r * r * r * r +
                    d[4] * r * r * r * r * r * r) +
             2 * d[2] * x_1 * y_1 + d[3] * (r * r + 2 * x_1));
      y_1 = (y_1 * (1.0 + d[0] * r * r + d[1] * r * r * r * r +
                    d[4] * r * r * r * r * r * r) +
             2 * d[3] * x_1 * y_1 + d[2] * (r * r + 2 * y_1));
      point2d.at<double>(0, 0) = x_1;
      point2d.at<double>(1, 0) = y_1;
      point2d = K * point2d;
      int x = floor(point2d.at<double>(0, 0) + C.at<double>(0, 0));
      int y = floor(point2d.at<double>(1, 0) + C.at<double>(1, 0));
      is_valid = true;
      proj_point[0] = x;
      proj_point[1] = y;
      // check if pojection lies inside any bbox... if yes assign them to
      // corresponding cluster
    } else {
      is_valid = false;
    }
    return proj_point;
  }
  bool BboxClusterOverlap(const Eigen::Vector4f &sub_cluster_centroid,
                          const darknet_ros_msgs::BoundingBox &bbox) {
    int xmin, ymin, xmax, ymax, x, y;
    cv::Mat point3d(3, 1, CV_64FC1);
    point3d.at<double>(0, 0) = sub_cluster_centroid(0);
    point3d.at<double>(1, 0) = sub_cluster_centroid(1);
    point3d.at<double>(2, 0) = sub_cluster_centroid(2);
    bool is_valid = false;
    std::vector<int> proj_point = GetImageProjection(point3d, is_valid);
    if (is_valid) {
      xmin = bbox.ymin;
      xmax = bbox.ymax;
      ymin = bbox.xmin;
      ymax = bbox.xmax;
      x = proj_point[0];
      y = proj_point[1];
      if (x >= xmin && x < xmax && y >= ymin && y < ymax) {
        return true;
      }
    }
    return false;
  }
  void GetBboxCloudIntersection(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &image_cloud,
      pcl::PointIndices &cluster_indices,
      const darknet_ros_msgs::BoundingBox &bbox, int &num_points) {
    num_points = 0;
    int xmin, ymin, xmax, ymax, x, y;
    for (int i = 0; i < cluster_indices.indices.size(); i++) {
      cv::Mat point3d(3, 1, CV_64FC1);
      point3d.at<double>(0, 0) =
          image_cloud->points[cluster_indices.indices[i]].x;
      point3d.at<double>(1, 0) =
          image_cloud->points[cluster_indices.indices[i]].y;
      point3d.at<double>(2, 0) =
          image_cloud->points[cluster_indices.indices[i]].z;
      bool is_valid = false;
      std::vector<int> proj_point = GetImageProjection(point3d, is_valid);
      if (is_valid) {
        xmin = bbox.ymin;
        xmax = bbox.ymax;
        ymin = bbox.xmin;
        ymax = bbox.xmax;
        x = proj_point[0];
        y = proj_point[1];
        if (x >= xmin && x < xmax && y >= ymin && y < ymax) {
          num_points++;
        }
      }
    }
  }

  void PubColorCloud(const pcl::PCLPointCloud2 &cloud,
                     const cv_bridge::CvImagePtr &cv_ptr,
                     const darknet_ros_msgs::BoundingBoxesConstPtr &bbox2d) {
    //  cv::imshow("image", cv_ptr->image);
    //  cv::waitKey(1);
    // extract point cloud corresponding to each bbox2d
    int num_bbox = bbox2d->bounding_boxes.size();
    int image_cloud_size = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_out_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromPCLPointCloud2(cloud, *in_cloud);
    image_cloud->points.resize(in_cloud->points.size());
    temp_out_cloud->points.resize(in_cloud->points.size());
    bbox_markers.markers.resize(100);
    // ROS_INFO("conversion to points done");

    // iterate over all the points and assign them to correspoindg bbox
    // cloud

    cv::Mat point3d(3, 1, CV_64FC1);
    for (size_t i = 0; i < in_cloud->points.size(); i++) {
      if (in_cloud->points[i].z > cloud_zmax_thresh ||
          in_cloud->points[i].z < cloud_zmin_thresh) {
        continue;
      }
      /*  temp_out_cloud->points[i].x = in_cloud->points[i].x;
          temp_out_cloud->points[i].y = in_cloud->points[i].y;
          temp_out_cloud->points[i].z = in_cloud->points[i].z;
          temp_out_cloud->points[i].r = 255;
          temp_out_cloud->points[i].g = 255;
          temp_out_cloud->points[i].b = 255;*/

      //   ROS_INFO("image rows %d", cv_ptr->image.rows);
      point3d.at<double>(0, 0) = in_cloud->points[i].x;
      point3d.at<double>(1, 0) = in_cloud->points[i].y;
      point3d.at<double>(2, 0) = in_cloud->points[i].z;

      /*   cv::Mat point2d(3, 1, CV_64FC1);
      // get the corresponding image pixel
      point2d = (R * point3d + T);
      double w = point2d.at<double>(2, 0);
      if (w > 0) {
      point2d = point2d / w;
      float x_1 = point2d.at<double>(0, 0);
      float y_1 = point2d.at<double>(1, 0);
      float r = x_1 * x_1 + y_1 * y_1;
      x_1 = (x_1 * (1.0 + d[0] * r * r + d[1] * r * r * r * r +
      d[4] * r * r * r * r * r * r) +
      2 * d[2] * x_1 * y_1 + d[3] * (r * r + 2 * x_1));
      y_1 = (y_1 * (1.0 + d[0] * r * r + d[1] * r * r * r * r +
      d[4] * r * r * r * r * r * r) +
      2 * d[3] * x_1 * y_1 + d[2] * (r * r + 2 * y_1));
      point2d.at<double>(0, 0) = x_1;
      point2d.at<double>(1, 0) = y_1;
      point2d = K * point2d;
      int x = floor(point2d.at<double>(0, 0) + C.at<double>(0, 0));
      int y = floor(point2d.at<double>(1, 0) + C.at<double>(1, 0)); */
      // check if pojection lies inside any bbox... if yes assign them to
      // corresponding cluster
      bool is_valid = false;
      std::vector<int> proj_point = GetImageProjection(point3d, is_valid);
      if (is_valid) {
        int x = proj_point[0];
        int y = proj_point[1];
        int image_margin = 200;
        for (int j = 0; j < num_bbox; j++) {
          if (x >= 0 - image_margin && x < cv_ptr->image.rows + image_margin &&
              y >= 0 - image_margin && y < cv_ptr->image.cols + image_margin) {
            int xmin = bbox2d->bounding_boxes[j].ymin;
            int xmax = bbox2d->bounding_boxes[j].ymax;
            int ymin = bbox2d->bounding_boxes[j].xmin;
            int ymax = bbox2d->bounding_boxes[j].xmax;
            int bbox_offset = 0;
            //  if (x >= xmin + bbox_offset && x < xmax - bbox_offset &&
            //    y >= ymin + bbox_offset && y < ymax - bbox_offset) {
            //    ROS_INFO("x = %lf y = %lf", x, y);
            //   cv::Vec3b color_info = cv_ptr->image.at<cv::Vec3b>(x, y);
            /*   temp_out_cloud->points[i].r = 0;
                 temp_out_cloud->points[i].g = 255;
                 temp_out_cloud->points[i].b = 0;*/
            image_cloud->points[image_cloud_size].x = in_cloud->points[i].x;
            image_cloud->points[image_cloud_size].y = in_cloud->points[i].y;
            image_cloud->points[image_cloud_size].z = in_cloud->points[i].z;
            /* image_cloud->points[image_cloud_size].r = color_info[2];
             image_cloud->points[image_cloud_size].g = color_info[1];
             image_cloud->points[image_cloud_size].b = color_info[0];*/
            image_cloud_size++;
          }
        }
      }
    }
    /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PCA<pcl::PointXYZRGB> pca;
      pca.setInputCloud(cloud_clusters[0]);
      pca.project(*cloud_clusters[0], *projected_cloud);*/
    // temp_out_cloud = cloud_clusters[0];
    // get subclusters of each bbox point cloud
    image_cloud->points.resize(image_cloud_size);
    image_cloud->height = image_cloud_size;
    // ROS_INFO("starting clustering");
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud(image_cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*filtered_cloud);
    *image_cloud = *filtered_cloud;

    // initialize Kd tree and Clustering algorithm
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(image_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.3);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(15000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(image_cloud);
    ec.extract(cluster_indices);
    //    ROS_INFO("number of clusters: [%d]", cluster_indices.size());
    if (cluster_indices.size() <= 0) {
      return;
    }
    // get the cluster index corresponding to closest cluster (most of the
    // time
    // this will be the detected object .. // needs to be better
    int significant_cluster_idx = 0;
    int t = 0;
    float closest_centroid = std::numeric_limits<float>::max();
    int *bbox_cluster_map = new int[num_bbox];
    int *bbox_cluster_num_points = new int[num_bbox];
    float *cluster_dist = new float[num_bbox];
    for (int i = 0; i < num_bbox; i++) {
      bbox_cluster_map[i] = -1;
      bbox_cluster_num_points[i] = 0;
      cluster_dist[i] = std::numeric_limits<float>::max();
    }
    std::vector<float> kitti_alpha(MAX_NUM_BBOX);
    std::vector<cv::Mat> kitti_location(MAX_NUM_BBOX);
    std::vector<float> kitti_rotation_y(MAX_NUM_BBOX);
    std::vector<Eigen::Vector3f> kitti_size(MAX_NUM_BBOX);
    for (std::vector<pcl::PointIndices>::const_iterator
             it = cluster_indices.begin();
         it != cluster_indices.end(); it++, t++) {
      Eigen::Vector4f sub_cluster_centroid;
      pcl::compute3DCentroid(*image_cloud, *it, sub_cluster_centroid);
      float temp_cluster_dist = sub_cluster_centroid.squaredNorm();

      //}
      for (int j = 0; j < num_bbox; j++) {
        int num_points = 0;
        //   GetBboxCloudIntersection(image_cloud, cluster_indices[t],
        //                           bbox2d->bounding_boxes[j], num_points);
        if (BboxClusterOverlap(sub_cluster_centroid,
                               bbox2d->bounding_boxes[j])) {
          //   if (num_points > bbox_cluster_num_points[j]) {
          if (temp_cluster_dist < cluster_dist[j]) {
            bbox_cluster_map[j] = t;
            bbox_cluster_num_points[j] = num_points;
            cluster_dist[j] = temp_cluster_dist;
          }
        }
      }
    }
    for (int j = 0; j < num_bbox; j++) {
      if (bbox_cluster_map[j] == -1) {
        continue;
      }
      significant_cluster_idx = bbox_cluster_map[j];
      //  if (1 || t != 0) {
      /* pcl::ExtractIndices<pcl::PointXYZRGB> extract;
         extract.setInputCloud(cloud_clusters[i]);
         ROS_INFO("num of clusters = %d", t);
         ROS_INFO("cluster index size = %d",
         cluster_indices[significant_cluster_idx].indices.size());
         pcl::PointIndices::Ptr cluster_index_pointer(new
         pcl::PointIndices);
      //  cluster_index_pointer.reset(
      //    &(cluster_indices[significant_cluster_idx]));
      //  ROS_INFO("cluster index size2 = %d",
      //         cluster_index_pointer->indices.size());
      //  extract.setIndices(cluster_index_pointer);
      extract.setNegative(false);*/
      //   extract.filter(*filtered_cloud);
      filtered_cloud->points.resize(0);
      filtered_cloud->height = 0;
      int cluster_size =
          cluster_indices[significant_cluster_idx].indices.size();
      for (int p = 0; p < cluster_size; p++) {
        filtered_cloud->points.push_back(
            image_cloud
                ->points[cluster_indices[significant_cluster_idx].indices[p]]);
      }
      /*else {
      // if no significant clusters found return
      return;
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud(image_cloud);
      sor.setMeanK(10);
      sor.setStddevMulThresh(0.1);
      // ROS_INFO("applying filtering");
      sor.filter(*filtered_cloud);
      }*/
      //  cloud_clusters[i]->reset(filtered_cloud);
      //     filtered_cloud = cloud_clusters[i];
      //  ROS_INFO("filter applied");
      // start PCA for significant cluster to get the oreinted bounding
      // box
      //    ROS_INFO("starting PCA");
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PCA<pcl::PointXYZRGB> pca;
      pca.setInputCloud(filtered_cloud);
      pca.project(*filtered_cloud, *projected_cloud);
      Eigen::Matrix3f eigen_vector_pca = pca.getEigenVectors();
      Eigen::Vector3f eigen_values = pca.getEigenValues();
      Eigen::Matrix3d eigen_vector_pca_double = eigen_vector_pca.cast<double>();
      pcl::PointXYZRGB min_point, max_point;
      pcl::getMinMax3D(*projected_cloud, min_point, max_point);
      // pcl::getMinMax3D(*filtered_cloud, min_point, max_point);
      // std::cout << "Eigen vectors" << std::endl
      //         << eigen_vector_pca << std::endl;
      Eigen::Vector4f cluster_centroid;
      pcl::compute3DCentroid(*filtered_cloud, cluster_centroid);
      *temp_out_cloud += *filtered_cloud;
      //  std::cout << "cluster_centroid" << std::endl
      //          << cluster_centroid << std::endl;
      tf::Quaternion quat;
      tf::Matrix3x3 tf_rotation;
      tf::matrixEigenToTF(eigen_vector_pca_double, tf_rotation);
      double roll, pitch, yaw;
      tf_rotation.getRPY(roll, pitch, yaw);
      // ROS_INFO("filling data for %d box", j);
      kitti_rotation_y[j] = yaw;
      kitti_alpha[j] = atan2(cluster_centroid(1), cluster_centroid(0));
      Eigen::Vector3f bbox_size;
      bbox_size(0) = max_point.z - min_point.z;
      bbox_size(1) = max_point.y - min_point.y;
      bbox_size(2) = max_point.x - min_point.x;
      kitti_size[j] = (bbox_size);
      cv::Mat cluster_center(3, 1, CV_64FC1);
      cluster_center.at<double>(0, 0) = cluster_centroid(0);
      cluster_center.at<double>(1, 0) = cluster_centroid(1);
      cluster_center.at<double>(2, 0) = cluster_centroid(2);
      kitti_location[j] = (R * cluster_center + T);
      // tf_rotation.getRotation(quat);
      quat.setRPY(0, 0, yaw);
      quat.normalize();
      geometry_msgs::Quaternion msg_quat;
      tf::quaternionTFToMsg(quat, msg_quat);
      // quat.normalize();
      // create marker correspoinding to the bbox
      visualization_msgs::Marker marker;
      marker.header.frame_id = lidar_frame_id;
      marker.text = bbox2d->bounding_boxes[j].Class;
      // marker.text = "car";
      marker.header.stamp = ros::Time::now();
      marker.id = j;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(0.3);
      marker.pose.position.x = cluster_centroid(0);
      marker.pose.position.y = cluster_centroid(1);
      marker.pose.position.z = cluster_centroid(2);
      marker.pose.orientation.x = msg_quat.x;
      marker.pose.orientation.y = msg_quat.y;
      marker.pose.orientation.z = msg_quat.z;
      marker.pose.orientation.w = msg_quat.w;
      marker.scale.x = max_point.x - min_point.x + 0.3;
      marker.scale.y = max_point.y - min_point.y + 0.3;
      marker.scale.z = max_point.z - min_point.z + 0.3;
      //  marker.scale.x = 3;
      //  marker.scale.y = 2;
      //  marker.scale.z = 2;
      marker.color.a = 0.2;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0;
      bbox_markers.markers.push_back(marker);

      //  ROS_INFO("info pushing marker");
    }
    int *track_result = new int[bbox2d->bounding_boxes.size()];
    if (enable_tracking) {
      int h_bins = 50;
      int s_bins = 60;
      int histSize[] = {h_bins, s_bins};
      float h_ranges[] = {0, 180};
      float s_ranges[] = {0, 256};
      const float *ranges[] = {h_ranges, s_ranges};

      int channels[] = {0, 1};
      int success_matches = 0;
      int bottle_flag = 0;
      int list_size = track_list.size();
      for (size_t j = 0; j < bbox2d->bounding_boxes.size(); j++) {
        if (bbox_cluster_map[j] == -1) {
          continue;
        }
        int xmin, ymin, xmax, ymax;
        xmin = bbox2d->bounding_boxes[j].xmin;
        ymin = bbox2d->bounding_boxes[j].ymin;
        xmax = bbox2d->bounding_boxes[j].xmax;
        ymax = bbox2d->bounding_boxes[j].ymax;
        // ROS_INFO("bbox %d %d %d %d", xmin, ymin, xmax, ymax);
        cv::Rect2d track_bbox(xmin, ymin, xmax - xmin, ymax - ymin);
        // cv::rectangle(cv_ptr->image, track_bbox, cv::Scalar(255, 0, 0), 2,
        // 1);
        // imshow("tracker", cv_ptr->image);
        // cv::waitKey(0);
        cv::Rect2d track_bbox2;
        cv::Mat bbox_image(cv_ptr->image, track_bbox);
        cv::resize(bbox_image, bbox_image, cv::Size2i(50, 50));
        cv::Mat hsv_bbox_image;
        cv::cvtColor(bbox_image, hsv_bbox_image, cv::COLOR_BGR2HSV);
        cv::MatND hist_bbox;
        cv::calcHist(&hsv_bbox_image, 1, channels, cv::Mat(), hist_bbox, 2,
                     histSize, ranges, true, false);
        cv::normalize(hist_bbox, hist_bbox, 0, 1, cv::NORM_MINMAX, -1,
                      cv::Mat());
        // ROS_INFO("before tracking = %f, %f, %f, %f", track_bbox.x,
        // track_bbox.y,
        // track_bbox.width, track_bbox.height);
        // ROS_INFO("after tracking = %f, %f, %f, %f", track_bbox.x,
        // track_bbox.y,
        // track_bbox.width, track_bbox.height);
        //    cv::imshow("matching this", bbox_image);
        //   cv::waitKey(0);
        cv::Mat best_match;
        double score = -1;
        std::list<BBox>::iterator match_it;
        int i = 0;
        for (std::list<BBox>::iterator it = track_list.begin(); i < list_size;
             ++it, i++) {
          double base = cv::compareHist(hist_bbox, it->hist_bbox, 0);
          if (score < base) {
            score = base;
            match_it = it;
            //      cv::imshow("last best match",
            //      match_it->bbox_image);
            //     cv::waitKey(10);
          }
        }
        if (score > 0.8) {
          match_it->hist_bbox = hist_bbox;
          track_result[j] = match_it->detection_index;
          //  cv::imshow("matched with this", match_it->bbox_image);
          // cv::waitKey(0);

        } else {
          // ROS_INFO("adding with score = %f", score);
          // cv::imshow("adding this", bbox_image);
          BBox current_bbox;
          current_bbox.hist_bbox = hist_bbox;
          current_bbox.bbox_image = bbox_image;
          current_bbox.detection_time = ros::Time::now();
          current_bbox.detection_index = current_track_id;
          track_result[j] = current_track_id;
          current_track_id++;
          track_list.push_back(current_bbox);
          if (track_list.size() > max_track_list_size) {
            track_list.pop_front();
          }
        }

        /*for (int i = 0; i < num_bbox; i++) {
          if (bbox2d->bounding_boxes[i].Class == "bottle") {
            bottle_flag = 1;
            continue;
          }
          xmin = bbox2d->bounding_boxes[i].xmin;
          ymin = bbox2d->bounding_boxes[i].ymin;
          xmax = bbox2d->bounding_boxes[i].xmax;
          ymax = bbox2d->bounding_boxes[i].ymax;
          cv::Rect2d prev_bbox(xmin, ymin, xmax - xmin, ymax - ymin);
          cv::Mat prev_bbox_image(cv_ptr->image, prev_bbox);
          cv::Mat prev_hsv_bbox_image;
          cv::cvtColor(prev_bbox_image, prev_hsv_bbox_image,
        cv::COLOR_BGR2HSV);
          cv::MatND prev_hist_bbox;
          cv::calcHist(&prev_hsv_bbox_image, 1, channels, cv::Mat(),
                       prev_hist_bbox, 2, histSize, ranges, true,
        false);
          cv::normalize(prev_hist_bbox, prev_hist_bbox, 0, 1,
        cv::NORM_MINMAX,
        -1,
                        cv::Mat());
          double base = cv::compareHist(hist_bbox, prev_hist_bbox, 0);
          if (score < base) {
            match_id = i;
            score = base;
            //   best_match = prev_bbox_image;
          }
          ROS_INFO("compare result for j=%d, i=%d %f", j, i, base);
          //   ROS_INFO("overlap for i, j = %d, %d, %f", j, i,
          //           (track_bbox & prev_bbox).area() /
          //              (track_bbox | prev_bbox).area());
        }*/
        // cv::imshow("matched with", best_match);
        // ROS_INFO("match score %f", score);
        // if (score > 0.9) {
        //  success_matches++;
        //  }

        // cv::waitKey(0);
        // cv::rectangle(cv_ptr->image, track_bbox2, cv::Scalar(255, 0,
        // 0), 2,
        // 1);
        // cv::rectangle(cv_ptr->image, track_bbox, cv::Scalar(0, 0,
        // 255), 2,
        // 1);
      }
    }
    // if (num_bbox - bottle_flag - success_matches > 0) {
    //  max_track_id = max_track_id + num_bbox - success_matches -
    //  bottle_flag;
    // }
    // ROS_INFO("done Tracking");
    /* current_bboxes.bounding_boxes.resize(0);
     for (int j = 0; j < num_bbox; j++) {
       if (bbox2d->bounding_boxes[j].Class == "bottle") {
         continue;
       }
       current_bboxes.bounding_boxes.push_back(bbox2d->bounding_boxes[j]);
     }*/
    for (size_t j = 0; j < num_bbox; j++) {
      if (bbox_cluster_map[j] == -1) {
        continue;
      }
      int xmin, ymin, xmax, ymax;
      xmin = bbox2d->bounding_boxes[j].xmin;
      xmax = bbox2d->bounding_boxes[j].xmax;
      ymin = bbox2d->bounding_boxes[j].ymin;
      ymax = bbox2d->bounding_boxes[j].ymax;
      if (enable_tracking) {
        label_file << frame_num << " " << track_result[j] << " ";
      }
      label_file << bbox2d->bounding_boxes[j].Class << " 0"
                 << " 3 " << kitti_alpha[j] << " " << xmin << " " << ymin << " "
                 << xmax << " "
                 << " " << ymax << " " << kitti_size[j](0) << " "
                 << kitti_size[j](1) << " " << kitti_size[j](2) << " "
                 << kitti_location[j].at<double>(0, 0) << " "
                 << kitti_location[j].at<double>(1, 0) << " "
                 << kitti_location[j].at<double>(2, 0) << " "
                 << kitti_rotation_y[j] << " "
                 << "\n";
    }
    // ROS_INFO("max track id = %d", current_track_id);
    // ROS_INFO("num objects in track list track = %d", track_list.size());
    //  ROS_INFO("pushed the marker");
    //   break;

    // *temp_out_cloud += *cloud_clusters[i];
    //    for (int j = 0; j < bbox2d->bounding_boxes.size(); j++) {
    //   }

    pcl::PCLPointCloud2 *temp_cloud = new pcl::PCLPointCloud2;
    pcl::toPCLPointCloud2(*temp_out_cloud, *temp_cloud);
    //   pcl::toPCLPointCloud2(*image_cloud, *temp_cloud);
    pcl_conversions::fromPCL(*temp_cloud, out_cloud);
    out_cloud.header.stamp = ros::Time::now();
    out_cloud.header.frame_id = lidar_frame_id;

    // publish point cluster cloud with color and bbox
    bbox3d_pub.publish(bbox_markers);
    cloud_pub.publish(out_cloud);
    frame_num++;
  }
  void SyncCB(const sensor_msgs::PointCloud2ConstPtr &cloud,
              const sensor_msgs::ImageConstPtr &img,
              const darknet_ros_msgs::BoundingBoxesConstPtr &bbox2d) {
    if ((cloud->width * cloud->height) == 0) {
      return; // cloud is not dense;
    }
    // converting to PCL point cloud type
    //   ROS_INFO("converting point cloud type");
    pcl::PCLPointCloud2 *in_cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*cloud, *in_cloud);
    //   ROS_INFO("conversion done");

    /* pcl::PCLPointCloud2 cloud_filtered;
       pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
       pcl::PCLPointCloud2ConstPtr cloudPtr(in_cloud);
       ROS_INFO("setting input cloud");
       sor.setInputCloud(cloudPtr);
       sor.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
       ROS_INFO("applying filter");
       sor.filter(cloud_filtered);
       ROS_INFO("publishing filtered cloud");
       pcl_conversions::fromPCL(cloud_filtered, out_cloud);*/
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // pass cloud, image and bbox2d to get 3d bbox
    PubColorCloud(*in_cloud, cv_ptr, bbox2d);
  }
  static void SigintHandler(int sig) {
    ROS_INFO("closing file and shutting down");
    label_file.close();
    ros::shutdown();
  }

private:
  ros::NodeHandle *nh_;
  sensor_msgs::PointCloud2 out_cloud;
  // topic subscribers
  std::string in_cloud_topic, out_cloud_topic, depth_image_topic,
      in_image_topic, lidar_frame_id, bbox2d_topic, marker_array_topic;
  // pointcloud, detection image , bounding box subscribers
  message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub;
  message_filters::Subscriber<sensor_msgs::Image> *image_sub;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> *bbox_sub;
  // sync policy
  message_filters::Synchronizer<MySyncPolicy> *sync;
  ros::Publisher cloud_pub, depth_image_pub, bbox3d_pub;
  // remove the cloud above this value
  float cloud_zmax_thresh, cloud_zmin_thresh;
  // camera lidar calibration data, ros parameters
  std::vector<double> r, t, k, c, d;
  // camera lidar calibration data , cv mat
  cv::Mat R, T, K, C, D;
  Eigen::Matrix3f cam_to_world_rotation;
  Eigen::Vector3f cam_to_world_translation;
  visualization_msgs::MarkerArray bbox_markers;
  darknet_ros_msgs::BoundingBoxes current_bboxes;
  static std::ofstream label_file;
  std::vector<int> track_id;
  std::string label_file_path;
  int current_track_id;
  int max_track_list_size;
  std::list<BBox> track_list;
  long long int frame_num;
  bool enable_tracking;
};
std::ofstream DrawBbox::label_file;
int main(int argc, char **argv) {
  ros::init(argc, argv, "bbox", ros::init_options::NoSigintHandler);
  DrawBbox draw_bbox;
  signal(SIGINT, DrawBbox::SigintHandler);
  ros::spin();
}
