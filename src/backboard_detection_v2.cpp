//read PCD
#include <pcl/io/pcd_io.h>

//ROI segmentation
#include <pcl/filters/passthrough.h>

//RANSAC plane segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

//Visualization
#include <pcl/visualization/pcl_visualizer.h>

//2D image processing
#include <opencv2/opencv.hpp>

//common
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <cstdio>
#include <cmath>

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void * viewer){}


void roi_seg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, double x, double X, double y, double Y, double z, double Z, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    
    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z, Z);
    pass.filter (*cloud_out);
    pass.setInputCloud (cloud_out);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y, Y);
    pass.filter (*cloud_out);
    pass.setInputCloud (cloud_out);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x, X);
    pass.filter (*cloud_out);
}

void plane_seg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, double thres, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, pcl::ModelCoefficients::Ptr coeff)
{
    
    //RANSAC平面偵測
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(thres);
    seg.setInputCloud(cloud_in);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coeff);
    //將平面上的點留下其餘刪掉
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_out);
}

void plane_projection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr plane_coeff, cv::Mat& image_out, Eigen::Matrix4d& transform)
{
    //calculate the euivalent angle axist rotational matrix
    Eigen::Vector3d plane_nm(plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2]);
    plane_nm = plane_nm/plane_nm.norm();
    Eigen::Vector3d rot_axis = plane_nm.cross(Eigen::Vector3d(0.0, 0.0, 1.0));
    
    if(rot_axis.norm() > 0.01)
    {
	rot_axis = rot_axis/rot_axis.norm();
	double kx,ky,kz,st,ct,vt;
	kx = rot_axis(0);
	ky = rot_axis(1);
	kz = rot_axis(2);
	st = plane_nm.cross(Eigen::Vector3d(0.0, 0.0, 1.0)).norm();
	ct = plane_nm.dot(Eigen::Vector3d(0.0, 0.0, 1.0));
	vt = 1-ct;
	
	transform<<kx*kx*vt+ct,    kx*ky*vt-kz*st, kx*kz*vt+ky*st, 0,
		   kx*ky*vt+kz*st, ky*ky*vt+ct,    ky*kz*vt-kx*st, 0,
		   kx*kz*vt-ky*st, ky*kz*vt+kx*st, kz*kz*vt+ct,    0,
		   0,              0,              0,              1;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud<pcl::PointXYZ>(*cloud_in, *rotated_plane, transform);
	
	double x,y,z =  1000;
	double X,Y,Z = -1000;
	double z_avg = 0;
	for(unsigned int i = 0; i < rotated_plane->size(); ++i)
	{
	    if(x>rotated_plane->points[i].x)
		x = rotated_plane->points[i].x;
	    if(y>rotated_plane->points[i].y)
		y = rotated_plane->points[i].y;
	    if(z>rotated_plane->points[i].z)
		z = rotated_plane->points[i].z;
	    if(X<rotated_plane->points[i].x)
		X = rotated_plane->points[i].x;
	    if(Y<rotated_plane->points[i].y)
		Y = rotated_plane->points[i].y;
	    if(Z<rotated_plane->points[i].z)
		Z = rotated_plane->points[i].z;
	    z_avg += rotated_plane->points[i].z;
	}
	z_avg /= rotated_plane->size();
	printf("x=%f, y=%f, z=%f\nX=%f, Y=%f, Z=%f\nz_avg=%f\n", x,y,z,X,Y,Z,z_avg);
	
	transform.block(0,3,3,1) = Eigen::Vector3d(-x, -y, -z_avg);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud<pcl::PointXYZ>(*cloud_in, *transformed_plane, transform);
	
	int img_w, img_h;
	img_w = (int)ceil((X-x)*100);
	img_h = (int)ceil((Y-y)*100);
	image_out = cv::Mat::zeros(img_h, img_w, CV_8UC1);
	for(unsigned int i = 0; i < transformed_plane->size(); ++i)
	{
	    int col = (int)ceil(transformed_plane->points[i].x*100);
	    int row = (int)ceil(transformed_plane->points[i].y*100);
	    image_out.at<uchar>(row,col) = 255;
	}
    }
}

int main(int argc, char** argv)
{
    //Visualization
    pcl::visualization::PCLVisualizer viewer("result");
    viewer.setBackgroundColor(0.0,0.0,0.0);
    viewer.addCoordinateSystem(0.5, "coordinate");
    viewer.registerKeyboardCallback(keyboardEventOccurred, (void *) &viewer);
    
    //讀取PCD檔
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *src);
    viewer.addPointCloud(src, "src");
    viewer.spin();
    
    //ROI segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
    double x,X,y,Y,z,Z;
    x=-2.0;
    X=2.0;
    y=-2.0;
    Y=2.0;
    z=1.5;
    Z=5.0;
    roi_seg(src, x, X, y, Y, z, Z,roi);
    viewer.addPointCloud<pcl::PointXYZ>(roi, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(roi, 0.0, 255.0, 0.0), "roi");
    viewer.spin();
    
    //RANSAC plane segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr backboard (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
    double thres = 0.05;
    plane_seg(roi, 0.05, backboard, plane_coeff);
    viewer.addPointCloud<pcl::PointXYZ>(backboard, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(backboard, 0.0, 0.0, 255.0), "backboard");
    viewer.spin();
    
    //3D->2D projection
    cv::Mat backboard_img = cv::Mat::zeros(3,3,CV_8UC1);
    Eigen::Matrix4d transform;
    plane_projection(backboard, plane_coeff, backboard_img, transform);
    printf("Image size = %d x %d\n", backboard_img.rows, backboard_img.cols);
    cv::namedWindow("Projected Backboard", CV_WINDOW_NORMAL);
    cv::imshow("Projected Backboard", backboard_img);
    cv::waitKey();
    
    //Image hole filling
    //Image line detection
    //Image corner localization
    //Board reconstruction
    //2D->3D back-projection
}