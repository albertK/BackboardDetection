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

//ICP
#include <pcl/registration/icp.h>

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

void plane_projection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr plane_coeff, cv::Mat& image_out, Eigen::Matrix4f& transform)
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
	vt = 1.0-ct;
	
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
	
	transform.block(0,3,3,1) = Eigen::Vector3f(-x, -y, -z_avg);
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

void hole_filling(cv::Mat img_in, cv::Mat& img_out)
{
    int w,h;
    w=h=5;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*w+1, 2*h+1));
    cv::dilate(img_in, img_out, kernel);
    cv::erode(img_out, img_out, kernel);
}

void border_detection(cv::Mat img_in, cv::Mat& img_out)
{    
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( img_in, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    
    int largest_contour = 0;
    for(unsigned int i = 0; i < contours.size(); ++i)
	if(contours[i].size() > contours[largest_contour].size())
	    largest_contour = i;
	
    //img_out = cv::Mat::zeros(img_in.size(), CV_8UC1);
    //cv::drawContours(img_out, contours, largest_contour, cv::Scalar(255));
	
    //Convex hull
    std::vector<std::vector<cv::Point> > hull(1);
    cv::convexHull(contours[largest_contour], hull[0]);
    
    img_out = cv::Mat::zeros(img_in.size(), CV_8UC1);
    cv::drawContours(img_out, hull, 0, cv::Scalar(255));
}

void plane_back_projection(cv::Mat img_in, Eigen::Matrix4f projection_transform, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    for(unsigned int i = 0; i < img_in.rows; ++i)
	for(unsigned int j = 0; j < img_in.cols; ++j)
	    if(img_in.at<uchar>(i,j) == 255)
		cloud_out->push_back(pcl::PointXYZ(((float)j)/100.0, ((float)i)/100.0, 0.0));
    
    Eigen::Matrix4f back_projectiion_transform = projection_transform.inverse();
    pcl::transformPointCloud<pcl::PointXYZ>(*cloud_out, *cloud_out, back_projectiion_transform);
}

bool backboard_align(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, Eigen::Matrix4f initial_guess, pcl::PointCloud<pcl::PointXYZ>::Ptr aligned, Eigen::Matrix4f& transform)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(model);
    icp.setInputTarget(scene);
    icp.setEuclideanFitnessEpsilon(0.0005);
    icp.setMaximumIterations(1000000);
    icp.setTransformationEpsilon(1e-12);
    icp.align(*aligned, initial_guess);
    
    if(icp.hasConverged())
	transform = icp.getFinalTransformation();
    else
	transform = Eigen::Matrix4f::Identity();
    
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    
    return icp.hasConverged();
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
    cv::Mat backboard_img;
    Eigen::Matrix4f transform;
    plane_projection(backboard, plane_coeff, backboard_img, transform);
    printf("Image size = %d x %d\n", backboard_img.rows, backboard_img.cols);
    cv::namedWindow("Projected Backboard", CV_WINDOW_NORMAL);
    cv::imshow("Projected Backboard", backboard_img);
    cv::waitKey();
    cv::destroyWindow("Projected Backboard");
    
    //Image hole filling
    cv::Mat hole_filled_img;
    hole_filling(backboard_img, hole_filled_img);
    cv::namedWindow("Hole filled Backboard", CV_WINDOW_NORMAL);
    cv::imshow("Hole filled Backboard", hole_filled_img);
    cv::waitKey();
    cv::destroyWindow("Hole filled Backboard");
    
    //Backbaoard border detection
    cv::Mat border_img;
    border_detection(hole_filled_img, border_img);
    cv::namedWindow("Backboard border", CV_WINDOW_NORMAL);
    cv::imshow("Backboard border", border_img);
    cv::waitKey();
    cv::destroyWindow("Backboard border");
    
    //Image line detect
    /*
    std::vector<cv::Vec4i> lines;
    cv::Mat line_img;
    cv::cvtColor(border_img, line_img, CV_GRAY2BGR);
    cv::HoughLinesP(border_img, lines, 5, CV_PI/50, 80, 80, 60 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
	cv::Vec4i l = lines[i];
	cv::line(line_img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
    }
    cv::namedWindow("Backboard edge line", CV_WINDOW_NORMAL);
    cv::imshow("Backboard edge line", line_img);
    cv::waitKey();
    cv::destroyWindow("Backboard edge line");
    */
    
    //plane back projection    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_border (new pcl::PointCloud<pcl::PointXYZ>);
    plane_back_projection(border_img, transform, cloud_border);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_border, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_border, 255.0, 255.0, 0.0), "border cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "border cloud");
    viewer.spin();
    
    //model generation of backbord border
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_border_model (new pcl::PointCloud<pcl::PointXYZ>);
    float w,h,res;
    w=1.80;
    h=1.20;
    res=0.03;
    for(float i = 0.0; i < w/res; ++i)
    {
	cloud_border_model->push_back(pcl::PointXYZ(i*res, 0.0, 0.0));
	cloud_border_model->push_back(pcl::PointXYZ(i*res, h, 0.0));
    }
    for(float i = 0.0; i < h/res; ++i)
    {
	cloud_border_model->push_back(pcl::PointXYZ(0.0, i*res, 0.0));
	cloud_border_model->push_back(pcl::PointXYZ(w, i*res, 0.0));
    }
    
    //backbord model ICP matching
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid<pcl::PointXYZ>(*cloud_border, centroid);
    initial_guess.block(0,3,4,1) = centroid;
    Eigen::Matrix4f result_transform = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_model (new pcl::PointCloud<pcl::PointXYZ>);
    if(backboard_align(cloud_border_model, cloud_border, initial_guess, aligned_model, result_transform))
    {
	viewer.addPointCloud<pcl::PointXYZ>(aligned_model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(aligned_model, 255.0, 0.0, 0.0), "aligned backboard");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "aligned backboard");
	pcl::compute3DCentroid<pcl::PointXYZ>(*aligned_model, centroid);
	viewer.addSphere<pcl::PointXYZ>(pcl::PointXYZ(centroid(0,0), centroid(1,0), centroid(2,0)), 0.03, 255.0, 255.0, 0.0);
	viewer.spin();
    }
    else
	printf("Alignment failed...\n");
    
    //Image corner localization
    //Board reconstruction
    //2D->3D back-projection
}