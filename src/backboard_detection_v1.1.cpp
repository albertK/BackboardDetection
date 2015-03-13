//read PCD
#include <pcl/io/pcd_io.h>

//ROI segmentation
#include <pcl/filters/passthrough.h>

//RANSAC plane segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

//ICP
#include <pcl/registration/icp.h>

//Visualization
#include <pcl/visualization/pcl_visualizer.h>

//common
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <cstdio>
#include <cmath>
#include <pcl/common/centroid.h>

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
    
    //Backboard PointCloud model generation
    double w,h,res;
    w=1.70;
    h=1.00;
    res=0.03;
    pcl::PointCloud<pcl::PointXYZ>::Ptr backboard_model(new pcl::PointCloud<pcl::PointXYZ>);
    for(unsigned int i = 0; i <= (unsigned int) (h/res); ++i)
	for(unsigned int j = 0; j <= (unsigned int) (w/res); ++j)
	    backboard_model->push_back(pcl::PointXYZ(-w/2+j*res, -h/2+i*res, 0.0));
    
    //Backboard ICP matching
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid<pcl::PointXYZ>(*backboard, centroid);
    initial_guess.block(0,3,4,1) = centroid;
    Eigen::Matrix4f result_transform = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_model (new pcl::PointCloud<pcl::PointXYZ>);
    if(backboard_align(backboard_model, backboard, initial_guess, aligned_model, result_transform))
    {
	viewer.addPointCloud<pcl::PointXYZ>(aligned_model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(aligned_model, 255.0, 0.0, 0.0), "aligned_backboard");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "aligned_backboard");
	pcl::compute3DCentroid<pcl::PointXYZ>(*aligned_model, centroid);
	viewer.addSphere<pcl::PointXYZ>(pcl::PointXYZ(centroid(0,0), centroid(1,0), centroid(2,0)), 0.08, 255.0, 255.0, 0.0);
	viewer.spin();
    }
    else
	printf("Alignment failed...\n");
    
    return 0;
}