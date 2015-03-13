//basic PointCloud types
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//read PCD
#include <pcl/io/pcd_io.h>

//OpenNI device
#include <pcl/apps/3d_rec_framework/tools/openni_frame_source.h>

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

//frame rate
#include <pcl/common/time.h>

//2D image processing
#include <opencv2/opencv.hpp>

//common
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <cstdio>
#include <cmath>

#define DBG 0
#define KINECT 1


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void * viewer) {}

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

        transform<<kx*kx*vt+ct,   kx*ky*vt-kz*st, kx*kz*vt+ky*st, 0,
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
        //printf("x=%f, y=%f, z=%f\nX=%f, Y=%f, Z=%f\nz_avg=%f\n", x,y,z,X,Y,Z,z_avg);

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

void img_hole_filling(cv::Mat img_in, cv::Mat& img_out)
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

bool backboard_align(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr aligned, Eigen::Matrix4f& transform)
{
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid<pcl::PointXYZ>(*scene, centroid);
    initial_guess.block(0,3,4,1) = centroid;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(model);
    icp.setInputTarget(scene);
    icp.align(*aligned, initial_guess);

    if(icp.hasConverged())
        transform = icp.getFinalTransformation();
    else
        transform = Eigen::Matrix4f::Identity();

    //std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

    return icp.hasConverged();
}

void transform_kinrobot_estimation(pcl::ModelCoefficients::Ptr coeff, Eigen::Vector3f p, Eigen::Matrix4f& transform)
{
    float theta_x = atan2(fabs(coeff->values[1]), fabs(coeff->values[2]));
#if DBG
    printf("Kinect elevation angle = %f deg\n", theta_x/3.14159*180.0);
#endif
    
    Eigen::Matrix3f R;
    R<<1,	     0,		    0,
       0, cos(theta_x), -sin(theta_x),
       0, sin(theta_x),  cos(theta_x);
    
    transform.block(0,0,3,3)=R;
    transform.block(0,3,3,1)=p;
}

void offset_angle(Eigen::Vector3f shoot_goal, float& theta)
{
    float cos_theta = shoot_goal(2);
    float sin_theta = shoot_goal(0);
    theta = atan2(sin_theta, cos_theta)/3.14159*180.0;
}

int main(int argc, char** argv)
{
    //Visualization
    pcl::visualization::PCLVisualizer viewer("result");
    viewer.setBackgroundColor(0.0,0.0,0.0);
    viewer.addCoordinateSystem(0.5, "coordinate");
    viewer.registerKeyboardCallback(keyboardEventOccurred, (void *) &viewer);

#if KINECT
    //初始化OpenNI device
    OpenNIFrameSource::OpenNIFrameSource camera;
    sleep(2);
    OpenNIFrameSource::PointCloudPtr frame;
#endif
    
#if KINECT
    //讀取場景PointCloud
    while(camera.isActive())
    {
#if !DBG
        //calculate the frame rate
        pcl::ScopeTime frame_rate("Backboard detection takes");
#endif
        viewer.removeAllPointClouds();
        viewer.removeAllShapes();
#endif
        pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
#if KINECT
        frame = camera.snap();
        pcl::copyPointCloud(*frame, *src);
#else
	pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *src);
#endif
        viewer.addPointCloud(src, "src");
#if DBG
        viewer.spin();
#endif

        //ROI segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
        double x,X,y,Y,z,Z;
        x=-2.0;
        X=2.0;
        y=-2.0;
        Y=1.0;
        z=1.5;
        Z=5.0;
        roi_seg(src, x, X, y, Y, z, Z,roi);
#if DBG
        viewer.addPointCloud<pcl::PointXYZ>(roi, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(roi, 0.0, 255.0, 0.0), "roi");
        viewer.spin();
#endif

        //RANSAC plane segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr backboard (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
        double thres = 0.05;
        plane_seg(roi, thres, backboard, plane_coeff);
#if DBG
        viewer.addPointCloud<pcl::PointXYZ>(backboard, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(backboard, 0.0, 0.0, 255.0), "backboard");
        viewer.spin();
#endif
	
	//estimate the transform of kinect
	Eigen::Vector3f offset(0.0, 0.0, 0.47);
	Eigen::Matrix4f kinect_transform = Eigen::Matrix4f::Identity();
	transform_kinrobot_estimation(plane_coeff, offset, kinect_transform);

        //Project the points onto the plane
        //3D->2D for easier border detection
        cv::Mat backboard_img;
        Eigen::Matrix4f transform;
        plane_projection(backboard, plane_coeff, backboard_img, transform);
#if DBG
        cv::namedWindow("Projected Backboard", CV_WINDOW_NORMAL);
        cv::imshow("Projected Backboard", backboard_img);
        cv::waitKey();
        cv::destroyWindow("Projected Backboard");
#endif

        //Image hole filling
        cv::Mat hole_filled_img;
        img_hole_filling(backboard_img, hole_filled_img);
#if DBG
        cv::namedWindow("Backboard with holes filled", CV_WINDOW_NORMAL);
        cv::imshow("Backboard with holes filled", hole_filled_img);
        cv::waitKey();
        cv::destroyWindow("Hole filled Backboard");
#endif

        //Backbaoard border detection
        cv::Mat border_img;
        border_detection(hole_filled_img, border_img);
#if DBG
        cv::namedWindow("Backboard border", CV_WINDOW_NORMAL);
        cv::imshow("Backboard border", border_img);
        cv::waitKey();
        cv::destroyWindow("Backboard border");
#endif

        //Back-project the border image pixels to 3D points
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_border (new pcl::PointCloud<pcl::PointXYZ>);
        plane_back_projection(border_img, transform, cloud_border);
	#if DBG
        viewer.addPointCloud<pcl::PointXYZ>(cloud_border, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_border, 255.0, 255.0, 0.0), "border cloud");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "border cloud");
        viewer.spin();
#endif

        //Generate the ideal model of backbord border
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_border_model (new pcl::PointCloud<pcl::PointXYZ>);
        float w,h,res;
        w=1.90;
        h=1.25;
        res=0.03;
        for(float i = 0.0; i < w/res; ++i)
        {
            cloud_border_model->push_back(pcl::PointXYZ(i*res-w/2, -h/2, 0.0));
            cloud_border_model->push_back(pcl::PointXYZ(i*res-w/2, h/2, 0.0));
        }
        for(float i = 0.0; i < h/res; ++i)
        {
            cloud_border_model->push_back(pcl::PointXYZ(-w/2, i*res-h/2, 0.0));
            cloud_border_model->push_back(pcl::PointXYZ(w/2, i*res-h/2, 0.0));
        }

        //Match the ideal model of backboard border to the detected one using ICP
        Eigen::Matrix4f result_transform = Eigen::Matrix4f::Identity();
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_model (new pcl::PointCloud<pcl::PointXYZ>);
        if(backboard_align(cloud_border_model, cloud_border, aligned_model, result_transform))
        {
            printf("Alignment succeeded!\n");
            viewer.addPointCloud<pcl::PointXYZ>(aligned_model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(aligned_model, 255.0, 0.0, 0.0), "aligned backboard");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "aligned backboard");
            Eigen::Vector4f backboard_center;
            pcl::compute3DCentroid<pcl::PointXYZ>(*aligned_model, backboard_center);
	    printf("Estimated backboard center at (%f, %f, %f)\n", backboard_center(0), backboard_center(1), backboard_center(2));
            viewer.addSphere<pcl::PointXYZ>(pcl::PointXYZ(backboard_center(0,0), backboard_center(1,0), backboard_center(2,0)), 0.03, 255.0, 255.0, 0.0);
#if DBG
            viewer.spin();
#else
            viewer.spinOnce();
#endif
	    //calculate the offset angle of the robot
	    Eigen::Vector3f shoot_goal = (kinect_transform * backboard_center).block(0,0,3,1);
	    float theta_err;
	    offset_angle(shoot_goal, theta_err);
#if DBG
	    printf("Offset angle = %f deg\n", theta_err);
	    viewer.spin();
#endif
        }
        else
        {
            printf("Alignment failed...\n");
        }
#if KINECT
    }
#endif
    
    return 0;
}

