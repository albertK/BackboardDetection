#ifndef OPENNI_CAPTURE_H
#define OPENNI_CAPTURE_H
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
namespace OpenNIFrameSource
{
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
    /* A simple class for capturing data from an OpenNI camera */
    class PCL_EXPORTS OpenNIFrameSourceAlbert
    {
    public:
	OpenNIFrameSourceAlbert (const std::string& device_id = "");
	~OpenNIFrameSourceAlbert ();
	const PointCloudPtr
	snap ();
	bool
	isActive ();
	void
	onKeyboardEvent (const pcl::visualization::KeyboardEvent & event);
	void start ();
	void stop ();
    protected:
	void
	onNewFrame (const PointCloudConstPtr &cloud);
	pcl::OpenNIGrabber grabber_;
	int frame_counter_;
	PointCloudPtr most_recent_frame_;
	boost::mutex mutex_;
	bool active_;
    };
}
#endif