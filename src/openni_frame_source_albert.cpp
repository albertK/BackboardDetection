#include "openni_frame_source_albert.h"
#include <pcl/io/pcd_io.h>
#include <boost/thread/mutex.hpp>
#include <boost/make_shared.hpp>
namespace OpenNIFrameSource
{
    OpenNIFrameSourceAlbert::OpenNIFrameSourceAlbert (const std::string& device_id) :
    grabber_ (device_id), most_recent_frame_ (), frame_counter_ (0), active_ (true)
    {
	boost::function<void
	(const PointCloudConstPtr&)> frame_cb = boost::bind (&OpenNIFrameSourceAlbert::onNewFrame, this, _1);
	grabber_.registerCallback (frame_cb);
	grabber_.start ();
    }
    OpenNIFrameSourceAlbert::~OpenNIFrameSourceAlbert ()
    {
	// Stop the grabber when shutting down
	grabber_.stop ();
    }
    bool
    OpenNIFrameSourceAlbert::isActive ()
    {
	return active_;
    }
    const PointCloudPtr
    OpenNIFrameSourceAlbert::snap ()
    {
	return (most_recent_frame_);
    }
    void
    OpenNIFrameSourceAlbert::onNewFrame (const PointCloudConstPtr &cloud)
    {
	mutex_.lock ();
	++frame_counter_;
	most_recent_frame_ = boost::make_shared<PointCloud> (*cloud); // Make a copy of the frame
	mutex_.unlock ();
    }
    void
    OpenNIFrameSourceAlbert::onKeyboardEvent (const pcl::visualization::KeyboardEvent & event)
    {
	// When the spacebar is pressed, trigger a frame capture
	mutex_.lock ();
	if (event.keyDown () && event.getKeySym () == "e")
	{
	    active_ = false;
	}
	mutex_.unlock ();
    }
    void
    OpenNIFrameSourceAlbert::start()
    {
	grabber_.start();
    }
    void
    OpenNIFrameSourceAlbert::stop()
    {
	grabber_.stop();
    }
}