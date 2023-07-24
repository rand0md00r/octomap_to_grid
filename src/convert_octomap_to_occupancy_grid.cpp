#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
using namespace ros;

class OctomapToGridmap
{
public:
    OctomapToGridmap()
    {
        n.param<double>("min_height", _min_height, 0.1);
        n.param<double>("max_height", _max_height, 1.0);

        // Initialize subscriber
        octomap_sub = n.subscribe("/octomap_full", 1000, &OctomapToGridmap::callback, this);

        // Initialize publisher
        gridmap_pub = n.advertise<nav_msgs::OccupancyGrid>("/convert_map", 1000);
    }

    void callback(const octomap_msgs::Octomap::ConstPtr& msg);

private:
    NodeHandle n;
    Subscriber octomap_sub;
    Publisher gridmap_pub;

    double _min_height, _max_height;
};

void OctomapToGridmap::callback(const octomap_msgs::Octomap::ConstPtr& msg)
{
    // Check that the octomap is binary
    if(msg->id != "OcTree"){
        ROS_ERROR("Octomap is not binary");
        return;
    }

    // Convert the binary octomap msg to an octree
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));
    

    // Check that conversion was successful
    if(octree == NULL){
        ROS_ERROR("Failed to convert Octomap msg to OcTree");
        return;
    }

    // Initialize OccupancyGrid
    nav_msgs::OccupancyGrid gridmap;

    // Set map size and resolution
    double resolution = octree->getResolution();
    gridmap.info.resolution = resolution;

    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree->getMetricMin(min_x, min_y, min_z);
    octree->getMetricMax(max_x, max_y, max_z);
    gridmap.info.width = (max_x - min_x) / resolution;
    gridmap.info.height = (max_y - min_y)/ resolution;

    cout << " msg->header.frame_id: " <<  msg->header.frame_id << endl;
    gridmap.header.frame_id = msg->header.frame_id;
    gridmap.info.origin.position.x = min_x;
    gridmap.info.origin.position.y = min_y;

    // Reserve size for map data
    gridmap.data.resize(gridmap.info.width * gridmap.info.height, 0);

    // Iterate over the octree
    double grid_ = 0.0;
    double grid_heigit = 0.0;

    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it){
        octomap::OcTreeNode node = *it;
        double occupancy_probability = node.getOccupancy();
        cout << "occupancy_probability: " << occupancy_probability << endl;
        int occupancy_percentage = static_cast<int>(occupancy_probability * 100.0);

        if(it.getZ() > _min_height && it.getZ() < _max_height)
        {
            if (octree->isNodeOccupied(*it))
            {
                int index = round((it.getX() - resolution / 2 - min_x) / resolution)  + round((gridmap.info.width * (it.getY()  - resolution / 2 - min_y)) / resolution);
                if(index > 0 && index < gridmap.info.width * gridmap.info.height)
                {
                    gridmap.data[index] = occupancy_percentage;
                }
            }
        }
    }

    // Publish the gridmap
    gridmap_pub.publish(gridmap);

    // Don't forget to delete the octree
    delete octree;
}



int main(int argc, char** argv)
{
    // Initialize ROS
    init(argc, argv, "octomap_to_gridmap");

    // Create a new OctomapToGridmap object
    OctomapToGridmap otg;

    // Spin
    spin();

    return 0;
}