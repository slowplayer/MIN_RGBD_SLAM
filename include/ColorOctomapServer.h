#ifndef COLOR_OCTOMAP_SERVER_H
#define COLOR_OCTOMAP_SERVER_H


#include "ParameterServer.h"

#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>

#include <memory>

namespace MIN_RGBD_SLAM
{
//TODO:m_octoMap need mutex
class ColorOctomapServer
{
public:
    ColorOctomapServer();
    ~ColorOctomapServer();
    void reset();
    bool save(const char* filename) const;
    
    ///Raycas cloud into the octomap
    /// @param cloud pointcloud in arbitrary frame (specified in the clouds header)
    void insertCloudCallback(const pointcloud_type::ConstPtr cloud, double max_range = -1.0);
    
    ///Raycast cloud into the octomap
    /// @param cloud pointcloud in map frame
    /// @param origin sensor location in map frame
    virtual void insertCloudCallbackCommon(boost::shared_ptr<octomap::Pointcloud> cloud,
                                           pointcloud_type::ConstPtr colors,
                                           const octomap::point3d& origin, double max_range = -1.0);

    
    ///Filter cloud by occupancy of voxels, e.g. remove points in free space
    void occupancyFilter(pointcloud_type::ConstPtr input, 
                         pointcloud_type::Ptr output, 
                         double occupancy_threshold);

    void render();
protected:
    octomap::ColorOcTree m_octoMap;
    
};
}
#endif

