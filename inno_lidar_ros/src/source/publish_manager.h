#pragma once
#include "utility/yaml_reader.hpp"
#include "msg/cloud_types.hpp"
#include <memory>
namespace innolight
{
namespace lidar
{
class PublishManager
{
public:
    PublishManager();
    virtual ~PublishManager();
    void Init(const YAML::Node& config);
    void SendPointCloud(const RosPointCloud& msg);
private:
    struct Impl;
    std::shared_ptr<Impl> m_impl{nullptr};
};
}
}
