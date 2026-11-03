#include "node_manager.h"
#include "source/source_driver.hpp"
#include <vector>
namespace innolight
{
namespace lidar
{

struct NodeManager::Impl
{
    Impl()
    {

    }
    ~Impl()
    {
        for (auto& iter : m_sources)
        {
            if (iter != nullptr)
            {
                iter->Stop();
                iter.reset();
                iter=nullptr;
            }
    }
    }
    void Init(const YAML::Node& config)
    {
        YAML::Node common_config = yamlSubNodeAbort(config, "common");
        int msg_source = 0;
        yamlRead<int>(common_config, "msg_source", msg_source, 0);

        bool send_point_cloud_ros;
        yamlRead<bool>(common_config, "send_point_cloud_ros", send_point_cloud_ros, false);

        YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");

        for (uint8_t i = 0; i < lidar_config.size(); ++i)
        {
            std::shared_ptr<SourceDriver> source;
            switch (msg_source)
            {
            case SourceType::MSG_FROM_LIDAR:  // online lidar
                INNO_INFO << "------------------------------------------------------" << INNO_REND;
                INNO_INFO << "Receive Packets From : Online LiDAR" << INNO_REND;
                INNO_INFO << "Point Cloud for UDP Port: " << lidar_config[i]["driver"]["cloud_port"].as<uint16_t>() << INNO_REND;
                INNO_INFO << "------------------------------------------------------" << INNO_REND;

                source = std::make_shared<SourceDriver>();
                source->Init(SourceType::MSG_FROM_LIDAR,lidar_config[i]);
            break;
            case SourceType::MSG_FROM_PCAP:  // pcap

                INNO_INFO << "------------------------------------------------------" << INNO_REND;
                INNO_INFO << "Receive Packets From : Pcap" << INNO_REND;
                INNO_INFO << "Point Cloud for UDP Port: " << lidar_config[i]["driver"]["cloud_port"].as<uint16_t>() << INNO_REND;
                INNO_INFO << "------------------------------------------------------" << INNO_REND;

                source = std::make_shared<SourceDriver>();
                source->Init(SourceType::MSG_FROM_PCAP,lidar_config[i]);
            break;
            default:
                INNO_ERROR << "Unsupported LiDAR message source:" << msg_source << "." << INNO_REND;
            exit(-1);
            }
            m_sources.emplace_back(source);
        }
    }
    void Start()
    {
        for (auto& item : m_sources)
        {
            if (item != nullptr)
            {
                item->Start();
            }
        }
    }
    void Stop()
    {
        for (auto& item : m_sources)
        {
            if (item != nullptr)
            {
                item->Stop();
                item.reset();
                item=nullptr;
            }
        }
    }
private:
    std::vector<SourceDriver::Ptr> m_sources;
};

NodeManager::~NodeManager()
{
    m_impl.reset();
    m_impl=nullptr;
}

void NodeManager::Init(const YAML::Node& config)
{
    if(m_impl==nullptr)
    {
        m_impl=std::make_shared<NodeManager::Impl>();
    }
    m_impl->Init(config);
}
void NodeManager::Start()
{
    m_impl->Start();
}

void NodeManager::Stop()
{
    m_impl->Stop();
}
}
}