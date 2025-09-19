#include "source_driver.hpp"
#include <inno_driver/api/inno_driver.h>
#include "utility/sync_queue.hpp"
#include "publish_manager.h"
#include <thread>
namespace innolight
{
namespace lidar
{
struct SourceDriver::Impl
{
    Impl()
    {

    }
    ~Impl()
    {

    }
    inline void Init(SourceType src_type,const YAML::Node& config)
    {
        YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
        DriverParam driver_param;
        // input related
        yamlRead<uint16_t>(driver_config, "cloud_port", driver_param.input_param.local_port, 8080);
        yamlRead<std::string>(driver_config, "local_ip", driver_param.input_param.local_ip, "0.0.0.0");
        yamlRead<std::string>(driver_config, "pcap_file", driver_param.input_param.pcap_file, "");
        

        //CalibrationParam
        yamlRead<std::string>(driver_config, "check_local_ip", driver_param.calibration_param.local_ip, "0.0.0.0");
        yamlRead<int>(driver_config, "check_local_port", driver_param.calibration_param.local_port, 8081);
        yamlRead<std::string>(driver_config, "check_lidar_ip", driver_param.calibration_param.remote_ip, "0.0.0.0");
        yamlRead<int>(driver_config, "check_lidar_port", driver_param.calibration_param.remote_port, 8081);
        yamlRead<std::string>(driver_config, "calibrate_folder", driver_param.calibration_param.calibration_folders, "");
        
        //DecodeParam
        yamlRead<float>(driver_config, "min_distance", driver_param.decoder_param.min_distance, 0);
        yamlRead<float>(driver_config, "max_distance", driver_param.decoder_param.max_distance, 200);
        yamlRead<float>(driver_config, "h_start_angle", driver_param.decoder_param.h_start_angle, 0);
        yamlRead<float>(driver_config, "h_end_angle", driver_param.decoder_param.h_end_angle, 360);
        yamlRead<float>(driver_config, "v_start_angle", driver_param.decoder_param.v_start_angle, 0);
        yamlRead<float>(driver_config, "v_end_angle", driver_param.decoder_param.v_end_angle, 360);

        yamlRead<bool>(driver_config, "is_device_load_calibration", driver_param.is_device_load, false);

        yamlRead<bool>(driver_config, "pcap_repeat", m_pcap_repeat, false);
        // decoder related
        std::string lidar_type;
        yamlReadAbort<std::string>(driver_config, "lidar_type", lidar_type);
        driver_param.lidar_type = StrToLidarType(lidar_type);

        // transform
        yamlRead<float>(driver_config, "x", driver_param.decoder_param.transform_param.x, 0);
        yamlRead<float>(driver_config, "y", driver_param.decoder_param.transform_param.y, 0);
        yamlRead<float>(driver_config, "z", driver_param.decoder_param.transform_param.z, 0);
        yamlRead<float>(driver_config, "roll", driver_param.decoder_param.transform_param.roll, 0);
        yamlRead<float>(driver_config, "pitch", driver_param.decoder_param.transform_param.pitch, 0);
        yamlRead<float>(driver_config, "yaw", driver_param.decoder_param.transform_param.yaw, 0);
        
        switch (src_type)
        {
        case SourceType::MSG_FROM_LIDAR:
            driver_param.input_type = InputType::ONLINE_LIDAR;
            m_is_local_file=false;
        break;
        case SourceType::MSG_FROM_PCAP:
            driver_param.input_type = InputType::PCAP_FILE;
            m_is_local_file=true;
        break;
        }
        driver_param.Print();
        m_inno_driver=std::make_shared<InnoDriver<DrawPointCloud>>();
        
        
#ifdef ENABLE_IMU_MSG_PARSE
        std::cout << "----------------ENABLE_IMU_MSG_PARSE" << std::endl;
        m_inno_driver->RegisterImuMsgCallback(std::bind(&SourceDriver::GetImuMsg,this),std::bind(&SourceDriver::PutImuMsg,this,std::placeholders::_1));
        m_imu_msg_process_thread = std::thread(std::bind(&SourceDriver::ProcessImuMsg, this));
#endif  
        m_inno_driver->RegisterPointCloudCallBack(std::bind(&SourceDriver::Impl::PutPointCloud,this,std::placeholders::_1),
                                                std::bind(&SourceDriver::Impl::GetPointCloud,this));
        if (!m_inno_driver->InitDriver(driver_param))
        {
            INNO_ERROR << "Driver Initialize Error...." << INNO_REND;
            exit(-1);
        }
        
        m_publish_manager=std::make_shared<PublishManager>();
        
        m_publish_manager->Init(config);
    }

    inline void Start()
    {
        if(m_inno_driver->Start())
        {
            m_point_cloud_process_thread = std::thread(std::bind(&SourceDriver::Impl::ProcessPointCloud, this));
        }
    }

    inline void Stop()
    {
        m_inno_driver->Stop();
        m_to_exit_process = true;
        m_point_cloud_process_thread.join();
#ifdef ENABLE_IMU_MSG_PARSE
        m_to_imu_exit_process = true;
        m_imu_msg_process_thread.join();
#endif  
    }

    inline std::shared_ptr<BasePointCloud>  GetPointCloud(void)
    {
        if(m_free_point_cloud_queue.GetSize()>=2)
        {
            std::shared_ptr<RosPointCloud>  point_cloud = m_free_point_cloud_queue.Pop();
            if (point_cloud!= nullptr)
            {
                point_cloud->points.clear();
                return point_cloud;
            }
        }
        return std::make_shared<RosPointCloud>();
    }

    void PutPointCloud(std::shared_ptr<BasePointCloud> msg)
    {
        m_point_cloud_queue.Push(std::static_pointer_cast<RosPointCloud>(msg));
        //m_point_cloud_queue.Push(msg);
    }

    void ProcessPointCloud()
    {
        int frame_count{-1};
        bool is_load_finished{false};
        bool is_play{false};
        int frame_index{0};
        m_to_exit_process=false;
        while (!m_to_exit_process)
        {
            if(m_is_local_file)
            {
                if(is_load_finished==false)
                {
                    is_load_finished=m_inno_driver->GetLocalFileStatus(frame_count);
                    if(is_load_finished)
                    {
                        std::cout<<"is_load_finished:"<<is_load_finished<<" frame_count:"<<frame_count<<std::endl;
                    }  
                }
                else
                {
                    if(frame_count>0)
                    {
                        if(frame_index<frame_count)
                        {
                            if(is_play==false)
                            {
                                if(m_inno_driver->SetLocalFrame(frame_index))
                                {
                                    is_play=true;
                                    frame_index+=1;
                                }
                            }
                        }
                        else{
                            if(m_pcap_repeat==false)
                            {
                                continue;
                            }
                            is_play=false;
                            if(is_play==false)
                            {
                                frame_index=0;
                                if(m_inno_driver->SetLocalFrame(frame_index))
                                {
                                    is_play=true;
                                    frame_index+=1;
                                }
                            }
                            //continue;
                        }
                        
                    }
                    std::shared_ptr<RosPointCloud> msg = m_point_cloud_queue.PopWait(100);
                    if (msg == NULL)
                    {
                        continue;
                    }
                    //std::cout<<"msg->points.size():"<<msg->points.size()<<std::endl;
                    SendPointCloud(msg);
                    //m_free_point_cloud_queue.Push(msg);
                    m_free_point_cloud_queue.Push(std::static_pointer_cast<RosPointCloud>(msg));
                    is_play=false;
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }
            else
            {
                std::shared_ptr<RosPointCloud> msg = m_point_cloud_queue.PopWait(100);
                if (msg == NULL)
                {
                    continue;
                }
                SendPointCloud(msg);
                m_free_point_cloud_queue.Push(std::static_pointer_cast<RosPointCloud>(msg));
                //m_free_point_cloud_queue.Push(msg);
            }
        }
    }

    inline void PutException(const Error& msg)
    {
        switch (msg.error_code_type)
        {
        case ErrCodeType::INFO_CODE:
            INNO_INFO << msg.toString() << INNO_REND;
        break;
        case ErrCodeType::WARNING_CODE:
            INNO_WARNING << msg.toString() << INNO_REND;
        break;
        case ErrCodeType::ERROR_CODE:
            INNO_ERROR << msg.toString() << INNO_REND;
        break;
        }
    }
#ifdef ENABLE_IMU_MSG_PARSE
    std::shared_ptr<ImuMsg> GetImuMsg(void)
    {
        return nullptr;
    }
    void PutImuMsg(std::shared_ptr<ImuMsg> msg)
    {
        return ;
    }
    void ProcessImuMsg()
    {

    }
#endif
    void SendPointCloud(std::shared_ptr<RosPointCloud> msg)
    {
        if(m_publish_manager)
        {
            m_publish_manager->SendPointCloud(*msg);
        }
    }
protected:
    bool                                                m_is_local_file{false};
    bool                                                m_to_exit_process{false};
    std::thread                                         m_point_cloud_process_thread;
    SyncQueue<std::shared_ptr<RosPointCloud>>          m_free_point_cloud_queue;
    SyncQueue<std::shared_ptr<RosPointCloud>>          m_point_cloud_queue;
#ifdef ENABLE_IMU_MSG_PARSE
  
  
  bool                                                  m_to_imu_exit_process;
  std::thread                                           m_imu_msg_process_thread;
  SyncQueue<std::shared_ptr<ImuMs>>                     m_free_imu_msg_queue;
  SyncQueue<std::shared_ptr<ImuMsgg>>                   m_imu_msg_queue;
#endif
private:
    bool                                                m_pcap_repeat{false};
    std::shared_ptr<InnoDriver<DrawPointCloud>>          m_inno_driver{nullptr};
    std::shared_ptr<PublishManager>                     m_publish_manager{nullptr};
};

SourceDriver::SourceDriver()
:m_impl(std::make_shared<SourceDriver::Impl>())
{

}
inline SourceDriver::~SourceDriver()
{
    m_impl.reset();
    m_impl=nullptr;
}
inline void SourceDriver::Init(SourceType src_type,const YAML::Node& config)
{
    m_impl->Init(src_type,config);
}

inline void SourceDriver::Start()
{
    m_impl->Start();
}

inline void SourceDriver::Stop()
{
    m_impl->Stop();
}
}
}