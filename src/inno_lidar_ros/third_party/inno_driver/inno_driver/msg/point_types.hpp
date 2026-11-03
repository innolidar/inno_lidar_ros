#pragma once

// PointT 
#include <vector>
#include <inno_driver/common/inno_common.hpp>
#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push, 1)
struct Point
{
  // basic 
  float x;
  float y;
  float z;
  uint16_t intensity;
  uint16_t ring;
};

struct PointTime:public Point
{
  // time 
  double  timestamp;
};

struct PointSource:public PointTime
{
  // optional
  int32_t distance;
  int32_t horizontal;
  int32_t vertical;
  int8_t  speed;
};

struct IFW192Point:public Point
{
  uint16_t row;
  uint16_t col;
  uint16_t amb_light_cnt;   //distance_end
  uint16_t dsp_num_start;   //distance_start
  uint32_t peak_sum;
  uint32_t peak_center;
  uint32_t peak_radius_sum;
};
#pragma pack(pop)
#ifdef __cplusplus
}
#endif
template <typename T_Point>
class PointCloud
{
public:
    typedef T_Point PointT;
    typedef std::vector<PointT> VectorT;
    std::string frame_id{""};
    uint32_t    height{0};          ///< Height of point cloud
    uint32_t    width{0};           ///< Width of point cloud
    bool        is_dense{false};    ///< If is_dense is true, the point cloud does not contain NAN points,
    double      timestamp{0.0};     ///< cloud timestamp
    uint32_t    seq{0};             ///< Sequence number of message

    uint16_t    device_number;
    double      trx_temperature{0.f};
    double      main_temperature{0.f};
    uint8_t     abnormal_flag{0};
    VectorT     points{};      
public:
    virtual void PushPoint(const PointT &point) 
    {
      points.emplace_back(point);
    }
};
//typedef PointCloud<PointT>       PointCloudT;

typedef PointCloud<Point>       BasePointCloud;
typedef PointCloud<PointTime>   TimePointCloud;
typedef PointCloud<PointSource> SourcePointCloud;
typedef PointCloud<IFW192Point> IFW192SPointCloud;
