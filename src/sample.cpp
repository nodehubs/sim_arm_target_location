// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cv_bridge/cv_bridge.h>
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "hobot_cv/hobotcv_imgproc.h"
#include "sensor_msgs/msg/image.hpp"

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"

#include "sim_arm_location_msg/srv/choose_cube.hpp"
#include "sim_arm_location_msg/srv/target_location.hpp"
#include "sim_arm_target_location/parser.h"
#include "sim_arm_target_location/image_utils.h"
#include "sim_arm_target_location/positioning.h"

// 使用hobotcv resize nv12格式图片，固定图片宽高比

static int ResizeNV12Img(const char *in_img_data,
                         const int &in_img_height,
                         const int &in_img_width,
                         const int &scaled_img_height,
                         const int &scaled_img_width,
                         cv::Mat &out_img,
                         float &ratio)
{
  cv::Mat src(
      in_img_height * 3 / 2, in_img_width, CV_8UC1, (void *)(in_img_data));
  float ratio_w =
      static_cast<float>(in_img_width) / static_cast<float>(scaled_img_width);
  float ratio_h =
      static_cast<float>(in_img_height) / static_cast<float>(scaled_img_height);
  float dst_ratio = std::max(ratio_w, ratio_h);
  int resized_width, resized_height;
  if (dst_ratio == ratio_w)
  {
    resized_width = scaled_img_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  }
  else if (dst_ratio == ratio_h)
  {
    resized_width = static_cast<float>(in_img_width) / dst_ratio;
    resized_height = scaled_img_height;
  }

  // hobot_cv要求输出宽度为16的倍数
  int remain = resized_width % 16;
  if (remain != 0)
  {
    // 向下取16倍数，重新计算缩放系数
    resized_width -= remain;
    dst_ratio = static_cast<float>(in_img_width) / resized_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  }
  // 高度向下取偶数
  resized_height =
      resized_height % 2 == 0 ? resized_height : resized_height - 1;
  ratio = dst_ratio;

  return hobot_cv::hobotcv_resize(
      src, in_img_height, in_img_width, out_img, resized_height, resized_width);
}

static int InitClassNames(const std::string &cls_name_file, sim_arm_target_location::PTQYolo5Config &yolo5_config)
{
  std::ifstream fi(cls_name_file);
  if (fi)
  {
    yolo5_config.class_names.clear();
    std::string line;
    while (std::getline(fi, line))
    {
      yolo5_config.class_names.push_back(line);
    }
    int size = yolo5_config.class_names.size();
    if (size != yolo5_config.class_num)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
                   "class_names length %d is not equal to class_num %d",
                   size, yolo5_config.class_num);
      return -1;
    }
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
                 "can not open cls name file: %s",
                 cls_name_file.c_str());
    return -1;
  }
  return 0;
}

static int InitClassNum(const int &class_num, sim_arm_target_location::PTQYolo5Config &yolo5_config)
{
  if (class_num > 0)
  {
    yolo5_config.class_num = class_num;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
                 "class_num = %d is not allowed, only support class_num > 0",
                 class_num);
    return -1;
  }
  return 0;
}

static void LoadConfig(const std::string &config_file, sim_arm_target_location::PTQYolo5Config &yolo5_config)
{
  if (config_file.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("LoadConfig"),
                 "Config file [%s] is empty!",
                 config_file.data());
    return;
  }
  // Parsing config
  std::ifstream ifs(config_file.c_str());
  if (!ifs)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LoadConfig"),
                 "Read config file [%s] fail!",
                 config_file.data());
    return;
  }
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError())
  {
    RCLCPP_ERROR(rclcpp::get_logger("LoadConfig"),
                 "Parsing config file %s failed",
                 config_file.data());
    return;
  }

  if (document.HasMember("class_num"))
  {
    int class_num = document["class_num"].GetInt();
    if (InitClassNum(class_num, yolo5_config) < 0)
    {
      return;
    }
  }
  if (document.HasMember("cls_names_list"))
  {
    std::string cls_name_file = document["cls_names_list"].GetString();
    if (InitClassNames(cls_name_file, yolo5_config) < 0)
    {
      return;
    }
  }
  return;
}

struct TargetLocationNodeOutput : public hobot::dnn_node::DnnNodeOutput
{
  // 缩放比例系数，原图和模型输入分辨率的比例。
  float ratio = 1.904;
};

// 继承DnnNode虚基类，创建算法推理节点
class TargetLocationNode : public hobot::dnn_node::DnnNode
{
public:
  TargetLocationNode(const std::string &node_name = "TargetLocationNode",
                     const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  // ~TargetLocationNode();
protected:
  // 实现基类的纯虚接口，用于配置Node参数
  int SetNodePara() override;
  // 实现基类的虚接口，将解析后结构化的算法输出数据封装成ROS Msg后发布
  int PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &
                      node_output) override;

private:
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      hbm_img_subscription_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;

  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_ = nullptr;
  rclcpp::Service<sim_arm_location_msg::srv::ChooseCube>::SharedPtr choose_cube_service_ = nullptr;
  rclcpp::Client<sim_arm_location_msg::srv::TargetLocation>::SharedPtr pick_target_client_ = nullptr;

  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr callback_group_service_ = nullptr;

  std::shared_ptr<std::vector<std::shared_ptr<sim_arm_target_location::Target2D>>> solving_targets_ = nullptr;

  int model_input_width_ = -1;
  int model_input_height_ = -1;
  bool has_val_ = false;
  bool has_request_ = false;
  bool is_shared_mem_sub_ = true;
  std::mutex target_mutex_;
  std::string sub_img_topic_ = "/hbmem_img";
  std::string config_file_ = "config/number_cube.json";

  sim_arm_target_location::PTQYolo5Config yolo5_config_ = {
      {8, 16, 32},
      {{{10, 13}, {16, 30}, {33, 23}},
       {{30, 61}, {62, 45}, {59, 119}},
       {{116, 90}, {156, 198}, {373, 326}}},
      1,
      {"num3_cube", "num2_cube", "num1_cube"}};

  void FeedHbmImg(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
  void FeedImg(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void ServiceCallBack(const sim_arm_location_msg::srv::ChooseCube::Request::SharedPtr request,
                       const sim_arm_location_msg::srv::ChooseCube::Response::SharedPtr response);
};

TargetLocationNode::TargetLocationNode(const std::string &node_name,
                                       const rclcpp::NodeOptions &options)
    : hobot::dnn_node::DnnNode(node_name, options)
{

  this->declare_parameter<std::string>("sub_img_topic", sub_img_topic_);
  this->declare_parameter<std::string>("config_file", config_file_);
  this->declare_parameter<bool>("is_shared_mem_sub", is_shared_mem_sub_);

  this->get_parameter<std::string>("sub_img_topic", sub_img_topic_);
  this->get_parameter<std::string>("config_file", config_file_);
  this->get_parameter<bool>("is_shared_mem_sub", is_shared_mem_sub_);

  if (Init() != 0 ||
      GetModelInputSize(0, model_input_width_, model_input_height_) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TargetLocationNode"), "Node init fail!");
    rclcpp::shutdown();
  }
  LoadConfig(config_file_, yolo5_config_);

  solving_targets_ = std::make_shared<std::vector<std::shared_ptr<sim_arm_target_location::Target2D>>>();

  callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_subscriber_;

  callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if (is_shared_mem_sub_ == true)
  {
    hbm_img_subscription_ =
        this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            sub_img_topic_,
            10,
            std::bind(&TargetLocationNode::FeedHbmImg, this, std::placeholders::_1), sub_opt);
  }
  else
  {
    ros_img_subscription_ =
        this->create_subscription<sensor_msgs::msg::Image>(
            sub_img_topic_,
            10,
            std::bind(&TargetLocationNode::FeedImg, this, std::placeholders::_1), sub_opt);
  }

  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>("/sim_arm_target_detection", 10);
  choose_cube_service_ = this->create_service<sim_arm_location_msg::srv::ChooseCube>(
      "/sim_arm_target_location/choose_cube",
      std::bind(&TargetLocationNode::ServiceCallBack, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_default,
      callback_group_service_);
  pick_target_client_ = this->create_client<sim_arm_location_msg::srv::TargetLocation>("/sim_arm_target_location/target_location");
}

int TargetLocationNode::SetNodePara()
{
  if (!dnn_node_para_ptr_)
    return -1;
  // 指定算法推理使用的模型文件路径
  std::ifstream ifs(config_file_.c_str());
  if (!ifs)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SetNodePara"),
                 "Read config file [%s] fail!",
                 config_file_.data());
    return -1;
  }
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError())
  {
    RCLCPP_ERROR(rclcpp::get_logger("SetNodePara"),
                 "Parsing config file %s failed",
                 config_file_.data());
    return -1;
  }

  std::string model_file;
  if (document.HasMember("model_file"))
  {
    model_file = document["model_file"].GetString();
  }
  dnn_node_para_ptr_->model_file = model_file;
  // 指定算法推理任务类型
  // 本示例使用的人体检测算法输入为单张图片，对应的算法推理任务类型为ModelInferType
  // 只有当算法输入为图片和roi（Region of
  // Interest，例如目标的检测框）时，算法推理任务类型为ModelRoiInferType
  dnn_node_para_ptr_->model_task_type =
      hobot::dnn_node::ModelTaskType::ModelInferType;
  // 指定算法推理使用的任务数量，YOLOv5算法推理耗时较长，指定使用4个任务进行推理
  dnn_node_para_ptr_->task_num = 4;
  // 不通过bpu_core_ids参数指定算法推理使用的BPU核，使用负载均衡模式
  // dnn_node_para_ptr_->bpu_core_ids.push_back(hobot::dnn_node::BPUCoreIDType::BPU_CORE_0);
  return 0;
}

void TargetLocationNode::FeedHbmImg(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg)
{
  if (!rclcpp::ok() || !img_msg)
  {
    return;
  }
  // 1 对订阅到的图片消息进行验证，本示例只支持处理NV12格式图片数据
  // 如果是其他格式图片，订阅hobot_codec解码/转码后的图片消息
  if ("nv12" !=
      std::string(reinterpret_cast<const char *>(img_msg->encoding.data())))
  {
    RCLCPP_ERROR(rclcpp::get_logger("TargetLocationNode"),
                 "Only support nv12 img encoding! Using hobot codec to process "
                 "%d encoding img.",
                 img_msg->encoding.data());
    return;
  }

  // 2 创建算法输出数据，填充消息头信息，用于推理完成后AI结果的发布
  auto dnn_output = std::make_shared<TargetLocationNodeOutput>();
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->msg_header->set__stamp(img_msg->time_stamp);

  // 3 算法前处理，即创建算法输入数据
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  if (img_msg->height != static_cast<uint32_t>(model_input_height_) ||
      img_msg->width != static_cast<uint32_t>(model_input_width_))
  {
    // 3.1 订阅到的图片和算法输入分辨率不一致，需要做resize处理
    cv::Mat out_img;
    if (ResizeNV12Img(reinterpret_cast<const char *>(img_msg->data.data()),
                      img_msg->height,
                      img_msg->width,
                      model_input_height_,
                      model_input_width_,
                      out_img,
                      dnn_output->ratio) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("TargetLocationNode"),
                   "Resize nv12 img fail!");
      return;
    }

    uint32_t out_img_width = out_img.cols;
    uint32_t out_img_height = out_img.rows * 2 / 3;
    // 3.2 根据算法输入图片分辨率，使用hobot_dnn中提供的方法创建算法输入数据
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(out_img.data),
        out_img_height,
        out_img_width,
        model_input_height_,
        model_input_width_);
  }
  else
  {
    // 3.3
    // 不需要进行resize，直接根据算法输入图片分辨率，使用hobot_dnn中提供的方法创建算法输入数据
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        model_input_height_,
        model_input_width_);
  }
  // 3.4 校验算法输入数据
  if (!pyramid)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TargetLocationNode"), "Get pym fail");
    return;
  }
  // 3.5 将算法输入数据转成dnn node推理输入的格式
  auto inputs =
      std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{pyramid};

  // 4
  // 使用创建的算法输入和输出数据，以异步模式运行推理，推理结果通过PostProcess接口回调返回
  if (Run(inputs, dnn_output, nullptr, false) < 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("TargetLocationNode"), "Run predict fail!");
  }
}

void TargetLocationNode::FeedImg(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  if (!rclcpp::ok() || !img_msg)
  {
    return;
  }
  // 2 创建算法输出数据，填充消息头信息，用于推理完成后AI结果的发布
  auto dnn_output = std::make_shared<TargetLocationNodeOutput>();
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->msg_header->set__stamp(img_msg->header.stamp);
  auto cv_img = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");
  // 3 算法前处理，即创建算法输入数据
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  pyramid = ImageUtils::GetNV12Pyramid(
      cv_img->image, model_input_height_, model_input_width_);
  // 3.4 校验算法输入数据
  if (!pyramid)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TargetLocationNode"), "Get pym fail");
    return;
  }
  // // 3.5 将算法输入数据转成dnn node推理输入的格式
  auto inputs =
      std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{pyramid};

  // // 4
  // // 使用创建的算法输入和输出数据，以异步模式运行推理，推理结果通过PostProcess接口回调返回
  if (Run(inputs, dnn_output, nullptr, false) < 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("TargetLocationNode"), "Run predict fail!");
  }
}

// 推理结果回调，解析算法输出，通过ROS Msg发布消息
int TargetLocationNode::PostProcess(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output)
{
  if (!rclcpp::ok())
  {
    return 0;
  }

  // 后处理开始时间
  auto tp_start = std::chrono::system_clock::now();

  // 1 创建用于发布推理结果的ROS Msg
  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());

  // 2 将推理输出对应图片的消息头填充到ROS Msg
  pub_data->set__header(*node_output->msg_header);

  // 3 使用自定义的Parse解析方法，解析算法输出的DNNTensor类型数据
  // 3.1
  // 创建解析输出数据，输出YoloV5Result是自定义的算法输出数据类型，results的维度等于检测出来的目标数
  std::vector<std::shared_ptr<sim_arm_target_location::YoloV5Result>>
      results;

  // 3.2 开始解析
  if (sim_arm_target_location::Parse(node_output, results, yolo5_config_) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TargetLocationNode"),
                 "Parse node_output fail!");
    return -1;
  }

  // 3.3 使用解析后的数据填充到ROS Msg
  for (auto &rect : results)
  {
    if (!rect)
      continue;
    if (rect->xmin < 0)
      rect->xmin = 0;
    if (rect->ymin < 0)
      rect->ymin = 0;
    if (rect->xmax >= model_input_width_)
    {
      rect->xmax = model_input_width_ - 1;
    }
    if (rect->ymax >= model_input_height_)
    {
      rect->ymax = model_input_height_ - 1;
    }

    std::stringstream ss;
    ss << "det rect: " << rect->xmin << " " << rect->ymin << " " << rect->xmax
       << " " << rect->ymax << ", det type: " << rect->class_name
       << ", score:" << rect->score;
    RCLCPP_INFO(rclcpp::get_logger("TargetLocationNode"), "%s", ss.str().c_str());

    ai_msgs::msg::Roi roi;
    roi.rect.set__x_offset(rect->xmin);
    roi.rect.set__y_offset(rect->ymin);
    roi.rect.set__width(rect->xmax - rect->xmin);
    roi.rect.set__height(rect->ymax - rect->ymin);
    roi.set__confidence(rect->score);

    ai_msgs::msg::Target target;
    target.set__type(rect->class_name);
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));
  }

  auto sample_node_output =
      std::dynamic_pointer_cast<TargetLocationNodeOutput>(node_output);
  if (!sample_node_output)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TargetLocationNode"),
                 "Cast dnn node output fail!");
    return -1;
  }

  if (sample_node_output->ratio != 1.0)
  {
    // 前处理有对图片进行resize，需要将坐标映射到对应的订阅图片分辨率
    for (auto &target : pub_data->targets)
    {
      for (auto &roi : target.rois)
      {
        roi.rect.x_offset *= sample_node_output->ratio;
        roi.rect.y_offset *= sample_node_output->ratio;
        roi.rect.width *= sample_node_output->ratio;
        roi.rect.height *= sample_node_output->ratio;
      }
    }
  }

  // 当有请求时，将结果填充给solving_targets_
  std::unique_lock<std::mutex> lock(target_mutex_);
  if (has_request_ == true && solving_targets_->empty() == true)
  {
    for (auto &target : pub_data->targets)
    {
      auto solving_target = std::make_shared<sim_arm_target_location::Target2D>(
          int(target.rois[0].rect.x_offset + target.rois[0].rect.width / 2),
          int(target.rois[0].rect.y_offset + target.rois[0].rect.height / 2),
          target.type);
      solving_targets_->emplace_back(solving_target);
    }
  }
  lock.unlock();

  if (node_output->rt_stat)
  {
    pub_data->set__fps(round(node_output->rt_stat->output_fps));
    // 如果算法推理统计有更新，输出算法输入和输出的帧率统计、推理耗时
    if (node_output->rt_stat->fps_updated)
    {
      // 后处理结束时间
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start)
                          .count();
      RCLCPP_WARN(rclcpp::get_logger("TargetLocationNode"),
                  "input fps: %.2f, out fps: %.2f, infer time ms: %d, "
                  "post process time ms: %d",
                  node_output->rt_stat->input_fps,
                  node_output->rt_stat->output_fps,
                  node_output->rt_stat->infer_time_ms,
                  interval);
    }
  }
  // 6 发布ROS Msg
  msg_publisher_->publish(std::move(pub_data));

  return 0;
}

// 接收数字的请求，计算出对应数字的方块在机械臂坐标系下的3D位置，并请求机械臂控制节点夹取物体
void TargetLocationNode::ServiceCallBack(
    const sim_arm_location_msg::srv::ChooseCube::Request::SharedPtr request,
    const sim_arm_location_msg::srv::ChooseCube::Response::SharedPtr response)
{
  if (request->num > 3 || request->num <= 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("TargetLocationNode"),
                "Please enter a number between 1 and 3 , your num is %d ",
                request->num);
    response->success = false;
  }
  else
  {
    std::unique_lock<std::mutex> lock(target_mutex_);
    has_request_ = true;
    lock.unlock();
    while (!pick_target_client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupt waiting");
        response->success = false;
        rclcpp::shutdown();
        return;
      }
      RCLCPP_WARN(this->get_logger(), "Waiting for service to start");
    }
    while (1)
    {
      std::unique_lock<std::mutex> lock(target_mutex_);
      if (solving_targets_->empty() != true)
      {
        auto target_location = std::make_shared<sim_arm_target_location::Location3D>(0.0, 0.0, 0.0);
        auto res_solving = sim_arm_target_location::PositionSolving(solving_targets_, request->num, target_location);
        if (res_solving == false)
        {
          response->success = false;
        }
        else
        {
          auto target_location_request = std::make_shared<sim_arm_location_msg::srv::TargetLocation::Request>();
          target_location_request->x = target_location->x;
          target_location_request->y = target_location->y;
          target_location_request->z = target_location->z;
          RCLCPP_WARN(rclcpp::get_logger("TargetLocationNode"),
                      "Target position x:%f y:%f z:%f", target_location->x, target_location->y, target_location->z);
          auto result = pick_target_client_->async_send_request(target_location_request);
          result.wait();
          response->success = result.get()->success;
          solving_targets_->clear();
          has_request_ = false;
        }
        break;
      }
      lock.unlock();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  return;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetLocationNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
