

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros/ROSWrapper.h"
#include "lio/super_lio_reloc.h"


using namespace LI2Sup;

int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  ROSWrapper::Ptr data_wrapper = std::make_shared<ROSWrapper>();
  
  auto lio = std::make_shared<SuperLIO>();
  lio->setROSWrapper(data_wrapper);
  lio->init();

  std::weak_ptr<SuperLIO> weak_lio = lio;
  data_wrapper->setMeasurementReadyCallback([weak_lio]() {
    if (auto active_lio = weak_lio.lock()) {
      active_lio->process();
    }
  });

  rclcpp::spin(data_wrapper);

  lio->saveMap();
  lio->printTimeRecord();

  rclcpp::shutdown();
  return 0;
}
