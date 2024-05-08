//
// Created by ros on 2/3/24.
//

#include "xmlrp/xmlrp.h"

std::string Xmlrp::get_string_param(XmlRpc::XmlRpcValue obj, std::string param_name) {
  XmlRpc::XmlRpcValue &item = obj[param_name];
  if (item.getType() == XmlRpc::XmlRpcValue::TypeString) {
    return static_cast<std::string>(item);
  }

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "invalid paramter type for %s - expected TypeString", param_name.c_str());
  return 0;
}

int Xmlrp::get_int_param(XmlRpc::XmlRpcValue obj, std::string param_name) {
  XmlRpc::XmlRpcValue &item = obj[param_name];
  if (item.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return item;
  }

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "invalid paramter type for %s - expected TypeInt", param_name.c_str());
  return 0;
}

float Xmlrp::get_float_param(XmlRpc::XmlRpcValue obj, std::string param_name) {
  XmlRpc::XmlRpcValue &item = obj[param_name];
  if (item.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<float>((double)item);
  }

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "invalid paramter type for %s - expected TypeDouble", param_name.c_str());
  return 0;
}

double Xmlrp::get_double_param(XmlRpc::XmlRpcValue obj, std::string param_name) {
  XmlRpc::XmlRpcValue &item = obj[param_name];
  if (item.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return item;
  }

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "invalid paramter type for %s - expected TypeDouble", param_name.c_str());
  return 0;
}
