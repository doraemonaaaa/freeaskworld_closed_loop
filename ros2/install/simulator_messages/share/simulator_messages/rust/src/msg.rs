#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to simulator_messages__msg__SimulatorCommand
/// Header

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SimulatorCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// 调用接收端的函数方法名
    pub method: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub method_params: std::string::String,

}



impl Default for SimulatorCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SimulatorCommand::default())
  }
}

impl rosidl_runtime_rs::Message for SimulatorCommand {
  type RmwMsg = super::msg::rmw::SimulatorCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        method: msg.method.as_str().into(),
        method_params: msg.method_params.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        method: msg.method.as_str().into(),
        method_params: msg.method_params.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      method: msg.method.to_string(),
      method_params: msg.method_params.to_string(),
    }
  }
}


