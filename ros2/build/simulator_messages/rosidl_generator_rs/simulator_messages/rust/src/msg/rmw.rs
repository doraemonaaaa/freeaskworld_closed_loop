#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "simulator_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__simulator_messages__msg__SimulatorCommand() -> *const std::ffi::c_void;
}

#[link(name = "simulator_messages__rosidl_generator_c")]
extern "C" {
    fn simulator_messages__msg__SimulatorCommand__init(msg: *mut SimulatorCommand) -> bool;
    fn simulator_messages__msg__SimulatorCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SimulatorCommand>, size: usize) -> bool;
    fn simulator_messages__msg__SimulatorCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SimulatorCommand>);
    fn simulator_messages__msg__SimulatorCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SimulatorCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<SimulatorCommand>) -> bool;
}

// Corresponds to simulator_messages__msg__SimulatorCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Header

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SimulatorCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// 调用接收端的函数方法名
    pub method: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub method_params: rosidl_runtime_rs::String,

}



impl Default for SimulatorCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !simulator_messages__msg__SimulatorCommand__init(&mut msg as *mut _) {
        panic!("Call to simulator_messages__msg__SimulatorCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SimulatorCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { simulator_messages__msg__SimulatorCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { simulator_messages__msg__SimulatorCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { simulator_messages__msg__SimulatorCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SimulatorCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SimulatorCommand where Self: Sized {
  const TYPE_NAME: &'static str = "simulator_messages/msg/SimulatorCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__simulator_messages__msg__SimulatorCommand() }
  }
}


