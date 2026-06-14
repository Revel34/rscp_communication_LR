
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_Goal() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__SearchArea_Goal__init(msg: *mut SearchArea_Goal) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_Goal>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_Goal>);
    fn rscp_mission_interfaces__action__SearchArea_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SearchArea_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<SearchArea_Goal>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__SearchArea_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub lat: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lon: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub radius: f64,

}



impl Default for SearchArea_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__SearchArea_Goal__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__SearchArea_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SearchArea_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SearchArea_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SearchArea_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/SearchArea_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_Goal() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_Result() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__SearchArea_Result__init(msg: *mut SearchArea_Result) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_Result>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_Result>);
    fn rscp_mission_interfaces__action__SearchArea_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SearchArea_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<SearchArea_Result>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__SearchArea_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub found_lat: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub found_lon: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for SearchArea_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__SearchArea_Result__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__SearchArea_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SearchArea_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SearchArea_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SearchArea_Result where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/SearchArea_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_Result() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__SearchArea_Feedback__init(msg: *mut SearchArea_Feedback) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_Feedback>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_Feedback>);
    fn rscp_mission_interfaces__action__SearchArea_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SearchArea_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<SearchArea_Feedback>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__SearchArea_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub current_lat: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_lon: f64,

}



impl Default for SearchArea_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__SearchArea_Feedback__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__SearchArea_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SearchArea_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SearchArea_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SearchArea_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/SearchArea_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_Feedback() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__SearchArea_FeedbackMessage__init(msg: *mut SearchArea_FeedbackMessage) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_FeedbackMessage>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_FeedbackMessage>);
    fn rscp_mission_interfaces__action__SearchArea_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SearchArea_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<SearchArea_FeedbackMessage>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__SearchArea_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::SearchArea_Feedback,

}



impl Default for SearchArea_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__SearchArea_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__SearchArea_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SearchArea_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SearchArea_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SearchArea_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/SearchArea_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_FeedbackMessage() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_Goal() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__NavigateToGPS_Goal__init(msg: *mut NavigateToGPS_Goal) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_Goal>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_Goal>);
    fn rscp_mission_interfaces__action__NavigateToGPS_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToGPS_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_Goal>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub lat: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lon: f64,

}



impl Default for NavigateToGPS_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__NavigateToGPS_Goal__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__NavigateToGPS_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToGPS_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToGPS_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/NavigateToGPS_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_Goal() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_Result() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__NavigateToGPS_Result__init(msg: *mut NavigateToGPS_Result) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_Result>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_Result>);
    fn rscp_mission_interfaces__action__NavigateToGPS_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToGPS_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_Result>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for NavigateToGPS_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__NavigateToGPS_Result__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__NavigateToGPS_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToGPS_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToGPS_Result where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/NavigateToGPS_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_Result() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__NavigateToGPS_Feedback__init(msg: *mut NavigateToGPS_Feedback) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_Feedback>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_Feedback>);
    fn rscp_mission_interfaces__action__NavigateToGPS_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToGPS_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_Feedback>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_remaining: f64,

}



impl Default for NavigateToGPS_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__NavigateToGPS_Feedback__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__NavigateToGPS_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToGPS_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToGPS_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/NavigateToGPS_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_Feedback() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage__init(msg: *mut NavigateToGPS_FeedbackMessage) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_FeedbackMessage>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_FeedbackMessage>);
    fn rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToGPS_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_FeedbackMessage>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::NavigateToGPS_Feedback,

}



impl Default for NavigateToGPS_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToGPS_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToGPS_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/NavigateToGPS_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_Goal() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__Explore_Goal__init(msg: *mut Explore_Goal) -> bool;
    fn rscp_mission_interfaces__action__Explore_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Explore_Goal>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__Explore_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Explore_Goal>);
    fn rscp_mission_interfaces__action__Explore_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Explore_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<Explore_Goal>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__Explore_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for Explore_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__Explore_Goal__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__Explore_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Explore_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Explore_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Explore_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/Explore_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_Goal() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_Result() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__Explore_Result__init(msg: *mut Explore_Result) -> bool;
    fn rscp_mission_interfaces__action__Explore_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Explore_Result>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__Explore_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Explore_Result>);
    fn rscp_mission_interfaces__action__Explore_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Explore_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<Explore_Result>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__Explore_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for Explore_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__Explore_Result__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__Explore_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Explore_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Explore_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Explore_Result where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/Explore_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_Result() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__Explore_Feedback__init(msg: *mut Explore_Feedback) -> bool;
    fn rscp_mission_interfaces__action__Explore_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Explore_Feedback>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__Explore_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Explore_Feedback>);
    fn rscp_mission_interfaces__action__Explore_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Explore_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<Explore_Feedback>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__Explore_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub distance: f64,

}



impl Default for Explore_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__Explore_Feedback__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__Explore_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Explore_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Explore_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Explore_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/Explore_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_Feedback() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__Explore_FeedbackMessage__init(msg: *mut Explore_FeedbackMessage) -> bool;
    fn rscp_mission_interfaces__action__Explore_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Explore_FeedbackMessage>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__Explore_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Explore_FeedbackMessage>);
    fn rscp_mission_interfaces__action__Explore_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Explore_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<Explore_FeedbackMessage>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__Explore_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::Explore_Feedback,

}



impl Default for Explore_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__Explore_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__Explore_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Explore_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Explore_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Explore_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/Explore_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_FeedbackMessage() }
  }
}




#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__SearchArea_SendGoal_Request__init(msg: *mut SearchArea_SendGoal_Request) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_SendGoal_Request>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_SendGoal_Request>);
    fn rscp_mission_interfaces__action__SearchArea_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SearchArea_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SearchArea_SendGoal_Request>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__SearchArea_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::SearchArea_Goal,

}



impl Default for SearchArea_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__SearchArea_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__SearchArea_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SearchArea_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SearchArea_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SearchArea_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/SearchArea_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_SendGoal_Request() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__SearchArea_SendGoal_Response__init(msg: *mut SearchArea_SendGoal_Response) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_SendGoal_Response>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_SendGoal_Response>);
    fn rscp_mission_interfaces__action__SearchArea_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SearchArea_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SearchArea_SendGoal_Response>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__SearchArea_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for SearchArea_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__SearchArea_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__SearchArea_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SearchArea_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SearchArea_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SearchArea_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/SearchArea_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_SendGoal_Response() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__SearchArea_GetResult_Request__init(msg: *mut SearchArea_GetResult_Request) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_GetResult_Request>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_GetResult_Request>);
    fn rscp_mission_interfaces__action__SearchArea_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SearchArea_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SearchArea_GetResult_Request>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__SearchArea_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for SearchArea_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__SearchArea_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__SearchArea_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SearchArea_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SearchArea_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SearchArea_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/SearchArea_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_GetResult_Request() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__SearchArea_GetResult_Response__init(msg: *mut SearchArea_GetResult_Response) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_GetResult_Response>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__SearchArea_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SearchArea_GetResult_Response>);
    fn rscp_mission_interfaces__action__SearchArea_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SearchArea_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SearchArea_GetResult_Response>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__SearchArea_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::SearchArea_Result,

}



impl Default for SearchArea_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__SearchArea_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__SearchArea_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SearchArea_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__SearchArea_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SearchArea_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SearchArea_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/SearchArea_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__SearchArea_GetResult_Response() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request__init(msg: *mut NavigateToGPS_SendGoal_Request) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_SendGoal_Request>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_SendGoal_Request>);
    fn rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToGPS_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_SendGoal_Request>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::NavigateToGPS_Goal,

}



impl Default for NavigateToGPS_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToGPS_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToGPS_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/NavigateToGPS_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response__init(msg: *mut NavigateToGPS_SendGoal_Response) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_SendGoal_Response>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_SendGoal_Response>);
    fn rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToGPS_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_SendGoal_Response>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for NavigateToGPS_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToGPS_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToGPS_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/NavigateToGPS_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request__init(msg: *mut NavigateToGPS_GetResult_Request) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_GetResult_Request>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_GetResult_Request>);
    fn rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToGPS_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_GetResult_Request>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for NavigateToGPS_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToGPS_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToGPS_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/NavigateToGPS_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response__init(msg: *mut NavigateToGPS_GetResult_Response) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_GetResult_Response>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_GetResult_Response>);
    fn rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToGPS_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToGPS_GetResult_Response>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::NavigateToGPS_Result,

}



impl Default for NavigateToGPS_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToGPS_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToGPS_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/NavigateToGPS_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__Explore_SendGoal_Request__init(msg: *mut Explore_SendGoal_Request) -> bool;
    fn rscp_mission_interfaces__action__Explore_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Explore_SendGoal_Request>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__Explore_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Explore_SendGoal_Request>);
    fn rscp_mission_interfaces__action__Explore_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Explore_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Explore_SendGoal_Request>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__Explore_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::Explore_Goal,

}



impl Default for Explore_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__Explore_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__Explore_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Explore_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Explore_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Explore_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/Explore_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_SendGoal_Request() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__Explore_SendGoal_Response__init(msg: *mut Explore_SendGoal_Response) -> bool;
    fn rscp_mission_interfaces__action__Explore_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Explore_SendGoal_Response>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__Explore_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Explore_SendGoal_Response>);
    fn rscp_mission_interfaces__action__Explore_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Explore_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Explore_SendGoal_Response>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__Explore_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for Explore_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__Explore_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__Explore_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Explore_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Explore_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Explore_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/Explore_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_SendGoal_Response() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__Explore_GetResult_Request__init(msg: *mut Explore_GetResult_Request) -> bool;
    fn rscp_mission_interfaces__action__Explore_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Explore_GetResult_Request>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__Explore_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Explore_GetResult_Request>);
    fn rscp_mission_interfaces__action__Explore_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Explore_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Explore_GetResult_Request>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__Explore_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for Explore_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__Explore_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__Explore_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Explore_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Explore_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Explore_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/Explore_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_GetResult_Request() }
  }
}


#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "rscp_mission_interfaces__rosidl_generator_c")]
extern "C" {
    fn rscp_mission_interfaces__action__Explore_GetResult_Response__init(msg: *mut Explore_GetResult_Response) -> bool;
    fn rscp_mission_interfaces__action__Explore_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Explore_GetResult_Response>, size: usize) -> bool;
    fn rscp_mission_interfaces__action__Explore_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Explore_GetResult_Response>);
    fn rscp_mission_interfaces__action__Explore_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Explore_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Explore_GetResult_Response>) -> bool;
}

// Corresponds to rscp_mission_interfaces__action__Explore_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::Explore_Result,

}



impl Default for Explore_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rscp_mission_interfaces__action__Explore_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to rscp_mission_interfaces__action__Explore_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Explore_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rscp_mission_interfaces__action__Explore_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Explore_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Explore_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "rscp_mission_interfaces/action/Explore_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rscp_mission_interfaces__action__Explore_GetResult_Response() }
  }
}






#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__rscp_mission_interfaces__action__SearchArea_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to rscp_mission_interfaces__action__SearchArea_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct SearchArea_SendGoal;

impl rosidl_runtime_rs::Service for SearchArea_SendGoal {
    type Request = SearchArea_SendGoal_Request;
    type Response = SearchArea_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__rscp_mission_interfaces__action__SearchArea_SendGoal() }
    }
}




#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__rscp_mission_interfaces__action__SearchArea_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to rscp_mission_interfaces__action__SearchArea_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct SearchArea_GetResult;

impl rosidl_runtime_rs::Service for SearchArea_GetResult {
    type Request = SearchArea_GetResult_Request;
    type Response = SearchArea_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__rscp_mission_interfaces__action__SearchArea_GetResult() }
    }
}




#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct NavigateToGPS_SendGoal;

impl rosidl_runtime_rs::Service for NavigateToGPS_SendGoal {
    type Request = NavigateToGPS_SendGoal_Request;
    type Response = NavigateToGPS_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_SendGoal() }
    }
}




#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct NavigateToGPS_GetResult;

impl rosidl_runtime_rs::Service for NavigateToGPS_GetResult {
    type Request = NavigateToGPS_GetResult_Request;
    type Response = NavigateToGPS_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS_GetResult() }
    }
}




#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__rscp_mission_interfaces__action__Explore_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to rscp_mission_interfaces__action__Explore_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct Explore_SendGoal;

impl rosidl_runtime_rs::Service for Explore_SendGoal {
    type Request = Explore_SendGoal_Request;
    type Response = Explore_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__rscp_mission_interfaces__action__Explore_SendGoal() }
    }
}




#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__rscp_mission_interfaces__action__Explore_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to rscp_mission_interfaces__action__Explore_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct Explore_GetResult;

impl rosidl_runtime_rs::Service for Explore_GetResult {
    type Request = Explore_GetResult_Request;
    type Response = Explore_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__rscp_mission_interfaces__action__Explore_GetResult() }
    }
}


