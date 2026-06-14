
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to rscp_mission_interfaces__action__SearchArea_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SearchArea_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for SearchArea_Goal {
  type RmwMsg = super::action::rmw::SearchArea_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        lat: msg.lat,
        lon: msg.lon,
        radius: msg.radius,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      lat: msg.lat,
      lon: msg.lon,
      radius: msg.radius,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      lat: msg.lat,
      lon: msg.lon,
      radius: msg.radius,
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__SearchArea_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SearchArea_Result::default())
  }
}

impl rosidl_runtime_rs::Message for SearchArea_Result {
  type RmwMsg = super::action::rmw::SearchArea_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        found_lat: msg.found_lat,
        found_lon: msg.found_lon,
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      found_lat: msg.found_lat,
      found_lon: msg.found_lon,
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      found_lat: msg.found_lat,
      found_lon: msg.found_lon,
      success: msg.success,
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__SearchArea_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SearchArea_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for SearchArea_Feedback {
  type RmwMsg = super::action::rmw::SearchArea_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_lat: msg.current_lat,
        current_lon: msg.current_lon,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      current_lat: msg.current_lat,
      current_lon: msg.current_lon,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      current_lat: msg.current_lat,
      current_lon: msg.current_lon,
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__SearchArea_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::SearchArea_Feedback,

}



impl Default for SearchArea_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SearchArea_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for SearchArea_FeedbackMessage {
  type RmwMsg = super::action::rmw::SearchArea_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::SearchArea_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::SearchArea_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::SearchArea_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToGPS_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_Goal {
  type RmwMsg = super::action::rmw::NavigateToGPS_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        lat: msg.lat,
        lon: msg.lon,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      lat: msg.lat,
      lon: msg.lon,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      lat: msg.lat,
      lon: msg.lon,
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for NavigateToGPS_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToGPS_Result::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_Result {
  type RmwMsg = super::action::rmw::NavigateToGPS_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_remaining: f64,

}



impl Default for NavigateToGPS_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToGPS_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_Feedback {
  type RmwMsg = super::action::rmw::NavigateToGPS_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        distance_remaining: msg.distance_remaining,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      distance_remaining: msg.distance_remaining,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      distance_remaining: msg.distance_remaining,
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::NavigateToGPS_Feedback,

}



impl Default for NavigateToGPS_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToGPS_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_FeedbackMessage {
  type RmwMsg = super::action::rmw::NavigateToGPS_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::NavigateToGPS_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::NavigateToGPS_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::NavigateToGPS_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__Explore_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for Explore_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Explore_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for Explore_Goal {
  type RmwMsg = super::action::rmw::Explore_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__Explore_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for Explore_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Explore_Result::default())
  }
}

impl rosidl_runtime_rs::Message for Explore_Result {
  type RmwMsg = super::action::rmw::Explore_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__Explore_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub distance: f64,

}



impl Default for Explore_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Explore_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for Explore_Feedback {
  type RmwMsg = super::action::rmw::Explore_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        distance: msg.distance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      distance: msg.distance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      distance: msg.distance,
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__Explore_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::Explore_Feedback,

}



impl Default for Explore_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Explore_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for Explore_FeedbackMessage {
  type RmwMsg = super::action::rmw::Explore_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::Explore_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::Explore_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::Explore_Feedback::from_rmw_message(msg.feedback),
    }
  }
}






// Corresponds to rscp_mission_interfaces__action__SearchArea_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::SearchArea_Goal,

}



impl Default for SearchArea_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SearchArea_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SearchArea_SendGoal_Request {
  type RmwMsg = super::action::rmw::SearchArea_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::SearchArea_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::SearchArea_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::SearchArea_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__SearchArea_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for SearchArea_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SearchArea_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SearchArea_SendGoal_Response {
  type RmwMsg = super::action::rmw::SearchArea_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__SearchArea_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for SearchArea_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SearchArea_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SearchArea_GetResult_Request {
  type RmwMsg = super::action::rmw::SearchArea_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__SearchArea_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchArea_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::SearchArea_Result,

}



impl Default for SearchArea_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SearchArea_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SearchArea_GetResult_Response {
  type RmwMsg = super::action::rmw::SearchArea_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::SearchArea_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::SearchArea_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::SearchArea_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::NavigateToGPS_Goal,

}



impl Default for NavigateToGPS_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToGPS_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_SendGoal_Request {
  type RmwMsg = super::action::rmw::NavigateToGPS_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::NavigateToGPS_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::NavigateToGPS_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::NavigateToGPS_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for NavigateToGPS_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToGPS_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_SendGoal_Response {
  type RmwMsg = super::action::rmw::NavigateToGPS_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for NavigateToGPS_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToGPS_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_GetResult_Request {
  type RmwMsg = super::action::rmw::NavigateToGPS_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToGPS_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::NavigateToGPS_Result,

}



impl Default for NavigateToGPS_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToGPS_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToGPS_GetResult_Response {
  type RmwMsg = super::action::rmw::NavigateToGPS_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::NavigateToGPS_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::NavigateToGPS_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::NavigateToGPS_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__Explore_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::Explore_Goal,

}



impl Default for Explore_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Explore_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Explore_SendGoal_Request {
  type RmwMsg = super::action::rmw::Explore_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::Explore_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::Explore_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::Explore_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__Explore_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for Explore_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Explore_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Explore_SendGoal_Response {
  type RmwMsg = super::action::rmw::Explore_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__Explore_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for Explore_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Explore_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Explore_GetResult_Request {
  type RmwMsg = super::action::rmw::Explore_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to rscp_mission_interfaces__action__Explore_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Explore_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::Explore_Result,

}



impl Default for Explore_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Explore_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Explore_GetResult_Response {
  type RmwMsg = super::action::rmw::Explore_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::Explore_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::Explore_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::Explore_Result::from_rmw_message(msg.result),
    }
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






#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__rscp_mission_interfaces__action__SearchArea() -> *const std::ffi::c_void;
}

// Corresponds to rscp_mission_interfaces__action__SearchArea
#[allow(missing_docs, non_camel_case_types)]
pub struct SearchArea;

impl rosidl_runtime_rs::Action for SearchArea {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = SearchArea_Goal;

  /// The result message defined in the action definition.
  type Result = SearchArea_Result;

  /// The feedback message defined in the action definition.
  type Feedback = SearchArea_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::SearchArea_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::SearchArea_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::SearchArea_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__rscp_mission_interfaces__action__SearchArea() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::SearchArea_Goal,
  ) -> super::action::rmw::SearchArea_SendGoal_Request {
   super::action::rmw::SearchArea_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::SearchArea_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::SearchArea_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::SearchArea_SendGoal_Response {
   super::action::rmw::SearchArea_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::SearchArea_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::SearchArea_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::SearchArea_Feedback,
  ) -> super::action::rmw::SearchArea_FeedbackMessage {
    let mut message = super::action::rmw::SearchArea_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::SearchArea_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::SearchArea_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::SearchArea_GetResult_Request {
   super::action::rmw::SearchArea_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::SearchArea_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::SearchArea_Result,
  ) -> super::action::rmw::SearchArea_GetResult_Response {
   super::action::rmw::SearchArea_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::SearchArea_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::SearchArea_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS() -> *const std::ffi::c_void;
}

// Corresponds to rscp_mission_interfaces__action__NavigateToGPS
#[allow(missing_docs, non_camel_case_types)]
pub struct NavigateToGPS;

impl rosidl_runtime_rs::Action for NavigateToGPS {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = NavigateToGPS_Goal;

  /// The result message defined in the action definition.
  type Result = NavigateToGPS_Result;

  /// The feedback message defined in the action definition.
  type Feedback = NavigateToGPS_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::NavigateToGPS_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::NavigateToGPS_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::NavigateToGPS_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__rscp_mission_interfaces__action__NavigateToGPS() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::NavigateToGPS_Goal,
  ) -> super::action::rmw::NavigateToGPS_SendGoal_Request {
   super::action::rmw::NavigateToGPS_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::NavigateToGPS_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::NavigateToGPS_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::NavigateToGPS_SendGoal_Response {
   super::action::rmw::NavigateToGPS_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::NavigateToGPS_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::NavigateToGPS_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::NavigateToGPS_Feedback,
  ) -> super::action::rmw::NavigateToGPS_FeedbackMessage {
    let mut message = super::action::rmw::NavigateToGPS_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::NavigateToGPS_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::NavigateToGPS_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::NavigateToGPS_GetResult_Request {
   super::action::rmw::NavigateToGPS_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::NavigateToGPS_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::NavigateToGPS_Result,
  ) -> super::action::rmw::NavigateToGPS_GetResult_Response {
   super::action::rmw::NavigateToGPS_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::NavigateToGPS_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::NavigateToGPS_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "rscp_mission_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__rscp_mission_interfaces__action__Explore() -> *const std::ffi::c_void;
}

// Corresponds to rscp_mission_interfaces__action__Explore
#[allow(missing_docs, non_camel_case_types)]
pub struct Explore;

impl rosidl_runtime_rs::Action for Explore {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = Explore_Goal;

  /// The result message defined in the action definition.
  type Result = Explore_Result;

  /// The feedback message defined in the action definition.
  type Feedback = Explore_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::Explore_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::Explore_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::Explore_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__rscp_mission_interfaces__action__Explore() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::Explore_Goal,
  ) -> super::action::rmw::Explore_SendGoal_Request {
   super::action::rmw::Explore_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::Explore_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::Explore_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::Explore_SendGoal_Response {
   super::action::rmw::Explore_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::Explore_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::Explore_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::Explore_Feedback,
  ) -> super::action::rmw::Explore_FeedbackMessage {
    let mut message = super::action::rmw::Explore_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::Explore_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::Explore_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::Explore_GetResult_Request {
   super::action::rmw::Explore_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::Explore_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::Explore_Result,
  ) -> super::action::rmw::Explore_GetResult_Response {
   super::action::rmw::Explore_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::Explore_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::Explore_Result,
  ) {
    (response.status, response.result)
  }
}


