# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rscp_mission_interfaces:action/SearchArea.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SearchArea_Goal(type):
    """Metaclass of message 'SearchArea_Goal'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rscp_mission_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rscp_mission_interfaces.action.SearchArea_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__search_area__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__search_area__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__search_area__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__search_area__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__search_area__goal

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SearchArea_Goal(metaclass=Metaclass_SearchArea_Goal):
    """Message class 'SearchArea_Goal'."""

    __slots__ = [
        '_lat',
        '_lon',
        '_radius',
    ]

    _fields_and_field_types = {
        'lat': 'double',
        'lon': 'double',
        'radius': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.lat = kwargs.get('lat', float())
        self.lon = kwargs.get('lon', float())
        self.radius = kwargs.get('radius', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.lat != other.lat:
            return False
        if self.lon != other.lon:
            return False
        if self.radius != other.radius:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def lat(self):
        """Message field 'lat'."""
        return self._lat

    @lat.setter
    def lat(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lat' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'lat' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._lat = value

    @builtins.property
    def lon(self):
        """Message field 'lon'."""
        return self._lon

    @lon.setter
    def lon(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lon' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'lon' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._lon = value

    @builtins.property
    def radius(self):
        """Message field 'radius'."""
        return self._radius

    @radius.setter
    def radius(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'radius' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'radius' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._radius = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_SearchArea_Result(type):
    """Metaclass of message 'SearchArea_Result'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rscp_mission_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rscp_mission_interfaces.action.SearchArea_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__search_area__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__search_area__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__search_area__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__search_area__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__search_area__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SearchArea_Result(metaclass=Metaclass_SearchArea_Result):
    """Message class 'SearchArea_Result'."""

    __slots__ = [
        '_found_lat',
        '_found_lon',
        '_success',
    ]

    _fields_and_field_types = {
        'found_lat': 'double',
        'found_lon': 'double',
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.found_lat = kwargs.get('found_lat', float())
        self.found_lon = kwargs.get('found_lon', float())
        self.success = kwargs.get('success', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.found_lat != other.found_lat:
            return False
        if self.found_lon != other.found_lon:
            return False
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def found_lat(self):
        """Message field 'found_lat'."""
        return self._found_lat

    @found_lat.setter
    def found_lat(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'found_lat' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'found_lat' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._found_lat = value

    @builtins.property
    def found_lon(self):
        """Message field 'found_lon'."""
        return self._found_lon

    @found_lon.setter
    def found_lon(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'found_lon' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'found_lon' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._found_lon = value

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_SearchArea_Feedback(type):
    """Metaclass of message 'SearchArea_Feedback'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rscp_mission_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rscp_mission_interfaces.action.SearchArea_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__search_area__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__search_area__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__search_area__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__search_area__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__search_area__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SearchArea_Feedback(metaclass=Metaclass_SearchArea_Feedback):
    """Message class 'SearchArea_Feedback'."""

    __slots__ = [
        '_current_lat',
        '_current_lon',
    ]

    _fields_and_field_types = {
        'current_lat': 'double',
        'current_lon': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.current_lat = kwargs.get('current_lat', float())
        self.current_lon = kwargs.get('current_lon', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.current_lat != other.current_lat:
            return False
        if self.current_lon != other.current_lon:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def current_lat(self):
        """Message field 'current_lat'."""
        return self._current_lat

    @current_lat.setter
    def current_lat(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_lat' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'current_lat' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._current_lat = value

    @builtins.property
    def current_lon(self):
        """Message field 'current_lon'."""
        return self._current_lon

    @current_lon.setter
    def current_lon(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_lon' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'current_lon' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._current_lon = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SearchArea_SendGoal_Request(type):
    """Metaclass of message 'SearchArea_SendGoal_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rscp_mission_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rscp_mission_interfaces.action.SearchArea_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__search_area__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__search_area__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__search_area__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__search_area__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__search_area__send_goal__request

            from rscp_mission_interfaces.action import SearchArea
            if SearchArea.Goal.__class__._TYPE_SUPPORT is None:
                SearchArea.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SearchArea_SendGoal_Request(metaclass=Metaclass_SearchArea_SendGoal_Request):
    """Message class 'SearchArea_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'rscp_mission_interfaces/SearchArea_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['rscp_mission_interfaces', 'action'], 'SearchArea_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from rscp_mission_interfaces.action._search_area import SearchArea_Goal
        self.goal = kwargs.get('goal', SearchArea_Goal())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from rscp_mission_interfaces.action._search_area import SearchArea_Goal
            assert \
                isinstance(value, SearchArea_Goal), \
                "The 'goal' field must be a sub message of type 'SearchArea_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SearchArea_SendGoal_Response(type):
    """Metaclass of message 'SearchArea_SendGoal_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rscp_mission_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rscp_mission_interfaces.action.SearchArea_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__search_area__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__search_area__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__search_area__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__search_area__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__search_area__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SearchArea_SendGoal_Response(metaclass=Metaclass_SearchArea_SendGoal_Response):
    """Message class 'SearchArea_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_SearchArea_SendGoal(type):
    """Metaclass of service 'SearchArea_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rscp_mission_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rscp_mission_interfaces.action.SearchArea_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__search_area__send_goal

            from rscp_mission_interfaces.action import _search_area
            if _search_area.Metaclass_SearchArea_SendGoal_Request._TYPE_SUPPORT is None:
                _search_area.Metaclass_SearchArea_SendGoal_Request.__import_type_support__()
            if _search_area.Metaclass_SearchArea_SendGoal_Response._TYPE_SUPPORT is None:
                _search_area.Metaclass_SearchArea_SendGoal_Response.__import_type_support__()


class SearchArea_SendGoal(metaclass=Metaclass_SearchArea_SendGoal):
    from rscp_mission_interfaces.action._search_area import SearchArea_SendGoal_Request as Request
    from rscp_mission_interfaces.action._search_area import SearchArea_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SearchArea_GetResult_Request(type):
    """Metaclass of message 'SearchArea_GetResult_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rscp_mission_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rscp_mission_interfaces.action.SearchArea_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__search_area__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__search_area__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__search_area__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__search_area__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__search_area__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SearchArea_GetResult_Request(metaclass=Metaclass_SearchArea_GetResult_Request):
    """Message class 'SearchArea_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SearchArea_GetResult_Response(type):
    """Metaclass of message 'SearchArea_GetResult_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rscp_mission_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rscp_mission_interfaces.action.SearchArea_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__search_area__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__search_area__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__search_area__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__search_area__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__search_area__get_result__response

            from rscp_mission_interfaces.action import SearchArea
            if SearchArea.Result.__class__._TYPE_SUPPORT is None:
                SearchArea.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SearchArea_GetResult_Response(metaclass=Metaclass_SearchArea_GetResult_Response):
    """Message class 'SearchArea_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'rscp_mission_interfaces/SearchArea_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['rscp_mission_interfaces', 'action'], 'SearchArea_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from rscp_mission_interfaces.action._search_area import SearchArea_Result
        self.result = kwargs.get('result', SearchArea_Result())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from rscp_mission_interfaces.action._search_area import SearchArea_Result
            assert \
                isinstance(value, SearchArea_Result), \
                "The 'result' field must be a sub message of type 'SearchArea_Result'"
        self._result = value


class Metaclass_SearchArea_GetResult(type):
    """Metaclass of service 'SearchArea_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rscp_mission_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rscp_mission_interfaces.action.SearchArea_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__search_area__get_result

            from rscp_mission_interfaces.action import _search_area
            if _search_area.Metaclass_SearchArea_GetResult_Request._TYPE_SUPPORT is None:
                _search_area.Metaclass_SearchArea_GetResult_Request.__import_type_support__()
            if _search_area.Metaclass_SearchArea_GetResult_Response._TYPE_SUPPORT is None:
                _search_area.Metaclass_SearchArea_GetResult_Response.__import_type_support__()


class SearchArea_GetResult(metaclass=Metaclass_SearchArea_GetResult):
    from rscp_mission_interfaces.action._search_area import SearchArea_GetResult_Request as Request
    from rscp_mission_interfaces.action._search_area import SearchArea_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SearchArea_FeedbackMessage(type):
    """Metaclass of message 'SearchArea_FeedbackMessage'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rscp_mission_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rscp_mission_interfaces.action.SearchArea_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__search_area__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__search_area__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__search_area__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__search_area__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__search_area__feedback_message

            from rscp_mission_interfaces.action import SearchArea
            if SearchArea.Feedback.__class__._TYPE_SUPPORT is None:
                SearchArea.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SearchArea_FeedbackMessage(metaclass=Metaclass_SearchArea_FeedbackMessage):
    """Message class 'SearchArea_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'rscp_mission_interfaces/SearchArea_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['rscp_mission_interfaces', 'action'], 'SearchArea_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from rscp_mission_interfaces.action._search_area import SearchArea_Feedback
        self.feedback = kwargs.get('feedback', SearchArea_Feedback())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from rscp_mission_interfaces.action._search_area import SearchArea_Feedback
            assert \
                isinstance(value, SearchArea_Feedback), \
                "The 'feedback' field must be a sub message of type 'SearchArea_Feedback'"
        self._feedback = value


class Metaclass_SearchArea(type):
    """Metaclass of action 'SearchArea'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rscp_mission_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rscp_mission_interfaces.action.SearchArea')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__search_area

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from rscp_mission_interfaces.action import _search_area
            if _search_area.Metaclass_SearchArea_SendGoal._TYPE_SUPPORT is None:
                _search_area.Metaclass_SearchArea_SendGoal.__import_type_support__()
            if _search_area.Metaclass_SearchArea_GetResult._TYPE_SUPPORT is None:
                _search_area.Metaclass_SearchArea_GetResult.__import_type_support__()
            if _search_area.Metaclass_SearchArea_FeedbackMessage._TYPE_SUPPORT is None:
                _search_area.Metaclass_SearchArea_FeedbackMessage.__import_type_support__()


class SearchArea(metaclass=Metaclass_SearchArea):

    # The goal message defined in the action definition.
    from rscp_mission_interfaces.action._search_area import SearchArea_Goal as Goal
    # The result message defined in the action definition.
    from rscp_mission_interfaces.action._search_area import SearchArea_Result as Result
    # The feedback message defined in the action definition.
    from rscp_mission_interfaces.action._search_area import SearchArea_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from rscp_mission_interfaces.action._search_area import SearchArea_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from rscp_mission_interfaces.action._search_area import SearchArea_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from rscp_mission_interfaces.action._search_area import SearchArea_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
