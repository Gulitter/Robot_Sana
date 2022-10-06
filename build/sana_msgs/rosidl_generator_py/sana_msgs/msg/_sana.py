# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sana_msgs:msg/Sana.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Sana(type):
    """Metaclass of message 'Sana'."""

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
            module = import_type_support('sana_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'sana_msgs.msg.Sana')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sana
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sana
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sana
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sana
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sana

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Sana(metaclass=Metaclass_Sana):
    """Message class 'Sana'."""

    __slots__ = [
        '_hotword',
        '_sttcon',
        '_cam',
        '_arrive',
        '_stt',
        '_intent',
        '_tts',
    ]

    _fields_and_field_types = {
        'hotword': 'boolean',
        'sttcon': 'boolean',
        'cam': 'boolean',
        'arrive': 'boolean',
        'stt': 'string',
        'intent': 'string',
        'tts': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.hotword = kwargs.get('hotword', bool())
        self.sttcon = kwargs.get('sttcon', bool())
        self.cam = kwargs.get('cam', bool())
        self.arrive = kwargs.get('arrive', bool())
        self.stt = kwargs.get('stt', str())
        self.intent = kwargs.get('intent', str())
        self.tts = kwargs.get('tts', str())

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
        if self.hotword != other.hotword:
            return False
        if self.sttcon != other.sttcon:
            return False
        if self.cam != other.cam:
            return False
        if self.arrive != other.arrive:
            return False
        if self.stt != other.stt:
            return False
        if self.intent != other.intent:
            return False
        if self.tts != other.tts:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def hotword(self):
        """Message field 'hotword'."""
        return self._hotword

    @hotword.setter
    def hotword(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'hotword' field must be of type 'bool'"
        self._hotword = value

    @property
    def sttcon(self):
        """Message field 'sttcon'."""
        return self._sttcon

    @sttcon.setter
    def sttcon(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'sttcon' field must be of type 'bool'"
        self._sttcon = value

    @property
    def cam(self):
        """Message field 'cam'."""
        return self._cam

    @cam.setter
    def cam(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'cam' field must be of type 'bool'"
        self._cam = value

    @property
    def arrive(self):
        """Message field 'arrive'."""
        return self._arrive

    @arrive.setter
    def arrive(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'arrive' field must be of type 'bool'"
        self._arrive = value

    @property
    def stt(self):
        """Message field 'stt'."""
        return self._stt

    @stt.setter
    def stt(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'stt' field must be of type 'str'"
        self._stt = value

    @property
    def intent(self):
        """Message field 'intent'."""
        return self._intent

    @intent.setter
    def intent(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'intent' field must be of type 'str'"
        self._intent = value

    @property
    def tts(self):
        """Message field 'tts'."""
        return self._tts

    @tts.setter
    def tts(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'tts' field must be of type 'str'"
        self._tts = value
