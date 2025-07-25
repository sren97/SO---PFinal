/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: wifi_config.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "wifi_config.pb-c.h"
void   cmd_get_status__init
                     (CmdGetStatus         *message)
{
  static const CmdGetStatus init_value = CMD_GET_STATUS__INIT;
  *message = init_value;
}
size_t cmd_get_status__get_packed_size
                     (const CmdGetStatus *message)
{
  assert(message->base.descriptor == &cmd_get_status__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t cmd_get_status__pack
                     (const CmdGetStatus *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &cmd_get_status__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t cmd_get_status__pack_to_buffer
                     (const CmdGetStatus *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &cmd_get_status__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
CmdGetStatus *
       cmd_get_status__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (CmdGetStatus *)
     protobuf_c_message_unpack (&cmd_get_status__descriptor,
                                allocator, len, data);
}
void   cmd_get_status__free_unpacked
                     (CmdGetStatus *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &cmd_get_status__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
void   resp_get_status__init
                     (RespGetStatus         *message)
{
  static const RespGetStatus init_value = RESP_GET_STATUS__INIT;
  *message = init_value;
}
size_t resp_get_status__get_packed_size
                     (const RespGetStatus *message)
{
  assert(message->base.descriptor == &resp_get_status__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t resp_get_status__pack
                     (const RespGetStatus *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &resp_get_status__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t resp_get_status__pack_to_buffer
                     (const RespGetStatus *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &resp_get_status__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
RespGetStatus *
       resp_get_status__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (RespGetStatus *)
     protobuf_c_message_unpack (&resp_get_status__descriptor,
                                allocator, len, data);
}
void   resp_get_status__free_unpacked
                     (RespGetStatus *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &resp_get_status__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
void   cmd_set_config__init
                     (CmdSetConfig         *message)
{
  static const CmdSetConfig init_value = CMD_SET_CONFIG__INIT;
  *message = init_value;
}
size_t cmd_set_config__get_packed_size
                     (const CmdSetConfig *message)
{
  assert(message->base.descriptor == &cmd_set_config__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t cmd_set_config__pack
                     (const CmdSetConfig *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &cmd_set_config__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t cmd_set_config__pack_to_buffer
                     (const CmdSetConfig *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &cmd_set_config__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
CmdSetConfig *
       cmd_set_config__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (CmdSetConfig *)
     protobuf_c_message_unpack (&cmd_set_config__descriptor,
                                allocator, len, data);
}
void   cmd_set_config__free_unpacked
                     (CmdSetConfig *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &cmd_set_config__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
void   resp_set_config__init
                     (RespSetConfig         *message)
{
  static const RespSetConfig init_value = RESP_SET_CONFIG__INIT;
  *message = init_value;
}
size_t resp_set_config__get_packed_size
                     (const RespSetConfig *message)
{
  assert(message->base.descriptor == &resp_set_config__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t resp_set_config__pack
                     (const RespSetConfig *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &resp_set_config__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t resp_set_config__pack_to_buffer
                     (const RespSetConfig *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &resp_set_config__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
RespSetConfig *
       resp_set_config__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (RespSetConfig *)
     protobuf_c_message_unpack (&resp_set_config__descriptor,
                                allocator, len, data);
}
void   resp_set_config__free_unpacked
                     (RespSetConfig *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &resp_set_config__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
void   cmd_apply_config__init
                     (CmdApplyConfig         *message)
{
  static const CmdApplyConfig init_value = CMD_APPLY_CONFIG__INIT;
  *message = init_value;
}
size_t cmd_apply_config__get_packed_size
                     (const CmdApplyConfig *message)
{
  assert(message->base.descriptor == &cmd_apply_config__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t cmd_apply_config__pack
                     (const CmdApplyConfig *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &cmd_apply_config__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t cmd_apply_config__pack_to_buffer
                     (const CmdApplyConfig *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &cmd_apply_config__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
CmdApplyConfig *
       cmd_apply_config__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (CmdApplyConfig *)
     protobuf_c_message_unpack (&cmd_apply_config__descriptor,
                                allocator, len, data);
}
void   cmd_apply_config__free_unpacked
                     (CmdApplyConfig *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &cmd_apply_config__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
void   resp_apply_config__init
                     (RespApplyConfig         *message)
{
  static const RespApplyConfig init_value = RESP_APPLY_CONFIG__INIT;
  *message = init_value;
}
size_t resp_apply_config__get_packed_size
                     (const RespApplyConfig *message)
{
  assert(message->base.descriptor == &resp_apply_config__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t resp_apply_config__pack
                     (const RespApplyConfig *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &resp_apply_config__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t resp_apply_config__pack_to_buffer
                     (const RespApplyConfig *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &resp_apply_config__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
RespApplyConfig *
       resp_apply_config__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (RespApplyConfig *)
     protobuf_c_message_unpack (&resp_apply_config__descriptor,
                                allocator, len, data);
}
void   resp_apply_config__free_unpacked
                     (RespApplyConfig *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &resp_apply_config__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
void   wi_fi_config_payload__init
                     (WiFiConfigPayload         *message)
{
  static const WiFiConfigPayload init_value = WI_FI_CONFIG_PAYLOAD__INIT;
  *message = init_value;
}
size_t wi_fi_config_payload__get_packed_size
                     (const WiFiConfigPayload *message)
{
  assert(message->base.descriptor == &wi_fi_config_payload__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t wi_fi_config_payload__pack
                     (const WiFiConfigPayload *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &wi_fi_config_payload__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t wi_fi_config_payload__pack_to_buffer
                     (const WiFiConfigPayload *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &wi_fi_config_payload__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
WiFiConfigPayload *
       wi_fi_config_payload__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (WiFiConfigPayload *)
     protobuf_c_message_unpack (&wi_fi_config_payload__descriptor,
                                allocator, len, data);
}
void   wi_fi_config_payload__free_unpacked
                     (WiFiConfigPayload *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &wi_fi_config_payload__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
#define cmd_get_status__field_descriptors NULL
#define cmd_get_status__field_indices_by_name NULL
#define cmd_get_status__number_ranges NULL
const ProtobufCMessageDescriptor cmd_get_status__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "CmdGetStatus",
  "CmdGetStatus",
  "CmdGetStatus",
  "",
  sizeof(CmdGetStatus),
  0,
  cmd_get_status__field_descriptors,
  cmd_get_status__field_indices_by_name,
  0,  cmd_get_status__number_ranges,
  (ProtobufCMessageInit) cmd_get_status__init,
  NULL,NULL,NULL    /* reserved[123] */
};
static const ProtobufCFieldDescriptor resp_get_status__field_descriptors[5] =
{
  {
    "status",
    1,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(RespGetStatus, status),
    &status__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sta_state",
    2,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(RespGetStatus, sta_state),
    &wifi_station_state__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "fail_reason",
    10,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_ENUM,
    offsetof(RespGetStatus, state_case),
    offsetof(RespGetStatus, fail_reason),
    &wifi_connect_failed_reason__descriptor,
    NULL,
    0 | PROTOBUF_C_FIELD_FLAG_ONEOF,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "connected",
    11,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(RespGetStatus, state_case),
    offsetof(RespGetStatus, connected),
    &wifi_connected_state__descriptor,
    NULL,
    0 | PROTOBUF_C_FIELD_FLAG_ONEOF,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "attempt_failed",
    12,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(RespGetStatus, state_case),
    offsetof(RespGetStatus, attempt_failed),
    &wifi_attempt_failed__descriptor,
    NULL,
    0 | PROTOBUF_C_FIELD_FLAG_ONEOF,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned resp_get_status__field_indices_by_name[] = {
  4,   /* field[4] = attempt_failed */
  3,   /* field[3] = connected */
  2,   /* field[2] = fail_reason */
  1,   /* field[1] = sta_state */
  0,   /* field[0] = status */
};
static const ProtobufCIntRange resp_get_status__number_ranges[2 + 1] =
{
  { 1, 0 },
  { 10, 2 },
  { 0, 5 }
};
const ProtobufCMessageDescriptor resp_get_status__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "RespGetStatus",
  "RespGetStatus",
  "RespGetStatus",
  "",
  sizeof(RespGetStatus),
  5,
  resp_get_status__field_descriptors,
  resp_get_status__field_indices_by_name,
  2,  resp_get_status__number_ranges,
  (ProtobufCMessageInit) resp_get_status__init,
  NULL,NULL,NULL    /* reserved[123] */
};
static const ProtobufCFieldDescriptor cmd_set_config__field_descriptors[4] =
{
  {
    "ssid",
    1,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_BYTES,
    0,   /* quantifier_offset */
    offsetof(CmdSetConfig, ssid),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "passphrase",
    2,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_BYTES,
    0,   /* quantifier_offset */
    offsetof(CmdSetConfig, passphrase),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "bssid",
    3,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_BYTES,
    0,   /* quantifier_offset */
    offsetof(CmdSetConfig, bssid),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "channel",
    4,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_INT32,
    0,   /* quantifier_offset */
    offsetof(CmdSetConfig, channel),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned cmd_set_config__field_indices_by_name[] = {
  2,   /* field[2] = bssid */
  3,   /* field[3] = channel */
  1,   /* field[1] = passphrase */
  0,   /* field[0] = ssid */
};
static const ProtobufCIntRange cmd_set_config__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 4 }
};
const ProtobufCMessageDescriptor cmd_set_config__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "CmdSetConfig",
  "CmdSetConfig",
  "CmdSetConfig",
  "",
  sizeof(CmdSetConfig),
  4,
  cmd_set_config__field_descriptors,
  cmd_set_config__field_indices_by_name,
  1,  cmd_set_config__number_ranges,
  (ProtobufCMessageInit) cmd_set_config__init,
  NULL,NULL,NULL    /* reserved[123] */
};
static const ProtobufCFieldDescriptor resp_set_config__field_descriptors[1] =
{
  {
    "status",
    1,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(RespSetConfig, status),
    &status__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned resp_set_config__field_indices_by_name[] = {
  0,   /* field[0] = status */
};
static const ProtobufCIntRange resp_set_config__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 1 }
};
const ProtobufCMessageDescriptor resp_set_config__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "RespSetConfig",
  "RespSetConfig",
  "RespSetConfig",
  "",
  sizeof(RespSetConfig),
  1,
  resp_set_config__field_descriptors,
  resp_set_config__field_indices_by_name,
  1,  resp_set_config__number_ranges,
  (ProtobufCMessageInit) resp_set_config__init,
  NULL,NULL,NULL    /* reserved[123] */
};
#define cmd_apply_config__field_descriptors NULL
#define cmd_apply_config__field_indices_by_name NULL
#define cmd_apply_config__number_ranges NULL
const ProtobufCMessageDescriptor cmd_apply_config__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "CmdApplyConfig",
  "CmdApplyConfig",
  "CmdApplyConfig",
  "",
  sizeof(CmdApplyConfig),
  0,
  cmd_apply_config__field_descriptors,
  cmd_apply_config__field_indices_by_name,
  0,  cmd_apply_config__number_ranges,
  (ProtobufCMessageInit) cmd_apply_config__init,
  NULL,NULL,NULL    /* reserved[123] */
};
static const ProtobufCFieldDescriptor resp_apply_config__field_descriptors[1] =
{
  {
    "status",
    1,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(RespApplyConfig, status),
    &status__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned resp_apply_config__field_indices_by_name[] = {
  0,   /* field[0] = status */
};
static const ProtobufCIntRange resp_apply_config__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 1 }
};
const ProtobufCMessageDescriptor resp_apply_config__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "RespApplyConfig",
  "RespApplyConfig",
  "RespApplyConfig",
  "",
  sizeof(RespApplyConfig),
  1,
  resp_apply_config__field_descriptors,
  resp_apply_config__field_indices_by_name,
  1,  resp_apply_config__number_ranges,
  (ProtobufCMessageInit) resp_apply_config__init,
  NULL,NULL,NULL    /* reserved[123] */
};
static const ProtobufCFieldDescriptor wi_fi_config_payload__field_descriptors[7] =
{
  {
    "msg",
    1,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(WiFiConfigPayload, msg),
    &wi_fi_config_msg_type__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "cmd_get_status",
    10,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(WiFiConfigPayload, payload_case),
    offsetof(WiFiConfigPayload, cmd_get_status),
    &cmd_get_status__descriptor,
    NULL,
    0 | PROTOBUF_C_FIELD_FLAG_ONEOF,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "resp_get_status",
    11,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(WiFiConfigPayload, payload_case),
    offsetof(WiFiConfigPayload, resp_get_status),
    &resp_get_status__descriptor,
    NULL,
    0 | PROTOBUF_C_FIELD_FLAG_ONEOF,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "cmd_set_config",
    12,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(WiFiConfigPayload, payload_case),
    offsetof(WiFiConfigPayload, cmd_set_config),
    &cmd_set_config__descriptor,
    NULL,
    0 | PROTOBUF_C_FIELD_FLAG_ONEOF,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "resp_set_config",
    13,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(WiFiConfigPayload, payload_case),
    offsetof(WiFiConfigPayload, resp_set_config),
    &resp_set_config__descriptor,
    NULL,
    0 | PROTOBUF_C_FIELD_FLAG_ONEOF,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "cmd_apply_config",
    14,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(WiFiConfigPayload, payload_case),
    offsetof(WiFiConfigPayload, cmd_apply_config),
    &cmd_apply_config__descriptor,
    NULL,
    0 | PROTOBUF_C_FIELD_FLAG_ONEOF,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "resp_apply_config",
    15,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(WiFiConfigPayload, payload_case),
    offsetof(WiFiConfigPayload, resp_apply_config),
    &resp_apply_config__descriptor,
    NULL,
    0 | PROTOBUF_C_FIELD_FLAG_ONEOF,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned wi_fi_config_payload__field_indices_by_name[] = {
  5,   /* field[5] = cmd_apply_config */
  1,   /* field[1] = cmd_get_status */
  3,   /* field[3] = cmd_set_config */
  0,   /* field[0] = msg */
  6,   /* field[6] = resp_apply_config */
  2,   /* field[2] = resp_get_status */
  4,   /* field[4] = resp_set_config */
};
static const ProtobufCIntRange wi_fi_config_payload__number_ranges[2 + 1] =
{
  { 1, 0 },
  { 10, 1 },
  { 0, 7 }
};
const ProtobufCMessageDescriptor wi_fi_config_payload__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "WiFiConfigPayload",
  "WiFiConfigPayload",
  "WiFiConfigPayload",
  "",
  sizeof(WiFiConfigPayload),
  7,
  wi_fi_config_payload__field_descriptors,
  wi_fi_config_payload__field_indices_by_name,
  2,  wi_fi_config_payload__number_ranges,
  (ProtobufCMessageInit) wi_fi_config_payload__init,
  NULL,NULL,NULL    /* reserved[123] */
};
static const ProtobufCEnumValue wi_fi_config_msg_type__enum_values_by_number[6] =
{
  { "TypeCmdGetStatus", "WI_FI_CONFIG_MSG_TYPE__TypeCmdGetStatus", 0 },
  { "TypeRespGetStatus", "WI_FI_CONFIG_MSG_TYPE__TypeRespGetStatus", 1 },
  { "TypeCmdSetConfig", "WI_FI_CONFIG_MSG_TYPE__TypeCmdSetConfig", 2 },
  { "TypeRespSetConfig", "WI_FI_CONFIG_MSG_TYPE__TypeRespSetConfig", 3 },
  { "TypeCmdApplyConfig", "WI_FI_CONFIG_MSG_TYPE__TypeCmdApplyConfig", 4 },
  { "TypeRespApplyConfig", "WI_FI_CONFIG_MSG_TYPE__TypeRespApplyConfig", 5 },
};
static const ProtobufCIntRange wi_fi_config_msg_type__value_ranges[] = {
{0, 0},{0, 6}
};
static const ProtobufCEnumValueIndex wi_fi_config_msg_type__enum_values_by_name[6] =
{
  { "TypeCmdApplyConfig", 4 },
  { "TypeCmdGetStatus", 0 },
  { "TypeCmdSetConfig", 2 },
  { "TypeRespApplyConfig", 5 },
  { "TypeRespGetStatus", 1 },
  { "TypeRespSetConfig", 3 },
};
const ProtobufCEnumDescriptor wi_fi_config_msg_type__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "WiFiConfigMsgType",
  "WiFiConfigMsgType",
  "WiFiConfigMsgType",
  "",
  6,
  wi_fi_config_msg_type__enum_values_by_number,
  6,
  wi_fi_config_msg_type__enum_values_by_name,
  1,
  wi_fi_config_msg_type__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
