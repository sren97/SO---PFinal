# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: wifi_config.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import constants_pb2 as constants__pb2
import wifi_constants_pb2 as wifi__constants__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x11wifi_config.proto\x1a\x0f\x63onstants.proto\x1a\x14wifi_constants.proto\"\x0e\n\x0c\x43mdGetStatus\"\xe0\x01\n\rRespGetStatus\x12\x17\n\x06status\x18\x01 \x01(\x0e\x32\x07.Status\x12$\n\tsta_state\x18\x02 \x01(\x0e\x32\x11.WifiStationState\x12/\n\x0b\x66\x61il_reason\x18\n \x01(\x0e\x32\x18.WifiConnectFailedReasonH\x00\x12(\n\tconnected\x18\x0b \x01(\x0b\x32\x13.WifiConnectedStateH\x00\x12,\n\x0e\x61ttempt_failed\x18\x0c \x01(\x0b\x32\x12.WifiAttemptFailedH\x00\x42\x07\n\x05state\"P\n\x0c\x43mdSetConfig\x12\x0c\n\x04ssid\x18\x01 \x01(\x0c\x12\x12\n\npassphrase\x18\x02 \x01(\x0c\x12\r\n\x05\x62ssid\x18\x03 \x01(\x0c\x12\x0f\n\x07\x63hannel\x18\x04 \x01(\x05\"(\n\rRespSetConfig\x12\x17\n\x06status\x18\x01 \x01(\x0e\x32\x07.Status\"\x10\n\x0e\x43mdApplyConfig\"*\n\x0fRespApplyConfig\x12\x17\n\x06status\x18\x01 \x01(\x0e\x32\x07.Status\"\xc3\x02\n\x11WiFiConfigPayload\x12\x1f\n\x03msg\x18\x01 \x01(\x0e\x32\x12.WiFiConfigMsgType\x12\'\n\x0e\x63md_get_status\x18\n \x01(\x0b\x32\r.CmdGetStatusH\x00\x12)\n\x0fresp_get_status\x18\x0b \x01(\x0b\x32\x0e.RespGetStatusH\x00\x12\'\n\x0e\x63md_set_config\x18\x0c \x01(\x0b\x32\r.CmdSetConfigH\x00\x12)\n\x0fresp_set_config\x18\r \x01(\x0b\x32\x0e.RespSetConfigH\x00\x12+\n\x10\x63md_apply_config\x18\x0e \x01(\x0b\x32\x0f.CmdApplyConfigH\x00\x12-\n\x11resp_apply_config\x18\x0f \x01(\x0b\x32\x10.RespApplyConfigH\x00\x42\t\n\x07payload*\x9e\x01\n\x11WiFiConfigMsgType\x12\x14\n\x10TypeCmdGetStatus\x10\x00\x12\x15\n\x11TypeRespGetStatus\x10\x01\x12\x14\n\x10TypeCmdSetConfig\x10\x02\x12\x15\n\x11TypeRespSetConfig\x10\x03\x12\x16\n\x12TypeCmdApplyConfig\x10\x04\x12\x17\n\x13TypeRespApplyConfig\x10\x05\x62\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'wifi_config_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _WIFICONFIGMSGTYPE._serialized_start=816
  _WIFICONFIGMSGTYPE._serialized_end=974
  _CMDGETSTATUS._serialized_start=60
  _CMDGETSTATUS._serialized_end=74
  _RESPGETSTATUS._serialized_start=77
  _RESPGETSTATUS._serialized_end=301
  _CMDSETCONFIG._serialized_start=303
  _CMDSETCONFIG._serialized_end=383
  _RESPSETCONFIG._serialized_start=385
  _RESPSETCONFIG._serialized_end=425
  _CMDAPPLYCONFIG._serialized_start=427
  _CMDAPPLYCONFIG._serialized_end=443
  _RESPAPPLYCONFIG._serialized_start=445
  _RESPAPPLYCONFIG._serialized_end=487
  _WIFICONFIGPAYLOAD._serialized_start=490
  _WIFICONFIGPAYLOAD._serialized_end=813
# @@protoc_insertion_point(module_scope)
