/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.1 at Sun Nov 18 20:04:23 2018. */

#ifndef PB_STM32MSG_PB_H_INCLUDED
#define PB_STM32MSG_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _EnumVersion {
    EnumVersion_P_VERSION_MAJOR = 1,
    EnumVersion_P_VERSION_MINOR = 0
} EnumVersion;
#define _EnumVersion_MIN EnumVersion_P_VERSION_MAJOR
#define _EnumVersion_MAX EnumVersion_P_VERSION_MINOR
#define _EnumVersion_ARRAYSIZE ((EnumVersion)(EnumVersion_P_VERSION_MINOR+1))

typedef enum _EnumLowLevelIO {
    EnumLowLevelIO_IO_HEADER_EOM_FLAG = 128,
    EnumLowLevelIO_IO_HEADER_SIZE_MSK = 127,
    EnumLowLevelIO_IO_IN_PACKET_SIZE = 32,
    EnumLowLevelIO_IO_OUT_PACKET_SIZE = 32
} EnumLowLevelIO;
#define _EnumLowLevelIO_MIN EnumLowLevelIO_IO_HEADER_EOM_FLAG
#define _EnumLowLevelIO_MAX EnumLowLevelIO_IO_OUT_PACKET_SIZE
#define _EnumLowLevelIO_ARRAYSIZE ((EnumLowLevelIO)(EnumLowLevelIO_IO_OUT_PACKET_SIZE+1))

typedef enum _EnumCapability {
    EnumCapability_CAP_INSPECTOR = 1,
    EnumCapability_CAP_SELF_TEST = 128
} EnumCapability;
#define _EnumCapability_MIN EnumCapability_CAP_INSPECTOR
#define _EnumCapability_MAX EnumCapability_CAP_SELF_TEST
#define _EnumCapability_ARRAYSIZE ((EnumCapability)(EnumCapability_CAP_SELF_TEST+1))

typedef enum _EnumCmd {
    EnumCmd_CMD_SYNC = 0,
    EnumCmd_CMD_SYS_INFO = 1,
    EnumCmd_CMD_NETWORK_INFO = 10,
    EnumCmd_CMD_NETWORK_RUN = 11,
    EnumCmd_CMD_NETWORK_REPORT = 12,
    EnumCmd_CMD_TEST = 100,
    EnumCmd_CMD_TEST_UNSUPPORTED = 200
} EnumCmd;
#define _EnumCmd_MIN EnumCmd_CMD_SYNC
#define _EnumCmd_MAX EnumCmd_CMD_TEST_UNSUPPORTED
#define _EnumCmd_ARRAYSIZE ((EnumCmd)(EnumCmd_CMD_TEST_UNSUPPORTED+1))

typedef enum _EnumState {
    EnumState_S_IDLE = 0,
    EnumState_S_WAITING = 1,
    EnumState_S_PROCESSING = 2,
    EnumState_S_DONE = 3,
    EnumState_S_ERROR = 4
} EnumState;
#define _EnumState_MIN EnumState_S_IDLE
#define _EnumState_MAX EnumState_S_ERROR
#define _EnumState_ARRAYSIZE ((EnumState)(EnumState_S_ERROR+1))

typedef enum _EnumError {
    EnumError_E_NONE = 0,
    EnumError_E_INVALID_SIZE = 1,
    EnumError_E_INVALID_FORMAT = 2,
    EnumError_E_INVALID_STATE = 3,
    EnumError_E_INVALID_PARAM = 4,
    EnumError_E_GENERIC = 5
} EnumError;
#define _EnumError_MIN EnumError_E_NONE
#define _EnumError_MAX EnumError_E_GENERIC
#define _EnumError_ARRAYSIZE ((EnumError)(EnumError_E_GENERIC+1))

typedef enum _EnumFormat {
    EnumFormat_F_NONE = 0,
    EnumFormat_F_FLOAT = 1,
    EnumFormat_F_U8 = 10,
    EnumFormat_F_Q7 = 31,
    EnumFormat_F_Q15 = 32
} EnumFormat;
#define _EnumFormat_MIN EnumFormat_F_NONE
#define _EnumFormat_MAX EnumFormat_F_Q15
#define _EnumFormat_ARRAYSIZE ((EnumFormat)(EnumFormat_F_Q15+1))

typedef enum _EnumRunParam {
    EnumRunParam_P_RUN_MODE_NORMAL = 0,
    EnumRunParam_P_RUN_MODE_INSPECTOR = 1,
    EnumRunParam_P_RUN_MODE_INSPECTOR_WITHOUT_DATA = 2
} EnumRunParam;
#define _EnumRunParam_MIN EnumRunParam_P_RUN_MODE_NORMAL
#define _EnumRunParam_MAX EnumRunParam_P_RUN_MODE_INSPECTOR_WITHOUT_DATA
#define _EnumRunParam_ARRAYSIZE ((EnumRunParam)(EnumRunParam_P_RUN_MODE_INSPECTOR_WITHOUT_DATA+1))

typedef enum _EnumLayerType {
    EnumLayerType_LAYER_TYPE_OUTPUT = 0,
    EnumLayerType_LAYER_TYPE_INTERNAL = 1,
    EnumLayerType_LAYER_TYPE_INTERNAL_LAST = 2
} EnumLayerType;
#define _EnumLayerType_MIN EnumLayerType_LAYER_TYPE_OUTPUT
#define _EnumLayerType_MAX EnumLayerType_LAYER_TYPE_INTERNAL_LAST
#define _EnumLayerType_ARRAYSIZE ((EnumLayerType)(EnumLayerType_LAYER_TYPE_INTERNAL_LAST+1))

/* Struct definitions */
typedef struct _aiBufferFloat32 {
    pb_callback_t datas;
/* @@protoc_insertion_point(struct:aiBufferFloat32) */
} aiBufferFloat32;

typedef struct _aiBufferU16 {
    pb_callback_t datas;
/* @@protoc_insertion_point(struct:aiBufferU16) */
} aiBufferU16;

typedef struct _ackMsg {
    uint32_t param;
/* @@protoc_insertion_point(struct:ackMsg) */
} ackMsg;

typedef struct _aiBufferMsg {
    EnumFormat format;
    uint32_t n_batches;
    uint32_t height;
    uint32_t width;
    uint32_t channels;
    pb_callback_t datas;
/* @@protoc_insertion_point(struct:aiBufferMsg) */
} aiBufferMsg;

typedef struct _aiBufferMsg2 {
    EnumFormat format;
    uint32_t n_batches;
    uint32_t height;
    uint32_t width;
    uint32_t channels;
    pb_size_t which_datas;
    union {
        aiBufferFloat32 fdata;
        aiBufferU16 udata;
    } datas;
/* @@protoc_insertion_point(struct:aiBufferMsg2) */
} aiBufferMsg2;

typedef struct _aiRunReportMsg {
    uint32_t id;
    uint32_t signature;
    uint32_t num_inferences;
    uint32_t n_nodes;
    float elapsed_ms;
/* @@protoc_insertion_point(struct:aiRunReportMsg) */
} aiRunReportMsg;

typedef struct _logMsg {
    uint32_t level;
    char str[128];
/* @@protoc_insertion_point(struct:logMsg) */
} logMsg;

typedef struct _reqMsg {
    uint32_t reqid;
    EnumCmd cmd;
    uint32_t param;
    char name[64];
/* @@protoc_insertion_point(struct:reqMsg) */
} reqMsg;

typedef struct _syncMsg {
    uint32_t version;
    uint32_t capability;
/* @@protoc_insertion_point(struct:syncMsg) */
} syncMsg;

typedef struct _sysinfoMsg {
    uint32_t devid;
    uint32_t sclock;
    uint32_t hclock;
    uint32_t cache;
/* @@protoc_insertion_point(struct:sysinfoMsg) */
} sysinfoMsg;

typedef struct _aiNetworkInfoMsg {
    char model_name[64];
    char model_signature[64];
    char model_datetime[64];
    char compile_datetime[64];
    char runtime_revision[64];
    uint32_t runtime_version;
    char tool_revision[64];
    uint32_t tool_version;
    uint32_t tool_api_version;
    uint32_t api_version;
    uint32_t interface_api_version;
    uint32_t n_macc;
    uint32_t n_inputs;
    uint32_t n_outputs;
    uint32_t n_nodes;
    pb_callback_t inputs;
    pb_callback_t outputs;
    aiBufferMsg activations;
    aiBufferMsg weights;
    uint32_t signature;
/* @@protoc_insertion_point(struct:aiNetworkInfoMsg) */
} aiNetworkInfoMsg;

typedef struct _layerMsg {
    uint32_t type;
    uint32_t id;
    float duration;
    aiBufferMsg buffer;
/* @@protoc_insertion_point(struct:layerMsg) */
} layerMsg;

typedef struct _respMsg {
    uint32_t reqid;
    EnumState state;
    pb_size_t which_payload;
    union {
        syncMsg sync;
        sysinfoMsg sinfo;
        ackMsg ack;
        logMsg log;
        layerMsg layer;
        aiNetworkInfoMsg ninfo;
        aiRunReportMsg report;
    } payload;
/* @@protoc_insertion_point(struct:respMsg) */
} respMsg;

/* Default values for struct fields */
extern const EnumCmd reqMsg_cmd_default;
extern const EnumFormat aiBufferMsg_format_default;
extern const uint32_t aiBufferMsg_n_batches_default;
extern const EnumFormat aiBufferMsg2_format_default;
extern const uint32_t aiBufferMsg2_n_batches_default;

/* Initializer values for message structs */
#define reqMsg_init_default                      {0, EnumCmd_CMD_SYS_INFO, 0, ""}
#define aiRunReportMsg_init_default              {0, 0, 0, 0, 0}
#define aiNetworkInfoMsg_init_default            {"", "", "", "", "", 0, "", 0, 0, 0, 0, 0, 0, 0, 0, {{NULL}, NULL}, {{NULL}, NULL}, aiBufferMsg_init_default, aiBufferMsg_init_default, 0}
#define aiBufferMsg_init_default                 {EnumFormat_F_FLOAT, 1u, 0, 0, 0, {{NULL}, NULL}}
#define aiBufferFloat32_init_default             {{{NULL}, NULL}}
#define aiBufferU16_init_default                 {{{NULL}, NULL}}
#define aiBufferMsg2_init_default                {EnumFormat_F_FLOAT, 1u, 0, 0, 0, 0, {aiBufferFloat32_init_default}}
#define syncMsg_init_default                     {0, 0}
#define sysinfoMsg_init_default                  {0, 0, 0, 0}
#define ackMsg_init_default                      {0}
#define logMsg_init_default                      {0, ""}
#define layerMsg_init_default                    {0, 0, 0, aiBufferMsg_init_default}
#define respMsg_init_default                     {0, _EnumState_MIN, 0, {syncMsg_init_default}}
#define reqMsg_init_zero                         {0, _EnumCmd_MIN, 0, ""}
#define aiRunReportMsg_init_zero                 {0, 0, 0, 0, 0}
#define aiNetworkInfoMsg_init_zero               {"", "", "", "", "", 0, "", 0, 0, 0, 0, 0, 0, 0, 0, {{NULL}, NULL}, {{NULL}, NULL}, aiBufferMsg_init_zero, aiBufferMsg_init_zero, 0}
#define aiBufferMsg_init_zero                    {_EnumFormat_MIN, 0, 0, 0, 0, {{NULL}, NULL}}
#define aiBufferFloat32_init_zero                {{{NULL}, NULL}}
#define aiBufferU16_init_zero                    {{{NULL}, NULL}}
#define aiBufferMsg2_init_zero                   {_EnumFormat_MIN, 0, 0, 0, 0, 0, {aiBufferFloat32_init_zero}}
#define syncMsg_init_zero                        {0, 0}
#define sysinfoMsg_init_zero                     {0, 0, 0, 0}
#define ackMsg_init_zero                         {0}
#define logMsg_init_zero                         {0, ""}
#define layerMsg_init_zero                       {0, 0, 0, aiBufferMsg_init_zero}
#define respMsg_init_zero                        {0, _EnumState_MIN, 0, {syncMsg_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define aiBufferFloat32_datas_tag                1
#define aiBufferU16_datas_tag                    1
#define ackMsg_param_tag                         1
#define aiBufferMsg_format_tag                   1
#define aiBufferMsg_n_batches_tag                2
#define aiBufferMsg_height_tag                   3
#define aiBufferMsg_width_tag                    4
#define aiBufferMsg_channels_tag                 5
#define aiBufferMsg_datas_tag                    6
#define aiBufferMsg2_fdata_tag                   10
#define aiBufferMsg2_udata_tag                   11
#define aiBufferMsg2_format_tag                  1
#define aiBufferMsg2_n_batches_tag               2
#define aiBufferMsg2_height_tag                  3
#define aiBufferMsg2_width_tag                   4
#define aiBufferMsg2_channels_tag                5
#define aiRunReportMsg_id_tag                    1
#define aiRunReportMsg_signature_tag             2
#define aiRunReportMsg_num_inferences_tag        3
#define aiRunReportMsg_n_nodes_tag               4
#define aiRunReportMsg_elapsed_ms_tag            5
#define logMsg_level_tag                         1
#define logMsg_str_tag                           2
#define reqMsg_reqid_tag                         1
#define reqMsg_cmd_tag                           2
#define reqMsg_param_tag                         3
#define reqMsg_name_tag                          4
#define syncMsg_version_tag                      1
#define syncMsg_capability_tag                   4
#define sysinfoMsg_devid_tag                     1
#define sysinfoMsg_sclock_tag                    2
#define sysinfoMsg_hclock_tag                    3
#define sysinfoMsg_cache_tag                     4
#define aiNetworkInfoMsg_model_name_tag          1
#define aiNetworkInfoMsg_model_signature_tag     2
#define aiNetworkInfoMsg_model_datetime_tag      3
#define aiNetworkInfoMsg_compile_datetime_tag    4
#define aiNetworkInfoMsg_runtime_revision_tag    5
#define aiNetworkInfoMsg_runtime_version_tag     6
#define aiNetworkInfoMsg_tool_revision_tag       7
#define aiNetworkInfoMsg_tool_version_tag        8
#define aiNetworkInfoMsg_tool_api_version_tag    9
#define aiNetworkInfoMsg_api_version_tag         10
#define aiNetworkInfoMsg_interface_api_version_tag 11
#define aiNetworkInfoMsg_n_macc_tag              12
#define aiNetworkInfoMsg_n_inputs_tag            13
#define aiNetworkInfoMsg_n_outputs_tag           14
#define aiNetworkInfoMsg_n_nodes_tag             15
#define aiNetworkInfoMsg_inputs_tag              16
#define aiNetworkInfoMsg_outputs_tag             17
#define aiNetworkInfoMsg_activations_tag         18
#define aiNetworkInfoMsg_weights_tag             19
#define aiNetworkInfoMsg_signature_tag           20
#define layerMsg_type_tag                        1
#define layerMsg_id_tag                          2
#define layerMsg_duration_tag                    3
#define layerMsg_buffer_tag                      4
#define respMsg_sync_tag                         10
#define respMsg_sinfo_tag                        11
#define respMsg_ack_tag                          12
#define respMsg_log_tag                          13
#define respMsg_layer_tag                        14
#define respMsg_ninfo_tag                        20
#define respMsg_report_tag                       21
#define respMsg_reqid_tag                        1
#define respMsg_state_tag                        2

/* Struct field encoding specification for nanopb */
extern const pb_field_t reqMsg_fields[5];
extern const pb_field_t aiRunReportMsg_fields[6];
extern const pb_field_t aiNetworkInfoMsg_fields[21];
extern const pb_field_t aiBufferMsg_fields[7];
extern const pb_field_t aiBufferFloat32_fields[2];
extern const pb_field_t aiBufferU16_fields[2];
extern const pb_field_t aiBufferMsg2_fields[8];
extern const pb_field_t syncMsg_fields[3];
extern const pb_field_t sysinfoMsg_fields[5];
extern const pb_field_t ackMsg_fields[2];
extern const pb_field_t logMsg_fields[3];
extern const pb_field_t layerMsg_fields[5];
extern const pb_field_t respMsg_fields[10];

/* Maximum encoded size of messages (where known) */
#define reqMsg_size                              81
#define aiRunReportMsg_size                      29
/* aiNetworkInfoMsg_size depends on runtime parameters */
/* aiBufferMsg_size depends on runtime parameters */
/* aiBufferFloat32_size depends on runtime parameters */
/* aiBufferU16_size depends on runtime parameters */
#define aiBufferMsg2_size                        (26 + ((aiBufferU16_size > aiBufferFloat32_size ? aiBufferU16_size : aiBufferFloat32_size) > 0 ? (aiBufferU16_size > aiBufferFloat32_size ? aiBufferU16_size : aiBufferFloat32_size) : 0))
#define syncMsg_size                             12
#define sysinfoMsg_size                          24
#define ackMsg_size                              6
#define logMsg_size                              137
#define layerMsg_size                            (23 + aiBufferMsg_size)
#define respMsg_size                             (8 + ((aiNetworkInfoMsg_size > layerMsg_size ? aiNetworkInfoMsg_size : layerMsg_size) > 140 ? (aiNetworkInfoMsg_size > layerMsg_size ? aiNetworkInfoMsg_size : layerMsg_size) : 140))

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define STM32MSG_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
