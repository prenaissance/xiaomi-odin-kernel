/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ODIN_QSEECOM_H
#define _UAPI_LINUX_ODIN_QSEECOM_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ODIN_QSEECOM_MAX_APP_NAME	64

struct odin_qseecom_lookup_req {
	char app_name[ODIN_QSEECOM_MAX_APP_NAME];
	__u32 app_id;
	__u32 qsee_result;
};

struct odin_qseecom_load_req {
	__u64 img_uaddr;
	__u32 mdt_len;
	__u32 img_len;
	__u32 app_id;
	__u32 qsee_result;
	__u32 blocked_listener_id;
	__u32 reserved;
};

struct odin_qseecom_unload_req {
	__u32 app_id;
	__u32 qsee_result;
};

struct odin_qseecom_send_cmd_req {
	__u32 app_id;
	__u32 req_len;
	__u32 rsp_len;
	__u32 qsee_result;
	__u64 req_uaddr;
	__u64 rsp_uaddr;
	__u64 payload_uaddr;
	__u32 payload_len;
	__u32 payload_patch_offset;
	__u32 blocked_listener_id;
	__u32 reserved;
};

struct odin_qseecom_listener_req {
	__u32 listener_id;
	__u32 sb_len;
	__u32 status;
	__u32 qsee_result;
};

#define ODIN_QSEECOM_IOC_MAGIC		0x97

#define ODIN_QSEECOM_IOC_LOOKUP \
	_IOWR(ODIN_QSEECOM_IOC_MAGIC, 1, struct odin_qseecom_lookup_req)
#define ODIN_QSEECOM_IOC_LOAD \
	_IOWR(ODIN_QSEECOM_IOC_MAGIC, 2, struct odin_qseecom_load_req)
#define ODIN_QSEECOM_IOC_UNLOAD \
	_IOWR(ODIN_QSEECOM_IOC_MAGIC, 3, struct odin_qseecom_unload_req)
#define ODIN_QSEECOM_IOC_SEND_CMD \
	_IOWR(ODIN_QSEECOM_IOC_MAGIC, 4, struct odin_qseecom_send_cmd_req)
#define ODIN_QSEECOM_IOC_REG_LISTENER \
	_IOWR(ODIN_QSEECOM_IOC_MAGIC, 5, struct odin_qseecom_listener_req)
#define ODIN_QSEECOM_IOC_UNREG_LISTENER \
	_IOWR(ODIN_QSEECOM_IOC_MAGIC, 6, struct odin_qseecom_listener_req)
#define ODIN_QSEECOM_IOC_WAIT_LISTENER \
	_IOWR(ODIN_QSEECOM_IOC_MAGIC, 7, struct odin_qseecom_listener_req)
#define ODIN_QSEECOM_IOC_RESP_LISTENER \
	_IOWR(ODIN_QSEECOM_IOC_MAGIC, 8, struct odin_qseecom_listener_req)

#endif /* _UAPI_LINUX_ODIN_QSEECOM_H */
