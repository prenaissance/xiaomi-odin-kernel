// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal Qualcomm QSEECOM experimental bridge for SM8350 (Xiaomi MIX 4)
 *
 * Uses mainline qcom_scm_qseecom_raw_call() and qcom_tzmem_shm_bridge_create()
 * with coherent <4GB CMA DMA allocations to load/unload QSEECom TAs, send
 * command/response payloads (including modified-cmd payload physical address
 * patching), and serve re-entrant QSEE listener callbacks.
 */

#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <uapi/linux/odin_qseecom.h>

#define ODIN_TZ_OWNER_TZ_APPS		48
#define ODIN_TZ_OWNER_QSEE_OS		50

#define ODIN_TZ_SVC_APP_ID_PLACEHOLDER	0
#define ODIN_TZ_SVC_APP_MGR		1
#define ODIN_TZ_SVC_LISTENER		2

#define ODIN_TZ_CMD_APP_START		1
#define ODIN_TZ_CMD_APP_SHUTDOWN	2
#define ODIN_TZ_CMD_APP_LOOKUP		3
#define ODIN_TZ_CMD_APP_SEND		1
#define ODIN_TZ_CMD_APP_SEND_WHITELIST	6

#define ODIN_TZ_CMD_REG_LISTENER	1
#define ODIN_TZ_CMD_DEREG_LISTENER	2
#define ODIN_TZ_CMD_LISTENER_RESP	3

#define ODIN_QSEE_RES_SUCCESS		0
#define ODIN_QSEE_RES_INCOMPLETE	1
#define ODIN_QSEE_RES_BLOCKED_LISTENER	2
#define ODIN_QSEE_RES_FAILURE		0xFFFFFFFFULL

#define ODIN_QSEE_TYPE_APP_ID		0xEE01ULL
#define ODIN_QSEE_TYPE_LISTENER_ID	0xEE02ULL

#define ODIN_SCM_VAL			0
#define ODIN_SCM_RO			1
#define ODIN_SCM_RW			2

#define ODIN_SCM_ARGS1(a)		(1 | ((a) << 4))
#define ODIN_SCM_ARGS2(a, b)		(2 | ((a) << 4) | ((b) << 6))
#define ODIN_SCM_ARGS3(a, b, c)		(3 | ((a) << 4) | ((b) << 6) | \
					 ((c) << 8))
#define ODIN_SCM_ARGS5(a, b, c, d, e)	(5 | ((a) << 4) | ((b) << 6) | \
					 ((c) << 8) | ((d) << 10) | ((e) << 12))
#define ODIN_SCM_ARGS7(a, b, c, d, e, f, g) \
					(7 | ((a) << 4) | ((b) << 6) | \
					 ((c) << 8) | ((d) << 10) | \
					 ((e) << 12) | ((f) << 14) | \
					 ((g) << 16))

struct odin_sglist_info {
	u32 index_and_flags;
	u32 size_or_count;
};

#define ODIN_SGLIST_MAX_ENTRIES		4
#define ODIN_SGLIST_TABLE_SIZE		(sizeof(struct odin_sglist_info) * \
					 ODIN_SGLIST_MAX_ENTRIES)
#define ODIN_SGLIST_CONTIG_32(off)	(BIT(31) | ((off) & 0x3fffffffU))

#define ODIN_MAX_IMG_SIZE		(24 * 1024 * 1024)
#define ODIN_MAX_CMD_SIZE		(4 * 1024 * 1024)
#define ODIN_MAX_LISTENER_SB		(1 * 1024 * 1024)

struct odin_shm_buf {
	void *vaddr;
	dma_addr_t paddr;
	size_t size;
	u64 bridge_handle;
};

struct odin_listener {
	struct list_head list;
	struct kref refcount;
	struct odin_qseecom_ctx *ctx;
	u32 id;
	struct odin_shm_buf sb;
	wait_queue_head_t req_wq;
	struct completion resp_done;
	bool req_pending;
	bool exiting;
	u32 resp_status;
};

struct odin_qseecom_ctx {
	struct device *dev;
	struct platform_device *pdev;
	struct miscdevice misc;
	struct mutex app_lock;		/* serializes QSEECom app SMC loops */
	struct mutex listener_lock;	/* protects listeners list */
	struct list_head listeners;
};

struct odin_file_priv {
	struct odin_qseecom_ctx *ctx;
	struct odin_listener *listener;
};

static struct odin_qseecom_ctx *g_qseecom;

static int odin_shm_alloc(struct device *dev, size_t size,
			  struct odin_shm_buf *buf)
{
	int ret;

	size = PAGE_ALIGN(size);
	buf->vaddr = dma_alloc_coherent(dev, size, &buf->paddr, GFP_KERNEL);
	if (!buf->vaddr)
		return -ENOMEM;

	buf->size = size;
	buf->bridge_handle = 0;

	ret = qcom_tzmem_shm_bridge_create(buf->paddr, buf->size,
					   &buf->bridge_handle);
	if (ret) {
		dev_err(dev, "SHM bridge create failed (pa=%pad sz=%zu): %d\n",
			&buf->paddr, buf->size, ret);
		dma_free_coherent(dev, buf->size, buf->vaddr, buf->paddr);
		buf->vaddr = NULL;
		return ret;
	}

	return 0;
}

static void odin_shm_free(struct device *dev, struct odin_shm_buf *buf)
{
	if (!buf->vaddr)
		return;

	if (buf->bridge_handle)
		qcom_tzmem_shm_bridge_delete(buf->bridge_handle);
	dma_free_coherent(dev, buf->size, buf->vaddr, buf->paddr);
	memset(buf, 0, sizeof(*buf));
}

static struct odin_listener *odin_find_listener(struct odin_qseecom_ctx *ctx,
						u32 id)
{
	struct odin_listener *lsnr;

	list_for_each_entry(lsnr, &ctx->listeners, list) {
		if (lsnr->id == id)
			return lsnr;
	}

	return NULL;
}

static void odin_listener_release(struct kref *kref)
{
	struct odin_listener *lsnr =
		container_of(kref, struct odin_listener, refcount);
	struct odin_qseecom_ctx *ctx = lsnr->ctx;
	u64 args[10] = { 0 };
	u64 result = 0, resp_type = 0, data = 0;

	args[0] = lsnr->id;
	qcom_scm_qseecom_raw_call(ODIN_TZ_OWNER_QSEE_OS,
				  ODIN_TZ_SVC_LISTENER,
				  ODIN_TZ_CMD_DEREG_LISTENER,
				  ODIN_SCM_ARGS1(ODIN_SCM_VAL),
				  args, &result, &resp_type, &data);

	odin_shm_free(ctx->dev, &lsnr->sb);
	kfree(lsnr);
}

static struct odin_listener *odin_get_priv_listener(struct odin_file_priv *priv)
{
	struct odin_qseecom_ctx *ctx = priv->ctx;
	struct odin_listener *lsnr;

	mutex_lock(&ctx->listener_lock);
	lsnr = priv->listener;
	if (lsnr)
		kref_get(&lsnr->refcount);
	mutex_unlock(&ctx->listener_lock);

	return lsnr;
}

static int odin_detach_priv_listener(struct odin_file_priv *priv)
{
	struct odin_qseecom_ctx *ctx = priv->ctx;
	struct odin_listener *lsnr;

	mutex_lock(&ctx->listener_lock);
	lsnr = priv->listener;
	if (!lsnr) {
		mutex_unlock(&ctx->listener_lock);
		return -EINVAL;
	}
	priv->listener = NULL;
	lsnr->exiting = true;
	list_del_init(&lsnr->list);
	complete_all(&lsnr->resp_done);
	wake_up_all(&lsnr->req_wq);
	mutex_unlock(&ctx->listener_lock);

	kref_put(&lsnr->refcount, odin_listener_release);
	return 0;
}

static int odin_run_scm_with_listeners(struct odin_qseecom_ctx *ctx,
				       u32 owner, u32 svc, u32 cmd,
				       u32 arginfo, const u64 args[10],
				       u64 *result, u64 *resp_type,
				       u64 *data, u32 *blocked_lsnr_id)
{
	int ret;

	*blocked_lsnr_id = 0;

	ret = qcom_scm_qseecom_raw_call(owner, svc, cmd, arginfo, args,
					result, resp_type, data);
	if (ret)
		return ret;

	while ((*result == ODIN_QSEE_RES_INCOMPLETE ||
		*result == ODIN_QSEE_RES_BLOCKED_LISTENER) &&
	       *resp_type == ODIN_QSEE_TYPE_LISTENER_ID) {
		u32 lid = (u32)*data;
		u64 resp_args[10] = { 0 };
		struct odin_listener *lsnr;
		u32 resp_status = (u32)-1;
		long wret;

		mutex_lock(&ctx->listener_lock);
		lsnr = odin_find_listener(ctx, lid);
		if (lsnr && !lsnr->exiting) {
			kref_get(&lsnr->refcount);
			reinit_completion(&lsnr->resp_done);
			lsnr->req_pending = true;
			wake_up_interruptible(&lsnr->req_wq);
			mutex_unlock(&ctx->listener_lock);

			wret = wait_for_completion_interruptible_timeout(&lsnr->resp_done,
									 msecs_to_jiffies(10000));
			if (wret > 0) {
				resp_status = lsnr->resp_status;
			} else {
				dev_err(ctx->dev,
					"Listener 0x%x timed out/interrupted (%ld)\n",
					lid, wret);
				*blocked_lsnr_id = lid;
			}
			kref_put(&lsnr->refcount, odin_listener_release);
		} else {
			mutex_unlock(&ctx->listener_lock);
			dev_warn(ctx->dev,
				 "QSEE requested unregistered listener 0x%x\n",
				 lid);
			*blocked_lsnr_id = lid;
		}

		resp_args[0] = lid;
		resp_args[1] = resp_status;
		ret = qcom_scm_qseecom_raw_call(ODIN_TZ_OWNER_QSEE_OS,
						ODIN_TZ_SVC_LISTENER,
						ODIN_TZ_CMD_LISTENER_RESP,
						ODIN_SCM_ARGS2(ODIN_SCM_VAL,
							       ODIN_SCM_VAL),
						resp_args, result, resp_type,
						data);
		if (ret)
			return ret;
	}

	return 0;
}

static int odin_ioc_lookup(struct odin_qseecom_ctx *ctx, void __user *argp)
{
	struct odin_qseecom_lookup_req req;
	struct odin_shm_buf buf = {};
	u64 args[10] = { 0 };
	u64 result = 0, resp_type = 0, data = 0;
	size_t name_len;
	int ret;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	req.app_name[ODIN_QSEECOM_MAX_APP_NAME - 1] = '\0';
	name_len = strlen(req.app_name);
	if (!name_len)
		return -EINVAL;

	ret = odin_shm_alloc(ctx->dev, PAGE_SIZE, &buf);
	if (ret)
		return ret;

	memcpy(buf.vaddr, req.app_name, name_len);
	args[0] = buf.paddr;
	args[1] = name_len;

	mutex_lock(&ctx->app_lock);
	ret = qcom_scm_qseecom_raw_call(ODIN_TZ_OWNER_QSEE_OS,
					ODIN_TZ_SVC_APP_MGR,
					ODIN_TZ_CMD_APP_LOOKUP,
					ODIN_SCM_ARGS2(ODIN_SCM_RW,
						       ODIN_SCM_VAL),
					args, &result, &resp_type, &data);
	mutex_unlock(&ctx->app_lock);

	odin_shm_free(ctx->dev, &buf);
	if (ret)
		return ret;

	req.qsee_result = (u32)result;
	req.app_id = (result == ODIN_QSEE_RES_SUCCESS &&
		      resp_type == ODIN_QSEE_TYPE_APP_ID) ? (u32)data : 0;

	if (copy_to_user(argp, &req, sizeof(req)))
		return -EFAULT;

	return 0;
}

static int odin_ioc_load(struct odin_qseecom_ctx *ctx, void __user *argp)
{
	struct odin_qseecom_load_req req;
	struct odin_shm_buf buf = {};
	u64 args[10] = { 0 };
	u64 result = 0, resp_type = 0, data = 0;
	u32 blocked_lid = 0;
	int ret;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	if (!req.mdt_len || !req.img_len || req.img_len > ODIN_MAX_IMG_SIZE ||
	    req.mdt_len > ODIN_MAX_IMG_SIZE)
		return -EINVAL;

	/*
	 * Reserve mdt_len + img_len in the SHM bridge in case QSEE validates
	 * [pa, pa + mdt_len + img_len) against the bridge bounds.
	 */
	ret = odin_shm_alloc(ctx->dev, (size_t)req.img_len + req.mdt_len, &buf);
	if (ret)
		return ret;

	if (copy_from_user(buf.vaddr, u64_to_user_ptr(req.img_uaddr),
			   req.img_len)) {
		odin_shm_free(ctx->dev, &buf);
		return -EFAULT;
	}

	args[0] = req.mdt_len;
	args[1] = req.img_len;
	args[2] = buf.paddr;

	dev_info(ctx->dev, "APP_START mdt=%u img=%u shm_sz=%zu pa=%pad\n",
		 req.mdt_len, req.img_len, buf.size, &buf.paddr);

	mutex_lock(&ctx->app_lock);
	ret = odin_run_scm_with_listeners(ctx, ODIN_TZ_OWNER_QSEE_OS,
					  ODIN_TZ_SVC_APP_MGR,
					  ODIN_TZ_CMD_APP_START,
					  ODIN_SCM_ARGS3(ODIN_SCM_VAL,
							 ODIN_SCM_VAL,
							 ODIN_SCM_VAL),
					  args, &result, &resp_type, &data,
					  &blocked_lid);
	mutex_unlock(&ctx->app_lock);

	odin_shm_free(ctx->dev, &buf);

	dev_info(ctx->dev,
		 "APP_START done: ret=%d result=0x%llx resp_type=0x%llx data=0x%llx blocked_lsnr=0x%x\n",
		 ret, result, resp_type, data, blocked_lid);

	if (ret)
		return ret;

	req.qsee_result = (u32)result;
	req.blocked_listener_id = blocked_lid;
	req.app_id = (result == ODIN_QSEE_RES_SUCCESS &&
		      resp_type == ODIN_QSEE_TYPE_APP_ID) ? (u32)data : 0;

	if (copy_to_user(argp, &req, sizeof(req)))
		return -EFAULT;

	return 0;
}

static int odin_ioc_unload(struct odin_qseecom_ctx *ctx, void __user *argp)
{
	struct odin_qseecom_unload_req req;
	u64 args[10] = { 0 };
	u64 result = 0, resp_type = 0, data = 0;
	u32 blocked_lid = 0;
	int ret;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	args[0] = req.app_id;

	mutex_lock(&ctx->app_lock);
	ret = odin_run_scm_with_listeners(ctx, ODIN_TZ_OWNER_QSEE_OS,
					  ODIN_TZ_SVC_APP_MGR,
					  ODIN_TZ_CMD_APP_SHUTDOWN,
					  ODIN_SCM_ARGS1(ODIN_SCM_VAL),
					  args, &result, &resp_type, &data,
					  &blocked_lid);
	mutex_unlock(&ctx->app_lock);

	if (ret)
		return ret;

	req.qsee_result = (u32)result;
	if (copy_to_user(argp, &req, sizeof(req)))
		return -EFAULT;

	return 0;
}

static int odin_ioc_send_cmd(struct odin_qseecom_ctx *ctx, void __user *argp)
{
	struct odin_qseecom_send_cmd_req req;
	struct odin_shm_buf buf = {};
	struct odin_shm_buf payload = {};
	struct odin_shm_buf carveout = {};
	struct odin_shm_buf sglist = {};
	u64 args[10] = { 0 };
	u64 result = 0, resp_type = 0, data = 0;
	u32 blocked_lid = 0;
	size_t req_aligned, rsp_off, total_size;
	bool in_place;
	u32 scm_cmd, arginfo;
	int ret;

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	if (!req.app_id || !req.req_len || !req.rsp_len ||
	    req.req_len > ODIN_MAX_CMD_SIZE ||
	    req.rsp_len > ODIN_MAX_CMD_SIZE ||
	    req.payload_len > ODIN_MAX_CMD_SIZE)
		return -EINVAL;

	if (req.payload_len &&
	    (req.payload_patch_offset > req.req_len - sizeof(u32)))
		return -EINVAL;

	in_place = (req.req_uaddr == req.rsp_uaddr);
	req_aligned = ALIGN(req.req_len, 64);
	rsp_off = in_place ? 0 : req_aligned;
	total_size = in_place ? max_t(size_t, req_aligned,
				      ALIGN(req.rsp_len, 64))
			      : (req_aligned + ALIGN(req.rsp_len, 64));

	ret = odin_shm_alloc(ctx->dev, total_size, &buf);
	if (ret)
		return ret;

	if (copy_from_user(buf.vaddr, u64_to_user_ptr(req.req_uaddr),
			   req.req_len)) {
		ret = -EFAULT;
		goto out_free;
	}

	args[0] = req.app_id;
	args[1] = buf.paddr;
	args[2] = req.req_len;
	args[3] = buf.paddr + rsp_off;
	args[4] = req.rsp_len;

	if (req.payload_len) {
		struct odin_sglist_info *tbl;

		ret = odin_shm_alloc(ctx->dev, req.payload_len, &payload);
		if (ret)
			goto out_free;

		if (copy_from_user(payload.vaddr,
				   u64_to_user_ptr(req.payload_uaddr),
				   req.payload_len)) {
			ret = -EFAULT;
			goto out_free;
		}

		*(u32 *)((u8 *)buf.vaddr + req.payload_patch_offset) =
			(u32)payload.paddr;

		ret = odin_shm_alloc(ctx->dev, PAGE_SIZE, &sglist);
		if (ret)
			goto out_free;

		memset(sglist.vaddr, 0, ODIN_SGLIST_TABLE_SIZE);
		tbl = sglist.vaddr;
		tbl[0].index_and_flags =
			ODIN_SGLIST_CONTIG_32(req.payload_patch_offset);
		tbl[0].size_or_count = payload.size;

		/* Optional 4 MiB carveout buffer patched at env + 32 (0x20) */
		if (req.reserved && req.req_len >= 40) {
			ret = odin_shm_alloc(ctx->dev, SZ_4M, &carveout);
			if (ret)
				goto out_free;
			memset(carveout.vaddr, 0, carveout.size);
			*(u32 *)((u8 *)buf.vaddr + 32) = (u32)carveout.paddr;
			*(u32 *)((u8 *)buf.vaddr + 36) = (u32)carveout.size;
			tbl[1].index_and_flags = ODIN_SGLIST_CONTIG_32(32);
			tbl[1].size_or_count = carveout.size;
		}

		args[5] = sglist.paddr;
		args[6] = ODIN_SGLIST_TABLE_SIZE;
		scm_cmd = ODIN_TZ_CMD_APP_SEND_WHITELIST;
		arginfo = ODIN_SCM_ARGS7(ODIN_SCM_VAL, ODIN_SCM_RW,
					 ODIN_SCM_VAL, ODIN_SCM_RW,
					 ODIN_SCM_VAL, ODIN_SCM_RW,
					 ODIN_SCM_VAL);
	} else {
		scm_cmd = ODIN_TZ_CMD_APP_SEND;
		arginfo = ODIN_SCM_ARGS5(ODIN_SCM_VAL, ODIN_SCM_RW,
					 ODIN_SCM_VAL, ODIN_SCM_RW,
					 ODIN_SCM_VAL);
	}

	mutex_lock(&ctx->app_lock);
	ret = odin_run_scm_with_listeners(ctx, ODIN_TZ_OWNER_TZ_APPS,
					  ODIN_TZ_SVC_APP_ID_PLACEHOLDER,
					  scm_cmd, arginfo,
					  args, &result, &resp_type, &data,
					  &blocked_lid);
	mutex_unlock(&ctx->app_lock);

	dev_info(ctx->dev,
		 "SEND_CMD app=%u ret=%d res=0x%llx type=0x%llx data=0x%llx blid=0x%x\n",
		 req.app_id, ret, result, resp_type, data, blocked_lid);

	/* Always copy back response & payload so caller can inspect TA logs */
	if (copy_to_user(u64_to_user_ptr(req.rsp_uaddr),
			 (u8 *)buf.vaddr + rsp_off, req.rsp_len)) {
		if (!ret)
			ret = -EFAULT;
	} else if (req.payload_len &&
		   copy_to_user(u64_to_user_ptr(req.payload_uaddr),
				payload.vaddr, req.payload_len)) {
		if (!ret)
			ret = -EFAULT;
	}

out_free:
	odin_shm_free(ctx->dev, &sglist);
	odin_shm_free(ctx->dev, &carveout);
	odin_shm_free(ctx->dev, &payload);
	odin_shm_free(ctx->dev, &buf);
	if (ret)
		return ret;

	req.qsee_result = (u32)result;
	req.blocked_listener_id = blocked_lid;

	if (copy_to_user(argp, &req, sizeof(req)))
		return -EFAULT;

	return 0;
}

static int odin_ioc_reg_listener(struct odin_file_priv *priv, void __user *argp)
{
	struct odin_qseecom_ctx *ctx = priv->ctx;
	struct odin_qseecom_listener_req req;
	struct odin_listener *lsnr;
	u64 args[10] = { 0 };
	u64 result = 0, resp_type = 0, data = 0;
	int ret;

	mutex_lock(&ctx->listener_lock);
	if (priv->listener) {
		mutex_unlock(&ctx->listener_lock);
		return -EBUSY;
	}
	mutex_unlock(&ctx->listener_lock);

	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	if (!req.listener_id || !req.sb_len ||
	    req.sb_len > ODIN_MAX_LISTENER_SB)
		return -EINVAL;

	lsnr = kzalloc_obj(*lsnr, GFP_KERNEL);
	if (!lsnr)
		return -ENOMEM;

	INIT_LIST_HEAD(&lsnr->list);
	kref_init(&lsnr->refcount);
	lsnr->ctx = ctx;
	lsnr->id = req.listener_id;
	init_waitqueue_head(&lsnr->req_wq);
	init_completion(&lsnr->resp_done);

	ret = odin_shm_alloc(ctx->dev, req.sb_len, &lsnr->sb);
	if (ret) {
		kfree(lsnr);
		return ret;
	}

	args[0] = lsnr->id;
	args[1] = lsnr->sb.paddr;
	args[2] = lsnr->sb.size;

	ret = qcom_scm_qseecom_raw_call(ODIN_TZ_OWNER_QSEE_OS,
					ODIN_TZ_SVC_LISTENER,
					ODIN_TZ_CMD_REG_LISTENER,
					ODIN_SCM_ARGS3(ODIN_SCM_VAL,
						       ODIN_SCM_RW,
						       ODIN_SCM_VAL),
					args, &result, &resp_type, &data);
	if (ret || result != ODIN_QSEE_RES_SUCCESS) {
		dev_err(ctx->dev,
			"REG_LISTENER 0x%x failed: ret=%d qsee_res=0x%llx\n",
			lsnr->id, ret, result);
		odin_shm_free(ctx->dev, &lsnr->sb);
		kfree(lsnr);
		if (ret)
			return ret;
		req.qsee_result = (u32)result;
		if (copy_to_user(argp, &req, sizeof(req)))
			return -EFAULT;
		return -EIO;
	}

	mutex_lock(&ctx->listener_lock);
	if (priv->listener) {
		mutex_unlock(&ctx->listener_lock);
		kref_put(&lsnr->refcount, odin_listener_release);
		return -EBUSY;
	}
	list_add_tail(&lsnr->list, &ctx->listeners);
	priv->listener = lsnr;
	mutex_unlock(&ctx->listener_lock);

	req.sb_len = lsnr->sb.size;
	req.qsee_result = (u32)result;
	if (copy_to_user(argp, &req, sizeof(req)))
		return -EFAULT;

	dev_info(ctx->dev, "Registered QSEE listener 0x%x (sb=%zu)\n",
		 lsnr->id, lsnr->sb.size);
	return 0;
}

static long odin_qseecom_ioctl(struct file *filp, unsigned int cmd,
			       unsigned long arg)
{
	struct odin_file_priv *priv = filp->private_data;
	struct odin_qseecom_ctx *ctx = priv->ctx;
	struct odin_qseecom_listener_req lreq;
	struct odin_listener *lsnr;
	void __user *argp = (void __user *)arg;
	long ret;

	switch (cmd) {
	case ODIN_QSEECOM_IOC_LOOKUP:
		return odin_ioc_lookup(ctx, argp);
	case ODIN_QSEECOM_IOC_LOAD:
		return odin_ioc_load(ctx, argp);
	case ODIN_QSEECOM_IOC_UNLOAD:
		return odin_ioc_unload(ctx, argp);
	case ODIN_QSEECOM_IOC_SEND_CMD:
		return odin_ioc_send_cmd(ctx, argp);
	case ODIN_QSEECOM_IOC_REG_LISTENER:
		return odin_ioc_reg_listener(priv, argp);
	case ODIN_QSEECOM_IOC_UNREG_LISTENER:
		return odin_detach_priv_listener(priv);
	case ODIN_QSEECOM_IOC_WAIT_LISTENER:
		lsnr = odin_get_priv_listener(priv);
		if (!lsnr)
			return -EINVAL;
		if (wait_event_interruptible(lsnr->req_wq,
					     lsnr->req_pending ||
					     lsnr->exiting)) {
			kref_put(&lsnr->refcount, odin_listener_release);
			return -ERESTARTSYS;
		}
		if (lsnr->exiting) {
			kref_put(&lsnr->refcount, odin_listener_release);
			return -ENODEV;
		}
		lsnr->req_pending = false;
		kref_put(&lsnr->refcount, odin_listener_release);
		return 0;
	case ODIN_QSEECOM_IOC_RESP_LISTENER:
		lsnr = odin_get_priv_listener(priv);
		if (!lsnr)
			return -EINVAL;
		if (copy_from_user(&lreq, argp, sizeof(lreq))) {
			kref_put(&lsnr->refcount, odin_listener_release);
			return -EFAULT;
		}
		lsnr->resp_status = lreq.status;
		complete(&lsnr->resp_done);
		kref_put(&lsnr->refcount, odin_listener_release);
		return 0;
	default:
		ret = -ENOTTY;
		return ret;
	}
}

static int odin_qseecom_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct odin_file_priv *priv = filp->private_data;
	struct odin_listener *lsnr;
	unsigned long size = vma->vm_end - vma->vm_start;
	int ret;

	lsnr = odin_get_priv_listener(priv);
	if (!lsnr)
		return -EINVAL;

	if (!lsnr->sb.vaddr || size > lsnr->sb.size) {
		kref_put(&lsnr->refcount, odin_listener_release);
		return -EINVAL;
	}

	ret = dma_mmap_coherent(priv->ctx->dev, vma,
				lsnr->sb.vaddr, lsnr->sb.paddr, size);
	kref_put(&lsnr->refcount, odin_listener_release);
	return ret;
}

static int odin_qseecom_open(struct inode *inode, struct file *filp)
{
	struct odin_file_priv *priv;

	priv = kzalloc_obj(*priv, GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->ctx = g_qseecom;
	filp->private_data = priv;
	return 0;
}

static int odin_qseecom_release(struct inode *inode, struct file *filp)
{
	struct odin_file_priv *priv = filp->private_data;

	odin_detach_priv_listener(priv);
	kfree(priv);
	return 0;
}

static const struct file_operations odin_qseecom_fops = {
	.owner = THIS_MODULE,
	.open = odin_qseecom_open,
	.release = odin_qseecom_release,
	.unlocked_ioctl = odin_qseecom_ioctl,
	.mmap = odin_qseecom_mmap,
};

static int __init odin_qseecom_init(void)
{
	struct odin_qseecom_ctx *ctx;
	int ret;

	ctx = kzalloc_obj(*ctx, GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->pdev = platform_device_register_simple("odin_qseecom", -1,
						    NULL, 0);
	if (IS_ERR(ctx->pdev)) {
		ret = PTR_ERR(ctx->pdev);
		kfree(ctx);
		return ret;
	}

	ctx->dev = &ctx->pdev->dev;
	ret = dma_coerce_mask_and_coherent(ctx->dev, DMA_BIT_MASK(32));
	if (ret) {
		platform_device_unregister(ctx->pdev);
		kfree(ctx);
		return ret;
	}

	mutex_init(&ctx->app_lock);
	mutex_init(&ctx->listener_lock);
	INIT_LIST_HEAD(&ctx->listeners);

	ctx->misc.minor = MISC_DYNAMIC_MINOR;
	ctx->misc.name = "odin_qseecom";
	ctx->misc.fops = &odin_qseecom_fops;
	ctx->misc.parent = ctx->dev;

	g_qseecom = ctx;
	ret = misc_register(&ctx->misc);
	if (ret) {
		g_qseecom = NULL;
		platform_device_unregister(ctx->pdev);
		kfree(ctx);
		return ret;
	}

	dev_info(ctx->dev, "Registered /dev/odin_qseecom (32-bit CMA DMA)\n");
	return 0;
}

static void __exit odin_qseecom_exit(void)
{
	struct odin_qseecom_ctx *ctx = g_qseecom;

	if (!ctx)
		return;

	misc_deregister(&ctx->misc);
	platform_device_unregister(ctx->pdev);
	g_qseecom = NULL;
	kfree(ctx);
}

module_init(odin_qseecom_init);
module_exit(odin_qseecom_exit);

MODULE_DESCRIPTION("Minimal Qualcomm QSEECOM experimental bridge for SM8350");
MODULE_LICENSE("GPL");
