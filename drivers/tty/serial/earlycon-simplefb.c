// SPDX-License-Identifier: GPL-2.0
/*
 * Early console driver for pre-initialized 32-bpp linear framebuffers.
 */

#include <linux/console.h>
#include <linux/font.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/serial_core.h>
#include <linux/string.h>
#include <linux/vt_kern.h>

#include <asm/early_ioremap.h>

static struct console *simplefb_earlycon_con;
static const struct font_desc *simplefb_font;
static resource_size_t simplefb_phys;
static u32 simplefb_width = 1080;
static u32 simplefb_height = 2400;
static u32 simplefb_stride = 1080 * 4;
static u32 simplefb_cur_x;
static u32 simplefb_cur_y;
static void *simplefb_virt;

static int __init simplefb_earlycon_remap(void)
{
	size_t size = (size_t)simplefb_stride * simplefb_height;

	if (!simplefb_earlycon_con || !console_is_registered(simplefb_earlycon_con))
		return 0;

	simplefb_virt = memremap(simplefb_phys, size, MEMREMAP_WC);
	return simplefb_virt ? 0 : -ENOMEM;
}
early_initcall(simplefb_earlycon_remap);

static bool simplefb_vt_bound_real_con(void)
{
#if IS_ENABLED(CONFIG_VT) && IS_ENABLED(CONFIG_DUMMY_CONSOLE)
	struct vc_data *vc = READ_ONCE(vc_cons[0].d);

	if (vc && READ_ONCE(vc->vc_sw) && READ_ONCE(vc->vc_sw) != &dummy_con)
		return true;
#endif
	return false;
}

static int __init simplefb_earlycon_unmap(void)
{
	if (simplefb_earlycon_con && console_is_registered(simplefb_earlycon_con) &&
	    simplefb_vt_bound_real_con())
		unregister_console(simplefb_earlycon_con);

	if (simplefb_virt &&
	    (!simplefb_earlycon_con || !console_is_registered(simplefb_earlycon_con))) {
		memunmap(simplefb_virt);
		simplefb_virt = NULL;
	}
	return 0;
}
late_initcall_sync(simplefb_earlycon_unmap);

static __ref void *simplefb_map_line(unsigned long offset, unsigned long len)
{
	pgprot_t prot;

	if (simplefb_virt)
		return simplefb_virt + offset;

	prot = pgprot_writecombine(PAGE_KERNEL);
	return early_memremap_prot(simplefb_phys + offset, len, pgprot_val(prot));
}

static __ref void simplefb_unmap_line(void *addr, unsigned long len)
{
	if (simplefb_virt)
		return;

	early_memunmap(addr, len);
}

static void simplefb_fill_scanline(u32 y, u32 pixel)
{
	u32 *dst;
	u32 i;

	dst = simplefb_map_line((unsigned long)y * simplefb_stride, simplefb_stride);
	if (!dst)
		return;

	for (i = 0; i < simplefb_width; i++)
		dst[i] = pixel;

	simplefb_unmap_line(dst, simplefb_stride);
}

static void simplefb_clear_band(u32 y_start, u32 lines, u32 pixel)
{
	u32 i;

	for (i = 0; i < lines && (y_start + i) < simplefb_height; i++)
		simplefb_fill_scanline(y_start + i, pixel);
}

static void simplefb_draw_glyph_row(u32 *dst, unsigned char c, u32 row)
{
	const u32 fg = 0x00ffffff;
	const u32 bg = 0x00000000;
	unsigned int bytes = BITS_TO_BYTES(simplefb_font->width);
	const u8 *src = simplefb_font->data +
			(unsigned int)c * simplefb_font->height * bytes +
			row * bytes;
	unsigned int col;

	for (col = 0; col < simplefb_font->width; col++) {
		u8 byte = src[col / 8];
		u8 bit = (byte >> (7 - (col % 8))) & 1;

		dst[col] = bit ? fg : bg;
	}
}

static void simplefb_newline(void)
{
	u32 fh = simplefb_font->height;

	simplefb_cur_x = 0;
	simplefb_cur_y += fh;
	if (simplefb_cur_y + fh > simplefb_height)
		simplefb_cur_y = 0;

	/* Clear current line and draw a thin marker band below it */
	simplefb_clear_band(simplefb_cur_y, fh, 0x00000000);
	if (simplefb_cur_y + fh + 2 <= simplefb_height)
		simplefb_clear_band(simplefb_cur_y + fh, 2, 0x0000a0ff);
}

static void simplefb_earlycon_write(struct console *con, const char *str,
				    unsigned int num)
{
	u32 fw = simplefb_font->width;
	u32 fh = simplefb_font->height;

	if (simplefb_vt_bound_real_con())
		return;

	while (num > 0) {
		unsigned int max_chars = (simplefb_width - simplefb_cur_x) / fw;
		const char *nl;
		unsigned int count;
		u32 row;

		if (*str == '\r') {
			str++;
			num--;
			continue;
		}

		if (*str == '\n') {
			simplefb_newline();
			str++;
			num--;
			continue;
		}

		if (max_chars == 0) {
			simplefb_newline();
			max_chars = simplefb_width / fw;
		}

		nl = memchr(str, '\n', num);
		count = nl ? (unsigned int)(nl - str) : num;
		if (count > max_chars)
			count = max_chars;
		/* Stop before CR as well */
		nl = memchr(str, '\r', count);
		if (nl)
			count = (unsigned int)(nl - str);

		if (count > 0) {
			for (row = 0; row < fh; row++) {
				unsigned long off = (unsigned long)(simplefb_cur_y + row) *
						    simplefb_stride;
				u32 *dst = simplefb_map_line(off, simplefb_stride);
				unsigned int i;

				if (!dst)
					return;

				for (i = 0; i < count; i++) {
					simplefb_draw_glyph_row(dst + simplefb_cur_x + i * fw,
								(unsigned char)str[i], row);
				}

				simplefb_unmap_line(dst, simplefb_stride);
			}

			simplefb_cur_x += count * fw;
			str += count;
			num -= count;
		}
	}
}

static int __init simplefb_earlycon_setup(struct earlycon_device *device,
					  const char *opt)
{
	const char *res_str = opt;
	u32 w = 0, h = 0;

	if (device->port.mapbase) {
		simplefb_phys = device->port.mapbase;
		res_str = device->options;
	} else if (opt) {
		const char *comma = strchr(opt, ',');
		char addr_buf[32];
		size_t len = comma ? (size_t)(comma - opt) : strlen(opt);
		u64 addr = 0;

		if (len > 0 && len < sizeof(addr_buf)) {
			memcpy(addr_buf, opt, len);
			addr_buf[len] = '\0';
			if (!kstrtoull(addr_buf, 0, &addr))
				simplefb_phys = addr;
		}
		res_str = comma ? (comma + 1) : NULL;
	}

	if (!simplefb_phys)
		return -EINVAL;

	if (res_str && *res_str) {
		if (sscanf(res_str, "%ux%u", &w, &h) == 2 && w >= 64 && h >= 64) {
			simplefb_width = w;
			simplefb_height = h;
		}
	}

	simplefb_stride = simplefb_width * 4;
	if (simplefb_width >= 1000 && simplefb_height >= 1000)
		simplefb_font = find_font("TER16x32");
	if (!simplefb_font)
		simplefb_font = get_default_font(simplefb_width, simplefb_height,
						 NULL, NULL);
	if (!simplefb_font)
		return -ENODEV;

	simplefb_cur_x = 0;
	simplefb_cur_y = 0;
	simplefb_clear_band(0, simplefb_font->height, 0x00000000);

	simplefb_earlycon_con = device->con;
	device->con->write = simplefb_earlycon_write;
	return 0;
}
EARLYCON_DECLARE(simplefb, simplefb_earlycon_setup);
