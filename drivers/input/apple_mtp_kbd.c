// SPDX-License-Identifier: GPL-2.0+ OR MIT
/*
 * Copyright The Asahi Linux Contributors
 */

#include <dm.h>
#include <dm/simple_bus.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <mailbox.h>
#include <keyboard.h>
#include <stdio_dev.h>
#include <asm/arch/rtkit.h>
#include <asm/io.h>
#include <linux/input.h>
#include "apple_kbd.h"

struct apple_mtp_kbd_priv {
	struct apple_kbd_priv kbd;
	struct udevice *helper;
	void *local;
	void *rmt;

	u8 *init_data;
	u32 init_size;
	u32 fifo_size;

	/* Interface indexes from the init packets, -1 until seen. */
	int stm_iface;
	int kbd_iface;
	bool stm_enabled;
	bool kbd_enabled;
	bool cmd_pending;
	u8 tx_seq;
};

#define DATA_TX8		0x4
#define DATA_TX_FREE		0x14
#define DATA_RX8		0x1c
#define DATA_RX_COUNT		0x2c

#define DCHID_CHANNEL_CMD	0x11
#define DCHID_CHANNEL_REPORT	0x12

#define IFACE_COMM		0
#define EVENT_INIT		0xf0
#define CMD_ENABLE_INTERFACE	0xb4
/* Feature report, set report: FLAGS_GROUP = 2, FLAGS_REQ = 0. */
#define FLAGS_FEATURE_SET	0x80

#define MAX_PKT			0x200

struct dchid_hdr {
	u8 hdr_len;
	u8 channel;
	__le16 length;
	u8 seq;
	u8 iface;
	__le16 pad;
} __packed;

struct dchid_subhdr {
	u8 flags;
	u8 unk;
	__le16 length;
	__le32 retcode;
} __packed;

struct dchid_init_hdr {
	u8 type;
	u8 unk1;
	u8 unk2;
	u8 iface;
	char name[16];
	u8 more_packets;
	u8 unkpad;
} __packed;

static int dockchannel_read(struct udevice *dev, void *buf, size_t size)
{
	struct apple_mtp_kbd_priv *priv = dev_get_priv(dev);
	int ret = 0;
	u8 b;
	u8 *p = buf;

	while (size--) {
		ulong start;
		start = get_timer(0);
		while (get_timer(start) < 100) {
			if (readl(priv->local + DATA_RX_COUNT) != 0)
				break;
		}

		if (readl(priv->local + DATA_RX_COUNT) == 0) {
			return -ETIME;
		}

		b = readl(priv->local + DATA_RX8) >> 8;
		if (buf)
			*p++ = b;

		ret++;
	}

	return ret;
}

static u32 dchid_checksum(const void *p, size_t length)
{
	const u8 *b = p;
	u32 sum = 0;

	for (; length >= 4; b += 4, length -= 4)
		sum += b[0] | b[1] << 8 | b[2] << 16 | (u32)b[3] << 24;
	return sum;
}

static void dockchannel_write(struct apple_mtp_kbd_priv *priv, const void *buf,
			      size_t size)
{
	const u8 *p = buf;

	while (size) {
		if (!readl(priv->local + DATA_TX_FREE))
			continue;
		writel(*p++, priv->local + DATA_TX8);
		size--;
	}
}

/*
 * Enable an MTP interface the way the Linux dockchannel-hid driver does on
 * J713: command 0xb4 as a feature set report to the comm interface. The
 * 26A428 MTP firmware sends keyboard reports only once the keyboard
 * interface is enabled ("Keyboard interface configuration completed",
 * keyboard_cr.c, then "Keyboard ready", keyboard.c).
 */
static void apple_mtp_kbd_enable(struct apple_mtp_kbd_priv *priv, int iface)
{
	struct {
		struct dchid_hdr hdr;
		struct dchid_subhdr sub;
		u8 msg[4];
	} __packed pkt = {
		.hdr = {
			.hdr_len = sizeof(struct dchid_hdr),
			.channel = DCHID_CHANNEL_CMD,
			.length = cpu_to_le16(sizeof(struct dchid_subhdr) + 4),
			.seq = priv->tx_seq++,
			.iface = IFACE_COMM,
		},
		.sub = {
			.flags = FLAGS_FEATURE_SET,
			.length = cpu_to_le16(2),
		},
		.msg = { CMD_ENABLE_INTERFACE, iface },
	};
	u32 checksum = 0xffffffff - dchid_checksum(&pkt, sizeof(pkt));

	dockchannel_write(priv, &pkt, sizeof(pkt));
	dockchannel_write(priv, &checksum, sizeof(checksum));
	priv->cmd_pending = true;
}

static void apple_mtp_kbd_note_init(struct apple_mtp_kbd_priv *priv,
				    const u8 *payload, size_t length)
{
	const struct dchid_init_hdr *init = (const void *)payload;

	if (length < sizeof(*init) || init->type != EVENT_INIT || init->more_packets)
		return;
	if (!strncmp(init->name, "stm", sizeof(init->name)))
		priv->stm_iface = init->iface;
	else if (!strncmp(init->name, "keyboard", sizeof(init->name)))
		priv->kbd_iface = init->iface;
}

/* STM first, then the keyboard, one command at a time, as Linux does. */
static void apple_mtp_kbd_advance(struct apple_mtp_kbd_priv *priv)
{
	if (!priv->helper || priv->cmd_pending)
		return;
	if (!priv->stm_enabled && priv->stm_iface >= 0) {
		apple_mtp_kbd_enable(priv, priv->stm_iface);
		priv->stm_enabled = true;
	} else if (priv->stm_enabled && !priv->kbd_enabled && priv->kbd_iface >= 0) {
		apple_mtp_kbd_enable(priv, priv->kbd_iface);
		priv->kbd_enabled = true;
	}
}

static int apple_mtp_kbd_check(struct input_config *input)
{
	struct udevice *dev = input->dev;
	struct apple_mtp_kbd_priv *priv = dev_get_priv(dev);
	struct dchid_hdr hdr;
	u8 buf[MAX_PKT + 4];
	const struct dchid_subhdr *sub = (const void *)buf;
	int ret;

	/* Poll for syslogs if RTKit is up */
	if (priv->helper)
		apple_rtkit_helper_poll(priv->helper, 0);

	apple_mtp_kbd_advance(priv);

	u32 pending = readl(priv->local + DATA_RX_COUNT);
	if (pending < 8)
		return 0;

	ret = dockchannel_read(dev, &hdr, sizeof(hdr));
	if (ret < 0) {
		dev_err(dev, "failed to read packet header\n");
		return ret;
	}

	if (hdr.length > MAX_PKT) {
		printk("mtp: oversized packet ch=%02x l=%04x if=%02x\n",
		       hdr.channel, hdr.length, hdr.iface);
		return dockchannel_read(dev, NULL, hdr.length + 4);
	}

	ret = dockchannel_read(dev, buf, hdr.length + 4);
	if (ret < 0)
		return ret;

	/* Acknowledges of our own commands stay here. */
	if (hdr.channel == DCHID_CHANNEL_CMD) {
		priv->cmd_pending = false;
		return 0;
	}

	/* Save comm init messages for the next stage */
	if (hdr.iface == IFACE_COMM) {
		int space = priv->fifo_size - priv->init_size;
		int need = hdr.length + sizeof(hdr) + 4;

		if (hdr.length >= sizeof(*sub))
			apple_mtp_kbd_note_init(priv, buf + sizeof(*sub),
						min_t(size_t, sub->length,
						      hdr.length - sizeof(*sub)));

		if (space < need) {
			dev_err(dev, "out of buf space (%d > %d)\n",
				need, space);
		} else {
			u8 *p = &priv->init_data[priv->init_size];

			memcpy(p, &hdr, sizeof(hdr));
			memcpy(p + sizeof(hdr), buf, hdr.length + 4);
			priv->init_size += need;
		}
		return 0;
	}

	if (hdr.channel == DCHID_CHANNEL_REPORT && hdr.iface == priv->kbd_iface &&
	    hdr.length >= sizeof(*sub)) {
		if (!priv->helper)
			return 1; /* Ignore if shutting down */

		return apple_kbd_handle_report(input, &priv->kbd, buf + sizeof(*sub),
					       min_t(size_t, sub->length,
						     hdr.length - sizeof(*sub)));
	}

	return 0;
}

static int get_rtkit_helper(struct udevice *dev)
{
	struct apple_mtp_kbd_priv *priv = dev_get_priv(dev);
	int ret;
	u32 phandle;
	ofnode of_mtp;

	ret = dev_read_u32(dev, "apple,helper-cpu", &phandle);
	if (ret < 0)
		return ret;

	of_mtp = ofnode_get_by_phandle(phandle);
	ret = uclass_get_device_by_ofnode(UCLASS_MISC, of_mtp, &priv->helper);
	if (ret < 0)
		return ret;

	return 0;
}

static int apple_mtp_kbd_probe(struct udevice *dev)
{
	struct apple_mtp_kbd_priv *priv = dev_get_priv(dev);
	struct keyboard_priv *uc_priv = dev_get_uclass_priv(dev);
	struct stdio_dev *sdev = &uc_priv->sdev;
	struct input_config *input = &uc_priv->input;
	int ret;
	fdt_addr_t reg;

	printf("mtp_kbd_probe\n");

	reg = dev_read_addr_name(dev, "data");
	if (reg == FDT_ADDR_T_NONE) {
		dev_err(dev, "no reg property for local FIFO data registers\n");
		return -EINVAL;
	}
	priv->local = (void *)reg;

	reg = dev_read_addr_name(dev, "rmt-data");
	if (reg == FDT_ADDR_T_NONE) {
		dev_err(dev, "no reg property for remote FIFO data registers\n");
		return -EINVAL;
	}
	priv->rmt = (void *)reg;

	ret = dev_read_u32(dev, "apple,fifo-size", &priv->fifo_size);
	if (ret < 0 || !priv->fifo_size) {
		dev_err(dev, "no apple,fifo-size property\n");
		return ret;
	}

	ret = get_rtkit_helper(dev);
	if (ret < 0) {
		dev_err(dev, "Failed to get helper device (%d)\n", ret);
		return ret;
	}

	priv->init_data = malloc(priv->fifo_size);
	if (!priv->init_data)
		return -ENOMEM;

	priv->stm_iface = -1;
	priv->kbd_iface = -1;

	input->dev = dev;
	input->read_keys = apple_mtp_kbd_check;
	input_add_tables(input, false);
	strcpy(sdev->name, "mtpkbd");

	return input_stdio_register(sdev);
}

static int apple_mtp_kbd_remove(struct udevice *dev)
{
	struct apple_mtp_kbd_priv *priv = dev_get_priv(dev);
	struct keyboard_priv *uc_priv = dev_get_uclass_priv(dev);
	struct input_config *input = &uc_priv->input;
	int i;

	if (priv->helper) {
		device_remove(priv->helper, DM_REMOVE_NORMAL);
		priv->helper = NULL;
	}

	/* Drain the FIFO */
	while (readl(priv->local + DATA_RX_COUNT)) {
		if (apple_mtp_kbd_check(input) < 0) {
			dev_err(dev, "Failed to drain FIFO\n");
			break;
		}
	}

	/* Stuff init messages back into FIFO for the next stage to find */
	for (i = 0; i < priv->init_size; i++)
		writel(priv->init_data[i], priv->rmt + DATA_TX8);

	return 0;
}

static const struct keyboard_ops apple_mtp_kbd_ops = {
};

static const struct udevice_id apple_mtp_kbd_of_match[] = {
	{ .compatible = "apple,dockchannel-hid" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(apple_mtp_kbd) = {
	.name = "apple_mtp_kbd",
	.id = UCLASS_KEYBOARD,
	.of_match = apple_mtp_kbd_of_match,
	.probe = apple_mtp_kbd_probe,
	.remove = apple_mtp_kbd_remove,
	.priv_auto = sizeof(struct apple_mtp_kbd_priv),
	.ops = &apple_mtp_kbd_ops,
	.flags = DM_FLAG_OS_PREPARE,
};

/* Treat dockchannel as a simple-bus, since we don't use the IRQ stuff */

static const struct udevice_id dockchannel_bus_ids[] = {
	{ .compatible = "apple,dockchannel" },
	{ }
};

U_BOOT_DRIVER(dockchannel) = {
	.name	= "dockchannel",
	.id	= UCLASS_SIMPLE_BUS,
	.of_match = of_match_ptr(dockchannel_bus_ids),
};
