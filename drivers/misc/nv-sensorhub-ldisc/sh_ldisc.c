/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/module.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/circ_buf.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/crc32.h>
#include <linux/poll.h>

#include "sh_private.h"


/* NOTE: The order here matches the Message Type
 * macros listed in sh_interface.h interface header file */
enum client_devs_num {
	DEV_MCU = 0, /* NOTE: This is the Sensorhub MCU itself */

	/* The following are the connected sensor devices */
	DEV_CAM,
	DEV_ACCEL,
	DEV_GYRO,
	DEV_MAG,
	DEV_BARO,
	NUM_DEVS
};

static char *client_devs_name[] = {
	"shub_mcu", /* NOTE: This is the sensorhub itself */

	/* The following are the connected sensor devices */
	"shub_cam",
	"shub_accel",
	"shub_gyro",
	"shub_mag",
	"shub_baro"
};

struct client_dev {
	/* Refer to parent data structure */
	struct ldisc_priv *ld_data;

	/* Misc dev. node */
	struct miscdevice mdev;

	/* Circular buffer where payloads are buffered */
	char *read_buf; /* Alloc 32k */
	int read_head;
	int read_tail;
	struct mutex read_buf_lock;

	/* Allows blocking read. */
	wait_queue_head_t readq;
};

struct ldisc_priv {
	/* Refer to parent data structure */
	struct tty_struct *tty;

	/* Write lock to single underlying tty device */
	struct mutex tty_write_lock;

	/* Read buffer - to hold one pkt before de-muxing */
	union {
		struct sensor_hub_pkt_t pkt;
		unsigned char    pkt_buf[sizeof(struct sensor_hub_pkt_t)];
	};
	int pkt_byte_idx;
	int pyld_len;

	/* Device nodes to de-multiplex data from sensorhub */
	struct client_dev client_devs[NUM_DEVS];
};

/* We are using crc32 to validate packets */
#define CRC_SIZE		4

#define PYLD(pkt)		(*(uint32_t *)(pkt + \
				sizeof(struct sensor_hub_pkt_header_t)))

#define CRC(pkt, pyld_sz)	(*(uint32_t *)(pkt + \
				sizeof(struct sensor_hub_pkt_header_t) + \
				pyld_sz))

#define CRC_DATA_SZ(pyld_sz)	(sizeof(struct sensor_hub_pkt_header_t) \
				 + pyld_sz)

#define PKT_SZ(pyld_sz)		(sizeof(struct sensor_hub_pkt_header_t) \
				 + pyld_sz + CRC_SIZE)

#if 0
uint32_t add_pkt_cnt;
uint32_t read_pkt_cnt;
#endif

/*****************************************************************************/

static int
pkt_type_valid(unsigned char byte)
{
	if ((byte >= MSG_SENSOR_START && byte <= MSG_SENSOR_END))
		return 1;
	return 0;
}

static int
pkt_payload_len(unsigned char type)
{
	switch (type) {
	case MSG_CAMERA:
		return sizeof(struct camera_payload_t);
	case MSG_ACCEL:
		return sizeof(struct accel_payload_t);
	case MSG_GYRO:
		return sizeof(struct gyro_payload_t);
	case MSG_MAG:
		return sizeof(struct mag_payload_t);
	case MSG_BARO:
		return sizeof(struct baro_payload_t);
	case MSG_MCU:
		return sizeof(struct mcu_payload_t);
	default:
		return -1;
	}
}

static uint32_t
pkt_crc(char *pkt, int len)
{
#if 0
	uint32_t crc;

	crc = crc32_le(~0, pkt, len);

	return crc;
#endif

	/* FIX IT: Temp code until CRC module is coded up in fw */
	uint32_t crc;

	crc = 0xCAFEBABA;

	return crc;
}

/*****************************************************************************/

static ssize_t
client_dev_write(struct file *file, const char __user *buffer,
		 size_t count, loff_t *ppos)
{
	struct client_dev *dev = file->private_data;
	struct ldisc_priv *ld_data = dev->ld_data;
	struct tty_struct *tty = ld_data->tty;
	const char *b;
	int c;
	int retval = 0;
	char pkt[sizeof(struct sensor_hub_pkt_t)];

	if (count == 0)
		return 0;

	/* Only writes to shub_mcu device is supported for now */
	if (count != sizeof(struct mcu_payload_t))
		return 0;

	if (mutex_lock_interruptible(&ld_data->tty_write_lock))
		return -EINTR;

	pr_debug("sh_ldisc: write line to dev struct 0x%p\n", dev);

	/* Make a packet and then send to Sensorhub */
	/* Header */
	((struct sensor_hub_pkt_t *)pkt)->header.start = SENSOR_HUB_START;
	((struct sensor_hub_pkt_t *)pkt)->header.type = MSG_MCU;
	/* Payload */
	if (get_user(PYLD(pkt), buffer)) {
		pr_err("sh_ldisc: Copy from user-space failed.\n");
		return -EFAULT;
	}

	/* CRC */
	CRC(pkt, sizeof(struct mcu_payload_t)) =
	    pkt_crc(pkt, CRC_DATA_SZ(sizeof(struct mcu_payload_t)));

	b = pkt;
	count = PKT_SZ(sizeof(struct mcu_payload_t));

#if 0
	int idx = 0;
	int dbg_cnt = count;
	while (dbg_cnt--)
		pr_debug("sh_ldisc: write char: 0x%x\n", pkt[idx++]);
#endif

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			break;
		}
		while (count > 0) {
			c = tty->ops->write(tty, b, count);
			if (c < 0) {
				retval = c;
				goto break_out;
			}
			if (!c)
				break;
			b += c;
			count -= c;
		}
		if (!count)
			break;
		if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			break;
		}
		schedule();
	}
break_out:

	if (tty->ops->flush_chars)
		tty->ops->flush_chars(tty);

	__set_current_state(TASK_RUNNING);
	if (b - buffer != count && tty->fasync)
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

	mutex_unlock(&ld_data->tty_write_lock);

	return (b - buffer) ? b - buffer : retval;
}

static ssize_t
client_dev_read(struct file *file, char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct client_dev *dev = file->private_data;
	ssize_t size;
	ssize_t msg_size;
	ssize_t retval = 0;
	char __user *b = buffer;
	int type_idx;

	if (mutex_lock_interruptible(&dev->read_buf_lock))
		return -ERESTARTSYS;

	pr_debug("sh_ldisc: read line from dev struct 0x%p\n", dev);
#if 0
	pr_debug("sh_ldisc: read pkt # %d at tail %d\n",
		read_pkt_cnt++, dev->read_tail);
#endif

	while (CIRC_CNT(dev->read_head, dev->read_tail, N_TTY_BUF_SIZE) == 0) {
		/* nothing to read */
		mutex_unlock(&dev->read_buf_lock); /* release the lock */
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(dev->readq,
			(CIRC_CNT(dev->read_head, dev->read_tail,
				  N_TTY_BUF_SIZE) != 0)))
			/* signal: tell fs layer to handle it */
			return -ERESTARTSYS;

		/* otherwise loop, but first reacquire the lock */
		if (mutex_lock_interruptible(&dev->read_buf_lock))
			return -ERESTARTSYS;
	}

	/* get the payload size */
	type_idx = ((dev->read_tail + sizeof(char)) & (N_TTY_BUF_SIZE-1));
	msg_size = pkt_payload_len(dev->read_buf[type_idx]);

	if (msg_size > count)
		retval = -EINVAL;
	else {
		dev->read_tail = dev->read_tail +
				 sizeof(struct sensor_hub_pkt_header_t);

		while (msg_size) {
			int c;

			c = dev->read_buf[dev->read_tail];
			dev->read_tail = ((dev->read_tail+1) &
					  (N_TTY_BUF_SIZE-1));
			msg_size--;

			if (put_user(c, b++)) {
				b--;
				/* Move read_tail to end of packet for sync */
				dev->read_tail = ((dev->read_tail+msg_size) &
						  (N_TTY_BUF_SIZE-1));
				pr_err("sh_ldisc: Copy to user-space failed.\n");
				retval = -EFAULT;
				break;
			}
		}

		/* Move read_tail to past crc */
		dev->read_tail = ((dev->read_tail+CRC_SIZE) &
				  (N_TTY_BUF_SIZE-1));
	}

	mutex_unlock(&dev->read_buf_lock); /* release the lock */

	size = b - buffer;
	if (size)
		retval = size;

	return retval;
}

static unsigned int client_dev_poll(struct file *file, poll_table *wait)
{
	struct client_dev *dev = file->private_data;
	unsigned int retval = 0;

	poll_wait(file, &dev->readq, wait);

	mutex_lock(&dev->read_buf_lock);
	/* Check if there is data in dev buffer */
	if (CIRC_CNT(dev->read_head, dev->read_tail, N_TTY_BUF_SIZE))
		retval |= POLLIN | POLLRDNORM;
	mutex_unlock(&dev->read_buf_lock);

	/* You can always write */
	retval |= POLLOUT | POLLWRNORM;

	return retval;
}

static int
client_dev_open(struct inode *inode, struct file *file)
{
	struct miscdevice *md = file->private_data;
	struct client_dev *dev;

	pr_info("%s : %s\n", __func__, md->name);

	dev = container_of(md, struct client_dev, mdev);

	mutex_lock(&dev->read_buf_lock);
	dev->read_head = dev->read_tail = 0;
	mutex_unlock(&dev->read_buf_lock);

	file->private_data = dev;
	nonseekable_open(inode, file);

	return 0;
}

static int
client_dev_release(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);
	return 0;
}

static const struct file_operations client_dev_fops = {
	.owner      = THIS_MODULE,
	.open       = client_dev_open,
	.release    = client_dev_release,
	.read       = client_dev_read,
	.write      = client_dev_write,
	.poll       = client_dev_poll,
	.llseek     = no_llseek,
};

/*****************************************************************************/

static int
create_client_devs(struct ldisc_priv *ld_data)
{
	int i;

	for (i = 0; i < NUM_DEVS; i++) {
		struct client_dev *dev = &ld_data->client_devs[i];
		struct miscdevice *md = &dev->mdev;

		memset(dev, 0, sizeof(struct client_dev));

		dev->read_buf = kzalloc(N_TTY_BUF_SIZE, GFP_KERNEL);
		if (!dev->read_buf) {
			pr_err("%s: out of memory\n", __func__);
			goto err_exit;
		} else {
			dev->ld_data = ld_data;
			mutex_init(&dev->read_buf_lock);
			init_waitqueue_head(&dev->readq);

			md->fops = &client_dev_fops;
			md->minor = MISC_DYNAMIC_MINOR;
			md->name = client_devs_name[i];

			if (misc_register(md) != 0) {
				pr_err("%s: misc_register fail\n", __func__);
				kfree(dev->read_buf);
				goto err_exit;
			}
			pr_debug("sh_ldisc: init dev %s 0x%p\n",
				 client_devs_name[i], dev);
		}
	}

	return 1;

err_exit:
	while (--i >= 0) {
		struct client_dev *dev = &ld_data->client_devs[i];
		struct miscdevice *md = &dev->mdev;

		if (misc_deregister(md) != 0)
			pr_err("%s: misc_deregister fail\n", __func__);

			kfree(dev->read_buf);
	}

	return 0;
}

static void
delete_client_devs(struct ldisc_priv *ld_data)
{
	int i;

	for (i = 0; i < NUM_DEVS; i++) {
		struct client_dev *dev = &ld_data->client_devs[i];
		struct miscdevice *md = &dev->mdev;

		wake_up_interruptible(&dev->readq);

		if (misc_deregister(md) != 0)
			pr_err("%s: misc_register fail\n", __func__);

		kfree(dev->read_buf);
	}
}

/*****************************************************************************/

static void
client_dev_add_pkt(struct client_dev *dev, unsigned char *pkt, int count)
{
	int buf_space_available;

	mutex_lock(&dev->read_buf_lock);

	pr_debug("sh_ldisc: add line to dev struct 0x%p\n", dev);
#if 0
	pr_debug("sh_ldisc: add pkt # %d at head %d\n",
		 add_pkt_cnt++, dev->read_head);
#endif
	buf_space_available = CIRC_SPACE(dev->read_head, dev->read_tail,
					 N_TTY_BUF_SIZE);

	if (buf_space_available > count) {
		int space = CIRC_SPACE_TO_END(dev->read_head, dev->read_tail,
					      N_TTY_BUF_SIZE);
		if (space > count) {
			memcpy(&dev->read_buf[dev->read_head], pkt, count);
		} else {
			memcpy(&dev->read_buf[dev->read_head], pkt, space);
			memcpy(&dev->read_buf[0], pkt + space, count - space);
		}
		dev->read_head += count;
		dev->read_head &= (N_TTY_BUF_SIZE - 1);
		wake_up_interruptible(&dev->readq);
	} else {
		pr_err("sh_ldisc: Discard pkt due to lack of buffer space.\n");
	}

	mutex_unlock(&dev->read_buf_lock);
}

static inline void
sh_ldisc_parse_pkt(struct tty_struct *tty, unsigned char c)
{
	struct ldisc_priv *ld_data = tty->disc_data;
	uint32_t crc;

	/* sanity check index */
	if (ld_data->pkt_byte_idx >= sizeof(ld_data->pkt_buf)) {
		/* Reset if byte index longer than longest packet */
		ld_data->pkt_byte_idx = 0;
	}

	ld_data->pkt_buf[ld_data->pkt_byte_idx] = c;

	switch (ld_data->pkt_byte_idx++) {
	case 0:
		/* expecting Magic value */
		if (c != SENSOR_HUB_START) {
			ld_data->pkt_byte_idx = 0;
			pr_debug("sh_ldisc: msg start not recvd 0x%x\n", c);
		}
		break;

	case 1:
		/* Expecting message type */
		if (!pkt_type_valid(c)) {
			pr_debug("sh_ldisc: msg type 0x%x not valid\n", c);
			ld_data->pkt_byte_idx = 0;
			break;
		}
		/* Calc payload len from msg. type */
		ld_data->pyld_len = pkt_payload_len(c);
		if (ld_data->pyld_len < 0) {
			pr_debug("sh_ldisc: msg len 0x%x not valid\n",
				 ld_data->pyld_len);
			ld_data->pkt_byte_idx = 0;
		}
		break;

	default:
		/* Nothing to do until last byte has been received */
		if (ld_data->pkt_byte_idx ==
			sizeof(struct sensor_hub_pkt_header_t) +
				ld_data->pyld_len + CRC_SIZE) {
#if 0
			/* For debugging */
			/* Print the accumulated packet content */
			int dbg_cnt = sizeof(struct sensor_hub_pkt_header_t) +
			ld_data->pyld_len + CRC_SIZE;
			int dbg_idx = 0;
			pr_debug("sh_ldisc: hdr len 0x%x\n",
			       sizeof(struct sensor_hub_pkt_header_t));
			pr_debug("sh_ldisc: pyl len 0x%x\n",
			       ld_data->pyld_len);
			pr_debug("sh_ldisc: crc len 0x%x\n",
			       CRC_SIZE);
			pr_debug("sh_ldisc: pkt addr 0x%p\n",
			       ld_data->pkt_buf);
			pr_debug("sh_ldisc: crc addr 0x%p\n",
			       ld_data->pkt_buf +
			       sizeof(struct sensor_hub_pkt_header_t) +
			       ld_data->pyld_len);
			pr_debug("sh_ldisc: crc 0x%x\n",
			       *(uint32_t *)(ld_data->pkt_buf +
				sizeof(struct sensor_hub_pkt_header_t) +
				ld_data->pyld_len));
			while (dbg_cnt--) {
				pr_debug("sh_ldisc: pkt byte 0x%x\n",
					ld_data->pkt_buf[dbg_idx++]);
			}
#endif
			/* validate packet crc */
			crc = pkt_crc(ld_data->pkt_buf,
				      CRC_DATA_SZ(ld_data->pyld_len));

			if (crc ==
			    CRC(ld_data->pkt_buf, ld_data->pyld_len)) {

				unsigned char type =
				    ld_data->pkt.header.type;
				int dev = DEV_MCU;

				/* If sensor */
				if (MSG_SENSOR_START <= type &&
				    type <= MSG_SENSOR_END) {
					dev = type;
				}

				/* Add the payload to the
				 * corresponding dev buffer */
				client_dev_add_pkt(
					&ld_data->client_devs[dev],
					ld_data->pkt_buf,
					ld_data->pkt_byte_idx);
			} else {
				pr_err("sh_ldisc: crc not valid\n");
			}

			/* Packet de-muxed successfully or dropped.
			 * Clear to start over. */
			ld_data->pkt_byte_idx = 0;
			ld_data->pyld_len = 0;
		}
		break;
	}
}

static void
sh_ldisc_recv_from_tty(struct tty_struct *tty, const unsigned char *cp,
			char *fp, int count)
{
	const unsigned char *p;
	char *f, flags = TTY_NORMAL;
	int i;
	char    buf[64];

	for (i = count, p = cp, f = fp; i; i--, p++) {
		if (f)
			flags = *f++;
		switch (flags) {
		case TTY_NORMAL:
			pr_debug("sh_ldisc: tty recv 0x%X\n", *p);
			sh_ldisc_parse_pkt(tty, *p);
			break;
		case TTY_BREAK:
		case TTY_PARITY:
		case TTY_FRAME:
		case TTY_OVERRUN:
			pr_debug("sh_ldisc: tty ctrl\n");
			/* Skip errors */
			break;
		default:
			pr_err("sh_ldisc: %s: unknown flag %d\n",
			       tty_name(tty, buf), flags);
			break;
		}
	}
}

static int
sh_ldisc_ioctl(struct tty_struct *tty, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int err;

	err = -EFAULT;

	switch (cmd) {
	case TCFLSH:
		pr_debug("%s flush ioctl\n", __func__);

		/* flush our buffers and the serial port's buffer */
		if (arg == TCIOFLUSH || arg == TCOFLUSH)
			;
		err = n_tty_ioctl_helper(tty, file, cmd, arg);
		break;

	default:
		pr_debug("%s default ioctl\n", __func__);

		err = tty_mode_ioctl(tty, file, cmd, arg);
		break;
	}

	return err;
}

static int
sh_ldisc_open(struct tty_struct *tty)
{
	struct ldisc_priv *ld_data;

	pr_info("%s\n", __func__);

	ld_data = kzalloc(sizeof(*ld_data), GFP_KERNEL);
	if (!ld_data)
		goto err;

	mutex_init(&ld_data->tty_write_lock);

	/* Init priv data */
	ld_data->tty = tty;
	ld_data->pkt_byte_idx = 0;
	ld_data->pyld_len = 0;

#if 0
	add_pkt_cnt = 0;
	read_pkt_cnt = 0;
#endif

	if (!create_client_devs(ld_data))
		goto err_free_bufs;

	/* Init tty struct */
	tty->disc_data = ld_data;
	tty->receive_room = N_TTY_BUF_SIZE;

	return 0;

err_free_bufs:
	kfree(ld_data);
err:
	return -ENOMEM;
}

static void
sh_ldisc_close(struct tty_struct *tty)
{
	struct ldisc_priv *ld_data = tty->disc_data;

	pr_info("%s\n", __func__);

	delete_client_devs(ld_data);

	kfree(ld_data);

	tty->disc_data = NULL;
}


static struct tty_ldisc_ops sh_ldisc = {
	.owner  = THIS_MODULE,
	.magic  = TTY_LDISC_MAGIC,
	.name   = "nv_senshub",
	.open   = sh_ldisc_open,
	.close  = sh_ldisc_close,
	.ioctl  = sh_ldisc_ioctl,
	.receive_buf = sh_ldisc_recv_from_tty,
};

static int __init
sh_ldisc_init(void)
{
	int err;

	err = tty_register_ldisc(N_NV_SENSHUB, &sh_ldisc);
	if (err != 0)
		pr_err("nv-sensorhub: error %d registering line disc.\n", err);
	return err;
}

static void __exit
sh_ldisc_cleanup(void)
{
	if (tty_unregister_ldisc(N_NV_SENSHUB) != 0)
		pr_err("nv-sensorhub: failed to unregister line disc.\n");
}

module_init(sh_ldisc_init);
module_exit(sh_ldisc_cleanup);

MODULE_DESCRIPTION("Nvidia sensorhub driver");
MODULE_AUTHOR("Arun Kannan <akannan@nvidia.com>");
MODULE_LICENSE("GPL");
