/*
 * Renesas uPD78F0730 USB-to-serial converter driver
 *
 * Copyright (C) 2014 Maksim Salau <maksim.salau@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tty.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define DRIVER_DESC "Renesas uPD78F0730 USB-to-serial converter driver"

#define DRIVER_AUTHOR "Maksim Salau <maksim.salau@gmail.com>"

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(0x045B, 0x0212) }, /* YRPBRL78G13, YRPBRL78G14 */
	{ USB_DEVICE(0x0409, 0x0063) }, /* V850ESJX3-STICK */
	{}
};

MODULE_DEVICE_TABLE(usb, id_table);

struct upd78f0730_serial_private {
	spinlock_t	lock;
	__u8		line_signals;
};

static int upd78f0730_send_ctl(struct usb_serial_port *port,
			void *data, int size);
static int upd78f0730_attach(struct usb_serial *serial);
static void upd78f0730_release(struct usb_serial *serial);
static int upd78f0730_open(struct tty_struct *tty,
			struct usb_serial_port *port);
static void upd78f0730_close(struct usb_serial_port *port);
static void upd78f0730_set_termios(struct tty_struct *tty,
				struct usb_serial_port *port,
				struct ktermios *old_termios);
static int upd78f0730_tiocmget(struct tty_struct *tty);
static int upd78f0730_tiocmset(struct tty_struct *tty,
			unsigned int set, unsigned int clear);
static void upd78f0730_dtr_rts(struct usb_serial_port *port, int on);
static void upd78f0730_break_ctl(struct tty_struct *tty, int break_state);

static struct usb_serial_driver upd78f0730_device = {
	.driver	 = {
		.owner	= THIS_MODULE,
		.name	= "upd78f0730",
	},
	.id_table	= id_table,
	.num_ports	= 1,
	.bulk_in_size	= 64,
	.bulk_out_size	= 64,
	.attach		= upd78f0730_attach,
	.release	= upd78f0730_release,
	.open		= upd78f0730_open,
	.close		= upd78f0730_close,
	.set_termios	= upd78f0730_set_termios,
	.tiocmget	= upd78f0730_tiocmget,
	.tiocmset	= upd78f0730_tiocmset,
	.dtr_rts	= upd78f0730_dtr_rts,
	.break_ctl	= upd78f0730_break_ctl,
};

static struct usb_serial_driver * const serial_drivers[] = {
	&upd78f0730_device,
	NULL
};

#define LINE_CONTROL		0x00
#define SET_DTR_RTS		0x01
#define SET_XON_XOFF_CHR	0x02
#define OPEN_CLOSE		0x03
#define SET_ERR_CHR		0x04

#define DATA_SIZE_7_BITS	0x00
#define DATA_SIZE_8_BITS	0x01
#define DATA_SIZE_MASK		0x01

#define STOP_BIT_1_BIT	0x00
#define STOP_BIT_2_BIT	0x02
#define STOP_BIT_MASK	0x02

#define PARITY_NONE	0x00
#define PARITY_EVEN	0x04
#define PARITY_ODD	0x08
#define PARITY_MASK	0x0C

#define FLOW_CONTROL_NONE	0x00
#define FLOW_CONTROL_HW		0x10
#define FLOW_CONTROL_SW		0x20
#define FLOW_CONTROL_MASK	0x30

#define RESET_RTS	0x01
#define RESET_DTR	0x02
#define SET_BREAK	0x04

#define PORT_CLOSE	0x00
#define PORT_OPEN	0x01

#define ERR_CHR_DISABLED	0x00
#define ERR_CHR_ENABLED		0x01

struct line_control {
	__u8	opcode;
	__le32	baud_rate;
	__u8	params;
} __packed;

struct set_dtr_rts {
	__u8 opcode;
	__u8 params;
};

struct set_xon_xoff_chr {
	__u8 opcode;
	__u8 xon;
	__u8 xoff;
};

struct open_close {
	__u8 opcode;
	__u8 state;
};

struct set_err_chr {
	__u8 opcode;
	__u8 state;
	__u8 err_char;
};

static int upd78f0730_send_ctl(struct usb_serial_port *port,
			void *data, int size)
{
	int res;
	struct device *dev = &port->dev;
	struct usb_device *usbdev = port->serial->dev;

	if (port == NULL || data == NULL || size == 0) {
		dev_err(dev, "%s - invalid arguments\n", __func__);
		return -EINVAL;
	}

	res = usb_control_msg(usbdev, usb_sndctrlpipe(usbdev, 0), 0x00,
			USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
			0x0000, 0x0000, data, size, USB_CTRL_SET_TIMEOUT);

	if (res < 0 || res != size) {
		dev_err(dev, "%s - failed to send request opcode=%02x, size=%d res=%d\n",
			__func__, *(__u8 *)data, size, res);
		return -EIO;
	}

	return 0;
}

static int upd78f0730_attach(struct usb_serial *serial)
{
	struct upd78f0730_serial_private *private;
	struct device *dev = &serial->dev->dev;

	dev_dbg(dev, "%s\n", __func__);
	private = kzalloc(sizeof(*private), GFP_KERNEL);
	if (private == NULL) {
		dev_err(dev, "%s - unable to allocate memory for private data\n",
			__func__);
		return -ENOMEM;
	}
	spin_lock_init(&private->lock);
	private->line_signals = RESET_DTR | RESET_RTS;
	usb_set_serial_data(serial, private);
	return 0;
}

static void upd78f0730_release(struct usb_serial *serial)
{
	struct upd78f0730_serial_private *private;

	dev_dbg(&serial->dev->dev, "%s\n", __func__);
	private = usb_get_serial_data(serial);
	kfree(private);
}

static int upd78f0730_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	int res;
	unsigned long flags;
	struct upd78f0730_serial_private *private;
	struct open_close request_open = {
		.opcode = OPEN_CLOSE,
		.state = PORT_OPEN
	};
	struct set_dtr_rts request_set_dtr_rts = {
		.opcode = SET_DTR_RTS,
		.params = RESET_DTR | RESET_RTS
	};
	struct set_err_chr request_set_err_chr = {
		.opcode = SET_ERR_CHR,
		.state = ERR_CHR_DISABLED,
		.err_char = 0
	};

	struct {
		void	*data;
		int	size;
	} *request, requests[] = {
		{ &request_open, sizeof(request_open) },
		{ &request_set_dtr_rts, sizeof(request_set_dtr_rts) },
		{ &request_set_err_chr, sizeof(request_set_err_chr) },
		{ }
	};

	dev_dbg(&port->dev, "%s\n", __func__);
	private = usb_get_serial_data(port->serial);
	spin_lock_irqsave(&private->lock, flags);
	request_set_dtr_rts.params = private->line_signals;
	spin_unlock_irqrestore(&private->lock, flags);

	request = requests;
	do {
		res = upd78f0730_send_ctl(port, request->data, request->size);
		if (res)
			return res;
		++request;
	} while (request->data != NULL);

	upd78f0730_set_termios(tty, port, NULL);

	return usb_serial_generic_open(tty, port);
}

static void upd78f0730_close(struct usb_serial_port *port)
{
	struct open_close request_close = {
		.opcode = OPEN_CLOSE,
		.state = PORT_CLOSE
	};

	dev_dbg(&port->dev, "%s\n", __func__);
	usb_serial_generic_close(port);
	upd78f0730_send_ctl(port, &request_close, sizeof(request_close));
}

static void upd78f0730_set_termios(struct tty_struct *tty,
				struct usb_serial_port *port,
				struct ktermios *old_termios)
{
	struct device *dev = &port->dev;
	struct line_control request = { .opcode = LINE_CONTROL };
	struct set_xon_xoff_chr request_xchr = { .opcode = SET_XON_XOFF_CHR };
	tcflag_t cflag = tty->termios.c_cflag;
	speed_t baud_rate = tty_get_baud_rate(tty);

	request.baud_rate = cpu_to_le32(baud_rate);
	dev_dbg(dev, "%s - baud rate = %d\n", __func__, baud_rate);

	request_xchr.xon = START_CHAR(tty);
	request_xchr.xoff = STOP_CHAR(tty);
	dev_dbg(dev, "%s - XON = %02X, XOFF = %02X\n", __func__,
		request_xchr.xon, request_xchr.xoff);

	request.params = 0;
	switch (cflag & CSIZE) {
	case CS7:
		request.params |= DATA_SIZE_7_BITS;
		dev_dbg(dev, "%s - 7 data bits\n", __func__);
		break;
	case CS8:
		request.params |= DATA_SIZE_8_BITS;
		dev_dbg(dev, "%s - 8 data bits\n", __func__);
		break;
	default:
		request.params |= DATA_SIZE_8_BITS;
		dev_err(dev, "%s - data size is not supported, falling back to 8 bits\n",
			__func__);
		break;
	}
	if (cflag & PARENB) {
		if (cflag & PARODD) {
			request.params |= PARITY_ODD;
			dev_dbg(dev, "%s - odd parity\n", __func__);
		} else {
			request.params |= PARITY_EVEN;
			dev_dbg(dev, "%s - even parity\n", __func__);
		}
	} else {
		request.params |= PARITY_NONE;
		dev_dbg(dev, "%s - no parity\n", __func__);
	}
	if (cflag & CSTOPB) {
		request.params |= STOP_BIT_2_BIT;
		dev_dbg(dev, "%s - 2 stop bits\n", __func__);
	} else {
		request.params |= STOP_BIT_1_BIT;
		dev_dbg(dev, "%s - 1 stop bit\n", __func__);
	}
	if (cflag & CRTSCTS) {
		dev_err(dev, "%s - hardware flow control is not supported\n",
			__func__);
	}
	if (I_IXOFF(tty) || I_IXON(tty)) {
		request.params |= FLOW_CONTROL_SW;
		dev_dbg(dev, "%s - software flow control\n", __func__);
	} else {
		request.params |= FLOW_CONTROL_NONE;
		dev_dbg(dev, "%s - no flow control\n", __func__);
	}

	upd78f0730_send_ctl(port, &request, sizeof(request));
	upd78f0730_send_ctl(port, &request_xchr, sizeof(request_xchr));
}

static int upd78f0730_tiocmget(struct tty_struct *tty)
{
	int res = 0;
	int signals;
	unsigned long flags;
	struct device *dev = tty->dev;
	struct upd78f0730_serial_private *private;
	struct usb_serial_port *port = tty->driver_data;

	private = usb_get_serial_data(port->serial);

	spin_lock_irqsave(&private->lock, flags);
	signals = private->line_signals;
	spin_unlock_irqrestore(&private->lock, flags);

	res = ((signals & RESET_DTR) ? 0 : TIOCM_DTR)
		| ((signals & RESET_RTS) ? 0 : TIOCM_RTS);

	dev_dbg(dev, "%s - res = %x\n", __func__, res);

	return res;
}

static int upd78f0730_tiocmset(struct tty_struct *tty,
			unsigned int set, unsigned int clear)
{
	int res;
	unsigned long flags;
	struct device *dev = tty->dev;
	struct upd78f0730_serial_private *private;
	struct usb_serial_port *port = tty->driver_data;
	struct set_dtr_rts request = { .opcode = SET_DTR_RTS };

	private = usb_get_serial_data(port->serial);

	dev_dbg(dev, "%s\n", __func__);
	spin_lock_irqsave(&private->lock, flags);
	if (set & TIOCM_DTR) {
		private->line_signals &= ~RESET_DTR;
		dev_dbg(dev, "%s - set DTR\n", __func__);
	}
	if (set & TIOCM_RTS) {
		private->line_signals &= ~RESET_RTS;
		dev_dbg(dev, "%s - set RTS\n", __func__);
	}
	if (clear & TIOCM_DTR) {
		private->line_signals |= RESET_DTR;
		dev_dbg(dev, "%s - reset DTR\n", __func__);
	}
	if (clear & TIOCM_RTS) {
		private->line_signals |= RESET_RTS;
		dev_dbg(dev, "%s - reset RTS\n", __func__);
	}
	request.params = private->line_signals;
	spin_unlock_irqrestore(&private->lock, flags);

	res = upd78f0730_send_ctl(port, &request, sizeof(request));

	return res;
}

static void upd78f0730_dtr_rts(struct usb_serial_port *port, int on)
{
	unsigned long flags;
	struct device *dev = &port->dev;
	struct upd78f0730_serial_private *private;
	struct set_dtr_rts request = { .opcode = SET_DTR_RTS };

	private = usb_get_serial_data(port->serial);

	spin_lock_irqsave(&private->lock, flags);
	if (on) {
		private->line_signals &= ~(RESET_DTR | RESET_RTS);
		dev_dbg(dev, "%s - set DTR and RTS\n", __func__);
	} else {
		private->line_signals |= RESET_DTR | RESET_RTS;
		dev_dbg(dev, "%s - reset DTR and RTS\n", __func__);
	}
	request.params = private->line_signals;
	spin_unlock_irqrestore(&private->lock, flags);

	upd78f0730_send_ctl(port, &request, sizeof(request));
}

static void upd78f0730_break_ctl(struct tty_struct *tty, int break_state)
{
	unsigned long flags;
	struct device *dev = tty->dev;
	struct upd78f0730_serial_private *private;
	struct usb_serial_port *port = tty->driver_data;
	struct set_dtr_rts request = { .opcode = SET_DTR_RTS };

	private = usb_get_serial_data(port->serial);

	spin_lock_irqsave(&private->lock, flags);
	if (break_state) {
		private->line_signals |= SET_BREAK;
		dev_dbg(dev, "%s - set BREAK\n", __func__);
	} else {
		private->line_signals &= ~SET_BREAK;
		dev_dbg(dev, "%s - reset BREAK\n", __func__);
	}
	request.params = private->line_signals;
	spin_unlock_irqrestore(&private->lock, flags);

	upd78f0730_send_ctl(port, &request, sizeof(request));
}

module_usb_serial_driver(serial_drivers, id_table);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
