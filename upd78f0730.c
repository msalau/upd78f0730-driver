/*
 * Renesas Electronics uPD78F0730 USB to serial converter driver
 *
 * Copyright (C) 2014,2016 Maksim Salau <maksim.salau@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * Protocol of the adaptor is described in the application note U19660EJ1V0AN00
 * μPD78F0730 8-bit Single-Chip Microcontroller
 * USB-to-Serial Conversion Software
 * <http://documentation.renesas.com/doc/DocumentServer/U19660EJ1V0AN00.pdf>
 *
 * The adaptor functionality is limited to the following:
 * - data bits: 7 or 8
 * - stop bits: 1 or 2
 * - parity: even, odd or none
 * - flow control: XON/XOFF or none
 * - baudrates: 2400, 4800, 9600, 19200, 38400, 57600, 115200
 * - signals: DTS, RTS and BREAK
 * - there is an option to enable parity error character substitution,
 *   but it is not supported by this driver
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tty.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define DRIVER_DESC "Renesas uPD78F0730 USB to serial converter driver"

#define DRIVER_AUTHOR "Maksim Salau <maksim.salau@gmail.com>"

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(0x045B, 0x0212) }, /* YRPBRL78G13, YRPBRL78G14 */
	{ USB_DEVICE(0x0409, 0x0063) }, /* V850ESJX3-STICK */
	{}
};

MODULE_DEVICE_TABLE(usb, id_table);

/*
 * Private data structure type declaration.
 * Each adaptor is associated with a private structure, that holds the current
 * state of control signals (DTR, RTS and BREAK).
 */
struct upd78f0730_serial_private {
	spinlock_t	lock;	      /* spinlock for line_signals */
	u8		line_signals;
};

/* Op-codes of control commands */
#define UPD78F0730_CMD_LINE_CONTROL	0x00
#define UPD78F0730_CMD_SET_DTR_RTS	0x01
#define UPD78F0730_CMD_SET_XON_XOFF_CHR	0x02
#define UPD78F0730_CMD_OPEN_CLOSE	0x03
#define UPD78F0730_CMD_SET_ERR_CHR	0x04

/* Data sizes in UPD78F0730_CMD_LINE_CONTROL command */
#define UPD78F0730_DATA_SIZE_7_BITS	0x00
#define UPD78F0730_DATA_SIZE_8_BITS	0x01
#define UPD78F0730_DATA_SIZE_MASK	0x01

/* Stop-bit modes in UPD78F0730_CMD_LINE_CONTROL command */
#define UPD78F0730_STOP_BIT_1_BIT	0x00
#define UPD78F0730_STOP_BIT_2_BIT	0x02
#define UPD78F0730_STOP_BIT_MASK	0x02

/* Parity modes in UPD78F0730_CMD_LINE_CONTROL command */
#define UPD78F0730_PARITY_NONE	0x00
#define UPD78F0730_PARITY_EVEN	0x04
#define UPD78F0730_PARITY_ODD	0x08
#define UPD78F0730_PARITY_MASK	0x0C

/* Flow control modes in UPD78F0730_CMD_LINE_CONTROL command */
#define UPD78F0730_FLOW_CONTROL_NONE	0x00
#define UPD78F0730_FLOW_CONTROL_HW	0x10
#define UPD78F0730_FLOW_CONTROL_SW	0x20
#define UPD78F0730_FLOW_CONTROL_MASK	0x30

/* Control signal bits in UPD78F0730_CMD_SET_DTR_RTS command */
#define UPD78F0730_RTS		0x01
#define UPD78F0730_DTR		0x02
#define UPD78F0730_BREAK	0x04

/* Port modes in UPD78F0730_CMD_OPEN_CLOSE command */
#define UPD78F0730_PORT_CLOSE	0x00
#define UPD78F0730_PORT_OPEN	0x01

/* Error character substitution modes in UPD78F0730_CMD_SET_ERR_CHR command */
#define UPD78F0730_ERR_CHR_DISABLED	0x00
#define UPD78F0730_ERR_CHR_ENABLED	0x01

/*
 * Declaration of command structures
 */

/* UPD78F0730_CMD_LINE_CONTROL command */
struct line_control {
	u8	opcode;
	__le32	baud_rate;
	u8	params;
} __packed;

/* UPD78F0730_CMD_SET_DTR_RTS command */
struct set_dtr_rts {
	u8 opcode;
	u8 params;
};

/* UPD78F0730_CMD_SET_XON_OFF_CHR command */
struct set_xon_xoff_chr {
	u8 opcode;
	u8 xon;
	u8 xoff;
};

/* UPD78F0730_CMD_OPEN_CLOSE command */
struct open_close {
	u8 opcode;
	u8 state;
};

/* UPD78F0730_CMD_SET_ERR_CHR command */
struct set_err_chr {
	u8 opcode;
	u8 state;
	u8 err_char;
};

static int upd78f0730_send_ctl(struct usb_serial_port *port,
			void *data, int size)
{
	int res;
	struct device *dev = &port->dev;
	struct usb_device *usbdev = port->serial->dev;
	void *buf;

	if (!size)
		return 0;
	if (!data) {
		dev_err(dev, "%s - invalid arguments\n", __func__);
		return -EINVAL;
	}

	buf = kmemdup(data, size, GFP_KERNEL);

	if (!buf)
		return -ENOMEM;

	res = usb_control_msg(usbdev, usb_sndctrlpipe(usbdev, 0), 0x00,
			USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
			0x0000, 0x0000, buf, size, USB_CTRL_SET_TIMEOUT);

	kfree(buf);

	if (res < 0 || res != size) {
		dev_err(dev,
			"%s - send failed: opcode=%02x, size=%d, res=%d\n",
			__func__, *(u8 *)data, size, res);
		/* The maximum expected length of a transfer is 6 bytes */
		return -EIO;
	}

	return 0;
}

static int upd78f0730_attach(struct usb_serial *serial)
{
	struct upd78f0730_serial_private *private;
	struct device *dev = &serial->dev->dev;

	private = kzalloc(sizeof(*private), GFP_KERNEL);
	if (!private) {
		dev_err(dev, "%s - unable to allocate memory\n",
			__func__);
		return -ENOMEM;
	}
	spin_lock_init(&private->lock);
	private->line_signals = 0;
	usb_set_serial_data(serial, private);
	return 0;
}

static void upd78f0730_release(struct usb_serial *serial)
{
	struct upd78f0730_serial_private *private;

	private = usb_get_serial_data(serial);
	kfree(private);
}

static void upd78f0730_set_termios(struct tty_struct *tty,
				struct usb_serial_port *port,
				struct ktermios *old_termios)
{
	struct device *dev = &port->dev;
	struct line_control request_control;
	struct set_xon_xoff_chr request_xchr;
	tcflag_t cflag = tty->termios.c_cflag;
	speed_t baud_rate = tty_get_baud_rate(tty);

	request_control.opcode = UPD78F0730_CMD_LINE_CONTROL;
	request_control.baud_rate = cpu_to_le32(baud_rate);
	dev_dbg(dev, "%s - baud rate = %d\n", __func__, baud_rate);

	request_xchr.opcode = UPD78F0730_CMD_SET_XON_XOFF_CHR;
	request_xchr.xon = START_CHAR(tty);
	request_xchr.xoff = STOP_CHAR(tty);
	dev_dbg(dev, "%s - XON = %02X, XOFF = %02X\n", __func__,
		request_xchr.xon, request_xchr.xoff);

	request_control.params = 0;
	switch (cflag & CSIZE) {
	case CS7:
		request_control.params |= UPD78F0730_DATA_SIZE_7_BITS;
		dev_dbg(dev, "%s - 7 data bits\n", __func__);
		break;
	case CS8:
		request_control.params |= UPD78F0730_DATA_SIZE_8_BITS;
		dev_dbg(dev, "%s - 8 data bits\n", __func__);
		break;
	default:
		request_control.params |= UPD78F0730_DATA_SIZE_8_BITS;
		dev_err(dev, "%s - data size is not supported, using 8 bits\n",
			__func__);
		break;
	}
	if (cflag & PARENB) {
		if (cflag & PARODD) {
			request_control.params |= UPD78F0730_PARITY_ODD;
			dev_dbg(dev, "%s - odd parity\n", __func__);
		} else {
			request_control.params |= UPD78F0730_PARITY_EVEN;
			dev_dbg(dev, "%s - even parity\n", __func__);
		}
	} else {
		request_control.params |= UPD78F0730_PARITY_NONE;
		dev_dbg(dev, "%s - no parity\n", __func__);
	}
	if (cflag & CSTOPB) {
		request_control.params |= UPD78F0730_STOP_BIT_2_BIT;
		dev_dbg(dev, "%s - 2 stop bits\n", __func__);
	} else {
		request_control.params |= UPD78F0730_STOP_BIT_1_BIT;
		dev_dbg(dev, "%s - 1 stop bit\n", __func__);
	}
	if (cflag & CRTSCTS) {
		dev_err(dev, "%s - hardware flow control is not supported\n",
			__func__);
	}
	if (I_IXOFF(tty) || I_IXON(tty)) {
		request_control.params |= UPD78F0730_FLOW_CONTROL_SW;
		dev_dbg(dev, "%s - software flow control\n", __func__);
	} else {
		request_control.params |= UPD78F0730_FLOW_CONTROL_NONE;
		dev_dbg(dev, "%s - no flow control\n", __func__);
	}

	upd78f0730_send_ctl(port, &request_control, sizeof(request_control));
	upd78f0730_send_ctl(port, &request_xchr, sizeof(request_xchr));
}

static int upd78f0730_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	int res;
	unsigned long flags;
	struct upd78f0730_serial_private *private;
	struct open_close request_open = {
		.opcode = UPD78F0730_CMD_OPEN_CLOSE,
		.state = UPD78F0730_PORT_OPEN
	};
	struct set_dtr_rts request_set_dtr_rts = {
		.opcode = UPD78F0730_CMD_SET_DTR_RTS,
		.params = 0
	};
	struct set_err_chr request_set_err_chr = {
		.opcode = UPD78F0730_CMD_SET_ERR_CHR,
		.state = UPD78F0730_ERR_CHR_DISABLED,
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
	} while (request->data);

	upd78f0730_set_termios(tty, port, NULL);

	return usb_serial_generic_open(tty, port);
}

static void upd78f0730_close(struct usb_serial_port *port)
{
	struct open_close request_close = {
		.opcode = UPD78F0730_CMD_OPEN_CLOSE,
		.state = UPD78F0730_PORT_CLOSE
	};

	usb_serial_generic_close(port);
	upd78f0730_send_ctl(port, &request_close, sizeof(request_close));
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

	res = ((signals & UPD78F0730_DTR) ? TIOCM_DTR : 0)
		| ((signals & UPD78F0730_RTS) ? TIOCM_RTS : 0);

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
	struct set_dtr_rts request = { .opcode = UPD78F0730_CMD_SET_DTR_RTS };

	private = usb_get_serial_data(port->serial);

	spin_lock_irqsave(&private->lock, flags);
	if (set & TIOCM_DTR) {
		private->line_signals |= UPD78F0730_DTR;
		dev_dbg(dev, "%s - set DTR\n", __func__);
	}
	if (set & TIOCM_RTS) {
		private->line_signals |= UPD78F0730_RTS;
		dev_dbg(dev, "%s - set RTS\n", __func__);
	}
	if (clear & TIOCM_DTR) {
		private->line_signals &= ~UPD78F0730_DTR;
		dev_dbg(dev, "%s - reset DTR\n", __func__);
	}
	if (clear & TIOCM_RTS) {
		private->line_signals &= ~UPD78F0730_RTS;
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
	struct set_dtr_rts request = { .opcode = UPD78F0730_CMD_SET_DTR_RTS };

	private = usb_get_serial_data(port->serial);

	spin_lock_irqsave(&private->lock, flags);
	if (on) {
		private->line_signals |= UPD78F0730_DTR;
		private->line_signals |= UPD78F0730_RTS;
		dev_dbg(dev, "%s - set DTR and RTS\n", __func__);
	} else {
		private->line_signals &= ~UPD78F0730_DTR;
		private->line_signals &= ~UPD78F0730_RTS;
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
	struct set_dtr_rts request = { .opcode = UPD78F0730_CMD_SET_DTR_RTS };

	private = usb_get_serial_data(port->serial);

	spin_lock_irqsave(&private->lock, flags);
	if (break_state) {
		private->line_signals |= UPD78F0730_BREAK;
		dev_dbg(dev, "%s - set BREAK\n", __func__);
	} else {
		private->line_signals &= ~UPD78F0730_BREAK;
		dev_dbg(dev, "%s - reset BREAK\n", __func__);
	}
	request.params = private->line_signals;
	spin_unlock_irqrestore(&private->lock, flags);

	upd78f0730_send_ctl(port, &request, sizeof(request));
}

static struct usb_serial_driver upd78f0730_device = {
	.driver	 = {
		.owner	= THIS_MODULE,
		.name	= "upd78f0730",
	},
	.id_table	= id_table,
	.num_ports	= 1,
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

module_usb_serial_driver(serial_drivers, id_table);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
