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
 * <https://www.renesas.com/en-eu/doc/DocumentServer/026/U19660EJ1V0AN00.pdf>
 *
 * The adaptor functionality is limited to the following:
 * - data bits: 7 or 8
 * - stop bits: 1 or 2
 * - parity: even, odd or none
 * - flow control: hardware or none
 * - baud rates: 2400, 4800, 9600, 19200, 38400, 57600, 115200
 * - signals: DTS, RTS and BREAK
 * - there is an option to enable parity error character substitution,
 *   but is not supported by the driver
 * - XON/XOFF flow control is described in the document,
 *   but is not supported by the driver
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
	struct mutex	lock;		/* mutex to protect line_signals */
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

static int upd78f0730_port_probe(struct usb_serial_port *port)
{
	struct upd78f0730_serial_private *private;

	private = kzalloc(sizeof(*private), GFP_KERNEL);
	if (!private)
		return -ENOMEM;
	mutex_init(&private->lock);
	private->line_signals = 0;
	usb_set_serial_port_data(port, private);
	return 0;
}

static int upd78f0730_port_remove(struct usb_serial_port *port)
{
	struct upd78f0730_serial_private *private;

	private = usb_get_serial_port_data(port);
	mutex_destroy(&private->lock);
	kfree(private);
	return 0;
}

static int upd78f0730_tiocmget(struct tty_struct *tty)
{
	int res = 0;
	int signals;
	struct device *dev = tty->dev;
	struct upd78f0730_serial_private *private;
	struct usb_serial_port *port = tty->driver_data;

	private = usb_get_serial_port_data(port);

	mutex_lock(&private->lock);
	signals = private->line_signals;
	mutex_unlock(&private->lock);

	res = ((signals & UPD78F0730_DTR) ? TIOCM_DTR : 0) |
		((signals & UPD78F0730_RTS) ? TIOCM_RTS : 0);

	dev_dbg(dev, "%s - res = %x\n", __func__, res);

	return res;
}

static int upd78f0730_tiocmset(struct tty_struct *tty,
			unsigned int set, unsigned int clear)
{
	int res;
	struct device *dev = tty->dev;
	struct upd78f0730_serial_private *private;
	struct usb_serial_port *port = tty->driver_data;
	struct set_dtr_rts request = { .opcode = UPD78F0730_CMD_SET_DTR_RTS };

	private = usb_get_serial_port_data(port);

	mutex_lock(&private->lock);
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

	res = upd78f0730_send_ctl(port, &request, sizeof(request));
	mutex_unlock(&private->lock);

	return res;
}

static void upd78f0730_break_ctl(struct tty_struct *tty, int break_state)
{
	struct device *dev = tty->dev;
	struct upd78f0730_serial_private *private;
	struct usb_serial_port *port = tty->driver_data;
	struct set_dtr_rts request = { .opcode = UPD78F0730_CMD_SET_DTR_RTS };

	private = usb_get_serial_port_data(port);

	mutex_lock(&private->lock);
	if (break_state) {
		private->line_signals |= UPD78F0730_BREAK;
		dev_dbg(dev, "%s - set BREAK\n", __func__);
	} else {
		private->line_signals &= ~UPD78F0730_BREAK;
		dev_dbg(dev, "%s - reset BREAK\n", __func__);
	}
	request.params = private->line_signals;

	upd78f0730_send_ctl(port, &request, sizeof(request));
	mutex_unlock(&private->lock);
}

static void upd78f0730_dtr_rts(struct usb_serial_port *port, int on)
{
	struct tty_struct *tty = port->port.tty;
	unsigned int set = 0;
	unsigned int clear = 0;

	if (on) {
		set = TIOCM_DTR | TIOCM_RTS;
	} else {
		clear = TIOCM_DTR | TIOCM_RTS;
	}
	upd78f0730_tiocmset(tty, set, clear);
}

static speed_t upd78f0730_get_baud_rate(struct tty_struct *tty)
{
	int i;
	const tcflag_t baud_rate = C_BAUD(tty);
	const tcflag_t supported[] = {B2400, B4800, B9600,
					B19200, B38400, B57600, B115200};

	for (i = ARRAY_SIZE(supported) - 1; i >= 0; i--) {
		if (baud_rate == supported[i])
			return tty_get_baud_rate(tty);
	}

	/* If the baud rate is not supported, switch to the default baud rate */
	tty->termios.c_cflag &= ~CBAUD;
	tty->termios.c_cflag |= B9600;

	return tty_get_baud_rate(tty);
}

static void upd78f0730_set_termios(struct tty_struct *tty,
				struct usb_serial_port *port,
				struct ktermios *old_termios)
{
	struct device *dev = &port->dev;
	struct line_control request_control;
	speed_t baud_rate;

	if (old_termios && !tty_termios_hw_change(&tty->termios, old_termios))
		return;

	if (C_BAUD(tty) == B0)
		upd78f0730_dtr_rts(port, 0);

	baud_rate = upd78f0730_get_baud_rate(tty);
	request_control.opcode = UPD78F0730_CMD_LINE_CONTROL;
	request_control.baud_rate = cpu_to_le32(baud_rate);
	request_control.params = 0;
	dev_dbg(dev, "%s - baud rate = %d\n", __func__, baud_rate);

	switch (C_CSIZE(tty)) {
	case CS7:
		request_control.params |= UPD78F0730_DATA_SIZE_7_BITS;
		dev_dbg(dev, "%s - 7 data bits\n", __func__);
		break;
	default:
		tty->termios.c_cflag &= ~CSIZE;
		tty->termios.c_cflag |= CS8;
		dev_warn(dev, "%s - data size is not supported, using 8 bits\n",
			__func__);
		/* fall through */
	case CS8:
		request_control.params |= UPD78F0730_DATA_SIZE_8_BITS;
		dev_dbg(dev, "%s - 8 data bits\n", __func__);
		break;
	}

	if (C_PARENB(tty)) {
		if (C_PARODD(tty)) {
			request_control.params |= UPD78F0730_PARITY_ODD;
			dev_dbg(dev, "%s - odd parity\n", __func__);
		} else {
			request_control.params |= UPD78F0730_PARITY_EVEN;
			dev_dbg(dev, "%s - even parity\n", __func__);
		}

		if (C_CMSPAR(tty)) {
			tty->termios.c_cflag &= ~CMSPAR;
			dev_warn(dev, "%s - MARK/SPACE parity is not supported\n",
				__func__);
		}
	} else {
		request_control.params |= UPD78F0730_PARITY_NONE;
		dev_dbg(dev, "%s - no parity\n", __func__);
	}

	if (C_CSTOPB(tty)) {
		request_control.params |= UPD78F0730_STOP_BIT_2_BIT;
		dev_dbg(dev, "%s - 2 stop bits\n", __func__);
	} else {
		request_control.params |= UPD78F0730_STOP_BIT_1_BIT;
		dev_dbg(dev, "%s - 1 stop bit\n", __func__);
	}

	if (C_CRTSCTS(tty)) {
		request_control.params |= UPD78F0730_FLOW_CONTROL_HW;
		dev_dbg(dev, "%s - hardware flow control\n", __func__);
	} else {
		request_control.params |= UPD78F0730_FLOW_CONTROL_NONE;
		dev_dbg(dev, "%s - no flow control\n", __func__);
	}

	if (I_IXOFF(tty) || I_IXON(tty)) {
		tty->termios.c_iflag &= ~(IXOFF | IXON);
		dev_warn(dev, "%s - software flow control is not supported\n",
			__func__);
	}

	upd78f0730_send_ctl(port, &request_control, sizeof(request_control));
}

static int upd78f0730_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	int res;
	struct open_close request_open = {
		.opcode = UPD78F0730_CMD_OPEN_CLOSE,
		.state = UPD78F0730_PORT_OPEN
	};

	res = upd78f0730_send_ctl(port, &request_open, sizeof(request_open));
	if (res)
		return res;

	if (tty)
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

static struct usb_serial_driver upd78f0730_device = {
	.driver	 = {
		.owner	= THIS_MODULE,
		.name	= "upd78f0730",
	},
	.id_table	= id_table,
	.num_ports	= 1,
	.port_probe	= upd78f0730_port_probe,
	.port_remove	= upd78f0730_port_remove,
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
