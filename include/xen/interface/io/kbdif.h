/*
 * kbdif.h -- Xen virtual keyboard/mouse/multi-touch
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Copyright (C) 2005 Anthony Liguori <aliguori@us.ibm.com>
 * Copyright (C) 2006 Red Hat, Inc., Markus Armbruster <armbru@redhat.com>
 */

#ifndef __XEN_PUBLIC_IO_KBDIF_H__
#define __XEN_PUBLIC_IO_KBDIF_H__

/* In events (backend -> frontend) */

/*
 * Frontends should ignore unknown in events.
 */

/* Pointer movement event */
#define XENKBD_TYPE_MOTION  1
/* Event type 2 currently not used */
/* Key event (includes pointer buttons) */
#define XENKBD_TYPE_KEY     3
/*
 * Pointer position event
 * Capable backend sets feature-abs-pointer in xenstore.
 * Frontend requests ot instead of XENKBD_TYPE_MOTION by setting
 * request-abs-update in xenstore.
 */
#define XENKBD_TYPE_POS     4

/*
 * Multi-touch event
 * Capable backend sets feature-multi-touch in xenstore.
 * Frontend requests feature by setting request-multi-touch in xenstore.
 * Multi-touch input devices use dedicated event rings and are
 * configured via xenstore properties under mt-%d folder(s), %d being
 * a sequential number of the virtual input device:
 * Backend configures:
 *   o num-contacts - number of simultaneous touches supported
 *   o width - width of the touch area in pixels, 32-bit signed integer
 *   o height - height of the touch area in pixels, 32-bit signed integer
 * Frontend publishes:
 *   o page-ref - unique reference of this connection
 *   o page-gref - granted reference of the event ring shared page
 *   o event-channel - allocated event channel's port
 */
#define XENKBD_TYPE_MTOUCH  5

struct xenkbd_motion {
	uint8_t type;		/* XENKBD_TYPE_MOTION */
	int32_t rel_x;		/* relative X motion */
	int32_t rel_y;		/* relative Y motion */
	int32_t rel_z;		/* relative Z motion (wheel) */
};

struct xenkbd_key {
	uint8_t type;		/* XENKBD_TYPE_KEY */
	uint8_t pressed;	/* 1 if pressed; 0 otherwise */
	uint32_t keycode;	/* KEY_* from linux/input.h */
};

struct xenkbd_position {
	uint8_t type;		/* XENKBD_TYPE_POS */
	int32_t abs_x;		/* absolute X position (in FB pixels) */
	int32_t abs_y;		/* absolute Y position (in FB pixels) */
	int32_t rel_z;		/* relative Z motion (wheel) */
};

/* Sent when a new touch is made: touch is assigned a unique contact
 * ID, sent with this and consequent events related to this touch.
 */
#define XENKBD_MT_EV_DOWN	0
/* Touch point has been released */
#define XENKBD_MT_EV_UP		1
/* Touch point has changed its coordinate(s) */
#define XENKBD_MT_EV_MOTION	2
/* Input synchronization event: shows end of a set of events
 * which logically belong together.
 */
#define XENKBD_MT_EV_SYN	3
/* Touch point has changed its shape. Shape is approximated by an ellipse
 * through the major and minor axis lengths: major is the longer diameter
 * of the ellipse and minor is the shorter one. Center of the ellipse is
 * reported via XENKBD_MT_EV_DOWN/XENKBD_MT_EV_MOTION events.
 */
#define XENKBD_MT_EV_SHAPE	4
/* Touch point's shape has changed its orientation: calculated as a clockwise
 * angle between the major axis of the ellipse and positive Y axis in degrees,
 * [-180; +180].
 */
#define XENKBD_MT_EV_ORIENT	5

struct xenkbd_mtouch {
	uint8_t type;			/* XENKBD_TYPE_MTOUCH */
	uint8_t event_type;		/* XENKBD_MT_EV_??? */
	/* Touch interactions can consist of one or more contacts.
	 * For each contact, a series of events is generated, starting
	 * with a down event, followed by zero or more motion events,
	 * and ending with an up event. Events relating to the same
	 * contact point can be identified by the ID of the sequence: contact ID.
	 * Contact ID may be reused after XENKBD_MT_EV_UP event and
	 * is in the [0; num-contacts - 1] range.
	 */
	uint8_t contact_id;
	uint8_t reserved[5];		/* reserved for the future use */
	union {
		/* XENKBD_MT_EV_DOWN/XENKBD_MT_EV_MOTION */
		struct {
			uint32_t abs_x;	/* absolute X position, pixels */
			uint32_t abs_y;	/* absolute Y position, pixels */
		} pos;
		/* XENKBD_MT_EV_SHAPE */
		struct {
			uint32_t major;	/* length of the major axis, pixels */
			uint32_t minor;	/* length of the minor axis, pixels */
		} shape;
		/* XENKBD_MT_EV_ORIENT */
		uint16_t orientation;	/* clockwise angle of the major axis */
	} u;
};

#define XENKBD_IN_EVENT_SIZE 40

union xenkbd_in_event {
	uint8_t type;
	struct xenkbd_motion motion;
	struct xenkbd_key key;
	struct xenkbd_position pos;
	struct xenkbd_mtouch mtouch;
	char pad[XENKBD_IN_EVENT_SIZE];
};

/* Out events (frontend -> backend) */

/*
 * Out events may be sent only when requested by backend, and receipt
 * of an unknown out event is an error.
 * No out events currently defined.
 */

#define XENKBD_OUT_EVENT_SIZE 40

union xenkbd_out_event {
	uint8_t type;
	char pad[XENKBD_OUT_EVENT_SIZE];
};

/* shared page */

#define XENKBD_IN_RING_SIZE 2048
#define XENKBD_IN_RING_LEN (XENKBD_IN_RING_SIZE / XENKBD_IN_EVENT_SIZE)
#define XENKBD_IN_RING_OFFS 1024
#define XENKBD_IN_RING(page) \
	((union xenkbd_in_event *)((char *)(page) + XENKBD_IN_RING_OFFS))
#define XENKBD_IN_RING_REF(page, idx) \
	(XENKBD_IN_RING((page))[(idx) % XENKBD_IN_RING_LEN])

#define XENKBD_OUT_RING_SIZE 1024
#define XENKBD_OUT_RING_LEN (XENKBD_OUT_RING_SIZE / XENKBD_OUT_EVENT_SIZE)
#define XENKBD_OUT_RING_OFFS (XENKBD_IN_RING_OFFS + XENKBD_IN_RING_SIZE)
#define XENKBD_OUT_RING(page) \
	((union xenkbd_out_event *)((char *)(page) + XENKBD_OUT_RING_OFFS))
#define XENKBD_OUT_RING_REF(page, idx) \
	(XENKBD_OUT_RING((page))[(idx) % XENKBD_OUT_RING_LEN])

struct xenkbd_page {
	uint32_t in_cons, in_prod;
	uint32_t out_cons, out_prod;
};

#endif
