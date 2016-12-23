#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

xen-drmfront-y := xen-drm-front.o xen-drm.o xen-drm-kms.o xen-drm-crtc.o \
		  xen-drm-timer.o

ifneq ($(CONFIG_DRM_XENFRONT_CMA),y)
	xen-drmfront-y += xen-drm-gem.o
endif

obj-$(CONFIG_DRM_XENFRONT) := xen-drmfront.o
