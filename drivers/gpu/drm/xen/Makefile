#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

xen_drmfront-objs := xen_drm_front.o xen_drm.o xen_drm_kms.o xen_drm_crtc.o \
		  xen_drm_timer.o

ifneq ($(CONFIG_DRM_XENFRONT_CMA),y)
	xen_drmfront-objs += xen_drm_gem.o
endif

obj-$(CONFIG_DRM_XENFRONT) += xen_drmfront.o

obj-$(CONFIG_DRM_XENZCOPY) += xen_drmzcopy.o
