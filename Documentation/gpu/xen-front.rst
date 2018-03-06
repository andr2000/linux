====================================
Xen para-virtualized frontend driver
====================================
This frontend driver implements Xen para-virtualized display
according to the display protocol described at
include/xen/interface/io/displif.h

Driver modes of operation in terms of display buffers used
==========================================================

Depending on the requirements for the para-virtualized environment, namely
requirements dictated by the accompanying DRM/(v)GPU drivers running in both
host and guest environments, number of operating modes of para-virtualized
display driver are supported:

- display buffers can be allocated by either frontend driver or backend
- display buffers can be allocated to be contiguous in memory or not

Note! Frontend driver itself has no dependency on contiguous memory for
its operation.

Buffers allocated by the frontend driver
----------------------------------------

The below modes of operation are configured at compile-time via
frontend driver's kernel configuration:

With GEM CMA helpers
~~~~~~~~~~~~~~~~~~~~
 This use-case is useful when used with accompanying DRM/vGPU driver in
 guest domain which was designed to only work with contiguous buffers,
 e.g. DRM driver based on GEM CMA helpers: such drivers can only import
 contiguous PRIME buffers, thus requiring frontend driver to provide
 such. In order to implement this mode of operation para-virtualized
 frontend driver can be configured to use GEM CMA helpers.

Without GEM CMA helpers
~~~~~~~~~~~~~~~~~~~~~~~
 If accompanying drivers can cope with non-contiguous memory then, to
 lower pressure on CMA subsystem of the kernel, driver can allocate
 buffers from system memory.

 Note! If used with accompanying DRM/(v)GPU drivers this mode of operation
 may require IOMMU support on the platform, so accompanying DRM/vGPU
 hardware can still reach display buffer memory while importing PRIME
 buffers from the frontend driver.

Buffers allocated by the backend
--------------------------------

This mode of operation is run-time configured via guest domain configuration
through XenStore entries.

For systems which do not provide IOMMU support, but having specific
requirements for display buffers it is possible to allocate such buffers
at backend side and share those with the frontend.
For example, if host domain is 1:1 mapped and has DRM/GPU hardware expecting
physically contiguous memory, this allows implementing zero-copying
use-cases.

Note, while using this scenario the following should be considered:

#. If guest domain dies then pages/grants received from the backend
   cannot be claimed back

#. Misbehaving guest may send too many requests to the
   backend exhausting its grant references and memory
   (consider this from security POV).

Driver limitations
==================

#. Only primary plane without additional properties is supported.

#. Only one video mode per connector supported which is configured via XenStore.

#. All CRTCs operate at fixed frequency of 60Hz.
