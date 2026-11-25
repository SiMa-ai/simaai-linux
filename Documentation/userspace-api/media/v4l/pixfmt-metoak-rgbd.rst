.. SPDX-License-Identifier: GFDL-1.1-no-invariants-or-later

.. _V4L2-PIX-FMT-METOAK-RGBD:

*********************************
V4L2_PIX_FMT_METOAK_RGBD ('MORD')
*********************************

12-bit disparity packed with a reference YUV image


Description
===========

Output of the Metoak S315 stereo module: 24 bits per pixel carrying a
12-bit disparity value and a reference image. It is neither a picture nor
a plain depth map, and the vendor forbids passing it through an ISP or any
image processing stage.

The module sends it as RAW8 at three samples per pixel, so a 640-pixel
line is 1920 bytes on the wire. Subdevice and video node report the image
geometry, 640x360; ``MEDIA_BUS_FMT_METOAK_RGBD_3X8`` tells the receiver
the sample count.

**Byte Order.** B, G, R per pixel, one byte each.

.. flat-table::
    :header-rows:  1

    * - Source
      - Content
    * - ``G[3:0] << 8 | B``
      - 12-bit disparity, 4 fractional bits (1/16 pixel)
    * - ``G[7:4]``
      - Reference image UV, 2x2 subsampled, alternating by row and column
        parity. Which parity carries U is unspecified by the vendor.
    * - ``R``
      - Reference image Y, 8 bit

**Metadata.** Row 0 carries metadata instead of disparity, as 12-bit words
packed like disparity, word *i* in pixel *i*: words 0 to 3 are the magic
``0xACE``, ``0xBED``, ``0xCAB``, ``0xDEF``; words 16 to 21 carry the
baseline, the focal length and ``BF``, baseline times focal length as 20.12
fixed point in millimetre-pixels; words 23 to 25 are a 36-bit frame counter,
most significant word first. Other words are unspecified. Rows 1 to 3 are
unusable: shift disparity up by four rows and zero-pad the last four to
align it with the image. The vendor guide numbers the same words D1 to
D640, one higher.

**Depth.** ``depth_mm = (BF / 4096) / (disparity / 16)``, with ``BF`` from
the frame's own metadata. The specified sensing range is 0.2 m to 10 m at a
60 mm baseline.

``<linux/media/simaai/metoak_rgbd.h>`` defines the layout and accessors.

The payload was earlier exposed as ``V4L2_PIX_FMT_SBGGR8`` at 1920x360. No
alias is kept: ``VIDIOC_S_FMT`` adjusts such a request to this format at
640x360, so applications must check what it returns.
