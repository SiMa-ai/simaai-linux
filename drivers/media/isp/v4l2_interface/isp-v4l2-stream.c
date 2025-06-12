/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2021 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#include <asm/div64.h>
#include <linux/device.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/simaai-stu.h>

#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "acamera_command_api.h"

#include "acamera_logger.h"

#include "fw-interface.h"
#include "isp-v4l2-common.h"
#include "isp-v4l2.h"

#include "isp-v4l2-stream.h"

#if defined( ISP_HAS_METADATA_FSM )
#include "metadata_api.h"
#endif

/* metadata size */
#if defined( ISP_HAS_METADATA_FSM )
#define ISP_V4L2_METADATA_SIZE sizeof( isp_metadata_t )
#else
#define ISP_V4L2_METADATA_SIZE 4096
#endif

/* max size */
#define ISP_V4L2_MAX_WIDTH 4096
#define ISP_V4L2_MAX_HEIGHT 3072

extern struct simaai_stu *stu;
/**
 * @brief Struct describing stream supported format
 * 
 */
typedef struct _isp_v4l2_fmt {
    const char *description; // Friendly name
    uint32_t pixelformat;    // Format fourcc
    uint8_t data_width;      // Pixel datawidth
    uint8_t num_planes;      // Number of planes
    uint8_t is_yuv;          // YUV flag
	uint32_t mbus_code;		 // mbus code	
} isp_v4l2_fmt_t;

/**
 * @brief Struct to describe stream supported formats
 * 
 */
typedef struct isp_v4l2_stream_fmt_list_t {
    isp_v4l2_fmt_t *formats; // Pointer to an array of stream formats
    uint32_t num_formats;    // Number of stream formats
} isp_v4l2_stream_fmt_list_t;

// clang-format off
static isp_v4l2_stream_fmt_list_t isp_v4l2_stream_supported_formats[V4L2_STREAM_TYPE_MAX] = {

    [V4L2_STREAM_TYPE_RAW] = {
        .formats = (isp_v4l2_fmt_t[]) {
            {
                .description = "RAW 8",
                .pixelformat = V4L2_PIX_FMT_SRGGB8,
                .data_width = 8,
                .num_planes = 1,
                .is_yuv = 0,
				.mbus_code = MEDIA_BUS_FMT_SRGGB8_1X8,
            },
            {
                .description = "RAW 12",
                .pixelformat = V4L2_PIX_FMT_SRGGB12,
                .data_width = 12,
                .num_planes = 1,
                .is_yuv = 0,
				.mbus_code = MEDIA_BUS_FMT_SRGGB12_1X12,
            },
            {
                .description = "RAW 16",
                .pixelformat = V4L2_PIX_FMT_SRGGB16,
                .data_width = 16,
                .num_planes = 1,
                .is_yuv = 0,
				.mbus_code = MEDIA_BUS_FMT_SRGGB16_1X16,
            }
        },
        .num_formats = 3
    },
    [V4L2_STREAM_TYPE_OUT] = {
        .formats = (isp_v4l2_fmt_t[]) {
            {
                .description = "BGRA32",
                .pixelformat = V4L2_PIX_FMT_ABGR32,
                .data_width = 32,
                .num_planes = 1,
                .is_yuv = 0,
				.mbus_code = 0, //TODO : Add correct code here
            },
            {
                .description = "RGB24",
                .pixelformat = V4L2_PIX_FMT_RGB24,
                .data_width = 24,
                .num_planes = 1,
                .is_yuv = 0,
				.mbus_code = MEDIA_BUS_FMT_RGB888_1X24,
            },
            {
                .description = "NV12",
                .pixelformat = V4L2_PIX_FMT_NV12,
                .data_width = 8,
                .num_planes = 2,
                .is_yuv = 1,
				.mbus_code = 0, //TODO : Add correct code here.
            }
        },
        .num_formats = 3
    },
    [V4L2_STREAM_TYPE_META] = {
        .formats = (isp_v4l2_fmt_t[]) {
            {
                .description = "META",
                .pixelformat = ISP_V4L2_PIX_FMT_META,
                .data_width = 8,
                .num_planes = 1,
                .is_yuv = 0,
				.mbus_code = 0, //TODO : Add correct code here
            }
        },
        .num_formats = 1
    }
};
// clang-format on

/**
 * @brief Helper function to check whether stream direction is valid
 * 
 * @param stream_direction Stream direction to be checked
 * @return true If stream direction is within allowed range
 * @return false If stream direction is outside of allowed range
 */

static bool isp_v4l2_stream_check_direction( isp_v4l2_stream_direction_t stream_direction )
{
    return ( ( stream_direction >= 0 ) && ( stream_direction < V4L2_STREAM_DIRECTION_MAX ) );
}

/**
 * @brief Helper function to return stream direction string
 * 
 * @param stream_direction Stream direction
 * @return const char* Returns pointer to the stream direction on success, "n/a" otherwise 
 */
static const char *isp_v4l2_stream_get_direction_string( isp_v4l2_stream_direction_t stream_direction )
{
    static const char *stream_direction_names[] = {
        [V4L2_STREAM_DIRECTION_CAP] = "cap",
        [V4L2_STREAM_DIRECTION_OUT] = "out",
        [V4L2_STREAM_DIRECTION_MAX] = "n/a"};

    switch ( stream_direction ) {
    case V4L2_STREAM_DIRECTION_CAP: // fallthrough
    case V4L2_STREAM_DIRECTION_OUT: // fallthrough
        return stream_direction_names[stream_direction];
    default:
        return stream_direction_names[V4L2_STREAM_DIRECTION_MAX];
    }
}

/**
 * @brief Helper function to check whether stream type is valid
 * 
 * @param stream_type Stream type to be checked
 * @return true If stream type is within allowed range
 * @return false If stream type is outside of allowed range
 */
static bool isp_v4l2_stream_check_type( int stream_type )
{
    return ( ( stream_type >= 0 ) && ( stream_type < V4L2_STREAM_TYPE_MAX ) );
}

/**
 * @brief Helper function to get effective stream type based on current stream type and stream direction
 * 
 * @param stream_type Current stream type
 * @param stream_direction Stream direction
 * @return isp_v4l2_stream_type_t Effective stream type
 * 
 * @details For simple capture streams it is one-to-one mapping.
 *          For memory-to-memory streams effective stream type depends on stream direction
 */
static isp_v4l2_stream_type_t isp_v4l2_stream_get_effective_stream_type( isp_v4l2_stream_type_t stream_type, isp_v4l2_stream_direction_t stream_direction )
{
    isp_v4l2_stream_type_t effective_stream_type = V4L2_STREAM_TYPE_MAX;

    if ( isp_v4l2_stream_check_type( stream_type ) && ( isp_v4l2_stream_check_direction( stream_direction ) ) ) {
        if ( stream_type == V4L2_STREAM_TYPE_M2M ) {
            if ( stream_direction == V4L2_STREAM_DIRECTION_CAP ) {
                effective_stream_type = V4L2_STREAM_TYPE_OUT;
            } else {
                effective_stream_type = V4L2_STREAM_TYPE_RAW;
            }
        } else {
            effective_stream_type = stream_type;
        }
    }

    return effective_stream_type;
}

/**
 * @brief Helper function to get string representaition of fourcc pixelformat
 * 
 * @param pixelformat Pixel format in V4L2 fourcc format
 * @return const char* String representation for valid pixel format, or "N/A" otherwise
 * 
 * @warning This function is not reentrant
 */
static const char *isp_v4l2_stream_get_pixelformat_string( uint32_t pixelformat )
{
    static char pixelformat_string[5];

    if ( ( pixelformat & 0xFF ) > 0 ) {

        pixelformat_string[0] = (char)( ( pixelformat >> 0x00 ) & 0xFF );
        pixelformat_string[1] = (char)( ( pixelformat >> 0x08 ) & 0xFF );
        pixelformat_string[2] = (char)( ( pixelformat >> 0x10 ) & 0xFF );
        pixelformat_string[3] = (char)( ( pixelformat >> 0x18 ) & 0xFF );
        pixelformat_string[4] = 0;
    } else {
        pixelformat_string[0] = 'N';
        pixelformat_string[1] = '/';
        pixelformat_string[2] = 'A';
        pixelformat_string[3] = 0;
    }

    return pixelformat_string;
}

/**
 * @brief Helper function to update sensor info struct of the stream common to get latest sensor mode and submodes
 * 
 * @param pstream Pointer to isp_v4l2_stream_t structure
 */
static void isp_v4l2_stream_update_common_sensor_info( isp_v4l2_stream_t *pstream )
{
    fw_intf_isp_update_sensor_info_mode( pstream->ctx_id, &pstream->stream_common->sensor_info );
}

/**
 * @brief Helper function get default pixel format for the selected stream
 * 
 * @param pstream Pointer to isp_v4l2_stream_t structure
 * @param stream_direction Stream direction
 * @return uint32_t Returns pixel format if stream and stream type are correct, 0 otherwise
 */
static uint32_t isp_v4l2_stream_get_default_pixelformat( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction )
{
    if ( pstream == NULL || ( !isp_v4l2_stream_check_direction( stream_direction ) ) ) {
        return 0;
    }

    // Get effective stream type based on stream type and stream direction
    const isp_v4l2_stream_type_t effective_stream_type = isp_v4l2_stream_get_effective_stream_type( pstream->stream_type, stream_direction );

    switch ( effective_stream_type ) {
    case V4L2_STREAM_TYPE_RAW: {

        // Update stream common sensor info to get the current sensor mode and submode
        isp_v4l2_stream_update_common_sensor_info( pstream );

        // Check sensor data width for the current sensor preset
        const uint8_t mode = pstream->stream_common->sensor_info.cur_mode;
        const uint8_t sub_mode = pstream->stream_common->sensor_info.mode[mode].cur_sub_mode;
        const uint8_t data_width = pstream->stream_common->sensor_info.mode[mode].sub_mode[sub_mode].data_width;

        uint32_t pixelformat = 0, i;
        uint32_t nearest_suitable = isp_v4l2_stream_supported_formats[V4L2_STREAM_TYPE_RAW].num_formats;
        int last_data_width_diff = 20;

        // Look for pixelformat with required data_width, also check if there are suitable pixelformats
        for ( i = 0; i < isp_v4l2_stream_supported_formats[V4L2_STREAM_TYPE_RAW].num_formats; i++ ) {
            if ( isp_v4l2_stream_supported_formats[V4L2_STREAM_TYPE_RAW].formats[i].data_width == data_width ) {
                pixelformat = isp_v4l2_stream_supported_formats[V4L2_STREAM_TYPE_RAW].formats[i].pixelformat;
                break;
            }

            // Try to find best suitable pixel format based on data width difference
            int data_width_diff = isp_v4l2_stream_supported_formats[V4L2_STREAM_TYPE_RAW].formats[i].data_width - data_width;
            LOG( LOG_INFO, "[Stream#%d-%s] Supported format #%u. pixelformat data width: %u, diff: %d, last data diff: %d",
                 pstream->stream_type,
                 isp_v4l2_stream_get_direction_string( stream_direction ),
                 i, isp_v4l2_stream_supported_formats[V4L2_STREAM_TYPE_RAW].formats[i].data_width, data_width_diff, last_data_width_diff );

            if ( ( data_width_diff >= 0 ) && ( data_width_diff <= last_data_width_diff ) ) {
                nearest_suitable = i;
                last_data_width_diff = data_width_diff;
            }
        }

        // Check if exact pixelformat found
        if ( pixelformat == 0 ) {
            // Check if suitable pixelformat found
            if ( nearest_suitable < isp_v4l2_stream_supported_formats[V4L2_STREAM_TYPE_RAW].num_formats ) {
                pixelformat = isp_v4l2_stream_supported_formats[V4L2_STREAM_TYPE_RAW].formats[nearest_suitable].pixelformat;
                LOG( LOG_WARNING, "[Stream#%d-%s] Failed to find default pixelformat for sensor data width: %u. Substituted with: 0x%08x (%s).",
                     pstream->stream_type,
                     isp_v4l2_stream_get_direction_string( stream_direction ),
                     data_width, pixelformat, isp_v4l2_stream_get_pixelformat_string( pixelformat ) );
            } else {
                LOG( LOG_ERR, "[Stream#%d-%s] Failed to find default pixelformat for sensor data width: %u.",
                     pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), data_width );
            }
        }

        return pixelformat;
    } break;
    case V4L2_STREAM_TYPE_OUT:
        return V4L2_PIX_FMT_RGB24;
    case V4L2_STREAM_TYPE_META:
        return ISP_V4L2_PIX_FMT_META;
    default:
        return 0;
    }
}

static int isp_v4l2_stream_get_planes( struct vb2_buffer *vb,
                                       struct v4l2_pix_format_mplane *pix_mp,
                                       aframe_t *frame )
{
    uint32_t i;
	int rc = 0;
    dma_addr_t phys_addr = 0; 
	dma_addr_t bus_addr = 0;

    for ( i = 0; i < frame->num_planes; i++ ) {

        if ( frame->type == AFRAME_TYPE_META ) {
            frame->planes[i].address.low = 0;
            frame->planes[i].address.high = 0;
            frame->planes[i].virt_addr = vb2_plane_vaddr( vb, i );
        } else {
            phys_addr = vb2_dma_contig_plane_dma_addr( vb, i );
			if (stu) {
				rc = simaai_stu_get_bus_address(stu, phys_addr, &bus_addr);
				if (rc != 0) {
					LOG(LOG_CRIT, "Failed to get bus address for phys address %#llx\n", phys_addr);
					return rc;
				}
			} else {
				LOG( LOG_CRIT, "STU is not initialized\n");
				return -EINVAL;
			}

            frame->planes[i].address.low = bus_addr;
            frame->planes[i].address.high = phys_addr >> 32;
			frame->planes[i].virt_addr = vb2_plane_vaddr( vb, i );

            if ( vb->memory == VB2_MEMORY_USERPTR ) {
                // ISP driver will add ISPAS_MINUS_SYSPHY to the low 32-bit (ISP buffer base address register is 32-bit)
                // of address provided before writing it to the buffer base address register
                // so to write original user address as buffer base address we subtract it here
                frame->planes[i].address.low -= ISPAS_MINUS_SYSPHY;
            }
			LOG(LOG_INFO, "Buffer phy address : %#llx, bus addr: %#x[%#x], virt : %#llx", phys_addr,
					frame->planes[i].address.low,
					bus_addr, frame->planes[i].virt_addr);
        }

        // Check if pixel format is NV12 and adjust UV plane resolution according to subsampling
        if ( ( pix_mp->pixelformat == V4L2_PIX_FMT_NV12 ) && ( i == 1 ) ) {
            frame->planes[i].width = pix_mp->width / 2;
            frame->planes[i].height = pix_mp->height / 2;
        } else {
            frame->planes[i].width = pix_mp->width;
            frame->planes[i].height = pix_mp->height;
        }

        frame->planes[i].line_offset = pix_mp->plane_fmt[i].bytesperline;
        frame->planes[i].length = pix_mp->plane_fmt[i].sizeimage;
    }

    return 0;
}

static void isp_v4l2_stream_put_planes( struct vb2_buffer *vb, aframe_t *frame )
{
    uint32_t i;
    for ( i = 0; i < frame->num_planes; i++ ) {
        if ( frame->type == AFRAME_TYPE_META ) {
            void *addr = vb2_plane_vaddr( vb, i );
            if ( frame->planes[i].virt_addr != addr ) {
                LOG( LOG_CRIT, "VB2 virtual address mismatch for frame, context id: %u, type: %u, plane: %u (VB2: %p, frame: %p)",
                     frame->context_id, frame->type, i, addr, frame->planes[i].virt_addr );
                return;
            }
        } else {
            uint64_t addr = vb2_dma_contig_plane_dma_addr( vb, i );
            uint32_t addr_low = addr & 0xFFFFFFFF;
            uint32_t addr_high = addr >> 32;

            if ( vb->memory == VB2_MEMORY_USERPTR ) {
                // Original VB2 buffer address (its low 32-bit part) is reduced by ISPAS_MINUS_SYSPHY
                // in <isp_v4l2_stream_get_planes> before passing it to ISP driver, so do the same here before check
                addr_low -= ISPAS_MINUS_SYSPHY;
            }

            if ( ( frame->planes[i].address.low != addr_low ) || ( frame->planes[i].address.high != addr_high ) ) {
                LOG( LOG_CRIT, "VB2 DMA address mismatch for frame, context id: %u, type: %u, plane: %u (VB2: 0x%llx, frame: 0x%llx)",
                     frame->context_id, frame->type, i,
                     (uint64_t)addr_low | ( ( (uint64_t)addr_high ) << 32 ),
                     (uint64_t)frame->planes[i].address.low | ( ( (uint64_t)frame->planes[i].address.high ) << 32 ) );
                return;
            }
        }
    }
}

int isp_v4l2_stream_get_frame( const unsigned int ctx_id, const aframe_type_t type, const aframe_state_t state, aframe_t **frame )
{
    isp_v4l2_stream_t *pstream = NULL;
    isp_v4l2_buffer_t *pbuf = NULL;

    *frame = NULL;

    isp_v4l2_stream_type_t stream_type = isp_fw_aframe_type_to_stream_type( type );

    if ( stream_type == V4L2_STREAM_TYPE_MAX ) {
        return -1;
    }

    /* find stream pointer */
    int rc = isp_v4l2_find_stream( &pstream, ctx_id, stream_type );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "Error. Can't find stream matching to context id: %u, type: %d, rc: %d", ctx_id, stream_type, rc );
        return -1;
    }

    /* check if stream is on */
    if ( !pstream->stream_started ) {
        LOG( LOG_ERR, "[Stream#%d] Stream has not yet started on context id: %d", pstream->stream_type, ctx_id );
        return -1;
    }

    // Get stream direction
    isp_v4l2_stream_direction_t stream_direction;
    if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {
        if ( stream_type == V4L2_STREAM_TYPE_RAW ) {
            stream_direction = V4L2_STREAM_DIRECTION_OUT;
        } else if ( stream_type == V4L2_STREAM_TYPE_OUT ) {
            stream_direction = V4L2_STREAM_DIRECTION_CAP;
        } else {
            LOG( LOG_ERR, "Error. Can't find stream direction matching m2m stream, context id: %u, stream type: %d", ctx_id, stream_type );
            return -1;
        }

        LOG( LOG_INFO, "[Stream#%d-%s] Get frame, ready buffer queue size: %u, busy buffer queue size: %u",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ),
             ( ( stream_direction == V4L2_STREAM_DIRECTION_CAP ) ? v4l2_m2m_num_dst_bufs_ready( pstream->fh.m2m_ctx ) : v4l2_m2m_num_src_bufs_ready( pstream->fh.m2m_ctx ) ),
             pstream->buffer_list[stream_direction].busy.size );
    } else {
        stream_direction = V4L2_STREAM_DIRECTION_CAP;

        LOG( LOG_INFO, "[Stream#%d-%s] Get frame, ready buffer queue size: %u, busy buffer queue size: %u",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ),
             pstream->buffer_list[stream_direction].ready.size, pstream->buffer_list[stream_direction].busy.size );
    }

    struct v4l2_format *v4l2_fmt = &pstream->cur_v4l2_fmt[stream_direction];

    /* try to get an active buffer from vb2 queue */
    if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {
        struct vb2_v4l2_buffer *vbuf;
        if ( stream_direction == V4L2_STREAM_DIRECTION_CAP ) {
            vbuf = v4l2_m2m_dst_buf_remove( pstream->fh.m2m_ctx );
        } else {
            vbuf = v4l2_m2m_src_buf_remove( pstream->fh.m2m_ctx );
        }

        // Once buffer is removed from the m2m context internal ready queue put it on the stream busy queue
        // to ensure we release all the acquired buffers
        if ( vbuf ) {
            pbuf = container_of( vbuf, isp_v4l2_buffer_t, vvb );

            spin_lock( &pstream->buffer_list[stream_direction].lock );
            list_add_tail( &pbuf->list, &pstream->buffer_list[stream_direction].busy.head );
            pstream->buffer_list[stream_direction].busy.size++;
            spin_unlock( &pstream->buffer_list[stream_direction].lock );
        }

    } else {

		if (state == AFRAME_STATE_FULL) {
			LOG (LOG_INFO, "requested filled frame");
        	spin_lock( &pstream->buffer_list[stream_direction].lock );
	        if ( !list_empty( &pstream->buffer_list[stream_direction].busy.head ) ) {
    	        pbuf = list_entry( pstream->buffer_list[stream_direction].busy.head.next, isp_v4l2_buffer_t, list );
        		*frame = &pbuf->frame;
				
       		} else {
				LOG( LOG_INFO, "list is empty");
			}
        	spin_unlock( &pstream->buffer_list[stream_direction].lock );
			return 0;

		} else {
			LOG (LOG_INFO, "requested empty frame");
        	spin_lock( &pstream->buffer_list[stream_direction].lock );
        	if ( !list_empty( &pstream->buffer_list[stream_direction].ready.head ) ) {
            	pbuf = list_entry( pstream->buffer_list[stream_direction].ready.head.next, isp_v4l2_buffer_t, list );
            	list_del( &pbuf->list );
            	list_add_tail( &pbuf->list, &pstream->buffer_list[stream_direction].busy.head );
           		pstream->buffer_list[stream_direction].ready.size--;
        		pstream->buffer_list[stream_direction].busy.size++;
        	}
        	spin_unlock( &pstream->buffer_list[stream_direction].lock );
		}
    }

    if ( pbuf == NULL ) {
        LOG( LOG_ERR, "[Stream#%d-%s] Stream has no empty buffers",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ) );
        return -1;
    }

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    struct vb2_v4l2_buffer *vvb = &pbuf->vvb;
    struct vb2_buffer *vb = &vvb->vb2_buf;
#else
    struct vb2_buffer *vb = &pbuf->vb;
#endif

    if ( ( v4l2_fmt->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ) || ( v4l2_fmt->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ) ) {
        struct v4l2_pix_format_mplane *pix_mp = &v4l2_fmt->fmt.pix_mp;

        if ( pix_mp->num_planes != vb->num_planes ) {
            LOG( LOG_CRIT, "[Stream#%d-%s] Stream number of planes (%u) doesn't match to VB2 buffer number of planes (%u)",
                 pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), pix_mp->num_planes, vb->num_planes );
            return -1;
        }

        // Update required aframe structure fields
        pbuf->frame.context_id = ctx_id;
        pbuf->frame.source = AFRAME_SOURCE_V4L2_STREAMER;
        pbuf->frame.type = type;
        pbuf->frame.state = AFRAME_STATE_EMPTY;
        pbuf->frame.num_planes = pix_mp->num_planes;
        pbuf->frame.frame_id = vb->index;

        // Set frame memory type based on VB2 buffer memory type
        if ( vb->memory == VB2_MEMORY_USERPTR ) {
            pbuf->frame.memory = AFRAME_MEMORY_USER;
        } else {
            pbuf->frame.memory = AFRAME_MEMORY_AUTO;
        }

        isp_v4l2_stream_get_planes( vb, pix_mp, &pbuf->frame );

        *frame = &pbuf->frame;
		LOG (LOG_INFO, "frame returned is %#llx, address : %#x",
				*frame, pbuf->frame.planes[0].address.low);

    } else {
        LOG( LOG_CRIT, "[Stream#%d-%s] Stream has invalid v4l2_fmt->type: %d",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), v4l2_fmt->type );
        return -1;
    }

    return 0;
}

int isp_v4l2_stream_put_frame( aframe_t *frame )
{
    isp_v4l2_stream_t *pstream = NULL;
    isp_v4l2_buffer_t *pbuf = NULL;

    isp_v4l2_stream_type_t stream_type = isp_fw_aframe_type_to_stream_type( frame->type );
    if ( stream_type == V4L2_STREAM_TYPE_MAX ) {
        return -1;
    }

    /* find stream pointer */
    int rc = isp_v4l2_find_stream( &pstream, frame->context_id, stream_type );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "Error. Can't find stream matching to context id: %u, type: %d, rc: %d",
             frame->context_id, stream_type, rc );
        return -1;
    }

    /* check if stream is on */
    if ( !pstream->stream_started ) {
        LOG( LOG_ERR, "[Stream#%d] Stream has not yet started on context id: %d", pstream->stream_type, frame->context_id );
        return -1;
    }

    // Get stream direction
    isp_v4l2_stream_direction_t stream_direction;
    if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {
        if ( stream_type == V4L2_STREAM_TYPE_RAW ) {
            stream_direction = V4L2_STREAM_DIRECTION_OUT;
        } else if ( stream_type == V4L2_STREAM_TYPE_OUT ) {
            stream_direction = V4L2_STREAM_DIRECTION_CAP;
        } else {
            LOG( LOG_ERR, "Error. Can't find stream direction matching m2m stream, context id: %u, stream type: %d", frame->context_id, stream_type );
            return -1;
        }
    } else {
        stream_direction = V4L2_STREAM_DIRECTION_CAP;
    }

	LOG( LOG_INFO," stream directions : %d", stream_direction);
    /* try to get an active buffer from vb2 queue */
    bool found = false;
    spin_lock( &pstream->buffer_list[stream_direction].lock );
    if ( !list_empty( &pstream->buffer_list[stream_direction].busy.head ) ) {

        struct list_head *p;

        list_for_each( p, &pstream->buffer_list[stream_direction].busy.head )
        {
            pbuf = list_entry( p, isp_v4l2_buffer_t, list );

            // Some buffers can be released by ISP in different order
            // from the way they were received, so try to find the match
            if ( &pbuf->frame == frame ) {
                found = true;
   				LOG (LOG_INFO, "Found Match :  current frame is (%#llx)%#x, looking for (%#llx)%#x",
					&pbuf->frame, pbuf->frame.planes[0].address.low, frame, frame->planes[0].address.low);
             break;
            } else {
				LOG (LOG_INFO, "no match current frame is (%#llx)%#x, looking for (%#llx)%#x",
					&pbuf->frame, pbuf->frame.planes[0].address.low, frame, frame->planes[0].address.low);
			}
        }

        list_del( &pbuf->list );
        pstream->buffer_list[stream_direction].busy.size--;
    }
    spin_unlock( &pstream->buffer_list[stream_direction].lock );

    if ( !found ) {
        LOG( LOG_ERR, "[Stream#%d-%s] Frame not found in the stream busy buffer list",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ) );
    }

    if ( !pbuf ) {
        LOG( LOG_INFO, "[Stream#%d-%s] Stream has no filled buffers",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ) );
        return -1;
    }

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    struct vb2_v4l2_buffer *vvb = &pbuf->vvb;
    struct vb2_buffer *vb = &vvb->vb2_buf;
    vvb->sequence = frame->sequence;
    vvb->field = V4L2_FIELD_NONE;
#else
    struct vb2_buffer *vb = &pbuf->vb;
    vb->v4l2_buf.sequence = frame->sequence;
    vb->v4l2_buf.field = V4L2_FIELD_NONE;
#endif

    if ( frame->num_planes < vb->num_planes ) {
        LOG( LOG_CRIT, "[Stream#%d-%s] Stream number of planes (%u) doesn't match to VB2 buffer number of planes (%u)",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), frame->num_planes, vb->num_planes );
        return -1;
    }

    isp_v4l2_stream_put_planes( vb, frame );

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    vb->timestamp = ktime_get_ns();
#else
    v4l2_get_timestamp( &vb->v4l2_buf.timestamp );
#endif

    // Check frame state. Non full frame state means buffer data is not valid and should be discarded by userspace
    if ( frame->state == AFRAME_STATE_FULL ) {
        /* Put buffer back to vb2 queue */
        if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {
            v4l2_m2m_buf_done( vvb, VB2_BUF_STATE_DONE );
        } else {
            vb2_buffer_done( vb, VB2_BUF_STATE_DONE );
        }

        LOG( LOG_DEBUG, "[Stream#%d-%s] set vb2 buffer status VB2_BUF_STATE_DONE",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ) );
    } else {
        /* Put buffer back to vb2 queue with error flag set */
        if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {
            v4l2_m2m_buf_done( vvb, VB2_BUF_STATE_ERROR );
        } else {
            vb2_buffer_done( vb, VB2_BUF_STATE_ERROR );
        }

        LOG( LOG_DEBUG, "[Stream#%d-%s] set vb2 buffer status VB2_BUF_STATE_ERROR",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ) );
    }

    /* Notify buffer ready */
    isp_v4l2_notify_event( pstream->ctx_id, pstream->stream_type, frame, V4L2_EVENT_ACAMERA_FRAME_READY, stream_direction );

    // For m2m stream check if we don't have any busy buffers and mark job as finished
    if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {
        const uint32_t out_buf_ready = v4l2_m2m_num_src_bufs_ready( pstream->fh.m2m_ctx );
        const uint32_t cap_buf_ready = v4l2_m2m_num_dst_bufs_ready( pstream->fh.m2m_ctx );

        spin_lock( &pstream->buffer_list[V4L2_STREAM_DIRECTION_CAP].lock );
        const uint32_t cap_buf_busy = pstream->buffer_list[V4L2_STREAM_DIRECTION_CAP].busy.size;
        spin_unlock( &pstream->buffer_list[V4L2_STREAM_DIRECTION_CAP].lock );

        spin_lock( &pstream->buffer_list[V4L2_STREAM_DIRECTION_OUT].lock );
        const uint32_t out_buf_busy = pstream->buffer_list[V4L2_STREAM_DIRECTION_OUT].busy.size;
        spin_unlock( &pstream->buffer_list[V4L2_STREAM_DIRECTION_OUT].lock );

        LOG( LOG_DEBUG, "[Stream#%d-%s] buffer queue status, cap-ready: %u, out-ready: %u, cap-busy: %u, out-busy: %u",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ),
             out_buf_ready, cap_buf_ready, cap_buf_busy, out_buf_busy );

        // Check if we have returned all the busy buffers
        if ( ( cap_buf_busy == 0 ) && ( out_buf_busy == 0 ) ) {
            LOG( LOG_DEBUG, "[Stream#%d-%s] m2m job is done, calling v4l2_m2m_job_finish()",
                 pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ) );
            v4l2_m2m_job_finish( pstream->fh.m2m_ctx->m2m_dev, pstream->fh.m2m_ctx );
        }
    }

    // Update required aframe structure fields
    frame->state = AFRAME_STATE_EMPTY;

    return 0;
}

/* ----------------------------------------------------------------
 * Stream control interface
 */

/* sensor static informations */
static isp_v4l2_stream_common g_stream_common[FIRMWARE_CONTEXT_NUMBER];

int isp_v4l2_stream_init_static_resources( uint32_t ctx_id )
{
    isp_v4l2_stream_common *sc = &( g_stream_common[ctx_id] );
    int i;

    /* initialize stream common field */
    memset( sc, 0, sizeof( isp_v4l2_stream_common ) );
    fw_intf_isp_get_sensor_info( ctx_id, &sc->sensor_info );
    sc->snapshot_sizes.frmsize_num = sc->sensor_info.num_modes;
    for ( i = 0; i < sc->sensor_info.num_modes; i++ ) {
        sc->snapshot_sizes.frmsize[i].width = sc->sensor_info.mode[i].width;
        sc->snapshot_sizes.frmsize[i].height = sc->sensor_info.mode[i].height;
    }

    return 0;
}

#if 0
static void work_queue_fn(struct work_struct *work) {

	//LOG(LOG_ERR, "work queue fn invoked");

    isp_v4l2_stream_t *pstream = container_of(work, isp_v4l2_stream_t, work);
    int process = 0;
#if 0
    spin_lock(&(pstream->vb2_stream_lock));
    if (pstream->vb2_queue_on) {
		process = 1;
	}
    spin_unlock(&(pstream->vb2_stream_lock));

    if (process) {
        fw_intf_process_input( pstream->ctx_id);
        //LOG( LOG_ERR, "MIPI buffer submitted ctx: %d", pstream->ctx_id);	    
    } else {
        LOG( LOG_WARNING, "Dropped MIPI buffer from process ctx: %d", pstream->ctx_id);
    }
#endif

}
#endif


int isp_v4l2_stream_init( isp_v4l2_stream_t **ppstream, int stream_type, int ctx_id )
{
    isp_v4l2_stream_t *new_stream = NULL;
    //int current_sensor_preset;
    LOG( LOG_INFO, "[Stream#%d] Initializing stream .... ctx : %d", stream_type, ctx_id );

    /* allocate isp_v4l2_stream_t */
    new_stream = kzalloc( sizeof( isp_v4l2_stream_t ), GFP_KERNEL );
    if ( new_stream == NULL ) {
        LOG( LOG_ERR, "[Stream#%d] Failed to allocate isp_v4l2_stream_t.", stream_type );
        return -ENOMEM;
    }

    /* set default format */

    // All stream multiplanar
    new_stream->cur_v4l2_fmt[V4L2_STREAM_DIRECTION_CAP].type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    new_stream->cur_v4l2_fmt[V4L2_STREAM_DIRECTION_OUT].type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;

    /* set input stream info */
    new_stream->stream_common = &( g_stream_common[ctx_id] );

    /* init control fields */
    new_stream->ctx_id = ctx_id;
    new_stream->stream_type = stream_type;
    new_stream->stream_started = 0;

    // Set new stream format to default ISP settings
    isp_v4l2_stream_try_format( new_stream, V4L2_STREAM_DIRECTION_CAP, &( new_stream->cur_v4l2_fmt[V4L2_STREAM_DIRECTION_CAP] ) );

    int stream_direction;
    for ( stream_direction = 0; stream_direction < V4L2_STREAM_DIRECTION_MAX; stream_direction++ ) {
        /* init list and list size*/
        INIT_LIST_HEAD( &new_stream->buffer_list[stream_direction].ready.head );
        INIT_LIST_HEAD( &new_stream->buffer_list[stream_direction].busy.head );

        new_stream->buffer_list[stream_direction].ready.size = 0;
        new_stream->buffer_list[stream_direction].busy.size = 0;

        /* init locks */
        spin_lock_init( &new_stream->buffer_list[stream_direction].lock );
    }

	/* init the work queue */
	//INIT_WORK(&new_stream->work, work_queue_fn);
    /* return stream private ptr to caller */
    *ppstream = new_stream;

    return 0;
}

void isp_v4l2_stream_deinit( isp_v4l2_stream_t *pstream, unsigned long stream_on_mask )
{
    if ( !pstream ) {
        LOG( LOG_ERR, "Invalid parameter. Stream pointer is NULL" );
        return;
    }

    LOG( LOG_INFO, "[Stream#%d] Deinitializing stream", pstream->stream_type );

	//cancel_work_sync(&pstream->work);

    /* do stream-off first if it's on */
    if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {
        isp_v4l2_stream_off( pstream, V4L2_STREAM_DIRECTION_CAP, stream_on_mask );
        isp_v4l2_stream_off( pstream, V4L2_STREAM_DIRECTION_OUT, stream_on_mask );
    } else {
        isp_v4l2_stream_off( pstream, V4L2_STREAM_DIRECTION_CAP, stream_on_mask );
    }
}

void isp_v4l2_stream_free( isp_v4l2_stream_t *pstream )
{
    if ( !pstream ) {
        LOG( LOG_ERR, "Invalid parameter. Stream pointer is NULL" );
        return;
    }

    LOG( LOG_INFO, "[Stream#%d] Releasing stream memory", pstream->stream_type );

    kvfree( pstream );
    pstream = NULL;
}

static void isp_v4l2_stream_buffer_list_release( isp_v4l2_stream_t *pstream,
                                                 struct list_head *stream_buffer_list, isp_v4l2_stream_direction_t stream_direction )
{
    isp_v4l2_buffer_t *buf;
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    struct vb2_v4l2_buffer *vvb;
#endif
    struct vb2_buffer *vb;
    unsigned int buf_index;

    // Release m2m ready buffers ( managed by m2m context queue )
    if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {

        struct vb2_v4l2_buffer *vbuf;
        while ( 1 ) {
            if ( stream_direction == V4L2_STREAM_DIRECTION_CAP ) {
                vbuf = v4l2_m2m_dst_buf_remove( pstream->fh.m2m_ctx );
            } else {
                vbuf = v4l2_m2m_src_buf_remove( pstream->fh.m2m_ctx );
            }

            if ( vbuf == NULL ) {
                break;
            }

            LOG( LOG_INFO, "[Stream#%d-%s] Releasing VB2 buffer, index: %d",
                 pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), vbuf->vb2_buf.index );

            v4l2_m2m_buf_done( vbuf, VB2_BUF_STATE_ERROR );
        }
    }

    // Releasing stream ready and busy buffers
    while ( !list_empty( stream_buffer_list ) ) {
        buf = list_entry( stream_buffer_list->next,
                          isp_v4l2_buffer_t, list );
        list_del( &buf->list );

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
        vvb = &buf->vvb;
        vb = &vvb->vb2_buf;

        buf_index = vb->index;
#else
        vb = &buf->vb;

        buf_index = vb->v4l2_buf.index;
#endif

        vb2_buffer_done( vb, VB2_BUF_STATE_ERROR );

        LOG( LOG_INFO, "[Stream#%d-%s] Releasing VB2 buffer, index: %d",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), buf_index );
    }
}

int isp_v4l2_stream_on( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, unsigned long stream_open_mask )
{
    if ( !pstream ) {
        LOG( LOG_ERR, "Invalid parameter. Stream pointer is NULL" );
        return -EINVAL;
    }

    if ( !isp_v4l2_stream_check_direction( stream_direction ) ) {
        LOG( LOG_ERR, "Invalid parameter. Stream direction is out of range: %d", stream_direction );
        return -EINVAL;
    }

    pstream->stream_direction_started[stream_direction] = 1;

    int stream_started = pstream->stream_direction_started[V4L2_STREAM_DIRECTION_CAP];
    if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {
        stream_started &= pstream->stream_direction_started[V4L2_STREAM_DIRECTION_OUT];
    }

    LOG( LOG_INFO, "[Stream#%d-%s-%d] Enable streaming, stream is ready to start hardware: %s",
         pstream->stream_type,
         isp_v4l2_stream_get_direction_string( stream_direction ), pstream->ctx_id,
         ( ( stream_started ) ? "yes" : "no" ) );

    /** control fields update,
     *  set stream as preliminary started to let ISP acquire buffers for configuration and startup
     */
    pstream->stream_started = stream_started;

    // If all stream directions are started, call hardware stream on
    if ( stream_started ) {
        if ( fw_intf_stream_start( pstream->ctx_id, pstream->stream_type, stream_open_mask ) < 0 ) {
			LOG (LOG_ERR, "Failed to start the stream");
            pstream->stream_direction_started[stream_direction] = 0;
            pstream->stream_started = 0;
            return -1;
        }
    }

    return pstream->stream_started;
}

int isp_v4l2_stream_off( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, unsigned long stream_on_mask )
{
    if ( !pstream ) {
        LOG( LOG_ERR, "Invalid parameter. Stream pointer is NULL" );
        return -EINVAL;
    }

    if ( !isp_v4l2_stream_check_direction( stream_direction ) ) {
        LOG( LOG_ERR, "Invalid parameter. Stream direction is out of range: %d", stream_direction );
        return -EINVAL;
    }

    LOG( LOG_INFO, "[Stream#%d-%s-%d] Disable streaming", pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), pstream->ctx_id );

    fw_intf_stream_stop( pstream->ctx_id, pstream->stream_type, stream_on_mask );

    // Control fields update
    pstream->stream_direction_started[stream_direction] = 0;

    /* Release all active buffers */
    spin_lock( &pstream->buffer_list[stream_direction].lock );
    isp_v4l2_stream_buffer_list_release( pstream, &pstream->buffer_list[stream_direction].ready.head, stream_direction );
    isp_v4l2_stream_buffer_list_release( pstream, &pstream->buffer_list[stream_direction].busy.head, stream_direction );
    pstream->buffer_list[stream_direction].ready.size = 0;
    pstream->buffer_list[stream_direction].busy.size = 0;
    spin_unlock( &pstream->buffer_list[stream_direction].lock );


    if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {
        pstream->stream_started = pstream->stream_direction_started[V4L2_STREAM_DIRECTION_CAP] || pstream->stream_direction_started[V4L2_STREAM_DIRECTION_OUT];
    } else {
        pstream->stream_started = pstream->stream_direction_started[V4L2_STREAM_DIRECTION_CAP];
    }

    return pstream->stream_started;
}

/* ----------------------------------------------------------------
 * Stream configuration interface
 */

/**
 * @brief Checks whether requested format supported by the stream
 * 
 * @param pixelformat Requested pixelformat
 * @param stream_type Stream type to check
 * @param stream_direction Stream direction to check
 * @return isp_v4l2_fmt_t* Returns pointer to entry found, NULL otherwise
 */
static isp_v4l2_fmt_t *isp_v4l2_stream_find_format( uint32_t pixelformat, int stream_type, isp_v4l2_stream_direction_t stream_direction )
{
    // Get effective stream type
    const isp_v4l2_stream_type_t effective_stream_type = isp_v4l2_stream_get_effective_stream_type( stream_type, stream_direction );

    if ( effective_stream_type != V4L2_STREAM_TYPE_MAX ) {

        uint32_t i;
        for ( i = 0; i < isp_v4l2_stream_supported_formats[effective_stream_type].num_formats; i++ ) {
            isp_v4l2_fmt_t *fmt = &isp_v4l2_stream_supported_formats[effective_stream_type].formats[i];

            if ( fmt->pixelformat == pixelformat ) {
                return fmt;
            }
        }
    }

    return NULL;
}

int isp_v4l2_stream_enum_framesizes( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, struct v4l2_frmsizeenum *fsize )
{
    // Check stream type
    if ( ( !isp_v4l2_stream_check_type( pstream->stream_type ) ) || ( !isp_v4l2_stream_check_direction( stream_direction ) ) ) {
        return -EINVAL;
    }

    LOG( LOG_INFO, "[Stream#%d-%s] index: %d, pixel_format: 0x%x.\n",
         pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), fsize->index, fsize->pixel_format );

    if ( !isp_v4l2_stream_find_format( fsize->pixel_format, pstream->stream_type, stream_direction ) )
        return -EINVAL;

    /* check index */
    if ( fsize->index >= pstream->stream_common->snapshot_sizes.frmsize_num ) {
        LOG( LOG_INFO, "[Stream#%d-%s] index (%d) should be smaller than %d.",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ),
             fsize->index, pstream->stream_common->snapshot_sizes.frmsize_num );
        return -EINVAL;
    }

    /* return framesize */
    fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    fsize->discrete = pstream->stream_common->snapshot_sizes.frmsize[fsize->index];

    return 0;
}

int isp_v4l2_stream_enum_format( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, struct v4l2_fmtdesc *f )
{
    const isp_v4l2_fmt_t *fmt;
    int desc_size = 0;

    // Get effective stream type
    const isp_v4l2_stream_type_t effective_stream_type = isp_v4l2_stream_get_effective_stream_type( pstream->stream_type, stream_direction );

    // Check stream type
    if ( effective_stream_type == V4L2_STREAM_TYPE_MAX ) {
        return -EINVAL;
    }

    /* check index */
    if ( f->index >= isp_v4l2_stream_supported_formats[effective_stream_type].num_formats ) {
        LOG( LOG_INFO, "[Stream#%d-%s] format index (%d) should be smaller than %u.",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ),
             f->index, isp_v4l2_stream_supported_formats[effective_stream_type].num_formats );
        return -EINVAL;
    }

    /* get format from index */
    fmt = &isp_v4l2_stream_supported_formats[effective_stream_type].formats[f->index];

    /* check description length */
    if ( sizeof( fmt->description ) > sizeof( f->description ) ) {
        desc_size = sizeof( f->description );
    } else {
        desc_size = sizeof( fmt->description );
    }

    /* reset flag */
    f->flags = 0;

    /* copy description */
    strlcpy( f->description, fmt->description, desc_size );

    /* copy format code */
    f->pixelformat = fmt->pixelformat;

    LOG( LOG_INFO, "[Stream#%d-%s] index: %d, format: 0x%x, desc: %s.\n",
         pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), f->index, f->pixelformat, f->description );

    return 0;
}

int isp_v4l2_stream_try_format( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, struct v4l2_format *f )
{
    LOG( LOG_INFO, "[Stream#%d-%s] Try stream format, type: %u, pixelformat: 0x%08x (%s), width: %u, height: %u",
         pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), f->type, f->fmt.pix_mp.pixelformat,
         isp_v4l2_stream_get_pixelformat_string( f->fmt.pix_mp.pixelformat ), f->fmt.pix_mp.width, f->fmt.pix_mp.height );

    isp_v4l2_fmt_t *tfmt = isp_v4l2_stream_find_format( f->fmt.pix_mp.pixelformat, pstream->stream_type, stream_direction );

    // Get effective stream type
    const isp_v4l2_stream_type_t effective_stream_type = isp_v4l2_stream_get_effective_stream_type( pstream->stream_type, stream_direction );

    // RAW stream requires checking data width for requested format as this depends on the sensor data width
    if ( effective_stream_type == V4L2_STREAM_TYPE_RAW && ( tfmt != NULL ) ) {

        // Update stream common sensor info to get the current sensor mode and submode
        isp_v4l2_stream_update_common_sensor_info( pstream );

        const uint8_t mode = pstream->stream_common->sensor_info.cur_mode;
        const uint8_t sub_mode = pstream->stream_common->sensor_info.mode[mode].cur_sub_mode;
        const uint8_t sensor_data_width = pstream->stream_common->sensor_info.mode[mode].sub_mode[sub_mode].data_width;

        // Check current sensor data width against the requested format and reset format to default if there is a mismatch
        if ( sensor_data_width != tfmt->data_width ) {
            LOG( LOG_NOTICE, "[Stream#%d-%s] Requested format: %s data width (%d) does not match the sensor configured data width (%d), default format will be used instead.",
                 pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ),
                 isp_v4l2_stream_get_pixelformat_string( f->fmt.pix.pixelformat ), tfmt->data_width, sensor_data_width );
            tfmt = NULL;
        }
    }

    /* Check format and fall-back to default if not supported */
    if ( !tfmt ) {
        const uint32_t default_pixel_format = isp_v4l2_stream_get_default_pixelformat( pstream, stream_direction );
        if ( default_pixel_format != 0 ) {
            LOG( LOG_INFO, "[Stream#%d-%s] Requested format: 0x%08x is not supported, setting default format to: 0x%08x (%s)",
                 pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ),
                 f->fmt.pix.pixelformat, default_pixel_format, isp_v4l2_stream_get_pixelformat_string( default_pixel_format ) );
            if ( stream_direction == V4L2_STREAM_DIRECTION_CAP ) {
                f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            } else {
                f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            }
            f->fmt.pix_mp.pixelformat = default_pixel_format;
            tfmt = isp_v4l2_stream_find_format( f->fmt.pix_mp.pixelformat, pstream->stream_type, stream_direction );
        } else {
            LOG( LOG_ERR, "[Stream#%d-%s] Requested format: 0x%08x is not supported. Failed to get default format.",
                 pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), f->fmt.pix.pixelformat );
        }
    }

    // If at this stage format still cannot be found, something is really wrong with the config
    if ( !tfmt ) {
        return -EINVAL;
    }

    // Update plane number for raw stream format based on sensor channel number
    if ( effective_stream_type == V4L2_STREAM_TYPE_RAW ) {

        const uint8_t mode = pstream->stream_common->sensor_info.cur_mode;
        const uint8_t sub_mode = pstream->stream_common->sensor_info.mode[mode].cur_sub_mode;

        // Update planes number to match sensor channel number, so raw buffer would keep all sensor channels
        tfmt->num_planes = pstream->stream_common->sensor_info.mode[mode].sub_mode[sub_mode].num_channels;
    }

    /* Adjust width, height for META stream */
    if ( effective_stream_type == V4L2_STREAM_TYPE_META ) {
        f->fmt.pix.width = ISP_V4L2_METADATA_SIZE;
        f->fmt.pix.height = 1;
    } else {
        if ( f->fmt.pix.width == 0 || f->fmt.pix.height == 0 ) {
            const uint8_t mode = pstream->stream_common->sensor_info.cur_mode;

            f->fmt.pix.width = pstream->stream_common->sensor_info.mode[mode].width;
            f->fmt.pix.height = pstream->stream_common->sensor_info.mode[mode].height;
        }

        v4l_bound_align_image( &f->fmt.pix.width, 48, ISP_V4L2_MAX_WIDTH, 1,
                               &f->fmt.pix.height, 32, ISP_V4L2_MAX_HEIGHT, 1, 1 );
    }

    // All streams are multiplanar
    if ( stream_direction == V4L2_STREAM_DIRECTION_CAP ) {
        f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    } else {
        f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    }
    f->fmt.pix.field = V4L2_FIELD_NONE;
    f->fmt.pix_mp.num_planes = tfmt->num_planes;
    f->fmt.pix_mp.colorspace = ( tfmt->is_yuv ) ? V4L2_COLORSPACE_SMPTE170M : V4L2_COLORSPACE_SRGB;

    uint8_t i;
    for ( i = 0; i < tfmt->num_planes; i++ ) {
        // bytesperline should be multiple of 32 due to ISP AXI alignment requirements (AXI bus width is 256-bit)
        f->fmt.pix_mp.plane_fmt[i].bytesperline = ( ( ( ( f->fmt.pix_mp.width * tfmt->data_width ) >> 3 ) + 31 ) >> 5 ) << 5;
        f->fmt.pix_mp.plane_fmt[i].sizeimage = f->fmt.pix_mp.height * f->fmt.pix_mp.plane_fmt[i].bytesperline;

        if ( f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_NV12 ) {
            if ( i == 1 ) {
                f->fmt.pix_mp.plane_fmt[i].sizeimage /= 2;
            }
        }

        memset( f->fmt.pix_mp.plane_fmt[i].reserved, 0, sizeof( f->fmt.pix_mp.plane_fmt[i].reserved ) );
        memset( f->fmt.pix_mp.reserved, 0, sizeof( f->fmt.pix_mp.reserved ) );
    }

    return 0;
}

int isp_v4l2_stream_get_format( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, struct v4l2_format *f )
{
    if ( ( pstream == NULL ) || ( f == NULL ) ) {
        LOG( LOG_ERR, "Invalid parameter. Stream or format pointer is NULL" );
        return -EINVAL;
    }

    if ( !isp_v4l2_stream_check_direction( stream_direction ) ) {
        LOG( LOG_ERR, "Invalid parameter. Stream direction is not valid: %d", stream_direction );
        return -EINVAL;
    }

    *f = pstream->cur_v4l2_fmt[stream_direction];

    LOG( LOG_INFO, "[Stream#%d-%s] Get stream format: width: %4u, height: %4u, format: 0x%x (%s)",
         pstream->stream_type,
         isp_v4l2_stream_get_direction_string( stream_direction ),
         f->fmt.pix_mp.width,
         f->fmt.pix_mp.height,
         f->fmt.pix_mp.pixelformat,
         isp_v4l2_stream_get_pixelformat_string( f->fmt.pix_mp.pixelformat ) );

    if ( f->fmt.pix_mp.width == 0 || f->fmt.pix_mp.height == 0 || f->fmt.pix_mp.pixelformat == 0 ) { //not formatted yet
        LOG( LOG_NOTICE, "Compliance error. Uninitialized format" );
    }

    return 0;
}

int isp_v4l2_stream_set_format( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, struct v4l2_format *f )
{
    if ( ( pstream == NULL ) || ( f == NULL ) ) {
        LOG( LOG_ERR, "Invalid parameter. Stream or format pointer is NULL" );
        return -EINVAL;
    }

    if ( !isp_v4l2_stream_check_direction( stream_direction ) ) {
        LOG( LOG_ERR, "Invalid parameter. Stream direction is not valid: %d", stream_direction );
        return -EINVAL;
    }

    LOG( LOG_NOTICE, "[Stream#%d-%s] Set stream format: width: %4u, height: %4u, format: 0x%x (%s)",
         pstream->stream_type,
         isp_v4l2_stream_get_direction_string( stream_direction ),
         f->fmt.pix_mp.width,
         f->fmt.pix_mp.height,
         f->fmt.pix_mp.pixelformat,
         isp_v4l2_stream_get_pixelformat_string( f->fmt.pix_mp.pixelformat ) );

    /* try format first */
    int rc = isp_v4l2_stream_try_format( pstream, stream_direction, f );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "[Stream#%d-%s] Function isp_v4l2_stream_try_format call failed, rc: %d",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), rc );
        return rc;
    }

    /* update resolution */
    rc = fw_intf_stream_set_resolution( pstream->ctx_id, &pstream->stream_common->sensor_info,
                                        pstream->stream_type, &( f->fmt.pix_mp.width ), &( f->fmt.pix_mp.height ) );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "[Stream#%d-%s] Function stream_set_resolution call failed, rc: %d",
             pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), rc );
        return rc;
    }

    const uint32_t mode = pstream->stream_common->sensor_info.cur_mode;
    const uint32_t sub_mode = pstream->stream_common->sensor_info.mode[mode].cur_sub_mode;
    LOG( LOG_INFO, "[Stream#%d-%s] Current sensor mode: %u, sub mode: %u, exposures: %u, fps: %u",
         pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ), mode, sub_mode,
         pstream->stream_common->sensor_info.mode[mode].sub_mode[sub_mode].exposures,
         pstream->stream_common->sensor_info.mode[mode].sub_mode[sub_mode].fps / 256 );

    /* update format */
    rc = fw_intf_stream_set_output_format( pstream->ctx_id, pstream->stream_type, stream_direction, f->fmt.pix_mp.pixelformat );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "Error. stream_set_output_format call failed, rc: %d", rc );
        return rc;
    }

    /* update format field */
    pstream->cur_v4l2_fmt[stream_direction] = *f;

    LOG( LOG_NOTICE, "[Stream#%d-%s] Effective stream format: width: %4u, height: %4u, type: %u, format: 0x%x (%s)",
         pstream->stream_type, isp_v4l2_stream_get_direction_string( stream_direction ),
         pstream->cur_v4l2_fmt[stream_direction].fmt.pix_mp.width,
         pstream->cur_v4l2_fmt[stream_direction].fmt.pix_mp.height,
         pstream->cur_v4l2_fmt[stream_direction].type,
         pstream->cur_v4l2_fmt[stream_direction].fmt.pix_mp.pixelformat,
         isp_v4l2_stream_get_pixelformat_string( pstream->cur_v4l2_fmt[stream_direction].fmt.pix_mp.pixelformat ) );

    return 0;
}
