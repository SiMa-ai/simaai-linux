/*
 * Camera hook 
 *
 * (C) Copyright 2023
 *     SiMa.ai,<https://sima.ai>
 */

/***************************************************************************************************

 0x7FF000
 ---------------------------------------------------------------------------------------------------        
 |                                                |                                                |
 |            Frame Type                          |                  Stream Count                  |                
 |                                                |                                                | 
 ---------------------------------------------------------------------------------------------------
 0bit                                              16bit                                         32bit   


 ***************************************************************************************************/

/* OCM Location */
#define FRAME_STREAM_ADD 0x7FFFF0  

/* Frame Type */
#define COLORBAR                     0
#define RAW8_8_1920_1080_FULLHD      1
#define RAW8_1_4096_2160_CINEMA4K    2
#define RGB888_8_1920_1080_FULLHD    3
#define RAW888_1_4096_2160_CINEMA4K  4
#define YUV422_8_1920_1080_10BIT     5
#define YUV422_1_4096_2160_1BIT      6
#define YUV420_8_1920_1080_10BIT     7
#define YUV420_1_4096_2160_1BIT      8
#define RAW12_8_1920_1080_FULLHD     9
#define RAW12_1_4096_2160_CINEMA4K   10
#define RAW8_2_2048_1080             11
#define RAW8_2_2560_1440_QHD         12
#define RAW8_2_3840_2160_UHD         13

/* Stream Count */
#define STREAM_COUNT_ONE   1
#define STREAM_COUNT_TWO   2
#define STREAM_COUNT_THREE 3
#define STREAM_COUNT_FOUR  4
#define STREAM_COUNT_FIVE  5
#define STREAM_COUNT_SIX   6
#define STREAM_COUNT_SEVEN 7
#define STREAM_COUNT_EIGHT 8
