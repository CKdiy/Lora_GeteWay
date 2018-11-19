/**
  ******************************************************************************
  * @file    image.h 
  * @author  George
  * @date    
  * @brief   header file for image
  ******************************************************************************
***/
#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <stdint.h>

/********************************************
* Defines *
********************************************/
#define IH_MAGIC    0x20160826

enum
{
    GTWAY_IMGID     = 0x00
};

#define STM32_VERLEN        0x04
/********************************************
* Typedefs *
********************************************/
typedef struct
{
    uint8_t         imgid:5;
    uint8_t         len:3;
}image_type_t;

typedef struct
{
    uint32_t        ih_magic;           /* Image Magic Number */
    uint32_t        ih_hcrc;            /* Image CRC Checksum */
    uint32_t        ih_dcrc;            /* Image Data CRC Checksum */
    uint32_t        ih_size;            /* Image Data Size */
    image_type_t    ih_type;            /* Image type */
    uint8_t         ih_version[4];      /* Image version */
    /* the image header length is 128, so padding 107 bytes */
    uint8_t         reserved[107];      /* pad */
}image_header_t;


/********************************************
* Function defines *
********************************************/
int imageCheck(const uint32_t img_addr);

#endif
