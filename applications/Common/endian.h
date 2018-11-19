#ifndef __ENDIAN_H__
#define __ENDIAN_H__

#ifndef ntohs

#if (defined(__ICCARM__) && (__LITTLE_ENDIAN__ == 0)) || (defined(__CC_ARM) && defined(__BIG_ENDIAN))
#define ntohs(x) x
#define htons(x) x
#define ntohl(x) x
#define htonl(x) x
#else
#define ntohs(x) (uint16_t)( ((uint16_t)(x<<8) & 0xFF00) | ((uint16_t)(x>>8) & 0x00FF) )
#define htons(x) (uint16_t)( ((uint16_t)(x<<8) & 0xFF00) | ((uint16_t)(x>>8) & 0x00FF) )
#define ntohl(x) (uint32_t)( ((uint32_t)(x<<24) & 0xFF000000) |  \
                                ((uint32_t)(x<<8) & 0x00FF0000)  |  \
                                ((uint32_t)(x>>8) & 0x0000FF00)  |  \
                                ((uint32_t)(x>>24) & 0x000000FF) )
                                
#define htonl(x) (uint32_t)( ((uint32_t)(x<<24) & 0xFF000000) |  \
                                ((uint32_t)(x<<8) & 0x00FF0000)  |  \
                                ((uint32_t)(x>>8) & 0x0000FF00)  |  \
                                ((uint32_t)(x>>24) & 0x000000FF) )

#endif

#endif //ntohs

#endif
