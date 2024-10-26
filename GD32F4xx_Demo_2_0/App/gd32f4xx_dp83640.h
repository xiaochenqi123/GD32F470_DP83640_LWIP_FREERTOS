#ifndef GD32_F4XX_DP83640_H
#define GD32_F4XX_DP83640_H
#include "gd32f4xx.h"


ErrStatus enet_dp83640_init(enet_mediamode_enum mediamode, enet_chksumconf_enum checksum, enet_frmrecept_enum recept);
ErrStatus enet_phy_DP83640_config(void);

static void enet_delay(uint32_t ncount);
static void enet_default_init(void);


#endif // !GD32_F4XX_DP83640_H#define GD32_F4XX_DP83640_


