#ifndef HSMMC_H
#define HSMMC_H

#if defined(CONFIG_TECHNEXION_OMAP3PLUS_SERIES)
void omap_hsmmc_inform_detection(int onoff);
#else
static inline void omap_hsmmc_inform_detection(int onoff)
{
}
#endif
#endif
