#ifdef ALT_INICHE
    #include "ipport.h"
#endif

#include "system.h"
#include "altera_avalon_tse.h"
#include "altera_avalon_tse_system_info.h"

/* if using marvell phy and you want rgmii mode, uncomment this define */
/*#define SET_MARVELL_RGMII_MODE*/

#ifdef SET_MARVELL_RGMII_MODE
   #define ADDITIONAL_PHY_CFG  &marvell_cfg_rgmii
#else
   #define ADDITIONAL_PHY_CFG  0
#endif   

#ifdef TSE_MAC_BASE
alt_tse_system_info tse_mac_device[MAXNETS] = {
            TSE_SYSTEM_EXT_MEM_NO_SHARED_FIFO(TSE_MAC, 0, MSGDMA_TX, MSGDMA_RX, TSE_PHY_AUTO_ADDRESS, ADDITIONAL_PHY_CFG, DESCRIPTOR_MEMORY)
};

#else

#ifdef DESCRIPTOR_MEMORY_BASE
alt_tse_system_info tse_mac_device[MAXNETS] = {
        TSE_SYSTEM_EXT_MEM_NO_SHARED_FIFO(TSE_0_TSE, 0, TSE_0_DMA_TX, TSE_0_DMA_RX, TSE_PHY_AUTO_ADDRESS, ADDITIONAL_PHY_CFG, DESCRIPTOR_MEMORY)
        
};
#else
alt_tse_system_info tse_mac_device[MAXNETS] = {
        TSE_SYSTEM_INT_MEM_NO_SHARED_FIFO(TSE_0_TSE, 0, TSE_0_DMA_TX, TSE_0_DMA_RX, TSE_PHY_AUTO_ADDRESS, ADDITIONAL_PHY_CFG)
        
};
#endif

#endif
