#include "gd32f4xx_dp83640.h"
#include "epl.h"
#include "gd32f4xx_enet.h"

#define _ENET_DELAY_                              enet_delay

static enet_initpara_struct enet_initpara = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/*phy配置函数*/
ErrStatus enet_phy_DP83640_config(void)
{
    uint32_t ahbclk;
    uint32_t reg;
    uint16_t phy_value;
    ErrStatus enet_state = ERROR;

    /* clear the previous MDC clock */
    reg = ENET_MAC_PHY_CTL;
    reg &= ~ENET_MAC_PHY_CTL_CLR;

    /* get the HCLK frequency */
    ahbclk = rcu_clock_freq_get(CK_AHB);

    /* configure MDC clock according to HCLK frequency range */
    if(ENET_RANGE(ahbclk, 20000000U, 35000000U)) {
        reg |= ENET_MDC_HCLK_DIV16;
    } else if(ENET_RANGE(ahbclk, 35000000U, 60000000U)) {
        reg |= ENET_MDC_HCLK_DIV26;
    } else if(ENET_RANGE(ahbclk, 60000000U, 100000000U)) {
        reg |= ENET_MDC_HCLK_DIV42;
    } else if(ENET_RANGE(ahbclk, 100000000U, 150000000U)) {
        reg |= ENET_MDC_HCLK_DIV62;
    } else if((ENET_RANGE(ahbclk, 150000000U, 240000000U)) || (240000000U == ahbclk)) {
        reg |= ENET_MDC_HCLK_DIV102;
    } else {
        return enet_state;
    }
    ENET_MAC_PHY_CTL = reg;

    /* reset PHY */
    phy_value = BMCR_RESET;
    if(ERROR == (enet_phy_write_read(ENET_PHY_WRITE, PHY_ADDRESS, PHY_BMCR, &phy_value))) {
        return enet_state;
    }
    /* PHY reset need some time */
    _ENET_DELAY_(ENET_DELAY_TO);

    /* check whether PHY reset is complete */
    if(ERROR == (enet_phy_write_read(ENET_PHY_READ, PHY_ADDRESS, PHY_BMCR, &phy_value))) {
        return enet_state;
    }

    /* PHY reset complete */
    if(RESET == (phy_value & BMCR_RESET)) {
        enet_state = SUCCESS;
    }

    return enet_state;
}
/*phy初始化 */
ErrStatus enet_dp83640_init(enet_mediamode_enum mediamode, enet_chksumconf_enum checksum, enet_frmrecept_enum recept)
{
    uint32_t reg_value = 0U, reg_temp = 0U, temp = 0U;
    uint32_t media_temp = 0U;
    uint32_t timeout = 0U;
    uint16_t phy_value = 0U;
    ErrStatus phy_state = ERROR, enet_state = ERROR;

    /* PHY interface configuration, configure SMI clock and reset PHY chip */
    if(ERROR == enet_phy_config()) {
        _ENET_DELAY_(PHY_RESETDELAY);
        if(ERROR == enet_phy_config()) {
            return enet_state;
        }
    }
    /* initialize ENET peripheral with generally concerned parameters */
    enet_default_init();

    /* 1st, configure mediamode */
    media_temp = (uint32_t)mediamode;
    /* if is PHY auto negotiation */
    if((uint32_t)ENET_AUTO_NEGOTIATION == media_temp) {
        /* wait for PHY_LINKED_STATUS bit be set */
        do {
            enet_phy_write_read(ENET_PHY_READ, PHY_ADDRESS, PHY_BMSR, &phy_value);
            phy_value &= PHY_LINKED_STATUS;
            timeout++;
        } while((RESET == phy_value) && (timeout < PHY_READ_TO));
        /* return ERROR due to timeout */
        if(PHY_READ_TO == timeout) {
            return enet_state;
        }
        /* reset timeout counter */
        timeout = 0U;

        /* enable auto-negotiation */
        phy_value = PHY_AUTONEGOTIATION;
        phy_state = enet_phy_write_read(ENET_PHY_WRITE, PHY_ADDRESS, PHY_BMCR, &phy_value);
        if(!phy_state) {
            /* return ERROR due to write timeout */
            return enet_state;
        }

        /* wait for the PHY_AUTONEGO_COMPLETE bit be set */
        do {
            enet_phy_write_read(ENET_PHY_READ, PHY_ADDRESS, PHY_BMSR, &phy_value);
            phy_value &= PHY_AUTONEGO_COMPLETE;
            timeout++;
        } while((RESET == phy_value) && (timeout < (uint32_t)PHY_READ_TO));
        /* return ERROR due to timeout */
        if(PHY_READ_TO == timeout) {
            return enet_state;
        }
        /* reset timeout counter */
        timeout = 0U;

        /* read the result of the auto-negotiation */
        enet_phy_write_read(ENET_PHY_READ, PHY_ADDRESS, PHY_BMSR, &phy_value);
        /* configure the duplex mode of MAC following the auto-negotiation result */
        if((uint16_t)RESET != (phy_value & PHY_DUPLEX_STATUS)) {
            media_temp = ENET_MODE_FULLDUPLEX;
        } else {
            media_temp = ENET_MODE_HALFDUPLEX;
        }
        /* configure the communication speed of MAC following the auto-negotiation result */
        if((uint16_t)RESET != (phy_value & PHY_SPEED_STATUS)) {
            media_temp |= BMCR_FORCE_SPEED_10;
        } else {
            media_temp |= BMCR_FORCE_SPEED_100;
        }
    } else {
        phy_value = (uint16_t)((media_temp & ENET_MAC_CFG_DPM) >> 3);
        phy_value |= (uint16_t)((media_temp & ENET_MAC_CFG_SPD) >> 1);
        phy_state = enet_phy_write_read(ENET_PHY_WRITE, PHY_ADDRESS, PHY_BMCR, &phy_value);
        if(!phy_state) {
            /* return ERROR due to write timeout */
            return enet_state;
        }
        /* PHY configuration need some time */
        _ENET_DELAY_(PHY_CONFIGDELAY);
    }
    /* after configuring the PHY, use mediamode to configure registers */
    reg_value = ENET_MAC_CFG;
    /* configure ENET_MAC_CFG register */
    reg_value &= (~(ENET_MAC_CFG_SPD | ENET_MAC_CFG_DPM | ENET_MAC_CFG_LBM));
    reg_value |= media_temp;
    ENET_MAC_CFG = reg_value;


    /* 2st, configure checksum */
    if(RESET != ((uint32_t)checksum & ENET_CHECKSUMOFFLOAD_ENABLE)) {
        ENET_MAC_CFG |= ENET_CHECKSUMOFFLOAD_ENABLE;

        reg_value = ENET_DMA_CTL;
        /* configure ENET_DMA_CTL register */
        reg_value &= ~ENET_DMA_CTL_DTCERFD;
        reg_value |= ((uint32_t)checksum & ENET_DMA_CTL_DTCERFD);
        ENET_DMA_CTL = reg_value;
    }

    /* 3rd, configure recept */
    ENET_MAC_FRMF |= (uint32_t)recept;

    /* 4th, configure different function options */
    /* configure forward_frame related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)FORWARD_OPTION)) {
        reg_temp = enet_initpara.forward_frame;

        reg_value = ENET_MAC_CFG;
        temp = reg_temp;
        /* configure ENET_MAC_CFG register */
        reg_value &= (~(ENET_MAC_CFG_TFCD | ENET_MAC_CFG_APCD));
        temp &= (ENET_MAC_CFG_TFCD | ENET_MAC_CFG_APCD);
        reg_value |= temp;
        ENET_MAC_CFG = reg_value;

        reg_value = ENET_DMA_CTL;
        temp = reg_temp;
        /* configure ENET_DMA_CTL register */
        reg_value &= (~(ENET_DMA_CTL_FERF | ENET_DMA_CTL_FUF));
        temp &= ((ENET_DMA_CTL_FERF | ENET_DMA_CTL_FUF) << 2);
        reg_value |= (temp >> 2);
        ENET_DMA_CTL = reg_value;
    }

    /* configure dmabus_mode related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)DMABUS_OPTION)) {
        temp = enet_initpara.dmabus_mode;

        reg_value = ENET_DMA_BCTL;
        /* configure ENET_DMA_BCTL register */
        reg_value &= ~(ENET_DMA_BCTL_AA | ENET_DMA_BCTL_FB \
                       | ENET_DMA_BCTL_FPBL | ENET_DMA_BCTL_MB);
        reg_value |= temp;
        ENET_DMA_BCTL = reg_value;
    }

    /* configure dma_maxburst related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)DMA_MAXBURST_OPTION)) {
        temp = enet_initpara.dma_maxburst;

        reg_value = ENET_DMA_BCTL;
        /* configure ENET_DMA_BCTL register */
        reg_value &= ~(ENET_DMA_BCTL_RXDP | ENET_DMA_BCTL_PGBL | ENET_DMA_BCTL_UIP);
        reg_value |= temp;
        ENET_DMA_BCTL = reg_value;
    }

    /* configure dma_arbitration related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)DMA_ARBITRATION_OPTION)) {
        temp = enet_initpara.dma_arbitration;

        reg_value = ENET_DMA_BCTL;
        /* configure ENET_DMA_BCTL register */
        reg_value &= ~(ENET_DMA_BCTL_RTPR | ENET_DMA_BCTL_DAB);
        reg_value |= temp;
        ENET_DMA_BCTL = reg_value;
    }

    /* configure store_forward_mode related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)STORE_OPTION)) {
        temp = enet_initpara.store_forward_mode;

        reg_value = ENET_DMA_CTL;
        /* configure ENET_DMA_CTL register */
        reg_value &= ~(ENET_DMA_CTL_RSFD | ENET_DMA_CTL_TSFD | ENET_DMA_CTL_RTHC | ENET_DMA_CTL_TTHC);
        reg_value |= temp;
        ENET_DMA_CTL = reg_value;
    }

    /* configure dma_function related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)DMA_OPTION)) {
        reg_temp = enet_initpara.dma_function;

        reg_value = ENET_DMA_CTL;
        temp = reg_temp;
        /* configure ENET_DMA_CTL register */
        reg_value &= (~(ENET_DMA_CTL_DAFRF | ENET_DMA_CTL_OSF));
        temp &= (ENET_DMA_CTL_DAFRF | ENET_DMA_CTL_OSF);
        reg_value |= temp;
        ENET_DMA_CTL = reg_value;

        reg_value = ENET_DMA_BCTL;
        temp = reg_temp;
        /* configure ENET_DMA_BCTL register */
        reg_value &= (~ENET_DMA_BCTL_DFM);
        temp &= ENET_DMA_BCTL_DFM;
        reg_value |= temp;
        ENET_DMA_BCTL = reg_value;
    }

    /* configure vlan_config related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)VLAN_OPTION)) {
        reg_temp = enet_initpara.vlan_config;

        reg_value = ENET_MAC_VLT;
        /* configure ENET_MAC_VLT register */
        reg_value &= ~(ENET_MAC_VLT_VLTI | ENET_MAC_VLT_VLTC);
        reg_value |= reg_temp;
        ENET_MAC_VLT = reg_value;
    }

    /* configure flow_control related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)FLOWCTL_OPTION)) {
        reg_temp = enet_initpara.flow_control;

        reg_value = ENET_MAC_FCTL;
        temp = reg_temp;
        /* configure ENET_MAC_FCTL register */
        reg_value &= ~(ENET_MAC_FCTL_PTM | ENET_MAC_FCTL_DZQP | ENET_MAC_FCTL_PLTS \
                       | ENET_MAC_FCTL_UPFDT | ENET_MAC_FCTL_RFCEN | ENET_MAC_FCTL_TFCEN);
        temp &= (ENET_MAC_FCTL_PTM | ENET_MAC_FCTL_DZQP | ENET_MAC_FCTL_PLTS \
                 | ENET_MAC_FCTL_UPFDT | ENET_MAC_FCTL_RFCEN | ENET_MAC_FCTL_TFCEN);
        reg_value |= temp;
        ENET_MAC_FCTL = reg_value;

        reg_value = ENET_MAC_FCTH;
        temp = reg_temp;
        /* configure ENET_MAC_FCTH register */
        reg_value &= ~(ENET_MAC_FCTH_RFA | ENET_MAC_FCTH_RFD);
        temp &= ((ENET_MAC_FCTH_RFA | ENET_MAC_FCTH_RFD) << 8);
        reg_value |= (temp >> 8);
        ENET_MAC_FCTH = reg_value;
    }

    /* configure hashtable_high related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)HASHH_OPTION)) {
        ENET_MAC_HLH = enet_initpara.hashtable_high;
    }

    /* configure hashtable_low related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)HASHL_OPTION)) {
        ENET_MAC_HLL = enet_initpara.hashtable_low;
    }

    /* configure framesfilter_mode related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)FILTER_OPTION)) {
        reg_temp = enet_initpara.framesfilter_mode;

        reg_value = ENET_MAC_FRMF;
        /* configure ENET_MAC_FRMF register */
        reg_value &= ~(ENET_MAC_FRMF_SAFLT | ENET_MAC_FRMF_SAIFLT | ENET_MAC_FRMF_DAIFLT \
                       | ENET_MAC_FRMF_HMF | ENET_MAC_FRMF_HPFLT | ENET_MAC_FRMF_MFD \
                       | ENET_MAC_FRMF_HUF | ENET_MAC_FRMF_PCFRM);
        reg_value |= reg_temp;
        ENET_MAC_FRMF = reg_value;
    }

    /* configure halfduplex_param related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)HALFDUPLEX_OPTION)) {
        reg_temp = enet_initpara.halfduplex_param;

        reg_value = ENET_MAC_CFG;
        /* configure ENET_MAC_CFG register */
        reg_value &= ~(ENET_MAC_CFG_CSD | ENET_MAC_CFG_ROD | ENET_MAC_CFG_RTD \
                       | ENET_MAC_CFG_BOL | ENET_MAC_CFG_DFC);
        reg_value |= reg_temp;
        ENET_MAC_CFG = reg_value;
    }

    /* configure timer_config related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)TIMER_OPTION)) {
        reg_temp = enet_initpara.timer_config;

        reg_value = ENET_MAC_CFG;
        /* configure ENET_MAC_CFG register */
        reg_value &= ~(ENET_MAC_CFG_WDD | ENET_MAC_CFG_JBD);
        reg_value |= reg_temp;
        ENET_MAC_CFG = reg_value;
    }

    /* configure interframegap related registers */
    if(RESET != (enet_initpara.option_enable & (uint32_t)INTERFRAMEGAP_OPTION)) {
        reg_temp = enet_initpara.interframegap;

        reg_value = ENET_MAC_CFG;
        /* configure ENET_MAC_CFG register */
        reg_value &= ~ENET_MAC_CFG_IGBS;
        reg_value |= reg_temp;
        ENET_MAC_CFG = reg_value;
    }

    enet_state = SUCCESS;
    return enet_state;
}

static void enet_delay(uint32_t ncount)
{
    __IO uint32_t delay_time = 0U;

    for(delay_time = ncount; delay_time != 0U; delay_time--) {
    }
}

static void enet_default_init(void)
{
    uint32_t reg_value = 0U;

    /* MAC */
    /* configure ENET_MAC_CFG register */
    reg_value = ENET_MAC_CFG;
    reg_value &= MAC_CFG_MASK;
    reg_value |= ENET_WATCHDOG_ENABLE | ENET_JABBER_ENABLE | ENET_INTERFRAMEGAP_96BIT \
                 | ENET_SPEEDMODE_10M | ENET_MODE_HALFDUPLEX | ENET_LOOPBACKMODE_DISABLE \
                 | ENET_CARRIERSENSE_ENABLE | ENET_RECEIVEOWN_ENABLE \
                 | ENET_RETRYTRANSMISSION_ENABLE | ENET_BACKOFFLIMIT_10 \
                 | ENET_DEFERRALCHECK_DISABLE \
                 | ENET_TYPEFRAME_CRC_DROP_DISABLE \
                 | ENET_AUTO_PADCRC_DROP_DISABLE \
                 | ENET_CHECKSUMOFFLOAD_DISABLE;
    ENET_MAC_CFG = reg_value;

    /* configure ENET_MAC_FRMF register */
    ENET_MAC_FRMF = ENET_SRC_FILTER_DISABLE | ENET_DEST_FILTER_INVERSE_DISABLE \
                    | ENET_MULTICAST_FILTER_PERFECT | ENET_UNICAST_FILTER_PERFECT \
                    | ENET_PCFRM_PREVENT_ALL | ENET_BROADCASTFRAMES_ENABLE \
                    | ENET_PROMISCUOUS_DISABLE | ENET_RX_FILTER_ENABLE;

    /* configure ENET_MAC_HLH, ENET_MAC_HLL register */
    ENET_MAC_HLH = 0x0U;

    ENET_MAC_HLL = 0x0U;

    /* configure ENET_MAC_FCTL, ENET_MAC_FCTH register */
    reg_value = ENET_MAC_FCTL;
    reg_value &= MAC_FCTL_MASK;
    reg_value |= MAC_FCTL_PTM(0) | ENET_ZERO_QUANTA_PAUSE_DISABLE \
                 | ENET_PAUSETIME_MINUS4 | ENET_UNIQUE_PAUSEDETECT \
                 | ENET_RX_FLOWCONTROL_DISABLE | ENET_TX_FLOWCONTROL_DISABLE;
    ENET_MAC_FCTL = reg_value;

    /* configure ENET_MAC_VLT register */
    ENET_MAC_VLT = ENET_VLANTAGCOMPARISON_16BIT | MAC_VLT_VLTI(0);

    /* disable MAC interrupt */
    ENET_MAC_INTMSK |= ENET_MAC_INTMSK_TMSTIM | ENET_MAC_INTMSK_WUMIM;

    /* MSC */
    /* disable MSC Rx interrupt */
    ENET_MSC_RINTMSK |= ENET_MSC_RINTMSK_RFAEIM | ENET_MSC_RINTMSK_RFCEIM \
                        | ENET_MSC_RINTMSK_RGUFIM;

    /* disable MSC Tx interrupt */
    ENET_MSC_TINTMSK |= ENET_MSC_TINTMSK_TGFIM | ENET_MSC_TINTMSK_TGFMSCIM \
                        | ENET_MSC_TINTMSK_TGFSCIM;

    /* DMA */
    /* configure ENET_DMA_CTL register */
    reg_value = ENET_DMA_CTL;
    reg_value &= DMA_CTL_MASK;
    reg_value |= ENET_TCPIP_CKSUMERROR_DROP | ENET_RX_MODE_STOREFORWARD \
                 | ENET_FLUSH_RXFRAME_ENABLE | ENET_TX_MODE_STOREFORWARD \
                 | ENET_TX_THRESHOLD_64BYTES | ENET_RX_THRESHOLD_64BYTES \
                 | ENET_SECONDFRAME_OPT_DISABLE;
    ENET_DMA_CTL = reg_value;

    /* configure ENET_DMA_BCTL register */
    reg_value = ENET_DMA_BCTL;
    reg_value &= DMA_BCTL_MASK;
    reg_value = ENET_ADDRESS_ALIGN_ENABLE | ENET_ARBITRATION_RXTX_2_1 \
                | ENET_RXDP_32BEAT | ENET_PGBL_32BEAT | ENET_RXTX_DIFFERENT_PGBL \
                | ENET_FIXED_BURST_ENABLE | ENET_MIXED_BURST_DISABLE \
                | ENET_NORMAL_DESCRIPTOR;
    ENET_DMA_BCTL = reg_value;
}

