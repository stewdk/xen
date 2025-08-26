/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Based on Linux drivers/pci/controller/pci-host-common.c
 * Based on Linux drivers/pci/controller/pci-host-generic.c
 * Based on Linux drivers/pci/controller/dwc/pcie-designware.c
 * Based on xen/arch/arm/pci/pci-host-generic.c
 */

#include <xen/pci.h>
#include <xen/init.h>

#ifndef __PCI_DESIGNWARE_H__
#define __PCI_DESIGNWARE_H__

/* DWC PCIe IP-core versions (native support since v4.70a) */
#define DW_PCIE_VER_365A                0x3336352a
#define DW_PCIE_VER_460A                0x3436302a
#define DW_PCIE_VER_470A                0x3437302a
#define DW_PCIE_VER_480A                0x3438302a
#define DW_PCIE_VER_490A                0x3439302a
#define DW_PCIE_VER_520A                0x3532302a
#define DW_PCIE_VER_540A                0x3534302a

#define __dw_pcie_ver_cmp(_pci, _ver, _op) \
        ((_pci)->version _op DW_PCIE_VER_ ## _ver)

#define dw_pcie_ver_is(_pci, _ver) __dw_pcie_ver_cmp(_pci, _ver, ==)

#define dw_pcie_ver_is_ge(_pci, _ver) __dw_pcie_ver_cmp(_pci, _ver, >=)

#define dw_pcie_ver_type_is(_pci, _ver, _type) \
        (__dw_pcie_ver_cmp(_pci, _ver, ==) && \
         __dw_pcie_ver_cmp(_pci, TYPE_ ## _type, ==))

#define dw_pcie_ver_type_is_ge(_pci, _ver, _type) \
        (__dw_pcie_ver_cmp(_pci, _ver, ==) && \
         __dw_pcie_ver_cmp(_pci, TYPE_ ## _type, >=))

/* DWC PCIe controller capabilities */
#define DW_PCIE_CAP_REQ_RES             0
#define DW_PCIE_CAP_IATU_UNROLL         1
#define DW_PCIE_CAP_CDM_CHECK           2

#define dw_pcie_cap_is(_pci, _cap) \
        test_bit(DW_PCIE_CAP_ ## _cap, &(_pci)->caps)

#define dw_pcie_cap_set(_pci, _cap) \
        set_bit(DW_PCIE_CAP_ ## _cap, &(_pci)->caps)

/* Parameters for the waiting for link up routine */
#define LINK_WAIT_MAX_RETRIES           10
#define LINK_WAIT_SLEEP_MS              90

/* Parameters for the waiting for iATU enabled routine */
#define LINK_WAIT_MAX_IATU_RETRIES      5
#define LINK_WAIT_IATU                  9

/* Synopsys-specific PCIe configuration registers */
#define PCIE_PORT_FORCE                 0x708
#define PORT_FORCE_DO_DESKEW_FOR_SRIS   BIT(23, UL)

#define PCIE_PORT_AFR                   0x70C
#define PORT_AFR_N_FTS_MASK             GENMASK(15, 8)
#define PORT_AFR_N_FTS(n)               FIELD_PREP(PORT_AFR_N_FTS_MASK, n)
#define PORT_AFR_CC_N_FTS_MASK          GENMASK(23, 16)
#define PORT_AFR_CC_N_FTS(n)            FIELD_PREP(PORT_AFR_CC_N_FTS_MASK, n)
#define PORT_AFR_ENTER_ASPM             BIT(30, UL)
#define PORT_AFR_L0S_ENTRANCE_LAT_SHIFT 24
#define PORT_AFR_L0S_ENTRANCE_LAT_MASK  GENMASK(26, 24)
#define PORT_AFR_L1_ENTRANCE_LAT_SHIFT  27
#define PORT_AFR_L1_ENTRANCE_LAT_MASK   GENMASK(29, 27)

#define PCIE_PORT_LINK_CONTROL          0x710
#define PORT_LINK_DLL_LINK_EN           BIT(5, UL)
#define PORT_LINK_FAST_LINK_MODE        BIT(7, UL)
#define PORT_LINK_MODE_MASK             GENMASK(21, 16)
#define PORT_LINK_MODE(n)               FIELD_PREP(PORT_LINK_MODE_MASK, n)
#define PORT_LINK_MODE_1_LANES          PORT_LINK_MODE(0x1)
#define PORT_LINK_MODE_2_LANES          PORT_LINK_MODE(0x3)
#define PORT_LINK_MODE_4_LANES          PORT_LINK_MODE(0x7)
#define PORT_LINK_MODE_8_LANES          PORT_LINK_MODE(0xf)

#define PCIE_PORT_LANE_SKEW             0x714
#define PORT_LANE_SKEW_INSERT_MASK      GENMASK(23, 0)

#define PCIE_PORT_DEBUG0                0x728
#define PORT_LOGIC_LTSSM_STATE_MASK     0x1f
#define PORT_LOGIC_LTSSM_STATE_L0       0x11
#define PCIE_PORT_DEBUG1                0x72C
#define PCIE_PORT_DEBUG1_LINK_UP                BIT(4, UL)
#define PCIE_PORT_DEBUG1_LINK_IN_TRAINING       BIT(29, UL)

#define PCIE_LINK_WIDTH_SPEED_CONTROL   0x80C
#define PORT_LOGIC_N_FTS_MASK           GENMASK(7, 0)
#define PORT_LOGIC_SPEED_CHANGE         BIT(17, UL)
#define PORT_LOGIC_LINK_WIDTH_MASK      GENMASK(12, 8)
#define PORT_LOGIC_LINK_WIDTH(n)        FIELD_PREP(PORT_LOGIC_LINK_WIDTH_MASK, n)
#define PORT_LOGIC_LINK_WIDTH_1_LANES   PORT_LOGIC_LINK_WIDTH(0x1)
#define PORT_LOGIC_LINK_WIDTH_2_LANES   PORT_LOGIC_LINK_WIDTH(0x2)
#define PORT_LOGIC_LINK_WIDTH_4_LANES   PORT_LOGIC_LINK_WIDTH(0x4)
#define PORT_LOGIC_LINK_WIDTH_8_LANES   PORT_LOGIC_LINK_WIDTH(0x8)

#define PCIE_MSI_ADDR_LO                0x820
#define PCIE_MSI_ADDR_HI                0x824
#define PCIE_MSI_INTR0_ENABLE           0x828
#define PCIE_MSI_INTR0_MASK             0x82C
#define PCIE_MSI_INTR0_STATUS           0x830

#define GEN3_RELATED_OFF                        0x890
#define GEN3_RELATED_OFF_GEN3_ZRXDC_NONCOMPL    BIT(0, UL)
#define GEN3_RELATED_OFF_RXEQ_RGRDLESS_RXTS     BIT(13, UL)
#define GEN3_RELATED_OFF_GEN3_EQ_DISABLE        BIT(16, UL)
#define GEN3_RELATED_OFF_RATE_SHADOW_SEL_SHIFT  24
#define GEN3_RELATED_OFF_RATE_SHADOW_SEL_MASK   GENMASK(25, 24)
#define GEN3_RELATED_OFF_RATE_SHADOW_SEL_16_0GT 0x1

#define GEN3_EQ_CONTROL_OFF                     0x8A8
#define GEN3_EQ_CONTROL_OFF_FB_MODE             GENMASK(3, 0)
#define GEN3_EQ_CONTROL_OFF_PHASE23_EXIT_MODE   BIT(4, UL)
#define GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC        GENMASK(23, 8)
#define GEN3_EQ_CONTROL_OFF_FOM_INC_INITIAL_EVAL        BIT(24, UL)

#define GEN3_EQ_FB_MODE_DIR_CHANGE_OFF          0x8AC
#define GEN3_EQ_FMDC_T_MIN_PHASE23              GENMASK(4, 0)
#define GEN3_EQ_FMDC_N_EVALS                    GENMASK(9, 5)
#define GEN3_EQ_FMDC_MAX_PRE_CUSROR_DELTA       GENMASK(13, 10)
#define GEN3_EQ_FMDC_MAX_POST_CUSROR_DELTA      GENMASK(17, 14)

#define PCIE_PORT_MULTI_LANE_CTRL       0x8C0
#define PORT_MLTI_UPCFG_SUPPORT         BIT(7, UL)

#define PCIE_VERSION_NUMBER             0x8F8
#define PCIE_VERSION_TYPE               0x8FC

/*
 * iATU inbound and outbound windows CSRs. Before the IP-core v4.80a each
 * iATU region CSRs had been indirectly accessible by means of the dedicated
 * viewport selector. The iATU/eDMA CSRs space was re-designed in DWC PCIe
 * v4.80a in a way so the viewport was unrolled into the directly accessible
 * iATU/eDMA CSRs space.
 */
#define PCIE_ATU_VIEWPORT               0x900
#define PCIE_ATU_REGION_DIR_IB          BIT(31, UL)
#define PCIE_ATU_REGION_DIR_OB          0
#define PCIE_ATU_VIEWPORT_BASE          0x904
#define PCIE_ATU_UNROLL_BASE(dir, index) \
        (((index) << 9) | ((dir == PCIE_ATU_REGION_DIR_IB) ? BIT(8, UL) : 0))
#define PCIE_ATU_VIEWPORT_SIZE          0x2C
#define PCIE_ATU_REGION_CTRL1           0x000
#define PCIE_ATU_INCREASE_REGION_SIZE   BIT(13, UL)
#define PCIE_ATU_TYPE_MEM               0x0
#define PCIE_ATU_TYPE_IO                0x2
#define PCIE_ATU_TYPE_CFG0              0x4
#define PCIE_ATU_TYPE_CFG1              0x5
#define PCIE_ATU_TYPE_MSG               0x10
#define PCIE_ATU_TD                     BIT(8, UL)
#define PCIE_ATU_FUNC_NUM(pf)           ((pf) << 20)
#define PCIE_ATU_REGION_CTRL2           0x004
#define PCIE_ATU_ENABLE                 BIT(31, UL)
#define PCIE_ATU_BAR_MODE_ENABLE        BIT(30, UL)
#define PCIE_ATU_INHIBIT_PAYLOAD        BIT(22, UL)
#define PCIE_ATU_FUNC_NUM_MATCH_EN      BIT(19, UL)
#define PCIE_ATU_LOWER_BASE             0x008
#define PCIE_ATU_UPPER_BASE             0x00C
#define PCIE_ATU_LIMIT                  0x010
#define PCIE_ATU_LOWER_TARGET           0x014

#define FIELD_PREP(_mask, _val) \
    (((typeof(_mask))(_val) << (ffs64(_mask) - 1)) & (_mask))

#define PCIE_ATU_BUS(x)         FIELD_PREP(GENMASK(31, 24), (x))
#define PCIE_ATU_DEV(x)         FIELD_PREP(GENMASK(23, 19), (x))
#define PCIE_ATU_FUNC(x)        FIELD_PREP(GENMASK(18, 16), (x))
#define PCIE_ATU_UPPER_TARGET           0x018
#define PCIE_ATU_UPPER_LIMIT            0x020

#define PCIE_MISC_CONTROL_1_OFF         0x8BC
#define PCIE_DBI_RO_WR_EN               BIT(0, UL)

#define PCIE_MSIX_DOORBELL              0x948
#define PCIE_MSIX_DOORBELL_PF_SHIFT     24

/*
 * eDMA CSRs. DW PCIe IP-core v4.70a and older had the eDMA registers accessible
 * over the Port Logic registers space. Afterwards the unrolled mapping was
 * introduced so eDMA and iATU could be accessed via a dedicated registers
 * space.
 */
#define PCIE_DMA_VIEWPORT_BASE          0x970
#define PCIE_DMA_UNROLL_BASE            0x80000
#define PCIE_DMA_CTRL                   0x008
#define PCIE_DMA_NUM_WR_CHAN            GENMASK(3, 0)
#define PCIE_DMA_NUM_RD_CHAN            GENMASK(19, 16)

#define PCIE_PL_CHK_REG_CONTROL_STATUS                  0xB20
#define PCIE_PL_CHK_REG_CHK_REG_START                   BIT(0, UL)
#define PCIE_PL_CHK_REG_CHK_REG_CONTINUOUS              BIT(1, UL)
#define PCIE_PL_CHK_REG_CHK_REG_COMPARISON_ERROR        BIT(16, UL)
#define PCIE_PL_CHK_REG_CHK_REG_LOGIC_ERROR             BIT(17, UL)
#define PCIE_PL_CHK_REG_CHK_REG_COMPLETE                BIT(18, UL)

#define PCIE_PL_CHK_REG_ERR_ADDR                        0xB28

/*
 * 16.0 GT/s (Gen 4) lane margining register definitions
 */
#define GEN4_LANE_MARGINING_1_OFF               0xB80
#define MARGINING_MAX_VOLTAGE_OFFSET            GENMASK(29, 24)
#define MARGINING_NUM_VOLTAGE_STEPS             GENMASK(22, 16)
#define MARGINING_MAX_TIMING_OFFSET             GENMASK(13, 8)
#define MARGINING_NUM_TIMING_STEPS              GENMASK(5, 0)

#define GEN4_LANE_MARGINING_2_OFF               0xB84
#define MARGINING_IND_ERROR_SAMPLER             BIT(28, UL)
#define MARGINING_SAMPLE_REPORTING_METHOD       BIT(27, UL)
#define MARGINING_IND_LEFT_RIGHT_TIMING         BIT(26, UL)
#define MARGINING_IND_UP_DOWN_VOLTAGE           BIT(25, UL)
#define MARGINING_VOLTAGE_SUPPORTED             BIT(24, UL)
#define MARGINING_MAXLANES                      GENMASK(20, 16)
#define MARGINING_SAMPLE_RATE_TIMING            GENMASK(13, 8)
#define MARGINING_SAMPLE_RATE_VOLTAGE           GENMASK(5, 0)
/*
 * iATU Unroll-specific register definitions
 * From 4.80 core version the address translation will be made by unroll
 */
#define PCIE_ATU_UNR_REGION_CTRL1       0x00
#define PCIE_ATU_UNR_REGION_CTRL2       0x04
#define PCIE_ATU_UNR_LOWER_BASE         0x08
#define PCIE_ATU_UNR_UPPER_BASE         0x0C
#define PCIE_ATU_UNR_LOWER_LIMIT        0x10
#define PCIE_ATU_UNR_LOWER_TARGET       0x14
#define PCIE_ATU_UNR_UPPER_TARGET       0x18
#define PCIE_ATU_UNR_UPPER_LIMIT        0x20

/*
 * RAS-DES register definitions
 */
#define PCIE_RAS_DES_EVENT_COUNTER_CONTROL      0x8
#define EVENT_COUNTER_ALL_CLEAR         0x3
#define EVENT_COUNTER_ENABLE_ALL        0x7
#define EVENT_COUNTER_ENABLE_SHIFT      2
#define EVENT_COUNTER_EVENT_SEL_MASK    GENMASK(7, 0)
#define EVENT_COUNTER_EVENT_SEL_SHIFT   16
#define EVENT_COUNTER_EVENT_Tx_L0S      0x2
#define EVENT_COUNTER_EVENT_Rx_L0S      0x3
#define EVENT_COUNTER_EVENT_L1          0x5
#define EVENT_COUNTER_EVENT_L1_1        0x7
#define EVENT_COUNTER_EVENT_L1_2        0x8
#define EVENT_COUNTER_GROUP_SEL_SHIFT   24
#define EVENT_COUNTER_GROUP_5           0x5

#define PCIE_RAS_DES_EVENT_COUNTER_DATA         0xc

/*
 * The default address offset between dbi_base and atu_base. Root controller
 * drivers are not required to initialize atu_base if the offset matches this
 * default; the driver core automatically derives atu_base from dbi_base using
 * this offset, if atu_base not set.
 */
#define DEFAULT_DBI_ATU_OFFSET (0x3 << 20)
#define DEFAULT_DBI_DMA_OFFSET PCIE_DMA_UNROLL_BASE

#define MAX_MSI_IRQS                    256
#define MAX_MSI_IRQS_PER_CTRL           32
#define MAX_MSI_CTRLS                   (MAX_MSI_IRQS / MAX_MSI_IRQS_PER_CTRL)
#define MSI_REG_CTRL_BLOCK_SIZE         12
#define MSI_DEF_NUM_VECTORS             32

/* Maximum number of inbound/outbound iATUs */
#define MAX_IATU_IN                     256
#define MAX_IATU_OUT                    256

/* Default eDMA LLP memory size */
#define DMA_LLP_MEM_SIZE                PAGE_SIZE

struct dw_pcie_priv {
    void __iomem *dbi_base;
    size_t        dbi_size;
    void __iomem *atu_base;
    size_t        atu_size;
    uint32_t      num_ob_windows;
    uint32_t      region_align;
    uint64_t      region_limit;
    unsigned int version;
    void *priv;
    unsigned long caps;
    unsigned int  ranges;
};

void *dw_pcie_get_priv(struct pci_host_bridge *bridge);
void dw_pcie_set_priv(struct pci_host_bridge *bridge, void *other);

void dw_pcie_set_version(struct pci_host_bridge *bridge, unsigned int version);

void __iomem *dw_pcie_child_map_bus(struct pci_host_bridge *bridge,
                                    pci_sbdf_t sbdf, uint32_t where);

int dw_pcie_child_config_read(struct pci_host_bridge *bridge, pci_sbdf_t sbdf,
                              uint32_t reg, uint32_t len, uint32_t *value);

int dw_pcie_child_config_write(struct pci_host_bridge *bridge, pci_sbdf_t sbdf,
                               uint32_t reg, uint32_t len, uint32_t value);

bool __init dw_pcie_child_need_p2m_hwdom_mapping(struct domain *d,
                                                 struct pci_host_bridge *bridge,
                                                 uint64_t addr);

struct pci_host_bridge *__init
dw_pcie_host_probe(struct dt_device_node *dev, const void *data,
                   const struct pci_ecam_ops *ops,
                   const struct pci_ecam_ops *child_ops);
#endif /* __PCI_DESIGNWARE_H__ */
