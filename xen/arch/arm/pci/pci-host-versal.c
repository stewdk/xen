/*
 * Based on xen/arch/arm/pci/pci-host-zynqmp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <xen/bitops.h>
#include <xen/init.h>
#include <xen/pci.h>
#include <xen/vmap.h>
#include <asm/device.h>
#include <asm/io.h>
#include <asm/pci.h>

/* TODO */
#undef BIT
#define BIT(x) (1UL << (x))

/* Copied from Linux drivers/pci/controller/pcie-xilinx-cpm.c */

/* Register definitions */
#define XILINX_CPM_PCIE_REG_IDR         0x00000E10
#define XILINX_CPM_PCIE_REG_IMR         0x00000E14
#define XILINX_CPM_PCIE_REG_PSCR        0x00000E1C
#define XILINX_CPM_PCIE_REG_RPSC        0x00000E20
#define XILINX_CPM_PCIE_REG_RPEFR       0x00000E2C
#define XILINX_CPM_PCIE_REG_IDRN        0x00000E38
#define XILINX_CPM_PCIE_REG_IDRN_MASK   0x00000E3C
#define XILINX_CPM_PCIE_MISC_IR_STATUS  0x00000340
#define XILINX_CPM_PCIE_MISC_IR_ENABLE  0x00000348
#define XILINX_CPM_PCIE_MISC_IR_LOCAL   BIT(1)

#define XILINX_CPM_PCIE_IR_STATUS       0x000002A0
#define XILINX_CPM_PCIE_IR_ENABLE       0x000002A8
#define XILINX_CPM_PCIE_IR_LOCAL        BIT(0)

/* Interrupt registers definitions */
#define XILINX_CPM_PCIE_INTR_LINK_DOWN          0
#define XILINX_CPM_PCIE_INTR_HOT_RESET          3
#define XILINX_CPM_PCIE_INTR_CFG_PCIE_TIMEOUT   4
#define XILINX_CPM_PCIE_INTR_CFG_TIMEOUT        8
#define XILINX_CPM_PCIE_INTR_CORRECTABLE        9
#define XILINX_CPM_PCIE_INTR_NONFATAL           10
#define XILINX_CPM_PCIE_INTR_FATAL              11
#define XILINX_CPM_PCIE_INTR_CFG_ERR_POISON     12
#define XILINX_CPM_PCIE_INTR_PME_TO_ACK_RCVD    15
#define XILINX_CPM_PCIE_INTR_INTX               16
#define XILINX_CPM_PCIE_INTR_PM_PME_RCVD        17
#define XILINX_CPM_PCIE_INTR_SLV_UNSUPP         20
#define XILINX_CPM_PCIE_INTR_SLV_UNEXP          21
#define XILINX_CPM_PCIE_INTR_SLV_COMPL          22
#define XILINX_CPM_PCIE_INTR_SLV_ERRP           23
#define XILINX_CPM_PCIE_INTR_SLV_CMPABT         24
#define XILINX_CPM_PCIE_INTR_SLV_ILLBUR         25
#define XILINX_CPM_PCIE_INTR_MST_DECERR         26
#define XILINX_CPM_PCIE_INTR_MST_SLVERR         27
#define XILINX_CPM_PCIE_INTR_SLV_PCIE_TIMEOUT   28

#define IMR(x) BIT(XILINX_CPM_PCIE_INTR_ ##x)

#define XILINX_CPM_PCIE_IMR_ALL_MASK                    \
        (                                               \
                IMR(LINK_DOWN)          |               \
                IMR(HOT_RESET)          |               \
                IMR(CFG_PCIE_TIMEOUT)   |               \
                IMR(CFG_TIMEOUT)        |               \
                IMR(CORRECTABLE)        |               \
                IMR(NONFATAL)           |               \
                IMR(FATAL)              |               \
                IMR(CFG_ERR_POISON)     |               \
                IMR(PME_TO_ACK_RCVD)    |               \
                IMR(INTX)               |               \
                IMR(PM_PME_RCVD)        |               \
                IMR(SLV_UNSUPP)         |               \
                IMR(SLV_UNEXP)          |               \
                IMR(SLV_COMPL)          |               \
                IMR(SLV_ERRP)           |               \
                IMR(SLV_CMPABT)         |               \
                IMR(SLV_ILLBUR)         |               \
                IMR(MST_DECERR)         |               \
                IMR(MST_SLVERR)         |               \
                IMR(SLV_PCIE_TIMEOUT)                   \
        )

#define XILINX_CPM_PCIE_IDR_ALL_MASK            0xFFFFFFFF
#define XILINX_CPM_PCIE_IDRN_MASK               GENMASK(19, 16)
#define XILINX_CPM_PCIE_IDRN_SHIFT              16

/* Root Port Error FIFO Read Register definitions */
#define XILINX_CPM_PCIE_RPEFR_ERR_VALID         BIT(18)
#define XILINX_CPM_PCIE_RPEFR_REQ_ID            GENMASK(15, 0)
#define XILINX_CPM_PCIE_RPEFR_ALL_MASK          0xFFFFFFFF

/* Root Port Status/control Register definitions */
#define XILINX_CPM_PCIE_REG_RPSC_BEN            BIT(0)

/* Phy Status/Control Register definitions */
#define XILINX_CPM_PCIE_REG_PSCR_LNKUP          BIT(11)

static int __init cpm_cfg_reg_index(struct dt_device_node *np)
{
    return dt_property_match_string(np, "reg-names", "cfg");
}

/* ECAM ops */
static const struct pci_ecam_ops cpm_pcie_ops = {
    .bus_shift  = 20,
    .cfg_reg_index = cpm_cfg_reg_index,
    .pci_ops    = {
        .map_bus                = pci_ecam_map_bus,
        .read                   = pci_generic_config_read,
        .write                  = pci_generic_config_write,
        .need_p2m_hwdom_mapping = pci_ecam_need_p2m_hwdom_mapping,
    }
};

static const struct dt_device_match __initconstrel cpm_pcie_dt_match[] =
{
    { .compatible = "xlnx,versal-cpm-host-1.00" },
    { }
};

static bool cpm_pcie_link_up(pci_sbdf_t port)
{
    return (pci_conf_read32(port, XILINX_CPM_PCIE_REG_PSCR) &
            XILINX_CPM_PCIE_REG_PSCR_LNKUP);
}

static void cpm_pcie_init_port(pci_sbdf_t port, void __iomem *cpm_base)
{
    if ( cpm_pcie_link_up(port) )
        printk(XENLOG_INFO "PCIe Link is UP\n");
    else
        printk(XENLOG_INFO "PCIe Link is DOWN\n");

    /* Disable all interrupts */
    pci_conf_write32(port, XILINX_CPM_PCIE_REG_IMR,
                     ~XILINX_CPM_PCIE_IDR_ALL_MASK);

    /* Clear pending interrupts */
    pci_conf_write32(port, XILINX_CPM_PCIE_REG_IDR,
                     pci_conf_read32(port, XILINX_CPM_PCIE_REG_IDR) &
                     XILINX_CPM_PCIE_IMR_ALL_MASK);

    /*
     * XILINX_CPM_PCIE_MISC_IR_ENABLE register is mapped to
     * CPM SLCR block.
     */
    writel(XILINX_CPM_PCIE_MISC_IR_LOCAL,
           cpm_base + XILINX_CPM_PCIE_MISC_IR_ENABLE);

    /* Enable the Bridge enable bit */
    pci_conf_write32(port, XILINX_CPM_PCIE_REG_RPSC,
                     pci_conf_read32(port, XILINX_CPM_PCIE_REG_RPSC) |
                     XILINX_CPM_PCIE_REG_RPSC_BEN);
}

static int __init pci_host_generic_probe(struct dt_device_node *dev,
                                         const void *data)
{
    int rv, cpm_reg_idx;
    paddr_t addr, size;
    void __iomem *cpm_base;
    uint16_t segment;
    struct pci_host_bridge *bridge;
    pci_sbdf_t sbdf;

    rv = pci_host_common_probe(dev, &cpm_pcie_ops);
    if ( rv )
        return rv;

    rv = pci_get_host_bridge_segment(dev, &segment);
    if ( rv )
        return rv;

    sbdf = PCI_SBDF(segment, 0, 0, 0);

    bridge = pci_find_host_bridge(segment, 0);
    /* TODO: check error */

    cpm_reg_idx = dt_property_match_string(dev, "reg-names", "cpm_slcr");
    if ( cpm_reg_idx < 0 )
        return cpm_reg_idx;

    rv = dt_device_get_paddr(dev, cpm_reg_idx, &addr, &size);
    if ( rv )
        return rv;

    cpm_base = ioremap_nocache(addr, size);
    if ( !cpm_base )
        return -ENOMEM;

    cpm_pcie_init_port(sbdf, cpm_base);

    /* Configure host bridge with the bus range */
    pci_conf_write8(sbdf, PCI_PRIMARY_BUS, bridge->cfg->busn_start);
    pci_conf_write8(sbdf, PCI_SECONDARY_BUS, bridge->cfg->busn_start + 1);
    pci_conf_write8(sbdf, PCI_SUBORDINATE_BUS, bridge->cfg->busn_end);

    iounmap(cpm_base);
    return rv;
}

DT_DEVICE_START(pci_gen, "PCI HOST VERSAL", DEVICE_PCI_HOSTBRIDGE)
.dt_match = cpm_pcie_dt_match,
.init = pci_host_generic_probe,
DT_DEVICE_END

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
