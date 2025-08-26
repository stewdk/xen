/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Based on Linux drivers/pci/controller/pci-host-common.c
 * Based on Linux drivers/pci/controller/pci-host-generic.c
 * Based on Linux drivers/pci/controller/dwc/pcie-designware.c
 * Based on xen/arch/arm/pci/pci-host-generic.c
 *
 */

#include <xen/delay.h>
#include <xen/pci_ids.h>
#include <xen/sizes.h>
#include <asm/io.h>

#include "pci-designware.h"
/**
 * upper_32_bits - return bits 32-63 of a number
 * @n: the number we're accessing
 *
 * A basic shift-right of a 64- or 32-bit quantity.  Use this to suppress
 * the "right shift count >= width of type" warning when that quantity is
 * 32-bits.
 */
#define upper_32_bits(n) ((uint32_t)(((n) >> 16) >> 16))

/**
 * lower_32_bits - return bits 0-31 of a number
 * @n: the number we're accessing
 */
#define lower_32_bits(n) ((uint32_t)((n) & 0xffffffffU))

static int dw_pcie_read(void __iomem *addr, unsigned int len, uint32_t *val)
{
    if ( !IS_ALIGNED((uintptr_t)addr, len) )
    {
        *val = 0;
        return -EFAULT;
    }

    switch ( len )
    {
    case 1:
        *val = readb(addr);
        break;
    case 2:
        *val = readw(addr);
        break;
    case 4:
        *val = readl(addr);
        break;
    default:
        ASSERT_UNREACHABLE();
    }

    return 0;
}

static int dw_pcie_write(void __iomem *addr, unsigned int len, uint32_t val)
{
    if ( !IS_ALIGNED((uintptr_t)addr, len) )
        return -EFAULT;

    switch ( len )
    {
    case 1:
        writeb(val, addr);
        break;
    case 2:
        writew(val, addr);
        break;
    case 4:
        writel(val, addr);
        break;
    default:
        ASSERT_UNREACHABLE();
    }

    return 0;
}

static uint32_t dw_pcie_read_dbi(struct pci_host_bridge *bridge, uint32_t reg,
                                 size_t size)
{
    void __iomem *addr = bridge->cfg->win + reg;
    uint32_t val;
    int ret;

    ret = dw_pcie_read(addr, size, &val);
    if ( ret )
        printk(XENLOG_G_ERR "Read DBI address failed\n");

    return val;
}

static void dw_pcie_write_dbi(struct pci_host_bridge *bridge, uint32_t reg,
                              size_t size, uint32_t val)
{
    void __iomem *addr = bridge->cfg->win + reg;
    int ret;

    ret = dw_pcie_write(addr, size, val);
    if ( ret )
        printk(XENLOG_G_ERR "Write DBI address failed\n");
}

static uint32_t dw_pcie_readl_dbi(struct pci_host_bridge *bridge, uint32_t reg)
{
    return dw_pcie_read_dbi(bridge, reg, sizeof(uint32_t));
}

static void dw_pcie_writel_dbi(struct pci_host_bridge *pci, uint32_t reg,
                               uint32_t val)
{
    dw_pcie_write_dbi(pci, reg, sizeof(uint32_t), val);
}

static void dw_pcie_writew_dbi(struct pci_host_bridge *pci, uint32_t reg,
                               uint16_t val)
{
    dw_pcie_write_dbi(pci, reg, sizeof(uint16_t), val);
}

static uint32_t dw_pcie_readl_atu(struct pci_host_bridge *pci, uint32_t reg)
{
    struct dw_pcie_priv *priv = pci->priv;
    int ret;
    uint32_t val;

    ret = dw_pcie_read(priv->atu_base + reg, 4, &val);
    if ( ret )
        printk(XENLOG_G_ERR "Read ATU address %x failed\n", reg);

    return val;
}

static void dw_pcie_writel_atu(struct pci_host_bridge *pci, uint32_t reg,
                               uint32_t val)
{
    struct dw_pcie_priv *priv = pci->priv;
    int ret;

    ret = dw_pcie_write(priv->atu_base + reg, 4, val);
    if ( ret )
        printk(XENLOG_G_ERR "Write ATU address %x failed\n", reg);
}

static uint32_t dw_pcie_readl_ob_unroll(struct pci_host_bridge *pci,
                                        uint32_t index, uint32_t reg)
{
    uint32_t offset = PCIE_ATU_UNROLL_BASE(PCIE_ATU_REGION_DIR_OB, index);

    return dw_pcie_readl_atu(pci, offset + reg);
}

static void dw_pcie_writel_ob_unroll(struct pci_host_bridge *pci,
                                     uint32_t index, uint32_t reg, uint32_t val)
{
    uint32_t offset = PCIE_ATU_UNROLL_BASE(PCIE_ATU_REGION_DIR_OB, index);

    dw_pcie_writel_atu(pci, offset + reg, val);
}

static int dw_pcie_prog_outbound_atu_unroll(struct pci_host_bridge *pci,
                                            uint8_t func_no, int index,
                                            int type, uint64_t cpu_addr,
                                            uint64_t pci_addr, uint64_t size)
{
    uint32_t retries, val;
    uint64_t limit_addr = cpu_addr + size - 1;

    dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_LOWER_BASE,
                             lower_32_bits(cpu_addr));
    dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_UPPER_BASE,
                             upper_32_bits(cpu_addr));
    dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_LOWER_LIMIT,
                             lower_32_bits(limit_addr));
    dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_UPPER_LIMIT,
                             upper_32_bits(limit_addr));
    dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_LOWER_TARGET,
                             lower_32_bits(pci_addr));
    dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_UPPER_TARGET,
                             upper_32_bits(pci_addr));
    val = type | PCIE_ATU_FUNC_NUM(func_no);
    val = upper_32_bits(size - 1) ? val | PCIE_ATU_INCREASE_REGION_SIZE : val;
    dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_REGION_CTRL1, val);
    dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_REGION_CTRL2,
                             PCIE_ATU_ENABLE);

    /*
     * Make sure ATU enable takes effect before any subsequent config
     * and I/O accesses.
     */
    for ( retries = 0; retries < LINK_WAIT_MAX_IATU_RETRIES; retries++ )
    {
        val = dw_pcie_readl_ob_unroll(pci, index, PCIE_ATU_UNR_REGION_CTRL2);
        if ( val & PCIE_ATU_ENABLE )
            return 0;

        mdelay(LINK_WAIT_IATU);
    }
    printk(XENLOG_G_ERR "Outbound iATU is not being enabled\n");

    return -ENXIO;
}

static int __dw_pcie_prog_outbound_atu(struct pci_host_bridge *pci,
                                       uint8_t func_no, int index, int type,
                                       uint64_t cpu_addr, uint64_t pci_addr,
                                       uint64_t size)
{
    struct dw_pcie_priv *priv = pci->priv;
    uint32_t retries, val;

    if ( dw_pcie_cap_is(priv, IATU_UNROLL) )
        return dw_pcie_prog_outbound_atu_unroll(pci, func_no, index, type,
                                                cpu_addr, pci_addr, size);

    dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT,
                       PCIE_ATU_REGION_DIR_OB | index);
    dw_pcie_writel_dbi(pci, PCIE_ATU_LOWER_BASE, lower_32_bits(cpu_addr));
    dw_pcie_writel_dbi(pci, PCIE_ATU_UPPER_BASE, upper_32_bits(cpu_addr));
    dw_pcie_writel_dbi(pci, PCIE_ATU_LIMIT, lower_32_bits(cpu_addr + size - 1));
    if ( dw_pcie_ver_is_ge(priv, 460A) )
        dw_pcie_writel_dbi(pci, PCIE_ATU_UPPER_LIMIT,
                           upper_32_bits(cpu_addr + size - 1));
    dw_pcie_writel_dbi(pci, PCIE_ATU_LOWER_TARGET, lower_32_bits(pci_addr));
    dw_pcie_writel_dbi(pci, PCIE_ATU_UPPER_TARGET, upper_32_bits(pci_addr));
    val = type | PCIE_ATU_FUNC_NUM(func_no);
    val = ((upper_32_bits(size - 1)) && dw_pcie_ver_is_ge(priv, 460A))
              ? val | PCIE_ATU_INCREASE_REGION_SIZE
              : val;
    dw_pcie_writel_dbi(pci, PCIE_ATU_REGION_CTRL1, val);
    dw_pcie_writel_dbi(pci, PCIE_ATU_REGION_CTRL2, PCIE_ATU_ENABLE);

    /*
     * Make sure ATU enable takes effect before any subsequent config
     * and I/O accesses.
     */
    for ( retries = 0; retries < LINK_WAIT_MAX_IATU_RETRIES; retries++ )
    {
        val = dw_pcie_readl_dbi(pci, PCIE_ATU_REGION_CTRL2);
        if ( val & PCIE_ATU_ENABLE )
            return 0;

        mdelay(LINK_WAIT_IATU);
    }
    printk(XENLOG_G_ERR "Outbound iATU is not being enabled\n");

    return -ENXIO;
}

static int dw_pcie_prog_outbound_atu(struct pci_host_bridge *pci, int index,
                                     int type, uint64_t cpu_addr,
                                     uint64_t pci_addr, uint64_t size)
{
    return __dw_pcie_prog_outbound_atu(pci, 0, index, type, cpu_addr, pci_addr,
                                       size);
}

void dw_pcie_set_version(struct pci_host_bridge *bridge, unsigned int version)
{
    struct dw_pcie_priv *priv = bridge->priv;

    priv->version = version;
}

static void dw_pcie_version_detect(struct pci_host_bridge *bridge)
{
    struct dw_pcie_priv *pci = bridge->priv;
    uint32_t ver;

    /* The content of the CSR is zero on DWC PCIe older than v4.70a */
    ver = dw_pcie_readl_dbi(bridge, PCIE_VERSION_NUMBER);
    if ( !ver )
        return;

    if ( pci->version && pci->version != ver )
        printk(XENLOG_WARNING "Versions don't match (%08x != %08x)\n",
               pci->version, ver);
    else
        pci->version = ver;
}

void __iomem *dw_pcie_child_map_bus(struct pci_host_bridge *bridge,
                                    pci_sbdf_t sbdf, uint32_t where)
{
    uint32_t busdev;
    int ret;

    busdev = PCIE_ATU_BUS(sbdf.bus) | PCIE_ATU_DEV(PCI_SLOT(sbdf.devfn)) |
             PCIE_ATU_FUNC(PCI_FUNC(sbdf.devfn));

    /* FIXME: Parent is the root bus, so use PCIE_ATU_TYPE_CFG0. */
    ret = dw_pcie_prog_outbound_atu(bridge, 0, PCIE_ATU_TYPE_CFG0,
                                    bridge->child_cfg->phys_addr, busdev,
                                    bridge->child_cfg->size);
    if ( ret )
        return 0;

    return bridge->child_cfg->win + where;
}

int dw_pcie_child_config_read(struct pci_host_bridge *bridge, pci_sbdf_t sbdf,
                              uint32_t reg, uint32_t len, uint32_t *value)
{
    struct dw_pcie_priv *priv = bridge->priv;
    int ret;

    ret = pci_generic_config_read(bridge, sbdf, reg, len, value);
    if ( !ret && (priv->num_ob_windows <= 2) )
        ret = dw_pcie_prog_outbound_atu(bridge, 0, PCIE_ATU_TYPE_IO,
                                        bridge->child_cfg->phys_addr, 0,
                                        bridge->child_cfg->size);

    return ret;
}

int dw_pcie_child_config_write(struct pci_host_bridge *bridge, pci_sbdf_t sbdf,
                               uint32_t reg, uint32_t len, uint32_t value)
{
    struct dw_pcie_priv *priv = bridge->priv;
    int ret;

    ret = pci_generic_config_write(bridge, sbdf, reg, len, value);
    if ( !ret && (priv->num_ob_windows <= 2) )
        ret = dw_pcie_prog_outbound_atu(bridge, 0, PCIE_ATU_TYPE_IO,
                                        bridge->child_cfg->phys_addr, 0,
                                        bridge->child_cfg->size);
    return ret;
}

bool __init dw_pcie_child_need_p2m_hwdom_mapping(struct domain *d,
                                                 struct pci_host_bridge *bridge,
                                                 uint64_t addr)
{
    struct pci_config_window *cfg = bridge->child_cfg;

    /*
     * We do not want ECAM address space to be mapped in Domain-0's p2m,
     * so we can trap access to it.
     */
    return cfg->phys_addr != addr;
}

static int dw_pcie_iatu_detect(struct pci_host_bridge *bridge)
{
    struct dw_pcie_priv *pci = bridge->priv;
    unsigned int max_region, ob;
    uint32_t val, min_limit;
    uint64_t max;

    val = dw_pcie_readl_dbi(bridge, PCIE_ATU_VIEWPORT);
    if ( val == 0xFFFFFFFFU )
    {
        dw_pcie_cap_set(pci, IATU_UNROLL);

        max_region = min((int)pci->atu_size / 512, 256);
    }
    else
    {
        pci->atu_base = pci->dbi_base + PCIE_ATU_VIEWPORT_BASE;
        pci->atu_size = PCIE_ATU_VIEWPORT_SIZE;

        dw_pcie_writel_dbi(bridge, PCIE_ATU_VIEWPORT, 0xFF);
        max_region = dw_pcie_readl_dbi(bridge, PCIE_ATU_VIEWPORT) + 1;
    }

    for ( ob = 0; ob < max_region; ob++ )
    {
        dw_pcie_writel_ob_unroll(bridge, ob, PCIE_ATU_LOWER_TARGET, 0x11110000);
        val = dw_pcie_readl_ob_unroll(bridge, ob, PCIE_ATU_LOWER_TARGET);
        if ( val != 0x11110000 )
            break;
    }

    if ( !ob )
    {
        printk(XENLOG_ERR "No outbound iATU regions found\n");
        return -ENODEV;
    }

    dw_pcie_writel_atu(bridge, PCIE_ATU_LIMIT, 0x0);
    min_limit = dw_pcie_readl_atu(bridge, PCIE_ATU_LIMIT);

    if ( dw_pcie_ver_is_ge(pci, 460A) )
    {
        dw_pcie_writel_atu(bridge, PCIE_ATU_UPPER_LIMIT, 0xFFFFFFFFU);
        max = dw_pcie_readl_atu(bridge, PCIE_ATU_UPPER_LIMIT);
    }
    else
        max = 0;

    pci->num_ob_windows = ob;
    pci->region_align = 1 << fls(min_limit);
    pci->region_limit = (max << 32) | (SZ_4G - 1);

    printk(XENLOG_INFO "iATU: unroll %s, %u ob, align %uK, limit %luG\n",
           dw_pcie_cap_is(pci, IATU_UNROLL) ? "T" : "F",
           pci->num_ob_windows, pci->region_align / SZ_1K,
           (pci->region_limit + 1) / SZ_1G);

    return 0;
}

static void dw_pcie_dbi_ro_wr_en(struct pci_host_bridge *pci)
{
    uint32_t reg;
    uint32_t val;

    reg = PCIE_MISC_CONTROL_1_OFF;
    val = dw_pcie_readl_dbi(pci, reg);
    val |= PCIE_DBI_RO_WR_EN;
    dw_pcie_writel_dbi(pci, reg, val);
}

static void dw_pcie_dbi_ro_wr_dis(struct pci_host_bridge *pci)
{
    uint32_t reg;
    uint32_t val;

    reg = PCIE_MISC_CONTROL_1_OFF;
    val = dw_pcie_readl_dbi(pci, reg);
    val &= ~PCIE_DBI_RO_WR_EN;
    dw_pcie_writel_dbi(pci, reg, val);
}

static int dw_pcie_iatu_setup_range(const struct dt_device_node *dev,
                                    uint32_t flags, uint64_t addr,
                                    uint64_t length, void *data)
{
    struct pci_host_bridge *bridge = data;
    struct dw_pcie_priv *pci = bridge->priv;
    int ret;

    if ( !dt_range_is_memory(flags) )
        return 0;

    if ( pci->num_ob_windows <= ++(pci->ranges) )
        return 1;

    ret = dw_pcie_prog_outbound_atu(bridge, pci->ranges, PCIE_ATU_TYPE_MEM,
                                    addr, addr, length);
    if ( ret )
    {
        printk(XENLOG_ERR "Failed to set MEM range [%#lx-%#lx]\n",
               addr, addr + length - 1);
        return ret;
    }

    return 0;
}

static int dw_pcie_iatu_setup(struct pci_host_bridge *bridge)
{
    struct dw_pcie_priv *pci = bridge->priv;

    /* Note the very first outbound ATU is used for CFG IOs */
    if ( !pci->num_ob_windows )
    {
        printk(XENLOG_ERR "No outbound iATU found\n");
        return -EINVAL;
    }

    dt_for_each_range(bridge->dt_node, dw_pcie_iatu_setup_range, bridge);

    if ( pci->num_ob_windows <= pci->ranges )
        printk(XENLOG_WARNING "Ranges exceed outbound iATU size (%d)\n",
             pci->num_ob_windows);

    return 0;
}

static int dw_pcie_setup_rc(struct pci_host_bridge *pci)
{
    uint32_t val;

    /*
     * Enable DBI read-only registers for writing/updating configuration.
     * Write permission gets disabled towards the end of this function.
     */
    dw_pcie_dbi_ro_wr_en(pci);

    /* Setup RC BARs */
    dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0x00000004);
    dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_1, 0x00000000);

    /* Setup interrupt pins */
    val = dw_pcie_readl_dbi(pci, PCI_INTERRUPT_LINE);
    val &= 0xffff00ffU;
    val |= 0x00000100;
    dw_pcie_writel_dbi(pci, PCI_INTERRUPT_LINE, val);

    /* Setup bus numbers */
    val = dw_pcie_readl_dbi(pci, PCI_PRIMARY_BUS);
    val &= 0xff000000U;
    val |= 0x00ff0100;
    dw_pcie_writel_dbi(pci, PCI_PRIMARY_BUS, val);

    /* Setup command register */
    val = dw_pcie_readl_dbi(pci, PCI_COMMAND);
    val &= 0xffff0000U;
    val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
           PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
    dw_pcie_writel_dbi(pci, PCI_COMMAND, val);

    /*
     * If the platform provides its own child bus config accesses, it means
     * the platform uses its own address translation component rather than
     * ATU, so we should not program the ATU here.
     */
    if ( pci->child_ops->map_bus == dw_pcie_child_map_bus )
    {
        int ret;

        ret = dw_pcie_iatu_setup(pci);
        if ( ret )
            return ret;
    }

    dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0);

    /* Program correct class for RC */
    dw_pcie_writew_dbi(pci, PCI_CLASS_DEVICE, PCI_CLASS_BRIDGE_PCI);

    dw_pcie_dbi_ro_wr_dis(pci);

    return 0;
}

struct pci_host_bridge *__init
dw_pcie_host_probe(struct dt_device_node *dev, const void *data,
                   const struct pci_ecam_ops *ops,
                   const struct pci_ecam_ops *child_ops)
{
    struct pci_host_bridge *bridge;
    struct dw_pcie_priv *priv;

    paddr_t phys_addr;
    paddr_t size;
    int idx, ret;

    bridge = pci_host_common_probe(dev, ops, child_ops);
    if ( IS_ERR(bridge) )
        return bridge;

    priv = xzalloc(struct dw_pcie_priv);
    if ( !priv )
        return ERR_PTR(-ENOMEM);

    bridge->priv = priv;

    idx = dt_property_match_string(dev, "reg-names", "dbi");
    if ( idx < 0 )
    {
        printk(XENLOG_ERR "Cannot find \"dbi\" reg index in device tree\n");
        return ERR_PTR(idx);
    }
    ret = dt_device_get_address(dev, idx, &phys_addr, &size);
    if ( ret )
    {
        printk(XENLOG_ERR "Cannot find \"dbi\" reg in device tree\n");
        return ERR_PTR(ret);
    }
    priv->dbi_base = ioremap_nocache(phys_addr, size);
    if ( !priv->dbi_base )
    {
        printk(XENLOG_ERR "DBI ioremap failed\n");
        return ERR_PTR(ENXIO);
    }
    priv->dbi_size = size;
    printk("DBI at [mem 0x%" PRIpaddr "-0x%" PRIpaddr "]\n", phys_addr,
           phys_addr + size - 1);

    idx = dt_property_match_string(dev, "reg-names", "atu");
    if ( idx < 0 )
    {
        printk(XENLOG_ERR "Cannot find \"atu\" range index in device tree\n");
        return ERR_PTR(idx);
    }
    ret = dt_device_get_address(dev, idx, &phys_addr, &size);
    if ( ret )
    {
        printk(XENLOG_ERR "Cannot find \"atu\" range in device tree\n");
        return ERR_PTR(ret);
    }
    printk("iATU at [mem 0x%" PRIpaddr "-0x%" PRIpaddr "]\n", phys_addr,
           phys_addr + size - 1);
    priv->atu_base = ioremap_nocache(phys_addr, size);
    if ( !priv->atu_base )
    {
        printk(XENLOG_ERR "iATU ioremap failed\n");
        return ERR_PTR(ENXIO);
    }
    priv->atu_size = size;

    dw_pcie_version_detect(bridge);

    ret = dw_pcie_iatu_detect(bridge);
    if ( ret )
        return ERR_PTR(ret);

    ret = dw_pcie_setup_rc(bridge);
    if ( ret )
        return ERR_PTR(ret);

    /* iATU unroll enable will be read on first config space access */

    return bridge;
}

void *dw_pcie_get_priv(struct pci_host_bridge *bridge)
{
    struct dw_pcie_priv *priv = bridge->priv;
    return priv->priv;
}

void dw_pcie_set_priv(struct pci_host_bridge *bridge, void *other)
{
    struct dw_pcie_priv *priv = bridge->priv;
    priv->priv = other;
}
