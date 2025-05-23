/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * xen/arch/arm/vpci.c
 */
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/vpci.h>

#include <asm/mmio.h>

static pci_sbdf_t vpci_sbdf_from_gpa(const struct domain *d,
                                     const struct pci_host_bridge *bridge,
                                     paddr_t gpa)
{
    pci_sbdf_t sbdf;

    if ( !has_vpci_bridge(d) )
    {
        sbdf.sbdf = VPCI_ECAM_BDF(gpa - bridge->cfg->phys_addr);
        sbdf.seg = bridge->segment;
        sbdf.bus += bridge->cfg->busn_start;
    }
    else
    {
        paddr_t start = domain_use_host_layout(d) ? bridge->cfg->phys_addr :
                                                    GUEST_VPCI_ECAM_BASE;
        sbdf.sbdf = VPCI_ECAM_BDF(gpa - start);
    }

    return sbdf;
}

static int vpci_mmio_read(struct vcpu *v, mmio_info_t *info,
                          register_t *r, void *p)
{
    struct pci_host_bridge *bridge = p;
    pci_sbdf_t sbdf = vpci_sbdf_from_gpa(v->domain, bridge, info->gpa);
    const unsigned int access_size = (1U << info->dabt.size) * 8;
    const register_t invalid = GENMASK_ULL(access_size - 1, 0);
    /* data is needed to prevent a pointer cast on 32bit */
    unsigned long data;

    if ( vpci_ecam_read(sbdf, ECAM_REG_OFFSET(info->gpa),
                        1U << info->dabt.size, &data) )
    {
        *r = data & invalid;
        return 1;
    }

    *r = invalid;

    return 0;
}

static int vpci_mmio_write(struct vcpu *v, mmio_info_t *info,
                           register_t r, void *p)
{
    struct pci_host_bridge *bridge = p;
    pci_sbdf_t sbdf = vpci_sbdf_from_gpa(v->domain, bridge, info->gpa);

    return vpci_ecam_write(sbdf, ECAM_REG_OFFSET(info->gpa),
                           1U << info->dabt.size, r);
}

static const struct mmio_handler_ops vpci_mmio_handler = {
    .read  = vpci_mmio_read,
    .write = vpci_mmio_write,
};

static int vpci_setup_mmio_handler_cb(struct domain *d,
                                      struct pci_host_bridge *bridge)
{
    struct pci_config_window *cfg = bridge->cfg;

    register_mmio_handler(d, &vpci_mmio_handler,
                          cfg->phys_addr, cfg->size, bridge);

    /* We have registered a single MMIO handler. */
    return 1;
}

int domain_vpci_init(struct domain *d)
{
    if ( !has_vpci(d) )
        return 0;

    /*
     * The hardware domain gets as many MMIOs as required by the
     * physical host bridge.
     * Guests get the virtual platform layout: one virtual host bridge for now.
     */
    if ( !has_vpci_bridge(d) )
    {
        int ret;

        ret = pci_host_iterate_bridges_and_count(d, vpci_setup_mmio_handler_cb);
        if ( ret < 0 )
            return ret;
    }
    else
    {
        if ( !IS_ENABLED(CONFIG_HAS_VPCI_GUEST_SUPPORT) )
        {
            gdprintk(XENLOG_ERR, "vPCI requested but guest support not enabled\n");
            return -EINVAL;
        }
        if ( domain_use_host_layout(d) )
        {
            struct pci_host_bridge *bridge;

            /* XXX: assume physical bridge is segment 0 bus 0 */
            bridge = pci_find_host_bridge(0, 0);
            if ( !bridge )
                return 0;

            register_mmio_handler(d, &vpci_mmio_handler,
                                  bridge->cfg->phys_addr, bridge->cfg->size, bridge);
        }
        else
        {
            register_mmio_handler(d, &vpci_mmio_handler,
                                  GUEST_VPCI_ECAM_BASE, GUEST_VPCI_ECAM_SIZE, NULL);
        }
    }

    return 0;
}

static int vpci_get_num_handlers_cb(struct domain *d,
                                    struct pci_host_bridge *bridge)
{
    /* Each bridge has a single MMIO handler for the configuration space. */
    return 1;
}

unsigned int domain_vpci_get_num_mmio_handlers(struct domain *d)
{
    if ( !has_vpci(d) )
        return 0;

    if ( !has_vpci_bridge(d) )
    {
        int ret = pci_host_iterate_bridges_and_count(d, vpci_get_num_handlers_cb);

        if ( ret < 0 )
        {
            ASSERT_UNREACHABLE();
            return 0;
        }

        return ret;
    }

    /*
     * For guests each host bridge requires one region to cover the
     * configuration space. At the moment, we only expose a single host bridge.
     */
    return 1;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */

