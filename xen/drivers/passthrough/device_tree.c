/*
 * Code to passthrough a device tree node to a guest
 *
 * Julien Grall <julien.grall@linaro.org>
 * Copyright (c) 2014 Linaro Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <xen/device_tree.h>
#include <xen/guest_access.h>
#include <xen/iommu.h>
#include <xen/lib.h>
#include <xen/sched.h>
#include <xsm/xsm.h>

#include <asm/iommu_fwspec.h>

static spinlock_t dtdevs_lock = SPIN_LOCK_UNLOCKED;

int iommu_assign_dt_device(struct domain *d, struct dt_device_node *dev)
{
    int rc = -EBUSY;
    struct domain_iommu *hd = dom_iommu(d);

    ASSERT(system_state < SYS_STATE_active || rw_is_locked(&dt_host_lock));

    if ( !is_iommu_enabled(d) )
        return -EINVAL;

    if ( !device_is_protected(dt_to_dev(dev)) )
        return -EINVAL;

    spin_lock(&dtdevs_lock);

    if ( !list_empty(&dev->domain_list) )
        goto fail;

    /* The flag field doesn't matter to DT device. */
    rc = hd->platform_ops->assign_device(d, 0, dt_to_dev(dev), 0);

    if ( rc )
        goto fail;

    list_add(&dev->domain_list, &hd->dt_devices);
    dt_device_set_used_by(dev, d->domain_id);

fail:
    spin_unlock(&dtdevs_lock);

    return rc;
}

int iommu_deassign_dt_device(struct domain *d, struct dt_device_node *dev)
{
    const struct domain_iommu *hd = dom_iommu(d);
    int rc;

    ASSERT(rw_is_locked(&dt_host_lock));

    if ( !is_iommu_enabled(d) )
        return -EINVAL;

    if ( !device_is_protected(dt_to_dev(dev)) )
        return -EINVAL;

    spin_lock(&dtdevs_lock);

    rc = hd->platform_ops->reassign_device(d, NULL, 0, dt_to_dev(dev));
    if ( rc )
        goto fail;

    list_del_init(&dev->domain_list);
    dt_device_set_used_by(dev, DOMID_IO);

fail:
    spin_unlock(&dtdevs_lock);

    return rc;
}

static bool iommu_dt_device_is_assigned_locked(const struct dt_device_node *dev)
{
    bool assigned = false;

    ASSERT(spin_is_locked(&dtdevs_lock));

    if ( !device_is_protected(dt_to_dev(dev)) )
        return 0;

    assigned = !list_empty(&dev->domain_list);

    return assigned;
}

int iommu_dt_domain_init(struct domain *d)
{
    INIT_LIST_HEAD(&dom_iommu(d)->dt_devices);

    return 0;
}

int iommu_release_dt_devices(struct domain *d)
{
    const struct domain_iommu *hd = dom_iommu(d);
    struct dt_device_node *dev, *_dev;
    int rc;

    if ( !is_iommu_enabled(d) )
        return 0;

    read_lock(&dt_host_lock);

    list_for_each_entry_safe(dev, _dev, &hd->dt_devices, domain_list)
    {
        rc = iommu_deassign_dt_device(d, dev);
        if ( rc )
        {
            dprintk(XENLOG_ERR, "Failed to deassign %s in domain %u\n",
                    dt_node_full_name(dev), d->domain_id);
            read_unlock(&dt_host_lock);

            return rc;
        }
    }

    read_unlock(&dt_host_lock);

    return 0;
}

/* This correlation must not be altered */
#define NO_IOMMU    1

static int iommu_dt_xlate(struct device *dev,
                          const struct dt_phandle_args *iommu_spec)
{
    const struct iommu_ops *ops = iommu_get_ops();
    int rc;

    if ( !ops->dt_xlate )
        return -EINVAL;

    if ( !dt_device_is_available(iommu_spec->np) )
        return NO_IOMMU;

    rc = iommu_fwspec_init(dev, &iommu_spec->np->dev);
    if ( rc )
        return rc;

    /*
     * Provide DT IOMMU specifier which describes the IOMMU master
     * interfaces of that device (device IDs, etc) to the driver.
     * The driver is responsible to decide how to interpret them.
     */
    return ops->dt_xlate(dev, iommu_spec);
}

#ifdef CONFIG_HAS_PCI
int dt_map_id(const struct dt_device_node *np, uint32_t id,
              const char *map_name, const char *map_mask_name,
              struct dt_device_node **target, uint32_t *id_out)
{
    uint32_t map_mask, masked_id, map_len;
    const __be32 *map = NULL;

    if ( !np || !map_name || (!target && !id_out) )
        return -EINVAL;

    map = dt_get_property(np, map_name, &map_len);
    if ( !map )
    {
        if ( target )
            return -ENODEV;

        /* Otherwise, no map implies no translation */
        *id_out = id;
        return 0;
    }

    if ( !map_len || (map_len % (4 * sizeof(*map))) )
    {
        printk(XENLOG_ERR "%s: Error: Bad %s length: %u\n", np->full_name,
               map_name, map_len);
        return -EINVAL;
    }

    /* The default is to select all bits. */
    map_mask = 0xffffffffU;

    /*
     * Can be overridden by "{iommu,msi}-map-mask" property.
     * If df_property_read_u32() fails, the default is used.
     */
    if ( map_mask_name )
        dt_property_read_u32(np, map_mask_name, &map_mask);

    masked_id = map_mask & id;
    for ( ; (int)map_len > 0; map_len -= 4 * sizeof(*map), map += 4 )
    {
        struct dt_device_node *phandle_node;
        uint32_t id_base = be32_to_cpup(map + 0);
        uint32_t phandle = be32_to_cpup(map + 1);
        uint32_t out_base = be32_to_cpup(map + 2);
        uint32_t id_len = be32_to_cpup(map + 3);

        if ( id_base & ~map_mask )
        {
            printk(XENLOG_ERR "%s: Invalid %s translation - %s-mask (0x%"PRIx32") ignores id-base (0x%"PRIx32")\n",
                   np->full_name, map_name, map_name, map_mask, id_base);
            return -EFAULT;
        }

        if ( (masked_id < id_base) || (masked_id >= (id_base + id_len)) )
            continue;

        phandle_node = dt_find_node_by_phandle(phandle);
        if ( !phandle_node )
            return -ENODEV;

        if ( target )
        {
            if ( !*target )
                *target = phandle_node;

            if ( *target != phandle_node )
                continue;
        }

        if ( id_out )
            *id_out = masked_id - id_base + out_base;

        printk(XENLOG_DEBUG "%s: %s, using mask %08"PRIx32", id-base: %08"PRIx32", out-base: %08"PRIx32", length: %08"PRIx32", id: %08"PRIx32" -> %08"PRIx32"\n",
               np->full_name, map_name, map_mask, id_base, out_base, id_len, id,
               masked_id - id_base + out_base);
        return 0;
    }

    printk(XENLOG_ERR "%s: no %s translation for id 0x%"PRIx32" on %s\n",
           np->full_name, map_name, id, (target && *target) ? (*target)->full_name : NULL);

    /*
     * NOTE: Linux bypasses translation without returning an error here,
     * but should we behave in the same way on Xen? Restrict for now.
     */
    return -EFAULT;
}

int iommu_add_dt_pci_sideband_ids(struct pci_dev *pdev)
{
    const struct iommu_ops *ops = iommu_get_ops();
    struct dt_phandle_args iommu_spec = { .args_count = 1 };
    struct device *dev = pci_to_dev(pdev);
    const struct dt_device_node *np;
    int rc = NO_IOMMU;
    unsigned int devfn = pdev->devfn;

    if ( !iommu_enabled )
        return NO_IOMMU;

    if ( !ops )
        return -EINVAL;

    if ( dev_iommu_fwspec_get(dev) )
        return 0;

    np = pci_find_host_bridge_node(pdev);
    if ( !np )
        return -ENODEV;

    do {
        /*
         * According to the Documentation/devicetree/bindings/pci/pci-iommu.txt
         * from Linux.
         */
        rc = dt_map_id(np, PCI_BDF(pdev->bus, devfn), "iommu-map",
                       "iommu-map-mask", &iommu_spec.np, iommu_spec.args);
        if ( rc )
            return (rc == -ENODEV) ? NO_IOMMU : rc;

        rc = iommu_dt_xlate(dev, &iommu_spec);
        if ( rc < 0 )
        {
            iommu_fwspec_free(dev);
            return -EINVAL;
        }

        devfn += pdev->phantom_stride;
    }
    while ( (devfn != pdev->devfn) &&
            (PCI_SLOT(devfn) == PCI_SLOT(pdev->devfn)) );

    return rc;
}
#endif /* CONFIG_HAS_PCI */

int iommu_remove_dt_device(struct dt_device_node *np)
{
    const struct iommu_ops *ops = iommu_get_ops();
    struct device *dev = dt_to_dev(np);
    int rc;

    ASSERT(rw_is_locked(&dt_host_lock));

    if ( !iommu_enabled )
        return NO_IOMMU;

    if ( !ops )
        return -EOPNOTSUPP;

    spin_lock(&dtdevs_lock);

    if ( iommu_dt_device_is_assigned_locked(np) )
    {
        rc = -EBUSY;
        goto fail;
    }

    if ( !ops->remove_device )
    {
        rc = -EOPNOTSUPP;
        goto fail;
    }

    /*
     * De-register the device from the IOMMU driver.
     * The driver is responsible for removing is_protected flag.
     */
    rc = ops->remove_device(0, dev);

    if ( !rc )
    {
        ASSERT(!device_is_protected(dev));
        iommu_fwspec_free(dev);
    }

 fail:
    spin_unlock(&dtdevs_lock);
    return rc;
}

int iommu_add_dt_device(struct dt_device_node *np)
{
    const struct iommu_ops *ops = iommu_get_ops();
    struct dt_phandle_args iommu_spec;
    struct device *dev = dt_to_dev(np);
    int rc = NO_IOMMU, index = 0;

    ASSERT(system_state < SYS_STATE_active || rw_is_locked(&dt_host_lock));

    if ( !iommu_enabled )
        return NO_IOMMU;

    if ( !ops )
        return -EINVAL;

    /*
     * Devices that appear in the legacy mmu-masters list may have already been
     * registered with the SMMU. In case a device has both a mmu-masters entry
     * and iommus property, there is no need to register it again. In this case
     * simply return success early.
     */
    if ( dev_iommu_fwspec_get(dev) )
        return 0;

    spin_lock(&dtdevs_lock);

    /*
     * According to the Documentation/devicetree/bindings/iommu/iommu.txt
     * from Linux.
     */
    while ( !dt_parse_phandle_with_args(np, "iommus", "#iommu-cells",
                                        index, &iommu_spec) )
    {
        /*
         * The driver which supports generic IOMMU DT bindings must have
         * these callback implemented.
         */
        if ( !ops->add_device )
        {
            rc = -EINVAL;
            goto fail;
        }

        rc = iommu_dt_xlate(dev, &iommu_spec);
        if ( rc )
            break;

        index++;
    }

    /*
     * Add master device to the IOMMU if latter is present and available.
     * The driver is responsible to mark that device as protected.
     */
    if ( !rc )
        rc = ops->add_device(0, dev);

    if ( rc < 0 )
        iommu_fwspec_free(dev);

 fail:
    spin_unlock(&dtdevs_lock);
    return rc;
}

int iommu_do_dt_domctl(struct xen_domctl *domctl, struct domain *d,
                       XEN_GUEST_HANDLE_PARAM(xen_domctl_t) u_domctl)
{
    int ret;
    struct dt_device_node *dev;

    read_lock(&dt_host_lock);

    switch ( domctl->cmd )
    {
    case XEN_DOMCTL_assign_device:
        ASSERT(d);
        /* fall through */
    case XEN_DOMCTL_test_assign_device:
        ret = -ENODEV;
        if ( domctl->u.assign_device.dev != XEN_DOMCTL_DEV_DT )
            break;

        ret = -EINVAL;
        if ( (d && d->is_dying) || domctl->u.assign_device.flags )
            break;

        ret = dt_find_node_by_gpath(domctl->u.assign_device.u.dt.path,
                                    domctl->u.assign_device.u.dt.size,
                                    &dev);
        if ( ret )
            break;

        ret = xsm_assign_dtdevice(XSM_HOOK, d, dt_node_full_name(dev));
        if ( ret )
            break;

        if ( domctl->cmd == XEN_DOMCTL_test_assign_device )
        {
            spin_lock(&dtdevs_lock);

            if ( iommu_dt_device_is_assigned_locked(dev) )
            {
                printk(XENLOG_G_ERR "%s already assigned.\n",
                       dt_node_full_name(dev));
                ret = -EINVAL;
            }

            spin_unlock(&dtdevs_lock);
            break;
        }

        if ( d == dom_io )
        {
            ret = -EINVAL;
            break;
        }

        ret = iommu_add_dt_device(dev);
        if ( ret < 0 )
        {
            printk(XENLOG_G_ERR "Failed to add %s to the IOMMU\n",
                   dt_node_full_name(dev));
            break;
        }

        ret = iommu_assign_dt_device(d, dev);

        if ( ret )
            printk(XENLOG_G_ERR "XEN_DOMCTL_assign_dt_device: assign \"%s\""
                   " to dom%u failed (%d)\n",
                   dt_node_full_name(dev), d->domain_id, ret);
        break;

    case XEN_DOMCTL_deassign_device:
        ret = -ENODEV;
        if ( domctl->u.assign_device.dev != XEN_DOMCTL_DEV_DT )
            break;

        ret = -EINVAL;
        if ( domctl->u.assign_device.flags )
            break;

        ret = dt_find_node_by_gpath(domctl->u.assign_device.u.dt.path,
                                    domctl->u.assign_device.u.dt.size,
                                    &dev);
        if ( ret )
            break;

        ret = xsm_deassign_dtdevice(XSM_HOOK, d, dt_node_full_name(dev));
        if ( ret )
            break;

        if ( d == dom_io )
        {
            ret = -EINVAL;
            break;
        }

        ret = iommu_deassign_dt_device(d, dev);

        if ( ret )
            printk(XENLOG_G_ERR "XEN_DOMCTL_assign_dt_device: assign \"%s\""
                   " to dom%u failed (%d)\n",
                   dt_node_full_name(dev), d->domain_id, ret);
        break;

    default:
        ret = -ENOSYS;
        break;
    }

    read_unlock(&dt_host_lock);

    return ret;
}
