/*
 * Handlers for accesses to the MSI-X capability structure and the memory
 * region.
 *
 * Copyright (C) 2017 Citrix Systems R&D
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms and conditions of the GNU General Public
 * License, version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; If not, see <http://www.gnu.org/licenses/>.
 */

#include <xen/io.h>
#include <xen/msi.h>
#include <xen/sched.h>
#include <xen/vmap.h>
#include <xen/vpci.h>

#include <asm/io.h>
#include <asm/p2m.h>

static uint32_t cf_check control_read(
    const struct pci_dev *pdev, unsigned int reg, void *data)
{
    const struct vpci_msix *msix = data;

    return (msix->max_entries - 1) |
           (msix->enabled ? PCI_MSIX_FLAGS_ENABLE : 0) |
           (msix->masked ? PCI_MSIX_FLAGS_MASKALL : 0);
}

static void update_entry(struct vpci_msix_entry *entry,
                         const struct pci_dev *pdev, unsigned int nr)
{
    int rc = vpci_msix_arch_disable_entry(entry, pdev);

    /* Ignore ENOENT, it means the entry wasn't setup. */
    if ( rc && rc != -ENOENT )
    {
        gprintk(XENLOG_WARNING,
                "%pp: unable to disable entry %u for update: %d\n",
                &pdev->sbdf, nr, rc);
        return;
    }

    rc = vpci_msix_arch_enable_entry(entry, pdev,
                                     vmsix_table_host_base(pdev->vpci,
                                                           VPCI_MSIX_TABLE));
    if ( rc )
    {
        gprintk(XENLOG_WARNING, "%pp: unable to enable entry %u: %d\n",
                &pdev->sbdf, nr, rc);
        /* Entry is likely not properly configured. */
        return;
    }

    entry->updated = false;
}

static void cf_check control_write(
    const struct pci_dev *pdev, unsigned int reg, uint32_t val, void *data)
{
    struct vpci_msix *msix = data;
    bool new_masked = val & PCI_MSIX_FLAGS_MASKALL;
    bool new_enabled = val & PCI_MSIX_FLAGS_ENABLE;
    unsigned int i;
    int rc;

    if ( new_masked == msix->masked && new_enabled == msix->enabled )
        return;

    /*
     * According to the PCI 3.0 specification, switching the enable bit to 1
     * or the function mask bit to 0 should cause all the cached addresses
     * and data fields to be recalculated.
     *
     * In order to avoid the overhead of disabling and enabling all the
     * entries every time the guest sets the maskall bit, Xen will only
     * perform the disable and enable sequence when the guest has written to
     * the entry.
     */
    if ( new_enabled && !new_masked && (!msix->enabled || msix->masked) )
    {
        for ( i = 0; i < msix->max_entries; i++ )
            if ( !msix->entries[i].masked && msix->entries[i].updated )
                update_entry(&msix->entries[i], pdev, i);
    }
    else if ( !new_enabled && msix->enabled )
    {
        /* Guest has disabled MSIX, disable all entries. */
        for ( i = 0; i < msix->max_entries; i++ )
        {
            /*
             * NB: vpci_msix_arch_disable can be called for entries that are
             * not setup, it will return -ENOENT in that case.
             */
            rc = vpci_msix_arch_disable_entry(&msix->entries[i], pdev);
            switch ( rc )
            {
            case 0:
                /*
                 * Mark the entry successfully disabled as updated, so that on
                 * the next enable the entry is properly setup. This is done
                 * so that the following flow works correctly:
                 *
                 * mask entry -> disable MSIX -> enable MSIX -> unmask entry
                 *
                 * Without setting 'updated', the 'unmask entry' step will fail
                 * because the entry has not been updated, so it would not be
                 * mapped/bound at all.
                 */
                msix->entries[i].updated = true;
                break;
            case -ENOENT:
                /* Ignore non-present entry. */
                break;
            default:
                gprintk(XENLOG_WARNING, "%pp: unable to disable entry %u: %d\n",
                        &pdev->sbdf, i, rc);
                return;
            }
        }
    }

    /* Make sure domU doesn't enable INTx while enabling MSI-X. */
    if ( new_enabled && !msix->enabled && !is_hardware_domain(pdev->domain) )
    {
        pci_intx(pdev, false);
        pdev->vpci->header.guest_cmd |= PCI_COMMAND_INTX_DISABLE;
    }

    msix->masked = new_masked;
    msix->enabled = new_enabled;

    val = control_read(pdev, reg, data);
    if ( pci_msi_conf_write_intercept(msix->pdev, reg, 2, &val) >= 0 )
        pci_conf_write16(pdev->sbdf, reg, val);
}

static bool access_allowed(const struct pci_dev *pdev, unsigned long addr,
                           unsigned int len)
{
    /* Only allow aligned 32/64b accesses. */
    if ( (len == 4 || len == 8) && !(addr & (len - 1)) )
        return true;

    gprintk(XENLOG_WARNING,
            "%pp: unaligned or invalid size MSI-X table access\n", &pdev->sbdf);

    return false;
}

static struct vpci_msix_entry *get_entry(struct vpci_msix *msix,
                                         paddr_t addr)
{
    paddr_t start = vmsix_table_addr(msix->pdev->vpci, VPCI_MSIX_TABLE);

    return &msix->entries[(addr - start) / PCI_MSIX_ENTRY_SIZE];
}

static void __iomem *get_table(const struct vpci *vpci, unsigned int slot)
{
    struct vpci_msix *msix = vpci->msix;
    paddr_t addr = 0;

    ASSERT(spin_is_locked(&vpci->lock));

    if ( likely(msix->table[slot]) )
        return msix->table[slot];

    switch ( slot )
    {
    case VPCI_MSIX_TBL_TAIL:
        addr = vmsix_table_size(vpci, VPCI_MSIX_TABLE);
        fallthrough;
    case VPCI_MSIX_TBL_HEAD:
        addr += vmsix_table_host_addr(vpci, VPCI_MSIX_TABLE);
        break;

    case VPCI_MSIX_PBA_TAIL:
        addr = vmsix_table_size(vpci, VPCI_MSIX_PBA);
        fallthrough;
    case VPCI_MSIX_PBA_HEAD:
        addr += vmsix_table_host_addr(vpci, VPCI_MSIX_PBA);
        break;

    default:
        ASSERT_UNREACHABLE();
        return NULL;
    }

    msix->table[slot] = ioremap(round_pgdown(addr), PAGE_SIZE);

    return msix->table[slot];
}

static unsigned int get_slot(const struct vpci *vpci, unsigned long addr)
{
    unsigned long pfn = PFN_DOWN(addr);

    if ( pfn == PFN_DOWN(vmsix_table_addr(vpci, VPCI_MSIX_TABLE)) )
        return VPCI_MSIX_TBL_HEAD;
    if ( pfn == PFN_DOWN(vmsix_table_addr(vpci, VPCI_MSIX_TABLE) +
                         vmsix_table_size(vpci, VPCI_MSIX_TABLE) - 1) )
        return VPCI_MSIX_TBL_TAIL;
    if ( pfn == PFN_DOWN(vmsix_table_addr(vpci, VPCI_MSIX_PBA)) )
        return VPCI_MSIX_PBA_HEAD;
    if ( pfn == PFN_DOWN(vmsix_table_addr(vpci, VPCI_MSIX_PBA) +
                         vmsix_table_size(vpci, VPCI_MSIX_PBA) - 1) )
        return VPCI_MSIX_PBA_TAIL;

    ASSERT_UNREACHABLE();
    return -1;
}

static bool adjacent_handle(const struct vpci_msix *msix, unsigned long addr)
{
    unsigned int i;

    if ( VMSIX_ADDR_IN_RANGE(addr, msix->pdev->vpci, VPCI_MSIX_PBA) )
        return true;

    if ( VMSIX_ADDR_IN_RANGE(addr, msix->pdev->vpci, VPCI_MSIX_TABLE) )
        return false;

    for ( i = 0; i < ARRAY_SIZE(msix->tables); i++ )
        if ( VMSIX_ADDR_SAME_PAGE(addr, msix->pdev->vpci, i) )
            return true;

    return false;
}

static bool adjacent_read(const struct domain *d, const struct vpci_msix *msix,
                         unsigned long addr, unsigned int len,
                         unsigned long *data)
{
    const void __iomem *mem;
    struct vpci *vpci = msix->pdev->vpci;
    unsigned int slot;

    *data = ~0UL;

    if ( !adjacent_handle(msix, addr + len - 1) )
        return true;

    if ( VMSIX_ADDR_IN_RANGE(addr, vpci, VPCI_MSIX_PBA) &&
         !access_allowed(msix->pdev, addr, len) )
        /* PBA accesses must be aligned and 4 or 8 bytes in size. */
        return true;

    slot = get_slot(vpci, addr);
    if ( slot >= ARRAY_SIZE(msix->table) )
        return true;

    if ( unlikely(!IS_ALIGNED(addr, len)) )
    {
        unsigned int i;

        gprintk(XENLOG_DEBUG, "%pp: unaligned read to MSI-X related page\n",
                &msix->pdev->sbdf);

        /*
         * Split unaligned accesses into byte sized ones. Shouldn't happen in
         * the first place, but devices shouldn't have registers in the same 4K
         * page as the MSIX tables either.
         *
         * It's unclear whether this could cause issues if a guest expects
         * registers to be accessed atomically, it better use an aligned access
         * if it has such expectations.
         */
        for ( i = 0; i < len; i++ )
        {
            unsigned long partial = ~0UL;
            int rc = adjacent_read(d, msix, addr + i, 1, &partial);

            if ( rc != true )
                return rc;

            *data &= ~(0xffUL << (i * 8));
            *data |= (partial & 0xff) << (i * 8);
        }

        return true;
    }

    spin_lock(&vpci->lock);
    mem = get_table(vpci, slot);
    if ( !mem )
    {
        spin_unlock(&vpci->lock);
        gprintk(XENLOG_WARNING,
                "%pp: unable to map MSI-X page, returning all bits set\n",
                &msix->pdev->sbdf);
        return true;
    }

    *data = read_mmio(mem + PAGE_OFFSET(addr), len);
    spin_unlock(&vpci->lock);

    return true;
}

bool cf_check vpci_msix_read(struct vpci_msix *msix, unsigned long addr,
    unsigned int len, unsigned long *data)
{
    struct domain *d = msix->pdev->domain;
    const struct vpci_msix_entry *entry;
    unsigned int offset;

    *data = ~0UL;

    read_lock(&d->pci_lock);

    if ( !msix )
    {
        read_unlock(&d->pci_lock);
        return false;
    }

    if ( adjacent_handle(msix, addr) )
    {
        int rc = adjacent_read(d, msix, addr, len, data);

        read_unlock(&d->pci_lock);
        return rc;
    }

    if ( !access_allowed(msix->pdev, addr, len) )
    {
        read_unlock(&d->pci_lock);
        return true;
    }

    spin_lock(&msix->pdev->vpci->lock);
    entry = get_entry(msix, addr);
    offset = addr & (PCI_MSIX_ENTRY_SIZE - 1);

    switch ( offset )
    {
    case PCI_MSIX_ENTRY_LOWER_ADDR_OFFSET:
        *data = entry->addr;
        break;

    case PCI_MSIX_ENTRY_UPPER_ADDR_OFFSET:
        *data = entry->addr >> 32;
        break;

    case PCI_MSIX_ENTRY_DATA_OFFSET:
        *data = entry->data;
        if ( len == 8 )
            *data |=
                (uint64_t)(entry->masked ? PCI_MSIX_VECTOR_BITMASK : 0) << 32;
        break;

    case PCI_MSIX_ENTRY_VECTOR_CTRL_OFFSET:
        *data = entry->masked ? PCI_MSIX_VECTOR_BITMASK : 0;
        break;

    default:
        ASSERT_UNREACHABLE();
        break;
    }
    spin_unlock(&msix->pdev->vpci->lock);
    read_unlock(&d->pci_lock);

    return true;
}

static bool adjacent_write(const struct domain *d, const struct vpci_msix *msix,
                          unsigned long addr, unsigned int len,
                          unsigned long data)
{
    void __iomem *mem;
    struct vpci *vpci = msix->pdev->vpci;
    unsigned int slot;

    if ( !adjacent_handle(msix, addr + len - 1) )
        return true;

    /*
     * Only check start and end of the access because the size of the PBA is
     * assumed to be equal or bigger (8 bytes) than the length of any access
     * handled here.
     */
    if ( VMSIX_ADDR_IN_RANGE(addr, vpci, VPCI_MSIX_PBA) &&
         (!access_allowed(msix->pdev, addr, len) || !is_hardware_domain(d)) )
        /* Ignore writes to PBA for DomUs, it's undefined behavior. */
        return true;

    slot = get_slot(vpci, addr);
    if ( slot >= ARRAY_SIZE(msix->table) )
        return true;

    if ( unlikely(!IS_ALIGNED(addr, len)) )
    {
        unsigned int i;

        gprintk(XENLOG_DEBUG, "%pp: unaligned write to MSI-X related page\n",
                &msix->pdev->sbdf);

        for ( i = 0; i < len; i++ )
        {
            int rc = adjacent_write(d, msix, addr + i, 1, data >> (i * 8));

            if ( rc != true )
                return rc;
        }

        return true;
    }

    spin_lock(&vpci->lock);
    mem = get_table(vpci, slot);
    if ( !mem )
    {
        spin_unlock(&vpci->lock);
        gprintk(XENLOG_WARNING,
                "%pp: unable to map MSI-X page, dropping write\n",
                &msix->pdev->sbdf);
        return true;
    }

    write_mmio(mem + PAGE_OFFSET(addr), data, len);
    spin_unlock(&vpci->lock);

    return true;
}

bool cf_check vpci_msix_write(struct vpci_msix *msix, unsigned long addr,
    unsigned int len, unsigned long data)
{
    struct domain *d = msix->pdev->domain;
    struct vpci_msix_entry *entry;
    unsigned int offset;

    read_lock(&d->pci_lock);

    if ( !msix )
    {
        read_unlock(&d->pci_lock);
        return false;
    }

    if ( adjacent_handle(msix, addr) )
    {
        int rc = adjacent_write(d, msix, addr, len, data);

        read_unlock(&d->pci_lock);
        return rc;
    }

    if ( !access_allowed(msix->pdev, addr, len) )
    {
        read_unlock(&d->pci_lock);
        return true;
    }

    spin_lock(&msix->pdev->vpci->lock);
    entry = get_entry(msix, addr);
    offset = addr & (PCI_MSIX_ENTRY_SIZE - 1);

    /*
     * NB: Xen allows writes to the data/address registers with the entry
     * unmasked. The specification says this is undefined behavior, and Xen
     * implements it as storing the written value, which will be made effective
     * in the next mask/unmask cycle. This also mimics the implementation in
     * QEMU.
     */
    switch ( offset )
    {
    case PCI_MSIX_ENTRY_LOWER_ADDR_OFFSET:
        entry->updated = true;
        if ( len == 8 )
        {
            entry->addr = data;
            break;
        }
        entry->addr &= ~0xffffffffULL;
        entry->addr |= data;
        break;

    case PCI_MSIX_ENTRY_UPPER_ADDR_OFFSET:
        entry->updated = true;
        entry->addr  = (uint32_t)entry->addr;
        entry->addr |= (uint64_t)data << 32;
        break;

    case PCI_MSIX_ENTRY_DATA_OFFSET:
        entry->updated = true;
        entry->data = data;

        if ( len == 4 )
            break;

        data >>= 32;
        /* fallthrough */
    case PCI_MSIX_ENTRY_VECTOR_CTRL_OFFSET:
    {
        bool new_masked = data & PCI_MSIX_VECTOR_BITMASK;
        const struct pci_dev *pdev = msix->pdev;

        if ( entry->masked == new_masked )
            /* No change in the mask bit, nothing to do. */
            break;

        /*
         * Update the masked state before calling vpci_msix_arch_enable_entry,
         * so that it picks the new state.
         */
        entry->masked = new_masked;
        if ( !new_masked && msix->enabled && !msix->masked && entry->updated )
        {
            /*
             * If MSI-X is enabled, the function mask is not active, the entry
             * is being unmasked and there have been changes to the address or
             * data fields Xen needs to disable and enable the entry in order
             * to pick up the changes.
             */
            update_entry(entry, pdev, vmsix_entry_nr(msix, entry));
        }
        else
            vpci_msix_arch_mask_entry(entry, pdev, entry->masked);

        break;
    }

    default:
        ASSERT_UNREACHABLE();
        break;
    }
    spin_unlock(&msix->pdev->vpci->lock);
    read_unlock(&d->pci_lock);

    return true;
}

static int cf_check init_msix(struct pci_dev *pdev)
{
    struct domain *d = pdev->domain;
    unsigned int msix_offset, i, max_entries;
    uint16_t control;
    struct vpci_msix *msix;
    int rc;

    msix_offset = pdev->msix_pos;
    if ( !msix_offset )
        return 0;

    control = pci_conf_read16(pdev->sbdf, msix_control_reg(msix_offset));

    max_entries = msix_table_size(control);

    msix = xzalloc_flex_struct(struct vpci_msix, entries, max_entries);
    if ( !msix )
        return -ENOMEM;

    INIT_LIST_HEAD(&msix->next);

    rc = vpci_add_register(pdev->vpci, control_read, control_write,
                           msix_control_reg(msix_offset), 2, msix);
    if ( rc )
        goto out;

    if ( has_vpci_bridge(d) )
    {
        unsigned long val;

        val = pci_conf_read32(pdev->sbdf, msix_table_offset_reg(msix_offset));
        rc = vpci_add_register(pdev->vpci, vpci_read_val, NULL,
                               msix_table_offset_reg(msix_offset), 4,
                               (void *)(uintptr_t)val);
        if (rc)
        {
            printk("%pd: %pp register msix_table_offset_reg failed\n",
                   d, &pdev->sbdf);
            goto out_control;
        }

        val = pci_conf_read32(pdev->sbdf, msix_pba_offset_reg(msix_offset));
        rc = vpci_add_register(pdev->vpci, vpci_read_val, NULL,
                               msix_pba_offset_reg(msix_offset), 4,
                               (void *)(uintptr_t)val);
        if (rc)
        {
            printk("%pd: %pp register msix_pba_offset failed\n",
                   d, &pdev->sbdf);
            goto out_table;
        }
    }

    msix->max_entries = max_entries;
    msix->pdev = pdev;

    msix->tables[VPCI_MSIX_TABLE] =
        pci_conf_read32(pdev->sbdf, msix_table_offset_reg(msix_offset));
    msix->tables[VPCI_MSIX_PBA] =
        pci_conf_read32(pdev->sbdf, msix_pba_offset_reg(msix_offset));

    for ( i = 0; i < max_entries; i++)
    {
        msix->entries[i].masked = true;
        vpci_msix_arch_init_entry(&msix->entries[i]);
    }

    pdev->vpci->msix = msix;

    vpci_msix_arch_register(msix, d);

    return 0;

 out_table:
    if ( !vpci_remove_register(pdev->vpci,
                               msix_table_offset_reg(msix_offset), 4) )
        printk("%pd: %pp remove msix_table_offset failed\n", d, &pdev->sbdf);

 out_control:
    if ( !vpci_remove_register(pdev->vpci,
                               msix_control_reg(msix_offset), 2) )
        printk("%pd: %pp remove msix_control failed\n", d, &pdev->sbdf);

 out:
    xfree(msix);
    return rc;
}
REGISTER_VPCI_INIT(init_msix, VPCI_PRIORITY_HIGH);

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
