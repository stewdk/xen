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

#include <xen/sched.h>
#include <xen/vpci.h>

#include <asm/msi.h>
#include <asm/p2m.h>

static struct vpci_msix *msix_find(const struct domain *d, unsigned long addr)
{
    struct vpci_msix *msix;

    list_for_each_entry ( msix, &d->arch.hvm.msix_tables, next )
    {
        const struct vpci_bar *bars;
        unsigned int i;

        if ( !msix->pdev->vpci )
            continue;

        bars = msix->pdev->vpci->header.bars;
        for ( i = 0; i < ARRAY_SIZE(msix->tables); i++ )
            if ( bars[msix->tables[i] & PCI_MSIX_BIRMASK].enabled &&
                 VMSIX_ADDR_IN_RANGE(addr, msix->pdev->vpci, i) )
                return msix;
    }

    return NULL;
}

static int cf_check msix_accept(struct vcpu *v, unsigned long addr)
{
    int rc;

    pcidevs_read_lock();
    rc = !!msix_find(v->domain, addr);
    pcidevs_read_unlock();

    return rc;
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

static void __iomem *get_pba(struct vpci *vpci)
{
    struct vpci_msix *msix = vpci->msix;
    /*
     * PBA will only be unmapped when the device is deassigned, so access it
     * without holding the vpci lock.
     */
    void __iomem *pba = read_atomic(&msix->pba);

    if ( likely(pba) )
        return pba;

    pba = ioremap(vmsix_table_addr(vpci, VPCI_MSIX_PBA),
                  vmsix_table_size(vpci, VPCI_MSIX_PBA));
    if ( !pba )
        return read_atomic(&msix->pba);

    spin_lock(&vpci->lock);
    if ( !msix->pba )
    {
        write_atomic(&msix->pba, pba);
        spin_unlock(&vpci->lock);
    }
    else
    {
        spin_unlock(&vpci->lock);
        iounmap(pba);
    }

    return read_atomic(&msix->pba);
}

static int cf_check msix_read(
    struct vcpu *v, unsigned long addr, unsigned int len, unsigned long *data)
{
    const struct domain *d = v->domain;
    struct vpci_msix *msix;
    const struct vpci_msix_entry *entry;
    unsigned int offset;

    *data = ~0ul;

    pcidevs_read_lock();

    msix = msix_find(d, addr);
    if ( !msix )
    {
        pcidevs_read_unlock();
        return X86EMUL_RETRY;
    }

    if ( !access_allowed(msix->pdev, addr, len) )
    {
        pcidevs_read_unlock();
        return X86EMUL_OKAY;
    }

    if ( VMSIX_ADDR_IN_RANGE(addr, msix->pdev->vpci, VPCI_MSIX_PBA) )
    {
        struct vpci *vpci = msix->pdev->vpci;
        unsigned int idx = addr - vmsix_table_addr(vpci, VPCI_MSIX_PBA);
        const void __iomem *pba = get_pba(vpci);

        /*
         * Access to PBA.
         *
         * TODO: note that this relies on having the PBA identity mapped to the
         * guest address space. If this changes the address will need to be
         * translated.
         */
        if ( !pba )
        {
            gprintk(XENLOG_WARNING,
                    "%pp: unable to map MSI-X PBA, report all pending\n",
                    &msix->pdev->sbdf);
            return X86EMUL_OKAY;
        }

        switch ( len )
        {
        case 4:
            *data = readl(pba + idx);
            break;

        case 8:
            *data = readq(pba + idx);
            break;

        default:
            ASSERT_UNREACHABLE();
            break;
        }

        return X86EMUL_OKAY;
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
    pcidevs_read_unlock();

    return X86EMUL_OKAY;
}

static int cf_check msix_write(
    struct vcpu *v, unsigned long addr, unsigned int len, unsigned long data)
{
    const struct domain *d = v->domain;
    struct vpci_msix *msix;
    struct vpci_msix_entry *entry;
    unsigned int offset;

    pcidevs_read_lock();

    msix = msix_find(d, addr);
    if ( !msix )
    {
        pcidevs_read_unlock();
        return X86EMUL_RETRY;
    }

    if ( !access_allowed(msix->pdev, addr, len) )
    {
        pcidevs_read_unlock();
        return X86EMUL_OKAY;
    }

    if ( VMSIX_ADDR_IN_RANGE(addr, msix->pdev->vpci, VPCI_MSIX_PBA) )
    {
        struct vpci *vpci = msix->pdev->vpci;
        unsigned int idx = addr - vmsix_table_addr(vpci, VPCI_MSIX_PBA);
        const void __iomem *pba = get_pba(vpci);

        if ( !is_hardware_domain(d) )
            /* Ignore writes to PBA for DomUs, it's behavior is undefined. */
            return X86EMUL_OKAY;

        if ( !pba )
        {
            /* Unable to map the PBA, ignore write. */
            gprintk(XENLOG_WARNING,
                    "%pp: unable to map MSI-X PBA, write ignored\n",
                    &msix->pdev->sbdf);
            return X86EMUL_OKAY;
        }

        switch ( len )
        {
        case 4:
            writel(data, pba + idx);
            break;

        case 8:
            writeq(data, pba + idx);
            break;

        default:
            ASSERT_UNREACHABLE();
            break;
        }

        pcidevs_read_unlock();
        return X86EMUL_OKAY;
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
        entry->addr &= ~0xffffffffull;
        entry->addr |= data;
        break;

    case PCI_MSIX_ENTRY_UPPER_ADDR_OFFSET:
        entry->updated = true;
        entry->addr &= 0xffffffff;
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
    pcidevs_read_unlock();

    return X86EMUL_OKAY;
}

static const struct hvm_mmio_ops vpci_msix_table_ops = {
    .check = msix_accept,
    .read = msix_read,
    .write = msix_write,
};

int vpci_make_msix_hole(const struct pci_dev *pdev)
{
    struct domain *d;
    unsigned int i;

    ASSERT(pcidevs_read_locked());

    d = pdev->domain;

    if ( !pdev->vpci->msix )
        return 0;

    /* Make sure there's a hole for the MSIX table/PBA in the p2m. */
    for ( i = 0; i < ARRAY_SIZE(pdev->vpci->msix->tables); i++ )
    {
        unsigned long start = PFN_DOWN(vmsix_table_addr(pdev->vpci, i));
        unsigned long end = PFN_DOWN(vmsix_table_addr(pdev->vpci, i) +
                                     vmsix_table_size(pdev->vpci, i) - 1);

        for ( ; start <= end; start++ )
        {
            p2m_type_t t;
            mfn_t mfn = get_gfn_query(d, start, &t);

            switch ( t )
            {
            case p2m_mmio_dm:
            case p2m_invalid:
                break;
            case p2m_mmio_direct:
                if ( mfn_x(mfn) == start )
                {
                    p2m_remove_identity_entry(d, start);
                    break;
                }
                /* fallthrough. */
            default:
                put_gfn(d, start);
                gprintk(XENLOG_WARNING,
                        "%pp: existing mapping (mfn: %" PRI_mfn
                        "type: %d) at %#lx clobbers MSIX MMIO area\n",
                        &pdev->sbdf, mfn_x(mfn), t, start);
                return -EEXIST;
            }
            put_gfn(d, start);
        }
    }

    return 0;
}

void register_msix_mmio_handler(struct domain *d)
{
    if ( list_empty(&d->arch.hvm.msix_tables) )
        register_mmio_handler(d, &vpci_msix_table_ops);
}
void vpci_msix_add_to_msix_table(struct vpci_msix *msix,
                                 struct domain *d)
{
    list_add(&msix->next, &d->arch.hvm.msix_tables);
}
/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
