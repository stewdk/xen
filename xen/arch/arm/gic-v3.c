/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * xen/arch/arm/gic-v3.c
 *
 * ARM Generic Interrupt Controller support v3 version
 * based on xen/arch/arm/gic-v2.c and kernel GICv3 driver
 *
 * Copyright (C) 2012,2013 - ARM Ltd
 * Marc Zyngier <marc.zyngier@arm.com>
 *
 * Vijaya Kumar K <vijaya.kumar@caviumnetworks.com>, Cavium Inc
 * ported to Xen
 */

#include <xen/acpi.h>
#include <xen/delay.h>
#include <xen/device_tree.h>
#include <xen/errno.h>
#include <xen/init.h>
#include <xen/iocap.h>
#include <xen/irq.h>
#include <xen/lib.h>
#include <xen/libfdt/libfdt.h>
#include <xen/mm.h>
#include <xen/sched.h>
#include <xen/sizes.h>

#include <asm/cpufeature.h>
#include <asm/device.h>
#include <asm/gic.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic_v3_its.h>
#include <asm/io.h>
#include <asm/pci.h>
#include <asm/sysregs.h>

/* Global state */
static struct {
    void __iomem *map_dbase;  /* Mapped address of distributor registers */
    struct rdist_region *rdist_regions;
    uint32_t  rdist_stride;
    unsigned int rdist_count; /* Number of rdist regions count */
    unsigned int nr_priorities;
    spinlock_t lock;
} gicv3;

static struct gic_info gicv3_info;

/* per-cpu re-distributor base */
static DEFINE_PER_CPU(void __iomem*, rbase);

#define GICD                   (gicv3.map_dbase)
#define GICD_RDIST_BASE        (this_cpu(rbase))
#define GICD_RDIST_SGI_BASE    (GICD_RDIST_BASE + SZ_64K)

/*
 * Saves all 16(Max) LR registers. Though number of LRs implemented
 * is implementation specific.
 */
static inline void gicv3_save_lrs(struct vcpu *v)
{
    /* Fall through for all the cases */
    switch ( gicv3_info.nr_lrs )
    {
    case 16:
        v->arch.gic.v3.lr[15] = READ_SYSREG_LR(15);
        fallthrough;
    case 15:
        v->arch.gic.v3.lr[14] = READ_SYSREG_LR(14);
        fallthrough;
    case 14:
        v->arch.gic.v3.lr[13] = READ_SYSREG_LR(13);
        fallthrough;
    case 13:
        v->arch.gic.v3.lr[12] = READ_SYSREG_LR(12);
        fallthrough;
    case 12:
        v->arch.gic.v3.lr[11] = READ_SYSREG_LR(11);
        fallthrough;
    case 11:
        v->arch.gic.v3.lr[10] = READ_SYSREG_LR(10);
        fallthrough;
    case 10:
        v->arch.gic.v3.lr[9] = READ_SYSREG_LR(9);
        fallthrough;
    case 9:
        v->arch.gic.v3.lr[8] = READ_SYSREG_LR(8);
        fallthrough;
    case 8:
        v->arch.gic.v3.lr[7] = READ_SYSREG_LR(7);
        fallthrough;
    case 7:
        v->arch.gic.v3.lr[6] = READ_SYSREG_LR(6);
        fallthrough;
    case 6:
        v->arch.gic.v3.lr[5] = READ_SYSREG_LR(5);
        fallthrough;
    case 5:
        v->arch.gic.v3.lr[4] = READ_SYSREG_LR(4);
        fallthrough;
    case 4:
        v->arch.gic.v3.lr[3] = READ_SYSREG_LR(3);
        fallthrough;
    case 3:
        v->arch.gic.v3.lr[2] = READ_SYSREG_LR(2);
        fallthrough;
    case 2:
        v->arch.gic.v3.lr[1] = READ_SYSREG_LR(1);
        fallthrough;
    case 1:
         v->arch.gic.v3.lr[0] = READ_SYSREG_LR(0);
         break;
    default:
         BUG();
    }
}

/*
 * Restores all 16(Max) LR registers. Though number of LRs implemented
 * is implementation specific.
 */
static inline void gicv3_restore_lrs(const struct vcpu *v)
{
    /* Fall through for all the cases */
    switch ( gicv3_info.nr_lrs )
    {
    case 16:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[15], 15);
        fallthrough;
    case 15:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[14], 14);
        fallthrough;
    case 14:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[13], 13);
        fallthrough;
    case 13:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[12], 12);
        fallthrough;
    case 12:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[11], 11);
        fallthrough;
    case 11:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[10], 10);
        fallthrough;
    case 10:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[9], 9);
        fallthrough;
    case 9:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[8], 8);
        fallthrough;
    case 8:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[7], 7);
        fallthrough;
    case 7:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[6], 6);
        fallthrough;
    case 6:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[5], 5);
        fallthrough;
    case 5:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[4], 4);
        fallthrough;
    case 4:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[3], 3);
        fallthrough;
    case 3:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[2], 2);
        fallthrough;
    case 2:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[1], 1);
        fallthrough;
    case 1:
        WRITE_SYSREG_LR(v->arch.gic.v3.lr[0], 0);
        break;
    default:
         BUG();
    }
}

static uint64_t gicv3_ich_read_lr(int lr)
{
    switch ( lr )
    {
    case 0: return READ_SYSREG_LR(0);
    case 1: return READ_SYSREG_LR(1);
    case 2: return READ_SYSREG_LR(2);
    case 3: return READ_SYSREG_LR(3);
    case 4: return READ_SYSREG_LR(4);
    case 5: return READ_SYSREG_LR(5);
    case 6: return READ_SYSREG_LR(6);
    case 7: return READ_SYSREG_LR(7);
    case 8: return READ_SYSREG_LR(8);
    case 9: return READ_SYSREG_LR(9);
    case 10: return READ_SYSREG_LR(10);
    case 11: return READ_SYSREG_LR(11);
    case 12: return READ_SYSREG_LR(12);
    case 13: return READ_SYSREG_LR(13);
    case 14: return READ_SYSREG_LR(14);
    case 15: return READ_SYSREG_LR(15);
    default:
        BUG();
    }
}

static void gicv3_ich_write_lr(int lr, uint64_t val)
{
    switch ( lr )
    {
    case 0:
        WRITE_SYSREG_LR(val, 0);
        break;
    case 1:
        WRITE_SYSREG_LR(val, 1);
        break;
    case 2:
        WRITE_SYSREG_LR(val, 2);
        break;
    case 3:
        WRITE_SYSREG_LR(val, 3);
        break;
    case 4:
        WRITE_SYSREG_LR(val, 4);
        break;
    case 5:
        WRITE_SYSREG_LR(val, 5);
        break;
    case 6:
        WRITE_SYSREG_LR(val, 6);
        break;
    case 7:
        WRITE_SYSREG_LR(val, 7);
        break;
    case 8:
        WRITE_SYSREG_LR(val, 8);
        break;
    case 9:
        WRITE_SYSREG_LR(val, 9);
        break;
    case 10:
        WRITE_SYSREG_LR(val, 10);
        break;
    case 11:
        WRITE_SYSREG_LR(val, 11);
        break;
    case 12:
        WRITE_SYSREG_LR(val, 12);
        break;
    case 13:
        WRITE_SYSREG_LR(val, 13);
        break;
    case 14:
        WRITE_SYSREG_LR(val, 14);
        break;
    case 15:
        WRITE_SYSREG_LR(val, 15);
        break;
    default:
        return;
    }
    isb();
}

/*
 * System Register Enable (SRE). Enable to access CPU & Virtual
 * interface registers as system registers in EL2
 */
static void gicv3_enable_sre(void)
{
    register_t val;

    val = READ_SYSREG(ICC_SRE_EL2);
    val |= GICC_SRE_EL2_SRE;

    WRITE_SYSREG(val, ICC_SRE_EL2);
    isb();
}

/* Wait for completion of a distributor change */
static void gicv3_do_wait_for_rwp(void __iomem *base)
{
    uint32_t val;
    bool timeout = false;
    s_time_t deadline = NOW() + MILLISECS(1000);

    do {
        val = readl_relaxed(base + GICD_CTLR);
        if ( !(val & GICD_CTLR_RWP) )
            break;
        if ( NOW() > deadline )
        {
            timeout = true;
            break;
        }
        cpu_relax();
        udelay(1);
    } while ( 1 );

    if ( timeout )
        dprintk(XENLOG_ERR, "RWP timeout\n");
}

static void gicv3_dist_wait_for_rwp(void)
{
    gicv3_do_wait_for_rwp(GICD);
}

static void gicv3_redist_wait_for_rwp(void)
{
    gicv3_do_wait_for_rwp(GICD_RDIST_BASE);
}

static void gicv3_wait_for_rwp(int irq)
{
    if ( irq < NR_LOCAL_IRQS )
         gicv3_redist_wait_for_rwp();
    else
         gicv3_dist_wait_for_rwp();
}

static unsigned int gicv3_get_cpu_from_mask(const cpumask_t *cpumask)
{
    unsigned int cpu;
    cpumask_t possible_mask;

    cpumask_and(&possible_mask, cpumask, &cpu_possible_map);
    cpu = cpumask_any(&possible_mask);

    return cpu;
}

static void restore_aprn_regs(const union gic_state_data *d)
{
    /* Write APRn register based on number of priorities
       platform has implemented */
    switch ( gicv3.nr_priorities )
    {
    case 7:
        WRITE_SYSREG(d->v3.apr0[2], ICH_AP0R2_EL2);
        WRITE_SYSREG(d->v3.apr1[2], ICH_AP1R2_EL2);
        /* Fall through */
    case 6:
        WRITE_SYSREG(d->v3.apr0[1], ICH_AP0R1_EL2);
        WRITE_SYSREG(d->v3.apr1[1], ICH_AP1R1_EL2);
        /* Fall through */
    case 5:
        WRITE_SYSREG(d->v3.apr0[0], ICH_AP0R0_EL2);
        WRITE_SYSREG(d->v3.apr1[0], ICH_AP1R0_EL2);
        break;
    default:
        BUG();
    }
}

static void save_aprn_regs(union gic_state_data *d)
{
    /* Read APRn register based on number of priorities
       platform has implemented */
    switch ( gicv3.nr_priorities )
    {
    case 7:
        d->v3.apr0[2] = READ_SYSREG(ICH_AP0R2_EL2);
        d->v3.apr1[2] = READ_SYSREG(ICH_AP1R2_EL2);
        /* Fall through */
    case 6:
        d->v3.apr0[1] = READ_SYSREG(ICH_AP0R1_EL2);
        d->v3.apr1[1] = READ_SYSREG(ICH_AP1R1_EL2);
        /* Fall through */
    case 5:
        d->v3.apr0[0] = READ_SYSREG(ICH_AP0R0_EL2);
        d->v3.apr1[0] = READ_SYSREG(ICH_AP1R0_EL2);
        break;
    default:
        BUG();
    }
}

/*
 * As per section 4.8.17 of the GICv3 spec following
 * registers are save and restored on guest swap
 */
static void gicv3_save_state(struct vcpu *v)
{

    /* No need for spinlocks here because interrupts are disabled around
     * this call and it only accesses struct vcpu fields that cannot be
     * accessed simultaneously by another pCPU.
     *
     * Make sure all stores to the GIC via the memory mapped interface
     * are now visible to the system register interface
     */
    dsb(sy);
    gicv3_save_lrs(v);
    save_aprn_regs(&v->arch.gic);
    v->arch.gic.v3.vmcr = READ_SYSREG(ICH_VMCR_EL2);
    v->arch.gic.v3.sre_el1 = READ_SYSREG(ICC_SRE_EL1);
}

static void gicv3_restore_state(const struct vcpu *v)
{
    register_t val;

    val = READ_SYSREG(ICC_SRE_EL2);
    /*
     * Don't give access to system registers when the guest is using
     * GICv2
     */
    if ( v->domain->arch.vgic.version == GIC_V2 )
        val &= ~GICC_SRE_EL2_ENEL1;
    else
        val |= GICC_SRE_EL2_ENEL1;
    WRITE_SYSREG(val, ICC_SRE_EL2);

    /*
     * VFIQEn is RES1 if ICC_SRE_EL1.SRE is 1. This causes a Group0
     * interrupt (as generated in GICv2 mode) to be delivered as a FIQ
     * to the guest, with potentially consequence. So we must make sure
     * that ICC_SRE_EL1 has been actually programmed with the value we
     * want before starting to mess with the rest of the GIC, and
     * VMCR_EL1 in particular.
     */
    WRITE_SYSREG(v->arch.gic.v3.sre_el1, ICC_SRE_EL1);
    isb();
    WRITE_SYSREG(v->arch.gic.v3.vmcr, ICH_VMCR_EL2);
    restore_aprn_regs(&v->arch.gic);
    gicv3_restore_lrs(v);

    /*
     * Make sure all stores are visible the GIC
     */
    dsb(sy);
}

static void gicv3_dump_state(const struct vcpu *v)
{
    int i;

    if ( v == current )
    {
        for ( i = 0; i < gicv3_info.nr_lrs; i++ )
            printk("   HW_LR[%d]=%" PRIx64 "\n", i, gicv3_ich_read_lr(i));
    }
    else
    {
        for ( i = 0; i < gicv3_info.nr_lrs; i++ )
            printk("   VCPU_LR[%d]=%" PRIx64 "\n", i, v->arch.gic.v3.lr[i]);
    }
}

static void gicv3_poke_irq(struct irq_desc *irqd, u32 offset, bool wait_for_rwp)
{
    u32 mask = 1U << (irqd->irq % 32);
    void __iomem *base;

    if ( irqd->irq < NR_GIC_LOCAL_IRQS )
        base = GICD_RDIST_SGI_BASE;
    else
        base = GICD;

    writel_relaxed(mask, base + offset + (irqd->irq / 32) * 4);

    if ( wait_for_rwp )
        gicv3_wait_for_rwp(irqd->irq);
}

static bool gicv3_peek_irq(struct irq_desc *irqd, u32 offset)
{
    void __iomem *base;
    unsigned int irq = irqd->irq;

    if ( irq >= NR_GIC_LOCAL_IRQS)
        base = GICD + (irq / 32) * 4;
    else
        base = GICD_RDIST_SGI_BASE;

    return !!(readl(base + offset) & (1U << (irq % 32)));
}

static void gicv3_unmask_irq(struct irq_desc *irqd)
{
    gicv3_poke_irq(irqd, GICD_ISENABLER, false);
}

static void gicv3_mask_irq(struct irq_desc *irqd)
{
    gicv3_poke_irq(irqd, GICD_ICENABLER, true);
}

static void gicv3_eoi_irq(struct irq_desc *irqd)
{
    /* Lower the priority */
    WRITE_SYSREG(irqd->irq, ICC_EOIR1_EL1);
    isb();
}

static void gicv3_dir_irq(struct irq_desc *irqd)
{
    /* Deactivate */
    WRITE_SYSREG(irqd->irq, ICC_DIR_EL1);
    isb();
}

static unsigned int gicv3_read_irq(void)
{
    register_t irq = READ_SYSREG(ICC_IAR1_EL1);

    dsb(sy);

    /* IRQs are encoded using 23bit. */
    return (irq & GICC_IAR_INTID_MASK);
}

/*
 * This is forcing the active state of an interrupt, somewhat circumventing
 * the normal interrupt flow and the GIC state machine. So use with care
 * and only if you know what you are doing. For this reason we also have to
 * tinker with the _IRQ_INPROGRESS bit here, since the normal IRQ handler
 * will not be involved.
 */
static void gicv3_set_active_state(struct irq_desc *irqd, bool active)
{
    ASSERT(spin_is_locked(&irqd->lock));

    if ( active )
    {
        set_bit(_IRQ_INPROGRESS, &irqd->status);
        gicv3_poke_irq(irqd, GICD_ISACTIVER, false);
    }
    else
    {
        clear_bit(_IRQ_INPROGRESS, &irqd->status);
        gicv3_poke_irq(irqd, GICD_ICACTIVER, false);
    }
}

static void gicv3_set_pending_state(struct irq_desc *irqd, bool pending)
{
    ASSERT(spin_is_locked(&irqd->lock));

    if ( pending )
        /* The _IRQ_INPROGRESS bit will be set when the interrupt fires. */
        gicv3_poke_irq(irqd, GICD_ISPENDR, false);
    else
        /* The _IRQ_INPROGRESS bit will remain unchanged. */
        gicv3_poke_irq(irqd, GICD_ICPENDR, false);
}

static inline uint64_t gicv3_mpidr_to_affinity(int cpu)
{
     uint64_t mpidr = cpu_logical_map(cpu);
     return (
#ifdef CONFIG_ARM_64
             MPIDR_AFFINITY_LEVEL(mpidr, 3) << 32 |
#endif
             MPIDR_AFFINITY_LEVEL(mpidr, 2) << 16 |
             MPIDR_AFFINITY_LEVEL(mpidr, 1) << 8  |
             MPIDR_AFFINITY_LEVEL(mpidr, 0));
}

static void gicv3_set_irq_type(struct irq_desc *desc, unsigned int type)
{
    uint32_t cfg, actual, edgebit;
    void __iomem *base;
    unsigned int irq = desc->irq;

    /* SGI's are always edge-triggered not need to call GICD_ICFGR0 */
    ASSERT(irq >= NR_GIC_SGI);

    spin_lock(&gicv3.lock);

    if ( irq >= NR_GIC_LOCAL_IRQS)
        base = GICD + GICD_ICFGR + (irq / 16) * 4;
    else
        base = GICD_RDIST_SGI_BASE + GICR_ICFGR1;

    cfg = readl_relaxed(base);

    edgebit = 2u << (2 * (irq % 16));
    if ( type & IRQ_TYPE_LEVEL_MASK )
        cfg &= ~edgebit;
    else if ( type & IRQ_TYPE_EDGE_BOTH )
        cfg |= edgebit;

    writel_relaxed(cfg, base);

    actual = readl_relaxed(base);
    if ( ( cfg & edgebit ) ^ ( actual & edgebit ) )
    {
        printk(XENLOG_WARNING "GICv3: WARNING: "
               "CPU%d: Failed to configure IRQ%u as %s-triggered. "
               "H/w forces to %s-triggered.\n",
               smp_processor_id(), desc->irq,
               cfg & edgebit ? "Edge" : "Level",
               actual & edgebit ? "Edge" : "Level");
        desc->arch.type = actual & edgebit ?
            IRQ_TYPE_EDGE_RISING :
            IRQ_TYPE_LEVEL_HIGH;
    }
    spin_unlock(&gicv3.lock);
}

static void gicv3_set_irq_priority(struct irq_desc *desc,
                                   unsigned int priority)
{
    unsigned int irq = desc->irq;

    spin_lock(&gicv3.lock);

    /* Set priority */
    if ( irq < NR_GIC_LOCAL_IRQS )
        writeb_relaxed(priority, GICD_RDIST_SGI_BASE + GICR_IPRIORITYR0 + irq);
    else
        writeb_relaxed(priority, GICD + GICD_IPRIORITYR + irq);

    spin_unlock(&gicv3.lock);
}

static void __init gicv3_dist_init(void)
{
    uint32_t type;
    uint64_t affinity;
    unsigned int nr_lines;
    int i;

    /* Disable the distributor */
    writel_relaxed(0, GICD + GICD_CTLR);

    type = readl_relaxed(GICD + GICD_TYPER);
    nr_lines = 32 * ((type & GICD_TYPE_LINES) + 1);

    if ( type & GICD_TYPE_LPIS )
        gicv3_lpi_init_host_lpis(GICD_TYPE_ID_BITS(type));

    /* Only 1020 interrupts are supported */
    nr_lines = min(1020U, nr_lines);
    gicv3_info.nr_lines = nr_lines;

    printk("GICv3: %d lines, (IID %8.8x).\n",
           nr_lines, readl_relaxed(GICD + GICD_IIDR));

    /* Default all global IRQs to level, active low */
    for ( i = NR_GIC_LOCAL_IRQS; i < nr_lines; i += 16 )
        writel_relaxed(0, GICD + GICD_ICFGR + (i / 16) * 4);

    /* Default priority for global interrupts */
    for ( i = NR_GIC_LOCAL_IRQS; i < nr_lines; i += 4 )
        writel_relaxed(GIC_PRI_IRQ_ALL, GICD + GICD_IPRIORITYR + (i / 4) * 4);

    /* Disable/deactivate all global interrupts */
    for ( i = NR_GIC_LOCAL_IRQS; i < nr_lines; i += 32 )
    {
        writel_relaxed(0xffffffffU, GICD + GICD_ICENABLER + (i / 32) * 4);
        writel_relaxed(0xffffffffU, GICD + GICD_ICACTIVER + (i / 32) * 4);
    }

    /*
     * Configure SPIs as non-secure Group-1. This will only matter
     * if the GIC only has a single security state.
     */
    for ( i = NR_GIC_LOCAL_IRQS; i < nr_lines; i += 32 )
        writel_relaxed(GENMASK(31, 0), GICD + GICD_IGROUPR + (i / 32) * 4);

    gicv3_dist_wait_for_rwp();

    /* Turn on the distributor */
    writel_relaxed(GICD_CTLR_ARE_NS | GICD_CTLR_ENABLE_G1A |
                   GICD_CTLR_ENABLE_G1, GICD + GICD_CTLR);

    /* Route all global IRQs to this CPU */
    affinity = gicv3_mpidr_to_affinity(smp_processor_id());
    /* Make sure we don't broadcast the interrupt */
    affinity &= ~GICD_IROUTER_SPI_MODE_ANY;

    for ( i = NR_GIC_LOCAL_IRQS; i < nr_lines; i++ )
        writeq_relaxed_non_atomic(affinity, GICD + GICD_IROUTER + i * 8);
}

static int gicv3_enable_redist(void)
{
    uint32_t val;
    bool timeout = false;
    s_time_t deadline = NOW() + MILLISECS(1000);

    /* Wake up this CPU redistributor */
    val = readl_relaxed(GICD_RDIST_BASE + GICR_WAKER);
    val &= ~GICR_WAKER_ProcessorSleep;
    writel_relaxed(val, GICD_RDIST_BASE + GICR_WAKER);

    do {
        val = readl_relaxed(GICD_RDIST_BASE + GICR_WAKER);
        if ( !(val & GICR_WAKER_ChildrenAsleep) )
            break;
        if ( NOW() > deadline )
        {
            timeout = true;
            break;
        }
        cpu_relax();
        udelay(1);
    } while ( timeout );

    if ( timeout )
    {
        dprintk(XENLOG_ERR, "GICv3: Redist enable RWP timeout\n");
        return 1;
    }

    return 0;
}

/* Enable LPIs on this redistributor (only useful when the host has an ITS). */
static bool gicv3_enable_lpis(void)
{
    uint32_t val;

    val = readl_relaxed(GICD_RDIST_BASE + GICR_TYPER);
    if ( !(val & GICR_TYPER_PLPIS) )
        return false;

    val = readl_relaxed(GICD_RDIST_BASE + GICR_CTLR);
    writel_relaxed(val | GICR_CTLR_ENABLE_LPIS, GICD_RDIST_BASE + GICR_CTLR);

    /* Make sure the GIC has seen the above */
    wmb();

    return true;
}

static int __init gicv3_populate_rdist(void)
{
    int i;
    uint32_t aff;
    uint32_t reg;
    uint64_t typer;
    uint64_t mpidr = cpu_logical_map(smp_processor_id());

    /*
     * If we ever get a cluster of more than 16 CPUs, just scream.
     */
    if ( (mpidr & 0xff) >= 16 )
          dprintk(XENLOG_WARNING, "GICv3:Cluster with more than 16's cpus\n");

    /*
     * Convert affinity to a 32bit value that can be matched to GICR_TYPER
     * bits [63:32]
     */
    aff = (
#ifdef CONFIG_ARM_64
           MPIDR_AFFINITY_LEVEL(mpidr, 3) << 24 |
#endif
           MPIDR_AFFINITY_LEVEL(mpidr, 2) << 16 |
           MPIDR_AFFINITY_LEVEL(mpidr, 1) << 8 |
           MPIDR_AFFINITY_LEVEL(mpidr, 0));

    for ( i = 0; i < gicv3.rdist_count; i++ )
    {
        void __iomem *ptr = gicv3.rdist_regions[i].map_base;

        reg = readl_relaxed(ptr + GICR_PIDR2) & GIC_PIDR2_ARCH_MASK;
        if ( reg != GIC_PIDR2_ARCH_GICv3 && reg != GIC_PIDR2_ARCH_GICv4 )
        {
            dprintk(XENLOG_ERR,
                    "GICv3: No redistributor present @%"PRIpaddr"\n",
                    gicv3.rdist_regions[i].base);
            break;
        }

        do {
            typer = readq_relaxed_non_atomic(ptr + GICR_TYPER);

            if ( (typer >> 32) == aff )
            {
                this_cpu(rbase) = ptr;

                if ( typer & GICR_TYPER_PLPIS )
                {
                    paddr_t rdist_addr;
                    unsigned int procnum;
                    int ret;

                    /*
                     * The ITS refers to redistributors either by their physical
                     * address or by their ID. Which one to use is an ITS
                     * choice. So determine those two values here (which we
                     * can do only here in GICv3 code) and tell the
                     * ITS code about it, so it can use them later to be able
                     * to address those redistributors accordingly.
                     */
                    rdist_addr = gicv3.rdist_regions[i].base;
                    rdist_addr += ptr - gicv3.rdist_regions[i].map_base;
                    procnum = (typer & GICR_TYPER_PROC_NUM_MASK);
                    procnum >>= GICR_TYPER_PROC_NUM_SHIFT;

                    gicv3_set_redist_address(rdist_addr, procnum);

                    ret = gicv3_lpi_init_rdist(ptr);
                    if ( ret && ret != -ENODEV )
                    {
                        printk("GICv3: CPU%d: Cannot initialize LPIs: %u\n",
                               smp_processor_id(), ret);
                        break;
                    }
                }

                printk("GICv3: CPU%d: Found redistributor in region %d @%p\n",
                        smp_processor_id(), i, ptr);
                return 0;
            }

            if ( gicv3.rdist_regions[i].single_rdist )
                break;

            if ( gicv3.rdist_stride )
                ptr += gicv3.rdist_stride;
            else
            {
                ptr += SZ_64K * 2; /* Skip RD_base + SGI_base */
                if ( typer & GICR_TYPER_VLPIS )
                    ptr += SZ_64K * 2; /* Skip VLPI_base + reserved page */
            }

        } while ( !(typer & GICR_TYPER_LAST) );
    }

    dprintk(XENLOG_ERR, "GICv3: CPU%d: mpidr 0x%"PRIregister" has no re-distributor!\n",
            smp_processor_id(), cpu_logical_map(smp_processor_id()));

    return -ENODEV;
}

static int gicv3_cpu_init(void)
{
    int i, ret;

    /* Register ourselves with the rest of the world */
    if ( gicv3_populate_rdist() )
        return -ENODEV;

    if ( gicv3_enable_redist() )
        return -ENODEV;

    /* If the host has any ITSes, enable LPIs now. */
    if ( gicv3_its_host_has_its() )
    {
        if ( !gicv3_enable_lpis() )
            return -EBUSY;
        ret = gicv3_its_setup_collection(smp_processor_id());
        if ( ret )
            return ret;
    }

    /* Set priority on PPI and SGI interrupts */
    for (i = 0; i < NR_GIC_SGI; i += 4)
        writel_relaxed(GIC_PRI_IPI_ALL,
                GICD_RDIST_SGI_BASE + GICR_IPRIORITYR0 + (i / 4) * 4);

    for (i = NR_GIC_SGI; i < NR_GIC_LOCAL_IRQS; i += 4)
        writel_relaxed(GIC_PRI_IRQ_ALL,
                GICD_RDIST_SGI_BASE + GICR_IPRIORITYR0 + (i / 4) * 4);

    /*
     * The activate state is unknown at boot, so make sure all
     * SGIs and PPIs are de-activated.
     */
    writel_relaxed(0xffffffffU, GICD_RDIST_SGI_BASE + GICR_ICACTIVER0);
    /*
     * Disable all PPI interrupts, ensure all SGI interrupts are
     * enabled.
     */
    writel_relaxed(0xffff0000U, GICD_RDIST_SGI_BASE + GICR_ICENABLER0);
    writel_relaxed(0x0000ffffU, GICD_RDIST_SGI_BASE + GICR_ISENABLER0);
    /* Configure SGIs/PPIs as non-secure Group-1 */
    writel_relaxed(GENMASK(31, 0), GICD_RDIST_SGI_BASE + GICR_IGROUPR0);

    gicv3_redist_wait_for_rwp();

    /* Enable system registers */
    gicv3_enable_sre();

    /* No priority grouping */
    WRITE_SYSREG(0, ICC_BPR1_EL1);

    /* Set priority mask register */
    WRITE_SYSREG(DEFAULT_PMR_VALUE, ICC_PMR_EL1);

    /* EOI drops priority, DIR deactivates the interrupt (mode 1) */
    WRITE_SYSREG(GICC_CTLR_EL1_EOImode_drop, ICC_CTLR_EL1);

    /* Enable Group1 interrupts */
    WRITE_SYSREG(1, ICC_IGRPEN1_EL1);

    /* Sync at once at the end of cpu interface configuration */
    isb();

    return 0;
}

static void gicv3_cpu_disable(void)
{
    WRITE_SYSREG(0, ICC_CTLR_EL1);
    isb();
}

static void gicv3_hyp_init(void)
{
    register_t vtr;

    vtr = READ_SYSREG(ICH_VTR_EL2);
    gicv3_info.nr_lrs  = (vtr & ICH_VTR_NRLRGS) + 1;
    gicv3.nr_priorities = ((vtr >> ICH_VTR_PRIBITS_SHIFT) &
                          ICH_VTR_PRIBITS_MASK) + 1;

    if ( !((gicv3.nr_priorities > 4) && (gicv3.nr_priorities < 8)) )
        panic("GICv3: Invalid number of priority bits\n");

    WRITE_SYSREG(ICH_VMCR_EOI | ICH_VMCR_VENG1, ICH_VMCR_EL2);
    WRITE_SYSREG(GICH_HCR_EN, ICH_HCR_EL2);
}

/* Set up the per-CPU parts of the GIC for a secondary CPU */
static int gicv3_secondary_cpu_init(void)
{
    int res;

    spin_lock(&gicv3.lock);

    res = gicv3_cpu_init();
    if ( res )
        goto out;

    gicv3_hyp_init();

out:
    spin_unlock(&gicv3.lock);

    return res;
}

static void gicv3_hyp_disable(void)
{
    register_t hcr;

    hcr = READ_SYSREG(ICH_HCR_EL2);
    hcr &= ~GICH_HCR_EN;
    WRITE_SYSREG(hcr, ICH_HCR_EL2);
    isb();
}

static u16 gicv3_compute_target_list(int *base_cpu, const struct cpumask *mask,
                                     uint64_t cluster_id)
{
    int cpu = *base_cpu;
    uint64_t mpidr = cpu_logical_map(cpu);
    u16 tlist = 0;

    while ( cpu < nr_cpu_ids )
    {
        /*
         * Assume that each cluster does not have more than 16 CPU's.
         * Check is made during GICv3 initialization (gicv3_populate_rdist())
         * on mpidr value for this. So skip this check here.
         */
        tlist |= 1 << (mpidr & 0xf);

        cpu = cpumask_next(cpu, mask);
        if ( cpu == nr_cpu_ids )
        {
            cpu--;
            goto out;
        }

        mpidr = cpu_logical_map(cpu);
        if ( cluster_id != (mpidr & ~MPIDR_AFF0_MASK) ) {
            cpu--;
            goto out;
        }
    }
out:
    *base_cpu = cpu;

    return tlist;
}

static void gicv3_send_sgi_list(enum gic_sgi sgi, const cpumask_t *cpumask)
{
    int cpu = 0;
    uint64_t val;

    for_each_cpu(cpu, cpumask)
    {
        /* Mask lower 8 bits. It represent cpu in affinity level 0 */
        uint64_t cluster_id = cpu_logical_map(cpu) & ~MPIDR_AFF0_MASK;
        u16 tlist;

        /* Get targetlist for the cluster to send SGI */
        tlist = gicv3_compute_target_list(&cpu, cpumask, cluster_id);

        /*
         * Prepare affinity path of the cluster for which SGI is generated
         * along with SGI number
         */
        val = (
#ifdef CONFIG_ARM_64
               MPIDR_AFFINITY_LEVEL(cluster_id, 3) << 48  |
#endif
               MPIDR_AFFINITY_LEVEL(cluster_id, 2) << 32  |
               sgi << 24                                  |
               MPIDR_AFFINITY_LEVEL(cluster_id, 1) << 16  |
               tlist);

        WRITE_SYSREG64(val, ICC_SGI1R_EL1);
    }
    /* Force above writes to ICC_SGI1R_EL1 */
    isb();
}

static void gicv3_send_sgi(enum gic_sgi sgi, enum gic_sgi_mode mode,
                           const cpumask_t *cpumask)
{
    /*
     * Ensure that stores to Normal memory are visible to the other CPUs
     * before issuing the IPI.
     */
    dsb(st);

    switch ( mode )
    {
    case SGI_TARGET_OTHERS:
        WRITE_SYSREG64(ICH_SGI_TARGET_OTHERS << ICH_SGI_IRQMODE_SHIFT |
                       (uint64_t)sgi << ICH_SGI_IRQ_SHIFT,
                       ICC_SGI1R_EL1);
        isb();
        break;
    case SGI_TARGET_SELF:
        gicv3_send_sgi_list(sgi, cpumask_of(smp_processor_id()));
        break;
    case SGI_TARGET_LIST:
        gicv3_send_sgi_list(sgi, cpumask);
        break;
    default:
        BUG();
    }
}

/* Shut down the per-CPU GIC interface */
static void gicv3_disable_interface(void)
{
    spin_lock(&gicv3.lock);

    gicv3_cpu_disable();
    gicv3_hyp_disable();

    spin_unlock(&gicv3.lock);
}

static void gicv3_update_lr(int lr, unsigned int virq, uint8_t priority,
                            unsigned int hw_irq, unsigned int state)
{
    uint64_t val = 0;

    BUG_ON(lr >= gicv3_info.nr_lrs);
    BUG_ON(lr < 0);

    val =  (((uint64_t)state & 0x3) << ICH_LR_STATE_SHIFT);

    /*
     * When the guest is GICv3, all guest IRQs are Group 1, as Group0
     * would result in a FIQ in the guest, which it wouldn't expect
     */
    if ( current->domain->arch.vgic.version == GIC_V3 )
        val |= ICH_LR_GRP1;

    val |= (uint64_t)priority << ICH_LR_PRIORITY_SHIFT;
    val |= ((uint64_t)virq & ICH_LR_VIRTUAL_MASK) << ICH_LR_VIRTUAL_SHIFT;

   if ( hw_irq != INVALID_IRQ )
       val |= ICH_LR_HW | (((uint64_t)hw_irq & ICH_LR_PHYSICAL_MASK)
                           << ICH_LR_PHYSICAL_SHIFT);

    gicv3_ich_write_lr(lr, val);
}

static void gicv3_clear_lr(int lr)
{
    gicv3_ich_write_lr(lr, 0);
}

static void gicv3_read_lr(int lr, struct gic_lr *lr_reg)
{
    uint64_t lrv;

    lrv = gicv3_ich_read_lr(lr);

    lr_reg->virq = (lrv >> ICH_LR_VIRTUAL_SHIFT) & ICH_LR_VIRTUAL_MASK;

    lr_reg->priority  = (lrv >> ICH_LR_PRIORITY_SHIFT) & ICH_LR_PRIORITY_MASK;
    lr_reg->pending   = lrv & ICH_LR_STATE_PENDING;
    lr_reg->active    = lrv & ICH_LR_STATE_ACTIVE;
    lr_reg->hw_status = lrv & ICH_LR_HW;

    if ( lr_reg->hw_status )
        lr_reg->hw.pirq = (lrv >> ICH_LR_PHYSICAL_SHIFT) & ICH_LR_PHYSICAL_MASK;
    else
    {
        lr_reg->virt.eoi = (lrv & ICH_LR_MAINTENANCE_IRQ);
        /* Source only exists in GICv2 compatible mode */
        if ( current->domain->arch.vgic.version == GIC_V2 )
        {
            /*
             * This is only valid for SGI, but it does not matter to always
             * read it as it should be 0 by default.
             */
            lr_reg->virt.source = (lrv >> ICH_LR_CPUID_SHIFT)
                & ICH_LR_CPUID_MASK;
        }
    }
}

static void gicv3_write_lr(int lr, const struct gic_lr *lr_reg)
{
    uint64_t lrv = 0;
    const enum gic_version vgic_version = current->domain->arch.vgic.version;


    lrv = ( ((u64)(lr_reg->virq & ICH_LR_VIRTUAL_MASK)  << ICH_LR_VIRTUAL_SHIFT) |
        ((u64)(lr_reg->priority & ICH_LR_PRIORITY_MASK) << ICH_LR_PRIORITY_SHIFT) );

    if ( lr_reg->active )
        lrv |= ICH_LR_STATE_ACTIVE;

    if ( lr_reg->pending )
        lrv |= ICH_LR_STATE_PENDING;

    if ( lr_reg->hw_status )
    {
        lrv |= ICH_LR_HW;
        lrv |= (uint64_t)lr_reg->hw.pirq << ICH_LR_PHYSICAL_SHIFT;
    }
    else
    {
        if ( lr_reg->virt.eoi )
            lrv |= ICH_LR_MAINTENANCE_IRQ;
        /* Source is only set in GICv2 compatible mode */
        if ( vgic_version == GIC_V2 )
        {
            /*
             * Source is only valid for SGIs, the caller should make
             * sure the field virt.source is always 0 for non-SGI.
             */
            ASSERT(!lr_reg->virt.source || lr_reg->virq < NR_GIC_SGI);
            lrv |= (uint64_t)lr_reg->virt.source << ICH_LR_CPUID_SHIFT;
        }
    }

    /*
     * When the guest is using vGICv3, all the IRQs are Group 1. Group 0
     * would result in a FIQ, which will not be expected by the guest OS.
     */
    if ( vgic_version == GIC_V3 )
        lrv |= ICH_LR_GRP1;

    gicv3_ich_write_lr(lr, lrv);
}

static void gicv3_hcr_status(uint32_t flag, bool status)
{
    register_t hcr;

    hcr = READ_SYSREG(ICH_HCR_EL2);
    if ( status )
        WRITE_SYSREG(hcr | flag, ICH_HCR_EL2);
    else
        WRITE_SYSREG(hcr & (~flag), ICH_HCR_EL2);
    isb();
}

static unsigned int gicv3_read_vmcr_priority(void)
{
   return ((READ_SYSREG(ICH_VMCR_EL2) >> ICH_VMCR_PRIORITY_SHIFT) &
            ICH_VMCR_PRIORITY_MASK);
}

/* Only support reading GRP1 APRn registers */
static unsigned int gicv3_read_apr(int apr_reg)
{
    register_t apr;

    switch ( apr_reg )
    {
    case 0:
        ASSERT(gicv3.nr_priorities > 4 && gicv3.nr_priorities < 8);
        apr = READ_SYSREG(ICH_AP1R0_EL2);
        break;
    case 1:
        ASSERT(gicv3.nr_priorities > 5 && gicv3.nr_priorities < 8);
        apr = READ_SYSREG(ICH_AP1R1_EL2);
        break;
    case 2:
        ASSERT(gicv3.nr_priorities > 6 && gicv3.nr_priorities < 8);
        apr = READ_SYSREG(ICH_AP1R2_EL2);
        break;
    default:
        BUG();
    }

    /* Number of priority levels do not exceed 32bit. */
    return apr;
}

static bool gicv3_read_pending_state(struct irq_desc *irqd)
{
    return gicv3_peek_irq(irqd, GICD_ISPENDR);
}

static void gicv3_irq_enable(struct irq_desc *desc)
{
    unsigned long flags;

    ASSERT(spin_is_locked(&desc->lock));

    spin_lock_irqsave(&gicv3.lock, flags);
    clear_bit(_IRQ_DISABLED, &desc->status);
    dsb(sy);
    /* Enable routing */
    gicv3_unmask_irq(desc);
    spin_unlock_irqrestore(&gicv3.lock, flags);
}

static void gicv3_irq_disable(struct irq_desc *desc)
{
    unsigned long flags;

    ASSERT(spin_is_locked(&desc->lock));

    spin_lock_irqsave(&gicv3.lock, flags);
    /* Disable routing */
    gicv3_mask_irq(desc);
    set_bit(_IRQ_DISABLED, &desc->status);
    spin_unlock_irqrestore(&gicv3.lock, flags);
}

static unsigned int gicv3_irq_startup(struct irq_desc *desc)
{
    gicv3_irq_enable(desc);

    return 0;
}

static void gicv3_irq_shutdown(struct irq_desc *desc)
{
    gicv3_irq_disable(desc);
}

static void gicv3_irq_ack(struct irq_desc *desc)
{
    /* No ACK -- reading IAR has done this for us */
}

static void gicv3_host_irq_end(struct irq_desc *desc)
{
    /* Lower the priority */
    gicv3_eoi_irq(desc);
    /* Deactivate */
    gicv3_dir_irq(desc);
}

static void gicv3_guest_irq_end(struct irq_desc *desc)
{
    /* Lower the priority of the IRQ */
    gicv3_eoi_irq(desc);
    /* Deactivation happens in maintenance interrupt / via GICV */
}

static void gicv3_irq_set_affinity(struct irq_desc *desc, const cpumask_t *mask)
{
    unsigned int cpu;
    uint64_t affinity;

    ASSERT(!cpumask_empty(mask));

    spin_lock(&gicv3.lock);

    cpu = gicv3_get_cpu_from_mask(mask);
    affinity = gicv3_mpidr_to_affinity(cpu);
    /* Make sure we don't broadcast the interrupt */
    affinity &= ~GICD_IROUTER_SPI_MODE_ANY;

    if ( desc->irq >= NR_GIC_LOCAL_IRQS )
        writeq_relaxed_non_atomic(affinity, (GICD + GICD_IROUTER + desc->irq * 8));

    spin_unlock(&gicv3.lock);
}

static int gicv3_make_hwdom_dt_node(const struct domain *d,
                                    const struct dt_device_node *gic,
                                    void *fdt)
{
    const void *compatible, *hw_reg;
    uint32_t len, new_len;
    int res;

    compatible = dt_get_property(gic, "compatible", &len);
    if ( !compatible )
    {
        dprintk(XENLOG_ERR, "Can't find compatible property for the gic node\n");
        return -FDT_ERR_XEN(ENOENT);
    }

    res = fdt_property(fdt, "compatible", compatible, len);
    if ( res )
        return res;

    res = fdt_property_cell(fdt, "#redistributor-regions",
                            d->arch.vgic.nr_regions);
    if ( res )
        return res;

    new_len = dt_cells_to_size(dt_n_addr_cells(gic) + dt_n_size_cells(gic));
    /*
     * GIC has two memory regions: Distributor + rdist regions
     * CPU interface and virtual cpu interfaces accessesed as System registers
     * So cells are created only for Distributor and rdist regions
     * The hardware domain may not use all the regions. So only copy
     * what is necessary.
     */
    new_len = new_len * (d->arch.vgic.nr_regions + 1);

    hw_reg = dt_get_property(gic, "reg", &len);
    if ( !hw_reg )
        return -FDT_ERR_XEN(ENOENT);
    if ( new_len > len )
        return -FDT_ERR_XEN(ERANGE);

    res = fdt_property(fdt, "reg", hw_reg, new_len);
    if ( res )
        return res;

    if ( has_vpci_bridge(d) )
        return gicv3_its_make_emulated_dt_node(d, fdt);

    return gicv3_its_make_hwdom_dt_nodes(d, gic, fdt);
}

static const hw_irq_controller gicv3_host_irq_type = {
    .typename     = "gic-v3",
    .startup      = gicv3_irq_startup,
    .shutdown     = gicv3_irq_shutdown,
    .enable       = gicv3_irq_enable,
    .disable      = gicv3_irq_disable,
    .ack          = gicv3_irq_ack,
    .end          = gicv3_host_irq_end,
    .set_affinity = gicv3_irq_set_affinity,
};

static const hw_irq_controller gicv3_guest_irq_type = {
    .typename     = "gic-v3",
    .startup      = gicv3_irq_startup,
    .shutdown     = gicv3_irq_shutdown,
    .enable       = gicv3_irq_enable,
    .disable      = gicv3_irq_disable,
    .ack          = gicv3_irq_ack,
    .end          = gicv3_guest_irq_end,
    .set_affinity = gicv3_irq_set_affinity,
};

static paddr_t __initdata dbase = INVALID_PADDR;
static paddr_t __initdata vbase = INVALID_PADDR, vsize = 0;
static paddr_t __initdata cbase = INVALID_PADDR, csize = 0;

#ifdef CONFIG_VGICV2
/* If the GICv3 supports GICv2, initialize it */
static void __init gicv3_init_v2(void)
{
    if ( cbase == INVALID_PADDR || vbase == INVALID_PADDR )
        return;

    /*
     * We emulate a vGICv2 using a GIC CPU interface of GUEST_GICC_SIZE.
     * So only support GICv2 on GICv3 when the virtual CPU interface is
     * at least GUEST_GICC_SIZE.
     */
    if ( vsize < GUEST_GICC_SIZE )
    {
        printk(XENLOG_WARNING
               "GICv3: WARNING: Not enabling support for GICv2 compat mode.\n"
               "Size of GICV (%#"PRIpaddr") must at least be %#llx.\n",
               vsize, GUEST_GICC_SIZE);
        return;
    }

    printk("GICv3 compatible with GICv2 cbase %#"PRIpaddr" vbase %#"PRIpaddr"\n",
           cbase, vbase);

    vgic_v2_setup_hw(dbase, cbase, csize, vbase, 0);
}
#else
static inline void gicv3_init_v2(void) { }
#endif

static void __init gicv3_ioremap_distributor(paddr_t dist_paddr)
{
    if ( dist_paddr & ~PAGE_MASK )
        panic("GICv3:  Found unaligned distributor address %"PRIpaddr"\n",
              dbase);

    gicv3.map_dbase = ioremap_nocache(dist_paddr, SZ_64K);
    if ( !gicv3.map_dbase )
        panic("GICv3: Failed to ioremap for GIC distributor\n");
}

static void __init gicv3_dt_init(void)
{
    struct rdist_region *rdist_regs;
    int res, i;
    const struct dt_device_node *node = gicv3_info.node;

    res = dt_device_get_paddr(node, 0, &dbase, NULL);
    if ( res )
        panic("GICv3: Cannot find a valid distributor address\n");

    gicv3_ioremap_distributor(dbase);

    if ( !dt_property_read_u32(node, "#redistributor-regions",
                &gicv3.rdist_count) )
        gicv3.rdist_count = 1;

    rdist_regs = xzalloc_array(struct rdist_region, gicv3.rdist_count);
    if ( !rdist_regs )
        panic("GICv3: Failed to allocate memory for rdist regions\n");

    for ( i = 0; i < gicv3.rdist_count; i++ )
    {
        paddr_t rdist_base, rdist_size;

        res = dt_device_get_paddr(node, 1 + i, &rdist_base, &rdist_size);
        if ( res )
            panic("GICv3: No rdist base found for region %d\n", i);

        rdist_regs[i].base = rdist_base;
        rdist_regs[i].size = rdist_size;
    }

    if ( !dt_property_read_u32(node, "redistributor-stride", &gicv3.rdist_stride) )
        gicv3.rdist_stride = 0;

    gicv3.rdist_regions= rdist_regs;

    res = platform_get_irq(node, 0);
    if ( res < 0 )
        panic("GICv3: Cannot find the maintenance IRQ\n");
    gicv3_info.maintenance_irq = res;

    /*
     * For GICv3 supporting GICv2, GICC and GICV base address will be
     * provided.
     */
    res = dt_device_get_paddr(node, 1 + gicv3.rdist_count,
                                &cbase, &csize);
    if ( !res )
        dt_device_get_paddr(node, 1 + gicv3.rdist_count + 2,
                              &vbase, &vsize);
}

static int gicv3_iomem_deny_access(struct domain *d)
{
    int rc, i;
    unsigned long mfn, nr;

    mfn = dbase >> PAGE_SHIFT;
    nr = PFN_UP(SZ_64K);
    rc = iomem_deny_access(d, mfn, mfn + nr);
    if ( rc )
        return rc;

    rc = gicv3_its_deny_access(d);
    if ( rc )
        return rc;

    for ( i = 0; i < gicv3.rdist_count; i++ )
    {
        mfn = gicv3.rdist_regions[i].base >> PAGE_SHIFT;
        nr = PFN_UP(gicv3.rdist_regions[i].size);
        rc = iomem_deny_access(d, mfn, mfn + nr);
        if ( rc )
            return rc;
    }

    if ( cbase != INVALID_PADDR )
    {
        mfn = cbase >> PAGE_SHIFT;
        nr = PFN_UP(csize);
        rc = iomem_deny_access(d, mfn, mfn + nr);
        if ( rc )
            return rc;
    }

    if ( vbase != INVALID_PADDR )
    {
        mfn = vbase >> PAGE_SHIFT;
        nr = PFN_UP(csize);
        return iomem_deny_access(d, mfn, mfn + nr);
    }

    return 0;
}

#ifdef CONFIG_ACPI
static void __init
gic_acpi_add_rdist_region(paddr_t base, paddr_t size, bool single_rdist)
{
    unsigned int idx = gicv3.rdist_count++;

    gicv3.rdist_regions[idx].single_rdist = single_rdist;
    gicv3.rdist_regions[idx].base = base;
    gicv3.rdist_regions[idx].size = size;
}

static inline bool gic_dist_supports_dvis(void)
{
    return !!(readl_relaxed(GICD + GICD_TYPER) & GICD_TYPER_DVIS);
}

static int gicv3_make_hwdom_madt(const struct domain *d, u32 offset)
{
    struct acpi_subtable_header *header;
    struct acpi_madt_generic_interrupt *host_gicc, *gicc;
    struct acpi_madt_generic_redistributor *gicr;
    u8 *base_ptr = d->arch.efi_acpi_table + offset;
    u32 i, table_len = 0, size;

    /* Add Generic Interrupt */
    header = acpi_table_get_entry_madt(ACPI_MADT_TYPE_GENERIC_INTERRUPT, 0);
    if ( !header )
    {
        printk("Can't get GICC entry");
        return -EINVAL;
    }

    host_gicc = container_of(header, struct acpi_madt_generic_interrupt,
                             header);
    size = ACPI_MADT_GICC_LENGTH;
    for ( i = 0; i < d->max_vcpus; i++ )
    {
        gicc = (struct acpi_madt_generic_interrupt *)(base_ptr + table_len);
        memcpy(gicc, host_gicc, size);
        gicc->cpu_interface_number = i;
        gicc->uid = i;
        gicc->flags = ACPI_MADT_ENABLED;
        gicc->arm_mpidr = vcpuid_to_vaffinity(i);
        gicc->parking_version = 0;
        gicc->performance_interrupt = 0;
        gicc->gicv_base_address = 0;
        gicc->gich_base_address = 0;
        gicc->gicr_base_address = 0;
        gicc->vgic_interrupt = 0;
        table_len += size;
    }

    /* Add Generic Redistributor */
    size = sizeof(struct acpi_madt_generic_redistributor);
    /*
     * The hardware domain may not used all the regions. So only copy
     * what is necessary.
     */
    for ( i = 0; i < d->arch.vgic.nr_regions; i++ )
    {
        gicr = (struct acpi_madt_generic_redistributor *)(base_ptr + table_len);
        gicr->header.type = ACPI_MADT_TYPE_GENERIC_REDISTRIBUTOR;
        gicr->header.length = size;
        gicr->base_address = gicv3.rdist_regions[i].base;
        gicr->length = gicv3.rdist_regions[i].size;
        table_len += size;
    }

    table_len += gicv3_its_make_hwdom_madt(d, base_ptr + table_len);

    return table_len;
}

static unsigned long gicv3_get_hwdom_extra_madt_size(const struct domain *d)
{
    unsigned long size;

    size = sizeof(struct acpi_madt_generic_redistributor) * gicv3.rdist_count;

    size += sizeof(struct acpi_madt_generic_translator)
            * vgic_v3_its_count(d);

    return size;
}

static int __init
gic_acpi_parse_madt_cpu(struct acpi_subtable_header *header,
                        const unsigned long end)
{
    static int cpu_base_assigned = 0;
    struct acpi_madt_generic_interrupt *processor =
               container_of(header, struct acpi_madt_generic_interrupt, header);

    if ( BAD_MADT_GICC_ENTRY(processor, end) )
        return -EINVAL;

    /* Read from APIC table and fill up the GIC variables */
    if ( !cpu_base_assigned )
    {
        cbase = processor->base_address;
        vbase = processor->gicv_base_address;
        gicv3_info.maintenance_irq = processor->vgic_interrupt;

        if ( processor->flags & ACPI_MADT_VGIC_IRQ_MODE )
            irq_set_type(gicv3_info.maintenance_irq, IRQ_TYPE_EDGE_BOTH);
        else
            irq_set_type(gicv3_info.maintenance_irq, IRQ_TYPE_LEVEL_MASK);

        cpu_base_assigned = 1;
    }
    else
    {
        if ( cbase != processor->base_address
             || vbase != processor->gicv_base_address
             || gicv3_info.maintenance_irq != processor->vgic_interrupt )
        {
            printk("GICv3: GICC entries are not same in MADT table\n");
            return -EINVAL;
        }
    }

    return 0;
}

static int __init
gic_acpi_parse_madt_distributor(struct acpi_subtable_header *header,
                                const unsigned long end)
{
    struct acpi_madt_generic_distributor *dist =
             container_of(header, struct acpi_madt_generic_distributor, header);

    if ( BAD_MADT_ENTRY(dist, end) )
        return -EINVAL;

    dbase = dist->base_address;

    return 0;
}

static int __init
gic_acpi_parse_cpu_redistributor(struct acpi_subtable_header *header,
                                 const unsigned long end)
{
    struct acpi_madt_generic_interrupt *processor;
    u32 size;

    processor = (struct acpi_madt_generic_interrupt *)header;
    if ( !(processor->flags & ACPI_MADT_ENABLED) )
        return 0;

    size = gic_dist_supports_dvis() ? 4 * SZ_64K : 2 * SZ_64K;
    gic_acpi_add_rdist_region(processor->gicr_base_address, size, true);

    return 0;
}

static int __init
gic_acpi_get_madt_cpu_num(struct acpi_subtable_header *header,
                          const unsigned long end)
{
    struct acpi_madt_generic_interrupt *cpuif;

    cpuif = (struct acpi_madt_generic_interrupt *)header;
    if ( BAD_MADT_GICC_ENTRY(cpuif, end) || !cpuif->gicr_base_address )
        return -EINVAL;

    return 0;
}

static int __init
gic_acpi_parse_madt_redistributor(struct acpi_subtable_header *header,
                                  const unsigned long end)
{
    struct acpi_madt_generic_redistributor *rdist;

    rdist = (struct acpi_madt_generic_redistributor *)header;
    if ( BAD_MADT_ENTRY(rdist, end) )
        return -EINVAL;

    gic_acpi_add_rdist_region(rdist->base_address, rdist->length, false);

    return 0;
}

static int __init
gic_acpi_get_madt_redistributor_num(struct acpi_subtable_header *header,
                                    const unsigned long end)
{
    /* Nothing to do here since it only wants to get the number of GIC
     * redistributors.
     */
    return 0;
}

static void __init gicv3_acpi_init(void)
{
    struct rdist_region *rdist_regs;
    bool gicr_table = true;
    int count;

    /*
     * Find distributor base address. We expect one distributor entry since
     * ACPI 5.0 spec neither support multi-GIC instances nor GIC cascade.
     */
    count = acpi_table_parse_madt(ACPI_MADT_TYPE_GENERIC_DISTRIBUTOR,
                                  gic_acpi_parse_madt_distributor, 0);
    if ( count <= 0 )
        panic("GICv3: No valid GICD entries exists\n");

    gicv3_ioremap_distributor(dbase);

    /* Get number of redistributor */
    count = acpi_table_parse_madt(ACPI_MADT_TYPE_GENERIC_REDISTRIBUTOR,
                                  gic_acpi_get_madt_redistributor_num, 0);
    /* Count the total number of CPU interface entries */
    if ( count <= 0 ) {
        count = acpi_table_parse_madt(ACPI_MADT_TYPE_GENERIC_INTERRUPT,
                                      gic_acpi_get_madt_cpu_num, 0);
        if (count <= 0)
            panic("GICv3: No valid GICR entries exists\n");

        gicr_table = false;
    }

    rdist_regs = xzalloc_array(struct rdist_region, count);
    if ( !rdist_regs )
        panic("GICv3: Failed to allocate memory for rdist regions\n");

    gicv3.rdist_regions = rdist_regs;

    if ( gicr_table )
        /* Parse always-on power domain Re-distributor entries */
        count = acpi_table_parse_madt(ACPI_MADT_TYPE_GENERIC_REDISTRIBUTOR,
                                      gic_acpi_parse_madt_redistributor, count);
    else
        /* Parse Re-distributor entries described in CPU interface table */
        count = acpi_table_parse_madt(ACPI_MADT_TYPE_GENERIC_INTERRUPT,
                                      gic_acpi_parse_cpu_redistributor, count);
    if ( count <= 0 )
        panic("GICv3: Can't get Redistributor entry\n");

    /* Collect CPU base addresses */
    count = acpi_table_parse_madt(ACPI_MADT_TYPE_GENERIC_INTERRUPT,
                                  gic_acpi_parse_madt_cpu, 0);
    if ( count <= 0 )
        panic("GICv3: No valid GICC entries exists\n");

    gicv3.rdist_stride = 0;

    /*
     * In ACPI, 0 is considered as the invalid address. However the rest
     * of the initialization rely on the invalid address to be
     * INVALID_ADDR.
     *
     * Also set the size of the GICC and GICV when there base address
     * is not invalid as those values are not present in ACPI.
     */
    if ( !cbase )
        cbase = INVALID_PADDR;
    else
        csize = SZ_8K;

    if ( !vbase )
        vbase = INVALID_PADDR;
    else
        vsize = GUEST_GICC_SIZE;

}
#else
static void __init gicv3_acpi_init(void) { }
#endif

static bool gic_dist_supports_lpis(void)
{
    return (readl_relaxed(GICD + GICD_TYPER) & GICD_TYPE_LPIS);
}

/* Set up the GIC */
static int __init gicv3_init(void)
{
    int res, i;
    uint32_t reg;
    unsigned int intid_bits;

    if ( !cpu_has_gicv3 )
    {
        dprintk(XENLOG_ERR, "GICv3: driver requires system register support\n");
        return -ENODEV;
    }

    if ( acpi_disabled )
        gicv3_dt_init();
    else
        gicv3_acpi_init();

    reg = readl_relaxed(GICD + GICD_PIDR2) & GIC_PIDR2_ARCH_MASK;
    if ( reg != GIC_PIDR2_ARCH_GICv3 && reg != GIC_PIDR2_ARCH_GICv4 )
         panic("GICv3: no distributor detected\n");

    for ( i = 0; i < gicv3.rdist_count; i++ )
    {
        /* map dbase & rdist regions */
        gicv3.rdist_regions[i].map_base =
                ioremap_nocache(gicv3.rdist_regions[i].base,
                                gicv3.rdist_regions[i].size);

        if ( !gicv3.rdist_regions[i].map_base )
            panic("GICv3: Failed to ioremap rdist region for region %d\n", i);
    }

    printk("GICv3 initialization:\n"
           "      gic_dist_addr=%#"PRIpaddr"\n"
           "      gic_maintenance_irq=%u\n"
           "      gic_rdist_stride=%#x\n"
           "      gic_rdist_regions=%d\n",
           dbase, gicv3_info.maintenance_irq,
           gicv3.rdist_stride, gicv3.rdist_count);
    printk("      redistributor regions:\n");
    for ( i = 0; i < gicv3.rdist_count; i++ )
    {
        const struct rdist_region *r = &gicv3.rdist_regions[i];

        printk("        - region %u: %#"PRIpaddr" - %#"PRIpaddr"\n",
               i, r->base, r->base + r->size);
    }

    reg = readl_relaxed(GICD + GICD_TYPER);
    intid_bits = GICD_TYPE_ID_BITS(reg);

    vgic_v3_setup_hw(dbase, gicv3.rdist_count, gicv3.rdist_regions, intid_bits);
    gicv3_init_v2();

    spin_lock_init(&gicv3.lock);

    spin_lock(&gicv3.lock);

    gicv3_dist_init();

    if ( gic_dist_supports_lpis() )
    {
        res = gicv3_its_init();
        if ( res )
            panic("GICv3: ITS: initialization failed: %d\n", res);
    }

    res = gicv3_cpu_init();
    if ( res )
        goto out;

    gicv3_hyp_init();

out:
    spin_unlock(&gicv3.lock);

    return res;
}

static const struct gic_hw_operations gicv3_ops = {
    .info                = &gicv3_info,
    .init                = gicv3_init,
    .save_state          = gicv3_save_state,
    .restore_state       = gicv3_restore_state,
    .dump_state          = gicv3_dump_state,
    .gic_host_irq_type   = &gicv3_host_irq_type,
    .gic_guest_irq_type  = &gicv3_guest_irq_type,
    .eoi_irq             = gicv3_eoi_irq,
    .deactivate_irq      = gicv3_dir_irq,
    .read_irq            = gicv3_read_irq,
    .set_active_state    = gicv3_set_active_state,
    .set_pending_state   = gicv3_set_pending_state,
    .set_irq_type        = gicv3_set_irq_type,
    .set_irq_priority    = gicv3_set_irq_priority,
    .send_SGI            = gicv3_send_sgi,
    .disable_interface   = gicv3_disable_interface,
    .update_lr           = gicv3_update_lr,
    .update_hcr_status   = gicv3_hcr_status,
    .clear_lr            = gicv3_clear_lr,
    .read_lr             = gicv3_read_lr,
    .write_lr            = gicv3_write_lr,
    .read_vmcr_priority  = gicv3_read_vmcr_priority,
    .read_apr            = gicv3_read_apr,
    .read_pending_state  = gicv3_read_pending_state,
    .secondary_init      = gicv3_secondary_cpu_init,
    .make_hwdom_dt_node  = gicv3_make_hwdom_dt_node,
#ifdef CONFIG_ACPI
    .make_hwdom_madt     = gicv3_make_hwdom_madt,
    .get_hwdom_extra_madt_size = gicv3_get_hwdom_extra_madt_size,
#endif
    .iomem_deny_access   = gicv3_iomem_deny_access,
    .do_LPI              = gicv3_do_LPI,
};

static int __init gicv3_dt_preinit(struct dt_device_node *node, const void *data)
{
    gicv3_info.hw_version = GIC_V3;
    gicv3_info.node = node;
    register_gic_ops(&gicv3_ops);
    dt_irq_xlate = gic_irq_xlate;

    return 0;
}

static const struct dt_device_match gicv3_dt_match[] __initconst =
{
    DT_MATCH_GIC_V3,
    { /* sentinel */ },
};

DT_DEVICE_START(gicv3, "GICv3", DEVICE_INTERRUPT_CONTROLLER)
        .dt_match = gicv3_dt_match,
        .init = gicv3_dt_preinit,
DT_DEVICE_END

#ifdef CONFIG_ACPI
/* Set up the GIC */
static int __init gicv3_acpi_preinit(const void *data)
{
    gicv3_info.hw_version = GIC_V3;
    register_gic_ops(&gicv3_ops);

    return 0;
}

ACPI_DEVICE_START(agicv3, "GICv3", DEVICE_INTERRUPT_CONTROLLER)
        .class_type = ACPI_MADT_GIC_VERSION_V3,
        .init = gicv3_acpi_preinit,
ACPI_DEVICE_END

ACPI_DEVICE_START(agicv4, "GICv4", DEVICE_INTERRUPT_CONTROLLER)
        .class_type = ACPI_MADT_GIC_VERSION_V4,
        .init = gicv3_acpi_preinit,
ACPI_DEVICE_END
#endif

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
