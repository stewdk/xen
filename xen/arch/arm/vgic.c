/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * xen/arch/arm/vgic.c
 *
 * ARM Virtual Generic Interrupt Controller support
 *
 * Ian Campbell <ian.campbell@citrix.com>
 * Copyright (c) 2011 Citrix Systems.
 */

#include <xen/bitops.h>
#include <xen/lib.h>
#include <xen/init.h>
#include <xen/domain_page.h>
#include <xen/softirq.h>
#include <xen/irq.h>
#include <xen/sched.h>
#include <xen/perfc.h>

#include <asm/event.h>
#include <asm/current.h>

#include <asm/mmio.h>
#include <asm/gic.h>
#include <asm/vgic.h>

static inline struct vgic_irq_rank *vgic_get_rank(struct vcpu *v,
                                                  unsigned int rank)
{
    if ( rank == 0 )
        return v->arch.vgic.private_irqs;
    else if ( rank <= DOMAIN_NR_RANKS(v->domain) )
        return &v->domain->arch.vgic.shared_irqs[rank - 1];
    else
        return NULL;
}

/*
 * Returns rank corresponding to a GICD_<FOO><n> register for
 * GICD_<FOO> with <b>-bits-per-interrupt.
 */
struct vgic_irq_rank *vgic_rank_offset(struct vcpu *v, unsigned int b,
                                       unsigned int n, unsigned int s)
{
    unsigned int rank = REG_RANK_NR(b, (n >> s));

    return vgic_get_rank(v, rank);
}

struct vgic_irq_rank *vgic_rank_irq(struct vcpu *v, unsigned int irq)
{
    unsigned int rank = irq / 32;

    return vgic_get_rank(v, rank);
}

void vgic_init_pending_irq(struct pending_irq *p, unsigned int virq)
{
    /* The lpi_vcpu_id field must be big enough to hold a VCPU ID. */
    BUILD_BUG_ON(BIT(sizeof(p->lpi_vcpu_id) * 8, UL) < MAX_VIRT_CPUS);

    memset(p, 0, sizeof(*p));
    INIT_LIST_HEAD(&p->inflight);
    INIT_LIST_HEAD(&p->lr_queue);
    p->irq = virq;
    p->lpi_vcpu_id = INVALID_VCPU_ID;
}

static void vgic_rank_init(struct vgic_irq_rank *rank, uint8_t index,
                           unsigned int vcpu)
{
    unsigned int i;

    /*
     * Make sure that the type chosen to store the target is able to
     * store an VCPU ID between 0 and the maximum of virtual CPUs
     * supported.
     */
    BUILD_BUG_ON((1 << (sizeof(rank->vcpu[0]) * 8)) < MAX_VIRT_CPUS);

    spin_lock_init(&rank->lock);

    rank->index = index;

    for ( i = 0; i < NR_INTERRUPT_PER_RANK; i++ )
        write_atomic(&rank->vcpu[i], vcpu);
}

int domain_vgic_register(struct domain *d, unsigned int *mmio_count)
{
    switch ( d->arch.vgic.version )
    {
#ifdef CONFIG_GICV3
    case GIC_V3:
        if ( vgic_v3_init(d, mmio_count) )
           return -ENODEV;
        break;
#endif
#ifdef CONFIG_VGICV2
    case GIC_V2:
        if ( vgic_v2_init(d, mmio_count) )
            return -ENODEV;
        break;
#endif
    default:
        printk(XENLOG_G_ERR "d%d: Unknown vGIC version %u\n",
               d->domain_id, d->arch.vgic.version);
        return -ENODEV;
    }

    return 0;
}

int domain_vgic_init(struct domain *d, unsigned int nr_spis)
{
    int i;
    int ret;

    d->arch.vgic.ctlr = 0;

    /*
     * The vGIC relies on having a pending_irq available for every IRQ
     * described in the ranks. As each rank describes 32 interrupts, we
     * need to make sure the number of SPIs is a multiple of 32.
     */
    nr_spis = ROUNDUP(nr_spis, 32);

    /* Limit the number of virtual SPIs supported to (1020 - 32) = 988  */
    if ( nr_spis > (1020 - NR_LOCAL_IRQS) )
        return -EINVAL;

    d->arch.vgic.nr_spis = nr_spis;

    spin_lock_init(&d->arch.vgic.lock);

    d->arch.vgic.shared_irqs =
        xzalloc_array(struct vgic_irq_rank, DOMAIN_NR_RANKS(d));
    if ( d->arch.vgic.shared_irqs == NULL )
        return -ENOMEM;

    d->arch.vgic.pending_irqs =
        xzalloc_array(struct pending_irq, d->arch.vgic.nr_spis);
    if ( d->arch.vgic.pending_irqs == NULL )
        return -ENOMEM;

    for (i=0; i<d->arch.vgic.nr_spis; i++)
        vgic_init_pending_irq(&d->arch.vgic.pending_irqs[i], i + 32);

    /* SPIs are routed to VCPU0 by default */
    for ( i = 0; i < DOMAIN_NR_RANKS(d); i++ )
        vgic_rank_init(&d->arch.vgic.shared_irqs[i], i + 1, 0);

    ret = d->arch.vgic.handler->domain_init(d);
    if ( ret )
        return ret;

    d->arch.vgic.allocated_irqs =
        xzalloc_array(unsigned long, BITS_TO_LONGS(vgic_num_irqs(d)));
    if ( !d->arch.vgic.allocated_irqs )
        return -ENOMEM;

    /* vIRQ0-15 (SGIs) are reserved */
    for ( i = 0; i < NR_GIC_SGI; i++ )
        set_bit(i, d->arch.vgic.allocated_irqs);

    return 0;
}

void register_vgic_ops(struct domain *d, const struct vgic_ops *ops)
{
   d->arch.vgic.handler = ops;
}

void domain_vgic_free(struct domain *d)
{
    int i;
    int ret;

    for ( i = 0; i < (d->arch.vgic.nr_spis); i++ )
    {
        struct pending_irq *p = spi_to_pending(d, i + 32);

        if ( p->desc )
        {
            ret = release_guest_irq(d, p->irq);
            if ( ret )
                dprintk(XENLOG_G_WARNING, "d%u: Failed to release virq %u ret = %d\n",
                        d->domain_id, p->irq, ret);
        }
    }

    if ( d->arch.vgic.handler )
        d->arch.vgic.handler->domain_free(d);
    xfree(d->arch.vgic.shared_irqs);
    xfree(d->arch.vgic.pending_irqs);
    xfree(d->arch.vgic.allocated_irqs);
}

int domain_vgic_late_init(struct domain *d)
{
    if ( d->arch.vgic.handler->domain_late_init )
        return d->arch.vgic.handler->domain_late_init(d);

    return 0;
}

int vcpu_vgic_init(struct vcpu *v)
{
    int i;

    v->arch.vgic.private_irqs = xzalloc(struct vgic_irq_rank);
    if ( v->arch.vgic.private_irqs == NULL )
      return -ENOMEM;

    /* SGIs/PPIs are always routed to this VCPU */
    vgic_rank_init(v->arch.vgic.private_irqs, 0, v->vcpu_id);

    v->domain->arch.vgic.handler->vcpu_init(v);

    memset(&v->arch.vgic.pending_irqs, 0, sizeof(v->arch.vgic.pending_irqs));
    for (i = 0; i < 32; i++)
        vgic_init_pending_irq(&v->arch.vgic.pending_irqs[i], i);

    INIT_LIST_HEAD(&v->arch.vgic.inflight_irqs);
    INIT_LIST_HEAD(&v->arch.vgic.lr_pending);
    spin_lock_init(&v->arch.vgic.lock);

    return 0;
}

int vcpu_vgic_free(struct vcpu *v)
{
    xfree(v->arch.vgic.private_irqs);
    return 0;
}

struct vcpu *vgic_get_target_vcpu(struct vcpu *v, unsigned int virq)
{
    struct vgic_irq_rank *rank = vgic_rank_irq(v, virq);
    int target = read_atomic(&rank->vcpu[virq & INTERRUPT_RANK_MASK]);
    return v->domain->vcpu[target];
}

static int vgic_get_virq_priority(struct vcpu *v, unsigned int virq)
{
    struct vgic_irq_rank *rank;

    /* LPIs don't have a rank, also store their priority separately. */
    if ( is_lpi(virq) )
        return v->domain->arch.vgic.handler->lpi_get_priority(v->domain, virq);

    rank = vgic_rank_irq(v, virq);
    return ACCESS_ONCE(rank->priority[virq & INTERRUPT_RANK_MASK]);
}

bool vgic_migrate_irq(struct vcpu *old, struct vcpu *new, unsigned int irq)
{
    unsigned long flags;
    struct pending_irq *p;

    /* This will never be called for an LPI, as we don't migrate them. */
    ASSERT(!is_lpi(irq));

    spin_lock_irqsave(&old->arch.vgic.lock, flags);

    p = irq_to_pending(old, irq);

    /* nothing to do for virtual interrupts */
    if ( p->desc == NULL )
    {
        spin_unlock_irqrestore(&old->arch.vgic.lock, flags);
        return true;
    }

    /* migration already in progress, no need to do anything */
    if ( test_bit(GIC_IRQ_GUEST_MIGRATING, &p->status) )
    {
        gprintk(XENLOG_WARNING, "irq %u migration failed: requested while in progress\n", irq);
        spin_unlock_irqrestore(&old->arch.vgic.lock, flags);
        return false;
    }

    perfc_incr(vgic_irq_migrates);

    if ( list_empty(&p->inflight) )
    {
        irq_set_affinity(p->desc, cpumask_of(new->processor));
        spin_unlock_irqrestore(&old->arch.vgic.lock, flags);
        return true;
    }
    /* If the IRQ is still lr_pending, re-inject it to the new vcpu */
    if ( !list_empty(&p->lr_queue) )
    {
        vgic_remove_irq_from_queues(old, p);
        irq_set_affinity(p->desc, cpumask_of(new->processor));
        spin_unlock_irqrestore(&old->arch.vgic.lock, flags);
        vgic_inject_irq(new->domain, new, irq, true);
        return true;
    }
    /* if the IRQ is in a GICH_LR register, set GIC_IRQ_GUEST_MIGRATING
     * and wait for the EOI */
    if ( !list_empty(&p->inflight) )
        set_bit(GIC_IRQ_GUEST_MIGRATING, &p->status);

    spin_unlock_irqrestore(&old->arch.vgic.lock, flags);
    return true;
}

void arch_move_irqs(struct vcpu *v)
{
    const cpumask_t *cpu_mask = cpumask_of(v->processor);
    struct domain *d = v->domain;
    struct pending_irq *p;
    struct vcpu *v_target;
    int i;

    /*
     * We don't migrate LPIs at the moment.
     * If we ever do, we must make sure that the struct pending_irq does
     * not go away, as there is no lock preventing this here.
     * To ensure this, we check if the loop below ever touches LPIs.
     * In the moment vgic_num_irqs() just covers SPIs, as it's mostly used
     * for allocating the pending_irq and irq_desc array, in which LPIs
     * don't participate.
     */
    ASSERT(!is_lpi(vgic_num_irqs(d) - 1));

    for ( i = 32; i < vgic_num_irqs(d); i++ )
    {
        v_target = vgic_get_target_vcpu(v, i);
        p = irq_to_pending(v_target, i);

        if ( v_target == v && !test_bit(GIC_IRQ_GUEST_MIGRATING, &p->status) )
            irq_set_affinity(p->desc, cpu_mask);
    }
}

void vgic_disable_irqs(struct vcpu *v, uint32_t r, unsigned int n)
{
    const unsigned long mask = r;
    struct pending_irq *p;
    struct irq_desc *desc;
    unsigned int irq;
    unsigned long flags;
    unsigned int i = 0;
    struct vcpu *v_target;

    /* LPIs will never be disabled via this function. */
    ASSERT(!is_lpi(32 * n + 31));

    while ( (i = find_next_bit(&mask, 32, i)) < 32 ) {
        irq = i + (32 * n);
        v_target = vgic_get_target_vcpu(v, irq);

        spin_lock_irqsave(&v_target->arch.vgic.lock, flags);
        p = irq_to_pending(v_target, irq);
        clear_bit(GIC_IRQ_GUEST_ENABLED, &p->status);
        gic_remove_from_lr_pending(v_target, p);
        desc = p->desc;
        spin_unlock_irqrestore(&v_target->arch.vgic.lock, flags);

        if ( desc != NULL )
        {
            spin_lock_irqsave(&desc->lock, flags);
            desc->handler->disable(desc);
            spin_unlock_irqrestore(&desc->lock, flags);
        }
        i++;
    }
}

#define VGIC_ICFG_MASK(intr) (1U << ((2 * ((intr) % 16)) + 1))

/* The function should be called with the rank lock taken */
static inline unsigned int vgic_get_virq_type(struct vcpu *v,
                                              unsigned int n,
                                              unsigned int index)
{
    struct vgic_irq_rank *r = vgic_get_rank(v, n);
    uint32_t tr = r->icfg[index >> 4];

    ASSERT(spin_is_locked(&r->lock));

    if ( tr & VGIC_ICFG_MASK(index) )
        return IRQ_TYPE_EDGE_RISING;
    else
        return IRQ_TYPE_LEVEL_HIGH;
}

void vgic_enable_irqs(struct vcpu *v, uint32_t r, unsigned int n)
{
    const unsigned long mask = r;
    struct pending_irq *p;
    unsigned int irq;
    unsigned long flags;
    unsigned int i = 0;
    struct vcpu *v_target;
    struct domain *d = v->domain;

    /* LPIs will never be enabled via this function. */
    ASSERT(!is_lpi(32 * n + 31));

    while ( (i = find_next_bit(&mask, 32, i)) < 32 ) {
        irq = i + (32 * n);
        v_target = vgic_get_target_vcpu(v, irq);
        spin_lock_irqsave(&v_target->arch.vgic.lock, flags);
        p = irq_to_pending(v_target, irq);
        set_bit(GIC_IRQ_GUEST_ENABLED, &p->status);
        if ( !list_empty(&p->inflight) && !test_bit(GIC_IRQ_GUEST_VISIBLE, &p->status) )
            gic_raise_guest_irq(v_target, irq, p->priority);
        spin_unlock_irqrestore(&v_target->arch.vgic.lock, flags);
        if ( p->desc != NULL )
        {
            irq_set_affinity(p->desc, cpumask_of(v_target->processor));
            spin_lock_irqsave(&p->desc->lock, flags);
            /*
             * The irq cannot be a PPI, we only support delivery of SPIs
             * to guests.
             */
            ASSERT(irq >= 32);
            if ( irq_type_set_by_domain(d) )
                gic_set_irq_type(p->desc, vgic_get_virq_type(v, n, i));
            p->desc->handler->enable(p->desc);
            spin_unlock_irqrestore(&p->desc->lock, flags);
        }
        i++;
    }
}

void vgic_set_irqs_pending(struct vcpu *v, uint32_t r, unsigned int rank)
{
    /* The first rank is always per-vCPU */
    bool private = rank == 0;

    /* LPIs will never be set pending via this function */
    ASSERT(!is_lpi(32 * rank + 31));

    for_each_set_bit ( i, r )
    {
        unsigned int irq = i + 32 * rank;

        if ( !private )
        {
            struct pending_irq *p = spi_to_pending(v->domain, irq);

            /*
             * When the domain sets the pending state for a HW interrupt on
             * the virtual distributor, we set the pending state on the
             * physical distributor.
             *
             * XXX: Investigate whether we would be able to set the
             * physical interrupt active and save an interruption. (This
             * is what the new vGIC does).
             */
            if ( p->desc != NULL )
            {
                unsigned long flags;

                spin_lock_irqsave(&p->desc->lock, flags);
                gic_set_pending_state(p->desc, true);
                spin_unlock_irqrestore(&p->desc->lock, flags);
                continue;
            }
        }

        /*
         * If the interrupt is per-vCPU, then we want to inject the vIRQ
         * to v, otherwise we should let the function figuring out the
         * correct vCPU.
         */
        vgic_inject_irq(v->domain, private ? v : NULL, irq, true);
    }
}

bool vgic_to_sgi(struct vcpu *v, register_t sgir, enum gic_sgi_mode irqmode,
                 int virq, const struct sgi_target *target)
{
    struct domain *d = v->domain;
    unsigned int base, bitmap;

    ASSERT( virq < 16 );

    switch ( irqmode )
    {
    case SGI_TARGET_LIST:
        perfc_incr(vgic_sgi_list);
        base = target->aff1 << 4;
        bitmap = target->list;

        for_each_set_bit ( i, bitmap )
        {
            unsigned int vcpuid = base + i;

            if ( vcpuid >= d->max_vcpus || d->vcpu[vcpuid] == NULL ||
                 !is_vcpu_online(d->vcpu[vcpuid]) )
            {
                gprintk(XENLOG_WARNING,
                        "vGIC: write %#"PRIregister", target->list=%#x, bad target vcpu%u\n",
                        sgir, target->list, vcpuid);
                continue;
            }
            vgic_inject_irq(d, d->vcpu[vcpuid], virq, true);
        }
        break;
    case SGI_TARGET_OTHERS:
        perfc_incr(vgic_sgi_others);
        for ( unsigned int i = 0; i < d->max_vcpus; i++ )
        {
            if ( i != current->vcpu_id && d->vcpu[i] != NULL &&
                 is_vcpu_online(d->vcpu[i]) )
                vgic_inject_irq(d, d->vcpu[i], virq, true);
        }
        break;
    case SGI_TARGET_SELF:
        perfc_incr(vgic_sgi_self);
        vgic_inject_irq(d, current, virq, true);
        break;
    default:
        gprintk(XENLOG_WARNING,
                "vGICD: GICD_SGIR write %#"PRIregister" with unhandled mode %d\n",
                sgir, irqmode);
        return false;
    }

    return true;
}

/*
 * Returns the pointer to the struct pending_irq belonging to the given
 * interrupt.
 * This can return NULL if called for an LPI which has been unmapped
 * meanwhile.
 */
struct pending_irq *irq_to_pending(struct vcpu *v, unsigned int irq)
{
    struct pending_irq *n;
    /* Pending irqs allocation strategy: the first vgic.nr_spis irqs
     * are used for SPIs; the rests are used for per cpu irqs */
    if ( irq < 32 )
        n = &v->arch.vgic.pending_irqs[irq];
    else if ( is_lpi(irq) )
        n = v->domain->arch.vgic.handler->lpi_to_pending(v->domain, irq);
    else
        n = &v->domain->arch.vgic.pending_irqs[irq - 32];
    return n;
}

struct pending_irq *spi_to_pending(struct domain *d, unsigned int irq)
{
    ASSERT(irq >= NR_LOCAL_IRQS);

    return &d->arch.vgic.pending_irqs[irq - 32];
}

void vgic_clear_pending_irqs(struct vcpu *v)
{
    struct pending_irq *p, *t;
    unsigned long flags;

    spin_lock_irqsave(&v->arch.vgic.lock, flags);
    list_for_each_entry_safe ( p, t, &v->arch.vgic.inflight_irqs, inflight )
        list_del_init(&p->inflight);
    gic_clear_pending_irqs(v);
    spin_unlock_irqrestore(&v->arch.vgic.lock, flags);
}

void vgic_remove_irq_from_queues(struct vcpu *v, struct pending_irq *p)
{
    ASSERT(spin_is_locked(&v->arch.vgic.lock));

    clear_bit(GIC_IRQ_GUEST_QUEUED, &p->status);
    list_del_init(&p->inflight);
    gic_remove_from_lr_pending(v, p);
}

void vgic_inject_irq(struct domain *d, struct vcpu *v, unsigned int virq,
                     bool level)
{
    uint8_t priority;
    struct pending_irq *iter, *n;
    unsigned long flags;

    /*
     * For edge triggered interrupts we always ignore a "falling edge".
     * For level triggered interrupts we shouldn't, but do anyways.
     */
    if ( !level )
        return;

    if ( !v )
    {
        /* The IRQ needs to be an SPI if no vCPU is specified. */
        ASSERT(virq >= 32 && virq <= vgic_num_irqs(d));

        v = vgic_get_target_vcpu(d->vcpu[0], virq);
    };

    spin_lock_irqsave(&v->arch.vgic.lock, flags);

    n = irq_to_pending(v, virq);
    /* If an LPI has been removed, there is nothing to inject here. */
    if ( unlikely(!n) )
    {
        spin_unlock_irqrestore(&v->arch.vgic.lock, flags);
        return;
    }

    /* vcpu offline */
    if ( test_bit(_VPF_down, &v->pause_flags) )
    {
        spin_unlock_irqrestore(&v->arch.vgic.lock, flags);
        return;
    }

    set_bit(GIC_IRQ_GUEST_QUEUED, &n->status);

    if ( !list_empty(&n->inflight) )
    {
        gic_raise_inflight_irq(v, virq);
        goto out;
    }

    priority = vgic_get_virq_priority(v, virq);
    n->priority = priority;

    /* the irq is enabled */
    if ( test_bit(GIC_IRQ_GUEST_ENABLED, &n->status) )
        gic_raise_guest_irq(v, virq, priority);

    list_for_each_entry ( iter, &v->arch.vgic.inflight_irqs, inflight )
    {
        if ( iter->priority > priority )
        {
            list_add_tail(&n->inflight, &iter->inflight);
            goto out;
        }
    }
    list_add_tail(&n->inflight, &v->arch.vgic.inflight_irqs);
out:
    spin_unlock_irqrestore(&v->arch.vgic.lock, flags);

    /* we have a new higher priority irq, inject it into the guest */
    vcpu_kick(v);

    return;
}

bool vgic_evtchn_irq_pending(struct vcpu *v)
{
    struct pending_irq *p;

    p = irq_to_pending(v, v->domain->arch.evtchn_irq);
    /* Does not work for LPIs. */
    ASSERT(!is_lpi(v->domain->arch.evtchn_irq));

    return list_empty(&p->inflight);
}

bool vgic_emulate(struct cpu_user_regs *regs, union hsr hsr)
{
    struct vcpu *v = current;

    ASSERT(v->domain->arch.vgic.handler->emulate_reg != NULL);

    return v->domain->arch.vgic.handler->emulate_reg(regs, hsr);
}

bool vgic_reserve_virq(struct domain *d, unsigned int virq)
{
    if ( virq >= vgic_num_irqs(d) )
        return false;

    return !test_and_set_bit(virq, d->arch.vgic.allocated_irqs);
}

int vgic_allocate_virq(struct domain *d, bool spi)
{
    int first, end;
    unsigned int virq;

    if ( !spi )
    {
        /* We only allocate PPIs. SGIs are all reserved */
        first = 16;
        end = 32;
    }
    else
    {
        first = 32;
        end = vgic_num_irqs(d);
    }

    /*
     * There is no spinlock to protect allocated_irqs, therefore
     * test_and_set_bit may fail. If so retry it.
     */
    do
    {
        virq = find_next_zero_bit(d->arch.vgic.allocated_irqs, end, first);
        if ( virq >= end )
            return -1;
    }
    while ( test_and_set_bit(virq, d->arch.vgic.allocated_irqs) );

    return virq;
}

void vgic_free_virq(struct domain *d, unsigned int virq)
{
    clear_bit(virq, d->arch.vgic.allocated_irqs);
}

unsigned int vgic_max_vcpus(unsigned int domctl_vgic_version)
{
    switch ( domctl_vgic_version )
    {
    case XEN_DOMCTL_CONFIG_GIC_V2:
        return 8;

#ifdef CONFIG_GICV3
    case XEN_DOMCTL_CONFIG_GIC_V3:
        return 4096;
#endif

    default:
        return 0;
    }
}

void vgic_check_inflight_irqs_pending(struct vcpu *v, unsigned int rank, uint32_t r)
{
    for_each_set_bit ( i, r )
    {
        struct pending_irq *p;
        struct vcpu *v_target;
        unsigned long flags;
        unsigned int irq = i + 32 * rank;

        v_target = vgic_get_target_vcpu(v, irq);

        spin_lock_irqsave(&v_target->arch.vgic.lock, flags);

        p = irq_to_pending(v_target, irq);

        if ( p && !list_empty(&p->inflight) )
            printk(XENLOG_G_WARNING
                   "%pv trying to clear pending interrupt %u.\n",
                   v, irq);

        spin_unlock_irqrestore(&v_target->arch.vgic.lock, flags);
    }
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */

