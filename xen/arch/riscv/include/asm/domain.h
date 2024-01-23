/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ASM_RISCV_DOMAIN_H__
#define __ASM_RISCV_DOMAIN_H__

#include <xen/xmalloc.h>
#include <public/hvm/params.h>

struct hvm_domain
{
    uint64_t              params[HVM_NR_PARAMS];
};

#define is_domain_direct_mapped(d) ((void)(d), 0)

struct arch_vcpu_io {
};

struct arch_vcpu {
};

struct arch_domain {
    struct hvm_domain hvm;
};

#include <xen/sched.h>

static inline struct vcpu_guest_context *alloc_vcpu_guest_context(void)
{
    return xmalloc(struct vcpu_guest_context);
}

static inline void free_vcpu_guest_context(struct vcpu_guest_context *vgc)
{
    xfree(vgc);
}

struct guest_memory_policy {};
static inline void update_guest_memory_policy(struct vcpu *v,
                                              struct guest_memory_policy *gmp)
{}

static inline void arch_vcpu_block(struct vcpu *v) {}

#endif /* __ASM_RISCV_DOMAIN_H__ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
