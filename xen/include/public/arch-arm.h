/* SPDX-License-Identifier: MIT */
/******************************************************************************
 * arch-arm.h
 *
 * Guest OS interface to ARM Xen.
 *
 * Copyright 2011 (C) Citrix Systems
 */

#ifndef __XEN_PUBLIC_ARCH_ARM_H__
#define __XEN_PUBLIC_ARCH_ARM_H__

/*
 * `incontents 50 arm_abi Hypercall Calling Convention
 *
 * A hypercall is issued using the ARM HVC instruction.
 *
 * A hypercall can take up to 5 arguments. These are passed in
 * registers, the first argument in x0/r0 (for arm64/arm32 guests
 * respectively irrespective of whether the underlying hypervisor is
 * 32- or 64-bit), the second argument in x1/r1, the third in x2/r2,
 * the forth in x3/r3 and the fifth in x4/r4.
 *
 * The hypercall number is passed in r12 (arm) or x16 (arm64). In both
 * cases the relevant ARM procedure calling convention specifies this
 * is an inter-procedure-call scratch register (e.g. for use in linker
 * stubs). This use does not conflict with use during a hypercall.
 *
 * The HVC ISS must contain a Xen specific TAG: XEN_HYPERCALL_TAG.
 *
 * The return value is in x0/r0.
 *
 * The hypercall will clobber x16/r12 and the argument registers used
 * by that hypercall (except r0 which is the return value) i.e. in
 * addition to x16/r12 a 2 argument hypercall will clobber x1/r1 and a
 * 4 argument hypercall will clobber x1/r1, x2/r2 and x3/r3.
 *
 * Parameter structs passed to hypercalls are laid out according to
 * the Procedure Call Standard for the ARM Architecture (AAPCS, AKA
 * EABI) and Procedure Call Standard for the ARM 64-bit Architecture
 * (AAPCS64). Where there is a conflict the 64-bit standard should be
 * used regardless of guest type. Structures which are passed as
 * hypercall arguments are always little endian.
 *
 * All memory which is shared with other entities in the system
 * (including the hypervisor and other guests) must reside in memory
 * which is mapped as Normal Inner Write-Back Outer Write-Back Inner-Shareable.
 * This applies to:
 *  - hypercall arguments passed via a pointer to guest memory.
 *  - memory shared via the grant table mechanism (including PV I/O
 *    rings etc).
 *  - memory shared with the hypervisor (struct shared_info, struct
 *    vcpu_info, the grant table, etc).
 *
 * Any cache allocation hints are acceptable.
 */

/*
 * `incontents 55 arm_hcall Supported Hypercalls
 *
 * Xen on ARM makes extensive use of hardware facilities and therefore
 * only a subset of the potential hypercalls are required.
 *
 * Since ARM uses second stage paging any machine/physical addresses
 * passed to hypercalls are Guest Physical Addresses (Intermediate
 * Physical Addresses) unless otherwise noted.
 *
 * The following hypercalls (and sub operations) are supported on the
 * ARM platform. Other hypercalls should be considered
 * unavailable/unsupported.
 *
 *  HYPERVISOR_memory_op
 *   All generic sub-operations
 *
 *  HYPERVISOR_domctl
 *   All generic sub-operations, with the exception of:
 *    * XEN_DOMCTL_irq_permission (not yet implemented)
 *
 *  HYPERVISOR_sched_op
 *   All generic sub-operations, with the exception of:
 *    * SCHEDOP_block -- prefer wfi hardware instruction
 *
 *  HYPERVISOR_console_io
 *   All generic sub-operations
 *
 *  HYPERVISOR_xen_version
 *   All generic sub-operations
 *
 *  HYPERVISOR_event_channel_op
 *   All generic sub-operations
 *
 *  HYPERVISOR_physdev_op
 *   Exactly these sub-operations are supported:
 *   PHYSDEVOP_pci_device_add
 *   PHYSDEVOP_pci_device_remove
 *
 *  HYPERVISOR_sysctl
 *   All generic sub-operations, with the exception of:
 *    * XEN_SYSCTL_page_offline_op
 *    * XEN_SYSCTL_get_pmstat
 *    * XEN_SYSCTL_pm_op
 *
 *  HYPERVISOR_hvm_op
 *   Exactly these sub-operations are supported:
 *    * HVMOP_set_param
 *    * HVMOP_get_param
 *    * HVMOP_guest_request_vm_event
 *
 *  HYPERVISOR_grant_table_op
 *   All generic sub-operations
 *
 *  HYPERVISOR_vcpu_op
 *   Exactly these sub-operations are supported:
 *    * VCPUOP_register_vcpu_info
 *    * VCPUOP_register_runstate_memory_area
 *
 *  HYPERVISOR_argo_op
 *   All generic sub-operations
 *
 *  HYPERVISOR_hypfs_op
 *   All generic sub-operations
 *
 *  HYPERVISOR_platform_op
 *   Exactly these sub-operations are supported:
 *    * XENPF_settime64
 *
 *  HYPERVISOR_vm_assist
 *   All generic sub-operations
 *
 *  HYPERVISOR_dm_op
 *   Exactly these sub-operations are supported:
 *    * XEN_DMOP_create_ioreq_server
 *    * XEN_DMOP_get_ioreq_server_info
 *    * XEN_DMOP_map_io_range_to_ioreq_server
 *    * XEN_DMOP_unmap_io_range_from_ioreq_server
 *    * XEN_DMOP_set_ioreq_server_state
 *    * XEN_DMOP_destroy_ioreq_server
 *    * XEN_DMOP_set_irq_level
 *    * XEN_DMOP_nr_vcpus
 *
 *  HYPERVISOR_xsm_op
 *   All generic sub-operations
 *
 *  HYPERVISOR_multicall
 *
 * Other notes on the ARM ABI:
 *
 * - struct start_info is not exported to ARM guests.
 *
 * - struct shared_info is mapped by ARM guests using the
 *   HYPERVISOR_memory_op sub-op XENMEM_add_to_physmap, passing
 *   XENMAPSPACE_shared_info as space parameter.
 *
 * - All the per-cpu struct vcpu_info are mapped by ARM guests using the
 *   HYPERVISOR_vcpu_op sub-op VCPUOP_register_vcpu_info, including cpu0
 *   struct vcpu_info.
 *
 * - The grant table is mapped using the HYPERVISOR_memory_op sub-op
 *   XENMEM_add_to_physmap, passing XENMAPSPACE_grant_table as space
 *   parameter. The memory range specified under the Xen compatible
 *   hypervisor node on device tree can be used as target gpfn for the
 *   mapping.
 *
 * - Xenstore is initialized by using the two hvm_params
 *   HVM_PARAM_STORE_PFN and HVM_PARAM_STORE_EVTCHN. They can be read
 *   with the HYPERVISOR_hvm_op sub-op HVMOP_get_param.
 *
 * - The paravirtualized console is initialized by using the two
 *   hvm_params HVM_PARAM_CONSOLE_PFN and HVM_PARAM_CONSOLE_EVTCHN. They
 *   can be read with the HYPERVISOR_hvm_op sub-op HVMOP_get_param.
 *
 * - Event channel notifications are delivered using the percpu GIC
 *   interrupt specified under the Xen compatible hypervisor node on
 *   device tree.
 *
 * - The device tree Xen compatible node is fully described under Linux
 *   at Documentation/devicetree/bindings/arm/xen.txt.
 */

#define XEN_HYPERCALL_TAG   0XEA1

#if defined(__XEN__) || defined(__XEN_TOOLS__) || defined(__GNUC__)
#define  int64_aligned_t  int64_t __attribute__((__aligned__(8)))
#define uint64_aligned_t uint64_t __attribute__((__aligned__(8)))
#endif

#ifndef __ASSEMBLY__
#define ___DEFINE_XEN_GUEST_HANDLE(name, type)                  \
    typedef union { type *p; unsigned long q; }                 \
        __guest_handle_ ## name;                                \
    typedef union { type *p; uint64_aligned_t q; }              \
        __guest_handle_64_ ## name

/*
 * XEN_GUEST_HANDLE represents a guest pointer, when passed as a field
 * in a struct in memory. On ARM is always 8 bytes sizes and 8 bytes
 * aligned.
 * XEN_GUEST_HANDLE_PARAM represents a guest pointer, when passed as an
 * hypercall argument. It is 4 bytes on aarch32 and 8 bytes on aarch64.
 */
#define __DEFINE_XEN_GUEST_HANDLE(name, type) \
    ___DEFINE_XEN_GUEST_HANDLE(name, type);   \
    ___DEFINE_XEN_GUEST_HANDLE(const_##name, const type)
#define DEFINE_XEN_GUEST_HANDLE(name)   __DEFINE_XEN_GUEST_HANDLE(name, name)
#define __XEN_GUEST_HANDLE(name)        __guest_handle_64_ ## name
#define XEN_GUEST_HANDLE(name)          __XEN_GUEST_HANDLE(name)
#define XEN_GUEST_HANDLE_PARAM(name)    __guest_handle_ ## name
#define set_xen_guest_handle_raw(hnd, val)                  \
    do {                                                    \
        __typeof__(&(hnd)) _sxghr_tmp = &(hnd);             \
        _sxghr_tmp->q = 0;                                  \
        _sxghr_tmp->p = (val);                              \
    } while ( 0 )
#define set_xen_guest_handle(hnd, val) set_xen_guest_handle_raw(hnd, val)

typedef uint64_t xen_pfn_t;
#define PRI_xen_pfn PRIx64
#define PRIu_xen_pfn PRIu64

/*
 * Maximum number of virtual CPUs in legacy multi-processor guests.
 * Only one. All other VCPUS must use VCPUOP_register_vcpu_info.
 */
#define XEN_LEGACY_MAX_VCPUS 1

typedef uint64_t xen_ulong_t;
#define PRI_xen_ulong PRIx64

#if defined(__XEN__) || defined(__XEN_TOOLS__)
#if defined(__GNUC__) && !defined(__STRICT_ANSI__)
/* Anonymous union includes both 32- and 64-bit names (e.g., r0/x0). */
# define __DECL_REG(n64, n32) union {          \
        uint64_t n64;                          \
        uint32_t n32;                          \
    }
#else
/* Non-gcc sources must always use the proper 64-bit name (e.g., x0). */
#define __DECL_REG(n64, n32) uint64_t n64
#endif

struct vcpu_guest_core_regs
{
    /*         Aarch64       Aarch32 */
    __DECL_REG(x0,           r0_usr);
    __DECL_REG(x1,           r1_usr);
    __DECL_REG(x2,           r2_usr);
    __DECL_REG(x3,           r3_usr);
    __DECL_REG(x4,           r4_usr);
    __DECL_REG(x5,           r5_usr);
    __DECL_REG(x6,           r6_usr);
    __DECL_REG(x7,           r7_usr);
    __DECL_REG(x8,           r8_usr);
    __DECL_REG(x9,           r9_usr);
    __DECL_REG(x10,          r10_usr);
    __DECL_REG(x11,          r11_usr);
    __DECL_REG(x12,          r12_usr);

    __DECL_REG(x13,          sp_usr);
    __DECL_REG(x14,          lr_usr);

    __DECL_REG(x15,          __unused_sp_hyp);

    __DECL_REG(x16,          lr_irq);
    __DECL_REG(x17,          sp_irq);

    __DECL_REG(x18,          lr_svc);
    __DECL_REG(x19,          sp_svc);

    __DECL_REG(x20,          lr_abt);
    __DECL_REG(x21,          sp_abt);

    __DECL_REG(x22,          lr_und);
    __DECL_REG(x23,          sp_und);

    __DECL_REG(x24,          r8_fiq);
    __DECL_REG(x25,          r9_fiq);
    __DECL_REG(x26,          r10_fiq);
    __DECL_REG(x27,          r11_fiq);
    __DECL_REG(x28,          r12_fiq);

    __DECL_REG(x29,          sp_fiq);
    __DECL_REG(x30,          lr_fiq);

    /* Return address and mode */
    __DECL_REG(pc64,         pc32);             /* ELR_EL2 */
    uint64_t cpsr;                              /* SPSR_EL2 */

    union {
        uint64_t spsr_el1;       /* AArch64 */
        uint32_t spsr_svc;       /* AArch32 */
    };

    /* AArch32 guests only */
    uint32_t spsr_fiq, spsr_irq, spsr_und, spsr_abt;

    /* AArch64 guests only */
    uint64_t sp_el0;
    uint64_t sp_el1, elr_el1;
};
typedef struct vcpu_guest_core_regs vcpu_guest_core_regs_t;
DEFINE_XEN_GUEST_HANDLE(vcpu_guest_core_regs_t);

#undef __DECL_REG

struct vcpu_guest_context {
#define _VGCF_online                   0
#define VGCF_online                    (1<<_VGCF_online)
    uint32_t flags;                         /* VGCF_* */

    struct vcpu_guest_core_regs user_regs;  /* Core CPU registers */

    uint64_t sctlr;
    uint64_t ttbcr, ttbr0, ttbr1;
};
typedef struct vcpu_guest_context vcpu_guest_context_t;
DEFINE_XEN_GUEST_HANDLE(vcpu_guest_context_t);

/*
 * struct xen_arch_domainconfig's ABI is covered by
 * XEN_DOMCTL_INTERFACE_VERSION.
 */
#define XEN_DOMCTL_CONFIG_GIC_NATIVE    0
#define XEN_DOMCTL_CONFIG_GIC_V2        1
#define XEN_DOMCTL_CONFIG_GIC_V3        2

#define XEN_DOMCTL_CONFIG_TEE_NONE      0
#define XEN_DOMCTL_CONFIG_TEE_OPTEE     1
#define XEN_DOMCTL_CONFIG_TEE_FFA       2

struct xen_arch_domainconfig {
    /* IN/OUT */
    uint8_t gic_version;
    /* IN - Contains SVE vector length divided by 128 */
    uint8_t sve_vl;
    /* IN */
    uint16_t tee_type;
    /* IN */
    uint32_t nr_spis;
    /*
     * OUT
     * Based on the property clock-frequency in the DT timer node.
     * The property may be present when the bootloader/firmware doesn't
     * set correctly CNTFRQ which hold the timer frequency.
     *
     * As it's not possible to trap this register, we have to replicate
     * the value in the guest DT.
     *
     * = 0 => property not present
     * > 0 => Value of the property
     *
     */
    uint32_t clock_frequency;
};
#endif /* __XEN__ || __XEN_TOOLS__ */

struct arch_vcpu_info {
};
typedef struct arch_vcpu_info arch_vcpu_info_t;

struct arch_shared_info {
};
typedef struct arch_shared_info arch_shared_info_t;
typedef uint64_t xen_callback_t;

#endif

#if defined(__XEN__) || defined(__XEN_TOOLS__)

/* PSR bits (CPSR, SPSR) */

#define PSR_THUMB       (1U <<5)      /* Thumb Mode enable */
#define PSR_FIQ_MASK    (1U <<6)      /* Fast Interrupt mask */
#define PSR_IRQ_MASK    (1U <<7)      /* Interrupt mask */
#define PSR_ABT_MASK    (1U <<8)      /* Asynchronous Abort mask */
#define PSR_BIG_ENDIAN  (1U << 9)     /* arm32: Big Endian Mode */
#define PSR_DBG_MASK    (1U << 9)     /* arm64: Debug Exception mask */
#define PSR_IT_MASK     (0x0600fc00U) /* Thumb If-Then Mask */
#define PSR_JAZELLE     (1U << 24)    /* Jazelle Mode */
#define PSR_Z           (1U << 30)    /* Zero condition flag */

/* 32 bit modes */
#define PSR_MODE_USR 0x10U
#define PSR_MODE_FIQ 0x11U
#define PSR_MODE_IRQ 0x12U
#define PSR_MODE_SVC 0x13U
#define PSR_MODE_MON 0x16U
#define PSR_MODE_ABT 0x17U
#define PSR_MODE_HYP 0x1aU
#define PSR_MODE_UND 0x1bU
#define PSR_MODE_SYS 0x1fU

/* 64 bit modes */
#define PSR_MODE_BIT  0x10U /* Set iff AArch32 */
#define PSR_MODE_EL3h 0x0dU
#define PSR_MODE_EL3t 0x0cU
#define PSR_MODE_EL2h 0x09U
#define PSR_MODE_EL2t 0x08U
#define PSR_MODE_EL1h 0x05U
#define PSR_MODE_EL1t 0x04U
#define PSR_MODE_EL0t 0x00U

/*
 * We set PSR_Z to be able to boot Linux kernel versions with an invalid
 * encoding of the first 8 NOP instructions. See commit a92882a4d270 in
 * Linux.
 *
 * Note that PSR_Z is also set by U-Boot and QEMU -kernel when loading
 * zImage kernels on aarch32.
 */
#define PSR_GUEST32_INIT (PSR_Z|PSR_ABT_MASK|PSR_FIQ_MASK|PSR_IRQ_MASK|PSR_MODE_SVC)
#define PSR_GUEST64_INIT (PSR_ABT_MASK|PSR_FIQ_MASK|PSR_IRQ_MASK|PSR_MODE_EL1h)

#define SCTLR_GUEST_INIT    xen_mk_ullong(0x00c50078)

/*
 * Virtual machine platform (memory layout, interrupts)
 *
 * These are defined for consistency between the tools and the
 * hypervisor. Guests must not rely on these hardcoded values but
 * should instead use the FDT.
 */

/* Physical Address Space */

/* Virtio MMIO mappings */
#define GUEST_VIRTIO_MMIO_BASE   xen_mk_ullong(0x02000000)
#define GUEST_VIRTIO_MMIO_SIZE   xen_mk_ullong(0x00100000)

/*
 * vGIC mappings: Only one set of mapping is used by the guest.
 * Therefore they can overlap.
 */

/* vGIC v2 mappings */
#define GUEST_GICD_BASE   xen_mk_ullong(0x03001000)
#define GUEST_GICD_SIZE   xen_mk_ullong(0x00001000)
#define GUEST_GICC_BASE   xen_mk_ullong(0x03002000)
#define GUEST_GICC_SIZE   xen_mk_ullong(0x00002000)

/* vGIC v3 mappings */
#define GUEST_GICV3_GICD_BASE      xen_mk_ullong(0x03001000)
#define GUEST_GICV3_GICD_SIZE      xen_mk_ullong(0x00010000)

#define GUEST_GICV3_RDIST_REGIONS  1

#define GUEST_GICV3_GICR0_BASE     xen_mk_ullong(0x03020000) /* vCPU0..127 */
#define GUEST_GICV3_GICR0_SIZE     xen_mk_ullong(0x01000000)

/*
 * 256 MB is reserved for VPCI configuration space based on calculation
 * 256 buses x 32 devices x 8 functions x 4 KB = 256 MB
 */
#define GUEST_VPCI_ECAM_BASE    xen_mk_ullong(0x10000000)
#define GUEST_VPCI_ECAM_SIZE    xen_mk_ullong(0x10000000)

/* vGIC ITS mappings */
#define GUEST_GICV3_ITS_BASE   xen_mk_ullong(0x04020000)
#define GUEST_GICV3_ITS_SIZE   xen_mk_ullong(0x00020000)

/* ACPI tables physical address */
#define GUEST_ACPI_BASE xen_mk_ullong(0x20000000)
#define GUEST_ACPI_SIZE xen_mk_ullong(0x02000000)

/* PL011 mappings */
#define GUEST_PL011_BASE    xen_mk_ullong(0x22000000)
#define GUEST_PL011_SIZE    xen_mk_ullong(0x00001000)

/* Guest PCI-PCIe memory space where config space and BAR will be available.*/
#define GUEST_VPCI_ADDR_TYPE_MEM            xen_mk_ullong(0x02000000)
#define GUEST_VPCI_MEM_ADDR                 xen_mk_ullong(0x23000000)
#define GUEST_VPCI_MEM_SIZE                 xen_mk_ullong(0x10000000)

/*
 * 16MB == 4096 pages reserved for guest to use as a region to map its
 * grant table in.
 */
#define GUEST_GNTTAB_BASE xen_mk_ullong(0x38000000)
#define GUEST_GNTTAB_SIZE xen_mk_ullong(0x01000000)

#define GUEST_MAGIC_BASE  xen_mk_ullong(0x39000000)
#define GUEST_MAGIC_SIZE  xen_mk_ullong(0x01000000)

#define GUEST_RAM_BANKS   2

/*
 * The way to find the extended regions (to be exposed to the guest as unused
 * address space) relies on the fact that the regions reserved for the RAM
 * below are big enough to also accommodate such regions.
 */
#define GUEST_RAM0_BASE   xen_mk_ullong(0x40000000) /* 3GB of low RAM @ 1GB */
#define GUEST_RAM0_SIZE   xen_mk_ullong(0xc0000000)

/* 4GB @ 4GB Prefetch Memory for VPCI */
#define GUEST_VPCI_ADDR_TYPE_PREFETCH_MEM   xen_mk_ullong(0x43000000)
#define GUEST_VPCI_PREFETCH_MEM_ADDR        xen_mk_ullong(0x100000000)
#define GUEST_VPCI_PREFETCH_MEM_SIZE        xen_mk_ullong(0x100000000)

#define GUEST_RAM1_BASE   xen_mk_ullong(0x0200000000) /* 1016GB of RAM @ 8GB */
#define GUEST_RAM1_SIZE   xen_mk_ullong(0xfe00000000)

#define GUEST_RAM_BASE    GUEST_RAM0_BASE /* Lowest RAM address */
/* Largest amount of actual RAM, not including holes */
#define GUEST_RAM_MAX     (GUEST_RAM0_SIZE + GUEST_RAM1_SIZE)
/* Suitable for e.g. const uint64_t ramfoo[] = GUEST_RAM_BANK_FOOS; */
#define GUEST_RAM_BANK_BASES   { GUEST_RAM0_BASE, GUEST_RAM1_BASE }
#define GUEST_RAM_BANK_SIZES   { GUEST_RAM0_SIZE, GUEST_RAM1_SIZE }

/* Current supported guest VCPUs */
#define GUEST_MAX_VCPUS 128

/* Interrupts */

#define GUEST_TIMER_VIRT_PPI    27
#define GUEST_TIMER_PHYS_S_PPI  29
#define GUEST_TIMER_PHYS_NS_PPI 30
#define GUEST_EVTCHN_PPI        31

#define GUEST_VPL011_SPI        32

#define GUEST_VIRTIO_MMIO_SPI_FIRST   33
#define GUEST_VIRTIO_MMIO_SPI_LAST    43

/*
 * SGI is the preferred delivery mechanism of FF-A pending notifications or
 * schedule recveive interrupt. SGIs 8-15 are normally not used by a guest
 * as they in a non-virtualized system typically are assigned to the secure
 * world. Here we're free to use SGI 8-15 since they are virtual and have
 * nothing to do with the secure world.
 *
 * For partitioning of SGIs see also Arm Base System Architecture v1.0C,
 * https://developer.arm.com/documentation/den0094/
 */
#define GUEST_FFA_NOTIF_PEND_INTR_ID      8
#define GUEST_FFA_SCHEDULE_RECV_INTR_ID   9

/* PSCI functions */
#define PSCI_cpu_suspend 0
#define PSCI_cpu_off     1
#define PSCI_cpu_on      2
#define PSCI_migrate     3

#endif

#ifndef __ASSEMBLY__
/* Stub definition of PMU structure */
typedef struct xen_pmu_arch { uint8_t dummy; } xen_pmu_arch_t;
#endif

#endif /*  __XEN_PUBLIC_ARCH_ARM_H__ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
