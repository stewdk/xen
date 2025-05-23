/*
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

#ifndef __ARM_PCI_H__
#define __ARM_PCI_H__

#ifdef CONFIG_HAS_PCI

#include <asm/p2m.h>

#define pci_to_dev(pcidev) (&(pcidev)->arch.dev)

extern bool pci_passthrough_enabled;
extern bool pci_scan_enabled;

/* Arch pci dev struct */
struct arch_pci_dev {
    struct device dev;
};

/* Arch-specific MSI data for vPCI. */
struct vpci_arch_msi {
};

/* Arch-specific MSI-X entry data for vPCI. */
struct vpci_arch_msix_entry {
};

/*
 * Because of the header cross-dependencies, e.g. we need both
 * struct pci_dev and struct arch_pci_dev at the same time, this cannot be
 * done with an inline here. Macro can be implemented, but looks scary.
 */
struct pci_dev *dev_to_pci(struct device *dev);

/*
 * struct to hold the mappings of a config space window. This
 * is expected to be used as sysdata for PCI controllers that
 * use ECAM.
 */
struct pci_config_window {
    paddr_t         phys_addr;
    paddr_t         size;
    uint8_t         busn_start;
    uint8_t         busn_end;
    void __iomem    *win;
};

/*
 * struct to hold pci host bridge information
 * for a PCI controller.
 */
struct pci_host_bridge {
    struct dt_device_node *dt_node;  /* Pointer to the associated DT node */
    struct list_head node;           /* Node in list of host bridges */
    uint16_t segment;                /* Segment number */
    struct pci_config_window* cfg;   /* Pointer to the bridge config window */
    const struct pci_ops *ops;
    uint64_t its_msi_base;
    struct rangeset *bar_ranges;
    struct rangeset *bar_ranges_prefetch;
};

struct pci_ops {
    void __iomem *(*map_bus)(struct pci_host_bridge *bridge, pci_sbdf_t sbdf,
                             uint32_t offset);
    int (*read)(struct pci_host_bridge *bridge, pci_sbdf_t sbdf,
                uint32_t reg, uint32_t len, uint32_t *value);
    int (*write)(struct pci_host_bridge *bridge, pci_sbdf_t sbdf,
                 uint32_t reg, uint32_t len, uint32_t value);
    bool (*need_p2m_hwdom_mapping)(struct domain *d,
                                   struct pci_host_bridge *bridge,
                                   uint64_t addr);
};

/*
 * struct to hold pci ops and bus shift of the config window
 * for a PCI controller.
 */
struct pci_ecam_ops {
    unsigned int            bus_shift;
    struct pci_ops          pci_ops;
    int (*cfg_reg_index)(struct dt_device_node *dev);
    int (*init)(struct pci_config_window *);
};

/* Default ECAM ops */
extern const struct pci_ecam_ops pci_generic_ecam_ops;

int pci_host_common_probe(struct dt_device_node *dev,
                          const struct pci_ecam_ops *ops);
int pci_init_bridge(pci_sbdf_t sbdf);
int pci_generic_config_read(struct pci_host_bridge *bridge, pci_sbdf_t sbdf,
                            uint32_t reg, uint32_t len, uint32_t *value);
int pci_generic_config_write(struct pci_host_bridge *bridge, pci_sbdf_t sbdf,
                             uint32_t reg, uint32_t len, uint32_t value);
void __iomem *pci_ecam_map_bus(struct pci_host_bridge *bridge,
                               pci_sbdf_t sbdf, uint32_t where);
bool pci_ecam_need_p2m_hwdom_mapping(struct domain *d,
                                     struct pci_host_bridge *bridge,
                                     uint64_t addr);
struct pci_host_bridge *pci_find_host_bridge(uint16_t segment, uint8_t bus);
const struct dt_device_node *
pci_find_host_bridge_node(const struct pci_dev *pdev);
int pci_get_host_bridge_segment(const struct dt_device_node *node,
                                uint16_t *segment);

static always_inline bool is_pci_passthrough_enabled(void)
{
    return pci_passthrough_enabled;
}

static inline bool is_pci_scan_enabled(void)
{
    return pci_scan_enabled;
}

void arch_pci_init_pdev(struct pci_dev *pdev);

int pci_get_new_domain_nr(void);

int pci_host_iterate_bridges_and_count(struct domain *d,
                                       int (*cb)(struct domain *d,
                                                 struct pci_host_bridge *bridge));

int pci_host_bridge_mappings(struct domain *d);

bool pci_check_bar(const struct pci_dev *pdev, mfn_t start, mfn_t end);

bool arch_pci_device_physdevop(void);

uint64_t pci_get_new_bar_addr(const struct pci_dev *pdev, uint64_t size,
                              bool is_64bit, bool prefetch);
int pci_reserve_bar_range(const struct pci_dev *pdev, uint64_t addr,
                          uint64_t size, bool prefetch);

static inline int
pci_msi_conf_write_intercept(struct pci_dev *pdev, unsigned int reg,
                             unsigned int size, uint32_t *data)
{
    return 0;
}

#else   /*!CONFIG_HAS_PCI*/

static inline bool is_pci_scan_enabled(void)
{
    return false;
}

struct pci_dev;

static inline void arch_pci_init_pdev(struct pci_dev *pdev) {}

static inline int pci_get_host_bridge_segment(const struct dt_device_node *node,
                                              uint16_t *segment)
{
    ASSERT_UNREACHABLE();
    return -EINVAL;
}

static inline int pci_get_new_domain_nr(void)
{
    ASSERT_UNREACHABLE();
    return -1;
}

struct pci_host_bridge;

static inline int pci_host_iterate_bridges_and_count(
    struct domain *d,
    int (*cb)(struct domain *d, struct pci_host_bridge *bridge))
{
    return 0;
}

#endif  /*!CONFIG_HAS_PCI*/

#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
static inline bool hwdom_uses_vpci(void)
{
    return is_pci_scan_enabled();
}

#else  /*!CONFIG_HAS_VPCI_GUEST_SUPPORT*/

static inline bool hwdom_uses_vpci(void)
{
    return false;
}
#endif  /*!CONFIG_HAS_VPCI_GUEST_SUPPORT*/

#endif /* __ARM_PCI_H__ */
