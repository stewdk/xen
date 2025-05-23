/******************************************************************************
 * rangeset.h
 * 
 * Creation, maintenance and automatic destruction of per-domain sets of
 * numeric ranges.
 * 
 * Copyright (c) 2005, K A Fraser
 */

#ifndef __XEN_RANGESET_H__
#define __XEN_RANGESET_H__

#include <xen/types.h>

struct domain;
struct rangeset;

/*
 * Initialise/destroy per-domain rangeset information.
 * 
 * It is invalid to create or destroy a rangeset belonging to a domain @d
 * before rangeset_domain_initialise(d) returns or after calling
 * rangeset_domain_destroy(d).
 */
void rangeset_domain_initialise(
    struct domain *d);
void rangeset_domain_destroy(
    struct domain *d);

/*
 * Create/destroy a rangeset. Optionally attach to specified domain @d for
 * auto-destruction when the domain dies. A name may be specified, for use
 * in debug pretty-printing, and various RANGESETF flags (defined below).
 * 
 * It is invalid to perform any operation on a rangeset @r after calling
 * rangeset_destroy(r).
 */
struct rangeset *rangeset_new(
    struct domain *d, const char *name, unsigned int flags);
void rangeset_destroy(
    struct rangeset *r);

/*
 * Set a limit on the number of ranges that may exist in set @r.
 * NOTE: This must be called while @r is empty.
 */
void rangeset_limit(
    struct rangeset *r, unsigned int limit);

/* Flags for passing to rangeset_new(). */
 /* Pretty-print range limits in hexadecimal. */
#define RANGESETF_prettyprint_hex   (1U << 0)
 /* Do not print entries marked with this flag. */
#define RANGESETF_no_print          (1U << 1)

bool __must_check rangeset_is_empty(
    const struct rangeset *r);

/* Add/claim/find/remove/query/purge a numeric range. */
int __must_check rangeset_add_range(
    struct rangeset *r, unsigned long s, unsigned long e);
int __must_check rangeset_claim_range(struct rangeset *r, unsigned long size,
                                      unsigned long *s);
int __must_check rangeset_find_aligned_range(struct rangeset *r,
                                             unsigned long size,
                                             unsigned long min,
                                             unsigned long *s);
int __must_check rangeset_remove_range(
    struct rangeset *r, unsigned long s, unsigned long e);
bool __must_check rangeset_contains_range(
    struct rangeset *r, unsigned long s, unsigned long e);
bool __must_check rangeset_overlaps_range(
    struct rangeset *r, unsigned long s, unsigned long e);
int rangeset_report_ranges(
    struct rangeset *r, unsigned long s, unsigned long e,
    int (*cb)(unsigned long s, unsigned long e, void *data), void *ctxt);
void rangeset_purge(struct rangeset *r);

/*
 * Note that the consume function can return an error value apart from
 * -ERESTART, and that no cleanup is performed (ie: the user should call
 * rangeset_destroy if needed).
 */
int rangeset_consume_ranges(struct rangeset *r,
                            int (*cb)(unsigned long s, unsigned long e,
                                      void *ctxt, unsigned long *c),
                            void *ctxt);

/* Merge rangeset r2 into rangeset r1. */
int __must_check rangeset_merge(struct rangeset *r1, struct rangeset *r2);

/* Subtract rangeset r2 from rangeset r1. */
int __must_check rangeset_subtract(struct rangeset *r1, struct rangeset *r2);

/* Add/remove/query a single number. */
int __must_check rangeset_add_singleton(
    struct rangeset *r, unsigned long s);
int __must_check rangeset_remove_singleton(
    struct rangeset *r, unsigned long s);
bool __must_check rangeset_contains_singleton(
    struct rangeset *r, unsigned long s);

/* swap contents */
void rangeset_swap(struct rangeset *a, struct rangeset *b);

/* Rangeset pretty printing. */
void rangeset_domain_printk(
    struct domain *d);

#endif /* __XEN_RANGESET_H__ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
