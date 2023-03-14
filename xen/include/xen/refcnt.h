#ifndef __XEN_REFCNT_H__
#define __XEN_REFCNT_H__

#include <asm/atomic.h>
#include <asm/bug.h>

#define REFCNT_SATURATED (INT_MIN / 2)

typedef struct {
    atomic_t refcnt;
} refcnt_t;

static inline void refcnt_init(refcnt_t *refcnt)
{
    atomic_set(&refcnt->refcnt, 1);
}

static inline int refcnt_read(refcnt_t *refcnt)
{
    return atomic_read(&refcnt->refcnt);
}

static inline void refcnt_get(refcnt_t *refcnt)
{
    int old = atomic_add_unless(&refcnt->refcnt, 1, 0);

    if ( unlikely(old < 0) || unlikely (old + 1 < 0) )
    {
        atomic_set(&refcnt->refcnt, REFCNT_SATURATED);
        printk(XENLOG_ERR"refcnt saturation: old = %d\n", old);
        WARN();
    }
}

static inline void refcnt_put(refcnt_t *refcnt, void (*destructor)(refcnt_t *refcnt))
{
    int ret = atomic_dec_return(&refcnt->refcnt);

    if ( ret == 0 )
        destructor(refcnt);

    if ( unlikely(ret < 0))
    {
        atomic_set(&refcnt->refcnt, REFCNT_SATURATED);
        printk(XENLOG_ERR"refcnt already hit 0: val = %d\n", ret);
        WARN();
    }
}

#endif

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
