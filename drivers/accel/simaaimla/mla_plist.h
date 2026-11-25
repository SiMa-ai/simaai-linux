/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Vendored plist_add / plist_del — see mla_plist.c for rationale.
 *
 * Everything else (plist_node_init, plist_head_init, plist_first_entry,
 * plist_head_empty, plist_node_empty) is a static inline in the
 * upstream <linux/plist.h> and works from modules without any patch —
 * use those directly.
 */
#ifndef _MLA_PLIST_H_
#define _MLA_PLIST_H_

#include <linux/plist.h>

void mla_plist_add(struct plist_node *node, struct plist_head *head);
void mla_plist_del(struct plist_node *node, struct plist_head *head);

#endif /* _MLA_PLIST_H_ */
