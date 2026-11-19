// SPDX-License-Identifier: GPL-2.0-or-later
//
// Vendored from upstream lib/plist.c — kernel doesn't export plist_add
// and plist_del to modules, so we carry our own copies and call them
// via mla_plist_add() / mla_plist_del(). The struct plist_head /
// struct plist_node layouts come from <linux/plist.h> directly and are
// fully ABI-compatible with the kernel's; only the two big out-of-line
// helpers are vendored. Everything else (plist_head_init,
// plist_node_init, plist_first_entry, plist_head_empty,
// plist_node_empty) is already a static inline in the upstream header
// and works from modules without any patch.
//
// Other SiMa accelerator drivers reusing this pattern: copy this file
// verbatim and the four call-site changes in mla_sched.c; no kernel
// patch required, no symbol export needed.

#include <linux/bug.h>
#include <linux/list.h>
#include <linux/plist.h>

#include "mla_plist.h"

/* CONFIG_DEBUG_PLIST checks are stripped — the production kernels we
 * deploy on don't enable it, and the cross-list debug helpers reference
 * a static test_head we don't want to drag along.
 */

void mla_plist_add(struct plist_node *node, struct plist_head *head)
{
	struct plist_node *first, *iter, *prev = NULL, *last, *reverse_iter;
	struct list_head *node_next = &head->node_list;

	WARN_ON(!plist_node_empty(node));
	WARN_ON(!list_empty(&node->prio_list));

	if (plist_head_empty(head))
		goto ins_node;

	first = iter = plist_first(head);
	last = reverse_iter = list_entry(first->prio_list.prev,
					 struct plist_node, prio_list);

	do {
		if (node->prio < iter->prio) {
			node_next = &iter->node_list;
			break;
		} else if (node->prio >= reverse_iter->prio) {
			prev = reverse_iter;
			iter = list_entry(reverse_iter->prio_list.next,
					  struct plist_node, prio_list);
			if (likely(reverse_iter != last))
				node_next = &iter->node_list;
			break;
		}

		prev = iter;
		iter = list_entry(iter->prio_list.next,
				  struct plist_node, prio_list);
		reverse_iter = list_entry(reverse_iter->prio_list.prev,
					  struct plist_node, prio_list);
	} while (iter != first);

	if (!prev || prev->prio != node->prio)
		list_add_tail(&node->prio_list, &iter->prio_list);
ins_node:
	list_add_tail(&node->node_list, node_next);
}

void mla_plist_del(struct plist_node *node, struct plist_head *head)
{
	if (!list_empty(&node->prio_list)) {
		if (node->node_list.next != &head->node_list) {
			struct plist_node *next;

			next = list_entry(node->node_list.next,
					  struct plist_node, node_list);

			if (list_empty(&next->prio_list))
				list_add(&next->prio_list, &node->prio_list);
		}
		list_del_init(&node->prio_list);
	}

	list_del_init(&node->node_list);
}
