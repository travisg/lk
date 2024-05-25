/*
 * Copyright (c) 2015 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#include <dev/virtio/net.h>

#include <stdlib.h>
#include <inttypes.h>
#include <lk/debug.h>
#include <assert.h>
#include <lk/trace.h>
#include <lk/compiler.h>
#include <lk/list.h>
#include <string.h>
#include <lk/err.h>
#include <kernel/thread.h>
#include <kernel/event.h>
#include <kernel/spinlock.h>
#include <lib/pktbuf.h>
#include <lib/minip.h>
#include <dev/virtio/virtio-device.h>

#define LOCAL_TRACE 0

struct virtio_net_config {
    uint8_t mac[6];
    uint16_t status;
    uint16_t max_virtqueue_pairs;
    uint16_t mtu;
    uint32_t speed;
    uint8_t  duplex;
    uint8_t  rss_max_key_size;
    uint16_t rss_max_indirection_table_length;
    uint32_t supported_hash_types;
    uint32_t supported_tunnel_types;
};
STATIC_ASSERT(sizeof(struct virtio_net_config) == 28);

struct virtio_net_hdr {
#define VIRTIO_NET_HDR_F_NEEDS_CSUM 1
#define VIRTIO_NET_HDR_F_DATA_VALID 2
#define VIRTIO_NET_HDR_F_RSC_INFO   4
    uint8_t  flags;
#define VIRTIO_NET_HDR_GSO_NONE     0
#define VIRTIO_NET_HDR_GSO_TCPV4    1
#define VIRTIO_NET_HDR_GSO_UDP      3
#define VIRTIO_NET_HDR_GSO_TCPV6    4
#define VIRTIO_NET_HDR_GSO_UDP_L4   5
#define VIRTIO_NET_HDR_GSO_ECN      0x80
    uint8_t  gso_type;
    uint16_t hdr_len;
    uint16_t gso_size;
    uint16_t csum_start;
    uint16_t csum_offset;
    uint16_t num_buffers; // unused in tx

    // Only if VIRTIO_NET_HASH_REPORT negotiated
    //uint32_t hash_value;
    //uint16_t hash_report;
    //uint16_t padding_reserved;
};
STATIC_ASSERT(sizeof(struct virtio_net_hdr) == 12);

#define VIRTIO_NET_F_CSUM                   (1<<0)
#define VIRTIO_NET_F_GUEST_CSUM             (1<<1)
#define VIRTIO_NET_F_CTRL_GUEST_OFFLOADS    (1<<2)
#define VIRTIO_NET_F_MTU                    (1<<3)
#define VIRTIO_NET_F_MAC                    (1<<5)
#define VIRTIO_NET_F_GSO                    (1<<6) // removed in v1.3
#define VIRTIO_NET_F_GUEST_TSO4             (1<<7)
#define VIRTIO_NET_F_GUEST_TSO6             (1<<8)
#define VIRTIO_NET_F_GUEST_ECN              (1<<9)
#define VIRTIO_NET_F_GUEST_UFO              (1<<10)
#define VIRTIO_NET_F_HOST_TSO4              (1<<11)
#define VIRTIO_NET_F_HOST_TSO6              (1<<12)
#define VIRTIO_NET_F_HOST_ECN               (1<<13)
#define VIRTIO_NET_F_HOST_UFO               (1<<14)
#define VIRTIO_NET_F_MRG_RXBUF              (1<<15)
#define VIRTIO_NET_F_STATUS                 (1<<16)
#define VIRTIO_NET_F_CTRL_VQ                (1<<17)
#define VIRTIO_NET_F_CTRL_RX                (1<<18)
#define VIRTIO_NET_F_CTRL_VLAN              (1<<19)
#define VIRTIO_NET_F_GUEST_ANNOUNCE         (1<<21)
#define VIRTIO_NET_F_MQ                     (1<<22)
#define VIRTIO_NET_F_CTRL_MAC_ADDR          (1<<23)
#define VIRTIO_NET_F_HASH_TUNNEL            (1ULL<<51)
#define VIRTIO_NET_F_VQ_NOTF_COAL           (1ULL<<52)
#define VIRTIO_NET_F_NOTF_COAL              (1ULL<<53)
#define VIRTIO_NET_F_GUEST_USO4             (1ULL<<54)
#define VIRTIO_NET_F_GUEST_USO6             (1ULL<<55)
#define VIRTIO_NET_F_HOST_USO               (1ULL<<56)
#define VIRTIO_NET_F_HASH_REPORT            (1ULL<<57)
#define VIRTIO_NET_F_GUEST_HDRLEN           (1ULL<<59)
#define VIRTIO_NET_F_RSS                    (1ULL<<60)
#define VIRTIO_NET_F_RSC_EXT                (1ULL<<61)
#define VIRTIO_NET_F_STANDBY                (1ULL<<62)
#define VIRTIO_NET_F_SPEED_DUPLEX           (1ULL<<63)

#define VIRTIO_NET_S_LINK_UP                (1<<0)
#define VIRTIO_NET_S_ANNOUNCE               (1<<1)

#define TX_RING_SIZE 16
#define RX_RING_SIZE 16

#define RING_RX 0
#define RING_TX 1

#define VIRTIO_NET_MSS 1514

struct virtio_net_dev {
    struct virtio_device *dev;
    bool started;

    const virtio_net_config *config;

    spin_lock_t lock;
    event_t rx_event;

    /* list of active tx/rx packets to be freed at irq time */
    pktbuf_t *pending_tx_packet[TX_RING_SIZE];
    pktbuf_t *pending_rx_packet[RX_RING_SIZE];

    uint tx_pending_count;
    struct list_node completed_rx_queue;
};

static enum handler_return virtio_net_irq_driver_callback(struct virtio_device *dev, uint ring, const struct vring_used_elem *e);
static int virtio_net_rx_worker(void *arg);
static status_t virtio_net_queue_rx(struct virtio_net_dev *ndev, pktbuf_t *p);

// XXX remove need for this
static struct virtio_net_dev *the_ndev;

static void dump_feature_bits(uint64_t feature) {
    printf("virtio-net host features (%#" PRIx64 "):", feature);
    if (feature & VIRTIO_NET_F_CSUM) printf(" CSUM");
    if (feature & VIRTIO_NET_F_GUEST_CSUM) printf(" GUEST_CSUM");
    if (feature & VIRTIO_NET_F_CTRL_GUEST_OFFLOADS) printf(" CTRL_GUEST_OFFLOADS");
    if (feature & VIRTIO_NET_F_MTU) printf(" MTU");
    if (feature & VIRTIO_NET_F_MAC) printf(" MAC");
    if (feature & VIRTIO_NET_F_GSO) printf(" GSO");
    if (feature & VIRTIO_NET_F_GUEST_TSO4) printf(" GUEST_TSO4");
    if (feature & VIRTIO_NET_F_GUEST_TSO6) printf(" GUEST_TSO6");
    if (feature & VIRTIO_NET_F_GUEST_ECN) printf(" GUEST_ECN");
    if (feature & VIRTIO_NET_F_GUEST_UFO) printf(" GUEST_UFO");
    if (feature & VIRTIO_NET_F_HOST_TSO4) printf(" HOST_TSO4");
    if (feature & VIRTIO_NET_F_HOST_TSO6) printf(" HOST_TSO6");
    if (feature & VIRTIO_NET_F_HOST_ECN) printf(" HOST_ECN");
    if (feature & VIRTIO_NET_F_HOST_UFO) printf(" HOST_UFO");
    if (feature & VIRTIO_NET_F_MRG_RXBUF) printf(" MRG_RXBUF");
    if (feature & VIRTIO_NET_F_STATUS) printf(" STATUS");
    if (feature & VIRTIO_NET_F_CTRL_VQ) printf(" CTRL_VQ");
    if (feature & VIRTIO_NET_F_CTRL_RX) printf(" CTRL_RX");
    if (feature & VIRTIO_NET_F_CTRL_VLAN) printf(" CTRL_VLAN");
    if (feature & VIRTIO_NET_F_GUEST_ANNOUNCE) printf(" GUEST_ANNOUNCE");
    if (feature & VIRTIO_NET_F_MQ) printf(" MQ");
    if (feature & VIRTIO_NET_F_CTRL_MAC_ADDR) printf(" CTRL_MAC_ADDR");
    if (feature & VIRTIO_NET_F_HASH_TUNNEL) printf(" HASH_TUNNEL");
    if (feature & VIRTIO_NET_F_VQ_NOTF_COAL) printf(" VQ_NOTF_COAL");
    if (feature & VIRTIO_NET_F_NOTF_COAL) printf(" NOTF_COAL");
    if (feature & VIRTIO_NET_F_GUEST_USO4) printf(" GUEST_USO4");
    if (feature & VIRTIO_NET_F_GUEST_USO6) printf(" GUEST_USO6");
    if (feature & VIRTIO_NET_F_HOST_USO) printf(" HOST_USO");
    if (feature & VIRTIO_NET_F_HASH_REPORT) printf(" HASH_REPORT");
    if (feature & VIRTIO_NET_F_GUEST_HDRLEN) printf(" GUEST_HDRLEN");
    if (feature & VIRTIO_NET_F_RSS) printf(" RSS");
    if (feature & VIRTIO_NET_F_RSC_EXT) printf(" RSC_EXT");
    if (feature & VIRTIO_NET_F_STANDBY) printf(" STANDBY");
    if (feature & VIRTIO_NET_F_SPEED_DUPLEX) printf(" SPEED_DUPLEX");
    printf("\n");
}

status_t virtio_net_init(virtio_device *dev) {
    LTRACEF("dev %p\n", dev);

    /* allocate a new net device */
    auto *ndev = (virtio_net_dev *)calloc(1, sizeof(virtio_net_dev));
    if (!ndev)
        return ERR_NO_MEMORY;

    ndev->dev = dev;
    dev->set_priv(ndev);
    ndev->started = false;

    ndev->lock = SPIN_LOCK_INITIAL_VALUE;
    event_init(&ndev->rx_event, false, EVENT_FLAG_AUTOUNSIGNAL);
    list_initialize(&ndev->completed_rx_queue);

    ndev->config = (const virtio_net_config *)dev->get_config_ptr();

    /* ack and set the driver status bit */
    dev->bus()->virtio_status_acknowledge_driver();

    // XXX check features bits and ack/nak them
    uint64_t host_features = dev->bus()->virtio_read_host_feature_word(0) | (uint64_t)dev->bus()->virtio_read_host_feature_word(1) << 32;
    dump_feature_bits(host_features);

    /* set our irq handler */
    dev->set_irq_callbacks(&virtio_net_irq_driver_callback, nullptr);

    /* set DRIVER_OK */
    dev->bus()->virtio_status_driver_ok();

    /* allocate a pair of virtio rings */
    dev->virtio_alloc_ring(RING_RX, RX_RING_SIZE); // rx
    dev->virtio_alloc_ring(RING_TX, TX_RING_SIZE); // tx

    the_ndev = ndev;

    return NO_ERROR;
}

status_t virtio_net_start(void) {
    if (the_ndev->started)
        return ERR_ALREADY_STARTED;

    the_ndev->started = true;

    /* start the rx worker thread */
    thread_resume(thread_create("virtio_net_rx", &virtio_net_rx_worker, (void *)the_ndev, HIGH_PRIORITY, DEFAULT_STACK_SIZE));

    /* queue up a bunch of rxes */
    for (uint i = 0; i < RX_RING_SIZE - 1; i++) {
        pktbuf_t *p = pktbuf_alloc();
        if (p) {
            virtio_net_queue_rx(the_ndev, p);
        }
    }

    return NO_ERROR;
}

static status_t virtio_net_queue_tx_pktbuf(struct virtio_net_dev *ndev, pktbuf_t *p2) {
    struct virtio_device *vdev = ndev->dev;

    uint16_t i;
    pktbuf_t *p;

    DEBUG_ASSERT(ndev);

    p = pktbuf_alloc();
    if (!p)
        return ERR_NO_MEMORY;

    /* point our header to the base of the first pktbuf */
    struct virtio_net_hdr *hdr = (virtio_net_hdr *)pktbuf_append(p, sizeof(struct virtio_net_hdr) - 2);
    memset(hdr, 0, p->dlen);

    spin_lock_saved_state_t state;
    spin_lock_irqsave(&ndev->lock, state);

    struct vring_desc *desc = {};

    /* only queue if we have enough tx descriptors */
    if (ndev->tx_pending_count + 2 <= TX_RING_SIZE) {
        /* allocate a chain of descriptors for our transfer */
        desc = vdev->virtio_alloc_desc_chain(RING_TX, 2, &i);
    }
    if (!desc) {
        spin_unlock_irqrestore(&ndev->lock, state);

        TRACEF("out of virtio tx descriptors, tx_pending_count %u\n", ndev->tx_pending_count);
        pktbuf_free(p, true);

        return ERR_NO_MEMORY;
    }

    ndev->tx_pending_count += 2;

    /* save a pointer to our pktbufs for the irq handler to free */
    LTRACEF("saving pointer to pkt in index %u and %u\n", i, desc->next);
    DEBUG_ASSERT(ndev->pending_tx_packet[i] == NULL);
    DEBUG_ASSERT(ndev->pending_tx_packet[desc->next] == NULL);
    ndev->pending_tx_packet[i] = p;
    ndev->pending_tx_packet[desc->next] = p2;

    /* set up the descriptor pointing to the header */
    desc->addr = pktbuf_data_phys(p);
    desc->len = p->dlen;
    desc->flags |= VRING_DESC_F_NEXT;

    /* set up the descriptor pointing to the buffer */
    desc = vdev->virtio_desc_index_to_desc(RING_TX, desc->next);
    desc->addr = pktbuf_data_phys(p2);
    desc->len = p2->dlen;
    desc->flags = 0;

    /* submit the transfer */
    vdev->virtio_submit_chain(RING_TX, i);

    /* kick it off */
    vdev->bus()->virtio_kick(RING_TX);

    spin_unlock_irqrestore(&ndev->lock, state);

    return NO_ERROR;
}

/* variant of the above function that copies the buffer into a pktbuf before sending */
static status_t virtio_net_queue_tx(struct virtio_net_dev *ndev, const void *buf, size_t len) {
    DEBUG_ASSERT(ndev);
    DEBUG_ASSERT(buf);

    pktbuf_t *p = pktbuf_alloc();
    if (!p)
        return ERR_NO_MEMORY;

    /* copy the outgoing packet into the pktbuf */
    p->data = p->buffer;
    p->dlen = len;
    memcpy(p->data, buf, len);

    /* call through to the variant of the function that takes a pre-populated pktbuf */
    status_t err = virtio_net_queue_tx_pktbuf(ndev, p);
    if (err < 0) {
        pktbuf_free(p, true);
    }

    return err;
}

static status_t virtio_net_queue_rx(struct virtio_net_dev *ndev, pktbuf_t *p) {
    struct virtio_device *vdev = ndev->dev;

    DEBUG_ASSERT(ndev);
    DEBUG_ASSERT(p);

    /* point our header to the base of the pktbuf */
    p->data = p->buffer;
    struct virtio_net_hdr *hdr = (struct virtio_net_hdr *)p->data;
    memset(hdr, 0, sizeof(struct virtio_net_hdr) - 2);

    p->dlen = sizeof(struct virtio_net_hdr) - 2 + VIRTIO_NET_MSS;

    spin_lock_saved_state_t state;
    spin_lock_irqsave(&ndev->lock, state);

    /* allocate a chain of descriptors for our transfer */
    uint16_t i;
    struct vring_desc *desc = vdev->virtio_alloc_desc_chain(RING_RX, 1, &i);
    DEBUG_ASSERT(desc); /* shouldn't be possible not to have a descriptor ready */

    /* save a pointer to our pktbufs for the irq handler to use */
    DEBUG_ASSERT(ndev->pending_rx_packet[i] == NULL);
    ndev->pending_rx_packet[i] = p;

    /* set up the descriptor pointing to the header */
    desc->addr = pktbuf_data_phys(p);
    desc->len = p->dlen;
    desc->flags = VRING_DESC_F_WRITE;

    /* submit the transfer */
    vdev->virtio_submit_chain(RING_RX, i);

    /* kick it off */
    vdev->bus()->virtio_kick(RING_RX);

    spin_unlock_irqrestore(&ndev->lock, state);

    return NO_ERROR;
}

static enum handler_return virtio_net_irq_driver_callback(struct virtio_device *dev, uint ring, const struct vring_used_elem *e) {
    struct virtio_net_dev *ndev = (struct virtio_net_dev *)dev->priv();

    LTRACEF("dev %p, ring %u, e %p, id %u, len %u\n", dev, ring, e, e->id, e->len);

    spin_lock(&ndev->lock);

    /* parse our descriptor chain, add back to the free queue */
    uint16_t i = e->id;
    for (;;) {
        int next;
        struct vring_desc *desc = dev->virtio_desc_index_to_desc(ring, i);

        if (desc->flags & VRING_DESC_F_NEXT) {
            next = desc->next;
        } else {
            /* end of chain */
            next = -1;
        }

        dev->virtio_free_desc(ring, i);

        if (ring == RING_RX) {
            /* put the freed rx buffer in a queue */
            pktbuf_t *p = ndev->pending_rx_packet[i];
            ndev->pending_rx_packet[i] = NULL;

            DEBUG_ASSERT(p);
            LTRACEF("rx pktbuf %p filled\n", p);

            /* trim the pktbuf according to the written length in the used element descriptor */
            if (e->len > (sizeof(struct virtio_net_hdr) - 2 + VIRTIO_NET_MSS)) {
                TRACEF("bad used len on RX %u\n", e->len);
                p->dlen = 0;
            } else {
                p->dlen = e->len;
            }

            list_add_tail(&ndev->completed_rx_queue, &p->list);
        } else { // ring == RING_TX
            /* free the pktbuf associated with the tx packet we just consumed */
            pktbuf_t *p = ndev->pending_tx_packet[i];
            ndev->pending_tx_packet[i] = NULL;
            ndev->tx_pending_count--;

            DEBUG_ASSERT(p);
            LTRACEF("freeing pktbuf %p\n", p);

            pktbuf_free(p, false);
        }

        if (next < 0)
            break;
        i = next;
    }

    spin_unlock(&ndev->lock);

    /* if rx ring, signal our event */
    if (ring == 0) {
        event_signal(&ndev->rx_event, false);
    }

    return INT_RESCHEDULE;
}

static int virtio_net_rx_worker(void *arg) {
    struct virtio_net_dev *ndev = (struct virtio_net_dev *)arg;

    for (;;) {
        event_wait(&ndev->rx_event);

        /* pull some packets from the received queue */
        for (;;) {
            spin_lock_saved_state_t state;
            spin_lock_irqsave(&ndev->lock, state);

            pktbuf_t *p = list_remove_head_type(&ndev->completed_rx_queue, pktbuf_t, list);

            spin_unlock_irqrestore(&ndev->lock, state);

            if (!p)
                break; /* nothing left in the queue, go back to waiting */

            LTRACEF("got packet len %u\n", p->dlen);

            /* process our packet */
            struct virtio_net_hdr *hdr = (virtio_net_hdr *)pktbuf_consume(p, sizeof(struct virtio_net_hdr) - 2);
            if (hdr) {
                /* call up into the stack */
                minip_rx_driver_callback(p);
            }

            /* requeue the pktbuf in the rx queue */
            virtio_net_queue_rx(ndev, p);
        }
    }
    return 0;
}

int virtio_net_found(void) {
    return the_ndev ? 1 : 0;
}

status_t virtio_net_get_mac_addr(uint8_t mac_addr[6]) {
    if (!the_ndev)
        return ERR_NOT_FOUND;

    memcpy(mac_addr, the_ndev->config->mac, 6);

    return NO_ERROR;
}

status_t virtio_net_send_minip_pkt(void *arg, pktbuf_t *p) {
    LTRACEF("p %p, dlen %u, flags 0x%x\n", p, p->dlen, p->flags);

    DEBUG_ASSERT(p && p->dlen);

    if ((p->flags & PKTBUF_FLAG_EOF) == 0) {
        /* can't handle multi part packets yet */
        PANIC_UNIMPLEMENTED;

        return ERR_NOT_IMPLEMENTED;
    }

    /* hand the pktbuf off to the nic, it owns the pktbuf from now on out unless it fails */
    status_t err = virtio_net_queue_tx_pktbuf(the_ndev, p);
    if (err < 0) {
        pktbuf_free(p, true);
    }

    return err;
}

