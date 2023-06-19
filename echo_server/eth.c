/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "eth.h"
#include "shared_ringbuffer.h"
#include "util.h"

#define IRQ_CH 0
#define TX_CH  1
#define RX_CH  2
#define INIT   3
#define ETH_CLI 3

#define MDC_FREQ    20000000UL

/* Memory regions. These all have to be here to keep compiler happy */
uintptr_t hw_ring_buffer_vaddr;
uintptr_t hw_ring_buffer_paddr;
uintptr_t rx_cookies;
uintptr_t tx_cookies;
uintptr_t rx_free;
uintptr_t rx_used;
uintptr_t tx_free;
uintptr_t tx_used;
uintptr_t uart_base;

/* Make the minimum frame buffer 2k. This is a bit of a waste of memory, but ensures alignment */
#define PACKET_BUFFER_SIZE  2048
#define MAX_PACKET_SIZE     1536

#define RX_COUNT 256
#define TX_COUNT 256

_Static_assert((RX_COUNT + TX_COUNT) * 2 * PACKET_BUFFER_SIZE <= 0x200000, "Expect rx+tx buffers to fit in single 2MB page");
_Static_assert(sizeof(ring_buffer_t) <= 0x200000, "Expect ring buffer ring to fit in single 2MB page");


ring_ctx_t *rx = (void *)(uintptr_t)0x5400000; /* RX NIC Ring*/
ring_ctx_t *tx = (void *)(uintptr_t)0x5200000; /* TX NIC Ring */

/* Pointers to shared_ringbuffers */
ring_handle_t rx_ring;
ring_handle_t tx_ring;

static uint8_t mac[6];
volatile struct enet_regs *eth = (void *)(uintptr_t)0x2000000;
uint32_t irq_mask = IRQ_MASK;

static void complete_tx(volatile struct enet_regs *eth);

static void get_mac_addr(volatile struct enet_regs *reg, uint8_t *mac)
{
    uint32_t l, h;
    l = reg->palr;
    h = reg->paur;

    mac[0] = l >> 24;
    mac[1] = l >> 16 & 0xff;
    mac[2] = l >> 8 & 0xff;
    mac[3] = l & 0xff;
    mac[4] = h >> 24;
    mac[5] = h >> 16 & 0xff;
}

static void set_mac(volatile struct enet_regs *reg, uint8_t *mac)
{
    reg->palr = (mac[0] << 24) | (mac[1] << 16) | (mac[2] << 8) | (mac[3]);
    reg->paur = (mac[4] << 24) | (mac[5] << 16);
}

static void
dump_mac(uint8_t *mac)
{
    for (unsigned i = 0; i < 6; i++) {
        putC(hexchar((mac[i] >> 4) & 0xf));
        putC(hexchar(mac[i] & 0xf));
        if (i < 5) {
            putC(':');
        }
    }
    putC('\n');
}

static void update_ring_slot(
    ring_ctx_t *ring,
    unsigned int idx,
    uintptr_t phys,
    uint16_t len,
    uint16_t stat)
{
    volatile struct descriptor *d = &(ring->descr[idx]);
    d->addr = phys;
    d->len = len;

    /* Ensure all writes to the descriptor complete, before we set the flags
     * that makes hardware aware of this slot.
     */
    __sync_synchronize();

    d->stat = stat;
}

static inline void
enable_irqs(volatile struct enet_regs *eth, uint32_t mask)
{
    eth->eimr = mask;
    irq_mask = mask;
}

static uintptr_t
alloc_rx_buf(size_t buf_size, void **cookie)
{
    uintptr_t addr;
    unsigned int len;

    /* Try to grab a buffer from the free ring */
    if (driver_dequeue(rx_ring.free_ring, &addr, &len, cookie)) {
        return 0;
    }

    return addr;
}

static int
ring_remaining(ring_ctx_t *ring)
{
    return (ring->tail - ring->head - 2) % ring->cnt;
}

static void
fill_rx_bufs(void)
{
    ring_ctx_t *ring = rx;
    __sync_synchronize();
    while (ring_remaining(ring) > 0) {
        /* request a buffer */
        void *cookie = NULL;
        uintptr_t phys = alloc_rx_buf(MAX_PACKET_SIZE, &cookie);
        if (!phys) {
            break;
        }
        uint16_t stat = RXD_EMPTY;
        int idx = ring->tail;
        int new_tail = idx + 1;
        if (new_tail == ring->cnt) {
            new_tail = 0;
            stat |= WRAP;
        }
        ring->cookies[idx] = cookie;
        update_ring_slot(ring, idx, phys, 0, stat);
        ring->tail = new_tail;
    }
    __sync_synchronize();

    if (ring->tail != ring->head) {
        /* Make sure rx is enabled */
        eth->rdar = RDAR_RDAR;
        if (!(irq_mask & NETIRQ_RXF))
            enable_irqs(eth, IRQ_MASK);
    } else {
        enable_irqs(eth, NETIRQ_TXF | NETIRQ_EBERR);
    }
}

static void
handle_rx(volatile struct enet_regs *eth)
{
    ring_ctx_t *ring = rx;
    unsigned int head = ring->head;
    int num = 0;
    int og_size = ring_size(rx_ring.used_ring);

    if (ring_full(rx_ring.used_ring))
    {
        /*
         * we can't process any packets because the queues are full
         * so disable Rx irqs.
         */
        enable_irqs(eth, NETIRQ_TXF | NETIRQ_EBERR);
        return;
    }
    /*
     * Dequeue until either:
     *   we run out of filled buffers in the NIC ring
     *   we run out of slots in the upstream Rx ring.
     */
    while (head != ring->tail && !ring_full(rx_ring.used_ring)) {
        volatile struct descriptor *d = &(ring->descr[head]);

        /*
         * If a slot is still marked as empty we are done.
         * Still need to signal upstream if
         * we've dequeued anything.
         */
        if (d->stat & RXD_EMPTY) {
            break;
        }

        void *cookie = ring->cookies[head];
        /* Go to next buffer, handle roll-over. */
        if (++head == ring->cnt) {
            head = 0;
        }
        ring->head = head;

        buff_desc_t *desc = (buff_desc_t *)cookie;
        int err = enqueue_used(&rx_ring, desc->encoded_addr, d->len, desc->cookie);
        if (err) {
            print("ETH|ERROR: Failed to enqueue to RX used ring\n");
        }
        assert(!err);
        num++;
    }

    /* Notify client (only if we have actually processed a packet
     * and the client hasn't already been notified!)
     * Driver runs at highest priority, so buffers will be refilled
     * by caller before the notify causes a context switch.
     */
    if ((og_size == 0 || og_size + num != ring_size(rx_ring.used_ring)) && num != 0) {
        sel4cp_notify(RX_CH);
    }
}

static void
raw_tx(volatile struct enet_regs *eth, uintptr_t phys,
                  unsigned int len, void *cookie)
{
    ring_ctx_t *ring = tx;

    __sync_synchronize();

    unsigned int tail = ring->tail;
    unsigned int tail_new = tail;

    uint16_t stat = TXD_READY | TXD_ADDCRC | TXD_LAST;

    unsigned int idx = tail_new;
    if (++tail_new == TX_COUNT) {
        tail_new = 0;
        stat |= WRAP;
    }
    update_ring_slot(ring, idx, phys, len, stat);

    ring->cookies[tail] = cookie;
    __sync_synchronize();
    ring->tail = tail_new;

    if (!(eth->tdar & TDAR_TDAR)) {
        eth->tdar = TDAR_TDAR;
    }
}

static void
handle_tx(volatile struct enet_regs *eth)
{
    uintptr_t buffer = 0;
    unsigned int len = 0;
    void *cookie = NULL;

    while ((ring_remaining(tx) > 1) && !driver_dequeue(tx_ring.used_ring, &buffer, &len, &cookie)) {
        raw_tx(eth, buffer, len, cookie);
    }
}

static void
complete_tx(volatile struct enet_regs *eth)
{
    void *cookie;
    ring_ctx_t *ring = tx;
    unsigned int head = ring->head;
    int enqueued = 0;

    int was_empty = ring_empty(tx_ring.free_ring);
    int num_remaning = ring_remaining(ring);

    while (head != ring->tail && !ring_full(tx_ring.free_ring)) {
        cookie = ring->cookies[head];

        volatile struct descriptor *d = &(ring->descr[head]);

        /* If this buffer was not sent, we can't release any more buffers */
        if (d->stat & TXD_READY) {
            break;
        }

        /* Go to next buffer, handle roll-over. */
        if (++head == ring->cnt) {
            head = 0;
        }

        __sync_synchronize();
        ring->head = head;
        /* give the buffer back */
        buff_desc_t *desc = (buff_desc_t *)cookie;
        int err = enqueue_free(&tx_ring, desc->encoded_addr, desc->len, desc->cookie);
        assert(!err);
        enqueued++;
    }

    if (was_empty && enqueued) {
        sel4cp_notify(TX_CH);
    }

    if (num_remaning == 0 && enqueued) {
        sel4cp_notify(ETH_CLI);
    }
}

static void
handle_eth(volatile struct enet_regs *eth)
{
    uint32_t e = eth->eir & irq_mask;
    /* write to clear events */
    eth->eir = e;

    while (e & irq_mask) {
        if (e & NETIRQ_TXF) {
            complete_tx(eth);
        }
        if (e & NETIRQ_RXF) {
            handle_rx(eth);
            fill_rx_bufs();
        }
        if (e & NETIRQ_EBERR) {
            print("Error: System bus/uDMA");
            while (1);
        }
        e = eth->eir & irq_mask;
        eth->eir = e;
    }
}

static void
eth_setup(void)
{
    get_mac_addr(eth, mac);
    sel4cp_dbg_puts("MAC: ");
    //dump_mac(mac);
    sel4cp_dbg_puts("\n");

    /* set up descriptor rings */
    rx->cnt = RX_COUNT;
    rx->tail = 0;
    rx->head = 0;
    rx->cookies = (void **)rx_cookies;
    rx->descr = (volatile struct descriptor *)hw_ring_buffer_vaddr;

    tx->cnt = TX_COUNT;
    tx->tail = 0;
    tx->head = 0;
    tx->cookies = (void **)tx_cookies;
    tx->descr = (volatile struct descriptor *)(hw_ring_buffer_vaddr + (sizeof(struct descriptor) * RX_COUNT));

    /* Perform reset */
    eth->ecr = ECR_RESET;
    while (eth->ecr & ECR_RESET);
    eth->ecr |= ECR_DBSWP;

    /* Clear and mask interrupts */
    eth->eimr = 0x00000000;
    eth->eir  = 0xffffffff;

    /* set MDIO freq */
    eth->mscr = 24 << 1;

    /* Disable */
    eth->mibc |= MIBC_DIS;
    while (!(eth->mibc & MIBC_IDLE));
    /* Clear */
    eth->mibc |= MIBC_CLEAR;
    while (!(eth->mibc & MIBC_IDLE));
    /* Restart */
    eth->mibc &= ~MIBC_CLEAR;
    eth->mibc &= ~MIBC_DIS;

    /* Descriptor group and individual hash tables - Not changed on reset */
    eth->iaur = 0;
    eth->ialr = 0;
    eth->gaur = 0;
    eth->galr = 0;

    if (eth->palr == 0) {
        // the mac address needs setting again.
        set_mac(eth, mac);
    }

    eth->opd = PAUSE_OPCODE_FIELD;

    /* coalesce transmit IRQs to batches of 128 *///0x200
    eth->txic0 = ICEN | ICFT(128) | 0xFF;
    eth->tipg = TIPG;
    /* Transmit FIFO Watermark register - store and forward */
    eth->tfwr = STRFWD;
    /* Rx clear store and forward. This must be done for hardware csums*/
    eth->rsfl = 0;
    /* Do not forward frames with errors + check the csum */
    eth->racc = RACC_LINEDIS | RACC_IPDIS | RACC_PRODIS;
    eth->tacc = TACC_IPCHK | TACC_PROCHK;
    /* Set RDSR */
    eth->rdsr = hw_ring_buffer_paddr;
    eth->tdsr = hw_ring_buffer_paddr + (sizeof(struct descriptor) * RX_COUNT);

    /* Size of max eth packet size */
    eth->mrbr = MAX_PACKET_SIZE;

    eth->rcr = RCR_MAX_FL(1518) | RCR_RGMII_EN | RCR_MII_MODE | RCR_PROMISCUOUS;
    eth->tcr = TCR_FDEN;

    /* set speed */
    eth->ecr |= ECR_SPEED;

    /* Set Enable  in ECR */
    eth->ecr |= ECR_ETHEREN;

    eth->rdar = RDAR_RDAR;

    /* enable events */
    eth->eir = eth->eir;
    eth->eimr = IRQ_MASK;
}

void init(void)
{
    //print(sel4cp_name);
    //print(": elf PD init function running\n");

    eth_setup();

    /* Set up shared memory regions */
    ring_init(&rx_ring, (ring_buffer_t *)rx_free, (ring_buffer_t *)rx_used, NULL, 0);
    ring_init(&tx_ring, (ring_buffer_t *)tx_free, (ring_buffer_t *)tx_used, NULL, 0);

    sel4cp_notify(INIT);
    /* Now wait for notification that buffers are initialised */
}

seL4_MessageInfo_t
protected(sel4cp_channel ch, sel4cp_msginfo msginfo)
{
    switch (ch) {
        /*case TX_CH:
            handle_tx(eth);
            break;*/
        default:
            sel4cp_dbg_puts("Received ppc on unexpected channel ");
            puthex64(ch);
            break;
    }
    return sel4cp_msginfo_new(0, 0);
}

void notified(sel4cp_channel ch)
{
    switch(ch) {
        case IRQ_CH:
            handle_eth(eth);
            /*
             * Delay calling into the kernel to ack the IRQ until the next loop
             * in the seL4CP event handler loop.
             */
            sel4cp_irq_ack_delayed(ch);
            break;
        case RX_CH:
            fill_rx_bufs();
            break;
        /*case TX_CH:
            handle_tx(eth);
            break;*/
        default:
            print("eth driver: received notification on unexpected channel: ");
            puthex64(ch);
            print("\n");
            assert(0);
            break;
    }
}
