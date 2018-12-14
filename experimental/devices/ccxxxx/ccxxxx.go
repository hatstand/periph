// Copyright 2018 The Periph Authors. All rights reserved.
// Use of this source code is governed under the Apache License, Version 2.0
// that can be found in the LICENSE file.

// Package ccxxxx controls a TI CCxxxx RF Transceiver.
// Currently only tested with a CC1101 but should work for similar chips like
// the CC2500 with minor modifications.

package ccxxxx

import (
	"errors"
	"fmt"
	"log"
	"time"

	"periph.io/x/periph/conn/gpio"
	"periph.io/x/periph/conn/physic"
	"periph.io/x/periph/conn/spi"
)

const (
	speed = 5 * physic.MegaHertz
	bits  = 8

	// Max packet length = length of FIFO buffer - packet length byte - 2 status bytes
	maxPacketLen = 64 - 1 - 2

	// Read/write flags.
	writeSingleByte = 0x00
	writeBurst      = 0x40
	readSingleByte  = 0x80
	readBurst       = 0xc0
	bytesInRxFifo   = 0x7f
	overflow        = 0x80

	// FIFO buffers.
	rxFifo = 0x3f
	txFifo = 0x3f

	// Strobes
	sres    = 0x30
	sfstxon = 0x31
	sxoff   = 0x32
	scal    = 0x33
	srx     = 0x34
	stx     = 0x35
	sidle   = 0x36
	swor    = 0x38
	spwd    = 0x39
	sfrx    = 0x3a
	sftx    = 0x3b
	sworrst = 0x3c
	snop    = 0x3d

	// Status registers
	partnum = 0xf0
	version = 0xf1
	rxbytes = 0x3b

	// Config registers
	iocfg2 = 0x00
	iocfg1 = 0x01
	iocfg0 = 0x02

	fifothr = 0x03

	sync1 = 0x04
	sync0 = 0x05

	pktlen   = 0x06
	pktctrl1 = 0x07
	pktctrl0 = 0x08

	addr = 0x09

	channr  = 0x0a
	fsctrl1 = 0x0b
	fsctrl0 = 0x0c

	freq2 = 0x0d
	freq1 = 0x0e
	freq0 = 0x0f

	mdmcfg4 = 0x10
	mdmcfg3 = 0x11
	mdmcfg2 = 0x12
	mdmcfg1 = 0x13
	mdmcfg0 = 0x14

	deviatn = 0x15

	mcsm2 = 0x16
	mcsm1 = 0x17
	mcsm0 = 0x18

	foccfg = 0x19
	bscfg  = 0x1a

	agcctrl2 = 0x1b
	agcctrl1 = 0x1c
	agcctrl0 = 0x1d

	worevt1 = 0x1e
	worevt0 = 0x1f
	worctrl = 0x20

	frend1 = 0x21
	frend0 = 0x22

	fscal3 = 0x23
	fscal2 = 0x24
	fscal1 = 0x25
	fscal0 = 0x26

	rcctrl1 = 0x27
	rcctrl0 = 0x28

	fstest  = 0x29
	ptest   = 0x2a
	agctest = 0x2b
	test2   = 0x2c
	test1   = 0x2d
	test0   = 0x2e
)

// Generated with SmartRF Studio (http://www.ti.com/tool/SMARTRFTM-STUDIO).
// Frequency: 868.3MHz
// Modulation: 2-FSK
// Manchester encoding disabled
var Config_868_3 = map[byte]byte{
	fsctrl1:  0x06,
	fsctrl0:  0x00,
	freq2:    0x21,
	freq1:    0x65,
	freq0:    0x44,
	mdmcfg4:  0xf5,
	mdmcfg3:  0x83,
	mdmcfg2:  0x03,
	mdmcfg1:  0x22,
	mdmcfg0:  0xf8,
	deviatn:  0x34,
	foccfg:   0x16,
	bscfg:    0x6c,
	agcctrl2: 0x03,
	agcctrl1: 0x40,
	agcctrl0: 0x91,
	frend1:   0x56,
	frend0:   0x10,
	fscal3:   0xe9,
	fscal2:   0x2a,
	fscal1:   0x00,
	fscal0:   0x1f,
}

// Datasheet:
// http://www.ti.com/lit/ds/symlink/cc1101.pdf

// New returns a handle to a CCxxxx device.
func New(p spi.Port, gdo0 gpio.PinIn, gdo2 gpio.PinIn) (*Dev, error) {
	gdo0.In(gpio.PullDown, gpio.NoEdge)
	gdo2.In(gpio.PullDown, gpio.NoEdge)
	c, err := p.Connect(speed, spi.Mode0, bits)
	if err != nil {
		return nil, err
	}
	d := &Dev{
		c:    c,
		gdo0: gdo0,
		gdo2: gdo2,
	}

	err = d.strobe(sres)
	if err != nil {
		return nil, err
	}

	ver, err := d.readSingleByte(version)
	if err != nil {
		return nil, err
	}
	part, err := d.readSingleByte(partnum)
	if err != nil {
		return nil, err
	}
	if ver != 0x14 || part != 0x00 { // CC1101
		log.Printf("Found unexpected CCxxx device: version 0x%x partnum: 0x%x", ver, part)
	}

	d.Config(map[byte]byte{
		// Append RSSI and LQI to packets.
		pktctrl1: 0x04,
		// Data whitening off.
		// Normal packet format.
		// CRC enabled.
		// Variable length packets.
		pktctrl0: 0x05,
		// Rising edge on GDO2 when sending/receiving a packet and falling edge at
		// end of packet.
		iocfg2: 0x06,
		// Rising edge on receiving a packet on GDO0.
		iocfg0: 0x07,

		pktlen: maxPacketLen,

		// Enable automatic calibration when going from IDLE -> RX or TX.
		mcsm0: 0x14,
	})

	return d, nil
}

// Dev is a handle to the device.
type Dev struct {
	c spi.Conn
	// Emits rising edges when packets arrive by default.
	gdo0 gpio.PinIn
	// Emits rising edge then faling edge when sending a packet by default.
	gdo2 gpio.PinIn
}

// Packet represents a single received packet with metadata.
type Packet struct {
	Data []byte
	RSSI int
	LQI  int
	CRC  bool
}

// Receive waits for timeout for a single packet to arrive.
func (d *Dev) Receive(timeout time.Duration) (*Packet, error) {
	d.gdo0.In(gpio.PullDown, gpio.RisingEdge)
	defer d.gdo0.In(gpio.PullDown, gpio.NoEdge)

	d.setRX()
	if d.gdo0.WaitForEdge(timeout) {
		rx, err := d.readSingleByte(rxbytes)
		if err != nil {
			return nil, err
		}
		if rx&overflow > 0 {
			return nil, errors.New("RX buffer overflow")
		}
		if rx&bytesInRxFifo > 0 {
			numBytes, err := d.readSingleByte(rxFifo)
			if err != nil {
				return nil, err
			}
			// Read packet data.
			packet, err := d.readBurst(rxFifo, numBytes)

			// Read extra status bytes.
			status, err := d.readBurst(rxFifo, 2)
			rssi := convertRSSI(int(status[0]))
			rawLQI := status[1] & 0x7f
			rawCRC := status[1] >> 7

			// Flush the RX buffer afterwards.
			d.setState(sidle)
			d.strobe(sfrx)
			return &Packet{
				Data: packet,
				RSSI: rssi,
				LQI:  int(rawLQI),
				CRC:  rawCRC == 1,
			}, nil
		}
	}
	return nil, nil
}

func (d *Dev) Send(data []byte) error {
	if len(data) > maxPacketLen {
		return fmt.Errorf(
				"Packet too long: %d (maximum size: %d)", len(data), maxPacketLen)
	}
	// Write the length of the packet to the FIFO buffer.
	err := d.writeSingleByte(txFifo, byte(len(data)))
	if err != nil {
		return err
	}
	// Write the contents to the FIFO buffer.
	err = d.writeBurst(txFifo, data)
	if err != nil {
		return err
	}

	d.gdo2.In(gpio.PullDown, gpio.FallingEdge)
	defer d.gdo2.In(gpio.PullDown, gpio.NoEdge)
	// Start transmitting the packet.
	d.strobe(stx)
	// Falling edge for end of packet.
	d.gdo2.WaitForEdge(-1)
	// Flush the FIFO buffer.
	d.strobe(sftx)
	d.strobe(sidle)
	return nil
}

// Config sets device registers from a map of address -> value.
func (d *Dev) Config(config map[byte]byte) error {
	for k, v := range config {
		err := d.writeSingleByte(k, v)
		if err != nil {
			return err
		}
	}
	return nil
}

func (d *Dev) readSingleByte(address byte) (byte, error) {
	w := []byte{address | readSingleByte, 0x00}
	r := make([]byte, len(w))

	err := d.c.Tx(w, r)
	if err != nil {
		return 0x00, err
	}
	return r[1], nil
}

func (d *Dev) readBurst(address byte, num byte) ([]byte, error) {
	var buf []byte
	for i := byte(0); i < num+1; i++ {
		buf = append(buf, (address+i*8)|readBurst)
	}
	r := make([]byte, len(buf))
	err := d.c.Tx(buf, r)
	if err != nil {
		return nil, err
	}
	return r[1:], nil
}

func (d *Dev) writeSingleByte(address byte, data byte) error {
	r := []byte{address | writeSingleByte, data}
	return d.c.Tx(r, nil)
}

func (d *Dev) writeBurst(address byte, data []byte) error {
	buf := []byte{address|writeBurst}
	buf = append(buf, data...)
	return d.c.Tx(buf, nil)
}

func (d *Dev) strobe(address byte) error {
	r := []byte{address, 0x00}
	err := d.c.Tx(r, nil)
	if err != nil {
		return err
	}
	return nil
}

func (d *Dev) setState(state byte) error {
	err := d.strobe(state)
	if err != nil {
		return err
	}
	// Worst case state change is ~1ms for IDLE -> RX with calibration.
	time.Sleep(1 * time.Millisecond)
	return nil
}

func (d *Dev) setRX() error {
	err := d.setState(sidle)
	if err != nil {
		return err
	}
	err = d.setState(srx)
	if err != nil {
		return err
	}
	return nil
}

func convertRSSI(raw int) int {
	if raw >= 128 {
		return (raw - 256/2) - 74
	}
	return raw/2 - 74
}
