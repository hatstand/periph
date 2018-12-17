// Copyright 2018 The Periph Authors. All rights reserved.
// Use of this source code is governed under the Apache License, Version 2.0
// that can be found in the LICENSE file.

// Package ccxxxx controls a TI CCxxxx RF Transceiver.
// Currently only tested with a CC1101 but should work for similar chips like
// the CC2500 with minor modifications.

package ccxxxx

import (
	"encoding/binary"
	"errors"
	"fmt"
	"log"
	"math"
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

	// Oscillator output configs for GDOx pins.
	clkXosc192 = 0x3f
)

// Generated with SmartRF Studio (http://www.ti.com/tool/SMARTRFTM-STUDIO).
// Frequency: 868.3MHz
// Modulation: 2-FSK
// Manchester encoding disabled
var Config_868_3 = map[byte]byte{
	fsctrl1: 0x06,
	fsctrl0: 0x00,
	mdmcfg4: 0xf5,
	mdmcfg3: 0x83,
	mdmcfg2: 0x03,
	mdmcfg1: 0x22,
	mdmcfg0: 0xf8,
	deviatn: 0x34,
	foccfg:  0x16,
	fscal3:  0xe9,
	fscal2:  0x2a,
	fscal1:  0x00,
	fscal0:  0x1f,
}

type Modulation int

const (
	FSK_2 = 0
	GFSK  = 1
	ASK   = 3
	FSK_4 = 4
	MSK   = 7
)

type Options struct {
	// Oscillator is the frequency of the attached oscillator (usually 26Mhz or 27Mhz).
	Oscillator physic.Frequency
}

func DefaultOptions() Options {
	return Options{
		Oscillator: 26 * physic.MegaHertz,
	}
}

// Datasheet:
// http://www.ti.com/lit/ds/symlink/cc1101.pdf

// NewWithOptions returns a handle to a CCxxxx device.
func NewWithOptions(p spi.Port, gdo0 gpio.PinIn, gdo2 gpio.PinIn, opts Options) (*Dev, error) {
	gdo0.In(gpio.PullDown, gpio.NoEdge)
	gdo2.In(gpio.PullDown, gpio.NoEdge)
	c, err := p.Connect(speed, spi.Mode0, bits)
	if err != nil {
		return nil, fmt.Errorf("failed to connect to CCxxxx over SPI: %v", err)
	}
	d := &Dev{
		c:          c,
		gdo0:       gdo0,
		gdo2:       gdo2,
		oscillator: opts.Oscillator,
	}

	err = d.strobe(sres)
	if err != nil {
		return nil, errors.New("failed to reset CCxxxx")
	}

	ver, err := d.readSingleByte(version)
	if err != nil {
		return nil, errors.New("failed to read version from CCxxxx")
	}
	part, err := d.readSingleByte(partnum)
	if err != nil {
		return nil, errors.New("failed to read partnum from CCxxxx")
	}
	// TODO(hatstand): Explicit support for more variants, e.g. CC2500.
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

// New returns a handle to a CCxxxx device.
func New(p spi.Port, gdo0 gpio.PinIn, gdo2 gpio.PinIn) (*Dev, error) {
	return NewWithOptions(p, gdo0, gdo2, DefaultOptions())
}

// Dev is a handle to the device.
type Dev struct {
	c spi.Conn
	// Emits rising edges when packets arrive by default.
	gdo0 gpio.PinIn
	// Emits rising edge then faling edge when sending a packet by default.
	gdo2 gpio.PinIn

	oscillator physic.Frequency
}

func calculateFreq(xosc physic.Frequency, target physic.Frequency) []byte {
	f := int64(target) / (int64(xosc) >> 16)
	enc := make([]byte, 4)
	binary.BigEndian.PutUint32(enc, uint32(f))
	return enc[1:]
}

func (d *Dev) SetFrequency(freq physic.Frequency) error {
	enc := calculateFreq(d.oscillator, freq)
	return d.writeBurst(freq2, enc[1:])
}

func calculateDeviatn(xosc physic.Frequency, target physic.Frequency) byte {
	c := int64(xosc) >> 17

	// DEVIATN is an unsigned float with a 3 bit mantissa and a 3 bit exponent
	// with a bias of 8.
	// The simplest way to convert an integer frequency into this float format
	// is to just try all of them and use the nearest as there are only 2^6.
	mant := byte(0)
	exp := byte(0)
	candDistance := math.MaxFloat64
	for m := byte(0); m < 8; m++ {
		for e := byte(0); e < 8; e++ {
			val := c * (8 + int64(m)) * (int64(1) << e)
			dist := int64(target) - val
			if math.Abs(float64(dist)) < candDistance {
				mant = m
				exp = e
				candDistance = math.Abs(float64(dist))
			}
		}
	}
	return (exp << 4) | mant
}

// SetDeviation sets the frequency deviation setting (DEVIATN) for FSK schemes.
func (d *Dev) SetDeviation(freq physic.Frequency) error {
	return d.writeSingleByte(deviatn, calculateDeviatn(d.oscillator, freq))
}

// SetModulation sets the modulation format for sending and receiving.
func (d *Dev) SetModulation(mod Modulation) error {
	cfg, err := d.readSingleByte(mdmcfg2)
	if err != nil {
		return fmt.Errorf("failed to read MDMCFG2: %v", err)
	}

	return d.writeSingleByte(mdmcfg2, cfg|(byte(mod)<<4))
}

// Packet represents a single received packet with metadata.
type Packet struct {
	Data []byte
	RSSI int
	LQI  int
	CRC  bool
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

// Read implements io.Reader.Read by receiving a single packet.
func (d *Dev) Read(p []byte) (int, error) {
	packet, err := d.Receive(-1)
	if err != nil {
		return 0, err
	}
	l := min(len(p), len(packet.Data))
	for i := 0; i < l; i++ {
		p[i] = packet.Data[i]
	}
	// TODO(hatstand): Internally buffer when len(p) < len(packet.Data).
	return l, nil
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

			// By default, MCSM1.RXOFF_MODE is set to return to IDLE state automatically.

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

// Write implements io.Writer.Write by sending a single packet.
func (d *Dev) Write(p []byte) (int, error) {
	data := p
	if len(data) > maxPacketLen {
		data = data[:maxPacketLen]
	}
	// Estimate how long it will take to send this packet.
	timeout, err := d.packetTimeout(len(data))
	if err != nil {
		return 0, err
	}
	// Write the length of the packet to the FIFO buffer.
	err = d.writeSingleByte(txFifo, byte(len(data)))
	if err != nil {
		return 0, fmt.Errorf("failed to write packet length: %v", err)
	}
	// Write the contents to the FIFO buffer.
	err = d.writeBurst(txFifo, data)
	if err != nil {
		return 0, fmt.Errorf("failed to write packet: %v", err)
	}
	d.gdo2.In(gpio.PullDown, gpio.FallingEdge)
	defer d.gdo2.In(gpio.PullDown, gpio.NoEdge)
	// Start transmitting the packet.
	if err = d.strobe(stx); err != nil {
		return 0, fmt.Errorf("failed to transmit packet: %v", err)
	}

	// Falling edge for end of packet send.
	log.Printf("Send timeout: %v", timeout)
	if !d.gdo2.WaitForEdge(timeout) {
		return 0, fmt.Errorf("failed to send packet after %v", timeout)
	}
	// By default, MCSM1.TXOFF_MODE is set to return to IDLE state automatically.
	if len(data) == len(p) {
		return len(data), nil
	} else {
		return len(data), fmt.Errorf("buffer size larger than maximum packet length of %d", maxPacketLen)
	}
}

// Config sets device registers from a map of address -> value.
func (d *Dev) Config(config map[byte]byte) error {
	for k, v := range config {
		err := d.writeSingleByte(k, v)
		if err != nil {
			return fmt.Errorf("failed to configure register %d: %v", k, err)
		}
	}
	return nil
}

func (d *Dev) ReadConfig() (map[byte]byte, error) {
	values, err := d.readBurst(iocfg2, test0-iocfg2+1)
	if err != nil {
		return nil, fmt.Errorf("failed to read config: %v", err)
	}
	ret := make(map[byte]byte)
	for i := byte(iocfg2); i < test0+1; i++ {
		ret[i] = values[i]
	}
	return ret, nil
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
	buf := []byte{address | writeBurst}
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
	// See Table 34 in datasheet.
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

// dataRateMantissa gets MDMCFG4.DRATE_M
func (d *Dev) dataRateMantissa() (uint, error) {
	cfg, err := d.readSingleByte(mdmcfg3)
	if err != nil {
		return 0, err
	}
	return uint(cfg) | (1 << 9), nil
}

// dataRateExponent gets MDMCFG3.DRATE_E
func (d *Dev) dataRateExponent() (uint, error) {
	cfg, err := d.readSingleByte(mdmcfg4)
	if err != nil {
		return 0, err
	}
	return uint(cfg) & 0x0f, nil
}

func (d *Dev) dataRate() (uint64, error) {
	mantissa, err := d.dataRateMantissa()
	if err != nil {
		return 0, err
	}
	exponent, err := d.dataRateExponent()
	if err != nil {
		return 0, err
	}
	return uint64(float64((256+mantissa)*(1<<exponent)) / (1 << 28) * 26 * 1000 * 1000), nil
}

// packetTimeout calculates a rough estimate of how long it should take to transmit a packet.
func (d *Dev) packetTimeout(length int) (time.Duration, error) {
	rate, err := d.dataRate()
	if err != nil {
		return 0, fmt.Errorf("failed to get data rate: %v", err)
	}
	bits := length * 8
	msec := float64(bits) / float64(rate) * 1000
	// Estimate as bits / baud rate + 150ms
	return time.Duration(msec)*time.Millisecond + 150*time.Millisecond, nil
}

// OscillatorFrequency estimates the frequency of XOSC by sampling GDO0
// configured to output XOSC/192.
func (d *Dev) OscillatorFrequency() (physic.Frequency, error) {
	log.Printf("Estimating oscillator frequency...")
	orig, err := d.readSingleByte(iocfg0)
	if err != nil {
		return 0, fmt.Errorf("failed to read IOCFG0: %v", err)
	}
	err = d.writeSingleByte(iocfg0, 0x3f) // CLK_XOSC/192
	if err != nil {
		return 0, fmt.Errorf("failed to set GDO0 to CLK_XOSC/192")
	}
	defer d.writeSingleByte(iocfg0, orig)

	// Raspberry Pi Zero can sample at ~3.75Mhz which is enough to recover a
	// 26Mhz clock when divided by 192
	d.gdo0.In(gpio.PullDown, gpio.NoEdge)
	samples := 10000000
	data := make([]gpio.Level, samples)
	t := time.Now()
	for i := 0; i < samples; i++ {
		data[i] = d.gdo0.Read()
	}
	t1 := time.Now()
	avgSampleDuration := float64(t1.Sub(t).Nanoseconds()) / float64(samples)

	// Calculate moving average of how many periods since last edge.
	cma := 0.0
	lastEdge := 0
	count := 0
	for i := 1; i < samples; i++ {
		if data[i] != data[i-1] {
			period := i - lastEdge
			cma = (float64(period) + float64(count)*cma) / float64(count+1)
			lastEdge = i
			count++
		}
	}
	period := time.Duration(cma*avgSampleDuration) * 2
	freq := physic.PeriodToFrequency(period) * 192
	return freq, nil
}

func frequency(xosc physic.Frequency, freq []byte) physic.Frequency {
	f := int64(freq[0])<<16 | int64(freq[1])<<8 | int64(freq[2])
	return physic.Frequency((int64(xosc) >> 16) * f)
}

func (d *Dev) Frequency() (physic.Frequency, error) {
	freq, err := d.readBurst(freq2, 3)
	if err != nil {
		return 0, fmt.Errorf("failed to read frequency configuration: %v", err)
	}
	return frequency(d.oscillator, freq), nil
}

func convertRSSI(raw int) int {
	if raw >= 128 {
		return (raw - 256/2) - 74
	}
	return raw/2 - 74
}
