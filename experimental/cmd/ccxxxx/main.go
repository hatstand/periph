package main

import (
	"encoding/hex"
	"flag"
	"log"

	"periph.io/x/periph/conn/physic"
	"periph.io/x/periph/conn/gpio/gpioreg"
	"periph.io/x/periph/conn/spi/spireg"
	"periph.io/x/periph/experimental/devices/ccxxxx"
	"periph.io/x/periph/host"
)

var send = flag.Bool("send", false, "Whether to send or receive packets")

func main() {
	flag.Parse()
	host.Init()
	port, err := spireg.Open("")
	if err != nil {
		log.Fatalf("spireg.Open: %v", err)
	}

	gdo0 := gpioreg.ByName("25")
	gdo2 := gpioreg.ByName("24")

	dev, err := ccxxxx.New(port, gdo0, gdo2)
	if err != nil {
		log.Fatalf("ccxxxx: %v", err)
	}

	dev.Config(ccxxxx.Config_868_3)
	freq, err := dev.OscillatorFrequency()
	if err != nil {
		log.Fatalf("Failed to get XOSC: %v", err)
	}
	log.Printf("XOSC: %v", freq)

	f, err := dev.Frequency()
	if err != nil {
		log.Fatalf("Failed to get configured frequency: %v", err)
	}
	log.Printf("Configured FREQ: %v", f)
	carrier := physic.Frequency(int64(freq) / (1 << 16) * int64(f))
	log.Printf("Carrier: %v", carrier)

	dev.SetDeviation(40 * physic.KiloHertz)
	dev.SetFrequency(868300 * physic.KiloHertz)

	if *send {
		log.Println("Sending packet")
		_, err := dev.Write([]byte{0x57, 0x16, 0x0a, 0x2e, 0x04, 0x05, 60, 60, 20})
		if err != nil {
			log.Fatalf("Failed to send packet: %v", err)
		}
	} else {
		for {
			packet, err := dev.Receive(-1)
			if err != nil {
				log.Printf("Error receiving packet: %v", err)
			} else {
				log.Printf("RSSI: %d LQI %d CRC: %v", packet.RSSI, packet.LQI, packet.CRC)
				log.Println(hex.Dump(packet.Data))
			}
		}
	}
}
