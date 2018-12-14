package main

import (
	"encoding/hex"
	"flag"
	"log"

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

	if *send {
		dev.Send([]byte{})
	}

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
