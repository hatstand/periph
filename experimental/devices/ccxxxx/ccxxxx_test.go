package ccxxxx

import (
	"bytes"
	"testing"

	"periph.io/x/periph/conn/physic"
)

func TestCCxxxx_calculateFreq(t *testing.T) {
	f := calculateFreq(26*physic.MegaHertz, 868300*physic.KiloHertz)
	if !bytes.Equal([]byte{0x21, 0x65, 0x6a}, f) {
		t.Fatalf("calculateFreq() = %v != %v", f, 0x21656a)
	}
}

func TestCCxxxx_calculateDeviatn(t *testing.T) {
	d := calculateDeviatn(26*physic.MegaHertz, 40*physic.KiloHertz)
	if d != 0x45 {
		t.Fatalf("calculateDeviatn() = %v != %v", d, 0x45)
	}
}

func TestCCxxx_parseFreq(t *testing.T) {
	f := frequency(26*physic.MegaHertz, []byte{0x21, 0x65, 0x6a})
	if f/physic.MegaHertz != 868 { // Close enough to 868.3MHz
		t.Fatalf("frequency() = %d != %d", f, 868300*physic.KiloHertz)
	}
}
