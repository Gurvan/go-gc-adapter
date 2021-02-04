package gcadapter

import (
	"math"
	"unsafe"
)

func byteToInt8(b byte) int8 {
	i := int8(0)
	*(*uint8)(unsafe.Pointer(uintptr(unsafe.Pointer(&i)) + uintptr(0))) = b
	return i
}

func int8ToByte(i int8) byte {
	b := byte(0)
	*(*int8)(unsafe.Pointer(uintptr(unsafe.Pointer(&b)) + uintptr(0))) = i
	return b
}

func clampStick(x, y uint8) (uint8, uint8) {
	fX, fY := float64(int8(x-128)), float64(int8(y-128))
	magnitudeSquared := fX*fX + fY*fY

	if magnitudeSquared < 1e-3 {
		return 0, 0
	}

	magnitude := math.Sqrt(magnitudeSquared)
	threshold := 80.

	if magnitude > threshold {
		fX = float64(float32(fX) * float32(threshold) / float32(magnitude))
		fY = float64(float32(fY) * float32(threshold) / float32(magnitude))
	}
	iX, iY := int8(fX), int8(fY)
	return int8ToByte(iX), int8ToByte(iY)
}

func correctStickOffset(x, xNeutral uint8) uint8 {
	if x > xNeutral {
		xDiff := x - xNeutral
		xNew := uint8(128) + xDiff
		if xNew < 128 { // check for overflow
			// fmt.Println("add overflow")
			xNew = 255
		}
		return xNew
	}
	xDiff := xNeutral - x
	xNew := uint8(128) - xDiff
	if xNew > 128 { // check for overflow
		// fmt.Println("sub overflow")
		xNew = 0
	}
	return xNew
}

func correctTriggerOffset(x, xNeutral uint8) uint8 {
	xNew := x - xNeutral
	if xNew > (255 - xNeutral) {
		xNew = 0
	}
	return xNew
}
