package gcadapter

/*
#cgo CFLAGS: -I${SRCDIR}/libusb/libusb
#cgo windows LDFLAGS: ${SRCDIR}/libusb-1.0.a -lsetupapi -lole32 -ladvapi32
#include <libusb.h>
#include <stdlib.h>

static int control_transfer(libusb_device_handle *dev_handle,
    uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
    unsigned char *data, uint16_t wLength, unsigned int timeout)
{
    return libusb_control_transfer(dev_handle, bmRequestType, bRequest,
        wValue, wIndex, data, wLength, timeout);
}

static int interrupt_transfer(libusb_device_handle *dev_handle,
    unsigned char endpoint, unsigned char *data, int length,
    int *actual_length, unsigned int timeout)
{
    return libusb_interrupt_transfer(dev_handle, endpoint, data, length,
        actual_length, timeout);
}
*/
import "C"
import (
	"errors"
	"fmt"
	"sync"
)

const (
	vendorID  = 0x057E
	productID = 0x0337
)

var startPayload = []byte{0x13}

// GCAdapter represents a Gamecube controller USB adapter
type GCAdapter struct {
	controllers map[uint8]*rawGCInput
	offsets     map[uint8]*Offsets
	mutex       sync.RWMutex
	context     *C.libusb_context
	device      *C.libusb_device_handle
	buffer      []byte
	endpoint    uint8
	stopChan    chan bool
}

// NewGCAdapter connects to a Gamecube controller USB adapter and returns a pointer to it.
func NewGCAdapter() (*GCAdapter, error) {
	adapter := &GCAdapter{
		controllers: make(map[uint8]*rawGCInput),
		offsets:     make(map[uint8]*Offsets),
		buffer:      make([]byte, 37),
		endpoint:    0x81, // IN endpoint
		stopChan:    make(chan bool),
	}

	if err := C.libusb_init(&adapter.context); err != 0 {
		return nil, fmt.Errorf("failed to initialize libusb: %d", err)
	}

	adapter.device = C.libusb_open_device_with_vid_pid(adapter.context, C.uint16_t(vendorID), C.uint16_t(productID))
	if adapter.device == nil {
		C.libusb_exit(adapter.context)
		return nil, errors.New("GC Adapter: no adapter found")
	}

	if err := C.libusb_claim_interface(adapter.device, 0); err != 0 {
		C.libusb_close(adapter.device)
		C.libusb_exit(adapter.context)
		return nil, fmt.Errorf("failed to claim interface: %d", err)
	}

	// Send start payload using control transfer
	result := C.control_transfer(adapter.device, 0x21, 0x09, 0x0200, 0, (*C.uchar)(&startPayload[0]), C.uint16_t(len(startPayload)), 1000)
	if result < 0 {
		C.libusb_release_interface(adapter.device, 0)
		C.libusb_close(adapter.device)
		C.libusb_exit(adapter.context)
		return nil, fmt.Errorf("failed to send start payload: %d", result)
	}

	for _, PORT := range []uint8{0, 1, 2, 3} {
		adapter.controllers[PORT] = neutralRawInput()
		adapter.offsets[PORT] = neutralRawInput()
	}

	return adapter, nil
}

// Poll polls the Gamecube USB adapter once
func (adapter *GCAdapter) Poll() error {
	return adapter.step()
}

// StartPolling starts a Gamecube adapter polling loop
func (adapter *GCAdapter) StartPolling() {
	go func() {
		for {
			select {
			case <-adapter.stopChan:
				return
			default:
				if err := adapter.step(); err != nil {
					fmt.Printf("Polling error: %v\n", err)
				}
			}
		}
	}()
}

// Close properly closes the adapter
func (adapter *GCAdapter) Close() error {
	adapter.stopChan <- true
	if adapter.device != nil {
		C.libusb_release_interface(adapter.device, 0)
		C.libusb_close(adapter.device)
		adapter.device = nil
	}
	if adapter.context != nil {
		C.libusb_exit(adapter.context)
		adapter.context = nil
	}

	return nil
}

func (adapter *GCAdapter) step() error {
	var transferred C.int
	result := C.interrupt_transfer(adapter.device, C.uchar(adapter.endpoint), (*C.uchar)(&adapter.buffer[0]), C.int(len(adapter.buffer)), &transferred, 1000)
	if result != 0 {
		return fmt.Errorf("failed to read from device: %d", result)
	}

	if int(transferred) != len(adapter.buffer) {
		return fmt.Errorf("incomplete read: got %d bytes, expected %d", int(transferred), len(adapter.buffer))
	}

	controllers := DeserializeGCControllers(adapter.buffer)

	adapter.mutex.Lock()
	defer adapter.mutex.Unlock()

	for PORT, controller := range controllers {
		if adapter.offsets[PORT].PluggedIn != controller.PluggedIn {
			adapter.offsets[PORT] = controller
		}
		adapter.controllers[PORT] = controller
	}

	return nil
}

// The rest of the code (Controller, Controllers, AllControllers, etc.) remains the same...

// Helper function to convert libusb errors to Go errors
func fromRawErrno(errno C.int) error {
	if errno < 0 {
		return fmt.Errorf("libusb error: %d", errno)
	}
	return nil
}

// // Poll polls the Gamecube USB adapter once
// func (adapter *GCAdapter) Poll() error {
// 	return adapter.step()
// }

// // StartPolling starts a Gamecube adapter polling loop
// func (adapter *GCAdapter) StartPolling() {
// 	for {
// 		if err := adapter.step(); err != nil {
// 			fmt.Printf("Polling error: %v\n", err)
// 		}
// 	}
// }

// // Close properly closes the adapter
// func (adapter *GCAdapter) Close() {
// 	if adapter.transfer != nil {
// 		C.free_transfer(adapter.transfer)
// 	}
// 	if adapter.device != nil {
// 		C.libusb_release_interface(adapter.device, 0)
// 		C.libusb_close(adapter.device)
// 	}
// 	if adapter.context != nil {
// 		C.libusb_exit(adapter.context)
// 	}
// }

// func (adapter *GCAdapter) step() error {
// 	C.libusb_fill_interrupt_transfer(adapter.transfer, adapter.device, 0x81, (*C.uchar)(&adapter.buffer[0]), C.int(len(adapter.buffer)), nil, nil, 1000)

// 	if err := C.submit_transfer(adapter.transfer); err != 0 {
// 		return fmt.Errorf("failed to submit transfer: %d", err)
// 	}

// 	if err := C.libusb_handle_events(adapter.context); err != 0 {
// 		return fmt.Errorf("failed to handle events: %d", err)
// 	}

// 	controllers := DeserializeGCControllers(adapter.buffer)

// 	adapter.mutex.Lock()
// 	defer adapter.mutex.Unlock()

// 	for PORT, controller := range controllers {
// 		if adapter.offsets[PORT].PluggedIn != controller.PluggedIn {
// 			adapter.offsets[PORT] = controller
// 		}
// 		adapter.controllers[PORT] = controller
// 	}

// 	return nil
// }

// Controller returns the current state of the controller on port PORT.
func (adapter *GCAdapter) Controller(PORT uint8) *GCInputs {
	adapter.mutex.RLock()
	defer adapter.mutex.RUnlock()
	return processRawController(adapter.controllers[PORT], adapter.offsets[PORT])
}

// Controllers returns the current state of the plugged in controllers.
func (adapter *GCAdapter) Controllers() map[uint8]*GCInputs {
	adapter.mutex.RLock()
	defer adapter.mutex.RUnlock()
	gcInputs := make(map[uint8]*GCInputs)
	for _, PORT := range []uint8{0, 1, 2, 3} {
		if adapter.controllers[PORT].PluggedIn {
			gcInputs[PORT] = processRawController(adapter.controllers[PORT], adapter.offsets[PORT])
		}
	}
	return gcInputs
}

// AllControllers returns the current state of the 4 controllers (even unplugged ones)
func (adapter *GCAdapter) AllControllers() map[uint8]*GCInputs {
	adapter.mutex.RLock()
	defer adapter.mutex.RUnlock()
	gcInputs := make(map[uint8]*GCInputs)
	for _, PORT := range []uint8{0, 1, 2, 3} {
		gcInputs[PORT] = processRawController(adapter.controllers[PORT], adapter.offsets[PORT])
	}
	return gcInputs
}

// The rest of the code (Buttons, rawGCInput, GCInputs, Offsets, processRawController, DeserializeGCControllers)
// remains the same as in the original implementation.

// Buttons represents the Gamecube controller buttons
type Buttons struct {
	UP    bool
	DOWN  bool
	RIGHT bool
	LEFT  bool
	Y     bool
	X     bool
	B     bool
	A     bool
	L     bool
	R     bool
	Z     bool
	START bool
}

type rawGCInput struct {
	Button    Buttons
	StickX    uint8
	StickY    uint8
	CX        uint8
	CY        uint8
	LAnalog   uint8
	RAnalog   uint8
	PluggedIn bool
}

func neutralRawInput() *rawGCInput {
	return &rawGCInput{
		Button:    Buttons{},
		StickX:    128,
		StickY:    128,
		CX:        128,
		CY:        128,
		LAnalog:   255,
		RAnalog:   255,
		PluggedIn: false,
	}
}

// GCInputs represent the state of the gamecube controller, with sticks values in [-1, 1], triggers values in [0, 1] and buttons as boolean
type GCInputs struct {
	Button    Buttons
	StickX    float32
	StickY    float32
	CX        float32
	CY        float32
	LAnalog   float32
	RAnalog   float32
	PluggedIn bool
}

// Offsets are the sticks and trigger values read right after plugging the controller or resetting it with X+Y+Start.
type Offsets = rawGCInput

func processRawController(rawInput *rawGCInput, offsets *Offsets) *GCInputs {
	gcinput := GCInputs{}
	gcinput.Button = rawInput.Button

	x, y := rawInput.StickX, rawInput.StickY
	dx, dy := offsets.StickX, offsets.StickY
	x, y = correctStickOffset(x, dx), correctStickOffset(y, dy)
	x, y = clampStick(x, y)
	gcinput.StickX = float32(0.0125 * float64(byteToInt8(x)))
	gcinput.StickY = float32(0.0125 * float64(byteToInt8(y)))

	x, y = rawInput.CX, rawInput.CY
	dx, dy = offsets.CX, offsets.CY
	x, y = correctStickOffset(x, dx), correctStickOffset(y, dy)
	x, y = clampStick(x, y)
	gcinput.CX = float32(0.0125 * float64(byteToInt8(x)))
	gcinput.CY = float32(0.0125 * float64(byteToInt8(y)))

	l := rawInput.LAnalog
	l = correctTriggerOffset(l, offsets.LAnalog)
	if l > 140 {
		l = 140
	}
	gcinput.LAnalog = float32(l) / 140.

	r := rawInput.RAnalog
	r = correctTriggerOffset(r, offsets.LAnalog)
	if r > 140 {
		r = 140
	}
	gcinput.RAnalog = float32(r) / 140.

	gcinput.PluggedIn = rawInput.PluggedIn

	return &gcinput
}

func DeserializeGCControllers(data []byte) map[uint8]*rawGCInput {
	gcInputs := make(map[uint8]*rawGCInput)
	for _, PORT := range []uint8{0, 1, 2, 3} {
		gcInput := &rawGCInput{}

		gcInput.PluggedIn = data[9*PORT+1] == 20 || data[9*PORT+1] == 16

		gcInput.Button.UP = data[9*PORT+2]&0b10000000 != 0
		gcInput.Button.DOWN = data[9*PORT+2]&0b01000000 != 0
		gcInput.Button.RIGHT = data[9*PORT+2]&0b00100000 != 0
		gcInput.Button.LEFT = data[9*PORT+2]&0b00010000 != 0
		gcInput.Button.Y = data[9*PORT+2]&0b00001000 != 0
		gcInput.Button.X = data[9*PORT+2]&0b00000100 != 0
		gcInput.Button.B = data[9*PORT+2]&0b00000010 != 0
		gcInput.Button.A = data[9*PORT+2]&0b00000001 != 0
		gcInput.Button.L = data[9*PORT+3]&0b00001000 != 0
		gcInput.Button.R = data[9*PORT+3]&0b00000100 != 0
		gcInput.Button.Z = data[9*PORT+3]&0b00000010 != 0
		gcInput.Button.START = data[9*PORT+3]&0b00000001 != 0

		gcInput.StickX = data[9*PORT+4]
		gcInput.StickY = data[9*PORT+5]
		gcInput.CX = data[9*PORT+6]
		gcInput.CY = data[9*PORT+7]
		gcInput.LAnalog = data[9*PORT+8]
		gcInput.RAnalog = data[9*PORT+9]

		gcInputs[PORT] = gcInput
	}
	return gcInputs
}
