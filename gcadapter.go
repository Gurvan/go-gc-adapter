package gcadapter

import (
	"errors"
	"log"
	"sync"

	// "github.com/google/gousb"
	"github.com/karalabe/hid"
)

const (
	vendorID  uint16 = 0x057E
	productID uint16 = 0x0337
)

var startPayload = []byte{0x13}

// GCAdapter represents a Gamecube controller usb adapter
type GCAdapter struct {
	controllers map[uint8]*rawGCInput
	offsets     map[uint8]*Offsets
	mutex       sync.RWMutex
	device      *hid.Device
}

// NewGCAdapter connects to a Gamecube controller usb adapter and returns a pointer to it.
// An adapter should be closed with adapter.Close() once it's not required anymore or when the program closes.
// The best way of achieving that is calling defer adapter.Close() right after adapter, err := NewGCAdapter()
func NewGCAdapter() (*GCAdapter, error) {
	var adapter = &GCAdapter{}

	if !hid.Supported() {
		return adapter, errors.New("HID is not supported on this device")
	}

	devices := hid.Enumerate(vendorID, productID)
	var deviceInfo hid.DeviceInfo
	if len(devices) > 0 {
		deviceInfo = devices[0]
	} else {
		return adapter, errors.New("GC Adapter: no adapter found")
	}
	device, err := deviceInfo.Open()
	if err != nil {
		return adapter, err
	}

	_, err = device.Write(startPayload)
	if err != nil {
		return adapter, err
	}

	adapter.device = device

	adapter.mutex.Lock()
	adapter.controllers = make(map[uint8]*rawGCInput)
	adapter.offsets = make(map[uint8]*Offsets)
	for _, PORT := range []uint8{0, 1, 2, 3} {
		adapter.controllers[PORT] = neutralRawInput()
		adapter.offsets[PORT] = neutralRawInput()
	}
	adapter.mutex.Unlock()
	return adapter, nil
}

// Poll polls the Gamecube usb adapter once
func (adapter *GCAdapter) Poll() error {
	return adapter.step()
}

// StartPolling starts a Gamecube adapter polling loop, where the adapter is polled at its defined polling rate
// (which is 125Hz by default on the official adapter).
// The function is meant to be wrapped in a goroutine.
func (adapter *GCAdapter) StartPolling() {
	var err error
	for {
		err = adapter.step()
		if err != nil {
			log.Printf("%v", err)
		}
	}
}

// Close properly closes the adapter once it's not required anymore
func (adapter *GCAdapter) Close() error {
	return adapter.device.Close()
}

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
// Sticks and triggers values are calibrated, and sticks values are clamped inside the unit circle, but deadzones are not enforced.
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
// They are used to calibrate the values returned in GCInputs.
type Offsets = rawGCInput

func (adapter *GCAdapter) step() error {
	controllers, err := readGCAdapter(adapter.device)
	if err != nil {
		return err
	}

	for PORT, controller := range controllers {
		if adapter.offsets[PORT].PluggedIn != controllers[PORT].PluggedIn {
			adapter.offsets[PORT].PluggedIn = controllers[PORT].PluggedIn
			adapter.offsets[PORT].StickX = controllers[PORT].StickX
			adapter.offsets[PORT].StickY = controllers[PORT].StickY
			adapter.offsets[PORT].CX = controllers[PORT].CX
			adapter.offsets[PORT].CY = controllers[PORT].CY
			adapter.offsets[PORT].LAnalog = controllers[PORT].LAnalog
			adapter.offsets[PORT].RAnalog = controllers[PORT].RAnalog
		}
		adapter.mutex.Lock()
		adapter.controllers[PORT] = controller
		adapter.mutex.Unlock()
	}

	return nil
}

// Controller return the current state of the controller on port PORT.
func (adapter *GCAdapter) Controller(PORT uint8) *GCInputs {
	adapter.mutex.RLock()
	defer adapter.mutex.RUnlock()
	return processRawController(adapter.controllers[PORT], adapter.offsets[PORT])
}

// Controllers return the current state of the plugged in controllers.
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

// AllControllers return the current state of the 4 controllers (even unplugged ones)
func (adapter *GCAdapter) AllControllers() map[uint8]*GCInputs {
	adapter.mutex.RLock()
	defer adapter.mutex.RUnlock()
	gcInputs := make(map[uint8]*GCInputs)
	for _, PORT := range []uint8{0, 1, 2, 3} {
		gcInputs[PORT] = processRawController(adapter.controllers[PORT], adapter.offsets[PORT])
	}
	return gcInputs
}

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

	// Deadzone
	// if math.Abs(float64(gcinput.StickX)) < 0.28 {
	// 	gcinput.StickX = 0
	// }
	// if math.Abs(float64(gcinput.StickY)) < 0.28 {
	// 	gcinput.StickY = 0
	// }
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

func readGCAdapter(device *hid.Device) (map[uint8]*rawGCInput, error) {
	rawGcInputs := make(map[uint8]*rawGCInput)

	data := make([]byte, 37)
	_, err := device.Read(data)
	if err != nil {
		// fmt.Printf("Couldn't read adapter. Error: %v\n", err)
		return rawGcInputs, err
	}
	rawGcInputs = DeserializeGCControllers(data)
	return rawGcInputs, nil
}
