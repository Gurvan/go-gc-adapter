package gcadapter

import (
	"testing"
	"time"
)

func TestAdapter(t *testing.T) {
	adapter, err := NewGCAdapter()
	if err != nil {
		t.Fatalf("%v", err)
	}
	defer adapter.Close()
	go adapter.StartPolling()

	ticker := time.NewTicker(time.Second / time.Duration(60))
	go func() {
		for range ticker.C {
			t.Logf("%v", time.Now())
			inputs := adapter.Controllers()
			for k := range inputs {
				t.Logf("Controller %d start pressed: %v", k, inputs[k].Button.START)
			}
		}
	}()
	time.Sleep(3 * time.Second)
	ticker.Stop()
	t.FailNow()
}
