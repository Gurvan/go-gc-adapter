//go:build none
// +build none

package main

import (
	"fmt"
	"time"

	gca "github.com/Gurvan/go-gc-adapter"
)

func main() {
	adapter, err := gca.NewGCAdapter()
	if err != nil {
		fmt.Printf("%v\n", err)
	}
	defer adapter.Close()
	go adapter.StartPolling()

	ticker := time.NewTicker(time.Second / time.Duration(60))
	go func() {
		for range ticker.C {
			// t.Logf("%v", time.Now())
			inputs := adapter.Controllers()
			for k := range inputs {
				fmt.Printf("Controller %d start pressed: %v\n", k, inputs[k].Button.START)
			}
		}
	}()
	time.Sleep(3 * time.Second)
	ticker.Stop()
}
