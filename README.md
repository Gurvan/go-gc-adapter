# go-gc-adapter

A Go library for interfacing with the Gamecube controller adapter.

## Example

```go
package main

import (
	"fmt"
	"log"
	"time"

	gcadapter "github.com/Gurvan/go-gc-adapter"
)

func main() {
	adapter, err := gcadapter.NewGCAdapter()
	if err != nil {
		log.Fatalf("%v", err)
	}
	defer adapter.Close()
	go adapter.StartPolling()

	go func() {
		for range time.Tick(time.Second / time.Duration(60.)) {
			inputs := adapter.Controllers()
			fmt.Println(inputs[0])
		}
	}()
	time.Sleep(10 * time.Second)
}
```