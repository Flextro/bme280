# AVR

Seed for new avr-gcc projects

#### Install

```Shell
$ git clone https://github.com/maxmclau/avr.git
$ cd avr/
$ make
```

#### Targets

| Target | Action |
|:---|:---|
| ```*``` | Build all targets |
| ```stats``` | Print output of avr-size |
| ```clean``` | Remove all compiled files and objects |
| ```flash_programmer``` | Flash .hex file to AVR with avrdude and programmer flags |
| ```flash_bootloader``` | Flash .hex file to AVR with avrdude and bootloader flags |
| ```directories``` | Create associated directories if they do not exist |

#### License

[**`MIT`**](LICENSE)
