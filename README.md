# Steering Wheel

Respondible for handling driver input through dashboard interface, and outputing mission data on an LCD screen. Uses STM32L432KC.

### Main functions
* Takes driver input from buttons and switches on a dashboard
* Displays mission data on a 16x2 character LCD screen

The LCD screen layout is in the following format  
|L_100_100_100_R|  
|C__ABCDEFGH__M|  
Top row: [Left indicator] [battery voltage] [speed] [motor temperature] [Right indicator]  
Bot row: [Cruise enabled] [Warnings] [Drive mode]  

The warnings are the ```ErrorFlags``` struct found in the [Prohelion WaveSculptor Library](https://github.com/team-arrow-racing/phln-rs/blob/main/src/wavesculptor.rs)
 
### Important Pins
PA0 - Drive mode select rotary switch (Drive/Neutral/Reverse)
PA11 - CAN RX  
PA12 - CAN TX
PC11 - Cruise button   
PC6 - Left indicator switch  
PC8 - Right indicator switch
PC1 - ADC IN (Data pin on a linear potentiometer connected to the cars accelerator pedal)  
PC2 - LCD RS Pin  
PC3 - LCD EN Pin  
PA6 - LCD D4 Pin  
PA7 - LCD D5 Pin  
PA4 - LCD D6 Pin  
PA5 - LCD D7 Pin  

## Requisites

- Rust
- ARM Toolchain (for debugging)
- OpenOCD (for debugging)

## Setup

```shell
# Add cross-compilation target
rustup target add thumbv7em-none-eabihf
# Install probe-run
cargo install probe-run
```

On Linux you may need to configure your udev rules to allow running without root.

## Running

The following will compile, flash and debug the program.

```shell
DEFMT_LOG=info cargo run
```

## References

- [Real-Time Interrupt-driven Concurrency Framework](https://rtic.rs/1/)
- [defmt](https://defmt.ferrous-systems.com/)
- [probe-run](https://github.com/knurling-rs/probe-run)
- [Rust HAL Documentation](https://docs.rs/stm32l4xx-hal/latest/stm32l4xx_hal/)
- [RM0394 Reference Manual for STM32L41xxx/42xxx/43xxx/44xxx/45xxx/46xxx](https://www.st.com/resource/en/reference_manual/dm00151940-stm32l41xxx42xxx43xxx44xxx45xxx46xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

## About Team Arrow

Team Arrow Racing Association is a volunteer organisation that designs, develops and races world-class solar powered vehicles. This repository is part of our endevour to build in the open and make our learnings accessible to anyone.

You can find our more about Team Arrow on [our website](https://www.teamarrow.com.au/).
