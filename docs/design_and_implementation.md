[Click here](../README.md) to view the README.

## Design and implementation


### Resources and settings

The design of this application allows you to run the core benchmarking on either the CM33 or CM55 CPU, which can be configured by setting the `BENCHMARK_CPU` value in the *[common.mk](../common.mk)* file. By default, this code example runs the core benchmarking on CM55 CPU.

<br>

The firmware applies a few power optimizations to achieve the lowest power numbers while obtaining the best core benchmark score for a given configuration. The optimizations for each project are as follows:

CM33 power optimizations | CM55 power optimizations
-------------------------|--------------------------
Disable unused SRAM MACROs (64 KB each) | Disable unused SOCMEM partitions (512 KB each)
Reduce the PLL clock to 200 MHz | Change the low-power domain clock (HFCLK0) to run from IHO (50 MHz)
Turn off PD1 | Set the HFCLK0 divider to 16

<br>