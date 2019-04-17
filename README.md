# simple_uvm_example
A simple example of a UVM test environment, all in one file.

Includes:
* a simple DUT (a flip-flop)
* an interface shared as a virtual interface
* a driver, driven by a sequencer, sequences, and items
* a monitor that reports transactions through an analysis port
* a checker that checks transactions from an analysis port
* an environment object to instantiate and connect the test-bench components
* a uvm_test to instantiate the environment and run the sequences
* a top module to instantiate the DUT (device under test) and the interface
