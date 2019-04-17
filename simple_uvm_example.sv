`include "uvm_pkg.sv"
import uvm_pkg::*;
`include "uvm_macros.svh"


interface my_interface(
  output reset_n,
  output clk,
  output [31:0] value,
  input [31:0] previous_value
);
endinterface


module my_dut(
  input reset_n,
  input clk,
  input [31:0] value,
  output reg [31:0] previous_value
);
  always_ff @(posedge clk, negedge reset_n) begin
    if (!reset_n) previous_value <= 0;
    else previous_value <= value;
  end
endmodule


class my_item extends uvm_sequence_item;
  rand reg [31:0] value;
  bit reset = 0;
  rand int unsigned reset_duration;
  constraint reset_duration_c { reset_duration > 1_000; reset_duration < 10_000; }
  `uvm_object_utils(my_item)
  function new(string name="my_item");
    super.new(name);
  endfunction
endclass


typedef uvm_sequencer #(my_item) my_sequencer;


class my_driver extends uvm_driver#(my_item);
  virtual my_interface vif;
  `uvm_component_utils(my_driver)
  function new(string name="my_driver", uvm_component parent=null);
    super.new(name, parent);
  endfunction
  virtual function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    assert (uvm_config_db#(virtual my_interface)::get(null, "", "ifc", vif) && vif != null)
    else `uvm_fatal("MY_DRIVER_VIF", "Could not get my_interface from uvm_config_db")
  endfunction
  virtual task run_phase(uvm_phase phase);
    super.run_phase(phase);
    fork drive_input_task(); join_none
  endtask
  virtual task drive_input_task();
    #1;
    forever begin
      seq_item_port.get(req);
      if (req.reset) handle_reset(); else handle_value();
    end
  endtask
  bit clk_active = 0;
  virtual task handle_reset();
    `uvm_info(get_name(), "Driving reset", UVM_LOW)
    vif.reset_n = 0;
    vif.value = 'x;
    clk_active = 0;
    #(req.reset_duration - 50);
    fork drive_clock_task(); join_none
    vif.value = '0;
    #50;
    `uvm_info(get_name(), "Reset complete", UVM_LOW)
    vif.reset_n = 1;
    #1;
  endtask
  virtual task drive_clock_task();
    clk_active = 1;
    vif.clk = '0;
    `uvm_info(get_name(), "Starting clock", UVM_LOW)
    while (clk_active) begin
      #10;
      vif.clk = ~vif.clk;
    end
    `uvm_info(get_name(), "Stopping clock", UVM_LOW)
    vif.clk = 'x;
  endtask
  virtual task handle_value();
    @(negedge vif.clk)
    `uvm_info(get_name(), $sformatf("Driving req.value: 'h%x", req.value), UVM_LOW)
    vif.value = req.value;
  endtask
endclass


class my_transaction extends uvm_object;
  bit reset;
  reg [31:0] value, previous_value;
  `uvm_object_utils(my_transaction)
  function new(string name="my_transaction");
    super.new(name);
  endfunction
  virtual function string convert2string();
    return reset ? "{reset}" : $sformatf("{value:'h%x, previous_value:'h%x}", value, previous_value);
  endfunction
endclass


class my_monitor extends uvm_monitor;
  virtual my_interface vif;
  uvm_analysis_port #(my_transaction) out_port;
  `uvm_component_utils(my_monitor)
  function new(string name="my_monitor", uvm_component parent=null);
    super.new(name, parent);
    out_port = new("out_port", this);
  endfunction
  virtual function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    assert (uvm_config_db#(virtual my_interface)::get(null, "", "ifc", vif) && vif != null)
    else `uvm_fatal("MY_MONITOR_VIF", "Could not get my_interface from uvm_config_db")
  endfunction
  virtual task run_phase(uvm_phase phase);
    super.run_phase(phase);
    fork
      monitor_reset_thread();
      monitor_value_thread();
    join_none
  endtask
  my_transaction txn;
  virtual task monitor_reset_thread();
    forever begin
      wait (vif.reset_n === 1'b1);
      @(posedge vif.clk);
      if (vif.reset_n) begin
        txn = my_transaction::type_id::create("txn", this);
        txn.value = vif.value;
        txn.previous_value = vif.previous_value;
        out_port.write(txn);
      end
    end
  endtask
  virtual task monitor_value_thread();
    forever begin
      @(negedge vif.reset_n);
      txn = my_transaction::type_id::create("txn", this);
      txn.reset = 1;
      out_port.write(txn);
    end
  endtask
endclass


class my_checker extends uvm_subscriber#(my_transaction);
  `uvm_component_utils(my_checker)
  function new(string name="my_checker", uvm_component parent=null);
    super.new(name, parent);
  endfunction
  my_transaction last_txn;
  virtual function void write(my_transaction t); // Called automatically when something comes through the analysis port
    if (t.reset) last_txn = null;
    else begin
      check_txn(t);
      last_txn = t;
    end
  endfunction
  virtual function void check_txn(my_transaction t);
    if (last_txn == null) return;
    if (t.previous_value !== last_txn.value) begin
      `uvm_error("PREVIOUS_VALUE_ERROR", $sformatf("previous_value (%x) did not match the expected value (%x)",
        t.previous_value, last_txn.value))
    end else begin
      `uvm_info(get_name(), $sformatf("txn passed check: previous_value:%x", t.previous_value), UVM_HIGH)
    end
  endfunction
endclass


class my_env extends uvm_env;
  `uvm_component_utils(my_env)
  my_driver driver;
  my_sequencer sequencer;
  my_monitor monitor;
  my_checker checker;
  function new(string name="my_env", uvm_component parent=null);
    super.new(name, parent);
  endfunction
  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    driver = my_driver::type_id::create("driver", this);
    sequencer = my_sequencer::type_id::create("sequencer", this);
    monitor = my_monitor::type_id::create("monitor", this);
    checker = my_checker::type_id::create("checker", this);
  endfunction
  virtual function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    driver.seq_item_port.connect(sequencer.seq_item_export);
    monitor.out_port.connect(checker.analysis_export);
  endfunction
endclass


class my_sequence extends uvm_sequence #(my_item);
  virtual my_interface vif;
  `uvm_object_utils(my_sequence)
  function new(string name="my_sequence");
    super.new(name);
    assert (uvm_config_db#(virtual my_interface)::get(null, "", "ifc", vif) && vif != null)
    else `uvm_fatal("MY_SEQUENCE_VIF", "Could not get my_interface from uvm_config_db")
  endfunction
endclass


class my_reset_sequence extends my_sequence;
  `uvm_object_utils(my_reset_sequence)
  function new(string name="my_reset_sequence");
    super.new(name);
  endfunction
  virtual task body();
    req = my_item::type_id::create("req");
    start_item(req);
    assert (req.randomize())
    else `uvm_fatal("ITEM_RAND", "Failed to randomize my_item.")
    req.reset = 1;
    finish_item(req);
  endtask
endclass


class my_value_sequence extends my_sequence;
  rand int unsigned delay;
  constraint delay_c { delay < 15; }
  `uvm_object_utils(my_value_sequence)
  function new(string name="my_value_sequence");
    super.new(name);
  endfunction
  virtual task body();
    repeat (delay) @(posedge vif.clk);
    req = my_item::type_id::create("req");
    start_item(req);
    assert (req.randomize())
    else `uvm_fatal("ITEM_RAND", "Failed to randomize my_item.")
    finish_item(req);
  endtask
endclass


class my_test extends uvm_test;
  `uvm_component_utils(my_test)
  my_env env;
  my_sequence seq;
  function new(string name="my_test", uvm_component parent=null);
    super.new(name, parent);
  endfunction
  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    env = my_env::type_id::create("env", this);
  endfunction
  virtual task run_phase(uvm_phase phase);
    super.run_phase(phase);
    phase.raise_objection(this);
    `uvm_info(get_name(), "Starting my_test", UVM_LOW)
    do_reset();
    do_sequences(100);
    do_reset();
    do_sequences(50);
    `uvm_info(get_name(), "End of my_test", UVM_LOW)
    phase.drop_objection(this);
  endtask
  virtual task do_reset();
    seq = my_reset_sequence::type_id::create("seq");
    seq.start(env.sequencer);
  endtask
  virtual task do_sequences(int n);
    repeat (n) begin
      seq = my_value_sequence::type_id::create("seq");
      seq.start(env.sequencer);
      assert (seq.randomize())
      else `uvm_fatal("SEQ_RAND", "Failed to randomize my_sequence")
      seq.start(env.sequencer);
    end
  endtask
endclass


module top;
  reg reset_n, clk;
  reg [31:0] value, previous_value;
  my_dut dut(.*);
  my_interface ifc(.*);
  initial uvm_config_db#(virtual my_interface)::set(null, "", "ifc", ifc);
  initial run_test("my_test");
endmodule
