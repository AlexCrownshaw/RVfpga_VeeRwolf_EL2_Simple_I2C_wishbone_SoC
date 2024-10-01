#!/bin/bash

# Compile the Verilog testbench and design files
xvlog --sv tb_simple_i2c.v \
      ../src/simplei2c.sv

# Elaborate the design
xelab tb_i2c_master_wb_slave -debug typical --timescale 1ns/1ps -s tb_i2c_master_wb_slave_sim

# Simulate the design and generate the waveform
xsim tb_i2c_master_wb_slave_sim --tclbatch run_commands.tcl --wdb tb_i2c_master_wb_slave_sim.wdb --log sim_log.txt
