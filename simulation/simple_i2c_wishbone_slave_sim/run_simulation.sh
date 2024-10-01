#!/bin/bash

# Set coverage database directory and name
COV_DB_DIR="./coverage_db"
COV_DB_NAME="coverage_db"

# Set report directory for the HTML report
REPORT_DIR="./coverage_report"
REPORT_FORMAT="html"

# Compile the Verilog testbench and design files with functional coverage enabled
xvlog --sv tb_simple_i2c_wishbone_slave.sv \
      ../../src/VeeRwolf/Peripherals/simple_i2c/simple_i2c.sv \
      ../../src/VeeRwolf/Peripherals/simple_i2c/simple_i2c_wishbone_slave.sv \
      ../../src/VeeRwolf/Peripherals/simple_i2c/simple_i2c_wishbone_mem_map.vh

# Elaborate the design with coverage database enabled
xelab tb_simple_i2c_wishbone_slave -debug typical --timescale 1ns/1ps \
      -s simple_i2c_wishbone_slave_sim -cov_db_dir $COV_DB_DIR -cov_db_name $COV_DB_NAME -R

# Simulate the design and collect functional coverage
xsim simple_i2c_wishbone_slave_sim --tclbatch run_commands.tcl --wdb tb_simple_i2c_wishbone_slave.wdb --log sim_log.txt

# Generate functional coverage report in HTML format
xcrg -dir $COV_DB_DIR -db_name $COV_DB_NAME -report_dir $REPORT_DIR -report_format $REPORT_FORMAT

# Open the HTML report in a web browser
firefox $REPORT_DIR/dashboard.html &