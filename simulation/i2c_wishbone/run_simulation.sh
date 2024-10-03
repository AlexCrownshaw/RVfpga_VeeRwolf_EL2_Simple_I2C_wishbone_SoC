#!/bin/bash

# Set coverage database directory and name
COV_DB_DIR="./coverage_db"
COV_DB_NAME="coverage_db"

# Set report directory for the HTML report
REPORT_DIR="./coverage_report"
REPORT_FORMAT="html"

# Compile the Verilog testbench and design files with functional coverage enabled
xvlog --sv tb_i2c_wishbone.sv \
      ../../src/VeeRwolf/Peripherals/i2c/i2c_core.sv \
      ../../src/VeeRwolf/Peripherals/i2c/i2c_memory_map.sv \
      ../../src/VeeRwolf/Peripherals/i2c/i2c_wishbone.sv

# Elaborate the design with coverage database enabled
xelab tb_i2c_wishbone -debug typical --timescale 1ns/1ps \
      -s i2c_wishbone -cov_db_dir $COV_DB_DIR -cov_db_name $COV_DB_NAME -R

# Simulate the design and collect functional coverage
xsim i2c_wishbone --tclbatch run_commands.tcl --wdb tb_i2c_wishbone.wdb --log sim_log.txt --coverage functional

# Generate functional coverage report in HTML format
xcrg -dir $COV_DB_DIR -db_name $COV_DB_NAME -report_dir $REPORT_DIR -report_format $REPORT_FORMAT

# Open the HTML report in a web browser
firefox $REPORT_DIR/dashboard.html &