###############################################################################
# Created by write_sdc
# Mon Jun  5 20:20:30 2023
###############################################################################
current_design FFTSPIInterconnectRTL
###############################################################################
# Timing Constraints
###############################################################################
create_clock -name clk -period 25.0000 
set_clock_uncertainty 0.2500 clk
set_clock_latency -source -min 4.5000 [get_clocks {clk}]
set_clock_latency -source -max 6.0000 [get_clocks {clk}]
###############################################################################
# Environment
###############################################################################
set_load -pin_load 0.1400 [get_ports {io_oeb[15]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[14]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[13]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[12]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[11]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[10]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[9]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[8]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[7]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[6]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[5]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[4]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[3]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[2]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[1]}]
set_load -pin_load 0.1400 [get_ports {io_oeb[0]}]
set_timing_derate -early 0.9500
set_timing_derate -late 1.0500
###############################################################################
# Design Rules
###############################################################################
set_max_transition 0.7500 [current_design]
set_max_fanout 10.0000 [current_design]
