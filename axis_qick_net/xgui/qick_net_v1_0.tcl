# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_static_text $IPINST -name "PS Freq" -parent ${Page_0} -text {ps_clk Freq should be 99.999001}
  ipgui::add_param $IPINST -name "SIM_LEVEL" -parent ${Page_0}


}

proc update_PARAM_VALUE.SIM_LEVEL { PARAM_VALUE.SIM_LEVEL } {
	# Procedure called to update SIM_LEVEL when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.SIM_LEVEL { PARAM_VALUE.SIM_LEVEL } {
	# Procedure called to validate SIM_LEVEL
	return true
}


proc update_MODELPARAM_VALUE.SIM_LEVEL { MODELPARAM_VALUE.SIM_LEVEL PARAM_VALUE.SIM_LEVEL } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.SIM_LEVEL}] ${MODELPARAM_VALUE.SIM_LEVEL}
}

