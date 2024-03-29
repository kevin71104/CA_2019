# Begin_DVE_Session_Save_Info
# DVE view(Wave.1 ) session
# Saved on Mon Sep 14 17:18:01 2015
# Toplevel windows open: 2
# 	TopLevel.1
# 	TopLevel.2
#   Wave.1: 22 signals
# End_DVE_Session_Save_Info

# DVE version: I-2014.03_Full64
# DVE build date: Feb 27 2014 20:44:00


#<Session mode="View" path="/home/alavrov/ta/ele475/lab_tests/ele475-lab1/build/dve_template.vpd.tcl" type="Debug">

#<Database>

gui_set_time_units 1s
#</Database>

# DVE View/pane content session: 

# Begin_DVE_Session_Save_Info (Wave.1)
# DVE wave signals session
# Saved on Mon Sep 14 17:18:01 2015
# 22 signals
# End_DVE_Session_Save_Info

# DVE version: I-2014.03_Full64
# DVE build date: Feb 27 2014 20:44:00


#Add ncecessay scopes
gui_load_child_values {tester.t0.imuldiv}

gui_set_time_units 1s

set _wave_session_group_1 imuldiv
if {[gui_sg_is_group -name "$_wave_session_group_1"]} {
    set _wave_session_group_1 [gui_sg_generate_new_name]
}
set Group1 "$_wave_session_group_1"

gui_sg_addsignal -group "$_wave_session_group_1" { {V1:tester.t0.imuldiv.clk} {V1:tester.t0.imuldiv.is_result_signed} {V1:tester.t0.imuldiv.is_result_signed_divmul} {V1:tester.t0.imuldiv.is_result_signed_rem} {V1:tester.t0.imuldiv.muldivreq_msg_a} {V1:tester.t0.imuldiv.muldivreq_msg_b} {V1:tester.t0.imuldiv.muldivreq_msg_fn} {V1:tester.t0.imuldiv.muldivreq_rdy} {V1:tester.t0.imuldiv.muldivreq_val} {V1:tester.t0.imuldiv.muldivresp_msg_result} {V1:tester.t0.imuldiv.muldivresp_rdy} {V1:tester.t0.imuldiv.muldivresp_val} {V1:tester.t0.imuldiv.reset} {V1:tester.t0.imuldiv.sign_bit_a} {V1:tester.t0.imuldiv.sign_bit_b} {V1:tester.t0.imuldiv.unsigned_a} {V1:tester.t0.imuldiv.unsigned_b} {V1:tester.t0.imuldiv.unsigned_result} {V1:tester.t0.imuldiv.a_reg} {V1:tester.t0.imuldiv.b_reg} {V1:tester.t0.imuldiv.fn_reg} {V1:tester.t0.imuldiv.val_reg} }
if {![info exists useOldWindow]} { 
	set useOldWindow true
}
if {$useOldWindow && [string first "Wave" [gui_get_current_window -view]]==0} { 
	set Wave.1 [gui_get_current_window -view] 
} else {
	gui_open_window Wave
set Wave.1 [ gui_get_current_window -view ]
}
set groupExD [gui_get_pref_value -category Wave -key exclusiveSG]
gui_set_pref_value -category Wave -key exclusiveSG -value {false}
set origWaveHeight [gui_get_pref_value -category Wave -key waveRowHeight]
gui_list_set_height -id Wave -height 25
set origGroupCreationState [gui_list_create_group_when_add -wave]
gui_list_create_group_when_add -wave -disable
gui_marker_set_ref -id ${Wave.1}  C1
gui_wv_zoom_timerange -id ${Wave.1} 0 445
gui_list_add_group -id ${Wave.1} -after {New Group} [list ${Group1}]
gui_seek_criteria -id ${Wave.1} {Any Edge}


gui_set_pref_value -category Wave -key exclusiveSG -value $groupExD
gui_list_set_height -id Wave -height $origWaveHeight
if {$origGroupCreationState} {
	gui_list_create_group_when_add -wave -enable
}
if { $groupExD } {
 gui_msg_report -code DVWW028
}
gui_list_set_filter -id ${Wave.1} -list { {Buffer 1} {Input 1} {Others 1} {Linkage 1} {Output 1} {Parameter 1} {All 1} {Aggregate 1} {LibBaseMember 1} {Event 1} {Assertion 1} {Constant 1} {Interface 1} {BaseMembers 1} {Signal 1} {$unit 1} {Inout 1} {Variable 1} }
gui_list_set_filter -id ${Wave.1} -text {*}
gui_list_set_insertion_bar  -id ${Wave.1} -group ${Group1}  -position in

gui_marker_move -id ${Wave.1} {C1} 0
gui_view_scroll -id ${Wave.1} -vertical -set 0
gui_show_grid -id ${Wave.1} -enable false
#</Session>

