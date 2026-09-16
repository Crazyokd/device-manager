std_msgs/Header header

bool connected
string last_error

uint16 device_address
uint16 work_state
string work_state_text

float32 pack_voltage_v
float32 pack_current_a
float32 max_cell_voltage_v
float32 min_cell_voltage_v
float32 max_cell_temperature_c
float32 min_cell_temperature_c
float32 max_board_temperature_c
float32 min_board_temperature_c

float32 soc
float32 soh
float32 remain_capacity_mah
uint32 cycle_count

uint32 protect_status
string[] active_protections

uint32 io_status
bool charge_mos_control
bool discharge_mos_control
bool charge_mos_state
bool discharge_mos_state

string serial_number
string software_version
string hardware_version
