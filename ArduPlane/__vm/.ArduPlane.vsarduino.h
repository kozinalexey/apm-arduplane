/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Mega 2560 HAL (Apm 2), Platform=avr, Package=arduino
*/

#define __AVR_ATmega2560__
#define ARDUINO 166
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define CONFIG_HAL_BOARD HAL_BOARD_APM2
#define EXCLUDECORE
extern "C" void __cxa_pure_virtual() {;}

//
//
static void ahrs_update();
static void update_speed_height(void);
static void update_mount(void);
static void update_compass(void);
static void compass_accumulate(void);
static void barometer_accumulate(void);
static void update_logging1(void);
static void update_logging2(void);
static void obc_fs_check(void);
static void update_aux(void);
static void one_second_loop();
static void log_perf_info();
static void compass_save();
static void terrain_update(void);
static void airspeed_ratio_update(void);
static void update_GPS_50Hz(void);
static void update_GPS_10Hz(void);
static void handle_auto_mode(void);
static void update_flight_mode(void);
static void update_navigation();
static void update_alt();
static void update_flight_stage(void);
static void determine_is_flying(void);
static bool is_flying(void);
static void update_optical_flow(void);
static void adjust_altitude_target();
static void setup_glide_slope(void);
static int32_t get_RTL_altitude();
static float relative_altitude(void);
static int32_t relative_altitude_abs_cm(void);
static void set_target_altitude_current(void);
static void set_target_altitude_current_adjusted(void);
static void set_target_altitude_location(const Location &loc);
static int32_t relative_target_altitude_cm(void);
static void change_target_altitude(int32_t change_cm);
static void set_target_altitude_proportion(const Location &loc, float proportion);
static void constrain_target_altitude_location(const Location &loc1, const Location &loc2);
static int32_t calc_altitude_error_cm(void);
static void check_minimum_altitude(void);
static void reset_offset_altitude(void);
static void set_offset_altitude_location(const Location &loc);
static bool above_location_current(const Location &loc);
static void setup_terrain_target_alt(Location &loc);
static int32_t adjusted_altitude_cm(void);
static int32_t adjusted_relative_altitude_cm(void);
static float height_above_target(void);
static float lookahead_adjustment(void);
static float rangefinder_correction(void);
static void rangefinder_height_update(void);
static float get_speed_scaler(void);
static bool stick_mixing_enabled(void);
static void stabilize_roll(float speed_scaler);
static void stabilize_pitch(float speed_scaler);
static void stick_mix_channel(RC_Channel *channel, int16_t &servo_out);
static void stabilize_stick_mixing_direct();
static void stabilize_stick_mixing_fbw();
static void stabilize_yaw(float speed_scaler);
static void stabilize_training(float speed_scaler);
static void stabilize_acro(float speed_scaler);
static void stabilize();
static void calc_throttle();
static void calc_nav_yaw_coordinated(float speed_scaler);
static void calc_nav_yaw_course(void);
static void calc_nav_yaw_ground(void);
static void calc_nav_pitch();
static void calc_nav_roll();
static void throttle_slew_limit(int16_t last_throttle);
static void flap_slew_limit(int8_t &last_value, int8_t &new_value);
static bool suppress_throttle(void);
static void channel_output_mixer(uint8_t mixing_type, int16_t &chan1_out, int16_t &chan2_out);
static void flaperon_update(int8_t flap_percent);
static void set_servos(void);
static void demo_servos(uint8_t i);
static void adjust_nav_pitch_throttle(void);
static void update_load_factor(void);
void add_altitude_data(unsigned long xl, long y);
static void set_next_WP(const struct Location &loc);
static void set_guided_WP(void);
static void init_home();
static void update_home();
static void do_RTL(void);
static bool verify_takeoff();
static bool verify_loiter_unlim();
static bool verify_loiter_time();
static bool verify_loiter_turns();
static bool verify_loiter_to_alt();
static bool verify_RTL();
static bool verify_continue_and_change_alt();
static bool verify_wait_delay();
static bool verify_change_alt();
static bool verify_within_distance();
static void do_loiter_at_location();
static void do_take_picture();
static void log_picture();
static void log_picture_feedback( bool newshot );
static void exit_mission_callback();
static void update_commands(void);
static void mavlink_delay(uint32_t ms);
static uint32_t millis();
static uint32_t micros();
static void read_control_switch();
static uint8_t readSwitch(void);
static void reset_control_switch();
static void autotune_start(void);
static void autotune_restore(void);
static bool fly_inverted(void);
static void failsafe_short_on_event(enum failsafe_state fstype);
static void failsafe_long_on_event(enum failsafe_state fstype);
static void failsafe_short_off_event();
void low_battery_event(void);
void failsafe_check(void);
static NOINLINE void send_heartbeat(mavlink_channel_t chan);
static NOINLINE void send_attitude(mavlink_channel_t chan);
static NOINLINE void send_fence_status(mavlink_channel_t chan);
static NOINLINE void send_extended_status1(mavlink_channel_t chan);
static void NOINLINE send_location(mavlink_channel_t chan);
static void NOINLINE send_nav_controller_output(mavlink_channel_t chan);
void NOINLINE send_servo_out(mavlink_channel_t chan);
static void NOINLINE send_radio_out(mavlink_channel_t chan);
static void NOINLINE send_vfr_hud(mavlink_channel_t chan);
static void NOINLINE send_simstate(mavlink_channel_t chan);
static void NOINLINE send_hwstatus(mavlink_channel_t chan);
static void NOINLINE send_wind(mavlink_channel_t chan);
static void NOINLINE send_rangefinder(mavlink_channel_t chan);
static void NOINLINE send_current_waypoint(mavlink_channel_t chan);
static void NOINLINE send_statustext(mavlink_channel_t chan);
static bool telemetry_delayed(mavlink_channel_t chan);
static void mavlink_delay_cb();
static void gcs_send_message(enum ap_message id);
static void gcs_data_stream_send(void);
static void gcs_update(void);
static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str);
static void gcs_send_airspeed_calibration(const Vector3f &vg);
static void gcs_retry_deferred(void);
static uint8_t max_fencepoints(void);
static Vector2l get_fence_point_with_index(unsigned i);
static void set_fence_point_with_index(Vector2l &point, unsigned i);
static void geofence_load(void);
static bool geofence_present(void);
static void geofence_update_pwm_enabled_state();
static bool geofence_set_enabled(bool enable, GeofenceEnableReason r);
static bool geofence_enabled(void);
static bool geofence_set_floor_enabled(bool floor_enable);
static bool geofence_check_minalt(void);
static bool geofence_check_maxalt(void);
static void geofence_check(bool altitude_check_only);
static bool geofence_stickmixing(void);
static void geofence_send_status(mavlink_channel_t chan);
static bool geofence_breached(void);
static void geofence_check(bool altitude_check_only);
static bool geofence_stickmixing(void);
static bool geofence_enabled(void);
static bool geofence_present(void);
static bool geofence_set_enabled(bool enable, GeofenceEnableReason r);
static bool geofence_set_floor_enabled(bool floor_enable);
bool geofence_breached(void);
static bool verify_land();
static void disarm_if_autoland_complete();
static void setup_landing_glide_slope(void);
static bool jump_to_landing_sequence(void);
static float tecs_hgt_afe(void);
static bool print_log_menu(void);
static void do_erase_logs(void);
static void Log_Write_Attitude(void);
static void Log_Write_Performance();
static void Log_Write_Startup(uint8_t type);
static void Log_Write_EntireMission();
static void Log_Write_Control_Tuning();
static void Log_Write_TECS_Tuning(void);
static void Log_Write_Nav_Tuning();
static void Log_Write_Status();
static void Log_Write_Sonar();
static void Log_Write_Optflow();
static void Log_Write_Current();
static void Log_Arm_Disarm();
static void Log_Write_GPS(uint8_t instance);
static void Log_Write_IMU();
static void Log_Write_RC(void);
static void Log_Write_Baro(void);
static void Log_Write_Airspeed(void);
static void Log_Read(uint16_t log_num, int16_t start_page, int16_t end_page);
static void start_logging();
static void Log_Write_Startup(uint8_t type);
static void Log_Write_EntireMission();
static void Log_Write_Current();
static void Log_Write_Nav_Tuning();
static void Log_Write_TECS_Tuning();
static void Log_Write_Performance();
static void Log_Write_Attitude();
static void Log_Write_Control_Tuning();
static void Log_Write_GPS(uint8_t instance);
static void Log_Write_IMU();
static void Log_Write_RC();
static void Log_Write_Airspeed(void);
static void Log_Write_Baro(void);
static void Log_Write_Status();
static void Log_Write_Sonar();
static void Log_Write_Optflow();
static void Log_Arm_Disarm();
static void set_nav_controller(void);
static void loiter_angle_reset(void);
static void loiter_angle_update(void);
static void navigate();
static void calc_airspeed_errors();
static void calc_gndspeed_undershoot();
static void update_loiter();
static void update_cruise();
static void update_fbwb_speed_height(void);
static void setup_turn_angle(void);
static void parachute_check();
static void parachute_release();
static bool parachute_manual_release();
static void load_parameters(void);
static void set_control_channels(void);
static void init_rc_in();
static void init_rc_out();
static void rudder_arm_check();
static void read_radio();
static void control_failsafe(uint16_t pwm);
static void trim_control_surfaces();
static void trim_radio();
static bool rc_failsafe_active(void);
static void init_barometer(void);
static void init_rangefinder(void);
static void read_rangefinder(void);
static void read_airspeed(void);
static void zero_airspeed(bool in_startup);
static void read_battery(void);
void read_receiver_rssi(void);
static void report_radio();
static void report_ins();
static void report_compass();
static void print_radio_values();
static void print_done();
static void print_blanks(int16_t num);
static void print_divider(void);
static void zero_eeprom(void);
static void print_enabled(bool b);
static void print_accel_offsets_and_scaling(void);
static void print_gyro_offsets(void);
static void init_ardupilot();
static void startup_ground(void);
static enum FlightMode get_previous_mode();
static void set_mode(enum FlightMode mode);
static bool mavlink_set_mode(uint8_t mode);
static void exit_mode(enum FlightMode mode);
static void check_long_failsafe();
static void check_short_failsafe();
static void startup_INS_ground(void);
static void update_notify();
static void resetPerfData(void);
static void check_usb_mux(void);
static void print_comma(void);
static void servo_write(uint8_t ch, uint16_t pwm);
static bool should_log(uint32_t mask);
static void frsky_telemetry_send(void);
static uint8_t throttle_percentage(void);
static void change_arm_state(void);
static bool disarm_motors(void);
static bool auto_takeoff_check(void);
static void takeoff_calc_roll(void);
static void takeoff_calc_pitch(void);
static int8_t takeoff_tail_hold(void);
static void print_hit_enter();

#include <..\ArduPlane\arduplane.ino>
#include <..\ArduPlane\altitude.ino>
#include <..\ArduPlane\apm_config.h>
#include <..\ArduPlane\attitude.ino>
#include <..\ArduPlane\climb_rate.ino>
#include <..\ArduPlane\commands.ino>
#include <..\ArduPlane\commands_logic.ino>
#include <..\ArduPlane\commands_process.ino>
#include <..\ArduPlane\compat.h>
#include <..\ArduPlane\compat.ino>
#include <..\ArduPlane\config.h>
#include <..\ArduPlane\control_modes.ino>
#include <..\ArduPlane\defines.h>
#include <..\ArduPlane\events.ino>
#include <..\ArduPlane\failsafe.ino>
#include <..\ArduPlane\gcs_mavlink.ino>
#include <..\ArduPlane\geofence.ino>
#include <..\ArduPlane\landing.ino>
#include <..\ArduPlane\log.ino>
#include <..\ArduPlane\navigation.ino>
#include <..\ArduPlane\parachute.ino>
#include <..\ArduPlane\parameters.h>
#include <..\ArduPlane\parameters.ino>
#include <..\ArduPlane\px4_mixer.ino>
#include <..\ArduPlane\radio.ino>
#include <..\ArduPlane\sensors.ino>
#include <..\ArduPlane\setup.ino>
#include <..\ArduPlane\system.ino>
#include <..\ArduPlane\takeoff.ino>
#include <..\ArduPlane\test.ino>
