/**
 * 
 * About Rotrics DexArm
 * Official Website:           https://rotrics.com
 * Rotrics Arm Diagram:        https://manual.rotrics.com/product-profile/rotrics-arm-diagram
 * DexArm's Coordinate System: https://manual.rotrics.com/product-profile/robot-arm-coordinate-system
 *
 */

#include "../../inc/MarlinConfig.h"
#include "dexarm.h"
#include "../planner.h"

int calibration_position_sensor_value[3]; //Set by M889
int door_open_message_counter;
float x_axis_scaling_factor;			  //leveling
float y_axis_scaling_factor;			  //leveling
float front_module_offset = PEN_MODULE_OFFSET;

float start_angle_a = START_A_ANGLE;
float start_angle_b = START_B_ANGLE;
float start_angle_c = START_C_ANGLE;

float dexarm_offset = front_module_offset + OFFSET_ROT;

float delta_segments_per_second = SCARA_SEGMENTS_PER_SECOND;

bool laser_protection_enable_flag = false;
bool laser_door_open_flag = false;
bool laser_fan_flag = false;

bool position_init_flag = false; //DexArm will not move without position init.
bool current_position_flag = false;

float current_position_init[XYZE] = {START_X, START_Y + dexarm_offset, START_Z, 0.0};

bool INVERT_E0_DIR = true;
extern bool home_z_before_xy;

move_mode_t G0_MOVE_MODE = FAST_MODE;

#define CURRENT_POSITION_INIT                                            \
	if (current_position_flag)                                           \
	{                                                                    \
		current_position_flag = false;                                   \
		LOOP_XYZE(i) { current_position[i] = current_position_init[i]; } \
	}

void print_current_module_type()
{
	if (fabs(front_module_offset - PEN_MODULE_OFFSET) < 0.1)
	{
		MYSERIAL0.println("The current module is PEN");
	}else if (fabs(front_module_offset - LASER_MODULE_OFFSET) < 0.1)
	{
		MYSERIAL0.println("The current module is LASER");
	}else if (fabs(front_module_offset - PUMP_MODULE_OFFSET) < 0.1)
	{
		MYSERIAL0.println("The current module is PUMP");
	}else if (fabs(front_module_offset - _3D_MODULE_OFFSET) < 0.1)
	{
		MYSERIAL0.println("The current module is 3D");
	}else if (fabs(front_module_offset - CAMERA_MODULE_OFFSET) < 0.1)
	{
		MYSERIAL0.println("The current module is Camera");
	}else
	{
		MYSERIAL0.println("The current module maybe Custom Module!");
	}
}

void module_position_init()
{
	MYSERIAL0.print("OFFSET_FRONT IS ");
	MYSERIAL0.println(front_module_offset);

	dexarm_offset = (front_module_offset + OFFSET_ROT);
	MYSERIAL0.print("dexarm_offset IS ");
	MYSERIAL0.println(dexarm_offset);
	print_current_module_type();
	if (fabs(front_module_offset - LASER_MODULE_OFFSET) < 0.1)
	{
		laser_protection_enable_flag = true;
		//MYSERIAL0.println("The current module is LASER");
	}
}

int get_position_sensor_diff(int target_position, int current_position)
{
	int position_sensor_diff = 0;

	position_sensor_diff = current_position - target_position;
	if (position_sensor_diff > MAX_POSITION_SENSOR_DIFF)
	{
		position_sensor_diff -= MAX_POSITION_SENSOR_RANGE;
	}
	else if (position_sensor_diff < -1 * MAX_POSITION_SENSOR_DIFF)
	{
		position_sensor_diff += MAX_POSITION_SENSOR_RANGE;
	}

	return position_sensor_diff;
}

/**
 * Rotate angle of 3 Axis from current positon
 */
void rotate_angle_diff(abc_pos_t &angle_abc)
{
	xyz_pos_t pos = {0, 0, 0};
	abce_pos_t target;

	planner.set_machine_position_mm(pos);
	LOOP_XYZE(axis) { target[axis] = 0; }
	planner.buffer_segment(target, 30, active_extruder);

	LOOP_ABC(axis) { target[axis] = -1 * angle_abc[axis]; }
	planner.buffer_segment(target, 30, active_extruder);

	planner.set_machine_position_mm(pos);
	LOOP_ABCE(axis) { target[axis] = 0; }
	planner.buffer_segment(target, 30, active_extruder);
}

void get_current_encoder(){
	int current_position_sensor_value[3] = {0};
	LOOP_ABC(axis) { current_position_sensor_value[axis] = position_sensor_value_read(axis); }
	SERIAL_ECHOPAIR("M894 X", current_position_sensor_value[0], " Y", current_position_sensor_value[1], " Z", current_position_sensor_value[2]);
	SERIAL_ECHOPAIR("\r\n");
}

void get_angle_diff_from_position_sensor(abc_pos_t &angle_diff){
	int current_position_sensor_value[3] = {0};
	LOOP_ABC(axis) { current_position_sensor_value[axis] = position_sensor_value_read(axis); }
	LOOP_ABC(axis) {
		angle_diff[axis] = ((float)get_position_sensor_diff(calibration_position_sensor_value[axis], current_position_sensor_value[axis]))*360/4096;
	}
}

void get_current_position_from_position_sensor(xyz_pos_t &position){
	abce_pos_t angle_diff;
	get_angle_diff_from_position_sensor(angle_diff);
	angle_diff[E_AXIS] = current_position.e;
	//planner.set_machine_position_mm(target);
	forward_kinematics_DEXARM_position(angle_diff, position);
}

void set_current_position_from_position_sensor(){
	abce_pos_t angle_diff;
	get_angle_diff_from_position_sensor(angle_diff);
	angle_diff[E_AXIS] = current_position.e;
	planner.set_machine_position_mm(angle_diff);
	xyz_pos_t position;
	forward_kinematics_DEXARM_position(angle_diff, position);
	current_position = position;
}

void process_encoder(int x, int y, int z){
	planner.synchronize(); 
	int current_position_sensor_value[3] = {0};
	int target_position_sensor_value[3] = {0};
	int diff_position_sensor_value[3] = {0};
	LOOP_ABC(axis) { current_position_sensor_value[axis] = position_sensor_value_read(axis); }
	target_position_sensor_value[0] = x;
	target_position_sensor_value[1] = y;
	target_position_sensor_value[2] = z;
	LOOP_ABC(axis) {
		diff_position_sensor_value[axis] = get_position_sensor_diff(target_position_sensor_value[axis], current_position_sensor_value[axis]);
	}

	abc_pos_t angle_diff;
	LOOP_ABC(axis)
	{
		angle_diff[axis] = ((float)diff_position_sensor_value[axis]) * 360.0 / 4096.0;
	}
	rotate_angle_diff(angle_diff);

	int diff_angle[3] = {0};
	LOOP_ABC(axis) {
		diff_angle[axis] = get_position_sensor_diff(calibration_position_sensor_value[axis], target_position_sensor_value[axis])*360/4096;
	}
	/*
	SERIAL_ECHOLNPAIR(
		"diff_angle a=", diff_angle[A_AXIS],
		" b=", diff_angle[B_AXIS],
		" c=", diff_angle[C_AXIS]);
	//*/
	abce_pos_t target;
	target[A_AXIS] = diff_angle[A_AXIS];
	target[B_AXIS] = diff_angle[B_AXIS];
	target[C_AXIS] = diff_angle[C_AXIS];
	target[E_AXIS] = current_position.e;
	planner.set_machine_position_mm(target);
	forward_kinematics_DEXARM(target);

	xyze_pos_t pos = cartes;
	pos.e = planner.get_axis_position_mm(E_AXIS);
	current_position = pos;

	planner.synchronize();
	abc_float_t deg = {
		planner.get_axis_position_degrees(A_AXIS),
		planner.get_axis_position_degrees(B_AXIS),
		planner.get_axis_position_degrees(C_AXIS)};
	/*
	SERIAL_ECHOLNPAIR(
		"Current Angle a=", deg[A_AXIS],
		" b=", deg[B_AXIS],
		" c=", deg[C_AXIS]);
	//*/
	planner.synchronize();
}

static abce_pos_t angle_diff_ary[6];
static abce_pos_t diff_sum;

static void position_sensor_diff_error(const uint8_t position_sensor_state)
{

	SERIAL_ERROR_START();
	serialprintPGM(PSTR(STR_POSITION_SENSOR_DIFF_ERROR));
	SERIAL_ECHOPGM(STR_POSITION_SENSOR_AXIS);

	if ((position_sensor_state & (0x01 << 0)) > 0)
		SERIAL_ECHO("A ");
	if ((position_sensor_state & (0x01 << 1)) > 0)
		SERIAL_ECHO("B ");
	if ((position_sensor_state & (0x01 << 2)) > 0)
		SERIAL_ECHO("C ");

	SERIAL_EOL();
	kill();
}

void check_position_sensor_diff(const int sensor_diff[])
{
	uint8_t position_sensor_diff_state = 0;
	SERIAL_ECHO("Position Sensor Diff A:");
	SERIAL_ECHO(sensor_diff[0]);

	SERIAL_ECHO(" B: ");
	SERIAL_ECHO(sensor_diff[1]);

	SERIAL_ECHO(" C: ");
	SERIAL_ECHO(sensor_diff[2]);
	SERIAL_EOL();

	if (abs(sensor_diff[1]) > 1200)
	{
		position_sensor_diff_state |= 0x01 << 1;
	}

	if (abs(sensor_diff[2]) > 1300)
	{
		position_sensor_diff_state |= 0x01 << 2;
	}

	if (position_sensor_diff_state > 0)
	{
		position_sensor_diff_error(position_sensor_diff_state);
	}
}

/*
1:big arm
2:small arm
3:dipan xuanzhuan 
*/
void calibration_error_log(uint8_t num, uint8_t mode)
{

	/*
	char str[30];
	memset(str,0,sizeof(char)*30);
	if(mode)
	{
		sprintf(str,"Error M1111 failed : %d  ",num);
	}
	else 
	{
		sprintf(str,"Error M1112 failed : %d  ",num);
	}
		
	MYSERIAL0.println(str);
*/
	SERIAL_ERROR_START();
	if (mode)
	{
		serialprintPGM(PSTR(STR_POSITION_M1111_ERROR));
	}
	else
	{
		serialprintPGM(PSTR(STR_POSITION_M1112_ERROR));
	}
	SERIAL_ECHOPGM(STR_POSITION_SENSOR_AXIS);

	if (num == 0)
	{
		SERIAL_ECHO("A ");
	}
	else if (num == 1)
	{
		SERIAL_ECHO("B ");
	}
	else if (num == 2)
	{
		SERIAL_ECHO("C ");
	}

	SERIAL_EOL();
}


void analysis_error(abce_pos_t angle_diff[], uint8_t count, uint8_t mode)
{
	if (count > 7)		
	{ 
		if (mode){
			SERIAL_ECHO("M1111 Error");
			SERIAL_EOL();	
		}
		else{
			SERIAL_ECHO("M1112 Error");
			SERIAL_EOL();	
		}
	}
	else if (count > 3){
		for(int i = 2; i < count;i++)
		{
			diff_sum[0] = diff_sum[0] + angle_diff[i][0];
			diff_sum[1] = diff_sum[1] + angle_diff[i][1];
			diff_sum[2] = diff_sum[2] + angle_diff[i][2];
		}
		for(int i = 0; i < 3;i++)
		{
			if (abs(diff_sum[i]) > 3.0f)
				calibration_error_log(i,mode);
		}
	}
	else if (count > 1)
	{
		for(int i = 0; i < 3;i++)
		{
			if (abs(angle_diff[count][i]) > 1.0f)
				calibration_error_log(i,mode);
		}
	}
	
	// SERIAL_ECHOLNPAIR("1a = ",angle_diff_ary[1][0],"  1b = ", angle_diff_ary[1][1] ,"  1c = ", angle_diff_ary[1][2]);
	// SERIAL_ECHOLNPAIR("2a = ",angle_diff_ary[2][0],"  2b = ", angle_diff_ary[2][1] ,"  2c = ", angle_diff_ary[2][2]);
	// SERIAL_ECHOLNPAIR("3a = ",angle_diff_ary[3][0],"  3b = ", angle_diff_ary[3][1] ,"  3c = ", angle_diff_ary[3][2]);
	// SERIAL_ECHOLNPAIR("4a = ",angle_diff_ary[4][0],"  4b = ", angle_diff_ary[4][1] ,"  3c = ", angle_diff_ary[4][2]);
	memset(angle_diff_ary,0,sizeof(abce_pos_t)*6);
		
}
/**
 * go to position M1111
 */
int position_M1111()
{
	int dif[3] = {0};
	int current_position_sensor_value[3];
	static uint8_t fix_num = 0;
	
	home_z_before_xy = false;

	while (1)
	{
		LOOP_ABC(axis) { current_position_sensor_value[axis] = position_sensor_value_read(axis);
		}
		check_position_sensor(current_position_sensor_value);
		LOOP_ABC(axis)
		{
			dif[axis] = get_position_sensor_diff(calibration_position_sensor_value[axis], current_position_sensor_value[axis]);
		}

		check_position_sensor_diff(dif);

		if ((abs(dif[0]) LESS_THAN_VAL) && (abs(dif[1]) LESS_THAN_VAL) && (abs(dif[2]) LESS_THAN_VAL))
		{
			start_angle_a = START_A_ANGLE;
			start_angle_b = START_B_ANGLE;
			start_angle_c = START_C_ANGLE;
			current_position[0] = START_X;
			current_position[1] = START_Y + dexarm_offset;
			current_position[2] = START_Z;
			position_init_flag = true;

			abce_pos_t target;
			target[A_AXIS] = 0;
			target[B_AXIS] = 0;
			target[C_AXIS] = 0;
			target[E_AXIS] = current_position.e;
			LOOP_XYZE(i) { current_position[i] = current_position_init[i];}
			sync_plan_position();
			planner.set_machine_position_mm(target);

			planner.synchronize();
			abc_float_t deg = {
				planner.get_axis_position_degrees(A_AXIS),
				planner.get_axis_position_degrees(B_AXIS),
				planner.get_axis_position_degrees(C_AXIS)};
			fix_num = 0;
			/*SERIAL_ECHOLNPAIR(
				"Current Angle a=", deg[A_AXIS],
				" b=", deg[B_AXIS],
				" c=", deg[C_AXIS]);
			//*/

			return 1;
		}
		else
		{
			abc_pos_t angle_diff;
			float diff_sum = 0.0;
			uint8_t ret_flag = 0;
			fix_num++;

			LOOP_ABC(axis)
			{
				angle_diff[axis] = ((float)dif[axis]) * 360.0 / (MAX_POSITION_SENSOR_RANGE*1.0);
				/*
				if(axis == 0){
					MYSERIAL0.print("A axis");
				}else if(axis == 1){
					MYSERIAL0.print("B axis");
				}else if(axis == 2){
					MYSERIAL0.print("C axis");
				}
				MYSERIAL0.print(axis);	
				MYSERIAL0.print("angle diff is : ");
				MYSERIAL0.println(angle_diff[axis]);
				*/
				diff_sum = diff_sum + abs(angle_diff[axis]);
				// save data
				angle_diff_ary[fix_num][axis] = angle_diff[axis];
				if(angle_diff[axis]>(MAX_POSITION_SENSOR_RANGE * 1.0f))
				{
					//disable_all_steppers();
					calibration_error_log(axis,1);
					kill();
					ret_flag = 1;
				}
			}
			if(ret_flag)
			{
				return 0;
			}
			
			// SERIAL_ECHOLNPAIR("fix_num = ",fix_num,"  Current Angle Diff Sum = ", diff_sum);
			
			if(fix_num > 7)
			{
				//disable_all_steppers();
				analysis_error(angle_diff_ary,fix_num,1);
				kill();
				fix_num = 0;
				return 0;
			}
			else if (fix_num > 3 && diff_sum > 3.0f)		
			{
				//disable_all_steppers();
				analysis_error(angle_diff_ary,fix_num,1);
				kill();
				fix_num = 0;				
				return 0;
			}
			else if (fix_num == 2 && diff_sum > 2.0f)
			{
				//disable_all_steppers();
				analysis_error(angle_diff_ary,fix_num,1);
				kill();
				fix_num = 0;				
				return 0;
			}


			rotate_angle_diff(angle_diff);
			planner.synchronize();			
		}
	}
}

void forward_kinematics_DEXARM(abc_pos_t &angle){
	float l1, x, y, z;
	l1 = 150 * cos((START_B_ANGLE - angle.b) / MATH_TRANS) + 150 * sin((START_C_ANGLE + angle.c) / MATH_TRANS) + dexarm_offset;
	z = 150 * sin((START_B_ANGLE - angle.b) / MATH_TRANS) - 150 * cos((START_C_ANGLE + angle.c) / MATH_TRANS);
	y = l1 * cos(angle.a / MATH_TRANS);
	x = l1 * sin(angle.a / MATH_TRANS);
	/*
	SERIAL_ECHOLNPAIR(
		" x=", x,
		" y=", y,
		" z=", z, );
	//*/
	cartes.set(x, y, z);
}

void forward_kinematics_DEXARM_position(abc_pos_t &angle, xyz_pos_t &position){
	float l1;
	l1 = 150 * cos((START_B_ANGLE - angle.b) / MATH_TRANS) + 150 * sin((START_C_ANGLE + angle.c) / MATH_TRANS) + dexarm_offset;
	position.z = 150 * sin((START_B_ANGLE - angle.b) / MATH_TRANS) - 150 * cos((START_C_ANGLE + angle.c) / MATH_TRANS);
	position.y = l1 * cos(angle.a / MATH_TRANS);
	position.x = l1 * sin(angle.a / MATH_TRANS);
}

char inverse_kinematics_dexarm_xy_legace(const xyz_pos_t &position, abc_pos_t &angle)
{
	float x = position.x;
	float y = position.y;
	float z = position.z;

	//apply_leveling
	z += (x_axis_scaling_factor * x + y_axis_scaling_factor * (y - 300));

	float tmps = sqrt(x * x + y * y);

	x -= x * dexarm_offset / tmps;
	y -= y * dexarm_offset / tmps;

	float angleRot = 0;
	float angleLeft = 0;
	float angleRight = 0;

	if (x < 0.01)
	{
		x = 0.01;
	}

	// Calculate value of theta 1: the rotation angle
	if (y == 0)
	{
		angleRot = 90;
	}
	else
	{
		if (y < 0)
		{
			angleRot = -atan(x / y) * MATH_TRANS;
		}
		if (y > 0)
		{
			angleRot = 180 - atan(x / y) * MATH_TRANS;
		}
	}

	float s = sqrt(x * x + y * y);
	float d = sqrt(x * x + y * y + z * z);

	float thb1 = (acos(d / 2 / 150)) * MATH_TRANS;
	float thb2 = (atan(z / s)) * MATH_TRANS;

	angleLeft = thb1 + thb2;
	angleRight = 90.0 + (thb2 - thb1);

	angle[0] = angleRot;
	angle[1] = angleLeft;
	angle[2] = angleRight;

	return 0;
}

char inverse_kinematics_dexarm(const xyz_pos_t &position, abc_pos_t &angle)
{
	float x = position.x;
	float y = position.y;
	float z = position.z;

	//apply_leveling
	z += (x_axis_scaling_factor * x + y_axis_scaling_factor * (y - 300));

	float tmps = sqrt(x * x + y * y);

	x -= x * dexarm_offset / tmps;
	y -= y * dexarm_offset / tmps;

	float angleRot = 0;
	float angleLeft = 0;
	float angleRight = 0;

	if (y < 0.01)
	{
		y = 0.01;
	}

	// Calculate value of theta 1: the rotation angle
	if (x == 0)
	{
		angleRot = 90;
	}
	else
	{
		if (x < 0)
		{
			angleRot = -atan(y / x) * MATH_TRANS;
		}
		if (x > 0)
		{
			angleRot = 180 - atan(y / x) * MATH_TRANS;
		}
	}

	float s = sqrt(x * x + y * y);
	float d = sqrt(x * x + y * y + z * z);

	float thb1 = (acos(d / 2 / 150)) * MATH_TRANS;
	float thb2 = (atan(z / s)) * MATH_TRANS;

	angleLeft = thb1 + thb2;
	angleRight = 90.0 + (thb2 - thb1);

	angle[0] = angleRot;
	angle[1] = angleLeft;
	angle[2] = angleRight;
	return 0;
}

/**
 * go to position M1112
 */
int m1112_position(xyz_pos_t &position)
{
	int current_position_sensor_value[3] = {0};
	int dif[3] = {0};
	abc_pos_t target_angle;
	abc_pos_t diff_target_calibration_angle;

	int diff_position_sensor_value[3];
	int target_position_sensor_value[3] = {0};
	static uint8_t fix_num = 0;

	home_z_before_xy = false;
	if (inverse_kinematics_dexarm(position, target_angle) == 0)
	{
		diff_target_calibration_angle[A_AXIS] = target_angle[A_AXIS] - START_A_ANGLE;
		//diff_target_calibration_angle[0] *= -1;
		diff_target_calibration_angle[B_AXIS] = START_B_ANGLE - target_angle[B_AXIS];
		diff_target_calibration_angle[C_AXIS] = target_angle[C_AXIS] - START_C_ANGLE;
	}

	LOOP_ABC(axis) { diff_position_sensor_value[axis] = (int)(diff_target_calibration_angle[axis] * MAX_POSITION_SENSOR_RANGE / 360); }
	LOOP_ABC(axis)
	{
		target_position_sensor_value[axis] = calibration_position_sensor_value[axis] + diff_position_sensor_value[axis];
		if (target_position_sensor_value[axis] > MAX_POSITION_SENSOR_RANGE)
		{
			target_position_sensor_value[axis] -= MAX_POSITION_SENSOR_RANGE;
		}
		else if (target_position_sensor_value[axis] < 0)
		{
			target_position_sensor_value[axis] += MAX_POSITION_SENSOR_RANGE;
		}
	}
	while (1)
	{
		//_delay_ms(500);
		LOOP_ABC(axis) { current_position_sensor_value[axis] = position_sensor_value_read(axis); }
		check_position_sensor(current_position_sensor_value);
		LOOP_ABC(axis)
		{
			dif[axis] = get_position_sensor_diff(target_position_sensor_value[axis], current_position_sensor_value[axis]);
		}

		check_position_sensor_diff(dif);

		if ((abs(dif[0]) LESS_THAN_VAL) && (abs(dif[1]) LESS_THAN_VAL) && (abs(dif[2]) LESS_THAN_VAL))
		{
			start_angle_a = target_angle[0];
			start_angle_b = target_angle[1];
			start_angle_c = target_angle[2];
			LOOP_XYZ(axis) { current_position[axis] = position[axis]; }
			position_init_flag = true;

			abce_pos_t target;
			target[A_AXIS] = diff_target_calibration_angle[A_AXIS];
			target[B_AXIS] = diff_target_calibration_angle[B_AXIS];
			target[C_AXIS] = diff_target_calibration_angle[C_AXIS];
			target[E_AXIS] = current_position.e;
			sync_plan_position();
			planner.set_machine_position_mm(target);

			planner.synchronize();
			abc_float_t deg = {
				planner.get_axis_position_degrees(A_AXIS),
				planner.get_axis_position_degrees(B_AXIS),
				planner.get_axis_position_degrees(C_AXIS)};
			fix_num = 0;
			/*
			SERIAL_ECHOLNPAIR(
				"Current Angle a=", deg[A_AXIS],
				" b=", deg[B_AXIS],
				" c=", deg[C_AXIS]);
			//*/
			return 1;
		}
		else
		{
			abc_pos_t angle_diff;
			float diff_sum = 0.0;
			uint8_t ret_flag = 0;
			fix_num++;
			LOOP_ABC(axis)
			{
				angle_diff[axis] = ((float)dif[axis]) * 360.0 / (MAX_POSITION_SENSOR_RANGE*1.0);	
				//MYSERIAL0.print("angle diff is : ");
				//MYSERIAL0.println(angle_diff[axis]);
				diff_sum = diff_sum + abs(angle_diff[axis]);
				// save data
				angle_diff_ary[fix_num][axis] = angle_diff[axis];
				if(angle_diff[axis]>(MAX_POSITION_SENSOR_RANGE * 1.0f))
				{
					//disable_all_steppers();
					calibration_error_log(axis,0);
					kill();
					ret_flag = 1;
				}								
			}
			if(ret_flag)
			{
				return 0;
			}
			
			// SERIAL_ECHOLNPAIR("fix_num = ",fix_num,"  Current Angle Diff Sum = ", diff_sum);
			
			if(fix_num > 7)
			{
				//disable_all_steppers();
				analysis_error(angle_diff_ary,fix_num,0);
				kill();
				fix_num = 0;
				return 0;
			}
			else if (fix_num > 3 && diff_sum > 3.0f)		
			{
				//disable_all_steppers();
				analysis_error(angle_diff_ary,fix_num,0);
				kill();
				fix_num = 0;				
				return 0;
			}
			else if (fix_num == 2 && diff_sum > 2.0f)
			{
				//disable_all_steppers();
				analysis_error(angle_diff_ary,fix_num,0);
				kill();
				fix_num = 0;				
				return 0;
			}
			rotate_angle_diff(angle_diff);
			planner.synchronize();		

		}
	}
}

int m1113_position(xyz_pos_t &position)
{
	position_M1111();
	abc_pos_t target_angle;
	abc_pos_t diff_target_calibration_angle;
	LOOP_XYZ(axis) { target_angle[axis] = 1; }
	if (inverse_kinematics_dexarm(position, target_angle) == 0)
	{
		diff_target_calibration_angle[A_AXIS] = START_A_ANGLE - target_angle[A_AXIS];
		diff_target_calibration_angle[B_AXIS] = target_angle[B_AXIS] - START_B_ANGLE;
		diff_target_calibration_angle[C_AXIS] = START_C_ANGLE - target_angle[C_AXIS];
	}
	rotate_angle_diff(diff_target_calibration_angle);
	start_angle_a = target_angle[0];
	start_angle_b = target_angle[1];
	start_angle_c = target_angle[2];
	LOOP_XYZ(axis) { current_position[axis] = position[axis]; }
	position_init_flag = true;

	abce_pos_t target;
	target[A_AXIS] = diff_target_calibration_angle[A_AXIS];
	target[B_AXIS] = diff_target_calibration_angle[B_AXIS];
	target[C_AXIS] = diff_target_calibration_angle[C_AXIS];
	target[E_AXIS] = current_position.e;
	planner.set_machine_position_mm(target);

	planner.synchronize();
	abc_float_t deg = {
		planner.get_axis_position_degrees(A_AXIS),
		planner.get_axis_position_degrees(B_AXIS),
		planner.get_axis_position_degrees(C_AXIS)};
	/*	
	SERIAL_ECHOLNPAIR(
		"Current Angle a=", deg[A_AXIS],
		" b=", deg[B_AXIS],
		" c=", deg[C_AXIS]);
	//*/
	return 1;
}

void inverse_kinematics(const xyz_pos_t &raw)
{
	if (!position_init_flag)
	{
		MYSERIAL0.println("Send M1112 or click HOME to initialize DexArm first before any motion.");
		return;
	}
	abc_pos_t diff_angle;
	//if (!dexarm_position_is_reachable(raw))
	//{
	//LOOP_XYZE(i) { destination[i] = current_position[i]; }
	//return;
	//}
	CURRENT_POSITION_INIT;

	/*
	SERIAL_ECHOLNPAIR(
		"target x=", raw[X_AXIS],
		" y=", raw[Y_AXIS],
		" z=", raw[Z_AXIS]);
	//*/
	if (inverse_kinematics_dexarm(raw, diff_angle) == 0)
	{
		/*
		SERIAL_ECHOLNPAIR(
			"Inverse Angle a=", diff_angle[A_AXIS],
			" b=", diff_angle[B_AXIS],
			" c=", diff_angle[C_AXIS]);
		//*/
		//diff_angle[A_AXIS] -= start_angle_a;
		//diff_angle[B_AXIS] = start_angle_b - diff_angle[B_AXIS];
		//diff_angle[C_AXIS] -= start_angle_c;
		diff_angle[A_AXIS] -= START_A_ANGLE;
		diff_angle[B_AXIS] = START_B_ANGLE - diff_angle[B_AXIS];
		diff_angle[C_AXIS] -= START_C_ANGLE;
	}
	delta.set(diff_angle[0], diff_angle[1], diff_angle[2]);
}

void update_dexarm_offset(void)
{
	dexarm_offset = (front_module_offset + OFFSET_ROT);
}

bool dexarm_position_is_reachable(const xyz_pos_t &position)
{
	float x = position.x;
	float y = position.y;
	float z = position.z;

	int tmp_z = (int(z + 0.5) + abs(ZMINARM)) / 2 + 0.5;
	float tmps = sqrt(x * x + y * y) - dexarm_offset;

	if (z > ZMAXARM || z < ZMINARM)
	{
		MYSERIAL0.println("Z beyond limit.....");
		return false;
	}

	if (y < 0)
	{
		MYSERIAL0.println("Y beyond limit.....");
		return false;
	}

	if (tmps < ARMLIMIT[tmp_z * 2 + 1] + 1 || tmps > ARMLIMIT[tmp_z * 2] - 1)
	{
		MYSERIAL0.println("XY beyond limit.....");
		return false;
	}
	return true;
}

void dexarm_report_positions() {
  SERIAL_ECHOLNPAIR("DEXARM Theta A:", planner.get_axis_position_degrees(A_AXIS), "  Theta B:", planner.get_axis_position_degrees(B_AXIS), "  Theta C:", planner.get_axis_position_degrees(C_AXIS));
  SERIAL_EOL();
}