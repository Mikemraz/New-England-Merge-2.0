/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*  Dummy version that simply sends back VISSIM's suggestions to VISSIM.    */
/*                                                                          */
/*  Version of 2017-03-21                                   Lukas Kautzsch  */
/*==========================================================================*/
#define NOMINMAX
#include "DriverModel.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <algorithm>
using namespace std;
/*==========================================================================*/
long meter_link [] = {21,22,30,52,53,61,83,84,92};
long current_link = 0;
long current_lane = 0;
long vehicle_id = 111;
double  desired_lane_angle   = 0.0;
long    active_lane_change   = 0;
long    rel_target_lane      = 0;
double  desired_speed     = 0.0;
long    turning_indicator    = 0.0;
long    vehicle_color        = RGB(0,0,0);

double current_speed = 0.0;
double max_acc = 0.0;

double acc_of_lead_vehicle = 0.0;
double net_distance_to_lead_vehicle = 99.0;
double current_speed_of_lead_vehicle = 0.0;

double distance_to_nveh_0_1 = 999.0;
double distance_to_nveh_l1_1 = 999.0;
double distance_to_nveh_r1_1 = 999.0;
double distance_to_nveh_l2_1 = 999.0;
double distance_to_nveh_r2_1 = 999.0;

double net_distance_to_nveh_0_1 = 999.0;
double net_distance_to_nveh_l1_1 = 999.0;
double net_distance_to_nveh_r1_1 = 999.0;
double net_distance_to_nveh_l2_1 = 999.0;
double net_distance_to_nveh_r2_1 = 999.0;

double speed_difference_to_nveh_0_1 = 0.0;
double speed_difference_to_nveh_l1_1 = 0.0;
double speed_difference_to_nveh_r1_1 = 0.0;
double speed_difference_to_nveh_l2_1 = 0.0;
double speed_difference_to_nveh_r2_1 = 0.0;

double speed_of_nveh_0_1 = 0.0;
double speed_of_nveh_l1_1 = 0.0;
double speed_of_nveh_r1_1 = 0.0;
double speed_of_nveh_l2_1 = 0.0;
double speed_of_nveh_r2_1 = 0.0;


double acc_of_nveh_0_1 = 0.0;
double acc_of_nveh_l1_1 = 0.0;
double acc_of_nveh_r1_1 = 0.0;
double acc_of_nveh_l2_1 = 0.0;
double acc_of_nveh_r2_1 = 0.0;

double length_of_nveh_0_1 = 0.0;
double length_of_nveh_l1_1 = 0.0;
double length_of_nveh_r1_1 = 0.0;
double length_of_nveh_l2_1 = 0.0;
double length_of_nveh_r2_1 = 0.0;


long intac_state = 0;
long intac_target_type = 0;

double headway_for_CACC = 1;
double coef_acc = 1.0;
double coef_speed = 0.58;
double coef_distance = 0.1;

double acc_next = 0.0;

int mode = 0;

// function declaration
int get_driving_mode_at_meter_zone();
double get_acc_for_free_driving();
double get_acc_for_GCACC();
double get_acc_for_meter_zone();
void OutputTrafficStateToFile();
double ResetDistanceToLeadCar(double);
double GetReferenceDistance();
bool CheckEmergency();
double GetAccOfEmergencybrake();
bool UseInternalModel();
long determine_lead_vehicle_for_GCACC();
void set_parameters_for_lead_vehicle();
void set_net_gap_to_nveh();
void set_nveh_speed();
bool is_now_in_meter_zone();
double find_min_net_distance();
ofstream myfile("traffic_state.txt", std::fstream::trunc);
/*==========================================================================*/

BOOL APIENTRY DllMain (HANDLE  hModule,
                       DWORD   ul_reason_for_call,
                       LPVOID  lpReserved)
{
  switch (ul_reason_for_call) {
      case DLL_PROCESS_ATTACH:
      case DLL_THREAD_ATTACH:
      case DLL_THREAD_DETACH:
      case DLL_PROCESS_DETACH:
         break;
  }
  return TRUE;
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelSetValue(long   type,
	long   index1,
	long   index2,
	long   long_value,
	double double_value,
	char   *string_value)
{
	/* Sets the value of a data object of type <type>, selected by <index1> */
	/* and possibly <index2>, to <long_value>, <double_value> or            */
	/* <*string_value> (object and value selection depending on <type>).    */
	/* Return value is 1 on success, otherwise 0.                           */

	switch (type) {
	case DRIVER_DATA_PATH:
	case DRIVER_DATA_TIMESTEP:
	case DRIVER_DATA_TIME:
	case DRIVER_DATA_VEH_ID:
		vehicle_id = long_value;
		return 1;
	case DRIVER_DATA_VEH_LANE:
		current_lane = long_value;
		return 1;
	case DRIVER_DATA_VEH_ODOMETER:
	case DRIVER_DATA_VEH_LANE_ANGLE:
	case DRIVER_DATA_VEH_LATERAL_POSITION:
	case DRIVER_DATA_VEH_VELOCITY:
		current_speed = double_value;
		return 1;
	case DRIVER_DATA_VEH_ACCELERATION:
	case DRIVER_DATA_VEH_LENGTH:
	case DRIVER_DATA_VEH_WIDTH:
	case DRIVER_DATA_VEH_WEIGHT:
	case DRIVER_DATA_VEH_MAX_ACCELERATION:
		max_acc = double_value;
		return 1;
	case DRIVER_DATA_VEH_TURNING_INDICATOR:
		turning_indicator = long_value;
		return 1;
	case DRIVER_DATA_VEH_CATEGORY:
	case DRIVER_DATA_VEH_PREFERRED_REL_LANE:
	case DRIVER_DATA_VEH_USE_PREFERRED_LANE:
		return 1;
	case DRIVER_DATA_VEH_DESIRED_VELOCITY:
		desired_speed = double_value;
		return 1;
	case DRIVER_DATA_VEH_X_COORDINATE:
	case DRIVER_DATA_VEH_Y_COORDINATE:
	case DRIVER_DATA_VEH_Z_COORDINATE:
	case DRIVER_DATA_VEH_REAR_X_COORDINATE:
	case DRIVER_DATA_VEH_REAR_Y_COORDINATE:
	case DRIVER_DATA_VEH_REAR_Z_COORDINATE:
	case DRIVER_DATA_VEH_TYPE:
		return 1;
	case DRIVER_DATA_VEH_COLOR:
		vehicle_color = long_value;
		return 1;
	case DRIVER_DATA_VEH_CURRENT_LINK:
		current_link = long_value;
		return 1; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS messages) */
				  /* Must return 1 if these messages are to be sent from VISSIM!         */
	case DRIVER_DATA_VEH_NEXT_LINKS:
	case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE:
	case DRIVER_DATA_VEH_REL_TARGET_LANE:
	case DRIVER_DATA_VEH_INTAC_STATE:
		intac_state = long_value;
		return 1;
	case DRIVER_DATA_VEH_INTAC_TARGET_TYPE:
		intac_target_type = long_value;
		return 1;
	case DRIVER_DATA_VEH_INTAC_TARGET_ID:
	case DRIVER_DATA_VEH_INTAC_HEADWAY:
	case DRIVER_DATA_NVEH_ID:
	case DRIVER_DATA_NVEH_LANE_ANGLE:
	case DRIVER_DATA_NVEH_LATERAL_POSITION:
	case DRIVER_DATA_NVEH_DISTANCE: {
		// improve: get info of more downstream vehicles
		if ((index1 == 0)&(index2 == 1)) {
			distance_to_nveh_0_1 = double_value;
			distance_to_nveh_0_1 = ResetDistanceToLeadCar(distance_to_nveh_0_1);
			return 1;
		}
		if ((index1 == -1)&(index2 == 1)) {
			//the nveh on the left side of subject vehicle
			distance_to_nveh_l1_1 = double_value;
			distance_to_nveh_l1_1 = ResetDistanceToLeadCar(distance_to_nveh_l1_1);
			return 1;
		}
		if ((index1 == 1)&(index2 == 1)) {
			//the nveh on the right side of subject vehicle
			distance_to_nveh_r1_1 = double_value;
			distance_to_nveh_r1_1 = ResetDistanceToLeadCar(distance_to_nveh_r1_1);
			return 1;
		}
		if ((index1 ==-2)&(index2 == 1)) {
			distance_to_nveh_l2_1 = double_value;
			distance_to_nveh_l2_1 = ResetDistanceToLeadCar(distance_to_nveh_l2_1);
			return 1;
		}
		if ((index1 == 2)&(index2 == 1)) {
			distance_to_nveh_r2_1 = double_value;
			distance_to_nveh_r2_1 = ResetDistanceToLeadCar(distance_to_nveh_r2_1);
			return 1;
		}
	}

	case DRIVER_DATA_NVEH_REL_VELOCITY: {
		// improve: get info of more downstream vehicles
		if ((index1 == 0)&(index2 == 1)) {
			speed_difference_to_nveh_0_1 = double_value;
			return 1;
		}
		if ((index1 == -1)&(index2 == 1)) {
			speed_difference_to_nveh_l1_1 = double_value;
			return 1;
		}
		if ((index1 == 1)&(index2 == 1)) {
			speed_difference_to_nveh_r1_1 = double_value;
			return 1;
		}
		if ((index1 == -2)&(index2 == 1)) {
			speed_difference_to_nveh_l2_1 = double_value;
			return 1;
		}
		if ((index1 == 2)&(index2 == 1)) {
			speed_difference_to_nveh_r2_1 = double_value;
			return 1;
		}

	}

	case DRIVER_DATA_NVEH_ACCELERATION: {
		// improve: get info of more downstream vehicles
		if ((index1 == 0)&(index2 == 1)) {
			acc_of_nveh_0_1 = double_value;
			return 1;
		}
		if ((index1 == -1)&(index2 == 1)) {
			acc_of_nveh_l1_1 = double_value;
			return 1;
		}
		if ((index1 == 1)&(index2 == 1)) {
			acc_of_nveh_r1_1 = double_value;
			return 1;
		}
		if ((index1 == -2)&(index2 == 1)) {
			acc_of_nveh_l2_1 = double_value;
			return 1;
		}
		if ((index1 == 2)&(index2 == 1)) {
			acc_of_nveh_r2_1 = double_value;
			return 1;
		}

	}
	case DRIVER_DATA_NVEH_LENGTH: {
		//improve: get info of more downstream vehicles
		if ((index1 == 0)&(index2 == 1)) {
			length_of_nveh_0_1 = double_value;
			return 1;
		}
		if ((index1 == -1)&(index2 == 1)) {
			length_of_nveh_l1_1 = double_value;
			return 1;
		}
		if ((index1 == 1)&(index2 == 1)) {
			length_of_nveh_r1_1 = double_value;
			return 1;
		}
		if ((index1 == -2)&(index2 == 1)) {
			length_of_nveh_l2_1 = double_value;
			return 1;
		}
		if ((index1 == 2)&(index2 == 1)) {
			length_of_nveh_r2_1 = double_value;
			return 1;
		}

	}
	case DRIVER_DATA_NVEH_WIDTH:
	case DRIVER_DATA_NVEH_WEIGHT:
	case DRIVER_DATA_NVEH_TURNING_INDICATOR:
	case DRIVER_DATA_NVEH_CATEGORY:
	case DRIVER_DATA_NVEH_LANE_CHANGE:
	case DRIVER_DATA_NVEH_TYPE:
	case DRIVER_DATA_NO_OF_LANES:
	case DRIVER_DATA_LANE_WIDTH:
	case DRIVER_DATA_LANE_END_DISTANCE:
	case DRIVER_DATA_RADIUS:
	case DRIVER_DATA_MIN_RADIUS:
	case DRIVER_DATA_DIST_TO_MIN_RADIUS:
	case DRIVER_DATA_SLOPE:
	case DRIVER_DATA_SLOPE_AHEAD:
	case DRIVER_DATA_SIGNAL_DISTANCE:
	case DRIVER_DATA_SIGNAL_STATE:
	case DRIVER_DATA_SIGNAL_STATE_START:
	case DRIVER_DATA_SPEED_LIMIT_DISTANCE:
	case DRIVER_DATA_SPEED_LIMIT_VALUE:
		return 1;
	case DRIVER_DATA_DESIRED_ACCELERATION:
	case DRIVER_DATA_DESIRED_LANE_ANGLE:
		desired_lane_angle = double_value;
		return 1;
	case DRIVER_DATA_ACTIVE_LANE_CHANGE:
		active_lane_change = long_value;
		return 1;
	case DRIVER_DATA_REL_TARGET_LANE:
		rel_target_lane = long_value;
		return 1;
	default:
		return 0;
	}
	}

/*--------------------------------------------------------------------------*/

  DRIVERMODEL_API  int  DriverModelGetValue(long   type,
	  long   index1,
	  long   index2,
	  long   *long_value,
	  double *double_value,
	  char   **string_value)
  {
	  /* Gets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, and writes that value to <*double_value>,     */
  /* <*float_value> or <**string_value> (object and value selection       */
  /* depending on <type>).                                                */
  /* Return value is 1 on success, otherwise 0.                           */

	  switch (type) {
	  case DRIVER_DATA_STATUS:
		  *long_value = 0;
		  return 1;
	  case DRIVER_DATA_VEH_TURNING_INDICATOR:
		  *long_value = turning_indicator;
		  return 1;
	  case DRIVER_DATA_VEH_DESIRED_VELOCITY:
		  *double_value = desired_speed;
		  return 1;
	  case DRIVER_DATA_VEH_COLOR:
		  *long_value = vehicle_color;
		  return 1;
	  case DRIVER_DATA_WANTS_SUGGESTION:
		  *long_value = 1;
		  return 1;
	  case DRIVER_DATA_DESIRED_ACCELERATION: {
		  if (UseInternalModel() == FALSE) {
			  acc_next = get_acc_for_meter_zone();
			  *double_value = acc_next;
		  }
		  return 1;
	  }
	  case DRIVER_DATA_DESIRED_LANE_ANGLE:
		  *double_value = desired_lane_angle;
		  return 1;
	  case DRIVER_DATA_ACTIVE_LANE_CHANGE:
		  *long_value = active_lane_change;
		  return 1;
	  case DRIVER_DATA_REL_TARGET_LANE:
		  *long_value = rel_target_lane;
		  return 1;
	  case DRIVER_DATA_SIMPLE_LANECHANGE:
		  *long_value = 1;
		  return 1;
	  case DRIVER_DATA_USE_INTERNAL_MODEL: {
		  if (UseInternalModel()) {
			  *long_value = 1;
		  }
		  else {
			  *long_value = 0;
		  } /* must be set to 0 if external model is to be applied */
		  return 1;
	  }
	  default:
		  return 0;
	  }
  }


/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelExecuteCommand (long number)
{
  /* Executes the command <number> if that is available in the driver */
  /* module. Return value is 1 on success, otherwise 0.               */

  switch (number) {
    case DRIVER_COMMAND_INIT :
      return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
      return 1;
    case DRIVER_COMMAND_KILL_DRIVER :
      return 1;
    case DRIVER_COMMAND_MOVE_DRIVER :
      return 1;
    default :
      return 0;
  }
}
/*==========================================================================*/

int get_driving_mode_at_meter_zone() {
	switch (net_distance_to_lead_vehicle<100) {
	case 1: 
		return 1; 
		break;
	case 0: 
		return 2;
	}
}

double get_acc_for_free_driving() {
	double acc_local = 0.0;
	if (current_speed < desired_speed) {
		acc_local = min(max_acc, (desired_speed - current_speed) / 2);
	}
	else {
		acc_local = max(-max_acc, (desired_speed - current_speed) / 2);
	}
	return acc_local;
}

double get_acc_for_GCACC() {
	double acc_local = 0.0;
	double reference_distance = GetReferenceDistance();
	acc_local = coef_acc * acc_of_lead_vehicle + coef_speed * (current_speed_of_lead_vehicle - current_speed) + coef_distance * (net_distance_to_lead_vehicle - reference_distance);
	acc_local = min(max_acc, acc_local);
	return acc_local;
}

double get_acc_for_meter_zone() {
	// set up everything
	set_net_gap_to_nveh();
	set_nveh_speed();
	set_parameters_for_lead_vehicle();
	// pick one mode to proceed
	long flag = get_driving_mode_at_meter_zone();
	double output;
	if (flag == 1) {
		output = get_acc_for_GCACC();
	}
	if (flag == 2) {
		output = get_acc_for_free_driving();
	}
	//check emergency
	if (CheckEmergency()) {
		output = GetAccOfEmergencybrake();
	}
	return output;
}

void OutputTrafficStateToFile() {
	//ofstream myfile;
	//myfile.open("traffic_state.txt", std::fstream::app);
	//myfile << "intac_target_type:" <<"\t" << intac_target_type << "\t" << turning_indicator <<"\n";
	//myfile.close();
}

double ResetDistanceToLeadCar(double distance) {
	if (distance == 0) {
		return 999.0;
	}
	else {
		return distance;
	}
}

double GetReferenceDistance() {
	double reference_distance_local = 0.0;
	reference_distance_local = std::max(4.0, headway_for_CACC * current_speed);
	return reference_distance_local;
}

bool CheckEmergency() {
	if ((net_distance_to_nveh_0_1 < 2)&(net_distance_to_nveh_0_1 > 0)) {
		return true;
	}
	else {
		return false;
	}
}

double GetAccOfEmergencybrake() {
	return -max_acc;
}

bool UseInternalModel() {
	// if current_link is not in meter_link, then use internal model. 
	if (is_now_in_meter_zone()) {
		return false;
	}
	else {
		return true;
	}
}

long determine_lead_vehicle_for_GCACC() {
	double smallest = find_min_net_distance();
	long output = 0;
	if (smallest == net_distance_to_nveh_0_1) {
		output = 0;
	}
	if (smallest == net_distance_to_nveh_l1_1) {
		output = 1;
	}
	if (smallest == net_distance_to_nveh_r1_1) {
		output = 2;
	}
	if (smallest == net_distance_to_nveh_l2_1) {
		output = 3;
	}
	if (smallest == net_distance_to_nveh_r2_1) {
		output = 4;
	}


	return output;
}

void set_parameters_for_lead_vehicle() {
	long flag = determine_lead_vehicle_for_GCACC();
	if (flag == 0) {
		acc_of_lead_vehicle = acc_of_nveh_0_1;
		net_distance_to_lead_vehicle = net_distance_to_nveh_0_1;
		current_speed_of_lead_vehicle = speed_of_nveh_0_1;
	}
	if (flag == 1) {
		acc_of_lead_vehicle = acc_of_nveh_l1_1;
		net_distance_to_lead_vehicle = net_distance_to_nveh_l1_1;
		current_speed_of_lead_vehicle = speed_of_nveh_l1_1;
	}
	if (flag == 2) {
		acc_of_lead_vehicle = acc_of_nveh_r1_1;
		net_distance_to_lead_vehicle = net_distance_to_nveh_r1_1;
		current_speed_of_lead_vehicle = speed_of_nveh_r1_1;
	}
	if (flag == 3) {
		acc_of_lead_vehicle = acc_of_nveh_l2_1;
		net_distance_to_lead_vehicle = net_distance_to_nveh_l2_1;
		current_speed_of_lead_vehicle = speed_of_nveh_l2_1;
	}
	if (flag == 4) {
		acc_of_lead_vehicle = acc_of_nveh_r2_1;
		net_distance_to_lead_vehicle = net_distance_to_nveh_r2_1;
		current_speed_of_lead_vehicle = speed_of_nveh_r2_1;
	}
}

void set_net_gap_to_nveh() {
	net_distance_to_nveh_0_1 = distance_to_nveh_0_1 - length_of_nveh_0_1;
	net_distance_to_nveh_l1_1 = distance_to_nveh_l1_1 - length_of_nveh_l1_1;
	net_distance_to_nveh_r1_1 = distance_to_nveh_r1_1 - length_of_nveh_r1_1;
	net_distance_to_nveh_l2_1 = distance_to_nveh_l2_1 - length_of_nveh_l2_1;
	net_distance_to_nveh_r2_1 = distance_to_nveh_r2_1 - length_of_nveh_r2_1;
}

void set_nveh_speed() {
	speed_of_nveh_0_1 = current_speed - speed_difference_to_nveh_0_1;
	speed_of_nveh_l1_1 = current_speed - speed_difference_to_nveh_l1_1;
	speed_of_nveh_r1_1 = current_speed - speed_difference_to_nveh_r1_1;
	speed_of_nveh_l2_1 = current_speed - speed_difference_to_nveh_l2_1;
	speed_of_nveh_r2_1 = current_speed - speed_difference_to_nveh_r2_1;
}

bool is_now_in_meter_zone() {
	for (int a = 0; a < sizeof(meter_link) / sizeof(meter_link[0]); a = a + 1) {
		if (current_link == meter_link[a]) {
			return true;
		}
	}
	return false;
}

double find_min_net_distance() {
	double all_net_distance[] = {net_distance_to_nveh_0_1,net_distance_to_nveh_l1_1,net_distance_to_nveh_r1_1,net_distance_to_nveh_l2_1,net_distance_to_nveh_r2_1};
	double smallest = all_net_distance[0];
	for (int a = 0; a < sizeof(all_net_distance) / sizeof(all_net_distance[0]); a = a + 1) {
		smallest = std::min(smallest, all_net_distance[a]);
	}
	return smallest;
}
/*==========================================================================*/
/*  End of DriverModel.cpp                                                  */
/*==========================================================================*/
