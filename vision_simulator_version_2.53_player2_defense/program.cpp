
#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>

using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer5.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

#define PI 3.14159

extern robot_system S1;

//GLOBAL VARIABLES TO CHANGE THE INITIAL POSITION
//ROBOT
double X0 = 140;
double Y0 = 400;
double THETA0 = 0;

//OBSTACLE
double X_OBS = 270;
double Y_OBS = 270;

struct target{
	char color;
	double iPos;
	double jPos;
};

//struct for each color's RGB value
struct color{
	int R;
	int G;
	int B;
};

int initialization();
int activate();
int deactivate();
int thresholdRGB();// to initialize the RGB values of 5 the colors given
int thresholdtarget(image &invCol, target &obj);
int defend();
void draw_centroids();
void move(char m);

image rgb;
image rgb0;
image label;
image grey1, grey2;

int nlabels;
target user1, user2, opponent1, opponent2, obstacle1;
color red, blue, green, orange, black;

const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;

int main()
{
	initialization();

	join_player();

	while (1) {

		acquire_image_sim(rgb);

		defend();

		Sleep(10); // 100 fps max

	}

	deactivate();

	deactivate_vision();

	deactivate_simulation();

	cout << "\ndone.\n";

	return 0;
}


int initialization(){

	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;

	// number of obstacle1s
	N_obs = 1;

	x_obs[1] = X_OBS;
	y_obs[1] = Y_OBS;
	size_obs[1] = 1.0;

	D = 121.0;

	Lx = 31.0;
	Ly = 0.0;

	Ax = 37.0;
	Ay = 0.0;

	alpha_max = 3.14159 / 2;

	n_robot = 2;

	cout << "\npress space key to begin program.";
	pause();

	activate_vision();

	activate_simulation(width1, height1, x_obs, y_obs, size_obs, N_obs,
		"robot_A.bmp", "robot_B.bmp", "background.bmp", "obstacle.bmp", D, Lx, Ly,
		Ax, Ay, alpha_max, n_robot);

	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 2;
	level = 1;
	set_simulation_mode(mode, level);

	// set robot initial position (pixels) and angle (rad)
	x0 = X0;
	y0 = Y0;
	theta0 = THETA0;
	set_robot_position(x0, y0, theta0);

	pw_l = 1500;
	pw_r = 1500;
	pw_laser = 1500;
	laser = 0;

	max_speed = 100;
	opponent_max_speed = 100;

	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	set_inputs(pw_l, pw_r, pw_laser, laser,
		light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	user1.color = 'g';//the front of our robot
	user2.color = 'r';//the back of our robot
	opponent1.color = 'o';//the front of the opponent robot
	opponent2.color = 'b';//the back of the opponent robot

	obstacle1.iPos = x_obs[1];
	obstacle1.jPos = y_obs[1];

	thresholdRGB();

	activate();

	return 0;
}


int defend(){

	int itarg1, jtarg1;
	double teta_e, teta_u, dist, dist_f, dist_b, difference_e_u, difference_o_u, teta_o1, dist_o1_f, dist_o1_b, alfa_e, alfa_o;

	thresholdtarget(label, user1);
	thresholdtarget(label, user2);
	thresholdtarget(label, opponent1);
	thresholdtarget(label, opponent2);


	// creating a mirror point of the target of the enemy with respect to the obstacle and basically attacking that mirror

	itarg1 = (int)(2 * obstacle1.iPos - opponent1.iPos);
	jtarg1 = (int)(2 * obstacle1.jPos - opponent1.jPos);


	//So robot does not move too far from the obstacle
	if (itarg1 < obstacle1.iPos - 110)
		itarg1 = (int)obstacle1.iPos - 110;
	if (itarg1 > obstacle1.iPos + 110)
		itarg1 = (int)obstacle1.iPos + 110;
	if (jtarg1 < obstacle1.jPos - 110)
		jtarg1 = (int)obstacle1.jPos - 110;
	if (jtarg1 > obstacle1.jPos + 110)
		jtarg1 = (int)obstacle1.jPos + 110;

    //So robot does not move too close to obstacle 
	if ((itarg1 < obstacle1.iPos + 100) && (itarg1 > obstacle1.iPos - 100) && (jtarg1 > obstacle1.jPos - 100) && (jtarg1 < obstacle1.jPos + 100)){

		if (itarg1 < obstacle1.iPos)
			itarg1 = (int)(2 * obstacle1.iPos - opponent1.iPos - 10);
		if (itarg1 > obstacle1.iPos)
			itarg1 = (int)(2 * obstacle1.iPos - opponent1.iPos + 10);
		if (jtarg1 < obstacle1.jPos)
			jtarg1 = (int)(2 * obstacle1.jPos - opponent1.jPos - 10);
		if (jtarg1 > obstacle1.jPos)
			jtarg1 = (int)(2 * obstacle1.jPos - opponent1.jPos + 10);
	}

	teta_e = (atan2(jtarg1 - user2.jPos, itarg1 - user2.iPos)) * 180 / PI;
	teta_u = (atan2(user1.jPos - user2.jPos, user1.iPos - user2.iPos)) * 180 / PI;
	teta_o1 = (atan2(obstacle1.iPos - user2.jPos, obstacle1.jPos - user2.iPos)) * 180 / PI;
	dist = sqrt(pow(opponent1.iPos - user2.iPos, 2) + pow(opponent1.jPos - user2.jPos, 2));
	dist_f = sqrt(pow(itarg1 - user1.iPos, 2) + pow(jtarg1 - user1.jPos, 2));
	dist_b = sqrt(pow(itarg1 - user2.iPos, 2) + pow(jtarg1 - user2.jPos, 2));
	dist_o1_f = sqrt(pow(obstacle1.iPos - user1.iPos, 2) + pow(obstacle1.jPos - user1.jPos, 2));
	dist_o1_b = sqrt(pow(obstacle1.iPos - user2.iPos, 2) + pow(obstacle1.jPos - user2.jPos, 2));
	alfa_e = 2 * atan2(14, dist_b) * 180 / PI;
	alfa_o = 2 * atan2(42, dist_o1_b) * 180 / PI;

	
	draw_point_rgb(rgb, (int)user1.iPos, (int)user1.jPos, 255, 0, 0);
	draw_point_rgb(rgb, (int)user2.iPos, (int)user2.jPos, 0, 255, 0);
	draw_point_rgb(rgb, (int)opponent1.iPos, (int)opponent1.jPos, 0, 0, 255);
	draw_point_rgb(rgb, (int)opponent2.iPos, (int)opponent2.jPos, 255, 255, 0);
	draw_point_rgb(rgb, (int)itarg1, (int)jtarg1, 0, 255, 255);


	if (teta_u > 0 && teta_e < 0 && teta_u > teta_e + 180){

		difference_e_u = (180 - teta_u) + (teta_e + 180);
	}
	else if (teta_u < 0 && teta_e>0 && teta_e > teta_u + 180){

		difference_e_u = -((teta_u + 180) + (180 - teta_e));
	}
	else{
		difference_e_u = teta_e - teta_u;
	}

	if (teta_u > 0 && teta_o1 < 0 && teta_u > teta_o1 + 180){

		difference_o_u = (180 - teta_u) + (teta_o1 + 180);
	}
	else if (teta_u < 0 && teta_o1>0 && teta_o1 > teta_u + 180){

		difference_o_u = -((teta_u + 180) + (180 - teta_o1));
	}
	else{
		difference_o_u = teta_o1 - teta_u;
	}


	if (dist < 20){
		move('g');
	}

	else{

		if (dist_o1_f < 80){

			if (abs(difference_o_u < 15)){
				if (difference_o_u < 0)
					move('l');
				else if (difference_o_u > 0)
					move('r');

				else
					move('b');

			}
			if (dist_o1_b < 80)
				move('f');
		}

		else if (dist_o1_f > 90) {


			if (abs(teta_e - teta_o1) < alfa_o / 2){

				if (abs(difference_e_u) < 30) {

					if (difference_e_u < 0){
						move('l');
					}

					else{
						move('r');
					}
				}

				else if (abs(difference_e_u) > 70){

					if (difference_e_u < 0){
						move('r');
					}

					else{
						move('l');
					}

				}
				else{
					move('f');
				}
			}

			else if (abs(difference_e_u) > alfa_e) {

				if (difference_e_u < 0){
					move('r');
				}

				else{
					move('l');
				}


			}
			else {

				if (dist_f > 30)

					move('f');

				else if ((dist_f < 30) && (dist_f > 10))
					move('g');

				else
					move('b');
			}

		}

	}
	//view_rgb_image(rgb);

	return 0;
}


int activate()
{


	rgb.type = RGB_IMAGE;
	rgb.width = IMAGE_WIDTH;
	rgb.height = IMAGE_HEIGHT;

	rgb0.type = RGB_IMAGE;
	rgb0.width = IMAGE_WIDTH;
	rgb0.height = IMAGE_HEIGHT;

	label.type = LABEL_IMAGE;
	label.width = IMAGE_WIDTH;
	label.height = IMAGE_HEIGHT;

	grey1.type = GREY_IMAGE;
	grey1.width = IMAGE_WIDTH;
	grey1.height = IMAGE_HEIGHT;

	grey2.type = GREY_IMAGE;
	grey2.width = IMAGE_WIDTH;
	grey2.height = IMAGE_HEIGHT;

	allocate_image(rgb);
	allocate_image(rgb0);
	allocate_image(label);

	allocate_image(grey1);
	allocate_image(grey2);

	return 0;
}


int deactivate()

{
	free_image(rgb);
	free_image(rgb0);
	free_image(label);
	free_image(grey1);
	free_image(grey2);

	return 0;
}

int thresholdRGB(){

	red.R = 226;
	red.G = 90;
	red.B = 77;

	green.R = 70;
	green.G = 176;
	green.B = 132;

	blue.R = 51;
	blue.G = 161;
	blue.B = 231;

	orange.R = 255;
	orange.G = 190;
	orange.B = 124;

	black.R = 45;
	black.G = 45;
	black.B = 45;

	return 0;
}

int thresholdtarget(image &label, target &obj){

	i4byte size, i;
	ibyte *pa, *pg1, Rs, Gs, Bs, Rt, Gt, Bt;//RGBs are the values 'seen' by the camera, RGBt are the predefined threshold values predefined

	size = IMAGE_WIDTH*IMAGE_HEIGHT;

	acquire_image_sim(rgb0);

	pa = rgb0.pdata;
	pg1 = grey1.pdata;

	switch (obj.color){

	case 'r':
		Rt = red.R;
		Gt = red.G;
		Bt = red.B;
		break;
	case 'b':
		Rt = blue.R;
		Gt = blue.G;
		Bt = blue.B;
		break;
	case 'g':
		Rt = green.R;
		Gt = green.G;
		Bt = green.B;
		break;
	case 'o':
		Rt = orange.R;
		Gt = orange.G;
		Bt = orange.B;
		break;
	case 'n':// for black
		Rt = black.R;
		Gt = black.G;
		Bt = black.B;
		break;
	default:
		Rt = 0;
		Gt = 0;
		Bt = 0;
		break;
	}

	for (i = 0; i < size; i++){

		Bs = *pa;
		Gs = *(pa + 1);
		Rs = *(pa + 2);

		if ((Rs > Rt - 10) && (Rs < Rt + 10) && (Gs > Gt - 10) && (Gs < Gt + 10) && (Bs > Bt - 10) && (Bs < Bt + 10))
			*pg1 = 255;
		else
			*pg1 = 0;
		pg1++;
		pa += 3;
	}

	lowpass_filter(grey1, grey1);
	erode(grey1, grey2);
	dialate(grey2, grey1);
	erode(grey1, grey2);
	dialate(grey2, grey1);

	label_image(grey1, label, nlabels);
	centroid(grey1, label, 1, obj.iPos, obj.jPos);

	return 0;
}

void move(char m){
	double max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;

	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0;
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100;


	switch (m){

	case 'f':
		pw_l = 1000;
		pw_r = 2000;
		break;
	case 'b':
		pw_l = 2000;
		pw_r = 1000;
		break;
	case 'l':
		pw_l = 1850;
		pw_r = 2000;
		break;
	case 'r':
		pw_l = 1000;
		pw_r = 1150;
		break;
	case 's':
		pw_l = 1500;
		pw_r = 1500;
		laser = 1;
	default:
		pw_l = 1500;
		pw_r = 1500;
		break;
	}


	set_inputs(pw_l, pw_r, pw_laser, laser,
		light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);
}
