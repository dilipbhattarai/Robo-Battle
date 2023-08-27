
#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>

using namespace std; 

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

extern robot_system S1;

void control_robot1(int x_target, int y_target, image &rgb, int &pwr, int &pwl); // two different robot control logic
void control_robot2(int x_target, int y_target, image& rgb, int& pwr, int& pwl, int&count1); //best option
int distance_ro(int x_target, int y_target, image& rgb);

double calculate_centroids(double& ic, double& jc, image& rgb);

double calculate_angle(double x1, double y1, double x2, double y2);
void set_point(int ip, int jp, image &rgb);
void count1(int &count);

int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed,thetar;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	int distance;
	int count = 0;

	// TODO: it might be better to put this model initialization
	// section in a separate function
	
	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1  = 640;
	height1 = 480;
	
	// number of obstacles
	N_obs  = 2;

	x_obs[1] = 350; // pixels
	y_obs[1] = 170; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 400; // pixels
	y_obs[2] = 400; // pixels
	size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////
	
	D = 121.0; // distance between front wheels (pixels)
		
	Lx = 31.0;
	Ly = 0.0;
	
	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;
	
	alpha_max = 3.14159/2; // max range of laser / gripper (rad)
	
	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;
	
	cout << "\npress space key to begin program.";
	pause();


	activate_vision();

	activate_simulation(width1,height1,x_obs,y_obs,size_obs,N_obs,
		"robot_A.bmp","robot_B.bmp","background.bmp","obstacle.bmp",D,Lx,Ly,
		Ax,Ay,alpha_max,n_robot);	

	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 0;
	level = 1;
	set_simulation_mode(mode,level);	
	
	// set robot initial position (pixels) and angle (rad)
	x0 = 130;
	y0 = 370;
	theta0 = -3.14159;
	set_robot_position(x0,y0,theta0);
	
/*
	// set opponent initial position (pixels) and angle (rad)
	x0 = 150;
	y0 = 375;
	theta0 = 3.14159/4;
	set_opponent_position(x0,y0,theta0);
*/
	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)
	
	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)
	
	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	// set initial inputs
	set_inputs(pw_l,pw_r,pw_laser,laser,
		light,light_gradient,light_dir,image_noise,
		max_speed,opponent_max_speed);

	// opponent inputs
	pw_l_o = 1500; // pulse width for left wheel servo (us)
	pw_r_o = 1500; // pulse width for right wheel servo (us)
	pw_laser_o = 1500; // pulse width for laser servo (us)
	laser_o = 0; // laser input (0 - off, 1 - fire)

	// manually set opponent inputs for the simulation
	// -- good for testing your program
	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
				opponent_max_speed);

	image rgb;
	int height, width;

	width  = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	// allocate memory for the images
	allocate_image(rgb);

	// measure initial clock time
	tc0 = high_resolution_time(); 
	
	double dpw = 250;
	int x_target = x_obs[1];
	int y_target = y_obs[1]; 
	distance = distance_ro(x_target, y_target, rgb);

	while (1) {
		
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;
		cout << "\n tc = " << tc;
	
		control_robot2(x_target, y_target, rgb, pw_r, pw_l,count);
	

		pw_l_o = 1500;
		pw_r_o = 1500;

		// read the keyboard and set the opponent inputs
		if (KEY('I')) {
			pw_l_o = 1500 - dpw;
			pw_r_o = 1500 + dpw;
		}

		if (KEY('K')) {
			pw_l_o = 1500 + dpw;
			pw_r_o = 1500 - dpw;
		}

		if (KEY('J')) {
			pw_l_o = 1500 + dpw;
			pw_r_o = 1500 + dpw;
		}

		if (KEY('L')) {
			pw_l_o = 1500 - dpw;
			pw_r_o = 1500 - dpw;
		}


		set_inputs(pw_l,pw_r,pw_laser,laser,
			light,light_gradient,light_dir,image_noise,
			max_speed,opponent_max_speed);

		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
			opponent_max_speed);

	//	distance = distance_ro(x_target, y_target, rgb);
	//	cout << "\n distance from distance_ro = " << distance;
	//	thetar = calculate_angle();
		view_rgb_image(rgb);

		Sleep(10); // 100 fps max
	}
	//cout << "\n distance after first while loop is = " << distance;
	// free the image memory before the program completes
	free_image(rgb);

	deactivate_vision();
	
	deactivate_simulation();	
	
	cout << "\ndone.\n";

	return 0;
}


int distance_ro(int x_target, int y_target, image& rgb) {
	int distance, x_distance, y_distance, tolerance = 80;
	double ix, jx;
	double theta_robot;					//theta_ro=theta between robot to obstacle, theta_robot= robot angle in the coordinate

	theta_robot = calculate_centroids(ix, jx, rgb);
	//cout << "\n theta_robot = " << theta_robot;
	// calculate the distance to the target

	x_distance = ix - x_target;
	y_distance = jx - y_target;
	distance = sqrt(x_distance * x_distance + y_distance * y_distance);

	return distance;
}



void control_robot1(int x_target, int y_target, image &rgb, int &pwr, int &pwl)
{
	//double dpw = 250;
	int distance, x_distance, y_distance, tolerance = 80;
	double ix, jx;
	double theta_robot,theta_ro;					//theta_ro=theta between robot to obstacle, theta_robot= robot angle in the coordinate
	int count = 0;
	theta_robot=calculate_centroids(ix, jx, rgb);
	cout << "\n theta_robot = " << theta_robot;
	// calculate the distance to the target

	x_distance = ix - x_target;
	y_distance = jx - y_target;
	distance = sqrt(x_distance * x_distance + y_distance * y_distance);
	theta_ro = -atan2(y_distance, x_distance);
	cout << "\n distance = " << distance;
	cout << "\n theta_ro =  " << theta_ro;
	// if the distance to the target is small then stop
	// the robot, otherwise move forward

	pwl = 1500;
	pwr = 1500;

	if (distance < 120) count++;
	cout << "\n count"<< count;
/*
	if (distance > 150 && count==0) {
		if (theta_robot<theta_ro + 0.05 && theta_robot>theta_ro - 0.05) {
			pwl = 1200;
			pwr = 1800;
		}
		else if (theta_robot < theta_ro-0.05 ) {

			pwl = 1300;
			pwr = 1300;
		}
		else if (theta_robot > theta_ro+0.05 ) {

			pwl = 1700;
			pwr = 1700;
		}
	}
	else if (distance >= 140 && distance <= 150 && count==0) {
		// turning left, the robot is moveing clockwise.
		pwl = 1700;
		pwr = 1800;
		cout << "\n left turning 1 ";
	} 
	else if (distance > 130 && count>=1){
		// turning right quickly,
	//	pwl = 1300;
	//	pwr = 1650;

		pwl = 1200;
		pwr = 1300;
		cout << "\n quick right turning,130<distance<140 ";
	}
	else if (distance > 110 && distance <= 130) {
		// turing right,main path
	//	pwl = 1300;
	//	pwr = 1700;

		pwl = 1200;
		pwr = 1600;
		cout << "\n turning right,110<distance<130, move in the center ";
	}
	else if (distance <= 110) {
		// turning left, too close to obstacle
	//	pwl = 1600;
	//	pwr = 1800;

		pwl = 1600;
		pwr = 1800;
		cout << "\n left turning,100<distance<110 too close to obstacle, i ";
	}
*/


	if (distance >= 130 && count == 0) {
		if (theta_robot<theta_ro + 0.05 && theta_robot>theta_ro - 0.05) {
			pwl = 1200;
			pwr = 1800;
		}
		else if (theta_robot < theta_ro - 0.05) {

			pwl = 1300;
			pwr = 1300;
		}
		else if (theta_robot > theta_ro + 0.05) {

			pwl = 1700;
			pwr = 1700;
		}
	}
	else if (distance >= 140 && distance <= 150 && count == 0) {
		// turning left, the robot is moveing clockwise.
		pwl = 1700;
		pwr = 1800;
		cout << "\n left turning 1 ";
	}
	else if (distance > 130 && count >= 1) {
		// turning right quickly,
	//	pwl = 1300;
	//	pwr = 1650;

		pwl = 1200;
		pwr = 1300;
		cout << "\n quick right turning,130<distance<140 ";
	}
	else if (distance > 110 && distance <= 130) {
		// turing right,main path
	//	pwl = 1300;
	//	pwr = 1700;

		pwl = 1200;
		pwr = 1600;
		cout << "\n turning right,110<distance<130, move in the center ";
	}
	else if (distance <= 110) {
		// turning left, too close to obstacle
	//	pwl = 1600;
	//	pwr = 1800;

		pwl = 1600;
		pwr = 1800;
		cout << "\n left turning,100<distance<110 too close to obstacle, i ";
	}

}

void control_robot2(int x_target, int y_target, image& rgb, int& pwr, int& pwl, int& count1)
{
	int distance, x_distance, y_distance;
	double ix, jx;
	double theta_robot, theta_ro;					//theta_ro=theta between robot to obstacle, theta_robot= robot angle in the coordinate
	theta_robot = calculate_centroids(ix, jx, rgb);
	cout << "\n theta_robot = " << theta_robot;
	// calculate the distance to the target
	//int count = count1;
	x_distance = ix - x_target;
	y_distance = jx - y_target;
	distance = sqrt(x_distance * x_distance + y_distance * y_distance);
	theta_ro = -atan2(y_distance, x_distance);
	cout << "\n distance = " << distance;

	// if the distance to the target is small then stop
	// the robot, otherwise move forward

	pwl = 1500;
	pwr = 1500;

	cout << "\n count = " << count1;

	if (distance > 95 && count1==0) {
		if (theta_robot<theta_ro + 0.05 && theta_robot>theta_ro - 0.05) {
			pwl = 1300;
			pwr = 1700;
		}
		else if (theta_robot < theta_ro ) {

			pwl = 1200;
			pwr = 1200;
		}
		else if (theta_robot > theta_ro ) {

			pwl = 1800;
			pwr = 1800;
		}
	}
	
	else if (distance <= 95 && count1 == 0) {
		cout << "\n initial rotation found";
		if (theta_ro > -3.14159 / 2 && theta_ro <= 0) {	// the robot is at the top right of obstacle
			if (theta_robot <= theta_ro - 3.14159 / 2) {
				pwl = 1500;
				pwr = 1500;
				count1++;
				cout << "\n top right";
			}
			else {
				pwl = 2000;
				pwr = 2000;
			}
		}
		else if (theta_ro < 3.14159 / 2 && theta_ro >= 0) {	// the robot is at the bottom right of obstacle
			if (theta_robot <= theta_ro - 3.14159 / 2) {
				pwl = 1500;
				pwr = 1500;
				count1++;
				cout << "\n bottom right";
			}
			else {
				pwl = 2000;
				pwr = 2000;
			}
		}
		else if (theta_ro > 3.14159 / 2) {	// the robot is at the bottom left of obstacle
			if (theta_robot <= theta_ro - 3.14159 / 2) {
				pwl = 1500;
				pwr = 1500;
				count1++;
				cout << "\n bottom left";
			}
			else {
				pwl = 2000;
				pwr = 2000;
			}
		}
		else if (theta_ro < -3.13159 / 2) {	// the robot is at the top left of obstacles 
			if (abs(theta_robot) < 3.14159 / 2 + (theta_ro + 3.14159)) {

				pwl = 1500;
				pwr = 1500;
				count1++;
				cout << "\n top left";
			}
			else {
				pwl = 2000;
				pwr = 2000;
			}

		}
		
	}
	else if (distance<=110 && count1!=0) {
		
		pwl = 1200;
		pwr = 1560;
		cout << "\n move around obstacle";

	}
	else if (distance < 70 && count1 != 0) {
		pwl = 1400;
		pwr = 1700;
	}
	

}


double calculate_centroids(double &ic, double &jc, image &rgb)
{
	int i,j,k;
	ibyte R,G,B;
	int height, width;
	ibyte *p,*pc;
	double ic1, jc1, ic2, jc2;
	double mi, mj, m, eps;
	double theta_robot;

	width  = rgb.width;
	height = rgb.height;

	p = rgb.pdata;

	// always initialize summation variables
	mi = mj = m = 0.0; 

	for (j = 0; j < height; j++) { // j coord

		for (i = 0; i < width; i++) { // i coord

			k = i + width * j;
			pc = p + 3 * k; // pointer to the kth pixel (3 bytes/pixel)

			B = *pc;
			G = *(pc + 1);
			R = *(pc + 2);

			// find blue pixels and calculate their centroid stats
			if ((B < 140) && (R < 80) && (G > 160)) {

				R = 0;
				G = 255;
				B = 0;

				// highlight blue pixels in the image
				*pc = B;
				*(pc + 1) = G;
				*(pc + 2) = R;

				// to calculate the centroid you need to calculate mk 
				// - the mass of each pixel k

				// mk = volume * density
				// mk = (pixel area) * (blue intensity)
				// mk = (blue intensity) = B
				// since (pixel area) = 1 pixel x 1 pixel = 1

				// calculate total mass m = sum (mk)
				m += G;

				// calculate total moments in the i and j directions
				mi += i * G; // (i moment of mk) = mk * i
				mj += j * G; // (j moment of mk) = mk * j

			} // end if

		} // end for i

	} // end for j
	eps = 1.0e-10; // small constant to protect against /0
	ic1 = mi / (m + eps);
	jc1 = mj / (m + eps);

	cout << "\nic1 = " << ic1 << " , jc1 = " << jc1;

//	cout << "\nfinal result -- press space key to continue";
	//pause();


	for (j = 0; j < height; j++) { // j coord

		for (i = 0; i < width; i++) { // i coord

			k = i + width * j;
			pc = p + 3 * k; // pointer to the kth pixel (3 bytes/pixel)

			B = *pc;
			G = *(pc + 1);
			R = *(pc + 2);

			// find blue pixels and calculate their centroid stats
			if ((B < 80) && (R > 210) && (G < 95)) {

				R = 255;
				G = 0;
				B = 0;

				// highlight blue pixels in the image
				*pc = B;
				*(pc + 1) = G;
				*(pc + 2) = R;

				// to calculate the centroid you need to calculate mk 
				// - the mass of each pixel k

				// mk = volume * density
				// mk = (pixel area) * (blue intensity)
				// mk = (blue intensity) = B
				// since (pixel area) = 1 pixel x 1 pixel = 1

				// calculate total mass m = sum (mk)
				m += R;

				// calculate total moments in the i and j directions
				mi += i * R; // (i moment of mk) = mk * i
				mj += j * R; // (j moment of mk) = mk * j

			} // end if

		} // end for i

	} // end for j

	view_rgb_image(rgb);
	// calculate the centroid (ic,jc)
	// -- note that ic, jc can have subpixel accuracy 
	// (1/10 of a pixel or better typically)

	eps = 1.0e-10; // small constant to protect against /0
	ic2 = mi / (m + eps);
	jc2 = mj / (m + eps);

	cout << "\nic2 = " << ic2 << " , jc2 = " << jc2;

	cout << "\nfinal result -- press space key to continue";
//	ic = (ic1 + ic2) / 2;
//	jc = (jc1 + jc2) / 2;
	//pause();

	// we consider the front the circle is the center 
	ic = ic1;			
	jc = jc1;


	theta_robot=calculate_angle(ic1, jc1, ic2, jc2);

	// mark centroid as a visual check
	// -- this type of marking up the image is useful
	// for testing/debugging with the project
	set_point((int)ic,(int)jc,rgb);
//	set_point((int)ic2, (int)jc2, rgb);
	view_rgb_image(rgb);

	return theta_robot;
}


void set_point(int ip, int jp, image &rgb)
{
	ibyte *p, *p0, grey, R, G, B;
	int i, j, k, i1, j1, n, w, width, height;
	
	width = rgb.width;
	height = rgb.height;
	
	// get the pointer to the image data
	p0 = rgb.pdata;
	
	// nxn square
	// assume n is odd -- want pixel ip,jp to be in the center
	n = 3;
	w = (n - 1)/2;
	
	for(j = jp - w; j <= jp + w; j++) { // row j
	
		for(i = ip - w; i <= ip + w; i++) { // col i
		
			i1 = i;
			j1 = j;

			if( i1 < 0 ) {
				i1 = 0;
			}
			
			if( i1 > width - 1 ) {
				i1 = width - 1;
			}
		
			if( j1 < 0 ) {
				j1 = 0;
			}
			
			if( j1 > height - 1 ) {
				j1 = height - 1;
			}
		
			k = i1 + j1*width;
			
			p = p0 + 3*k;
			
			*p     = 255; // B
			*(p+1) = 0; // G
			*(p+2) = 0; // R
			
		} // end for i
		
	} // end for j

}
double calculate_angle(double x1, double y1, double x2, double y2)
{
	double dot = x2 - x1;
	double det = y2 - y1;
	double result = -atan2(det, dot);

	// convert result to degrees
	double degree = result * (180 / 3.141592);

//	cout << "atan2(det/dot) = " << result << " radians" << endl;
//	cout << "atan2(det/dot) = " << degree << " degrees";

	
	return result;
}

void count1(int &count) {
	count++;
}
