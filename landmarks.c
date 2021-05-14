#include <math.h>

#include "landmarks.h"

static uint16_t N_surface_landmarks = 0;
static uint16_t N_wall_landmarks = 0;
static uint16_t N_corners = 0;
static wall_t wall_landmarks[NB_WALL_LANDMARK_MAX];
static landmark_t surface_landmarks[NB_SURFACE_LANDMARK_MAX];
static wall_t corners[NB_CORNERS_MAX];


//Calculate euclidean distance between two wall landmarks
static float distance(wall_t a, wall_t b){
	return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

//Calculate distance between line and point
static float error(line_t line, wall_t l){
	return fabs(l.y-line.alpha-line.beta*l.x)/sqrt(line.beta*line.beta+1);
}

//Calculate intersection between two lines
static wall_t calculateIntersection(line_t a, line_t b){
	wall_t l;
	if(a.beta==b.beta)a.beta*=0.0001;
	l.x = (b.alpha-a.alpha)/(a.beta-b.beta);
	l.y = a.beta*l.x+a.alpha;
	return l;
}

//Calculates line which passes through two given landmarks
static line_t fitTwoPoints(wall_t a, wall_t b){
	line_t l;
	if(a.x==b.x){
		a.x++;
	}
	l.beta = (a.y-b.y)/(1.0*a.x-1.0*b.x);
	l.alpha = (b.y*a.x-a.y*b.x)/(1.0*a.x-1.0*b.x);
	return l;
}

//Calculate maximal distance of a line passing through first and last point of array and all other points
static int16_t calculateMaxError(wall_t *l_ptr_begin, uint16_t N){
	uint16_t maxErrorIndex = 0;
	line_t line = fitTwoPoints(*l_ptr_begin, *(l_ptr_begin+N-1));
	for(uint16_t i = 0; i < N; i++){
		if(error(line, *(l_ptr_begin+i)) > error(line, *(l_ptr_begin+maxErrorIndex))){
			maxErrorIndex = i;
		}
	}
	if(error(line, *(l_ptr_begin+maxErrorIndex))< THRESHOLD_FITTING)return -1;
	return maxErrorIndex;
}

//Linear Regression fitting
static line_t fitLine(wall_t* l_ptr_begin, uint16_t N){
	line_t line;
	float x_mean = 0, y_mean = 0, x_var = 0, xy_covar = 0;
	wall_t* l_ptr = l_ptr_begin;
	for(uint16_t i = 0; i < N; i++){
		x_mean+=l_ptr->x;
		y_mean+=l_ptr->y;
		l_ptr++;
	}
	x_mean/=N;
	y_mean/=N;
	l_ptr = l_ptr_begin;
	for(uint16_t i = 0; i < N; i++){
		x_var+=(l_ptr->x-x_mean)*(l_ptr->x-x_mean);
		xy_covar+= (l_ptr->x-x_mean)*(l_ptr->y-y_mean);
		l_ptr++;
	}
	line.beta = xy_covar/x_var;
	line.alpha = y_mean-line.beta*x_mean;
	return line;
}

//Calculate new corners for polygon fitting of walls
static void calculate_linesegments(bool closeLoop){
	static line_t prevLine = {0, 0};
	static line_t firstLine = {0, 0};

	if(closeLoop){
		corners[N_corners] = calculateIntersection(prevLine, firstLine);
		corners[0] = corners[N_corners];
		N_corners++;
		return;
	}

	if(N_wall_landmarks<2)return;
	int16_t devide = calculateMaxError(&wall_landmarks[0], N_wall_landmarks);
	if(devide < 2)return;
	if(prevLine.alpha == 0 && prevLine.beta == 0){
		prevLine = fitLine(&wall_landmarks[0], devide);
		firstLine = prevLine;
		return;
	}
	line_t currentLine = fitLine(&wall_landmarks[0], devide);
	corners[N_corners] = calculateIntersection(prevLine, currentLine);
	N_corners++;
	prevLine = currentLine;
	chSysLock();
	for(uint16_t i = 0; i < N_wall_landmarks-devide-1; i++){
		wall_landmarks[i] = wall_landmarks[i+devide+1];
	}
	N_wall_landmarks = N_wall_landmarks-devide-1;
	chSysUnlock();
	return;
}

/****************************PUBLIC FUNCTIONS*************************************/


bool enter_landmark(landmark_t coordinates){
	if(coordinates.z == TOF)return false;
	else if(coordinates.z == IR){
		//Memory protection to avoid access of uninitialized memory
		if(N_wall_landmarks >= NB_WALL_LANDMARK_MAX)return true;

		//Store first wall coordinates for close loop detection
		static wall_t first_wall;
		wall_landmarks[N_wall_landmarks] = (wall_t){coordinates.x, coordinates.y};
		if(N_wall_landmarks==0){
			first_wall = wall_landmarks[0];
		}
		N_wall_landmarks++;

		//Start line fitting algorithm and close loop if robot arrives at original position
		calculate_linesegments(false);
		if(N_corners > 1 && distance(first_wall, wall_landmarks[N_wall_landmarks-1]) < CLOSE_LOOP_RADIUS){
			calculate_linesegments(true);
			N_wall_landmarks = 0;
			return true;
		}
		return false;
	}
	else{
		//Memory protection to avoid access of uninitialized memory
		if(N_surface_landmarks > NB_SURFACE_LANDMARK_MAX)return true;

		surface_landmarks[N_surface_landmarks] = coordinates;
		N_surface_landmarks++;
		return false;
	}
}

uint16_t get_nb_surface_landmarks(void){
	return N_surface_landmarks;
}


landmark_t* get_surface_landmark_ptr(void){
	return &surface_landmarks[0];
}


uint16_t get_nb_wall_landmarks(void){
	return N_wall_landmarks;
}


wall_t* get_wall_landmark_ptr(void){
	return &wall_landmarks[0];
}


uint16_t get_nb_corners(void){
	return N_corners;
}


wall_t* get_corner_ptr(void){
	return &corners[0];
}


