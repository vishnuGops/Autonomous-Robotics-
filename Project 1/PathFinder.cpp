//============================================================================
// Name        : PathFinder.cpp
// Author      : $(author)
// Version     :
// Copyright   : $(copyright)
// Description : Hello World in C++
//============================================================================

#include <ev3.h>
#include <string>
#include <math.h>

const int engine_l = OUT_B;  // right
const int engine_r = OUT_A;  // left
const int engine_b = OUT_AB; // both
const double PI_M = 3.141592653589;

const double wheal_radius = 57;
const double axle_radius =250;


const int forward_speed = 20;
const int rotation_speed = 5;
const double deg90 = 370;
const double deg180 = deg90*2;
const double deg45 = deg90/2;
const int block_distance = 305;
const int vblock_distance = block_distance/2;


#define NUM_ROWS 16
#define NUM_COLS 10
#define MAX_OBSTACLES 25
#define NUM_ROTATE 4

#define ROW_SIZE 3.66
#define COL_SIZE 3.05

// for the path finder
#define NOTHING 0
#define OBSTACLE 1
//#define NEAR_OBSTACLE 'N'
#define GOAL 2
#define START 99

#define ROTATIONS 175

int Map[NUM_ROWS][NUM_COLS] ={
		{0,0,0,99,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,1,1},
		{0,0,0,0,0,0,0,0,1,1},
		{0,0,0,0,0,0,0,0,1,1},
		{0,0,0,0,0,0,0,0,1,1},
		{0,0,0,0,0,0,0,0,0,0},
		{0,0,0,1,1,1,1,1,0,0},
		{0,0,0,1,1,1,1,1,0,0},
		{0,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0},
		{1,1,1,1,0,0,0,0,1,1},
		{1,1,1,1,0,0,0,0,1,1},
		{0,0,0,0,0,0,0,0,1,1},
		{0,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,2,0,0,0,0}
};

const double jerk_degree = 1.5;

// vishnu

 int x_blocks = 16;
 int y_blocks = 10;



void incFoward(int v_f,int v_i){
	int increment = 5;
	int time_increment = 500;
	while(v_i!=v_f){
		if(v_i < v_f){
			v_i+=increment;
		}
		else{
			v_i-=increment;
		}
		if(v_i!=v_f){
			OnFwdSync(engine_b,v_i);
			Wait(time_increment);
		}

	}
}

void travelFwd2(double distance){
//	double calcDistance = 0;

//	get current rotation
	int r_i = MotorRotationCount(engine_l);
	int r_f = r_i;
	int r_d = r_f - r_i;


	incFoward(forward_speed,0);
	OnFwdSync(engine_b,forward_speed);
	int rotation_max = 630;
	while(r_d < rotation_max){
		r_f = MotorRotationCount(engine_l);
		r_d = r_f-r_i;
	}

	Off(engine_b);

	LcdPrintf(1,"r_d: %d\n",r_d);

}

// Move Right
void moveRight(double distance){

//	right
	int r_i = MotorRotationCount(engine_r);
	int r_f = r_i;
	int r_d = abs(r_f - r_i);
	OnRevReg(engine_r,rotation_speed);
	while(r_d < ROTATIONS){
		r_f = MotorRotationCount(engine_r);
		r_d = abs(r_f - r_i);
	}
	Off(engine_r);
	int r_d2 = r_d;
//	left
	r_i = MotorRotationCount(engine_l);
	r_f = r_i;
	r_d = abs(r_f - r_i);
	OnFwdReg(engine_l,rotation_speed);
	while(r_d < ROTATIONS){
		r_f = MotorRotationCount(engine_l);
		r_d = abs(r_f - r_i);
	}
	Off(engine_l);
//	LcdPrintf(1,"r_d: %d\n",r_d);
	LcdPrintf(1,"Right l: %d, r: %d\n",r_d,r_d2);
}

// Move Left
void moveLeft(double distance){

//	left
	int r_i = MotorRotationCount(engine_l);
	int r_f = r_i;
	int r_d = abs(r_f - r_i);
	OnRevReg(engine_l,rotation_speed);
	while(r_d < ROTATIONS){
		r_f = MotorRotationCount(engine_l);
		r_d = abs(r_f - r_i);
	}
	Off(engine_l);

//	right
	int r_d2 = r_d;
	r_i = MotorRotationCount(engine_r);
	r_f = r_i;
	r_d = abs(r_f - r_i);
	OnFwdReg(engine_r,rotation_speed);
	while(r_d < ROTATIONS){
		r_f = MotorRotationCount(engine_r);
		r_d = abs(r_f - r_i);
	}
	Off(engine_r);
	LcdPrintf(1,"Left l: %d, r: %d\n",r_d2,r_d);
}
//void travelRot2Left(double distance){
//    RotateMotor(engine_r,rotation_speed,distance);
//}


void moveForward(double distance){
	int numIterations = distance/block_distance;
	int r_i = MotorRotationCount(engine_r);
	int l_i = MotorRotationCount(engine_l);
	for(int i = 0; i<numIterations;i++){

	//travelRot2Left(deg90*jerk_degree/90.0);
	RotateMotor(engine_r,abs(rotation_speed),abs(deg90*jerk_degree/90.0));
	Wait(100);
	travelFwd2(block_distance*1);
	Wait(100);

	}
//	double dist = distance - ((double)(numIterations)*block_distance);
	int r_d = r_i - MotorRotationCount(engine_r);
	int r_d2 = l_i - MotorRotationCount(engine_l);
	LcdPrintf(1,"r: %d, l: %d\n",r_d,r_d2);
}

//vishnu path finding code

void pathsearch()
{
	// setUP
	int x_goal , y_goal;
	bool foundpath = true;
	int goalvalue = GOAL;   //looks for goal value

	while(foundpath)
	{
		foundpath = false;
		for(int y=0; y<y_blocks;y++)
		{
			for(int x=0; x<x_blocks;x++)
			{
				//checks the direction to move in
				if(Map[x][y] == goalvalue)
				{
					foundpath =true;
					x_goal =x;
					y_goal = y;

					if(x_goal>0) //checks array bounds heading west
						if(Map[x_goal-1][y_goal]==0) //check the west direction
							Map[x_goal-1][y_goal] = goalvalue+1;


					if(x_goal<(x_blocks-1)) //checks array bounds heading east
						if(Map[x_goal+1][y_goal]==0) //check the east direction
							Map[x_goal+1][y_goal] = goalvalue+1;


					if(y_goal>0)//checks array bounds heading south
						if(Map[x_goal][y_goal-1]==0) //check the south direction
							Map[x_goal][y_goal-1] = goalvalue+1;


					if(x_goal<(y_blocks-1))//checks array bounds heading north
						if(Map[x_goal][y_goal+1]==0) //check the north direction
							Map[x_goal][y_goal+1] = goalvalue+1;
				}
			}
		}
		goalvalue++;
	}

}

void navigation()
{
// find initial coordinates
	int robot_x,robot_y;
	for(int x=0; x< NUM_ROWS; x++){
		for(int y=0; y<NUM_COLS; y++){
			if(Map[x][y]==99){
				robot_x=x;
				robot_y=y;
			}
		}
	}

 // map for directions
 // 0 is North
 // 1 is East
 // 2 is South
 // 3 is West

// init variables
	int cur_x=robot_x;
	int cur_y=robot_y;
	int cur_dir=1;
	int next_dir=3;
	int cur_low=99;
	int finishState = -2;
	while(cur_low>2){
		if(ButtonIsDown(BTNCENTER)){
			break;
		}
//		setup
		cur_low=99;
		int next_x=-1;
		int next_y=-1;

		//check the north block to see if its free
		if(cur_y < NUM_COLS-1){
			int testLow = Map[cur_x][cur_y+1];
			if(testLow < cur_low && testLow != OBSTACLE && testLow != finishState)
			{
				cur_low = testLow;//set the next number
				next_dir=0;//set next direction to north
				next_x = cur_x;
				next_y = cur_y+1;
			}
		}

		//check the east block to see if its free
		if(cur_x < NUM_ROWS-1){
			int testLow = Map[cur_x+1][cur_y];
			if(testLow < cur_low && testLow != OBSTACLE && testLow != finishState)
			{
				cur_low = testLow;//set the next number
				next_dir = 1;//set next direction to east
				next_x = cur_x+1;
				next_y = cur_y;
			}
		}

		//check the south block to see if its free
		if(cur_y > 0){
			int testLow = Map[cur_x][cur_y-1];
			if(testLow < cur_low && testLow != OBSTACLE && testLow != finishState)
			{
				cur_low = testLow;//set the next number
				next_dir = 2;//set next direction to south
				next_x = cur_x;
				next_y = cur_y-1;
			}
		}

		// check the west block to see if its free
		if(cur_x > 0)
		{
			int testLow = Map[cur_x-1][cur_y];
			if(testLow < cur_low && testLow != OBSTACLE && testLow != finishState)
			{
				cur_low = testLow; //set the next number
				next_dir = 3; //set next direction to west
				next_x = cur_x-1;
				next_y = cur_y;
			}
		}

		cur_x = next_x;
		cur_y = next_y;

		Map[cur_x][cur_y]=finishState;  //to get a mapping of the robot traverse

		//to track the robot turns
		bool TtZ = cur_dir == 3 && next_dir == 0;
		bool ZtT = cur_dir == 0 && next_dir==3;
		if((4+cur_dir - next_dir)%4 == 2){
			moveLeft(deg90);
			moveLeft(deg90);
		}
		else if((cur_dir > next_dir && !TtZ) || ZtT){
			moveLeft(deg90);
		}
		else if(cur_dir == next_dir){}
		else{
			moveRight(deg90);
		}
		cur_dir = next_dir;
		Wait(500);

//		move forward
		moveForward(block_distance);
		Wait(500);
		//other wise if same direction and free space then move forward
		//and the go to next block and check there
	}

}

//end of vishnu code

int main()
{
  InitEV3();
  //LcdPrintf(1,"Hello world");

  int r_i_r = MotorRotationCount(engine_r);
  int r_i_l = MotorRotationCount(engine_l);
  /* number of obstacles */
  double obstacle[MAX_OBSTACLES][2] = {
		  {0.61, 2.743},{0.915, 2.743},{1.219, 2.743},{1.829, 1.219},
		  {1.829, 1.524},{ 1.829, 1.829}, {1.829, 2.134},{2.743, 0.305},
		  {2.743, 0.61},{2.743, 0.915},{2.743, 2.743},{3.048, 2.743},
		  {3.353, 2.743},
		  {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
		  {-1,-1},{-1,-1},{-1,-1}};
  double start[2] = {0.305, 1.219}; /* start location */
  double goal[2] = {3.658, 1.829}; /* goal location */
//
  pathsearch();
  navigation();


//  rotation_speed = 100;
//  moveLeft(deg90);
//  moveLeft(deg90);
//  moveLeft(deg90);
//  moveLeft(deg90);

//  for(int i = NUM_ROWS/2; i < NUM_ROWS; i++){
//	  for(int j = 0; j < NUM_COLS; j++){
//		  if(Map[i][j]==OBSTACLE){
//			  LcdPrintf(1,"!");
//		  }
//		  else if(Map[i][j] == -2){
//			  LcdPrintf(1,"*");
//		  }
//		  else{
//			  LcdPrintf(1," ");
//		  }
////		  LcdPrintf(1,"%2d",Map[i][j]);
//	  }
//	  LcdPrintf(1,"\n");
//  }
  int r_f_r = MotorRotationCount(engine_r);
  int r_f_l = MotorRotationCount(engine_l);
  int r_d_r = r_f_r - r_i_r;
  int r_d_l = r_f_l - r_i_l;
//  LcdPrintf(1,"r_d R: %d\n",r_d_r);
//  LcdPrintf(1,"r_d L: %d\n",r_d_l);

  Wait(5000);
  FreeEV3();

}
