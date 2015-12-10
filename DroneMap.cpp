#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <queue>
#include <fstream>

typedef struct{
	int state; //state of drone
	int x; //x position
	int y; //y position
	int xdestination;
	int ydestination;
	/*
	0: stationed
	1: runway queue
	2: takeoff
	3: navigate
	4: avoid obstacle
	5: at destination
	6: returning
	7: landing queue
	8: landing
	*/
}drone;

typedef struct{
	char state;
	/*
	 : open
	P: open with package
	T: static obstacle (Tree)
	^: drone obstacle
	S: station
	*/
}cell;

typedef struct{
	int x;
	int y;
}package;

#define XMAP 50 //50X50 map
#define YMAP 50
#define NUMDRONES 10
int XSTATION;
int YSTATION;
package packages[10]; //10 packages
int packagecounter = 0;
cell map[XMAP][YMAP]; //50x50 map
drone drones[NUMDRONES]; //array with all drones
pthread_t thread[NUMDRONES];
pthread_mutex_t mutex;
std::queue<drone*> takeoffqueue; //queue for taking off

int main(int argc, char *argv[]){
	std::fstream f;
	f.open(argv[1]);//open input file
	for(int y = 0; y<YMAP; y++){ //initialize map bottom right is (49,19) top left is 0,0
		for(int x = 0; x<XMAP; x++){
			char c;
			f>>c;
			if(c == 'X') {map[x][y].state = ' ';}else{ //make X the spaces, everything thing else is that char
			map[x][y].state = c;}
			if(c=='P'){//assign Package to drone
				packages[packagecounter].x = x;
				packages[packagecounter].y = y;
				packagecounter++;
			}
			if(c=='S'){//get coordinates of the station
				XSTATION = x;
				YSTATION = y;
			}
		}
	}
	
	void *status;
	pthread_attr_t attr;
	pthread_mutex_init(&mutex,NULL); //initialize mutex
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	
	for(int i = 0; i < packagecounter; i++){ //initialize to stationed state
		drones[i].state = 0;
	}
	void sleep(long d);
	void assigntasks();
	void *takeoff(void *arg);
	void navigate(long offset); //declare functions that will be used
	for(int y = 0; y<YMAP; y++){//print map loop
		for(int x = 0; x<XMAP; x++){
			std::cout<<map[x][y].state;
		}
		std::cout<<std::endl;
	}//end print map loop
	assigntasks();//assign tasks at T=0
	for(int i = 0; i<packagecounter; i++){
		pthread_create(&thread[i],&attr,takeoff,(void *) i);
	}//create threads
	pthread_attr_destroy(&attr);
	for(int i = 0; i<packagecounter; i++){
		pthread_join(thread[i], &status);
	}
	pthread_mutex_destroy(&mutex);
	pthread_exit(NULL);
}
void sleep(long d) //this function will be used to delay drones
{ 
	clock_t start=clock(); 
	while(clock() - start < d); ///loop until time's up 
}
void navigate(long offset){//this function makes the drone go to their destination
	void avoidobstacle(long offset,int direction,int land);
	while(drones[offset].x != drones[offset].xdestination){ //move to correct column
		if(drones[offset].x < drones[offset].xdestination){ // drone is to the left
			pthread_mutex_lock(&mutex);//check collisions
			if(map[drones[offset].x+1][drones[offset].y].state == 'T'||map[drones[offset].x+1][drones[offset].y].state == 'S'||map[drones[offset].x+1][drones[offset].y].state == '^'){
				drones[offset].state = 4;
				pthread_mutex_unlock(&mutex);
				avoidobstacle(offset, 2,0);
			}
			map[drones[offset].x][drones[offset].y].state = ' ';
			drones[offset].x++;
			map[drones[offset].x][drones[offset].y].state = '^';
			pthread_mutex_unlock(&mutex);
		}
		if(drones[offset].x > drones[offset].xdestination){ // a is to the right
			pthread_mutex_lock(&mutex);
			if(map[drones[offset].x-1][drones[offset].y].state == 'T'||map[drones[offset].x-1][drones[offset].y].state == 'S'||map[drones[offset].x-1][drones[offset].y].state == '^'){
				drones[offset].state = 4;
				pthread_mutex_unlock(&mutex);
				avoidobstacle(offset, 4,0);
			}
			map[drones[offset].x][drones[offset].y].state = ' ';
			drones[offset].x--;
			map[drones[offset].x][drones[offset].y].state = '^';
			pthread_mutex_unlock(&mutex);
		}
		printf("\033[2J\033[1;1H");
		pthread_mutex_lock(&mutex);
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
	}
	while(drones[offset].y != drones[offset].ydestination){ //move to correct row
		if(drones[offset].y < drones[offset].ydestination){ // a is above
			pthread_mutex_lock(&mutex);
			if(map[drones[offset].x][drones[offset].y+1].state == 'T'||map[drones[offset].x][drones[offset].y+1].state == 'S'||map[drones[offset].x][drones[offset].y+1].state == '^'){
				drones[offset].state = 4;
				pthread_mutex_unlock(&mutex);
				avoidobstacle(offset, 3,0);
			}
			map[drones[offset].x][drones[offset].y].state = ' ';
			drones[offset].y++;
			map[drones[offset].x][drones[offset].y].state = '^';
			pthread_mutex_unlock(&mutex);
		}
		if(drones[offset].y > drones[offset].ydestination){ // a is below
			pthread_mutex_lock(&mutex);
			if(map[drones[offset].x][drones[offset].y-1].state == 'T'||map[drones[offset].x][drones[offset].y-1].state == 'S'||map[drones[offset].x][drones[offset].y-1].state == '^'){
				drones[offset].state = 4;
				pthread_mutex_unlock(&mutex);
				avoidobstacle(offset, 1,0);
			}
			map[drones[offset].x][drones[offset].y].state = ' ';
			drones[offset].y--;
			map[drones[offset].x][drones[offset].y].state = '^';
			pthread_mutex_unlock(&mutex);
		}
		printf("\033[2J\033[1;1H");
		pthread_mutex_lock(&mutex);
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		} //end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
	}
	void navigateback(long offset);
	void droppackage(long &off);
	droppackage(offset); //drop package, get return coordinates
	navigateback(offset); //navigate back
	//if(count==1){goto Start;}
	drones[offset].state = 6;
}
void navigateback(long offset){ //opposite order of navigate to ensure same path
	void land(long offset);
	void avoidobstacle(long offset,int direction,int land);
	while(drones[offset].y != drones[offset].ydestination){
		if(drones[offset].y < drones[offset].ydestination){ // a is below
			pthread_mutex_lock(&mutex);
			if(map[drones[offset].x][drones[offset].y+1].state == 'T'||map[drones[offset].x][drones[offset].y+1].state == 'S'||map[drones[offset].x][drones[offset].y+1].state == '^'){
				drones[offset].state = 4;
				pthread_mutex_unlock(&mutex);
				avoidobstacle(offset, 3,1);
			}
			map[drones[offset].x][drones[offset].y].state = ' ';
			drones[offset].y++;
			map[drones[offset].x][drones[offset].y].state = '^';
			pthread_mutex_unlock(&mutex);
		}
		if(drones[offset].y > drones[offset].ydestination){ // a is above
			pthread_mutex_lock(&mutex);
			if(map[drones[offset].x][drones[offset].y-1].state == 'T'||map[drones[offset].x][drones[offset].y-1].state == 'S'||map[drones[offset].x][drones[offset].y-1].state == '^'){
				drones[offset].state = 4;
				pthread_mutex_unlock(&mutex);
				avoidobstacle(offset, 1,1);
			}
			map[drones[offset].x][drones[offset].y].state = ' ';
			drones[offset].y--;
			map[drones[offset].x][drones[offset].y].state = '^';
			pthread_mutex_unlock(&mutex);
		}
		printf("\033[2J\033[1;1H");
		pthread_mutex_lock(&mutex);
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		} //end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
	}
	while(drones[offset].x != drones[offset].xdestination){
		if(drones[offset].x < drones[offset].xdestination){ // a is to the right
			pthread_mutex_lock(&mutex);
			if(map[drones[offset].x+1][drones[offset].y].state == 'T'||map[drones[offset].x+1][drones[offset].y].state == 'S'||map[drones[offset].x+1][drones[offset].y].state == '^'){
				drones[offset].state = 4;
				pthread_mutex_unlock(&mutex);
				avoidobstacle(offset, 2,1);
			}
			map[drones[offset].x][drones[offset].y].state = ' ';
			drones[offset].x++;
			map[drones[offset].x][drones[offset].y].state = '^';
			pthread_mutex_unlock(&mutex);
		}
		if(drones[offset].x > drones[offset].xdestination){ // a is to the left
			pthread_mutex_lock(&mutex);
			if(map[drones[offset].x-1][drones[offset].y].state == 'T'||map[drones[offset].x-1][drones[offset].y].state == 'S'||map[drones[offset].x-1][drones[offset].y].state == '^'){
				drones[offset].state = 4;
				pthread_mutex_unlock(&mutex);
				avoidobstacle(offset, 4,1);
			}
			map[drones[offset].x][drones[offset].y].state = ' ';
			drones[offset].x--;
			map[drones[offset].x][drones[offset].y].state = '^';
			pthread_mutex_unlock(&mutex);
		}
		pthread_mutex_lock(&mutex);
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
	}
	land(offset);
	drones[offset].state = 7;
	pthread_exit((void*) 0 );
}
void assigntasks(){ //assign destinations to each drone
	for(int i = 0; i<packagecounter; i++){
		drones[i].xdestination = packages[i].x;
		drones[i].ydestination = packages[i].y;
		drones[i].x = XSTATION+1;//starting point for drone
		drones[i].y = YSTATION;
		drones[i].state = 1;
		takeoffqueue.push(&drones[i]);
	}
}
void *takeoff(void *arg){//drones take off when it is their turn, this goes in the pthread
	long offset;
	offset = (long)arg;
	while(&drones[offset]!=(takeoffqueue.front())){sleep(500000);}//only takeoff if in front of queue
		takeoffqueue.front()->state = 3;
		sleep(3000000);
		pthread_mutex_lock(&mutex);
		map[takeoffqueue.front()->x][takeoffqueue.front()->y].state = '^';//takeoff
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		} //end print map loop
		pthread_mutex_unlock(&mutex);
		takeoffqueue.pop();
		sleep(100000);
	//}
	void navigate(long offset);
	navigate(offset);
	pthread_exit((void*) 0 );
}
void droppackage(long &off){
	drones[off].xdestination = XSTATION+1;//reset destination
	drones[off].ydestination = YSTATION;
}
void avoidobstacle(long offset,int direction,int land){ //0 navigate , 1 navigate back
//goes around any obstacle as well as checks for new obstacles.
//direction 1 means obstacle is up, 2 right, 3 down, 4 up
	if(direction == 2&&drones[offset].y-1!=-1){
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y--;//up
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x++;//right
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x++;//right
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y++;//down
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		land == 0 ? navigate(offset): navigateback(offset);
	}
	if(direction == 2&&drones[offset].y+1!=50){
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y++;//down
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x++;//right
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x++;//right
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y--;//down
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		land == 0 ? navigate(offset): navigateback(offset);
	}
	if(direction == 4&&drones[offset].y+1!=50){
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y++;//down
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x--;//left
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x--;//left
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y--;//up
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		land == 0 ? navigate(offset): navigateback(offset);
	}
	if(direction == 4&&drones[offset].y-1!=-1){
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y--;//up
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x--;//left
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x--;//left
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y++;//down
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		land == 0 ? navigate(offset): navigateback(offset);
	}
	if(direction == 1&&drones[offset].x-1!=-1){
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x--;//left
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y--;//up
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y--;//up
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x++;//right
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		land == 0 ? navigate(offset): navigateback(offset);
	}
	if(direction == 1&&drones[offset].x+1!=50){
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x++;//right
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y--;//up
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y--;//up
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x--;//left
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		land == 0 ? navigate(offset): navigateback(offset);
	}
	if(direction == 3&&drones[offset].x+1!=50){
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x++;//right
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y++;//left
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y++;//left
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x--;//up
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		land == 0 ? navigate(offset): navigateback(offset);
	}
	if(direction == 3&&drones[offset].x-1!=-1){
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x--;//left
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y++;//down
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].y++;//down
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		pthread_mutex_lock(&mutex);
		map[drones[offset].x][drones[offset].y].state = ' ';
		drones[offset].x++;//right
		map[drones[offset].x][drones[offset].y].state = '^';
		printf("\033[2J\033[1;1H");
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		}//end print map loop
		pthread_mutex_unlock(&mutex);
		sleep(500000);
		land == 0 ? navigate(offset): navigateback(offset);
	}
}
void land(long offset){//lands the drones
	pthread_mutex_lock(&mutex);
	map[drones[offset].x][drones[offset].y].state = ' ';
	map[XSTATION][YSTATION].state = 'S';
		for(int y = 0; y<YMAP; y++){//print map loop
			for(int x = 0; x<XMAP; x++){
				//std::cout<<a[x][y];
				std::cout<<map[x][y].state;
			}
			std::cout<<std::endl;
		} //end print map loop
	pthread_mutex_unlock(&mutex);
}
