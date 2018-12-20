#include <khepera/khepera.h>
#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#define PLIK_ENK "enkoders.txt"
#define PLIK_US "us_readings.txt"
#define PLIK_IR "ir_readings.txt"
#define PLIK_XY "pozycjaxy.txt"
//#define DEBUG 1

static knet_dev_t * dsPic; // robot pic microcontroller access
int maxsp,accinc,accdiv,minspacc, minspdec; // for speed profile
static int quitReq = 0; // quit variable for loop
/*--------------------------------------------------------------------*/
/*!
 * Make sure the program terminate properly on a ctrl-c
 */
static void ctrlc_handler( int sig )
{
  quitReq = 1;
  kh4_set_speed(0 ,0 ,dsPic); // stop robot
  kh4_SetMode( kh4RegIdle,dsPic );
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy
  kb_change_term_mode(0); // revert to original terminal if called
  exit(0);
}

#define BR_IRGAIN 5
#define fwSpeed 150//150
#define BMIN_SPEED 10
#define BMAX_SPEED 200
#define RotSpeedL 50
#define RotSpeedR 10
#define MAX_DIST 500
#define MIN_DIST 80 // 70
#define IMOBIL_SENS 250
#define IMOBIL_POS 30.0/KH4_PULSE_TO_MM
#define PI 3.14159265358979

int sl, sr; //predkosci lewego i prawego
//zmienne do odometrii
int pos_left_prev;
int pos_right_prev;
float result_x;
float result_y;
float result_theta;
int pos_left,pos_right;
float wheel_distance;
float wheel_conversion_left;
float wheel_conversion_right;
int acceleration_step;
float speed_max;
float speed_min;
float goal_x;
float goal_y;
int speed_left_internal;
int speed_right_internal;
int r_speed_left;
int r_speed_right;
int closetogoal;
int veryclosetogoal;
int atgoal;
float goal_theta, diff_theta;
//zmienne
int mapka[3][3];
short sensors[8]; // sensory IR
short usvalues[5]; // sensory Ultradzwiekowe

void odometria_init()
{
	//Odometria - domyslne dane
	wheel_distance = 0.10470;
	wheel_conversion_left = 0.13194 / 19456.0; // perimeter over pulses per rev
	wheel_conversion_right = 0.13194 / 19456.0;
    acceleration_step = 2.;
    speed_min = 10.;
    speed_max = 150.;
    goal_x = 0.;
    goal_y = 0.;
    speed_left_internal = 0;
    speed_right_internal = 0;
    r_speed_left = 0;
    r_speed_right = 0;
    atgoal = 1;
    closetogoal = 1;
    veryclosetogoal = 1;

    float x,y;
    printf("Podaj pozycje docelowa x):\n");
    scanf("%f", &x);
    printf("\n");
    printf("Podaj pozycje docelowa y):\n");
    scanf("%f", &y);
    printf("Podaj theta docelowe:\n");
    scanf("%f", &goal_theta);
    printf("\n");
   //SET GOAL
   goal_x = x;
   goal_y = y;
   atgoal = 0;
   closetogoal = 0;
   veryclosetogoal = 0;
}

void odometria()
{
	long delta_pos_left, delta_pos_right;
	float delta_left, delta_right, delta_theta, theta2;
	float delta_x, delta_y;

		kh4_get_position(&pos_left,&pos_right,dsPic);
		delta_pos_left = pos_left - pos_left_prev;
		delta_pos_right = pos_right - pos_right_prev;
		delta_left = delta_pos_left * wheel_conversion_left;
		delta_right = delta_pos_right * wheel_conversion_right;
		delta_theta = (delta_left - delta_right) / wheel_distance;
		theta2 = result_theta + delta_theta * 0.5;
		delta_x = (delta_left + delta_right) * 0.5 * cosf(theta2);
		delta_y = (delta_left + delta_right) * 0.5 * sinf(theta2);
		result_x += delta_x;
		result_y += delta_y;
		result_theta += delta_theta;
		//if (result_theta >= 6.2831853072){result_theta=0;}
		//timestamp = khepera4_current_time();
		pos_left_prev = pos_left;
		pos_right_prev = pos_right;
}
void odometria_goto()
{
	       //STEP
	       float dx, dy;
	       float distance, goalangle, alpha;
	       float speedfactor;
	       long speed_left_wish, speed_right_wish;
	       int atmaxspeed = 0;
	       // Do nothing if we are at goal
	       if (atgoal != 0)
	       {
	    	   kh4_SetRGBLeds(0,5,0,0,5,0,0,5,0,dsPic);
	    	   printf("U CELU \n");
	    	   return;
	       }
	       // Calculate new wish speeds
	       dx = goal_x - result_x;
	       dy = goal_y - result_y;
	       distance = sqrt(dx * dx + dy * dy);
	       goalangle = atan2(dy, dx);
	       alpha = goalangle - result_theta;
	       while (alpha > PI)
	       {
	    	   alpha -= 2 * PI;
	       }
	       while (alpha < -PI)
	       {
	           alpha += 2 * PI;
	       }

	       // Calculate the speed factor
	       speedfactor = (distance + 0.05) * 10. * speed_max;
	       if (speedfactor > speed_max)
	       {
	           speedfactor = speed_max;
	           atmaxspeed = 1;
	       }
	        // Calculate the theoretical speed
	        printf("dist %f - goalangle %f - alpha %f \n", distance, goalangle, alpha);
	        speed_left_wish = speedfactor * (PI - 2 * abs(alpha) + alpha) / PI + 0.5;
	        speed_right_wish = speedfactor * (PI - 2 * abs(alpha) - alpha) / PI + 0.5;

	        // Close to termination condition: just stop
	        if (veryclosetogoal)
	        {
	        	speed_left_wish = 0;
	            speed_right_wish = 0;
	        }

	        // Limit acceleration
	        if (speed_left_wish > speed_left_internal)
	        {
	        	speed_left_internal += acceleration_step;
	        }
	        if (speed_left_wish < speed_left_internal)
	        {
	            speed_left_internal -= acceleration_step;
	        }
	        if (speed_right_wish > speed_right_internal)
	        {
	            speed_right_internal += acceleration_step;
	        }
	        if (speed_right_wish < speed_right_internal)
	        {
	            speed_right_internal -= acceleration_step;
	        }

	        // Don't set speeds < MIN_SPEED (for accuracy reasons)
	        r_speed_left = speed_left_internal;
	        r_speed_right = speed_right_internal;
	        if (abs(r_speed_left) < speed_min)
	        {
	        	r_speed_left = 0;
	        }
	        if (abs(r_speed_right) < speed_min)
	        {
	            r_speed_right = 0;
	        }
	        // Termination condition
	        if (atmaxspeed == 0)
	        {
	            closetogoal = 1;
	            if ((r_speed_left == 0) || (r_speed_right == 0))
	            {
	            	veryclosetogoal = 1;
	            	   //atgoal = 1;
	            }
	            if ((r_speed_left == 0) && (r_speed_right == 0))
	            {
	            	atgoal = 1;
	            }
	        }
}

void skret_lewo()
{
	sl=-150;
	sr= 150;
	kh4_set_speed(0, 0, dsPic);
	kh4_set_speed(sl,sr, dsPic);
	kh4_SetRGBLeds(8,0,0,0,0,0,0,0,0,dsPic);
}
void skret_prawo()
{
	sl=150;
	sr=-150;
	kh4_set_speed(0, 0, dsPic);
	kh4_set_speed(sl,sr, dsPic);
	kh4_SetRGBLeds(0,0,0,8,0,0,0,0,0,dsPic);
}
int behavioral()
{
	int i;
	int ii,jj;
	int pred;
	char Buffer[256];
	//char Buffer2[256];
	char Buffer3[256];
	char Buffer4[256];
	int lpos,rpos;
	FILE *enktxt;
	enktxt = fopen(PLIK_ENK, "w");
	FILE *ustxt;
	ustxt = fopen(PLIK_US, "w");
	FILE *irtxt;
	irtxt = fopen(PLIK_IR, "w");
	FILE *xytxt;
	xytxt = fopen(PLIK_XY, "w");
	accinc=10;//3;
	accdiv=0;
	minspacc=20;
	minspdec=1;
	maxsp=400;
	kh4_SetSpeedProfile(accinc,accdiv,minspacc, minspdec,maxsp,dsPic ); // Acceleration increment ,  Acceleration divider, Minimum speed acc, Minimum speed dec, maximum speed
	kh4_SetMode(kh4RegSpeedProfile,dsPic );
	//mini mapka punktów
	for(ii=0;ii<3;ii++)
	{
		for(jj=0;jj<3;jj++)
		{
			mapka[ii][jj]=0;
		}
	}
	for(ii=0;ii<3;ii++)
	{
		for(jj=0;jj<3;jj++)
		{
			printf("%d  ",mapka[ii][jj]);
		}
		printf("\n");
	}
	printf("Podaj predkosc:\n");
	scanf("%d", &pred);
	//kh4_set_speed(-pred, pred, dsPic);
	//usleep(200000);
	sr=sl=pred;
	//inicjalizacja odometrii
	kh4_get_position(&pos_left,&pos_right,dsPic);
	result_x = 0;
	result_y = 0;
	result_theta = 0;
	//timestamp = 0;
	pos_left_prev = pos_left;
	pos_right_prev = pos_right;
	//kh4_set_speed(sl, sr, dsPic);
	odometria_init(); //inicjalizacja odometrii
	while(!kb_kbhit())
    {
		kb_clrscr();
		odometria();
		odometria_goto();
		//Wyswietla % baterii
		kh4_battery_status(Buffer3,dsPic);
		printf("Bateria: %3d %%\n",Buffer3[3]);
		//Wyswietla aktualna pozycje robota
		printf("Aktualna pozycja robota: x: %.3f  y: %.3f  theta: %.4f \n",result_x, result_y, result_theta);
		//printf("pozycja: lewe - %d  prawe - %d",  pos_left_prev, pos_right_prev);
		fprintf(xytxt,"%.4f %.4f %.4f\n", result_x, result_y, result_theta);
		float radians=result_theta;
		float degrees=radians * 180.0 / M_PI;
		printf("Kat robota %.4f \n", degrees);// kat w stopniach
		//get encoders
		kh4_get_position(&lpos,&rpos,dsPic);
		//printf("\n Enkodery: left %7d | right %7d \n",lpos,rpos);
		fprintf(enktxt,"%7d %7d \n", lpos, rpos);
		/*if (atgoal != 0)
		{
			printf("U CELU \n");
			break;
		}*/
		// pomiar czujników podczerwieni
		kh4_proximity_ir(Buffer, dsPic);
        for (i=0;i<12;i++)
		{
			sensors[i]=(Buffer[i*2] | Buffer[i*2+1]<<8);
        }
        //printf("Czujniki bl: %4u l: %4u fl: %4u f: %4u fr: %4u r: %4u br: %4u b: %4u\n", sensors[0], sensors[1], sensors[2], sensors[3], sensors[4], sensors[5], sensors[6], sensors[7]);
        fprintf(irtxt,"bl: %4u l: %4u fl: %4u f: %4u fr: %4u r: %4u br: %4u b: %4u\n", sensors[0], sensors[1], sensors[2], sensors[3], sensors[4], sensors[5], sensors[6], sensors[7]);

        for(i=0;i<8;i++)
        {
        	if (i==0) { ii=2; jj=0;}
        	else if (i==1) {ii=1; jj=0;}
        	else if (i==2) {ii=0; jj=0;}
        	else if (i==3) {ii=0; jj=1;}
        	else if (i==4) {ii=0; jj=2;}
        	else if (i==5) {ii=1; jj=2;}
        	else if (i==6) {ii=2; jj=2;}
        	else if (i==7) {ii=2; jj=1;}
        	if (sensors[i]>200)
        	{
        		mapka[ii][jj]=1;
        	}
        	else{
        		mapka[ii][jj]=0;
        	}
        }
        for(ii=0;ii<3;ii++)
        {
        	for(jj=0;jj<3;jj++)
        	{
        		printf("%d  ",mapka[ii][jj]);
        	}
        	printf("\n");
        }
       //kh4_activate_us(0,dsPic); //wyl. ultradzwiekowe
        //CZUJNIKI ULTRADZWIEKOWE
        /*
        kh4_measure_us(Buffer2,dsPic);

        for (i=0;i<5;i++)
        {
        	usvalues[i] = (short)(Buffer2[i*2] | Buffer2[i*2+1]<<8);
        }
        //printf("Czujniki us \n l: %4d \n fl: %4d \n f: %4d\n fr: %4d \n r: %4d \n  ", usvalues[0], usvalues[1], usvalues[2], usvalues[3], usvalues[4]);
        fprintf(ustxt, "l: %4d fl: %4d \n f: %4d\n fr: %4d \n r: %4d \n  ", usvalues[0], usvalues[1], usvalues[2], usvalues[3], usvalues[4]);
		*/

        /*
        if (sensors[3]>300)
        {
        	kh4_set_speed(0, 0, dsPic);

        	kh4_set_speed(-50, 50, dsPic);
        	//kh4_ResetEncoders(dsPic);
        	//usleep(200000);
        	//kh4_SetMode(kh4RegPosition,dsPic );
        	//kh4_set_position(sl, sr, dsPic);
        	//kh4_SetMode(kh4RegSpeedProfile,dsPic );

        }
        */
        /*
        if (usvalues[2] < 40)
        {
        	sl=sr=pred;
        	kh4_set_speed(sl, sr, dsPic);
        }
        else
		{
        	//kh4_set_speed(pred, pred, dsPic);
		*/

//OMIJANIE PRZESZKOD

        	/*if (sensors[3]>=300)
        	        {
        	        	kh4_set_speed(0,0, dsPic);
        	        	usleep(20000);
        	        	kh4_set_speed(50,0,dsPic);

        	        }
        	        else */
//NIE BRAITENBERG START
        			if (sensors[3] > 200 && sensors[4] >200 && sensors[2] > 200)
        	        {
        	        	sl=sr=-100;
        	        	kh4_set_speed(0, 0, dsPic);
        	        	kh4_set_speed(sl,sr, dsPic);
        	        }
        	        else if (sensors[3] > 300 || sensors[2] > 200) //przeszkoda lewo i przod
        	        {
        	        	skret_prawo();

        	        	//kh4_set_speed(100,-100, dsPic);
        	        	//kh4_SetRGBLeds(1,0,0,0,0,0,0,0,0,dsPic);
        	        }
        	        else if (sensors[3] > 300 || sensors[4] > 200) //przeszkoda prawo i przod
        	        {
        	           skret_lewo();
        	        	//kh4_set_speed(-100,100, dsPic);
        	        	//kh4_SetRGBLeds(0,0,0,24,0,0,0,0,0,dsPic);
        	        }
        	        else if (sensors[1] > 300) //przeszkoda tylko po lewej
        	        {
        	        	kh4_set_speed(150,148,dsPic);
        	        	kh4_SetRGBLeds(0,0,8,0,0,0,0,0,0,dsPic);
        	        }
        	        else if (sensors[5] > 300) //przeszkoda tylko po prawej
        	        {
        	        	kh4_set_speed(148,150,dsPic);
        	        	kh4_SetRGBLeds(0,0,0,0,0,8,0,0,0,dsPic);

        	        }
        	        //dobre
//NIE BRAITENBERG END

        else
        {
        	//usleep(200000);
        	kh4_SetRGBLeds(0,0,0,0,0,0,0,8,8,dsPic);
        	//sl=sr=pred;
        	//kh4_set_speed(sl, sr, dsPic);
        	kh4_set_speed(r_speed_left,r_speed_right, dsPic); //predkosci wyliczone z odometrii celu

//OBRACANIE DO KĄTA
        	/*
        	 * float dom_theta=0;
        	if (degrees>degrees2+0.7)
        	{
        		kh4_set_speed(sl-50, sr, dsPic);
        	}
        	else if (degrees<degrees2-0.7)
        	{
        		kh4_set_speed(sl,sr-50,dsPic);
        	}
        	else //if (result_theta==dom_theta)
        	{
        		kh4_set_speed(0, 0, dsPic);
        		sleep(0.9);
        		kh4_set_speed(sl, sr, dsPic);
        	} */

		//}
        }
        printf("\nNacisnij klawisz aby zatrzymac\n");
        usleep(20000);
        }
    //tcflush(0, TCIFLUSH); // flush input
    kh4_set_speed(0,0,dsPic ); // stop robot
    kh4_SetMode( kh4RegIdle,dsPic ); // set motors to idle
    accinc=3;//3;
    accdiv=0;
    minspacc=20;
    minspdec=1;
    maxsp=400;
    // configure acceleration slope
    kh4_SetSpeedProfile(accinc,accdiv,minspacc, minspdec,maxsp,dsPic ); // Acceleration increment ,  Acceleration divider, Minimum speed acc, Minimum speed dec, maximum speed
    fclose(enktxt);
    fclose(irtxt);
    fclose(ustxt);
    fclose(xytxt);
    return 0;
}

int main(int argc, char * argv[])
{

#define MAX_US_DISTANCE 250.0 // max distance US
#define MAX_G 2 		// max acceleration in g

  //double fpos,dval,dmean;
  //long lpos,rpos;
  char Bufer[100];
  //char bar[12][64];
  char revision,version;
  //int i,n,type_of_test=0,sl,sr,pl,pr;
  //short index, value,sensors[12],usvalues[5];
  //char c;
  //long motspeed;
  //char line[80],l[9];
  int kp,ki,kd;
  int pmarg;

// initiate libkhepera and robot access
 if ( kh4_init(argc ,argv)!=0)
 {
	 printf("\nERROR: could not initiate the libkhepera!\n\n");
	 return -1;
 }
/* open robot socket and store the handle in their respective pointers */
 dsPic  = knet_open( "Khepera4:dsPic" , KNET_BUS_I2C , 0 , NULL );
 if ( dsPic==NULL)
  {
  	printf("\nERROR: could not initiate communication with Kh4 dsPic\n\n");
  	return -2;
  }
  /* initialize the motors controlers*/
  /* tuned parameters */
  pmarg=20;
  kh4_SetPositionMargin(pmarg,dsPic ); 				// position control margin
  kp=10;
  ki=5;
  kd=1;
  kh4_ConfigurePID( kp , ki , kd,dsPic  ); 		// configure P,I,D
  accinc=3;//3;
  accdiv=0;
  minspacc=20;
  minspdec=1;
  maxsp=400;
  // configure acceleration slope
  kh4_SetSpeedProfile(accinc,accdiv,minspacc, minspdec,maxsp,dsPic ); // Acceleration increment ,  Acceleration divider, Minimum speed acc, Minimum speed dec, maximum speed
  kh4_ResetEncoders(dsPic);
  kh4_SetMode( kh4RegIdle,dsPic );  // Put in idle mode (no control)

  // get revision
  if(kh4_revision(Bufer, dsPic)==0){
   	version=(Bufer[0]>>4) +'A';
  	revision=Bufer[0] & 0x0F;
    printf("\r\nVersion = %c, Revision = %u\r\n",version,revision);
  }

  signal( SIGINT , ctrlc_handler ); // set signal for catching ctrl-c
  /* For ever loop until ctrl-c key */
  //while(quitReq==0)
  //{
      behavioral(); //funkcja glowna(?)
    //  tcflush(0, TCIFLUSH); //?????
    //  break;
  //}
//zakończenie działania
  kh4_set_speed(0 ,0 ,dsPic); // stop robot
  kh4_SetMode( kh4RegIdle,dsPic ); // set motors to idle
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy

  return 0;
}


