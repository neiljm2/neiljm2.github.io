// TITLE:  Final Project NM CE JY RW
//Code commented by TEAM initials: NM CE JY RW
//-----------------------MAIN------------------------//
#include "FinalProject_NM_CE_JY_RW.h"

//Final proj student vars
float Vavg = 0;
float Vavgprev = 0;
int8_t bindleFlag = 0;
float bindleRad = 1.0;
float distFromStart = 0;
//obs vars
#define MAPSIZE1 101
#define UNIT_MAP1_LEN_INV 3
#define MAPSIZE2 27 //
#define UNIT_MAP2_LEN_INV 0.801980198 //(101/(3*27))^-1
int16_t map[MAPSIZE1][MAPSIZE1] = {0};

#define BUFFER_SIZE 1024 //NM power of 2, size of buffer for labview data
#define BUFFER_MASK (BUFFER_SIZE - 1)

typedef struct {
    int16_t data[BUFFER_SIZE];
    uint16_t head; //NM next value to be popped
    uint16_t tail; //NM end of buffer values
} RingBuffer;

RingBuffer LVbuffer; //NM create the LV buffer

int16_t obs_idx_x;
int16_t obs_idx_y;

int16_t ball_idx1_x;
int16_t ball_idx1_y;
int16_t swi2 = 0;
long int swi1 = 0;

int16_t firstCase1 = 1;
int arrivedxy = 0;
int arrivedtheta = 0;

int16_t rob_idx2_x = 0;
int16_t rob_idx2_y = 0;

int16_t visitedmap[MAPSIZE2][MAPSIZE2] = {0};
int16_t xvisit = 0;
int16_t yvisit = 0;

int16_t minX1, maxX1, minY1, maxY1;
int16_t minX2, maxX2, minY2, maxY2;

float Vx = 0.0;
float Vy = 0.0f;
int16_t obs_dx = 0;
int16_t obs_dy = 0;
float obs_dist_squared = 0.0;
int16_t ball_dx = 0;
int16_t ball_dy = 0;
float ball_dist_squared = 0.0;

float theta_d;
float thetaD;
float theta_err;
float f_v;

// Waypoint and Target Logic
float xdiff = 0.0f;
float ydiff = 0.0f;
float dist_ball_goal = 0.0f;
float waypoint_x = 0.0f;
float waypoint_y = 0.0f;
int16_t searchTargetX;
int16_t searchTargetY;
bool test_wpt_flag = 0;
int16_t test_wpt_ctr = 0;
float waypointDist;


//kicking
int16_t kickState = 0; //start @ 1
int16_t kickWaitCnt = 0;
int16_t kickTest = 0;
int16_t storeOBScntr = 0;

float Vavg1 = 0.0;
float Vavg2 = 0.0;



void detectBall(void);
void detectAprilTag(void);
void captureBall(void);
void go_to_April(void);

// ball detection variables
float kpvision = 0.15;
float colcentroid = 0.0;
float ballDist1 = 0.0;    // IMPORTANT: SET THIS USING THE FORMULA in SWI or wherever the code is running - yuan41         ballDist1 = 264.2167 - 6.144*MaxRowThreshold1 + 0.0523*MaxRowThreshold1*MaxRowThreshold1 - 0.00015432*MaxRowThreshold1*MaxRowThreshold1*MaxRowThreshold1;
// ball detection flags
int ball_in_view = 0;
int ball_centered = 0;
int ball_position_recorded = 0;
int cover_cntr = 0;



// april tag detection variables
#define CENTER_X 56.5 // the tagx value (mm) when apriltag is straight in front of robot  - yuan41
#define TAG_CENTER_THRESHOLD 20.0 // how many mm of horizontal variation for the robot to still count the tag as centered and then record the distance  - yuan41

float goal_x_min = CENTER_X - TAG_CENTER_THRESHOLD;
float goal_x_max = CENTER_X + TAG_CENTER_THRESHOLD;
float goalDist = 0.0;  // IMPORTANT: set this to goalDist = tagz in SWI or wherever the code is running - yuan41
float kptag = 0.01;  // EDIT THIS

// april tag detection flags
int goal_in_view = 0;
int tag_centered = 0;
int goal_position_recorded = 0;

int capture_ball_cntr = 0;
float capture_time = 0.0;
int centered_first = 0;


//xy detectBall() {
//    // Create a local instance of the struct to hold the result
//    xy ball_location;
//
//    // Check your sensor data (LADAR, Camera, etc.)
//    if (ball_is_in_view == true) {
//        // Populate with the real coordinates
//        ball_location.x = detected_x_coord;
//        ball_location.y = detected_y_coord;
//    }
//    else {
//        // Return "Neutral" values if the ball is missing
//        ball_location.x = 0.0;
//        ball_location.y = 0.0;
//    }
//
//    return ball_location;
//}
void setWaypoint3(void);
void deadReckon(void);
void processLADAR(void);
void checkBindle(void);
void storeOBS(void);
void clearPath(int x0, int y0, int x1, int y1);
void rightWallFollow(void);
void findminmax_xy(void);
void setwaypoint1(void);
void nav1(void);
void setWaypoint2(void);
void nav2(void);
void constrainwaypointidx(void);
void turnToGoal(void);
void rb_push(int16_t x, int16_t y);
void storeEXPLORED_LOCATIONS(void);
//void kick(void);

void main(void)
{
    ALL_INIT();
    while(1)
    {
        if (UARTPrint == 1 ) {
            if (readbuttons() == 0) {
//                UART_printfLine(1,"S %ld A %ld",swi1,numADCCcalls);
                UART_printfLine(1,"s %d xy %d th %d",RobotState,arrivedxy,arrivedtheta);
                //UART_printfLine(1,"x:%.2f:y:%.2f:a%.2f",ROBOTps.x,ROBOTps.y,ROBOTps.theta);
                UART_printfLine(2,"turn %.2f vref %.2f", turn, vref);
//                UART_printfLine(2,"F%.4f R%.4f",LADARfront,LADARrightfront);
            } else if (readbuttons() == 1) {
                UART_printfLine(1,"O1A:%.0fC:%.0fR:%.0f",MaxAreaThreshold1,MaxColThreshold1,MaxRowThreshold1);
                UART_printfLine(2,"P1A:%.0fC:%.0fR:%.0f",MaxAreaThreshold2,MaxColThreshold2,MaxRowThreshold2);
                //UART_printfLine(1,"LV1:%.3f LV2:%.3f",printLV1,printLV2);
                //UART_printfLine(2,"Ln1:%.3f Ln2:%.3f",printLinux1,printLinux2);
            } else if (readbuttons() == 2) {
                UART_printfLine(1,"O2A:%.0fC:%.0fR:%.0f",NextLargestAreaThreshold1,NextLargestColThreshold1,NextLargestRowThreshold1);
                UART_printfLine(2,"P2A:%.0fC:%.0fR:%.0f",NextLargestAreaThreshold2,NextLargestColThreshold2,NextLargestRowThreshold2);
                // UART_printfLine(1,"%.2f %.2f",adcC2Volt,adcC3Volt);
                // UART_printfLine(2,"%.2f %.2f",adcC4Volt,adcC5Volt);
            } else if (readbuttons() == 4) {
                UART_printfLine(1,"O3A:%.0fC:%.0fR:%.0f",NextNextLargestAreaThreshold1,NextNextLargestColThreshold1,NextNextLargestRowThreshold1);
                UART_printfLine(2,"P3A:%.0fC:%.0fR:%.0f",NextNextLargestAreaThreshold2,NextNextLargestColThreshold2,NextNextLargestRowThreshold2);
                // UART_printfLine(1,"L:%.3f R:%.3f",LeftVel,RightVel);
                // UART_printfLine(2,"uL:%.2f uR:%.2f",uLeft,uRight);
            } else if (readbuttons() == 8) {
                UART_printfLine(1,"020x%.2f y%.2f",ladar_pts[20].x,ladar_pts[20].y);
                UART_printfLine(2,"150x%.2f y%.2f",ladar_pts[150].x,ladar_pts[150].y);
            } else if (readbuttons() == 3) {
                UART_printfLine(1,"Vrf:%.2f trn:%.2f",vref,turn);
                UART_printfLine(2,"MPU:%.2f LPR:%.2f",gyro9250_radians,gyroLPR510_radians);
            } else if (readbuttons() == 5) {
                UART_printfLine(1,"Ox:%.2f:Oy:%.2f:Oa%.2f",OPTITRACKps.x,OPTITRACKps.y,OPTITRACKps.theta);
                UART_printfLine(2,"State:%d : %d",RobotState,statePos);
            } else if (readbuttons() == 6) {
                UART_printfLine(1,"D1 %ld D2 %ld",dis_1,dis_2);
                UART_printfLine(2,"St1 %ld St2 %ld",measure_status_1,measure_status_2);
            } else if (readbuttons() == 7) {
                UART_printfLine(1,"%.0f,%.1f,%.1f,%.1f",tagid,tagx,tagy,tagz);
                UART_printfLine(2,"%.1f,%.1f,%.1f",tagthetax,tagthetay,tagthetaz);
            }
            UARTPrint = 0;
        }
    }
}

// Connected to PIEIER12_9 (use MINT12 and MG12_9 masks):
__interrupt void SWI1_HighestPriority(void)     // EMIF_ERROR
{
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_9;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    swi1_before_sm();

    if (calibration_state == 3){
        //--------------------------------------------------------------------------------------------------------\\
        //--------------------------------------------------------------------------------------------------------\\
        //--------------------------------------------------------------------------------------------------------\\

        //        CODE THAT IS ALWAYS CALLED//////////////////////////////////////////////////////////////////
        deadReckon();
        rob_idx1_x = (int)floor(ROBOTps.x*UNIT_MAP1_LEN_INV)+(MAPSIZE1-1)/2;
        rob_idx1_y = -(int)floor(ROBOTps.y*UNIT_MAP1_LEN_INV)+(MAPSIZE1-1)/2;
        //////////////////////////////////////////////////////////////////////////////////////////////
        // state machine
//TODO

        switch (RobotState) {
        case 0:
            // STATE 0: Initialization and Sensing /////////////////////////////////
            rightWallFollow();        // C: Determine Vref and Turn
            detectAprilTag();                 // E: Camera XY coordinates
            detectBall();                  // F: Ball detection
            checkBindle();                      // G: Update bindleflag and XY

            // Transition Logic
            if ((bindleFlag == 2) && ball_position_recorded){RobotState = 10;}
            if ((bindleFlag == 2) && !ball_position_recorded){RobotState = 1;}
            break;

        case 1:
            if (firstCase1) {
                vref = 0.0;
                turn  = 0.0;
            }
            if (!firstCase1){
                setwaypoint1(); // Set target Xd, Yd
//                if (test_wpt_flag == 0){waypoint.x = 70, waypoint.y = 40;} //NM test nav
//                if (test_wpt_flag == 1){waypoint.x = 50, waypoint.y = 50;}
                nav1();
                detectAprilTag();
                detectBall();
                if (ball_position_recorded && goal_position_recorded) {RobotState = 10;}
            }
            break;

        case 10:
            // STATE 10: Goal Alignment //////////////////////////////////////////
            if (firstCase1) {
                vref = 0.0;
                turn = 0.0;
            }
            if (!firstCase1){
                setWaypoint2(); // Get target Xd, Yd, Theta_d
                nav2(); // go to behind ball and face ball and goal
                if (arrivedtheta) {RobotState = 20;} //NM flag at end of nav2
            }
            break;

        case 20:
            // STATE 20: Ball Acquisition  ////////////////////////////////////////////
            captureBall();
            if (capture_ball_cntr > capture_time){
                RobotState = 21;
                vref = 0;
                turn = 0;
            }
            break;

        case 21:
            // STATE 21: going to goal ///////////////////////////////////////////////////
            if (cover_cntr <= 500){
                setEPWM5B_RCServo(-80); //NM pull down cover
                cover_cntr++;
            } else {
                setWaypoint3();  // set target to goal
                nav1();                  // Navigate to goal
//                go_to_April();
                goalDist = sqrt((ROBOTps.x - goal.x)*(ROBOTps.x - goal.x) + (ROBOTps.y - goal.y)*(ROBOTps.y - goal.y));
                if (goalDist < 3.0) {RobotState = 22; vref = 0; turn = 0;} //NM if close to goal, stop robot and change state
            }
            break;

        case 22:
            // STATE 22: Scoring ///////////////////////////////////////////////////
            turnToGoal(); // Align; set turn rate and Vref = 0
            if (abs(theta_err) < 0.025) {
                setEPWM5B_RCServo(0.0);
                setEPWM5A_RCServo(-50.0);
            } // FINAL ACTION

            break;
        }
        swi1_after_sm();
    }
    //--------------------------------------------------------------------------------------------------------\\
    //--------------------------------------------------------------------------------------------------------\\
    //--------------------------------------------------------------------------------------------------------\\

    timecount++;
    if((timecount%200) == 0)
    {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
        UARTPrint = 1; // Tell While loop to print
    }

    swi1++;

//    SpibRegs.SPIFFCT.bit.TXDLY = 16;
//    CurrentChip = DAN28027;
//    SpibRegs.SPIFFRX.bit.RXFFIL = 3;
//    GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
//    SpibRegs.SPITXBUF = 0x00DA;
//    SpibRegs.SPITXBUF = EPwm1A_F28027;
//    SpibRegs.SPITXBUF = EPwm2A_F28027;

    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
}

// Connected to PIEIER12_10 (use MINT12 and MG12_10 masks):
__interrupt void SWI2_MiddlePriority(void)     // RAM_CORRECTABLE_ERROR
{
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_10;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    //###############################################################################################
    // Insert SWI ISR Code here.......
    // LED1
    GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;
    if (LADARpingpong == 1) {
        // LADARrightfront is the min of dist 52, 53, 54, 55, 56
        LADARrightfront = 19; // 19 is greater than max feet
        for (LADARi = 52; LADARi <= 56 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARrightfront) {
                LADARrightfront = ladar_data[LADARi].distance_ping;
            }
        }
        // LADARfront is the min of dist 111, 112, 113, 114, 115
        LADARfront = 19;
        for (LADARi = 111; LADARi <= 115 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARfront) {
                LADARfront = ladar_data[LADARi].distance_ping;
            }
        }
        LADARxoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta));
        LADARyoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)+LADARps.y*cosf(ROBOTps.theta));
        for (LADARi = 0; LADARi < 228; LADARi++) {
            ladar_pts[LADARi].x = LADARxoffset + ladar_data[LADARi].distance_ping*cosf(ladar_data[LADARi].angle + ROBOTps.theta);
            ladar_pts[LADARi].y = LADARyoffset + ladar_data[LADARi].distance_ping*sinf(ladar_data[LADARi].angle + ROBOTps.theta);
            if (ladar_data[LADARi].distance_ping > 17.0){
                ladar_pts[LADARi].x = 0.0;
                ladar_pts[LADARi].y = 0.0;
            }
        }
    } else if (LADARpingpong == 0) {
        // LADARrightfront is the min of dist 52, 53, 54, 55, 56
        LADARrightfront = 19; // 19 is greater than max feet
        for (LADARi = 52; LADARi <= 56 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARrightfront) {
                LADARrightfront = ladar_data[LADARi].distance_pong;
            }
        }
        // LADARfront is the min of dist 111, 112, 113, 114, 115
        LADARfront = 19;
        for (LADARi = 111; LADARi <= 115 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARfront) {
                LADARfront = ladar_data[LADARi].distance_pong;
            }
        }
        LADARrightback = 19;
        for (LADARi = 20; LADARi <= 25 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARrightback) {
                LADARrightback = ladar_data[LADARi].distance_ping;
            }
        }
        LADARxoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta - PI/2.0));
        LADARyoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)-LADARps.y*cosf(ROBOTps.theta - PI/2.0));
        for (LADARi = 0; LADARi < 228; LADARi++) {
            ladar_pts[LADARi].x = LADARxoffset + ladar_data[LADARi].distance_pong*cosf(ladar_data[LADARi].angle + ROBOTps.theta);
            ladar_pts[LADARi].y = LADARyoffset + ladar_data[LADARi].distance_pong*sinf(ladar_data[LADARi].angle + ROBOTps.theta);
            if (ladar_data[LADARi].distance_pong > 17.0){
                ladar_pts[LADARi].x = 0.0;
                ladar_pts[LADARi].y = 0.0;
            }
        }
    }
    processLADAR();
    storeOBS(); //NM update grid

    storeEXPLORED_LOCATIONS();

    if (RobotState == 1){
        findminmax_xy(); //NM call minmax xy once after RWF
        rob_idx2_x = (int)floor(ROBOTps.x * UNIT_MAP2_LEN_INV) + (MAPSIZE2 - 1) * 0.5;
        rob_idx2_y = -(int)floor(ROBOTps.y * UNIT_MAP2_LEN_INV) + (MAPSIZE2 - 1) * 0.5;
        searchTargetX = rob_idx2_x;
        searchTargetY = rob_idx2_y;
        firstCase1 = 0;
    }
    if (RobotState == 10){
        findminmax_xy(); //NM call minmax xy once after RWF
        firstCase1 = 0;
    }
    //###############################################################################################
    //
    // Restore registers saved:
    //
    swi2++;
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;
}

// Connected to PIEIER12_11 (use MINT12 and MG12_11 masks):
__interrupt void SWI3_LowestPriority(void)     // FLASH_CORRECTABLE_ERROR
{
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_11;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    //###############################################################################################
    // Insert SWI ISR Code here.......

    //###############################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}

void deadReckon(){
    //integrate the IMU and wheel encoders to determine new position
    Vavg = (((LeftVel + RightVel)/2.0)+ Vavg1 + Vavg2)/3.0; //NM take avg velocity of both wheels
    ROBOTps.theta = gyro9250_radians; //NM gyro is already integrated
    ROBOTps.x = ROBOTps.x + cos(ROBOTps.theta)*((Vavg + Vavg1)/2)*0.001; //NM update x and y by integrating velocity
    ROBOTps.y = ROBOTps.y + sin(ROBOTps.theta)*((Vavg + Vavg1)/2)*0.001;
    Vavg2 = Vavg1;
    Vavg1 = Vavg;
}

void processLADAR(){
    //remove erroneous ladar points, like the two supports
    //OUTPUT: processedLADAR
    ladar_pts[0].x = 0.0;
    ladar_pts[0].y = 0.0;
    ladar_pts[227].x = 0.0;
    ladar_pts[227].y = 0.0;
}

void storeOBS(void){
    //update the map to show where obstacles are with confidence ratings.
    //each time an obstacle is spotted, raise its value, if an obstacle
    //that was previously spotted is not spotted, lower its value (remove
    //ghost obstacles)
    //OUTPUT: updated map

    const int16_t offset = (MAPSIZE1-1)/2;

    for (int i = 1; i < LADAR_LENGTH-1; i++) {
        //        if (ladar_pts[i].x < 0.001 && ladar_pts[i].x > -0.001){} //NM if the point is near 0, skip
        //compute obstacle indices
        obs_idx_x = (int)floor(ladar_pts[i].x*UNIT_MAP1_LEN_INV)+offset; //NM since we're defining each square in our grid by the bottom left corner, we need to floor the converted value so that the two ints we get are low x and low y, which corresponds to the bottom left corner. we then apply an offset because the array has 0,0 top left, but our grid coordinates have 0,0 in the middle
        obs_idx_y = -(int)floor(ladar_pts[i].y*UNIT_MAP1_LEN_INV)+offset;
        int16_t *hit_pointer = &map[obs_idx_y][obs_idx_x];
        if ((*hit_pointer <= 0) && (*hit_pointer >= -8)){ rb_push(obs_idx_y,obs_idx_x);} //NM if the sign will flip, send coords to LV
        *hit_pointer += 9; //IF hit, add 9

        //compute robot indices

        clearPath(rob_idx1_x, rob_idx1_y, obs_idx_x, obs_idx_y); //NM count misses between hit and robot, subtract 4 from those "misses"

        //NM constrain
        if (*hit_pointer  > 30) {*hit_pointer  = 30;}

        storeOBScntr++;
    }
}

void clearPath(int x0, int y0, int x1, int y1) {
    //NM count misses between hit and robot, subtract 4 from those "misses"
    int16_t dx = abs(x1 - x0), sx = (x0 < x1 ? 1 : -1);
    int16_t dy = -abs(y1 - y0), sy = (y0 < y1 ? 1 : -1);
    int16_t err = dx + dy, e2;

    while (x0 != x1 || y0 != y1) {
        if (x0 >= 0 && x0 < MAPSIZE1 && y0 >= 0 && y0 < MAPSIZE1){
            int16_t *miss_pointer = &map[y0][x0];
            if ((*miss_pointer >= 1) && (*miss_pointer <= 4)){ rb_push(y0,x0);} //NM if the sign will flip, send coords to LV
            *miss_pointer -= 4; //NM if miss, subtract 4
            if (*miss_pointer < -30) {*miss_pointer = -30;}
        }

        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void rb_push(int16_t x, int16_t y) {
    //NM check if there is room for 2 values
    uint16_t next_tail2 = (LVbuffer.tail + 2) & BUFFER_MASK;
    uint16_t next_tail1 = (LVbuffer.tail + 1) & BUFFER_MASK;

    //NM if both can fit, push both
    if (next_tail2 != LVbuffer.head && next_tail1 != LVbuffer.head) {
        LVbuffer.data[LVbuffer.tail] = x;
        LVbuffer.tail = next_tail1;
        LVbuffer.data[LVbuffer.tail] = y;
        LVbuffer.tail = (LVbuffer.tail + 1) & BUFFER_MASK;
    }
}

int16_t rb_pop(void) {
    if (LVbuffer.head == LVbuffer.tail) {
        return -1; //NM if buffer is empty, send invalid value
    }

    int16_t value = LVbuffer.data[LVbuffer.head];
    LVbuffer.head = (LVbuffer.head + 1) & BUFFER_MASK; //NM if buffer not empty, send head value and increment head with a mask to prevent overflow
    return value;
}

void rightWallFollow(void){
    //use LADAR data to RWF like in class
    //OUTPUT: vref, turn
    switch (right_wall_follow_state) {
    case 1:
        //Left Turn
        turn = -Kp_front_wall*(14.5 - LADARfront);
        vref = front_turn_velocity;
        if (LADARfront > left_turn_Stop_threshold) {
            right_wall_follow_state = 2;
        }
        break;
    case 2:
        //Right Wall Follow
        turn = -Kp_right_wall*(ref_right_wall - LADARrightfront);
        vref = forward_velocity;
        if (LADARfront < left_turn_Start_threshold) {
            right_wall_follow_state = 1;
        }
        break;
    }
    if (turn > turn_command_saturation) {turn = turn_command_saturation;}
    if (turn < -turn_command_saturation) {turn = -turn_command_saturation;}
}

void checkBindle(){
    //for boundary scouting. at the start, bindleFlag is 0. when the robot
    //leaves a certain radius of the start, set to 1. when the robot returns
    //to within a certain radius of the start, set to 2
    //OUPTUT: bindleFlag

    distFromStart = sqrt(ROBOTps.x*ROBOTps.x + ROBOTps.y*ROBOTps.y);
    if (bindleFlag == 0 && distFromStart > 1.05*bindleRad){ //NM 1.05 and 0.95 chosen to prevent sensor noise from triggering the flag
        bindleFlag = 1;
    }
    if (bindleFlag == 1 && distFromStart < 0.95*bindleRad){
        bindleFlag = 2;
    }
}

void setwaypoint1(void){
    rob_idx2_x = (int)floor(ROBOTps.x * UNIT_MAP2_LEN_INV) + (MAPSIZE2 - 1) / 2;
    rob_idx2_y = -(int)floor(ROBOTps.y * UNIT_MAP2_LEN_INV) + (MAPSIZE2 - 1) / 2;
    int16_t *visited_pointer = &visitedmap[rob_idx2_y][rob_idx2_x];

    if (*visited_pointer == 0) {*visited_pointer = 1;}

    if (searchTargetX == rob_idx2_x && searchTargetY == rob_idx2_y){
        int foundNext = 0;
        for (; yvisit <= maxY2; yvisit++){
            int startX = (yvisit == searchTargetY) ? xvisit : minX2;
            for (xvisit = startX; xvisit <= maxX2; xvisit++){
                if (visitedmap[yvisit][xvisit] == 0){
                    searchTargetX = xvisit;
                    searchTargetY = yvisit;
                    foundNext = 1;
                    float scaleUp = (float)(MAPSIZE1 - 1) / (MAPSIZE2 - 1);
                    waypoint.x = (int16_t)roundf(searchTargetX * scaleUp + 0.5);
                    waypoint.y = (int16_t)roundf(searchTargetY * scaleUp + 0.5);
                    break;
                }
            }
            if (foundNext) break;
        }
        if (!foundNext){
            searchTargetX = 50;
            searchTargetY = 50;
            waypoint.x = 50;
            waypoint.y = 50;
            yvisit = minY2;
            xvisit = minX2;

            test_wpt_flag = 1;
        }
    }
}

void storeEXPLORED_LOCATIONS(void){
    rob_idx2_x = (int)floor(ROBOTps.x*UNIT_MAP2_LEN_INV)+(MAPSIZE2-1)/2;
    rob_idx2_y = -(int)floor(ROBOTps.y*UNIT_MAP2_LEN_INV)+(MAPSIZE2-1)/2;
    visitedmap[rob_idx2_y][rob_idx2_x] = 1;
}

void findminmax_xy(void) {
    int l_minX = MAPSIZE2, l_maxX = -1;
    int l_minY = -1, l_maxY = -1;
    int x, y;

    for (y = 0; y < MAPSIZE2; y++) {
        for (x = 0; x < MAPSIZE2; x++) {
            if (visitedmap[y][x]) {
                l_minY = y;
                goto found_min_y;
            }
        }
    }
    return;

    found_min_y:
    for (y = MAPSIZE2 - 1; y >= l_minY; y--) {
        for (x = 0; x < MAPSIZE2; x++) {
            if (visitedmap[y][x]) {
                l_maxY = y;
                goto found_max_y;
            }
        }
    }
    found_max_y:


    for (y = l_minY; y <= l_maxY; y++) {

        for (x = 0; x < l_minX; x++) {
            if (visitedmap[y][x]) {
                l_minX = x;
                break;
            }
        }

        for (x = MAPSIZE2 - 1; x > l_maxX; x--) {
            if (visitedmap[y][x]) {
                l_maxX = x;
                break;
            }
        }
    }

    minX1 = l_minX + 1; maxX1 = l_maxX - 1;
    minY1 = l_minY + 1; maxY1 = l_maxY - 1;

    minX2 = l_minX + 1;
    maxX2 = l_maxX - 1;
    minY2 = l_minY + 1;
    maxY2 = l_maxY - 1;

    xvisit = minX2;
    yvisit = minY2;
}

void nav1(void){
    //weighted gradient descent algorithm to navigate around obstacles
    //OUTPUT: vref and turn
    Vx = 0;
    Vy = 0;
//TODO
//    minY1 = 35;
//    maxY1 = 55;
//    minX1 = 45;
//    maxX1 = 80;

    for (int16_t i = minY1; i < maxY1; i++){ //NM loop through each row of map
        for (int16_t j = minX1; j < maxX1; j++){ //NM loop through each value in row
            if (map[i][j] <= 0) {continue;} //NM if value below 0, no obstacle, so skip
//TODO bound obstacle repulsion, only look at close obs
            //add chris minmax code from email
            obs_dx = rob_idx1_x - j; //NM x dist from obs to robot
            obs_dy = rob_idx1_y - i; //NM y dist from obs to robot
            obs_dist_squared = obs_dx*obs_dx + obs_dy*obs_dy; //NM squared dist to avoid slow sqrt()
            if (obs_dist_squared < 0.001) obs_dist_squared = 0.001; //NM avoid divide by 0
            Vx += (k_obs/(2*obs_dist_squared*obs_dist_squared))*(obs_dx); //NM vx = k/(2d^3)*(unit vector x component) = k/(2*d^3)*(dx/d) = k*dx(2*d^4) = k*dx(2*(d^2)^2)
            Vy += (k_obs/(2*obs_dist_squared*obs_dist_squared))*(obs_dy);
        }
    }
    //NM goal attraction
    Vx += k_goal*(waypoint.x - rob_idx1_x); //k * tiles * 4in
    Vy += k_goal*(waypoint.y - rob_idx1_y);

    //NM ball repulsion
    ball_dx = rob_idx1_x - ball_idx1_x; //NM x dist from obs to robot
    ball_dy = rob_idx1_y - ball_idx1_y; //NM y dist from obs to robot
    ball_dist_squared = ball_dx*ball_dx + ball_dy*ball_dy; //NM squared dist to avoid slow sqrt()
    if (ball_dist_squared < 0.001) ball_dist_squared = 0.001; //NM avoid divide by 0
    Vx += (k_ball/(2*ball_dist_squared*ball_dist_squared))*(ball_dx); //NM vx = k/(2d^3)*(unit vector x component) = k/(2*d^3)*(dx/d) = k*dx(2*d^4) = k*dx(2*(d^2)^2)
    Vy += (k_ball/(2*ball_dist_squared*ball_dist_squared))*(ball_dy); //NM units: k / (tiles^3 * 4in)

    //NM
    theta_d = atan2(-Vy,Vx);
    theta_err = theta_d - ROBOTps.theta;
    while (theta_err > M_PI)  theta_err -= 2 * M_PI;
    while (theta_err < -M_PI) theta_err += 2 * M_PI;
    if (fabs(theta_err) > 3.14) {
        theta_err = 3.14;
    }
    f_v = 1 - fabs(theta_err/HALFPI); //NM add a scale factor to the velocity, the more it needs to turn, the lower the velocity should be
    if (f_v < 0) {
        f_v = 0;
    } //NM constrain
    vref = f_v*1.0; //f_v*(Vx*Vx + Vy*Vy); //NM should be sqrt(), but sqrt() is slow. magnitude change will be handled by k values
    turn = -k_turn*theta_err;

    goalDist = sqrt((ROBOTps.x - goal.x)*(ROBOTps.x - goal.x) + (ROBOTps.y - goal.y)*(ROBOTps.y - goal.y)); //NM calc for flag

    waypointDist = sqrt((rob_idx1_x - waypoint.x)*(rob_idx1_x - waypoint.x) + (rob_idx1_y - waypoint.y)*(rob_idx1_y - waypoint.y));
    if (waypointDist < 2.5 && test_wpt_ctr > 100) {test_wpt_flag = !test_wpt_flag; test_wpt_ctr = 0;}
    test_wpt_ctr++;
}

void setWaypoint2(){
    xdiff = goal.x - ball.x;
    ydiff = goal.y - ball.y;
    dist_ball_goal = sqrt(xdiff*xdiff + ydiff*ydiff);
    if (dist_ball_goal > 0.0001){
        waypoint_x = ball.x - 1*xdiff/dist_ball_goal;
        waypoint_y = ball.y - 1*ydiff/dist_ball_goal;
    }
    waypoint.x = (int)floor(waypoint_x*UNIT_MAP1_LEN_INV)+(MAPSIZE1-1)/2;
    waypoint.y = -(int)floor(waypoint_y*UNIT_MAP1_LEN_INV)+(MAPSIZE1-1)/2;
    thetaD = atan2(ydiff, xdiff);
    constrainwaypointidx();
}

void constrainwaypointidx(void){
    if (waypoint.x < 0) waypoint.x = 0;
    if (waypoint.x >= MAPSIZE1) waypoint.x = MAPSIZE1 - 1;
    if (waypoint.y < 0) waypoint.y = 0;
    if (waypoint.y >= MAPSIZE1) waypoint.y = MAPSIZE1 - 1;
}

void nav2(void){
    //weighted gradient descent algorithm to navigate around obstacles
    //and face certain direction
    //OUTPUT: vref and turn
    if ((!((rob_idx1_x <= waypoint.x + 1) && (rob_idx1_x >= waypoint.x - 1)) || !(rob_idx1_y <= (waypoint.y + 1) && rob_idx1_y >= (waypoint.y - 1))) && !arrivedxy){ //if the robot is NOT within x+-1 or y+-1 nav1
        nav1(); //NM if we arent at target position, go to target position
    } else {
        arrivedxy = 1;
        theta_err = thetaD - ROBOTps.theta;
        while (theta_err > M_PI)  theta_err -= 2 * M_PI;
        while (theta_err < -M_PI) theta_err += 2 * M_PI;
        turn = -k_turn*theta_err; //NM once weve arrived at the target position, turn to the target heading
        vref = 0;
    }
    if (arrivedxy && (fabs(theta_err) < 0.05)){
        arrivedtheta = 1;
        vref = 0;
        turn = 0;
    }
}

void setWaypoint3(void){
    //go to goal
    //OUTPUT: xd, yd
    waypoint.x = (int)floor(goal.x*UNIT_MAP1_LEN_INV)+(MAPSIZE1-1)/2;
    waypoint.y = -(int)floor(goal.y*UNIT_MAP1_LEN_INV)+(MAPSIZE1-1)/2;
}

void turnToGoal(void){
    // face the goal, prepare to kick
    //OUTPUT: vref, turn
    xdiff = goal.x - ROBOTps.x;
    ydiff = goal.y - ROBOTps.y;
    thetaD = atan2(ydiff, xdiff);
    theta_err = thetaD - ROBOTps.theta;
    while (theta_err > M_PI)  theta_err -= 2 * M_PI;
    while (theta_err < -M_PI) theta_err += 2 * M_PI;
    turn = -k_turn*theta_err; //NM once weve arrived at the target position, turn to the target heading
    vref = 0;
}

void detectBall(void) {
    if ((MaxAreaThreshold1 != 0.0) && (MaxRowThreshold1 != 0.0) && (MaxColThreshold1 != 0.0) && (!ball_position_recorded)) {
        colcentroid =  100 - MaxColThreshold1;
        if ((MaxColThreshold1 > 94) && (MaxColThreshold1 < 106)) { //if the blob is in the center of the screen -G8
            ball_centered = 1;  // set this global variable to true  -yuan41
            ballDist1 = 264.2167 - 6.144*MaxRowThreshold1 + 0.0523*MaxRowThreshold1*MaxRowThreshold1 - 0.00015432*MaxRowThreshold1*MaxRowThreshold1*MaxRowThreshold1;
            ballDist1 *= 0.08333;
            ball.x = ROBOTps.x + ballDist1*cos(ROBOTps.theta);
            ball.y = ROBOTps.y + ballDist1*sin(ROBOTps.theta);
            ball_position_recorded = 1;

            ball_idx1_x = (int)floor(ball.x*UNIT_MAP1_LEN_INV)+(MAPSIZE1-1)/2;
            ball_idx1_y = -(int)floor(ball.y*UNIT_MAP1_LEN_INV)+(MAPSIZE1-1)/2;

            vref = 0;
            turn = 0; //do nothing -G8
        } else {
            ball_centered = 0;
            ball_position_recorded = 0;
            vref = 0.0;  // changed to stop immediately if ball is seen
            turn = -kpvision * colcentroid;   // the turn is very jittery. adjust kpvision?  - yuan41
            //turn towards the col if it is not centered -G8
        }
    }
}

void captureBall(void){
    if ((MaxAreaThreshold1 != 0.0) && (MaxRowThreshold1 != 0.0) && (MaxColThreshold1 != 0.0)) {
        capture_ball_cntr = 0;
        colcentroid =  100 - MaxColThreshold1;
        if ((MaxColThreshold1 > 94) && (MaxColThreshold1 < 106)) { //if the blob is in the center of the screen -G8
            ball_centered = 1;  // set this global variable to true  -yuan41
            ballDist1 = 264.2167 - 6.144*MaxRowThreshold1 + 0.0523*MaxRowThreshold1*MaxRowThreshold1 - 0.00015432*MaxRowThreshold1*MaxRowThreshold1*MaxRowThreshold1;
            ballDist1 *= 0.08333; //in to feet
            centered_first = 1;
            vref = 0.1;
            turn = 0.0;
        } else {
            ball_centered = 0;
            ball_position_recorded = 0;
            vref = 0.0;  // changed to stop immediately if ball is seen
            turn = -kpvision * colcentroid;   // the turn is very jittery. adjust kpvision?  - yuan41
            //turn towards the col if it is not centered -G8
        }
    } else if(ball_centered){
        capture_time = (int)((ballDist1-0.7)/vref)*1000;
        if (capture_ball_cntr <= capture_time){
            vref = 0.1; //slowly approach ball G8
            turn = 0;
            capture_ball_cntr++;
        }
    }
}

void detectAprilTag(void) {
    if ((tagx != 0.0) && (tagy != 0.0) && (tagz != 0.0) && (!goal_position_recorded))  {
        if ((tagx > goal_x_min) && (tagx < goal_x_max)) {
            tag_centered = 1;
            goalDist = fabs(tagz*0.00425) + (3.5/12.0);
            goal.x = ROBOTps.x + goalDist*cos(ROBOTps.theta);
            goal.y = ROBOTps.y + goalDist*sin(ROBOTps.theta);
            goal_position_recorded = 1;

            goal_idx1_x = (int)floor(goal.x*UNIT_MAP1_LEN_INV)+(MAPSIZE1-1)/2;
            goal_idx1_y = -(int)floor(goal.y*UNIT_MAP1_LEN_INV)+(MAPSIZE1-1)/2;

            vref = 0.0;
            turn = 0.0;
        } else {
            tag_centered = 0;
            goal_position_recorded = 0;
            vref = 0.0;
            turn = -kptag * (CENTER_X - tagx);
        }
    }
}

void go_to_April(void){
    if ((tagx != 0.0) && (tagy != 0.0) && (tagz != 0.0))  {
        if ((tagx > goal_x_min) && (tagx < goal_x_max)) {
            tag_centered = 1;
            vref = 0.25;
            turn = 0.0;
        } else {
            tag_centered = 0;
            goal_position_recorded = 0;
            vref = 0.0;
            turn = -kptag * (CENTER_X - tagx);
        }
    }
}
//-----------------------Other interupt services-------------------//
// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    numTimer0calls++;
    if ((numTimer0calls%5) == 0) {
        // Blink LaunchPad Red LED
        //GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{

    serial_sendSCIC(&SerialC, G_command, G_len);

    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void) //called every 40ms
{
    // Blink LaunchPad Blue LED
    //GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    //servo vv
    if(kickState==1){ //wait RW
        kickWaitCnt++;

        setEPWM5B_RCServo(0.0); //pull cover up RW
        if(kickWaitCnt >= 10){//wait  RW
            kickWaitCnt = 0;
            kickState = 2;
        }

    }else if(kickState == 2){

        //kick out
        setEPWM5A_RCServo(-50.0); //5A motor RW

        kickWaitCnt++;
        if(kickWaitCnt == 10){//wait  RW
            kickState = 3;
            kickWaitCnt = 0;
        }

    }else if(kickState == 3){

        kickWaitCnt++;

        if(kickWaitCnt == 10){//wait  RW
            kickWaitCnt = 0;
            kickState = 4;
        }

    }else if(kickState == 4){

        //reset
        kickWaitCnt++;
        setEPWM5A_RCServo(-90); //rest is -90
        if(kickWaitCnt == 50){//wait  RW
            kickState = 0;
            kickWaitCnt = 0;
        }
    }
    //servo ^





    CpuTimer2.InterruptCount++;

    //  if ((CpuTimer2.InterruptCount % 10) == 0) {
    //      UARTPrint = 1;
    //  }
}

__interrupt void can_isr(void)
{
    int i = 0;
    uint32_t status;
    // Read the CAN interrupt status to find the cause of the interrupt
    status = CANgetInterruptCause(CANB_BASE);
    // If the cause is a controller status interrupt, then get the status
    if(status == CAN_INT_INT0ID_STATUS)
    {
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        status = CANgetStatus(CANB_BASE);
    }
    // Check if the cause is the receive message object 2
    else if(status == RX_MSG_OBJ_ID_1)
    {
        //
        // Get the received message
        //
        CANreadMessage(CANB_BASE, RX_MSG_OBJ_ID_1, rxMsgData);

        for(i = 0; i<2; i++)
        {
            dis_raw_1[i] = rxMsgData[i];
        }

        dis_1 = 256*dis_raw_1[1] + dis_raw_1[0];

        measure_status_1 = rxMsgData[2];

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANclearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID_1);

        //
        // Increment a counter to keep track of how many messages have been
        // received. In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        rxMsgCount_1++;

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
    }

    else if(status == RX_MSG_OBJ_ID_2)
    {
        //
        // Get the received message
        //
        CANreadMessage(CANB_BASE, RX_MSG_OBJ_ID_2, rxMsgData);

        for(i = 0; i<2; i++)
        {
            dis_raw_2[i] = rxMsgData[i];
        }

        dis_2 = 256*dis_raw_2[1] + dis_raw_2[0];

        measure_status_2 = rxMsgData[2];

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANclearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID_2);

        //
        // Increment a counter to keep track of how many messages have been
        // received. In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        rxMsgCount_2++;

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
    }

    else if(status == RX_MSG_OBJ_ID_3)
    {
        //
        // Get the received message
        //
        CANreadMessage(CANB_BASE, RX_MSG_OBJ_ID_3, rxMsgData);

        for(i = 0; i<4; i++)
        {
            lightlevel_raw_1[i] = rxMsgData[i];
            quality_raw_1[i] = rxMsgData[i+4];

        }

        lightlevel_1 = ((256.0*256.0*256.0)*lightlevel_raw_1[3] + (256.0*256.0)*lightlevel_raw_1[2] + 256.0*lightlevel_raw_1[1] + lightlevel_raw_1[0])/65535;
        quality_1 = ((256.0*256.0*256.0)*quality_raw_1[3] + (256.0*256.0)*quality_raw_1[2] + 256.0*quality_raw_1[1] + quality_raw_1[0])/65535;


        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANclearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID_3);

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
    }


    else if(status == RX_MSG_OBJ_ID_4)
    {
        //
        // Get the received message
        //
        CANreadMessage(CANB_BASE, RX_MSG_OBJ_ID_4, rxMsgData);

        for(i = 0; i<4; i++)
        {
            lightlevel_raw_2[i] = rxMsgData[i];
            quality_raw_2[i] = rxMsgData[i+4];

        }

        lightlevel_2 = ((256.0*256.0*256.0)*lightlevel_raw_2[3] + (256.0*256.0)*lightlevel_raw_2[2] + 256.0*lightlevel_raw_2[1] + lightlevel_raw_2[0])/65535;
        quality_2 = ((256.0*256.0*256.0)*quality_raw_2[3] + (256.0*256.0)*quality_raw_2[2] + 256.0*quality_raw_2[1] + quality_raw_2[0])/65535;

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANclearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID_4);

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
    }



    //
    // If something unexpected caused the interrupt, this would handle it.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CANclearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    InterruptclearACKGroup(INTERRUPT_ACK_GROUP9);
}

__interrupt void SPIB_isr(void){

    uint16_t i;
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    GpioDataRegs.GPCSET.bit.GPIO66 = 1;                 //Pull CS high as done R/W
    GpioDataRegs.GPASET.bit.GPIO29 = 1;  // Pull CS high for DAN28027

    if (CurrentChip == MPU9250) {

        for (i=0; i<8; i++) {
            readdata[i] = SpibRegs.SPIRXBUF; // readdata[0] is garbage
        }

        PostSWI1(); // Manually cause the interrupt for the SWI1

    } else if (CurrentChip == DAN28027) {

        DAN28027Garbage = SpibRegs.SPIRXBUF;
        dan28027adc1 = SpibRegs.SPIRXBUF;
        dan28027adc2 = SpibRegs.SPIRXBUF;
        CurrentChip = MPU9250;
        SpibRegs.SPIFFCT.bit.TXDLY = 0;
    }

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
}

//adcd1 pie interrupt
__interrupt void ADCC_ISR (void)
{

    GpioDataRegs.GPASET.bit.GPIO11 = 1;
    adcc2result = AdccResultRegs.ADCRESULT0;
    adcc3result = AdccResultRegs.ADCRESULT1;
    adcc4result = AdccResultRegs.ADCRESULT2;
    adcc5result = AdccResultRegs.ADCRESULT3;

    // Here covert ADCIND0 to volts
    adcC2Volt = adcc2result*3.0/4095.0;
    adcC3Volt = adcc3result*3.0/4095.0;
    adcC4Volt = adcc4result*3.0/4095.0;
    adcC5Volt = adcc5result*3.0/4095.0;

    if (MPU9250ignoreCNT >= 1) {
        CurrentChip = MPU9250;
        SpibRegs.SPIFFCT.bit.TXDLY = 0;
        SpibRegs.SPIFFRX.bit.RXFFIL = 8;

        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

        SpibRegs.SPITXBUF = ((0x8000)|(0x3A00));
        SpibRegs.SPITXBUF = 0;
        SpibRegs.SPITXBUF = 0;
        SpibRegs.SPITXBUF = 0;
        SpibRegs.SPITXBUF = 0;
        SpibRegs.SPITXBUF = 0;
        SpibRegs.SPITXBUF = 0;
        SpibRegs.SPITXBUF = 0;
    } else {
        MPU9250ignoreCNT++;
    }

    numADCCcalls++;
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;

}

void setF28027EPWM1A(float controleffort){
    if (controleffort < -10) {
        controleffort = -10;
    }
    if (controleffort > 10) {
        controleffort = 10;
    }
    float value = (controleffort+10)*3000.0/20.0;
    EPwm1A_F28027 = (int16_t)value;  // Set global variable that is sent over SPI to F28027
}
void setF28027EPWM2A(float controleffort){
    if (controleffort < -10) {
        controleffort = -10;
    }
    if (controleffort > 10) {
        controleffort = 10;
    }
    float value = (controleffort+10)*3000.0/20.0;
    EPwm2A_F28027 = (int16_t)value;  // Set global variable that is sent over SPI to F28027
}
