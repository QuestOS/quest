#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>


#define FAULT_1
#define FAULT_2
#define FAULT_3

#define MAX_GEARS 10
#define AMARG 0.6
#define RPC_SIZE 1024
#define MAX_CARS 10

#define RPC_CMD_PIT 1
#define RPC_CMD_DRIVE 2
#define RPC_CMD_SHUTDOWN 3

const double PI = 3.14159265358979323846;  /**< PI */

#define PIT_STATE_NO            -1
#define PIT_STATE_NONE           0
#define PIT_STATE_ASKED          1
#define PIT_STATE_ENTERED        2
#define PIT_STATE_PITLANE_BEFORE 3
#define PIT_STATE_PIT_ENTRY      4
#define PIT_STATE_PIT_ALIGN      5
#define PIT_STATE_PIT_EXIT       6
#define PIT_STATE_PITLANE_AFTER  7
#define PIT_STATE_EXIT           8
#define PIT_STATE_DECEL          9

#define RM_RACE_RUNNING		0X00000001
#define RM_RACE_FINISHING	0X00000002
#define RM_RACE_ENDED		0X00000004
#define RM_RACE_STARTING	0X00000008
#define RM_RACE_PRESTART	0X00000010
#define RM_RACE_PAUSED		0X40000000

#define RM_LIGHT_HEAD1		0x00000001	/**< head light 1 */
#define RM_LIGHT_HEAD2		0x00000002	/**< head light 2 */

#define RM_CMD_NONE		0	/**< No race command */
#define RM_CMD_PIT_ASKED	1	/**< Race command: Pit asked */


#define RM_CAR_STATE_FINISH	 	0x00000100				/**< Car having passed the finish line */
#define RM_CAR_STATE_PIT	 	0x00000001				/**< Car currently stopped in pits */
#define RM_CAR_STATE_DNF	 	0x00000002				/**< Car did not finish */
#define RM_CAR_STATE_PULLUP	 	0x00000004				/**< Car pulled out in the air */
#define RM_CAR_STATE_PULLSIDE	 	0x00000008				/**< Car pulled out in the air */
#define RM_CAR_STATE_PULLDN	 	0x00000010				/**< Car pulled out in the air */
#define RM_CAR_STATE_OUT		(RM_CAR_STATE_DNF | RM_CAR_STATE_FINISH)/**< Car out of race */
#define RM_CAR_STATE_NO_SIMU	 	0x000000FF				/**< Do not simulate the car */
#define RM_CAR_STATE_BROKEN	 	0x00000200				/**< Engine no more working */
#define RM_CAR_STATE_OUTOFGAS	 	0x00000400				/**< Out of Gas */
#define RM_CAR_STATE_ELIMINATED	 	0x00000800				/**< Eliminated due to rules infringement */
#define RM_CAR_STATE_SIMU_NO_MOVE	0x00010000 				/**< Simulation without car move (i.e. clutch applied and no wheel move)  */

#define RELAXATION(target, prev, rate) 				\
do {								\
    target = (prev) + (rate) * ((target) - (prev)) * 0.01;	\
    prev = (target);						\
} while (0)

/** Angle normalization between -PI and PI */
#define NORM_PI_PI(x) 				\
do {						\
	while ((x) > PI) { (x) -= 2*PI; }	\
	while ((x) < -PI) { (x) += 2*PI; } 	\
} while (0)

#ifndef MIN
/** Minimum between two values */
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif

#define SIGN(x)     ((x) < 0 ? -1.0 : 1.0)	/**< Sign of the expression */


float ConsFactor[10] = {0.0007f};
float VM;
float Gmax;
float PGain[10]     = {0.015f};
float AGain[10]     = {0.008f};
float PnGain[10]    = {0.02f};
float Advance[10]   = {3.5f};
float Advance2[10]  = {10.0f};
float AdvStep[10]   = {1.0f};
float VGain[10]     = {0.0005f};
float preDy[10]     = {0.0f};
float spdtgt[10]    = {250.0f};
float spdtgt2[10]   = {2.0f};
float steerMult[10] = {2.0f};
float Offset[10]    = {0.0f};
float OffsetApproach[10] = {0.0};
float OffsetFinal[10]    = {0.0};
float OffsetExit[10]     = {0.0};
float O1[10] = {60.0};
float O2[10] = {60.0};
float O3[10] = {0.0};
float O4[10] = {0.0};
float O5[10] = {20.0};
float OP[10] = {15.0};
float OA[10] = {0.0};
float OW[10] = {2.0};
float VM1[10] = {15.0};
float VM2[10] = {0.0};
float VM3[10] = {25.0};
float Tright[10];
float Trightprev[10];
float hold[10] = {0};
float shiftThld[10][MAX_GEARS+1];
float LgfsFinal[10];
int PitState[10]  = {0};
float MaxSpeed[10];
float DynOffset[10] = {0.0};
float VI[10];
float lastDv[10] = {0};
float lastAccel[10] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
float lastBrkCmd[10] = {0};

int curidx;
float Gear;
float InvBrkCmd;
float TargetSpeed;


//#define DEBUG
/* XXX: right now, doesn't convert byte order */
inline void marshall_int(char *buf, int *index, int data)
{
	*((int *)(&buf[*index])) = data;
	*index = *index + sizeof(int);
#ifdef DEBUG
        printf("%d\n", data);
#endif
}

inline void marshall_float(char *buf, int *index, float data)
{
	*((float *)(&buf[*index])) = data;
	*index = *index + sizeof(float);
#ifdef DEBUG
        printf("%f\n", data);
#endif
}

inline void marshall_double(char *buf, int *index, double data)
{
	*((double *)(&buf[*index])) = data;
	*index = *index + sizeof(double);
#ifdef DEBUG
        printf("%f\n", data);
#endif
}

inline int unmarshall_int(char *buf, int *index)
{
	int tmp = *((int *)(&buf[*index]));
	*index = *index + sizeof(int);
#ifdef DEBUG
        printf("%d\n", tmp);
#endif
	return tmp;
}

inline float unmarshall_float(char *buf, int *index)
{
	float tmp = *((float *)(&buf[*index]));
	*index = *index + sizeof(float);
#ifdef DEBUG
        printf("%f\n", tmp);
#endif
	return tmp;
}

inline double unmarshall_double(char *buf, int *index)
{
	double tmp = *((double *)(&buf[*index]));
	*index = *index + sizeof(double);
#ifdef DEBUG
        printf("%f\n", tmp);
#endif
	return tmp;
}

typedef struct {
  int gear, dammage;
  float fuel, toright, angle_e, angle_s, pos_x, pos_y;
  int laps, lapsbehind;
  float width, yaw, rate, dimension, dist, speed, tomiddle;
  int offset;
  float angle, radius, spinvel[4], tank, steerlock;
  float global_x, global_y;
  int light, race, gearcmd;//output:lightCmd, raceCmd, gearCmd
  float accel, steer, brake;//output: accelCmd, steerCmd, brakeCmd
  float pitfuel;//output:pitFuel
  int repair;//output:pitRepair
}
Car_info;

Car_info car;

typedef struct {
  int state;
  float dist, speed;
  int laps;
  float toright;
}
OtherCar;

typedef struct {
  double cur_t, delta_t;
  int maxdammage, state, totlaps, ncars;
  OtherCar cars[MAX_CARS];
}
Situation;

Situation s;

typedef struct {
  float len, n_len, entry_lg, exit_lg, start_lg, end_lg;
}
Track;

Track track;


void
SpeedStrategy(int idx, float Vtarget, float aspect)
{
    const float Dx  = 0.02f;
    const float Dxx = 0.01f;
    const float Dxb  = 0.05f;
    const float Dxxb = 0.01f;
    float	Dv;
    float	Dvv;
    float 	slip;
    int 	gear;

    gear = car.gear;
    Dv = Vtarget - car.speed;
    Dvv = Dv - lastDv[idx];
    lastDv[idx] = Dv;

    //RELAXATION(Vtarget, lastTarget[idx], 2.0);
    
    if (Dv > 0.0) {
	/* speed management */
	car.accel = MIN(Dv * Dx + Dvv * Dxx, 1.0);
	car.accel = 1.0;
	
	/* anti-slip */
	/* assume SPOOL differential and rwd */
	if (car.speed > 0) {
	    slip = (car.radius * car.spinvel[3] - car.speed) / car.speed;
	} else {
	    slip = 0;
	}
	if (gear == 1) {
	    car.accel = car.accel * q_exp(-q_fabs(car.steer) * 0.1) * q_exp(-q_fabs(aspect) * 5.0) + .1;
	} else if ((gear > 1) && (car.speed < 40.0)) {
	    car.accel = car.accel * q_exp(-q_fabs(aspect) * 4.0) + 0.15;
	}
	
	
	if ((slip > 1.0) && (gear > 1)) {
	    car.accel *= 0.5;
	} else {
	    RELAXATION(car.accel, lastAccel[idx], 50.0);
	}
	car.accel = MIN(car.accel, q_fabs(Dv/6.0));
	//lastBrkCmd[idx] = 0.8;
    } else {
	float meanSpd = 0;
	int i;

	slip = 0;
	for (i = 0; i < 4; i++) {
	    meanSpd += car.spinvel[i];
	}
	meanSpd /= 4.0;

	if (meanSpd > 15.0) {
	    for (i = 0; i < 4; i++) {
		if (((meanSpd - car.spinvel[i]) / meanSpd) < -0.1) {
		    slip = 1.0;
		}
	    }
	}
	car.brake = MIN(-Dv * Dxb + Dvv * Dxxb, 1.0);
	if (slip > 0.3) {
	    float maxslp = q_exp(-3.47*(slip - 0.2));
	    car.brake = MIN(car.brake, maxslp);
	} else {
	    RELAXATION(car.brake, lastBrkCmd[idx], 50.0);
	}
	car.brake = MIN(car.brake, q_fabs(Dv/5.0));
	//lastAccel[idx] = 1.0;
    }

    /* shift */
    gear += car.offset;
    car.gearcmd = car.gear;
    if (car.speed > shiftThld[idx][gear]) {
	car.gearcmd++;
    } else if ((car.gearcmd > 1) && (car.speed < (shiftThld[idx][gear-1] - 10.0))) {
	car.gearcmd--;
    }
    if (car.gearcmd <= 0) {
	car.gearcmd++;
    }
}



float
Spline(float p0, float p1, float pos, float start, float end)
{
    float t2, t3;
    float h0, h1;
    float t;

    if (start < 0) {
	start += track.len;
    }
    if (start > track.len) {
	start -= track.len;
    }
    if (end < 0) {
	end += track.len;
    }
    if (end > track.len) {
	end -= track.len;
    }
    if (start > end) {
	end += track.len;
	if (pos < start) {
	    pos += track.len;
	}
    }
    t = (pos - start) / (end - start);
    
    t2 = t * t;
    t3 = t * t2;
    h1 = 3 * t2 - 2 * t3;
    h0 = 1 - h1;
    
    return h0 * p0 + h1 * p1;
}


int
isBetween(float lgfs, float start, float end)
{
    if (start < 0) {
	start += track.len;
    }
    if (start > track.len) {
	start -= track.len;
    }
    if (end < 0) {
	end += track.len;
    }
    if (end > track.len) {
	end -= track.len;
    }
    if (((lgfs > start) && (lgfs < end)) ||
	((start > end) &&
	 ((lgfs > start) || (lgfs < end)))) {
	return 1;
    }
    return 0;
}

float Entry;
float Start;
float End;
float Exit;

float
getOffset(int idx, float *maxSpeed)
{
    float	offset = 0;
    float	lgfs = 0;
//    static tTrackPitInfo *pits = &DmTrack->pits;;
    Entry = track.entry_lg;
    Start = track.start_lg;
    End   = track.end_lg;
    Exit  = track.exit_lg;

    switch (PitState[idx]) {
    case PIT_STATE_NONE:
	break;
	
    case PIT_STATE_ASKED:
    case PIT_STATE_ENTERED:
	lgfs = car.dist;
	car.light |= RM_LIGHT_HEAD2;

	if (isBetween(lgfs, Entry - O1[idx], Start - O2[idx])) {
	    //GfOut("PIT_STATE_ENTERED\n");
	    offset = Spline(0, OffsetApproach[idx], lgfs, Entry - O1[idx], Start - O2[idx]);
	    if (PitState[idx] == PIT_STATE_ASKED) {
		VI[idx] = car.speed;
	    }
	    *maxSpeed = Spline(VI[idx], VM, lgfs, Entry - O1[idx], Start - O2[idx]);
	    hold[idx] = 0;
	    PitState[idx] = PIT_STATE_ENTERED;
	    break;
	}
	if (PitState[idx] == PIT_STATE_ASKED) {
	    break;
	}
	
	/* FALL THROUGH */
    case PIT_STATE_DECEL:
	if (!lgfs) lgfs = car.dist;
	car.light |= RM_LIGHT_HEAD2;

	if (isBetween(lgfs, Start - O2[idx], LgfsFinal[idx] - OP[idx] - O5[idx])) {
	    //GfOut("PIT_STATE_DECEL\n");
	    PitState[idx] = PIT_STATE_DECEL;
	    offset = OffsetApproach[idx];
	    *maxSpeed = VM;
	    hold[idx] = 0;
	    break;
	}
	
	/* FALL THROUGH */
    case PIT_STATE_PITLANE_BEFORE:
	if (!lgfs) lgfs = car.dist;
	car.light |= RM_LIGHT_HEAD2;

	if (isBetween(lgfs, LgfsFinal[idx] - OP[idx] - O5[idx], LgfsFinal[idx] - OP[idx])) {
	    //GfOut("PIT_STATE_PITLANE_BEFORE\n");
	    PitState[idx] = PIT_STATE_PITLANE_BEFORE;
	    offset = OffsetApproach[idx];
	    VM1[idx] = MIN(VM1[idx], VM);
	    *maxSpeed = Spline(VM, VM1[idx], lgfs, LgfsFinal[idx] - OP[idx] - O5[idx], LgfsFinal[idx] - OP[idx]);
	    hold[idx] = 0;
	    break;
	}

	/* FALL THROUGH */
    case PIT_STATE_PIT_ENTRY:
	if (!lgfs) lgfs = car.dist;
	car.light |= RM_LIGHT_HEAD2;
	
	if (isBetween(lgfs, LgfsFinal[idx] - OP[idx], LgfsFinal[idx] - OA[idx])) {
	    //GfOut("PIT_STATE_PIT_ENTRY\n");
	    PitState[idx] = PIT_STATE_PIT_ENTRY;
	    //offset = Spline(OffsetApproach, OffsetFinal, lgfs, LgfsFinal[idx] - OP[idx], LgfsFinal[idx]);
	    offset = OffsetFinal[idx] + SIGN(OffsetFinal[idx]) * OW[idx];
	    *maxSpeed = Spline(VM2[idx], 0, lgfs, LgfsFinal[idx] - OP[idx], LgfsFinal[idx]);
	    hold[idx] = 0;
	    car.race = RM_CMD_PIT_ASKED;
	    break;
	}

	/* FALL THROUGH */
    case PIT_STATE_PIT_ALIGN:
	if (!lgfs) lgfs = car.dist;
	car.light |= RM_LIGHT_HEAD2;
	
	if (isBetween(lgfs, LgfsFinal[idx] - OA[idx], LgfsFinal[idx])) {
	    //GfOut("PIT_STATE_PIT_ALIGN\n");
	    PitState[idx] = PIT_STATE_PIT_ALIGN;
	    offset = OffsetApproach[idx];
	    *maxSpeed = Spline(VM2[idx], 0, lgfs, LgfsFinal[idx] - OP[idx], LgfsFinal[idx]);
	    hold[idx] = 0;
	    car.race = RM_CMD_PIT_ASKED;
	    break;
	}

	/* FALL THROUGH */
    case PIT_STATE_PIT_EXIT:
	if (!lgfs) lgfs = car.dist;
	car.light |= RM_LIGHT_HEAD2;

	if (isBetween(lgfs, LgfsFinal[idx] - OP[idx], LgfsFinal[idx] + OP[idx])) {
	    //GfOut("PIT_STATE_PIT_EXIT\n");
	    PitState[idx] = PIT_STATE_PIT_EXIT;
	    //offset = Spline(OffsetFinal, OffsetApproach, lgfs, LgfsFinal[idx], LgfsFinal[idx] + OP[idx]);
	    offset = OffsetExit[idx];
	    VM3[idx] = MIN(VM3[idx], VM);
	    *maxSpeed = Spline(VM3[idx], VM, lgfs, LgfsFinal[idx], LgfsFinal[idx] + OP[idx]);
	    hold[idx] = 0;
	    break;
	}

	/* FALL THROUGH */
    case PIT_STATE_PITLANE_AFTER:
	if (!lgfs) lgfs = car.dist;
	car.light |= RM_LIGHT_HEAD2;

	if (isBetween(lgfs, LgfsFinal[idx] + OP[idx], End + O3[idx])) {
	    //GfOut("PIT_STATE_PITLANE_AFTER %f - %f - %f\n", lgfs, LgfsFinal[idx] + OP[idx], End + O3[idx]);
	    PitState[idx] = PIT_STATE_PITLANE_AFTER;
	    offset = OffsetExit[idx];
	    *maxSpeed = VM;
	    hold[idx] = 0;
	    break;
	}

	/* FALL THROUGH */
    case PIT_STATE_EXIT:
	if (!lgfs) lgfs = car.dist;
	car.light |= RM_LIGHT_HEAD2;

	if (isBetween(lgfs, End + O3[idx], Exit + O4[idx])) {
	    //GfOut("PIT_STATE_EXIT\n");
	    PitState[idx] = PIT_STATE_EXIT;
	    offset = Spline(OffsetExit[idx], 0, lgfs, End + O3[idx], Exit + O4[idx]);
	    hold[idx] = 0;
	    break;
	}
	//GfOut("End of Pit\n");
	PitState[idx] = PIT_STATE_NONE;
	break;
    }

    return offset;
}


void
CollDet(int idx, float dny)
{
    int		i;
    OtherCar	*otherCar;
    float	lgfs, lgfs2, dlg;
    float	dspd;
    float	maxdlg;
//    tTrackSeg	*seg;
    int		canOverlap = 1;
    const float MARGIN = 8.0;

    maxdlg = 200.0;
//    seg = car->_trkPos.seg;
    lgfs = car.dist;

    DynOffset[idx] = 0;
    /* Automatic pit every lap (test) */
#if 0
        if ((PitState[idx] == PIT_STATE_NONE) && (car->_laps)) {
     	PitState[idx] = PIT_STATE_ASKED;
    }
#endif

    if ((PitState[idx] == PIT_STATE_NONE) && ((s.state & RM_RACE_FINISHING) == 0) && 
	(((car.dammage > 5000) && ((s.totlaps - car.laps) > 2)) || 
	 ((car.fuel < ConsFactor[idx]) && ((s.totlaps - car.laps) > 1)))) {
	PitState[idx] = PIT_STATE_ASKED;
    }
    if (PitState[idx] != PIT_STATE_NO) {
	DynOffset[idx] = getOffset(idx, &MaxSpeed[idx]);
	if ((PitState[idx] != PIT_STATE_NONE) && (PitState[idx] != PIT_STATE_ASKED)) {
	    canOverlap = 0;
	}
    }

    for (i = 0; i < s.ncars; i++) {
	otherCar = &s.cars[i];
	if (otherCar->state & RM_CAR_STATE_NO_SIMU) {
	    continue;
	}
	lgfs2 = otherCar->dist;
	dlg = lgfs2 - lgfs;
	if (dlg > (track.len / 2.0)) dlg -= track.len;
	if (dlg < -(track.len / 2.0)) dlg += track.len;

	dspd = car.speed - otherCar->speed;
	if ((car.laps < otherCar->laps) && 
	    (dlg > -maxdlg) && (dlg < (car.dimension + 1.0)) &&
	    (dlg > (dspd * 6.0))) {
	    if ((q_fabs(car.toright - otherCar->toright) < (MARGIN / 2.0)) &&
		(otherCar->speed > car.speed)) {
		maxdlg = q_fabs(dlg);
		hold[idx] = s.cur_t + 1.0;
		if (car.toright < otherCar->toright) {
		    Tright[idx] = otherCar->toright - (MARGIN * 3.0);
		} else {
		    Tright[idx] = otherCar->toright + (MARGIN * 3.0);
		}
	    }
	} else	if (((dlg < maxdlg) && (dlg > -(car.dimension + 1.0))) &&
		    ((dlg < (dspd*4.5)) ||
		     (dlg < (car.dimension * 4.0)))) {

	    if (canOverlap) {
		maxdlg = q_fabs(dlg);
		/* risk of collision */
		car.light |= RM_LIGHT_HEAD2;

		if (q_fabs(car.toright - otherCar->toright) < (MARGIN  - 2.0)) {
		    if (car.toright < otherCar->toright) {
			if (otherCar->toright > MARGIN / 2.0) {
			    Tright[idx] = otherCar->toright - (MARGIN * 2.0 - 1.0);
			    if (dny < 0) {
				if (car.toright > 2.0) {
				    MaxSpeed[idx] = otherCar->speed * .99;
				} else {
				    Tright[idx] += MARGIN * 2.0;
				}
			    }
			} else {
			    if ((dlg > (car.dimension * 2.0)) &&
				(q_fabs(car.toright - otherCar->toright) < MARGIN)) {
				MaxSpeed[idx] = otherCar->speed * .99;
				Tright[idx] = otherCar->toright + (MARGIN * 2.0);
			    }
			}
		    } else {
			if (otherCar->toright < car.width - MARGIN / 2.0) {
			    Tright[idx] = otherCar->toright + (MARGIN * 2.0 - 1.0);
			    if (dny > 0) {
				if (car.toright < (car.width - 2.0)) {
				    MaxSpeed[idx] = otherCar->speed * .99;
				} else {
				    Tright[idx] -= MARGIN * 2.0;
				}
			    }
			} else {
			    if ((dlg > (car.dimension * 2.0)) &&
				(q_fabs(car.toright - otherCar->toright) < (MARGIN / 2.0))) {
				MaxSpeed[idx] = otherCar->speed * .99;
				Tright[idx] = otherCar->toright - (MARGIN * 2.0);
			    }
			}
		    }
		    hold[idx] = s.cur_t + 1.0;
		    if ((dlg > (car.dimension /2.0)) && (dlg < (car.dimension * 3.0)) && (q_fabs(car.toright - otherCar->toright) < 2.0)) {
			MaxSpeed[idx] = otherCar->speed * .95;
			car.light |= RM_LIGHT_HEAD1;
		    }
		}
	    } else {
		/* Stay behind the front car */
		MaxSpeed[idx] = MIN(MaxSpeed[idx], otherCar->speed * .99);
	    }
	}
    }

    if (Tright[idx] < 0.0) {
	Tright[idx] = 0.0;
    } else if (Tright[idx] > car.width) {
	Tright[idx] = car.width;
    }
}

float g_right, g_max;

void RPC_drive(int data)
{
  int idx = data;
  float toRight = g_right;
  float max = g_max;

  float	Dy, Dny;
  float	Vy;
//  tTrkLocPos		trkPos, trkPos2;
  float	X, Y, x, y, CosA, SinA;
//  tTrackSeg		*seg;
  float	Da, Db;
  float	tgtSpeed = -1.0;
  float	lgfs;
  float	vtgt1, vtgt2;
  float	curAdv, /*curAdvMax,*/ Amax, Atmp, AdvMax;

  //static int		lap[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static float lgfsprev[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static float adv[10];

  MaxSpeed[idx] = 10000.0; 
  X = car.pos_x;
  Y = car.pos_y;

  CosA = q_cos(car.yaw);
  SinA = q_sin(car.yaw);
  lgfs = car.dist + q_fabs(preDy[idx]);
  
  if (lgfs < track.n_len) {
    curidx = 0;
    if (lgfsprev[idx] > lgfs) {
      lgfsprev[idx] = 0;
    }
  }

  adv[idx] = Advance[idx] + 5.0 * q_sqrt(q_fabs(car.speed));

  if (s.cur_t > hold[idx]) {
    Tright[idx] = car.width / 2.0 + Offset[idx] + DynOffset[idx];
  }
    
  vtgt1 = spdtgt[idx];
  vtgt2 = spdtgt2[idx];

  x = X + (CosA) * adv[idx];
  y = Y + (SinA) * adv[idx];

  Dny = car.width / 2.0 - toRight + Offset[idx] + DynOffset[idx];

  CollDet(idx, Dny);

  RELAXATION(Tright[idx], Trightprev[idx], 2.0);

  /* proportionnal */
  Dy = Tright[idx] - car.toright;

  /* derivation */
  Vy = (Dy - preDy[idx]) / s.delta_t;

  preDy[idx] = Dy;

  Da = car.angle - car.yaw;
  NORM_PI_PI(Da);

  car.steer = PGain[idx] * Dy + VGain[idx] * Vy + PnGain[idx] * Dny + AGain[idx] * Da * Da;

  if (car.speed < 0) {
    car.steer *= 1.5;
  } else if (car.speed < 10) {
    car.steer *= 2.0;
  }

  /*
   * speed control
   */
  CosA = q_cos(car.yaw + car.steer*2.0);
  SinA = q_sin(car.yaw + car.steer*2.0);
  curAdv = Advance2[idx];
  AdvMax = q_fabs(car.speed) * 5.0 + 1.0;
  Amax = max;
  
  Db = car.rate;
  Amax = 1.0 - Amax;
  Amax = Amax * Amax;
  if (tgtSpeed < 0) {
    tgtSpeed = (vtgt1 * Amax  + vtgt2) *
	(1.0 + q_tan(q_fabs(car.angle_e + car.angle_s)));
    tgtSpeed -= (car.dammage / s.maxdammage) * 0.2;
    tgtSpeed = MIN(tgtSpeed, MaxSpeed[idx] / 1.15);
  }
  TargetSpeed = tgtSpeed * 1.15;
  SpeedStrategy(idx, TargetSpeed, Db);

  if ((((Da > (PI/2.0-AMARG)) && (car.toright < car.width/3.0)) ||
    ((Da < (AMARG-PI/2.0)) && (car.toright > (car.width - car.width/3.0)))) && 
    (car.gear < 2) && (car.speed < 1.0)) {
    car.steer = -car.steer * 3.0;
    car.gearcmd = -1;
  } else if ((q_fabs(Da) > (PI - (PI/4.0))) &&
	       ((car.toright < 0) ||
		(car.toright > car.width))) {
	car.steer = -car.steer * 3.0;
    }
    if ((car.speed < -0.5) && (car.gear > 0)) {
	car.brake = 1.0;
    }

    if ((PitState[idx] > PIT_STATE_DECEL) && (PitState[idx] < PIT_STATE_EXIT) && (car.speed < 15.0)) {
	car.steer *= 5.0;
    }

}

void
RPC_pitCmd(int idx)
{
  int remainLaps = s.totlaps - car.laps - car.lapsbehind + 1;
  float remainDist = remainLaps * track.len;
  float	fuel;
	
  PitState[idx] = PIT_STATE_PIT_EXIT;
  fuel = ConsFactor[idx] * (remainLaps + 1);
	
  if (fuel > car.tank) fuel = car.tank;
	
  fuel -=  car.fuel;
	
  if (fuel < 0) fuel = 0;
	
  car.pitfuel = fuel;
  remainDist = remainLaps * track.len;

  if (remainDist > 100) {
    car.repair = (int)(car.dammage);
  } else if (remainDist > 60) {
    car.repair = (int)(car.dammage / 1.5);
  } else {
    car.repair = (int)(car.dammage / 2.0);
  }
}

struct game_data{
  float track_len, track_nlen, track_entry_lg, track_start_lg;
  float track_end_lg, track_exit_lg;
  int idx;
  float VM, Gmax, PGain, AGain, PnGain, Advance, Advance2;
  float AdvStep, VGain, preDy, spdtgt, spdtgt2, steerMult;
  float Offset, OffsetApproach, OffsetFinal, OffsetExit;
  float O1, O2, OP, OA, O3, O4, O5, VM1, VM2, VM3;
  float Tright, shiftThld[MAX_GEARS];
  int tmptmp, PitState;
  float LgfsFinal;  
} game = {
  .track_len = 2057.559326,
  .track_nlen = 7.500000,
  .track_entry_lg = 1222.161987,
  .track_start_lg = 1386.812866,
  .track_end_lg = 1679.312866,
  .track_exit_lg = 1866.766724,
  .idx = 2,
  .VM = 25.0,
  .Gmax = 1.0,
  .PGain = 0.015,
  .AGain = 0.008,
  .PnGain = 0.02,
  .Advance = 5.0,
  .Advance2 = 10.0,
  .AdvStep = 1.0,
  .VGain = 0.0005,
  .preDy = 0,
  .spdtgt = 100.0,
  .spdtgt2 = 1.0,
  .steerMult = 1.0,
  .Offset = 0.0,
  .OffsetApproach = 0.0,
  .OffsetFinal = 0.0,
  .OffsetExit = 0.0,
  .O1 = 60.0,
  .O2 = 60.0,
  .OP = 15.0,
  .OA = 0.0,
  .O3 = 0.0,
  .O4 = 0.0,
  .O5 = 20.0,
  .VM1 = 15.0,
  .VM2 = 0.0,
  .VM3 = 25.0,
  .Tright = 7.5,
  .tmptmp = 0,
  .PitState = -1
};

int order = 0;

int main (int argc, char *argv[])
{
  game.shiftThld[0] = 10000.0;
  game.shiftThld[1] = 10000.0;
  game.shiftThld[2] = 26.225941;
  game.shiftThld[3] = 43.086746;
  game.shiftThld[4] = 58.226402;
  game.shiftThld[5] = 66.111229;
  game.shiftThld[6] = 74.666801;
  game.shiftThld[7] = 10000.0;
  game.shiftThld[8] = 10000.0;
  game.shiftThld[9] = 10000.0;

  int i, fault_sig, idx, length, slen, cpu;
  printf ("Argument: ");
  printf (argv[0]);
  printf ("\n");
  fault_sig = 1;
  if (argv[0][0] == 'a')
    fault_sig = 1;
  if (argv[0][0] == 'b')
    fault_sig = 2;
  if (argv[0][0] == 'c')
    fault_sig = 3;
  if (argv[0][0] == 'd')
    fault_sig = 4;
  printf ("fault_sig=%d\n", fault_sig);

  //if(fault_sig == 2 || fault_sig == 3 || fault_sig == 4){
  if (1) {
    track.len = game.track_len;
    track.n_len = game.track_nlen;
    track.entry_lg = game.track_entry_lg;
    track.start_lg = game.track_start_lg;
    track.end_lg = game.track_end_lg;
    track.exit_lg = game.track_exit_lg;
    idx = game.idx;
    VM = game.VM;
    Gmax = game.Gmax;
    PGain[idx] = game.PGain;
    AGain[idx] = game.AGain;
    PnGain[idx] = game.PnGain;
    Advance[idx] = game.Advance;
    Advance2[idx] = game.Advance2;
    AdvStep[idx] = game.AdvStep;
    VGain[idx] = game.VGain;
    preDy[idx] = game.preDy;
    spdtgt[idx] = game.spdtgt;
    spdtgt2[idx] = game.spdtgt2;
    steerMult[idx] = game.steerMult;
    Offset[idx] = game.Offset;
    OffsetApproach[idx] = game.OffsetApproach;
    OffsetFinal[idx] = game.OffsetFinal;
    OffsetExit[idx] = game.OffsetExit;
    O1[idx] = game.O1;
    O2[idx] = game.O2;
    OP[idx] = game.OP;
    OA[idx] = game.OA;
    O3[idx] = game.O3;
    O4[idx] = game.O4;
    O5[idx] = game.O5;
    VM1[idx] = game.VM1;
    VM2[idx] = game.VM2;
    VM3[idx] = game.VM3;

    hold[idx] = 8.0;
    curidx = 0;
    Tright[idx] = game.Tright;
    Trightprev[idx] = Tright[idx];
    for(i = 0; i < MAX_GEARS; i++){
      shiftThld[idx][i] = game.shiftThld[i];
    }
    if(game.tmptmp == 1 && OffsetFinal[idx] != 0.0)
      LgfsFinal[idx] = game.LgfsFinal;
    else
      PitState[idx] = game.PitState;
  }

  if(fault_sig == 2){
    time ();
  }
  else if(fault_sig == 3){
    time ();
  }
  else if(fault_sig == 4){
    time ();
  }

  //set up TCP port: 50401
  int port = 50401;

#if 0
  int sd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if(sd < 0) printf("Can't create socket\n");

  struct sockaddr_in SerAddr, CliAddr;
  bzero(&SerAddr, sizeof(SerAddr));
  SerAddr.sin_family = AF_INET;
  SerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  SerAddr.sin_port = htons(port);
  if(bind(sd, (struct sockaddr *)&SerAddr, sizeof(SerAddr)) < 0){
    printf("Fails to bind\n");
    exit(-1);
  }

  if(listen(sd, 5) < 0)
    printf("Listen fails\n");
  printf("listening on port: %d\n", port);

  int clen;
  int new_sd = accept(sd, (struct sockaddr *)&CliAddr, (socklen_t *)&clen);
  if(new_sd < 0) printf("Accept fails\n");
#else
  int sd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if(sd < 0){ 
    printf("socket() error\n");
    exit(-1);
  }

  struct sockaddr_in name, cli_name;
  bzero(&name, sizeof(name));
  name.sin_family = AF_INET;
  name.sin_port = htons(port);
  name.sin_addr.s_addr = htonl(INADDR_ANY);
  if(bind(sd, (struct sockaddr *)&name, sizeof(name)) < 0){
    printf("bind() fails\n");
    exit(-1);
  }
  printf("listening on port: %d\n", port);
#endif
  char buf[RPC_SIZE];
  int pos = 0;

  if(fault_sig == 0){
    /* initialize track */
    length = 33 * sizeof(float) + 1 * sizeof(int);
#if 0
    if(recv(new_sd, buf, length, 0) != length)
      printf("Recv 1 fails\n");
#else
    slen = sizeof(cli_name);
    if(recvfrom(sd, buf, length, 0, (struct sockaddr *)&cli_name, &slen) != length){
      printf("receiving track fails\n");
      exit(-1);
    }
#endif
    track.len = unmarshall_float(buf, &pos);
    track.n_len = unmarshall_float(buf, &pos);
    track.entry_lg = unmarshall_float(buf, &pos);
    track.start_lg = unmarshall_float(buf, &pos);
    track.end_lg = unmarshall_float(buf, &pos);
    track.exit_lg = unmarshall_float(buf, &pos);

    idx = unmarshall_int(buf, &pos);

    VM = unmarshall_float(buf, &pos);
    Gmax = unmarshall_float(buf, &pos);
    PGain[idx] = unmarshall_float(buf, &pos);
    AGain[idx] = unmarshall_float(buf, &pos);
    PnGain[idx] = unmarshall_float(buf, &pos);
    Advance[idx] = unmarshall_float(buf, &pos);
    Advance2[idx] = unmarshall_float(buf, &pos);
    AdvStep[idx] = unmarshall_float(buf, &pos);
    VGain[idx] = unmarshall_float(buf, &pos);
    preDy[idx] = unmarshall_float(buf, &pos);
    spdtgt[idx] = unmarshall_float(buf, &pos);
    spdtgt2[idx] = unmarshall_float(buf, &pos);
    steerMult[idx] = unmarshall_float(buf, &pos);
    Offset[idx] = unmarshall_float(buf, &pos);
    OffsetApproach[idx] = unmarshall_float(buf, &pos);
    OffsetFinal[idx] = unmarshall_float(buf, &pos);
    OffsetExit[idx] = unmarshall_float(buf, &pos);
    O1[idx] = unmarshall_float(buf, &pos);
    O2[idx] = unmarshall_float(buf, &pos);
    OP[idx] = unmarshall_float(buf, &pos);
    OA[idx] = unmarshall_float(buf, &pos);
    O3[idx] = unmarshall_float(buf, &pos);
    O4[idx] = unmarshall_float(buf, &pos);
    O5[idx] = unmarshall_float(buf, &pos);
    VM1[idx] = unmarshall_float(buf, &pos);
    VM2[idx] = unmarshall_float(buf, &pos);
    VM3[idx] = unmarshall_float(buf, &pos);
  }

  ConsFactor[idx] = 0.0007 * track.len;

  if(fault_sig == 0){
    /* initialize race */
    length = 2 * sizeof(int) + (2 + MAX_GEARS) * sizeof(float);
    bzero(buf, RPC_SIZE);
#if 0
    if(recv(new_sd, buf, length, 0) != length)
      printf("Recv 2 fails\n");
#else
    if(recvfrom(sd, buf, length, 0, (struct sockaddr *)&cli_name, &slen) != length){
      printf("receiving race fails\n");
      exit(-1);
    }
#endif
    pos = 0;
    if(idx != unmarshall_int(buf, &pos)){
      printf("idx mismatch\n");
      exit(-1);
    }

    hold[idx] = 8.0;
    curidx = 0;

    Tright[idx] = unmarshall_float(buf, &pos);
    Trightprev[idx] = Tright[idx];
    int i;
    for(i = 0; i < MAX_GEARS; i++){
      shiftThld[idx][i] = unmarshall_float(buf, &pos);
    }

    if((unmarshall_int(buf, &pos) == 1) && OffsetFinal[idx] != 0.0){
      LgfsFinal[idx] = unmarshall_float(buf, &pos);
    }
    else PitState[idx] = unmarshall_int(buf, &pos); //XXX: assume sizes of float and int are equal
  }

  /* drive */
  pos = 0;
  bzero(buf, RPC_SIZE);

  union{
    int arg[2];
    unsigned char data[2 * sizeof(int)];
  }temp;

  length = sizeof(temp);
#if 0
  while(recv(new_sd, temp.data, length, 0) == length){
#else
  cpu = socket_get_sb_id ();
  if (cpu == 1) {
    socket_recovery (3);
  }
  while(recvfrom(sd, temp.data, length, 0, (struct sockaddr *)&cli_name, &slen) == length){
#endif
    length = temp.arg[0];
    int cmd = temp.arg[1];
    static int counter = 0;
    static float old_brake = 0;
    static int brake_mon = 0;

    switch(cmd){
    case RPC_CMD_DRIVE:
      //printf("drive\n");

      pos = 0;
#if 0
      if(recv(new_sd, buf, length, 0) != length){
        printf("Recv 3 fails\n");
        break;
      }
#else
      if(recvfrom(sd, buf, length, 0, (struct sockaddr *)&cli_name, &slen) != length){
        printf("receiving drive fails\n");
        break;
      }
#endif
      car.gear = unmarshall_int(buf, &pos);
      car.dammage = unmarshall_int(buf, &pos);
      car.fuel = unmarshall_float(buf, &pos);
      car.toright = unmarshall_float(buf, &pos);
      car.angle_e = unmarshall_float(buf, &pos);
      car.angle_s = unmarshall_float(buf, &pos);
      car.pos_x = unmarshall_float(buf, &pos);
      car.laps = unmarshall_int(buf, &pos);
      car.pos_y = unmarshall_float(buf, &pos);
      car.width = unmarshall_float(buf, &pos);
      car.yaw = unmarshall_float(buf, &pos);
      car.rate = unmarshall_float(buf, &pos);
      car.dimension = unmarshall_float(buf, &pos);
      car.dist = unmarshall_float(buf, &pos);
      car.speed = unmarshall_float(buf, &pos);
      car.offset = unmarshall_int(buf, &pos);
      car.angle = unmarshall_float(buf, &pos);
      car.radius = unmarshall_float(buf, &pos);
      car.spinvel[0] = unmarshall_float(buf, &pos);
      car.spinvel[1] = unmarshall_float(buf, &pos);
      car.spinvel[2] = unmarshall_float(buf, &pos);
      car.spinvel[3] = unmarshall_float(buf, &pos);
      car.light = unmarshall_int(buf, &pos);
      car.race = unmarshall_int(buf, &pos);
      car.steerlock = unmarshall_float(buf, &pos);
      car.tomiddle = unmarshall_float(buf, &pos);
      car.global_x = unmarshall_float(buf, &pos);
      car.global_y = unmarshall_float(buf, &pos);

      //printf ("offset=%d, gear=%d, laps=%d, damage=%d\n", car.offset, car.gear,
      //        car.laps, car.dammage);

      if(idx != unmarshall_int(buf, &pos)){
        printf("idx mismatch\n");
        break;
      }

      s.cur_t = unmarshall_double(buf, &pos);
      s.delta_t = unmarshall_double(buf, &pos);
      s.maxdammage = unmarshall_int(buf, &pos);
      s.state = unmarshall_int(buf, &pos);
      s.totlaps = unmarshall_int(buf, &pos);
      s.ncars = unmarshall_int(buf, &pos);
      for(i = 0; i < s.ncars; i++){
        s.cars[i].state = unmarshall_int(buf, &pos);
        s.cars[i].dist = unmarshall_float(buf, &pos);
        s.cars[i].speed = unmarshall_float(buf, &pos);
        s.cars[i].laps = unmarshall_int(buf, &pos);
        s.cars[i].toright = unmarshall_float(buf, &pos);
      }

      g_right = unmarshall_float(buf, &pos);
      g_max = unmarshall_float(buf, &pos);

      if (cpu == 1) goto no_fault;
#ifdef FAULT_1
      /* fault injection when car first passes coordinate x: 500 */
      if(fault_sig == 1 && car.global_x > 500){
        printf ("inject fault 1\n");
        close (sd);
        time ();
        exit (1);
      }

#endif

#ifdef FAULT_2
      /* refork fault when car first times passes coordinate y: 600 */
      if(fault_sig == 2 && car.global_y > 600){
        //TODO
        close (sd);
        time ();
        exit (1);
      }

#endif

#ifdef FAULT_3
      /* network corruption fault when car second times passes coordinate x: 50 */
      if(fault_sig == 3 && car.global_x < 50){
        //TODO
        close (sd);
        time ();
        exit (1);
      }

#endif

no_fault:

      RPC_drive(idx);
      unsigned int ctime = time ();
      /* 2995814 */
      while ((time () - ctime) < 35650186);

      /* try to release brake if car stops */
      if(car.speed < 3.0) counter++;
      if(counter > 100){
        car.brake = 0.0;
        //TODO: fix steer
        counter = 0;
        printf("reset\n");
      }

#if 0
      float diff = car.brake - old_brake;
      if(q_fabs(diff) < 1e-10) brake_mon++;
      else{
        old_brake = car.brake;
        brake_mon = 0;
      }
      if(brake_mon > 20){
        car.brake = car.brake / 2;  //loose half brake
        old_brake = car.brake;
        brake_mon = 0;
      }
#endif
      /* send back control info */
      bzero(buf, RPC_SIZE);
      pos = 0;
      
      marshall_int(buf, &pos, order);
      marshall_float(buf, &pos, TargetSpeed);
      marshall_int(buf, &pos, car.light);
      marshall_int(buf, &pos, car.race);
      marshall_float(buf, &pos, car.brake);
      marshall_int(buf, &pos, car.gearcmd);
      marshall_float(buf, &pos, car.accel);
      marshall_float(buf, &pos, car.steer);

      order++;

#if 0
      if(send(new_sd, buf, pos, 0) != pos)
        printf("send 2 fails\n");
#else
      if(sendto(sd, buf, pos, 0, (struct sockaddr *)&cli_name, 
        sizeof(cli_name)) != pos){
        printf("send 2 fails\n");
      }
#endif      
      break;

    case RPC_CMD_PIT:
      printf("pit\n");

      pos = 0;
#if 0
      if(recv(new_sd, buf, length, 0) != length){
        printf("Recv 3 fails\n");
        break;
      }
#else
      if(recvfrom(sd, buf, length, 0, (struct sockaddr *)&cli_name, &slen) != length){
        printf("receiving pit fails\n");
        break;
      }
#endif
      if(idx != unmarshall_int(buf, &pos)){
        printf("idx mismatch\n");
        exit(-1);
      }

      s.totlaps = unmarshall_int(buf, &pos);
      car.laps = unmarshall_int(buf, &pos);
      car.lapsbehind = unmarshall_int(buf, &pos);
      car.tank = unmarshall_float(buf, &pos);
      car.fuel = unmarshall_float(buf, &pos);
      car.dammage = unmarshall_int(buf, &pos);

      RPC_pitCmd(idx);

      /* send back control info */
      length = sizeof(int) + sizeof(float);
      bzero(buf, RPC_SIZE);
      pos = 0;
      marshall_int(buf, &pos, length);
#if 0
      if(send(new_sd, buf, sizeof(int), 0) != sizeof(int))
        printf("send 1 fails\n");
#else
      if(sendto(sd, buf, sizeof(int), 0, (struct sockaddr *)&cli_name, 
        sizeof(cli_name)) != sizeof(int)){
        printf("send 1 fails\n");
        break;
      }
#endif

      pos = 0;
      marshall_float(buf, &pos, car.pitfuel);
      marshall_int(buf, &pos, car.repair);
#if 0
      if(send(new_sd, buf, pos, 0) != pos)
        printf("send 2 fails\n");
#else
      if(sendto(sd, buf, pos, 0, (struct sockaddr *)&cli_name, 
        sizeof(cli_name)) != pos){
        printf("send 2 fails\n");
      }
#endif
      break;

    case RPC_CMD_SHUTDOWN:
      goto END;

    default:
      printf("CMD error: %d %d\n", temp.arg[0], temp.arg[1]);
    }

    length = sizeof(temp);
  }

END:
  printf("close\n");

  close(sd);
//  close(new_sd);

  return 0;
}


/* vi: set et sw=2 sts=2: */
