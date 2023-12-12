

#ifndef my_platform_h
#define my_platform_h

/*****************************************************************************************************/
// Pinbelegung             (GPIO)
#define DC_Motor1_Encoder_A	(0) //Pin1
#define DC_Motor1_Encoder_B	(1)	//Pin2
//ground				        //Pin3
#define Stepper1_A1		    (2)	//Pin4
#define Stepper1_B1		    (3)	//Pin5
#define Stepper1_A2		    (4)	//Pin6	
#define Stepper1_B2		    (5)	//Pin7
//ground				        //Pin8
#define Stepper2_A1		    (6)	//Pin9
#define Stepper2_B1		    (7)	//Pin10
#define Stepper2_A2		    (8)	//Pin11	
#define Stepper2_B2		    (9)	//Pin12
//ground				        //Pin13
#define DC_Motor1_PWM       (10)//Pin14
#define DC_Motor1_In1       (11)//Pin15
#define DC_Motor1_In2       (12)//Pin16
#define DC_Motor2_In2       (13)//Pin17
//ground				        //Pin18
#define DC_Motor2_In1       (14)//Pin19
#define DC_Motor2_PWM       (15)//Pin20


#define DC_Motor2_Encoder_A	(16)//Pin21
#define DC_Motor2_Encoder_B	(17)//Pin22
//ground				        //Pin23
#define Button1             (18)//Pin24
#define Button2             (19)//Pin25
#define Button3             (20)//Pin26
#define Switch              (21)//Pin27
//ground for buttons	        //Pin28
//                              //Pin29
//                              //Pin30
#define Potentiometer1      (26)//Pin31
#define Potentiometer2      (27)//Pin32
//ground for ADC                //Pin33

/*****************************************************************************************************/
//Stepper-Motor Parameter
#define Steps_Per_Revolution    (2048)
//Encoder Parameter
#define Puls_Per_Revolution     (2752)

//PID Parameter
#define PID_Kp  (400)
#define PID_Ki  (600)
#define PID_Ti  (0.5)
#define PID_Kd  (0)
#define PID_N   (10)


void My_Platform_Init(void);

void My_Platform_Test(void);


#endif