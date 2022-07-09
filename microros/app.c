#include <stdio.h>

#include <unistd.h>

#include <math.h>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include "driver/pcnt.h"
#include "driver/timer.h"



#include <rcl/rcl.h>

#include <rcl/error_handling.h>



#include <rclc/rclc.h>

#include <rclc/executor.h>



//#include <std_msgs/msg/int64.h>
//#include <std_msgs/msg/float64.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/quaternion.h>
#include <nav_msgs/msg/odometry.h>


#ifdef ESP_PLATFORM

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"

#endif



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}



#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_IO          (25) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (8000) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 5 kHz


#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  //80MHz/16 convert counter value to seconds 80Mhz / 16it -> 1초에 5M카운트 됨.


//rcl_subscription_t subscriber;
rcl_subscription_t velocity_sub;
rcl_publisher_t odom_pub;
//rcl_publisher_t vel_L_pub;
//rcl_publisher_t test_pub;


geometry_msgs__msg__Twist velocity_msg;
geometry_msgs__msg__Quaternion quaternion;
//std_msgs__msg__Int64 motor_msg;
//std_msgs__msg__Float64 vel_L_msg;
//std_msgs__msg__Float64 vel_R_msg;
nav_msgs__msg__Odometry odom_msg;
//std_msgs__msg__Float64 test_msg;


double vel_L, vel_R, encoder_L, encoder_R;

double final_theta, final_x, final_y;
double yaw;
double dvel;
double dtheta;

double error_L;
double error_previous_L;
double error_R;
double error_previous_R;

double Kp = 1.0; //진동하기 바로 전까지 
double Ki = 150.0; // 진동하다가 멈출 때 까지
double Kd = 0.73;//더 줄여보면 좋을듯?

double P_control_L, I_control_L, D_control_L;
double P_control_R, I_control_R, D_control_R;

double Time = 0.08; //주기는 최대한 빠르게

double PID_control_L, PID_control_R;
double PWM_L, PWM_R;

double learning_reset;
int PWM = 8000;

double PID_MAX = 2.0;
typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;
/*
typedef struct {
    double theta;
    double x;
    double y;
}encoder_t;*/


//encoder_t pose_encoder_;


void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time)

{
	RCLC_UNUSED(last_call_time);



	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
		//RCSOFTCHECK(rcl_publish(&publisher2, &encoder_msg2, NULL));

	}

}
/*
void test_timer_callback(rcl_timer_t * timer, int64_t last_call_time)

{
	RCLC_UNUSED(last_call_time);



	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&test_pub, &test_msg, NULL));
		//RCSOFTCHECK(rcl_publish(&publisher2, &encoder_msg2, NULL));

	}

}*/

static void IRAM_ATTR doEncoderA(void* arg)
{

	if (gpio_get_level(39) == 1) { 
		// check channel B to see which way encoder is turning
		if (gpio_get_level(36) == 0) {  
			encoder_L = encoder_L + 1;
		} 
		else {
			encoder_L = encoder_L - 1;
		}
	}
	else   // must be a high-to-low edge on channel A                                       
	{ 
		// check channel B to see which way encoder is turning  
		if (gpio_get_level(36) == 1) {   
			encoder_L = encoder_L + 1;
		} 
		else {
			encoder_L = encoder_L - 1;
		}
	}
}

static void IRAM_ATTR doEncoderB(void* arg)
{
	if (gpio_get_level(36) == 1) { 
		// check channel B to see which way encoder is turning
		if (gpio_get_level(39) == 1) {  
			encoder_L = encoder_L + 1;
		} 
		else {
			encoder_L = encoder_L - 1;
		}
	}
	else   // must be a high-to-low edge on channel A                                       
	{ 
		// check channel B to see which way encoder is turning  
		if (gpio_get_level(39) == 0) {   
			encoder_L = encoder_L + 1;
		} 
		else {
			encoder_L = encoder_L - 1;
		}
	}
}
static void IRAM_ATTR doEncoderC(void* arg)
{
	if (gpio_get_level(35) == 1) { 
		// check channel B to see which way encoder is turning
		if (gpio_get_level(34) == 0) {  
			encoder_R = encoder_R + 1;
		} 
		else {
			encoder_R = encoder_R - 1;
		}
	}
	else   // must be a high-to-low edge on channel A                                       
	{ 
		// check channel B to see which way encoder is turning  
		if (gpio_get_level(34) == 1) {   
			encoder_R = encoder_R + 1;
		} 
		else {
			encoder_R = encoder_R - 1;
		}
	}
}

static void IRAM_ATTR doEncoderD(void* arg)
{
	if (gpio_get_level(34) == 1) { 
		// check channel B to see which way encoder is turning
		if (gpio_get_level(35) == 1) {  
			encoder_R = encoder_R + 1;
		} 
		else {
			encoder_R = encoder_R - 1;
		}
	}
	else   // must be a high-to-low edge on channel A                                       
	{ 
		// check channel B to see which way encoder is turning  
		if (gpio_get_level(35) == 0) {   
			encoder_R = encoder_R + 1;
		} 
		else {
			encoder_R = encoder_R - 1;
		}
	}
}

static void IRAM_ATTR timer_group_isr_callback(void *args)
{
    example_timer_info_t *info = (example_timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    /* Prepare basic event data that will be then sent back to task */
    if(learning_reset == 0)
    {
    float nor_L = 0;
    float nor_R = 0;
    
    nor_L = (double)encoder_L/10400; //number of rotation = 주기 당 펄스 수 / 펄스 한바퀴에 26개 * 모든엣지 4 * 감속비 100
    nor_R = (double)encoder_R/10400; //number of rotation = 주기 당 펄스 수 / 펄스 한바퀴에 26개 * 모든엣지 4 * 감속비 100
    double distance_L = 0;
    double distance_R = 0;
    
    distance_L = nor_L * 0.408407044967;//바퀴지름 130mm 바퀴 둘레 408.407044967mm = 0.408407044967m
    distance_R = nor_R * 0.408407044967;//바퀴지름 130mm 바퀴 둘레 408.407044967mm = 0.408407044967m
    
    double cur_vel_L = distance_L / Time; // 1초에 0.16m 0.16m/sec
    double cur_vel_R = distance_R / Time; // 1초에 0.16m 0.16m/sec
    //vel_L_msg.data = distance_L / 0.001; // 1초에 0.16m 0.16m/sec
    //vel_R_msg.data = distance_R / 0.001; // 1초에 0.16m 0.16m/sec
    
    double odom_distance_sum = (distance_L+distance_R)/2;
    double odom_theta = (distance_R-distance_L) / 0.45;
    
    //dtheta = theta / Time;
    
    double x;
    double y;
    
    if(learning_reset == 1)
    {
        final_theta = 0;
        final_x = 0;
        final_y = 0;
        yaw = 0;
        learning_reset = 0;
    }
    
    dvel = odom_distance_sum / Time;
    dtheta = odom_theta / Time;
    if(odom_distance_sum != 0)
    {
    	x = cos(odom_theta) * odom_distance_sum;
    	y = -sin(odom_theta) * odom_distance_sum; 
    	
    	final_x = final_x + (cos(final_theta) * x - sin(final_theta)*y);
    	final_y = final_y + (sin(final_theta) * x + cos(final_theta)*y);
    }
    
    
    if (odom_theta != 0)
    {
    	final_theta = final_theta + odom_theta;
    }
    
    if (odom_theta != 0)
    {
    	yaw = yaw + odom_theta;
    }
    if(yaw>3.14)yaw = yaw - 6.28;
    else if(yaw<-3.14)yaw = yaw + 6.28;
    
    
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    double q[4] = {0,0,0,0};
    q[0] = cy * cp * sr - sy * sp * cr;
    q[1] = sy * cp * sr + cy * sp * cr;
    q[2] = sy * cp * cr - cy * sp * sr;
    q[3] = cy * cp * cr + sy * sp * sr;
    
    
    quaternion.x = q[0];
    quaternion.y = q[1];
    quaternion.z = q[2];
    quaternion.w = q[3];
    
    odom_msg.pose.pose.position.x = final_x;
    odom_msg.pose.pose.position.y = final_y;
    odom_msg.pose.pose.position.z = yaw;
    odom_msg.pose.pose.orientation = quaternion;
    odom_msg.twist.twist.linear.x = dvel;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = dtheta;
   
    error_L = vel_L - cur_vel_L;
    error_R = vel_R - cur_vel_R;

    P_control_L = Kp * error_L;
    P_control_R = Kp * error_R;

    I_control_L = I_control_L + Ki * error_L * Time;
    I_control_R = I_control_R + Ki * error_R * Time;

    D_control_L = Kd * (error_L - error_previous_L) / Time;
    D_control_R = Kd * (error_R - error_previous_R) / Time;

    error_previous_L = error_L;
    error_previous_R = error_R;


	//PID_control_L = P_control_L + I_control_L + D_control_L;
	PID_control_R = P_control_R + I_control_R + D_control_R;

	PID_control_L = P_control_L + I_control_L + D_control_L;
	//PID_control_R = P_control_R + I_control_R;
    
    if(PID_control_L > PID_MAX) PID_control_L=PID_MAX;
    if(PID_control_R > PID_MAX) PID_control_R=PID_MAX;
    if(PID_control_L < -PID_MAX) PID_control_L=-PID_MAX;
    if(PID_control_R < -PID_MAX) PID_control_R=-PID_MAX;
    //vel_L_msg.data = cur_vel_L;
    //vel_R_msg.data = cur_vel_R;
    //test_msg.data = cur_vel_L;
    
	if(PID_control_L > 0) PWM_L = (((PID_control_L / PID_MAX) * 8000));
	else PWM_L = (((-1*PID_control_L / PID_MAX) * 8000));
    if(PID_control_R > 0) PWM_R = (((PID_control_R / PID_MAX) * 8000));
	else PWM_R = (((-1*PID_control_R / PID_MAX) * 8000));
	if(PID_control_L > 0)
		{
			if(PID_control_R > 0)
			{
				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0);
		// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);

				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, PWM_R);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
				
				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, 0);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);

				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, PWM_L);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3);
			}
			else
			{
				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, PWM_R);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);

				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
				
				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, 0);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);

				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, PWM_L);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3);
			}
		}
		else
		{
			if(PID_control_R > 0)
			{
				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);

				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, PWM_R);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
				
				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, PWM_L);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);

				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, 0);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3);
			}
			else
			{
				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, PWM_R);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);

				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
				
				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, PWM_L);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);

				ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, 0);
				// Update duty to apply the new value
				ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3);
			}
		}

    //test_msg.data = velocity; //16V 넣어야 0.2m/sec 나오는 듯 .. 무게 추가되면 더 느려질 텐데 
    /*
    double sl = velocity_L * ( 0.13 / 2.0 ) * 0.0004; // 속도 * (바퀴지름 / 2) * 주기
    double sr = velocity_R * ( 0.13 / 2.0 ) * 0.0004;
    double ssum = sl + sr;
    double sdiff = sr - sl;
    
    test_msg.data = velocity_L;
    
    double dx = ( ssum ) /2.0 * cos ( pose_encoder_.theta + ( sdiff ) / ( 2.0* 0.45 ) );
    double dy = ( ssum ) /2.0 * sin ( pose_encoder_.theta + ( sdiff ) / ( 2.0* 0.45 ) );
    double dtheta = ( sdiff ) / 0.45; // sdiff / 바퀴 사이 거리
    
    
    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;
    
    
    double w = dtheta/0.0004;
    double v = sqrt ( dx*dx+dy*dy ) / 0.0004;
    
    
    Quaternion qt; 
    Vector3 vt;
    qt.setRPY(0, 0, pose_encoder_.theta);
    vt = Vector3(pose_encoder_.x, pose_encoder_.y, 0);
    
    */
    encoder_L=0; // 1초에 한번씩 0으로 초기화
    encoder_R=0; // 1초에 한번씩 0으로 초기화
    }
    else
    {
	double stop_move_L = (((double)encoder_L/10400)*0.408407044967)/Time; //number of rotation = 주기 당 펄스 수 / 펄스 한바퀴에 26개 * 모든엣지 4 * 감속비 100
	double stop_move_R = (((double)encoder_R/10400)*0.408407044967)/Time; //number of rotation = 주기 당 펄스 수 / 펄스 한바퀴에 26개 * 모든엣지 4 * 감속비 100
	
	final_theta = 0;
	final_x = 0;
	final_y = 0;
	yaw = 0;
	ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0);
	// Update duty to apply the new value
	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);

	ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0);
	// Update duty to apply the new value
	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);

	ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, 0);
	// Update duty to apply the new value
	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);

	ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, 0);
	// Update duty to apply the new value
	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3);
	error_L = 0;
	error_previous_L = 0;
	error_R = 0;
	error_previous_R = 0;
	P_control_L = 0;
	I_control_L = 0; 
	D_control_L = 0;
	P_control_R = 0;
	I_control_R = 0;
	D_control_R = 0;
	PID_control_L = 0;
	PID_control_R = 0;
	PWM_L = 0;
	PWM_R = 0;
	encoder_L=0; // 1초에 한번씩 0으로 초기화
	encoder_R=0; // 1초에 한번씩 0으로 초기화
	if ((stop_move_L + stop_move_R) == 0)learning_reset = 0;
    
    }

    if (!info->auto_reload) {
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }

    /* Now just send the event data back to the main program task */

}

void vel_sub_callback(const void * msgin)//ros2 topic pub --rate 1 /microROS/velocity geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
{
	const geometry_msgs__msg__Twist * vel_msg = (const geometry_msgs__msg__Twist *)msgin;
	double linear = vel_msg->linear.x;
	double angular = vel_msg->angular.z;
	vel_L = linear - angular * 0.45 / 2.0; //
	vel_R = linear + angular * 0.45 / 2.0; //
	if(linear == 0.0) 
	{
		learning_reset = 1;
	}
	else
	{
		learning_reset = 0;
	}
}

//선속도 m/s = 고정. 각속도 rad/s를 sub받는다.
// 

void appMain(void * arg)

{
	timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE, //init 하자마자 타이머 시작할지 말지.
        .alarm_en = TIMER_ALARM_EN, //일정 카운트에 도달할 시 작동함
	.auto_reload = true,
	}; // default clock source is APB
	timer_init(TIMER_GROUP_0, TIMER_0, &config); //타이머 인터럽트 설정

	/* Timer's counter will initially start from value below.
	Also, if auto_reload is set, this value will be automatically reload on alarm */
	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0); //초기값 0으로 설정

	/* Configure the alarm value and the interrupt on alarm. */
	timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 0.08 * TIMER_SCALE); //설정한 값이 되면 인터럽트 실행 0.0004s
	timer_enable_intr(TIMER_GROUP_0, TIMER_0); //인터럽트 가능하게 설정

	example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
	timer_info->timer_group = TIMER_GROUP_0;
	timer_info->timer_idx = TIMER_0;
	timer_info->auto_reload = true;
	timer_info->alarm_interval = 1;
	timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, timer_info, 0); //인터럽트 함수 배정

	timer_start(TIMER_GROUP_0, TIMER_0);//타이머 시작
	
	
	gpio_config_t io_conf;

	// 상승 에지 인터럽트
	io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
	// 핀의 비트 마스크
	io_conf.pin_bit_mask = 1ULL << GPIO_NUM_39;
	// 입력 모드로 설정
	io_conf.mode = GPIO_MODE_INPUT;
	// 풀업 모드 활성화
	io_conf.pull_up_en = 1 ;
	// io_conf.pull_down_en = 1;
	gpio_config (&io_conf);


	// 상승 에지 인터럽트
	io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
	// 핀의 비트 마스크
	io_conf.pin_bit_mask = 1ULL << GPIO_NUM_36;
	// 입력 모드로 설정
	io_conf.mode = GPIO_MODE_INPUT;
	// 풀업 모드 활성화
	io_conf.pull_up_en = 1 ;
	// io_conf.pull_down_en = 1;
	gpio_config (&io_conf);
	

	// 상승 에지 인터럽트
	io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
	// 핀의 비트 마스크
	io_conf.pin_bit_mask = 1ULL << GPIO_NUM_34; //ULL = Unsigned long long
	// 입력 모드로 설정
	io_conf.mode = GPIO_MODE_INPUT;
	// 풀업 모드 활성화
	io_conf.pull_up_en = 1 ;
	// io_conf.pull_down_en = 1;
	gpio_config (&io_conf);
	

	// 상승 에지 인터럽트
	io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
	// 핀의 비트 마스크
	io_conf.pin_bit_mask = 1ULL << GPIO_NUM_35;
	// 입력 모드로 설정
	io_conf.mode = GPIO_MODE_INPUT;
	// 풀업 모드 활성화
	io_conf.pull_up_en = 1 ;
	// io_conf.pull_down_en = 1;
	gpio_config (&io_conf);


	
	
	gpio_install_isr_service(0); // 인터럽트 우선순위 0번. intr priority level, esp_intr_alloc.h

	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_NUM_39, doEncoderA, (void*) GPIO_NUM_39);
	gpio_isr_handler_add(GPIO_NUM_36, doEncoderB, (void*) GPIO_NUM_36);
	gpio_isr_handler_add(GPIO_NUM_35, doEncoderC, (void*) GPIO_NUM_35);
	gpio_isr_handler_add(GPIO_NUM_34, doEncoderD, (void*) GPIO_NUM_34);



	
	ledc_timer_config_t ledc_timer = {
        	.speed_mode       = LEDC_MODE,
        	.timer_num        = LEDC_TIMER_1,
        	.duty_resolution  = LEDC_DUTY_RES,
        	.freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        	.clk_cfg          = LEDC_AUTO_CLK
    	};
	ledc_timer_config(&ledc_timer);

    	ledc_channel_config_t ledc_channel0 = {
		.speed_mode     = LEDC_MODE,
		.channel        = LEDC_CHANNEL_0,
		.timer_sel      = LEDC_TIMER_1,
		.gpio_num       = 26,
		.duty           = 0, // Set duty to 0%
		.hpoint         = 0
    	};
    	ledc_channel_config(&ledc_channel0);

    	ledc_channel_config_t ledc_channel1 = {
		.speed_mode     = LEDC_MODE,
		.channel        = LEDC_CHANNEL_1,
		.timer_sel      = LEDC_TIMER_1,
		.gpio_num       = 25,
		.duty           = 0, // Set duty to 0%
		.hpoint         = 0
    	};
    	ledc_channel_config(&ledc_channel1);

    	ledc_channel_config_t ledc_channel2 = {
		.speed_mode     = LEDC_MODE,
		.channel        = LEDC_CHANNEL_2,
		.timer_sel      = LEDC_TIMER_1,
		.gpio_num       = 17,
		.duty           = 0, // Set duty to 0%
		.hpoint         = 0
    	};
    	ledc_channel_config(&ledc_channel2);


    	ledc_channel_config_t ledc_channel3 = {
		.speed_mode     = LEDC_MODE,
		.channel        = LEDC_CHANNEL_3,
		.timer_sel      = LEDC_TIMER_1,
		.gpio_num       = 16,
		.duty           = 0, // Set duty to 0%
		.hpoint         = 0
    	};
    	ledc_channel_config(&ledc_channel3);



	rcl_allocator_t allocator = rcl_get_default_allocator();

	// create init_options

	rclc_support_t support;

	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node

	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "motor", "esp32", &support));
	// create publisher

	RCCHECK(rclc_publisher_init_default(
		&odom_pub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
		"/microROS/odom"));
		
		
	RCCHECK(rclc_subscription_init_default(
		&velocity_sub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"/cmd_vel"));

	/*RCCHECK(rclc_subscription_init_default(
		&test_pub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
		"/microROS/test"));*/
	// create timer,
	geometry_msgs__msg__Twist__init(&velocity_msg);
	nav_msgs__msg__Odometry__init(&odom_msg);
	//std_msgs__msg__Float64__init(&test_msg);
	//rcl_timer_t test_timer;
	rcl_timer_t odom_timer;

	const unsigned int timer_timeout = 0.001; //publish 전송 주기 1nsec
		
	RCCHECK(rclc_timer_init_default(

		&odom_timer,

		&support,

		RCL_MS_TO_NS(timer_timeout),

		odom_timer_callback));

		
	/*RCCHECK(rclc_timer_init_default(

		&test_timer,

		&support,

		RCL_MS_TO_NS(timer_timeout),

		test_timer_callback));*/
	// create executor

	rclc_executor_t executor;



	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
	//RCCHECK(rclc_executor_add_timer(&executor, &test_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &velocity_sub, &velocity_msg, &vel_sub_callback, ON_NEW_DATA));



	rclc_executor_spin(&executor);



	// free resources

	RCCHECK(rclc_executor_fini(&executor));
	RCCHECK(rcl_publisher_fini(&odom_pub, &node));
	//RCCHECK(rcl_publisher_fini(&test_pub, &node));
	RCCHECK(rcl_timer_fini(&odom_timer));
	//RCCHECK(rcl_timer_fini(&test_timer));
	RCCHECK(rcl_node_fini(&node));
	RCCHECK(rclc_support_fini(&support));
	RCCHECK(rcl_subscription_fini(&velocity_sub, &node));
	geometry_msgs__msg__Twist__fini(&velocity_msg);
	nav_msgs__msg__Odometry__fini(&odom_msg);
	//std_msgs__msg__Float64__fini(&test_msg);



  	vTaskDelete(NULL);

}
