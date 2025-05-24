#include <Common/Component.h>
#include <Actuators/StepperMotor.h>

#include <L6474_config.h>
#include <L6474_def.h>
#include <motor_def.h>
#include <L6474.h>

#include <stdlib.h>
#include <stdio.h>

#include <EtherShield.h>
#include <cstring>

#define STEPS_1 (400 * 8)   /* 1 revolution given a 400 steps motor configured at 1/8 microstep mode. */

/* Delay in milliseconds. */
#define DELAY_1 1000
#define DELAY_2 2000
#define DELAY_3 6000
#define DELAY_4 8000

/* Speed in pps (Pulses Per Second).
   In Full Step mode: 1 pps = 1 step/s).
   In 1/N Step Mode:  N pps = 1 step/s). */
#define SPEED_1 2400
#define SPEED_2 1200

L6474_init_t init = {
    160,                              /* Acceleration rate in pps^2. Range: (0..+inf). */
    160,                              /* Deceleration rate in pps^2. Range: (0..+inf). */
    1600,                             /* Maximum speed in pps. Range: (30..10000]. */
    800,                              /* Minimum speed in pps. Range: [30..10000). */
    250,                              /* Torque regulation current in mA. Range: 31.25mA to 4000mA. */
    L6474_OCD_TH_750mA,               /* Overcurrent threshold (OCD_TH register). */
    L6474_CONFIG_OC_SD_ENABLE,        /* Overcurrent shutwdown (OC_SD field of CONFIG register). */
    L6474_CONFIG_EN_TQREG_TVAL_USED,  /* Torque regulation method (EN_TQREG field of CONFIG register). */
    L6474_STEP_SEL_1_8,               /* Step selection (STEP_SEL field of STEP_MODE register). */
    L6474_SYNC_SEL_1_2,               /* Sync selection (SYNC_SEL field of STEP_MODE register). */
    L6474_FAST_STEP_12us,             /* Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us. */
    L6474_TOFF_FAST_8us,              /* Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us. */
    3,                                /* Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us. */
    21,                               /* Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us. */
    L6474_CONFIG_TOFF_044us,          /* Target Swicthing Period (field TOFF of CONFIG register). */
    L6474_CONFIG_SR_320V_us,          /* Slew rate (POW_SR field of CONFIG register). */
    L6474_CONFIG_INT_16MHZ,           /* Clock setting (OSC_CLK_SEL field of CONFIG register). */
    L6474_ALARM_EN_OVERCURRENT |
    L6474_ALARM_EN_THERMAL_SHUTDOWN |
    L6474_ALARM_EN_THERMAL_WARNING |
    L6474_ALARM_EN_UNDERVOLTAGE |
    L6474_ALARM_EN_SW_TURN_ON |
    L6474_ALARM_EN_WRONG_NPERF_CMD    /* Alarm (ALARM_EN register). */
};

L6474 *motor;

void flag_irq_handler(void)
{
    /* Set ISR flag. */
    motor->isr_flag = TRUE;

    /* Get the value of the status register. */
    unsigned int status = motor->get_status();

    /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed. */
    /* This often occures when a command is sent to the L6474 while it is not in HiZ state. */
    if ((status & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD) {
        printf("    WARNING: \"FLAG\" interrupt triggered. Non-performable command detected when updating L6474's registers while not in HiZ state.\r\n");
    }

    /* Reset ISR flag. */
    motor->isr_flag = FALSE;
}

extern "C" void L6474_create()
{
	motor = new L6474;
	if(motor->init(&init) != COMPONENT_OK)
	{
		printf("failed to initialize\r\n");
	}
	else
	{
		printf("ok\r\n");
	}

	motor->attach_flag_irq(&flag_irq_handler);
	motor->enable_flag_irq();
}



int position = 0;
int diff = 0;
int speed = 1000;
extern "C" void L6474_move()
{
    motor->attach_flag_irq(&flag_irq_handler);
    motor->enable_flag_irq();

     motor->set_parameter(L6474_TVAL, 500);


     position = motor->get_position();

     motor->move(StepperMotor::FWD, STEPS_1 / 8);
     motor->wait_while_active();

     diff = motor->get_position() - position;
     position = motor->get_position();

     //motor->move(StepperMotor::BWD, STEPS_1 / 8);
     //motor->wait_while_active();
     //position = motor->get_position();

     motor->set_max_speed(10000);
     motor->set_min_speed(1000);
}



extern "C" void L6474_run()
{

	motor->set_max_speed(20000);

	speed = 1000;
	float s = 1;
	const float min = 5000, max = 9000;

	while(true)
	{
		for(int i =0; i<10;++i) {
			motor->move(StepperMotor::FWD, STEPS_1 / 8);
			motor->wait_while_active();
		}

		if(speed == min)
		{
			s = 1;
		}
		else if(speed == max)
		{
			s = -1;
		}

		speed = speed + 1000 * s;
		motor->set_min_speed(speed);
		printf("speed: %i\r\n", speed);
	}

}

extern SPI_HandleTypeDef hspi1;

extern "C" int ES_readMacAddr(char *mac);

void CS_Low()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

void CS_High()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

void RESET_Low()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

void RESET_High()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}

#define MAX_CHANNELS	16

#define NET_BUF_SIZE (1<<10)
#define UART_BUF_SIZE (NET_BUF_SIZE << 2)

#define MAX(a,b) ((a)>(b)?(a):(b))
#define BUF_SIZE MAX(NET_BUF_SIZE, UART_BUF_SIZE)

typedef uint16_t dataitem_t;

struct sensorcommand {
	dataitem_t	command_id;
	dataitem_t	 sensor_id;
	dataitem_t	channels;
	dataitem_t	channel[MAX_CHANNELS];
};


typedef struct sensorcommand sensorcommand_t;

struct collectorcommand {
	dataitem_t	command_id;
	dataitem_t	 sensor_id;
};
typedef struct collectorcommand collectorcommand_t;

enum command_id {
	CMD_GETDATA = 1,
	CMD_SETDATA = 2,
};


#define NET_HEADERS_LENGTH (ETH_HEADER_LEN + IP_HEADER_LEN + UDP_HEADER_LEN)
#define RECV_TIMEOUT 1000

static uint32_t ticked = 0;
uint8_t net_sendbuf[NET_BUF_SIZE + 1];
uint8_t net_recvbuf[NET_BUF_SIZE + 1];

void main_tick() {
	ticked++;

	return;
}

uint16_t get_udp_data_len(uint8_t *buf)
{
	int16_t i;
	i=(((int16_t)buf[IP_TOTLEN_H_P])<<8)|(buf[IP_TOTLEN_L_P]&0xff);
	i-=IP_HEADER_LEN;
	i-=8;
	if (i<=0){
		i=0;
	}
	return((uint16_t)i);
}

static uint16_t info_data_len = 0;
uint16_t packetloop_icmp_udp(uint8_t *buf,uint16_t plen)
{
	if(eth_type_is_arp_and_my_ip(buf,plen)){
		if (buf[ETH_ARP_OPCODE_L_P]==ETH_ARP_OPCODE_REQ_L_V){
			// is it an arp request
			make_arp_answer_from_request(buf);
		}
		return(0);
	}
	// check if ip packets are for us:
	if(eth_type_is_ip_and_my_ip(buf,plen)==0){
		return(0);
	}

	if(buf[IP_PROTO_P]==IP_PROTO_ICMP_V && buf[ICMP_TYPE_P]==ICMP_TYPE_ECHOREQUEST_V){
		make_echo_reply_from_request(buf,plen);
		return(0);
	}

	if (buf[IP_PROTO_P]==IP_PROTO_UDP_V) {
		info_data_len=get_udp_data_len(buf);
		return(IP_HEADER_LEN+8+14);
	}

	return(0);
}

extern UART_HandleTypeDef huart2;

static uint8_t   local_mac[] = {0x02, 0x03, 0x04, 0x05, 0x06, 0x09};
static uint8_t  remote_mac[] = {0x00, 0x1b, 0x21, 0x39, 0x37, 0x26};
//uint8_t   local_ip[]  = {10,  4, 33, 126};
//uint8_t   local_ip[]  = {192,  168, 4, 10};
static uint8_t   local_ip[]  = {10,  0, 0, 29};
static uint8_t  remote_ip[]  = {10,  4, 33, 242};

#define BUFFER_SIZE 550
static uint8_t buf[BUFFER_SIZE+1] {0};

#define PSTR(s) s

uint16_t http200ok(void)
{
	return(fill_tcp_data(buf,0,PSTR("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n")));
}
uint16_t print_webpage(uint8_t *buf)
{
  uint16_t plen=0;
  plen+=http200ok();
  plen+=fill_tcp_data(buf,plen,PSTR("<pre>"));
  plen+=fill_tcp_data(buf,plen,PSTR("Hi!\nYour web server works great."));
  plen+=fill_tcp_data(buf,plen,PSTR("</pre>\n"));
  return(plen);
}

extern "C" void ENC28J60_test()
{

	fputs("===============\r\n", stdout);
	fputs("START\r\n",stdout);

	CS_High();

	RESET_Low();
	HAL_Delay(1);
	RESET_High();

	fputs("Setting SPI: ",stdout);

	ES_enc28j60SpiInit(&hspi1);
	fputs("OK\r\n",stdout);


	uint8_t test_macaddr[6]{
	};

	fputs("ENC28J60 Init: ",stdout);
	ES_enc28j60Init(local_mac);
	fputs("OK\r\n",stdout);

	fputs("MAC addr is now: ", stdout);
	ES_readMacAddr((char*)test_macaddr);
	for(int i =0; i<6; ++i)
	{
		printf("0x%x:", test_macaddr[i]);
	}
	fputs("\r\n",stdout);




	uint8_t enc28j60_rev = ES_enc28j60Revision();
	printf("Revision: ");
	if (enc28j60_rev <= 0)
	{
		printf("error\r\n");
		exit(EXIT_FAILURE);
	}

	printf("%i\r\n", enc28j60_rev);
	printf("INIT arp udp tcp: ");
	ES_init_ip_arp_udp_tcp(local_mac, local_ip, 80);
	printf("OK\r\n");

	uint16_t dat_p;
	uint16_t plen;

	//while(1)
	//{
	//	client_icmp_request(buf,dest_ip);
	//	printf("ping respone: %s\r\n", buf);
	//}

	while(1)
	{

  /* USER CODE END WHILE */
		// read packet, handle ping and wait for a tcp packet:
		dat_p=ES_packetloop_icmp_tcp(buf, ES_enc28j60PacketReceive(BUFFER_SIZE, buf));
		//plen=enc28j60PacketReceive(BUFFER_SIZE, buf);

		if(dat_p==0)
		{
			// no http request
			continue;
		}

		//printf("dat_p: %i\r\n", dat_p);
		//printf("str: %s\r\n", &buf[TCP_CHECKSUM_L_P+3+4]);
		//for(int i =dat_p; i<BUFFER_SIZE; ++i)
		//{
		//	printf("%c\r\n", buf[i]);
		//}

		// just one web page in the "root directory" of the web server
		if (strncmp("/ ",(char *)&(buf[TCP_CHECKSUM_L_P+3+4]),2)==0){

			printf("GET /\r\n");
			dat_p=print_webpage(buf);
			//printf("str: %s\r\n", &buf[TCP_CHECKSUM_L_P+3]);
			goto SENDTCP;
		}

		// tcp port 80 begin
		if (strncmp("GET ",(char *)&(buf[dat_p]),4)==0)
		{
			printf("GET\r\n");
		//	// head, post and other methods:
			dat_p=http200ok();
			dat_p+=fill_tcp_data(buf,dat_p,PSTR("<h1>200 OK</h1>"));
			//printf("%str: %s", &buf[TCP_CHECKSUM_L_P+3]);
			goto SENDTCP;
		}


		dat_p=fill_tcp_data(buf,0,PSTR("HTTP/1.0 401 Unauthorized\r\nContent-Type: text/html\r\n\r\n<h1>401 Unauthorized</h1>"));
		goto SENDTCP;

SENDTCP:

		www_server_reply(buf,dat_p);

	}


     /*
        
        
	uint8_t   local_mac[] = {0x02, 0x03, 0x04, 0x05, 0x06, 0x09};
	uint8_t  remote_mac[] = {0x00, 0x1b, 0x21, 0x39, 0x37, 0x26};


	ES_enc28j60SpiInit(&hspi1);
	ES_enc28j60Init(local_mac);

	uint8_t enc28j60_rev = ES_enc28j60Revision();
	if (enc28j60_rev <= 0)
		error(2, 0);

	ES_init_ip_arp_udp_tcp(local_mac, local_ip, 80);
        
        */


	//enc28j60.DumpBank1();

	//enc28j60.CS_High();
	//enc28j60.RESET_Low();
	//HAL_Delay(100);
	//enc28j60.RESET_High();
	//enc28j60.CS_High();

	//enc28j60._setBank(ERXSTL);
	//printf("OK\n\r");
	//enc28j60.writeOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
	//enc28j60.writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1 | ECON1_BSEL0));

	//HAL_Delay(RESET_TIME_OUT_MS);
	//enc28j60.DumpBank1();

	//HAL_Delay(200);
	//enc28j60.Init();

}
