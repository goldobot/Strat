#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <zmq.h>

#include "comm_serializer.hpp"

#if 1 /* FIXME : DEBUG : dirty hack */
	enum class CommMessageType : uint16_t
	{
		// General messages
		Sync=0, // "goldobot" synchronization message, used to synchronize stream parser
		Heartbeat=1, // Current OS time in ms as uint32, sent every second
		Reset=2, // Sent once on startup
		CommStats=3,
		DbgPrintf=4,
		// Propulsion telemetry
		PropulsionTelemetry=8, //
		PropulsionTelemetryEx=9,
		PropulsionStateChange=10,
		// Match events
		MatchStateChange=15,
		SensorsChange=20,
		GPIODebug=21,
		// Commands
		CmdEmergencyStop=32, // Order an emergency stop
		CmdSelectSide=33, // Select side. payload is an unsigned byte, 0=green, 1=orange
		CmdEnterDebugMode=34,
		CmdExitDebugMode=35,
		MainSequenceBeginLoad=40,
		MainSequenceEndLoad=41,
		MainSequenceLoadData=42,
		MainSequenceStartSequence=43,
		// Debug mode messages
		// Robot configuration
		DbgGetOdometryConfig=64,
		DbgSetOdometryConfig=65,
		DbgGetPropulsionConfig=66,
		DbgSetPropulsionConfig=67,

		// Dynamixels debug
		DbgDynamixelsList=72,
		DbgDynamixelDescr=73,
		DbgDynamixelSetTorqueEnable=74,
		DbgDynamixelSetGoalPosition=75,
		DbgDynamixelSetTorqueLimit=76,
		DbgDynamixelGetRegisters=77,
		DbgDynamixelSetRegisters=78,

		// Propulsion debug
		DbgSetMotorsEnable=80,
		DbgSetMotorsPwm=81,
		DbgSetPropulsionEnable=82,
		DbgPropulsionSetPose=83,
		DbgPropulsionTest=84,
		DbgPropulsionExecuteTrajectory=85,
		DbgPropulsionExecuteRotation=86,
		DbgPropulsionExecuteReposition=87,
		DbgPropulsionExecutePointTo=88,
		DbgPropulsionExecuteMoveTo=89,
		PropulsionStateChanged=90,


		DbgMiscRepositionStartGreen=100,
		DbgReset=127,

		DbgArmsSetPose=160,
		DbgArmsSetCommand=161,
		DbgArmsSetTorques=162,
		DbgArmsSetSequences=163,
		DbgArmsExecuteSequence=164,
		DbgArmsGoToPosition=165,

		DbgRobotSetCommand=176,
		DbgRobotSetPoint=177,
		DbgRobotSetSequence=178,
		DbgRobotExecuteSequence=179,
		DbgRobotSetTrajectoryPoint=180,

		// FPGA
		FpgaGetVersion=256,
		FpgaDbgReadReg=257,
		FpgaDbgWriteReg=258,
		FpgaCmdServo=272,
		FpgaCmdDCMotor=288,
		FpgaCmdPumpR=289,
		FpgaCmdPumpL=290,
		FpgaCmdConveyorBelt=291,
		FpgaCmdStepper=304,
		FpgaGetStepperPos=305,
		FpgaColumnsCalib=320,
		FpgaColumnsMove=321,
		FpgaColumnsSetOffset=322,
	};

	struct PropulsionTelemetry
	{
		int16_t x;//quarters of mm
		int16_t y;
		int16_t yaw;
		int16_t speed;// mm per second
		int16_t yaw_rate;// mradian per second
		int16_t acceleration;
		int16_t angular_acceleration;
		uint16_t left_encoder;
		uint16_t right_encoder;
		int8_t left_pwm;// percents
		int8_t right_pwm;
		uint8_t state;
		uint8_t error;
	};

	struct PropulsionTelemetryEx
	{
		int16_t target_x;//quarters of mm
		int16_t target_y;
		int16_t target_yaw;
		int16_t target_speed;// mm per second
		int16_t target_yaw_rate;// mradian per second
		int16_t longitudinal_error;
		int16_t lateral_error;
		int16_t yaw_error;
		int16_t speed_error;
		int16_t yaw_rate_error;
		int32_t left_acc;
		int32_t right_acc;
	};

	extern "C" {
		extern unsigned int g_odo_thread_time_ms;
		extern unsigned int g_odo_time_ms;
		extern int          g_odo_x_mm;
		extern int          g_odo_y_mm;
		extern double       g_odo_theta_deg;
	}


#endif

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("comm_uart : Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("comm_uart : Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("comm_uart : Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("comm_uart : Error tcsetattr: %s\n", strerror(errno));
}


extern bool slave1_stop;

int comm_uart_main(const char *portname)
{
    int fd;
    void* zmq_context;
    void* pub_socket;
    void* pull_socket;
    int rc;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd<0) {
        printf("comm_uart : Cannot open uart device (%s)\n", portname);
        return -1;
    }
    set_interface_attribs(fd, B115200);

    zmq_context = zmq_init(1);

    pub_socket = zmq_socket(zmq_context, ZMQ_PUB);
    rc = zmq_bind(pub_socket, "tcp://*:3001");

    pull_socket = zmq_socket(zmq_context, ZMQ_PULL);
    rc = zmq_bind(pull_socket, "tcp://*:3002");

    zmq_pollitem_t poll_items[2];

    poll_items[0].socket = 0;
    poll_items[0].fd = fd;
    poll_items[0].events = ZMQ_POLLIN;

    poll_items[1].socket = pull_socket;
    poll_items[1].fd = 0;
    poll_items[1].events = ZMQ_POLLIN;

    unsigned char recv_buffer[4096];
    unsigned char serialize_buffer[4096];
    unsigned char tmp_buffer[1024];
    goldobot::CommDeserializer deserializer(recv_buffer, sizeof(recv_buffer));
    goldobot::CommSerializer serializer(serialize_buffer, sizeof(serialize_buffer));

    while(!slave1_stop)
    {
        zmq_poll (poll_items, 2, -1);

        if(poll_items[0].revents && ZMQ_POLLIN)
        {
            int rdlen;

            rdlen = read(fd, tmp_buffer, sizeof(tmp_buffer));
            deserializer.push_data(tmp_buffer, rdlen);

            while(deserializer.message_ready())
            {
                /*printf("comm_uart : received message \n");*/
                uint16_t recv_message_type = deserializer.message_type();
                size_t recv_message_size = deserializer.message_size();
                deserializer.pop_message(tmp_buffer, sizeof(tmp_buffer));

                zmq_send(pub_socket, (const char*)(&recv_message_type), 2, ZMQ_SNDMORE);
                zmq_send(pub_socket, (const char*)(tmp_buffer), recv_message_size, 0);

#if 1 /* FIXME : DEBUG : dirty hack */
                if(recv_message_type == 8)
                {
                    // Intercept odometric telemetry
					struct PropulsionTelemetry *my_telemetry = (struct PropulsionTelemetry *)(void *)(tmp_buffer); 
					g_odo_x_mm = my_telemetry->x/4;
					g_odo_y_mm = my_telemetry->y/4;
					g_odo_theta_deg = my_telemetry->yaw*90.0/32767;
                }           
#endif

                if(recv_message_type == 0)
                {
                    // Echo synchronization messages
                    serializer.push_message(0, (unsigned char*)"goldobot", 8);
                }           
            }
        }
        if(poll_items[1].revents && ZMQ_POLLIN)
        {            
            unsigned char buff[1024];
            size_t bytes_read = 0;
            int64_t more=1;
            size_t more_size = sizeof(more);
            while(more)
            {
                bytes_read += zmq_recv(pull_socket, buff + bytes_read, sizeof(buff) - bytes_read, 0);           
                zmq_getsockopt(pull_socket, ZMQ_RCVMORE, &more, &more_size);
            }
            buff[bytes_read] = 0;
            uint16_t message_type = *(uint16_t*)(buff);
            serializer.push_message(message_type, buff+2, bytes_read - 2);
        }
        size_t dlen = serializer.pop_data(tmp_buffer, sizeof(tmp_buffer));
        write(fd, tmp_buffer, dlen);        
    }
    zmq_term(zmq_context);

    return 0;
}
