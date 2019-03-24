#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <getopt.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/termios.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>

#define __USE_GNU
#include <pthread.h>

#ifndef EMBED
#include <sys/select.h>
#endif


/*****************************************************************************/

char *version = "1.1.0";

/*****************************************************************************/

/*
 *    Define some parity flags, internal use only.
 */
#define    PARITY_NONE    0
#define    PARITY_EVEN    1
#define    PARITY_ODD     2

/*
 *    Default port settings.
 */
int        clocal;
int        hardware;
int        software;
int        passflow=0;
int        parity = PARITY_NONE;
int        databits = 8;
int        twostopb;
unsigned int    baud = 9600;

int        translate;
int        ocasemode, icasemode;

char        *devname;
char        *filename;
int        verbose = 1;
int        net_connection = 0;
int        gotdevice;
int        ifd, ofd;
int        rfd;
FILE       *dump_file;

/*
 *    Working termios settings.
 */
struct termios    savetio_local;
struct termios    savetio_remote;

/*
 *    Signal handling.
 */
struct sigaction    sact;

/*
 *    Temporary buffer to use when working.
 */
unsigned char    ibuf[512];
unsigned char    obuf[1024];

/*****************************************************************************/

/*
 *    Baud rate table for baud rate conversions.
 */
typedef struct baudmap {
    unsigned int    baud;
    unsigned int    flag;
} baudmap_t;


struct baudmap    baudtable[] = {
    { 0, B0 },
    { 50, B50 },
    { 75, B75 },
    { 110, B110 },
    { 134, B134 },
    { 150, B150 },
    { 200, B200 },
    { 300, B300 },
    { 600, B600 },
    { 1200, B1200 },
    { 1800, B1800 },
    { 2400, B2400 },
    { 4800, B4800 },
    { 9600, B9600 },
    { 19200, B19200 },
    { 38400, B38400 },
    { 57600, B57600 },
    { 115200, B115200 },
    { 230400, B230400 },
    { 460800, B460800 }
};

#define    NRBAUDS        (sizeof(baudtable) / sizeof(struct baudmap))


#if 0 /* RAW ODOMETRY */
typedef enum _read_odo_state {
    READ_ODO_STATE_INIT,

    READ_ODO_STATE_HEAD0,
    READ_ODO_STATE_HEAD1,
    READ_ODO_STATE_HEAD2,
    READ_ODO_STATE_HEAD3,

    READ_ODO_STATE_TS0,
    READ_ODO_STATE_TS1,
    READ_ODO_STATE_TS2,
    READ_ODO_STATE_TS3,

    READ_ODO_STATE_ODOR0,
    READ_ODO_STATE_ODOR1,
    READ_ODO_STATE_ODOR2,
    READ_ODO_STATE_ODOR3,

    READ_ODO_STATE_ODOL0,
    READ_ODO_STATE_ODOL1,
    READ_ODO_STATE_ODOL2,
    READ_ODO_STATE_ODOL3,
} read_odo_state_t;
#endif

#if 1 /* NUCLEO ESTIMATED POSITION VECTOR */
typedef enum _read_odo_state {
    READ_ODO_STATE_INIT,

    READ_ODO_STATE_HEAD0,
    READ_ODO_STATE_HEAD1,
    READ_ODO_STATE_HEAD2,
    READ_ODO_STATE_HEAD3,

    READ_ODO_STATE_TS0,
    READ_ODO_STATE_TS1,
    READ_ODO_STATE_TS2,
    READ_ODO_STATE_TS3,

    READ_ODO_STATE_X0,
    READ_ODO_STATE_X1,

    READ_ODO_STATE_Y0,
    READ_ODO_STATE_Y1,

    READ_ODO_STATE_THETA0,
    READ_ODO_STATE_THETA1,
    READ_ODO_STATE_THETA2,
    /*READ_ODO_STATE_THETA3,*/
} read_odo_state_t;
#endif

read_odo_state_t read_odo_state;

/*****************************************************************************/

volatile unsigned int g_odo_thread_time_ms;
volatile unsigned int g_odo_time_ms;
volatile unsigned int g_odo_x_mm;
volatile unsigned int g_odo_y_mm;
volatile double       g_odo_theta_deg;


volatile int read_odo_flag_running = 0;

void exit_thread(int err_code)
{
    while (1) {
        read_odo_flag_running = 0;
        pthread_yield();
    }
}


/*
 *    Verify that the supplied baud rate is valid.
 */

int baud2flag(unsigned int speed)
{
    int    i;

    for (i = 0; (i < NRBAUDS); i++) {
        if (speed == baudtable[i].baud)
            return(baudtable[i].flag);
    }
    return(-1);
}

/*****************************************************************************/

void restorelocaltermios(void)
{
    if (tcsetattr(1, TCSAFLUSH, &savetio_local) < 0) {
        fprintf(stderr, "ERROR: local tcsetattr(TCSAFLUSH) failed, "
            "errno=%d\n", errno);
    }
}

/*****************************************************************************/

void savelocaltermios(void)
{
    if (tcgetattr(1, &savetio_local) < 0) {
        fprintf(stderr, "ERROR: local tcgetattr() failed, errno=%d\n",
            errno);
        exit_thread(0);
    }
}

/*****************************************************************************/

void restoreremotetermios(void)
{
	/*
	 *	This can fail if remote hung up, don't check return status.
	 */
	tcsetattr(rfd, TCSAFLUSH, &savetio_remote);
}

/*****************************************************************************/

int saveremotetermios(void)
{
	if (tcgetattr(rfd, &savetio_remote) < 0) {
		fprintf(stderr, "ERROR: remote tcgetattr() failed, errno=%d\n",
			errno);
		return(0);
	}
	return(1);
}

/*****************************************************************************/

/*
 *    Set local port to raw mode, no input mappings.
 */

int setlocaltermios(void)
{
    struct termios    tio;

    if (tcgetattr(1, &tio) < 0) {
        fprintf(stderr, "ERROR: local tcgetattr() failed, errno=%d\n",
            errno);
        exit_thread(1);
    }

    if (passflow)
        tio.c_iflag &= ~(ICRNL|IXON);
    else
        tio.c_iflag &= ~ICRNL;
#if 0 /* FIXME : TODO : usefull? (ca bloque le CTRL_C sur la console sinon..) */
    tio.c_lflag = 0;
#endif
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(1, TCSAFLUSH, &tio) < 0) {
        fprintf(stderr, "ERROR: local tcsetattr(TCSAFLUSH) failed, "
            "errno=%d\n", errno);
        exit_thread(1);
    }
    return(0);
}

int setremotetermios(void)
{
	struct termios	tio;

	memset(&tio, 0, sizeof(tio));
	tio.c_cflag = CREAD | HUPCL | baud2flag(baud);

	if (clocal)
		tio.c_cflag |= CLOCAL;

	switch (parity) {
	case PARITY_ODD:	tio.c_cflag |= PARENB | PARODD; break;
	case PARITY_EVEN:	tio.c_cflag |= PARENB; break;
	default:		break;
	}

	switch (databits) {
	case 5:		tio.c_cflag |= CS5; break;
	case 6:		tio.c_cflag |= CS6; break;
	case 7:		tio.c_cflag |= CS7; break;
	default:	tio.c_cflag |= CS8; break;
	}
	
	if (twostopb)
		tio.c_cflag |= CSTOPB;

	if (software)
		tio.c_iflag |= IXON | IXOFF;
	if (hardware)
		tio.c_cflag |= CRTSCTS;

	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 0;

	if (tcsetattr(rfd, TCSAFLUSH, &tio) < 0) {
		fprintf(stderr, "ERROR: remote tcsetattr(TCSAFLUSH) failed, "
			"errno=%d\n", errno);
		return(0);
	}
	return(1);
}


/*****************************************************************************/

/*****************************************************************************/

void sighandler(int signal)
{
    printf("\n\nGot signal %d!\n", signal);
    printf("Cleaning up...");
    restorelocaltermios();
    printf("Done\n");
    exit_thread(1);
}

/*****************************************************************************/

/*
 *    Code to support 5bit translation to ascii.
 *    Whacky 5 bit system used on some older teletype equipment.
 */
unsigned char    ascii2code[128] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9a,
    0x00, 0x00, 0x08, 0x00, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x00, 0x00, 0x00, 0x85, 0x96, 0x00, 0x94,
    0x97, 0x89, 0x00, 0x91, 0x86, 0x98, 0x87, 0x97,
    0x8d, 0x9d, 0x99, 0x90, 0x8a, 0x81, 0x95, 0x9c,
    0x8c, 0x83, 0x8e, 0x00, 0x00, 0x8f, 0x00, 0x93,
    0x8b, 0x18, 0x13, 0x0e, 0x12, 0x10, 0x16, 0x0a,
    0x05, 0x0c, 0x1a, 0x1e, 0x09, 0x07, 0x06, 0x03,
    0x0d, 0x1d, 0x0a, 0x14, 0x01, 0x1c, 0x0f, 0x19,
    0x17, 0x15, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x18, 0x13, 0x0e, 0x12, 0x10, 0x16, 0x0a,
    0x05, 0x0c, 0x1a, 0x1e, 0x09, 0x07, 0x06, 0x03,
    0x0d, 0x1d, 0x0a, 0x14, 0x01, 0x1c, 0x0f, 0x19,
    0x17, 0x15, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00,
};

unsigned char    lower2ascii[32] = {
    0x00, 0x74, 0x0d, 0x6f, 0x20, 0x68, 0x6e, 0x6d,
    0x0a, 0x6c, 0x72, 0x67, 0x69, 0x70, 0x63, 0x76,
    0x65, 0x7a, 0x64, 0x62, 0x73, 0x79, 0x66, 0x78,
    0x61, 0x77, 0x6a, 0x80, 0x75, 0x71, 0x6b, 0x80,
};

unsigned char    upper2ascii[32] = {
    0x00, 0x35, 0x0d, 0x39, 0x20, 0x24, 0x2c, 0x2e,
    0x0a, 0x29, 0x34, 0x40, 0x38, 0x30, 0x3a, 0x3d,
    0x33, 0x2b, 0x00, 0x3f, 0x27, 0x36, 0x25, 0x2f,
    0x2d, 0x32, 0x07, 0x80, 0x37, 0x31, 0x28, 0x80,
};

int translateread(unsigned char *ip, unsigned char *op, int n)
{
    unsigned char    *sop, c;
    int        i;

    for (sop = op, i = 0; (i < n); i++) {
        c = *ip++;
        if (c == 0x1f)
            icasemode = 0;
        else if (c == 0x1b)
            icasemode = 1;
        else
            c = (icasemode) ? upper2ascii[c] : lower2ascii[c];
        *op++ = c;
    }
    return(op - sop);
}

int translatewrite(unsigned char *ip, unsigned char *op, int n)
{
    unsigned char    *sop, c;
    int        i;

    for (sop = op, i = 0; (i < n); i++) {
        c = *ip++;
        c = ascii2code[c & 0x7f];
        if (ocasemode && ((c & 0x80) == 0)) {
            *op++ = 0x1f;
            ocasemode = 0;
        }
        if ((ocasemode == 0) && (c & 0x80)) {
            *op++ = 0x1b;
            ocasemode = 1;
        }
        *op++ = (c & 0x1f);
    }
    return(op - sop);
}

/*****************************************************************************/

/*****************************************************************************/

/*
 *    Do the connection session. Pass data between local and remote
 *    ports.
 */

int loopit(void)
{
    fd_set infds;
    int maxfd, n;

    unsigned int cur_odo_time;

#if 0 /* RAW ODOMETRY */
    int cur_odo_r;
    int cur_odo_l;
#endif

#if 1 /* NUCLEO ESTIMATED POSITION VECTOR */
    double cur_odo_x_mm;
    double cur_odo_y_mm;
    double cur_odo_theta_deg;
    short cur_odo_x_16;
    short cur_odo_y_16;
    int cur_odo_theta_32;
#endif

    unsigned int *wp;
    unsigned short *swp;
    unsigned char *cp;

    maxfd = ifd;
    if (maxfd < rfd)
        maxfd = rfd;
    maxfd++;

    read_odo_state = READ_ODO_STATE_INIT;

    for (;;) {
        FD_ZERO(&infds);
        FD_SET(ifd, &infds);
        FD_SET(rfd, &infds);

        if (select(maxfd, &infds, NULL, NULL, NULL) < 0) {
            fprintf(stderr, "ERROR: select() failed, errno=%d\n", errno);
            exit_thread(1);
        }

        if (FD_ISSET(rfd, &infds)) {
            if ((n = read(rfd, ibuf, 1)) < 0) {
                fprintf(stderr, "ERROR: read(fd=%d) failed, "
                        "errno=%d\n", rfd, errno);
                exit_thread(1);
            }

            //printf ("%.2x\n", ibuf[0]);
            //fprintf(stderr, "%.2x\n", ibuf[0]);
            //fputc(ibuf[0], stderr);

#if 0 /* RAW ODOMETRY */
            switch (read_odo_state) {
            case READ_ODO_STATE_INIT:
                if (ibuf[0]==0x0d)
                    read_odo_state = READ_ODO_STATE_HEAD0;
                else
                    read_odo_state = READ_ODO_STATE_INIT;
                break;
            case READ_ODO_STATE_HEAD0:
                if (ibuf[0]==0x00)
                    read_odo_state = READ_ODO_STATE_HEAD1;
                else
                    read_odo_state = READ_ODO_STATE_INIT;
                break;
            case READ_ODO_STATE_HEAD1:
                if (ibuf[0]==0x0d)
                    read_odo_state = READ_ODO_STATE_HEAD2;
                else
                    read_odo_state = READ_ODO_STATE_INIT;
                break;
            case READ_ODO_STATE_HEAD2:
                if (ibuf[0]==0x00)
                    read_odo_state = READ_ODO_STATE_HEAD3;
                else
                    read_odo_state = READ_ODO_STATE_INIT;
                break;
            case READ_ODO_STATE_HEAD3:
                wp = &cur_odo_time;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_TS0;
                cp[0] = ibuf[0];
                break;
            case READ_ODO_STATE_TS0:
                wp = &cur_odo_time;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_TS1;
                cp[1] = ibuf[0];
                break;
            case READ_ODO_STATE_TS1:
                wp = &cur_odo_time;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_TS2;
                cp[2] = ibuf[0];
                break;
            case READ_ODO_STATE_TS2:
                wp = &cur_odo_time;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_TS3;
                cp[3] = ibuf[0];
                break;
            case READ_ODO_STATE_TS3:
                wp = &cur_odo_r;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_ODOR0;
                cp[0] = ibuf[0];
                break;
            case READ_ODO_STATE_ODOR0:
                wp = &cur_odo_r;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_ODOR1;
                cp[1] = ibuf[0];
                break;
            case READ_ODO_STATE_ODOR1:
                wp = &cur_odo_r;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_ODOR2;
                cp[2] = ibuf[0];
                break;
            case READ_ODO_STATE_ODOR2:
                wp = &cur_odo_r;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_ODOR3;
                cp[3] = ibuf[0];
                break;
            case READ_ODO_STATE_ODOR3:
                wp = &cur_odo_l;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_ODOL0;
                cp[0] = ibuf[0];
                break;
            case READ_ODO_STATE_ODOL0:
                wp = &cur_odo_l;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_ODOL1;
                cp[1] = ibuf[0];
                break;
            case READ_ODO_STATE_ODOL1:
                wp = &cur_odo_l;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_ODOL2;
                cp[2] = ibuf[0];
                break;
            case READ_ODO_STATE_ODOL2:
                wp = &cur_odo_l;
                cp = (unsigned char *) wp;
                //read_odo_state = READ_ODO_STATE_ODOL3;
                read_odo_state = READ_ODO_STATE_INIT;
                cp[3] = ibuf[0];

                printf ("ts = %u ; odo_l = %d ; odo_r = %d\n",
                        cur_odo_time, cur_odo_l, cur_odo_r);
                {
                    struct timespec my_tp;
                    unsigned int my_time_ms;

                    clock_gettime(1, &my_tp);

                    my_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;

                    printf ("my_time_ms = %u\n\n", my_time_ms);
                }


                break;
            }
#endif

#if 1 /* NUCLEO ESTIMATED POSITION VECTOR */
            switch (read_odo_state) {
            case READ_ODO_STATE_INIT:
                if (ibuf[0]==0x0a)
                    read_odo_state = READ_ODO_STATE_HEAD0;
                else
                    read_odo_state = READ_ODO_STATE_INIT;
                break;
            case READ_ODO_STATE_HEAD0:
                if (ibuf[0]==0x35)
                    read_odo_state = READ_ODO_STATE_HEAD1;
                else
                    read_odo_state = READ_ODO_STATE_INIT;
                break;
            case READ_ODO_STATE_HEAD1:
                if (ibuf[0]==0x0a)
                    read_odo_state = READ_ODO_STATE_HEAD2;
                else
                    read_odo_state = READ_ODO_STATE_INIT;
                break;
            case READ_ODO_STATE_HEAD2:
                if (ibuf[0]==0x35)
                    read_odo_state = READ_ODO_STATE_HEAD3;
                else
                    read_odo_state = READ_ODO_STATE_INIT;
                break;
            case READ_ODO_STATE_HEAD3:
                wp = &cur_odo_time;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_TS0;
                cp[0] = ibuf[0];
                break;
            case READ_ODO_STATE_TS0:
                wp = &cur_odo_time;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_TS1;
                cp[1] = ibuf[0];
                break;
            case READ_ODO_STATE_TS1:
                wp = &cur_odo_time;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_TS2;
                cp[2] = ibuf[0];
                break;
            case READ_ODO_STATE_TS2:
                wp = &cur_odo_time;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_TS3;
                cp[3] = ibuf[0];
                break;
            case READ_ODO_STATE_TS3:
                swp = (unsigned short *) &cur_odo_x_16;
                cp = (unsigned char *) swp;
                read_odo_state = READ_ODO_STATE_X0;
                cp[0] = ibuf[0];
                break;
            case READ_ODO_STATE_X0:
                swp = (unsigned short *) &cur_odo_x_16;
                cp = (unsigned char *) swp;
                read_odo_state = READ_ODO_STATE_X1;
                cp[1] = ibuf[0];
                break;
            case READ_ODO_STATE_X1:
                swp = (unsigned short *) &cur_odo_y_16;
                cp = (unsigned char *) swp;
                read_odo_state = READ_ODO_STATE_Y0;
                cp[0] = ibuf[0];
                break;
            case READ_ODO_STATE_Y0:
                swp = (unsigned short *) &cur_odo_y_16;
                cp = (unsigned char *) swp;
                read_odo_state = READ_ODO_STATE_Y1;
                cp[1] = ibuf[0];
                break;
            case READ_ODO_STATE_Y1:
                wp = (unsigned int *) &cur_odo_theta_32;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_THETA0;
                cp[0] = ibuf[0];
                break;
            case READ_ODO_STATE_THETA0:
                wp = (unsigned int *) &cur_odo_theta_32;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_THETA1;
                cp[1] = ibuf[0];
                break;
            case READ_ODO_STATE_THETA1:
                wp = (unsigned int *) &cur_odo_theta_32;
                cp = (unsigned char *) wp;
                read_odo_state = READ_ODO_STATE_THETA2;
                cp[2] = ibuf[0];
                break;
            case READ_ODO_STATE_THETA2:
                wp = (unsigned int *) &cur_odo_theta_32;
                cp = (unsigned char *) wp;
                //read_odo_state = READ_ODO_STATE_THETA3;
                read_odo_state = READ_ODO_STATE_INIT;
                cp[3] = ibuf[0];

                cur_odo_x_mm = cur_odo_x_16;
                cur_odo_y_mm = cur_odo_y_16;
                cur_odo_theta_deg = cur_odo_theta_32/1000.0;

                g_odo_time_ms = cur_odo_time;
                g_odo_x_mm = cur_odo_x_mm;
                g_odo_y_mm = cur_odo_y_mm;
                g_odo_theta_deg = cur_odo_theta_deg;

#if 0 /* FIXME : DEBUG */
                printf ("ts = %u ; pos = < %f , %f > ; theta = %f\n",
                        cur_odo_time, 
                        cur_odo_x_mm, cur_odo_y_mm, 
                        cur_odo_theta_deg);
#endif
                if (dump_file) {
                    fprintf (dump_file, 
                             "ts = %u ; pos = < %f , %f > ; theta = %f\n",
                             cur_odo_time, 
                             cur_odo_x_mm, cur_odo_y_mm, 
                             cur_odo_theta_deg);
                }

                {
                    struct timespec my_tp;
                    unsigned int my_time_ms;

                    clock_gettime(1, &my_tp);

                    my_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;

                    g_odo_thread_time_ms = my_time_ms;
#if 0 /* FIXME : DEBUG */
                    printf ("my_time_ms = %u\n\n", my_time_ms);
#endif
                    if (dump_file) {
                        fprintf (dump_file, "my_time_ms = %u\n\n", my_time_ms);
                    }
                }


                break;
            }
#endif

        }

        if (FD_ISSET(ifd, &infds)) {
            char *bp = (char *) ibuf;
            if ((n = read(ifd, ibuf, sizeof(ibuf))) < 0) {
                fprintf(stderr, "ERROR: read(fd=%d) failed, "
                        "errno=%d\n", 1, errno);
                exit_thread(1);
            }

            if (n == 0)
                break;
            if ((n == 1) && (*bp == 0x1d))
                break;
            if ((n == 1) && (*bp == 0x1))
                break;
        }

    }

    return (0);
}

/*****************************************************************************/

int read_odo_main(void)
{
    struct stat statbuf;
    size_t len;
    char *path = NULL;

    read_odo_flag_running = 1;

    g_odo_thread_time_ms = 0;
    g_odo_time_ms = 0;
    g_odo_x_mm = 0;
    g_odo_y_mm = 0;
    g_odo_theta_deg = 0.0;

    dump_file = NULL;

    ifd = 0;
    ofd = 1;
    gotdevice = 0;

    baud = 115200;

    if (((devname = getenv("ODOMETRY_DEV"))!=NULL) && (devname[0]!='\0')) {
        gotdevice++;
        printf("INFO: using odometry device : %s\n", devname);
    }

    if (gotdevice == 0) {
        fprintf(stderr, "ERROR: no device specified\n");
        exit_thread (-1);
    }

    /*
     *    Check device is real, and open it.
     */
    /* If devname does not exist as is, prepend '/dev/' */
    if (devname[0] != '/' && stat(devname, &statbuf) == -1) {
        len = strlen(devname) + strlen("/dev/") + 1;
        path = calloc(len, sizeof(*path));
        strncpy(path, "/dev/", len);
        strncat(path, devname, len);
    } else {
        path = strdup(devname);
    }
    if (path == NULL) {
        fprintf(stderr, "ERROR: failed to alloc() path, "
                "errno=%d\n", errno);
        exit_thread(1);
    }
    if ((rfd = open(path, (O_RDWR | O_NDELAY))) < 0) {
        fprintf(stderr, "ERROR: failed to open() %s, "
                "errno=%d\n", path, errno);
    }
    if (path != NULL) {
        free(path);
    }
    if (rfd < 0) {
        exit_thread(1);
    }

#if 0 /* FIXME : DEBUG */
    if ((dump_file = fopen("dump_odom.txt", "w")) == NULL) {
        fprintf(stderr, "ERROR: failed to open odom capture file, errno=%d\n",
                errno);
    }
#endif

    savelocaltermios();
    setlocaltermios();
    printf("Connected.\n");

    saveremotetermios();
    setremotetermios();

#if 0 /* FIXME : TODO : usefull? */
    /*
     *    Set the signal handler to restore the old termios .
     */
    sact.sa_handler = sighandler;
    sigaction(SIGHUP, &sact, NULL);
    sigaction(SIGINT, &sact, NULL);
    sigaction(SIGQUIT, &sact, NULL);
    sigaction(SIGPIPE, &sact, NULL);
    sigaction(SIGTERM, &sact, NULL);
#endif

    loopit();

    printf("Disconnected.\n");
    restorelocaltermios();

	if (dump_file)
		fclose(dump_file);
    close(rfd);
    return 0;
}

/*****************************************************************************/
