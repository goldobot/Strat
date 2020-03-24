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
unsigned int    baud = 115200;

int        translate;
int        ocasemode, icasemode;

char        devname[] = "/dev/odometry";
char        *filename;
int        verbose = 1;
int        net_connection = 0;
int        gotdevice;
int        ifd;
int        rfd;

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


typedef enum _read_odo_state {
    READ_ODO_STATE_INIT,

    READ_ODO_STATE_HEAD0,
    READ_ODO_STATE_HEAD1,
    READ_ODO_STATE_HEAD2_ODO,
    READ_ODO_STATE_HEAD3_ODO,
    READ_ODO_STATE_HEAD2_DBG,
    READ_ODO_STATE_HEAD3_DBG,

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

read_odo_state_t read_odo_state;

volatile int debug_num_points;
volatile short debug_traj_x_mm[16];
volatile short debug_traj_y_mm[16];


volatile int read_odo_flag_running = 0;


/*****************************************************************************/

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

void restoreremotetermios(void)
{
	/*
	 *	This can fail if remote hung up, don't check return status.
	 */
	tcsetattr(rfd, TCSAFLUSH, &savetio_remote);
}

int saveremotetermios(void)
{
	if (tcgetattr(rfd, &savetio_remote) < 0) {
		fprintf(stderr, "ERROR: remote tcgetattr() failed, errno=%d\n",
			errno);
		return(0);
	}
	return(1);
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

/*
 *    Do the connection session. Pass data between local and remote
 *    ports.
 */
#define PAYLOAD_BUF_SZ 4096
unsigned char payload_buf[PAYLOAD_BUF_SZ];

#define READ_SHORT_DBG(_lvalue, _offset) do { \
    cp = &payload_buf[_offset];               \
    swp = (unsigned short *) cp;              \
    _lvalue = *swp;                           \
} while (0)

#define SEND_BUFF_SZ 256
unsigned char send_buff[SEND_BUFF_SZ];

int loopit(void)
{
    fd_set infds;
    int maxfd, n;

    int payload_byte_cnt;

    unsigned int cur_odo_time;

    double cur_odo_x_mm;
    double cur_odo_y_mm;
    double cur_odo_theta_deg;
    short cur_odo_x_16;
    short cur_odo_y_16;
    int cur_odo_theta_32;

    unsigned int *wp;
    unsigned short *swp;
    unsigned char *cp;

    maxfd = ifd;
    if (maxfd < rfd)
        maxfd = rfd;
    maxfd++;

    read_odo_state = READ_ODO_STATE_INIT;

    payload_byte_cnt = 0;

    debug_num_points = 0;

    for (;;) {
        FD_ZERO(&infds);
        FD_SET(rfd, &infds);
        FD_SET(ifd, &infds);

        if (select(maxfd, &infds, NULL, NULL, NULL) < 0) {
            fprintf(stderr, "ERROR: select() failed, errno=%d\n", errno);
            exit(1);
        }

        if (FD_ISSET(rfd, &infds)) {
            if ((n = read(rfd, ibuf, 1)) < 0) {
                fprintf(stderr, "ERROR: read(fd=%d) failed, "
                        "errno=%d\n", rfd, errno);
                exit(1);
            }

            //printf ("%.2x\n", ibuf[0]);
            //fprintf(stderr, "%.2x\n", ibuf[0]);
            //fputc(ibuf[0], stderr);

            switch (read_odo_state) {
            case READ_ODO_STATE_INIT:
                payload_byte_cnt = 0;
                memset(payload_buf, 0, PAYLOAD_BUF_SZ);
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
                    read_odo_state = READ_ODO_STATE_HEAD2_ODO;
                else if (ibuf[0]==0x00)
                    read_odo_state = READ_ODO_STATE_HEAD2_DBG;
                else
                    read_odo_state = READ_ODO_STATE_INIT;
                break;

            case READ_ODO_STATE_HEAD2_ODO:
                if (ibuf[0]==0x35)
                    read_odo_state = READ_ODO_STATE_HEAD3_ODO;
                else
                    read_odo_state = READ_ODO_STATE_INIT;
                break;
            case READ_ODO_STATE_HEAD3_ODO:
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

#if 0 /* FIXME : DEBUG */
                printf ("ts = %u ; pos = < %f , %f > ; theta = %f\n",
                        cur_odo_time, 
                        cur_odo_x_mm, cur_odo_y_mm, 
                        cur_odo_theta_deg);
                {
                    struct timespec my_tp;
                    unsigned int my_time_ms;

                    clock_gettime(1, &my_tp);

                    my_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;

                    printf ("my_time_ms = %u\n\n", my_time_ms);
                }
#endif


                break;


            case READ_ODO_STATE_HEAD2_DBG:
                if ((ibuf[0]>0x00) && (ibuf[0]<=0x10))
                {
                    read_odo_state = READ_ODO_STATE_HEAD3_DBG;
                    debug_num_points = ibuf[0];
                }
                else
                {
                    read_odo_state = READ_ODO_STATE_INIT;
                }
                break;
            case READ_ODO_STATE_HEAD3_DBG:
                payload_buf[payload_byte_cnt] = ibuf[0];
                payload_byte_cnt++;
                if (payload_byte_cnt<(debug_num_points*4)) {
                    read_odo_state = READ_ODO_STATE_HEAD3_DBG;
                } else {
                    int i;

                    for (i=0; i<debug_num_points; i++) {
                        READ_SHORT_DBG(debug_traj_x_mm[i], (4*i));
                        READ_SHORT_DBG(debug_traj_y_mm[i], (4*i+2));
                    }

                    printf (" debug_traj (%d) : \n", debug_num_points);
                    for (i=0; i<debug_num_points; i++) {
                        printf ("  (%d,%d)\n", 
                                debug_traj_x_mm[i], debug_traj_y_mm[i]);
                    }

                    read_odo_state = READ_ODO_STATE_INIT;
                    payload_byte_cnt = 0;
                }
                break;

            }

        }

        if (FD_ISSET(ifd, &infds)) {
            char *bp = ibuf;
            char *tok = NULL;
            char *sb = &send_buff[0];
            int val;

            memset(ibuf, 0, SEND_BUFF_SZ);
            if ((n = read(ifd, ibuf, sizeof(ibuf))) < 0) {
                fprintf(stderr, "ERROR: read(fd=%d) failed, "
                        "errno=%d\n", 1, errno);
                exit(1);
            }

            printf ("USER : %s\n", bp);

            if (n == 0)
                break;
            if ((n == 1) && (*bp == 0x1d))
                break;
            if ((n == 1) && (*bp == 0x1))
                break;

            memset(send_buff, 0, SEND_BUFF_SZ);

            tok = strtok (bp, " ");
            while (tok!=NULL) {
                val = strtol(tok, NULL, 16);
                //printf (" %.2x", val);
                *sb = (unsigned char) val;
                sb++;
                tok = strtok (NULL, " ");
            }

            if ((n = write(rfd, send_buff, SEND_BUFF_SZ)) < 0) {
                fprintf(stderr, "ERROR: write(fd=%d) failed, "
                        "errno=%d\n", rfd, errno);
                exit(1);
            }

        }

    }

    return (0);
}

/*****************************************************************************/

int main(int argc, char *argv[])
{
    struct stat statbuf;
    size_t len;
    char *path = NULL;

    read_odo_flag_running = 1;

    ifd = 0;

    baud = 115200;

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
        exit(1);
    }
    if ((rfd = open(path, (O_RDWR | O_NDELAY))) < 0) {
        fprintf(stderr, "ERROR: failed to open() %s, "
                "errno=%d\n", path, errno);
    }
    if (path != NULL) {
        free(path);
    }
    if (rfd < 0) {
        exit(1);
    }


    saveremotetermios();
    setremotetermios();
    printf("Connected.\n");

    loopit();

    printf("Disconnected.\n");
    restoreremotetermios();

    close(rfd);
    exit(0);
}

/*****************************************************************************/
