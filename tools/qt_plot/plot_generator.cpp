#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <math.h>

float my_x[360];
float my_y[360];
char send_buf[2*4 + 4*4 + 4 + 720*2*4 + 4];

typedef struct _my_point_t {
  int x;
  int y;
} my_point_t;

int main(int argc, char *argv[])
{
  struct sockaddr_in viewer_saddr;
  unsigned int ab0, ab1, ab2, ab3;
  my_point_t *my_p;
  double rot_theta;

  char *ip_address_str = argv[1];

  if (sscanf (ip_address_str, "%d.%d.%d.%d", 
	      &ab3, &ab2, &ab1, &ab0) != 4) {
    printf(" error : cannot parse viewer addr (%s)\n", ip_address_str);
    return -1;
  }

  rot_theta = (2.0*M_PI/360.0)*atof(argv[2]);

  int my_sock = socket (AF_INET, SOCK_DGRAM, 0);

  viewer_saddr.sin_family= AF_INET;
  viewer_saddr.sin_port= htons(1413);
  viewer_saddr.sin_addr.s_addr= htonl((ab3<<24)|(ab2<<16)|(ab1<<8)|(ab0));

  my_p = (my_point_t *)((void *)(&send_buf[2*4 + 4*4 + 4]));

  double my_R = 0.2 + 90*0.016;;
  for (int i=0; i<360; i++) {
    double my_theta = (2.0 * M_PI / 360.0) * i;
    double my_R_scaled;
    if ((i>=0) && (i<90)) {
      my_R -= 0.016;
    } else if ((i>=90) && (i<180)) {
      my_R = 0.2;
    } else if ((i>=180) && (i<270)) {
      my_R = 0.2;
    } else /*if ((i>=270) && (i<360))*/ {
      my_R += 0.016;
    }
    my_R_scaled = my_R * 1000.0;
    my_x[i] = my_R_scaled * cos (my_theta);
    my_y[i] = my_R_scaled * sin (my_theta);
  }

  for (int i=0; i<360; i++) {
    double my_x_new = cos(rot_theta)*my_x[i] - sin(rot_theta)*my_y[i];
    double my_y_new = sin(rot_theta)*my_x[i] + cos(rot_theta)*my_y[i];
    my_p->x = my_x_new;
    my_p->y = my_y_new;
    my_p++;
  }

  int ret = sendto (my_sock, (void *)send_buf, sizeof (send_buf), 0, 
		    (const sockaddr *) &viewer_saddr, 
		    sizeof (struct sockaddr_in));
  if (ret!=sizeof(send_buf)) {
    printf ("sendto() failed\n");
    return -1;
  }

  return 0;
}
