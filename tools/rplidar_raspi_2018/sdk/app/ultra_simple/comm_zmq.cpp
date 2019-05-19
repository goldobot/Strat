#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <zmq.h>
#include <math.h>
#include <pthread.h>

#include "comm_serializer.hpp"


extern bool slave1_stop;


#if 1 /* FIXME : DEBUG */
float adv_x_m[3];
float adv_y_m[3];

float R_adv_m = 0.2;
double theta_adv_rad[3];

#endif



int comm_zmq_main(int port_nb)
{
    struct timespec curr_tp;
    int curr_time_ms = 0;
    int old_time_ms = 0;
    void* zmq_context;
    void* pub_socket;
    void* pull_socket;
    int rc;
    char char_buff[64];

#if 1 /* FIXME : DEBUG */
    theta_adv_rad[0] = 0.0;
    adv_x_m[0] = 1.0 + R_adv_m*cos(theta_adv_rad[0]);
    adv_y_m[0] =-1.2 + R_adv_m*sin(theta_adv_rad[0]);

    theta_adv_rad[1] = 0.0;
    adv_x_m[1] = 0.4 + R_adv_m*cos(theta_adv_rad[1]);
    adv_y_m[1] = 1.0 + R_adv_m*sin(theta_adv_rad[1]);

    theta_adv_rad[2] = 0.0;
    adv_x_m[2] = 1.0 + R_adv_m*cos(theta_adv_rad[2]);
    adv_y_m[2] = 1.0 + R_adv_m*sin(theta_adv_rad[2]);
#endif

    zmq_context = zmq_init(1);

    pub_socket = zmq_socket(zmq_context, ZMQ_PUB);
    sprintf(char_buff, "tcp://*:%d", port_nb);
    printf("DEBUG: char_buff = %s\n", char_buff);
    rc = zmq_bind(pub_socket, char_buff);

    pull_socket = zmq_socket(zmq_context, ZMQ_PULL);
    sprintf(char_buff, "tcp://*:%d", port_nb+1);
    printf("DEBUG: char_buff = %s\n", char_buff);
    rc = zmq_bind(pull_socket, char_buff);

    zmq_pollitem_t poll_items[1];

    poll_items[0].socket = pull_socket;
    poll_items[0].fd = 0;
    poll_items[0].events = ZMQ_POLLIN;

    //unsigned char recv_buffer[4096];
    //unsigned char serialize_buffer[4096];
    //unsigned char tmp_buffer[1024];
    //goldobot::CommDeserializer deserializer(recv_buffer, sizeof(recv_buffer));
    //goldobot::CommSerializer serializer(serialize_buffer, sizeof(serialize_buffer));

    while(!slave1_stop)
    {
        zmq_poll (poll_items, 1, -1);

        clock_gettime(1, &curr_tp);

        curr_time_ms = curr_tp.tv_sec*1000 + curr_tp.tv_nsec/1000000;

        if (curr_time_ms > (old_time_ms + 100)) {
            unsigned short my_message_type;
            short my_message[9];
#if 1 /* FIXME : DEBUG */
            theta_adv_rad[0] += M_PI/100.0;
            adv_x_m[0] = 1.0 + R_adv_m*cos(theta_adv_rad[0]);
            adv_y_m[0] =-1.2 + R_adv_m*sin(theta_adv_rad[0]);
            my_message[0] = 0;
            my_message[1] = 4*adv_x_m[0];
            my_message[2] = 4*adv_y_m[0];

            theta_adv_rad[1] += 2.0*M_PI/100.0;
            adv_x_m[1] = 0.4 + R_adv_m*cos(theta_adv_rad[1]);
            adv_y_m[1] = 1.0 + R_adv_m*sin(theta_adv_rad[1]);
            my_message[3] = 0;
            my_message[4] = 4*adv_x_m[1];
            my_message[5] = 4*adv_y_m[1];

            theta_adv_rad[2] += 4.0*M_PI/100.0;
            adv_x_m[2] = 1.0 + R_adv_m*cos(theta_adv_rad[2]);
            adv_y_m[2] = 1.0 + R_adv_m*sin(theta_adv_rad[2]);
            my_message[6] = 0;
            my_message[7] = 4*adv_x_m[1];
            my_message[8] = 4*adv_y_m[1];

            my_message_type = 400; /* FIXME : TODO : use mesage_types.hpp */
            zmq_send(pub_socket, (const char*)(&my_message_type), 2, ZMQ_SNDMORE);
            zmq_send(pub_socket, (const char*)(my_message), 18, 0);
#endif

            /* FIXME : TODO : send adversary coordinates */
            //zmq_send(pub_socket, (const char*)(&my_message_type), 2, ZMQ_SNDMORE);
            //zmq_send(pub_socket, (const char*)(tmp_buffer), my_message_size, 0);
            old_time_ms = curr_time_ms;
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
            printf("DEBUG: received message_type = %d\n", message_type);
            /* FIXME : TODO : process commands */
            //serializer.push_message(message_type, buff+2, bytes_read - 2);
        }

        pthread_yield();
    }
    zmq_term(zmq_context);

    return 0;
}
