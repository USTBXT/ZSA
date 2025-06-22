#include <stdio.h>
#include <stddef.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <termios.h>
#include <math.h>
#include <fcntl.h>
#include <signal.h>
#include <khepera/khepera.h>
#include <sys/select.h>
#include <termios.h>

#define SERVER_PORT     65080
#define SERVER_IP       "192.168.1.101"


#define CONCENTRATION_THRESHOLD 1     
#define SPIRAL_R_START 0.0            
#define SPIRAL_K 10                  
#define SPIRAL_TURNS 6                
#define FIXED_ARC_LENGTH 20.0         
#define ZIGZAG_LARGE_STEP 25.0        
#define ZIGZAG_SMALL_STEP 15.0       
#define SOURCE_DISTANCE_THRESHOLD 20.0 
#define MAX_X 500.0                   
#define MAX_Y 500.0                   
#define SPIRAL_TYPE 0                 


typedef enum {
    STATE_SPIRAL_OUT,      
    STATE_ZIGZAG,         
    STATE_ZIGZAG_BACK,    
    STATE_RECIPROCATING_SPIRAL, 
    STATE_INNER_SPIRAL,   
    STATE_COMPLETED        
} AlgorithmState;


typedef struct {
    AlgorithmState state;
    

    double spiral_x0;
    double spiral_y0;
    double spiral_r_start;
    double spiral_start_angle;
    int spiral_point_index;
    double* spiral_xs;
    double* spiral_ys;
    int spiral_num_points;
    

    double zigzag_x0;
    double zigzag_y0;
    double* zigzag_angles;   
    double* zigzag_distances; 
    int zigzag_direction;    
    int zigzag_segment_count;
    int zigzag_sign;
    
  
    double recip_spiral_x0;
    double recip_spiral_y0;
    double* recip_spiral_xs;
    double* recip_spiral_ys;
    int recip_spiral_index;
    int recip_spiral_count;
    
  
    double inner_spiral_x0;
    double inner_spiral_y0;
    double* inner_spiral_xs;
    double* inner_spiral_ys;
    int inner_spiral_index;
    int inner_spiral_count;
    

    int sign;
    int sign_spiral;
    int count_segment;
    int sign_zigzag_break;
    int sign_rz_end;
    int* index_segment;
    double sum_dis;
    double* con;
    int con_count;
    

    double start_x;
    double start_y;
    double start_angle;
    

    double* trajectory_x;
    double* trajectory_y;
    int trajectory_count;
    int trajectory_capacity;
} AlgorithmContext;

int connect_fd = -1;
static knet_dev_t *dsPic;
static int quitReq = 0;

typedef struct {
    double x;
    double y;
} Point;

static void ctrlc_handler(int sig) {
    quitReq = 1;
    kh4_set_speed(0, 0, dsPic);
    kh4_SetMode(kh4RegIdle, dsPic);
    kh4_SetRGBLeds(0, 0, 0, 0, 0, 0, 0, 0, 0, dsPic);
    kb_change_term_mode(0);
    if (connect_fd >= 0) close(connect_fd);
    exit(0);
}

void set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

void set_blocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
}


double clip_value(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return value;
    return value;
}


void record_trajectory(AlgorithmContext* ctx, double x, double y) {
    if (ctx->trajectory_count >= ctx->trajectory_capacity) {
     
        ctx->trajectory_capacity *= 2;
        ctx->trajectory_x = realloc(ctx->trajectory_x, ctx->trajectory_capacity * sizeof(double));
        ctx->trajectory_y = realloc(ctx->trajectory_y, ctx->trajectory_capacity * sizeof(double));
    }
    ctx->trajectory_x[ctx->trajectory_count] = x;
    ctx->trajectory_y[ctx->trajectory_count] = y;
    ctx->trajectory_count++;
}


void save_trajectory(AlgorithmContext* ctx, const char* filename) {
    FILE* file = fopen(filename, "w");
    if (file) {
        fprintf(file, "x,y\n");
        for (int i = 0; i < ctx->trajectory_count; i++) {
            fprintf(file, "%.2f,%.2f\n", ctx->trajectory_x[i], ctx->trajectory_y[i]);
        }
        fclose(file);
        printf("Trajectory saved to %s\n", filename);
    } else {
        perror("Error saving trajectory");
    }
}

void move_to_target(double target_x, double target_y, double *current_x, double *current_y, double *theta_copy, AlgorithmContext* ctx) {
    int lpos, rpos;
    long motspeed;

    double delta_x = target_x - *current_x;
    double delta_y = target_y - *current_y;
    double delta_R = sqrt(delta_x * delta_x + delta_y * delta_y);
    double theta = atan2(delta_y, delta_x) * 180 / M_PI - *theta_copy;
    *theta_copy = atan2(delta_y, delta_x) * 180 / M_PI;

    double theta1 = fmod(theta + 360.0, 360.0);
    double distance = delta_R; 

    if (theta1 >= 180.0) {
        theta1 = 360.0 - theta1;
        double angle_time = theta1 / 360.0;
        kh4_set_speed(0, 0, dsPic);
        kh4_get_position(&lpos, &rpos, dsPic);
        motspeed = (double)(KH4_WHEELS_DISTANCE * M_PI / KH4_SPEED_TO_MM_S);
        kh4_SetMode(kh4RegSpeed, dsPic);
        kh4_set_speed(motspeed * angle_time, -motspeed * angle_time, dsPic);
        sleep(1);
    } else {
        double angle_time = theta1 / 360.0;
        kh4_set_speed(0, 0, dsPic);
        kh4_get_position(&lpos, &rpos, dsPic);
        motspeed = (double)(KH4_WHEELS_DISTANCE * M_PI / KH4_SPEED_TO_MM_S);
        kh4_SetMode(kh4RegSpeed, dsPic);
        kh4_set_speed(-motspeed * angle_time, motspeed * angle_time, dsPic);
        sleep(1);
    }



    kh4_set_speed(0, 0, dsPic);
    usleep(200000);
    kh4_ResetEncoders(dsPic);

    motspeed = (double)(100.0 / KH4_SPEED_TO_MM_S);
    kh4_SetMode(kh4RegSpeedProfile, dsPic);
    kh4_set_speed(motspeed, motspeed, dsPic);


    double time_sec = distance / 100.0; 
    usleep(time_sec * 1000000);

    kh4_set_speed(0, 0, dsPic);
    sleep(1);
    kh4_get_position(&lpos, &rpos, dsPic);
    printf("\nencoders after straight: left %d | right %d\n", lpos, rpos);


    *current_x = target_x;
    *current_y = target_y;


    record_trajectory(ctx, *current_x, *current_y);

    kh4_ResetEncoders(dsPic);
}


void rotate_to_angle(double target_angle, double *current_angle) {
    double diff = target_angle - *current_angle;
 
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;

    if (fabs(diff) < 1.0) {
        return;
    }

    double angle_time = fabs(diff) / 180.0; 
    long motspeed = (double)(KH4_WHEELS_DISTANCE * M_PI / KH4_SPEED_TO_MM_S) * 0.5; 

    kh4_SetMode(kh4RegSpeed, dsPic);
    if (diff > 0) {
        kh4_set_speed(-motspeed * angle_time, motspeed * angle_time, dsPic);
    } else {
        kh4_set_speed(motspeed * angle_time, -motspeed * angle_time, dsPic);
    }
    sleep(1);
    kh4_set_speed(0, 0, dsPic);

    *current_angle = target_angle;
}

int read_sensor_data(int fd) {
    char buffer[256];
    int data = -1;
    int start = 0;
    int n = read(fd, buffer, sizeof(buffer));
    if (n > 0) {
        for (start = 0; start + 4 < n; start++) {
            if (buffer[start] == (char)255 && 
                buffer[start + 1] == (char)170 && 
                buffer[start + 4] == (char)60) {
                data = (unsigned char)buffer[start + 2] * 255 + (unsigned char)buffer[start + 3];
                break;
            }
        }
    }
    return data;
}


void generate_spiral_points(double x0, double y0, double r_start, double start_angle, 
                           double s_step, int turns, double** xs, double** ys, int* num_points) {
    double k = SPIRAL_K;
    double theta = start_angle;
    double total_angle = start_angle + 2 * M_PI * turns;
    int count = 0;
    int max_points = 1000; 
    *xs = (double*)malloc(max_points * sizeof(double));
    *ys = (double*)malloc(max_points * sizeof(double));
    

    double r = r_start + k * (theta - start_angle);
    double x = r * cos(theta) + x0;
    double y = r * sin(theta) + y0;
    

    x = clip_value(x, 0, MAX_X);
    y = clip_value(y, 0, MAX_Y);
    
    (*xs)[count] = x;
    (*ys)[count] = y;
    count++;
    
    while (theta < total_angle && count < max_points-1) {
        r = r_start + k * (theta - start_angle);
        double ds_dtheta = sqrt(k*k + r*r);
        double dtheta = s_step / ds_dtheta;
        
        
        if (theta + dtheta > total_angle) {
            dtheta = total_angle - theta;
        }
        theta += dtheta;
        
  
        double r_new = r_start + k * (theta - start_angle);
        double x_new = r_new * cos(theta) + x0;
        double y_new = r_new * sin(theta) + y0;
        

        x_new = clip_value(x_new, 0, MAX_X);
        y_new = clip_value(y_new, 0, MAX_Y);
        
        (*xs)[count] = x_new;
        (*ys)[count] = y_new;
        count++;
    }
    
    *num_points = count;
}


void create_spiral(double x0, double y0, double x1, double y1, double sign1, int sign,
                   double k, double** xs, double** ys, int* num_points, 
                   double r_start, int num_points_spiral) {
    double a;

    if (sign1 >= 250) {
        if (sign % 2 == 0) {
            a = 1;
        } else {
            a = -1;
        }
    }
  
    else {
        if (sign % 2 == 0) {
            a = -1;
        } else {
            a = 1;
        }
    }
    
  
    double dx = x1 - x0;
    double dy = y1 - y0;
    double R_end = sqrt(dx*dx + dy*dy);
    double theta_end = atan2(dy, dx);
    

    double delta_theta = (R_end - r_start) / k;
    double theta_start = theta_end + (a * delta_theta);
    

    *xs = (double*)malloc(num_points_spiral * sizeof(double));
    *ys = (double*)malloc(num_points_spiral * sizeof(double));
    *num_points = num_points_spiral;
    
    for (int j = 0; j < num_points_spiral; j++) {
        double t = (double)j / (num_points_spiral - 1);
        double theta = theta_start + t * (theta_end - theta_start);
        double r = r_start + k * (theta_start - theta) * a;
        
        double x_val = r * cos(theta) + x0;
        double y_val = r * sin(theta) + y0;
        
      
        x_val = clip_value(x_val, 0, MAX_X);
        y_val = clip_value(y_val, 0, MAX_Y);
        
        (*xs)[j] = x_val;
        (*ys)[j] = y_val;
    }
}


Point* get_key_points(Point start, Point end, int n, double sign, int* num_points) {
 
    Point mid;
    mid.x = (start.x + end.x) / 2.0;
    mid.y = (start.y + end.y) / 2.0;
    

    Point top, bottom;
    if (start.y > end.y) {
        top = start;
        bottom = end;
    } else {
        top = end;
        bottom = start;
    }
    
    Point extreme;
    if (sign > 250) {
        extreme = top;
    } else {
        extreme = bottom;
    }
    

    int max_points = n + 2; 
    Point* points = (Point*)malloc(max_points * sizeof(Point));
    *num_points = 0;
    

    points[*num_points] = mid;
    (*num_points)++;
    

    points[*num_points] = extreme;
    (*num_points)++;
    
 
    for (int i = 1; i < n; i++) {
        double ratio = (double)i / (double)n;
        Point p;
        p.x = (1 - ratio) * mid.x + ratio * extreme.x;
        p.y = (1 - ratio) * mid.y + ratio * extreme.y;
        points[*num_points] = p;
        (*num_points)++;
    }
    
    return points;
}


void generate_inner_spiral_points(double x0, double y0, double x_target, double y_target, 
                                double** xs, double** ys, int* count) {
    *count = 0;
    *xs = (double*)malloc(10000 * sizeof(double));
    *ys = (double*)malloc(10000 * sizeof(double));
    
  
    double dx = x_target - x0;
    double dy = y_target - y0;
    double line1 = sqrt(dx*dx + dy*dy);
    

    double theta_rot = atan2(dy, dx);
    
 
    double theta_max = line1 / 1.5; 
    

    int total_turns = (int)(theta_max / (2 * M_PI));
    if (total_turns < 1) total_turns = 1;

    int points_per_turn = 8;
    int total_points = total_turns * points_per_turn;
    
    for (int k = 0; k < total_turns; k++) {
        double start_theta = k * 2 * M_PI;
        double end_theta = (k + 1) * 2 * M_PI;
        
        if (start_theta > theta_max) break;
        
  
        for (int j = 0; j < points_per_turn; j++) {
            double theta = start_theta + j * (2 * M_PI) / points_per_turn;
            
         
            double radius = line1 - theta * 2;
            if (radius < 0) radius = 0;
            
      
            double x_val = x0 + radius * cos(theta_rot + theta) - (line1 ) * cos(theta_rot);
            double y_val = y0 + radius * sin(theta_rot + theta) - (line1 ) * sin(theta_rot);
            
   
            x_val = clip_value(x_val, 0, MAX_X);
            y_val = clip_value(y_val, 0, MAX_Y);
            
            (*xs)[*count] = x_val;
            (*ys)[*count] = y_val;
            (*count)++;
        }
    }
}


void init_algorithm_context(AlgorithmContext* ctx, double start_x, double start_y, double start_angle) {
    memset(ctx, 0, sizeof(AlgorithmContext));
    
    
    ctx->state = STATE_SPIRAL_OUT;
    

    ctx->start_x = start_x;
    ctx->start_y = start_y;
    ctx->start_angle = start_angle;
    
 
    ctx->spiral_x0 = start_x;
    ctx->spiral_y0 = start_y;
    ctx->spiral_r_start = SPIRAL_R_START;
    ctx->spiral_start_angle = 0;
    ctx->spiral_point_index = 0;
    

    generate_spiral_points(ctx->spiral_x0, ctx->spiral_y0, ctx->spiral_r_start, 
                          ctx->spiral_start_angle, FIXED_ARC_LENGTH, SPIRAL_TURNS,
                          &ctx->spiral_xs, &ctx->spiral_ys, &ctx->spiral_num_points);
    

    ctx->zigzag_angles = (double*)malloc(100 * sizeof(double));
    ctx->zigzag_distances = (double*)malloc(100 * sizeof(double));
    ctx->zigzag_segment_count = 0;
    ctx->zigzag_direction = -1; 
    ctx->zigzag_sign = 0;

    ctx->recip_spiral_xs = NULL;
    ctx->recip_spiral_ys = NULL;
    ctx->recip_spiral_index = 0;
    ctx->recip_spiral_count = 0;
    

    ctx->inner_spiral_xs = NULL;
    ctx->inner_spiral_ys = NULL;
    ctx->inner_spiral_index = 0;
    ctx->inner_spiral_count = 0;
    

    ctx->sign = 0;
    ctx->sign_spiral = 1;
    ctx->count_segment = 0;
    ctx->sign_zigzag_break = 0;
    ctx->sign_rz_end = 0;
    ctx->index_segment = (int*)malloc(10 * sizeof(int));
    ctx->index_segment[0] = 0;
    ctx->sum_dis = 0.0;
    ctx->con = (double*)malloc(100 * sizeof(double));
    ctx->con_count = 0;
    

    ctx->trajectory_capacity = 1000;
    ctx->trajectory_count = 0;
    ctx->trajectory_x = (double*)malloc(ctx->trajectory_capacity * sizeof(double));
    ctx->trajectory_y = (double*)malloc(ctx->trajectory_capacity * sizeof(double));
    

    record_trajectory(ctx, start_x, start_y);
}


void free_algorithm_context(AlgorithmContext* ctx) {
    if (ctx->spiral_xs) free(ctx->spiral_xs);
    if (ctx->spiral_ys) free(ctx->spiral_ys);
    if (ctx->zigzag_angles) free(ctx->zigzag_angles);
    if (ctx->zigzag_distances) free(ctx->zigzag_distances);
    if (ctx->recip_spiral_xs) free(ctx->recip_spiral_xs);
    if (ctx->recip_spiral_ys) free(ctx->recip_spiral_ys);
    if (ctx->inner_spiral_xs) free(ctx->inner_spiral_xs);
    if (ctx->inner_spiral_ys) free(ctx->inner_spiral_ys);
    if (ctx->index_segment) free(ctx->index_segment);
    if (ctx->con) free(ctx->con);
    if (ctx->trajectory_x) free(ctx->trajectory_x);
    if (ctx->trajectory_y) free(ctx->trajectory_y);
}


double distance_between_points(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx*dx + dy*dy);
}


double distance_to_source(double x, double y) {
    return distance_between_points(x, y, 200, 200); 
}


void reset_to_start(AlgorithmContext* ctx, double *current_x, double *current_y, double *theta_copy) {
    printf("Resetting to start position...\n");
    
   
    kh4_set_speed(0, 0, dsPic);
    
  
    move_to_target(ctx->start_x, ctx->start_y, current_x, current_y, theta_copy, ctx);
    *current_x = ctx->start_x;
    *current_y = ctx->start_y;
    
   
    rotate_to_angle(ctx->start_angle, theta_copy);
    *theta_copy = ctx->start_angle;
    
    printf("Reset complete. Position: (%.1f, %.1f), Angle: %.1f\n", 
           *current_x, *current_y, *theta_copy);
}

int init_serial_port(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        perror("Error getting serial attributes");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CREAD | CLOCAL);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error setting serial attributes");
        close(fd);
        return -1;
    }

    return fd;
}

Point decide_next_target(AlgorithmContext* ctx, int concentration, double current_x, double current_y) {
    Point target;
    target.x = current_x;
    target.y = current_y;
    
    switch (ctx->state) {
        case STATE_SPIRAL_OUT:
        
            if (concentration > CONCENTRATION_THRESHOLD) {
               
                ctx->state = STATE_ZIGZAG;
                ctx->zigzag_x0 = current_x;
                ctx->zigzag_y0 = current_y;
                printf("Entered plume, switching to ZIGZAG mode\n");
                
                
                return decide_next_target(ctx, concentration, current_x, current_y);
            } else {
           
                if (ctx->spiral_point_index < ctx->spiral_num_points) {
                    target.x = ctx->spiral_xs[ctx->spiral_point_index];
                    target.y = ctx->spiral_ys[ctx->spiral_point_index];
                    ctx->spiral_point_index++;
                } else {
                 
                    ctx->state = STATE_ZIGZAG;
                    ctx->zigzag_x0 = current_x;
                    ctx->zigzag_y0 = current_y;
                    printf("Spiral completed, switching to ZIGZAG mode\n");
                }
            }
            break;
            
        case STATE_ZIGZAG:
        
            if (ctx->sign == 0) {
               
                ctx->zigzag_x0 = current_x;
                ctx->zigzag_y0 = current_y;
                
             
                if (current_y >= 250) {
                    ctx->zigzag_direction = 0; 
                    ctx->zigzag_angles[0] = 245.0 * M_PI / 180.0; 
                } else {
                    ctx->zigzag_direction = 1; 
                    ctx->zigzag_angles[0] = 115.0 * M_PI / 180.0;
                }
                ctx->zigzag_distances[0] = ZIGZAG_LARGE_STEP;
                ctx->zigzag_segment_count = 1;
                ctx->sign++;
            }
            
      
            if (ctx->con_count < 150) {
                ctx->con[ctx->con_count++] = concentration;
            }
            
            if (concentration > CONCENTRATION_THRESHOLD) {
           
                ctx->zigzag_distances[ctx->zigzag_segment_count] = ZIGZAG_LARGE_STEP;
                ctx->zigzag_angles[ctx->zigzag_segment_count] = 0.0;
                ctx->zigzag_segment_count++;
            } else if (ctx->sum_dis <= 45.0) {
              
                ctx->zigzag_distances[ctx->zigzag_segment_count] = ZIGZAG_SMALL_STEP;
                ctx->zigzag_angles[ctx->zigzag_segment_count] = 0.0;
                ctx->zigzag_segment_count++;
                ctx->sum_dis += ZIGZAG_SMALL_STEP;
            } else {
      
                ctx->zigzag_distances[ctx->zigzag_segment_count] = 30.0; 
                if (ctx->zigzag_direction == 0) {
                    ctx->zigzag_angles[ctx->zigzag_segment_count] = 225.0 * M_PI / 180.0;
                    ctx->zigzag_direction = 1; 
                } else {
                    ctx->zigzag_angles[ctx->zigzag_segment_count] = 135.0 * M_PI / 180.0;
                    ctx->zigzag_direction = 0; 
                }
                ctx->zigzag_segment_count++;
                ctx->sum_dis = 0.0;
                ctx->count_segment++;
                ctx->index_segment[ctx->count_segment] = ctx->sign;
                
              
                if (ctx->count_segment >= 2) {
                    int start_idx = ctx->index_segment[ctx->count_segment - 1];
                    int end_idx = ctx->index_segment[ctx->count_segment];
                    int max_con = 0;
                    for (int i = start_idx; i < end_idx; i++) {
                        if (ctx->con[i] > max_con) max_con = ctx->con[i];
                    }
                    if (max_con <= CONCENTRATION_THRESHOLD) {
                        ctx->state = STATE_ZIGZAG_BACK;
                        ctx->sign_rz_end = ctx->sign;
                        printf("Switching to BACK ZIGZAG mode\n");
                    }
                }
            }
            
  
            double total_angle = 0.0;
            for (int i = 0; i < ctx->zigzag_segment_count; i++) {
                total_angle += ctx->zigzag_angles[i];
            }
            
            double total_distance = 0.0;
            for (int i = 0; i < ctx->zigzag_segment_count; i++) {
                total_distance += ctx->zigzag_distances[i];
            }
            
            target.x = ctx->zigzag_x0 + total_distance * cos(total_angle);
            target.y = ctx->zigzag_y0 + total_distance * sin(total_angle);
            ctx->sign++;
            break;
            
        case STATE_ZIGZAG_BACK:
        
            if (ctx->sign == ctx->sign_rz_end + 1) {
             
                if (ctx->zigzag_direction == 0) {
                    ctx->zigzag_angles[ctx->zigzag_segment_count] = 195.0 * M_PI / 180.0;
                } else {
                    ctx->zigzag_angles[ctx->zigzag_segment_count] = 165.0 * M_PI / 180.0;
                }
                ctx->zigzag_distances[ctx->zigzag_segment_count] = 15.0;
                ctx->zigzag_segment_count++;
            }
            
            if (concentration > CONCENTRATION_THRESHOLD) {
               
                ctx->zigzag_distances[ctx->zigzag_segment_count] = 15.0;
                ctx->zigzag_angles[ctx->zigzag_segment_count] = 0.0;
                ctx->zigzag_segment_count++;
                ctx->sign_zigzag_break++;
            } else if (ctx->sum_dis <= 30.0) {
              
                ctx->zigzag_distances[ctx->zigzag_segment_count] = 10.0;
                ctx->zigzag_angles[ctx->zigzag_segment_count] = 0.0;
                ctx->zigzag_segment_count++;
                ctx->sum_dis += 10.0;
            } else {
          
                ctx->zigzag_distances[ctx->zigzag_segment_count] = 30.0; 
                if (ctx->zigzag_direction == 0) {
                    ctx->zigzag_angles[ctx->zigzag_segment_count] = 190.0 * M_PI / 180.0;
                    ctx->zigzag_direction = 1;
                } else {
                    ctx->zigzag_angles[ctx->zigzag_segment_count] = 170.0 * M_PI / 180.0;
                    ctx->zigzag_direction = 0; 
                }
                ctx->zigzag_segment_count++;
                ctx->sum_dis = 0.0;
                ctx->count_segment++;
                ctx->index_segment[ctx->count_segment] = ctx->sign;
            }
            
         
            double total_angle = 0.0;
            for (int i = 0; i < ctx->zigzag_segment_count; i++) {
                total_angle += ctx->zigzag_angles[i];
            }
            
            double total_distance = 0.0;
            for (int i = 0; i < ctx->zigzag_segment_count; i++) {
                total_distance += ctx->zigzag_distances[i];
            }
            
            target.x = ctx->zigzag_x0 + total_distance * cos(total_angle);
            target.y = ctx->zigzag_y0 + total_distance * sin(total_angle);
            ctx->sign++;
            
      
            if (ctx->sign_zigzag_break >= 1) {
                int start_idx = ctx->index_segment[ctx->count_segment - 1];
                int end_idx = ctx->index_segment[ctx->count_segment];
                
         
                double x1 = ctx->zigzag_x0;
                double y1 = ctx->zigzag_y0;
                for (int i = 0; i < start_idx; i++) {
                    double angle = ctx->zigzag_angles[i];
                    double dist = ctx->zigzag_distances[i];
                    x1 += dist * cos(angle);
                    y1 += dist * sin(angle);
                }
                
                double x2 = ctx->zigzag_x0;
                double y2 = ctx->zigzag_y0;
                for (int i = 0; i < end_idx; i++) {
                    double angle = ctx->zigzag_angles[i];
                    double dist = ctx->zigzag_distances[i];
                    x2 += dist * cos(angle);
                    y2 += dist * sin(angle);
                }
                
                double mid_x = (x1 + x2) / 2.0;
                double mid_y = (y1 + y2) / 2.0;
                
                if (SPIRAL_TYPE == 0) {
                    ctx->state = STATE_INNER_SPIRAL;
                    printf("Switching to INNER SPIRAL mode\n");
               
                    generate_inner_spiral_points(current_x, current_y, mid_x, mid_y,
                                                &ctx->inner_spiral_xs, &ctx->inner_spiral_ys,
                                                &ctx->inner_spiral_count);
                    ctx->inner_spiral_index = 0;
                } else {
                    ctx->state = STATE_RECIPROCATING_SPIRAL;
                    printf("Switching to RECIPROCATING SPIRAL mode\n");
                    
                
                    ctx->recip_spiral_count = 10000;
                    ctx->recip_spiral_xs = (double*)malloc(ctx->recip_spiral_count * sizeof(double));
                    ctx->recip_spiral_ys = (double*)malloc(ctx->recip_spiral_count * sizeof(double));
                    ctx->recip_spiral_index = 0;
                    ctx->recip_spiral_count = 0;
                    
             
                    Point start_point, end_point;
                    start_point.x = current_x;
                    start_point.y = current_y;
                    end_point.x = mid_x;
                    end_point.y = mid_y;
                    
                
                    int n = 10; 
                    int key_point_count = 0;
                    Point* key_points = get_key_points(start_point, end_point, n, current_y, &key_point_count);
                    
         
                    double x0 = current_x;
                    double y0 = current_y;
                    ctx->recip_spiral_xs[ctx->recip_spiral_count] = x0;
                    ctx->recip_spiral_ys[ctx->recip_spiral_count] = y0;
                    ctx->recip_spiral_count++;
                    
           
                    for (int i = 0; i < key_point_count; i++) {
                        double* spiral_xs = NULL;
                        double* spiral_ys = NULL;
                        int spiral_count = 0;
                        
                   
                        double k_val = 500 * log((i / 100.0) + 1.1);
                        
                        create_spiral(x0, y0, key_points[i].x, key_points[i].y, 
                                     current_y, i, k_val, &spiral_xs, &spiral_ys, 
                                     &spiral_count, 0.0, 6);
                        
                     
                        for (int j = 0; j < spiral_count; j++) {
                            ctx->recip_spiral_xs[ctx->recip_spiral_count] = spiral_xs[j];
                            ctx->recip_spiral_ys[ctx->recip_spiral_count] = spiral_ys[j];
                            ctx->recip_spiral_count++;
                        }
                        
                   
                        x0 = key_points[i].x;
                        y0 = key_points[i].y;
                        
                        free(spiral_xs);
                        free(spiral_ys);
                    }
                    
                    free(key_points);
                }
            }
            break;
            
        case STATE_RECIPROCATING_SPIRAL:
    
            if (ctx->recip_spiral_index < ctx->recip_spiral_count) {
                target.x = ctx->recip_spiral_xs[ctx->recip_spiral_index];
                target.y = ctx->recip_spiral_ys[ctx->recip_spiral_index];
                ctx->recip_spiral_index++;
            } else {
                ctx->state = STATE_COMPLETED;
                printf("Reciprocating spiral completed, Finished\n");
            }
            break;
            
        case STATE_INNER_SPIRAL:
            if (ctx->inner_spiral_index < ctx->inner_spiral_count) {
                target.x = ctx->inner_spiral_xs[ctx->inner_spiral_index];
                target.y = ctx->inner_spiral_ys[ctx->inner_spiral_index];
                ctx->inner_spiral_index++;
            } else {
                ctx->state = STATE_COMPLETED;
                printf("Inner spiral completed, Finished\n");
            }
            break;
        case STATE_COMPLETED:

            break;
            
        default:
            break;
    }
    

    target.x = clip_value(target.x, 0, MAX_X);
    target.y = clip_value(target.y, 0, MAX_Y);
    
    return target;
}

int main() {
    signal(SIGINT, ctrlc_handler);
    int i = 0;
    double start_x = 350, start_y = 350, start_angle = 180; 
    double current_x = start_x, current_y = start_y, theta_copy = start_angle;
    double target_x = 0, target_y = 0;
    int source_found = 0; 
    int reset_requested = 0;

    set_nonblocking(STDIN_FILENO);

    if (kh4_init(0, NULL) != 0) return -1;
    dsPic = knet_open("Khepera4:dsPic", KNET_BUS_I2C, 0, NULL);
    if (dsPic == NULL) return -2;

    kh4_SetPositionMargin(20, dsPic);
    kh4_ConfigurePID(10, 5, 1, dsPic);
    kh4_SetSpeedProfile(3, 0, 20, 1, 400, dsPic);
    kh4_SetMode(kh4RegIdle, dsPic);

    const char *serial_port = "/dev/ttyO0";
    int fd = init_serial_port(serial_port);
    if (fd < 0) return -1;


    AlgorithmContext algo_ctx;
    init_algorithm_context(&algo_ctx, start_x, start_y, start_angle);

    i = 0;
    while (i < 1000 && !quitReq && !source_found) {

        char ch;
        if (read(STDIN_FILENO, &ch, 1) > 0) {
            if (ch == 'r' || ch == 'R') {
                printf("Reset requested by user...\n");
                reset_requested = 1;
                break;
            }
        }
        
        int concentration = read_sensor_data(fd);
        if (concentration >= 0) {
            printf("Sensor concentration: %d | State: %d | Position: (%.1f, %.1f)\n", 
                   concentration, algo_ctx.state, current_x, current_y);
            
            double dist_to_source = distance_to_source(current_x, current_y);
            if (dist_to_source < SOURCE_DISTANCE_THRESHOLD) {
                printf("\n\n*** SOURCE FOUND! ***\n");
                printf("Final position: (%.1f, %.1f)\n", current_x, current_y);
                printf("Distance to source: %.1f cm\n", dist_to_source);
                source_found = 1;
                break;
            }
            
            Point next_target = decide_next_target(&algo_ctx, concentration, current_x, current_y);
            target_x = next_target.x;
            target_y = next_target.y;
            
            printf("Next target: (%.1f, %.1f) | Current: (%.1f, %.1f)\n", 
                   target_x, target_y, current_x, current_y);
            
            move_to_target(target_x, target_y, &current_x, &current_y, &theta_copy, &algo_ctx);
            
            dist_to_source = distance_to_source(current_x, current_y);
            if (dist_to_source < SOURCE_DISTANCE_THRESHOLD) {
                printf("\n\n*** SOURCE FOUND AFTER MOVEMENT! ***\n");
                printf("Final position: (%.1f, %.1f)\n", current_x, current_y);
                printf("Distance to source: %.1f cm\n", dist_to_source);
                source_found = 1;
                break;
            }
        } else {
            printf("No valid sensor data, retrying...\n");
        }
        usleep(500000);
        i++;
    }

    if (reset_requested) {
    
        kh4_set_speed(0, 0, dsPic);
        

        reset_to_start(&algo_ctx, &current_x, &current_y, &theta_copy);
        

        free_algorithm_context(&algo_ctx);
        init_algorithm_context(&algo_ctx, start_x, start_y, start_angle);
        

        i = 0;
        source_found = 0;
        reset_requested = 0;
        

        while (i < 1000 && !quitReq && !source_found) {

            char ch;
            if (read(STDIN_FILENO, &ch, 1) > 0) {
                if (ch == 'r' || ch == 'R') {
                    printf("Reset requested by user...\n");
                    reset_requested = 1;
                    break;
                }
            }
            
            int concentration = read_sensor_data(fd);
            if (concentration >= 0) {
                printf("Sensor concentration: %d | State: %d | Position: (%.1f, %.1f)\n", 
                       concentration, algo_ctx.state, current_x, current_y);
                
                double dist_to_source = distance_to_source(current_x, current_y);
                if (dist_to_source < SOURCE_DISTANCE_THRESHOLD) {
                    printf("\n\n*** SOURCE FOUND! ***\n");
                    printf("Final position: (%.1f, %.1f)\n", current_x, current_y);
                    printf("Distance to source: %.1f cm\n", dist_to_source);
                    source_found = 1;
                    break;
                }
                
                Point next_target = decide_next_target(&algo_ctx, concentration, current_x, current_y);
                target_x = next_target.x;
                target_y = next_target.y;
                
                printf("Next target: (%.1f, %.1f) | Current: (%.1f, %.1f)\n", 
                       target_x, target_y, current_x, current_y);
                
                move_to_target(target_x, target_y, &current_x, &current_y, &theta_copy, &algo_ctx);
                
                dist_to_source = distance_to_source(current_x, current_y);
                if (dist_to_source < SOURCE_DISTANCE_THRESHOLD) {
                    printf("\n\n*** SOURCE FOUND AFTER MOVEMENT! ***\n");
                    printf("Final position: (%.1f, %.1f)\n", current_x, current_y);
                    printf("Distance to source: %.1f cm\n", dist_to_source);
                    source_found = 1;
                    break;
                }
            } else {
                printf("No valid sensor data, retrying...\n");
            }
            usleep(500000);
            i++;
        }
    }


    save_trajectory(&algo_ctx, "trajectory.txt");
    

    free_algorithm_context(&algo_ctx);
    close(fd);
    

    set_blocking(STDIN_FILENO);
    

    kh4_set_speed(0, 0, dsPic);
    kh4_SetMode(kh4RegIdle, dsPic);
    
    if (!source_found) {
        printf("Source not found within iteration limit\n");
    }
    
    return 0;
}
