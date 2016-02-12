//
// Created on 15.05.15.
//
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <bcm2835.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <signal.h>


#define TRIG 23
#define ECHO 24

enum {
    MIN, MID, MAX
} distMode;

const int leftForward = 28;
const int leftBackward = 27;

const int rightForward = 25;
const int rightBackward = 24;

const int servo = 1;

void setup() {
    pinMode(servo, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(1024);
    pwmSetClock(262);

    pinMode(leftForward, OUTPUT);
    pinMode(rightForward, OUTPUT);
    pinMode(leftBackward, OUTPUT);
    pinMode(rightBackward, OUTPUT);

    digitalWrite(leftForward, LOW);
    digitalWrite(rightForward, LOW);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightBackward, LOW);

    softPwmCreate(leftForward, 0, 99);
    softPwmCreate(rightForward, 0, 99);
    softPwmCreate(leftBackward, 0, 99);
    softPwmCreate(rightBackward, 0, 99);

    //for distance stuff
    if (!bcm2835_init()) {
        printf("Fehler: bcm_init()\n");
        exit(-1);
    }
}

void servoPwm(int pwm) {
    if (pwm < 0 || pwm > 140) {
        return;
    }
    int real = pwm + 40;
    pwmWrite(servo, real);
}

/**
 * STARTS
 */
void forward_left() {
    softPwmWrite(leftForward, 100);
}

void forward_right() {
    softPwmWrite(rightForward, 100);
}

void backward_left() {
    softPwmWrite(leftBackward, 100);
}

void backward_right() {
    softPwmWrite(rightBackward, 100);
}

void forward_leftPWM(int pwm) {
    softPwmWrite(leftForward, pwm);
}

void forward_rightPWM(int pwm) {
    softPwmWrite(rightForward, pwm);
}

void backward_leftPWM(int pwm) {
    softPwmWrite(leftBackward, pwm);
}

void backward_rightPWM(int pwm) {
    softPwmWrite(rightBackward, pwm);
}

void resetServo() {
    pwmWrite(servo, 40);
}

void forward() {
    forward_left();
    forward_right();
}

/**
 * STOPS
 */
void stop_forward_left() {
    softPwmWrite(leftForward, 0);
}

void stop_forward_right() {
    softPwmWrite(rightForward, 0);

}

void stop_backward_left() {
    softPwmWrite(leftBackward, 0);
}

void stop_backward_right() {
    softPwmWrite(rightBackward, 0);
}

/**
 * RESET
 */
void resetAll() {
    stop_forward_left();
    stop_forward_right();
    stop_backward_left();
    stop_backward_right();
    resetServo();
}

#define BUFFER_PAYLOAD 4;
#define BUFFER_SIZE 7;
#define BUFFER_INPUT_LEN 6;
#define PORT 2048;

int sockfd;
int newsockfd;

void sig_handler(int signo) {
    if (signo == SIGINT) {
        resetAll();
        shutdown(sockfd, 2);
        shutdown(newsockfd, 2);

        close(sockfd);
        close(newsockfd);
        printf("\nexiting\n");
        exit(0);
    }

}

int writeStringToClient(const char *buf) {
    if (write(newsockfd, buf, strlen(buf)) == -1) {
        printf("Error while sending message to client!");
    }
}

//TODO: Rewrite in wiringPi, not BCM2835
int distance(int mode) {
//    printf("Measuring distance!");
    char sbuf[20];
    uint64_t b_t; //begin timer
    float t_d; // Timer difference
    int min = 0, max = 0, mid = 0;
    float dist; // Distance
    int i, n;

    /* init */
    bcm2835_gpio_fsel(TRIG, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(ECHO, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(ECHO, BCM2835_GPIO_PUD_DOWN);

    int fff;
    const int count = 1; // how often to measure
    for (fff = 0; fff < count; fff++) {
        // send triger
        bcm2835_gpio_set(TRIG); // sets trigger pin to HIGH
        bcm2835_delayMicroseconds(10); // waits 10 microseconds
        bcm2835_gpio_clr(TRIG); // sets trigger pin to LOW
        bcm2835_delayMicroseconds(400); // waits 400 microseconds
        // wait for rising edge
        for (n = 0; n < 400; n++) {
            if (bcm2835_gpio_lev(ECHO)) break; // waits until echo pin is HIGH
            bcm2835_delayMicroseconds(5); // waits 5 microseconds
        }
        if (n >= 400) continue; // if the 400 timer ran out, skip
        b_t = bcm2835_st_read(); //returns "the value read from the System Timer Counter Lower 32 bits register"

        for (i = 0; i < 2000; i++) {
            if (!bcm2835_gpio_lev(ECHO)) break; // waits until echo pin is LOW
            bcm2835_delayMicroseconds(10); // waits 10 microseconds
        }
        // wait for falling edge
        t_d = bcm2835_st_read() - b_t; // calculate timer difference
        dist = (t_d * .0343) / 2; // do some magic, probably some correction - time<->distance mapping

        if (dist == 0) continue; // ignore if failed

        // set min and max if not set
        if (min == 0 || max == 0) {
            max = dist;
            min = dist;
        }
        if (dist > max) {
            max = dist; // set max if max
        }
        if (dist < min) {
            min = dist; // set min if min
        }
//        printf("%d\n", dist);
        mid += dist; // add distance to average
        delay(50);
    }
    if (mode == MID) return mid / count;
    if (mode == MIN) return min;
    if (mode == MAX) return max;
}


int main(int argc, char **argv) {
    int port = PORT;

    char buffer[7];
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;
    int bytes_read = 0;

    buffer[6] = '\0';


    printf("Starting Server.\n");
    wiringPiSetup();
    setup();
    signal(SIGINT, sig_handler);

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        printf("Error opening socket.\n");
    }

    int one = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(int));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);

    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        printf("Can't bind socket. Exiting!\n");
        exit(1);
    }
    listen(sockfd, 5);
    clilen = sizeof(cli_addr);
    while (1) {
        printf("Waiting for connections...\n");
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
        if (newsockfd < 0) {
            printf("Error while connecting the client!\n");
        } else {
            printf("Client connected!\n");
        }


        while (1) {
            bzero(buffer, 6);
            bytes_read = read(newsockfd, buffer, 6);
            if (bytes_read == 0) {
                printf("Lost client.\n");
                resetAll();
                break;
            }

            if (bytes_read != 6 && bytes_read != 5) { // nc sends 5 bytes, telnet 6
                printf("Skip command, length: %d - %s\n", bytes_read, buffer);
                printf("Bytes read: %d\n", bytes_read);
                continue;
            }

            buffer[4] = '\0';

            char c[2];
            c[0] = buffer[2];
            c[1] = buffer[3];
            int pwm = atoi(c);

            printf("%d\n", pwm);

            if (buffer[0] == 'Y') {
                char s[3];
                s[0] = buffer[1];
                s[1] = buffer[2];
                s[2] = buffer[3];
                int pwmS = atoi(s);
                servoPwm(pwmS);

            } else if (buffer[0] == 'F' && buffer[1] == 'L') {
                printf("forward left argument");
                forward_leftPWM(pwm);
            } else if (buffer[0] == 'F' && buffer[1] == 'R') {
                printf("forward right argument");
                forward_rightPWM(pwm);
            } else if (buffer[0] == 'B' && buffer[1] == 'L') {
                printf("backward left argument");
                backward_leftPWM(pwm);
            } else if (buffer[0] == 'B' && buffer[1] == 'R') {
                printf("backward right argument");
                backward_rightPWM(pwm);
            } else if (buffer[0] == 'F' && buffer[1] == '0') {
                resetAll();
                printf("forward argument");
                forward_leftPWM(pwm);
                forward_rightPWM(pwm);
            } else if (buffer[0] == 'B' && buffer[1] == '0') {
                resetAll();
                printf("backward argument");
                backward_rightPWM(pwm);
                backward_leftPWM(pwm);
            } else if (buffer[0] == 'R' && buffer[1] == 'L') {
                resetAll();
                printf("rotate left argument");
                forward_rightPWM(pwm);
                backward_leftPWM(pwm);
            } else if (buffer[0] == 'R' && buffer[1] == 'R') {
                resetAll();
                printf("rotate right argument");
                backward_rightPWM(pwm);
                forward_leftPWM(pwm);
            } else if (strcmp(buffer, "0000") == 0) {
                printf("reset argument");
                resetAll();
            } else if (strcmp(buffer, "EXIT") == 0) {
                printf("exiting");
                resetAll();
                sig_handler(SIGINT);
            } else if (strcmp(buffer, "C001") == 0) {
                resetAll();
                printf("Client disconnected\n");
                shutdown(newsockfd, 2);
                close(newsockfd);
                break;
            } else if (strcmp(buffer, "AUTO") == 0) {
                int dist;
                int threshold = 10;
                resetAll();

                dist = distance(MID);
                if (dist < threshold && dist != 0) {
                    printf("Distance lower than threshold: %d.\n", dist);
                    continue;
                }
                forward();

                while (1) {
                    dist = distance(MIN);
                    printf("Dist: %d \n", dist);
                    if (dist < threshold && dist != 0) {
                        resetAll();
                        writeStringToClient("STOP\n");
                        break;
                    }
                }
                continue;
            } else if (strcmp(buffer, "LOOP") == 0) {
                while (1) {
                    printf("%d\n", distance(MIN));
                    usleep(500000);
                }
            } else if (strcmp(buffer, "TEST") == 0) {
                writeStringToClient("Response test 2\n");
                forward_leftPWM(100);
                usleep(5000000);
                forward_leftPWM(75);
                usleep(5000000);
                forward_leftPWM(50);
                usleep(5000000);
                softPwmWrite(leftForward, 0);
            } else if (strcmp(buffer, "DIST") == 0) {
                int dist = distance(MID);
                char str[10];
                snprintf(str, 10, "%d", dist);
                writeStringToClient(str);
            } else {
                printf("ERROR IN PARAMETER!");
                writeStringToClient("Unknown command!\n");
            }
            printf("\n");
        }
    }
}
