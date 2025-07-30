// sleep.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#define UART_DEV "/dev/ttyS1"  // 실제 장치 경로로 변경 필요
#define BAUDRATE B9600

/** 
 * @brief UART 포트로 명령을 전송하고, 실패 시 에러 메시지를 JSON으로 출력 후 종료
 * @param cmd 전송할 문자열(SLEEP or WAKE) 명령
*/
void send_uart_command(const char *cmd) {
    int fd = open(UART_DEV, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("open");
        printf("{\"result\":\"error\",\"msg\":\"UART open failed\"}\n");
        exit(1);
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        printf("{\"result\":\"error\",\"msg\":\"UART config read failed\"}\n");
        exit(1);
    }

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    // 시리얼 8N1 구성
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 데이터 비트 8bit
    tty.c_iflag = IGNBRK;                        // 브레이크 무시
    tty.c_lflag = 0;                             // 캐노니컬 모드 비활성
    tty.c_oflag = 0;                             // 출력 플래그 제거
    tty.c_cc[VMIN] = 1;                          // 최소 1바이트 대기
    tty.c_cc[VTIME] = 5;                         // 타임아웃 0.5초

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        printf("{\"result\":\"error\",\"msg\":\"UART config apply failed\"}\n");
        exit(1);
    }

    write(fd, cmd, strlen(cmd));                // 명령 전송
    close(fd);
}

int main(void) {
    char *query = getenv("QUERY_STRING");
    printf("Content-Type: application/json\n\n");

    if (!query) {
        printf("{\"result\":\"error\",\"msg\":\"No query string\"}\n");
        return 1;
    }

    char mode[16] = {0};
    // 쿼리 스트링에서 mode 값을 파싱
    if (sscanf(query, "mode=%15s", mode) != 1) {
        printf("{\"result\":\"error\",\"msg\":\"Missing mode param\"}\n");
        return 1;
    }

    if (strcmp(mode, "sleep") == 0) {
        //send_uart_command("SLEEP\n");
        printf("{\"result\":\"ok\",\"msg\":\"Sleep command sent\"}\n");
    } else if (strcmp(mode, "wake") == 0) {
        //send_uart_command("WAKE\n");
        printf("{\"result\":\"ok\",\"msg\":\"Wake command sent\"}\n");
    } else {
        printf("{\"result\":\"error\",\"msg\":\"Unknown mode\"}\n");
        return 1;
    }

    return 0;
}
