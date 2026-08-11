// hps3d160_stream.c
//
// Runs on the Arduino UNO Q's Linux side. Connects to the HPS-3D160-U over
// its USB CDC-ACM serial port using Hypersen's official HPS3D_SDK, starts
// continuous capture, and streams each frame's point cloud (XYZ, one
// float triple per pixel) over a TCP socket to a receiver running
// elsewhere on the LAN (see pointcloud_viewer.py).
//
// Wire protocol (all integers little-endian -- true on both aarch64 and
// x86_64, so no byte-swapping is done):
//   Per frame:
//     uint32_t points      (e.g. 9600 = 160*60)
//     uint16_t width        (e.g. 160)
//     uint16_t height       (e.g. 60)
//     uint32_t frame_cnt    (sensor's own frame counter, for drop detection)
//     float    xyz[points][3]   (raw HPS3D_PerPointCloudData_t array)
//
// Usage: ./hps3d160_stream <receiver-host> <receiver-port> [serial-device]
//   serial-device defaults to /dev/ttyACM0.

#include <arpa/inet.h>
#include <netdb.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "HPS3DUser_IF.h"

static int g_handle = -1;
static int g_sock = -1;
static volatile bool g_running = true;
static HPS3D_MeasureData_t g_measureData;

// Rolling FPS accounting, printed to stderr once per second.
static uint32_t g_frames_this_second = 0;
static struct timeval g_fps_window_start;

static void report_fps_if_due(void) {
    struct timeval now;
    gettimeofday(&now, NULL);
    double elapsed = (now.tv_sec - g_fps_window_start.tv_sec) +
                      (now.tv_usec - g_fps_window_start.tv_usec) / 1e6;
    if (elapsed >= 1.0) {
        fprintf(stderr, "[hps3d160_stream] ~%.1f fps\n", g_frames_this_second / elapsed);
        g_frames_this_second = 0;
        g_fps_window_start = now;
    }
}

// Sends `len` bytes in full, handling short writes. Returns false on error.
static bool send_all(int sock, const void *buf, size_t len) {
    const uint8_t *p = (const uint8_t *)buf;
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(sock, p + sent, len - sent, 0);
        if (n <= 0) {
            perror("send");
            return false;
        }
        sent += (size_t)n;
    }
    return true;
}

static void send_point_cloud_frame(const HPS3D_DepthData_t *depth) {
    if (g_sock < 0) return;

    uint32_t points = depth->point_cloud_data.points;
    uint16_t width = depth->point_cloud_data.width;
    uint16_t height = depth->point_cloud_data.height;
    uint32_t frame_cnt = depth->frame_cnt;

    uint8_t header[12];
    memcpy(header + 0, &points, 4);
    memcpy(header + 4, &width, 2);
    memcpy(header + 6, &height, 2);
    memcpy(header + 8, &frame_cnt, 4);

    size_t payload_bytes = (size_t)points * sizeof(HPS3D_PerPointCloudData_t);

    if (!send_all(g_sock, header, sizeof(header)) ||
        !send_all(g_sock, depth->point_cloud_data.point_data, payload_bytes)) {
        fprintf(stderr, "[hps3d160_stream] send failed, stopping\n");
        g_running = false;
        return;
    }

    g_frames_this_second++;
    report_fps_if_due();
}

static void event_callback(int handle, int eventType, uint8_t *data, int dataLen, void *userPara) {
    (void)handle;
    (void)dataLen;
    (void)userPara;

    switch ((HPS3D_EventType_t)eventType) {
        case HPS3D_FULL_DEPTH_EVEN:
            HPS3D_ConvertToMeasureData(data, &g_measureData, (HPS3D_EventType_t)eventType);
            send_point_cloud_frame(&g_measureData.full_depth_data);
            break;
        case HPS3D_DISCONNECT_EVEN:
            fprintf(stderr, "[hps3d160_stream] sensor disconnected\n");
            g_running = false;
            break;
        default:
            break;
    }
}

static void signal_handler(int sig) {
    (void)sig;
    g_running = false;
}

static int connect_to_receiver(const char *host, const char *port) {
    struct addrinfo hints, *res, *rp;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int err = getaddrinfo(host, port, &hints, &res);
    if (err != 0) {
        fprintf(stderr, "getaddrinfo(%s:%s): %s\n", host, port, gai_strerror(err));
        return -1;
    }

    int sock = -1;
    for (rp = res; rp != NULL; rp = rp->ai_next) {
        sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (sock < 0) continue;
        if (connect(sock, rp->ai_addr, rp->ai_addrlen) == 0) break;
        close(sock);
        sock = -1;
    }
    freeaddrinfo(res);
    return sock;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <receiver-host> <receiver-port> [serial-device]\n", argv[0]);
        return 1;
    }
    const char *host = argv[1];
    const char *port = argv[2];
    const char *device = (argc > 3) ? argv[3] : "/dev/ttyACM0";

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("SDK Ver: %s\n", HPS3D_GetSDKVersion());

    if (HPS3D_MeasureDataInit(&g_measureData) != HPS3D_RET_OK) {
        fprintf(stderr, "MeasureDataInit failed\n");
        return 1;
    }

    if (HPS3D_USBConnectDevice((char *)device, &g_handle) != HPS3D_RET_OK) {
        fprintf(stderr, "Failed to connect to %s\n", device);
        return 1;
    }
    printf("Device version: %s\n", HPS3D_GetDeviceVersion(g_handle));

    printf("Connecting to receiver at %s:%s ...\n", host, port);
    g_sock = connect_to_receiver(host, port);
    if (g_sock < 0) {
        fprintf(stderr, "Failed to connect to receiver %s:%s\n", host, port);
        HPS3D_CloseDevice(g_handle);
        HPS3D_MeasureDataFree(&g_measureData);
        return 1;
    }
    printf("Connected. Streaming point cloud frames (Ctrl-C to stop)...\n");

    gettimeofday(&g_fps_window_start, NULL);

    if (HPS3D_RegisterEventCallback(event_callback, NULL) != HPS3D_RET_OK) {
        fprintf(stderr, "RegisterEventCallback failed\n");
        goto cleanup;
    }

    if (HPS3D_StartCapture(g_handle) != HPS3D_RET_OK) {
        fprintf(stderr, "StartCapture failed\n");
        goto cleanup;
    }

    while (g_running) {
        usleep(100 * 1000);
    }

    HPS3D_StopCapture(g_handle);

cleanup:
    if (g_sock >= 0) close(g_sock);
    HPS3D_CloseDevice(g_handle);
    HPS3D_MeasureDataFree(&g_measureData);
    printf("Stopped.\n");
    return 0;
}
