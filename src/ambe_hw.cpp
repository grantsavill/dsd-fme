#define AMBE_HW_IMPLEMENTATION
#include "ambe_hw.h"
#include "dsd.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <stdint.h>

#if defined(_WIN32) || defined(__CYGWIN__)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>
#endif

static const unsigned char AMBE_START_BYTE       = 0x61U;
static const unsigned char AMBE_TYPE_CONTROL     = 0x00U;
static const unsigned char AMBE_TYPE_AMBE        = 0x01U;
static const unsigned char AMBE_TYPE_AUDIO       = 0x02U;
static const unsigned char AMBE_CONTROL_RATEP    = 0x0AU;
static const unsigned char AMBE_CONTROL_INIT     = 0x0BU;
static const unsigned char AMBE_CONTROL_PRODID   = 0x30U;
static const unsigned char AMBE_CONTROL_RESET    = 0x33U;
static const unsigned char AMBE3003_PKT_CHANNEL0 = 0x40U; // Timeslot 1
static const unsigned char AMBE3003_PKT_CHANNEL1 = 0x41U; // Timeslot 2

static const unsigned char PKT_REQ_PRODID[] = {
    AMBE_START_BYTE, 0x00U, 0x01U, AMBE_TYPE_CONTROL, AMBE_CONTROL_PRODID
};

static const unsigned char PKT_SOFT_RESET[] = {
    AMBE_START_BYTE, 0x00U, 0x01U, AMBE_TYPE_CONTROL, AMBE_CONTROL_RESET
};

// Initialize AMBE3000 decoder state (bit 1 = 0x02) when switching streams/slots
static const unsigned char PKT_AMBE3000_INIT_DEC[6] = {
    AMBE_START_BYTE, 0x00U, 0x02U, AMBE_TYPE_CONTROL, AMBE_CONTROL_INIT, 0x02U
};

// Single-channel AMBE3000 RATEP for 72-bit 3600x2450 bps AMBE+2 with FEC (SerialDV DV3000_REQ_3600X2450_RATEP)
static const unsigned char PKT_AMBE3000_RATEP_3600X2450[17] = {
    AMBE_START_BYTE, 0x00U, 0x0DU, AMBE_TYPE_CONTROL, AMBE_CONTROL_RATEP,
    0x04U, 0x31U, 0x07U, 0x54U, 0x24U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x6FU, 0x48U
};

// Single-channel AMBE3000 RATEP for 72-bit D-STAR 3600x2400 AMBE (SerialDV DV3000_REQ_3600X2400_RATEP)
static const unsigned char PKT_AMBE3000_RATEP_3600X2400[17] = {
    AMBE_START_BYTE, 0x00U, 0x0DU, AMBE_TYPE_CONTROL, AMBE_CONTROL_RATEP,
    0x01U, 0x30U, 0x07U, 0x63U, 0x40U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x48U
};

enum AmbeRateMode
{
    AMBE_RATE_NONE      = 0,
    AMBE_RATE_3600X2450 = 2450,
    AMBE_RATE_3600X2400 = 3600
};

// DMR / P25p2 / NXDN / YSF V/D1 72-bit air-frame interleave schedule matching dmr_const.h (rW, rX, rY, rZ)
static const int DMR_RW[36] = {
    0, 1, 0, 1, 0, 1,
    0, 1, 0, 1, 0, 1,
    0, 1, 0, 1, 0, 1,
    0, 1, 0, 1, 0, 2,
    0, 2, 0, 2, 0, 2,
    0, 2, 0, 2, 0, 2
};

static const int DMR_RX[36] = {
    23, 10, 22,  9, 21,  8,
    20,  7, 19,  6, 18,  5,
    17,  4, 16,  3, 15,  2,
    14,  1, 13,  0, 12, 10,
    11,  9, 10,  8,  9,  7,
     8,  6,  7,  5,  6,  4
};

static const int DMR_RY[36] = {
    0, 2, 0, 2, 0, 2,
    0, 2, 0, 3, 0, 3,
    1, 3, 1, 3, 1, 3,
    1, 3, 1, 3, 1, 3,
    1, 3, 1, 3, 1, 3,
    1, 3, 1, 3, 1, 3
};

static const int DMR_RZ[36] = {
     5,  3,  4,  2,  3,  1,
     2,  0,  1, 13,  0, 12,
    22, 11, 21, 10, 20,  9,
    19,  8, 18,  7, 17,  6,
    16,  5, 15,  4, 14,  3,
    13,  2, 12,  1, 11,  0
};

// Golay(23,12) generator polynomials matching mbelib ecc.c golayGenerator[12] and fec.c Golay_23_12_m_G
static const uint16_t GOLAY_GEN[12] = {
    0x63aU, 0x31dU, 0x7b4U, 0x3daU, 0x1edU, 0x6ccU,
    0x366U, 0x1b3U, 0x6e3U, 0x54bU, 0x49fU, 0x475U
};

// D-STAR 72-bit air-frame interleave coordinates matching dstar_const.h (dW, dX)
static const int DSTAR_DW[72] = {
    0, 0, 3, 2, 1, 1, 0, 0, 1, 1, 0, 0,
    3, 2, 1, 1, 3, 2, 1, 1, 0, 0, 3, 2,
    0, 0, 3, 2, 1, 1, 0, 0, 1, 1, 0, 0,
    3, 2, 1, 1, 3, 2, 1, 1, 0, 0, 3, 2,
    0, 0, 3, 2, 1, 1, 0, 0, 1, 1, 0, 0,
    3, 2, 1, 1, 3, 3, 2, 1, 0, 0, 3, 3
};

static const int DSTAR_DX[72] = {
    10, 22, 11,  9, 10, 22, 11, 23,  8, 20,  9, 21,
    10,  8,  9, 21,  8,  6,  7, 19,  8, 20,  9,  7,
     6, 18,  7,  5,  6, 18,  7, 19,  4, 16,  5, 17,
     6,  4,  5, 17,  4,  2,  3, 15,  4, 16,  5,  3,
     2, 14,  3,  1,  2, 14,  3, 15,  0, 12,  1, 13,
     2,  0,  1, 13,  0, 12, 10, 11,  0, 12,  1, 13
};

static int g_ambe_active = 0;
static int g_ambe_type = 0; // 3003 for AMBE3003, 3000 for AMBE3000
static int g_channel_rate[2] = {AMBE_RATE_NONE, AMBE_RATE_NONE};
static const dsd_opts *g_opts = NULL;
static dsd_state *g_state = NULL;
static const mbe_parms *g_slot2_mp = NULL;
static char g_port_label[64] = {0};
static char g_last_dev_path[64] = {0};
static char g_last_port_label[64] = {0};

// First-active-slot lock state for single-channel AMBE3000
static int g_ambe3000_locked_slot = -1;
static int g_ambe3000_last_decoded_slot = -1;
static bool g_channel_needs_init[2] = {false, false};
static uint64_t g_ambe3000_locked_last_ms = 0;
static uint64_t g_ambe3000_muted_last_ms[2] = {0, 0};
static int g_frame_muted[2] = {0, 0};
static int g_frame_hw_decoded[2] = {0, 0};
static int g_consecutive_errors = 0;
static bool g_io_error = false;
static bool g_auto_reconnect = false;
static uint64_t g_next_reconnect_ms = 0;
static const uint64_t AMBE_RECONNECT_INTERVAL_MS = 5000ULL;

// Captured 72-bit over-the-air AMBE+2 frame (before mbelib in-place ECC/demodulation) and post-ECC 49-bit voice
static char g_captured_ambe_fr[4][24] = {{0}};
static char g_captured_ambe_d[49] = {0};
static int g_captured_e0 = 0;
static int g_captured_e1 = 0;
static bool g_captured_fr_valid = false;

// Rolling 90-frame (5 superframes = 1.8 s) bit-error rate (BER) window per slot
static const int BER_WINDOW_FRAMES = 90;
static uint8_t g_ber_err_ring[2][BER_WINDOW_FRAMES] = {{0}};
static uint8_t g_ber_tot_ring[2][BER_WINDOW_FRAMES] = {{0}};
static int g_ber_ring_idx[2] = {0, 0};
static int g_ber_ring_cnt[2] = {0, 0};
static uint64_t g_call_last_frame_ms[2] = {0, 0};

static void get_time_str(char *buf, size_t len)
{
    time_t t = ::time(NULL);
    struct tm *ptm = ::localtime(&t);
    if (ptm)
        snprintf(buf, len, "%02d:%02d:%02d", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    else
        snprintf(buf, len, "00:00:00");
}

static uint64_t get_monotonic_ms(void)
{
#if defined(_WIN32) || defined(__CYGWIN__)
    return (uint64_t)::GetTickCount64();
#else
    struct timespec ts;
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) == 0)
    {
        return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
    }
    return 0;
#endif
}

static bool is_silent_ambe_d(const char ambe_d[49])
{
    bool allZero = true;
    for (int i = 0; i < 49; i++)
    {
        if (ambe_d[i] & 1)
        {
            allZero = false;
            break;
        }
    }
    if (allZero)
        return true;

    int b0 = ((ambe_d[0] & 1) << 6) |
             ((ambe_d[1] & 1) << 5) |
             ((ambe_d[2] & 1) << 4) |
             ((ambe_d[3] & 1) << 3) |
             ((ambe_d[37] & 1) << 2) |
             ((ambe_d[38] & 1) << 1) |
             (ambe_d[39] & 1);
    return (b0 >= 120 && b0 <= 123);
}

static bool is_valid_ambe_header(const unsigned char hdr[6], unsigned char expectedType)
{
    if (hdr[0] != AMBE_START_BYTE)
        return false;
    unsigned int payloadLen = ((unsigned int)hdr[1] << 8) | (unsigned int)hdr[2];
    unsigned char pktType = hdr[3];
    if (pktType != expectedType)
        return false;

    if (pktType == AMBE_TYPE_AUDIO)
    {
        // AMBE3000 audio packet: 61 01 42 02 00 A0 <320 PCM bytes> (payloadLen = 322)
        if (payloadLen == 322U && hdr[4] == 0x00U && hdr[5] == 0xA0U)
            return true;
        // AMBE3003 audio packet: 61 01 43 02 [40|41] 00 A0 <320 PCM bytes> (payloadLen = 323)
        if (payloadLen == 323U &&
            (hdr[4] == AMBE3003_PKT_CHANNEL0 || hdr[4] == AMBE3003_PKT_CHANNEL1) &&
            hdr[5] == 0x00U)
            return true;
        return false;
    }
    if (pktType == AMBE_TYPE_CONTROL)
    {
        return (payloadLen >= 1U && payloadLen <= 64U);
    }
    return false;
}

#if defined(_WIN32) || defined(__CYGWIN__)

static bool is_fatal_win32_error(DWORD err)
{
    return (err == ERROR_GEN_FAILURE ||
            err == ERROR_DEVICE_NOT_CONNECTED ||
            err == ERROR_BAD_COMMAND ||
            err == ERROR_ACCESS_DENIED ||
            err == ERROR_INVALID_HANDLE);
}

struct Win32SerialPort
{
    HANDLE handle;
    OVERLAPPED readOverlapped;
    OVERLAPPED writeOverlapped;
};

static Win32SerialPort g_win_port = {
    INVALID_HANDLE_VALUE, {0}, {0}
};

static void close_win32_port(Win32SerialPort *port)
{
    if (port->handle != INVALID_HANDLE_VALUE)
    {
        ::CancelIo(port->handle);
        ::CloseHandle(port->handle);
        port->handle = INVALID_HANDLE_VALUE;
    }
    if (port->readOverlapped.hEvent != NULL)
    {
        ::CloseHandle(port->readOverlapped.hEvent);
        port->readOverlapped.hEvent = NULL;
    }
    if (port->writeOverlapped.hEvent != NULL)
    {
        ::CloseHandle(port->writeOverlapped.hEvent);
        port->writeOverlapped.hEvent = NULL;
    }
}

static bool open_win32_port(Win32SerialPort *port, const char *devPath, DWORD baudRate)
{
    close_win32_port(port);

    DWORD errCode = 0;
    HANDLE h = ::CreateFileA(
        devPath,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,
        NULL
    );

    if (h == INVALID_HANDLE_VALUE)
        return false;

    ::SetupComm(h, 4096, 4096);

    DCB dcb;
    memset(&dcb, 0, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);

    if (::GetCommState(h, &dcb) == 0)
    {
        ::ClearCommError(h, &errCode, NULL);
        ::CloseHandle(h);
        return false;
    }

    dcb.BaudRate = baudRate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.fParity = FALSE;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fInX = FALSE;
    dcb.fOutX = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;

    if (::SetCommState(h, &dcb) == 0)
    {
        ::ClearCommError(h, &errCode, NULL);
        ::CloseHandle(h);
        return false;
    }

    COMMTIMEOUTS timeouts;
    memset(&timeouts, 0, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = 1UL;
    timeouts.ReadTotalTimeoutMultiplier = 0UL;
    timeouts.ReadTotalTimeoutConstant = 25UL;
    timeouts.WriteTotalTimeoutMultiplier = 0UL;
    timeouts.WriteTotalTimeoutConstant = 50UL;

    if (!::SetCommTimeouts(h, &timeouts))
    {
        ::ClearCommError(h, &errCode, NULL);
        ::CloseHandle(h);
        return false;
    }

    ::EscapeCommFunction(h, CLRDTR);
    ::EscapeCommFunction(h, SETRTS);
    ::ClearCommError(h, &errCode, NULL);
    ::PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

    memset(&port->readOverlapped, 0, sizeof(OVERLAPPED));
    memset(&port->writeOverlapped, 0, sizeof(OVERLAPPED));
    port->readOverlapped.hEvent = ::CreateEventA(NULL, TRUE, FALSE, NULL);
    port->writeOverlapped.hEvent = ::CreateEventA(NULL, TRUE, FALSE, NULL);
    port->handle = h;
    return true;
}

static bool win32_write_all(Win32SerialPort *port, const unsigned char *buffer, DWORD length)
{
    if (port->handle == INVALID_HANDLE_VALUE)
    {
        g_io_error = true;
        return false;
    }

    DWORD ptr = 0;
    while (ptr < length)
    {
        DWORD bytes = 0;
        ::ResetEvent(port->writeOverlapped.hEvent);
        BOOL res = ::WriteFile(port->handle, buffer + ptr, length - ptr, &bytes, &port->writeOverlapped);
        if (!res)
        {
            DWORD error = ::GetLastError();
            if (error != ERROR_IO_PENDING)
            {
                if (is_fatal_win32_error(error))
                    g_io_error = true;
                return false;
            }
            DWORD waitRes = ::WaitForSingleObject(port->writeOverlapped.hEvent, 150);
            if (waitRes != WAIT_OBJECT_0)
            {
                ::CancelIo(port->handle);
                return false;
            }
            if (!::GetOverlappedResult(port->handle, &port->writeOverlapped, &bytes, FALSE))
            {
                if (is_fatal_win32_error(::GetLastError()))
                    g_io_error = true;
                return false;
            }
        }
        if (bytes == 0)
            return false;
        ptr += bytes;
    }
    return true;
}

static int win32_read_bytes(Win32SerialPort *port, unsigned char *buffer, DWORD length, int timeoutMs)
{
    if (port->handle == INVALID_HANDLE_VALUE)
    {
        g_io_error = true;
        return -1;
    }

    uint64_t startMs = get_monotonic_ms();
    DWORD totalRead = 0;

    while (totalRead < length)
    {
        int elapsed = (int)(get_monotonic_ms() - startMs);
        int remMs = timeoutMs - elapsed;
        if (remMs <= 0)
            return -1;

        DWORD errCode = 0;
        COMSTAT stat;
        if (!::ClearCommError(port->handle, &errCode, &stat))
        {
            g_io_error = true;
            return -1;
        }

        DWORD toRead = length - totalRead;
        DWORD bytes = 0;
        ::ResetEvent(port->readOverlapped.hEvent);
        BOOL res = ::ReadFile(port->handle, buffer + totalRead, toRead, &bytes, &port->readOverlapped);
        if (!res)
        {
            DWORD error = ::GetLastError();
            if (error != ERROR_IO_PENDING)
            {
                if (is_fatal_win32_error(error))
                    g_io_error = true;
                return -1;
            }
            DWORD waitSlice = (DWORD)((remMs < 35) ? remMs : 35);
            DWORD waitRes = ::WaitForSingleObject(port->readOverlapped.hEvent, waitSlice);
            if (waitRes == WAIT_TIMEOUT)
            {
                ::CancelIo(port->handle);
                ::GetOverlappedResult(port->handle, &port->readOverlapped, &bytes, TRUE);
                totalRead += bytes;
                continue;
            }
            if (waitRes != WAIT_OBJECT_0)
            {
                ::CancelIo(port->handle);
                return -1;
            }
            if (!::GetOverlappedResult(port->handle, &port->readOverlapped, &bytes, FALSE))
            {
                if (is_fatal_win32_error(::GetLastError()))
                    g_io_error = true;
                return -1;
            }
        }
        totalRead += bytes;
    }

    return (int)totalRead;
}

static int win32_read_packet(Win32SerialPort *port, unsigned char *buf, unsigned int maxLen,
                             unsigned char expectedType, int timeoutMs)
{
    uint64_t startMs = get_monotonic_ms();
    unsigned int hdrSize = (expectedType == AMBE_TYPE_AUDIO) ? 6U : 4U;
    unsigned char hdr[6] = {0};

    // Fast path for audio packets: read the full fixed-size audio response in a single syscall
    if (expectedType == AMBE_TYPE_AUDIO)
    {
        unsigned int expectedTotal = (g_ambe_type == 3003) ? 327U : 326U;
        if (expectedTotal <= maxLen)
        {
            if (win32_read_bytes(port, buf, expectedTotal, timeoutMs) != (int)expectedTotal)
                return -1;

            if (is_valid_ambe_header(buf, AMBE_TYPE_AUDIO) &&
                (expectedTotal != 327U || buf[6] == 0xA0U))
            {
                return (int)expectedTotal;
            }

            // Resync within already-read bytes if stray bytes preceded the packet header
            for (unsigned int offset = 1U; offset + hdrSize <= expectedTotal; offset++)
            {
                if (is_valid_ambe_header(buf + offset, AMBE_TYPE_AUDIO))
                {
                    unsigned int pLen = ((unsigned int)buf[offset + 1] << 8) | (unsigned int)buf[offset + 2];
                    unsigned int tLen = pLen + 4U;
                    if (tLen > maxLen || tLen < expectedTotal - offset)
                        return -1;
                    unsigned int have = expectedTotal - offset;
                    memmove(buf, buf + offset, have);
                    unsigned int missing = tLen - have;
                    if (missing > 0 && win32_read_bytes(port, buf + have, missing, 100) != (int)missing)
                        return -1;
                    return (int)tLen;
                }
            }

            memcpy(hdr, buf + expectedTotal - hdrSize, hdrSize);
        }
    }
    else
    {
        if (win32_read_bytes(port, hdr, hdrSize, timeoutMs) != (int)hdrSize)
            return -1;
    }

    while (!is_valid_ambe_header(hdr, expectedType))
    {
        int elapsed = (int)(get_monotonic_ms() - startMs);
        if (elapsed >= timeoutMs)
            return -1;
        for (unsigned int i = 0; i + 1 < hdrSize; i++)
            hdr[i] = hdr[i + 1];
        if (win32_read_bytes(port, &hdr[hdrSize - 1], 1, timeoutMs - elapsed) != 1)
            return -1;
    }

    unsigned int payloadLen = ((unsigned int)hdr[1] << 8) | (unsigned int)hdr[2];
    unsigned int totalLen = payloadLen + 4U;
    if (totalLen > maxLen || totalLen < hdrSize)
        return -1;

    memcpy(buf, hdr, hdrSize);
    unsigned int remBytes = totalLen - hdrSize;
    if (remBytes > 0)
    {
        if (win32_read_bytes(port, buf + hdrSize, remBytes, 100) != (int)remBytes)
            return -1;
    }

    return (int)totalLen;
}

static bool probe_win32_port(const char *devPath, const char *displayPort)
{
    // Probe 460800 (AMBE3000) BEFORE 921600 (AMBE3003) so an AMBE3000 is never hit with 921600-baud framing noise
    const DWORD baudRates[2] = {460800UL, 921600UL};

    for (int s = 0; s < 2; s++)
    {
        DWORD baud = baudRates[s];
        Win32SerialPort port = {INVALID_HANDLE_VALUE, {0}, {0}};
        if (!open_win32_port(&port, devPath, baud))
        {
            if (s == 0)
                return false;
            continue;
        }

        unsigned char resp[512];
        memset(resp, 0, sizeof(resp));
        bool identified = false;

        // First attempt: direct PKT_REQ_PRODID
        if (win32_write_all(&port, PKT_REQ_PRODID, sizeof(PKT_REQ_PRODID)))
        {
            int rlen = win32_read_packet(&port, resp, sizeof(resp), AMBE_TYPE_CONTROL, 80);
            if (rlen >= 6 && resp[3] == AMBE_TYPE_CONTROL && resp[4] == AMBE_CONTROL_PRODID)
            {
                identified = true;
            }
        }

        // Recovery attempt: if chip UART state machine was left mid-packet, flush with 360 zero bytes + soft reset
        if (!identified && !g_io_error)
        {
            unsigned char flushZeros[360];
            memset(flushZeros, 0, sizeof(flushZeros));
            win32_write_all(&port, flushZeros, sizeof(flushZeros));
            win32_write_all(&port, PKT_SOFT_RESET, sizeof(PKT_SOFT_RESET));
            ::Sleep(50);
            DWORD errCode = 0;
            ::ClearCommError(port.handle, &errCode, NULL);
            ::PurgeComm(port.handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
            memset(resp, 0, sizeof(resp));
            if (win32_write_all(&port, PKT_REQ_PRODID, sizeof(PKT_REQ_PRODID)))
            {
                int rlen = win32_read_packet(&port, resp, sizeof(resp), AMBE_TYPE_CONTROL, 100);
                if (rlen >= 6 && resp[3] == AMBE_TYPE_CONTROL && resp[4] == AMBE_CONTROL_PRODID)
                {
                    identified = true;
                }
            }
        }

        if (identified)
        {
            bool is3003 = (baud == 921600) || (strstr((const char *)(resp + 5), "3003") != NULL);
            if (is3003)
            {
                bool configured = true;
                for (unsigned char ch = 0; ch < 2; ch++)
                {
                    unsigned char ratepPkt[18] = {
                        AMBE_START_BYTE, 0x00U, 0x0EU, AMBE_TYPE_CONTROL,
                        (unsigned char)(AMBE3003_PKT_CHANNEL0 + ch),
                        AMBE_CONTROL_RATEP,
                        0x04U, 0x31U, 0x07U, 0x54U, 0x24U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x6FU, 0x48U
                    };
                    if (!win32_write_all(&port, ratepPkt, sizeof(ratepPkt)))
                    {
                        configured = false;
                        break;
                    }
                    int rlen = win32_read_packet(&port, resp, sizeof(resp), AMBE_TYPE_CONTROL, 100);
                    if (rlen < 6 || resp[3] != AMBE_TYPE_CONTROL ||
                        resp[4] != (unsigned char)(AMBE3003_PKT_CHANNEL0 + ch) ||
                        resp[5] != AMBE_CONTROL_RATEP)
                    {
                        configured = false;
                        break;
                    }
                }
                if (configured)
                {
                    snprintf(g_port_label, sizeof(g_port_label), "%s", displayPort);
                    snprintf(g_last_dev_path, sizeof(g_last_dev_path), "%s", devPath);
                    snprintf(g_last_port_label, sizeof(g_last_port_label), "%s", displayPort);
                    g_win_port = port;
                    g_ambe_type = 3003;
                    g_ambe_active = 1;
                    g_channel_rate[0] = AMBE_RATE_3600X2450;
                    g_channel_rate[1] = AMBE_RATE_3600X2450;
                    g_io_error = false;
                    g_auto_reconnect = false;
                    g_next_reconnect_ms = 0;
                    fprintf(stderr, "AMBE3003 Detected on %s (%lu baud)\n", g_port_label, (unsigned long)baud);
                    fflush(stderr);
                    return true;
                }
            }
            else
            {
                if (win32_write_all(&port, PKT_AMBE3000_RATEP_3600X2450, sizeof(PKT_AMBE3000_RATEP_3600X2450)))
                {
                    int rlen = win32_read_packet(&port, resp, sizeof(resp), AMBE_TYPE_CONTROL, 100);
                    if (rlen >= 5 && resp[3] == AMBE_TYPE_CONTROL && resp[4] == AMBE_CONTROL_RATEP)
                    {
                        snprintf(g_port_label, sizeof(g_port_label), "%s", displayPort);
                        snprintf(g_last_dev_path, sizeof(g_last_dev_path), "%s", devPath);
                        snprintf(g_last_port_label, sizeof(g_last_port_label), "%s", displayPort);
                        g_win_port = port;
                        g_ambe_type = 3000;
                        g_ambe_active = 1;
                        g_channel_rate[0] = AMBE_RATE_3600X2450;
                        g_channel_rate[1] = AMBE_RATE_NONE;
                        g_io_error = false;
                        g_auto_reconnect = false;
                        g_next_reconnect_ms = 0;
                        fprintf(stderr, "AMBE3000 Detected on %s (%lu baud)\n", g_port_label, (unsigned long)baud);
                        fflush(stderr);
                        return true;
                    }
                }
            }
        }

        close_win32_port(&port);
        g_io_error = false;
    }

    return false;
}

#else // POSIX (Linux / macOS)

static int g_serial_fd = -1;

static int open_posix_serial(const char *devPath, speed_t speedConstant)
{
    int fd = ::open(devPath, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
        return -1;

    if (!::isatty(fd))
    {
        ::close(fd);
        return -1;
    }

    struct termios tio;
    if (::tcgetattr(fd, &tio) < 0)
    {
        ::close(fd);
        return -1;
    }

    tio.c_lflag &= ~(ECHO | ECHOE | ICANON | IEXTEN | ISIG);
    tio.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON | IXOFF | IXANY);
    tio.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
    tio.c_cflag |= (CS8 | CLOCAL | CREAD);
    tio.c_oflag &= ~(OPOST);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;

    ::cfsetospeed(&tio, speedConstant);
    ::cfsetispeed(&tio, speedConstant);

    if (::tcsetattr(fd, TCSANOW, &tio) < 0)
    {
        ::close(fd);
        return -1;
    }

    ::tcflush(fd, TCIOFLUSH);
    return fd;
}

static bool posix_write_all(int fd, const unsigned char *buf, unsigned int len)
{
    unsigned int written = 0;
    while (written < len)
    {
        ssize_t n = ::write(fd, buf + written, len - written);
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EINTR)
                continue;
            g_io_error = true;
            return false;
        }
        written += (unsigned int)n;
    }
    return true;
}

static int posix_read_bytes(int fd, unsigned char *buf, unsigned int len, int timeoutMs)
{
    uint64_t startMs = get_monotonic_ms();
    unsigned int got = 0;
    while (got < len)
    {
        int elapsed = (int)(get_monotonic_ms() - startMs);
        int remMs = timeoutMs - elapsed;
        if (remMs <= 0)
            return -1;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        struct timeval tv;
        tv.tv_sec = remMs / 1000;
        tv.tv_usec = (remMs % 1000) * 1000;

        int r = ::select(fd + 1, &fds, NULL, NULL, &tv);
        if (r < 0)
        {
            if (errno == EINTR)
                continue;
            g_io_error = true;
            return -1;
        }
        if (r == 0)
            return -1;

        ssize_t n = ::read(fd, buf + got, len - got);
        if (n <= 0)
        {
            if (n < 0 && (errno == EAGAIN || errno == EINTR))
                continue;
            g_io_error = true;
            return -1;
        }
        got += (unsigned int)n;
    }
    return (int)got;
}

static int posix_read_packet(int fd, unsigned char *buf, unsigned int maxLen,
                             unsigned char expectedType, int timeoutMs)
{
    uint64_t startMs = get_monotonic_ms();
    unsigned int hdrSize = (expectedType == AMBE_TYPE_AUDIO) ? 6U : 4U;
    unsigned char hdr[6] = {0};

    // Fast path for audio packets: read the full fixed-size audio response in a single read loop
    if (expectedType == AMBE_TYPE_AUDIO)
    {
        unsigned int expectedTotal = (g_ambe_type == 3003) ? 327U : 326U;
        if (expectedTotal <= maxLen)
        {
            if (posix_read_bytes(fd, buf, expectedTotal, timeoutMs) != (int)expectedTotal)
                return -1;

            if (is_valid_ambe_header(buf, AMBE_TYPE_AUDIO) &&
                (expectedTotal != 327U || buf[6] == 0xA0U))
            {
                return (int)expectedTotal;
            }

            for (unsigned int offset = 1U; offset + hdrSize <= expectedTotal; offset++)
            {
                if (is_valid_ambe_header(buf + offset, AMBE_TYPE_AUDIO))
                {
                    unsigned int pLen = ((unsigned int)buf[offset + 1] << 8) | (unsigned int)buf[offset + 2];
                    unsigned int tLen = pLen + 4U;
                    if (tLen > maxLen || tLen < expectedTotal - offset)
                        return -1;
                    unsigned int have = expectedTotal - offset;
                    memmove(buf, buf + offset, have);
                    unsigned int missing = tLen - have;
                    if (missing > 0 && posix_read_bytes(fd, buf + have, missing, 100) != (int)missing)
                        return -1;
                    return (int)tLen;
                }
            }

            memcpy(hdr, buf + expectedTotal - hdrSize, hdrSize);
        }
    }
    else
    {
        if (posix_read_bytes(fd, hdr, hdrSize, timeoutMs) != (int)hdrSize)
            return -1;
    }

    while (!is_valid_ambe_header(hdr, expectedType))
    {
        int elapsed = (int)(get_monotonic_ms() - startMs);
        if (elapsed >= timeoutMs)
            return -1;
        for (unsigned int i = 0; i + 1 < hdrSize; i++)
            hdr[i] = hdr[i + 1];
        if (posix_read_bytes(fd, &hdr[hdrSize - 1], 1, timeoutMs - elapsed) != 1)
            return -1;
    }

    unsigned int payloadLen = ((unsigned int)hdr[1] << 8) | (unsigned int)hdr[2];
    unsigned int totalLen = payloadLen + 4U;
    if (totalLen > maxLen || totalLen < hdrSize)
        return -1;

    memcpy(buf, hdr, hdrSize);
    unsigned int remBytes = totalLen - hdrSize;
    if (remBytes > 0)
    {
        if (posix_read_bytes(fd, buf + hdrSize, remBytes, 100) != (int)remBytes)
            return -1;
    }

    return (int)totalLen;
}

static bool probe_posix_port(const char *devPath)
{
    struct SpeedEntry { speed_t speed; int baud; };
    const SpeedEntry speeds[] = {
        {B460800, 460800},
#ifdef B921600
        {B921600, 921600}
#endif
    };

    for (size_t s = 0; s < sizeof(speeds) / sizeof(speeds[0]); s++)
    {
        int fd = open_posix_serial(devPath, speeds[s].speed);
        if (fd < 0)
        {
            if (s == 0)
                return false;
            continue;
        }

        unsigned char resp[512];
        memset(resp, 0, sizeof(resp));
        bool identified = false;

        if (posix_write_all(fd, PKT_REQ_PRODID, sizeof(PKT_REQ_PRODID)))
        {
            int rlen = posix_read_packet(fd, resp, sizeof(resp), AMBE_TYPE_CONTROL, 80);
            if (rlen >= 6 && resp[3] == AMBE_TYPE_CONTROL && resp[4] == AMBE_CONTROL_PRODID)
            {
                identified = true;
            }
        }

        if (!identified && !g_io_error)
        {
            unsigned char flushZeros[360];
            memset(flushZeros, 0, sizeof(flushZeros));
            posix_write_all(fd, flushZeros, sizeof(flushZeros));
            posix_write_all(fd, PKT_SOFT_RESET, sizeof(PKT_SOFT_RESET));
            usleep(50000);
            ::tcflush(fd, TCIOFLUSH);
            memset(resp, 0, sizeof(resp));
            if (posix_write_all(fd, PKT_REQ_PRODID, sizeof(PKT_REQ_PRODID)))
            {
                int rlen = posix_read_packet(fd, resp, sizeof(resp), AMBE_TYPE_CONTROL, 100);
                if (rlen >= 6 && resp[3] == AMBE_TYPE_CONTROL && resp[4] == AMBE_CONTROL_PRODID)
                {
                    identified = true;
                }
            }
        }

        if (identified)
        {
            bool is3003 = (speeds[s].baud == 921600) || (strstr((const char *)(resp + 5), "3003") != NULL);
            if (is3003)
            {
                bool configured = true;
                for (unsigned char ch = 0; ch < 2; ch++)
                {
                    unsigned char ratepPkt[18] = {
                        AMBE_START_BYTE, 0x00U, 0x0EU, AMBE_TYPE_CONTROL,
                        (unsigned char)(AMBE3003_PKT_CHANNEL0 + ch),
                        AMBE_CONTROL_RATEP,
                        0x04U, 0x31U, 0x07U, 0x54U, 0x24U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x6FU, 0x48U
                    };
                    if (!posix_write_all(fd, ratepPkt, sizeof(ratepPkt)))
                    {
                        configured = false;
                        break;
                    }
                    int rlen = posix_read_packet(fd, resp, sizeof(resp), AMBE_TYPE_CONTROL, 100);
                    if (rlen < 6 || resp[3] != AMBE_TYPE_CONTROL ||
                        resp[4] != (unsigned char)(AMBE3003_PKT_CHANNEL0 + ch) ||
                        resp[5] != AMBE_CONTROL_RATEP)
                    {
                        configured = false;
                        break;
                    }
                }
                if (configured)
                {
                    snprintf(g_port_label, sizeof(g_port_label), "%s", devPath);
                    snprintf(g_last_dev_path, sizeof(g_last_dev_path), "%s", devPath);
                    snprintf(g_last_port_label, sizeof(g_last_port_label), "%s", devPath);
                    g_serial_fd = fd;
                    g_ambe_type = 3003;
                    g_ambe_active = 1;
                    g_channel_rate[0] = AMBE_RATE_3600X2450;
                    g_channel_rate[1] = AMBE_RATE_3600X2450;
                    g_io_error = false;
                    g_auto_reconnect = false;
                    g_next_reconnect_ms = 0;
                    fprintf(stderr, "AMBE3003 Detected on %s (%d baud)\n", g_port_label, speeds[s].baud);
                    fflush(stderr);
                    return true;
                }
            }
            else
            {
                if (posix_write_all(fd, PKT_AMBE3000_RATEP_3600X2450, sizeof(PKT_AMBE3000_RATEP_3600X2450)))
                {
                    int rlen = posix_read_packet(fd, resp, sizeof(resp), AMBE_TYPE_CONTROL, 100);
                    if (rlen >= 5 && resp[3] == AMBE_TYPE_CONTROL && resp[4] == AMBE_CONTROL_RATEP)
                    {
                        snprintf(g_port_label, sizeof(g_port_label), "%s", devPath);
                        snprintf(g_last_dev_path, sizeof(g_last_dev_path), "%s", devPath);
                        snprintf(g_last_port_label, sizeof(g_last_port_label), "%s", devPath);
                        g_serial_fd = fd;
                        g_ambe_type = 3000;
                        g_ambe_active = 1;
                        g_channel_rate[0] = AMBE_RATE_3600X2450;
                        g_channel_rate[1] = AMBE_RATE_NONE;
                        g_io_error = false;
                        g_auto_reconnect = false;
                        g_next_reconnect_ms = 0;
                        fprintf(stderr, "AMBE3000 Detected on %s (%d baud)\n", g_port_label, speeds[s].baud);
                        fflush(stderr);
                        return true;
                    }
                }
            }
        }

        ::close(fd);
        g_io_error = false;
    }

    return false;
}

#endif

static void purge_rx_buffer(void)
{
#if defined(_WIN32) || defined(__CYGWIN__)
    if (g_win_port.handle != INVALID_HANDLE_VALUE)
    {
        DWORD errCode = 0;
        ::ClearCommError(g_win_port.handle, &errCode, NULL);
        ::PurgeComm(g_win_port.handle, PURGE_RXCLEAR);
    }
#else
    if (g_serial_fd >= 0)
    {
        ::tcflush(g_serial_fd, TCIFLUSH);
    }
#endif
}

static bool ensure_channel_rate(int slot, int targetRate)
{
    int chIdx = (g_ambe_type == 3003 && slot == 1) ? 1 : 0;
    if (g_channel_rate[chIdx] == targetRate)
        return true;

    const unsigned char *baseRatePkt = (targetRate == AMBE_RATE_3600X2400)
                                           ? PKT_AMBE3000_RATEP_3600X2400
                                           : PKT_AMBE3000_RATEP_3600X2450;
    unsigned char resp[512];

    if (g_ambe_type == 3003)
    {
        unsigned char ratepPkt[18];
        ratepPkt[0] = AMBE_START_BYTE;
        ratepPkt[1] = 0x00U;
        ratepPkt[2] = 0x0EU;
        ratepPkt[3] = AMBE_TYPE_CONTROL;
        ratepPkt[4] = (unsigned char)(AMBE3003_PKT_CHANNEL0 + chIdx);
        ratepPkt[5] = AMBE_CONTROL_RATEP;
        memcpy(ratepPkt + 6, baseRatePkt + 5, 12);

#if defined(_WIN32) || defined(__CYGWIN__)
        if (g_win_port.handle == INVALID_HANDLE_VALUE)
            return false;
        if (!win32_write_all(&g_win_port, ratepPkt, sizeof(ratepPkt)))
            return false;
        int rlen = win32_read_packet(&g_win_port, resp, sizeof(resp), AMBE_TYPE_CONTROL, 100);
#else
        if (g_serial_fd < 0)
            return false;
        if (!posix_write_all(g_serial_fd, ratepPkt, sizeof(ratepPkt)))
            return false;
        int rlen = posix_read_packet(g_serial_fd, resp, sizeof(resp), AMBE_TYPE_CONTROL, 100);
#endif
        if (rlen >= 6 && resp[3] == AMBE_TYPE_CONTROL &&
            resp[4] == (unsigned char)(AMBE3003_PKT_CHANNEL0 + chIdx) &&
            resp[5] == AMBE_CONTROL_RATEP)
        {
            g_channel_rate[chIdx] = targetRate;
            return true;
        }
        purge_rx_buffer();
        return false;
    }
    else
    {
#if defined(_WIN32) || defined(__CYGWIN__)
        if (g_win_port.handle == INVALID_HANDLE_VALUE)
            return false;
        if (!win32_write_all(&g_win_port, baseRatePkt, 17))
            return false;
        int rlen = win32_read_packet(&g_win_port, resp, sizeof(resp), AMBE_TYPE_CONTROL, 100);
#else
        if (g_serial_fd < 0)
            return false;
        if (!posix_write_all(g_serial_fd, baseRatePkt, 17))
            return false;
        int rlen = posix_read_packet(g_serial_fd, resp, sizeof(resp), AMBE_TYPE_CONTROL, 100);
#endif
        if (rlen >= 5 && resp[3] == AMBE_TYPE_CONTROL && resp[4] == AMBE_CONTROL_RATEP)
        {
            g_channel_rate[0] = targetRate;
            return true;
        }
        purge_rx_buffer();
        return false;
    }
}

static bool decode_ambe_frame(short *audioFrame, const unsigned char *mbeFrame,
                              unsigned char numBits, unsigned char numBytes, int slot)
{
    unsigned char pkt[20];
    unsigned int totalPktLen = 0;
    unsigned char pktChan = (slot == 1) ? AMBE3003_PKT_CHANNEL1 : AMBE3003_PKT_CHANNEL0;

    if (g_ambe_type == 3003)
    {
        totalPktLen = 7U + (unsigned int)numBytes;
        pkt[0] = AMBE_START_BYTE;                // 0x61
        pkt[1] = 0x00U;
        pkt[2] = (unsigned char)(3U + numBytes); // 10 (0x0A) for 2450, 12 (0x0C) for 3600x2400
        pkt[3] = AMBE_TYPE_AMBE;                 // 0x01
        pkt[4] = pktChan;                        // 0x40 (Timeslot 1) or 0x41 (Timeslot 2)
        pkt[5] = 0x01U;                          // CHAND
        pkt[6] = numBits;                        // 49 (0x31) or 72 (0x48)
        memcpy(pkt + 7, mbeFrame, numBytes);
    }
    else
    {
        totalPktLen = 6U + (unsigned int)numBytes;
        pkt[0] = AMBE_START_BYTE;                // 0x61
        pkt[1] = 0x00U;
        pkt[2] = (unsigned char)(2U + numBytes); // 9 (0x09) for 2450, 11 (0x0B) for 3600x2400
        pkt[3] = AMBE_TYPE_AMBE;                 // 0x01
        pkt[4] = 0x01U;                          // CHAND
        pkt[5] = numBits;                        // 49 (0x31) or 72 (0x48)
        memcpy(pkt + 6, mbeFrame, numBytes);
    }

    unsigned char resp[512];
    int rlen = -1;

#if defined(_WIN32) || defined(__CYGWIN__)
    if (g_win_port.handle == INVALID_HANDLE_VALUE)
        return false;
    if (!win32_write_all(&g_win_port, pkt, (DWORD)totalPktLen))
        return false;
    rlen = win32_read_packet(&g_win_port, resp, sizeof(resp), AMBE_TYPE_AUDIO, 200);
#else
    if (g_serial_fd < 0)
        return false;
    if (!posix_write_all(g_serial_fd, pkt, totalPktLen))
        return false;
    rlen = posix_read_packet(g_serial_fd, resp, sizeof(resp), AMBE_TYPE_AUDIO, 200);
#endif

    int pcmOffset = 0;
    if (g_ambe_type == 3003)
    {
        if (rlen != 327 || resp[3] != AMBE_TYPE_AUDIO || resp[4] != pktChan || resp[5] != 0x00U || resp[6] != 0xA0U)
        {
            purge_rx_buffer();
            return false;
        }
        pcmOffset = 7;
    }
    else
    {
        if (rlen != 326 || resp[3] != AMBE_TYPE_AUDIO || resp[4] != 0x00U || resp[5] != 0xA0U)
        {
            purge_rx_buffer();
            return false;
        }
        pcmOffset = 6;
    }

    const uint8_t *q = resp + pcmOffset;
    for (int i = 0; i < 160; i++, q += 2)
    {
        audioFrame[i] = (short)((q[0] << 8) | q[1]);
    }
    return true;
}

static bool is_disabled_port_str(const char *s)
{
    if (s == NULL || s[0] == '\0')
        return false;
    return (strcmp(s, "0") == 0 ||
            strcmp(s, "none") == 0 || strcmp(s, "NONE") == 0 ||
            strcmp(s, "off") == 0 || strcmp(s, "OFF") == 0 ||
            strcmp(s, "false") == 0 || strcmp(s, "FALSE") == 0);
}

static bool is_scanner_serial_port(const char *candidate)
{
    if (g_opts == NULL || g_opts->resume <= 0 || g_opts->serial_dev[0] == '\0' || candidate == NULL)
        return false;
    return (strstr(g_opts->serial_dev, candidate) != NULL || strstr(candidate, g_opts->serial_dev) != NULL);
}

static int ambe_hw_init_internal(const void *opts, const void *state, bool manualToggle)
{
    ambe_hw_close();
    g_opts = (const dsd_opts *)opts;
    g_state = (dsd_state *)state;
    g_slot2_mp = (g_state != NULL) ? g_state->cur_mp2 : NULL;

    const char *envVars[] = {"AMBE_PORT", "AMBE3000_PORT", "AMBE3003_PORT", "DVSERIAL_PORT"};
    const char *envPort = NULL;
    for (size_t e = 0; e < sizeof(envVars) / sizeof(envVars[0]); e++)
    {
        const char *val = getenv(envVars[e]);
        if (val && val[0])
        {
            envPort = val;
            break;
        }
    }

    if (is_disabled_port_str(envPort))
    {
        if (!manualToggle)
        {
            fprintf(stderr, "AMBE hardware disabled via environment, using MBElib\n");
            fflush(stderr);
            return 0;
        }
        envPort = NULL;
    }

#if defined(_WIN32) || defined(__CYGWIN__)
    if (envPort && envPort[0])
    {
        char devPath[64];
        if (strncmp(envPort, "\\\\.\\", 4) == 0)
            snprintf(devPath, sizeof(devPath), "%s", envPort);
        else
            snprintf(devPath, sizeof(devPath), "\\\\.\\%s", envPort);

        if (probe_win32_port(devPath, envPort))
            return 1;
    }

    if (g_last_dev_path[0] != '\0' && probe_win32_port(g_last_dev_path, g_last_port_label))
        return 1;

    for (int i = 1; i <= 64; i++)
    {
        char devPath[32];
        char displayPort[16];
        char ntTarget[256] = {0};
        snprintf(displayPort, sizeof(displayPort), "COM%d", i);
        if (is_scanner_serial_port(displayPort))
            continue;
        // Skip non-existent COM ports and Bluetooth virtual RFCOMM ports that stall on open
        if (::QueryDosDeviceA(displayPort, ntTarget, sizeof(ntTarget)) == 0)
            continue;
        if (strstr(ntTarget, "BthModem") != NULL || strstr(ntTarget, "BTHENUM") != NULL)
            continue;

        snprintf(devPath, sizeof(devPath), "\\\\.\\COM%d", i);
        if (strcmp(devPath, g_last_dev_path) == 0)
            continue;
        if (probe_win32_port(devPath, displayPort))
            return 1;
    }
#else
    if (envPort && envPort[0])
    {
        if (probe_posix_port(envPort))
            return 1;
    }

    if (g_last_dev_path[0] != '\0' && probe_posix_port(g_last_dev_path))
        return 1;

    for (int i = 0; i < 16; i++)
    {
        char devPath[32];
        snprintf(devPath, sizeof(devPath), "/dev/ttyUSB%d", i);
        if (is_scanner_serial_port(devPath) || strcmp(devPath, g_last_dev_path) == 0)
            continue;
        if (probe_posix_port(devPath))
            return 1;
    }

    for (int i = 0; i < 16; i++)
    {
        char devPath[32];
        snprintf(devPath, sizeof(devPath), "/dev/ttyACM%d", i);
        if (is_scanner_serial_port(devPath) || strcmp(devPath, g_last_dev_path) == 0)
            continue;
        if (probe_posix_port(devPath))
            return 1;
    }
#endif

    fprintf(stderr, "AMBE3000/AMBE3003 not detected, falling back to MBElib\n");
    fflush(stderr);
    return 0;
}

extern "C" int ambe_hw_init(const void *opts, const void *state)
{
    int rc = ambe_hw_init_internal(opts, state, false);
    if (rc && g_opts != NULL && g_state != NULL && g_opts->audio_gain == 0)
    {
        ((dsd_state *)g_state)->aout_gain  = 25.0f;
        ((dsd_state *)g_state)->aout_gainR = 25.0f;
    }
    return rc;
}

extern "C" void ambe_hw_release_lock(void)
{
    if (g_state != NULL)
    {
        g_state->dmr_fid = 0;
        g_state->dmr_fidR = 0;
    }
    g_ambe3000_locked_slot = -1;
    g_ambe3000_locked_last_ms = 0;
    g_ambe3000_muted_last_ms[0] = 0;
    g_ambe3000_muted_last_ms[1] = 0;
    g_frame_muted[0] = 0;
    g_frame_muted[1] = 0;
    g_frame_hw_decoded[0] = 0;
    g_frame_hw_decoded[1] = 0;
    g_captured_fr_valid = false;
    ambe_hw_reset_call(0);
    ambe_hw_reset_call(1);
}

extern "C" void ambe_hw_reset_call(int slot)
{
    if (slot < 0 || slot > 1)
        return;
    g_channel_needs_init[slot] = true;
    g_frame_muted[slot] = 0;
    g_frame_hw_decoded[slot] = 0;
    memset(g_ber_err_ring[slot], 0, sizeof(g_ber_err_ring[slot]));
    memset(g_ber_tot_ring[slot], 0, sizeof(g_ber_tot_ring[slot]));
    g_ber_ring_idx[slot] = 0;
    g_ber_ring_cnt[slot] = 0;
    g_call_last_frame_ms[slot] = 0;
    if (g_state != NULL)
    {
        dsd_state *st = (dsd_state *)g_state;
        if (slot == 0)
        {
            st->errs = 0;
            st->errs2 = 0;
            st->dmr_so = 0;
            st->payload_algid = 0;
            st->payload_mi = 0;
            st->payload_miP = 0;
            st->payload_keyid = 0;
            st->dropL = 256;
            st->DMRvcL = 0;
        }
        else
        {
            st->errsR = 0;
            st->errs2R = 0;
            st->dmr_soR = 0;
            st->payload_algidR = 0;
            st->payload_miR = 0;
            st->payload_miN = 0;
            st->payload_keyidR = 0;
            st->dropR = 256;
            st->DMRvcR = 0;
        }
    }
}

extern "C" void ambe_hw_record_ber(int slot, int err_bits, int total_bits)
{
    if (slot < 0 || slot > 1 || total_bits <= 0)
        return;
    uint64_t now_ms = get_monotonic_ms();
    // Reset the rolling window if > 600 ms elapsed since the last voice frame on this slot
    if (g_call_last_frame_ms[slot] != 0 && (now_ms - g_call_last_frame_ms[slot] > 600ULL))
    {
        memset(g_ber_err_ring[slot], 0, sizeof(g_ber_err_ring[slot]));
        memset(g_ber_tot_ring[slot], 0, sizeof(g_ber_tot_ring[slot]));
        g_ber_ring_idx[slot] = 0;
        g_ber_ring_cnt[slot] = 0;
    }
    g_call_last_frame_ms[slot] = now_ms;
    if (err_bits < 0)
        err_bits = 0;
    if (err_bits > total_bits)
        err_bits = total_bits;

    int idx = g_ber_ring_idx[slot];
    g_ber_err_ring[slot][idx] = (uint8_t)err_bits;
    g_ber_tot_ring[slot][idx] = (uint8_t)total_bits;
    g_ber_ring_idx[slot] = (idx + 1) % BER_WINDOW_FRAMES;
    if (g_ber_ring_cnt[slot] < BER_WINDOW_FRAMES)
        g_ber_ring_cnt[slot]++;
}

extern "C" float ambe_hw_get_slot_ber(int slot)
{
    if (slot < 0 || slot > 1 || g_ber_ring_cnt[slot] == 0)
        return 0.0f;
    uint64_t now_ms = get_monotonic_ms();
    if (g_call_last_frame_ms[slot] == 0 || (now_ms - g_call_last_frame_ms[slot] > 400ULL))
        return 0.0f;
    if (g_opts != NULL && g_state != NULL && g_opts->dmr_stereo == 1)
    {
        uint8_t burst = (slot == 0) ? g_state->dmrburstL : g_state->dmrburstR;
        if (burst != 16 && burst != 21)
            return 0.0f;
    }
    int err_sum = 0;
    int tot_sum = 0;
    for (int i = 0; i < g_ber_ring_cnt[slot]; i++)
    {
        err_sum += g_ber_err_ring[slot][i];
        tot_sum += g_ber_tot_ring[slot][i];
    }
    if (tot_sum <= 0)
        return 0.0f;
    float ber = (100.0f * (float)err_sum) / (float)tot_sum;
    if (ber > 99.9f)
        ber = 99.9f;
    return ber;
}

extern "C" void ambe_hw_reset_slot(int slot)
{
    if (slot < 0 || slot > 1)
        return;
    g_channel_needs_init[slot] = true;
    g_frame_muted[slot] = 0;
    g_frame_hw_decoded[slot] = 0;
    if (g_state != NULL)
    {
        if (slot == 0 && g_state->dmrburstL == 9)
        {
            g_state->dmr_so = 0;
            g_state->dmr_fid = 0;
            g_state->payload_algid = 0;
            g_state->payload_mi = 0;
            g_state->payload_miP = 0;
            g_state->payload_keyid = 0;
            g_state->dropL = 256;
            g_state->DMRvcL = 0;
        }
        else if (slot == 1 && g_state->dmrburstR == 9)
        {
            g_state->dmr_soR = 0;
            g_state->dmr_fidR = 0;
            g_state->payload_algidR = 0;
            g_state->payload_miR = 0;
            g_state->payload_miN = 0;
            g_state->payload_keyidR = 0;
            g_state->dropR = 256;
            g_state->DMRvcR = 0;
        }
    }
}

extern "C" int ambe_hw_toggle(const void *opts, const void *state)
{
    char timestr[16];
    get_time_str(timestr, sizeof(timestr));
    g_auto_reconnect = false;
    g_next_reconnect_ms = 0;
    if (g_ambe_active)
    {
        fprintf(stderr, "\n%s %s disabled on %s, switching to MBElib\n",
                timestr, (g_ambe_type == 3000) ? "AMBE3000" : "AMBE3003", g_port_label);
        fflush(stderr);
        ambe_hw_close();
        if (g_opts != NULL && g_state != NULL && g_opts->audio_gain == 0)
        {
            ((dsd_state *)g_state)->aout_gain  = 15.0f;
            ((dsd_state *)g_state)->aout_gainR = 15.0f;
        }
        return 0;
    }
    fprintf(stderr, "\n%s Probing for AMBE3000/AMBE3003...\n", timestr);
    fflush(stderr);
    int rc = ambe_hw_init_internal(opts, state, true);
    if (rc && g_opts != NULL && g_state != NULL && g_opts->audio_gain == 0)
    {
        ((dsd_state *)g_state)->aout_gain  = 25.0f;
        ((dsd_state *)g_state)->aout_gainR = 25.0f;
    }
    return rc;
}

extern "C" void ambe_hw_close(void)
{
#if defined(_WIN32) || defined(__CYGWIN__)
    if (g_ambe_active && !g_io_error && g_win_port.handle != INVALID_HANDLE_VALUE)
    {
        unsigned char flushZeros[360];
        memset(flushZeros, 0, sizeof(flushZeros));
        win32_write_all(&g_win_port, flushZeros, sizeof(flushZeros));
        win32_write_all(&g_win_port, PKT_SOFT_RESET, sizeof(PKT_SOFT_RESET));
    }
    close_win32_port(&g_win_port);
#else
    if (g_serial_fd >= 0)
    {
        if (g_ambe_active && !g_io_error)
        {
            unsigned char flushZeros[360];
            memset(flushZeros, 0, sizeof(flushZeros));
            posix_write_all(g_serial_fd, flushZeros, sizeof(flushZeros));
            posix_write_all(g_serial_fd, PKT_SOFT_RESET, sizeof(PKT_SOFT_RESET));
        }
        ::close(g_serial_fd);
        g_serial_fd = -1;
    }
#endif
    g_ambe_active = 0;
    g_ambe_type = 0;
    g_channel_rate[0] = AMBE_RATE_NONE;
    g_channel_rate[1] = AMBE_RATE_NONE;
    g_ambe3000_locked_slot = -1;
    g_ambe3000_last_decoded_slot = -1;
    g_channel_needs_init[0] = false;
    g_channel_needs_init[1] = false;
    g_ambe3000_locked_last_ms = 0;
    g_ambe3000_muted_last_ms[0] = 0;
    g_ambe3000_muted_last_ms[1] = 0;
    g_frame_muted[0] = 0;
    g_frame_muted[1] = 0;
    g_frame_hw_decoded[0] = 0;
    g_frame_hw_decoded[1] = 0;
    g_captured_fr_valid = false;
    g_consecutive_errors = 0;
    g_io_error = false;
    g_port_label[0] = '\0';
}

static void maybe_auto_reconnect(void);

extern "C" int ambe_hw_is_active(void)
{
    return g_ambe_active;
}

extern "C" const char *ambe_hw_status_str(void)
{
    if (!g_ambe_active)
    {
        if (g_auto_reconnect)
            maybe_auto_reconnect();
        if (!g_ambe_active)
            return "MBElib";
    }

    // Modes that exclusively use IMBE 7200x4400 / 7100x4400 or unencoded .dmb playback are decoded in software via MBElib
    if (g_opts != NULL)
    {
        if (g_opts->playfiles == 1 && g_state != NULL &&
            (g_state->mbe_file_type == 0 || g_state->mbe_file_type == 2))
        {
            return "MBElib";
        }
        if (g_opts->frame_provoice == 1)
            return "MBElib";
        if (g_opts->frame_p25p1 == 1 &&
            g_opts->frame_p25p2 == 0 && g_opts->frame_dmr == 0 &&
            g_opts->frame_dstar == 0 && g_opts->frame_x2tdma == 0 &&
            g_opts->frame_nxdn48 == 0 && g_opts->frame_nxdn96 == 0 &&
            g_opts->frame_dpmr == 0 && g_opts->frame_ysf == 0)
        {
            return "MBElib";
        }
    }

    if (g_state != NULL && g_state->carrier == 1)
    {
        int st = g_state->synctype;
        // EDACS / ProVoice
        if (st == 14 || st == 15 || st == 37 || st == 38)
            return "MBElib";
        // Active P25 Phase 1 FDMA voice (HDU / LDU1 / LDU2); keep hardware status shown on TSBK CC
        if ((st == 0 || st == 1) &&
            (g_state->dmrburstL == 25 || g_state->dmrburstL == 26 || g_state->dmrburstL == 27))
        {
            return "MBElib";
        }
        // YSF Voice Wide Full-Rate (VWFR)
        if ((st == 30 || st == 31) && g_state->ysf_dt == 3)
            return "MBElib";
    }

    return (g_ambe_type == 3000) ? "AMBE3000" : "AMBE3003";
}

static bool is_slot_playable(int slot)
{
    if (g_opts == NULL || g_state == NULL)
        return true;

    // Skip hardware I/O when audio output is muted (-o null) and no WAV file is being written
    if (g_opts->audio_out == 0 && g_opts->wav_out_f == NULL && g_opts->wav_out_fR == NULL)
        return false;

    int st = g_state->synctype;
    int tg = (slot == 0) ? g_state->lasttg : g_state->lasttgR;
    if (slot == 0 && (st == 28 || st == 29))
        tg = g_state->nxdn_last_tg;

    bool isHeldTg = (g_state->tg_hold != 0 && tg != 0 && (uint32_t)tg == g_state->tg_hold);

    // Respect TG Hold if set (an explicit TG hold overrides slot off and group block/allow rules)
    if (g_state->tg_hold != 0 && !isHeldTg)
        return false;

    if (!isHeldTg)
    {
        // Respect user slot on/off toggles
        if (slot == 0 && g_opts->slot1_on == 0)
            return false;
        if (slot == 1 && g_opts->slot2_on == 0)
            return false;

        // Respect group block/allow list
        char mode[8] = "";
        if (g_opts->trunk_use_allow_list == 1)
            snprintf(mode, sizeof(mode), "B");
        for (unsigned int i = 0; i < g_state->group_tally; i++)
        {
            if (g_state->group_array[i].groupNumber == (unsigned long)tg)
            {
                snprintf(mode, sizeof(mode), "%s", g_state->group_array[i].groupMode);
                break;
            }
        }
        if (strcmp(mode, "B") == 0)
            return false;
    }

    // Check encryption and scrambling status matching dsd_mbe.c / dsd_audio2.c
    uint8_t alg = (slot == 0) ? g_state->payload_algid : g_state->payload_algidR;
    unsigned long long int rkey = (slot == 0) ? g_state->R : g_state->RR;
    int aesLoaded = g_state->aes_key_loaded[slot];
    bool isEncrypted = false;
    bool hasKey = false;

    if (st == 35 || st == 36) // P25 Phase 2
    {
        // Mute if Phase 2 scrambling parameters (WACN/SYSID/CC) are missing
        if (g_state->p2_wacn == 0 || g_state->p2_sysid == 0 || g_state->p2_cc == 0)
            return false;
        if (alg != 0 && alg != 0x80)
        {
            isEncrypted = true;
            hasKey = ((alg == 0xAA && rkey != 0) ||
                      (alg == 0x81 && g_state->R != 0) ||
                      ((alg == 0x84 || alg == 0x89) && aesLoaded == 1));
            if (g_opts->reverse_mute == 0 && !hasKey && g_opts->unmute_encrypted_p25 == 0)
                return false;
        }
    }
    else if (st == 28 || st == 29) // NXDN
    {
        if (g_state->nxdn_cipher_type != 0)
        {
            isEncrypted = true;
            hasKey = (((g_state->nxdn_cipher_type == 0x01 || g_state->nxdn_cipher_type == 0x02) && g_state->R != 0) ||
                      (g_state->nxdn_cipher_type == 0x03 && g_state->aes_key_loaded[0] == 1));
            if (g_opts->reverse_mute == 0 && !hasKey)
                return false;
        }
    }
    else if (g_opts->dmr_stereo == 1 || g_opts->dmr_mono == 1) // DMR
    {
        if (slot == 0 && g_state->dmr_so == 0x100)
        {
            g_state->dmr_so = 0;
            g_state->payload_algid = 0;
            g_state->payload_keyid = 0;
            g_state->payload_mi = 0;
        }
        else if (slot == 1 && g_state->dmr_soR == 0x100)
        {
            g_state->dmr_soR = 0;
            g_state->payload_algidR = 0;
            g_state->payload_keyidR = 0;
            g_state->payload_miR = 0;
        }
        alg = (slot == 0) ? g_state->payload_algid : g_state->payload_algidR;
        uint16_t so = (slot == 0) ? g_state->dmr_so : g_state->dmr_soR;
        uint8_t muteEnc = (slot == 0) ? g_opts->dmr_mute_encL : g_opts->dmr_mute_encR;
        unsigned long long int mi = (slot == 0) ? g_state->payload_mi : g_state->payload_miR;
        unsigned long long int miDes = (slot == 0) ? g_state->payload_miP : g_state->payload_miN;

        bool hasStaticKey = (g_state->K != 0 || g_state->K1 != 0 || g_state->vtx_key_loaded == 1 ||
                             g_state->ken_sc == 1 || g_state->any_bp == 1 || g_state->straight_ks == 1 ||
                             g_state->tyt_ap == 1 || g_state->tyt_bp == 1 || g_state->tyt_ep == 1 ||
                             g_state->retevis_ap == 1 || g_state->baofeng_ap == 1 ||
                             g_state->csi_ee == 1 || g_state->rc2_context != NULL ||
                             (g_state->straight_ks == 0x26 && g_state->forced_alg_id == 0x26));
        bool hasDynamicCfg = (rkey != 0 || aesLoaded == 1 ||
                              (g_state->forced_alg_id > 1 && g_state->forced_alg_id != 0x16) ||
                              g_state->keyloader == 1);

        isEncrypted = (((so >> 6) & 1) != 0) || (alg != 0 && alg != 0x80);
        // Late Entry before VLC/PI/EMB LC: mute until link control or MI locks when dynamic cipher is configured
        if (!isEncrypted && (so & 0x8000) == 0 && alg == 0 && mi == 0 &&
            hasDynamicCfg && !hasStaticKey)
        {
            isEncrypted = true;
        }


        if (isEncrypted)
        {
            if (alg == 0)
            {
                hasKey = hasStaticKey;
            }
            else if (alg == 0x02 || alg == 0x21)
            {
                hasKey = (rkey != 0 && mi != 0);
            }
            else if (alg == 0x22)
            {
                hasKey = (rkey != 0 && mi != 0 && miDes != 0);
            }
            else if (alg == 0x24 || alg == 0x25 || (alg >= 0x31 && alg <= 0x3A))
            {
                hasKey = (aesLoaded == 1 && mi != 0);
            }
            else if (alg == 0x26)
            {
                hasKey = (g_state->straight_ks == 0x26);
            }
            else
            {
                hasKey = hasStaticKey;
            }
            bool waitingForMi = false;
            if (alg == 0 && hasDynamicCfg && !hasStaticKey)
                waitingForMi = true;
            else if ((alg == 0x02 || alg == 0x21) && (rkey != 0 || g_state->keyloader == 1) && mi == 0)
                waitingForMi = true;
            else if (alg == 0x22 && (rkey != 0 || g_state->keyloader == 1) && (mi == 0 || miDes == 0))
                waitingForMi = true;
            else if ((alg == 0x24 || alg == 0x25 || (alg >= 0x31 && alg <= 0x3A)) &&
                     (aesLoaded == 1 || g_state->keyloader == 1) && mi == 0)
                waitingForMi = true;

            if (g_opts->reverse_mute == 0 && !hasKey && (muteEnc == 1 || waitingForMi))
                return false;
        }
    }

    if (g_opts->reverse_mute == 1 && (!isEncrypted || hasKey))
        return false;

    return true;
}

static const uint64_t AMBE3000_HANGTIME_MS = 3000ULL;

static bool is_locked_slot_still_allowed(int lockedSlot)
{
    if (g_opts == NULL)
        return true;

    int tg = (g_state != NULL) ? ((lockedSlot == 0) ? g_state->lasttg : g_state->lasttgR) : 0;
    bool isHeldTg = (g_state != NULL && g_state->tg_hold != 0 && tg != 0 && (uint32_t)tg == g_state->tg_hold);

    // Release hang-time lock immediately if user placed a TG Hold that does not match this slot
    if (g_state != NULL && g_state->tg_hold != 0 && !isHeldTg)
        return false;

    if (!isHeldTg)
    {
        if (lockedSlot == 0 && g_opts->slot1_on == 0)
            return false;
        if (lockedSlot == 1 && g_opts->slot2_on == 0)
            return false;
        if (g_state != NULL && tg != 0)
        {
            char mode[8] = "";
            if (g_opts->trunk_use_allow_list == 1)
                snprintf(mode, sizeof(mode), "B");
            for (unsigned int i = 0; i < g_state->group_tally; i++)
            {
                if (g_state->group_array[i].groupNumber == (unsigned long)tg)
                {
                    snprintf(mode, sizeof(mode), "%s", g_state->group_array[i].groupMode);
                    break;
                }
            }
            if (strcmp(mode, "B") == 0)
                return false;
        }
    }
    return true;
}

static bool is_slot_voice_active(int slot)
{
    if (g_opts == NULL || g_state == NULL)
        return true;
    if (g_state->carrier == 0)
        return false;
    if (g_opts->dmr_stereo == 1)
    {
        uint8_t burst = (slot == 0) ? g_state->dmrburstL : g_state->dmrburstR;
        if (burst != 16 && burst != 21)
            return false;
    }
    return is_slot_playable(slot);
}

extern "C" int ambe_hw_single_channel_slot(void)
{
    if (!g_ambe_active || g_ambe_type != 3000 || g_ambe3000_locked_slot < 0)
        return -1;
    uint64_t now_ms = get_monotonic_ms();
    if (!is_locked_slot_still_allowed(g_ambe3000_locked_slot) ||
        (now_ms - g_ambe3000_locked_last_ms > AMBE3000_HANGTIME_MS))
    {
        g_ambe3000_locked_slot = -1;
        return -1;
    }
    return g_ambe3000_locked_slot;
}

extern "C" int ambe_hw_is_slot_muted(int slot)
{
    if (!g_ambe_active || g_ambe_type != 3000)
        return 0;
    if (slot < 0 || slot > 1)
        return 0;
    if (g_ambe3000_locked_slot != (1 - slot))
        return 0;
    uint64_t now_ms = get_monotonic_ms();
    if (!is_locked_slot_still_allowed(g_ambe3000_locked_slot) ||
        (now_ms - g_ambe3000_locked_last_ms > AMBE3000_HANGTIME_MS))
    {
        return 0;
    }
    if (!is_slot_voice_active(slot) ||
        (now_ms - g_ambe3000_muted_last_ms[slot] > 150ULL))
    {
        return 0;
    }
    return 1;
}

extern "C" const char *ambe_hw_slot_status_str(int slot)
{
    if (slot < 0 || slot > 1 || g_opts == NULL || g_state == NULL)
        return "";

    uint64_t now_ms = get_monotonic_ms();
    bool inVoice = false;
    if (g_state->carrier == 1)
    {
        if (g_opts->dmr_stereo == 1)
        {
            uint8_t burst = (slot == 0) ? g_state->dmrburstL : g_state->dmrburstR;
            inVoice = (burst == 16 || burst == 21 || (slot == 0 && (burst == 26 || burst == 27)));
        }
        else if (slot == 0)
        {
            inVoice = (g_call_last_frame_ms[0] != 0 && (now_ms - g_call_last_frame_ms[0] <= 250ULL)) ||
                      (g_state->dmrburstL == 26 || g_state->dmrburstL == 27);
        }
    }

    if (inVoice)
    {
        int st = g_state->synctype;
        int tg = (slot == 0) ? g_state->lasttg : g_state->lasttgR;
        if (slot == 0 && (st == 28 || st == 29))
            tg = g_state->nxdn_last_tg;

        bool isHeldTg = (g_state->tg_hold != 0 && tg != 0 && (uint32_t)tg == g_state->tg_hold);
        if (g_state->tg_hold != 0 && !isHeldTg)
            return "*Muted (TG Hold)";

        if (!isHeldTg)
        {
            if ((slot == 0 && g_opts->slot1_on == 0) || (slot == 1 && g_opts->slot2_on == 0))
                return "";

            char mode[8] = "";
            if (g_opts->trunk_use_allow_list == 1)
                snprintf(mode, sizeof(mode), "B");
            for (unsigned int i = 0; i < g_state->group_tally; i++)
            {
                if (g_state->group_array[i].groupNumber == (unsigned long)tg)
                {
                    snprintf(mode, sizeof(mode), "%s", g_state->group_array[i].groupMode);
                    break;
                }
            }
            if (strcmp(mode, "B") == 0)
                return "*Muted (TG)";
        }

        uint8_t alg = (slot == 0) ? g_state->payload_algid : g_state->payload_algidR;
        unsigned long long int rkey = (slot == 0) ? g_state->R : g_state->RR;
        int aesLoaded = g_state->aes_key_loaded[slot];
        bool isEncrypted = false;
        bool hasKey = false;

        if (st == 35 || st == 36) // P25 Phase 2
        {
            if (g_state->p2_wacn == 0 || g_state->p2_sysid == 0 || g_state->p2_cc == 0)
                return "*Muted (No Parms)";
            if (alg != 0 && alg != 0x80)
            {
                isEncrypted = true;
                hasKey = ((alg == 0xAA && rkey != 0) ||
                          (alg == 0x81 && g_state->R != 0) ||
                          ((alg == 0x84 || alg == 0x89) && aesLoaded == 1));
                if (g_opts->reverse_mute == 0 && !hasKey && g_opts->unmute_encrypted_p25 == 0)
                    return "*Muted (ENC)";
            }
        }
        else if (st == 0 || st == 1) // P25 Phase 1
        {
            if (alg != 0 && alg != 0x80)
            {
                isEncrypted = true;
                hasKey = (((alg == 0xAA || alg == 0x81 || alg == 0x9F || alg == 0x83) && g_state->R != 0) ||
                          ((alg == 0x84 || alg == 0x89 || alg == 0x83) && aesLoaded == 1));
                if (g_opts->reverse_mute == 0 && !hasKey && g_opts->unmute_encrypted_p25 == 0)
                    return "*Muted (ENC)";
            }
        }
        else if (st == 28 || st == 29) // NXDN
        {
            if (g_state->nxdn_cipher_type != 0)
            {
                isEncrypted = true;
                hasKey = (((g_state->nxdn_cipher_type == 0x01 || g_state->nxdn_cipher_type == 0x02) && g_state->R != 0) ||
                          (g_state->nxdn_cipher_type == 0x03 && g_state->aes_key_loaded[0] == 1));
                if (g_opts->reverse_mute == 0 && !hasKey)
                    return "*Muted (ENC)";
            }
        }
        else if (g_opts->dmr_stereo == 1 || g_opts->dmr_mono == 1) // DMR
        {
            uint16_t so = (slot == 0) ? g_state->dmr_so : g_state->dmr_soR;
            if (so == 0x100)
            {
                so = 0;
                alg = 0;
            }
            uint8_t muteEnc = (slot == 0) ? g_opts->dmr_mute_encL : g_opts->dmr_mute_encR;
            unsigned long long int mi = (slot == 0) ? g_state->payload_mi : g_state->payload_miR;
            unsigned long long int miDes = (slot == 0) ? g_state->payload_miP : g_state->payload_miN;

            bool hasStaticKey = (g_state->K != 0 || g_state->K1 != 0 || g_state->vtx_key_loaded == 1 ||
                                 g_state->ken_sc == 1 || g_state->any_bp == 1 || g_state->straight_ks == 1 ||
                                 g_state->tyt_ap == 1 || g_state->tyt_bp == 1 || g_state->tyt_ep == 1 ||
                                 g_state->retevis_ap == 1 || g_state->baofeng_ap == 1 ||
                                 g_state->csi_ee == 1 || g_state->rc2_context != NULL ||
                                 (g_state->straight_ks == 0x26 && g_state->forced_alg_id == 0x26));
            bool hasDynamicCfg = (rkey != 0 || aesLoaded == 1 ||
                                  (g_state->forced_alg_id > 1 && g_state->forced_alg_id != 0x16) ||
                                  g_state->keyloader == 1);

            isEncrypted = (((so >> 6) & 1) != 0) || (alg != 0 && alg != 0x80);
            if (!isEncrypted && (so & 0x8000) == 0 && alg == 0 && mi == 0 &&
                hasDynamicCfg && !hasStaticKey)
            {
                isEncrypted = true;
            }


            if (isEncrypted)
            {
                if (alg == 0)
                    hasKey = hasStaticKey;
                else if (alg == 0x02 || alg == 0x21)
                    hasKey = (rkey != 0 && mi != 0);
                else if (alg == 0x22)
                    hasKey = (rkey != 0 && mi != 0 && miDes != 0);
                else if (alg == 0x24 || alg == 0x25 || (alg >= 0x31 && alg <= 0x3A))
                    hasKey = (aesLoaded == 1 && mi != 0);
                else if (alg == 0x26)
                    hasKey = (g_state->straight_ks == 0x26);
                else
                    hasKey = hasStaticKey;

                bool waitingForMi = false;
                if (alg == 0 && hasDynamicCfg && !hasStaticKey)
                    waitingForMi = true;
                else if ((alg == 0x02 || alg == 0x21) && (rkey != 0 || g_state->keyloader == 1) && mi == 0)
                    waitingForMi = true;
                else if (alg == 0x22 && (rkey != 0 || g_state->keyloader == 1) && (mi == 0 || miDes == 0))
                    waitingForMi = true;
                else if ((alg == 0x24 || alg == 0x25 || (alg >= 0x31 && alg <= 0x3A)) &&
                         (aesLoaded == 1 || g_state->keyloader == 1) && mi == 0)
                    waitingForMi = true;

                if (g_opts->reverse_mute == 0 && !hasKey && (muteEnc == 1 || waitingForMi))
                {
                    return waitingForMi ? "*Muted (No MI)" : "*Muted (ENC)";
                }
            }
        }

        if (g_opts->reverse_mute == 1 && (!isEncrypted || hasKey))
            return "*Muted (Rev)";

        if (ambe_hw_is_slot_muted(slot))
            return "*Muted (Hold)";
    }

    return "";
}

extern "C" int ambe_hw_was_frame_muted(int slot)
{
    if (slot < 0 || slot > 1)
        return 0;
    if (g_state != NULL)
    {
        int st = g_state->synctype;
        // IMBE modes (P25p1, ProVoice, YSF VWFR) and .imb/.dmb playback never go through AMBE hardware
        if (st == 0 || st == 1 || st == 14 || st == 15 || st == 37 || st == 38 ||
            ((st == 30 || st == 31) && g_state->ysf_dt == 3) ||
            (g_opts != NULL && g_opts->playfiles == 1 &&
             (g_state->mbe_file_type == 0 || g_state->mbe_file_type == 2)))
        {
            return 0;
        }
    }
    return g_frame_muted[slot];
}

extern "C" int ambe_hw_is_slot_hw_decoded(int slot)
{
    if (!g_ambe_active || slot < 0 || slot > 1)
        return 0;
    if (g_state != NULL)
    {
        int st = g_state->synctype;
        // IMBE modes (P25p1, ProVoice, YSF VWFR) and .imb/.dmb playback never go through AMBE hardware
        if (st == 0 || st == 1 || st == 14 || st == 15 || st == 37 || st == 38 ||
            ((st == 30 || st == 31) && g_state->ysf_dt == 3) ||
            (g_opts != NULL && g_opts->playfiles == 1 &&
             (g_state->mbe_file_type == 0 || g_state->mbe_file_type == 2)))
        {
            return 0;
        }
    }
    return g_frame_hw_decoded[slot];
}

static void fill_err_str(char *err_str, const int *errs2)
{
    if (err_str != NULL)
    {
        int n = (errs2 != NULL) ? *errs2 : 0;
        if (n < 0) n = 0;
        if (n > 63) n = 63;
        for (int i = 0; i < n; i++)
            err_str[i] = '=';
        err_str[n] = '\0';
    }
}

static void finish_hardware_frame(int slot, float *aout_buf, const short audioFrame[160], const int *errs2, char *err_str)
{
    g_consecutive_errors = 0;
    if (slot >= 0 && slot <= 1)
        g_frame_hw_decoded[slot] = 1;
    for (int i = 0; i < 160; i++)
    {
        aout_buf[i] = (float)audioFrame[i] * 0.25f;
    }
    fill_err_str(err_str, errs2);
}

static void ensure_decoder_initialized(int slot)
{
    if (slot < 0 || slot > 1)
        return;

    bool needsInit = g_channel_needs_init[slot];
    if (g_ambe_type == 3000)
    {
        if (g_ambe3000_last_decoded_slot != -1 && g_ambe3000_last_decoded_slot != slot)
            needsInit = true;
        g_ambe3000_last_decoded_slot = slot;
    }

    g_channel_needs_init[slot] = false;
    if (!needsInit)
        return;

    unsigned char resp[64];
    if (g_ambe_type == 3003)
    {
        unsigned char pkt[7] = {
            AMBE_START_BYTE, 0x00U, 0x03U, AMBE_TYPE_CONTROL,
            (unsigned char)(AMBE3003_PKT_CHANNEL0 + slot),
            AMBE_CONTROL_INIT, 0x02U
        };
#if defined(_WIN32) || defined(__CYGWIN__)
        if (g_win_port.handle != INVALID_HANDLE_VALUE && win32_write_all(&g_win_port, pkt, sizeof(pkt)))
            win32_read_packet(&g_win_port, resp, sizeof(resp), AMBE_TYPE_CONTROL, 50);
#else
        if (g_serial_fd >= 0 && posix_write_all(g_serial_fd, pkt, sizeof(pkt)))
            posix_read_packet(g_serial_fd, resp, sizeof(resp), AMBE_TYPE_CONTROL, 50);
#endif
    }
    else
    {
#if defined(_WIN32) || defined(__CYGWIN__)
        if (g_win_port.handle != INVALID_HANDLE_VALUE &&
            win32_write_all(&g_win_port, PKT_AMBE3000_INIT_DEC, sizeof(PKT_AMBE3000_INIT_DEC)))
        {
            win32_read_packet(&g_win_port, resp, sizeof(resp), AMBE_TYPE_CONTROL, 50);
        }
#else
        if (g_serial_fd >= 0 &&
            posix_write_all(g_serial_fd, PKT_AMBE3000_INIT_DEC, sizeof(PKT_AMBE3000_INIT_DEC)))
        {
            posix_read_packet(g_serial_fd, resp, sizeof(resp), AMBE_TYPE_CONTROL, 50);
        }
#endif
    }
}

static void recover_hardware_device(void)
{
    int savedRate0 = (g_channel_rate[0] != AMBE_RATE_NONE) ? g_channel_rate[0] : AMBE_RATE_3600X2450;
    int savedRate1 = (g_channel_rate[1] != AMBE_RATE_NONE) ? g_channel_rate[1] : AMBE_RATE_3600X2450;
    unsigned char flushZeros[360];
    memset(flushZeros, 0, sizeof(flushZeros));

#if defined(_WIN32) || defined(__CYGWIN__)
    if (g_win_port.handle == INVALID_HANDLE_VALUE)
        return;
    win32_write_all(&g_win_port, flushZeros, sizeof(flushZeros));
    win32_write_all(&g_win_port, PKT_SOFT_RESET, sizeof(PKT_SOFT_RESET));
    ::Sleep(30);
    DWORD errCode = 0;
    ::ClearCommError(g_win_port.handle, &errCode, NULL);
    ::PurgeComm(g_win_port.handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
#else
    if (g_serial_fd < 0)
        return;
    posix_write_all(g_serial_fd, flushZeros, sizeof(flushZeros));
    posix_write_all(g_serial_fd, PKT_SOFT_RESET, sizeof(PKT_SOFT_RESET));
    usleep(30000);
    ::tcflush(g_serial_fd, TCIOFLUSH);
#endif

    g_channel_rate[0] = AMBE_RATE_NONE;
    g_channel_rate[1] = AMBE_RATE_NONE;
    g_ambe3000_last_decoded_slot = -1;
    g_channel_needs_init[0] = false;
    g_channel_needs_init[1] = false;
    ensure_channel_rate(0, savedRate0);
    if (g_ambe_type == 3003)
        ensure_channel_rate(1, savedRate1);
}

static void handle_hardware_error(void)
{
    g_consecutive_errors++;
#if defined(_WIN32) || defined(__CYGWIN__)
    DWORD commErrors = 0;
    BOOL statusOk = (g_win_port.handle != INVALID_HANDLE_VALUE) &&
                    ::ClearCommError(g_win_port.handle, &commErrors, NULL);
    bool fatal = (!statusOk || g_io_error);
#else
    bool fatal = g_io_error;
#endif

    // If the serial interface is still connected, attempt an un-wedge soft reset on the 5th consecutive timeout
    if (!fatal && g_consecutive_errors == 5)
    {
        recover_hardware_device();
    }

    if (fatal || g_consecutive_errors >= 25)
    {
        char timestr[16];
        get_time_str(timestr, sizeof(timestr));
        fprintf(stderr, "\n%s %s connection lost on %s, falling back to MBElib\n",
                timestr, (g_ambe_type == 3000) ? "AMBE3000" : "AMBE3003", g_port_label);
        fflush(stderr);
        g_io_error = true; // Skip soft-reset write during close
        ambe_hw_close();
        if (g_last_dev_path[0] != '\0')
        {
            g_auto_reconnect = true;
            g_next_reconnect_ms = get_monotonic_ms() + AMBE_RECONNECT_INTERVAL_MS;
        }
    }
}

static void maybe_auto_reconnect(void)
{
    if (g_ambe_active || !g_auto_reconnect || g_last_dev_path[0] == '\0')
        return;

    uint64_t now_ms = get_monotonic_ms();
    if (now_ms < g_next_reconnect_ms)
        return;
    g_next_reconnect_ms = now_ms + AMBE_RECONNECT_INTERVAL_MS;

#if defined(_WIN32) || defined(__CYGWIN__)
    if (g_last_port_label[0] != '\0' && strncmp(g_last_port_label, "\\\\.\\", 4) != 0)
    {
        char ntTarget[256] = {0};
        if (::QueryDosDeviceA(g_last_port_label, ntTarget, sizeof(ntTarget)) == 0)
            return;
    }
    if (probe_win32_port(g_last_dev_path, g_last_port_label))
    {
        g_auto_reconnect = false;
    }
#else
    if (::access(g_last_dev_path, R_OK | W_OK) != 0)
        return;
    if (probe_posix_port(g_last_dev_path))
    {
        g_auto_reconnect = false;
    }
#endif
}

static void pack_ambe3600x2450_frame(unsigned char mbeFrame[9], const char ambe_fr[4][24])
{
    memset(mbeFrame, 0, 9);
    for (int i = 0; i < 36; i++)
    {
        unsigned char b1 = (unsigned char)(ambe_fr[DMR_RW[i]][DMR_RX[i]] & 1);
        unsigned char b0 = (unsigned char)(ambe_fr[DMR_RY[i]][DMR_RZ[i]] & 1);
        unsigned char dibit = (unsigned char)((b1 << 1) | b0);
        mbeFrame[i >> 2] |= (unsigned char)(dibit << (6 - 2 * (i & 3)));
    }
}

static void encode_ambe3600x2450_from_d(unsigned char mbeFrame[9], const char ambe_d[49], int errs, int errs2)
{
    char fr[4][24];
    memset(fr, 0, sizeof(fr));

    // C0: 12 voice bits (ambe_d[0..11] at fr[0][23..12]) + 11 Golay(23,12) parity bits (fr[0][11..1]) + overall even parity (fr[0][0])
    uint16_t parity0 = 0;
    for (int i = 0; i < 12; i++)
    {
        char bit = (char)(ambe_d[i] & 1);
        fr[0][23 - i] = bit;
        if (bit)
            parity0 ^= GOLAY_GEN[i];
    }
    for (int b = 0; b < 11; b++)
    {
        fr[0][b + 1] = (char)((parity0 >> b) & 1);
    }
    int sum0 = 0;
    for (int i = 1; i < 24; i++)
    {
        sum0 ^= (fr[0][i] & 1);
    }
    fr[0][0] = (char)sum0;

    // C1: 12 voice bits (ambe_d[12..23] at fr[1][22..11]) + 11 Golay(23,12) parity bits (fr[1][10..0])
    uint16_t parity1 = 0;
    for (int i = 0; i < 12; i++)
    {
        char bit = (char)(ambe_d[12 + i] & 1);
        fr[1][22 - i] = bit;
        if (bit)
            parity1 ^= GOLAY_GEN[i];
    }
    for (int b = 0; b < 11; b++)
    {
        fr[1][b] = (char)((parity1 >> b) & 1);
    }

    // C2: 11 uncoded voice bits (ambe_d[24..34] at fr[2][10..0])
    for (int i = 0; i < 11; i++)
    {
        fr[2][10 - i] = (char)(ambe_d[24 + i] & 1);
    }

    // C3: 14 uncoded voice bits (ambe_d[35..48] at fr[3][13..0])
    for (int i = 0; i < 14; i++)
    {
        fr[3][13 - i] = (char)(ambe_d[35 + i] & 1);
    }

    // Inject over-the-air error counts into C0 and C1 parity bits so the hardware Golay decoder
    // corrects the parity flips without altering the decrypted 49 voice bits while registering
    // the exact RF bit-error counts (E0, E1) for native DVSI error mitigation / frame repeat / muting.
    int e0 = (errs > 0) ? errs : 0;
    int e1 = (errs2 > e0) ? (errs2 - e0) : 0;

    int flip0 = (e0 > 3) ? 3 : e0;
    for (int k = 0; k < flip0; k++)
    {
        fr[0][1 + k] ^= 1;
    }
    if (e0 >= 4)
    {
        // Flip the 24th overall parity bit as well to signal a 4-error uncorrectable C0 condition
        fr[0][0] ^= 1;
    }

    int flip1 = (e1 > 3) ? 3 : e1;
    for (int k = 0; k < flip1; k++)
    {
        fr[1][k] ^= 1;
    }

    // Modulate C1 with the PRNG seeded from the 12 voice bits of C0 (involution)
    mbe_demodulateAmbe3600x2450Data(fr);

    pack_ambe3600x2450_frame(mbeFrame, fr);
}

static bool is_unmodified_ambe_d(const char ambe_d[49], const char ref_d[49])
{
    for (int i = 0; i < 49; i++)
    {
        if ((ambe_d[i] & 1) != (ref_d[i] & 1))
            return false;
    }
    return true;
}

extern "C" int ambe_hw_eccAmbe3600x2450C0(char ambe_fr[4][24])
{
    memcpy(g_captured_ambe_fr, ambe_fr, sizeof(g_captured_ambe_fr));
    g_captured_fr_valid = false;
    int errs = mbe_eccAmbe3600x2450C0(ambe_fr);

    // mbelib's mbe_golay2312 only counts bit errors in the 12 data bits (fr[0][23..12]) and ignores
    // both the 11 Golay parity bits (fr[0][11..1]) and the 24th Extended Golay parity bit (fr[0][0]).
    // Compute the full Extended Golay(24,12) error count (0..4, including 4-bit uncorrectable C0 detection).
    uint16_t parity0 = 0;
    int w0 = 0;
    for (int i = 0; i < 12; i++)
    {
        char bit = (char)(ambe_fr[0][23 - i] & 1);
        if (bit)
            parity0 ^= GOLAY_GEN[i];
        if (bit != (g_captured_ambe_fr[0][23 - i] & 1))
            w0++;
    }
    for (int b = 0; b < 11; b++)
    {
        char pbit = (char)((parity0 >> b) & 1);
        if (pbit != (g_captured_ambe_fr[0][b + 1] & 1))
            w0++;
    }
    int p24 = 0;
    for (int i = 0; i < 24; i++)
    {
        p24 ^= (g_captured_ambe_fr[0][i] & 1);
    }
    g_captured_e0 = w0 + (p24 ^ (w0 & 1));

    return errs;
}

extern "C" int ambe_hw_eccAmbe3600x2450Data(char ambe_fr[4][24], char *ambe_d)
{
    int errs = mbe_eccAmbe3600x2450Data(ambe_fr, ambe_d);
    if (ambe_d != NULL)
    {
        memcpy(g_captured_ambe_d, ambe_d, 49);

        // Compute the full 23-bit Golay(23,12) error count (0..3) on PRNG-demodulated C1
        // including parity bit flips (fr[1][10..0]) that mbelib's mbe_golay2312 does not count.
        uint16_t parity1 = 0;
        int w1 = 0;
        for (int i = 0; i < 12; i++)
        {
            char bit = (char)(ambe_d[12 + i] & 1);
            if (bit)
                parity1 ^= GOLAY_GEN[i];
            if (bit != (ambe_fr[1][22 - i] & 1))
                w1++;
        }
        for (int b = 0; b < 11; b++)
        {
            char pbit = (char)((parity1 >> b) & 1);
            if (pbit != (ambe_fr[1][b] & 1))
                w1++;
        }
        g_captured_e1 = w1;
        g_captured_fr_valid = true;
    }
    return errs;
}

extern "C" void ambe_hw_processAmbe2450Dataf(float *aout_buf, int *errs, int *errs2, char *err_str,
                                             char ambe_d[49], mbe_parms *cur_mp, mbe_parms *prev_mp,
                                             mbe_parms *prev_mp_enhanced, int uvquality)
{
    bool hasCapturedFr = g_captured_fr_valid;
    g_captured_fr_valid = false;

    if (!g_ambe_active && g_auto_reconnect)
        maybe_auto_reconnect();

    int slot = (cur_mp != NULL && cur_mp == g_slot2_mp) ? 1 : 0;
    g_frame_hw_decoded[slot] = 0;

    int in_e0 = (errs != NULL) ? *errs : 0;
    int in_e2 = (errs2 != NULL) ? *errs2 : 0;
    if (hasCapturedFr)
    {
        int mbe_e1 = (in_e2 > in_e0) ? (in_e2 - in_e0) : 0;
        int e0 = (g_captured_e0 > in_e0) ? g_captured_e0 : in_e0;
        int e1 = (g_captured_e1 > mbe_e1) ? g_captured_e1 : mbe_e1;
        ambe_hw_record_ber(slot, e0 + e1, 47);
    }
    else
    {
        ambe_hw_record_ber(slot, in_e2, 47);
    }

    g_frame_muted[slot] = 0;

    if (!is_slot_playable(slot))
    {
        g_channel_needs_init[slot] = true;
        if (g_ambe_type == 3000 && g_ambe3000_locked_slot == slot)
            g_ambe3000_locked_slot = -1;
        g_frame_muted[slot] = 1;
        memset(aout_buf, 0, 160 * sizeof(float));
        if (g_state != NULL)
        {
            if (slot == 0)
            {
                memset(g_state->s_l, 0, sizeof(g_state->s_l));
                memset(g_state->s_lu, 0, sizeof(g_state->s_lu));
                memset(g_state->f_l, 0, sizeof(g_state->f_l));
            }
            else
            {
                memset(g_state->s_r, 0, sizeof(g_state->s_r));
                memset(g_state->s_ru, 0, sizeof(g_state->s_ru));
                memset(g_state->f_r, 0, sizeof(g_state->f_r));
            }
        }
        if (cur_mp != NULL && prev_mp != NULL && prev_mp_enhanced != NULL)
            mbe_initMbeParms(cur_mp, prev_mp, prev_mp_enhanced);
        fill_err_str(err_str, errs2);
        return;
    }

    if (g_ambe_active)
    {

        // Detect DMR RC4 frames where dsd_mbe.c skipped rc4_voice_decrypt because state->errs >= 3,
        // leaving ambe_d unmodified as undecrypted ciphertext (intended to repeat/mute rather than play raw ciphertext).
        bool rc4SkippedDecrypt = false;
        if (hasCapturedFr && in_e0 >= 3 && g_state != NULL && !is_silent_ambe_d(ambe_d))
        {
            if (slot == 0 && g_state->payload_algid == 0x21 && g_state->R != 0 && g_state->payload_mi != 0)
                rc4SkippedDecrypt = true;
            else if (slot == 1 && g_state->payload_algidR == 0x21 && g_state->RR != 0 && g_state->payload_miR != 0)
                rc4SkippedDecrypt = true;
        }

        unsigned char mbeFrame[9];
        if (hasCapturedFr && !rc4SkippedDecrypt && is_unmodified_ambe_d(ambe_d, g_captured_ambe_d))
        {
            // Unencrypted 3600x2450 voice: send the raw 72-bit over-the-air frame directly to hardware
            pack_ambe3600x2450_frame(mbeFrame, g_captured_ambe_fr);
        }
        else if (rc4SkippedDecrypt)
        {
            // Undecrypted high-BER RC4 frame: inject uncorrectable C0+C1 errors (e0=4, e1=3) so hardware repeats/mutes
            encode_ambe3600x2450_from_d(mbeFrame, ambe_d, 4, 7);
        }
        else if (hasCapturedFr)
        {
            // Decrypted live 3600x2450 voice: use full Extended Golay(24,12) C0 (0..4) and Golay(23,12) C1 (0..3) error counts
            int mbe_e1 = (in_e2 > in_e0) ? (in_e2 - in_e0) : 0;
            int e0 = (g_captured_e0 > in_e0) ? g_captured_e0 : in_e0;
            int e1 = (g_captured_e1 > mbe_e1) ? g_captured_e1 : mbe_e1;
            encode_ambe3600x2450_from_d(mbeFrame, ambe_d, e0, e0 + e1);
        }
        else
        {
            // 49-bit-only source (.amb playback or YSF V/D Mode 2):
            // Note that readAmbe2450Data (.amb) sets errs = errs2 (single combined error byte).
            int e0 = in_e0;
            int e1 = (in_e2 > in_e0) ? (in_e2 - in_e0) : 0;
            if (in_e0 == in_e2 && in_e2 > 0)
            {
                e1 = (in_e2 > 3) ? 3 : in_e2;
                e0 = in_e2 - e1;
            }
            encode_ambe3600x2450_from_d(mbeFrame, ambe_d, e0, e0 + e1);
        }

        short audioFrame[160];
        memset(audioFrame, 0, sizeof(audioFrame));
        bool ok = false;

        if (g_ambe_type == 3000)
        {
            uint64_t now_ms = get_monotonic_ms();
            bool silent = is_silent_ambe_d(ambe_d);

            // Hold the locked slot through AMBE3000_HANGTIME_MS (3 seconds) after voice ends before releasing
            if (g_ambe3000_locked_slot != -1 &&
                (!is_locked_slot_still_allowed(g_ambe3000_locked_slot) ||
                 (now_ms - g_ambe3000_locked_last_ms > AMBE3000_HANGTIME_MS)))
            {
                g_ambe3000_locked_slot = -1;
            }

            // Allow explicit user slot preference (*Preferred (3)) to preempt the non-preferred slot
            if (!silent && g_opts != NULL && g_opts->slot_preference == slot)
            {
                g_ambe3000_locked_slot = slot;
                g_ambe3000_locked_last_ms = now_ms;
            }
            else if (g_ambe3000_locked_slot == -1 && !silent)
            {
                g_ambe3000_locked_slot = slot;
                g_ambe3000_locked_last_ms = now_ms;
            }

            if (g_ambe3000_locked_slot != -1 && g_ambe3000_locked_slot != slot)
            {
                // First active slot owns the AMBE3000; mute the second slot while busy
                g_channel_needs_init[slot] = true;
                if (!silent)
                    g_ambe3000_muted_last_ms[slot] = now_ms;
                g_frame_muted[slot] = 1;
                memset(aout_buf, 0, 160 * sizeof(float));
                fill_err_str(err_str, errs2);
                return;
            }

            if (!silent && g_ambe3000_locked_slot == slot)
            {
                g_ambe3000_locked_last_ms = now_ms;
            }

            if (ensure_channel_rate(0, AMBE_RATE_3600X2450))
            {
                ensure_decoder_initialized(slot);
                ok = decode_ambe_frame(audioFrame, mbeFrame, 72U, 9U, 0);
            }
        }
        else
        {
            // AMBE3003: Timeslot 1 (slot 0) -> 0x40, Timeslot 2 (slot 1) -> 0x41
            if (ensure_channel_rate(slot, AMBE_RATE_3600X2450))
            {
                ensure_decoder_initialized(slot);
                ok = decode_ambe_frame(audioFrame, mbeFrame, 72U, 9U, slot);
            }
        }

        if (ok)
        {
            finish_hardware_frame(slot, aout_buf, audioFrame, errs2, err_str);
            return;
        }
        else
        {
            handle_hardware_error();
        }
    }

    mbe_processAmbe2450Dataf(aout_buf, errs, errs2, err_str, ambe_d, cur_mp, prev_mp, prev_mp_enhanced, uvquality);
}

extern "C" void ambe_hw_processAmbe3600x2400Framef(float *aout_buf, int *errs, int *errs2, char *err_str,
                                                   char ambe_fr[4][24], char ambe_d[49], mbe_parms *cur_mp,
                                                   mbe_parms *prev_mp, mbe_parms *prev_mp_enhanced, int uvquality)
{
    if (!g_ambe_active && g_auto_reconnect)
        maybe_auto_reconnect();

    int slot = (cur_mp != NULL && cur_mp == g_slot2_mp) ? 1 : 0;
    g_frame_hw_decoded[slot] = 0;
    g_frame_muted[slot] = 0;

    if (!is_slot_playable(slot))
    {
        g_channel_needs_init[slot] = true;
        g_frame_muted[slot] = 1;
        *errs = mbe_eccAmbe3600x2400C0(ambe_fr);
        *errs2 = *errs;
        mbe_demodulateAmbe3600x2400Data(ambe_fr);
        *errs2 += mbe_eccAmbe3600x2400Data(ambe_fr, ambe_d);
        ambe_hw_record_ber(slot, *errs2, 48);
        memset(aout_buf, 0, 160 * sizeof(float));
        fill_err_str(err_str, errs2);
        return;
    }

    if (g_ambe_active)
    {
        // Reconstruct 72-bit (9-byte) D-STAR air frame before mbelib demodulates ambe_fr in-place
        unsigned char dstarFrame[9];
        memset(dstarFrame, 0, sizeof(dstarFrame));
        for (int i = 0; i < 72; i++)
        {
            unsigned char bit = (unsigned char)(ambe_fr[DSTAR_DW[i]][DSTAR_DX[i]] & 1);
            dstarFrame[i >> 3] |= (unsigned char)(bit << (i & 7));
        }

        // Populate errs, errs2, and ambe_d via mbelib ECC so error bars and payload logging continue to work
        *errs = mbe_eccAmbe3600x2400C0(ambe_fr);
        *errs2 = *errs;
        mbe_demodulateAmbe3600x2400Data(ambe_fr);
        *errs2 += mbe_eccAmbe3600x2400Data(ambe_fr, ambe_d);
        ambe_hw_record_ber(slot, *errs2, 48);

        if (g_ambe_type == 3000)
            g_ambe3000_locked_slot = -1;

        short audioFrame[160];
        memset(audioFrame, 0, sizeof(audioFrame));
        bool ok = false;

        if (ensure_channel_rate(slot, AMBE_RATE_3600X2400))
        {
            ensure_decoder_initialized(slot);
            ok = decode_ambe_frame(audioFrame, dstarFrame, 72U, 9U, slot);
        }

        if (ok)
        {
            finish_hardware_frame(slot, aout_buf, audioFrame, errs2, err_str);
            return;
        }
        else
        {
            handle_hardware_error();
        }

        mbe_processAmbe2400Dataf(aout_buf, errs, errs2, err_str, ambe_d, cur_mp, prev_mp, prev_mp_enhanced, uvquality);
        return;
    }

    mbe_processAmbe3600x2400Framef(aout_buf, errs, errs2, err_str, ambe_fr, ambe_d, cur_mp, prev_mp, prev_mp_enhanced, uvquality);
    ambe_hw_record_ber(slot, *errs2, 48);
}

extern "C" void ambe_hw_initMbeParms(mbe_parms *cur_mp, mbe_parms *prev_mp, mbe_parms *prev_mp_enhanced)
{
    mbe_initMbeParms(cur_mp, prev_mp, prev_mp_enhanced);
    int slot = (cur_mp != NULL && cur_mp == g_slot2_mp) ? 1 : 0;
    ambe_hw_reset_slot(slot);
    if (slot == 1 || (g_opts != NULL && g_opts->playfiles == 1))
        ambe_hw_release_lock();
}

extern "C" int ambe_hw_eccImbe7200x4400Data(char imbe_fr[8][23], char *imbe_d)
{
    g_frame_hw_decoded[0] = 0;
    g_frame_muted[0] = 0;
    int errs2 = mbe_eccImbe7200x4400Data(imbe_fr, imbe_d);
    int total_errs = (g_state != NULL) ? (g_state->errs + errs2) : errs2;
    ambe_hw_record_ber(0, total_errs, 115);
    return errs2;
}

extern "C" void ambe_hw_processAmbe3600x2450Framef(float *aout_buf, int *errs, int *errs2, char *err_str,
                                                   char ambe_fr[4][24], char ambe_d[49], mbe_parms *cur_mp,
                                                   mbe_parms *prev_mp, mbe_parms *prev_mp_enhanced, int uvquality)
{
    *errs = ambe_hw_eccAmbe3600x2450C0(ambe_fr);
    *errs2 = *errs;
    mbe_demodulateAmbe3600x2450Data(ambe_fr);
    *errs2 += ambe_hw_eccAmbe3600x2450Data(ambe_fr, ambe_d);
    ambe_hw_processAmbe2450Dataf(aout_buf, errs, errs2, err_str, ambe_d, cur_mp, prev_mp, prev_mp_enhanced, uvquality);
}
