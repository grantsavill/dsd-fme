/*-------------------------------------------------------------------------------
 * aprs.c
 * Automatic Packet Reporting System (APRS) / AX.25 Packet Radio Decoder
 * Bell 202 1200-baud AFSK (1200 Hz Mark / 2200 Hz Space) & 9600-baud G3RUH FSK
 *
 * LWVMOBILE
 * 2026-11 DSD-FME Florida Man Edition
 *-----------------------------------------------------------------------------*/

#include "dsd.h"
#include <math.h>

#define APRS_SAMPLE_RATE        48000
#define APRS_BAUD_1200          1200
#define APRS_SPS_1200           40      /* 48000 / 1200 = 40 samples per bit */
#define APRS_LPF_LEN_1200       20      /* Half-symbol moving average LPF */
#define APRS_BAUD_9600          9600
#define APRS_SPS_9600           5       /* 48000 / 9600 = 5 samples per bit */

#define APRS_MIN_FRAME_LEN      17      /* Dest(7) + Src(7) + Ctrl(1) + FCS(2) - min */
#define APRS_MAX_FRAME_LEN      330     /* Max AX.25 frame bytes */
#define APRS_FIFO_DEPTH         8       /* Queued frames between symbol loops */
#define APRS_DEDUP_WINDOW       400     /* ~8.3 ms (10 bit periods) for parallel subchannel dedup */

/* Reflected CCITT CRC-16 table (polynomial 0x1021 -> reflected 0x8408) */
static const uint16_t aprs_crc16_tab[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/* HDLC + PLL subchannel state */
typedef struct {
  int32_t lpf_buf[APRS_LPF_LEN_1200];
  int     lpf_idx;
  int32_t lpf_sum;
  int32_t prev_demod;
  int32_t pll_phase;
  int32_t pll_step;
  uint8_t nrzi_prev;
  uint32_t descram_reg;
  int     ones_count;
  int     in_frame;
  uint8_t cur_byte;
  int     bit_idx;
  uint8_t frame_buf[APRS_MAX_FRAME_LEN];
  int     frame_len;
  int     baud;
  uint64_t frame_abs_sum;
  uint32_t frame_sample_cnt;
} aprs_subchan_t;

/* Queued validated AX.25 frame */
typedef struct {
  uint8_t data[APRS_MAX_FRAME_LEN];
  int     len;
  int     baud;
  uint16_t fcs;
  int32_t peak;
} aprs_queued_frame_t;

/* Module-level DSP & FIFO state (persists across 375 ms noCarrier() boundaries) */
static struct {
  int     initialized;

  /* 40-sample quadrature correlator tables (Q14 fixed-point Hann-windowed) */
  int16_t mark_i[APRS_SPS_1200];
  int16_t mark_q[APRS_SPS_1200];
  int16_t space_i[APRS_SPS_1200];
  int16_t space_q[APRS_SPS_1200];

  /* Delay lines for Subchannel 0 (flat) and Subchannel 1 (pre-emphasized) */
  int16_t delay_flat[APRS_SPS_1200];
  int16_t delay_pre[APRS_SPS_1200];
  int     delay_idx;
  int16_t prev_raw_sample;

  /* Leaky peak envelopes for Subchannel 0 twist compensation */
  int32_t mark_env_peak;
  int32_t space_env_peak;

  /* 9600-baud LPF state */
  int32_t fsk96_lpf;

  /* Peak audio sample tracker for In Level display */
  int32_t cur_peak;

  /* Decimator state if opts->wav_interpolator > 1 */
  int     decim_count;
  int32_t decim_sum;

  /* 3 parallel subchannels:
   *   [0] = 1200-baud flat + automatic twist AGC
   *   [1] = 1200-baud pre-emphasized (6 dB/oct boost for de-emphasized audio)
   *   [2] = 9600-baud G3RUH FSK baseband
   */
  aprs_subchan_t sub[3];

  /* Deduplication state across subchannels */
  uint32_t sample_clock;
  uint16_t last_fcs;
  int      last_len;
  uint32_t last_frame_clock;

  /* Validated frame FIFO */
  aprs_queued_frame_t fifo[APRS_FIFO_DEPTH];
  int fifo_head;
  int fifo_tail;
  int fifo_count;
} g_aprs;

/* Compute AX.25 CCITT-16 FCS over buffer (excluding the 2 FCS bytes) */
static uint16_t
aprs_calc_fcs(const uint8_t *buf, int len)
{
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++)
  {
    crc = (crc >> 8) ^ aprs_crc16_tab[(crc ^ buf[i]) & 0xFF];
  }
  return (uint16_t)(crc ^ 0xFFFF);
}

static void
aprs_init_subchan(aprs_subchan_t *sc, int baud)
{
  memset(sc, 0, sizeof(*sc));
  sc->baud = baud;
  if (baud == APRS_BAUD_9600)
    sc->pll_step = (1 << 24) / APRS_SPS_9600;
  else
    sc->pll_step = (1 << 24) / APRS_SPS_1200;
}

static void
aprs_ensure_dsp_tables(void)
{
  if (g_aprs.initialized)
    return;

  memset(&g_aprs, 0, sizeof(g_aprs));

  const double pi = 3.14159265358979323846;
  for (int n = 0; n < APRS_SPS_1200; n++)
  {
    /* Raised-cosine (Sine/Hann) window reduces ISI at 1200 baud */
    double win = 0.5 - 0.5 * cos((2.0 * pi * (double)(n + 0.5)) / (double)APRS_SPS_1200);
    /* Blend 70% rectangular + 30% Hann for sharp transitions with low sidelobes */
    double w = 0.70 + 0.30 * win;
    double scale = 4096.0 * w;

    double ang_m = (2.0 * pi * 1200.0 * (double)n) / (double)APRS_SAMPLE_RATE;
    double ang_s = (2.0 * pi * 2200.0 * (double)n) / (double)APRS_SAMPLE_RATE;

    g_aprs.mark_i[n]  = (int16_t)lrint(scale * cos(ang_m));
    g_aprs.mark_q[n]  = (int16_t)lrint(scale * sin(ang_m));
    g_aprs.space_i[n] = (int16_t)lrint(scale * cos(ang_s));
    g_aprs.space_q[n] = (int16_t)lrint(scale * sin(ang_s));
  }

  g_aprs.mark_env_peak  = 1024;
  g_aprs.space_env_peak = 1024;

  aprs_init_subchan(&g_aprs.sub[0], APRS_BAUD_1200);
  aprs_init_subchan(&g_aprs.sub[1], APRS_BAUD_1200);
  aprs_init_subchan(&g_aprs.sub[2], APRS_BAUD_9600);

  g_aprs.initialized = 1;
}

void
aprs_init_state(dsd_state *state)
{
  aprs_ensure_dsp_tables();
  if (state == NULL)
    return;

  state->aprs_frame_ready = 0;
  state->aprs_pkt_count = 0;
  state->aprs_last_pkt_time = 0;
  state->aprs_src_id = 0;
  state->aprs_src[0] = '\0';
  state->aprs_dst[0] = '\0';
  state->aprs_path[0] = '\0';
  state->aprs_type[0] = '\0';
  state->aprs_symbol[0] = '\0';
  state->aprs_symbol_desc[0] = '\0';
  state->aprs_lat = 0.0;
  state->aprs_lon = 0.0;
  state->aprs_has_pos = 0;
  state->aprs_speed_kmh = 0.0;
  state->aprs_course = -1;
  state->aprs_altitude_m = 0.0;
  state->aprs_has_alt = 0;
  state->aprs_wx_str[0] = '\0';
  state->aprs_comment[0] = '\0';
  state->aprs_tnc2[0] = '\0';
  state->aprs_baud = 1200;
  state->aprs_peak = 0;
}

void
aprs_reset_state(dsd_state *state)
{
  /* Intentionally preserve continuous AFSK demodulator & HDLC state across
   * periodic 375 ms noCarrier() calls so packets spanning window boundaries
   * are not truncated. Only synchronize the ready flag with the FIFO. */
  aprs_ensure_dsp_tables();
  if (state != NULL)
  {
    state->aprs_frame_ready = (g_aprs.fifo_count > 0) ? 1 : 0;
  }
}

/* Validate basic AX.25 address header structure before accepting a frame */
static int
aprs_validate_ax25_header(const uint8_t *buf, int len)
{
  if (len < APRS_MIN_FRAME_LEN)
    return 0;

  /* Address field is a multiple of 7 bytes (2 to 10 callsigns: Dest + Src + 0..8 Digis).
   * Must leave at least 1 Control byte before the 2-byte FCS (so max index < len - 3). */
  int addr_end = -1;
  int max_addr_bytes = (len - 3 < 70) ? (len - 3) : 70;
  for (int i = 0; i < max_addr_bytes; i++)
  {
    if (buf[i] & 0x01)
    {
      addr_end = i;
      break;
    }
  }

  /* Must end on the 7th byte of a callsign subfield (index 6 mod 7) and have >= 14 bytes */
  if (addr_end < 13 || ((addr_end + 1) % 7) != 0)
    return 0;

  /* Check callsign characters (shifted ASCII 'A'..'Z', '0'..'9', or ' ') across Dest, Src, and Digis */
  int num_calls = (addr_end + 1) / 7;
  for (int c = 0; c < num_calls; c++)
  {
    const uint8_t *cs = buf + (c * 7);
    if ((cs[0] >> 1) == ' ')
      return 0;
    for (int j = 0; j < 6; j++)
    {
      uint8_t ch = cs[j] >> 1;
      int is_upper = (ch >= 'A' && ch <= 'Z');
      int is_digit = (ch >= '0' && ch <= '9');
      int is_space = (ch == ' ');
      if (!is_upper && !is_digit && !is_space)
        return 0;
    }
  }

  return 1;
}

/* Feed a single decoded NRZ bit into the HDLC framer for a subchannel */
static void
aprs_hdlc_feed_bit(dsd_state *state, aprs_subchan_t *sc, uint8_t bit)
{
  if (bit)
  {
    sc->ones_count++;
    if (sc->ones_count >= 7)
    {
      /* HDLC Abort (>= 7 consecutive ones) */
      sc->in_frame         = 0;
      sc->frame_len        = 0;
      sc->bit_idx          = 0;
      sc->cur_byte         = 0;
      sc->frame_abs_sum    = 0;
      sc->frame_sample_cnt = 0;
      return;
    }

    if (sc->in_frame)
    {
      sc->cur_byte = (sc->cur_byte >> 1) | 0x80;
      sc->bit_idx++;
      if (sc->bit_idx == 8)
      {
        if (sc->frame_len < APRS_MAX_FRAME_LEN)
        {
          sc->frame_buf[sc->frame_len++] = sc->cur_byte;
        }
        else
        {
          /* Frame overflow - reset */
          sc->in_frame = 0;
        }
        sc->cur_byte = 0;
        sc->bit_idx  = 0;
      }
    }
  }
  else
  {
    /* Zero bit received */
    if (sc->ones_count == 6)
    {
      /* HDLC Flag (01111110 = 0x7E) - must be octet-aligned (bit_idx == 7) */
      if (sc->in_frame && sc->bit_idx == 7 && sc->frame_len >= APRS_MIN_FRAME_LEN)
      {
        /* Check CRC-16 CCITT FCS */
        int payload_len = sc->frame_len - 2;
        uint16_t calc = aprs_calc_fcs(sc->frame_buf, payload_len);
        uint16_t rx_fcs = (uint16_t)sc->frame_buf[payload_len] |
                          ((uint16_t)sc->frame_buf[payload_len + 1] << 8);

        if (calc == rx_fcs && aprs_validate_ax25_header(sc->frame_buf, sc->frame_len))
        {
          /* Check deduplication window across parallel subchannels */
          uint32_t dt = g_aprs.sample_clock - g_aprs.last_frame_clock;
          int is_dup = (dt < APRS_DEDUP_WINDOW &&
                        calc == g_aprs.last_fcs &&
                        sc->frame_len == g_aprs.last_len);

          if (!is_dup && g_aprs.fifo_count < APRS_FIFO_DEPTH)
          {
            aprs_queued_frame_t *qf = &g_aprs.fifo[g_aprs.fifo_tail];
            memcpy(qf->data, sc->frame_buf, sc->frame_len);
            qf->len  = sc->frame_len;
            qf->baud = sc->baud;
            qf->fcs  = calc;
            /* Compute true tone amplitude over the frame samples and scale 0..32768 -> 0..16400 (0..100%) */
            int32_t mean_abs = (sc->frame_sample_cnt > 0)
                                 ? (int32_t)(sc->frame_abs_sum / sc->frame_sample_cnt)
                                 : 0;
            int32_t scaled_peak = (mean_abs * 201) >> 8;
            qf->peak = (scaled_peak > 164) ? scaled_peak : 164;

            g_aprs.fifo_tail = (g_aprs.fifo_tail + 1) % APRS_FIFO_DEPTH;
            g_aprs.fifo_count++;

            g_aprs.last_fcs         = calc;
            g_aprs.last_len         = sc->frame_len;
            g_aprs.last_frame_clock = g_aprs.sample_clock;

            if (state != NULL)
              state->aprs_frame_ready = 1;
          }
        }
      }

      /* Start/restart frame collection after 0x7E flag */
      sc->in_frame         = 1;
      sc->frame_len        = 0;
      sc->bit_idx          = 0;
      sc->cur_byte         = 0;
      sc->ones_count       = 0;
      sc->frame_abs_sum    = 0;
      sc->frame_sample_cnt = 0;
      return;
    }
    else if (sc->ones_count == 5)
    {
      /* Stuffed zero bit after five consecutive ones: discard */
      sc->ones_count = 0;
      return;
    }

    sc->ones_count = 0;

    if (sc->in_frame)
    {
      sc->cur_byte = (sc->cur_byte >> 1);
      sc->bit_idx++;
      if (sc->bit_idx == 8)
      {
        if (sc->frame_len < APRS_MAX_FRAME_LEN)
        {
          sc->frame_buf[sc->frame_len++] = sc->cur_byte;
        }
        else
        {
          sc->in_frame = 0;
        }
        sc->cur_byte = 0;
        sc->bit_idx  = 0;
      }
    }
  }
}

/* Run PLL + NRZI (+ G3RUH descrambler if 9600) on a demodulated baseband sample */
static void
aprs_subchan_step(dsd_state *state, aprs_subchan_t *sc, int32_t demod_val)
{
  int32_t filtered;
  if (sc->baud == APRS_BAUD_1200)
  {
    /* 20-sample moving average LPF */
    sc->lpf_sum -= sc->lpf_buf[sc->lpf_idx];
    sc->lpf_buf[sc->lpf_idx] = demod_val;
    sc->lpf_sum += demod_val;
    sc->lpf_idx++;
    if (sc->lpf_idx >= APRS_LPF_LEN_1200)
      sc->lpf_idx = 0;
    filtered = sc->lpf_sum;
  }
  else
  {
    filtered = demod_val;
  }

  /* Zero-crossing detector for PLL timing recovery (locks transitions to phase == 0) */
  if ((filtered >= 0 && sc->prev_demod < 0) || (filtered < 0 && sc->prev_demod >= 0))
  {
    int32_t phase_err = sc->pll_phase;
    sc->pll_phase -= (phase_err >> 2); /* K_p = 0.25 */
  }
  sc->prev_demod = filtered;

  sc->pll_phase += sc->pll_step;

  /* Sample at center of bit interval (180 deg from transition, when signed 24-bit phase wraps) */
  if (sc->pll_phase >= (1 << 23))
  {
    sc->pll_phase -= (1 << 24);

    uint8_t raw_level = (filtered >= 0) ? 1 : 0;

    /* NRZI decoding: no transition = 1, transition = 0 */
    uint8_t nrzi_bit = (raw_level == sc->nrzi_prev) ? 1 : 0;
    sc->nrzi_prev = raw_level;

    uint8_t data_bit = nrzi_bit;
    if (sc->baud == APRS_BAUD_9600)
    {
      /* G3RUH self-synchronizing descrambler: 1 + x^12 + x^17 */
      uint8_t b12 = (sc->descram_reg >> 11) & 1;
      uint8_t b17 = (sc->descram_reg >> 16) & 1;
      data_bit = nrzi_bit ^ b12 ^ b17;
      sc->descram_reg = ((sc->descram_reg << 1) | nrzi_bit) & 0x1FFFF;
    }

    aprs_hdlc_feed_bit(state, sc, data_bit);
  }
}

/* Fast integer magnitude approximation: |I| + |Q| + min(|I|,|Q|)/2 */
static inline int32_t
aprs_fast_mag(int64_t i_val, int64_t q_val)
{
  int64_t ai = (i_val < 0) ? -i_val : i_val;
  int64_t aq = (q_val < 0) ? -q_val : q_val;
  int64_t mx = (ai > aq) ? ai : aq;
  int64_t mn = (ai > aq) ? aq : ai;
  return (int32_t)((mx + (mn >> 1)) >> 12);
}

void
aprs_push_audio_sample(dsd_opts *opts, dsd_state *state, short sample)
{
  aprs_ensure_dsp_tables();

  /* Handle upsampled WAV files (e.g., wav_interpolator > 1) by decimating to 48 kHz */
  int interp = (opts != NULL && opts->wav_interpolator > 1) ? opts->wav_interpolator : 1;
  if (interp > 1)
  {
    g_aprs.decim_sum += sample;
    g_aprs.decim_count++;
    if (g_aprs.decim_count < interp)
      return;
    sample = (short)(g_aprs.decim_sum / interp);
    g_aprs.decim_sum   = 0;
    g_aprs.decim_count = 0;
  }

  g_aprs.sample_clock++;

  int32_t abs_s = (sample < 0) ? -(int32_t)sample : (int32_t)sample;
  for (int s = 0; s < 3; s++)
  {
    if (g_aprs.sub[s].in_frame)
    {
      g_aprs.sub[s].frame_abs_sum += (uint64_t)abs_s;
      g_aprs.sub[s].frame_sample_cnt++;
    }
  }

  /* Pre-emphasis filter: y[n] = x[n] - 0.85 * x[n-1] to equalize de-emphasized FM audio */
  int32_t pre_val = (int32_t)sample - (((int32_t)g_aprs.prev_raw_sample * 218) >> 8);
  if (pre_val > 32767)  pre_val = 32767;
  if (pre_val < -32768) pre_val = -32768;
  g_aprs.prev_raw_sample = sample;

  int idx = g_aprs.delay_idx;
  g_aprs.delay_flat[idx] = sample;
  g_aprs.delay_pre[idx]  = (int16_t)pre_val;
  idx++;
  if (idx >= APRS_SPS_1200)
    idx = 0;
  g_aprs.delay_idx = idx;

  /* Compute 40-sample quadrature correlation for Subchannel 0 (flat) & Subchannel 1 (pre) */
  int64_t m0_i = 0, m0_q = 0, s0_i = 0, s0_q = 0;
  int64_t m1_i = 0, m1_q = 0, s1_i = 0, s1_q = 0;

  int k = idx;
  for (int n = 0; n < APRS_SPS_1200; n++)
  {
    int32_t x0 = g_aprs.delay_flat[k];
    int32_t x1 = g_aprs.delay_pre[k];
    k++;
    if (k >= APRS_SPS_1200)
      k = 0;

    int32_t mi = g_aprs.mark_i[n];
    int32_t mq = g_aprs.mark_q[n];
    int32_t si = g_aprs.space_i[n];
    int32_t sq = g_aprs.space_q[n];

    m0_i += x0 * mi;
    m0_q += x0 * mq;
    s0_i += x0 * si;
    s0_q += x0 * sq;

    m1_i += x1 * mi;
    m1_q += x1 * mq;
    s1_i += x1 * si;
    s1_q += x1 * sq;
  }

  /* Subchannel 0: Flat audio + automatic leaky peak-envelope twist compensation */
  int32_t mag_m0 = aprs_fast_mag(m0_i, m0_q);
  int32_t mag_s0 = aprs_fast_mag(s0_i, s0_q);

  if (mag_m0 > g_aprs.mark_env_peak)
    g_aprs.mark_env_peak = mag_m0;
  else
    g_aprs.mark_env_peak -= (g_aprs.mark_env_peak >> 10) + 1;
  if (g_aprs.mark_env_peak < 256)
    g_aprs.mark_env_peak = 256;

  if (mag_s0 > g_aprs.space_env_peak)
    g_aprs.space_env_peak = mag_s0;
  else
    g_aprs.space_env_peak -= (g_aprs.space_env_peak >> 10) + 1;
  if (g_aprs.space_env_peak < 256)
    g_aprs.space_env_peak = 256;

  int32_t norm_m0 = (mag_m0 << 10) / g_aprs.mark_env_peak;
  int32_t norm_s0 = (mag_s0 << 10) / g_aprs.space_env_peak;
  int32_t demod0  = norm_m0 - norm_s0;

  /* Subchannel 1: Pre-emphasized audio + normalized (M - S) / (M + S + 1) */
  int32_t mag_m1 = aprs_fast_mag(m1_i, m1_q);
  int32_t mag_s1 = aprs_fast_mag(s1_i, s1_q);
  int32_t denom1 = mag_m1 + mag_s1 + 1;
  int32_t demod1 = ((mag_m1 - mag_s1) << 10) / denom1;

  /* Step both 1200-baud AFSK subchannels */
  aprs_subchan_step(state, &g_aprs.sub[0], demod0);
  aprs_subchan_step(state, &g_aprs.sub[1], demod1);

  /* Subchannel 2: 9600-baud G3RUH FSK baseband (2-pole exponential LPF) */
  g_aprs.fsk96_lpf += (((int32_t)sample - g_aprs.fsk96_lpf) >> 1);
  aprs_subchan_step(state, &g_aprs.sub[2], g_aprs.fsk96_lpf);
}

int
aprs_check_sync(dsd_opts *opts, dsd_state *state)
{
  UNUSED(opts);
  if (g_aprs.fifo_count > 0)
  {
    if (state != NULL)
      state->aprs_frame_ready = 1;
    return 1;
  }
  if (state != NULL)
    state->aprs_frame_ready = 0;
  return 0;
}

/*-------------------------------------------------------------------------------
 * APRS Symbol Table Lookup (Primary '/' and Alternate '\')
 *-----------------------------------------------------------------------------*/
static const char *
aprs_lookup_symbol(char table, char code)
{
  if (table == '/')
  {
    switch (code)
    {
      case '!': return "Police/Sheriff";
      case '#': return "Digipeater";
      case '$': return "Phone";
      case '%': return "DX Cluster";
      case '&': return "HF Gateway";
      case '\'': return "Small Aircraft";
      case '(': return "Mobile Sat Station";
      case ')': return "Wheelchair";
      case '*': return "Snowmobile";
      case '+': return "Red Cross";
      case ',': return "Boy Scouts";
      case '-': return "House QTH (VHF)";
      case '.': return "X";
      case '/': return "Red Dot";
      case ';': return "Campground/Portable";
      case '<': return "Motorcycle";
      case '=': return "Railroad Engine";
      case '>': return "Car";
      case '?': return "File Server";
      case '@': return "HC Future Forecast";
      case 'A': return "Aid Station";
      case 'B': return "BBS";
      case 'C': return "Canoe";
      case 'E': return "Eyeball";
      case 'F': return "Farm Vehicle/Tractor";
      case 'G': return "Grid Square";
      case 'H': return "Hotel";
      case 'I': return "TCP/IP Station";
      case 'K': return "School";
      case 'L': return "PC User";
      case 'M': return "MacAPRS";
      case 'N': return "NTS Station";
      case 'O': return "Balloon";
      case 'P': return "Police";
      case 'R': return "RV";
      case 'S': return "Space Shuttle";
      case 'T': return "SSTV";
      case 'U': return "Bus";
      case 'V': return "ATV";
      case 'W': return "National WX Service";
      case 'X': return "Helicopter";
      case 'Y': return "Yacht/Sailboat";
      case 'Z': return "WinAPRS";
      case '[': return "Jogger/Person";
      case '^': return "Large Aircraft";
      case '_': return "Weather Station";
      case '`': return "Dish Antenna";
      case 'a': return "Ambulance";
      case 'b': return "Bicycle";
      case 'c': return "Incident Command Post";
      case 'd': return "Fire Dept";
      case 'e': return "Horse/Equestrian";
      case 'f': return "Fire Truck";
      case 'g': return "Glider";
      case 'h': return "Hospital";
      case 'i': return "IOTA";
      case 'j': return "Jeep";
      case 'k': return "Truck";
      case 'l': return "Laptop";
      case 'm': return "Mic-Repeater";
      case 'n': return "Node";
      case 'o': return "EOC";
      case 'p': return "Rover/Dog";
      case 'q': return "Grid Sq <2mi";
      case 'r': return "Repeater/Antenna";
      case 's': return "Ship/Power Boat";
      case 't': return "Truck Stop";
      case 'u': return "Semi Truck (18-wheeler)";
      case 'v': return "Van";
      case 'w': return "Water Station";
      case 'x': return "X-APRS";
      case 'y': return "Yagi @ QTH";
      case 'z': return "Shelter";
      default:  return "Primary Symbol";
    }
  }
  else
  {
    switch (code)
    {
      case '!': return "Emergency";
      case '#': return "Overlay Digipeater";
      case '$': return "Bank/ATM";
      case '&': return "I-Gate / Gateway";
      case '(': return "Cloudy";
      case '*': return "Snow";
      case '-': return "House (HF)";
      case '<': return "Advisory/Gale Flag";
      case '>': return "Overlay Car";
      case '@': return "Hurricane/Tropical Storm";
      case 'A': return "Overlay Box";
      case 'H': return "Haze";
      case 'I': return "Rain Shower";
      case 'K': return "Kenwood HT";
      case 'L': return "Lighthouse";
      case 'N': return "Navigation Buoy";
      case 'O': return "Rocket";
      case 'P': return "Parking";
      case 'Q': return "Earthquake";
      case 'R': return "Restaurant";
      case 'S': return "Satellite/PACsats";
      case 'T': return "Thunderstorm";
      case 'U': return "Sunny";
      case 'V': return "VORTAC Nav Aid";
      case 'W': return "NWS Site";
      case 'X': return "Pharmacy/Rx";
      case '^': return "Overlay Aircraft";
      case '_': return "WX Station (Digi)";
      case '`': return "Rain";
      case 'a': return "ARRL/ARES/RACES";
      case 'c': return "Civil Defense";
      case 'f': return "Funnel Cloud";
      case 'g': return "Gale Flags";
      case 'h': return "Ham Store";
      case 'k': return "SUV/ATV";
      case 'n': return "Overlay Node";
      case 'r': return "Restrooms";
      case 's': return "Overlay Ship/Boat";
      case 't': return "Tornado";
      case 'u': return "Overlay Truck";
      case 'v': return "Overlay Van";
      case 'w': return "Flooding";
      case 'y': return "Skywarn";
      default:  return "Alt/Overlay Symbol";
    }
  }
}

/* Format a 7-byte AX.25 address subfield into "CALL-SSID" */
static uint32_t
aprs_callsign_to_lrrp_id(const char *call)
{
  if (call == NULL || call[0] == '\0')
    return 1000000u;
  uint32_t h = 2166136261u;
  for (int i = 0; call[i] != '\0'; i++)
  {
    h = (h ^ (uint8_t)call[i]) * 16777619u;
  }
  return 1000000u + (h % 9000000u);
}

static void
aprs_format_callsign(const uint8_t *addr, char *out, size_t out_sz, int *has_hbit)
{
  char call[8];
  int len = 0;
  for (int i = 0; i < 6; i++)
  {
    char ch = (char)(addr[i] >> 1);
    if (ch != ' ' && ch >= 0x21 && ch <= 0x7E)
    {
      call[len++] = ch;
    }
  }
  call[len] = '\0';

  int ssid = (addr[6] >> 1) & 0x0F;
  if (has_hbit != NULL)
    *has_hbit = (addr[6] & 0x80) ? 1 : 0;

  if (ssid > 0)
    snprintf(out, out_sz, "%s-%d", call, ssid);
  else
    snprintf(out, out_sz, "%s", call);
}

/* Sanitize raw string into printable ASCII */
static void
aprs_sanitize_ascii(const char *in, int in_len, char *out, size_t out_sz)
{
  if (out_sz == 0)
    return;
  size_t o = 0;
  for (int i = 0; i < in_len && o + 1 < out_sz; i++)
  {
    unsigned char c = (unsigned char)in[i];
    if (c >= 0x20 && c <= 0x7E)
    {
      out[o++] = (char)c;
    }
  }
  out[o] = '\0';
}

/* Scan comment text for `/A=xxxxxx` altitude (feet -> meters) */
static void
aprs_extract_comment_altitude(dsd_state *state, const char *comment)
{
  if (comment == NULL)
    return;
  const char *p = strstr(comment, "/A=");
  if (p != NULL)
  {
    p += 3;
    char alt_buf[8];
    int valid = 1;
    for (int i = 0; i < 6; i++)
    {
      if ((p[i] >= '0' && p[i] <= '9') || (i == 0 && p[i] == '-'))
        alt_buf[i] = p[i];
      else
      {
        valid = 0;
        break;
      }
    }
    if (valid)
    {
      alt_buf[6] = '\0';
      long alt_ft = strtol(alt_buf, NULL, 10);
      state->aprs_altitude_m = (double)alt_ft * 0.3048;
      state->aprs_has_alt = 1;
    }
  }
}

/* Parse APRS weather fields (`c...s...g...t...r...p...P...h..b.....`) */
static int
aprs_parse_weather_fields(dsd_state *state, const char *s)
{
  if (s == NULL || *s == '\0')
    return 0;

  int wind_dir = -1;
  int wind_spd = -1;
  int wind_gst = -1;
  int temp_f   = -9999;
  int hum      = -1;
  double baro  = -1.0;
  double rain1h = -1.0;

  const char *p = s;

  /* Helper macro: check if `n` chars starting at `q` are digits, dots, or spaces */
  #define APRS_WX_CHARS(q, n) ({ \
    int _ok = 1; \
    for (int _k = 0; _k < (n); _k++) { \
      char _ch = (q)[_k]; \
      if (!((_ch >= '0' && _ch <= '9') || _ch == '.' || _ch == ' ' || (_k == 0 && _ch == '-'))) { _ok = 0; break; } \
    } \
    _ok; \
  })

  /* Positionless weather reports (`_`) begin with an 8-digit `MMDDhhmm` timestamp */
  {
    int is_ts8 = 1;
    for (int i = 0; i < 8; i++)
    {
      if (!(p[i] >= '0' && p[i] <= '9'))
      {
        is_ts8 = 0;
        break;
      }
    }
    if (is_ts8 && (p[8] == 'c' || p[8] == 's' || p[8] == 'g' || p[8] == 't'))
      p += 8;
  }

  /* Check if starts with CSE/SPD `ddd/sss` (or `.../...`) for WX symbol '_' */
  if (p[0] != '\0' && p[1] != '\0' && p[2] != '\0' && p[3] == '/' &&
      p[4] != '\0' && p[5] != '\0' && p[6] != '\0' &&
      APRS_WX_CHARS(p, 3) && APRS_WX_CHARS(p + 4, 3))
  {
    if (p[0] >= '0' && p[0] <= '9' && p[1] >= '0' && p[1] <= '9' && p[2] >= '0' && p[2] <= '9')
      wind_dir = (p[0] - '0') * 100 + (p[1] - '0') * 10 + (p[2] - '0');
    if (p[4] >= '0' && p[4] <= '9' && p[5] >= '0' && p[5] <= '9' && p[6] >= '0' && p[6] <= '9')
      wind_spd = (p[4] - '0') * 100 + (p[5] - '0') * 10 + (p[6] - '0');
    p += 7;
  }
  else if (state->aprs_course >= 0 && state->aprs_speed_kmh >= 0.0)
  {
    /* Compressed weather packets encode wind direction & speed in the `csT` bytes */
    wind_dir = state->aprs_course;
    wind_spd = (int)lrint(state->aprs_speed_kmh / 1.852);
  }

  /* Parse contiguous weather tokens; stop when free-text comment / software ID begins */
  while (*p != '\0')
  {
    if (p[0] == 'c' && APRS_WX_CHARS(p + 1, 3))
    {
      if (p[1] >= '0' && p[1] <= '9' && p[2] >= '0' && p[2] <= '9' && p[3] >= '0' && p[3] <= '9')
        wind_dir = (p[1] - '0') * 100 + (p[2] - '0') * 10 + (p[3] - '0');
      p += 4;
    }
    else if (p[0] == 's' && APRS_WX_CHARS(p + 1, 3))
    {
      /* Only set wind_spd on the first `sXXX` (before `tXXX`); later `sXXX` is 24h snowfall */
      if (wind_spd < 0 && temp_f == -9999 &&
          p[1] >= '0' && p[1] <= '9' && p[2] >= '0' && p[2] <= '9' && p[3] >= '0' && p[3] <= '9')
        wind_spd = (p[1] - '0') * 100 + (p[2] - '0') * 10 + (p[3] - '0');
      p += 4;
    }
    else if (p[0] == 'g' && APRS_WX_CHARS(p + 1, 3))
    {
      if (p[1] >= '0' && p[1] <= '9' && p[2] >= '0' && p[2] <= '9' && p[3] >= '0' && p[3] <= '9')
        wind_gst = (p[1] - '0') * 100 + (p[2] - '0') * 10 + (p[3] - '0');
      p += 4;
    }
    else if (p[0] == 't' && APRS_WX_CHARS(p + 1, 3))
    {
      if (((p[1] >= '0' && p[1] <= '9') || p[1] == '-') &&
          p[2] >= '0' && p[2] <= '9' && p[3] >= '0' && p[3] <= '9')
      {
        char tb[4] = { p[1], p[2], p[3], '\0' };
        temp_f = (int)strtol(tb, NULL, 10);
      }
      p += 4;
    }
    else if (p[0] == 'r' && APRS_WX_CHARS(p + 1, 3))
    {
      if (p[1] >= '0' && p[1] <= '9' && p[2] >= '0' && p[2] <= '9' && p[3] >= '0' && p[3] <= '9')
      {
        int r = (p[1] - '0') * 100 + (p[2] - '0') * 10 + (p[3] - '0');
        rain1h = r * 0.01;
      }
      p += 4;
    }
    else if ((p[0] == 'p' || p[0] == 'P') && APRS_WX_CHARS(p + 1, 3))
    {
      /* 24h / since-midnight rain (skip over token to reach subsequent fields) */
      p += 4;
    }
    else if ((p[0] == 'L' || p[0] == 'l') && APRS_WX_CHARS(p + 1, 3))
    {
      /* Luminosity token (skip over token to reach subsequent fields) */
      p += 4;
    }
    else if (p[0] == 'h' && APRS_WX_CHARS(p + 1, 2))
    {
      if (p[1] >= '0' && p[1] <= '9' && p[2] >= '0' && p[2] <= '9')
      {
        hum = (p[1] - '0') * 10 + (p[2] - '0');
        if (hum == 0)
          hum = 100;
      }
      p += 3;
    }
    else if (p[0] == 'b' && APRS_WX_CHARS(p + 1, 5))
    {
      if (p[1] >= '0' && p[1] <= '9' && p[2] >= '0' && p[2] <= '9' &&
          p[3] >= '0' && p[3] <= '9' && p[4] >= '0' && p[4] <= '9' && p[5] >= '0' && p[5] <= '9')
      {
        char bb[6] = { p[1], p[2], p[3], p[4], p[5], '\0' };
        baro = (double)strtol(bb, NULL, 10) / 10.0;
      }
      p += 6;
    }
    else
    {
      /* Reached end of contiguous weather fields (start of software ID / free-text comment) */
      break;
    }
  }
  #undef APRS_WX_CHARS

  if (temp_f != -9999 || wind_spd >= 0 || wind_dir >= 0 || wind_gst >= 0 || baro > 0.0 || hum >= 0 || rain1h >= 0.0)
  {
    char wx[128];
    wx[0] = '\0';
    size_t off = 0;

    if (temp_f != -9999)
    {
      double temp_c = ((double)temp_f - 32.0) * (5.0 / 9.0);
      off += snprintf(wx + off, sizeof(wx) - off, "Temp: %dF (%.1fC) ", temp_f, temp_c);
    }
    if (wind_dir >= 0 && wind_spd >= 0)
    {
      off += snprintf(wx + off, sizeof(wx) - off, "Wind: %03d@%dmph ", wind_dir, wind_spd);
    }
    else if (wind_spd >= 0)
    {
      off += snprintf(wx + off, sizeof(wx) - off, "Wind: %dmph ", wind_spd);
    }
    if (wind_gst >= 0)
    {
      off += snprintf(wx + off, sizeof(wx) - off, "Gust: %dmph ", wind_gst);
    }
    if (hum >= 0 && off < sizeof(wx))
    {
      off += snprintf(wx + off, sizeof(wx) - off, "RH: %d%% ", hum);
    }
    if (baro > 0.0 && off < sizeof(wx))
    {
      off += snprintf(wx + off, sizeof(wx) - off, "Baro: %.1fhPa ", baro);
    }
    if (rain1h > 0.0 && off < sizeof(wx))
    {
      off += snprintf(wx + off, sizeof(wx) - off, "Rain1h: %.2fin", rain1h);
    }

    snprintf(state->aprs_wx_str, sizeof(state->aprs_wx_str), "%s", wx);
    return 1;
  }

  return 0;
}

/* Parse Uncompressed or Base-91 Compressed APRS Position (`pos` points after DTI & optional timestamp) */
static void
aprs_parse_position_body(dsd_state *state, const char *pos, int pos_len)
{
  if (pos_len < 13)
  {
    aprs_sanitize_ascii(pos, pos_len, state->aprs_comment, sizeof(state->aprs_comment));
    return;
  }

  /* Check for Uncompressed Position: DDMM.mmN/DDDMM.mmW$ (19 chars, indices 0..18) */
  if (pos_len >= 19 &&
      pos[0] >= '0' && pos[0] <= '9' && pos[1] >= '0' && pos[1] <= '9' &&
      pos[4] == '.' &&
      (pos[7] == 'N' || pos[7] == 'S' || pos[7] == 'n' || pos[7] == 's') &&
      pos[9] >= '0' && pos[9] <= '9' && pos[10] >= '0' && pos[10] <= '9' && pos[11] >= '0' && pos[11] <= '9' &&
      pos[14] == '.' &&
      (pos[17] == 'E' || pos[17] == 'W' || pos[17] == 'e' || pos[17] == 'w'))
  {
    char deg_lat[3] = { pos[0], pos[1], '\0' };
    char min_lat[6] = { pos[2], pos[3], '.', pos[5], pos[6], '\0' };
    /* Replace APRS ambiguity spaces with '0' */
    for (int i = 0; i < 5; i++)
      if (min_lat[i] == ' ') min_lat[i] = '0';

    double mlat = strtod(min_lat, NULL);
    double lat = strtod(deg_lat, NULL) + (mlat / 60.0);
    if (pos[7] == 'S' || pos[7] == 's')
      lat = -lat;

    char deg_lon[4] = { pos[9], pos[10], pos[11], '\0' };
    char min_lon[6] = { pos[12], pos[13], '.', pos[15], pos[16], '\0' };
    for (int i = 0; i < 5; i++)
      if (min_lon[i] == ' ') min_lon[i] = '0';

    double mlon = strtod(min_lon, NULL);
    double lon = strtod(deg_lon, NULL) + (mlon / 60.0);
    if (pos[17] == 'W' || pos[17] == 'w')
      lon = -lon;

    char sym_table = pos[8];
    char sym_code  = pos[18];

    if (mlat >= 0.0 && mlat < 60.0 && mlon >= 0.0 && mlon < 60.0 &&
        fabs(lat) <= 90.0 && fabs(lon) <= 180.0)
    {
      state->aprs_lat     = lat;
      state->aprs_lon     = lon;
      state->aprs_has_pos = 1;
    }
    state->aprs_symbol[0] = sym_table;
    state->aprs_symbol[1] = sym_code;
    state->aprs_symbol[2] = '\0';
    snprintf(state->aprs_symbol_desc, sizeof(state->aprs_symbol_desc), "%s",
             aprs_lookup_symbol(sym_table, sym_code));

    const char *ext = pos + 19;
    int ext_len = pos_len - 19;

    /* Check for Weather Symbol '_' or weather fields */
    if (sym_code == '_')
    {
      snprintf(state->aprs_type, sizeof(state->aprs_type), "Weather");
      aprs_parse_weather_fields(state, ext);
    }
    else if (ext_len >= 7 &&
             ext[0] >= '0' && ext[0] <= '9' &&
             ext[1] >= '0' && ext[1] <= '9' &&
             ext[2] >= '0' && ext[2] <= '9' &&
             ext[3] == '/' &&
             ext[4] >= '0' && ext[4] <= '9' &&
             ext[5] >= '0' && ext[5] <= '9' &&
             ext[6] >= '0' && ext[6] <= '9')
    {
      int cse = (ext[0] - '0') * 100 + (ext[1] - '0') * 10 + (ext[2] - '0');
      int spd_kts = (ext[4] - '0') * 100 + (ext[5] - '0') * 10 + (ext[6] - '0');
      if (cse <= 360)
      {
        state->aprs_course    = (cse == 0 && spd_kts == 0) ? -1 : cse;
        state->aprs_speed_kmh = (double)spd_kts * 1.852;
      }
      ext     += 7;
      ext_len -= 7;
    }

    aprs_sanitize_ascii(ext, ext_len, state->aprs_comment, sizeof(state->aprs_comment));
    aprs_extract_comment_altitude(state, state->aprs_comment);
    return;
  }

  /* Check for Base-91 Compressed Position: `/YYYYXXXX$csT` (13 bytes) */
  if (pos_len >= 13 && (pos[0] == '/' || pos[0] == '\\' || (pos[0] >= '0' && pos[0] <= '9') || (pos[0] >= 'A' && pos[0] <= 'Z')))
  {
    int valid_b91 = 1;
    for (int i = 1; i <= 8; i++)
    {
      if (pos[i] < 33 || pos[i] > 124)
      {
        valid_b91 = 0;
        break;
      }
    }
    if (valid_b91)
    {
      char sym_table = pos[0];
      char sym_code  = pos[9];

      double y = (double)(pos[1] - 33) * 753571.0 +
                 (double)(pos[2] - 33) * 8281.0 +
                 (double)(pos[3] - 33) * 91.0 +
                 (double)(pos[4] - 33);
      double x = (double)(pos[5] - 33) * 753571.0 +
                 (double)(pos[6] - 33) * 8281.0 +
                 (double)(pos[7] - 33) * 91.0 +
                 (double)(pos[8] - 33);

      state->aprs_lat     = 90.0 - (y / 380926.0);
      state->aprs_lon     = -180.0 + (x / 190463.0);
      state->aprs_has_pos = 1;
      state->aprs_symbol[0] = sym_table;
      state->aprs_symbol[1] = sym_code;
      state->aprs_symbol[2] = '\0';
      snprintf(state->aprs_symbol_desc, sizeof(state->aprs_symbol_desc), "%s",
               aprs_lookup_symbol(sym_table, sym_code));

      unsigned char c_byte = (unsigned char)pos[10];
      unsigned char s_byte = (unsigned char)pos[11];
      unsigned char t_byte = (unsigned char)pos[12];

      if (c_byte >= 33 && c_byte <= 124 && s_byte >= 33 && s_byte <= 124 && t_byte >= 33 && t_byte <= 124)
      {
        int t_val = (int)t_byte - 33;
        if ((t_val & 0x18) == 0x10)
        {
          /* Compressed Altitude (NMEA GGA source): 1.002^(c*91 + s) feet */
          int cs = ((int)c_byte - 33) * 91 + ((int)s_byte - 33);
          double alt_ft = pow(1.002, (double)cs);
          state->aprs_altitude_m = alt_ft * 0.3048;
          state->aprs_has_alt    = 1;
        }
        else if (c_byte >= '!' && c_byte <= 'z')
        {
          int c_val = (int)c_byte - 33;
          int s_val = (int)s_byte - 33;
          if (c_val >= 0 && c_val <= 89)
          {
            state->aprs_course = (c_val == 0) ? 360 : (c_val * 4);
            double spd_kts = pow(1.08, (double)s_val) - 1.0;
            state->aprs_speed_kmh = spd_kts * 1.852;
          }
        }
      }

      const char *ext = pos + 13;
      int ext_len = pos_len - 13;
      if (sym_code == '_')
      {
        snprintf(state->aprs_type, sizeof(state->aprs_type), "Weather");
        aprs_parse_weather_fields(state, ext);
      }
      aprs_sanitize_ascii(ext, ext_len, state->aprs_comment, sizeof(state->aprs_comment));
      aprs_extract_comment_altitude(state, state->aprs_comment);
      return;
    }
  }

  aprs_sanitize_ascii(pos, pos_len, state->aprs_comment, sizeof(state->aprs_comment));
}

/* Parse Mic-E encoded packet (`'`, `` ` ``, `0x1C`, `0x1D`) */
static void
aprs_parse_mice(dsd_state *state, const uint8_t *dest_addr, const char *info, int info_len)
{
  snprintf(state->aprs_type, sizeof(state->aprs_type), "Mic-E");
  if (info_len < 9)
  {
    aprs_sanitize_ascii(info, info_len, state->aprs_comment, sizeof(state->aprs_comment));
    return;
  }

  /* Decode 6 destination address bytes for Latitude, Status (M0..M2), N/S, +100 Lon, E/W */
  int lat_digits[6] = { 0, 0, 0, 0, 0, 0 };
  int std_bits  = 0;
  int cust_bits = 0;
  int is_north  = 1;
  int lon_100   = 0;
  int is_west   = 1;

  for (int i = 0; i < 6; i++)
  {
    char c = (char)(dest_addr[i] >> 1);
    int d = 0;
    int bit_s = 0;
    int bit_c = 0;

    if (c >= '0' && c <= '9')
    {
      d = c - '0';
    }
    else if (c >= 'A' && c <= 'J')
    {
      d = c - 'A';
      bit_c = 1;
    }
    else if (c == 'K')
    {
      d = 0;
      bit_c = 1;
    }
    else if (c == 'L')
    {
      d = 0;
    }
    else if (c >= 'P' && c <= 'Y')
    {
      d = c - 'P';
      bit_s = 1;
    }
    else if (c == 'Z')
    {
      d = 0;
      bit_s = 1;
    }

    lat_digits[i] = d;
    if (i < 3)
    {
      std_bits  = (std_bits << 1) | bit_s;
      cust_bits = (cust_bits << 1) | bit_c;
    }
    else if (i == 3)
    {
      is_north = bit_s;
    }
    else if (i == 4)
    {
      lon_100 = bit_s;
    }
    else if (i == 5)
    {
      is_west = bit_s;
    }
  }

  double lat_deg = lat_digits[0] * 10.0 + lat_digits[1];
  double lat_min = lat_digits[2] * 10.0 + lat_digits[3] +
                   (lat_digits[4] * 10.0 + lat_digits[5]) / 100.0;
  double lat = lat_deg + (lat_min / 60.0);
  if (!is_north)
    lat = -lat;

  /* Decode Longitude, Speed, and Course from info[1..6] */
  int d28 = (int)((unsigned char)info[1]) - 28;
  if (d28 < 0) d28 = 0;
  if (lon_100)
    d28 += 100;
  if (d28 >= 180 && d28 <= 189)
    d28 -= 80;
  else if (d28 >= 190 && d28 <= 199)
    d28 -= 190;

  int m28 = (int)((unsigned char)info[2]) - 28;
  if (m28 < 0) m28 = 0;
  if (m28 >= 60)
    m28 -= 60;

  int h28 = (int)((unsigned char)info[3]) - 28;
  if (h28 < 0) h28 = 0;
  double lon = (double)d28 + (((double)m28 + ((double)h28 / 100.0)) / 60.0);
  if (is_west)
    lon = -lon;

  int sp28 = (int)((unsigned char)info[4]) - 28;
  int dc28 = (int)((unsigned char)info[5]) - 28;
  int se28 = (int)((unsigned char)info[6]) - 28;
  if (sp28 < 0) sp28 = 0;
  if (dc28 < 0) dc28 = 0;
  if (se28 < 0) se28 = 0;

  int spd_kts = (sp28 * 10) + (dc28 / 10);
  if (spd_kts >= 800)
    spd_kts -= 800;

  int cse = ((dc28 % 10) * 100) + se28;
  if (cse >= 400)
    cse -= 400;

  char sym_code  = info[7];
  char sym_table = info[8];

  if (lat_deg <= 90.0 && lat_min < 60.0 && fabs(lat) <= 90.0 && fabs(lon) <= 180.0)
  {
    state->aprs_lat     = lat;
    state->aprs_lon     = lon;
    state->aprs_has_pos = 1;
  }
  state->aprs_speed_kmh = (double)spd_kts * 1.852;
  state->aprs_course    = (cse >= 0 && cse <= 360) ? ((cse == 0 && spd_kts == 0) ? -1 : cse) : -1;
  state->aprs_symbol[0] = sym_table;
  state->aprs_symbol[1] = sym_code;
  state->aprs_symbol[2] = '\0';
  snprintf(state->aprs_symbol_desc, sizeof(state->aprs_symbol_desc), "%s",
           aprs_lookup_symbol(sym_table, sym_code));

  /* Mic-E Status Text */
  static const char * const std_mic_status[8] = {
    "Emergency", "Priority", "Special", "Committed",
    "Returning", "In Service", "En Route", "Off Duty"
  };
  const char *status_str = std_mic_status[std_bits & 7];
  if (std_bits == 0 && cust_bits != 0)
    status_str = "Custom";

  const char *rem = info + 9;
  int rem_len = info_len - 9;

  /* Check for Mic-E Base-91 Altitude (`xxx}` at start of comment or after 1..2 byte radio type prefix) */
  for (int off = 0; off <= 2 && off + 4 <= rem_len; off++)
  {
    if (rem[off + 3] == '}' &&
        (unsigned char)rem[off] >= 33 && (unsigned char)rem[off] <= 124 &&
        (unsigned char)rem[off + 1] >= 33 && (unsigned char)rem[off + 1] <= 124 &&
        (unsigned char)rem[off + 2] >= 33 && (unsigned char)rem[off + 2] <= 124)
    {
      int alt_m = ((int)((unsigned char)rem[off] - 33) * 8281) +
                  ((int)((unsigned char)rem[off + 1] - 33) * 91) +
                  ((int)((unsigned char)rem[off + 2] - 33)) - 10000;
      state->aprs_altitude_m = (double)alt_m;
      state->aprs_has_alt    = 1;
      rem     += (off + 4);
      rem_len -= (off + 4);
      break;
    }
  }

  char raw_cmt[180];
  aprs_sanitize_ascii(rem, rem_len, raw_cmt, sizeof(raw_cmt));
  if (raw_cmt[0] != '\0')
    snprintf(state->aprs_comment, sizeof(state->aprs_comment), "[%s] %s", status_str, raw_cmt);
  else
    snprintf(state->aprs_comment, sizeof(state->aprs_comment), "[%s]", status_str);
}

/* Parse Raw NMEA `$GPRMC` or `$GPGGA` sentences */
static void
aprs_parse_nmea(dsd_state *state, const char *info, int info_len)
{
  snprintf(state->aprs_type, sizeof(state->aprs_type), "NMEA GPS");
  char buf[256];
  aprs_sanitize_ascii(info, info_len, buf, sizeof(buf));
  snprintf(state->aprs_comment, sizeof(state->aprs_comment), "%s", buf);

  /* Tokenize comma-separated fields */
  char copy[256];
  snprintf(copy, sizeof(copy), "%s", buf);
  char *fields[20];
  int nf = 0;
  char *p = copy;
  while (nf < 20 && p != NULL)
  {
    fields[nf++] = p;
    char *comma = strchr(p, ',');
    if (comma != NULL)
    {
      *comma = '\0';
      p = comma + 1;
    }
    else
    {
      p = NULL;
    }
  }

  if (nf >= 7 && (strncmp(fields[0], "$GPRMC", 6) == 0 || strncmp(fields[0], "$GNRMC", 6) == 0))
  {
    if (fields[2][0] == 'A' && strlen(fields[3]) >= 4 && strlen(fields[5]) >= 5)
    {
      double raw_lat = strtod(fields[3], NULL);
      int ldeg = (int)(raw_lat / 100.0);
      double lmin = raw_lat - (ldeg * 100.0);
      double lat = (double)ldeg + (lmin / 60.0);
      if (fields[4][0] == 'S') lat = -lat;

      double raw_lon = strtod(fields[5], NULL);
      int odeg = (int)(raw_lon / 100.0);
      double omin = raw_lon - (odeg * 100.0);
      double lon = (double)odeg + (omin / 60.0);
      if (fields[6][0] == 'W') lon = -lon;

      state->aprs_lat = lat;
      state->aprs_lon = lon;
      state->aprs_has_pos = 1;
      if (nf >= 8 && fields[7][0] != '\0')
        state->aprs_speed_kmh = strtod(fields[7], NULL) * 1.852;
      if (nf >= 9 && fields[8][0] != '\0')
        state->aprs_course = (int)lrint(strtod(fields[8], NULL));
    }
  }
  else if (nf >= 10 && (strncmp(fields[0], "$GPGGA", 6) == 0 || strncmp(fields[0], "$GNGGA", 6) == 0))
  {
    if (fields[6][0] >= '1' && fields[6][0] <= '8' && strlen(fields[2]) >= 4 && strlen(fields[4]) >= 5)
    {
      double raw_lat = strtod(fields[2], NULL);
      int ldeg = (int)(raw_lat / 100.0);
      double lmin = raw_lat - (ldeg * 100.0);
      double lat = (double)ldeg + (lmin / 60.0);
      if (fields[3][0] == 'S') lat = -lat;

      double raw_lon = strtod(fields[4], NULL);
      int odeg = (int)(raw_lon / 100.0);
      double omin = raw_lon - (odeg * 100.0);
      double lon = (double)odeg + (omin / 60.0);
      if (fields[5][0] == 'W') lon = -lon;

      state->aprs_lat = lat;
      state->aprs_lon = lon;
      state->aprs_has_pos = 1;
      if (fields[9][0] != '\0')
      {
        state->aprs_altitude_m = strtod(fields[9], NULL);
        state->aprs_has_alt = 1;
      }
    }
  }
}

/* Parse APRS Information Field */
static void
aprs_parse_info(dsd_state *state, const uint8_t *dest_addr, const char *info, int info_len, int depth)
{
  if (info_len <= 0 || depth > 2)
    return;

  char dti = info[0];

  switch (dti)
  {
    case '!':
    case '=':
      /* Position without timestamp */
      if (info_len > 1 && info[1] == '!')
      {
        snprintf(state->aprs_type, sizeof(state->aprs_type), "Ultimeter WX");
        aprs_sanitize_ascii(info, info_len, state->aprs_comment, sizeof(state->aprs_comment));
      }
      else
      {
        snprintf(state->aprs_type, sizeof(state->aprs_type), "Position");
        aprs_parse_position_body(state, info + 1, info_len - 1);
      }
      break;

    case '/':
    case '@':
      /* Position with 7-char timestamp (DHMz / DHMl / HMSh) */
      snprintf(state->aprs_type, sizeof(state->aprs_type), "Pos+Time");
      if (info_len > 8)
        aprs_parse_position_body(state, info + 8, info_len - 8);
      else
        aprs_sanitize_ascii(info + 1, info_len - 1, state->aprs_comment, sizeof(state->aprs_comment));
      break;

    case '\'':
    case '`':
    case 0x1C:
    case 0x1D:
      /* Mic-E Compressed Position */
      aprs_parse_mice(state, dest_addr, info, info_len);
      break;

    case ':':
      /* Message / Bulletin / Ack / Telemetry Definition */
      snprintf(state->aprs_type, sizeof(state->aprs_type), "Message");
      if (info_len >= 11 && info[10] == ':')
      {
        char addressee[10];
        int alen = 0;
        for (int i = 1; i < 10; i++)
        {
          if (info[i] != ' ' && info[i] >= 0x21 && info[i] <= 0x7E)
            addressee[alen++] = info[i];
        }
        addressee[alen] = '\0';

        char msg_text[180];
        aprs_sanitize_ascii(info + 11, info_len - 11, msg_text, sizeof(msg_text));
        if (strncmp(addressee, "BLN", 3) == 0)
          snprintf(state->aprs_type, sizeof(state->aprs_type), "Bulletin");
        snprintf(state->aprs_comment, sizeof(state->aprs_comment), "To %s: %s", addressee, msg_text);
      }
      else
      {
        aprs_sanitize_ascii(info + 1, info_len - 1, state->aprs_comment, sizeof(state->aprs_comment));
      }
      break;

    case ';':
      /* Object: `;NAME     *DDHHMMzLAT/LON...` (18 chars before position) */
      snprintf(state->aprs_type, sizeof(state->aprs_type), "Object");
      if (info_len > 18 && (info[10] == '*' || info[10] == '_'))
      {
        char obj_name[10];
        memcpy(obj_name, info + 1, 9);
        obj_name[9] = '\0';
        for (int i = 8; i >= 0 && obj_name[i] == ' '; i--)
          obj_name[i] = '\0';
        if (obj_name[0] != '\0')
          state->aprs_src_id = aprs_callsign_to_lrrp_id(obj_name);

        aprs_parse_position_body(state, info + 18, info_len - 18);
        char prev_cmt[160];
        snprintf(prev_cmt, sizeof(prev_cmt), "%s", state->aprs_comment);
        snprintf(state->aprs_comment, sizeof(state->aprs_comment), "Obj[%s] %s", obj_name, prev_cmt);
      }
      else
      {
        aprs_sanitize_ascii(info + 1, info_len - 1, state->aprs_comment, sizeof(state->aprs_comment));
      }
      break;

    case ')':
      /* Item: `)NAME!LAT/LON...` or `)NAME_LAT/LON...` (3..9 char name) */
      snprintf(state->aprs_type, sizeof(state->aprs_type), "Item");
      {
        int sep = -1;
        for (int i = 1; i <= 10 && i < info_len; i++)
        {
          if (info[i] == '!' || info[i] == '_')
          {
            sep = i;
            break;
          }
        }
        if (sep > 1 && sep + 1 < info_len)
        {
          char item_name[12];
          int nlen = sep - 1;
          if (nlen > 9) nlen = 9;
          memcpy(item_name, info + 1, nlen);
          item_name[nlen] = '\0';
          for (int i = nlen - 1; i >= 0 && item_name[i] == ' '; i--)
            item_name[i] = '\0';
          if (item_name[0] != '\0')
            state->aprs_src_id = aprs_callsign_to_lrrp_id(item_name);

          aprs_parse_position_body(state, info + sep + 1, info_len - (sep + 1));
          char prev_cmt[160];
          snprintf(prev_cmt, sizeof(prev_cmt), "%s", state->aprs_comment);
          snprintf(state->aprs_comment, sizeof(state->aprs_comment), "Item[%s] %s", item_name, prev_cmt);
        }
        else
        {
          aprs_sanitize_ascii(info + 1, info_len - 1, state->aprs_comment, sizeof(state->aprs_comment));
        }
      }
      break;

    case '_':
      /* Positionless Weather Report */
      snprintf(state->aprs_type, sizeof(state->aprs_type), "Weather");
      snprintf(state->aprs_symbol, sizeof(state->aprs_symbol), "/_");
      snprintf(state->aprs_symbol_desc, sizeof(state->aprs_symbol_desc), "Weather Station");
      aprs_parse_weather_fields(state, info + 1);
      aprs_sanitize_ascii(info + 1, info_len - 1, state->aprs_comment, sizeof(state->aprs_comment));
      break;

    case '>':
      /* Status Report */
      snprintf(state->aprs_type, sizeof(state->aprs_type), "Status");
      aprs_sanitize_ascii(info + 1, info_len - 1, state->aprs_comment, sizeof(state->aprs_comment));
      break;

    case 'T':
      /* Telemetry Report (`T#...`) */
      snprintf(state->aprs_type, sizeof(state->aprs_type), "Telemetry");
      aprs_sanitize_ascii(info, info_len, state->aprs_comment, sizeof(state->aprs_comment));
      break;

    case '$':
      /* Raw NMEA GPS sentence */
      aprs_parse_nmea(state, info, info_len);
      break;

    case '?':
      /* Query */
      snprintf(state->aprs_type, sizeof(state->aprs_type), "Query");
      aprs_sanitize_ascii(info + 1, info_len - 1, state->aprs_comment, sizeof(state->aprs_comment));
      break;

    case '<':
      /* Station Capabilities */
      snprintf(state->aprs_type, sizeof(state->aprs_type), "Capabilities");
      aprs_sanitize_ascii(info + 1, info_len - 1, state->aprs_comment, sizeof(state->aprs_comment));
      break;

    case '}':
      /* Third-Party Bridged Packet: `}SRC>DST,PATH:PAYLOAD` */
      snprintf(state->aprs_type, sizeof(state->aprs_type), "3rd-Party");
      {
        const char *hdr = info + 1;
        const char *colon = (const char *)memchr(hdr, ':', info_len - 1);
        if (colon != NULL && (colon + 1) < (info + info_len))
        {
          const char *gt = (const char *)memchr(hdr, '>', (size_t)(colon - hdr));
          uint8_t inner_dest[7];
          memcpy(inner_dest, dest_addr, 7);

          if (gt != NULL && gt > hdr)
          {
            /* Extract inner originating station callsign (`SRC`) */
            size_t slen = (size_t)(gt - hdr);
            if (slen >= sizeof(state->aprs_src))
              slen = sizeof(state->aprs_src) - 1;
            memcpy(state->aprs_src, hdr, slen);
            state->aprs_src[slen] = '\0';
            state->aprs_src_id = aprs_callsign_to_lrrp_id(state->aprs_src);

            /* Extract inner destination callsign (`DST`) up to `,` or `:` */
            const char *dst_start = gt + 1;
            const char *comma = (const char *)memchr(dst_start, ',', (size_t)(colon - dst_start));
            const char *dst_end = (comma != NULL) ? comma : colon;
            size_t dlen = (size_t)(dst_end - dst_start);
            if (dlen > 0)
            {
              if (dlen >= sizeof(state->aprs_dst))
                dlen = sizeof(state->aprs_dst) - 1;
              memcpy(state->aprs_dst, dst_start, dlen);
              state->aprs_dst[dlen] = '\0';

              /* Build shifted 7-byte AX.25 destination for inner Mic-E decoding */
              const char *dash = (const char *)memchr(dst_start, '-', dlen);
              size_t call_len = (dash != NULL) ? (size_t)(dash - dst_start) : dlen;
              for (int i = 0; i < 6; i++)
              {
                char ch = (i < (int)call_len) ? dst_start[i] : ' ';
                inner_dest[i] = (uint8_t)(((unsigned char)ch) << 1);
              }
              inner_dest[6] = 0x60;
            }
          }

          int sub_len = (int)((info + info_len) - (colon + 1));
          aprs_parse_info(state, inner_dest, colon + 1, sub_len, depth + 1);
        }
        else
        {
          aprs_sanitize_ascii(info + 1, info_len - 1, state->aprs_comment, sizeof(state->aprs_comment));
        }
      }
      break;

    default:
      snprintf(state->aprs_type, sizeof(state->aprs_type), "Packet");
      aprs_sanitize_ascii(info, info_len, state->aprs_comment, sizeof(state->aprs_comment));
      break;
  }
}

void
processAPRS(dsd_opts *opts, dsd_state *state)
{
  if (state == NULL || g_aprs.fifo_count <= 0)
  {
    if (state != NULL)
      state->aprs_frame_ready = 0;
    return;
  }

  aprs_queued_frame_t qf = g_aprs.fifo[g_aprs.fifo_head];
  g_aprs.fifo_head = (g_aprs.fifo_head + 1) % APRS_FIFO_DEPTH;
  g_aprs.fifo_count--;
  state->aprs_frame_ready = (g_aprs.fifo_count > 0) ? 1 : 0;

  /* Reset per-packet display fields before parsing */
  state->aprs_src_id         = 0;
  state->aprs_src[0]         = '\0';
  state->aprs_dst[0]         = '\0';
  state->aprs_path[0]        = '\0';
  state->aprs_type[0]        = '\0';
  state->aprs_symbol[0]      = '\0';
  state->aprs_symbol_desc[0] = '\0';
  state->aprs_lat            = 0.0;
  state->aprs_lon            = 0.0;
  state->aprs_has_pos        = 0;
  state->aprs_speed_kmh      = 0.0;
  state->aprs_course         = -1;
  state->aprs_altitude_m     = 0.0;
  state->aprs_has_alt        = 0;
  state->aprs_wx_str[0]      = '\0';
  state->aprs_comment[0]     = '\0';
  state->aprs_tnc2[0]        = '\0';
  state->aprs_baud           = qf.baud;

  state->aprs_pkt_count++;
  state->aprs_last_pkt_time = time(NULL);
  state->last_cc_sync_time  = state->aprs_last_pkt_time;
  state->aprs_peak = qf.peak;
  state->max = qf.peak;
  state->min = -qf.peak;

  /* Parse AX.25 Address Header */
  const uint8_t *buf = qf.data;
  int len = qf.len - 2; /* Exclude 2-byte FCS */

  aprs_format_callsign(buf + 0, state->aprs_dst, sizeof(state->aprs_dst), NULL);
  aprs_format_callsign(buf + 7, state->aprs_src, sizeof(state->aprs_src), NULL);

  int pos = 14;
  size_t path_off = 0;
  int digi_idx = 0;
  while ((buf[pos - 1] & 0x01) == 0 && (pos + 7) <= len && digi_idx < 8)
  {
    char digi[16];
    int hbit = 0;
    aprs_format_callsign(buf + pos, digi, sizeof(digi), &hbit);
    if (path_off < sizeof(state->aprs_path) - 2)
    {
      path_off += snprintf(state->aprs_path + path_off, sizeof(state->aprs_path) - path_off,
                           "%s%s%s", (digi_idx > 0) ? "," : "", digi, hbit ? "*" : "");
    }
    pos += 7;
    digi_idx++;
  }

  /* Control & PID bytes */
  uint8_t ctrl = (pos < len) ? buf[pos++] : 0x00;
  uint8_t pid  = 0x00;
  if ((ctrl & 0x03) == 0x03 || (ctrl & 0x01) == 0x00)
  {
    /* UI-frame (0x03) or I-frame has a PID byte */
    if (pos < len)
      pid = buf[pos++];
  }

  const char *info_ptr = (const char *)(buf + pos);
  int info_len = (pos < len) ? (len - pos) : 0;

  /* Trim trailing CR/LF from info field */
  while (info_len > 0 && (info_ptr[info_len - 1] == '\r' || info_ptr[info_len - 1] == '\n'))
    info_len--;

  /* Build TNC2 monitor string: `SRC>DST,PATH:INFO` */
  char clean_info[256];
  aprs_sanitize_ascii(info_ptr, info_len, clean_info, sizeof(clean_info));
  if (state->aprs_path[0] != '\0')
  {
    snprintf(state->aprs_tnc2, sizeof(state->aprs_tnc2), "%s>%s,%s:%s",
             state->aprs_src, state->aprs_dst, state->aprs_path, clean_info);
  }
  else
  {
    snprintf(state->aprs_tnc2, sizeof(state->aprs_tnc2), "%s>%s:%s",
             state->aprs_src, state->aprs_dst, clean_info);
  }

  /* Copy info field into a guaranteed null-terminated buffer */
  char info_buf[APRS_MAX_FRAME_LEN + 1];
  if (info_len > APRS_MAX_FRAME_LEN)
    info_len = APRS_MAX_FRAME_LEN;
  if (info_len > 0)
    memcpy(info_buf, info_ptr, (size_t)info_len);
  info_buf[info_len] = '\0';

  /* Default LRRP / Event History numeric IDs from outer AX.25 callsigns;
   * aprs_parse_info() may override state->aprs_src_id for Objects, Items, or 3rd-party packets. */
  state->aprs_src_id = aprs_callsign_to_lrrp_id(state->aprs_src);

  /* Parse APRS Info Field if UI frame with PID 0xF0 (No Layer 3) */
  if (info_len > 0 && (pid == 0xF0 || ctrl == 0x03))
  {
    aprs_parse_info(state, buf + 0, info_buf, info_len, 0);
  }
  else
  {
    snprintf(state->aprs_type, sizeof(state->aprs_type), "AX.25 (Ctrl=%02X)", ctrl);
    snprintf(state->aprs_comment, sizeof(state->aprs_comment), "%s", clean_info);
  }

  uint32_t src_id = (state->aprs_src_id != 0) ? state->aprs_src_id : aprs_callsign_to_lrrp_id(state->aprs_src);
  uint32_t dst_id = aprs_callsign_to_lrrp_id(state->aprs_dst);
  state->aprs_src_id = src_id;

  /* Console Output */
  char *timestr = getTimeF(time(NULL));
  char *datestr = getDateF(time(NULL));
  fprintf(stderr, "\n%s", KCYN);
  fprintf(stderr, " %s %s APRS (%d bps) [FCS %04X] #%u\n",
          datestr, timestr, state->aprs_baud, qf.fcs, state->aprs_pkt_count);
  fprintf(stderr, " %sTNC2: %s%s\n", KGRN, state->aprs_tnc2, KNRM);

  fprintf(stderr, " %sFrom: %-9s (ID: %08u)  To: %-9s",
          KYEL, state->aprs_src, src_id, state->aprs_dst);
  if (state->aprs_path[0] != '\0')
    fprintf(stderr, "  Via: %s", state->aprs_path);
  fprintf(stderr, "\n Type: %-10s", state->aprs_type);
  if (state->aprs_symbol[0] != '\0')
    fprintf(stderr, "  Symbol: %s (%s)", state->aprs_symbol, state->aprs_symbol_desc);
  fprintf(stderr, "%s\n", KNRM);

  if (state->aprs_has_pos)
  {
    fprintf(stderr, " %sGPS:  Lat: %.5f (%c)  Lon: %.5f (%c)",
            KMAG,
            fabs(state->aprs_lat), (state->aprs_lat >= 0.0) ? 'N' : 'S',
            fabs(state->aprs_lon), (state->aprs_lon >= 0.0) ? 'E' : 'W');
    if (state->aprs_speed_kmh > 0.0 || state->aprs_course >= 0)
    {
      fprintf(stderr, "  Spd: %.1f km/h (%.1f mph)",
              state->aprs_speed_kmh, state->aprs_speed_kmh * 0.621371);
      if (state->aprs_course >= 0)
        fprintf(stderr, "  Cse: %03d deg", state->aprs_course);
    }
    if (state->aprs_has_alt)
    {
      fprintf(stderr, "  Alt: %.0f m (%.0f ft)",
              state->aprs_altitude_m, state->aprs_altitude_m * 3.28084);
    }
    fprintf(stderr, "%s\n", KNRM);
  }

  if (state->aprs_wx_str[0] != '\0')
  {
    fprintf(stderr, " %sWX:   %s%s\n", KCYN, state->aprs_wx_str, KNRM);
  }

  if (state->aprs_comment[0] != '\0')
  {
    fprintf(stderr, " %sInfo: %s%s\n", KWHT, state->aprs_comment, KNRM);
  }

  if (opts != NULL && opts->payload == 1)
  {
    fprintf(stderr, " %sAX.25 Payload [%d bytes]:", KCYN, qf.len);
    for (int i = 0; i < qf.len; i++)
    {
      if ((i % 16) == 0)
        fprintf(stderr, "\n  ");
      fprintf(stderr, "%02X ", qf.data[i]);
    }
    fprintf(stderr, "%s\n", KNRM);
  }

  free(timestr);
  free(datestr);

  /* Append position to LRRP file (`-L`) if configured.
   * DSDPlus LRRP.exe requires:
   *   YYYY/MM/DD\tHH:MM:SS\t%08u\t%.6lf\t%.6lf\t%d\t %d\t \n
   * where column 3 is a strictly numeric Radio ID. */
  if (opts != NULL && opts->lrrp_file_output == 1 && opts->lrrp_out_file[0] != '\0' && state->aprs_has_pos)
  {
    char *ts = getTimeC();
    char *ds = getDateS();
    int s = (int)lrint(state->aprs_speed_kmh);
    int a = (state->aprs_course >= 0) ? state->aprs_course : 0;
    FILE *pFile = fopen(opts->lrrp_out_file, "a");
    if (pFile != NULL)
    {
      fprintf(pFile, "%s\t", ds);
      fprintf(pFile, "%s\t", ts);
      fprintf(pFile, "%08u\t", src_id);
      fprintf(pFile, "%.6lf\t", state->aprs_lat);
      fprintf(pFile, "%.6lf\t", state->aprs_lon);
      fprintf(pFile, "%d\t ", s);
      fprintf(pFile, "%d\t ", a);
      fprintf(pFile, "\n");
      fflush(pFile);
      fclose(pFile);
    }
    free(ts);
    free(ds);
  }

  /* Record in DSD-FME Event History (`-N` ncurses panel and `-J` event log) */
  if (opts != NULL && state->event_history_s != NULL)
  {
    uint32_t src_hash = (src_id != 0) ? src_id : 0xFFFFFFU;
    uint32_t dst_hash = dst_id;

    /* Format a single concise Event History line (no extra TNC2 or GPS sub-lines) */
    char data_summary[128];
    if (state->aprs_wx_str[0] != '\0')
    {
      snprintf(data_summary, sizeof(data_summary), "%s>%s [WX] %s",
               state->aprs_src, state->aprs_dst, state->aprs_wx_str);
    }
    else if (state->aprs_has_pos)
    {
      snprintf(data_summary, sizeof(data_summary), "%s>%s [%s] %.4f, %.4f",
               state->aprs_src, state->aprs_dst, state->aprs_type, state->aprs_lat, state->aprs_lon);
    }
    else if (state->aprs_comment[0] != '\0')
    {
      snprintf(data_summary, sizeof(data_summary), "%s>%s [%s] %s",
               state->aprs_src, state->aprs_dst, state->aprs_type, state->aprs_comment);
    }
    else
    {
      snprintf(data_summary, sizeof(data_summary), "%s>%s [%s]",
               state->aprs_src, state->aprs_dst, state->aprs_type);
    }

    watchdog_event_datacall(opts, state, src_hash, dst_hash, data_summary, 0);
    state->lastsrc = 0;
    state->lasttg  = 0;
    SNDFILE *saved_wav = opts->wav_out_f;
    opts->wav_out_f = NULL;
    watchdog_event_history(opts, state, 0);
    opts->wav_out_f = saved_wav;
  }

  if (opts != NULL && opts->use_ncurses_terminal == 1)
  {
    ncursesPrinter(opts, state);
  }
}
