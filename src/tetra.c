/*-------------------------------------------------------------------------------
 * tetra.c
 * TETRA (Terrestrial Trunked Radio) Voice & Data Decoder for DSD-FME
 * Ported from TetraEar reference implementation (Lower MAC, Upper MAC, LLC,
 * MLE, CMCE, MM, SDS, LIP Location, TEA1-4 Decryption, MCC/MNC Lookup, TCH/S ACELP Voice)
 *-----------------------------------------------------------------------------*/

#include "dsd.h"

#define TETRA_CRC_OK 0x1D0F
#define TETRA_SCRAMB_INIT_DEFAULT 3U

/* Training sequences in bits */
/* Y_BITS (SYNC burst, 38 bits = 19 dibits at bit offset 214, symbol offset 107) */
static const uint8_t TETRA_Y_DIBITS[19] = {
  3, 0, 0, 1, 2, 1, 3, 0, 3, 2, 2, 1, 3, 0, 0, 1, 2, 1, 3
};

/* N_BITS (NORM_1 burst, 22 bits = 11 dibits at bit offset 244, symbol offset 122) */
static const uint8_t TETRA_N_DIBITS[11] = {
  3, 1, 0, 0, 3, 2, 2, 1, 3, 1, 0
};

/* P_BITS (NORM_2 burst, 22 bits = 11 dibits at bit offset 244, symbol offset 122) */
static const uint8_t TETRA_P_DIBITS[11] = {
  1, 3, 2, 2, 1, 0, 0, 3, 1, 3, 2
};

/* Q_BITS (NORM_3 burst, 22 bits = 11 dibits at bit offset 244, symbol offset 122) */
static const uint8_t TETRA_Q_DIBITS[11] = {
  2, 3, 1, 3, 0, 0, 1, 2, 2, 3, 1
};

/* X_BITS (Control Uplink Burst CB, 30 bits = 15 dibits, ETSI EN 300 392-2 Clause 9.4.4.3.4) */
static const uint8_t TETRA_X_DIBITS[15] = {
  2, 1, 3, 1, 0, 0, 3, 2, 2, 1, 3, 1, 0, 0, 3
};

/* MCC to Country Lookup Table (ASCII clean for console & ncurses) */
typedef struct {
  uint16_t mcc;
  const char *country;
} tetra_mcc_entry_t;

static const tetra_mcc_entry_t TETRA_MCC_TABLE[] = {
  {202, "Greece"}, {204, "Netherlands"}, {206, "Belgium"}, {208, "France"},
  {212, "Monaco"}, {213, "Andorra"}, {214, "Spain"}, {216, "Hungary"},
  {218, "Bosnia"}, {219, "Croatia"}, {220, "Serbia"}, {222, "Italy"},
  {225, "Vatican"}, {226, "Romania"}, {228, "Switzerland"}, {230, "Czechia"},
  {231, "Slovakia"}, {232, "Austria"}, {234, "UK"}, {235, "UK"},
  {238, "Denmark"}, {240, "Sweden"}, {242, "Norway"}, {244, "Finland"},
  {246, "Lithuania"}, {247, "Latvia"}, {248, "Estonia"}, {250, "Russia"},
  {255, "Ukraine"}, {257, "Belarus"}, {259, "Moldova"}, {260, "Poland"},
  {262, "Germany"}, {266, "Gibraltar"}, {268, "Portugal"}, {270, "Luxembourg"},
  {272, "Ireland"}, {274, "Iceland"}, {276, "Albania"}, {278, "Malta"},
  {280, "Cyprus"}, {282, "Georgia"}, {283, "Armenia"}, {284, "Bulgaria"},
  {286, "Turkey"}, {288, "Faroe Is."}, {290, "Greenland"}, {292, "San Marino"},
  {293, "Slovenia"}, {294, "N. Macedonia"}, {295, "Liechtenstein"}, {297, "Montenegro"},
  {302, "Canada"}, {310, "USA"}, {311, "USA"}, {312, "USA"},
  {313, "USA"}, {316, "USA"}, {330, "Puerto Rico"}, {334, "Mexico"},
  {400, "Azerbaijan"}, {401, "Kazakhstan"}, {404, "India"}, {405, "India"},
  {410, "Pakistan"}, {415, "Lebanon"}, {416, "Jordan"}, {419, "Kuwait"},
  {420, "Saudi Arabia"}, {422, "Oman"}, {424, "UAE"}, {425, "Israel"},
  {426, "Bahrain"}, {427, "Qatar"}, {432, "Iran"}, {440, "Japan"},
  {441, "Japan"}, {450, "South Korea"}, {452, "Vietnam"}, {454, "Hong Kong"},
  {455, "Macau"}, {460, "China"}, {466, "Taiwan"}, {502, "Malaysia"},
  {505, "Australia"}, {510, "Indonesia"}, {515, "Philippines"}, {520, "Thailand"},
  {525, "Singapore"}, {530, "New Zealand"}, {602, "Egypt"}, {603, "Algeria"},
  {604, "Morocco"}, {605, "Tunisia"}, {655, "South Africa"}, {716, "Peru"},
  {722, "Argentina"}, {724, "Brazil"}, {730, "Chile"}, {732, "Colombia"},
  {734, "Venezuela"}, {748, "Uruguay"}, {0, NULL}
};

static const char *tetra_lookup_country(uint16_t mcc)
{
  for (int i = 0; TETRA_MCC_TABLE[i].country != NULL; i++)
  {
    if (TETRA_MCC_TABLE[i].mcc == mcc)
      return TETRA_MCC_TABLE[i].country;
  }
  return "Unknown";
}

static uint32_t tetra_bits_to_uint(const uint8_t *bits, int offset, int len)
{
  uint32_t val = 0;
  for (int i = 0; i < len; i++)
  {
    val = (val << 1) | (bits[offset + i] & 1U);
  }
  return val;
}

static int32_t tetra_bits_to_int2s(const uint8_t *bits, int offset, int len)
{
  uint32_t val = tetra_bits_to_uint(bits, offset, len);
  if (val & (1U << (len - 1)))
  {
    return (int32_t)(val - (1U << len));
  }
  return (int32_t)val;
}

/*-------------------------------------------------------------------------------
 * Lower MAC Scrambling, Deinterleaving, Depuncturing, Viterbi, CRC16
 *-----------------------------------------------------------------------------*/

static uint32_t tetra_scramb_get_init(uint16_t mcc, uint16_t mnc, uint8_t colour)
{
  if (colour == 0 && mcc == 0 && mnc == 0)
    return TETRA_SCRAMB_INIT_DEFAULT;
  uint32_t val = ((uint32_t)(colour & 0x3FU)) |
                 ((uint32_t)(mnc & 0x3FFFU) << 6) |
                 ((uint32_t)(mcc & 0x3FFU) << 20);
  return (val << 2) | 3U;
}

/* ETSI EN 300 396-2 Clause 8.2.5.2: DMO Scrambling seed from 6 LSBs of MNI + 24-bit Source SSI */
static uint32_t tetra_dmo_scramb_get_init(uint32_t mni, uint32_t src_ssi)
{
  if (mni == 0 && src_ssi == 0)
    return TETRA_SCRAMB_INIT_DEFAULT;
  uint32_t val = (((mni & 0x3FU) << 24) | (src_ssi & 0xFFFFFFU));
  return (val << 2) | 3U;
}

/* ETSI EN 300 396-2 Clause 9.4.4.2: Check 12-bit DMO Preamble P1 (0x323), P2 (0x9A9), or P3 (0x147) */
static int tetra_check_dmo_preamble(const uint8_t *bits, int offset, uint16_t expected12, int max_errs)
{
  uint16_t got = (uint16_t)tetra_bits_to_uint(bits, offset, 12);
  uint16_t diff = (uint16_t)((got ^ expected12) & 0xFFFU);
  int errs = 0;
  for (int i = 0; i < 12; i++)
    errs += (diff >> i) & 1;
  return (errs <= max_errs) ? 1 : 0;
}

static inline uint8_t tetra_lfsr_next(uint32_t *lfsr_io)
{
  uint32_t lfsr = *lfsr_io;
  uint32_t bit = ((lfsr >> 0) ^ (lfsr >> 6) ^ (lfsr >> 9) ^ (lfsr >> 10) ^
                  (lfsr >> 16) ^ (lfsr >> 20) ^ (lfsr >> 21) ^ (lfsr >> 22) ^
                  (lfsr >> 24) ^ (lfsr >> 25) ^ (lfsr >> 27) ^ (lfsr >> 28) ^
                  (lfsr >> 30) ^ (lfsr >> 31)) & 1U;
  *lfsr_io = (lfsr >> 1) | (bit << 31);
  return (uint8_t)bit;
}

static void tetra_scramb_bits(uint32_t lfsr_init, const uint8_t *in_bits, uint8_t *out_bits, int len)
{
  uint32_t lfsr = lfsr_init;
  for (int i = 0; i < len; i++)
    out_bits[i] = (in_bits[i] & 1U) ^ tetra_lfsr_next(&lfsr);
}

static void tetra_scramb_soft(uint32_t lfsr_init, const int16_t *in_soft, int16_t *out_soft, int len)
{
  uint32_t lfsr = lfsr_init;
  for (int i = 0; i < len; i++)
    out_soft[i] = tetra_lfsr_next(&lfsr) ? (int16_t)(-in_soft[i]) : in_soft[i];
}

static void tetra_block_deinterleave_soft(const int16_t *in_soft, int16_t *out_soft, int K, int a)
{
  memset(out_soft, 0, K * sizeof(int16_t));
  for (int i = 1; i <= K; i++)
  {
    int k = 1 + ((a * i) % K);
    out_soft[i - 1] = in_soft[k - 1];
  }
}

static void tetra_rcpc_depunct_2_3_soft(const int16_t *in_soft, int in_len, int16_t *out_soft, int out_len)
{
  /* 1-based P_RATE2_3 from ETSI EN 300 392-2 / osmo-tetra / TetraEar:
   * P_RATE2_3 = {0, 1, 2, 5}, t = 3, period = 8 */
  static const int P_rate2_3[4] = {0, 1, 2, 5};
  memset(out_soft, 0, out_len * sizeof(int16_t)); /* 0 = punctured erasure */
  for (int j = 1; j <= in_len; j++)
  {
    int i = j;
    int k = 8 * ((i - 1) / 3) + P_rate2_3[i - 3 * ((i - 1) / 3)];
    if (k > 0 && k <= out_len)
    {
      out_soft[k - 1] = in_soft[j - 1];
    }
  }
}

static uint16_t tetra_crc16_ccitt_bits(const uint8_t *bits, int num_bits)
{
  uint16_t crc = 0xFFFFU;
  for (int i = 0; i < num_bits; i++)
  {
    uint16_t bit = bits[i] & 1U;
    if (((crc >> 15) & 1U) ^ bit)
      crc = (uint16_t)(((crc << 1) ^ 0x1021U) & 0xFFFFU);
    else
      crc = (uint16_t)((crc << 1) & 0xFFFFU);
  }
  return crc;
}

static int tetra_viterbi_decode_cch_soft(const int16_t *depunct_soft, int n_steps, int payload_len, uint8_t *out_bits)
{
  UNUSED(payload_len);
  /* Exact CONV_CCH_NEXT_OUTPUT table from osmo-tetra / TetraEar (output_order = [3, 2, 1, 0]) */
  static const uint8_t CONV_CCH_NEXT_OUTPUT[16][2] = {
    { 0, 15}, {11,  4}, { 6,  9}, {13,  2},
    { 5, 10}, {14,  1}, { 3, 12}, { 8,  7},
    {15,  0}, { 4, 11}, { 9,  6}, { 2, 13},
    {10,  5}, { 1, 14}, {12,  3}, { 7,  8}
  };

  if (n_steps <= 0 || n_steps > 300)
    return 0;

  int32_t metrics[16];
  int32_t new_metrics[16];
  uint8_t prev_state[300][16];
  uint8_t prev_bit[300][16];

  metrics[0] = 0;
  for (int s = 1; s < 16; s++)
    metrics[s] = -100000000;

  for (int t = 0; t < n_steps; t++)
  {
    const int16_t *soft_slice = &depunct_soft[t * 4];
    for (int s = 0; s < 16; s++)
      new_metrics[s] = -100000000;

    for (int s = 0; s < 16; s++)
    {
      if (metrics[s] <= -50000000)
        continue;
      for (int in_bit = 0; in_bit < 2; in_bit++)
      {
        int next_s = ((s << 1) | in_bit) & 0x0F;
        uint8_t out_sym = CONV_CCH_NEXT_OUTPUT[s][in_bit];
        int32_t branch = 0;
        for (int k = 0; k < 4; k++)
        {
          int32_t sv = soft_slice[k];
          if (sv == 0)
            continue;
          int expected = (out_sym >> (3 - k)) & 1;
          branch += (expected == 0) ? sv : -sv;
        }
        int32_t cand = metrics[s] + branch;
        if (cand > new_metrics[next_s])
        {
          new_metrics[next_s] = cand;
          prev_state[t][next_s] = (uint8_t)s;
          prev_bit[t][next_s] = (uint8_t)in_bit;
        }
      }
    }
    memcpy(metrics, new_metrics, sizeof(metrics));
  }

  /* Try state 0 first (tail bits force encoder to state 0) */
  int state_idx = 0;
  for (int t = n_steps - 1; t >= 0; t--)
  {
    out_bits[t] = prev_bit[t][state_idx];
    state_idx = prev_state[t][state_idx];
  }
  return n_steps;
}

static void tetra_decode_aach(dsd_state *state, const uint8_t *raw30, uint32_t scramb_init, int is_sb)
{
  uint8_t descramb[30];
  uint8_t aach[14];
  tetra_scramb_bits(scramb_init, raw30, descramb, 30);

  /* Per ETSI EN 300 392-2 Clause 8.2.3.2 & TetraEar lower_mac.py, the (30, 14)
   * Reed-Muller code is systematic: the first 14 descrambled bits are the 14
   * ACCESS-ASSIGN information bits directly. */
  memcpy(aach, descramb, 14);

  uint8_t hdr = (uint8_t)((aach[0] << 1) | aach[1]);
  uint8_t f1 = (uint8_t)tetra_bits_to_uint(aach, 2, 6);
  uint8_t f2 = (uint8_t)tetra_bits_to_uint(aach, 8, 6);

  if (is_sb || state->tetra_frame == 18)
  {
    state->tetra_dl_usage = 1; /* Control frame 18 */
    state->tetra_ul_usage = 1;
  }
  else
  {
    /* Per ETSI EN 300 392-2 Table 21.75 (ACCESS-ASSIGN PDU on frames 1..17):
     * hdr == 0 (00): Downlink is Common control (MCCH); f1=Access1, f2=Access2
     * hdr == 1 (01): Downlink usage marker = f1; Uplink = Assigned only (f2=Access)
     * hdr == 2 (10): Downlink usage marker = f1; Uplink = Common & Assigned (f2=Access)
     * hdr == 3 (11): Downlink usage marker = f1; Uplink usage marker = f2
     * Usage marker values: 0=Unallocated (Idle), 1=Assigned control, 2=Common control,
     *                      3=Reserved, 4..63=Assigned traffic usage marker */
    if (hdr == 0)
    {
      /* Guard against a 1-bit error in hdr on an all-zero Unallocated AACH (01/10 000000 000000) */
      state->tetra_dl_usage = (f1 == 0 && f2 == 0) ? 0 : 2; /* Common Control (MCCH) */
      state->tetra_ul_usage = (f1 == 0 && f2 == 0) ? 0 : 2;
    }
    else if (hdr == 1)
    {
      state->tetra_dl_usage = f1;
      state->tetra_ul_usage = 1;
    }
    else if (hdr == 2)
    {
      state->tetra_dl_usage = f1;
      state->tetra_ul_usage = 2;
    }
    else
    {
      state->tetra_dl_usage = f1;
      state->tetra_ul_usage = f2;
    }
  }
}

/*-------------------------------------------------------------------------------
 * GSM 7-bit Unpacking, SDS Text, and LIP Location Decoder
 *-----------------------------------------------------------------------------*/

static void tetra_unpack_gsm7bit(const uint8_t *bytes, int len_bytes, int skip_chars, char *out_str, int max_out)
{
  static const char gsm7_basic[128] = {
    '@', 'L', '$', 'Y', 'e', 'e', 'u', 'i', 'o', 'c', '\n', 'O', 'o', '\r', 'A', 'a',
    'D', '_', 'F', 'G', 'L', 'O', 'P', 'Y', 'S', 'O', 'X', ' ', 'A', 'a', 'B', 'E',
    ' ', '!', '"', '#', 'o', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?',
    '!', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
    'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'A', 'O', 'N', 'U', 'S',
    '?', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
    'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', 'a', 'o', 'n', 'u', 'a'
  };
  int max_septets = (len_bytes * 8) / 7;
  int out_idx = 0;
  for (int i = skip_chars; i < max_septets && out_idx < max_out - 1; i++)
  {
    int bit_off = i * 7;
    int byte_off = bit_off / 8;
    int shift = bit_off % 8;
    if (byte_off >= len_bytes)
      break;
    uint16_t w = bytes[byte_off];
    if (byte_off + 1 < len_bytes)
      w |= (uint16_t)(bytes[byte_off + 1] << 8);
    uint8_t septet = (uint8_t)((w >> shift) & 0x7FU);
    char c = gsm7_basic[septet];
    if (c >= 0x20 && c <= 0x7E)
      out_str[out_idx++] = c;
  }
  out_str[out_idx] = '\0';
}

static void tetra_decode_lip(dsd_opts *opts, dsd_state *state, const uint8_t *bits, int nbits, uint32_t src_ssi)
{
  /* bits[0..7] is protocol ID 0x0A. bits[8..9] is LIP PDU type (0 = Short, 1 = Long) */
  if (nbits < 64)
    return;

  uint8_t pdu_type = (uint8_t)tetra_bits_to_uint(bits, 8, 2);
  if (pdu_type == 0) /* Short Location Report PDU */
  {
    uint8_t time_elapsed = (uint8_t)tetra_bits_to_uint(bits, 10, 2);
    int32_t lon_raw = tetra_bits_to_int2s(bits, 12, 25);
    int32_t lat_raw = tetra_bits_to_int2s(bits, 37, 24);
    uint8_t pos_err = (uint8_t)tetra_bits_to_uint(bits, 61, 3);

    double lon = (double)lon_raw * 360.0 / (double)(1 << 25);
    double lat = (double)lat_raw * 180.0 / (double)(1 << 24);

    int vel_kmh = 0;
    int dir_deg = 0;
    const char *dir_card = "N";
    static const char *DIRS[16] = {
      "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
      "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
    };
    static const char *POS_ERR_STR[8] = {
      "<2m", "<20m", "<200m", "<2km", "<20km", "<=200km", ">200km", "Unk"
    };

    if (nbits >= 75)
    {
      uint8_t h_vel = (uint8_t)tetra_bits_to_uint(bits, 64, 7);
      uint8_t d_trav = (uint8_t)tetra_bits_to_uint(bits, 71, 4);
      if (h_vel < 127)
      {
        if (h_vel <= 28)
          vel_kmh = h_vel;
        else
          vel_kmh = (int)(16.0 * pow(1.038, (double)h_vel - 13.0));
      }
      dir_card = DIRS[d_trav & 0x0F];
      dir_deg = (int)((d_trav & 0x0F) * 22.5);
    }

    if (fabs(lat) <= 90.0 && fabs(lon) <= 180.0)
    {
      uint32_t effective_src = src_ssi ? src_ssi : state->tetra_last_src;
      snprintf(state->tetra_lip_str, sizeof(state->tetra_lip_str),
               "Lat: %.5f, Lon: %.5f (%s, %dkm/h %s)",
               lat, lon, POS_ERR_STR[pos_err & 7], vel_kmh, dir_card);
      snprintf(state->dmr_lrrp_gps[0], sizeof(state->dmr_lrrp_gps[0]),
               "LIP: %.5f, %.5f (%s %dkm/h %s)",
               lat, lon, POS_ERR_STR[pos_err & 7], vel_kmh, dir_card);
      snprintf(state->event_history_s[0].Event_History_Items[0].gps_s,
               sizeof(state->event_history_s[0].Event_History_Items[0].gps_s),
               "%s", state->tetra_lip_str);

      fprintf(stderr, "\n %s", KYEL);
      fprintf(stderr, "TETRA LIP Short Report - SSI: %u Lat: %.6f Lon: %.6f Err: %s Spd: %d km/h Dir: %s (TE:%u)",
              effective_src, lat, lon, POS_ERR_STR[pos_err & 7], vel_kmh, dir_card, time_elapsed);
      fprintf(stderr, "%s", KNRM);

      if (opts->lrrp_file_output == 1 && opts->lrrp_out_file[0] != 0)
      {
        FILE *pFile = fopen(opts->lrrp_out_file, "a");
        if (pFile != NULL)
        {
          char *datestr = getDateS();
          char *timestr = getTimeC();
          fprintf(pFile, "%s\t%s\t%08u\t%.6lf\t%.6lf\t%d\t%d\n",
                  datestr ? datestr : "", timestr ? timestr : "",
                  effective_src, lat, lon, vel_kmh, dir_deg);
          fclose(pFile);
          if (timestr) free(timestr);
          if (datestr) free(datestr);
        }
      }
    }
  }
  else if (pdu_type == 1 && nbits >= 90) /* Long Location Report PDU */
  {
    /* Scan for location shape (4 bits) followed by 25-bit lon and 24-bit lat */
    int pos = 14;
    uint8_t time_type = (uint8_t)tetra_bits_to_uint(bits, 12, 2);
    if (time_type == 1) pos += 2;
    else if (time_type == 2) pos += 25;
    else if (time_type == 3) pos += 22;

    if (pos + 53 <= nbits)
    {
      uint8_t shape_type = (uint8_t)tetra_bits_to_uint(bits, pos, 4);
      pos += 4;
      if (shape_type >= 1 && shape_type <= 8 && pos + 49 <= nbits)
      {
        int32_t lon_raw = tetra_bits_to_int2s(bits, pos, 25);
        int32_t lat_raw = tetra_bits_to_int2s(bits, pos + 25, 24);
        double lon = (double)lon_raw * 360.0 / (double)(1 << 25);
        double lat = (double)lat_raw * 180.0 / (double)(1 << 24);
        if (fabs(lat) <= 90.0 && fabs(lon) <= 180.0)
        {
          uint32_t effective_src = src_ssi ? src_ssi : state->tetra_last_src;
          snprintf(state->tetra_lip_str, sizeof(state->tetra_lip_str),
                   "Lat: %.5f, Lon: %.5f (Long LIP)", lat, lon);
          snprintf(state->dmr_lrrp_gps[0], sizeof(state->dmr_lrrp_gps[0]),
                   "LIP: %.5f, %.5f (Long)", lat, lon);
          snprintf(state->event_history_s[0].Event_History_Items[0].gps_s,
                   sizeof(state->event_history_s[0].Event_History_Items[0].gps_s),
                   "%s", state->tetra_lip_str);
          fprintf(stderr, "\n %sTETRA LIP Long Report - SSI: %u Lat: %.6f Lon: %.6f%s",
                  KYEL, effective_src, lat, lon, KNRM);

          if (opts->lrrp_file_output == 1 && opts->lrrp_out_file[0] != 0)
          {
            FILE *pFile = fopen(opts->lrrp_out_file, "a");
            if (pFile != NULL)
            {
              char *datestr = getDateS();
              char *timestr = getTimeC();
              fprintf(pFile, "%s\t%s\t%08u\t%.6lf\t%.6lf\t0\t0\n",
                      datestr ? datestr : "", timestr ? timestr : "",
                      effective_src, lat, lon);
              fclose(pFile);
              if (timestr) free(timestr);
              if (datestr) free(datestr);
            }
          }
        }
      }
    }
  }
}

static int tetra_decode_sds_text_payload(const uint8_t *bytes, int len_bytes, uint8_t coding,
                                         char *out_str, int max_out)
{
  if (len_bytes <= 0 || max_out <= 1)
  {
    if (max_out > 0)
      out_str[0] = '\0';
    return 0;
  }
  if (coding == 0x00)
  {
    tetra_unpack_gsm7bit(bytes, len_bytes, 0, out_str, max_out);
    return (int)strlen(out_str);
  }
  int j = 0;
  for (int i = 0; i < len_bytes && j < max_out - 1; i++)
  {
    if (bytes[i] >= 0x20 && bytes[i] <= 0x7E)
      out_str[j++] = (char)bytes[i];
  }
  out_str[j] = '\0';
  return j;
}

static void tetra_decode_sds_type4(dsd_opts *opts, dsd_state *state, const uint8_t *bits, int nbits, uint32_t src_ssi)
{
  if (nbits < 8)
    return;

  uint8_t proto_id = (uint8_t)tetra_bits_to_uint(bits, 0, 8);
  int nbytes = nbits / 8;
  uint8_t raw_bytes[256];
  if (nbytes > (int)sizeof(raw_bytes))
    nbytes = (int)sizeof(raw_bytes);
  for (int i = 0; i < nbytes; i++)
    raw_bytes[i] = (uint8_t)tetra_bits_to_uint(bits, i * 8, 8);

  if (proto_id == 0x0A) /* LIP Location Information Protocol */
  {
    tetra_decode_lip(opts, state, bits, nbits, src_ssi);
    return;
  }

  char text_buf[256];
  memset(text_buf, 0, sizeof(text_buf));

  if ((proto_id == 0x02 || proto_id == 0x03 || proto_id == 0x82) && nbytes >= 2)
  {
    /* Simple Text Messaging / Simple Immediate Text Messaging */
    uint8_t enc_byte = raw_bytes[1];
    uint8_t coding = enc_byte & 0x1FU;
    uint8_t has_ts = (enc_byte >> 7) & 1U;
    int start = has_ts ? 5 : 2;
    if (start < nbytes)
      tetra_decode_sds_text_payload(&raw_bytes[start], nbytes - start, coding, text_buf, sizeof(text_buf));
  }
  else if ((proto_id == 0x06 || proto_id == 0x09 || proto_id == 0x83 || proto_id == 0x89) && nbytes >= 4)
  {
    /* SDS-TL (Short Data Service - Transport Layer) */
    uint8_t b1 = raw_bytes[1];
    uint8_t msg_type = (b1 >> 4) & 0x0FU;
    if (msg_type == 0x00) /* SDS-TRANSFER */
    {
      uint8_t has_ts = (b1 >> 1) & 1U;
      int cs_idx = has_ts ? 6 : 3;
      if (cs_idx + 1 < nbytes)
      {
        uint8_t coding = raw_bytes[cs_idx] & 0x1FU;
        tetra_decode_sds_text_payload(&raw_bytes[cs_idx + 1], nbytes - (cs_idx + 1),
                                      coding, text_buf, sizeof(text_buf));
      }
    }
    else if (msg_type == 0x01)
    {
      snprintf(text_buf, sizeof(text_buf), "[SDS-REPORT]");
    }
    else if (msg_type == 0x02)
    {
      snprintf(text_buf, sizeof(text_buf), "[SDS-ACK]");
    }
  }
  else
  {
    /* Fallback: extract printable ASCII if present, otherwise hex summary */
    int j = tetra_decode_sds_text_payload(&raw_bytes[1], nbytes - 1, 0x01, text_buf, sizeof(text_buf));
    if (j < 2)
    {
      int pos = snprintf(text_buf, sizeof(text_buf), "PID:0x%02X HEX:", proto_id);
      for (int i = 1; i < nbytes && i <= 16 && pos < (int)sizeof(text_buf) - 4; i++)
        pos += snprintf(text_buf + pos, sizeof(text_buf) - pos, "%02X", raw_bytes[i]);
    }
  }

  if (text_buf[0] != '\0')
  {
    snprintf(state->tetra_sds_str, sizeof(state->tetra_sds_str), "%s", text_buf);
    snprintf(state->event_history_s[0].Event_History_Items[0].text_message,
             sizeof(state->event_history_s[0].Event_History_Items[0].text_message),
             "%s", text_buf);
    fprintf(stderr, "\n %sTETRA SDS [PID:0x%02X]: \"%s\"%s", KGRN, proto_id, text_buf, KNRM);
  }
}

/*-------------------------------------------------------------------------------
 * CMCE, MM, MLE, LLC, and Upper MAC Protocol Parsers
 *-----------------------------------------------------------------------------*/

const char *tetra_get_tea_name(const dsd_state *state, int is_encrypted)
{
  if (!is_encrypted)
    return "Clear";

  if (state->tetra_enc_alg >= 1 && state->tetra_enc_alg <= 7)
  {
    static const char *TEA_ALG_NAMES[8] = {
      "TEA?", "TEA1", "TEA2", "TEA3", "TEA4", "TEA5", "TEA6", "TEA7"
    };
    return TEA_ALG_NAMES[state->tetra_enc_alg & 7];
  }

  uint16_t mcc = state->tetra_mcc;
  uint16_t mnc = state->tetra_mnc;
  if (mcc >= 202 && mcc <= 297) /* Europe / Schengen / CEPT */
  {
    /* Known European public safety / emergency services networks -> TEA2 */
    if ((mcc == 234 && (mnc == 77 || mnc == 78 || mnc == 79)) ||          /* UK Airwave / ESN */
        (mcc == 204 && mnc == 9) ||                                       /* NL C2000 */
        (mcc == 206 && mnc == 1) ||                                       /* BE Astrid */
        (mcc == 262 && (mnc == 1001 || mnc == 1002 || mnc == 16383)) ||   /* DE BDBOS */
        (mcc == 208 && (mnc <= 100)) ||                                   /* FR Antares/INPT */
        (mcc == 240 && mnc == 36) ||                                      /* SE Rakel */
        (mcc == 242 && mnc == 19) ||                                      /* NO Nodnett */
        (mcc == 244 && mnc == 19) ||                                      /* FI Virve */
        (mcc == 238 && mnc == 99) ||                                      /* DK SINE */
        (mcc == 232 && mnc == 100) ||                                     /* AT Tetron */
        (mcc == 228 && (mnc == 54 || mnc == 9999)) ||                     /* CH Polycom */
        (mcc == 214 && mnc == 51) ||                                      /* ES SIRDEE */
        (mcc == 268 && mnc == 99) ||                                      /* PT SIRESP */
        (mcc == 272 && mnc == 51))                                        /* IE NDRS */
    {
      return "TEA2";
    }
    return "TEA1";
  }
  else if (mcc > 0)
  {
    return "TEA1/TEA3";
  }
  return "TEA1/TEA2";
}

/*-------------------------------------------------------------------------------
 * TEA1 / TEA2 / TEA3 / TEA4 Decryption (ported from TetraEar crypto.py & decoder.py)
 *-----------------------------------------------------------------------------*/

static uint32_t tetra_unpack_be32(const uint8_t *p)
{
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] << 8)  | ((uint32_t)p[3]);
}

static void tetra_pack_be32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)((v >> 24) & 0xFFU);
  p[1] = (uint8_t)((v >> 16) & 0xFFU);
  p[2] = (uint8_t)((v >> 8)  & 0xFFU);
  p[3] = (uint8_t)(v & 0xFFU);
}

/* Ported from TEADecryptor._tea1_decrypt_block in TetraEar/tetraear/core/crypto.py */
static void tetra_tea1_decrypt_block(const uint8_t *key, const uint8_t *block, uint8_t *out)
{
  uint16_t key_words[5];
  for (int i = 0; i < 5; i++)
    key_words[i] = (uint16_t)(((uint16_t)key[i * 2] << 8) | key[i * 2 + 1]);

  uint32_t v0 = tetra_unpack_be32(&block[0]);
  uint32_t v1 = tetra_unpack_be32(&block[4]);
  uint32_t delta = 0x9E3779B9U;
  uint32_t sum_val = 0xC6EF3720U; /* delta * 32 */

  for (int r = 0; r < 32; r++)
  {
    v1 -= ((((v0 << 4) ^ (v0 >> 5) ^ sum_val) + v0) ^ ((uint32_t)key_words[(sum_val >> 11) & 3U] + sum_val));
    sum_val -= delta;
    v0 -= ((((v1 << 4) ^ (v1 >> 5) ^ sum_val) + v1) ^ ((uint32_t)key_words[sum_val & 3U] + sum_val));
  }

  tetra_pack_be32(&out[0], v0);
  tetra_pack_be32(&out[4], v1);
}

/* Ported from TEADecryptor._tea2_decrypt_block (also TEA3/TEA4) in TetraEar/tetraear/core/crypto.py */
static void tetra_tea2_decrypt_block(const uint8_t *key, const uint8_t *block, uint8_t *out)
{
  uint32_t k0 = tetra_unpack_be32(&key[0]);
  uint32_t k1 = tetra_unpack_be32(&key[4]);
  uint32_t k2 = tetra_unpack_be32(&key[8]);
  uint32_t k3 = tetra_unpack_be32(&key[12]);

  uint32_t v0 = tetra_unpack_be32(&block[0]);
  uint32_t v1 = tetra_unpack_be32(&block[4]);
  uint32_t delta = 0x9E3779B9U;
  uint32_t sum_val = 0xC6EF3720U; /* delta * 32 */

  for (int r = 0; r < 32; r++)
  {
    v1 -= (((v0 << 4) + k2) ^ (v0 + sum_val) ^ ((v0 >> 5) + k3));
    sum_val -= delta;
    v0 -= (((v1 << 4) + k0) ^ (v1 + sum_val) ^ ((v1 >> 5) + k1));
  }

  tetra_pack_be32(&out[0], v0);
  tetra_pack_be32(&out[4], v1);
}

static void tetra_tea_decrypt_bytes(uint8_t alg, const uint8_t *key,
                                    const uint8_t *in_bytes, uint8_t *out_bytes, int nbytes)
{
  for (int off = 0; off < nbytes; off += 8)
  {
    int chunk = (nbytes - off >= 8) ? 8 : (nbytes - off);
    if (chunk < 8)
    {
      memcpy(&out_bytes[off], &in_bytes[off], (size_t)chunk);
      break;
    }
    uint8_t blk[8] = {0};
    uint8_t dec[8] = {0};
    memcpy(blk, &in_bytes[off], 8);
    if (alg == 1)
      tetra_tea1_decrypt_block(key, blk, dec);
    else
      tetra_tea2_decrypt_block(key, blk, dec);
    memcpy(&out_bytes[off], dec, 8);
  }
}

static uint8_t tetra_get_expected_alg(const dsd_state *state)
{
  if (state->tetra_enc_alg >= 1 && state->tetra_enc_alg <= 4)
    return state->tetra_enc_alg;
  const char *tea = tetra_get_tea_name(state, 1);
  if (strcmp(tea, "TEA2") == 0)
    return 2;
  if (strcmp(tea, "TEA3") == 0)
    return 3;
  if (strcmp(tea, "TEA4") == 0)
    return 4;
  return 1;
}

static int tetra_extract_user_key(const dsd_opts *opts, const dsd_state *state, uint8_t *key_out)
{
  memset(key_out, 0, 16);
  if (state->K1 == 0 && state->K2 == 0)
    return 0;

  /* Parse hex digits from -H (opts->szNumbers) if present so both spaced and contiguous 20/32-char keys work */
  if (opts != NULL && opts->szNumbers[0] != '\0')
  {
    uint8_t nibbles[32] = {0};
    int n_nib = 0;
    for (int i = 0; opts->szNumbers[i] != '\0' && n_nib < 32; i++)
    {
      char c = opts->szNumbers[i];
      if (c >= '0' && c <= '9')
        nibbles[n_nib++] = (uint8_t)(c - '0');
      else if (c >= 'a' && c <= 'f')
        nibbles[n_nib++] = (uint8_t)(c - 'a' + 10);
      else if (c >= 'A' && c <= 'F')
        nibbles[n_nib++] = (uint8_t)(c - 'A' + 10);
    }
    if (n_nib >= 2)
    {
      int n_bytes = n_nib / 2;
      for (int i = 0; i < n_bytes && i < 16; i++)
        key_out[i] = (uint8_t)((nibbles[i * 2] << 4) | nibbles[i * 2 + 1]);
      return (n_bytes > 10) ? 16 : 10;
    }
  }

  for (int i = 0; i < 8; i++)
    key_out[i] = (uint8_t)((state->K1 >> ((7 - i) * 8)) & 0xFFU);
  if (state->K2 != 0)
  {
    if ((state->K2 & 0xFFFFFFFFFFFF0000ULL) == 0)
    {
      key_out[8] = (uint8_t)((state->K2 >> 8) & 0xFFU);
      key_out[9] = (uint8_t)(state->K2 & 0xFFU);
      return 10;
    }
    for (int i = 0; i < 8; i++)
      key_out[8 + i] = (uint8_t)((state->K2 >> ((7 - i) * 8)) & 0xFFU);
    return 16;
  }
  return 10;
}

static void tetra_store_matched_key(dsd_state *state, const uint8_t *key, int key_len, uint8_t alg)
{
  if (key_len != 10 && key_len != 16)
    return;
  memcpy(state->tetra_matched_key, key, (size_t)key_len);
  state->tetra_matched_key_len = (uint8_t)key_len;
  state->tetra_matched_alg = alg;
  for (int i = 0; i < key_len; i++)
    snprintf(&state->tetra_matched_key_str[i * 2], 3, "%02X", key[i]);
  state->tetra_matched_key_str[key_len * 2] = '\0';
}

static void tetra_clear_matched_key(dsd_state *state)
{
  memset(state->tetra_matched_key, 0, sizeof(state->tetra_matched_key));
  state->tetra_matched_key_len = 0;
  state->tetra_matched_alg = 0;
  memset(state->tetra_matched_key_str, 0, sizeof(state->tetra_matched_key_str));
}

static int tetra_get_active_key(const dsd_opts *opts, dsd_state *state, uint8_t *key_out, uint8_t *alg_out)
{
  int u_len = tetra_extract_user_key(opts, state, key_out);
  if (u_len > 0)
  {
    uint8_t u_alg = (u_len == 16) ? 4 : tetra_get_expected_alg(state);
    if (state->tetra_matched_key_len != 0 &&
        (state->tetra_matched_key_len != (uint8_t)u_len ||
         memcmp(state->tetra_matched_key, key_out, (size_t)u_len) != 0))
    {
      tetra_clear_matched_key(state);
    }
    if (alg_out)
      *alg_out = u_alg;
    return u_len;
  }
  if (state->tetra_matched_key_len != 0)
    tetra_clear_matched_key(state);
  return 0;
}

static int tetra_get_llc_sdu_bounds(const uint8_t *bits, int nbits,
                                    uint8_t *llc_type_out, int *hdr_len_out, int *sdu_len_out)
{
  if (nbits < 4)
    return 0;
  uint8_t llc_type = (uint8_t)tetra_bits_to_uint(bits, 0, 4);
  int has_fcs = (llc_type >= 4 && llc_type <= 7);
  uint8_t base_type = has_fcs ? (uint8_t)(llc_type - 4) : llc_type;
  int hdr_len = (base_type == 0) ? 6 : (base_type == 1 || base_type == 3) ? 5 : (base_type == 2) ? 4 : 6;
  int sdu_end = (has_fcs && nbits > hdr_len + 32) ? (nbits - 32) : nbits;
  if (llc_type_out)
    *llc_type_out = llc_type;
  if (hdr_len_out)
    *hdr_len_out = hdr_len;
  if (sdu_len_out)
    *sdu_len_out = sdu_end - hdr_len;
  return 1;
}

static inline int tetra_is_cell_encrypted(const dsd_state *state)
{
  return (state->tetra_enc || state->tetra_aie_enc || state->tetra_sec_class >= 2) ? 1 : 0;
}

/* Ported from TetraDecoder._decrypt_frame scoring & verification in TetraEar/tetraear/core/decoder.py */
static int tetra_score_decrypted_payload(const uint8_t *dec_bits, int nbits,
                                         const uint8_t *dec_bytes, int nbytes,
                                         int *verified_out)
{
  *verified_out = 0;
  if (nbytes <= 0)
    return 0;

  int score = 0;
  int printable_count = 0;
  uint8_t seen[256] = {0};
  int unique_bytes = 0;
  int all_zero = 1;
  int all_ff = 1;

  for (int i = 0; i < nbytes; i++)
  {
    uint8_t b = dec_bytes[i];
    if (b >= 32 && b <= 126)
      printable_count++;
    if (!seen[b])
    {
      seen[b] = 1;
      unique_bytes++;
    }
    if (b != 0x00)
      all_zero = 0;
    if (b != 0xFF)
      all_ff = 0;
  }

  score += printable_count * 2;
  if (unique_bytes > nbytes / 8)
    score += 30;
  if (all_zero)
    score -= 50;
  if (all_ff)
    score -= 50;

  if (nbytes >= 4)
  {
    uint8_t b0 = dec_bytes[0];
    if (b0 != 0x00 && b0 != 0xFF)
      score += 10;
    if (b0 == 0x01 || b0 == 0x02 || b0 == 0x03 || b0 == 0x04 ||
        b0 == 0x05 || b0 == 0x08 || b0 == 0x0A || b0 == 0x0C)
      score += 20;
  }
  if (unique_bytes > 1)
    score += 10;

  /* Validate LLC + MLE header structure and verify CMCE D-SDS-DATA readable text */
  if (nbits >= 16 && !all_zero && !all_ff)
  {
    uint8_t llc_type = 0;
    int hdr_len = 0, sdu_len = 0;
    tetra_get_llc_sdu_bounds(dec_bits, nbits, &llc_type, &hdr_len, &sdu_len);
    if (llc_type > 6 || (llc_type & 3U) == 3U || sdu_len < 8)
    {
      score -= 100;
    }
    else
    {
      const uint8_t *sdu = &dec_bits[hdr_len];
      uint8_t mle_disc = (uint8_t)tetra_bits_to_uint(sdu, 0, 3);
      if (mle_disc != 1 && mle_disc != 2 && mle_disc != 4 && mle_disc != 5)
      {
        score -= 100;
      }
      else if (llc_type <= 2 && mle_disc == 2 && sdu_len >= 28)
      {
        uint8_t pdu_t = (uint8_t)tetra_bits_to_uint(sdu, 3, 5);
        if (pdu_t <= 16)
          score += 20;
        if (pdu_t == 7 && sdu_len >= 73) /* CMCE D-SETUP with Calling Party SSI */
        {
          uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
          uint8_t basic_svc = (uint8_t)tetra_bits_to_uint(sdu, 28, 8);
          uint8_t circuit_mode = (basic_svc >> 5) & 0x07U;
          uint8_t o_bit = sdu[43];
          if (call_id != 0 && circuit_mode == 0 && (basic_svc & 0x03U) == 0 && o_bit == 1)
          {
            int pos = 44;
            uint8_t notif_ind = sdu[pos++];
            if (notif_ind) pos += 6;
            uint8_t temp_addr = (pos < sdu_len) ? sdu[pos++] : 1;
            if (temp_addr) pos += 24;
            uint8_t cp_ind = (pos < sdu_len) ? sdu[pos++] : 0;
            if (cp_ind == 1 && pos + 26 <= sdu_len)
            {
              uint8_t cp_type = (uint8_t)tetra_bits_to_uint(sdu, pos, 2);
              uint32_t cp_ssi = tetra_bits_to_uint(sdu, pos + 2, 24);
              pos += 26;
              int rem_zeros = 1;
              for (int k = pos; k < pos + 3 && k < sdu_len; k++)
              {
                if (sdu[k] != 0)
                  rem_zeros = 0;
              }
              if ((cp_type == 1 || cp_type == 2) && cp_ssi != 0 && cp_ssi != 0xFFFFFFU && rem_zeros)
              {
                score += 100;
                *verified_out = 1;
              }
            }
          }
        }
        else if (pdu_t == 15 && sdu_len >= 43) /* CMCE D-SDS-DATA */
        {
          int pos = 8;
          uint8_t cp_ti = (uint8_t)tetra_bits_to_uint(sdu, pos, 2);
          pos += 2;
          if (cp_ti == 1 || cp_ti == 2) pos += 24;
          if (cp_ti == 2) pos += 24;
          if (pos + 2 <= sdu_len)
          {
            uint8_t sds_type = (uint8_t)tetra_bits_to_uint(sdu, pos, 2);
            pos += 2;
            if (sds_type == 3 && pos + 16 <= sdu_len)
            {
              uint16_t sds_len = (uint16_t)tetra_bits_to_uint(sdu, pos, 11);
              pos += 11;
              int rem = sdu_len - pos;
              if (sds_len >= 48 && (int)sds_len <= rem + 16)
              {
                if ((int)sds_len < rem)
                  rem = (int)sds_len;
                uint8_t proto_id = (uint8_t)tetra_bits_to_uint(sdu, pos, 8);
                int sds_bytes_len = rem / 8;
                uint8_t sds_bytes[128];
                if (sds_bytes_len > (int)sizeof(sds_bytes))
                  sds_bytes_len = (int)sizeof(sds_bytes);
                for (int i = 0; i < sds_bytes_len; i++)
                  sds_bytes[i] = (uint8_t)tetra_bits_to_uint(sdu, pos + i * 8, 8);

                char text_buf[128] = {0};
                if ((proto_id == 0x02 || proto_id == 0x03 || proto_id == 0x82) && sds_bytes_len >= 8)
                {
                  uint8_t coding = sds_bytes[1] & 0x1FU;
                  int start = ((sds_bytes[1] >> 7) & 1U) ? 5 : 2;
                  if (start < sds_bytes_len && coding <= 1)
                    tetra_decode_sds_text_payload(&sds_bytes[start], sds_bytes_len - start,
                                                  coding, text_buf, sizeof(text_buf));
                }
                int tlen = (int)strlen(text_buf);
                if (tlen >= 6)
                {
                  int clean_chars = 0;
                  int letters = 0;
                  for (int i = 0; i < tlen; i++)
                  {
                    char c = text_buf[i];
                    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
                    {
                      letters++;
                      clean_chars++;
                    }
                    else if ((c >= '0' && c <= '9') || c == ' ' || c == '.' || c == ',' ||
                             c == '!' || c == '?' || c == ':' || c == ';' || c == '-' || c == '_' || c == '/')
                    {
                      clean_chars++;
                    }
                  }
                  if (letters >= 4 && clean_chars * 20 >= tlen * 19)
                  {
                    score += 120;
                    *verified_out = 1;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  return score;
}

static int tetra_eval_candidate_key(uint8_t alg, const uint8_t *key,
                                    const uint8_t *payload_bytes, int nbytes, int nbits,
                                    uint8_t *cand_bytes, uint8_t *cand_bits, int *verified_out)
{
  tetra_tea_decrypt_bytes(alg, key, payload_bytes, cand_bytes, nbytes);
  for (int b = 0; b < nbits; b++)
    cand_bits[b] = (cand_bytes[b / 8] >> (7 - (b % 8))) & 1U;
  return tetra_score_decrypted_payload(cand_bits, nbits, cand_bytes, nbytes, verified_out);
}

static int tetra_try_decrypt_sdu(const dsd_opts *opts, dsd_state *state, const uint8_t *enc_bits,
                                 uint8_t *dec_bits, int nbits, uint8_t *used_alg_out)
{
  int nbytes = (nbits + 7) / 8;
  if (nbytes < 8 || nbytes > 64)
    return 0;

  uint8_t payload_bytes[64] = {0};
  for (int i = 0; i < nbytes; i++)
  {
    int b_cnt = (nbits - i * 8 >= 8) ? 8 : (nbits - i * 8);
    payload_bytes[i] = (uint8_t)(tetra_bits_to_uint(enc_bits, i * 8, b_cnt) << (8 - b_cnt));
  }

  uint8_t primary_alg = tetra_get_expected_alg(state);
  uint8_t user_key[16];
  int user_key_len = tetra_extract_user_key(opts, state, user_key);
  /* 1. Try user-provided key if configured */
  if (user_key_len > 0)
  {
    int already_verified = (state->tetra_matched_key_len == (uint8_t)user_key_len &&
                            memcmp(state->tetra_matched_key, user_key, (size_t)user_key_len) == 0);
    uint8_t u_alg = (user_key_len == 16) ? 4 : primary_alg;
    uint8_t cand_bytes[64] = {0};
    uint8_t cand_bits[512] = {0};
    int verified = 0;
    int score = tetra_eval_candidate_key(u_alg, user_key, payload_bytes, nbytes, nbits,
                                         cand_bytes, cand_bits, &verified);
    if (!verified && !(already_verified && score >= 80) && user_key_len == 10)
    {
      uint8_t alt_alg = (u_alg == 1) ? 2 : 1;
      int alt_ver = 0;
      uint8_t alt_bytes[64] = {0};
      uint8_t alt_bits[512] = {0};
      int alt_score = tetra_eval_candidate_key(alt_alg, user_key, payload_bytes, nbytes, nbits,
                                               alt_bytes, alt_bits, &alt_ver);
      if (alt_ver || (already_verified && alt_score > score))
      {
        u_alg = alt_alg;
        score = alt_score;
        verified = alt_ver;
        memcpy(cand_bytes, alt_bytes, sizeof(cand_bytes));
        memcpy(cand_bits, alt_bits, sizeof(cand_bits));
      }
    }
    if (verified || (already_verified && score >= 80))
    {
      tetra_store_matched_key(state, user_key, user_key_len, u_alg);
      memcpy(dec_bits, cand_bits, (size_t)nbits);
      if (used_alg_out)
        *used_alg_out = u_alg;
      return 1;
    }
    return 0;
  }

  if (state->tetra_matched_key_len != 0)
    tetra_clear_matched_key(state);

  /* 2. Try BYPASS (treat as clear if mis-flagged by network, per TetraEar decoder.py line 747) */
  int verified = 0;
  int score = tetra_score_decrypted_payload(enc_bits, nbits, payload_bytes, nbytes, &verified);
  if (verified && score >= 80)
  {
    memcpy(dec_bits, enc_bits, (size_t)nbits);
    if (used_alg_out)
      *used_alg_out = 0; /* 0 = BYPASS (Clear) */
    return 1;
  }

  return 0;
}

static double tetra_calc_band_freq_mhz(int band, uint16_t carrier, int offset_idx)
{
  static const double OFFSETS_MHZ[4] = {0.0, 0.00625, -0.00625, 0.0125};
  if (band >= 1 && band <= 9 && carrier > 0)
  {
    return (double)band * 100.0 +
           (double)carrier * 0.025 +
           OFFSETS_MHZ[offset_idx & 3];
  }
  return 0.0;
}

static double tetra_calc_carrier_freq_mhz(const dsd_state *state, uint16_t carrier)
{
  return tetra_calc_band_freq_mhz(state->tetra_freq_band, carrier, state->tetra_offset_idx);
}

static int g_tetra_wav_samples_written = 0;
static int g_tetra_wav_slot = -1;
static char g_tetra_last_sds_key[600] = {0};
static time_t g_tetra_last_console_log_time = 0;
static uint8_t g_tetra_last_bsch_log_mn = 0;
static char g_tetra_burst_hdr[160] = {0};
static int g_tetra_burst_hdr_printed = 0;
static uint32_t g_tetra_ul_last_sym = 0;
static uint8_t g_tetra_ul_cand_count = 0;

static void tetra_set_burst_hdr(const char *hdr)
{
  if (hdr != NULL)
    snprintf(g_tetra_burst_hdr, sizeof(g_tetra_burst_hdr), "%s", hdr);
  else
    g_tetra_burst_hdr[0] = '\0';
  g_tetra_burst_hdr_printed = 0;
}

static void tetra_ensure_burst_hdr(void)
{
  if (!g_tetra_burst_hdr_printed && g_tetra_burst_hdr[0] != '\0')
  {
    fprintf(stderr, "%s", g_tetra_burst_hdr);
    g_tetra_burst_hdr_printed = 1;
    g_tetra_last_console_log_time = time(NULL);
  }
}

static void tetra_finish_burst_hdr(int force_print)
{
  if (force_print)
    tetra_ensure_burst_hdr();
  if (g_tetra_burst_hdr_printed)
  {
    fprintf(stderr, "\n");
    g_tetra_last_console_log_time = time(NULL);
  }
  g_tetra_burst_hdr[0] = '\0';
  g_tetra_burst_hdr_printed = 0;
}

static int tetra_is_call_event_str(const char *s)
{
  return (s != NULL && strstr(s, "TETRA TGT:") != NULL &&
          strstr(s, "SDS-") == NULL && strstr(s, "Status:") == NULL && strstr(s, "MM ") == NULL) ? 1 : 0;
}

static void tetra_close_per_call_wav(dsd_opts *opts, dsd_state *state)
{
  int s = g_tetra_wav_slot;
  if (s < 0 && state != NULL && state->tetra_voice_slot >= 1 && state->tetra_voice_slot <= 4)
    s = state->tetra_voice_slot - 1;
  g_tetra_wav_slot = -1;

  if (!g_tetra_wav_samples_written)
    return;
  g_tetra_wav_samples_written = 0;

  if (opts == NULL || state == NULL || state->event_history_s == NULL)
    return;
  if (opts->dmr_stereo_wav != 1 || opts->static_wav_file != 0 || opts->wav_out_f == NULL)
    return;

  int call_idx = 0;
  char slot_tag[16] = {0};
  if (s >= 0 && s < 4)
    snprintf(slot_tag, sizeof(slot_tag), "TN: %d;", s + 1);

  for (int k = 1; k <= 8; k++)
  {
    const char *ev_str = state->event_history_s[0].Event_History_Items[k].event_string;
    if (tetra_is_call_event_str(ev_str))
    {
      if (slot_tag[0] != '\0' && strstr(ev_str, slot_tag) != NULL)
      {
        call_idx = k;
        break;
      }
      if (call_idx == 0)
        call_idx = k;
    }
  }

  if (call_idx > 0)
    state->event_history_s[0].Event_History_Items[0] = state->event_history_s[0].Event_History_Items[call_idx];
  else
    init_event_history(&state->event_history_s[0], 0, 1);

  if (state->event_history_s[0].Event_History_Items[0].event_time == 0)
    state->event_history_s[0].Event_History_Items[0].event_time = time(NULL);

  snprintf(state->event_history_s[0].Event_History_Items[0].sysid_string,
           sizeof(state->event_history_s[0].Event_History_Items[0].sysid_string),
           "TETRA_%03u_%04u_CC_%02u",
           state->tetra_mcc, state->tetra_mnc, state->tetra_colour_code);

  if (s >= 0 && s < 4)
  {
    if (state->tetra_ts_tg[s] != 0)
      state->event_history_s[0].Event_History_Items[0].target_id = state->tetra_ts_tg[s];
    if (state->tetra_ts_src[s] != 0)
      state->event_history_s[0].Event_History_Items[0].source_id = state->tetra_ts_src[s];
    if (strncmp(state->tetra_ts_type[s], "Indiv", 5) == 0)
      state->event_history_s[0].Event_History_Items[0].gi = 1;
    else if (state->tetra_ts_type[s][0] != '\0')
      state->event_history_s[0].Event_History_Items[0].gi = 0;
  }

  opts->wav_out_f = close_and_rename_wav_file(opts->wav_out_f, opts->wav_out_file,
                                              opts->wav_out_dir, opts->wav_custom_tag,
                                              &state->event_history_s[0]);
  opts->wav_out_f = open_wav_file(opts->wav_out_dir, opts->wav_out_file, 8000, 0);
  init_event_history(&state->event_history_s[0], 0, 1);
}

static void tetra_unlock_voice_slot(dsd_opts *opts, dsd_state *state, int flush_pending)
{
  if (flush_pending && state->tetra_cod_batch_count > 0)
    tetra_flush_voice_batch(opts, state);
  state->tetra_cod_batch_count = 0;
  state->tetra_voice_slot = 0;
  state->tetra_voice_usage = 0;
  state->tetra_serial_hist_frames = 0;
}

void tetra_release_voice_slot(dsd_opts *opts, dsd_state *state, int flush_pending)
{
  tetra_unlock_voice_slot(opts, state, flush_pending);
  tetra_close_per_call_wav(opts, state);
}

void tetra_clear_cell_state(dsd_state *state)
{
  state->tetra_mode = 0;
  state->tetra_dmo_comm_type = 0;
  state->tetra_la = 0;
  state->tetra_main_carrier = 0;
  state->tetra_carrier = 0;
  state->tetra_dl_freq_mhz = 0.0;
  state->tetra_ts_mask = 0;
  state->tetra_last_tg = 0;
  state->tetra_last_src = 0;
  state->tetra_call_id = 0;
  state->tetra_status_code = 0;
  state->tetra_enc = 0;
  state->tetra_enc_alg = 0;
  state->tetra_aie_enc = 0;
  state->tetra_sec_class = 0;
  state->tetra_bsch_tn = 0;
  state->tetra_bsch_fn = 0;
  state->tetra_bsch_mn = 0;
  tetra_clear_matched_key(state);
  state->tetra_voice_slot = 0;
  state->tetra_voice_usage = 0;
  state->tetra_cod_batch_count = 0;
  state->tetra_serial_hist_frames = 0;
  state->tetra_last_voice_time = 0;
  state->tetra_voice_last_sym = 0;
  state->tetra_nc_count = 0;
  memset(state->tetra_nc_carriers, 0, sizeof(state->tetra_nc_carriers));
  memset(state->tetra_nc_freqs, 0, sizeof(state->tetra_nc_freqs));
  memset(state->tetra_ts_usage, 0, sizeof(state->tetra_ts_usage));
  memset(state->tetra_ts_tg, 0, sizeof(state->tetra_ts_tg));
  memset(state->tetra_ts_src, 0, sizeof(state->tetra_ts_src));
  memset(state->tetra_ts_call_id, 0, sizeof(state->tetra_ts_call_id));
  memset(state->tetra_ts_type, 0, sizeof(state->tetra_ts_type));
  memset(state->tetra_ts_last_sym, 0, sizeof(state->tetra_ts_last_sym));
  memset(state->tetra_ts_cand_usage, 0, sizeof(state->tetra_ts_cand_usage));
  memset(state->tetra_ts_cand_count, 0, sizeof(state->tetra_ts_cand_count));
  memset(state->tetra_ts_cand_sym, 0, sizeof(state->tetra_ts_cand_sym));
  memset(state->tetra_ts_idle_count, 0, sizeof(state->tetra_ts_idle_count));
  memset(state->tetra_ts_logged, 0, sizeof(state->tetra_ts_logged));
  memset(state->tetra_usage_ssi, 0, sizeof(state->tetra_usage_ssi));
  memset(state->tetra_call_type, 0, sizeof(state->tetra_call_type));
  memset(state->tetra_pdu_str, 0, sizeof(state->tetra_pdu_str));
  memset(state->tetra_sds_str, 0, sizeof(state->tetra_sds_str));
  memset(state->tetra_lip_str, 0, sizeof(state->tetra_lip_str));
  memset(g_tetra_last_sds_key, 0, sizeof(g_tetra_last_sds_key));
  g_tetra_last_console_log_time = 0;
  g_tetra_last_bsch_log_mn = 0;
  g_tetra_ul_last_sym = 0;
  g_tetra_ul_cand_count = 0;
  state->tetra_mac_frag_active = 0;
  state->tetra_mac_frag_len = 0;
  state->tetra_mac_frag_enc = 0;
  state->tetra_mac_frag_ssi = 0;
  memset(state->tetra_mac_frag_buf, 0, sizeof(state->tetra_mac_frag_buf));
}

void tetra_reset_state(dsd_state *state)
{
  tetra_clear_cell_state(state);
  state->tetra_mode = 0;
  state->tetra_dmo_comm_type = 0;
  state->tetra_burst_type = 0;
  state->tetra_mcc = 0;
  state->tetra_mnc = 0;
  state->tetra_colour_code = 0;
  state->tetra_scramb_init = TETRA_SCRAMB_INIT_DEFAULT;
  state->tetra_scramb_valid = 0;
  state->tetra_dl_usage = 0;
  state->tetra_ul_usage = 0;
  state->tetra_slot = 0;
  state->tetra_frame = 0;
  state->tetra_multiframe = 0;
  state->tetra_hyperframe = 0;
  state->tetra_freq_band = 0;
  state->tetra_offset_idx = 0;
  state->tetra_duplex_spacing = 0;
  state->tetra_tdma_locked = 0;
  memset(state->tetra_country_str, 0, sizeof(state->tetra_country_str));
  memset(state->tetra_mac_frag_buf, 0, sizeof(state->tetra_mac_frag_buf));
}

static void tetra_reset_slot_state(dsd_opts *opts, dsd_state *state, int s)
{
  if (s < 0 || s >= 4)
    return;
  if (state->tetra_voice_slot == (uint8_t)(s + 1))
    tetra_unlock_voice_slot(opts, state, 1);
  if (g_tetra_wav_slot == s)
    tetra_close_per_call_wav(opts, state);
  uint32_t old_tg = state->tetra_ts_tg[s];
  uint32_t old_u = state->tetra_ts_usage[s];
  for (int i = 0; i < 16; i++)
  {
    if ((old_tg != 0 && state->tetra_ev_ssi_cache[i] == old_tg) ||
        (old_u != 0 && state->tetra_ev_ssi_cache[i] == old_u))
    {
      state->tetra_ev_ssi_cache[i] = 0;
      state->tetra_ev_time_cache[i] = 0;
    }
  }
  state->tetra_ts_usage[s] = 0;
  state->tetra_ts_tg[s] = 0;
  state->tetra_ts_src[s] = 0;
  state->tetra_ts_call_id[s] = 0;
  state->tetra_ts_type[s][0] = '\0';
  state->tetra_ts_cand_count[s] = 0;
  state->tetra_ts_idle_count[s] = 0;
  state->tetra_ts_logged[s] = 0;
}

static int tetra_should_log_ssi(dsd_state *state, uint32_t ssi)
{
  if (ssi == 0 || ssi == 0xFFFFFFU)
    return 0;
  time_t now = time(NULL);
  for (int i = 0; i < 16; i++)
  {
    if (state->tetra_ev_ssi_cache[i] == ssi && (now - state->tetra_ev_time_cache[i]) < 60)
    {
      state->tetra_ev_time_cache[i] = now;
      return 0;
    }
  }
  uint8_t idx = state->tetra_ev_cache_idx & 15U;
  state->tetra_ev_ssi_cache[idx] = ssi;
  state->tetra_ev_time_cache[idx] = now;
  state->tetra_ev_cache_idx = (uint8_t)((idx + 1) & 15U);
  return 1;
}

static const char *tetra_get_slot_call_mode_tag(const dsd_state *state, int s)
{
  const char *ct = (s >= 0 && s < 4 && state->tetra_ts_type[s][0] != '\0') ? state->tetra_ts_type[s] : state->tetra_call_type;
  if (strncmp(ct, "Group", 5) == 0) return "Group;";
  if (strncmp(ct, "Indiv", 5) == 0) return "Private;";
  if (strncmp(ct, "Direct", 6) == 0) return "Direct;";
  if (strncmp(ct, "Bcast", 5) == 0) return "Broadcast;";
  return "Voice;";
}

static const char *tetra_get_call_mode_tag(const dsd_state *state)
{
  if (strncmp(state->tetra_call_type, "Group", 5) == 0)
    return "Group;";
  if (strncmp(state->tetra_call_type, "Indiv", 5) == 0)
    return "Private;";
  if (strncmp(state->tetra_call_type, "Direct", 6) == 0)
    return "Direct;";
  if (strncmp(state->tetra_call_type, "Bcast", 5) == 0)
    return "Broadcast;";
  return "Voice;";
}

static void tetra_format_event_desc(const dsd_state *state, char *full_desc, size_t max_len,
                                    const char *desc, uint32_t src_ssi, uint32_t dst_ssi)
{
  uint16_t car = state->tetra_carrier ? state->tetra_carrier : state->tetra_main_carrier;
  int is_enc = tetra_is_cell_encrypted(state);
  int pos = snprintf(full_desc, max_len, "TETRA TGT: %08u; ", dst_ssi);
  if (pos < 0 || (size_t)pos >= max_len)
    return;

  if (src_ssi != 0)
  {
    pos += snprintf(full_desc + pos, max_len - (size_t)pos, "SRC: %08u; ", src_ssi);
    if (pos < 0 || (size_t)pos >= max_len)
      return;
  }

  pos += snprintf(full_desc + pos, max_len - (size_t)pos, "CC: %02u; ", state->tetra_colour_code);
  if (pos < 0 || (size_t)pos >= max_len)
    return;

  if (is_enc)
  {
    pos += snprintf(full_desc + pos, max_len - (size_t)pos, "ENC; ALG: %s; ",
                    tetra_get_tea_name(state, 1));
    if (pos < 0 || (size_t)pos >= max_len)
      return;
  }

  if (desc != NULL && desc[0] != '\0')
  {
    pos += snprintf(full_desc + pos, max_len - (size_t)pos, "%s ", desc);
    if (pos < 0 || (size_t)pos >= max_len)
      return;
  }

  if (car > 0)
    snprintf(full_desc + pos, max_len - (size_t)pos, "Car: %u;", car);
}

static void tetra_log_data_event_ex(dsd_opts *opts, dsd_state *state,
                                    uint32_t src_ssi, uint32_t dst_ssi,
                                    const char *desc, int update_in_place)
{
  char full_desc[256];
  tetra_format_event_desc(state, full_desc, sizeof(full_desc), desc, src_ssi, dst_ssi);
  uint32_t ev_src = (src_ssi != 0) ? src_ssi : ((dst_ssi != 0) ? dst_ssi : 0xFFFFFFU);

  /* Update the current active call's Event History entry in-place only while the slot remains active */
  if (update_in_place && state->event_history_s != NULL)
  {
    char tn_prefix[16] = {0};
    if (desc != NULL && strncmp(desc, "TN: ", 4) == 0 && desc[4] >= '1' && desc[4] <= '4')
      snprintf(tn_prefix, sizeof(tn_prefix), "TN: %c;", desc[4]);

    int call_idx = 0;
    for (int k = 1; k <= 8; k++)
    {
      const char *prev_str = state->event_history_s[0].Event_History_Items[k].event_string;
      if (tetra_is_call_event_str(prev_str))
      {
        if (tn_prefix[0] != '\0')
        {
          if (strstr(prev_str, tn_prefix) != NULL)
          {
            call_idx = k;
            break;
          }
        }
        else if (dst_ssi == 0 || state->event_history_s[0].Event_History_Items[k].target_id == 0 ||
                 state->event_history_s[0].Event_History_Items[k].target_id == dst_ssi)
        {
          call_idx = k;
          break;
        }
      }
    }
    if (call_idx > 0)
    {
      if (dst_ssi == 0 && src_ssi == 0)
        return;
      uint32_t prev_tgt = state->event_history_s[0].Event_History_Items[call_idx].target_id;
      uint32_t prev_src = state->event_history_s[0].Event_History_Items[call_idx].source_id;
      /* If a different radio (new SRC) keys up on the same talkgroup, push a new entry instead of overwriting */
      if (prev_src != 0 && src_ssi != 0 && prev_src != src_ssi)
      {
        call_idx = 0;
      }
      else
      {
        uint32_t eff_dst = (dst_ssi != 0) ? dst_ssi : prev_tgt;
        uint32_t eff_src = (src_ssi != 0) ? src_ssi : prev_src;
        tetra_format_event_desc(state, full_desc, sizeof(full_desc), desc, eff_src, eff_dst);

        time_t ev_t = state->event_history_s[0].Event_History_Items[call_idx].event_time;
        if (ev_t == 0)
          ev_t = time(NULL);
        char *timestr = getTimeN(ev_t);
        char *datestr = getDateN(ev_t);
        state->event_history_s[0].Event_History_Items[call_idx].source_id = eff_src;
        state->event_history_s[0].Event_History_Items[call_idx].target_id = eff_dst;
        state->event_history_s[0].Event_History_Items[call_idx].gi =
            (strncmp(state->tetra_call_type, "Indiv", 5) == 0) ? 1 : 0;
        snprintf(state->event_history_s[0].Event_History_Items[call_idx].sysid_string,
                 sizeof(state->event_history_s[0].Event_History_Items[call_idx].sysid_string),
                 "TETRA_%03u_%04u_CC_%02u",
                 state->tetra_mcc, state->tetra_mnc, state->tetra_colour_code);
        snprintf(state->event_history_s[0].Event_History_Items[call_idx].event_string,
                 sizeof(state->event_history_s[0].Event_History_Items[call_idx].event_string),
                 "%s %s %s", datestr ? datestr : "", timestr ? timestr : "", full_desc);
        if (timestr != NULL) free(timestr);
        if (datestr != NULL) free(datestr);
        return;
      }
    }
  }

  snprintf(state->call_string[0], sizeof(state->call_string[0]), "%s", desc);
  int is_call = (desc == NULL || (strstr(desc, "SDS-") == NULL && strstr(desc, "Status:") == NULL && strstr(desc, "MM ") == NULL));
  state->gi[0] = is_call ? ((strncmp(state->tetra_call_type, "Indiv", 5) == 0) ? 1 : 0) : -1;
  state->event_history_s[0].Event_History_Items[0].color_pair = 4;
  watchdog_event_datacall(opts, state, ev_src, dst_ssi, full_desc, 0);
  snprintf(state->event_history_s[0].Event_History_Items[0].sysid_string,
           sizeof(state->event_history_s[0].Event_History_Items[0].sysid_string),
           "TETRA_%03u_%04u_CC_%02u",
           state->tetra_mcc, state->tetra_mnc, state->tetra_colour_code);
  state->lastsrc = 0;
  state->lasttg = 0;
  SNDFILE *saved_wav = opts->wav_out_f;
  opts->wav_out_f = NULL;
  watchdog_event_history(opts, state, 0);
  opts->wav_out_f = saved_wav;
  if (state->event_history_s != NULL)
    state->event_history_s[0].Event_History_Items[1].source_id = src_ssi;
}

static void tetra_log_data_event(dsd_opts *opts, dsd_state *state, uint32_t src_ssi, uint32_t dst_ssi, const char *desc)
{
  tetra_log_data_event_ex(opts, state, src_ssi, dst_ssi, desc, 0);
}

static uint32_t tetra_parse_cmce_party_ssi(const uint8_t *sdu, int sdu_len, int *pos_io)
{
  int pos = *pos_io;
  if (pos + 2 > sdu_len)
    return 0;
  uint8_t ind = (uint8_t)tetra_bits_to_uint(sdu, pos, 2);
  pos += 2;
  uint32_t ssi = 0;
  if (ind == 1 && pos + 24 <= sdu_len)
  {
    ssi = tetra_bits_to_uint(sdu, pos, 24);
    pos += 24;
  }
  else if (ind == 2 && pos + 48 <= sdu_len)
  {
    ssi = tetra_bits_to_uint(sdu, pos, 24);
    pos += 48;
  }
  else if (ind == 2 && pos + 24 <= sdu_len)
  {
    ssi = tetra_bits_to_uint(sdu, pos, 24);
    pos += 24;
  }
  *pos_io = pos;
  return (ssi != 0 && ssi != 0xFFFFFFU) ? ssi : 0;
}

static void tetra_apply_cmce_call_slots(dsd_opts *opts, dsd_state *state,
                                        uint16_t call_id, int is_enc, int log_event)
{
  UNUSED(is_enc);
  uint8_t mask = state->tetra_ts_mask;
  if (mask == 0 && state->tetra_slot >= 2 && state->tetra_slot <= 4)
    mask = (uint8_t)(1U << (4 - state->tetra_slot));
  int matched_slot = 0;
  int src_changed = 0;
  uint8_t prev_logged = 0;
  uint8_t new_logged = ((state->tetra_last_tg != 0) ? 2U : 0U) |
                       ((state->tetra_last_src != 0) ? 1U : 0U);
  for (int s = 0; s < 4; s++)
  {
    int is_slot_match = (mask & (1U << (3 - s))) ||
                        (call_id != 0 && state->tetra_ts_call_id[s] == call_id) ||
                        (state->tetra_ts_usage[s] > 3 &&
                         (state->tetra_ts_tg[s] == 0 || state->tetra_ts_tg[s] == state->tetra_last_tg));
    if (is_slot_match)
    {
      if (state->tetra_ts_logged[s] > prev_logged)
        prev_logged = state->tetra_ts_logged[s];
      if (state->tetra_last_src != 0 && state->tetra_ts_src[s] != state->tetra_last_src)
        src_changed = 1;
      if (state->tetra_last_tg != 0)
        state->tetra_ts_tg[s] = state->tetra_last_tg;
      if (state->tetra_last_src != 0)
        state->tetra_ts_src[s] = state->tetra_last_src;
      state->tetra_ts_call_id[s] = call_id;
      if (state->tetra_call_type[0] != '\0')
        snprintf(state->tetra_ts_type[s], sizeof(state->tetra_ts_type[s]), "%s", state->tetra_call_type);
      state->tetra_ts_last_sym[s] = state->tetra_sym_count;
      matched_slot = s + 1;
    }
  }
  if (log_event && new_logged > 0)
  {
    int is_new_ssi = (state->tetra_last_tg != 0) ? tetra_should_log_ssi(state, state->tetra_last_tg)
                                                 : tetra_should_log_ssi(state, state->tetra_last_src);
    if ((prev_logged == 0 && is_new_ssi) ||
        (prev_logged > 0 && (new_logged > prev_logged || src_changed)))
    {
      char ev_desc[128];
      if (matched_slot > 0)
        snprintf(ev_desc, sizeof(ev_desc), "TN: %d; %s", matched_slot, tetra_get_slot_call_mode_tag(state, matched_slot - 1));
      else
        snprintf(ev_desc, sizeof(ev_desc), "%s", tetra_get_call_mode_tag(state));
      tetra_log_data_event_ex(opts, state, state->tetra_last_src, state->tetra_last_tg, ev_desc, (prev_logged > 0));
      for (int s = 0; s < 4; s++)
      {
        int is_slot_match = (mask & (1U << (3 - s))) ||
                            (call_id != 0 && state->tetra_ts_call_id[s] == call_id);
        if (is_slot_match && new_logged > state->tetra_ts_logged[s])
          state->tetra_ts_logged[s] = new_logged;
      }
    }
  }
}

static void tetra_clear_caller_from_usage_ssi(dsd_state *state, uint32_t caller_ssi)
{
  if (caller_ssi == 0)
    return;
  for (int u = 4; u < 64; u++)
  {
    if (state->tetra_usage_ssi[u] == caller_ssi)
      state->tetra_usage_ssi[u] = 0;
  }
  for (int s = 0; s < 4; s++)
  {
    if (state->tetra_ts_tg[s] == caller_ssi)
      state->tetra_ts_tg[s] = 0;
  }
}

static void tetra_parse_cmce(dsd_opts *opts, dsd_state *state, const uint8_t *sdu, int sdu_len, uint32_t mac_ssi)
{
  /* sdu[0..2] == 001 (CMCE). sdu[3..7] is 5-bit CMCE PDU type */
  if (sdu_len < 8)
    return;

  static const char *CMCE_NAMES[32] = {
    "D-ALERT", "D-CALL-PROCEEDING", "D-CONNECT", "D-CONNECT-ACK",
    "D-DISCONNECT", "D-INFO", "D-RELEASE", "D-SETUP",
    "D-STATUS", "D-TX-CEASED", "D-TX-CONTINUE", "D-TX-GRANTED",
    "D-TX-WAIT", "D-TX-INTERRUPT", "D-CALL-RESTORE", "D-SDS-DATA",
    "D-FACILITY", "CMCE-17", "CMCE-18", "CMCE-19", "CMCE-20", "CMCE-21",
    "CMCE-22", "CMCE-23", "CMCE-24", "CMCE-25", "CMCE-26", "CMCE-27",
    "CMCE-28", "CMCE-29", "CMCE-30", "CMCE-31"
  };

  uint8_t pdu_t = (uint8_t)tetra_bits_to_uint(sdu, 3, 5);
  const char *pdu_name = CMCE_NAMES[pdu_t & 0x1F];
  uint32_t clean_mac_ssi = (mac_ssi != 0 && mac_ssi != 0xFFFFFFU) ? mac_ssi : 0;
  if (clean_mac_ssi != 0 && (pdu_t == 7 || pdu_t == 10 || pdu_t == 14))
  {
    state->tetra_last_tg = clean_mac_ssi;
  }

  if (pdu_t == 7 && sdu_len >= 43) /* D-SETUP (ETSI EN 300 392-2 Clause 14.7.1.10, Table 14.18) */
  {
    uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
    uint8_t simplex = (sdu[27] == 0) ? 1 : 0;
    uint8_t basic_svc = (uint8_t)tetra_bits_to_uint(sdu, 28, 8);
    uint8_t circuit_mode = (basic_svc >> 5) & 0x07U;
    uint8_t svc_enc = (basic_svc >> 4) & 0x01U;
    uint8_t comm_type = (basic_svc >> 2) & 0x03U;
    uint8_t prio = (uint8_t)tetra_bits_to_uint(sdu, 39, 4);

    int is_new_call = (state->tetra_call_id != call_id);
    if (is_new_call)
      state->tetra_last_src = 0;
    state->tetra_call_id = call_id;
    int cell_allows_aie = (state->tetra_aie_enc != 0 || state->tetra_sec_class >= 2 || state->tetra_la == 0);
    state->tetra_enc = cell_allows_aie ? svc_enc : 0;

    int is_enc = (state->tetra_enc || state->tetra_aie_enc || state->tetra_sec_class >= 2) ? 1 : 0;
    const char *comm_str = (comm_type == 0) ? "Indiv" :
                           (comm_type == 1) ? "Group" :
                           (comm_type == 2) ? "Direct" : "Bcast";
    const char *mode_str = (circuit_mode == 0) ? "Call" : "Data";
    snprintf(state->tetra_call_type, sizeof(state->tetra_call_type),
             "%s %s [%s]", comm_str, mode_str, tetra_get_tea_name(state, is_enc));

    /* Check O-bit (sdu[43]) and Type 2 optional elements to extract Calling Party SSI */
    int pos = 43;
    uint32_t setup_src = 0;
    if (pos < sdu_len && sdu[pos++] == 1)
    {
      if (pos < sdu_len)
      {
        uint8_t notif_ind = sdu[pos++];
        if (notif_ind && pos + 6 <= sdu_len)
          pos += 6;
      }
      if (pos < sdu_len)
      {
        uint8_t temp_addr = sdu[pos++];
        if (temp_addr && pos + 24 <= sdu_len)
          pos += 24;
      }
      if (pos < sdu_len && sdu[pos++] == 1)
      {
        setup_src = tetra_parse_cmce_party_ssi(sdu, sdu_len, &pos);
      }
    }
    /* Late-entry D-SETUP PDUs always carry the original call initiator (Calling Party),
     * not a subsequent PTT speaker from D-TX-GRANTED. Only apply setup_src when starting
     * a new CallID or before any transmitting party has been logged on the slot. */
    int allow_setup_src = is_new_call;
    if (!allow_setup_src)
    {
      for (int s = 0; s < 4; s++)
      {
        if (state->tetra_ts_call_id[s] == call_id)
        {
          if (state->tetra_ts_src[s] == 0 && state->tetra_ts_idle_count[s] == 0 && state->tetra_ts_logged[s] < 3)
            allow_setup_src = 1;
          break;
        }
      }
    }
    if (setup_src != 0 && allow_setup_src)
      state->tetra_last_src = setup_src;

    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
             "D-SETUP CallID:%u %s (%s) Prio:%u",
             call_id, state->tetra_call_type, simplex ? "Simplex" : "Duplex", prio);
    fprintf(stderr, " %sD-SETUP CallID:%u TGT:%u SRC:%u Mode:%s%s",
            KCYN, call_id, state->tetra_last_tg,
            (state->tetra_last_src != 0) ? state->tetra_last_src : setup_src,
            state->tetra_call_type, KNRM);

    tetra_apply_cmce_call_slots(opts, state, call_id, is_enc, 1);
  }
  else if (pdu_t == 11 && sdu_len >= 27) /* D-TX-GRANTED (ETSI EN 300 392-2 Clause 14.7.1.13, Table 14.21) */
  {
    uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
    uint8_t tx_grant = (uint8_t)tetra_bits_to_uint(sdu, 22, 2);
    uint8_t encr_ctrl = sdu[25];
    state->tetra_call_id = call_id;
    int cell_allows_aie = (state->tetra_aie_enc != 0 || state->tetra_sec_class >= 2 || state->tetra_la == 0);
    state->tetra_enc = cell_allows_aie ? encr_ctrl : 0;

    /* Check O-bit (sdu[27]) and Type 2 optional elements to extract Transmitting Party SSI */
    int pos = 27;
    uint32_t grant_src = 0;
    if (pos < sdu_len && sdu[pos++] == 1)
    {
      if (pos < sdu_len)
      {
        uint8_t notif = sdu[pos++];
        if (notif && pos + 6 <= sdu_len)
          pos += 6;
      }
      if (pos < sdu_len && sdu[pos++] == 1)
      {
        grant_src = tetra_parse_cmce_party_ssi(sdu, sdu_len, &pos);
      }
    }
    if (tx_grant == 0)
    {
      tetra_clear_caller_from_usage_ssi(state, clean_mac_ssi);
      if (grant_src != 0)
        state->tetra_last_src = grant_src;
      else if (clean_mac_ssi != 0)
        state->tetra_last_src = clean_mac_ssi;
    }
    else
    {
      if (clean_mac_ssi != 0)
        state->tetra_last_tg = clean_mac_ssi;
      if (grant_src != 0)
        state->tetra_last_src = grant_src;
    }
    if (tx_grant == 0 || tx_grant == 3)
    {
      for (int s = 0; s < 4; s++)
      {
        if (call_id != 0 && state->tetra_ts_call_id[s] == call_id)
          state->tetra_ts_idle_count[s] = 0;
      }
    }
    int is_enc = (state->tetra_enc || state->tetra_aie_enc || state->tetra_sec_class >= 2) ? 1 : 0;
    static const char *GRANT_STR[4] = {"Granted", "NotGranted", "Queued", "GrantedOther"};
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
             "D-TX-GRANTED CallID:%u (%s) SRC:%u",
             call_id, GRANT_STR[tx_grant & 3], state->tetra_last_src);
    fprintf(stderr, " %sD-TX-GRANTED CallID:%u (%s) TGT:%u SRC:%u%s",
            KCYN, call_id, GRANT_STR[tx_grant & 3], state->tetra_last_tg, state->tetra_last_src, KNRM);
    tetra_apply_cmce_call_slots(opts, state, call_id, is_enc, (tx_grant == 0 || tx_grant == 3));
  }
  else if ((pdu_t == 9 || pdu_t == 13) && sdu_len >= 23) /* D-TX-CEASED or D-TX-INTERRUPT */
  {
    uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
    state->tetra_call_id = call_id;
    state->tetra_last_src = 0;
    if (clean_mac_ssi != 0 && state->tetra_last_tg == 0)
      state->tetra_last_tg = clean_mac_ssi;
    for (int s = 0; s < 4; s++)
    {
      if (call_id != 0 && state->tetra_ts_call_id[s] == call_id)
      {
        if (clean_mac_ssi != 0 && state->tetra_ts_tg[s] == 0)
          state->tetra_ts_tg[s] = clean_mac_ssi;
        if (state->tetra_voice_slot == (uint8_t)(s + 1))
          tetra_release_voice_slot(opts, state, 1);
        else if (g_tetra_wav_slot == s)
          tetra_close_per_call_wav(opts, state);
        state->tetra_ts_src[s] = 0;
        state->tetra_ts_cand_count[s] = 0;
        if (state->tetra_ts_idle_count[s] == 0)
          state->tetra_ts_idle_count[s] = 1;
        if (state->tetra_ts_logged[s] == 3)
          state->tetra_ts_logged[s] = 2;
      }
    }
    if (state->tetra_burst_type == 3 && state->tetra_voice_slot == state->tetra_slot)
      tetra_unlock_voice_slot(opts, state, 1);
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str), "%s CallID:%u", pdu_name, call_id);
    fprintf(stderr, " %s CallID:%u", pdu_name, call_id);
  }
  else if ((pdu_t == 4 || pdu_t == 6) && sdu_len >= 27) /* D-DISCONNECT or D-RELEASE */
  {
    uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
    uint8_t cause = (uint8_t)tetra_bits_to_uint(sdu, 22, 5);
    for (int s = 0; s < 4; s++)
    {
      if (call_id != 0 && state->tetra_ts_call_id[s] == call_id)
      {
        tetra_reset_slot_state(opts, state, s);
      }
    }
    if (state->tetra_burst_type == 3 && state->tetra_slot >= 1 && state->tetra_slot <= 4)
      tetra_reset_slot_state(opts, state, state->tetra_slot - 1);
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
             "%s CallID:%u Cause:%u", pdu_name, call_id, cause);
    fprintf(stderr, " %s CallID:%u Cause:%u", pdu_name, call_id, cause);
  }
  else if (pdu_t == 2 && sdu_len >= 30) /* D-CONNECT (ETSI EN 300 392-2 Clause 14.7.1.4, Table 14.12) */
  {
    uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
    uint8_t hook = sdu[26];
    uint8_t simplex = (sdu[27] == 0) ? 1 : 0;

    if (state->tetra_call_id != call_id)
    {
      state->tetra_last_tg = 0;
      state->tetra_call_type[0] = '\0';
    }
    state->tetra_call_id = call_id;

    /* D-CONNECT is sent by the BS to the calling MS (clean_mac_ssi is SRC, not TGT) */
    tetra_clear_caller_from_usage_ssi(state, clean_mac_ssi);
    if (clean_mac_ssi != 0)
      state->tetra_last_src = clean_mac_ssi;

    int is_enc = tetra_is_cell_encrypted(state);
    int has_basic_svc = 0;
    int pos = 32;
    if (pos < sdu_len && sdu[pos++] == 1) /* O-bit */
    {
      if (pos < sdu_len && sdu[pos++] == 1 && pos + 4 <= sdu_len)
        pos += 4; /* Call priority */
      if (pos < sdu_len && sdu[pos++] == 1 && pos + 8 <= sdu_len)
      {
        uint8_t basic_svc = (uint8_t)tetra_bits_to_uint(sdu, pos, 8);
        pos += 8;
        uint8_t circuit_mode = (basic_svc >> 5) & 0x07U;
        uint8_t svc_enc = (basic_svc >> 4) & 0x01U;
        uint8_t comm_type = (basic_svc >> 2) & 0x03U;
        state->tetra_enc = svc_enc;
        is_enc = (state->tetra_enc || state->tetra_aie_enc || state->tetra_sec_class >= 2) ? 1 : 0;
        const char *comm_str = (comm_type == 0) ? "Indiv" :
                               (comm_type == 1) ? "Group" :
                               (comm_type == 2) ? "Direct" : "Bcast";
        const char *mode_str = (circuit_mode == 0) ? "Call" : "Data";
        snprintf(state->tetra_call_type, sizeof(state->tetra_call_type),
                 "%s %s [%s]", comm_str, mode_str, tetra_get_tea_name(state, is_enc));
        has_basic_svc = 1;
      }
      if (pos < sdu_len && sdu[pos++] == 1 && pos + 24 <= sdu_len)
      {
        uint32_t temp_ssi = tetra_bits_to_uint(sdu, pos, 24);
        pos += 24;
        if (temp_ssi != 0 && temp_ssi != 0xFFFFFFU)
          state->tetra_last_tg = temp_ssi;
      }
    }
    if (!has_basic_svc && state->tetra_call_type[0] == '\0')
    {
      snprintf(state->tetra_call_type, sizeof(state->tetra_call_type),
               "%s Call [%s]", (hook == 0 && simplex == 1) ? "Group" : "Indiv",
               tetra_get_tea_name(state, is_enc));
    }
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str), "D-CONNECT CallID:%u", call_id);
    fprintf(stderr, " %sD-CONNECT CallID:%u TGT:%u SRC:%u%s",
            KCYN, call_id, state->tetra_last_tg, state->tetra_last_src, KNRM);
    tetra_apply_cmce_call_slots(opts, state, call_id, is_enc, 1);
  }
  else if ((pdu_t == 0 || pdu_t == 1) && sdu_len >= 22) /* D-ALERT or D-CALL-PROCEEDING */
  {
    tetra_clear_caller_from_usage_ssi(state, clean_mac_ssi);
    if (clean_mac_ssi != 0)
      state->tetra_last_src = clean_mac_ssi;
    uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
    state->tetra_call_id = call_id;
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str), "%s CallID:%u", pdu_name, call_id);
    fprintf(stderr, " %s CallID:%u", pdu_name, call_id);
  }
  else if (pdu_t == 8 && sdu_len >= 26) /* D-STATUS */
  {
    int pos = 8;
    uint32_t status_src = tetra_parse_cmce_party_ssi(sdu, sdu_len, &pos);
    uint32_t status_tgt = clean_mac_ssi;

    if (pos + 16 <= sdu_len)
    {
      uint16_t status_code = (uint16_t)tetra_bits_to_uint(sdu, pos, 16);
      state->tetra_status_code = status_code;
      snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
               "D-STATUS Code:0x%04X (%s) SRC:%u",
               status_code, (status_code == 0x8000) ? "EMERGENCY" : "Status", status_src);
      fprintf(stderr, " %sD-STATUS SRC:%u TGT:%u Status:0x%04X%s%s",
              (status_code == 0x8000) ? KRED : KGRN,
              status_src, status_tgt, status_code,
              (status_code == 0x8000) ? " [EMERGENCY]" : "", KNRM);
      char ev_desc[128];
      snprintf(ev_desc, sizeof(ev_desc), "Status: 0x%04X;", status_code);
      tetra_log_data_event(opts, state, status_src, status_tgt, ev_desc);
    }
  }
  else if (pdu_t == 15 && sdu_len >= 12) /* D-SDS-DATA */
  {
    int pos = 8;
    uint32_t sds_src = tetra_parse_cmce_party_ssi(sdu, sdu_len, &pos);
    uint32_t sds_tgt = clean_mac_ssi;

    if (pos + 2 <= sdu_len)
    {
      uint8_t sds_type = (uint8_t)tetra_bits_to_uint(sdu, pos, 2);
      pos += 2;
      if (sds_type == 0 && pos + 16 <= sdu_len)
      {
        uint16_t val16 = (uint16_t)tetra_bits_to_uint(sdu, pos, 16);
        snprintf(state->tetra_sds_str, sizeof(state->tetra_sds_str), "SDS-1: 0x%04X (%u)", val16, val16);
        if (state->event_history_s != NULL)
          snprintf(state->event_history_s[0].Event_History_Items[0].text_message,
                   sizeof(state->event_history_s[0].Event_History_Items[0].text_message),
                   "%s", state->tetra_sds_str);
        fprintf(stderr, " %sD-SDS-DATA (Type 1) SRC:%u TGT:%u Val:0x%04X%s",
                KGRN, sds_src, sds_tgt, val16, KNRM);
      }
      else if (sds_type == 1 && pos + 32 <= sdu_len)
      {
        uint32_t val32 = tetra_bits_to_uint(sdu, pos, 32);
        snprintf(state->tetra_sds_str, sizeof(state->tetra_sds_str), "SDS-2: 0x%08X", val32);
        if (state->event_history_s != NULL)
          snprintf(state->event_history_s[0].Event_History_Items[0].text_message,
                   sizeof(state->event_history_s[0].Event_History_Items[0].text_message),
                   "%s", state->tetra_sds_str);
        fprintf(stderr, " %sD-SDS-DATA (Type 2) SRC:%u TGT:%u Val:0x%08X%s",
                KGRN, sds_src, sds_tgt, val32, KNRM);
      }
      else if (sds_type == 2 && pos + 64 <= sdu_len)
      {
        uint32_t hi = tetra_bits_to_uint(sdu, pos, 32);
        uint32_t lo = tetra_bits_to_uint(sdu, pos + 32, 32);
        snprintf(state->tetra_sds_str, sizeof(state->tetra_sds_str), "SDS-3: 0x%08X%08X", hi, lo);
        if (state->event_history_s != NULL)
          snprintf(state->event_history_s[0].Event_History_Items[0].text_message,
                   sizeof(state->event_history_s[0].Event_History_Items[0].text_message),
                   "%s", state->tetra_sds_str);
        fprintf(stderr, " %sD-SDS-DATA (Type 3) SRC:%u TGT:%u Val:0x%08X%08X%s",
                KGRN, sds_src, sds_tgt, hi, lo, KNRM);
      }
      else if (sds_type == 3 && pos + 11 <= sdu_len)
      {
        uint16_t len_bits = (uint16_t)tetra_bits_to_uint(sdu, pos, 11);
        pos += 11;
        int avail = sdu_len - pos;
        if (len_bits < avail)
          avail = len_bits;
        fprintf(stderr, " %sD-SDS-DATA (Type 4, %d bits) SRC:%u TGT:%u%s",
                KGRN, avail, sds_src, sds_tgt, KNRM);
        if (avail >= 8)
          tetra_decode_sds_type4(opts, state, &sdu[pos], avail, sds_src);
      }
      snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
               "D-SDS-DATA (Type %u) SRC:%u", sds_type + 1, sds_src);
      char cur_sds_key[600];
      snprintf(cur_sds_key, sizeof(cur_sds_key), "%u:%s:%s",
               (unsigned int)sds_type, state->tetra_sds_str, state->tetra_lip_str);
      if (strcmp(cur_sds_key, g_tetra_last_sds_key) != 0)
      {
        snprintf(g_tetra_last_sds_key, sizeof(g_tetra_last_sds_key), "%s", cur_sds_key);
        char ev_desc[128];
        snprintf(ev_desc, sizeof(ev_desc), "SDS-%u;", sds_type + 1);
        tetra_log_data_event(opts, state, sds_src, sds_tgt, ev_desc);
      }
      else if (state->event_history_s != NULL)
      {
        snprintf(state->event_history_s[0].Event_History_Items[0].text_message,
                 sizeof(state->event_history_s[0].Event_History_Items[0].text_message),
                 "%s", "BUMBLEBEETUNA");
        snprintf(state->event_history_s[0].Event_History_Items[0].gps_s,
                 sizeof(state->event_history_s[0].Event_History_Items[0].gps_s),
                 "%s", "BUMBLEBEETUNA");
      }
    }
  }
  else
  {
    if ((pdu_t == 0 || pdu_t == 1 || pdu_t == 3 || pdu_t == 10 || pdu_t == 12 || pdu_t == 14) && sdu_len >= 22)
    {
      /* D-ALERT (0), D-CALL-PROCEEDING (1), D-CONNECT-ACK (3), D-TX-CONTINUE (10), D-TX-WAIT (12), D-CALL-RESTORE (14) */
      uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
      if (state->tetra_call_id != call_id)
        state->tetra_last_src = 0;
      state->tetra_call_id = call_id;
      snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str), "%s CallID:%u", pdu_name, call_id);
      fprintf(stderr, " %s%s CallID:%u%s", KCYN, pdu_name, call_id, KNRM);
      if (pdu_t == 10 || pdu_t == 14)
      {
        int is_enc = tetra_is_cell_encrypted(state);
        tetra_apply_cmce_call_slots(opts, state, call_id, is_enc, 1);
      }
    }
    else
    {
      snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str), "CMCE %s", pdu_name);
      fprintf(stderr, " CMCE:%s", pdu_name);
    }
  }
}

static void tetra_parse_llc_pdu(dsd_opts *opts, dsd_state *state, const uint8_t *bits, int nbits, uint32_t mac_ssi)
{
  static const char *LLC_NAMES[16] = {
    "BL-ADATA", "BL-DATA", "BL-UDATA", "BL-ACK",
    "BL-ADATA-FCS", "BL-DATA-FCS", "BL-UDATA-FCS", "BL-ACK-FCS",
    "AL-SETUP", "AL-DATA", "AL-UDATA", "AL-ACK",
    "AL-RECONNECT", "LLC-13", "LLC-14", "AL-DISC"
  };

  uint8_t llc_type = 0;
  int hdr_len = 0, sdu_len = 0;
  if (!tetra_get_llc_sdu_bounds(bits, nbits, &llc_type, &hdr_len, &sdu_len))
    return;
  if (llc_type > 6 || (llc_type & 3U) == 3U || sdu_len < 6)
    return;

  fprintf(stderr, " LLC:%s", LLC_NAMES[llc_type & 0x0F]);

  const uint8_t *sdu = &bits[hdr_len];
  uint8_t mle_disc = (uint8_t)tetra_bits_to_uint(sdu, 0, 3);

  if (mle_disc == 2) /* CMCE (ETSI EN 300 392-2 Table 18.33: 010_2 = 2) */
  {
    tetra_parse_cmce(opts, state, sdu, sdu_len, mac_ssi);
  }
  else if (mle_disc == 1 && sdu_len >= 7) /* MM (Mobility Management: 001_2 = 1) */
  {
    static const char *MM_NAMES[16] = {
      "D-OTAR", "D-AUTH", "D-CK-CHANGE", "D-DISABLE",
      "D-ENABLE", "D-LOC-UPD-ACCEPT", "D-LOC-UPD-CMD", "D-LOC-UPD-REJECT",
      "D-LOC-UPD-PROC", "D-ATT-DET-GRP", "D-ATT-DET-GRP-ACK", "D-MM-STATUS",
      "MM-12", "MM-13", "MM-14", "MM-RESERVED"
    };
    uint8_t mm_pdu = (uint8_t)tetra_bits_to_uint(sdu, 3, 4);
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
             "MM %s (SSI:%u)", MM_NAMES[mm_pdu & 0x0F], mac_ssi);
    fprintf(stderr, " MM:%s", MM_NAMES[mm_pdu & 0x0F]);
    if (mac_ssi != 0 && mac_ssi != 0xFFFFFFU)
    {
      char ev_desc[128];
      snprintf(ev_desc, sizeof(ev_desc), "MM %s;", MM_NAMES[mm_pdu & 0x0F]);
      tetra_log_data_event(opts, state, mac_ssi, 0, ev_desc);
    }
  }
  else if (mle_disc == 5 && sdu_len >= 6) /* MLE (101_2 = 5) */
  {
    static const char *MLE_NAMES[8] = {
      "D-NEW-CELL", "D-PREPARE", "D-NWRK-BROADCAST", "D-NWRK-BCAST-EXT",
      "D-RESTORE-ACK", "D-RESTORE-FAIL", "D-CHAN-RESP", "MLE-7"
    };
    uint8_t mle_pdu = (uint8_t)tetra_bits_to_uint(sdu, 3, 3);
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str), "MLE %s", MLE_NAMES[mle_pdu & 7]);
    fprintf(stderr, " MLE:%s", MLE_NAMES[mle_pdu & 7]);

    /* Parse D-NWRK-BROADCAST (mle_pdu == 2) Neighbour Cell Information */
    if (mle_pdu == 2 && sdu_len >= 25)
    {
      int pos = 6;
      pos += 16; /* Cell re-selection parameters (16 bits) */
      pos += 2;  /* Cell service level (2 bits) */
      uint8_t o_bit = sdu[pos++];
      if (o_bit && pos < sdu_len)
      {
        /* Presence bit 1: TETRA network time (24 + 1 + 23 = 48 bits) */
        uint8_t p_time = sdu[pos++];
        if (p_time)
          pos += 48;
        /* Presence bit 2: Number of neighbour cells element (3 bits) */
        if (pos < sdu_len)
        {
          uint8_t p_nc = sdu[pos++];
          if (p_nc && pos + 3 <= sdu_len)
          {
            uint8_t num_nc = (uint8_t)tetra_bits_to_uint(sdu, pos, 3);
            pos += 3;
            for (uint8_t i = 0; i < num_nc; i++)
            {
              if (pos + 23 > sdu_len)
                break;
              pos += 5; /* Cell identifier (5 bits) */
              pos += 2; /* Cell re-selection types supported (2 bits) */
              pos += 1; /* Neighbour cell synchronised (1 bit) */
              pos += 2; /* Neighbour cell service level (2 bits) */
              uint16_t nc_carr = (uint16_t)tetra_bits_to_uint(sdu, pos, 12);
              pos += 12;
              uint8_t nc_o_bit = sdu[pos++];
              int ext_band = -1;
              int ext_off = -1;
              if (nc_o_bit)
              {
                /* 1: Main carrier number extension (10 bits) */
                if (pos < sdu_len && sdu[pos++])
                {
                  if (pos + 10 <= sdu_len)
                  {
                    ext_band = (int)tetra_bits_to_uint(sdu, pos, 4);
                    ext_off = (int)tetra_bits_to_uint(sdu, pos + 4, 2);
                  }
                  pos += 10;
                }
                /* 2: MCC (10 bits) */
                if (pos < sdu_len && sdu[pos++]) pos += 10;
                /* 3: MNC (14 bits) */
                if (pos < sdu_len && sdu[pos++]) pos += 14;
                /* 4: Neighbour LA (14 bits) */
                if (pos < sdu_len && sdu[pos++]) pos += 14;
                /* 5: Maximum MS transmit power (3 bits) */
                if (pos < sdu_len && sdu[pos++]) pos += 3;
                /* 6: Minimum RX access level (4 bits) */
                if (pos < sdu_len && sdu[pos++]) pos += 4;
                /* 7: Subscriber class (16 bits) */
                if (pos < sdu_len && sdu[pos++]) pos += 16;
                /* 8: BS Service Details (12 bits) */
                if (pos < sdu_len && sdu[pos++]) pos += 12;
                /* 9: Timeshare cell and AI encryption (5 bits) */
                if (pos < sdu_len && sdu[pos++]) pos += 5;
                /* 10: TDMA frame offset (6 bits) */
                if (pos < sdu_len && sdu[pos++]) pos += 6;
              }
              if (nc_carr > 0 && nc_carr < 4095)
              {
                double nc_freq = (ext_band > 0 && ext_off >= 0)
                                 ? tetra_calc_band_freq_mhz(ext_band, nc_carr, ext_off)
                                 : tetra_calc_carrier_freq_mhz(state, nc_carr);
                int found = 0;
                for (uint8_t k = 0; k < state->tetra_nc_count; k++)
                {
                  if (state->tetra_nc_carriers[k] == nc_carr)
                  {
                    if (nc_freq > 0.0)
                      state->tetra_nc_freqs[k] = nc_freq;
                    found = 1;
                    break;
                  }
                }
                if (!found && state->tetra_nc_count < 16)
                {
                  state->tetra_nc_carriers[state->tetra_nc_count] = nc_carr;
                  state->tetra_nc_freqs[state->tetra_nc_count] = nc_freq;
                  state->tetra_nc_count++;
                }
              }
            }
          }
        }
      }
    }
  }
  else if (mle_disc == 4) /* SNDCP Packet Data */
  {
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
             "SNDCP Packet Data (%d bits)", sdu_len);
    fprintf(stderr, " SNDCP (%d bits)", sdu_len);
  }
}

static int tetra_parse_chan_alloc(dsd_state *state, const uint8_t *bits, int pos, int nbits)
{
  if (pos + 23 > nbits)
    return nbits;

  uint8_t alloc_type = (uint8_t)tetra_bits_to_uint(bits, pos, 2);
  uint8_t ts_mask = (uint8_t)tetra_bits_to_uint(bits, pos + 2, 4);
  uint8_t ul_dl = (uint8_t)tetra_bits_to_uint(bits, pos + 6, 2);
  uint16_t carrier = (uint16_t)tetra_bits_to_uint(bits, pos + 10, 12);
  uint8_t ext_carr = bits[pos + 22];

  if (ul_dl != 0 && ts_mask != 0 && carrier > 0 && carrier < 4095)
  {
    if (state->tetra_main_carrier == 0 || carrier == state->tetra_main_carrier ||
        (state->tetra_dl_usage != 2 && state->tetra_ts_usage[0] != 2))
    {
      state->tetra_ts_mask = ts_mask;
    }
    else
    {
      state->tetra_ts_mask = 0;
    }
    if (state->tetra_dl_usage != 2 && state->tetra_ts_usage[0] != 2 &&
        carrier != state->tetra_main_carrier)
    {
      state->tetra_carrier = carrier;
    }
  }
  pos += 23;

  if (ext_carr && pos + 10 <= nbits)
    pos += 10;

  if (pos + 2 <= nbits)
  {
    uint8_t mon = (uint8_t)tetra_bits_to_uint(bits, pos, 2);
    pos += 2;
    if (mon == 0 && pos + 2 <= nbits)
      pos += 2;
  }

  /* Per ETSI EN 300 392-2 Table 21.64, ul_dl == 0 indicates Augmented Channel Allocation */
  if (ul_dl == 0 && pos + 28 <= nbits)
  {
    pos += 26;
    uint8_t napping_sts = (uint8_t)tetra_bits_to_uint(bits, pos, 2);
    pos += 2;
    if (napping_sts == 1 && pos + 11 <= nbits)
      pos += 11;
    if (pos + 4 <= nbits)
      pos += 4;
    if (pos < nbits)
    {
      uint8_t cond_a = bits[pos++];
      if (cond_a && pos + 16 <= nbits)
        pos += 16;
    }
    if (pos < nbits)
    {
      uint8_t cond_b = bits[pos++];
      if (cond_b && pos + 16 <= nbits)
        pos += 16;
    }
    if (pos < nbits)
      pos += 1;
  }

  fprintf(stderr, " [ChanAlloc Carr:%u TS:0x%X Type:%u UL/DL:%u]",
          carrier, ts_mask, alloc_type, ul_dl);
  return pos;
}

static int tetra_find_fill_bits(const uint8_t *bits, int nbits)
{
  int idx = nbits - 1;
  while (idx >= 0 && bits[idx] == 0)
    idx--;
  return (idx >= 0) ? idx : nbits;
}

static void tetra_dispatch_mac_sdu(dsd_opts *opts, dsd_state *state, const uint8_t *sdu_bits,
                                   int sdu_len, uint8_t encr_mode, uint32_t ssi, int update_last_tg)
{
  if (sdu_len <= 0)
    return;
  if (encr_mode == 0)
  {
    tetra_parse_llc_pdu(opts, state, sdu_bits, sdu_len, ssi);
  }
  else if (sdu_len <= 512)
  {
    uint8_t dec_bits[512];
    uint8_t used_alg = 0;
    if (tetra_try_decrypt_sdu(opts, state, sdu_bits, dec_bits, sdu_len, &used_alg))
    {
      if (update_last_tg && ssi != 0 && ssi != 0xFFFFFFU)
        state->tetra_last_tg = ssi;
      if (used_alg == 0)
        fprintf(stderr, " %s[BYPASS Clear]%s", KGRN, KNRM);
      else
        fprintf(stderr, " %s[TEA%u Decrypted Key: %s]%s", KGRN, used_alg, state->tetra_matched_key_str, KNRM);
      tetra_parse_llc_pdu(opts, state, dec_bits, sdu_len, ssi);
    }
  }
}

static int tetra_parse_mac_pdu_single(dsd_opts *opts, dsd_state *state, const uint8_t *bits, int nbits, int *pdu_end_flag)
{
  if (nbits < 16)
  {
    *pdu_end_flag = 1;
    return nbits;
  }

  uint8_t pdu_type = (uint8_t)tetra_bits_to_uint(bits, 0, 2);

  if (pdu_type == 0) /* MAC-RESOURCE */
  {
    uint8_t fill_bit = bits[2];
    uint8_t encr_mode = (uint8_t)tetra_bits_to_uint(bits, 4, 2);
    uint8_t len_ind = (uint8_t)tetra_bits_to_uint(bits, 7, 6);
    uint8_t addr_type = (uint8_t)tetra_bits_to_uint(bits, 13, 3);

    if (addr_type == 0 || len_ind <= 2 || (len_ind > 58 && len_ind < 62))
    {
      /* Null MAC-RESOURCE PDU */
      *pdu_end_flag = 1;
      return nbits;
    }

    int pos = 16;
    uint32_t ssi = 0;
    static const int ADDR_LENS[8] = {0, 24, 10, 24, 24, 34, 30, 34};
    int alen = ADDR_LENS[addr_type & 7];
    if (pos + alen > nbits)
    {
      *pdu_end_flag = 1;
      return nbits;
    }

    uint8_t addr_usage_marker = 0;
    if (addr_type == 1 || addr_type == 3 || addr_type == 4 || addr_type >= 5)
    {
      ssi = tetra_bits_to_uint(bits, pos, 24);
      if (addr_type == 6 && pos + 30 <= nbits)
        addr_usage_marker = (uint8_t)tetra_bits_to_uint(bits, pos + 24, 6);
    }
    else if (addr_type == 2)
      ssi = tetra_bits_to_uint(bits, pos, 10);
    pos += alen;

    /* Per ETSI EN 300 392-2 Table 21.55, addr_type == 6 (24-bit SSI + 6-bit Usage Marker)
     * is in the unencrypted MAC-RESOURCE header and maps the call's Usage Marker to its SSI */
    if (addr_usage_marker >= 4 && addr_usage_marker < 64 && ssi != 0 && ssi != 0xFFFFFFU)
    {
      state->tetra_usage_ssi[addr_usage_marker] = ssi;
      for (int s = 0; s < 4; s++)
      {
        if (state->tetra_ts_usage[s] == addr_usage_marker && state->tetra_ts_call_id[s] == 0)
          state->tetra_ts_tg[s] = ssi;
      }
    }

    int s_cur = (state->tetra_slot >= 1 && state->tetra_slot <= 4) ? (state->tetra_slot - 1) : -1;
    int slot_active_traffic = (s_cur >= 0 && state->tetra_ts_usage[s_cur] > 3 &&
                               state->tetra_ts_cand_count[s_cur] >= 3 &&
                               !(s_cur == 0 && state->tetra_ts_usage[0] == 2));
    int cell_allows_aie = (state->tetra_aie_enc != 0 || state->tetra_sec_class >= 2 || state->tetra_la == 0);

    if (encr_mode > 0 && cell_allows_aie && ssi != 0 && ssi != 0xFFFFFFU && slot_active_traffic)
      state->tetra_last_tg = ssi;
    if (cell_allows_aie && (encr_mode > 0 || slot_active_traffic))
      state->tetra_enc = encr_mode;

    state->tetra_ts_mask = 0;
    tetra_ensure_burst_hdr();

    /* Per ETSI EN 300 392-7 clause 4.2.3, encryption in MAC-RESOURCE starts
     * immediately after the Address field. When encr_mode == 0 (unencrypted PDU,
     * including D-NWRK-BROADCAST on encrypted cells), Power Control, Slot Granting,
     * and Channel Allocation elements are always in the clear! */
    if (encr_mode == 0)
    {
      /* Power control element (present if addr_type != 2) */
      if (addr_type != 2 && pos < nbits)
      {
        uint8_t pwr_flag = bits[pos++];
        if (pwr_flag)
          pos += 4;
      }

      /* Slot granting element */
      if (pos < nbits)
      {
        uint8_t slot_flag = bits[pos++];
        if (slot_flag)
          pos += 8;
      }

      /* Channel allocation element */
      if (pos < nbits)
      {
        uint8_t ca_flag = bits[pos++];
        if (ca_flag)
          pos = tetra_parse_chan_alloc(state, bits, pos, nbits);
      }
    }

    fprintf(stderr, " MAC-RES SSI:%u", ssi);
    if (encr_mode > 0)
    {
      const char *tea = tetra_get_tea_name(state, 1);
      fprintf(stderr, " %s[%s]%s", KYEL, tea, KNRM);
      snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
               "Encrypted (%s) SSI:%u", tea, ssi);

      /* Only attribute encrypted MAC-RESOURCE SSI to a timeslot call when received
       * on a confirmed active Traffic channel (usage > 3, cand_count >= 3) on an AIE-enabled cell */
      if (cell_allows_aie && slot_active_traffic)
      {
        int s = s_cur;
        if (state->tetra_call_type[0] == '\0') snprintf(state->tetra_call_type, sizeof(state->tetra_call_type), "Voice Call [%s]", tea);
        if (state->tetra_ts_type[s][0] == '\0') snprintf(state->tetra_ts_type[s], sizeof(state->tetra_ts_type[s]), "Voice Call [%s]", tea);
        if (ssi != 0 && ssi != 0xFFFFFFU)
          state->tetra_ts_tg[s] = ssi;
        state->tetra_ts_last_sym[s] = state->tetra_sym_count;
        if (state->tetra_scramb_valid && tetra_should_log_ssi(state, ssi))
        {
          char ev_desc[128];
          snprintf(ev_desc, sizeof(ev_desc), "TN: %d; %s", s + 1, tetra_get_slot_call_mode_tag(state, s));
          tetra_log_data_event_ex(opts, state, state->tetra_ts_src[s], ssi, ev_desc, (state->tetra_ts_logged[s] > 0));
          if (state->tetra_ts_logged[s] < 2) state->tetra_ts_logged[s] = 2;
        }
      }
    }

    if (len_ind == 0x3F) /* Start of MAC fragmentation */
    {
      int frag_bits = nbits - pos;
      if (fill_bit && frag_bits > 0)
        frag_bits = tetra_find_fill_bits(&bits[pos], frag_bits);
      if (frag_bits > 0 && frag_bits <= (int)sizeof(state->tetra_mac_frag_buf))
      {
        memcpy(state->tetra_mac_frag_buf, &bits[pos], frag_bits);
        state->tetra_mac_frag_len = (uint16_t)frag_bits;
        state->tetra_mac_frag_active = 1;
        state->tetra_mac_frag_enc = encr_mode;
        state->tetra_mac_frag_ssi = ssi;
      }
      *pdu_end_flag = 1;
      return nbits;
    }

    state->tetra_mac_frag_active = 0;
    state->tetra_mac_frag_len = 0;

    int pdu_end = (len_ind >= 4 && len_ind <= 58) ? (len_ind * 8) : nbits;
    if (pdu_end > nbits)
      pdu_end = nbits;
    int sdu_end = pdu_end;
    if (fill_bit && sdu_end > pos)
      sdu_end = pos + tetra_find_fill_bits(&bits[pos], sdu_end - pos);

    int sdu_len = sdu_end - pos;
    tetra_dispatch_mac_sdu(opts, state, &bits[pos], sdu_len, encr_mode, ssi, 1);
    if (len_ind >= 4 && len_ind <= 58)
      return len_ind * 8;
    *pdu_end_flag = 1;
    return nbits;
  }
  else if (pdu_type == 1) /* MAC-FRAG or MAC-END */
  {
    uint8_t sub_type = bits[2];
    uint8_t fill_bit = bits[3];
    if (sub_type == 0) /* MAC-FRAG */
    {
      int pos = 4;
      int frag_len = nbits - pos;
      if (fill_bit && frag_len > 0)
        frag_len = tetra_find_fill_bits(&bits[pos], frag_len);
      if (state->tetra_mac_frag_active && frag_len > 0 &&
          state->tetra_mac_frag_len + frag_len <= (int)sizeof(state->tetra_mac_frag_buf))
      {
        memcpy(&state->tetra_mac_frag_buf[state->tetra_mac_frag_len], &bits[pos], frag_len);
        state->tetra_mac_frag_len += (uint16_t)frag_len;
      }
      tetra_ensure_burst_hdr();
      fprintf(stderr, " MAC-FRAG (%u bits)", state->tetra_mac_frag_len);
      *pdu_end_flag = 1;
      return nbits;
    }
    else if (nbits >= 13) /* MAC-END */
    {
      uint8_t end_len_ind = (uint8_t)tetra_bits_to_uint(bits, 5, 6);
      int pos = 11;
      tetra_ensure_burst_hdr();
      if (state->tetra_mac_frag_active && state->tetra_mac_frag_enc == 0)
      {
        uint8_t slot_flag = bits[pos++];
        if (slot_flag)
          pos += 8;
        if (pos < nbits)
        {
          uint8_t ca_flag = bits[pos++];
          if (ca_flag)
            pos = tetra_parse_chan_alloc(state, bits, pos, nbits);
        }
      }
      int pdu_end = (end_len_ind >= 2 && end_len_ind <= 58) ? (end_len_ind * 8) : nbits;
      if (pdu_end > nbits)
        pdu_end = nbits;
      int rem_len = pdu_end - pos;
      if (fill_bit && rem_len > 0)
        rem_len = tetra_find_fill_bits(&bits[pos], rem_len);
      if (state->tetra_mac_frag_active && rem_len > 0 &&
          state->tetra_mac_frag_len + rem_len <= (int)sizeof(state->tetra_mac_frag_buf))
      {
        memcpy(&state->tetra_mac_frag_buf[state->tetra_mac_frag_len], &bits[pos], rem_len);
        state->tetra_mac_frag_len += (uint16_t)rem_len;
      }
      fprintf(stderr, " MAC-END (Total:%u bits)", state->tetra_mac_frag_len);
      if (state->tetra_mac_frag_active && state->tetra_mac_frag_len > 0)
      {
        tetra_dispatch_mac_sdu(opts, state, state->tetra_mac_frag_buf, state->tetra_mac_frag_len,
                               state->tetra_mac_frag_enc, state->tetra_mac_frag_ssi, 0);
      }
      state->tetra_mac_frag_active = 0;
      state->tetra_mac_frag_len = 0;
      if (end_len_ind >= 2 && end_len_ind <= 58)
        return end_len_ind * 8;
      *pdu_end_flag = 1;
      return nbits;
    }
  }
  else if (pdu_type == 2 && nbits >= 26) /* MAC-BROADCAST */
  {
    uint8_t bcast_type = (uint8_t)tetra_bits_to_uint(bits, 2, 2);
    if (bcast_type == 0) /* SYSINFO */
    {
      uint16_t main_carr = (uint16_t)tetra_bits_to_uint(bits, 4, 12);
      uint8_t f_band = (uint8_t)tetra_bits_to_uint(bits, 16, 4);
      uint8_t off_idx = (uint8_t)tetra_bits_to_uint(bits, 20, 2);
      uint8_t dup_spc = (uint8_t)tetra_bits_to_uint(bits, 22, 3);
      if (nbits < 124 || main_carr == 0 || f_band < 1 || f_band > 9)
      {
        *pdu_end_flag = 1;
        return nbits;
      }
      uint16_t prev_main = state->tetra_main_carrier;
      uint16_t prev_la = state->tetra_la;
      if (prev_main != 0 && prev_main != main_carr)
      {
        tetra_release_voice_slot(opts, state, 1);
        tetra_clear_cell_state(state);
        prev_la = 0;
      }
      state->tetra_main_carrier = main_carr;
      state->tetra_freq_band = f_band;
      state->tetra_offset_idx = off_idx;
      state->tetra_duplex_spacing = dup_spc;

      double main_freq = tetra_calc_carrier_freq_mhz(state, main_carr);
      if (main_freq > 0.0)
      {
        state->tetra_dl_freq_mhz = main_freq;
        if (state->tetra_ts_usage[0] == 2)
          state->tetra_carrier = main_carr;
        for (uint8_t k = 0; k < state->tetra_nc_count; k++)
        {
          if (state->tetra_nc_freqs[k] == 0.0 && state->tetra_nc_carriers[k] > 0)
            state->tetra_nc_freqs[k] = tetra_calc_carrier_freq_mhz(state, state->tetra_nc_carriers[k]);
        }
      }

      if (nbits >= 60 && bits[43] == 0)
        state->tetra_hyperframe = (uint16_t)tetra_bits_to_uint(bits, 44, 16);

      /* Extract LA (14 bits at 82..95) and BS Service Details (112..123) from D-MLE-SYSINFO */
      uint16_t la = (uint16_t)tetra_bits_to_uint(bits, 82, 14);
      if (la > 0)
        state->tetra_la = la;
      state->tetra_aie_enc = bits[122]; /* Bit 10 of BS Service Details: Air interface encryption */

      /* Check MAC-SYSINFO Optional field (bits 60..81): option_id == 3 is Extended Services Broadcast */
      uint8_t opt_id = (uint8_t)tetra_bits_to_uint(bits, 60, 2);
      if (opt_id == 3)
      {
        /* Security information element (8 bits at 62..69, ETSI EN 300 392-7):
         * bit 62: Authentication required on cell
         * bit 63: Security class 1 supported on cell
         * bit 64: Security class 2 or 3 supported on cell (0 = SC2, 1 = SC3)
         * bits 65..69: SCK number (5 bits, SCKN 1..32) */
        uint8_t sc1_supp = bits[63];
        uint8_t sc3_supp = bits[64];
        if (state->tetra_aie_enc != 0 || !sc1_supp)
          state->tetra_sec_class = sc3_supp ? 3 : 2;
        else if (sc1_supp)
          state->tetra_sec_class = 1;
      }
      if (state->tetra_aie_enc == 0 && state->tetra_sec_class == 0)
        state->tetra_sec_class = 1;

      int sysinfo_changed = (prev_main != main_carr ||
                             (state->tetra_la > 0 && prev_la != state->tetra_la));
      if (sysinfo_changed || state->tetra_pdu_str[0] == '\0')
      {
        if (state->tetra_dl_freq_mhz > 0.0)
        {
          snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
                   "SYSINFO Carrier:%u (%.06lf MHz) LA:%u",
                   main_carr, state->tetra_dl_freq_mhz, state->tetra_la);
        }
        else
        {
          snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
                   "SYSINFO Carrier:%u Band:%u LA:%u",
                   main_carr, f_band, state->tetra_la);
        }
      }
      if (sysinfo_changed || opts->payload == 1 ||
          (state->tetra_burst_type != 0 && time(NULL) != g_tetra_last_console_log_time))
      {
        tetra_ensure_burst_hdr();
        fprintf(stderr, " MAC-SYSINFO Carrier:%u", main_carr);
        if (state->tetra_dl_freq_mhz > 0.0)
          fprintf(stderr, " (%.06lf MHz)", state->tetra_dl_freq_mhz);
        if (state->tetra_la > 0)
          fprintf(stderr, " LA:%u", state->tetra_la);
      }
      *pdu_end_flag = 1;
      return nbits;
    }
    else if (bcast_type == 1) /* ACCESS-DEFINE */
    {
      if (state->tetra_pdu_str[0] == '\0')
        snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str), "MAC-BROADCAST ACCESS-DEFINE");
      if (opts->payload == 1)
      {
        tetra_ensure_burst_hdr();
        fprintf(stderr, " MAC-ACCESS-DEFINE");
      }
      *pdu_end_flag = 1;
      return nbits;
    }
  }
  *pdu_end_flag = 1;
  return nbits;
}

static void tetra_parse_mac_pdu(dsd_opts *opts, dsd_state *state, const uint8_t *bits, int nbits)
{
  int block_offset = 0;
  int pdu_end = 0;
  for (int i = 0; i < 5; i++)
  {
    if (block_offset > nbits - 16 || pdu_end)
      break;
    int consumed = tetra_parse_mac_pdu_single(opts, state, &bits[block_offset], nbits - block_offset, &pdu_end);
    if (consumed <= 0)
      break;
    block_offset += consumed;
  }
}

/*-------------------------------------------------------------------------------
 * TETRA Direct Mode Operation (DMO) DMAC-SYNC & DMAC-DATA/FRAG/END Parser
 * ETSI EN 300 396-3 Clauses 8.4 & 8.5 (Tables 21, 22, 23, 24, 25, 27, 28, 31, 45)
 *-----------------------------------------------------------------------------*/

static const char *tetra_dmo_msg_type_name(uint8_t msg_type)
{
  switch (msg_type)
  {
    case 0:  return "DM-RESERVED";
    case 1:  return "DM-SDS-OCCUPIED";
    case 2:  return "DM-TIMING-REQ";
    case 3:  return "DM-TIMING-ACK";
    case 8:  return "DM-SETUP";
    case 9:  return "DM-SETUP-PRES";
    case 10: return "DM-CONNECT";
    case 11: return "DM-DISCONNECT";
    case 12: return "DM-CONNECT-ACK";
    case 13: return "DM-OCCUPIED";
    case 14: return "DM-RELEASE";
    case 15: return "DM-TX-CEASED";
    case 16: return "DM-TX-REQUEST";
    case 17: return "DM-TX-ACCEPT";
    case 18: return "DM-PREEMPT";
    case 19: return "DM-PRE-ACCEPT";
    case 20: return "DM-REJECT";
    case 21: return "DM-INFO";
    case 22: return "DM-SDS-UDATA";
    case 23: return "DM-SDS-DATA";
    case 24: return "DM-SDS-ACK";
    default: return "DM-MAC";
  }
}

/* Parse common DMO address/message header + DM-SDU starting at `Frame countdown` in SCH/H or DMAC-DATA */
static void tetra_parse_dmac_body(dsd_opts *opts, dsd_state *state, const uint8_t *bits, int pos, int end)
{
  if (pos + 6 > end)
    return;

  uint8_t countdown = (uint8_t)tetra_bits_to_uint(bits, pos, 2);
  pos += 2;
  UNUSED(countdown);

  uint8_t dst_type = (uint8_t)tetra_bits_to_uint(bits, pos, 2);
  pos += 2;
  uint32_t dst_ssi = 0;
  if (dst_type <= 1 && pos + 24 <= end)
  {
    dst_ssi = tetra_bits_to_uint(bits, pos, 24);
    pos += 24;
  }

  if (pos + 2 > end)
    return;
  uint8_t src_type = (uint8_t)tetra_bits_to_uint(bits, pos, 2);
  pos += 2;
  uint32_t src_ssi = 0;
  if (src_type <= 1 && pos + 24 <= end)
  {
    src_ssi = tetra_bits_to_uint(bits, pos, 24);
    pos += 24;
  }

  uint32_t mni = 0;
  if ((dst_type == 0 || src_type == 0) && pos + 24 <= end)
  {
    mni = tetra_bits_to_uint(bits, pos, 24);
    pos += 24;
    uint16_t mcc = (uint16_t)((mni >> 14) & 0x3FFU);
    uint16_t mnc = (uint16_t)(mni & 0x3FFFU);
    if (mcc != 0 || mnc != 0)
    {
      state->tetra_mcc = mcc;
      state->tetra_mnc = mnc;
      snprintf(state->tetra_country_str, sizeof(state->tetra_country_str),
               "%s", tetra_lookup_country(mcc));
    }
  }

  if (dst_ssi != 0 && dst_ssi != 0xFFFFFFU)
  {
    state->tetra_last_tg = dst_ssi;
    state->tetra_ts_tg[0] = dst_ssi;
  }
  if (src_ssi != 0 && src_ssi != 0xFFFFFFU)
  {
    state->tetra_last_src = src_ssi;
    state->tetra_ts_src[0] = src_ssi;
  }

  if (mni != 0 || src_ssi != 0)
  {
    uint32_t full_mni = (mni != 0) ? mni : (((uint32_t)state->tetra_mcc << 14) | (state->tetra_mnc & 0x3FFFU));
    state->tetra_scramb_init = tetra_dmo_scramb_get_init(full_mni, src_ssi);
    state->tetra_scramb_valid = 1;
  }

  if (pos + 5 > end)
    return;
  uint8_t msg_type = (uint8_t)tetra_bits_to_uint(bits, pos, 5);
  pos += 5;
  const char *msg_name = tetra_dmo_msg_type_name(msg_type);

  int is_enc = (state->tetra_enc || state->tetra_aie_enc) ? 1 : 0;
  const char *tea = tetra_get_tea_name(state, is_enc);

  if (msg_type == 8 || msg_type == 9 || msg_type == 13) /* DM-SETUP / DM-SETUP PRES / DM-OCCUPIED */
  {
    snprintf(state->tetra_call_type, sizeof(state->tetra_call_type),
             "DMO Voice [%s]", tea);
    snprintf(state->tetra_ts_type[0], sizeof(state->tetra_ts_type[0]),
             "DMO Voice [%s]", tea);
  }
  else if (msg_type == 14 || msg_type == 15 || msg_type == 11) /* DM-RELEASE / DM-TX CEASED / DM-DISCONNECT */
  {
    if (msg_type == 14 || msg_type == 11)
    {
      state->tetra_call_type[0] = '\0';
      state->tetra_ts_type[0][0] = '\0';
      tetra_release_voice_slot(opts, state, 1);
    }
    else
    {
      snprintf(state->tetra_call_type, sizeof(state->tetra_call_type), "DMO TX Ceased");
    }
  }
  else if (msg_type == 22 || msg_type == 23) /* DM-SDS UDATA / DM-SDS DATA */
  {
    snprintf(state->tetra_call_type, sizeof(state->tetra_call_type), "DMO SDS");
  }

  snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
           "%s TGT:%u SRC:%u", msg_name, state->tetra_last_tg, state->tetra_last_src);

  tetra_ensure_burst_hdr();
  fprintf(stderr, " %s", msg_name);
  if (dst_ssi != 0)
    fprintf(stderr, " TGT:%u", dst_ssi);
  if (src_ssi != 0)
    fprintf(stderr, " SRC:%u", src_ssi);

  /* Event History logging for new DMO call / SDS */
  if ((msg_type == 8 || msg_type == 9 || msg_type == 13) && state->tetra_last_tg != 0 &&
      tetra_should_log_ssi(state, state->tetra_last_tg))
  {
    char ev_str[256];
    snprintf(ev_str, sizeof(ev_str), "DMO (%s);",
             (state->tetra_dmo_comm_type == 1) ? "DM-REP" :
             (state->tetra_dmo_comm_type == 2) ? "DM-GATE" :
             (state->tetra_dmo_comm_type == 3) ? "DM-REP/GATE" : "MS-MS");
    tetra_log_data_event_ex(opts, state, state->tetra_last_src, state->tetra_last_tg, ev_str, 0);
  }

  /* Decode DMO SDS UDATA / DATA payload if unencrypted */
  if ((msg_type == 22 || msg_type == 23) && !is_enc && pos + 10 <= end)
  {
    /* Skip 8 bits Message dependent elements (SDS time remaining 4, trans 1, prio 2, fcs 1) */
    pos += 8;
    if (pos + 3 <= end)
    {
      uint8_t add_addr_flag = bits[pos] & 1U;
      pos += 1;
      if (add_addr_flag)
        pos += 52; /* 4-bit type + 48-bit Calling/Called TSI */
      if (pos + 2 <= end)
      {
        uint8_t sdti = (uint8_t)tetra_bits_to_uint(bits, pos, 2);
        pos += 2;
        if (sdti == 0 && pos + 16 <= end)
        {
          uint16_t val16 = (uint16_t)tetra_bits_to_uint(bits, pos, 16);
          state->tetra_status_code = val16;
          snprintf(state->tetra_sds_str, sizeof(state->tetra_sds_str),
                   "Status: %u (0x%04X)", val16, val16);
          fprintf(stderr, " %s[Status: 0x%04X]%s", KCYN, val16, KNRM);
        }
        else if (sdti == 3 && pos + 16 <= end)
        {
          uint16_t sds_len = (uint16_t)tetra_bits_to_uint(bits, pos, 11);
          pos += 11;
          int sdu_end = (pos + (int)sds_len <= end) ? (pos + (int)sds_len) : end;
          if (sdu_end - pos >= 8)
            tetra_decode_sds_type4(opts, state, &bits[pos], sdu_end - pos, state->tetra_last_src);
        }
      }
    }
  }
}

static void tetra_parse_dmac_sync(dsd_opts *opts, dsd_state *state,
                                  const uint8_t *sch_s_bits, const uint8_t *sch_h_bits)
{
  uint8_t sys_code = (uint8_t)tetra_bits_to_uint(sch_s_bits, 0, 4);
  uint8_t sync_pdu_type = (uint8_t)tetra_bits_to_uint(sch_s_bits, 4, 2);
  if (sync_pdu_type != 0 || sys_code > 7)
    return;

  uint8_t comm_type = (uint8_t)tetra_bits_to_uint(sch_s_bits, 6, 2);
  uint8_t tn = (uint8_t)(tetra_bits_to_uint(sch_s_bits, 10, 2) + 1);
  uint8_t fn = (uint8_t)tetra_bits_to_uint(sch_s_bits, 12, 5);
  uint8_t aie_state = (uint8_t)tetra_bits_to_uint(sch_s_bits, 17, 2);

  state->tetra_mode = 1;
  state->tetra_dmo_comm_type = comm_type;
  state->tetra_slot = tn;
  state->tetra_frame = fn;
  state->tetra_bsch_tn = tn;
  state->tetra_bsch_fn = fn;

  if (aie_state > 0)
  {
    uint8_t ksg = (uint8_t)tetra_bits_to_uint(sch_s_bits, 19, 4);
    state->tetra_aie_enc = 1;
    state->tetra_enc = 1;
    state->tetra_sec_class = 2;
    if (ksg < 8)
      state->tetra_enc_alg = (uint8_t)(ksg + 1);
  }
  else
  {
    state->tetra_aie_enc = 0;
    state->tetra_enc = 0;
    state->tetra_sec_class = 1;
  }

  /* Parse SCH/H (124 bits, Table 22) */
  int end = 124;
  uint8_t fill_ind = sch_h_bits[10] & 1U;
  if (fill_ind)
    end = tetra_find_fill_bits(sch_h_bits, 124);

  uint8_t frag_flag = sch_h_bits[11] & 1U;
  int pos = 12;
  if (frag_flag)
  {
    uint8_t n_slots = (uint8_t)tetra_bits_to_uint(sch_h_bits, pos, 4);
    pos += 4;
    state->tetra_mac_frag_active = (n_slots > 0) ? 1 : 0;
    state->tetra_mac_frag_len = 0;
  }

  tetra_parse_dmac_body(opts, state, sch_h_bits, pos, end);
}

static void tetra_parse_dmac_pdu(dsd_opts *opts, dsd_state *state, const uint8_t *bits, int nbits)
{
  if (nbits < 16)
    return;

  uint8_t mac_type = (uint8_t)tetra_bits_to_uint(bits, 0, 2);
  if (mac_type == 0) /* DMAC-DATA (Table 23) */
  {
    uint8_t fill_ind = bits[2] & 1U;
    uint8_t second_stolen = bits[3] & 1U;
    uint8_t frag_flag = bits[4] & 1U;
    uint8_t null_pdu = bits[5] & 1U;
    UNUSED(second_stolen);

    if (null_pdu)
      return;

    int end = fill_ind ? tetra_find_fill_bits(bits, nbits) : nbits;
    int pos = 6;
    if (frag_flag)
    {
      uint8_t n_slots = (uint8_t)tetra_bits_to_uint(bits, pos, 4);
      pos += 4;
      state->tetra_mac_frag_active = (n_slots > 0) ? 1 : 0;
      state->tetra_mac_frag_len = 0;
    }

    tetra_parse_dmac_body(opts, state, bits, pos, end);
  }
  else if (mac_type == 1) /* DMAC-FRAG (bit 2 == 0) or DMAC-END (bit 2 == 1) (Tables 24 & 25) */
  {
    uint8_t is_end = bits[2] & 1U;
    uint8_t fill_ind = bits[3] & 1U;
    int end = fill_ind ? tetra_find_fill_bits(bits, nbits) : nbits;
    tetra_ensure_burst_hdr();
    fprintf(stderr, " %s", is_end ? "DMAC-END" : "DMAC-FRAG");
    if (is_end && end > 4)
    {
      state->tetra_mac_frag_active = 0;
      state->tetra_mac_frag_len = 0;
    }
  }
}

/*-------------------------------------------------------------------------------
 * 144 kHz (8 sub-samples/symbol) Cubic Interpolator & Symbol Timing Recovery
 *-----------------------------------------------------------------------------*/

static int tetra_get_samplerate(dsd_opts *opts)
{
  if (opts->audio_in_type == 2 && opts->audio_in_file_info != NULL && opts->audio_in_file_info->samplerate > 0)
    return opts->audio_in_file_info->samplerate;
  if (opts->audio_in_type == 8 && opts->wav_sample_rate > 0)
    return opts->wav_sample_rate;
  return 48000 * ((opts->wav_interpolator > 0) ? opts->wav_interpolator : 1);
}

static inline int16_t tetra_clamp_i16(int v)
{
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

int tetra_prepare_symbol_steps(dsd_opts *opts, dsd_state *state)
{
  int sr = tetra_get_samplerate(opts);
  state->tetra_sub_read_idx += 8;
  /* Maintain 20 sub-samples (2.5 symbols) of lookahead in tetra_sub_buf */
  uint32_t target_w = state->tetra_sub_read_idx + 20;
  int deficit = (int)(target_w - state->tetra_sub_write_idx);
  if (deficit <= 0)
    return 0;

  if (sr == 18000)
    return (deficit + 7) / 8;
  else if (sr == 36000)
    return (deficit + 3) / 4;
  else if (sr == 54000)
    return (deficit * 3 + 7) / 8;
  else if (sr == 72000)
    return (deficit + 1) / 2;
  else if (sr == 96000)
    return (deficit * 2 + 2) / 3;
  else
    return (deficit + 2) / 3; /* 48000 Hz -> 3 sub-samples per input sample */
}

void tetra_push_audio_sample(dsd_opts *opts, dsd_state *state, short sample)
{
  int sr = tetra_get_samplerate(opts);
  int s0 = state->tetra_raw_hist[0];
  int s1 = state->tetra_raw_hist[1];
  int s2 = state->tetra_raw_hist[2];
  int s3 = sample;
  state->tetra_raw_hist[0] = (int16_t)s1;
  state->tetra_raw_hist[1] = (int16_t)s2;
  state->tetra_raw_hist[2] = (int16_t)s3;

  if (sr == 48000)
  {
    /* Exact 4-tap Catmull-Rom / Lagrange cubic interpolation at t = 0, 1/3, 2/3 */
    state->tetra_sub_buf[state->tetra_sub_write_idx++ & 8191] = (int16_t)s1;
    state->tetra_sub_buf[state->tetra_sub_write_idx++ & 8191] =
        tetra_clamp_i16((-s0 + 12 * s1 + 8 * s2 - s3) / 18);
    state->tetra_sub_buf[state->tetra_sub_write_idx++ & 8191] =
        tetra_clamp_i16((-s0 + 8 * s1 + 12 * s2 - s3) / 18);
  }
  else
  {
    int n_subs = 3;
    if (sr == 18000) n_subs = 8;
    else if (sr == 36000) n_subs = 4;
    else if (sr == 72000) n_subs = 2;
    else if (sr == 54000)
    {
      state->tetra_sps_phase = (state->tetra_sps_phase + 1) % 3;
      n_subs = (state->tetra_sps_phase == 0) ? 2 : 3;
    }
    else if (sr == 96000)
    {
      state->tetra_sps_phase = (state->tetra_sps_phase + 1) % 2;
      n_subs = (state->tetra_sps_phase == 0) ? 1 : 2;
    }

    float c0 = (float)s1;
    float c1 = 0.5f * (float)(s2 - s0);
    float c2 = (float)s0 - 2.5f * (float)s1 + 2.0f * (float)s2 - 0.5f * (float)s3;
    float c3 = 0.5f * (float)(s3 - s0) + 1.5f * (float)(s1 - s2);
    for (int m = 0; m < n_subs; m++)
    {
      float t = (float)m / (float)n_subs;
      float val = ((c3 * t + c2) * t + c1) * t + c0;
      state->tetra_sub_buf[state->tetra_sub_write_idx++ & 8191] = tetra_clamp_i16((int)val);
    }
  }
}

int tetra_finish_symbol(dsd_opts *opts, dsd_state *state)
{
  UNUSED(opts);
  if (state->samplesPerSymbol == 0)
    state->samplesPerSymbol = 2;
  int sym = state->tetra_sub_buf[state->tetra_sub_read_idx & 8191];
  state->tetra_sym_ring[state->tetra_sym_count & 1023] = sym;
  state->tetra_sym_count++;
  state->tetra_bin_write_count = state->tetra_sym_count;
  return sym;
}

/*-------------------------------------------------------------------------------
 * Sync Detection in dsd_frame_sync.c & Burst Processing in dsd_frame.c
 *-----------------------------------------------------------------------------*/

static int tetra_dibit_bit_diff(uint8_t d1, uint8_t d2)
{
  uint8_t x = (d1 ^ d2) & 3U;
  return (x & 1U) + ((x >> 1) & 1U);
}

static void tetra_apply_levels(dsd_opts *opts, dsd_state *state, int smax, int smin)
{
  state->max = smax;
  state->min = smin;
  state->center = (smax + smin) / 2;
  state->umid = state->center + ((smax - state->center) * 2 / 3);
  state->lmid = state->center + ((smin - state->center) * 2 / 3);
  state->maxref = (int)(smax * 0.80F);
  state->minref = (int)(smin * 0.80F);
  int msz = (opts->msize > 0 && opts->msize <= 1024) ? opts->msize : 15;
  for (int i = 0; i < msz; i++)
  {
    state->maxbuf[i] = smax;
    state->minbuf[i] = smin;
  }
  for (int i = 0; i < opts->ssize && i < 128; i++)
  {
    state->sbuf[i] = (i & 1) ? smax : smin;
  }
}

static inline int16_t tetra_clamp_soft(int v)
{
  if (v > 127) return 127;
  if (v < -127) return -127;
  return (int16_t)v;
}

/* Convert one analog symbol (or dibit if is_bin) into 2 hard bits and 2 soft bits [-127..+127].
 * Positive soft value (+127) = bit 0; negative soft value (-127) = bit 1.
 * swap == 0: dsd-fme / TetraEar processor.py convention (b0 = sign, b1 = mag)
 * swap == 1: ETSI EN 300 392-2 Table 5.1 convention (b0 = mag, b1 = sign)
 */
static void tetra_symbol_to_bits(int sym, int is_bin, float center, float step,
                                 int inv, int swap,
                                 uint8_t *b0, uint8_t *b1,
                                 int16_t *s0, int16_t *s1)
{
  if (is_bin)
  {
    uint8_t d = (uint8_t)(sym & 3);
    if (inv)
      d ^= 2U;
    uint8_t bit_sign = (d >> 1) & 1U;
    uint8_t bit_mag = d & 1U;
    if (swap)
    {
      *b0 = bit_mag;
      *b1 = bit_sign;
    }
    else
    {
      *b0 = bit_sign;
      *b1 = bit_mag;
    }
    *s0 = (*b0 == 0) ? 127 : -127;
    *s1 = (*b1 == 0) ? 127 : -127;
    return;
  }

  float denom = (step > 1.0f) ? step : 1.0f;
  float y = ((float)sym - center) / denom;
  if (inv)
    y = -y;

  uint8_t bit_sign = (y < 0.0f) ? 1U : 0U;
  uint8_t bit_mag = (fabsf(y) > 2.0f) ? 1U : 0U;

  int sv_sign = (int)(y * 64.0f);
  if (sv_sign == 0)
    sv_sign = (y >= 0.0f) ? 1 : -1;
  int sv_mag = (int)((2.0f - fabsf(y)) * 64.0f);
  if (sv_mag == 0)
    sv_mag = (fabsf(y) <= 2.0f) ? 1 : -1;

  if (swap)
  {
    *b0 = bit_mag;
    *b1 = bit_sign;
    *s0 = tetra_clamp_soft(sv_mag);
    *s1 = tetra_clamp_soft(sv_sign);
  }
  else
  {
    *b0 = bit_sign;
    *b1 = bit_mag;
    *s0 = tetra_clamp_soft(sv_sign);
    *s1 = tetra_clamp_soft(sv_mag);
  }
}

static int tetra_decode_block_cch_soft(const int16_t *raw_soft, int raw_len,
                                       int deint_a, int vit_steps, int payload_len,
                                       uint32_t scramb_init, uint8_t *out_bits)
{
  int16_t descramb[512];
  int16_t deint[512];
  int16_t depunct[1200];
  uint8_t vit_out[300];

  tetra_scramb_soft(scramb_init, raw_soft, descramb, raw_len);
  tetra_block_deinterleave_soft(descramb, deint, raw_len, deint_a);
  tetra_rcpc_depunct_2_3_soft(deint, raw_len, depunct, vit_steps * 4);
  tetra_viterbi_decode_cch_soft(depunct, vit_steps, payload_len, vit_out);

  uint16_t crc = tetra_crc16_ccitt_bits(vit_out, payload_len + 16);
  memcpy(out_bits, vit_out, payload_len);
  return (crc == TETRA_CRC_OK) ? 1 : 0;
}

/* Evaluate an L-symbol training sequence in tetra_sub_buf ending at end_sub_idx
 * using Ordinary Least Squares (OLS) linear regression and Pearson correlation. */
static float tetra_eval_train_seq_analog(const dsd_state *state, uint32_t end_sub_idx,
                                         const uint8_t *seq_dibits, int L, int swap,
                                         int *out_inv, int *out_errs,
                                         float *out_center, float *out_step)
{
  float x[20];
  float p[20];
  float sum_x = 0.0f, sum_p = 0.0f;

  for (int i = 0; i < L; i++)
  {
    int k = L - 1 - i;
    x[i] = (float)state->tetra_sub_buf[(end_sub_idx - (uint32_t)(8 * k)) & 8191];
    uint8_t d = seq_dibits[i] & 3U;
    uint8_t b0 = (d >> 1) & 1U;
    uint8_t b1 = d & 1U;
    uint8_t b_sign = swap ? b1 : b0;
    uint8_t b_mag = swap ? b0 : b1;
    float mag = b_mag ? 3.0f : 1.0f;
    p[i] = b_sign ? -mag : mag;
    sum_x += x[i];
    sum_p += p[i];
  }

  float mean_x = sum_x / (float)L;
  float mean_p = sum_p / (float)L;
  float cov = 0.0f, var_x = 0.0f, var_p = 0.0f;
  for (int i = 0; i < L; i++)
  {
    float dx = x[i] - mean_x;
    float dp = p[i] - mean_p;
    cov += dx * dp;
    var_x += dx * dx;
    var_p += dp * dp;
  }

  if (var_x < (float)L * 50.0f || var_p < 1.0f)
  {
    *out_errs = L * 2;
    return 0.0f;
  }

  int inv = (cov < 0.0f) ? 1 : 0;
  float slope = cov / var_p;
  float step = fabsf(slope);
  if (step < 5.0f || (state->tetra_tdma_locked && state->tetra_step_level > 100.0f && step < state->tetra_step_level * 0.35f))
  {
    *out_errs = L * 2;
    return 0.0f;
  }

  float center = mean_x - slope * mean_p;
  float corr = fabsf(cov) / sqrtf(var_x * var_p);

  int errs = 0;
  for (int i = 0; i < L; i++)
  {
    uint8_t b0, b1;
    int16_t s0, s1;
    tetra_symbol_to_bits((int)x[i], 0, center, step, inv, swap, &b0, &b1, &s0, &s1);
    uint8_t d = (uint8_t)((b0 << 1) | b1);
    errs += tetra_dibit_bit_diff(d, seq_dibits[i]);
  }

  *out_inv = inv;
  *out_errs = errs;
  *out_center = center;
  *out_step = step;
  return corr;
}

static void tetra_extract_lookback_soft(const dsd_state *state, int is_bin, uint32_t sync_end_idx,
                                        int sync_end_sym, int start_sym, int count_syms,
                                        float center, float step, int inv, int swap,
                                        uint8_t *out_bits, int16_t *out_soft)
{
  for (int i = 0; i < count_syms; i++)
  {
    int sym = start_sym + i;
    int k = sync_end_sym - sym;
    int val = is_bin ? state->tetra_sym_ring[(sync_end_idx - (uint32_t)k) & 1023]
                     : state->tetra_sub_buf[(sync_end_idx - (uint32_t)(8 * k)) & 8191];
    uint8_t b0, b1;
    tetra_symbol_to_bits(val, is_bin, center, step, inv, swap,
                         &b0, &b1, &out_soft[i * 2], &out_soft[i * 2 + 1]);
    if (out_bits != NULL)
    {
      out_bits[i * 2] = b0;
      out_bits[i * 2 + 1] = b1;
    }
  }
}

/* Test-decode SB1 (60 symbols = 120 bits at burst symbols 47..106) from lookback buffer
 * when a candidate SYNC training sequence (symbols 107..125) is detected. */
static int tetra_try_presync_sb1(const dsd_state *state, int is_bin, uint32_t sync_end_idx,
                                 float center, float step, int inv, int swap)
{
  int16_t sb1_soft[120];
  uint8_t sb1_bits[60];
  tetra_extract_lookback_soft(state, is_bin, sync_end_idx, 125, 47, 60,
                              center, step, inv, swap, NULL, sb1_soft);
  return tetra_decode_block_cch_soft(sb1_soft, 120, 11, 80, 60, TETRA_SCRAMB_INIT_DEFAULT, sb1_bits);
}

/* Test-decode BLK1 (108 symbols = 216 bits at burst symbols 7..114 for TMO NDB, or 14..121 for DMO DNB)
 * from lookback buffer when a candidate NORM training sequence (symbols 122..132) is detected. */
static int tetra_try_presync_blk1(const dsd_state *state, int is_bin, uint32_t sync_end_idx,
                                  float center, float step, int inv, int swap)
{
  int16_t blk1_soft[216];
  uint8_t blk1_bits[124];
  uint32_t scramb = state->tetra_scramb_valid ? state->tetra_scramb_init : TETRA_SCRAMB_INIT_DEFAULT;
  if (state->tetra_mode != 1)
  {
    tetra_extract_lookback_soft(state, is_bin, sync_end_idx, 132, 7, 108,
                                center, step, inv, swap, NULL, blk1_soft);
    if (tetra_decode_block_cch_soft(blk1_soft, 216, 101, 144, 124, scramb, blk1_bits))
      return 1;
  }
  tetra_extract_lookback_soft(state, is_bin, sync_end_idx, 132, 14, 108,
                              center, step, inv, swap, NULL, blk1_soft);
  if (tetra_decode_block_cch_soft(blk1_soft, 216, 101, 144, 124, scramb, blk1_bits))
    return 1;
  if (scramb != TETRA_SCRAMB_INIT_DEFAULT &&
      tetra_decode_block_cch_soft(blk1_soft, 216, 101, 144, 124, TETRA_SCRAMB_INIT_DEFAULT, blk1_bits))
    return 1;
  return 0;
}

static int tetra_is_on_analog_grid(const dsd_state *state, uint32_t best_sub, int sync_end_sym)
{
  if (!state->tetra_tdma_locked)
    return 0;
  int mod2040 = (int)((int32_t)(best_sub - (uint32_t)(sync_end_sym * 8) - state->tetra_last_sync_sub) % 2040);
  if (mod2040 < 0)
    mod2040 += 2040;
  int dist_subs = (mod2040 <= 1020) ? mod2040 : (2040 - mod2040);
  return (dist_subs <= 16) ? 1 : 0;
}

static int tetra_commit_analog_sync(dsd_opts *opts, dsd_state *state, uint32_t cur_sub,
                                    uint32_t best_sub, uint8_t burst_type, int swap,
                                    int inv, float center, float step)
{
  state->tetra_prev_sub_read_idx = cur_sub;
  state->tetra_sub_read_idx = best_sub;
  state->tetra_sync_sub_idx = best_sub;
  state->tetra_sync_sym_count = state->tetra_sym_count;
  state->tetra_bit_swap = (uint8_t)swap;
  state->tetra_center = center;
  state->tetra_step_level = step;
  tetra_apply_levels(opts, state,
                     (int)(center + 3.0f * step),
                     (int)(center - 3.0f * step));
  state->tetra_burst_type = burst_type;
  opts->inverted_tetra = inv;
  return inv ? 42 : 41;
}

static int tetra_eval_train_seq_bin(const dsd_state *state, uint32_t cur_sym,
                                    const uint8_t *seq_dibits, int L, int inv, int swap, int max_errs)
{
  int errs = 0;
  for (int i = 0; i < L; i++)
  {
    int k = L - 1 - i;
    uint8_t b0, b1;
    int16_t s0, s1;
    tetra_symbol_to_bits(state->tetra_sym_ring[(cur_sym - (uint32_t)k) & 1023],
                         1, 0.0f, 1.0f, inv, swap, &b0, &b1, &s0, &s1);
    uint8_t d = (uint8_t)((b0 << 1) | b1);
    errs += tetra_dibit_bit_diff(d, seq_dibits[i]);
    if (errs > max_errs)
      break;
  }
  return errs;
}

static int tetra_commit_bin_sync(dsd_opts *opts, dsd_state *state, uint32_t cur_sym,
                                 uint8_t burst_type, int swap, int inv)
{
  state->tetra_burst_type = burst_type;
  state->tetra_bit_swap = (uint8_t)swap;
  state->tetra_sync_sub_idx = cur_sym;
  state->tetra_sync_sym_count = state->tetra_sym_count;
  opts->inverted_tetra = inv;
  return inv ? 42 : 41;
}

static int tetra_is_analog_corr_peak(const dsd_state *state, uint32_t best_sub, int best_delta,
                                     int swap, float best_corr,
                                     const uint8_t * const *seqs, int n_seqs, int best_s, int L)
{
  int dummy_inv, dummy_errs;
  float dummy_C, dummy_A;
  float corr_m8 = 0.0f, corr_p8 = 0.0f, corr_p16 = 0.0f;
  float corr_p1 = (best_delta == 3)
                  ? tetra_eval_train_seq_analog(state, best_sub + 1, seqs[best_s], L, swap,
                                                &dummy_inv, &dummy_errs, &dummy_C, &dummy_A)
                  : 0.0f;
  for (int s = 0; s < n_seqs; s++)
  {
    float cm8 = tetra_eval_train_seq_analog(state, best_sub - 8, seqs[s], L, swap,
                                            &dummy_inv, &dummy_errs, &dummy_C, &dummy_A);
    if (cm8 > corr_m8) corr_m8 = cm8;
    float c8 = tetra_eval_train_seq_analog(state, best_sub + 8, seqs[s], L, swap,
                                           &dummy_inv, &dummy_errs, &dummy_C, &dummy_A);
    if (c8 > corr_p8) corr_p8 = c8;
    float c16 = tetra_eval_train_seq_analog(state, best_sub + 16, seqs[s], L, swap,
                                            &dummy_inv, &dummy_errs, &dummy_C, &dummy_A);
    if (c16 > corr_p16) corr_p16 = c16;
  }
  return (corr_m8 <= best_corr + 0.01f && corr_p1 <= best_corr &&
          corr_p8 <= best_corr + 0.01f && corr_p16 <= best_corr + 0.01f) ? 1 : 0;
}

static float tetra_find_best_analog_seq(const dsd_opts *opts, const dsd_state *state, uint32_t cur_sub,
                                        const uint8_t * const *seqs, int n_seqs, int L,
                                        int *delta_out, int *swap_out, int *inv_out,
                                        int *s_out, int *errs_out, float *C_out, float *A_out)
{
  float best_corr = 0.0f;
  *delta_out = 0;
  *swap_out = 0;
  *inv_out = 0;
  *s_out = 0;
  *errs_out = L * 2;
  *C_out = 0.0f;
  *A_out = 1.0f;

  for (int delta = -4; delta <= 3; delta++)
  {
    uint32_t sub_idx = (uint32_t)((int32_t)cur_sub + delta);
    for (int swap = 0; swap < 2; swap++)
    {
      if (state->tetra_tdma_locked && swap != state->tetra_bit_swap)
        continue;
      for (int s = 0; s < n_seqs; s++)
      {
        int inv = 0, errs = L * 2;
        float C = 0.0f, A = 1.0f;
        float corr = tetra_eval_train_seq_analog(state, sub_idx, seqs[s], L, swap,
                                                 &inv, &errs, &C, &A);
        if (state->tetra_tdma_locked && inv != opts->inverted_tetra)
          continue;
        if (corr > best_corr + 1e-5f ||
            (corr >= best_corr - 1e-5f && *inv_out != opts->inverted_tetra && inv == opts->inverted_tetra))
        {
          best_corr = corr;
          *delta_out = delta;
          *swap_out = swap;
          *inv_out = inv;
          *s_out = s;
          *errs_out = errs;
          *C_out = C;
          *A_out = A;
        }
      }
    }
  }
  return best_corr;
}

int tetra_check_sync(dsd_opts *opts, dsd_state *state)
{
  if (opts->frame_tetra == 0)
    return -1;

  int is_bin = (opts->audio_in_type == 4) ? 1 : 0;
  const uint8_t *sync_seqs[1] = {TETRA_Y_DIBITS};
  const uint8_t *norm_seqs[3] = {TETRA_N_DIBITS, TETRA_P_DIBITS, TETRA_Q_DIBITS};

  if (is_bin)
  {
    if (state->tetra_sym_count < 19)
      return -1;
    uint32_t cur_sym = state->tetra_sym_count - 1;
    if (state->tetra_tdma_locked && (cur_sym - state->tetra_last_sync_sym) > 36000)
      state->tetra_tdma_locked = 0;

    /* 1. Check SYNC (Y_BITS, 19 symbols) */
    for (int swap = 0; swap < 2; swap++)
    {
      if (state->tetra_tdma_locked && swap != state->tetra_bit_swap)
        continue;
      for (int inv = 0; inv < 2; inv++)
      {
        if (state->tetra_tdma_locked && inv != opts->inverted_tetra)
          continue;
        int errs = tetra_eval_train_seq_bin(state, cur_sym, TETRA_Y_DIBITS, 19, inv, swap, 6);
        if (errs <= 6)
        {
          int sb1_ok = (state->tetra_sym_count >= 126)
                       ? tetra_try_presync_sb1(state, 1, cur_sym, 0.0f, 1.0f, inv, swap) : 0;
          if (sb1_ok || errs <= 2)
            return tetra_commit_bin_sync(opts, state, cur_sym, 0, swap, inv);
        }
      }
    }

    /* 2. Check NORM (N_BITS, P_BITS, Q_BITS, 11 symbols) */
    int on_grid = 0;
    if (state->tetra_tdma_locked && cur_sym >= 132)
    {
      int mod255 = (int)((int32_t)(cur_sym - 132 - state->tetra_last_sync_sym) % 255);
      if (mod255 < 0) mod255 += 255;
      if (mod255 <= 2 || mod255 >= 253)
        on_grid = 1;
    }

    for (int swap = 0; swap < 2; swap++)
    {
      if (state->tetra_tdma_locked && swap != state->tetra_bit_swap)
        continue;
      for (int inv = 0; inv < 2; inv++)
      {
        if (state->tetra_tdma_locked && inv != opts->inverted_tetra)
          continue;
        for (int s = 0; s < 3; s++)
        {
          int errs = tetra_eval_train_seq_bin(state, cur_sym, norm_seqs[s], 11, inv, swap, 4);
          if (errs <= 4)
          {
            int blk1_ok = (state->tetra_sym_count >= 133)
                          ? tetra_try_presync_blk1(state, 1, cur_sym, 0.0f, 1.0f, inv, swap) : 0;
            if (blk1_ok || (on_grid && errs <= 4) ||
                (!state->tetra_tdma_locked && errs == 0))
            {
              return tetra_commit_bin_sync(opts, state, cur_sym, (uint8_t)(s + 1), swap, inv);
            }
          }
        }
      }
    }
    return -1;
  }

  /* Analog audio path using 144 kHz (8 sub-samples/symbol) ring buffer */
  uint32_t cur_sub = state->tetra_sub_read_idx;
  if (cur_sub < 135 * 8)
    return -1;

  if (state->tetra_tdma_locked && (cur_sub - state->tetra_last_sync_sub) > 36720 * 8)
  {
    state->tetra_tdma_locked = 0;
    noCarrier(opts, state);
  }

  /* 1. Search all 8 sub-sample phases (delta = -4..+3) for SYNC burst (Y_BITS, 19 symbols) */
  int best_sync_delta = 0, best_sync_swap = 0, best_sync_inv = 0, best_sync_s = 0, best_sync_errs = 38;
  float best_sync_C = 0.0f, best_sync_A = 1.0f;
  float best_sync_corr = tetra_find_best_analog_seq(opts, state, cur_sub, sync_seqs, 1, 19,
                                                    &best_sync_delta, &best_sync_swap, &best_sync_inv,
                                                    &best_sync_s, &best_sync_errs, &best_sync_C, &best_sync_A);

  if (best_sync_corr >= 0.65f && best_sync_errs <= 9)
  {
    uint32_t best_sub = (uint32_t)((int32_t)cur_sub + best_sync_delta);
    if (tetra_is_analog_corr_peak(state, best_sub, best_sync_delta, best_sync_swap,
                                  best_sync_corr, sync_seqs, 1, 0, 19))
    {
      int sb1_ok = tetra_try_presync_sb1(state, 0, best_sub, best_sync_C, best_sync_A,
                                         best_sync_inv, best_sync_swap);
      int on_grid = tetra_is_on_analog_grid(state, best_sub, 125);

      if (sb1_ok || (on_grid && best_sync_corr >= 0.80f && best_sync_errs <= 5))
      {
        return tetra_commit_analog_sync(opts, state, cur_sub, best_sub, 0,
                                        best_sync_swap, best_sync_inv, best_sync_C, best_sync_A);
      }
    }
  }

  /* 2. Search all 8 sub-sample phases (delta = -4..+3) for NORM burst (N_BITS, P_BITS, Q_BITS, 11 symbols) */
  int best_norm_delta = 0, best_norm_swap = 0, best_norm_inv = 0, best_norm_s = 0, best_norm_errs = 22;
  float best_norm_C = 0.0f, best_norm_A = 1.0f;
  float best_norm_corr = tetra_find_best_analog_seq(opts, state, cur_sub, norm_seqs, 3, 11,
                                                    &best_norm_delta, &best_norm_swap, &best_norm_inv,
                                                    &best_norm_s, &best_norm_errs, &best_norm_C, &best_norm_A);

  if (best_norm_corr >= 0.68f && best_norm_errs <= 5)
  {
    uint32_t best_sub = (uint32_t)((int32_t)cur_sub + best_norm_delta);
    if (tetra_is_analog_corr_peak(state, best_sub, best_norm_delta, best_norm_swap,
                                  best_norm_corr, norm_seqs, 3, best_norm_s, 11))
    {
      int on_grid = tetra_is_on_analog_grid(state, best_sub, 132);

      int blk1_ok = (best_norm_corr >= 0.70f && best_norm_errs <= 4)
                    ? tetra_try_presync_blk1(state, 0, best_sub, best_norm_C, best_norm_A,
                                             best_norm_inv, best_norm_swap)
                    : 0;

      if (blk1_ok || (on_grid && best_norm_corr >= 0.68f && best_norm_errs <= 4) ||
          (!state->tetra_tdma_locked && best_norm_corr >= 0.94f && best_norm_errs == 0 && best_norm_A >= 400.0f))
      {
        return tetra_commit_analog_sync(opts, state, cur_sub, best_sub, (uint8_t)(best_norm_s + 1),
                                        best_norm_swap, best_norm_inv, best_norm_C, best_norm_A);
      }
    }
  }

  /* 3. When unlocked or in TMO Uplink mode, check for TMO Control Uplink Burst (X_BITS, 15 symbols) */
  if (!state->tetra_tdma_locked || state->tetra_mode == 2)
  {
    const uint8_t *cb_seqs[1] = {TETRA_X_DIBITS};
    int best_cb_delta = 0, best_cb_swap = 0, best_cb_inv = 0, best_cb_s = 0, best_cb_errs = 30;
    float best_cb_C = 0.0f, best_cb_A = 1.0f;
    float best_cb_corr = tetra_find_best_analog_seq(opts, state, cur_sub, cb_seqs, 1, 15,
                                                    &best_cb_delta, &best_cb_swap, &best_cb_inv,
                                                    &best_cb_s, &best_cb_errs, &best_cb_C, &best_cb_A);
    if (best_cb_corr >= 0.94f && best_cb_errs == 0 && best_cb_A >= 400.0f)
    {
      uint32_t best_sub = (uint32_t)((int32_t)cur_sub + best_cb_delta);
      if (tetra_is_analog_corr_peak(state, best_sub, best_cb_delta, best_cb_swap,
                                    best_cb_corr, cb_seqs, 1, 0, 15))
      {
        return tetra_commit_analog_sync(opts, state, cur_sub, best_sub, 4,
                                        best_cb_swap, best_cb_inv, best_cb_C, best_cb_A);
      }
    }
  }

  return -1;
}

/*-------------------------------------------------------------------------------
 * TETRA TCH/S ACELP Speech Decoding (ETSI EN 300 395-2 cdecoder + sdecoder)
 *-----------------------------------------------------------------------------*/

#if defined(_WIN32) || defined(__CYGWIN__)
typedef void *TETRA_WIN_HANDLE;
typedef uint32_t TETRA_WIN_DWORD;
typedef int TETRA_WIN_BOOL;

typedef struct {
  TETRA_WIN_DWORD nLength;
  void *lpSecurityDescriptor;
  TETRA_WIN_BOOL bInheritHandle;
} TETRA_WIN_SECURITY_ATTRIBUTES;

typedef struct {
  TETRA_WIN_DWORD cb;
  char *lpReserved;
  char *lpDesktop;
  char *lpTitle;
  TETRA_WIN_DWORD dwX;
  TETRA_WIN_DWORD dwY;
  TETRA_WIN_DWORD dwXSize;
  TETRA_WIN_DWORD dwYSize;
  TETRA_WIN_DWORD dwXCountChars;
  TETRA_WIN_DWORD dwYCountChars;
  TETRA_WIN_DWORD dwFillAttribute;
  TETRA_WIN_DWORD dwFlags;
  uint16_t wShowWindow;
  uint16_t cbReserved2;
  uint8_t *lpReserved2;
  TETRA_WIN_HANDLE hStdInput;
  TETRA_WIN_HANDLE hStdOutput;
  TETRA_WIN_HANDLE hStdError;
} TETRA_WIN_STARTUPINFOA;

typedef struct {
  TETRA_WIN_HANDLE hProcess;
  TETRA_WIN_HANDLE hThread;
  TETRA_WIN_DWORD dwProcessId;
  TETRA_WIN_DWORD dwThreadId;
} TETRA_WIN_PROCESS_INFORMATION;

__attribute__((dllimport)) TETRA_WIN_HANDLE __stdcall CreateFileA(
    const char *lpFileName, TETRA_WIN_DWORD dwDesiredAccess, TETRA_WIN_DWORD dwShareMode,
    TETRA_WIN_SECURITY_ATTRIBUTES *lpSecurityAttributes, TETRA_WIN_DWORD dwCreationDisposition,
    TETRA_WIN_DWORD dwFlagsAndAttributes, TETRA_WIN_HANDLE hTemplateFile);
__attribute__((dllimport)) TETRA_WIN_BOOL __stdcall CreateProcessA(
    const char *lpApplicationName, char *lpCommandLine,
    TETRA_WIN_SECURITY_ATTRIBUTES *lpProcessAttributes,
    TETRA_WIN_SECURITY_ATTRIBUTES *lpThreadAttributes,
    TETRA_WIN_BOOL bInheritHandles, TETRA_WIN_DWORD dwCreationFlags,
    void *lpEnvironment, const char *lpCurrentDirectory,
    TETRA_WIN_STARTUPINFOA *lpStartupInfo,
    TETRA_WIN_PROCESS_INFORMATION *lpProcessInformation);
__attribute__((dllimport)) TETRA_WIN_DWORD __stdcall WaitForSingleObject(
    TETRA_WIN_HANDLE hHandle, TETRA_WIN_DWORD dwMilliseconds);
__attribute__((dllimport)) TETRA_WIN_BOOL __stdcall GetExitCodeProcess(
    TETRA_WIN_HANDLE hProcess, TETRA_WIN_DWORD *lpExitCode);
__attribute__((dllimport)) TETRA_WIN_BOOL __stdcall CloseHandle(TETRA_WIN_HANDLE hObject);
__attribute__((dllimport)) TETRA_WIN_DWORD __stdcall GetModuleFileNameA(
    void *hModule, char *lpFilename, TETRA_WIN_DWORD nSize);
#endif

static int g_tetra_codec_checked = 0;
static int g_tetra_codec_found = 0;
static char g_tetra_cdecoder_path[512];
static char g_tetra_sdecoder_path[512];

static int tetra_file_exists(const char *path)
{
  FILE *f = fopen(path, "rb");
  if (f != NULL)
  {
    fclose(f);
    return 1;
  }
  return 0;
}

static void tetra_find_codecs(void)
{
  if (g_tetra_codec_checked)
    return;
  g_tetra_codec_checked = 1;

  char exe_dir[384];
  memset(exe_dir, 0, sizeof(exe_dir));

#if defined(_WIN32) || defined(__CYGWIN__)
  if (GetModuleFileNameA(NULL, exe_dir, (TETRA_WIN_DWORD)(sizeof(exe_dir) - 1)) > 0)
  {
    for (int i = 0; exe_dir[i] != '\0'; i++)
    {
      if (exe_dir[i] == '\\')
        exe_dir[i] = '/';
    }
    char *last_slash = strrchr(exe_dir, '/');
    if (last_slash != NULL)
      *last_slash = '\0';
    else
      exe_dir[0] = '\0';
  }
#endif

  const char *dirs[2] = {exe_dir, "."};
  const char *c_names[2] = {"cdecoder.exe", "cdecoder"};
  const char *s_names[2] = {"sdecoder.exe", "sdecoder"};
  char cand_c[512], cand_s[512];

  for (int d = 0; d < 2; d++)
  {
    if (dirs[d][0] == '\0')
      continue;
    for (int k = 0; k < 2; k++)
    {
      snprintf(cand_c, sizeof(cand_c), "%s/%s", dirs[d], c_names[k]);
      snprintf(cand_s, sizeof(cand_s), "%s/%s", dirs[d], s_names[k]);
      if (tetra_file_exists(cand_c) && tetra_file_exists(cand_s))
      {
        snprintf(g_tetra_cdecoder_path, sizeof(g_tetra_cdecoder_path), "%s", cand_c);
        snprintf(g_tetra_sdecoder_path, sizeof(g_tetra_sdecoder_path), "%s", cand_s);
        g_tetra_codec_found = 1;
        return;
      }
    }
  }
}

int tetra_voice_codec_available(void)
{
  tetra_find_codecs();
  return g_tetra_codec_found;
}

static int tetra_run_codec_cmd(const char *exe_path, const char *in_file, const char *out_file)
{
#if defined(_WIN32) || defined(__CYGWIN__)
  char cmdline[1200];
  snprintf(cmdline, sizeof(cmdline), "\"%s\" \"%s\" \"%s\"", exe_path, in_file, out_file);

  TETRA_WIN_SECURITY_ATTRIBUTES sa;
  memset(&sa, 0, sizeof(sa));
  sa.nLength = sizeof(sa);
  sa.bInheritHandle = 1;

  /* Open NUL device to silence stdout/stderr of cdecoder.exe and sdecoder.exe */
  TETRA_WIN_HANDLE hNul = CreateFileA("NUL", 0x40000000U /* GENERIC_WRITE */,
                                      3U /* FILE_SHARE_READ | FILE_SHARE_WRITE */,
                                      &sa, 3U /* OPEN_EXISTING */, 0, NULL);

  TETRA_WIN_STARTUPINFOA si;
  TETRA_WIN_PROCESS_INFORMATION pi;
  memset(&si, 0, sizeof(si));
  memset(&pi, 0, sizeof(pi));
  si.cb = sizeof(si);
  if (hNul != NULL && hNul != (TETRA_WIN_HANDLE)(intptr_t)(-1))
  {
    si.dwFlags = 0x00000100U; /* STARTF_USESTDHANDLES */
    si.hStdOutput = hNul;
    si.hStdError = hNul;
  }

  TETRA_WIN_BOOL ok = CreateProcessA(NULL, cmdline, NULL, NULL, 1,
                                     0x08000000U /* CREATE_NO_WINDOW */,
                                     NULL, NULL, &si, &pi);
  if (hNul != NULL && hNul != (TETRA_WIN_HANDLE)(intptr_t)(-1))
    CloseHandle(hNul);

  if (!ok)
    return -1;

  WaitForSingleObject(pi.hProcess, 3000U);
  TETRA_WIN_DWORD exit_code = 1;
  GetExitCodeProcess(pi.hProcess, &exit_code);
  CloseHandle(pi.hThread);
  CloseHandle(pi.hProcess);
  return (int)exit_code;
#else
  char cmdline[1400];
  snprintf(cmdline, sizeof(cmdline), "\"%s\" \"%s\" \"%s\" >/dev/null 2>&1", exe_path, in_file, out_file);
  return system(cmdline);
#endif
}

static void tetra_build_cdecoder_block(const int16_t *type4_soft, int16_t *block690)
{
  memset(block690, 0, 690 * sizeof(int16_t));
  for (int i = 0; i < 6; i++)
    block690[115 * i] = (int16_t)(0x6B21 + i);

  for (int i = 0; i < 114; i++)
  {
    block690[1 + i]   = tetra_clamp_soft(type4_soft[i]);
    block690[116 + i] = tetra_clamp_soft(type4_soft[114 + i]);
    block690[231 + i] = tetra_clamp_soft(type4_soft[228 + i]);
  }
  for (int i = 0; i < 90; i++)
  {
    block690[346 + i] = tetra_clamp_soft(type4_soft[342 + i]);
  }
}

static inline int tetra_is_enc_unmuted(const dsd_opts *opts)
{
  return (opts->unmute_encrypted_p25 == 1 || opts->dmr_mute_encL == 0 || opts->reverse_mute == 1) ? 1 : 0;
}

static inline int tetra_cc_filter_mismatch(const dsd_opts *opts, uint8_t cc)
{
  return (opts->tetra_cc_filter >= 0 && opts->tetra_cc_filter <= 63 &&
          (int)cc != opts->tetra_cc_filter) ? 1 : 0;
}

static void tetra_write_audio_out(dsd_opts *opts, dsd_state *state, void *buf, size_t nbytes, int allow_type2)
{
  if (opts->audio_out_type == 0)
    pa_simple_write(opts->pulse_digi_dev_out, buf, nbytes, NULL);
  else if (opts->audio_out_type == 8)
    udp_socket_blaster(opts, state, nbytes, buf);
  else if (opts->audio_out_type == 1 || (allow_type2 && opts->audio_out_type == 2))
    write(opts->audio_out_fd, buf, nbytes);
}

static void tetra_play_voice_pcm(dsd_opts *opts, dsd_state *state, int16_t *pcm, int nsam)
{
  if (nsam <= 0)
    return;

  if (opts->use_hpf_d == 1)
    hpf_dL(state, pcm, nsam);

  /* Apply manual gain if set, or maintain auto-gain display metric */
  float gain = 1.0f;
  if (opts->audio_gain > 0)
  {
    gain = opts->audio_gain / 25.0f;
    state->aout_gain = opts->audio_gain;
  }
  else if (state->aout_gain <= 0.0f)
  {
    state->aout_gain = 25.0f;
  }

  if (gain != 1.0f)
  {
    for (int i = 0; i < nsam; i++)
    {
      int scaled = (int)((float)pcm[i] * gain);
      pcm[i] = tetra_clamp_i16(scaled);
    }
  }

  if (opts->slot1_on == 1 && opts->audio_out == 1)
  {
    if (opts->floating_point == 1)
    {
      if (opts->pulse_digi_out_channels == 2)
      {
        float f_stereo[nsam * 2];
        for (int i = 0; i < nsam; i++)
        {
          float v = (float)pcm[i] / 32768.0f;
          f_stereo[i * 2 + 0] = v;
          f_stereo[i * 2 + 1] = v;
        }
        tetra_write_audio_out(opts, state, f_stereo, sizeof(f_stereo), 0);
      }
      else
      {
        float f_mono[nsam];
        for (int i = 0; i < nsam; i++)
          f_mono[i] = (float)pcm[i] / 32768.0f;
        tetra_write_audio_out(opts, state, f_mono, sizeof(f_mono), 0);
      }
    }
    else
    {
      if (opts->pulse_digi_out_channels == 2)
      {
        int16_t s_stereo[nsam * 2];
        for (int i = 0; i < nsam; i++)
        {
          s_stereo[i * 2 + 0] = pcm[i];
          s_stereo[i * 2 + 1] = pcm[i];
        }
        tetra_write_audio_out(opts, state, s_stereo, sizeof(s_stereo), 1);
      }
      else
      {
        if (opts->audio_out_type == 5 && opts->pulse_digi_rate_out != 8000)
        {
          int16_t upsamp[nsam * 6];
          int16_t out6[6];
          int16_t prev = pcm[0];
          for (int i = 0; i < nsam; i++)
          {
            upsampleS(pcm[i], prev, out6);
            prev = pcm[i];
            for (int j = 0; j < 6; j++)
              upsamp[i * 6 + j] = out6[j];
          }
          write(opts->audio_out_fd, upsamp, sizeof(upsamp));
        }
        else if (opts->audio_out_type == 5)
        {
          write(opts->audio_out_fd, pcm, nsam * 2);
        }
        else
        {
          tetra_write_audio_out(opts, state, pcm, (size_t)(nsam * 2), 1);
        }
      }
    }
  }

  /* WAV file output (-w static wav file or -P per-call wav file) */
  if (opts->wav_out_f != NULL && opts->dmr_stereo_wav == 1)
  {
    sf_write_short(opts->wav_out_f, pcm, nsam);
    g_tetra_wav_samples_written = 1;
    if (state->tetra_voice_slot >= 1 && state->tetra_voice_slot <= 4)
      g_tetra_wav_slot = state->tetra_voice_slot - 1;
  }
  else if (opts->wav_out_f != NULL && opts->static_wav_file == 1)
  {
    int16_t ss[nsam * 2];
    for (int i = 0; i < nsam; i++)
    {
      ss[i * 2 + 0] = pcm[i];
      ss[i * 2 + 1] = pcm[i];
    }
    sf_write_short(opts->wav_out_f, ss, nsam * 2);
  }
}

void tetra_flush_voice_batch(dsd_opts *opts, dsd_state *state)
{
  int n_bursts = state->tetra_cod_batch_count;
  state->tetra_cod_batch_count = 0;
  if (n_bursts <= 0 || n_bursts > 6)
    return;

  if (!tetra_voice_codec_available())
    return;

  char cod_file[64], ser_file[64], syn_file[64];
  int pid = (int)getpid();
  snprintf(cod_file, sizeof(cod_file), ".tetra_tch_%d.cod", pid);
  snprintf(ser_file, sizeof(ser_file), ".tetra_tch_%d.ser", pid);
  snprintf(syn_file, sizeof(syn_file), ".tetra_tch_%d.syn", pid);

  FILE *fc = fopen(cod_file, "wb");
  if (fc == NULL)
    return;
  fwrite(state->tetra_cod_batch, sizeof(int16_t), (size_t)(n_bursts * 690), fc);
  fclose(fc);

  tetra_run_codec_cmd(g_tetra_cdecoder_path, cod_file, ser_file);

  int16_t serial_batch[6 * 276];
  FILE *fs = fopen(ser_file, "rb");
  if (fs == NULL)
  {
    remove(cod_file);
    return;
  }
  size_t nread = fread(serial_batch, sizeof(int16_t), (size_t)(n_bursts * 276), fs);
  fclose(fs);
  remove(cod_file);
  remove(ser_file);

  int actual_bursts = (int)(nread / 276);
  if (actual_bursts <= 0)
    return;

  int is_enc = tetra_is_cell_encrypted(state);
  uint8_t tmp_akey[16];
  uint8_t tmp_aalg = 0;
  int active_klen = is_enc ? tetra_get_active_key(opts, state, tmp_akey, &tmp_aalg) : 0;
  int unmute_raw_enc = is_enc && tetra_is_enc_unmuted(opts);
  int crc_ok_count = 0;
  for (int b = 0; b < actual_bursts; b++)
  {
    const int16_t *ser276 = &serial_batch[b * 276];
    if (ser276[0] == 0)
      crc_ok_count++;
    if (ser276[138] == 0)
      crc_ok_count++;
  }

  int key_verified_now = 0;
  if (active_klen > 0)
  {
    int already_verified = (state->tetra_matched_key_len == (uint8_t)active_klen &&
                            memcmp(state->tetra_matched_key, tmp_akey, (size_t)active_klen) == 0);
    int min_crc = (actual_bursts >= 4) ? 4 : (actual_bursts >= 2) ? 2 : 1;
    if (crc_ok_count >= min_crc || (already_verified && crc_ok_count >= 1))
    {
      tetra_store_matched_key(state, tmp_akey, active_klen, tmp_aalg);
      key_verified_now = 1;
    }
  }

  if (is_enc && !unmute_raw_enc && !key_verified_now)
  {
    state->errs = 1;
    state->errs2 = 1;
    state->tetra_voice_slot = 0;
    state->tetra_voice_usage = 0;
    return;
  }

  int valid_frames = 0;
  for (int b = 0; b < actual_bursts; b++)
  {
    const int16_t *ser276 = &serial_batch[b * 276];
    int bfi1 = ser276[0];
    int bfi2 = ser276[138];
    state->errs = (unsigned int)bfi1;
    state->errs2 = (unsigned int)bfi2;

    if (bfi1 == 0 || bfi2 == 0 || unmute_raw_enc)
    {
      if (state->tetra_serial_hist_frames >= 20)
      {
        memmove(&state->tetra_serial_hist[0],
                &state->tetra_serial_hist[2 * 138],
                18 * 138 * sizeof(int16_t));
        state->tetra_serial_hist_frames = 18;
      }
      int16_t *dst = &state->tetra_serial_hist[state->tetra_serial_hist_frames * 138];
      memcpy(dst, ser276, 2 * 138 * sizeof(int16_t));
      if (unmute_raw_enc && bfi1 != 0 && bfi2 != 0)
      {
        /* Clear BFI flags only when unmuting undecrypted ACELP frames */
        dst[0] = 0;
        dst[138] = 0;
      }
      state->tetra_serial_hist_frames += 2;
      valid_frames += 2;
    }
  }

  if (valid_frames <= 0)
  {
    state->tetra_voice_slot = 0;
    state->tetra_voice_usage = 0;
    return;
  }

  int total_frames = state->tetra_serial_hist_frames;
  FILE *fser_hist = fopen(ser_file, "wb");
  if (fser_hist == NULL)
    return;
  fwrite(state->tetra_serial_hist, sizeof(int16_t), (size_t)(total_frames * 138), fser_hist);
  fclose(fser_hist);

  tetra_run_codec_cmd(g_tetra_sdecoder_path, ser_file, syn_file);
  remove(ser_file);

  FILE *fsyn = fopen(syn_file, "rb");
  if (fsyn == NULL)
    return;

  if (total_frames > valid_frames)
    fseek(fsyn, (long)((total_frames - valid_frames) * 240 * (int)sizeof(int16_t)), SEEK_SET);

  int want_samples = valid_frames * 240;
  int16_t pcm_buf[12 * 240];
  size_t npcm = fread(pcm_buf, sizeof(int16_t), (size_t)want_samples, fsyn);
  fclose(fsyn);
  remove(syn_file);

  if (npcm > 0)
    tetra_play_voice_pcm(opts, state, pcm_buf, (int)npcm);
}

static int tetra_queue_tch_voice(dsd_opts *opts, dsd_state *state,
                                 const int16_t *raw432_soft, uint32_t scramb_init)
{
  if (!tetra_voice_codec_available())
    return 0;

  time_t now = time(NULL);
  if (g_tetra_wav_samples_written && g_tetra_wav_slot >= 0 &&
      state->tetra_slot >= 1 && state->tetra_slot <= 4 &&
      (state->tetra_slot - 1) != g_tetra_wav_slot)
  {
    if (state->tetra_cod_batch_count > 0)
      tetra_flush_voice_batch(opts, state);
    tetra_close_per_call_wav(opts, state);
    state->tetra_serial_hist_frames = 0;
  }
  else if (state->tetra_last_voice_time == 0 || (now - state->tetra_last_voice_time) > 2)
  {
    if (state->tetra_cod_batch_count > 0)
      tetra_flush_voice_batch(opts, state);
    state->tetra_serial_hist_frames = 0;
  }
  state->tetra_last_voice_time = now;

  int16_t type4_soft[432];
  tetra_scramb_soft(scramb_init, raw432_soft, type4_soft, 432);

  /* If encrypted and a user or matched common key is loaded, decrypt the 432 Type-4 bits (TetraEar decoder.py lines 1040-1057) */
  int is_enc = tetra_is_cell_encrypted(state);
  uint8_t active_key[16];
  uint8_t u_alg = 0;
  int active_key_len = is_enc ? tetra_get_active_key(opts, state, active_key, &u_alg) : 0;
  if (active_key_len > 0)
  {
    uint8_t t4_bytes[56] = {0};
    uint8_t dec_bytes[56] = {0};
    for (int b = 0; b < 432; b++)
    {
      if (type4_soft[b] < 0)
        t4_bytes[b / 8] |= (uint8_t)(1U << (7 - (b % 8)));
    }
    tetra_tea_decrypt_bytes(u_alg, active_key, t4_bytes, dec_bytes, 54);
    for (int b = 0; b < 432; b++)
    {
      uint8_t bit = (dec_bytes[b / 8] >> (7 - (b % 8))) & 1U;
      type4_soft[b] = bit ? -127 : 127;
    }
  }

  tetra_build_cdecoder_block(type4_soft, &state->tetra_cod_batch[state->tetra_cod_batch_count * 690]);
  state->tetra_cod_batch_count++;

  if (state->tetra_cod_batch_count >= 6)
    tetra_flush_voice_batch(opts, state);

  return 1;
}

static void tetra_update_tdma_counters(dsd_state *state, int is_bin, uint32_t cur_sync_idx)
{
  if (!state->tetra_tdma_locked || state->tetra_bsch_tn == 0 || state->tetra_bsch_fn == 0)
    return;

  int slots_elapsed = 0;
  if (is_bin)
  {
    uint32_t bstart = cur_sync_idx - 132;
    slots_elapsed = (int)((int32_t)(bstart - state->tetra_last_sync_sym) + 127) / 255;
  }
  else
  {
    uint32_t bstart = cur_sync_idx - 132 * 8;
    slots_elapsed = (int)((int32_t)(bstart - state->tetra_last_sync_sub) + 1020) / 2040;
  }
  if (slots_elapsed <= 0)
    return;

  int base_slots = (((int)state->tetra_bsch_mn - 1) * 18 + ((int)state->tetra_bsch_fn - 1)) * 4 +
                   ((int)state->tetra_bsch_tn - 1);
  int cur_slots = base_slots + slots_elapsed;
  if (cur_slots < 0)
    cur_slots = 0;

  state->tetra_slot = (uint8_t)((cur_slots % 4) + 1);
  state->tetra_frame = (uint8_t)(((cur_slots / 4) % 18) + 1);
  state->tetra_multiframe = (uint8_t)(((cur_slots / 72) % 60) + 1);
}

static void tetra_anchor_tdma_sync(dsd_state *state, int is_bin, uint32_t sync_idx)
{
  state->carrier = 1;
  state->last_cc_sync_time = time(NULL);
  if (state->tetra_tdma_locked && state->tetra_bsch_tn != 0 && state->tetra_bsch_fn != 0)
  {
    if (is_bin)
      state->tetra_last_sync_sym = sync_idx - 132;
    else
      state->tetra_last_sync_sub = sync_idx - 132 * 8;
    state->tetra_bsch_tn = state->tetra_slot;
    state->tetra_bsch_fn = state->tetra_frame;
    state->tetra_bsch_mn = state->tetra_multiframe;
  }
}

static void tetra_update_cch_slot_state(dsd_opts *opts, dsd_state *state)
{
  /* Control channel block (SCH/HD or SCH/F) is never a traffic burst; clamp any AACH bit errors */
  if (state->tetra_dl_usage > 3)
    state->tetra_dl_usage = 0;

  if (state->tetra_slot >= 1 && state->tetra_slot <= 4 &&
      state->tetra_frame >= 1 && state->tetra_frame <= 17)
  {
    int s = state->tetra_slot - 1;
    if (s == 0 && state->tetra_dl_usage == 2)
    {
      if (state->tetra_ts_cand_usage[0] == 2)
      {
        if (state->tetra_ts_cand_count[0] < 250)
          state->tetra_ts_cand_count[0]++;
      }
      else
      {
        state->tetra_ts_cand_usage[0] = 2;
        state->tetra_ts_cand_count[0] = 1;
      }
      state->tetra_ts_cand_sym[0] = state->tetra_sym_count;
      if (state->tetra_ts_usage[0] == 2 || state->tetra_ts_cand_count[0] >= 3)
      {
        state->tetra_ts_usage[0] = 2; /* Control (MCCH) is strictly on TN 1 */
        state->tetra_ts_last_sym[0] = state->tetra_sym_count;
        state->tetra_ts_idle_count[0] = 0;
      }
    }
    else
    {
      if (s == 0 && state->tetra_ts_cand_usage[0] == 2)
        state->tetra_ts_cand_count[0] = 0;
      else if (s > 0)
        state->tetra_ts_cand_count[s] = 0;
      if (state->tetra_dl_usage == 0 && state->tetra_ts_usage[s] != 0)
      {
        state->tetra_ts_idle_count[s]++;
        if (s > 0 && state->tetra_ts_idle_count[s] >= 3 && state->tetra_voice_slot == (uint8_t)(s + 1))
          tetra_unlock_voice_slot(opts, state, 1);
        uint8_t idle_thresh = (state->tetra_ts_usage[s] == 2) ? 15 :
                              ((state->tetra_ts_call_id[s] != 0) ? 90 : 10);
        if (state->tetra_ts_idle_count[s] >= idle_thresh)
        {
          tetra_reset_slot_state(opts, state, s);
        }
      }
    }
  }
}

static void tetra_log_voice_slot_event(dsd_opts *opts, dsd_state *state, int s, const char *tea)
{
  UNUSED(tea);
  if (state->tetra_ts_cand_count[s] >= 3)
  {
    uint32_t eff_tg = state->tetra_ts_tg[s];
    uint32_t eff_src = state->tetra_ts_src[s];
    uint8_t prev_logged = state->tetra_ts_logged[s];
    uint8_t new_logged = ((eff_tg != 0) ? 2U : 0U) |
                         ((eff_src != 0) ? 1U : 0U);
    if (new_logged > 0 && (prev_logged == 0 || new_logged > prev_logged))
    {
      int should_log = (prev_logged > 0) ? 1 :
                       ((eff_tg != 0) ? tetra_should_log_ssi(state, eff_tg) :
                                        tetra_should_log_ssi(state, eff_src));
      if (should_log)
      {
        char ev_desc[128];
        snprintf(ev_desc, sizeof(ev_desc), "TN: %u; %s", state->tetra_slot, tetra_get_slot_call_mode_tag(state, s));
        tetra_log_data_event_ex(opts, state, eff_src, eff_tg, ev_desc, (prev_logged > 0));
      }
      state->tetra_ts_logged[s] = new_logged;
    }
  }
}

static void tetra_extract_burst_at_offset(const dsd_state *state, uint32_t sync_idx, int sync_end_sym,
                                          int delta_sub, float center, float step, int inv, int swap,
                                          uint8_t *burst_bits, int16_t *burst_soft)
{
  for (int sym = 0; sym <= 248; sym++)
  {
    uint32_t idx = (uint32_t)((int32_t)sync_idx + (sym - sync_end_sym) * 8 + delta_sub) & 8191U;
    int val = state->tetra_sub_buf[idx];
    tetra_symbol_to_bits(val, 0, center, step, inv, swap,
                         &burst_bits[sym * 2], &burst_bits[sym * 2 + 1],
                         &burst_soft[sym * 2], &burst_soft[sym * 2 + 1]);
  }
}

static void tetra_print_payload_hex(const char *label, const uint8_t *bits, int nbytes)
{
  fprintf(stderr, "\n %s: ", label);
  for (int i = 0; i < nbytes; i++)
    fprintf(stderr, "%02X", (unsigned int)tetra_bits_to_uint(bits, i * 8, 8));
}

/* Verify that an unscrambled/unknown-seed burst is a genuine strong TMO Uplink burst
 * and NOT a weak/noisy signal or a continuous 4-slot TMO Downlink carrier:
 * 1. Check 4-level DQPSK constellation EVM across all 216 payload symbols (rejects weak/noisy signals).
 * 2. Check the other 3 timeslots (-255, -510, -765 symbols in tetra_sub_buf): a TMO Downlink transmits
 *    training sequences on all 4 slots, whereas a Mobile Station Uplink transmits on only 1 slot out of 4. */
static int tetra_is_valid_tmo_uplink_burst(const dsd_state *state, int is_bin, uint32_t sync_idx,
                                           int inv, int swap, float step,
                                           const int16_t *burst_soft)
{
  if (!is_bin)
  {
    if (step < 400.0f)
      return 0;

    /* 1. Check payload symbol EVM across symbols 14..121 and 133..240 (216 symbols) */
    int32_t evm_sum = 0;
    for (int sym = 14; sym <= 240; sym++)
    {
      if (sym >= 122 && sym <= 132)
        continue;
      int16_t s_mag = swap ? burst_soft[sym * 2] : burst_soft[sym * 2 + 1];
      int abs_mag = (s_mag >= 0) ? (int)s_mag : -(int)s_mag;
      int err = abs_mag - 64;
      if (err < 0)
        err = -err;
      evm_sum += err;
    }
    /* Ideal 4-level symbols have |s_mag| == 64 (err == 0); weak/noisy signals have avg err > 22 */
    if (evm_sum > 216 * 16)
      return 0;

    /* 2. Check the other 3 timeslots (-255, -510, -765 symbols = -2040, -4080, -6120 sub-samples)
     * in the 8192-element lookback buffer. If ANY of the other 3 slots contains a Downlink training
     * sequence (even a weak/noisy one with corr >= 0.52 or errs <= 6), this is a TMO Downlink! */
    if (sync_idx >= 6200)
    {
      const uint8_t *norm_seqs[3] = {TETRA_N_DIBITS, TETRA_P_DIBITS, TETRA_Q_DIBITS};
      for (int slot_back = 1; slot_back <= 3; slot_back++)
      {
        uint32_t adj_sub = sync_idx - (uint32_t)(slot_back * 2040);
        for (int s = 0; s < 3; s++)
        {
          int d_inv = 0, d_errs = 22;
          float d_C = 0.0f, d_A = 1.0f;
          float corr = tetra_eval_train_seq_analog(state, adj_sub, norm_seqs[s], 11, swap,
                                                   &d_inv, &d_errs, &d_C, &d_A);
          if (corr >= 0.65f && d_errs <= 5 && d_inv == inv)
            return 0;
        }
        /* Also check for BSCH Y_BITS in adjacent slot (ends at symbol 125 = 7 symbols before 132) */
        int y_inv = 0, y_errs = 38;
        float y_C = 0.0f, y_A = 1.0f;
        float y_corr = tetra_eval_train_seq_analog(state, adj_sub - 7 * 8, TETRA_Y_DIBITS, 19, swap,
                                                   &y_inv, &y_errs, &y_C, &y_A);
        if (y_corr >= 0.60f && y_errs <= 8 && y_inv == inv)
          return 0;
      }
    }
  }
  return 1;
}

void processTetra(dsd_opts *opts, dsd_state *state)
{
  uint8_t burst_bits[510];
  int16_t burst_soft[510];
  memset(burst_bits, 0, sizeof(burst_bits));
  memset(burst_soft, 0, sizeof(burst_soft));

  int inv = (state->synctype == 42) ? 1 : 0;
  int swap = state->tetra_bit_swap;
  int sync_end_sym = (state->tetra_burst_type == 0) ? 125 : 132;
  int is_bin = (opts->audio_in_type == 4) ? 1 : 0;
  uint32_t sync_idx = state->tetra_sync_sub_idx;
  float center = state->tetra_center;
  float step = (state->tetra_step_level > 1.0f) ? state->tetra_step_level : 1.0f;

  /* 1. Extract pre-sync symbols 0..sync_end_sym from tetra_sub_buf (or tetra_sym_ring for .bin) */
  tetra_extract_lookback_soft(state, is_bin, sync_idx, sync_end_sym, 0, sync_end_sym + 1,
                              center, step, inv, swap, burst_bits, burst_soft);

  /* 2. Read remaining symbols of the payload (up to symbol 248 = bit 498) */
  for (int sym = sync_end_sym + 1; sym <= 248; sym++)
  {
    int analog_sym = 0;
    int dibit = get_dibit_and_analog_signal(opts, state, &analog_sym);
    int val = is_bin ? (dibit & 3) : analog_sym;
    tetra_symbol_to_bits(val, is_bin, center, step, inv, swap,
                         &burst_bits[sym * 2], &burst_bits[sym * 2 + 1],
                         &burst_soft[sym * 2], &burst_soft[sym * 2 + 1]);
  }

  char *timestr = getTimeC();
  int crc_passed = 0;

  if ((state->tetra_voice_slot != 0 || state->tetra_voice_usage != 0) &&
      (uint32_t)(state->tetra_sym_count - state->tetra_voice_last_sym) > 9180U)
  {
    tetra_unlock_voice_slot(opts, state, 1);
  }

  for (int s = 0; s < 4; s++)
  {
    if ((state->tetra_ts_usage[s] != 0 || state->tetra_ts_type[s][0] != '\0') &&
        (uint32_t)(state->tetra_sym_count - state->tetra_ts_last_sym[s]) > 18360U)
    {
      tetra_reset_slot_state(opts, state, s);
    }
  }

  if (state->tetra_scramb_valid && tetra_cc_filter_mismatch(opts, state->tetra_colour_code))
  {
    tetra_release_voice_slot(opts, state, 1);
    tetra_clear_cell_state(state);
  }

  /* 3. Process Synchronization Burst (TMO BSCH SDB or DMO DMAC-SYNC DSB) */
  if (state->tetra_burst_type == 0)
  {
    uint8_t sb1_bits[60];
    int sb1_ok = tetra_decode_block_cch_soft(&burst_soft[94], 120, 11, 80, 60,
                                             TETRA_SCRAMB_INIT_DEFAULT, sb1_bits);
    if (sb1_ok)
    {
      /* Check if this is a DMO Synchronization Burst (DSB):
       * DMO DSB has NO 30-bit BBK after Y_BITS, so SCH/H (216 bits -> 124 bits) starts
       * immediately at bit 252 and is scrambled with TETRA_SCRAMB_INIT_DEFAULT (3U). */
      uint8_t dmo_sch_h[124];
      int is_dmo_dsb = (tetra_bits_to_uint(sb1_bits, 4, 2) == 0) &&
                       tetra_decode_block_cch_soft(&burst_soft[252], 216, 101, 144, 124,
                                                   TETRA_SCRAMB_INIT_DEFAULT, dmo_sch_h);
      if (is_dmo_dsb)
      {
        crc_passed = 1;
        state->carrier = 1;
        state->last_cc_sync_time = time(NULL);
        if (state->tetra_mode != 1)
          tetra_clear_cell_state(state);
        state->tetra_mode = 1;
        state->tetra_tdma_locked = 1;
        if (is_bin)
          state->tetra_last_sync_sym = sync_idx - 125;
        else
          state->tetra_last_sync_sub = sync_idx - 125 * 8;

        uint8_t comm_t = (uint8_t)tetra_bits_to_uint(sb1_bits, 6, 2);
        uint8_t tn = (uint8_t)(tetra_bits_to_uint(sb1_bits, 10, 2) + 1);
        uint8_t fn = (uint8_t)tetra_bits_to_uint(sb1_bits, 12, 5);
        const char *comm_str = (comm_t == 1) ? "DM-REP" :
                               (comm_t == 2) ? "DM-GATE" :
                               (comm_t == 3) ? "DM-REP/GATE" : "MS-MS";

        char hdr[160];
        snprintf(hdr, sizeof(hdr), "%s Sync: %s DMO DSB (%s) TN:%u FN:%02u",
                 timestr, inv ? "-TETRA" : "+TETRA", comm_str, tn, fn);
        tetra_set_burst_hdr(hdr);
        tetra_parse_dmac_sync(opts, state, sb1_bits, dmo_sch_h);

        if (opts->payload == 1)
        {
          tetra_ensure_burst_hdr();
          tetra_print_payload_hex("TETRA DMO SCH/S HEX", sb1_bits, 7);
          tetra_print_payload_hex("TETRA DMO SCH/H HEX", dmo_sch_h, 15);
        }
        tetra_finish_burst_hdr(1);
      }
      else
      {
        crc_passed = 1;
        state->carrier = 1;
        state->last_cc_sync_time = time(NULL);
        state->tetra_mode = 0;

        uint8_t colour = (uint8_t)tetra_bits_to_uint(sb1_bits, 4, 6);
        uint8_t ts = (uint8_t)(tetra_bits_to_uint(sb1_bits, 10, 2) + 1);
        uint8_t fn = (uint8_t)tetra_bits_to_uint(sb1_bits, 12, 5);
        uint8_t mn = (uint8_t)tetra_bits_to_uint(sb1_bits, 17, 6);
        uint16_t mcc = (uint16_t)tetra_bits_to_uint(sb1_bits, 31, 10);
        uint16_t mnc = (uint16_t)tetra_bits_to_uint(sb1_bits, 41, 14);

        int was_scramb_valid = state->tetra_scramb_valid;
        int cell_changed = ((state->tetra_mcc != 0 && state->tetra_mcc != mcc) ||
                            (state->tetra_mnc != 0 && state->tetra_mnc != mnc) ||
                            (state->tetra_scramb_valid && state->tetra_colour_code != colour));

        if (cell_changed || tetra_cc_filter_mismatch(opts, colour))
        {
          tetra_release_voice_slot(opts, state, 1);
          tetra_clear_cell_state(state);
        }

        state->tetra_tdma_locked = 1;
        if (is_bin)
          state->tetra_last_sync_sym = sync_idx - 125;
        else
          state->tetra_last_sync_sub = sync_idx - 125 * 8;

        state->tetra_colour_code = colour;
        state->tetra_slot = ts;
        state->tetra_frame = fn;
        state->tetra_multiframe = mn;
        state->tetra_bsch_tn = ts;
        state->tetra_bsch_fn = fn;
        state->tetra_bsch_mn = mn;
        state->tetra_mcc = mcc;
        state->tetra_mnc = mnc;
        snprintf(state->tetra_country_str, sizeof(state->tetra_country_str),
                 "%s", tetra_lookup_country(mcc));
        state->tetra_scramb_init = tetra_scramb_get_init(mcc, mnc, colour);
        state->tetra_scramb_valid = 1;

        if (tetra_cc_filter_mismatch(opts, colour))
        {
          fprintf(stderr, "%s Sync: %s BSCH MCC:%u (%s) MNC:%u CC:%u [TCC FILTER: %02d]\n",
                  timestr, inv ? "-TETRA" : "+TETRA",
                  mcc, state->tetra_country_str, mnc, colour, opts->tetra_cc_filter);
        }
        else
        {
          if (ts == 1 && state->tetra_ts_usage[0] == 2)
            state->tetra_ts_last_sym[0] = state->tetra_sym_count;
          tetra_decode_aach(state, &burst_bits[252], state->tetra_scramb_init, 1);

          int force_bsch_log = (!was_scramb_valid || cell_changed ||
                                (fn == 18 && mn != g_tetra_last_bsch_log_mn) ||
                                opts->payload == 1);
          char hdr[160];
          snprintf(hdr, sizeof(hdr), "%s Sync: %s BSCH MCC:%u (%s) MNC:%u CC:%u TN:%u FN:%02u MN:%02u",
                   timestr, inv ? "-TETRA" : "+TETRA",
                   mcc, state->tetra_country_str, mnc, colour, ts, fn, mn);
          tetra_set_burst_hdr(hdr);

          /* Decode SB2 (bits 282..497, 216 bits -> 124 bits) */
          uint8_t sb2_bits[124];
          int sb2_ok = tetra_decode_block_cch_soft(&burst_soft[282], 216, 101, 144, 124,
                                                   state->tetra_scramb_init, sb2_bits);
          if (sb2_ok)
          {
            tetra_parse_mac_pdu(opts, state, sb2_bits, 124);
          }

          if (opts->payload == 1)
          {
            tetra_ensure_burst_hdr();
            tetra_print_payload_hex("TETRA SB1 HEX", sb1_bits, 7);
            if (sb2_ok)
              tetra_print_payload_hex("TETRA SB2 HEX", sb2_bits, 15);
          }
          if (force_bsch_log || g_tetra_burst_hdr_printed)
            g_tetra_last_bsch_log_mn = mn;
          tetra_finish_burst_hdr(force_bsch_log);
        }
      }
    }
  }
  else if (state->tetra_burst_type == 4)
  /* 3b. TMO Control Uplink Burst (CB, X_BITS) -> Recognize as TETRA TMO Uplink */
  {
    uint32_t cur_b_sym = state->tetra_sync_sym_count;
    uint32_t delta_sym = (g_tetra_ul_last_sym != 0 && cur_b_sym > g_tetra_ul_last_sym)
                         ? (cur_b_sym - g_tetra_ul_last_sym) : 0;
    g_tetra_ul_last_sym = cur_b_sym;
    if ((delta_sym > 0 && delta_sym < 980) ||
        !tetra_is_valid_tmo_uplink_burst(state, is_bin, sync_idx, inv, swap, step, burst_soft))
    {
      /* Bursts < 4 slots apart (< 1020 symbols) or failing EVM/adjacent-slot check = Downlink or weak signal */
      g_tetra_ul_cand_count = 0;
    }
    else
    {
      if (g_tetra_ul_cand_count < 250)
        g_tetra_ul_cand_count++;
      if (g_tetra_ul_cand_count >= 2)
      {
        crc_passed = 1;
        state->carrier = 1;
        state->last_cc_sync_time = time(NULL);
        state->tetra_mode = 2;
        const char *ul_desc = "CB (Random Access / SCH/HU)";
        int changed = (strcmp(state->tetra_call_type, ul_desc) != 0);
        snprintf(state->tetra_call_type, sizeof(state->tetra_call_type), "%s", ul_desc);
        time_t now_ul = time(NULL);
        if (changed || now_ul != g_tetra_last_console_log_time)
        {
          g_tetra_last_console_log_time = now_ul;
          fprintf(stderr, "%s Sync: %s TMO Uplink - %s\n", timestr, inv ? "-TETRA" : "+TETRA", ul_desc);
        }
      }
    }
  }
  else if (state->tetra_mode != 1 &&
           (opts->tetra_cc_filter < 0 || opts->tetra_cc_filter > 63 ||
            (state->tetra_scramb_valid && !tetra_cc_filter_mismatch(opts, state->tetra_colour_code))))
  /* 4. Process Normal Downlink Burst (NORM_1 / NORM_2 / NORM_3) */
  {
    uint32_t scramb = state->tetra_scramb_valid ? state->tetra_scramb_init : TETRA_SCRAMB_INIT_DEFAULT;
    tetra_update_tdma_counters(state, is_bin, sync_idx);

    uint8_t bbk_raw[30];
    memcpy(&bbk_raw[0], &burst_bits[230], 14);
    memcpy(&bbk_raw[14], &burst_bits[266], 16);

    if (state->tetra_burst_type == 2) /* NORM_2: Two 216-bit half-slot blocks (SCH/HD or BNCH) */
    {
      uint8_t blk1[124], blk2[124];
      int ok1 = tetra_decode_block_cch_soft(&burst_soft[14], 216, 101, 144, 124, scramb, blk1);
      int ok2 = tetra_decode_block_cch_soft(&burst_soft[282], 216, 101, 144, 124, scramb, blk2);
      if (!is_bin && !ok1 && !ok2)
      {
        static const int RETRY_D[4] = {-1, 1, -2, 2};
        uint8_t alt_bits[510];
        int16_t alt_soft[510];
        for (int r = 0; r < 4 && !ok1 && !ok2; r++)
        {
          tetra_extract_burst_at_offset(state, sync_idx, sync_end_sym, RETRY_D[r],
                                        center, step, inv, swap, alt_bits, alt_soft);
          if (tetra_decode_block_cch_soft(&alt_soft[14], 216, 101, 144, 124, scramb, blk1))
          {
            ok1 = 1;
            memcpy(&bbk_raw[0], &alt_bits[230], 14);
            memcpy(&bbk_raw[14], &alt_bits[266], 16);
          }
          if (tetra_decode_block_cch_soft(&alt_soft[282], 216, 101, 144, 124, scramb, blk2))
          {
            ok2 = 1;
            memcpy(&bbk_raw[0], &alt_bits[230], 14);
            memcpy(&bbk_raw[14], &alt_bits[266], 16);
          }
        }
      }

      if (ok1 || ok2)
      {
        crc_passed = 1;
        tetra_decode_aach(state, bbk_raw, scramb, 0);
        tetra_update_cch_slot_state(opts, state);
        tetra_anchor_tdma_sync(state, is_bin, sync_idx);

        char hdr[160];
        snprintf(hdr, sizeof(hdr), "%s Sync: %s NDB (SCH/HD)", timestr, inv ? "-TETRA" : "+TETRA");
        tetra_set_burst_hdr(hdr);
        if (ok1)
          tetra_parse_mac_pdu(opts, state, blk1, 124);
        if (ok2)
          tetra_parse_mac_pdu(opts, state, blk2, 124);
        if (opts->payload == 1)
        {
          tetra_ensure_burst_hdr();
          if (ok1)
            tetra_print_payload_hex("TETRA BLK1 HEX", blk1, 15);
          if (ok2)
            tetra_print_payload_hex("TETRA BLK2 HEX", blk2, 15);
        }
        tetra_finish_burst_hdr(opts->payload == 1);
      }
    }
    else if (state->tetra_burst_type == 3) /* NORM_3: Stolen channel STCH (half-slot 1, and optionally half-slot 2) */
    {
      uint8_t blk1[124], blk2[124];
      int ok1 = tetra_decode_block_cch_soft(&burst_soft[14], 216, 101, 144, 124, scramb, blk1);
      int ok2 = ok1 ? tetra_decode_block_cch_soft(&burst_soft[282], 216, 101, 144, 124, scramb, blk2) : 0;
      if (ok1)
      {
        tetra_decode_aach(state, bbk_raw, scramb, 0);
        int s = (state->tetra_slot >= 1 && state->tetra_slot <= 4) ? (state->tetra_slot - 1) : -1;
        /* Ignore false-CRC STCH on TN 1 when TN 1 is MCCH, or on idle slots with no active call */
        if (s >= 0 && !(s == 0 && state->tetra_ts_usage[0] == 2) &&
            (state->tetra_ts_usage[s] > 3 || state->tetra_ts_call_id[s] != 0 || state->tetra_dl_usage > 3))
        {
          crc_passed = 1;
          uint8_t u = (state->tetra_dl_usage > 3) ? state->tetra_dl_usage
                                                  : ((state->tetra_ts_usage[s] > 3) ? state->tetra_ts_usage[s] : 4);
          state->tetra_dl_usage = u;
          if (state->tetra_ts_usage[s] > 3 || state->tetra_ts_call_id[s] != 0)
          {
            state->tetra_ts_usage[s] = u;
            state->tetra_ts_last_sym[s] = state->tetra_sym_count;
            state->tetra_ts_idle_count[s] = 0;
            state->tetra_ts_cand_usage[s] = u;
            state->tetra_ts_cand_count[s] = 3;
            state->tetra_ts_cand_sym[s] = state->tetra_sym_count;
            uint32_t mapped_ssi = state->tetra_usage_ssi[u & 63U];
            if (state->tetra_ts_tg[s] == 0 && mapped_ssi != 0)
              state->tetra_ts_tg[s] = mapped_ssi;
          }
          tetra_anchor_tdma_sync(state, is_bin, sync_idx);

          char hdr[160];
          snprintf(hdr, sizeof(hdr), "%s Sync: %s NDB (STCH)", timestr, inv ? "-TETRA" : "+TETRA");
          tetra_set_burst_hdr(hdr);
          tetra_parse_mac_pdu(opts, state, blk1, 124);
          if (ok2)
            tetra_parse_mac_pdu(opts, state, blk2, 124);
          tetra_finish_burst_hdr(opts->payload == 1);
        }
      }
    }
    else /* NORM_1: Full-slot SCH/F (432 bits -> 268 bits) or TCH/S Speech (432 bits -> 2 x 137 bits) */
    {
      int16_t raw432_soft[432];
      uint8_t schf_bits[268];
      memcpy(&raw432_soft[0], &burst_soft[14], 216 * sizeof(int16_t));
      memcpy(&raw432_soft[216], &burst_soft[282], 216 * sizeof(int16_t));

      int ok_f = tetra_decode_block_cch_soft(raw432_soft, 432, 103, 288, 268, scramb, schf_bits);
      if (!is_bin && !ok_f && (!state->tetra_tdma_locked || state->tetra_slot == 1))
      {
        static const int RETRY_D[4] = {-1, 1, -2, 2};
        uint8_t alt_bits[510];
        int16_t alt_soft[510];
        int16_t alt432[432];
        for (int r = 0; r < 4 && !ok_f; r++)
        {
          tetra_extract_burst_at_offset(state, sync_idx, sync_end_sym, RETRY_D[r],
                                        center, step, inv, swap, alt_bits, alt_soft);
          memcpy(&alt432[0], &alt_soft[14], 216 * sizeof(int16_t));
          memcpy(&alt432[216], &alt_soft[282], 216 * sizeof(int16_t));
          if (tetra_decode_block_cch_soft(alt432, 432, 103, 288, 268, scramb, schf_bits))
          {
            ok_f = 1;
            memcpy(&bbk_raw[0], &alt_bits[230], 14);
            memcpy(&bbk_raw[14], &alt_bits[266], 16);
          }
        }
      }

      if (ok_f)
      {
        crc_passed = 1;
        tetra_decode_aach(state, bbk_raw, scramb, 0);
        tetra_update_cch_slot_state(opts, state);
        tetra_anchor_tdma_sync(state, is_bin, sync_idx);

        char hdr[160];
        snprintf(hdr, sizeof(hdr), "%s Sync: %s NDB (SCH/F)", timestr, inv ? "-TETRA" : "+TETRA");
        tetra_set_burst_hdr(hdr);
        tetra_parse_mac_pdu(opts, state, schf_bits, 268);
        if (opts->payload == 1)
        {
          tetra_ensure_burst_hdr();
          tetra_print_payload_hex("TETRA SCH/F HEX", schf_bits, 33);
        }
        tetra_finish_burst_hdr(opts->payload == 1);
      }
      else if (state->tetra_tdma_locked && state->tetra_scramb_valid &&
               state->tetra_slot >= 1 && state->tetra_slot <= 4 &&
               state->tetra_frame >= 1 && state->tetra_frame <= 17)
      {
        tetra_decode_aach(state, bbk_raw, scramb, 0);
        int s = state->tetra_slot - 1;
        int is_traffic = 0;
        int is_enc = tetra_is_cell_encrypted(state);

        /* TN 1 on the main carrier (MCCH, usage == 2) never carries TCH/S voice */
        if (s == 0 && state->tetra_ts_usage[0] == 2)
        {
          state->tetra_dl_usage = 0;
        }

        if (state->tetra_dl_usage > 3)
        {
          if (state->tetra_ts_cand_usage[s] == state->tetra_dl_usage &&
              (uint32_t)(state->tetra_sym_count - state->tetra_ts_cand_sym[s]) < 27540U)
          {
            if (state->tetra_ts_cand_count[s] < 250)
              state->tetra_ts_cand_count[s]++;
          }
          else
          {
            state->tetra_ts_cand_usage[s] = state->tetra_dl_usage;
            state->tetra_ts_cand_count[s] = 1;
          }
          state->tetra_ts_cand_sym[s] = state->tetra_sym_count;

          uint8_t req_cnt = (state->tetra_ts_call_id[s] != 0 ||
                             (state->tetra_ts_mask & (1U << (3 - s)))) ? 2 : 3;
          if ((state->tetra_ts_usage[s] == state->tetra_dl_usage && state->tetra_ts_idle_count[s] == 0) ||
              state->tetra_ts_cand_count[s] >= req_cnt)
          {
            is_traffic = 1;
            crc_passed = 1;
            tetra_anchor_tdma_sync(state, is_bin, sync_idx);
            state->tetra_ts_usage[s] = state->tetra_dl_usage;
            state->tetra_ts_last_sym[s] = state->tetra_sym_count;
            state->tetra_ts_idle_count[s] = 0;
            uint32_t mapped_ssi = state->tetra_usage_ssi[state->tetra_dl_usage & 63U];
            if (state->tetra_ts_tg[s] == 0 && mapped_ssi != 0)
              state->tetra_ts_tg[s] = mapped_ssi;
            if (state->tetra_ts_type[s][0] == '\0')
            {
              if (is_enc)
                snprintf(state->tetra_ts_type[s], sizeof(state->tetra_ts_type[s]),
                         "Voice Call [%s]", tetra_get_tea_name(state, 1));
              else
                snprintf(state->tetra_ts_type[s], sizeof(state->tetra_ts_type[s]),
                         "%s", state->tetra_call_type[0] ? state->tetra_call_type : "Voice Call [Clear]");
            }
            if (state->tetra_ts_tg[s] == 0 && state->tetra_last_tg != 0 &&
                state->tetra_ts_mask != 0 && (state->tetra_ts_mask & (1U << (3 - s))))
            {
              state->tetra_ts_tg[s] = state->tetra_last_tg;
              state->tetra_ts_src[s] = state->tetra_last_src;
            }
          }
          else if (state->tetra_ts_usage[s] > 3 && state->tetra_ts_idle_count[s] == 0)
          {
            /* Keep active traffic slot locked across single-burst AACH bit errors */
            is_traffic = 1;
            crc_passed = 1;
            tetra_anchor_tdma_sync(state, is_bin, sync_idx);
            state->tetra_ts_last_sym[s] = state->tetra_sym_count;
            state->tetra_ts_idle_count[s] = 0;
          }
        }
        else if (state->tetra_dl_usage == 0 &&
                 state->tetra_ts_usage[s] > 3)
        {
          state->tetra_ts_idle_count[s]++;
          if (state->tetra_ts_idle_count[s] >= 10)
          {
            tetra_reset_slot_state(opts, state, s);
          }
        }

        /* Release active voice slot lock if the locked slot returns to unallocated/control */
        if (state->tetra_voice_slot != 0 &&
            state->tetra_slot == state->tetra_voice_slot &&
            state->tetra_ts_usage[s] <= 3)
        {
          tetra_release_voice_slot(opts, state, 1);
        }

        uint8_t tmp_ukey[16];
        uint8_t tmp_ualg = 0;
        int has_ukey = tetra_get_active_key(opts, state, tmp_ukey, &tmp_ualg) > 0;
        int unmute_enc = tetra_is_enc_unmuted(opts) || has_ukey;
        int play_voice = is_traffic && (!is_enc || unmute_enc);
        const char *tea = tetra_get_tea_name(state, is_enc);

        if (play_voice)
        {
          /* Lock onto the first active voice slot (or usage marker if TDMA slot is not yet known) */
          if (state->tetra_voice_slot == 0 && state->tetra_voice_usage == 0)
          {
            state->tetra_voice_slot = state->tetra_slot;
            state->tetra_voice_usage = state->tetra_dl_usage;
            state->tetra_voice_last_sym = state->tetra_sym_count;
          }

          int is_locked_slot = 0;
          if (state->tetra_voice_slot != 0 && state->tetra_slot != 0)
          {
            is_locked_slot = (state->tetra_slot == state->tetra_voice_slot);
          }
          else
          {
            is_locked_slot = (state->tetra_dl_usage == state->tetra_voice_usage);
            if (is_locked_slot && state->tetra_slot != 0)
              state->tetra_voice_slot = state->tetra_slot;
          }

          if (is_locked_slot)
          {
            state->tetra_voice_last_sym = state->tetra_sym_count;
            int has_codec = tetra_voice_codec_available();
            if (!has_codec || tetra_queue_tch_voice(opts, state, raw432_soft, scramb))
            {
              crc_passed = 1;
              state->carrier = 1;
              state->last_cc_sync_time = time(NULL);
              state->last_vc_sync_time = time(NULL);
              g_tetra_last_console_log_time = time(NULL);

              if (state->tetra_call_type[0] == '\0')
                if (state->tetra_call_type[0] == '\0') snprintf(state->tetra_call_type, sizeof(state->tetra_call_type), "Voice Call [%s]", tea);
              snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
                       "TCH/S Voice (%s) TN:%u Usage:%u",
                       has_codec ? "ACELP" : "Data Only", state->tetra_slot, state->tetra_dl_usage);

              if (is_enc && has_ukey && state->tetra_matched_key_str[0] != '\0')
              {
                if (has_codec)
                  fprintf(stderr, "%s Sync: %s NDB (TCH/S) TN:%u FN:%02u MN:%02u Usage:%u %s[TEA%u Decrypted Key: %s] VOICE [ACELP BFI:%u/%u]%s\n",
                          timestr, inv ? "-TETRA" : "+TETRA",
                          state->tetra_slot, state->tetra_frame, state->tetra_multiframe,
                          state->tetra_dl_usage, KGRN, tmp_ualg, state->tetra_matched_key_str, state->errs, state->errs2, KNRM);
                else
                  fprintf(stderr, "%s Sync: %s NDB (TCH/S) TN:%u FN:%02u MN:%02u Usage:%u %s[TEA%u Decrypted Key: %s] VOICE [Data Only]%s\n",
                          timestr, inv ? "-TETRA" : "+TETRA",
                          state->tetra_slot, state->tetra_frame, state->tetra_multiframe,
                          state->tetra_dl_usage, KGRN, tmp_ualg, state->tetra_matched_key_str, KNRM);
              }
              else if (is_enc)
              {
                fprintf(stderr, "%s Sync: %s NDB (TCH/S) TN:%u FN:%02u MN:%02u Usage:%u %sENCRYPTED VOICE [%s]%s%s\n",
                        timestr, inv ? "-TETRA" : "+TETRA",
                        state->tetra_slot, state->tetra_frame, state->tetra_multiframe,
                        state->tetra_dl_usage, KYEL, tea,
                        tetra_is_enc_unmuted(opts) ? (has_codec ? " (UNMUTED)" : " (Data Only)") : "", KNRM);
              }
              else if (has_codec)
              {
                fprintf(stderr, "%s Sync: %s NDB (TCH/S) TN:%u FN:%02u MN:%02u Usage:%u %sVOICE [ACELP BFI:%u/%u]%s\n",
                        timestr, inv ? "-TETRA" : "+TETRA",
                        state->tetra_slot, state->tetra_frame, state->tetra_multiframe,
                        state->tetra_dl_usage, KGRN, state->errs, state->errs2, KNRM);
              }
              else
              {
                fprintf(stderr, "%s Sync: %s NDB (TCH/S) TN:%u FN:%02u MN:%02u Usage:%u %sVOICE [Data Only]%s\n",
                        timestr, inv ? "-TETRA" : "+TETRA",
                        state->tetra_slot, state->tetra_frame, state->tetra_multiframe,
                        state->tetra_dl_usage, KGRN, KNRM);
              }

              tetra_log_voice_slot_event(opts, state, s, tea);
            }
          }
          else if (state->tetra_tdma_locked)
          {
            /* Another timeslot is carrying voice simultaneously while tetra_voice_slot is locked */
            crc_passed = 1;
            state->carrier = 1;
            state->last_cc_sync_time = time(NULL);
            g_tetra_last_console_log_time = time(NULL);

            fprintf(stderr, "%s Sync: %s NDB (TCH/S) TN:%u FN:%02u MN:%02u Usage:%u %sVOICE [%s - BUSY Locked TN:%u]%s\n",
                    timestr, inv ? "-TETRA" : "+TETRA",
                    state->tetra_slot, state->tetra_frame, state->tetra_multiframe,
                    state->tetra_dl_usage, KCYN, tea, state->tetra_voice_slot, KNRM);

            tetra_log_voice_slot_event(opts, state, s, tea);
          }
        }
        else if (is_traffic)
        {
          if (state->tetra_voice_slot == state->tetra_slot)
            tetra_release_voice_slot(opts, state, 1);
          crc_passed = 1;
          state->carrier = 1;
          state->last_cc_sync_time = time(NULL);
          g_tetra_last_console_log_time = time(NULL);

          state->tetra_ts_usage[s] = state->tetra_dl_usage;
          state->tetra_ts_last_sym[s] = state->tetra_sym_count;
          if (state->tetra_ts_type[s][0] == '\0')
            if (state->tetra_ts_type[s][0] == '\0') snprintf(state->tetra_ts_type[s], sizeof(state->tetra_ts_type[s]), "Voice Call [%s]", tea);

          fprintf(stderr, "%s Sync: %s NDB (TCH/S) TN:%u FN:%02u MN:%02u Usage:%u %sENCRYPTED VOICE [%s]%s\n",
                  timestr, inv ? "-TETRA" : "+TETRA",
                  state->tetra_slot, state->tetra_frame, state->tetra_multiframe,
                  state->tetra_dl_usage, KYEL, tea, KNRM);

          tetra_log_voice_slot_event(opts, state, s, tea);
        }
      }
    }
  }

  /* 5. Process DMO Normal Burst (DNB) or Recognize TMO Normal Uplink Burst (NUB) */
  if (!crc_passed && state->tetra_burst_type >= 1 && state->tetra_burst_type <= 3 &&
      (state->tetra_mode != 0 || !state->tetra_scramb_valid))
  {
    uint32_t scramb = state->tetra_scramb_valid ? state->tetra_scramb_init : TETRA_SCRAMB_INIT_DEFAULT;
    uint16_t exp_pre = (state->tetra_burst_type == 1) ? 0x323U : 0x9A9U;
    int has_dmo_pre = (state->tetra_burst_type <= 2)
                      ? tetra_check_dmo_preamble(burst_bits, 14, exp_pre, 2) : 0;

    if (state->tetra_burst_type == 2) /* DMO NORM_2: STCH in half-slot 1 (and optionally STCH in half-slot 2) */
    {
      uint8_t blk1[124], blk2[124];
      int ok1 = tetra_decode_block_cch_soft(&burst_soft[28], 216, 101, 144, 124, scramb, blk1);
      if (!ok1 && scramb != TETRA_SCRAMB_INIT_DEFAULT)
        ok1 = tetra_decode_block_cch_soft(&burst_soft[28], 216, 101, 144, 124, TETRA_SCRAMB_INIT_DEFAULT, blk1);

      int ok2 = tetra_decode_block_cch_soft(&burst_soft[266], 216, 101, 144, 124, scramb, blk2);
      if (!ok2 && scramb != TETRA_SCRAMB_INIT_DEFAULT)
        ok2 = tetra_decode_block_cch_soft(&burst_soft[266], 216, 101, 144, 124, TETRA_SCRAMB_INIT_DEFAULT, blk2);

      if (ok1 || ok2)
      {
        crc_passed = 1;
        state->carrier = 1;
        state->last_cc_sync_time = time(NULL);
        state->tetra_mode = 1;
        tetra_anchor_tdma_sync(state, is_bin, sync_idx);

        char hdr[160];
        snprintf(hdr, sizeof(hdr), "%s Sync: %s DMO DNB (STCH)", timestr, inv ? "-TETRA" : "+TETRA");
        tetra_set_burst_hdr(hdr);
        if (ok1)
          tetra_parse_dmac_pdu(opts, state, blk1, 124);
        if (ok2)
          tetra_parse_dmac_pdu(opts, state, blk2, 124);
        if (opts->payload == 1)
        {
          tetra_ensure_burst_hdr();
          if (ok1)
            tetra_print_payload_hex("TETRA DMO STCH1 HEX", blk1, 15);
          if (ok2)
            tetra_print_payload_hex("TETRA DMO STCH2 HEX", blk2, 15);
        }
        tetra_finish_burst_hdr(1);
      }
    }
    else if (state->tetra_burst_type == 1) /* DMO NORM_1: Full-slot SCH/F or DM-TCH/S Speech */
    {
      int16_t dmo432_soft[432];
      uint8_t schf_bits[268];
      memcpy(&dmo432_soft[0], &burst_soft[28], 216 * sizeof(int16_t));
      memcpy(&dmo432_soft[216], &burst_soft[266], 216 * sizeof(int16_t));

      int ok_f = tetra_decode_block_cch_soft(dmo432_soft, 432, 103, 288, 268, scramb, schf_bits);
      if (!ok_f && scramb != TETRA_SCRAMB_INIT_DEFAULT)
        ok_f = tetra_decode_block_cch_soft(dmo432_soft, 432, 103, 288, 268, TETRA_SCRAMB_INIT_DEFAULT, schf_bits);

      if (ok_f)
      {
        crc_passed = 1;
        state->carrier = 1;
        state->last_cc_sync_time = time(NULL);
        state->tetra_mode = 1;
        tetra_anchor_tdma_sync(state, is_bin, sync_idx);

        char hdr[160];
        snprintf(hdr, sizeof(hdr), "%s Sync: %s DMO DNB (SCH/F)", timestr, inv ? "-TETRA" : "+TETRA");
        tetra_set_burst_hdr(hdr);
        tetra_parse_dmac_pdu(opts, state, schf_bits, 268);
        if (opts->payload == 1)
        {
          tetra_ensure_burst_hdr();
          tetra_print_payload_hex("TETRA DMO SCH/F HEX", schf_bits, 33);
        }
        tetra_finish_burst_hdr(1);
      }
      else if (state->tetra_mode == 1 && has_dmo_pre)
      {
        int is_enc = (state->tetra_enc || state->tetra_aie_enc) ? 1 : 0;
        uint8_t tmp_ukey[16];
        uint8_t tmp_ualg = 0;
        int has_ukey = tetra_get_active_key(opts, state, tmp_ukey, &tmp_ualg) > 0;
        int unmute_enc = tetra_is_enc_unmuted(opts) || has_ukey;
        const char *tea = tetra_get_tea_name(state, is_enc);
        int has_codec = tetra_voice_codec_available();

        if (!is_enc || unmute_enc)
        {
          if (!has_codec || tetra_queue_tch_voice(opts, state, dmo432_soft, scramb))
          {
            crc_passed = 1;
            state->carrier = 1;
            state->last_cc_sync_time = time(NULL);
            state->last_vc_sync_time = time(NULL);
            g_tetra_last_console_log_time = time(NULL);
            state->tetra_voice_slot = 1;
            state->tetra_voice_last_sym = state->tetra_sym_count;
            tetra_anchor_tdma_sync(state, is_bin, sync_idx);

            if (state->tetra_call_type[0] == '\0')
              snprintf(state->tetra_call_type, sizeof(state->tetra_call_type), "DMO Voice [%s]", tea);
            snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
                     "DM-TCH/S Voice (%s)", has_codec ? "ACELP" : "Data Only");

            if (is_enc && has_ukey && state->tetra_matched_key_str[0] != '\0')
            {
              fprintf(stderr, "%s Sync: %s DMO DNB (DM-TCH/S) %s[TEA%u Decrypted Key: %s] VOICE [%s]%s\n",
                      timestr, inv ? "-TETRA" : "+TETRA",
                      KGRN, tmp_ualg, state->tetra_matched_key_str,
                      has_codec ? "ACELP" : "Data Only", KNRM);
            }
            else if (is_enc)
            {
              fprintf(stderr, "%s Sync: %s DMO DNB (DM-TCH/S) %sENCRYPTED VOICE [%s]%s\n",
                      timestr, inv ? "-TETRA" : "+TETRA", KYEL, tea, KNRM);
            }
            else
            {
              fprintf(stderr, "%s Sync: %s DMO DNB (DM-TCH/S) %sVOICE [%s BFI:%u/%u]%s\n",
                      timestr, inv ? "-TETRA" : "+TETRA",
                      KGRN, has_codec ? "ACELP" : "Data Only", state->errs, state->errs2, KNRM);
            }
          }
        }
        else
        {
          crc_passed = 1;
          state->carrier = 1;
          state->last_cc_sync_time = time(NULL);
          g_tetra_last_console_log_time = time(NULL);
          tetra_anchor_tdma_sync(state, is_bin, sync_idx);
          if (state->tetra_call_type[0] == '\0')
            snprintf(state->tetra_call_type, sizeof(state->tetra_call_type), "DMO Voice [%s]", tea);
          fprintf(stderr, "%s Sync: %s DMO DNB (DM-TCH/S) %sENCRYPTED VOICE [%s]%s\n",
                  timestr, inv ? "-TETRA" : "+TETRA", KYEL, tea, KNRM);
        }
      }
    }

    /* If neither TMO Downlink nor DMO matched, and we are not locked to a Downlink/DMO carrier,
     * check if this is a TMO Uplink Burst (CB with 30-bit X_BITS or NUB with N_BITS / P_BITS).
     * To avoid false positives on weak signals or TMO Downlink carriers before BSCH arrives:
     * 1. Require strong signal level (step >= 400) and 0 bit errors across the training sequence.
     * 2. Require single-slot mobile TDMA cadence (bursts spaced by 1020 symbols = 4 slots, with
     *    silence on the other 3 slots). Any burst arriving ~255/510/765 symbols later is a
     *    multi-slot Base Station Downlink and resets the uplink candidate counter. */
    if (!crc_passed && !state->tetra_scramb_valid && !has_dmo_pre &&
        (state->tetra_burst_type == 1 || state->tetra_burst_type == 2) &&
        (is_bin || step >= 400.0f))
    {
      const uint8_t *exp_seq = (state->tetra_burst_type == 1) ? TETRA_N_DIBITS : TETRA_P_DIBITS;
      int train_errs = 0;
      for (int i = 0; i < 11; i++)
      {
        uint8_t d = (uint8_t)((burst_bits[244 + i * 2] << 1) | burst_bits[245 + i * 2]);
        train_errs += tetra_dibit_bit_diff(d, exp_seq[i]);
      }

      uint32_t cur_b_sym = state->tetra_sync_sym_count;
      uint32_t delta_sym = (g_tetra_ul_last_sym != 0 && cur_b_sym > g_tetra_ul_last_sym)
                           ? (cur_b_sym - g_tetra_ul_last_sym) : 0;
      g_tetra_ul_last_sym = cur_b_sym;

      if (train_errs == 0 &&
          tetra_is_valid_tmo_uplink_burst(state, is_bin, sync_idx, inv, swap, step, burst_soft))
      {
        int is_cb_x = (state->tetra_burst_type == 1 &&
                       tetra_bits_to_uint(burst_bits, 240, 4) == 0x9U &&
                       tetra_bits_to_uint(burst_bits, 266, 4) == 0x3U);
        int on_ul_frame_cadence = (delta_sym >= 1012 && delta_sym <= 1028) ||
                                  (delta_sym >= 2030 && delta_sym <= 2050);

        if (delta_sym > 0 && delta_sym < 980)
        {
          /* Bursts arriving 1, 2, or 3 slots apart (255/510/765 symbols) = Downlink carrier! */
          g_tetra_ul_cand_count = 0;
        }
        else if (on_ul_frame_cadence || (is_cb_x && (delta_sym == 0 || delta_sym >= 1012)))
        {
          if (g_tetra_ul_cand_count < 250)
            g_tetra_ul_cand_count++;
        }
        else
        {
          g_tetra_ul_cand_count = 1;
        }

        if (g_tetra_ul_cand_count >= 4 || (is_cb_x && g_tetra_ul_cand_count >= 2))
        {
          crc_passed = 1;
          state->carrier = 1;
          state->last_cc_sync_time = time(NULL);
          state->tetra_mode = 2;
          const char *ul_desc = is_cb_x ? "CB (Random Access / SCH/HU)" :
                                (state->tetra_burst_type == 1) ? "NUB (Voice TCH/S or SCH/F)" :
                                                                 "NUB (Stolen STCH Signalling)";
          int changed = (strcmp(state->tetra_call_type, ul_desc) != 0);
          snprintf(state->tetra_call_type, sizeof(state->tetra_call_type), "%s", ul_desc);
          time_t now_ul = time(NULL);
          if (changed || now_ul != g_tetra_last_console_log_time)
          {
            g_tetra_last_console_log_time = now_ul;
            fprintf(stderr, "%s Sync: %s TMO Uplink - %s\n", timestr, inv ? "-TETRA" : "+TETRA", ul_desc);
          }
        }
      }
      else
      {
        g_tetra_ul_cand_count = 0;
      }
    }
  }

  /* If no block passed CRC-16, rewind the symbol/sub-sample stream so zero symbols are lost! */
  if (!crc_passed)
  {
    if (!is_bin)
      state->tetra_sub_read_idx = state->tetra_prev_sub_read_idx;
    state->tetra_sym_count = state->tetra_sync_sym_count;
    if (!state->tetra_scramb_valid)
    {
      state->lastsynctype = -1;
      state->carrier = 0;
    }
  }

  if (timestr != NULL)
    free(timestr);
}
