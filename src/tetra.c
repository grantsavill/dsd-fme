/*-------------------------------------------------------------------------------
 * tetra.c
 * TETRA (Terrestrial Trunked Radio) Voice & Data Decoder for DSD-FME
 * Ported from TetraEar reference implementation (Lower MAC, Upper MAC, LLC,
 * MLE, CMCE, MM, SDS, LIP Location, MCC/MNC Lookup, TCH/S ACELP Voice)
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
          fprintf(stderr, "\n %sTETRA LIP Long Report - SSI: %u Lat: %.6f Lon: %.6f%s",
                  KYEL, effective_src, lat, lon, KNRM);
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

static void tetra_release_voice_slot(dsd_opts *opts, dsd_state *state, int flush_pending)
{
  if (flush_pending && state->tetra_cod_batch_count > 0)
    tetra_flush_voice_batch(opts, state);
  state->tetra_cod_batch_count = 0;
  state->tetra_voice_slot = 0;
  state->tetra_voice_usage = 0;
  state->tetra_serial_hist_frames = 0;
}

void tetra_clear_cell_state(dsd_state *state)
{
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
}

void tetra_reset_state(dsd_state *state)
{
  tetra_clear_cell_state(state);
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

static void tetra_reset_slot_state(dsd_state *state, int s)
{
  if (s < 0 || s >= 4)
    return;
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
    if (state->tetra_ev_ssi_cache[i] == ssi && (now - state->tetra_ev_time_cache[i]) < 15)
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

static void tetra_log_data_event(dsd_opts *opts, dsd_state *state, uint32_t src_ssi, uint32_t dst_ssi, const char *desc)
{
  char full_desc[256];
  char car_str[24];
  uint16_t car = state->tetra_carrier ? state->tetra_carrier : state->tetra_main_carrier;
  if (car > 0)
    snprintf(car_str, sizeof(car_str), " Car:%u", car);
  else
    car_str[0] = '\0';

  if (src_ssi != 0 && dst_ssi != 0)
    snprintf(full_desc, sizeof(full_desc), "%s%s TGT:%u SRC:%u", desc, car_str, dst_ssi, src_ssi);
  else if (dst_ssi != 0)
    snprintf(full_desc, sizeof(full_desc), "%s%s TGT:%u", desc, car_str, dst_ssi);
  else if (src_ssi != 0)
    snprintf(full_desc, sizeof(full_desc), "%s%s SRC:%u", desc, car_str, src_ssi);
  else
    snprintf(full_desc, sizeof(full_desc), "%s%s", desc, car_str);

  uint32_t ev_src = (src_ssi != 0) ? src_ssi : ((dst_ssi != 0) ? dst_ssi : 0xFFFFFFU);

  /* Avoid duplicate Event History entries when a call starts on a TN before its TGT SSI is mapped */
  if (state->event_history_s != NULL &&
      strstr(state->event_history_s[0].Event_History_Items[1].event_string, desc) != NULL)
  {
    if (dst_ssi == 0 && src_ssi == 0)
      return;
    if (state->event_history_s[0].Event_History_Items[1].target_id == 0)
    {
      char *timestr = getTimeN(time(NULL));
      char *datestr = getDateN(time(NULL));
      state->event_history_s[0].Event_History_Items[1].source_id = ev_src;
      state->event_history_s[0].Event_History_Items[1].target_id = dst_ssi;
      snprintf(state->event_history_s[0].Event_History_Items[1].event_string,
               sizeof(state->event_history_s[0].Event_History_Items[1].event_string),
               "%s %s %s", datestr ? datestr : "", timestr ? timestr : "", full_desc);
      if (timestr != NULL) free(timestr);
      if (datestr != NULL) free(datestr);
      return;
    }
  }

  snprintf(state->call_string[0], sizeof(state->call_string[0]), "%s", desc);
  state->gi[0] = (dst_ssi != 0 && src_ssi != 0 && dst_ssi != src_ssi) ? 0 : 1;
  state->event_history_s[0].Event_History_Items[0].color_pair = 4;
  watchdog_event_datacall(opts, state, ev_src, dst_ssi, full_desc, 0);
  state->lastsrc = 0;
  state->lasttg = 0;
  watchdog_event_history(opts, state, 0);
}

static void tetra_parse_cmce_party_ssi(dsd_state *state, const uint8_t *sdu, int sdu_len, int *pos_io)
{
  int pos = *pos_io;
  if (pos + 2 > sdu_len)
    return;
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
  if (ssi != 0 && ssi != 0xFFFFFFU)
    state->tetra_last_src = ssi;
  *pos_io = pos;
}

static void tetra_apply_cmce_call_slots(dsd_opts *opts, dsd_state *state,
                                        uint16_t call_id, int is_enc, int log_event)
{
  uint8_t mask = state->tetra_ts_mask;
  if (mask == 0 && state->tetra_slot >= 1 && state->tetra_slot <= 4)
    mask = (uint8_t)(1U << (4 - state->tetra_slot));
  for (int s = 0; s < 4; s++)
  {
    if ((mask & (1U << (3 - s))) || (call_id != 0 && state->tetra_ts_call_id[s] == call_id))
    {
      if (state->tetra_last_tg != 0)
        state->tetra_ts_tg[s] = state->tetra_last_tg;
      if (state->tetra_last_src != 0)
        state->tetra_ts_src[s] = state->tetra_last_src;
      state->tetra_ts_call_id[s] = call_id;
      if (state->tetra_call_type[0] != '\0')
        snprintf(state->tetra_ts_type[s], sizeof(state->tetra_ts_type[s]), "%s", state->tetra_call_type);
      state->tetra_ts_last_sym[s] = state->tetra_sym_count;
    }
  }
  if (log_event && tetra_should_log_ssi(state, state->tetra_last_tg))
  {
    char ev_desc[128];
    snprintf(ev_desc, sizeof(ev_desc), "TETRA Call [%s]", tetra_get_tea_name(state, is_enc));
    tetra_log_data_event(opts, state, state->tetra_last_src, state->tetra_last_tg, ev_desc);
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
  if (mac_ssi != 0 && mac_ssi != 0xFFFFFFU)
    state->tetra_last_tg = mac_ssi;

  if (pdu_t == 7 && sdu_len >= 43) /* D-SETUP */
  {
    uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
    uint8_t simplex = (sdu[27] == 0) ? 1 : 0;
    uint8_t basic_svc = (uint8_t)tetra_bits_to_uint(sdu, 28, 8);
    uint8_t circuit_mode = (basic_svc >> 5) & 0x07U;
    uint8_t svc_enc = (basic_svc >> 4) & 0x01U;
    uint8_t comm_type = (basic_svc >> 2) & 0x03U;
    uint8_t prio = (uint8_t)tetra_bits_to_uint(sdu, 39, 4);

    state->tetra_call_id = call_id;
    if (svc_enc)
      state->tetra_enc = 1;

    int is_enc = (state->tetra_enc || svc_enc || state->tetra_aie_enc) ? 1 : 0;
    const char *comm_str = (comm_type == 0) ? "Indiv" :
                           (comm_type == 1) ? "Group" :
                           (comm_type == 2) ? "Direct" : "Bcast";
    const char *mode_str = (circuit_mode == 0) ? "Call" : "Data";
    snprintf(state->tetra_call_type, sizeof(state->tetra_call_type),
             "%s %s [%s]", comm_str, mode_str, tetra_get_tea_name(state, is_enc));

    /* Check optional elements to extract Calling Party SSI */
    int pos = 43;
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
    tetra_parse_cmce_party_ssi(state, sdu, sdu_len, &pos);

    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
             "D-SETUP CallID:%u %s (%s) Prio:%u",
             call_id, state->tetra_call_type, simplex ? "Simplex" : "Duplex", prio);
    fprintf(stderr, " %sD-SETUP CallID:%u TGT:%u SRC:%u Mode:%s%s",
            KCYN, call_id, state->tetra_last_tg, state->tetra_last_src,
            state->tetra_call_type, KNRM);

    tetra_apply_cmce_call_slots(opts, state, call_id, is_enc, 1);
  }
  else if (pdu_t == 11 && sdu_len >= 28) /* D-TX-GRANTED */
  {
    uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
    uint8_t tx_grant = (uint8_t)tetra_bits_to_uint(sdu, 22, 2);
    uint8_t encr_ctrl = sdu[25];
    state->tetra_call_id = call_id;
    if (encr_ctrl)
      state->tetra_enc = 1;

    int pos = 27;
    if (pos < sdu_len)
    {
      uint8_t notif = sdu[pos++];
      if (notif && pos + 6 <= sdu_len)
        pos += 6;
    }
    tetra_parse_cmce_party_ssi(state, sdu, sdu_len, &pos);
    int is_enc = (state->tetra_enc || encr_ctrl || state->tetra_aie_enc) ? 1 : 0;
    static const char *GRANT_STR[4] = {"Granted", "NotGranted", "Queued", "GrantedOther"};
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
             "D-TX-GRANTED CallID:%u (%s) SRC:%u",
             call_id, GRANT_STR[tx_grant & 3], state->tetra_last_src);
    fprintf(stderr, " %sD-TX-GRANTED CallID:%u (%s) TGT:%u SRC:%u%s",
            KCYN, call_id, GRANT_STR[tx_grant & 3], state->tetra_last_tg, state->tetra_last_src, KNRM);
    tetra_apply_cmce_call_slots(opts, state, call_id, is_enc, (tx_grant == 0));
  }
  else if ((pdu_t == 9 || pdu_t == 13) && sdu_len >= 23) /* D-TX-CEASED or D-TX-INTERRUPT */
  {
    tetra_release_voice_slot(opts, state, 1);
    uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
    state->tetra_call_id = call_id;
    for (int s = 0; s < 4; s++)
    {
      if (state->tetra_ts_call_id[s] == call_id)
      {
        state->tetra_ts_src[s] = 0;
      }
    }
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str), "%s CallID:%u", pdu_name, call_id);
    fprintf(stderr, " %s CallID:%u", pdu_name, call_id);
  }
  else if ((pdu_t == 4 || pdu_t == 6) && sdu_len >= 27) /* D-DISCONNECT or D-RELEASE */
  {
    tetra_release_voice_slot(opts, state, 1);
    uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
    uint8_t cause = (uint8_t)tetra_bits_to_uint(sdu, 22, 5);
    for (int s = 0; s < 4; s++)
    {
      if (call_id != 0 && state->tetra_ts_call_id[s] == call_id)
      {
        tetra_reset_slot_state(state, s);
      }
    }
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
             "%s CallID:%u Cause:%u", pdu_name, call_id, cause);
    fprintf(stderr, " %s CallID:%u Cause:%u", pdu_name, call_id, cause);
  }
  else if (pdu_t == 2 && sdu_len >= 30) /* D-CONNECT */
  {
    uint16_t call_id = (uint16_t)tetra_bits_to_uint(sdu, 8, 14);
    state->tetra_call_id = call_id;
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str), "D-CONNECT CallID:%u", call_id);
    fprintf(stderr, " %sD-CONNECT CallID:%u TGT:%u SRC:%u%s",
            KCYN, call_id, state->tetra_last_tg, state->tetra_last_src, KNRM);
  }
  else if (pdu_t == 8 && sdu_len >= 26) /* D-STATUS */
  {
    int pos = 8;
    tetra_parse_cmce_party_ssi(state, sdu, sdu_len, &pos);

    if (pos + 16 <= sdu_len)
    {
      uint16_t status_code = (uint16_t)tetra_bits_to_uint(sdu, pos, 16);
      state->tetra_status_code = status_code;
      int is_enc = (state->tetra_enc || state->tetra_aie_enc) ? 1 : 0;
      snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
               "D-STATUS Code:0x%04X (%s) SRC:%u",
               status_code, (status_code == 0x8000) ? "EMERGENCY" : "Status", state->tetra_last_src);
      fprintf(stderr, " %sD-STATUS SRC:%u TGT:%u Status:0x%04X%s%s",
              (status_code == 0x8000) ? KRED : KGRN,
              state->tetra_last_src, state->tetra_last_tg, status_code,
              (status_code == 0x8000) ? " [EMERGENCY]" : "", KNRM);
      char ev_desc[128];
      snprintf(ev_desc, sizeof(ev_desc), "TETRA Status 0x%04X [%s]",
               status_code, tetra_get_tea_name(state, is_enc));
      tetra_log_data_event(opts, state, state->tetra_last_src, state->tetra_last_tg, ev_desc);
    }
  }
  else if (pdu_t == 15 && sdu_len >= 12) /* D-SDS-DATA */
  {
    int pos = 8;
    tetra_parse_cmce_party_ssi(state, sdu, sdu_len, &pos);

    if (pos + 2 <= sdu_len)
    {
      uint8_t sds_type = (uint8_t)tetra_bits_to_uint(sdu, pos, 2);
      pos += 2;
      if (sds_type == 0 && pos + 16 <= sdu_len)
      {
        uint16_t val16 = (uint16_t)tetra_bits_to_uint(sdu, pos, 16);
        snprintf(state->tetra_sds_str, sizeof(state->tetra_sds_str), "SDS-1: 0x%04X (%u)", val16, val16);
        fprintf(stderr, " %sD-SDS-DATA (Type 1) SRC:%u TGT:%u Val:0x%04X%s",
                KGRN, state->tetra_last_src, state->tetra_last_tg, val16, KNRM);
      }
      else if (sds_type == 1 && pos + 32 <= sdu_len)
      {
        uint32_t val32 = tetra_bits_to_uint(sdu, pos, 32);
        snprintf(state->tetra_sds_str, sizeof(state->tetra_sds_str), "SDS-2: 0x%08X", val32);
        fprintf(stderr, " %sD-SDS-DATA (Type 2) SRC:%u TGT:%u Val:0x%08X%s",
                KGRN, state->tetra_last_src, state->tetra_last_tg, val32, KNRM);
      }
      else if (sds_type == 2 && pos + 64 <= sdu_len)
      {
        uint32_t hi = tetra_bits_to_uint(sdu, pos, 32);
        uint32_t lo = tetra_bits_to_uint(sdu, pos + 32, 32);
        snprintf(state->tetra_sds_str, sizeof(state->tetra_sds_str), "SDS-3: 0x%08X%08X", hi, lo);
        fprintf(stderr, " %sD-SDS-DATA (Type 3) SRC:%u TGT:%u Val:0x%08X%08X%s",
                KGRN, state->tetra_last_src, state->tetra_last_tg, hi, lo, KNRM);
      }
      else if (sds_type == 3 && pos + 11 <= sdu_len)
      {
        uint16_t len_bits = (uint16_t)tetra_bits_to_uint(sdu, pos, 11);
        pos += 11;
        int avail = sdu_len - pos;
        if (len_bits < avail)
          avail = len_bits;
        fprintf(stderr, " %sD-SDS-DATA (Type 4, %d bits) SRC:%u TGT:%u%s",
                KGRN, avail, state->tetra_last_src, state->tetra_last_tg, KNRM);
        if (avail >= 8)
          tetra_decode_sds_type4(opts, state, &sdu[pos], avail, state->tetra_last_src);
      }
      int is_enc = (state->tetra_enc || state->tetra_aie_enc) ? 1 : 0;
      snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
               "D-SDS-DATA (Type %u) SRC:%u", sds_type + 1, state->tetra_last_src);
      char ev_desc[128];
      snprintf(ev_desc, sizeof(ev_desc), "TETRA SDS-%u [%s]",
               sds_type + 1, tetra_get_tea_name(state, is_enc));
      tetra_log_data_event(opts, state, state->tetra_last_src, state->tetra_last_tg, ev_desc);
    }
  }
  else
  {
    snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str), "CMCE %s", pdu_name);
    fprintf(stderr, " CMCE:%s", pdu_name);
  }
}

static void tetra_parse_llc_pdu(dsd_opts *opts, dsd_state *state, const uint8_t *bits, int nbits, uint32_t mac_ssi)
{
  if (nbits < 4)
    return;

  static const char *LLC_NAMES[16] = {
    "BL-ADATA", "BL-DATA", "BL-UDATA", "BL-ACK",
    "BL-ADATA-FCS", "BL-DATA-FCS", "BL-UDATA-FCS", "BL-ACK-FCS",
    "AL-SETUP", "AL-DATA", "AL-UDATA", "AL-ACK",
    "AL-RECONNECT", "LLC-13", "LLC-14", "AL-DISC"
  };

  uint8_t llc_type = (uint8_t)tetra_bits_to_uint(bits, 0, 4);
  int has_fcs = (llc_type >= 4 && llc_type <= 7);
  uint8_t base_type = has_fcs ? (uint8_t)(llc_type - 4) : llc_type;

  int hdr_len = 4;
  if (base_type == 0) hdr_len = 6;      /* BL-ADATA: nr + ns */
  else if (base_type == 1) hdr_len = 5; /* BL-DATA: ns */
  else if (base_type == 2) hdr_len = 4; /* BL-UDATA */
  else if (base_type == 3) hdr_len = 5; /* BL-ACK: nr */
  else hdr_len = 6;

  int sdu_end = (has_fcs && nbits > hdr_len + 32) ? (nbits - 32) : nbits;
  int sdu_len = sdu_end - hdr_len;
  fprintf(stderr, " LLC:%s", LLC_NAMES[llc_type & 0x0F]);

  if (sdu_len < 3)
    return;

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
      tetra_log_data_event(opts, state, mac_ssi, 0, state->tetra_pdu_str);
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
    state->tetra_ts_mask = ts_mask;
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
      snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str), "MAC-RESOURCE (Null)");
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
        if (state->tetra_ts_usage[s] == addr_usage_marker)
          state->tetra_ts_tg[s] = ssi;
      }
    }

    if (ssi != 0 && ssi != 0xFFFFFFU &&
        (encr_mode == 0 || state->tetra_dl_usage > 3 || state->tetra_burst_type == 3))
      state->tetra_last_tg = ssi;
    if (encr_mode > 0)
    {
      state->tetra_enc = encr_mode;
      state->tetra_aie_enc = 1;
    }

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
       * on a Traffic channel (AACH usage > 3) or STCH (burst_type == 3), NOT on MCCH! */
      if (state->tetra_dl_usage > 3 || state->tetra_burst_type == 3)
      {
        snprintf(state->tetra_call_type, sizeof(state->tetra_call_type), "Voice Call [%s]", tea);
        int s = (state->tetra_slot >= 1 && state->tetra_slot <= 4) ? (state->tetra_slot - 1) : 0;
        snprintf(state->tetra_ts_type[s], sizeof(state->tetra_ts_type[s]), "Voice Call [%s]", tea);
        if (ssi != 0 && ssi != 0xFFFFFFU)
          state->tetra_ts_tg[s] = ssi;
        state->tetra_ts_last_sym[s] = state->tetra_sym_count;
        if (state->tetra_scramb_valid && tetra_should_log_ssi(state, ssi))
        {
          char ev_desc[128];
          snprintf(ev_desc, sizeof(ev_desc), "TETRA Call [%s]", tea);
          tetra_log_data_event(opts, state, 0, ssi, ev_desc);
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

    int pdu_end = (len_ind >= 4 && len_ind <= 58) ? (len_ind * 8) : nbits;
    if (pdu_end > nbits)
      pdu_end = nbits;
    int sdu_end = pdu_end;
    if (fill_bit && sdu_end > pos)
      sdu_end = pos + tetra_find_fill_bits(&bits[pos], sdu_end - pos);

    int sdu_len = sdu_end - pos;
    if (sdu_len > 0 && encr_mode == 0)
    {
      tetra_parse_llc_pdu(opts, state, &bits[pos], sdu_len, ssi);
    }
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
      fprintf(stderr, " MAC-FRAG (%u bits)", state->tetra_mac_frag_len);
      *pdu_end_flag = 1;
      return nbits;
    }
    else if (nbits >= 13) /* MAC-END */
    {
      uint8_t end_len_ind = (uint8_t)tetra_bits_to_uint(bits, 5, 6);
      int pos = 11;
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
        if (state->tetra_mac_frag_enc == 0)
        {
          tetra_parse_llc_pdu(opts, state, state->tetra_mac_frag_buf,
                              state->tetra_mac_frag_len, state->tetra_mac_frag_ssi);
        }
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
      uint16_t prev_main = state->tetra_main_carrier;
      if (prev_main != 0 && prev_main != main_carr)
      {
        tetra_clear_cell_state(state);
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

      /* Check MAC-SYSINFO Optional field (bits 60..81): option_id == 3 is Extended Services Broadcast */
      if (nbits >= 82)
      {
        uint8_t opt_id = (uint8_t)tetra_bits_to_uint(bits, 60, 2);
        if (opt_id == 3 && bits[62] == 1)
        {
          uint8_t sec_cls = (uint8_t)tetra_bits_to_uint(bits, 63, 2);
          state->tetra_sec_class = (uint8_t)(sec_cls + 1);
        }
      }

      /* Extract LA (14 bits at 82..95) and BS Service Details (112..123) from D-MLE-SYSINFO */
      if (nbits >= 124)
      {
        uint16_t la = (uint16_t)tetra_bits_to_uint(bits, 82, 14);
        if (la > 0)
          state->tetra_la = la;
        state->tetra_aie_enc = bits[122]; /* Bit 10 of BS Service Details: Air interface encryption */
      }

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
      fprintf(stderr, " MAC-SYSINFO Carrier:%u", main_carr);
      if (state->tetra_dl_freq_mhz > 0.0)
        fprintf(stderr, " (%.06lf MHz)", state->tetra_dl_freq_mhz);
      if (state->tetra_la > 0)
        fprintf(stderr, " LA:%u", state->tetra_la);
      return 124;
    }
    else if (bcast_type == 1) /* ACCESS-DEFINE */
    {
      snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str), "MAC-BROADCAST ACCESS-DEFINE");
      fprintf(stderr, " MAC-ACCESS-DEFINE");
      if (nbits >= 29)
      {
        uint8_t opt_flag = (uint8_t)tetra_bits_to_uint(bits, 27, 2);
        if (opt_flag == 0) return 32;
        if (opt_flag == 1) return 48;
        if (opt_flag == 2) return 56;
      }
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
  if (step < 5.0f)
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

/* Test-decode BLK1 (108 symbols = 216 bits at burst symbols 7..114) from lookback buffer
 * when a candidate NORM training sequence (symbols 122..132) is detected. */
static int tetra_try_presync_blk1(const dsd_state *state, int is_bin, uint32_t sync_end_idx,
                                  float center, float step, int inv, int swap)
{
  int16_t blk1_soft[216];
  uint8_t blk1_bits[124];
  tetra_extract_lookback_soft(state, is_bin, sync_end_idx, 132, 7, 108,
                              center, step, inv, swap, NULL, blk1_soft);
  uint32_t scramb = state->tetra_scramb_valid ? state->tetra_scramb_init : TETRA_SCRAMB_INIT_DEFAULT;
  return tetra_decode_block_cch_soft(blk1_soft, 216, 101, 144, 124, scramb, blk1_bits);
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
  float best_sync_corr = 0.0f;
  int best_sync_delta = 0, best_sync_swap = 0, best_sync_inv = 0, best_sync_errs = 38;
  float best_sync_C = 0.0f, best_sync_A = 1.0f;

  for (int delta = -4; delta <= 3; delta++)
  {
    uint32_t sub_idx = (uint32_t)((int32_t)cur_sub + delta);
    for (int swap = 0; swap < 2; swap++)
    {
      if (state->tetra_tdma_locked && swap != state->tetra_bit_swap)
        continue;
      int inv = 0, errs = 38;
      float C = 0.0f, A = 1.0f;
      float corr = tetra_eval_train_seq_analog(state, sub_idx, TETRA_Y_DIBITS, 19, swap,
                                               &inv, &errs, &C, &A);
      if (state->tetra_tdma_locked && inv != opts->inverted_tetra)
        continue;
      if (corr > best_sync_corr)
      {
        best_sync_corr = corr;
        best_sync_delta = delta;
        best_sync_swap = swap;
        best_sync_inv = inv;
        best_sync_errs = errs;
        best_sync_C = C;
        best_sync_A = A;
      }
    }
  }

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
  float best_norm_corr = 0.0f;
  int best_norm_delta = 0, best_norm_swap = 0, best_norm_inv = 0, best_norm_s = 0, best_norm_errs = 22;
  float best_norm_C = 0.0f, best_norm_A = 1.0f;

  for (int delta = -4; delta <= 3; delta++)
  {
    uint32_t sub_idx = (uint32_t)((int32_t)cur_sub + delta);
    for (int swap = 0; swap < 2; swap++)
    {
      if (state->tetra_tdma_locked && swap != state->tetra_bit_swap)
        continue;
      for (int s = 0; s < 3; s++)
      {
        int inv = 0, errs = 22;
        float C = 0.0f, A = 1.0f;
        float corr = tetra_eval_train_seq_analog(state, sub_idx, norm_seqs[s], 11, swap,
                                                 &inv, &errs, &C, &A);
        if (state->tetra_tdma_locked && inv != opts->inverted_tetra)
          continue;
        if (corr > best_norm_corr)
        {
          best_norm_corr = corr;
          best_norm_delta = delta;
          best_norm_swap = swap;
          best_norm_inv = inv;
          best_norm_s = s;
          best_norm_errs = errs;
          best_norm_C = C;
          best_norm_A = A;
        }
      }
    }
  }

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
          (!state->tetra_tdma_locked && best_norm_corr >= 0.85f && best_norm_errs <= 2))
      {
        return tetra_commit_analog_sync(opts, state, cur_sub, best_sub, (uint8_t)(best_norm_s + 1),
                                        best_norm_swap, best_norm_inv, best_norm_C, best_norm_A);
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

  int unmute_enc = tetra_is_enc_unmuted(opts);
  int valid_frames = 0;
  int any_crc_ok = 0;
  for (int b = 0; b < actual_bursts; b++)
  {
    const int16_t *ser276 = &serial_batch[b * 276];
    int bfi1 = ser276[0];
    int bfi2 = ser276[138];
    state->errs = (unsigned int)bfi1;
    state->errs2 = (unsigned int)bfi2;
    if (bfi1 == 0 || bfi2 == 0)
      any_crc_ok = 1;

    if (bfi1 == 0 || bfi2 == 0 || unmute_enc)
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
      if (unmute_enc)
      {
        /* Clear BFI flags so sdecoder synthesizes encrypted ACELP frames instead of muting */
        dst[0] = 0;
        dst[138] = 0;
      }
      state->tetra_serial_hist_frames += 2;
      valid_frames += 2;
    }
  }

  if (!any_crc_ok && state->tetra_voice_slot >= 1 && state->tetra_voice_slot <= 4)
  {
    int s = state->tetra_voice_slot - 1;
    snprintf(state->tetra_ts_type[s], sizeof(state->tetra_ts_type[s]),
             "Voice Call [%s]", tetra_get_tea_name(state, 1));
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
  if (state->tetra_last_voice_time == 0 || (now - state->tetra_last_voice_time) > 2)
  {
    if (state->tetra_cod_batch_count > 0)
      tetra_flush_voice_batch(opts, state);
    state->tetra_serial_hist_frames = 0;
  }
  state->tetra_last_voice_time = now;

  int16_t type4_soft[432];
  tetra_scramb_soft(scramb_init, raw432_soft, type4_soft, 432);
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

static void tetra_update_cch_slot_state(dsd_state *state)
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
        state->tetra_ts_cand_count[0]++;
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
      if (state->tetra_dl_usage == 0 && state->tetra_ts_usage[s] != 0)
      {
        state->tetra_ts_idle_count[s]++;
        uint8_t idle_thresh = (state->tetra_ts_usage[s] == 2) ? 3 : 2;
        if (state->tetra_ts_idle_count[s] >= idle_thresh)
        {
          tetra_reset_slot_state(state, s);
        }
      }
    }
  }
}

static void tetra_log_voice_slot_event(dsd_opts *opts, dsd_state *state, int s, const char *tea)
{
  if (state->tetra_ts_cand_count[s] >= 3)
  {
    uint32_t eff_tg = state->tetra_ts_tg[s];
    if (state->tetra_ts_logged[s] == 0 || (state->tetra_ts_logged[s] == 1 && eff_tg != 0))
    {
      char ev_desc[128];
      snprintf(ev_desc, sizeof(ev_desc), "TETRA Voice [%s] TN:%u", tea, state->tetra_slot);
      tetra_log_data_event(opts, state, state->tetra_ts_src[s], eff_tg, ev_desc);
      state->tetra_ts_logged[s] = (eff_tg != 0) ? 2 : 1;
    }
  }
}

static void tetra_print_payload_hex(const char *label, const uint8_t *bits, int nbytes)
{
  fprintf(stderr, "\n %s: ", label);
  for (int i = 0; i < nbytes; i++)
    fprintf(stderr, "%02X", (unsigned int)tetra_bits_to_uint(bits, i * 8, 8));
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
    tetra_release_voice_slot(opts, state, 1);
  }

  for (int s = 0; s < 4; s++)
  {
    if ((state->tetra_ts_usage[s] != 0 || state->tetra_ts_type[s][0] != '\0') &&
        (uint32_t)(state->tetra_sym_count - state->tetra_ts_last_sym[s]) > 18360U)
    {
      tetra_reset_slot_state(state, s);
    }
  }

  if (state->tetra_scramb_valid && tetra_cc_filter_mismatch(opts, state->tetra_colour_code))
  {
    tetra_clear_cell_state(state);
  }

  /* 3. Process Synchronization Burst (SB1 + BBK + SB2) */
  if (state->tetra_burst_type == 0)
  {
    uint8_t sb1_bits[60];
    int sb1_ok = tetra_decode_block_cch_soft(&burst_soft[94], 120, 11, 80, 60,
                                             TETRA_SCRAMB_INIT_DEFAULT, sb1_bits);
    if (sb1_ok)
    {
      crc_passed = 1;
      state->carrier = 1;
      state->last_cc_sync_time = time(NULL);

      uint8_t colour = (uint8_t)tetra_bits_to_uint(sb1_bits, 4, 6);
      uint8_t ts = (uint8_t)(tetra_bits_to_uint(sb1_bits, 10, 2) + 1);
      uint8_t fn = (uint8_t)tetra_bits_to_uint(sb1_bits, 12, 5);
      uint8_t mn = (uint8_t)tetra_bits_to_uint(sb1_bits, 17, 6);
      uint16_t mcc = (uint16_t)tetra_bits_to_uint(sb1_bits, 31, 10);
      uint16_t mnc = (uint16_t)tetra_bits_to_uint(sb1_bits, 41, 14);

      int cell_changed = ((state->tetra_mcc != 0 && state->tetra_mcc != mcc) ||
                          (state->tetra_mnc != 0 && state->tetra_mnc != mnc) ||
                          (state->tetra_scramb_valid && state->tetra_colour_code != colour));

      if (cell_changed || tetra_cc_filter_mismatch(opts, colour))
      {
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
        tetra_decode_aach(state, &burst_bits[252], state->tetra_scramb_init, 1);

        fprintf(stderr, "%s Sync: %s BSCH MCC:%u (%s) MNC:%u CC:%u TN:%u FN:%02u MN:%02u",
                timestr, inv ? "-TETRA" : "+TETRA",
                mcc, state->tetra_country_str, mnc, colour, ts, fn, mn);

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
          tetra_print_payload_hex("TETRA SB1 HEX", sb1_bits, 7);
          if (sb2_ok)
            tetra_print_payload_hex("TETRA SB2 HEX", sb2_bits, 15);
        }
        fprintf(stderr, "\n");
      }
    }
  }
  else if (opts->tetra_cc_filter < 0 || opts->tetra_cc_filter > 63 ||
           (state->tetra_scramb_valid && !tetra_cc_filter_mismatch(opts, state->tetra_colour_code)))
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

      if (ok1 || ok2)
      {
        crc_passed = 1;
        tetra_decode_aach(state, bbk_raw, scramb, 0);
        tetra_update_cch_slot_state(state);
        tetra_anchor_tdma_sync(state, is_bin, sync_idx);

        fprintf(stderr, "%s Sync: %s NDB (SCH/HD)", timestr, inv ? "-TETRA" : "+TETRA");
        if (ok1)
          tetra_parse_mac_pdu(opts, state, blk1, 124);
        if (ok2)
          tetra_parse_mac_pdu(opts, state, blk2, 124);
        if (opts->payload == 1)
        {
          if (ok1)
            tetra_print_payload_hex("TETRA BLK1 HEX", blk1, 15);
          if (ok2)
            tetra_print_payload_hex("TETRA BLK2 HEX", blk2, 15);
        }
        fprintf(stderr, "\n");
      }
    }
    else if (state->tetra_burst_type == 3) /* NORM_3: Stolen channel STCH (half-slot 1, and optionally half-slot 2) */
    {
      uint8_t blk1[124], blk2[124];
      int ok1 = tetra_decode_block_cch_soft(&burst_soft[14], 216, 101, 144, 124, scramb, blk1);
      int ok2 = ok1 ? tetra_decode_block_cch_soft(&burst_soft[282], 216, 101, 144, 124, scramb, blk2) : 0;
      if (ok1)
      {
        crc_passed = 1;
        tetra_decode_aach(state, bbk_raw, scramb, 0);
        if (state->tetra_slot >= 1 && state->tetra_slot <= 4)
        {
          int s = state->tetra_slot - 1;
          uint8_t u = (state->tetra_dl_usage > 3) ? state->tetra_dl_usage
                                                  : ((state->tetra_ts_usage[s] > 3) ? state->tetra_ts_usage[s] : 4);
          state->tetra_dl_usage = u;
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

        fprintf(stderr, "%s Sync: %s NDB (STCH)", timestr, inv ? "-TETRA" : "+TETRA");
        tetra_parse_mac_pdu(opts, state, blk1, 124);
        if (ok2)
          tetra_parse_mac_pdu(opts, state, blk2, 124);
        fprintf(stderr, "\n");
      }
    }
    else /* NORM_1: Full-slot SCH/F (432 bits -> 268 bits) or TCH/S Speech (432 bits -> 2 x 137 bits) */
    {
      int16_t raw432_soft[432];
      uint8_t schf_bits[268];
      memcpy(&raw432_soft[0], &burst_soft[14], 216 * sizeof(int16_t));
      memcpy(&raw432_soft[216], &burst_soft[282], 216 * sizeof(int16_t));

      int ok_f = tetra_decode_block_cch_soft(raw432_soft, 432, 103, 288, 268, scramb, schf_bits);
      if (ok_f)
      {
        crc_passed = 1;
        tetra_decode_aach(state, bbk_raw, scramb, 0);
        tetra_update_cch_slot_state(state);
        tetra_anchor_tdma_sync(state, is_bin, sync_idx);

        fprintf(stderr, "%s Sync: %s NDB (SCH/F)", timestr, inv ? "-TETRA" : "+TETRA");
        tetra_parse_mac_pdu(opts, state, schf_bits, 268);
        if (opts->payload == 1)
          tetra_print_payload_hex("TETRA SCH/F HEX", schf_bits, 33);
        fprintf(stderr, "\n");
      }
      else if (state->tetra_tdma_locked && state->tetra_scramb_valid &&
               state->tetra_slot >= 1 && state->tetra_slot <= 4 &&
               state->tetra_frame >= 1 && state->tetra_frame <= 17)
      {
        tetra_decode_aach(state, bbk_raw, scramb, 0);
        int s = state->tetra_slot - 1;
        int is_traffic = 0;
        int is_enc = (state->tetra_enc || state->tetra_aie_enc || state->tetra_sec_class >= 2) ? 1 : 0;

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

          uint8_t req_cnt = (state->tetra_ts_usage[s] == 2) ? 3 : 2;
          if (state->tetra_ts_usage[s] == state->tetra_dl_usage ||
              state->tetra_ts_cand_count[s] >= req_cnt)
          {
            is_traffic = 1;
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
        }
        else if (state->tetra_dl_usage == 0 &&
                 state->tetra_ts_usage[s] != 0)
        {
          state->tetra_ts_idle_count[s]++;
          if (state->tetra_ts_idle_count[s] >= 3)
          {
            tetra_reset_slot_state(state, s);
          }
        }

        /* Release active voice slot lock if the locked slot returns to unallocated/control */
        if (state->tetra_voice_slot != 0 &&
            state->tetra_slot == state->tetra_voice_slot &&
            state->tetra_ts_usage[s] <= 3)
        {
          tetra_release_voice_slot(opts, state, 1);
        }

        int unmute_enc = tetra_is_enc_unmuted(opts);
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
            if (tetra_queue_tch_voice(opts, state, raw432_soft, scramb))
            {
              crc_passed = 1;
              tetra_anchor_tdma_sync(state, is_bin, sync_idx);
              state->last_vc_sync_time = time(NULL);

              if (state->tetra_call_type[0] == '\0')
                snprintf(state->tetra_call_type, sizeof(state->tetra_call_type), "Voice Call [%s]", tea);
              snprintf(state->tetra_pdu_str, sizeof(state->tetra_pdu_str),
                       "TCH/S Voice (ACELP) TN:%u Usage:%u", state->tetra_slot, state->tetra_dl_usage);

              if (is_enc)
              {
                fprintf(stderr, "%s Sync: %s NDB (TCH/S) TN:%u FN:%02u MN:%02u Usage:%u %sENCRYPTED VOICE [%s] (UNMUTED)%s\n",
                        timestr, inv ? "-TETRA" : "+TETRA",
                        state->tetra_slot, state->tetra_frame, state->tetra_multiframe,
                        state->tetra_dl_usage, KYEL, tea, KNRM);
              }
              else
              {
                fprintf(stderr, "%s Sync: %s NDB (TCH/S) TN:%u FN:%02u MN:%02u Usage:%u %sVOICE [ACELP BFI:%u/%u]%s\n",
                        timestr, inv ? "-TETRA" : "+TETRA",
                        state->tetra_slot, state->tetra_frame, state->tetra_multiframe,
                        state->tetra_dl_usage, KGRN, state->errs, state->errs2, KNRM);
              }

              tetra_log_voice_slot_event(opts, state, s, tea);
            }
          }
          else if (state->tetra_tdma_locked)
          {
            /* Another timeslot is carrying voice simultaneously while tetra_voice_slot is locked */
            crc_passed = 1;
            tetra_anchor_tdma_sync(state, is_bin, sync_idx);

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
            tetra_release_voice_slot(opts, state, 0);
          crc_passed = 1;
          tetra_anchor_tdma_sync(state, is_bin, sync_idx);

          state->tetra_ts_usage[s] = state->tetra_dl_usage;
          state->tetra_ts_last_sym[s] = state->tetra_sym_count;
          snprintf(state->tetra_ts_type[s], sizeof(state->tetra_ts_type[s]), "Voice Call [%s]", tea);

          fprintf(stderr, "%s Sync: %s NDB (TCH/S) TN:%u FN:%02u MN:%02u Usage:%u %sENCRYPTED VOICE [%s]%s\n",
                  timestr, inv ? "-TETRA" : "+TETRA",
                  state->tetra_slot, state->tetra_frame, state->tetra_multiframe,
                  state->tetra_dl_usage, KYEL, tea, KNRM);

          tetra_log_voice_slot_event(opts, state, s, tea);
        }
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
