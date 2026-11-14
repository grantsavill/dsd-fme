

#include "dsd.h"
#define BSIZE 999

int csvGroupImport(dsd_opts * opts, dsd_state * state)
{
  char filename[1024] = "filename.csv";
  sprintf (filename, "%s", opts->group_in_file);

  char buffer[BSIZE];
  FILE * fp;
  fp = fopen(filename, "r");

  if (fp == NULL)
  {
    printf("Unable to open group file '%s'\n", filename);
    exit(1);
  }

  int row_count = 0;
  int field_count = 0;

  unsigned int i = 0;
  while (fgets(buffer, BSIZE, fp))
  {

    field_count = 0;
    row_count++;
    if (row_count == 1)
      continue; //don't want labels
    char * field = strtok(buffer, ","); //seperate by comma
    while (field)
    {

      if (field_count == 0)
      {
        //group_number = atol(field);
        state->group_array[i].groupNumber = atol(field);
        fprintf (stderr, "%ld, ", state->group_array[i].groupNumber);
      }
      else if (field_count == 1)
      {
        // strcpy(state->group_array[i].groupMode, field);
        strncpy(state->group_array[i].groupMode, field, 2); //only copy two letters max A, B, D, DE
        fprintf (stderr, "%s, ", state->group_array[i].groupMode);
      }
      else if (field_count == 2)
      {
        // strcpy(state->group_array[i].groupName, field);
        strncpy(state->group_array[i].groupName, field, 98); //this should also fix potential issues observed in past when there is no comma after the name

        uint16_t len = strlen((const char*)state->group_array[i].groupName);

        //remove any potential linebreaks at this point
        if (state->group_array[i].groupName[len-1] == '\n')
        {
          // fprintf(stderr, " ** LEN: %d; ** ", len); //debug
          state->group_array[i].groupName[len-1] = '\0';
        }

        fprintf (stderr, "%s ", state->group_array[i].groupName);
      }

      field_count++;
    }

    fprintf (stderr, "\n");

    i++;
    state->group_tally++;

  }
  fclose(fp);
  return 0;
}

//LCN import for EDACS, migrated to channel map (channel map does both)
int csvLCNImport(dsd_opts * opts, dsd_state * state)
{
  char filename[1024] = "filename.csv";
  sprintf (filename, "%s", opts->lcn_in_file);
  //filename[1023] = '\0'; //necessary?
  char buffer[BSIZE];
  FILE * fp;
  fp = fopen(filename, "r");
  if (fp == NULL) {
    printf("Unable to open lcn file '%s'\n", filename);
    //have this return -1 and handle it inside of main
    exit(1);
  }
  int row_count = 0;
  int field_count = 0;

  while (fgets(buffer, BSIZE, fp)) {
    field_count = 0;
    row_count++;
    if (row_count == 1)
      continue; //don't want labels
    char * cursor = buffer;
    char * field = NULL;
    while (cursor != NULL) {
      field = cursor;
      char * comma = strchr(cursor, ',');
      if (comma != NULL)
      {
        *comma = '\0';
        cursor = comma + 1;
      }
      else
      {
        cursor = NULL;
      }

      state->trunk_lcn_freq[field_count] = atol (field);
      state->lcn_freq_count++; //keep tally of number of Frequencies imported
      fprintf (stderr, "LCN [%d] [%ld]", field_count+1, state->trunk_lcn_freq[field_count]);
      fprintf (stderr, "\n");


      field_count++;
    }
    fprintf (stderr, "LCN Count %d\n", state->lcn_freq_count);

  }
  fclose(fp);
  return 0;
}

static dsd_opts * g_trunk_opts = NULL;
static dsd_state * g_trunk_state = NULL;
static int g_trunk_lcn_cc[26];
static int g_trunk_lcn_nac[26];
static int g_trunk_chan_cc[0xFFFF];
static int g_trunk_chan_nac[0xFFFF];
static int g_trunk_has_access_codes = 0;
static int g_trunk_access_inited = 0;
static long int g_trunk_cur_freq = 0;
static time_t g_trunk_last_poll = 0;
static int g_trunk_prev_cc = -1;
static int g_trunk_retune_bursts = 0;
static time_t g_trunk_retune_time = 0;
static int g_t3_cc_lpcn = 0;
static int g_t3_vc_lpcn = 0;
static int g_cap_plus_cur_lsn = 0;

void trunk_set_t3_cc_lpcn(int lpcn)
{
  if (lpcn > 0 && lpcn < 0xFFF)
    g_t3_cc_lpcn = lpcn;
}

void trunk_set_t3_vc_lpcn(int lpcn)
{
  g_t3_vc_lpcn = (lpcn > 0 && lpcn < 0xFFF) ? lpcn : 0;
}

void trunk_set_cap_plus_cur_lsn(int lsn)
{
  g_cap_plus_cur_lsn = (lsn > 0 && lsn <= 32) ? lsn : 0;
}

void trunk_reset_dmr_channel_state(void)
{
  g_t3_cc_lpcn = 0;
  g_t3_vc_lpcn = 0;
  g_cap_plus_cur_lsn = 0;
}

static void trunk_ensure_access_arrays_inited(void)
{
  if (!g_trunk_access_inited)
  {
    memset(g_trunk_lcn_cc, -1, sizeof(g_trunk_lcn_cc));
    memset(g_trunk_lcn_nac, -1, sizeof(g_trunk_lcn_nac));
    memset(g_trunk_chan_cc, -1, sizeof(g_trunk_chan_cc));
    memset(g_trunk_chan_nac, -1, sizeof(g_trunk_chan_nac));
    g_trunk_has_access_codes = 0;
    g_trunk_access_inited = 1;
  }
}

static void trunk_apply_access_values(dsd_opts * opts, dsd_state * state, int cc_val, int nac_val)
{
  if (opts == NULL || state == NULL || g_trunk_has_access_codes == 0)
    return;

  if (opts->frame_dmr == 1)
  {
    opts->dmr_cc_filter = (cc_val >= 0 && cc_val <= 15) ? cc_val : -1;
  }
  if (opts->frame_nxdn48 == 1 || opts->frame_nxdn96 == 1)
  {
    opts->nxdn_ran_filter = (cc_val >= 0 && cc_val <= 63) ? cc_val : -1;
  }
  if (opts->frame_p25p1 == 1 || opts->frame_p25p2 == 1)
  {
    opts->p25_nac_filter = (nac_val >= 0 && nac_val <= 0xFFF) ? nac_val : -1;
    if (opts->frame_p25p2 == 1 && state->p2_hardset == 0 && nac_val >= 0 && nac_val <= 0xFFF)
    {
      state->p2_cc = (unsigned long long int)nac_val;
    }
  }
}

static int trunk_lookup_freq_access(dsd_opts * opts, dsd_state * state, long int freq)
{
  if (opts == NULL || state == NULL || g_trunk_has_access_codes == 0 || freq <= 0)
    return 0;

  for (int i = 0; i < state->lcn_freq_count && i < 26; i++)
  {
    if (state->trunk_lcn_freq[i] == freq)
    {
      trunk_apply_access_values(opts, state, g_trunk_lcn_cc[i], g_trunk_lcn_nac[i]);
      return 1;
    }
  }

  for (int ch = 0; ch < 0xFFFF; ch++)
  {
    if (state->trunk_chan_map[ch] == freq)
    {
      trunk_apply_access_values(opts, state, g_trunk_chan_cc[ch], g_trunk_chan_nac[ch]);
      return 1;
    }
  }
  return 0;
}

int trunk_is_imported_freq(dsd_state * state, long int freq)
{
  if (state == NULL || freq <= 0)
    return 0;
  for (int i = 0; i < state->lcn_freq_count && i < 26; i++)
  {
    if (state->trunk_lcn_freq[i] == freq)
      return 1;
  }
  for (int ch = 0; ch < 0xFFFF; ch++)
  {
    if (state->trunk_chan_map[ch] == freq)
      return 1;
  }
  return 0;
}

static long int trunk_first_imported_freq(dsd_state * state)
{
  if (state == NULL)
    return 0;
  for (int i = 0; i < state->lcn_freq_count && i < 26; i++)
  {
    if (state->trunk_lcn_freq[i] > 0)
      return state->trunk_lcn_freq[i];
  }
  for (int ch = 0; ch < 0xFFFF; ch++)
  {
    if (state->trunk_chan_map[ch] > 0)
      return state->trunk_chan_map[ch];
  }
  return 0;
}

void trunk_update_filter_for_freq(long int freq)
{
  ambe_hw_release_lock();
  int had_prev_freq = 0;
  int prev_cc = -1;
  if (freq > 0)
  {
    if (freq != g_trunk_cur_freq)
    {
      if (g_trunk_cur_freq != 0)
      {
        had_prev_freq = 1;
        if (g_trunk_state != NULL && g_trunk_state->dmr_color_code <= 15)
          prev_cc = (int)g_trunk_state->dmr_color_code;
        else if (g_trunk_opts != NULL && g_trunk_opts->dmr_cc_filter >= 0 && g_trunk_opts->dmr_cc_filter <= 15)
          prev_cc = g_trunk_opts->dmr_cc_filter;
      }
      g_t3_vc_lpcn = 0;
      g_cap_plus_cur_lsn = 0;
      if (g_trunk_state != NULL)
      {
        g_trunk_state->color_code_ok = 0;
        g_trunk_state->dmr_unconfirmed_sync = 0;
        g_trunk_state->dmr_color_code = 16;
        g_trunk_state->nxdn_last_ran = -1;
        g_trunk_state->dmr_cach_counter = 0;
        memset(g_trunk_state->dmr_cach_fragment, 1, sizeof(g_trunk_state->dmr_cach_fragment));
      }
    }
    g_trunk_cur_freq = freq;
    g_trunk_last_poll = time(NULL);
  }
  trunk_lookup_freq_access(g_trunk_opts, g_trunk_state, freq);
  if (had_prev_freq)
  {
    if (prev_cc >= 0 && (g_trunk_opts == NULL || g_trunk_opts->dmr_cc_filter < 0 || prev_cc != g_trunk_opts->dmr_cc_filter))
      g_trunk_prev_cc = prev_cc;
    else if (prev_cc < 0)
      g_trunk_prev_cc = -2;
    else
      g_trunk_prev_cc = -1;
    g_trunk_retune_bursts = 25;
    g_trunk_retune_time = time(NULL);
  }
}

int trunk_is_stale_retune_burst(int cc)
{
  if (g_trunk_retune_bursts <= 0)
    return 0;
  if ((time(NULL) - g_trunk_retune_time) > 2)
  {
    g_trunk_retune_bursts = 0;
    g_trunk_prev_cc = -1;
    return 0;
  }
  if (cc < 0 ||
      (g_trunk_prev_cc >= 0 && cc == g_trunk_prev_cc) ||
      (g_trunk_prev_cc == -2 && g_trunk_opts != NULL &&
       g_trunk_opts->dmr_cc_filter >= 0 && g_trunk_opts->dmr_cc_filter <= 15 &&
       cc != g_trunk_opts->dmr_cc_filter))
  {
    g_trunk_retune_bursts--;
    return 1;
  }
  g_trunk_retune_bursts = 0;
  g_trunk_prev_cc = -1;
  return 0;
}

void trunk_check_rigctl_freq(dsd_opts * opts, dsd_state * state)
{
  (void)trunk_get_current_freq(opts, state);
}

void trunk_init_access_code_lookup(dsd_opts * opts, dsd_state * state)
{
  trunk_ensure_access_arrays_inited();
  g_trunk_opts = opts;
  g_trunk_state = state;
  if (opts == NULL || state == NULL)
    return;

  long int init_freq = 0;
  if (opts->use_rigctl == 1 && opts->rigctl_sockfd != 0)
  {
    init_freq = GetCurrentFreq(opts->rigctl_sockfd);
  }
  else if (opts->audio_in_type == 3 && opts->rtlsdr_center_freq != 0)
  {
    init_freq = (long int)opts->rtlsdr_center_freq;
  }

  long int first_freq = trunk_first_imported_freq(state);
  if (first_freq > 0 && !trunk_is_imported_freq(state, init_freq))
  {
    if (opts->use_rigctl == 1 && opts->rigctl_sockfd != 0)
    {
      if (opts->setmod_bw != 0)
        SetModulation(opts->rigctl_sockfd, opts->setmod_bw);
      SetFreq(opts->rigctl_sockfd, first_freq);
      fprintf(stderr, "Tuning to Frequency: %.06lf MHz\n", (double)first_freq / 1000000);
      init_freq = first_freq;
      state->lcn_freq_roll = (state->lcn_freq_count > 1) ? 1 : 0;
      g_trunk_prev_cc = -2;
      g_trunk_retune_bursts = 25;
      g_trunk_retune_time = time(NULL);
    }
    else if (opts->audio_in_type == 3)
    {
      opts->rtlsdr_center_freq = (uint32_t)first_freq;
      #ifdef USE_RTLSDR
      if (opts->rtl_started == 1)
        rtl_dev_tune(opts, first_freq);
      #endif
      fprintf(stderr, "Tuning to Frequency: %.06lf MHz\n", (double)first_freq / 1000000);
      init_freq = first_freq;
      state->lcn_freq_roll = (state->lcn_freq_count > 1) ? 1 : 0;
      g_trunk_prev_cc = -2;
      g_trunk_retune_bursts = 25;
      g_trunk_retune_time = time(NULL);
    }
  }

  if (init_freq > 0)
  {
    g_trunk_cur_freq = init_freq;
    g_trunk_last_poll = time(NULL);
  }

  if (g_trunk_has_access_codes == 0)
    return;

  if (trunk_lookup_freq_access(opts, state, init_freq))
    return;

  // If no explicit CLI filter was set yet, initialize from the first imported channel in the CSV
  if (state->lcn_freq_count > 0 &&
      opts->dmr_cc_filter == -1 && opts->nxdn_ran_filter == -1 && opts->p25_nac_filter == -1)
  {
    trunk_apply_access_values(opts, state, g_trunk_lcn_cc[0], g_trunk_lcn_nac[0]);
  }
}

long int trunk_get_current_freq(dsd_opts * opts, dsd_state * state)
{
  if (opts != NULL && opts->p25_trunk == 1 && opts->p25_is_tuned == 1 && state != NULL && state->p25_vc_freq[0] != 0)
    return state->p25_vc_freq[0];
  if (opts != NULL && opts->use_rigctl == 1 && opts->rigctl_sockfd != 0 &&
      (opts->p25_is_tuned == 0 || opts->p25_trunk == 0 || (state != NULL && state->dmr_rest_channel != -1)) &&
      opts->scanner_mode == 0)
  {
    time_t now = time(NULL);
    if (g_trunk_cur_freq == 0 || (now - g_trunk_last_poll) >= 1)
    {
      g_trunk_last_poll = now;
      long int f = GetCurrentFreq(opts->rigctl_sockfd);
      if (f > 0 && f != g_trunk_cur_freq)
      {
        g_trunk_cur_freq = f;
        trunk_reset_dmr_channel_state();
        if (state != NULL)
        {
          state->dmr_rest_channel = -1;
          state->dmr_t3_syscode = 0;
          state->p25_cc_freq = 0;
          state->p25_vc_freq[0] = state->p25_vc_freq[1] = 0;
          state->dmr_branding[0] = 0;
          state->dmr_branding_sub[0] = 0;
          state->dmr_site_parms[0] = 0;
          state->dmr_is_ras = 0;
          state->color_code_ok = 0;
          state->dmr_unconfirmed_sync = 0;
          state->dmr_color_code = 16;
          state->nxdn_last_ran = -1;
        }
        trunk_lookup_freq_access(opts, state, f);
      }
    }
  }
  if (g_trunk_cur_freq > 0)
    return g_trunk_cur_freq;
  if (opts != NULL && opts->audio_in_type == 3 && opts->rtlsdr_center_freq != 0)
    return (long int)opts->rtlsdr_center_freq;
  if (state != NULL && state->dmr_rest_channel > 0 && state->dmr_rest_channel < 0xFFFF &&
      state->trunk_chan_map[state->dmr_rest_channel] != 0)
    return state->trunk_chan_map[state->dmr_rest_channel];
  if (state != NULL && state->p25_cc_freq != 0)
    return state->p25_cc_freq;
  return 0;
}

int trunk_get_t3_cc_lpcn(dsd_state * state)
{
  if (state == NULL)
    return 0;
  if (strncmp(state->dmr_site_parms, "TIII", 4) != 0 && state->dmr_t3_syscode == 0)
    return 0;
  if (g_t3_cc_lpcn > 0)
    return g_t3_cc_lpcn;
  if (state->p25_cc_freq > 0)
  {
    int fallback = 0;
    for (int ch = 1; ch < 0xFFF; ch++)
    {
      if (state->trunk_chan_map[ch] == state->p25_cc_freq)
      {
        if (ch != 999)
          return ch;
        fallback = ch;
      }
    }
    return fallback;
  }
  return 0;
}

int trunk_is_dmr_slot_blocked(dsd_opts * opts, dsd_state * state, int slot)
{
  if (opts == NULL || state == NULL || opts->p25_trunk == 0)
    return 0;

  uint16_t so = (slot == 0) ? state->dmr_so : state->dmr_soR;
  uint8_t alg = (slot == 0) ? state->payload_algid : state->payload_algidR;
  uint32_t tg = (slot == 0) ? (uint32_t)state->lasttg : (uint32_t)state->lasttgR;

  if (state->tg_hold != 0 && tg != 0 && tg != state->tg_hold)
    return 1;

  char mode[8];
  sprintf(mode, "%s", (opts->trunk_use_allow_list == 1) ? "B" : "");
  if (tg != 0)
  {
    for (unsigned int i = 0; i < state->group_tally; i++)
    {
      if (state->group_array[i].groupNumber == tg)
      {
        snprintf(mode, sizeof(mode), "%s", state->group_array[i].groupMode);
        break;
      }
    }
    if (state->tg_hold != 0 && tg == state->tg_hold)
      sprintf(mode, "%s", "A");
    if (strcmp(mode, "B") == 0 || strcmp(mode, "DE") == 0)
      return 1;
  }

  if (opts->trunk_tune_enc_calls == 0 &&
      (((so & 0x40) != 0 && so != 0x100) || (alg != 0 && alg != 0x80)))
  {
    if (strcmp(mode, "A") != 0)
      return 1;
  }

  return 0;
}

void trunk_get_dmr_channel_str(dsd_opts * opts, dsd_state * state, long int cur_freq, char * buf, size_t buf_sz)
{
  UNUSED(opts);
  if (buf == NULL || buf_sz == 0)
    return;
  buf[0] = '\0';
  if (state == NULL)
    return;
  int lls = state->lastsynctype;
  if (!(lls > 1 && lls < 30))
    return;

  int is_cap_plus = (strcmp(state->dmr_branding_sub, "Cap+ ") == 0 || state->dmr_rest_channel > 0);
  int is_xpt = (strcmp(state->dmr_branding_sub, "XPT ") == 0);
  int is_con_plus = (strcmp(state->dmr_branding_sub, "Con+ ") == 0);
  int is_t3 = (strncmp(state->dmr_site_parms, "TIII", 4) == 0 || state->dmr_t3_syscode != 0);

  if (is_cap_plus || is_xpt)
  {
    if (cur_freq > 0)
    {
      for (int ch = 1; ch <= 32; ch++)
      {
        if (state->trunk_chan_map[ch] == cur_freq)
        {
          int lsn1 = ((ch - 1) & ~1) + 1;
          snprintf(buf, buf_sz, "LSN: %d/%d", lsn1, lsn1 + 1);
          return;
        }
      }
    }
    int cur_lsn = g_cap_plus_cur_lsn;
    if (cur_lsn <= 0 && state->p25_vc_freq[0] == 0 && state->dmr_rest_channel > 0)
      cur_lsn = state->dmr_rest_channel;
    if (cur_lsn > 0)
    {
      int lsn1 = ((cur_lsn - 1) & ~1) + 1;
      snprintf(buf, buf_sz, "LSN: %d/%d", lsn1, lsn1 + 1);
    }
    return;
  }

  if (is_t3)
  {
    if (state->p25_vc_freq[0] != 0 && g_t3_vc_lpcn > 0)
    {
      snprintf(buf, buf_sz, "LPCN: %d", g_t3_vc_lpcn);
      return;
    }
    if (cur_freq > 0)
    {
      int match = 0;
      for (int ch = 1; ch < 0xFFF; ch++)
      {
        if (state->trunk_chan_map[ch] == cur_freq)
        {
          if (ch != 999) { match = ch; break; }
          match = ch;
        }
      }
      if (match > 0)
      {
        snprintf(buf, buf_sz, "LPCN: %d", match);
        return;
      }
    }
    if (state->p25_vc_freq[0] == 0 && g_t3_cc_lpcn > 0)
      snprintf(buf, buf_sz, "LPCN: %d", g_t3_cc_lpcn);
    else if (g_t3_vc_lpcn > 0)
      snprintf(buf, buf_sz, "LPCN: %d", g_t3_vc_lpcn);
    return;
  }

  if (is_con_plus)
  {
    if (cur_freq > 0)
    {
      for (int ch = 1; ch <= 32; ch++)
      {
        if (state->trunk_chan_map[ch] == cur_freq)
        {
          snprintf(buf, buf_sz, "LCN: %d", ch);
          return;
        }
      }
    }
    return;
  }

  if (cur_freq > 0)
  {
    for (int ch = 1; ch <= 31; ch += 2)
    {
      if (state->trunk_chan_map[ch] == cur_freq && state->trunk_chan_map[ch + 1] == cur_freq)
      {
        snprintf(buf, buf_sz, "LSN: %02d/%02d", ch, ch + 1);
        return;
      }
    }
    int match = 0;
    for (int ch = 1; ch < 0xFFF; ch++)
    {
      if (state->trunk_chan_map[ch] == cur_freq)
      {
        if (ch != 999) { match = ch; break; }
        match = ch;
      }
    }
    if (match > 0)
      snprintf(buf, buf_sz, "LPCN: %04d", match);
  }
}

int csvChanImport(dsd_opts * opts, dsd_state * state) //channel map import
{
  trunk_ensure_access_arrays_inited();
  trunk_reset_dmr_channel_state();
  if (state != NULL)
  {
    state->dmr_rest_channel = -1;
    state->p25_cc_freq = 0;
  }
  g_trunk_opts = opts;
  g_trunk_state = state;
  char filename[1024] = "filename.csv";
  sprintf (filename, "%s", opts->chan_in_file);

  char buffer[BSIZE];
  FILE * fp;
  fp = fopen(filename, "r");
  if (fp == NULL) {
    printf("Unable to open channel map file '%s'\n", filename);
    //have this return -1 and handle it inside of main
    exit(1);
  }
  int row_count = 0;
  int field_count = 0;

  long int chan_number = 0;

  while (fgets(buffer, BSIZE, fp)) {
    field_count = 0;
    row_count++;
    if (row_count == 1)
      continue; //don't want labels
    chan_number = -1;
    int lcn_idx = -1;
    int has_access_col = 0;
    char * cursor = buffer;
    char * field = NULL;
    while (cursor != NULL) {
      field = cursor;
      char * comma = strchr(cursor, ',');
      if (comma != NULL)
      {
        *comma = '\0';
        cursor = comma + 1;
      }
      else
      {
        cursor = NULL;
      }

      if (field_count == 0)
      {
        sscanf (field, "%ld", &chan_number);
        if (chan_number < 0 || chan_number >= 0xFFFF)
          chan_number = -1;
      }

      if (field_count == 1 && chan_number >= 0)
      {
        sscanf (field, "%ld", &state->trunk_chan_map[chan_number]);
        //adding this should be compatible with EDACS, test and obsolete the LCN Import function if desired
        if (state->lcn_freq_count < 26)
        {
          sscanf (field, "%ld", &state->trunk_lcn_freq[state->lcn_freq_count]);
          lcn_idx = state->lcn_freq_count;
          state->lcn_freq_count++; //keep tally of number of Frequencies imported
        }
      }

      if (field_count == 2 && chan_number >= 0)
      {
        char * p = field;
        while (*p == ' ' || *p == '\t')
          p++;
        if (*p != '\0' && *p != '\r' && *p != '\n')
        {
          char * end_dec = NULL;
          long cc_val = strtol(p, &end_dec, 10);
          while (end_dec && (*end_dec == ' ' || *end_dec == '\t' || *end_dec == '\r' || *end_dec == '\n'))
            end_dec++;
          int valid_dec = (end_dec != p && end_dec != NULL && *end_dec == '\0');

          if (valid_dec)
          {
            if (cc_val >= 0 && cc_val <= 63)
            {
              g_trunk_chan_cc[chan_number] = (int)cc_val;
              if (lcn_idx >= 0 && lcn_idx < 26)
                g_trunk_lcn_cc[lcn_idx] = (int)cc_val;
              g_trunk_has_access_codes = 1;
              has_access_col = 1;
            }
            else if (cc_val == -1)
            {
              g_trunk_chan_cc[chan_number] = -1;
              g_trunk_chan_nac[chan_number] = -1;
              if (lcn_idx >= 0 && lcn_idx < 26)
              {
                g_trunk_lcn_cc[lcn_idx] = -1;
                g_trunk_lcn_nac[lcn_idx] = -1;
              }
              g_trunk_has_access_codes = 1;
              has_access_col = 1;
            }
          }

          char * end_hex = NULL;
          long nac_val = strtol(p, &end_hex, 16);
          while (end_hex && (*end_hex == ' ' || *end_hex == '\t' || *end_hex == '\r' || *end_hex == '\n'))
            end_hex++;
          int valid_hex = (end_hex != p && end_hex != NULL && *end_hex == '\0');

          if (!(valid_dec && cc_val == -1) && valid_hex && nac_val >= 0 && nac_val <= 0xFFF)
          {
            g_trunk_chan_nac[chan_number] = (int)nac_val;
            if (lcn_idx >= 0 && lcn_idx < 26)
              g_trunk_lcn_nac[lcn_idx] = (int)nac_val;
            g_trunk_has_access_codes = 1;
            has_access_col = 1;
          }
        }
      }

      field_count++;
    }
    if (chan_number >= 0 && state->trunk_chan_map[chan_number] != 0)
    {
      fprintf (stderr, "Channel [%05ld] [%09ld]", chan_number, state->trunk_chan_map[chan_number]);
      if (has_access_col)
      {
        if (g_trunk_chan_cc[chan_number] >= 0)
          fprintf (stderr, " [CC/RAN: %02d | NAC: %03X]", g_trunk_chan_cc[chan_number],
                   g_trunk_chan_nac[chan_number] >= 0 ? g_trunk_chan_nac[chan_number] : 0);
        else if (g_trunk_chan_nac[chan_number] >= 0)
          fprintf (stderr, " [NAC: %03X]", g_trunk_chan_nac[chan_number]);
      }
      fprintf (stderr, "\n");
    }

  }
  fclose(fp);
  return 0;
}

//Decimal Variant of Key Import
int csvKeyImportDec(dsd_opts * opts, dsd_state * state) //multi-key support
{
  char filename[1024] = "filename.csv";
  sprintf (filename, "%s", opts->key_in_file);

  char buffer[BSIZE];
  FILE * fp;
  fp = fopen(filename, "r");
  if (fp == NULL) {
    printf("Unable to open file '%s'\n", filename);
    exit(1);
  }
  int row_count = 0;
  int field_count = 0;

  unsigned long long int keynumber = 0;
  unsigned long long int keyvalue = 0;

  uint16_t hash = 0;
  uint8_t hash_bits[24];
  memset (hash_bits, 0, sizeof(hash_bits));

  while (fgets(buffer, BSIZE, fp)) {
    field_count = 0;
    row_count++;
    if (row_count == 1)
      continue; //don't want labels
    char * cursor = buffer;
    char * field = NULL;
    while (cursor != NULL) {
      field = cursor;
      char * comma = strchr(cursor, ',');
      if (comma != NULL)
      {
        *comma = '\0';
        cursor = comma + 1;
      }
      else
      {
        cursor = NULL;
      }

      if (field_count == 0)
      {
        sscanf (field, "%lld", &keynumber);
        if (keynumber > 0xFFFF) //if larger than 16-bits, get its hash instead
        {
          keynumber = keynumber & 0xFFFFFF; //truncate to 24-bits (max allowed)
          for (int i = 0; i < 24; i++)
          {
            hash_bits[i] = ((keynumber << i) & 0x800000) >> 23; //load into array for CRC16
          }
          hash = ComputeCrcCCITT16d (hash_bits, 24);
          keynumber = hash & 0xFFFF; //make sure its no larger than 16-bits
          fprintf (stderr, "Hashed ");
        }

      }

      if (field_count == 1)
      {
        sscanf (field, "%lld", &keyvalue);
        state->rkey_array[keynumber] = keyvalue & 0xFFFFFFFFFF; //doesn't exceed 40-bit value
      }

      field_count++;
    }
    fprintf (stderr, "Key [%03lld] [%05lld]", keynumber, state->rkey_array[keynumber]);
    fprintf (stderr, "\n");
    hash = 0;

  }
  fclose(fp);
  return 0;
}

//Hex Variant of Key Import
int csvKeyImportHex(dsd_opts * opts, dsd_state * state) //key import for hex keys
{
  char filename[1024] = "filename.csv";
  sprintf (filename, "%s", opts->key_in_file);
  char buffer[BSIZE];
  FILE * fp;
  fp = fopen(filename, "r");
  if (fp == NULL) {
    printf("Unable to open file '%s'\n", filename);
    exit(1);
  }
  int row_count = 0;
  int field_count = 0;
  unsigned long long int keynumber;

  while (fgets(buffer, BSIZE, fp)) {
    field_count = 0;
    row_count++;
    if (row_count == 1)
      continue; //don't want labels
    char * cursor = buffer;
    char * field = NULL;
    while (cursor != NULL) {
      field = cursor;
      char * comma = strchr(cursor, ',');
      if (comma != NULL)
      {
        *comma = '\0';
        cursor = comma + 1;
      }
      else
      {
        cursor = NULL;
      }

      if (field_count == 0)
      {
        sscanf (field, "%llX", &keynumber);
      }

      if (field_count == 1)
      {
        sscanf (field, "%llX", &state->rkey_array[keynumber]);
      }

      //this could also theoretically nuke other keys that are at the same offset
      if (field_count == 2)
      {
        sscanf (field, "%llX", &state->rkey_array[keynumber+0x101]);
      }

      if (field_count == 3)
      {
        sscanf (field, "%llX", &state->rkey_array[keynumber+0x201]);
      }

      if (field_count == 4)
      {
        sscanf (field, "%llX", &state->rkey_array[keynumber+0x301]);
      }

      field_count++;
    }

    fprintf (stderr, "Key [%04llX] [%016llX]", keynumber, state->rkey_array[keynumber]);

    //if longer key is loaded (or clash with the 0x101, 0x201, 0x301 offset, then print the full key listing)
    if ( (state->rkey_array[keynumber+0x101] != 0) || (state->rkey_array[keynumber+0x201] != 0) || (state->rkey_array[keynumber+0x301] != 0) )
      fprintf (stderr, " [%016llX] [%016llX] [%016llX]", state->rkey_array[keynumber+0x101], state->rkey_array[keynumber+0x201], state->rkey_array[keynumber+0x301]);
    fprintf (stderr, "\n");

  }
  fclose(fp);
  return 0;
}
