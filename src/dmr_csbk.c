/*-------------------------------------------------------------------------------
 * dmr_csbk.c
 * DMR Control Signal Data PDU (CSBK, MBC) Handler and Related Functions
 *
 * Portions of Connect+ code reworked from Boatbod OP25
 * Source: https://github.com/boatbod/op25/blob/master/op25/gr-op25_repeater/lib/dmr_slot.cc
 *
 * Portions of Capacity+ code reworked from Eric Cottrell
 * Source: https://github.com/LinuxSheeple-E/dsd/blob/Feature/DMRECC/dmr_csbk.c
 *
 * LWVMOBILE
 * 2023-12 DSD-FME Florida Man Edition
 *-----------------------------------------------------------------------------*/

#include "dsd.h"
#define PCLEAR_TUNE_AWAY //disable if slower return is preferred
#define DMR_T3_ACTIVE_MAX     31
#define DMR_T3_ACTIVE_TIMEOUT 10

static uint32_t s_t3_active_tgt[DMR_T3_ACTIVE_MAX];
static uint32_t s_t3_active_src[DMR_T3_ACTIVE_MAX];
static time_t   s_t3_active_grant_time[DMR_T3_ACTIVE_MAX];
static uint8_t  s_t3_active_logged[DMR_T3_ACTIVE_MAX];
static uint8_t  s_t3_active_cm_seen[DMR_T3_ACTIVE_MAX];
static uint8_t  s_t3_active_cm_miss[DMR_T3_ACTIVE_MAX];
static char     s_t3_active_kind[DMR_T3_ACTIVE_MAX][12];
static int      s_capmax_cc_seen = 0;
static int      s_capmax_status_slot = -1;
static int      s_capmax_slot_aloha_cnt = 0;
static int      s_capmax_non_status_csbks = 0;
static uint16_t s_last_pclear_lpcn = 0;
static uint8_t  s_last_pclear_slot = 0;
static time_t   s_last_pclear_time = 0;
static uint16_t s_last_cm_tune_lpcn = 0;
static uint8_t  s_last_cm_tune_slot = 0;
static uint16_t s_last_cm_tune_tg   = 0;
static time_t   s_last_cm_tune_time = 0;
#define DMR_CAP_ACTIVE_MISS_LIMIT 6
#define DMR_CAP_PRIV_MIN_MS 1500
static uint8_t  s_cap_active_miss[17];
static uint8_t  s_cap_active_present = 0;
static uint8_t  s_cap_mbc_valid[2] = {0, 0};
static struct timespec s_cap_priv_start[17];
static uint16_t s_cap_priv_tg[17];
static uint8_t  s_cap_priv_active[17];
static uint8_t  s_cap_priv_miss[17];

static void
dmr_active_channel_reset_rows (dsd_state * state)
{
  if (state == NULL)
    return;
  memset (state->active_channel, 0, sizeof(state->active_channel));
  memset (state->dmr_t3_active_chan, 0, sizeof(state->dmr_t3_active_chan));
  memset (state->dmr_t3_active_slot, 0, sizeof(state->dmr_t3_active_slot));
  memset (state->dmr_t3_active_time, 0, sizeof(state->dmr_t3_active_time));
  memset (s_t3_active_tgt, 0, sizeof(s_t3_active_tgt));
  memset (s_t3_active_src, 0, sizeof(s_t3_active_src));
  memset (s_t3_active_grant_time, 0, sizeof(s_t3_active_grant_time));
  memset (s_t3_active_logged, 0, sizeof(s_t3_active_logged));
  memset (s_t3_active_cm_seen, 0, sizeof(s_t3_active_cm_seen));
  memset (s_t3_active_cm_miss, 0, sizeof(s_t3_active_cm_miss));
  memset (s_t3_active_kind, 0, sizeof(s_t3_active_kind));
  s_capmax_slot_aloha_cnt = 0;
  s_capmax_non_status_csbks = 0;
  memset (s_cap_active_miss, 0, sizeof(s_cap_active_miss));
  s_cap_active_present = 0;
  memset (s_cap_priv_start, 0, sizeof(s_cap_priv_start));
  memset (s_cap_priv_tg, 0, sizeof(s_cap_priv_tg));
  memset (s_cap_priv_active, 0, sizeof(s_cap_priv_active));
  memset (s_cap_priv_miss, 0, sizeof(s_cap_priv_miss));
}

void
dmr_active_channel_clear (dsd_state * state)
{
  if (state == NULL)
    return;
  int cur_lpcn = trunk_get_t3_vc_lpcn ();
  if (cur_lpcn > 0)
  {
    s_last_pclear_lpcn = (uint16_t)cur_lpcn;
    s_last_pclear_slot = 2;
    s_last_pclear_time = time(NULL);
    trunk_set_t3_vc_lpcn (0);
  }
  if (state->dmr_t3_syscode == 0)
  {
    s_capmax_cc_seen = 0;
    s_capmax_status_slot = -1;
  }
  dmr_active_channel_reset_rows (state);
}

static int
dmr_t3_active_has_entries (const dsd_state * state)
{
  for (int i = 0; i < DMR_T3_ACTIVE_MAX; i++)
  {
    if (state->dmr_t3_active_chan[i] != 0)
      return 1;
  }
  return 0;
}

static void
dmr_t3_active_clear_if_present (dsd_state * state)
{
  if (dmr_t3_active_has_entries (state))
    dmr_active_channel_reset_rows (state);
}

static void
dmr_t3_active_clear_entry (dsd_state * state, int idx)
{
  state->active_channel[idx][0] = '\0';
  state->dmr_t3_active_chan[idx] = 0;
  state->dmr_t3_active_slot[idx] = 0;
  state->dmr_t3_active_time[idx] = 0;
  s_t3_active_tgt[idx] = 0;
  s_t3_active_src[idx] = 0;
  s_t3_active_grant_time[idx] = 0;
  s_t3_active_logged[idx] = 0;
  s_t3_active_cm_seen[idx] = 0;
  s_t3_active_cm_miss[idx] = 0;
  s_t3_active_kind[idx][0] = '\0';
}

static void
dmr_t3_active_move_entry (dsd_state * state, int dst, int src)
{
  memcpy (state->active_channel[dst], state->active_channel[src], sizeof(state->active_channel[dst]));
  state->dmr_t3_active_chan[dst] = state->dmr_t3_active_chan[src];
  state->dmr_t3_active_slot[dst] = state->dmr_t3_active_slot[src];
  state->dmr_t3_active_time[dst] = state->dmr_t3_active_time[src];
  s_t3_active_tgt[dst] = s_t3_active_tgt[src];
  s_t3_active_src[dst] = s_t3_active_src[src];
  s_t3_active_grant_time[dst] = s_t3_active_grant_time[src];
  s_t3_active_logged[dst] = s_t3_active_logged[src];
  s_t3_active_cm_seen[dst] = s_t3_active_cm_seen[src];
  s_t3_active_cm_miss[dst] = s_t3_active_cm_miss[src];
  memcpy (s_t3_active_kind[dst], s_t3_active_kind[src], sizeof(s_t3_active_kind[dst]));
}

static void
dmr_t3_active_write_entry (dsd_state * state, int idx, uint16_t key_chan, uint16_t disp_chan,
                           uint8_t slot, const char * ch_kind, uint32_t target, uint32_t source,
                           time_t now, int is_grant, uint8_t logged)
{
  snprintf (state->active_channel[idx], sizeof(state->active_channel[idx]),
            "Active Ch: %4u TS%u %-9s TGT: %u",
            disp_chan, slot + 1, ch_kind, target);
  state->dmr_t3_active_chan[idx] = key_chan;
  state->dmr_t3_active_slot[idx] = slot;
  state->dmr_t3_active_time[idx] = now;
  s_t3_active_tgt[idx] = target;
  s_t3_active_src[idx] = source;
  if (is_grant)
    s_t3_active_grant_time[idx] = now;
  else
  {
    s_t3_active_cm_seen[idx] = 1;
    s_t3_active_cm_miss[idx] = 0;
  }
  s_t3_active_logged[idx] = logged;
  snprintf (s_t3_active_kind[idx], sizeof(s_t3_active_kind[idx]), "%s", ch_kind);
}

// Prune expired active_channel entries. Tier III entries expire independently per
// (channel, slot) after DMR_T3_ACTIVE_TIMEOUT seconds when not tuned to that voice
// channel; other DMR systems retain the 3-second global timeout.
void
dmr_t3_active_expire (const dsd_opts * opts, dsd_state * state, time_t now)
{
  int has_t3 = 0;
  for (int i = 0; i < DMR_T3_ACTIVE_MAX; i++)
  {
    if (state->dmr_t3_active_chan[i] != 0)
    {
      if (state->active_channel[i][0] == '\0')
        dmr_t3_active_clear_entry (state, i);
      else
        has_t3 = 1;
    }
  }

  int is_t3 = (has_t3 || state->dmr_t3_syscode != 0 || strncmp (state->dmr_site_parms, "TIII", 4) == 0);
  int global_stale = (((now - state->last_active_time) > 3) && ((now - state->last_vc_sync_time) > 3));

  if (!is_t3)
  {
    if (global_stale)
    {
      dmr_active_channel_clear (state);
      state->dmr_neighbor_count = 0;
    }
    return;
  }

  int on_vc = ((now - state->last_vc_sync_time) <= 2);
  if ((opts != NULL && opts->p25_is_tuned == 1) || (on_vc && (now - state->last_active_time) > 1))
  {
    dmr_active_channel_reset_rows (state);
    return;
  }

  for (int i = 0; i < DMR_T3_ACTIVE_MAX; i++)
  {
    if (state->dmr_t3_active_chan[i] == 0)
    {
      if (global_stale && state->active_channel[i][0] != '\0')
        state->active_channel[i][0] = '\0';
      continue;
    }
    if ((now - state->dmr_t3_active_time[i]) > DMR_T3_ACTIVE_TIMEOUT)
      dmr_t3_active_clear_entry (state, i);
  }
}

// Key Tier III active_channel rows by (channel, slot) in sorted order so calls on different RF channels do not overwrite each other.
static void
dmr_t3_active_set (dsd_state * state, uint16_t disp_chan, uint8_t slot,
                   const char * ch_kind, uint32_t target, uint32_t source, time_t now, int is_grant)
{
  uint16_t key_chan = (disp_chan != 0) ? disp_chan : 0xFFF;
  slot &= 1;

  if (is_grant)
  {
    if (disp_chan == s_last_pclear_lpcn && (s_last_pclear_slot == slot || s_last_pclear_slot == 2))
      s_last_pclear_time = 0;
    if (disp_chan == s_last_cm_tune_lpcn && s_last_cm_tune_slot == slot)
      s_last_cm_tune_time = 0;
  }

  for (int i = 0; i < DMR_T3_ACTIVE_MAX; i++)
  {
    if (state->dmr_t3_active_chan[i] == key_chan && state->dmr_t3_active_slot[i] == slot)
    {
      uint8_t logged = 0;
      if (s_t3_active_tgt[i] == target && (now - state->dmr_t3_active_time[i]) <= 3)
        logged = s_t3_active_logged[i];
      else
      {
        s_t3_active_cm_seen[i] = 0;
        s_t3_active_cm_miss[i] = 0;
      }
      if (!is_grant && s_t3_active_tgt[i] == target)
      {
        source = s_t3_active_src[i];
        if (s_t3_active_kind[i][0] != '\0')
          ch_kind = s_t3_active_kind[i];
      }
      dmr_t3_active_write_entry (state, i, key_chan, disp_chan, slot, ch_kind, target, source, now, is_grant, logged);
      return;
    }
  }

  int n = 0;
  for (int i = 0; i < DMR_T3_ACTIVE_MAX; i++)
  {
    if (state->dmr_t3_active_chan[i] != 0)
    {
      if (n != i)
      {
        dmr_t3_active_move_entry (state, n, i);
        dmr_t3_active_clear_entry (state, i);
      }
      n++;
    }
    else if (state->active_channel[i][0] != '\0')
    {
      state->active_channel[i][0] = '\0';
    }
  }

  if (n == DMR_T3_ACTIVE_MAX)
  {
    int oldest = 0;
    for (int i = 1; i < n; i++)
    {
      if (state->dmr_t3_active_time[i] < state->dmr_t3_active_time[oldest])
        oldest = i;
    }
    for (int i = oldest; i < n - 1; i++)
      dmr_t3_active_move_entry (state, i, i + 1);
    n--;
    dmr_t3_active_clear_entry (state, n);
  }

  int ins = 0;
  while (ins < n &&
         (state->dmr_t3_active_chan[ins] < key_chan ||
          (state->dmr_t3_active_chan[ins] == key_chan && state->dmr_t3_active_slot[ins] < slot)))
  {
    ins++;
  }

  for (int i = n; i > ins; i--)
    dmr_t3_active_move_entry (state, i, i - 1);

  dmr_t3_active_clear_entry (state, ins);
  dmr_t3_active_write_entry (state, ins, key_chan, disp_chan, slot, ch_kind, target, source, now, is_grant, 0);
}

static void
dmr_t3_active_clear_slot_lpcn (dsd_state * state, uint16_t disp_chan, uint8_t slot, time_t now)
{
  uint16_t key_chan = (disp_chan != 0) ? disp_chan : 0xFFF;
  slot &= 1;
  for (int i = 0; i < DMR_T3_ACTIVE_MAX; i++)
  {
    if (state->dmr_t3_active_chan[i] == key_chan && state->dmr_t3_active_slot[i] == slot)
    {
      if (s_t3_active_cm_seen[i] || (now - s_t3_active_grant_time[i]) > 1)
        dmr_t3_active_clear_entry (state, i);
    }
  }
}

static uint32_t
dmr_t3_active_get_src (const dsd_state * state, uint16_t disp_chan, uint8_t slot, uint32_t target, time_t now)
{
  uint16_t key_chan = (disp_chan != 0) ? disp_chan : 0xFFF;
  slot &= 1;
  for (int i = 0; i < DMR_T3_ACTIVE_MAX; i++)
  {
    if (state->dmr_t3_active_chan[i] == key_chan &&
        state->dmr_t3_active_slot[i] == slot &&
        s_t3_active_tgt[i] == target &&
        (now - s_t3_active_grant_time[i]) <= 2)
      return s_t3_active_src[i];
  }
  return 0;
}

static int
dmr_t3_active_is_logged (const dsd_state * state, uint16_t disp_chan, uint8_t slot, uint32_t target)
{
  uint16_t key_chan = (disp_chan != 0) ? disp_chan : 0xFFF;
  slot &= 1;
  for (int i = 0; i < DMR_T3_ACTIVE_MAX; i++)
  {
    if (state->dmr_t3_active_chan[i] == key_chan &&
        state->dmr_t3_active_slot[i] == slot &&
        s_t3_active_tgt[i] == target)
      return s_t3_active_logged[i];
  }
  return 0;
}

static void
dmr_t3_active_mark_logged (const dsd_state * state, uint16_t disp_chan, uint8_t slot, uint32_t target)
{
  uint16_t key_chan = (disp_chan != 0) ? disp_chan : 0xFFF;
  slot &= 1;
  for (int i = 0; i < DMR_T3_ACTIVE_MAX; i++)
  {
    if (state->dmr_t3_active_chan[i] == key_chan &&
        state->dmr_t3_active_slot[i] == slot &&
        s_t3_active_tgt[i] == target)
    {
      s_t3_active_logged[i] = 1;
      return;
    }
  }
}

static int
dmr_log_t3_grant_event (dsd_opts * opts, dsd_state * state, uint8_t lcn, uint16_t disp_chan,
                        int csbk_o, int st2, int is_btv, uint32_t target, time_t now)
{
  if (opts == NULL || state == NULL || target == 0)
    return 0;

  uint8_t slot = lcn & 1;
  int slot_busy = (slot == 0) ? (state->lastsrc != 0 || state->lasttg != 0)
                              : (state->lastsrcR != 0 || state->lasttgR != 0);
  if (slot_busy)
  {
    uint8_t cc_slot = (uint8_t)(state->currentslot & 1);
    int cc_busy = (cc_slot == 0) ? (state->lastsrc != 0 || state->lasttg != 0)
                                 : (state->lastsrcR != 0 || state->lasttgR != 0);
    if (cc_busy)
      return 0;
    slot = cc_slot;
  }

  for (int s = 0; s < 2; s++)
  {
    const Event_History * eh1 = &state->event_history_s[s].Event_History_Items[1];
    if (eh1->target_id == target &&
        eh1->event_time != 0 && (now - eh1->event_time) <= 2)
      return 1;
  }

  watchdog_event_history (opts, state, slot);

  int8_t prev_gi = state->gi[slot];
  uint8_t prev_ht = state->dmr_hangtime[slot];
  state->gi[slot] = (csbk_o == 49 || csbk_o == 50) ? 0 : 1;
  state->dmr_hangtime[slot] = 1;
  if (slot == 0)
  {
    unsigned int prev_so = state->dmr_so;
    unsigned int prev_fid = state->dmr_fid;
    unsigned int prev_alg = state->payload_algid;
    unsigned int prev_kid = state->payload_keyid;
    unsigned long long int prev_mi = state->payload_mi;
    state->lasttg = target;
    state->lastsrc = DMR_CC_GRANT_SRC;
    state->dmr_fid = 0;
    state->dmr_so = (is_btv ? 0x08 : 0) | (st2 ? 0x80 : 0);
    state->payload_algid = 0;
    state->payload_keyid = 0;
    state->payload_mi = 0;
    watchdog_event_current (opts, state, 0);
    state->event_history_s[0].Event_History_Items[0].channel = disp_chan;
    state->lasttg = 0;
    state->lastsrc = 0;
    state->dmr_so = prev_so;
    state->dmr_fid = prev_fid;
    state->payload_algid = prev_alg;
    state->payload_keyid = prev_kid;
    state->payload_mi = prev_mi;
  }
  else
  {
    unsigned int prev_so = state->dmr_soR;
    unsigned int prev_fid = state->dmr_fidR;
    unsigned int prev_alg = state->payload_algidR;
    unsigned int prev_kid = state->payload_keyidR;
    unsigned long long int prev_mi = state->payload_miR;
    state->lasttgR = target;
    state->lastsrcR = DMR_CC_GRANT_SRC;
    state->dmr_fidR = 0;
    state->dmr_soR = (is_btv ? 0x08 : 0) | (st2 ? 0x80 : 0);
    state->payload_algidR = 0;
    state->payload_keyidR = 0;
    state->payload_miR = 0;
    watchdog_event_current (opts, state, 1);
    state->event_history_s[1].Event_History_Items[0].channel = disp_chan;
    state->lasttgR = 0;
    state->lastsrcR = 0;
    state->dmr_soR = prev_so;
    state->dmr_fidR = prev_fid;
    state->payload_algidR = prev_alg;
    state->payload_keyidR = prev_kid;
    state->payload_miR = prev_mi;
  }
  state->dmr_hangtime[slot] = prev_ht;
  state->gi[slot] = prev_gi;

  watchdog_event_history (opts, state, slot);
  return 1;
}

static void
dmr_csbk_clear_active_slot (dsd_state * state)
{
  uint8_t slot = (uint8_t)(state->currentslot & 1);
  int cur_lpcn = trunk_get_t3_vc_lpcn ();
  if (cur_lpcn > 0)
  {
    s_last_pclear_lpcn = (uint16_t)cur_lpcn;
    s_last_pclear_slot = slot;
    s_last_pclear_time = time(NULL);
  }
  if (!dmr_t3_active_has_entries (state))
  {
    state->active_channel[slot][0] = '\0';
    return;
  }

  for (int i = 0; i < DMR_T3_ACTIVE_MAX; i++)
  {
    if (state->dmr_t3_active_chan[i] != 0 &&
        state->dmr_t3_active_slot[i] == slot &&
        (cur_lpcn == 0 || state->dmr_t3_active_chan[i] == (uint16_t)cur_lpcn))
      dmr_t3_active_clear_entry (state, i);
  }
}

// Format Tier III voice grant call_string with consistent column alignment.
static void
dmr_csbk_set_call_string (dsd_state * state, uint8_t slot, int csbk_o, int st2, int is_btv, int is_dx)
{
  const char * kind = (csbk_o == 49) ? "Group" : "Private";
  const char * mod = st2 ? "Emergency" : (is_btv ? "Broadcast" : (is_dx ? "Duplex" : ""));
  dsd_set_call_string (state, slot, kind, mod);
}

// Initialize per-slot call metadata when tuning onto a Tier III voice grant.
static void
dmr_t3_grant_init_slot (dsd_state * state, uint8_t lcn, int data_call, int csbk_o,
                        int st2, int is_btv, int is_dx, uint32_t target, uint32_t source)
{
  if (data_call != 0)
    return;
  if (lcn == 0)
  {
    state->lasttg = target;
    state->lastsrc = source;
    state->gi[0] = (csbk_o == 49) ? 0 : 1;
    state->dmr_bcast_tg[0] = is_btv ? target : 0;
    state->dmr_hangtime[0] = 0;
    state->dmr_fid = 0;
    state->dmr_so = (is_btv ? 0x08 : 0) | (st2 ? 0x80 : 0);
    dmr_csbk_set_call_string (state, 0, csbk_o, st2, is_btv, is_dx);
  }
  else if (lcn == 1)
  {
    state->lasttgR = target;
    state->lastsrcR = source;
    state->gi[1] = (csbk_o == 49) ? 0 : 1;
    state->dmr_bcast_tg[1] = is_btv ? target : 0;
    state->dmr_hangtime[1] = 0;
    state->dmr_fidR = 0;
    state->dmr_soR = (is_btv ? 0x08 : 0) | (st2 ? 0x80 : 0);
    dmr_csbk_set_call_string (state, 1, csbk_o, st2, is_btv, is_dx);
  }
}

// Clear stale per-slot call metadata when tuning onto a different Tier III RF channel.
static void
dmr_csbk_clear_both_slots (dsd_state * state)
{
  state->lasttg = 0;
  state->lastsrc = 0;
  state->lasttgR = 0;
  state->lastsrcR = 0;
  state->dmr_so = 0;
  state->dmr_soR = 0;
  state->dmr_fid = 0;
  state->dmr_fidR = 0;
  state->dmr_bcast_tg[0] = 0;
  state->dmr_bcast_tg[1] = 0;
  state->dmr_hangtime[0] = 0;
  state->dmr_hangtime[1] = 0;
  state->dmrburstL = 9;
  state->dmrburstR = 9;
  sprintf (state->call_string[0], "%s", "                     ");
  sprintf (state->call_string[1], "%s", "                     ");
  dmr_talker_alias_reset (state, 0);
  dmr_talker_alias_reset (state, 1);
}

// Print 7-bit DMR Service Options flags to stderr and optionally append to out_buf.
static void
dmr_print_svc_opt_flags (uint8_t svc_opt, char * out_buf, size_t out_sz)
{
  const char * svc_flags[6] = {NULL};
  int nf = 0;
  if (svc_opt & 0x40) svc_flags[nf++] = "Emergency; ";
  if (svc_opt & 0x20) svc_flags[nf++] = "Encrypted; ";
  if (svc_opt & 0x10) svc_flags[nf++] = "Supp Data; ";
  if (svc_opt & 0x08) svc_flags[nf++] = "Broadcast; ";
  if (svc_opt & 0x04) svc_flags[nf++] = "OVCM; ";
  if      ((svc_opt & 0x03) == 1) svc_flags[nf++] = "Priority 1; ";
  else if ((svc_opt & 0x03) == 2) svc_flags[nf++] = "Priority 2; ";
  else if ((svc_opt & 0x03) == 3) svc_flags[nf++] = "Priority 3; ";
  for (int i = 0; i < nf; i++)
  {
    fprintf (stderr, "%s", svc_flags[i]);
    if (out_buf != NULL && strlen (out_buf) + strlen (svc_flags[i]) < out_sz)
      strcat (out_buf, svc_flags[i]);
  }
}

// Format an ETSI TS 102 361-4 Table A.1 call timer token into seconds or "Inf".
static void
dmr_format_timer_val (char * buf, size_t sz, uint16_t val, uint16_t inf_val, int step_s)
{
  if (val == 0)
    snprintf (buf, sz, "0s");
  else if (val == inf_val)
    snprintf (buf, sz, "Inf");
  else
    snprintf (buf, sz, "%ds", (int)val * step_s);
}

// Map common ETSI TS 102 361-4 clause 7.2.7 Reason_Code values to short text.
static const char *
dmr_t3_ack_reason_str (uint8_t reason_code)
{
  switch (reason_code)
  {
    case 0x00: return "MS Not Supported";
    case 0x01: return "Line Not Supported";
    case 0x02: return "Stack Full";
    case 0x03: return "Equip Busy";
    case 0x04: return "Recipient Refused";
    case 0x05: return "Custom Refused";
    case 0x06: return "Duplex Not Supported";
    case 0x10: return "System Busy";
    case 0x11: return "System Not Ready";
    case 0x12: return "Call Cancelled";
    case 0x13: return "Reg Refused";
    case 0x14: return "Reg Denied";
    case 0x15: return "MS Not Registered";
    case 0x16: return "Called Unit Busy";
    case 0x17: return "Called Group Busy";
    case 0x18: return "Invalid CRC";
    case 0x1F: return "Refused";
    case 0x20: return "Not Supported/Refused";
    case 0x21: return "Permitted Only/Denied";
    case 0x25: return "MS Not Registered";
    case 0x27: return "Called Unit Busy";
    case 0x2A: return "Call Cancelled";
    case 0x40: return "Message Accepted";
    case 0x41: return "Callback";
    case 0x42: return "Status Accepted";
    case 0x44: return "Auth Response";
    case 0x60: return "Message Accepted";
    case 0x61: return "Callback";
    case 0x62: return "Reg Accepted";
    case 0x63: return "Ahoy";
    case 0x80: return "Queued for Resource";
    case 0x81: return "Queued for Busy Target";
    case 0xA0: return "Queued for Resource";
    case 0xA1: return "Queued for Busy Target";
    case 0xC0: return "Wait";
    case 0xE0: return "Wait";
    default:   return NULL;
  }
}

// Extract network model, net/site IDs, and default DMRLA bit split from a 14-bit TIII syscode.
void
dmr_parse_t3_syscode (uint16_t syscode, uint16_t * net_out, uint16_t * site_out,
                      uint16_t * def_n_out, char * model_str, size_t model_sz)
{
  static const uint8_t net_bits[4]  = {9, 7, 4, 2};
  static const uint8_t site_bits[4] = {3, 5, 8, 10};
  static const char * const model_names[4] = {"Tiny", "Small", "Large", "Huge"};

  uint8_t model = (uint8_t)((syscode >> 12) & 0x3);
  uint8_t sb = site_bits[model];
  *net_out   = (uint16_t)((syscode >> sb) & ((1U << net_bits[model]) - 1U));
  *site_out  = (uint16_t)(syscode & ((1U << sb) - 1U));
  *def_n_out = sb;
  if (model_str != NULL && model_sz > 0)
    snprintf (model_str, model_sz, "%s", model_names[model]);
}

// Pack 2-bit Tier III network model and 9-bit net ID into a non-zero net key.
static uint16_t
dmr_t3_net_key (uint16_t syscode, uint16_t net)
{
  uint8_t model = (uint8_t)((syscode >> 12) & 0x3);
  return (uint16_t)(0x8000U | ((uint16_t)model << 10) | (net & 0x03FFU));
}

// Test whether splitting seen site IDs at bit boundary cand_n yields contiguous Areas,
// contiguous global sub-site offsets, and zero internal holes within each Area.
static int
dmr_t3_split_is_contiguous (const uint16_t * sites, uint8_t count, uint16_t cand_n)
{
  uint16_t sub_mask = (uint16_t)((1U << cand_n) - 1U);
  uint16_t a_vals[64];
  uint16_t a_min_b[64];
  uint16_t a_max_b[64];
  int a_cnt[64];
  uint16_t b_vals[64];
  int uniq_a = 0, uniq_b = 0;
  uint16_t min_a = 0xFFFFU, max_a = 0;
  uint16_t min_b = 0xFFFFU, max_b = 0;

  for (uint8_t i = 0; i < count; i++)
  {
    uint16_t a = (uint16_t)(sites[i] >> cand_n);
    uint16_t b = (uint16_t)(sites[i] & sub_mask);
    if (a < min_a) min_a = a;
    if (a > max_a) max_a = a;
    if (b < min_b) min_b = b;
    if (b > max_b) max_b = b;

    int seen_b = 0;
    for (int j = 0; j < uniq_b; j++)
    {
      if (b_vals[j] == b)
      {
        seen_b = 1;
        break;
      }
    }
    if (!seen_b)
      b_vals[uniq_b++] = b;

    int found = -1;
    for (int j = 0; j < uniq_a; j++)
    {
      if (a_vals[j] == a)
      {
        found = j;
        break;
      }
    }
    if (found < 0)
    {
      a_vals[uniq_a] = a;
      a_min_b[uniq_a] = b;
      a_max_b[uniq_a] = b;
      a_cnt[uniq_a] = 1;
      uniq_a++;
    }
    else
    {
      a_cnt[found]++;
      if (b < a_min_b[found])
        a_min_b[found] = b;
      if (b > a_max_b[found])
        a_max_b[found] = b;
    }
  }

  if (uniq_a < 2 || min_a > 1 || min_b > 1)
    return 0;
  if ((int)max_a - (int)min_a + 1 != uniq_a)
    return 0;
  if ((int)max_b - (int)min_b + 1 != uniq_b)
    return 0;

  for (int j = 0; j < uniq_a; j++)
  {
    if ((int)a_max_b[j] - (int)a_min_b[j] + 1 != a_cnt[j])
      return 0;
  }
  return 1;
}

// Record an observed Tier III syscode for the active (model, net) and infer best DMRLA split n.
void
dmr_record_t3_syscode (dsd_state * state, uint16_t syscode, int is_neighbor)
{
  if (state == NULL || syscode == 0)
    return;

  uint16_t net = 0, site = 0, def_n = 1;
  dmr_parse_t3_syscode (syscode, &net, &site, &def_n, NULL, 0);
  uint16_t net_key = dmr_t3_net_key (syscode, net);

  if (!is_neighbor && state->dmr_t3_net_key != net_key)
  {
    state->dmr_t3_net_key = net_key;
    state->dmr_t3_seen_count = 0;
    state->dmr_t3_auto_n = (uint8_t)def_n;
  }
  else if (is_neighbor && state->dmr_t3_net_key != net_key)
  {
    return;
  }

  for (uint8_t i = 0; i < state->dmr_t3_seen_count; i++)
  {
    if (state->dmr_t3_seen_sites[i] == site)
      return;
  }
  if (state->dmr_t3_seen_count >= sizeof(state->dmr_t3_seen_sites) / sizeof(state->dmr_t3_seen_sites[0]))
    return;

  state->dmr_t3_seen_sites[state->dmr_t3_seen_count++] = site;

  if (state->dmr_t3_seen_count < 2)
  {
    state->dmr_t3_auto_n = (uint8_t)def_n;
    return;
  }

  uint8_t best_n = (uint8_t)def_n;
  for (uint16_t cand_n = 1; cand_n < def_n; cand_n++)
  {
    if (dmr_t3_split_is_contiguous (state->dmr_t3_seen_sites, state->dmr_t3_seen_count, cand_n))
      best_n = (uint8_t)cand_n;
  }
  state->dmr_t3_auto_n = best_n;
}

// Return the effective DMRLA split n for a Tier III syscode (manual override, CapMax=0, or auto-inferred).
uint16_t
dmr_effective_t3_n (const dsd_opts * opts, const dsd_state * state,
                    uint16_t syscode, uint16_t def_n, int is_capmax)
{
  if (opts != NULL && opts->dmr_dmrla_is_set == 1)
    return (opts->dmr_dmrla_n > def_n) ? def_n : opts->dmr_dmrla_n;
  if (is_capmax || (state != NULL && strcmp (state->dmr_branding, "Motorola") == 0))
    return 0;
  if (state != NULL && syscode != 0)
  {
    uint16_t net = 0, site = 0, parsed_def_n = 1;
    dmr_parse_t3_syscode (syscode, &net, &site, &parsed_def_n, NULL, 0);
    uint16_t net_key = dmr_t3_net_key (syscode, net);
    if (state->dmr_t3_net_key == net_key && state->dmr_t3_auto_n >= 1 && state->dmr_t3_auto_n <= def_n)
      return state->dmr_t3_auto_n;
  }
  return def_n;
}

// Insert a unique neighbor Tier III syscode or site ID in ascending order.
static void
dmr_add_neighbor (dsd_state * state, uint16_t syscode)
{
  if (state == NULL || syscode == 0 || (state->dmr_t3_syscode != 0 && syscode == state->dmr_t3_syscode))
    return;

  uint8_t max_nb = (uint8_t)(sizeof(state->dmr_neighbors) / sizeof(state->dmr_neighbors[0]));
  uint8_t ins = 0;
  while (ins < state->dmr_neighbor_count && state->dmr_neighbors[ins] < syscode)
    ins++;
  if (ins < state->dmr_neighbor_count && state->dmr_neighbors[ins] == syscode)
    return;
  if (state->dmr_neighbor_count >= max_nb)
    return;

  for (uint8_t i = state->dmr_neighbor_count; i > ins; i--)
    state->dmr_neighbors[i] = state->dmr_neighbors[i - 1];
  state->dmr_neighbors[ins] = syscode;
  state->dmr_neighbor_count++;
}

// Reformat state->dmr_site_parms from the stored Tier III syscode using current DMRLA settings.
void
dmr_refresh_t3_site_parms (dsd_opts * opts, dsd_state * state)
{
  if (opts == NULL || state == NULL)
    return;
  if (state->dmr_t3_syscode == 0 && strncmp (state->dmr_site_parms, "TIII", 4) != 0)
    return;

  uint16_t net = 0, site = 0, n = 1;
  dmr_parse_t3_syscode ((uint16_t)state->dmr_t3_syscode, &net, &site, &n, NULL, 0);
  n = dmr_effective_t3_n (opts, state, (uint16_t)state->dmr_t3_syscode, n, 0);

  uint16_t sub_mask = (n == 0) ? 0 : (uint16_t)((1U << n) - 1);
  if (opts->dmr_dmrla_is_set == 1)
  {
    if (n != 0)
      snprintf (state->dmr_site_parms, sizeof(state->dmr_site_parms),
                "TIII %d-%d.%d [%d]; ", net + 1, (site >> n) + 1, (site & sub_mask) + 1, n);
    else
      snprintf (state->dmr_site_parms, sizeof(state->dmr_site_parms),
                "TIII %d-%d [0]; ", net, site);
  }
  else
  {
    if (n != 0)
      snprintf (state->dmr_site_parms, sizeof(state->dmr_site_parms),
                "TIII %d-%d.%d; ", net + 1, (site >> n) + 1, (site & sub_mask) + 1);
    else
      snprintf (state->dmr_site_parms, sizeof(state->dmr_site_parms),
                "TIII %d-%d; ", net, site);
  }
}

//function for handling Control Signalling PDUs (CSBK, MBC) messages
void dmr_cspdu (dsd_opts * opts, dsd_state * state, uint8_t cs_pdu_bits[], uint8_t cs_pdu[], uint32_t CRCCorrect, uint32_t IrrecoverableErrors)
{

  int  csbk_lb   = 0;
  int  csbk_pf   = 0;
  int  csbk_o    = 0;
  int  csbk_fid  = 0;

  long int ccfreq = 0;

  csbk_lb  = ( (cs_pdu[0] & 0x80) >> 7 );
  csbk_pf  = ( (cs_pdu[0] & 0x40) >> 6 );
  csbk_o   =    cs_pdu[0] & 0x3F;
  csbk_fid =    cs_pdu[1]; //feature set id
  UNUSED(csbk_lb);

  if (IrrecoverableErrors != 0 && state->hota_key_loaded == 1)
  {
    fprintf (stderr, "\n%s", KRED);
    fprintf (stderr, " BPTC Irrecoverable Errors; Encrypted;");
    fprintf (stderr, "%s", KNRM);
    return;
  }
  else if (state->hota_enc_check)
  {
    fprintf (stderr, "\n%s", KRED);
    fprintf (stderr, " Encrypted Control Signalling Block(s)");
    fprintf (stderr, "%s", KNRM);
    return;
  }

  //check, regardless of CRC err
  if (IrrecoverableErrors == 0)
  {
    //Hytera XPT CSBK Check -- if bits 0 and 1 are used as lcss, gi, ts, then the pf bit may be set on
    if ( csbk_fid == 0x68 && (csbk_o == 0x0A || csbk_o == 0x0B) ) csbk_pf = 0;
    if (csbk_pf == 1) //check the protect flag, don't run if set
    {
      fprintf (stderr, "%s", KRED);
      fprintf (stderr, "\n Protected Control Signalling Block(s)");
      fprintf (stderr, "%s", KNRM);
    }
  }

  if(IrrecoverableErrors == 0 && CRCCorrect == 1)
  {
    time_t now = time(NULL);

    //clear stale Active Channel messages here
    dmr_t3_active_expire (opts, state, now);

    //update time to prevent random 'Control Channel Signal Lost' hopping
    //in the middle of voice call on current Control Channel (con+ and t3)
    state->last_cc_sync_time = now;

    if (csbk_pf == 0) //okay to run
    {
      if (csbk_fid < 253)
      {
        ambe_hw_reset_call(state->currentslot);
        if (csbk_o != 46 && csbk_o != 47)
          state->dmr_hangtime[state->currentslot & 1] = 0;
      }

      //set overarching manufacturer in use when non-standard feature id set is up
      if (csbk_fid != 0 && csbk_fid < 253) state->dmr_mfid = csbk_fid;

      fprintf (stderr, "%s", KYEL);

      //7.1.1.1.1 Channel Grant CSBK/MBC PDU
      if (csbk_o >= 48 && csbk_o <= 56 )
      {

        //maintain this to allow users to hardset the cc freq as map[0]; otherwise, set from rigctl or rtl freq at c_aloha_sys_parms
        // if (state->p25_cc_freq == 0 && state->trunk_chan_map[0] != 0) state->p25_cc_freq = state->trunk_chan_map[0];

        //initial line break
        fprintf (stderr, "\n");

        long int freq = 0;

        //all of these messages share the same format (thankfully)
        uint8_t is_btv = (csbk_o == 50) ? 1 : 0;
        uint8_t is_dx  = (csbk_o == 53 || csbk_o == 54) ? 1 : 0;
        if (csbk_o == 48) fprintf (stderr, " Private Voice Channel Grant (PV_GRANT)");
        if (csbk_o == 49) fprintf (stderr, " Talkgroup Voice Channel Grant (TV_GRANT)");
        if (csbk_o == 50) fprintf (stderr, " Broadcast Voice Channel Grant (BTV_GRANT)");  //listed as private in the appendix (error?)
        if (csbk_o == 51) fprintf (stderr, " Private Data Channel Grant: Single Item (PD_GRANT)");
        if (csbk_o == 52) fprintf (stderr, " Talkgroup Data Channel Grant: Single Item (TD_GRANT)");
        if (csbk_o == 53) fprintf (stderr, " Duplex Private Voice Channel Grant (PV_GRANT_DX)");
        if (csbk_o == 54) fprintf (stderr, " Duplex Private Data Channel Grant (PD_GRANT_DX)");
        if (csbk_o == 55) fprintf (stderr, " Private Data Channel Grant: Multi Item (PD_GRANT)");
        if (csbk_o == 56 && state->synctype != 33) //when not MS Data Sync (shares same OPcode as BS_DWN_ACT)
          fprintf (stderr, " Talkgroup Data Channel Grant: Multi Item (TD_GRANT)");

        //Logical Physical Channel Number
        uint16_t lpchannum = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 12);
        if (lpchannum == 0) fprintf (stderr, " - Invalid Channel"); //invalid channel, not sure why this would even be transmitted
        else if (lpchannum == 0xFFF) fprintf (stderr, " - Absolute"); //This is from an MBC, signalling an absolute and not a logical
        else fprintf (stderr, " - Logical");

        //dsdplus channel values
        uint16_t pluschannum = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 13); //lcn bit included
        pluschannum += 1; //add a one for good measure

        //LCN conveyed here is the tdma timeslot variety, and not the RF frequency variety
        uint8_t lcn = cs_pdu_bits[28];
        //the next three bits can have different meanings depending on which item above for context
        uint8_t st1 = cs_pdu_bits[29]; //late_entry, hi_rate, reserved(dx)
        uint8_t st2 = cs_pdu_bits[30]; //emergency
        uint8_t st3 = cs_pdu_bits[31]; //offset, call direction
        //target and source are always the same bits
        uint32_t target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[32], 24);
        uint32_t source = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);
        UNUSED(st3);

        //broadcast calls show no tgt/src info in ETSI manaual, but seem to correspond with same values in embedded link control
        // if (csbk_o == 50)
        // {
        //   //seeting to all MS units and Dispatch? May need to just set to 0 and see what emb pulls out of it
        //   target = 0xFFFFFF; //ALLMSI
        //   source = 0xFFFECB; //DISPATI
        // }

        //move mbc variables out of if statement
        uint8_t mbc_lb = 0; //
        uint8_t mbc_pf = 0;
        uint8_t mbc_csbko = 0;
        uint8_t mbc_res = 0;
        uint8_t mbc_cc = 0;
        uint8_t mbc_cdeftype = 0;
        uint8_t mbc_res2 = 0;
        unsigned long long int mbc_cdefparms = 0;
        uint16_t mbc_lpchannum = 0;
        uint16_t mbc_abs_tx_int = 0;
        uint16_t mbc_abs_tx_step = 0;
        uint16_t mbc_abs_rx_int = 0;
        uint16_t mbc_abs_rx_step = 0;
        UNUSED5(mbc_lb, mbc_pf, mbc_csbko, mbc_res, mbc_cc);
        UNUSED4(mbc_res2, mbc_cdefparms, mbc_abs_tx_int, mbc_abs_tx_step);

        fprintf (stderr, "\n");
        //rewrote this to make it clear which is the lpcn, timeslot (lcn), and the combination of both (DSDPlus style combined LSN) on channel numbering
        //the TS is displayed as a +1 since while the bit values are 0 or 1, the slot numbering in the manual specifically states TDMA channel (slot) 1 or TDMA channel (slot) 2
        fprintf (stderr, "  LPCN: %04d; TS: %d; LPCN+TS: %04d; Target: %08d - Source: %08d ", lpchannum, lcn+1, pluschannum, target, source);

        if (st2) fprintf (stderr, "Emergency; ");
        if (st1 && (csbk_o == 51 || csbk_o == 52 || csbk_o == 55 || csbk_o == 56)) fprintf (stderr, "Hi-Rate; ");

        //check for special gateway identifiers (probably just on broadcast?)
        dmr_gateway_identifier (source, target);

        if (lpchannum == 0xFFF) //This is from an MBC, signalling an absolute and not a logical
        {
          //7.1.1.1.2 Channel Grant Absolute Parameters CG_AP appended MBC PDU
          mbc_lb = cs_pdu_bits[96]; //
          mbc_pf = cs_pdu_bits[97];
          mbc_csbko = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[98], 6);
          mbc_res = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[104], 4);
          mbc_cc = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[108], 4);
          mbc_cdeftype = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[112], 4); //see 7.2.19.7 = 0 for channel parms, 1 through FFFF reserved
          mbc_res2 = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[116], 2);
          long long int mbc_cdefparms = (unsigned long long int)ConvertBitIntoBytes(&cs_pdu_bits[118], 58); //see 7.2.19.7.1

          //this is how you read the 58 parm bits according to the appendix 7.2.19.7.1
          if (mbc_cdeftype == 0) //if 0, then absolute channel parms
          {
            mbc_lpchannum = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[118], 12);
            mbc_abs_tx_int = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[130], 10);
            mbc_abs_tx_step = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[140], 13);
            mbc_abs_rx_int = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[153], 10);
            mbc_abs_rx_step = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[163], 13);

            //tx_int (Mhz) + (tx_step * 125) = tx_freq
            //rx_int (Mhz) + (rx_step * 125) = rx_freq
            fprintf (stderr, "\n");
            fprintf (stderr, "  RX APCN: %04d; RX INT: %d; RX STEP: %d;", mbc_lpchannum, mbc_abs_rx_int, mbc_abs_rx_step );
            //The Frequency we want to tune is the RX Frequency
            freq = (mbc_abs_rx_int * 1000000 ) + (mbc_abs_rx_step * 125);
          }
          else fprintf (stderr, "\n  MBC Channel Grant - Unknown Parms: %015llX", mbc_cdefparms); //for any reserved values
        }

        //print frequency from absolute
        if (freq != 0 && lpchannum == 0xFFF)
          fprintf (stderr, "\n  Frequency: %.6lf MHz", (double)freq/1000000);

        //run external channel map function on logical
        if (lpchannum != 0 && lpchannum != 0xFFF)
        {
          freq = state->trunk_chan_map[lpchannum];
          if (freq != 0)
            fprintf (stderr, "\n  Frequency: %.6lf MHz", (double)freq/1000000);
          else fprintf (stderr, "\n  Frequency Not Found in Channel Map;");
        }

        int orig_csbk_o = csbk_o;
        int is_voice_grant = (orig_csbk_o == 48 || orig_csbk_o == 49 || orig_csbk_o == 50 || orig_csbk_o == 53);
        uint16_t grant_chan = (lpchannum == 0xFFF) ? mbc_lpchannum : lpchannum;
        int did_tune = 0;
        int on_grant_freq = 0;

        //add active channel string to display
        if (lpchannum != 0 && !(csbk_o == 56 && state->synctype == 33))
        {
          const char * ch_kind = is_btv ? "Broadcast" :
                                 (csbk_o == 49) ? "Group" :
                                 (csbk_o == 51 || csbk_o == 52 || csbk_o == 54 || csbk_o == 55 || csbk_o == 56) ? "Data" :
                                 (csbk_o == 53) ? "Duplex" : "Private";
          dmr_t3_active_set (state, grant_chan, lcn, ch_kind, target, source, now, 1);
          s_capmax_slot_aloha_cnt = 0;
          s_capmax_non_status_csbks = 0;
        }
        //update last active channel time
        state->last_active_time = now;

        if (is_voice_grant)
        {
          long int cur_f = trunk_get_current_freq(opts, state);
          if (cur_f == 0 && opts->p25_is_tuned == 0)
            cur_f = state->p25_cc_freq;
          int same_cc_chan = (opts->p25_is_tuned == 0 && grant_chan != 0 &&
                              grant_chan == (uint16_t)trunk_get_t3_cc_lpcn(state));
          on_grant_freq = ((cur_f != 0 && freq == cur_f) || same_cc_chan);
          int slot_busy = (lcn == 0)
                            ? (state->dmrburstL == 16 || state->dmrburstL == 0 || state->dmrburstL == 1 ||
                               state->dmrburstL == 2 || state->dmr_hangtime[0] ||
                               state->lasttg != 0 || state->lastsrc != 0)
                            : (state->dmrburstR == 16 || state->dmrburstR == 0 || state->dmrburstR == 1 ||
                               state->dmrburstR == 2 || state->dmr_hangtime[1] ||
                               state->lasttgR != 0 || state->lastsrcR != 0);
          if (!slot_busy && on_grant_freq)
          {
            int grant_csbk = (csbk_o == 50) ? 49 : csbk_o;
            dmr_t3_grant_init_slot (state, lcn, 0, grant_csbk, st2, is_btv, is_dx, target, source);
          }
        }

        //Skip tuning group calls if group calls are disabled
        if (opts->trunk_tune_group_calls == 0 && csbk_o == 49) goto SKIPCALL; //TV_GRANT
        if (opts->trunk_tune_group_calls == 0 && csbk_o == 50) goto SKIPCALL; //BTV_GRANT
        if (csbk_o == 50) csbk_o = 49; //flip to normal group call for group tuning

        //Allow tuning of data calls if user wishes by flipping the csbk_o to a group voice call
        int data_call = 0;
        if (csbk_o == 51 || csbk_o == 52 || csbk_o == 54 || csbk_o == 55 || csbk_o == 56)
        {
          data_call = 1; //don't add data calls to call history until redone
          if (opts->trunk_tune_data_calls == 1) csbk_o = 49;
        }

        //Skip tuning private calls if private calls are disabled
        if (opts->trunk_tune_private_calls == 0 && csbk_o != 49) goto SKIPCALL;

        //if not a data channel grant (only tuning to voice channel grants)
        if (csbk_o == 48 || csbk_o == 49 || csbk_o == 50 || csbk_o == 53) //48, 49, 50 are voice grants, 51 and 52 are data grants, 53 Duplex Private Voice, 54 Duplex Private Data
        {

          //if tg hold is specified and matches target, allow for a call pre-emption by nullifying the last vc sync time
          if (state->tg_hold != 0 && state->tg_hold == target)
            state->last_vc_sync_time = 0;

          //TIII tuner fix if voice assignment occurs to the control channel itself,
          //then it may not want to resume tuning due to no framesync loss after call ends
          if ( (time(NULL) - state->last_vc_sync_time > 2) )
          {
            opts->p25_is_tuned = 0;
            //zero out vc frequencies
            state->p25_vc_freq[0] = 0;
            state->p25_vc_freq[1] = 0;
          }

          //shim in here for ncurses freq display when not trunking (playback, not live)
          if (opts->p25_trunk == 0 && freq != 0)
          {
            //just set to both for now, could go on tslot later
            state->p25_vc_freq[0] = freq;
            state->p25_vc_freq[1] = freq;
            trunk_set_t3_vc_lpcn((lpchannum == 0xFFF) ? mbc_lpchannum : lpchannum);
          }
          if (opts->p25_trunk == 0 && data_call == 0 && (time(NULL) - state->last_vc_sync_time > 2))
          {
            state->dmr_bcast_tg[lcn] = is_btv ? target : 0;
          }

          //don't tune if currently a vc on the control channel
          if ( (time(NULL) - state->last_vc_sync_time > 2) )
          {
            char mode[8]; //allow, block, digital, enc, etc
            sprintf (mode, "%s", "");

            //if we are using allow/whitelist mode, then write 'B' to mode for block
            //comparison below will look for an 'A' to write to mode if it is allowed
            if (opts->trunk_use_allow_list == 1) sprintf (mode, "%s", "B");

            for (int i = 0; i < state->group_tally; i++)
            {
              if (state->group_array[i].groupNumber == target)
              {
                fprintf (stderr, " [%s]", state->group_array[i].groupName);
                strcpy (mode, state->group_array[i].groupMode);
                break;
              }
            }

            //TG hold on DMR T3 Systems -- block non-matching target, allow matching target
            if (state->tg_hold != 0 && state->tg_hold != target) sprintf (mode, "%s", "B");
            if (state->tg_hold != 0 && state->tg_hold == target) sprintf (mode, "%s", "A");

            if (state->p25_cc_freq != 0 && opts->p25_trunk == 1 && (strcmp(mode, "B") != 0) && (strcmp(mode, "DE") != 0))
            {
              if (freq != 0) //if we have a valid frequency
              {
                //RIGCTL
                if (opts->use_rigctl == 1)
                {
                  //Guess I forgot to add this condition here
                  long int cur_freq = trunk_get_current_freq(opts, state);
                  if (cur_freq != freq)
                  {
                    dmr_reset_blocks (opts, state); //reset all block gathering since we are tuning away from current frequency
                    dmr_csbk_clear_both_slots (state);
                    if (opts->setmod_bw != 0 ) SetModulation(opts->rigctl_sockfd, opts->setmod_bw);
                    SetFreq(opts->rigctl_sockfd, freq);
                  }
                  else
                  {
                    trunk_update_filter_for_freq(freq);
                  }
                  //we will want to set these values here, some Tier 3 systems prefer P_Protects (Tait) over VLC and TLC headers
                  //and is faster than waiting on good embedded link control
                  dmr_t3_grant_init_slot (state, lcn, data_call, csbk_o, st2, is_btv, is_dx, target, source);
                  state->p25_vc_freq[0] = state->p25_vc_freq[1] = freq;
                  trunk_set_t3_vc_lpcn((lpchannum == 0xFFF) ? mbc_lpchannum : lpchannum);
                  opts->p25_is_tuned = 1; //set to 1 to set as currently tuned so we don't keep tuning nonstop
                  state->last_vc_sync_time = time(NULL);
                  state->last_t3_tune_time = time(NULL); //set here so a random p_clear on the opposite slot doesn't send us back to the CC
                  did_tune = 1;

                }

                //rtl
                else if (opts->audio_in_type == 3)
                {
                  #ifdef USE_RTLSDR
                  //Guess I forgot to add this condition here
                  uint32_t tempf = (uint32_t)freq;
                  if (opts->rtlsdr_center_freq != tempf)
                  {
                    dmr_reset_blocks (opts, state); //reset all block gathering since we are tuning away from current frequency
                    dmr_csbk_clear_both_slots (state);
                    rtl_dev_tune (opts, freq);
                  }
                  else
                    trunk_update_filter_for_freq(freq);
                  //we will want to set these values here, some Tier 3 systems prefer P_Protects (Tait) over VLC and TLC headers
                  //and is faster than waiting on good embedded link control
                  dmr_t3_grant_init_slot (state, lcn, data_call, csbk_o, st2, is_btv, is_dx, target, source);
                  state->p25_vc_freq[0] = state->p25_vc_freq[1] = freq;
                  trunk_set_t3_vc_lpcn((lpchannum == 0xFFF) ? mbc_lpchannum : lpchannum);
                  opts->p25_is_tuned = 1;
                  state->last_vc_sync_time = time(NULL);
                  state->last_t3_tune_time = time(NULL); //set here so a random p_clear on the opposite slot doesn't send us back to the CC
                  did_tune = 1;

                  #endif
                }

              }
            }
          }

        }

        SKIPCALL: ;
        if (did_tune)
        {
          dmr_active_channel_reset_rows (state);
        }
        else if (is_voice_grant && lpchannum != 0 && target != 0)
        {
          if (on_grant_freq)
          {
            dmr_t3_active_mark_logged (state, grant_chan, lcn, target);
          }
          else if (!dmr_t3_active_is_logged (state, grant_chan, lcn, target))
          {
            if (dmr_log_t3_grant_event (opts, state, lcn, grant_chan, orig_csbk_o, st2, is_btv, target, now))
              dmr_t3_active_mark_logged (state, grant_chan, lcn, target);
          }
        }

      }

      //Move
      if (csbk_o == 57)
      {
        //initial line break
        fprintf (stderr, "\n");
        fprintf (stderr, " Move (C_MOVE) - ");
        uint8_t  move_mask    = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[25], 5);
        uint8_t  move_reg     = cs_pdu_bits[35];
        uint8_t  move_backoff = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[36], 4);
        uint16_t move_lpcn    = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[44], 12);
        uint32_t move_ms      = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);
        fprintf (stderr, "TSCC LPCN: %04d; MS: %08d; Reg: %d; Mask: %02X; Backoff: %d; ",
                 move_lpcn, move_ms, move_reg, move_mask, move_backoff);
        dmr_gateway_identifier (0, move_ms);
      }

      //Aloha
      if (csbk_o == 25)
      {
        //initial line break
        fprintf (stderr, "\n");

        dmr_decode_syscode(opts, state, cs_pdu_bits, csbk_fid, 0);

        if (csbk_fid == 0x10)
        {
          s_capmax_cc_seen = 1;
          if (s_capmax_status_slot < 0)
            s_capmax_status_slot = (int)((state->currentslot & 1) ^ 1);
        }
        if (s_capmax_cc_seen && opts->p25_is_tuned == 0 &&
            (now - state->last_vc_sync_time) > 2 && dmr_t3_active_has_entries (state))
        {
          s_capmax_non_status_csbks++;
          if ((int)(state->currentslot & 1) == s_capmax_status_slot || csbk_fid == 0x00)
            s_capmax_slot_aloha_cnt++;
          if (s_capmax_slot_aloha_cnt >= 2 || s_capmax_non_status_csbks >= 8)
          {
            for (int i = 0; i < DMR_T3_ACTIVE_MAX; i++)
            {
              if (state->dmr_t3_active_chan[i] == 0)
                continue;
              if (s_t3_active_cm_seen[i] ||
                  (strcmp(s_t3_active_kind[i], "Group") == 0 &&
                   (s_capmax_slot_aloha_cnt >= 3 || (now - s_t3_active_grant_time[i]) > 1)))
              {
                dmr_t3_active_clear_entry (state, i);
              }
            }
          }
        }

        //if using rigctl we can set an unknown or updated cc frequency
        //by polling rigctl for the current frequency
        if (opts->use_rigctl == 1 && opts->p25_is_tuned == 0) //&& state->p25_cc_freq == 0
        {
          ccfreq = trunk_get_current_freq (opts, state);
          trunk_set_cc_freq_if_valid (opts, state, ccfreq);
        }

        //if using rtl input, we can ask for the current frequency tuned
        if (opts->audio_in_type == 3 && opts->p25_is_tuned == 0) //&& state->p25_cc_freq == 0
        {
          ccfreq = (long int)opts->rtlsdr_center_freq;
          trunk_set_cc_freq_if_valid (opts, state, ccfreq);
        }

        //when on a CC, rotate the symbol out file every hour, if enabled
        if (opts->p25_is_tuned == 0) //if not currently tuned on Tier 3 system
          rotate_symbol_out_file(opts, state); //may need a second check to make sure other slot on T3 standard isn't busy as well

      }

      //P_CLEAR
      if (csbk_o == 46)
      {

        //NOTE: Do not zero out lasttg/lastsrc in TLC FLCO when using p_clear, otherwise,
        //it will affect the conditions below and fail to trigger on tg_hold

        //test misc conditions to trigger a clear and immediately return to CC, or remain on current VC;
        int clear = 0;
        int pslot = state->currentslot; //this slot
        int oslot = (state->currentslot ^ 1) & 1; //opposite slot
        pslot++; oslot++; //set to 1 and 2 and not 0 and 1

        //if the other slot is IDLE or TLC condition -- May consider disabling this one
        // if (state->currentslot == 0 && (state->dmrburstR == 9 && state->dmrburstR == 7)) clear = 1;
        // if (state->currentslot == 0 && (state->dmrburstL == 9 && state->dmrburstL == 7)) clear = 1;

        //if no voice header, pi header, or voice sync in the opposite slot currently (or opposite slot call is blocked/locked-out)
        int oslot_blocked = trunk_is_dmr_slot_blocked(opts, state, (state->currentslot ^ 1) & 1);
        if (state->currentslot == 0 && (oslot_blocked || (!state->dmr_hangtime[1] && state->dmrburstR != 16 && state->dmrburstR != 0 && state->dmrburstR != 1))) clear = 2;
        if (state->currentslot == 1 && (oslot_blocked || (!state->dmr_hangtime[0] && state->dmrburstL != 16 && state->dmrburstL != 0 && state->dmrburstL != 1))) clear = 3;

        //if tuning and decoding data is desired, then do a secondary check here for data headers and blocks in the opposite slot
        if (opts->trunk_tune_data_calls == 1)
        {
          //if no data header or data block sync in the opposite slot currently
          if (state->currentslot == 0 && (state->dmrburstR == 6 || state->dmrburstR == 7 || state->dmrburstR == 8 || state->dmrburstR == 10)) clear = 21;
          if (state->currentslot == 1 && (state->dmrburstL == 6 || state->dmrburstL == 7 || state->dmrburstL == 8 || state->dmrburstL == 10)) clear = 22;
        }

        //if we have a tg hold in place that matches traffic that was just on this slot (this will override other calls in the opposite slot)
        if (state->currentslot == 0 && state->tg_hold == state->lasttg && state->tg_hold != 0)  clear = 4;
        if (state->currentslot == 1 && state->tg_hold == state->lasttgR && state->tg_hold != 0) clear = 5;

        //make sure we aren't sent back immediately by an p_clear condition upon first tuning (i.e., random ENC LO, or end of Data Call)
        //one second default on hangtime should be an optimal time, but let it be user configurable with hangtime option
        if (csbk_fid != 253 &&
            !(csbk_fid == 255 && ((state->currentslot == 0 ? state->lasttgR == 0 : state->lasttg == 0) || oslot_blocked)) &&
            time(NULL)-state->last_t3_tune_time < opts->trunk_hangtime) clear = 0;

        //initial line break
        fprintf (stderr, "\n");
        fprintf (stderr, " Clear (P_CLEAR) ");
        if (csbk_fid < 253 && csbk_fid != 12)
        {
          uint16_t pclear_lpcn = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 12);
          uint8_t  pclear_gi   = cs_pdu_bits[31];
          uint32_t pclear_tgt  = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[32], 24);
          uint32_t pclear_src  = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);
          if (pclear_lpcn != 0 || pclear_tgt != 0 || pclear_src != 0)
          {
            fprintf (stderr, "- %s; ", pclear_gi ? "Group" : "Private");
            if (pclear_lpcn != 0 && pclear_lpcn < 0xFFE)
              fprintf (stderr, "TSCC LPCN: %04d; ", pclear_lpcn);
            fprintf (stderr, "Target: %08d; Source: %08d; ", pclear_tgt, pclear_src);
            dmr_gateway_identifier (pclear_src, pclear_tgt);
          }
        }
        int did_pclear_reset = 0;
        #ifdef PCLEAR_TUNE_AWAY

        if (opts->p25_trunk == 1)
        {

          //check the p_clear logic and report status
          const char * lo_reason = "No Encrypted Call Trunking";
          int8_t c_gi = state->gi[state->currentslot & 1];
          if (c_gi == 1 && opts->trunk_tune_private_calls == 0)
            lo_reason = "No Private Call Trunking";
          else if (c_gi == 0 && opts->trunk_tune_group_calls == 0)
            lo_reason = "No Group Call Trunking";
          else if (opts->trunk_tune_enc_calls == 1)
            lo_reason = "Call Locked Out";
          if      (clear && csbk_fid == 255)  fprintf (stderr, " Slot %d %s; Slot %d Free; Return to CC; ", pslot, lo_reason, oslot);
          else if (!clear && csbk_fid == 255) fprintf (stderr, " Slot %d %s; Slot %d Busy; Remain on VC;", pslot, lo_reason, oslot);
          else if (clear && csbk_fid == 254)  fprintf (stderr, " Cap+ Rest LSN Change: %d; Slot %d Free; Slot %d Free; Go To Rest LSN;", state->dmr_rest_channel, pslot, oslot); //disabled
          else if (!clear && csbk_fid == 254) fprintf (stderr, " Cap+ Rest LSN Change: %d; Slot %d Free; Slot %d Busy; Remain on LSN;", state->dmr_rest_channel, pslot, oslot); //disabled
          else if (clear && csbk_fid == 253)  fprintf (stderr, " Cap+ Rest LSN Change: %d; No CSBK Channel Activity; Go To Rest LSN;", state->dmr_rest_channel);
          else if (!clear && csbk_fid == 253) fprintf (stderr, " Cap+ Rest LSN Change: %d; CSBK Channel Activity; Remain on LSN;", state->dmr_rest_channel); //this should never happen in code
          else if (!clear && csbk_fid == 12)  fprintf (stderr, " Con+ Slot %d Termination: Slot %d Busy Voice or Data Call;", pslot, oslot); //Con+ test clears based on the Call Termination CSBK
          else if (clear && csbk_fid == 12)   fprintf (stderr, " Con+ Slot %d Termination: Slot %d Clear or Control CSBK;", pslot, oslot); //Con+ test clears based on the Call Termination CSBK
          else if (!clear)                    fprintf (stderr, " Slot %d Clear; Slot %d Busy; Remain on VC;", pslot, oslot);
          else if (clear == 1)                fprintf (stderr, " Slot %d Clear; Slot %d Idle; Return to CC;", pslot, oslot);
          else if (clear == 2 || clear == 3)  fprintf (stderr, " Slot %d Clear; Slot %d Free; Return to CC;", pslot, oslot);
          else if (clear == 4 || clear == 5)  fprintf (stderr, " Slot %d Clear w/ TG Hold %d; Slot %d Activity Override; Return to CC; ", pslot, state->tg_hold, oslot);
          //NOTE: Below clears are just conditions for reporting, and not clear to tune away, so they are set back to zero
          else if (clear == 21 || clear == 22)
          {
            fprintf (stderr, " Slot %d Clear; Slot %d Data; Remain on DC;", pslot, oslot);
            clear = 0; //flag as 0 so we won't tune away until data call is completed
          }

          if (clear)
          {
            long int pclear_cur_freq = trunk_get_current_freq(opts, state);
            if (opts->p25_trunk == 1 && state->p25_cc_freq != 0 &&
                (opts->p25_is_tuned == 1 || (pclear_cur_freq != 0 && pclear_cur_freq != state->p25_cc_freq) || csbk_fid == 253))
            {
              did_pclear_reset = 1;

              //run a watchdog here so we can update this with the most recent info
              watchdog_event_history(opts, state, 0);
              watchdog_event_current(opts, state, 0);
              watchdog_event_history(opts, state, 1);
              watchdog_event_current(opts, state, 1);

              //display/le/buzzer bug fix when p_clear activated (unsure why this was disabled)
              //clear only the current slot initially, then clear both if tuning to a different freq
              if (state->currentslot == 0 && csbk_fid != 253) //don't reset on Cap+ since we aren't testing based on the current TS
              {
                state->lastsrc = 0;
                state->lasttg = 0;
                state->payload_mi = 0;
                state->payload_algid = 0;
                state->payload_keyid = 0;
                state->dmr_so = 0;
                state->dmr_bcast_tg[0] = 0;
                state->dmr_hangtime[0] = 0;
                dmr_talker_alias_reset(state, 0);
              }
              if (state->currentslot == 1 && csbk_fid != 253)
              {
                state->lastsrcR = 0;
                state->lasttgR = 0;
                state->payload_miR = 0;
                state->payload_algidR = 0;
                state->payload_keyidR = 0;
                state->dmr_soR = 0;
                state->dmr_bcast_tg[1] = 0;
                state->dmr_hangtime[1] = 0;
                dmr_talker_alias_reset(state, 1);
              }
              if (state->currentslot == 0 || csbk_fid == 253) ambe_hw_reset_call(0);
              if (state->currentslot == 1 || csbk_fid == 253) ambe_hw_reset_call(1);

              //rigctl
              if (opts->use_rigctl == 1)
              {
                //Guess I forgot to add this condition here
                long int cur_freq = trunk_get_current_freq(opts, state);
                if (cur_freq != state->p25_cc_freq || csbk_fid == 253)
                {
                  dmr_reset_blocks (opts, state); //reset all block gathering since we are tuning away from current frequency

                  //clear both if tuning away to another frequency
                  state->lastsrc = 0;
                  state->lasttg = 0;
                  state->payload_mi = 0;
                  state->payload_algid = 0;
                  state->payload_keyid = 0;
                  state->dmr_so = 0;

                  state->lastsrcR = 0;
                  state->lasttgR = 0;
                  state->payload_miR = 0;
                  state->payload_algidR = 0;
                  state->payload_keyidR = 0;
                  state->dmr_soR = 0;
                  state->dmr_bcast_tg[0] = 0;
                  state->dmr_bcast_tg[1] = 0;
                  state->dmr_hangtime[0] = 0;
                  state->dmr_hangtime[1] = 0;

                  dmr_talker_alias_reset(state, 0);
                  dmr_talker_alias_reset(state, 1);
                  sprintf (state->call_string[0], "%s", "                     "); //21 spaces
                  sprintf (state->call_string[1], "%s", "                     "); //21 spaces
                  state->gi[0] = -1;
                  state->gi[1] = -1;
                  ambe_hw_reset_call(0);
                  ambe_hw_reset_call(1);
                }

                //reset some strings
                sprintf (state->call_string[state->currentslot], "%s", "                     "); //21 spaces
                dmr_csbk_clear_active_slot (state);
                dmr_active_channel_reset_rows (state);
                trunk_set_t3_vc_lpcn (0);
                state->last_vc_sync_time = 0;
                state->last_cc_sync_time = time(NULL);
                opts->p25_is_tuned = 0;
                state->p25_vc_freq[0] = state->p25_vc_freq[1] = 0;
                if (cur_freq != state->p25_cc_freq || csbk_fid == 253)
                {
                  if (opts->setmod_bw != 0 ) SetModulation(opts->rigctl_sockfd, opts->setmod_bw);
                  SetFreq(opts->rigctl_sockfd, state->p25_cc_freq);
                  state->color_code_ok = 0;
                }
                else
                {
                  trunk_update_filter_for_freq(state->p25_cc_freq);
                }
              }

              //rtl
              else if (opts->audio_in_type == 3)
              {
                #ifdef USE_RTLSDR
                //Guess I forgot to add this condition here
                uint32_t tempf = (uint32_t)state->p25_cc_freq;
                if (opts->rtlsdr_center_freq != tempf)
                  dmr_reset_blocks (opts, state); //reset all block gathering since we are tuning away from current frequency

                //clear both if tuning away to another frequency
                if (opts->rtlsdr_center_freq != tempf)
                {
                  state->lastsrc = 0;
                  state->lasttg = 0;
                  state->payload_mi = 0;
                  state->payload_algid = 0;
                  state->payload_keyid = 0;
                  state->dmr_so = 0;

                  state->lastsrcR = 0;
                  state->lasttgR = 0;
                  state->payload_miR = 0;
                  state->payload_algidR = 0;
                  state->payload_keyidR = 0;
                  state->dmr_soR = 0;
                  state->dmr_bcast_tg[0] = 0;
                  state->dmr_bcast_tg[1] = 0;
                  state->dmr_hangtime[0] = 0;
                  state->dmr_hangtime[1] = 0;

                  dmr_talker_alias_reset(state, 0);
                  dmr_talker_alias_reset(state, 1);
                  sprintf (state->call_string[0], "%s", "                     "); //21 spaces
                  sprintf (state->call_string[1], "%s", "                     "); //21 spaces
                  state->gi[0] = -1;
                  state->gi[1] = -1;
                  ambe_hw_reset_call(0);
                  ambe_hw_reset_call(1);
                }

                //reset some strings
                sprintf (state->call_string[state->currentslot], "%s", "                     "); //21 spaces
                dmr_csbk_clear_active_slot (state);
                dmr_active_channel_reset_rows (state);
                trunk_set_t3_vc_lpcn (0);
                state->last_cc_sync_time = time(NULL);
                state->last_vc_sync_time = 0;
                opts->p25_is_tuned = 0;
                state->p25_vc_freq[0] = state->p25_vc_freq[1] = 0;
                if (opts->rtlsdr_center_freq != tempf)
                {
                  rtl_dev_tune (opts, state->p25_cc_freq);
                  state->color_code_ok = 0;
                }
                else
                  trunk_update_filter_for_freq(state->p25_cc_freq);
                #endif
              }

              watchdog_event_history(opts, state, 0);
              watchdog_event_history(opts, state, 1);

            }
          }
        } //end if trunking is enabled
        #else
        UNUSED (clear); UNUSED(pslot); UNUSED(oslot);
        #endif //end if PCLEAR_TUNE_AWAY is enabled in code
        if (csbk_fid < 253)
        {
          uint8_t cslot = (uint8_t)(state->currentslot & 1);
          if (!did_pclear_reset)
          {
            watchdog_event_history(opts, state, cslot);
            watchdog_event_current(opts, state, cslot);
            if (cslot == 0)
            {
              state->lastsrc = 0;
              state->lasttg = 0;
              state->payload_mi = 0;
              state->payload_algid = 0;
              state->payload_keyid = 0;
              state->dmr_so = 0;
              dmr_talker_alias_reset(state, 0);
            }
            else
            {
              state->lastsrcR = 0;
              state->lasttgR = 0;
              state->payload_miR = 0;
              state->payload_algidR = 0;
              state->payload_keyidR = 0;
              state->dmr_soR = 0;
              dmr_talker_alias_reset(state, 1);
            }
            state->gi[cslot] = -1;
            ambe_hw_reset_call(cslot);
            sprintf (state->call_string[cslot], "%s", "                     "); //21 spaces
            dmr_csbk_clear_active_slot (state);
          }
          state->dmr_hangtime[cslot] = 0;
          state->dmr_bcast_tg[cslot] = 0;
          if (!did_pclear_reset)
            watchdog_event_history(opts, state, cslot);
        }
      }

      //(P_PROTECT)
      if (csbk_o == 47)
      {
        //initial line break
        fprintf (stderr, "\n");
        fprintf (stderr, " Protect (P_PROTECT) -");
        uint16_t reserved = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 12);
        uint8_t p_kind = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[28], 3);
        uint8_t gi = cs_pdu_bits[31];
        uint32_t target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[32], 24);
        uint32_t source = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);
        UNUSED(reserved);

        if (gi)  fprintf (stderr, " Group");
        if (!gi) fprintf (stderr, " Private");

        if (p_kind == 0) fprintf (stderr, " Disable Target PTT (DIS_PTT)");
        if (p_kind == 1) fprintf (stderr, " Enable Target PTT (EN_PTT)");
        if (p_kind == 2) fprintf (stderr, " Call Hangtime (ILLEGALLY_PARKED)");
        if (p_kind == 3) fprintf (stderr, " Enable Target MS PTT (EN_PTT_ONE_MS)");

        fprintf (stderr, "\n");
        fprintf (stderr, "  Source: %08d; Target: %08d; ", source, target);

        //check the source and/or target for special gateway identifiers
        dmr_gateway_identifier (source, target);

        //preserve active TGT/SRC from TLC during Call Hangtime, or populate if tuning in mid-hangtime
        uint8_t slot = state->currentslot;
        uint8_t is_bcast = (p_kind == 3 && source != 0 && state->dmr_bcast_tg[slot] == source) ? 1 : 0;
        int is_grp = (p_kind == 3) ? (state->gi[slot] == 0 || is_bcast) : (gi != 0);

        if (p_kind == 2 || p_kind == 3)
          state->dmr_hangtime[slot] = 1;

        if (p_kind == 2)
        {
          if (slot == 0 && (state->lasttg == 0 || state->lastsrc == 0))
          {
            if (state->lasttg == 0) state->lasttg = target;
            if (state->lastsrc == 0) state->lastsrc = source;
            state->gi[0] = gi ? 0 : 1;
          }
          else if (slot == 1 && (state->lasttgR == 0 || state->lastsrcR == 0))
          {
            if (state->lasttgR == 0) state->lasttgR = target;
            if (state->lastsrcR == 0) state->lastsrcR = source;
            state->gi[1] = gi ? 0 : 1;
          }
          if (state->call_string[slot][1] == ' ')
            dsd_set_call_string (state, slot, gi ? "Group" : "Private",
                                 (gi && state->dmr_bcast_tg[slot] != 0 && state->dmr_bcast_tg[slot] == target) ? "Broadcast" : "");
        }
        else if (p_kind == 3)
        {
          if (slot == 0 && (state->lasttg == 0 || state->lastsrc == 0))
          {
            if (state->lasttg == 0) state->lasttg = source;
            if (state->lastsrc == 0) state->lastsrc = target;
          }
          else if (slot == 1 && (state->lasttgR == 0 || state->lastsrcR == 0))
          {
            if (state->lasttgR == 0) state->lasttgR = source;
            if (state->lastsrcR == 0) state->lastsrcR = target;
          }
          if (state->call_string[slot][1] == ' ' && (state->gi[slot] == 0 || state->gi[slot] == 1 || is_bcast))
            dsd_set_call_string (state, slot, is_grp ? "Group" : "Private", is_bcast ? "Broadcast" : "");
        }

        if (p_kind != 3 || is_grp)
          state->gi[slot] = is_grp ? 0 : 1;

        if (opts->p25_trunk == 1 && state->p25_cc_freq != 0 &&
            trunk_is_dmr_slot_blocked(opts, state, slot))
        {
          long int cur_freq = trunk_get_current_freq(opts, state);
          if ((cur_freq != 0 && cur_freq != state->p25_cc_freq) ||
              (cur_freq == 0 && opts->p25_is_tuned == 1))
          {
            uint8_t dummy[12]; uint8_t dbits[96];
            memset (dummy, 0, sizeof(dummy));
            memset (dbits, 0, sizeof(dbits));
            dummy[0] = 46; dummy[1] = 255;
            dmr_cspdu (opts, state, dbits, dummy, 1, 0);
          }
          else if (opts->p25_is_tuned == 1)
          {
            int oslot = (slot ^ 1) & 1;
            uint8_t oburst = (oslot == 0) ? state->dmrburstL : state->dmrburstR;
            int oslot_blocked = trunk_is_dmr_slot_blocked(opts, state, oslot);
            if (oslot_blocked || (!state->dmr_hangtime[oslot] && oburst != 16 && oburst != 0 && oburst != 1))
            {
              opts->p25_is_tuned = 0;
              state->last_vc_sync_time = 0;
              state->p25_vc_freq[0] = state->p25_vc_freq[1] = 0;
            }
          }
        }

      }

      if (csbk_o == 40) //0x28
      {
        int i;
        //initial line break
        fprintf (stderr, "\n");
        fprintf (stderr, " Announcements (C_BCAST)");
        uint8_t a_type = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 5);
        if (a_type == 0) fprintf (stderr, " Announce/Withdraw TSCC (Ann_WD_TSCC)");
        if (a_type == 1) fprintf (stderr, " Specify Call Timer Parameters (CallTimer_Parms)");
        if (a_type == 2) fprintf (stderr, " Vote Now Advice (Vote_Now)");
        if (a_type == 3) fprintf (stderr, " Broadcast Local Time (Local_Time)");
        if (a_type == 4) fprintf (stderr, " Mass Registration (MassReg)");
        if (a_type == 5) fprintf (stderr, " Announce Logical Channel/Frequency Relationship (Chan_Freq)");
        if (a_type == 6) fprintf (stderr, " Adjacent Site Information (Adjacent_Site)");
        if (a_type == 7) fprintf (stderr, " General Site Parameters (Gen_Site_Params)");
        if (a_type > 7 && a_type < 0x1E) fprintf (stderr, " Reserved: %02X", a_type);
        if (a_type == 0x1E || a_type == 0x1F) fprintf (stderr, " Manufacturer Specific: %02X", a_type);

        //parms1 start 21, len 14
        uint16_t bparms1 = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[21], 14);
        uint8_t bpbits1[14]; for (i = 0; i < 14; i++) bpbits1[i] = cs_pdu_bits[21+i];

        //registration required
        uint8_t reg_req = cs_pdu_bits[35];

        //backoff number
        uint8_t backoff = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[36], 4);

        //syscode start 40, len 16
        uint16_t syscode = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[40], 14);

        //parms2 start 56, len 24
        uint32_t bparms2 = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);
        uint8_t bpbits2[24]; for (i = 0; i < 24; i++) bpbits2[i] = cs_pdu_bits[56+i];

        //MBC parms for when they will be needed
        uint8_t mbc_lb = cs_pdu_bits[96]; UNUSED(mbc_lb);
        uint8_t mbc_pf = cs_pdu_bits[97]; UNUSED(mbc_pf);
        uint8_t mbc_csbko = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[98], 6);
        uint8_t mbc_res = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[104], 4);
        uint8_t mbc_cc = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[108], 4);
        uint8_t mbc_cdeftype = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[112], 4);
        uint8_t mbc_res2 = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[116], 2);
        unsigned long long int mbc_cdefparms = (unsigned long long int)ConvertBitIntoBytes(&cs_pdu_bits[118], 58);
        uint16_t mbc_lpchannum = 0;
        uint16_t mbc_abs_tx_int = 0;
        uint16_t mbc_abs_tx_step = 0;
        uint16_t mbc_abs_rx_int = 0;
        uint16_t mbc_abs_rx_step = 0;
        long int freqt, freqr = 0;

        //common value among some a_types
        uint16_t a_channel = (uint16_t)ConvertBitIntoBytes(&bpbits2[12], 12);

        //Ann_WD_TSCC
        if (a_type == 0)
        {
          //Color Codes and Flags
          uint8_t ann_res = (uint8_t)ConvertBitIntoBytes(&bpbits1[0], 4);
          uint8_t cc_ch1 = (uint8_t)ConvertBitIntoBytes(&bpbits1[4], 4);
          uint8_t cc_ch2 = (uint8_t)ConvertBitIntoBytes(&bpbits1[8], 4);
          uint8_t ch1_flag = bpbits1[12];
          uint8_t ch2_flag = bpbits1[13];

          uint16_t bcast_ch1 = (uint16_t)ConvertBitIntoBytes(&bpbits2[0], 12);
          uint16_t bcast_ch2 = (uint16_t)ConvertBitIntoBytes(&bpbits2[12], 12);

          fprintf (stderr, "\n");
          if (ann_res) fprintf (stderr, " Res: %X;", ann_res);
          fprintf (stderr, " LPCN CH1: %d; CC: %d;", bcast_ch1, cc_ch1);
          if (ch1_flag == 0) fprintf (stderr, " Add;");
          if (ch1_flag == 1) fprintf (stderr, " Remove;");

          fprintf (stderr, " LPCN CH2: %d; CC: %d;", bcast_ch2, cc_ch2);
          if (ch2_flag == 0) fprintf (stderr, " Add;");
          if (ch2_flag == 1) fprintf (stderr, " Remove;");

          if (opts->p25_is_tuned == 0)
          {
            if (ch1_flag == 0 && bcast_ch1 != 0 && bcast_ch1 != 0xFFF && cc_ch1 == state->dmr_color_code)
              trunk_set_t3_cc_lpcn(bcast_ch1);
            else if (ch2_flag == 0 && bcast_ch2 != 0 && bcast_ch2 != 0xFFF && cc_ch2 == state->dmr_color_code)
              trunk_set_t3_cc_lpcn(bcast_ch2);
          }
        }

        //CallTimer_parms
        if (a_type == 1)
        {
          //7.9.19.2 Table 7.70 / Annex A Table A.1
          uint16_t t_emerg_timer = (uint16_t)ConvertBitIntoBytes(&bpbits1[0], 9);
          uint8_t t_packet_timer = (uint8_t)ConvertBitIntoBytes(&bpbits1[9], 5);

          uint16_t t_msms_timer = (uint16_t)ConvertBitIntoBytes(&bpbits2[0], 12);
          uint16_t t_msline_timer = (uint16_t)ConvertBitIntoBytes(&bpbits2[12], 12);

          char emerg_s[16], pkt_s[16], msms_s[16], line_s[16];
          dmr_format_timer_val (emerg_s, sizeof(emerg_s), t_emerg_timer, 511, 30);
          dmr_format_timer_val (pkt_s, sizeof(pkt_s), t_packet_timer, 31, 5);
          dmr_format_timer_val (msms_s, sizeof(msms_s), t_msms_timer, 4095, 10);
          dmr_format_timer_val (line_s, sizeof(line_s), t_msline_timer, 4095, 10);

          fprintf (stderr, "\n");
          fprintf (stderr, " Timers - Emergency: %s; Packet: %s; MS-MS: %s; Line: %s; ",
                   emerg_s, pkt_s, msms_s, line_s);
        }

        //Local_Time
        if (a_type == 3)
        {
          uint8_t lt_day = (uint8_t)ConvertBitIntoBytes(&bpbits1[0], 5); //1-31
          uint8_t lt_mon = (uint8_t)ConvertBitIntoBytes(&bpbits1[5], 4); //1-12, or 0 if not broadcast
          uint8_t lt_off = (uint8_t)ConvertBitIntoBytes(&bpbits1[9], 4); //0-14 hours, or 15 if not broadcast
          uint8_t lt_off_sign = bpbits1[13]; //0 is positive offset, 1 is negative offset

          uint8_t lt_hour = (uint8_t)ConvertBitIntoBytes(&bpbits2[0], 5); //0-23
          uint8_t lt_mins = (uint8_t)ConvertBitIntoBytes(&bpbits2[5], 6); //0-59
          uint8_t lt_secs = (uint8_t)ConvertBitIntoBytes(&bpbits2[11], 6); //0-59
          uint8_t lt_dofw = (uint8_t)ConvertBitIntoBytes(&bpbits2[17], 3); //day of week
          uint8_t lt_off_fr = (uint8_t)ConvertBitIntoBytes(&bpbits2[20], 2); //0 = 0; 1 = +15 mins; 2 = +30 mins; 3 = +45 mins;
          uint8_t lt_res = (uint8_t)ConvertBitIntoBytes(&bpbits2[22], 2);

          int off_mins = (int)lt_off * 60 + (int)lt_off_fr * 15;
          if (lt_off_sign == 1) off_mins = -off_mins;
          int total_mins = (int)lt_hour * 60 + (int)lt_mins + off_mins;
          while (total_mins < 0) total_mins += 1440;
          total_mins %= 1440;
          int localhour = total_mins / 60;
          int localmin  = total_mins % 60;

          fprintf (stderr, "\n");
          if (lt_mon != 0 && lt_day != 0)
            fprintf (stderr, " Date: %d.%d;", lt_mon, lt_day);
          //day of the week, 1 is Sunday, 7 is Saturday, 0 not broadcasted
          if (lt_dofw == 1) fprintf (stderr, " Sunday;");
          if (lt_dofw == 2) fprintf (stderr, " Monday;");
          if (lt_dofw == 3) fprintf (stderr, " Tuesday;");
          if (lt_dofw == 4) fprintf (stderr, " Wednesday;");
          if (lt_dofw == 5) fprintf (stderr, " Thursday;");
          if (lt_dofw == 6) fprintf (stderr, " Friday;");
          if (lt_dofw == 7) fprintf (stderr, " Saturday;");
          fprintf (stderr, " UTC Time: %02d:%02d:%02d;", lt_hour, lt_mins, lt_secs);
          if (lt_off != 15)
          {
            fprintf (stderr, " Local: %02d:%02d:%02d;", localhour, localmin, lt_secs);
            if (lt_off_fr == 0)
              fprintf (stderr, " Offset: %c%d;", lt_off_sign ? '-' : '+', lt_off);
            else
              fprintf (stderr, " Offset: %c%d:%02d;", lt_off_sign ? '-' : '+', lt_off, lt_off_fr * 15);
          }
          if (lt_res) fprintf (stderr, " Res: %d;", lt_res);

        }

        //Chan_Freq
        if (a_type == 5)
        {
          // if (a_channel != 0xFFF) fprintf (stderr, " LPCN: %d;", a_channel);
          if (a_channel == 0) fprintf (stderr, " LPCN: Null;");
          if (a_channel != 0)
          {
            if (mbc_cdeftype == 0) //if 0, then absolute channel parms
            {
              mbc_lpchannum = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[118], 12);
              mbc_abs_tx_int = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[130], 10);
              mbc_abs_tx_step = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[140], 13);
              mbc_abs_rx_int = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[153], 10);
              mbc_abs_rx_step = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[163], 13);
              freqr = (mbc_abs_rx_int * 1000000 ) + (mbc_abs_rx_step * 125);
              freqt = (mbc_abs_tx_int * 1000000 ) + (mbc_abs_tx_step * 125);

              if (a_channel == 0xFFF) fprintf (stderr, "\n APCN: %04d;", mbc_lpchannum); //absolute physical channel number
              else if (a_channel != 0) fprintf (stderr, "\n LPCN: %04d;", mbc_lpchannum); //logical physical channel number
              fprintf (stderr, " RX Base: %d; RX Step: %d; RX Freq: %ld;", mbc_abs_rx_int * 1000000, mbc_abs_rx_step * 125, freqr);
              fprintf (stderr, "\n            "); //12 spaces
              fprintf (stderr, " TX Base: %d; TX Step: %d; TX Freq: %ld;", mbc_abs_tx_int * 1000000, mbc_abs_tx_step * 125, freqt);

              //experimental -- assign a_channel or mbc_lpchannum and freqr to channel map if not available
              if (a_channel != 0 && a_channel != 0xFFF && freqr != 0)
              {
                if (state->trunk_chan_map[a_channel] == 0)
                {
                  state->trunk_chan_map[a_channel] = freqr;
                  //add to rotation for CC Hunting on extended noframesync
                  state->trunk_lcn_freq[state->lcn_freq_count++%25] = freqr; //no not exceed 25 entries
                  if (state->lcn_freq_count > 25) state->lcn_freq_count = 25;
                }
              }

              //this one probably doesn't matter, I think that both values have the same channel number (a_channel and mbc_lpchannum)
              //and also since absolute channel grants will also have these values available to figure out frequency to tune to
              // if (a_channel == 0xFFF && freqr != 0)
              // {
              //   if (state->trunk_chan_map[mbc_lpchannum] == 0 && mbc_lpchannum != 0xFFFF && mbc_lpchannum != 0)
              //   {
              //     state->trunk_chan_map[mbc_lpchannum] = freqr;
              //     //add to rotation for CC Hunting on extended noframesync
              //     state->trunk_lcn_freq[state->lcn_freq_count++%25] = freqr; //no not exceed 25 entries
              //     if (state->lcn_freq_count > 25) state->lcn_freq_count = 25;
              //   }
              // }

            }
            else
            {
              fprintf (stderr, "\n Unknown CDEFType: %X; CDEFParms: %015llX", mbc_cdeftype, mbc_cdefparms);
              fprintf (stderr, " MBC Op: %02X;", mbc_csbko);
              // fprintf (stderr, " CC: %d;", mbc_cc); //not on chan_freq
              fprintf (stderr, " RES1: %X;", mbc_res);
              fprintf (stderr, " RES2: %X;", mbc_res2);
            }
          }
        }

        //Vote Now, Adjacent Site,
        if (a_type == 2 || a_type == 6)
        {
          uint8_t active_ava = bpbits2[0];
          uint8_t active_con = bpbits2[1];
          uint8_t c_chan_pri = (uint8_t)ConvertBitIntoBytes(&bpbits2[2], 3);
          uint8_t a_chan_pri = (uint8_t)ConvertBitIntoBytes(&bpbits2[5], 3);
          uint8_t a_reserved = (uint8_t)ConvertBitIntoBytes(&bpbits2[8], 4);

          fprintf (stderr, "\n");
          dmr_decode_syscode(opts, state, cs_pdu_bits, csbk_fid, 1);

          // fprintf (stderr, "\n");
          if (active_ava == 1) ; //fprintf (stderr, " Active Connection Information Available;");
          else fprintf (stderr, " Active Connection Information Not Available;");

          if (active_ava == 1)
          {
            // if (active_con == 1) fprintf (stderr, " Connection Active;");
            if (active_con == 1)
              fprintf (stderr, " Online;");
            else fprintf (stderr, " Offline;");
            // fprintf (stderr, " Confirmed Channel Priority: %d;", c_chan_pri);
            // fprintf (stderr, " Active Channel Priority: %d;", a_chan_pri);
            fprintf (stderr, " CC Pri: %d;", c_chan_pri);
            fprintf (stderr, " AC Pri: %d;", a_chan_pri);
            if (a_reserved) fprintf (stderr, " Res: %X;", a_reserved);
            if (a_channel != 0xFFF && a_channel != 0) fprintf (stderr, " LPCN: %d;", a_channel);
            if (a_channel == 0) fprintf (stderr, " LPCN: Null;");
            if (a_channel == 0xFFF)
            {
              if (mbc_cdeftype == 0) //if 0, then absolute channel parms
              {
                mbc_lpchannum = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[118], 12);
                mbc_abs_tx_int = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[130], 10);
                mbc_abs_tx_step = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[140], 13);
                mbc_abs_rx_int = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[153], 10);
                mbc_abs_rx_step = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[163], 13);
                freqr = (mbc_abs_rx_int * 1000000 ) + (mbc_abs_rx_step * 125);
                freqt = (mbc_abs_tx_int * 1000000 ) + (mbc_abs_tx_step * 125);

                fprintf (stderr, "\n APCN: %04d;", mbc_lpchannum); //absolute physical channel number
                fprintf (stderr, " RX Base: %d; RX Step: %d; RX Freq: %ld;", mbc_abs_rx_int * 1000000, mbc_abs_rx_step * 125, freqr);
                fprintf (stderr, "\n            "); //12 spaces
                fprintf (stderr, " TX Base: %d; TX Step: %d; TX Freq: %ld;", mbc_abs_tx_int * 1000000, mbc_abs_tx_step * 125, freqt);

              }
              else
              {
                fprintf (stderr, "\n Unknown CDEFType: %X; CDEFParms: %015llX", mbc_cdeftype, mbc_cdefparms);
                fprintf (stderr, " MBC Op: %02X;", mbc_csbko);
                if (a_type == 2)
                  fprintf (stderr, " CC: %d;", mbc_cc);
                fprintf (stderr, " RES1: %X;", mbc_res);
                fprintf (stderr, " RES2: %X;", mbc_res2);
              }
            }
          }

        }

        //Gen_Site_Parms
        if (a_type == 7)
        {
          //parms1 are all reserved values

          //current (confirmed site information)
          uint8_t csi = (uint8_t)ConvertBitIntoBytes(&bpbits1[0], 8); //see clause 7.2.40
          //Network Information
          uint8_t nin = (uint8_t)ConvertBitIntoBytes(&bpbits2[16], 8); //see clause 7.2.41

          //the second bit of the CSI, the rest are reserved (wasteful)
          uint8_t hibernate_flag = bpbits2[1];

          //the first bit of the nin, the rest are reserved (yet again)
          uint8_t reg_tg_sub = bpbits2[16]; //if the MS has to send TG during Registration Process (Zzzzzz)

          fprintf (stderr, "\n");
          fprintf (stderr, " Hibernate Flag: %d; Reg Flag: %d; RES1: %d; RES2: %X; RES3: %X; BPARMS1: %X", hibernate_flag, reg_tg_sub, bpbits1[0], csi & 0x3F, nin & 0x7F, bparms1);
        }

        //MassReg
        if (a_type == 4)
        {
          static const char * const reg_win_str[16] = {
            "Cancel", "0.5s", "1s", "2s",
            "5s", "10s", "20s", "30s",
            "100s", "300s", "1000s", "3000s",
            "10000s", "30000s", "100000s", "200000s"
          };

          uint8_t mreg_res   = (uint8_t)ConvertBitIntoBytes(&bpbits1[0], 5);
          uint8_t reg_window = (uint8_t)ConvertBitIntoBytes(&bpbits1[5], 4); //Table 6.12
          uint8_t aloha_mask = (uint8_t)ConvertBitIntoBytes(&bpbits1[9], 5);

          //ADDRNull or MS Individual Address
          uint32_t reg_address = bparms2; //see clause 7.2.41

          fprintf (stderr, "\n");
          fprintf (stderr, " Reg Window: %s (%X); Aloha Mask: %02X; Target: %d; ",
                   reg_win_str[reg_window & 0xF], reg_window, aloha_mask, reg_address);

          if (mreg_res != 0)
            fprintf (stderr, "Res: %02X", mreg_res);

          if (reg_address != 0)
            dmr_gateway_identifier(0, reg_address);
        }

        //debug
        if (opts->payload == 1)
        {
          fprintf (stderr, "\n ");
          fprintf (stderr, " SYS: %04X;", syscode);
          fprintf (stderr, " Reg: %d;", reg_req);
          fprintf (stderr, " Backoff: %X;", backoff); //7.2.5, Table 7.39
          fprintf (stderr, " BParms1: %04X;", bparms1);
          fprintf (stderr, " BParms2: %06X;", bparms2);
          if (mbc_cdefparms != 0) //if this isn't an MBC block, then these will all be zeroes
          {
            fprintf (stderr, "\n ");
            fprintf (stderr, " MBC Op: %02X;", mbc_csbko);
            if (a_type == 2)
              fprintf (stderr, " CC: %d;", mbc_cc);
            fprintf (stderr, " RES1: %X;", mbc_res);
            fprintf (stderr, " RES2: %X;", mbc_res2);
            fprintf (stderr, " CDEFTYPE: %X;", mbc_cdeftype);
            fprintf (stderr, " CDEFPARMS: %015llX;", mbc_cdefparms);
          }
        }

      }

      if (csbk_o == 28 || csbk_o == 30 || csbk_o == 31)
      {
        //initial line break
        fprintf (stderr, "\n");
        if (csbk_o == 28)      fprintf (stderr, " C_AHOY - ");
        else if (csbk_o == 30) fprintf (stderr, " C_ACKVIT - ");
        else                   fprintf (stderr, " C_RAND - ");
        //C_AHOY / C_ACKVIT / C_RAND are single-block CSBKs sharing bits [16..79]
        uint16_t svc_opt = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 7);
        uint8_t svc_flag = cs_pdu_bits[23]; //service kind flag (AHOY/ACKVIT) or proxy flag (RAND)
        uint8_t als_flag = cs_pdu_bits[24]; //ambient listening service requested
        uint8_t ahoy_gi  = cs_pdu_bits[25]; //group or individual
        uint8_t ahoy_bf  = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[26], 2); //UDT blocks to follow (if required)
        uint8_t svc_kind = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[28], 4); //'Call' Type
        uint32_t ahoy_target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[32], 24);
        uint32_t ahoy_source = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);

        char ahoy_str[200]; memset(ahoy_str, 0, sizeof(ahoy_str));
        sprintf (ahoy_str, "%s TGT: %d; SRC: %d; ",
                 (csbk_o == 28) ? "AHOY" : (csbk_o == 30) ? "ACKVIT" : "RAND",
                 ahoy_target, ahoy_source);

        if (ahoy_gi == 0) fprintf (stderr, "Private ");
        else fprintf (stderr, "Group ");

        if (ahoy_gi == 0)
          strcat (ahoy_str, "Private; ");
        else strcat (ahoy_str, "Group; ");
        if (csbk_o == 28)
          state->gi[state->currentslot] = ahoy_gi ^ 1;

        fprintf (stderr, "FID: %02X SVC: %02X ", csbk_fid, svc_opt);
        if (svc_kind == 0 || svc_kind == 1) fprintf (stderr, "Voice Call ");
        else if (svc_kind == 2 || svc_kind == 3) fprintf (stderr, "Packet Data Call ");
        else if (svc_kind == 4 || svc_kind == 5) fprintf (stderr, "UDT Short Data Call ");
        else if (svc_kind == 6) fprintf (stderr, "UDT Short Data Polling Service ");
        else if (svc_kind == 7) fprintf (stderr, "Status Transport Service ");
        else if (svc_kind == 8) fprintf (stderr, "Call Diversion Service ");
        else if (svc_kind == 9) fprintf (stderr, "Call Answer Service ");
        else if (svc_kind == 10) fprintf (stderr, "Full Duplex Voice Call ");
        else if (svc_kind == 11) fprintf (stderr, "Full Duplex Packet Data Call ");
        else if (svc_kind == 12) fprintf (stderr, "Reserved ");
        else if (svc_kind == 13) fprintf (stderr, "Supplimentary Service (Stun/Revive/Kill/Auth): ");
        else if (svc_kind == 14) fprintf (stderr, "Registration/Authentication ");
        else if (svc_kind == 15) fprintf (stderr, "Cancel Call Service ");

        if (csbk_o != 31 && als_flag && ahoy_gi == 0 && (svc_kind == 0 || svc_kind == 1))
          fprintf (stderr, "Ambient Listening ");
        if (ahoy_bf > 0 && (svc_kind == 4 || svc_kind == 5 || svc_kind == 6))
          fprintf (stderr, "(%d UDT Blocks) ", ahoy_bf);
        if (csbk_o == 31 && svc_flag)
          fprintf (stderr, "Proxy ");

        fprintf (stderr, "Target: %d; Source: %d; ", ahoy_target, ahoy_source);

        if (svc_kind == 0 || svc_kind == 1) strcat (ahoy_str, "Voice Call; ");
        else if (svc_kind == 2 || svc_kind == 3) strcat (ahoy_str, "Packet Data Call; ");
        else if (svc_kind == 4 || svc_kind == 5) strcat (ahoy_str, "UDT Short Data Call; ");
        else if (svc_kind == 6) strcat (ahoy_str, "UDT Short Data Polling Service; ");
        else if (svc_kind == 7) strcat (ahoy_str, "Status Transport Service; ");
        else if (svc_kind == 8) strcat (ahoy_str, "Call Diversion Service; ");
        else if (svc_kind == 9) strcat (ahoy_str, "Call Answer Service; ");
        else if (svc_kind == 10) strcat (ahoy_str, "Full Duplex Voice Call; ");
        else if (svc_kind == 11) strcat (ahoy_str, "Full Duplex Packet Data Call; ");
        else if (svc_kind == 12) strcat (ahoy_str, "Reserved; ");
        else if (svc_kind == 13) strcat (ahoy_str, "Supplimentary Service (Stun/Revive/Kill/Auth); ");
        else if (svc_kind == 14) strcat (ahoy_str, "Registration/Authentication; ");
        else if (svc_kind == 15) strcat (ahoy_str, "Cancel Call Service; ");
        if (csbk_o != 31 && als_flag && ahoy_gi == 0 && (svc_kind == 0 || svc_kind == 1))
          strcat (ahoy_str, "Ambient Listening; ");

        if (svc_kind == 7)
        {
          char st_buf[32];
          fprintf (stderr, "Status: %d (%02X); ", svc_opt, svc_opt);
          snprintf (st_buf, sizeof(st_buf), "Status: %d (%02X); ", svc_opt, svc_opt);
          if (strlen(ahoy_str) + strlen(st_buf) < sizeof(ahoy_str))
            strcat (ahoy_str, st_buf);
        }
        else if ((svc_kind <= 6 || svc_kind == 10 || svc_kind == 11) && svc_opt != 0)
          dmr_print_svc_opt_flags ((uint8_t)svc_opt, ahoy_str, sizeof(ahoy_str));

        //check the source and/or target for special gateway identifiers
        dmr_gateway_identifier (ahoy_source, ahoy_target);

        //log ahoy as a data call event //re-enable this if you want, but it can clog up the event history
        // if (state->currentslot == 0)
        // {
        //   state->lastsrc = ahoy_source;
        //   state->lasttg = ahoy_target;
        // }
        // else
        // {
        //   state->lastsrcR = ahoy_source;
        //   state->lasttgR = ahoy_target;
        // }
        // watchdog_event_datacall (opts, state, ahoy_source, ahoy_target, ahoy_str, state->currentslot);
        // if (state->currentslot == 0)
        // {
        //   state->lastsrc = 0;
        //   state->lasttg = 0;
        // }
        // else
        // {
        //   state->lastsrcR = 0;
        //   state->lasttgR = 0;
        // }
        // watchdog_event_history(opts, state, 0);
        // watchdog_event_current(opts, state, 0);
        //end ahoy logging

      }

      if (csbk_o == 42)
      {
        //initial line break
        fprintf (stderr, "\n");
        fprintf (stderr, " P_MAINT -");

        //I can't recall ever seeing a p_maint use, always see a p_clear though
        uint16_t pm_res1 = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 12);
        uint8_t  pm_kind = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[28], 3);
        uint8_t  pm_res2 = cs_pdu_bits[31];
        uint32_t pm_target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[32], 24); //should be TSI (clear the call)
        uint32_t pm_source = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);

        if (pm_kind == 0) fprintf (stderr, "Disconnect; ");
        else fprintf (stderr, " Res Kind: %02X", pm_kind);

        if (pm_res1) fprintf (stderr, "Res A: %03X", pm_res1);
        if (pm_res2) fprintf (stderr, "Res B: 1");

        fprintf (stderr, "Target: %d; Source: %d; ", pm_target, pm_source);

        //check the source and/or target for special gateway identifiers
        dmr_gateway_identifier (pm_source, pm_target);
      }

      if (csbk_fid != 0x10) //if Not Motorola
      {
        //These opcodes to exist on CapMax as well, but don't function the same
        if (csbk_o == 32 || csbk_o == 33 || csbk_o == 34 || csbk_o == 35)
        {
          //initial line break
          fprintf (stderr, "\n"); //(Acknowledgement)
          if (csbk_o == 32) fprintf (stderr, " C_ACKD Outbound TSCC; ");
          if (csbk_o == 33) fprintf (stderr, " C_ACKU Inbound TSCC; ");
          if (csbk_o == 34) fprintf (stderr, " P_ACKD Outbound Payload; ");
          if (csbk_o == 35) fprintf (stderr, " P_ACKU Inbound Payload; ");

          uint8_t response_info = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 7);
          uint8_t reason_code = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[23], 8);
          uint8_t ack_res1 = cs_pdu_bits[31];

          uint32_t ack_target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[32], 24);
          uint32_t ack_source = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);

          static const char * const ack_class_str[4] = {"NACK", "ACK", "QACK", "WACK"};
          uint8_t ack_type = (reason_code >> 6) & 0x03;
          const char * reason_str = dmr_t3_ack_reason_str (reason_code);

          fprintf (stderr, "[%s] Response: %02X; Reason: %02X",
                   ack_class_str[ack_type], response_info, reason_code);
          if (reason_str != NULL)
            fprintf (stderr, " (%s)", reason_str);
          fprintf (stderr, "; ");
          if (ack_res1) fprintf (stderr, " Res: %d ", ack_res1);
          fprintf (stderr, "Target: %d; Source: %d; ", ack_target, ack_source);

          //check the source and/or target for special gateway identifiers
          dmr_gateway_identifier (ack_source, ack_target);

        }
      }

      if (csbk_fid == 0x10 && csbk_o == 34)
      {
        uint16_t cm_lpcn[2];
        uint16_t cm_tg[2][2];
        for (int e = 0; e < 2; e++)
        {
          int base = 16 + (e * 32);
          cm_lpcn[e]  = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[base], 12);
          cm_tg[e][0] = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[base + 12], 10);
          cm_tg[e][1] = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[base + 22], 10);
        }

        s_capmax_cc_seen = 1;
        s_capmax_status_slot = (int)(state->currentslot & 1);
        s_capmax_slot_aloha_cnt = 0;
        s_capmax_non_status_csbks = 0;

        sprintf (state->dmr_branding, "%s", "Motorola");

        if (opts->use_rigctl == 1 && opts->p25_is_tuned == 0)
        {
          ccfreq = trunk_get_current_freq (opts, state);
          trunk_set_cc_freq_if_valid (opts, state, ccfreq);
        }
        if (opts->audio_in_type == 3 && opts->p25_is_tuned == 0)
        {
          ccfreq = (long int)opts->rtlsdr_center_freq;
          trunk_set_cc_freq_if_valid (opts, state, ccfreq);
        }

        fprintf (stderr, "\n");
        fprintf (stderr, " Capacity Max Channel Status -");
        if (cm_lpcn[0] == 0 && cm_lpcn[1] == 0)
        {
          fprintf (stderr, " Idle;");
        }
        else
        {
          for (int e = 0; e < 2; e++)
          {
            if (cm_lpcn[e] == 0)
              continue;
            fprintf (stderr, " LPCN: %04u", cm_lpcn[e]);
            for (int s = 0; s < 2; s++)
            {
              if (cm_tg[e][s] != 0)
                fprintf (stderr, " TS%d: %u", s + 1, cm_tg[e][s]);
              else
                fprintf (stderr, " TS%d: Idle", s + 1);
            }
            fprintf (stderr, ";");
          }
        }

        for (int e = 0; e < 2; e++)
        {
          if (cm_lpcn[e] == 0)
            continue;
          for (int s = 0; s < 2; s++)
          {
            if (cm_tg[e][s] != 0)
            {
              int recently_cleared = ((now - s_last_pclear_time) <= 2 &&
                                      cm_lpcn[e] == s_last_pclear_lpcn &&
                                      (s_last_pclear_slot == (uint8_t)s || s_last_pclear_slot == 2));
              if (!recently_cleared)
                dmr_t3_active_set (state, cm_lpcn[e], (uint8_t)s, "Group", cm_tg[e][s], 0, now, 0);
            }
            else
            {
              dmr_t3_active_clear_slot_lpcn (state, cm_lpcn[e], (uint8_t)s, now);
            }
          }
        }

        int cur_full = (cm_lpcn[0] != 0 && cm_lpcn[1] != 0);
        for (int i = 0; i < DMR_T3_ACTIVE_MAX; i++)
        {
          if (state->dmr_t3_active_chan[i] == 0)
            continue;
          uint16_t ch = state->dmr_t3_active_chan[i];
          if (ch != cm_lpcn[0] && ch != cm_lpcn[1])
          {
            if (!s_t3_active_cm_seen[i] && (now - s_t3_active_grant_time[i]) <= 1)
              continue;
            if (!cur_full)
            {
              dmr_t3_active_clear_entry (state, i);
            }
            else
            {
              s_t3_active_cm_miss[i]++;
              if (s_t3_active_cm_miss[i] >= 3 || (now - state->dmr_t3_active_time[i]) > 1)
                dmr_t3_active_clear_entry (state, i);
            }
          }
        }

        if (cm_lpcn[0] != 0 || cm_lpcn[1] != 0)
          state->last_active_time = now;

        int cm_did_tune = 0;
        if (opts->trunk_tune_group_calls == 1 && (cm_lpcn[0] != 0 || cm_lpcn[1] != 0))
        {
          if (state->tg_hold != 0)
          {
            for (int e = 0; e < 2; e++)
            {
              if (cm_lpcn[e] == 0)
                continue;
              for (int s = 0; s < 2; s++)
              {
                if (cm_tg[e][s] != 0 && state->tg_hold == cm_tg[e][s])
                {
                  long int hfreq = state->trunk_chan_map[cm_lpcn[e]];
                  long int cur_f = trunk_get_current_freq(opts, state);
                  if (hfreq != 0 && cur_f != hfreq)
                    state->last_vc_sync_time = 0;
                }
              }
            }
          }

          if ((now - state->last_vc_sync_time) > 2)
          {
            opts->p25_is_tuned = 0;
            state->p25_vc_freq[0] = 0;
            state->p25_vc_freq[1] = 0;

            uint16_t tune_lpcn = 0;
            uint8_t  tune_slot = 0;
            uint16_t tune_tg   = 0;
            long int tune_freq = 0;

            for (int pass = 0; pass < 2 && tune_freq == 0; pass++)
            {
              if (pass == 0 && state->tg_hold == 0)
                continue;
              for (int e = 0; e < 2 && tune_freq == 0; e++)
              {
                if (cm_lpcn[e] == 0)
                  continue;
                long int f = state->trunk_chan_map[cm_lpcn[e]];
                if (f == 0)
                  continue;
                for (int s = 0; s < 2 && tune_freq == 0; s++)
                {
                  uint16_t tg = cm_tg[e][s];
                  if (tg == 0)
                    continue;
                  if (pass == 0 && tg != state->tg_hold)
                    continue;
                  if ((now - s_last_pclear_time) <= 2 &&
                      cm_lpcn[e] == s_last_pclear_lpcn &&
                      (s_last_pclear_slot == (uint8_t)s || s_last_pclear_slot == 2))
                    continue;
                  if ((now - s_last_cm_tune_time) <= 5 &&
                      cm_lpcn[e] == s_last_cm_tune_lpcn &&
                      s_last_cm_tune_slot == (uint8_t)s &&
                      s_last_cm_tune_tg == tg)
                    continue;

                  char mode[8];
                  sprintf (mode, "%s", (opts->trunk_use_allow_list == 1) ? "B" : "");
                  for (int i = 0; i < state->group_tally; i++)
                  {
                    if (state->group_array[i].groupNumber == tg)
                    {
                      fprintf (stderr, " [%s]", state->group_array[i].groupName);
                      strcpy (mode, state->group_array[i].groupMode);
                      break;
                    }
                  }
                  if (state->tg_hold != 0 && state->tg_hold != tg) sprintf (mode, "%s", "B");
                  if (state->tg_hold != 0 && state->tg_hold == tg) sprintf (mode, "%s", "A");

                  if (strcmp(mode, "B") != 0 && strcmp(mode, "DE") != 0)
                  {
                    tune_lpcn = cm_lpcn[e];
                    tune_slot = (uint8_t)s;
                    tune_tg   = tg;
                    tune_freq = f;
                  }
                }
              }
            }

            if (tune_freq != 0)
            {
              uint32_t known_src = dmr_t3_active_get_src (state, tune_lpcn, tune_slot, tune_tg, now);
              long int cur_f = trunk_get_current_freq(opts, state);
              if (cur_f == 0 && opts->p25_is_tuned == 0)
                cur_f = state->p25_cc_freq;
              int same_cc_chan = (opts->p25_is_tuned == 0 && tune_lpcn != 0 &&
                                  tune_lpcn == (uint16_t)trunk_get_t3_cc_lpcn(state));
              int slot_busy = (tune_slot == 0)
                                ? (state->dmrburstL == 16 || state->dmrburstL == 0 || state->dmrburstL == 1 ||
                                   state->dmrburstL == 2 || state->dmr_hangtime[0] ||
                                   state->lasttg != 0 || state->lastsrc != 0)
                                : (state->dmrburstR == 16 || state->dmrburstR == 0 || state->dmrburstR == 1 ||
                                   state->dmrburstR == 2 || state->dmr_hangtime[1] ||
                                   state->lasttgR != 0 || state->lastsrcR != 0);
              if (!slot_busy && ((cur_f != 0 && tune_freq == cur_f) || same_cc_chan))
              {
                dmr_t3_grant_init_slot (state, tune_slot, 0, 49, 0, 0, 0, tune_tg, known_src);
                dmr_t3_active_mark_logged (state, tune_lpcn, tune_slot, tune_tg);
              }

              if (opts->p25_trunk == 0)
              {
                state->p25_vc_freq[0] = tune_freq;
                state->p25_vc_freq[1] = tune_freq;
                trunk_set_t3_vc_lpcn(tune_lpcn);
              }
              else if (state->p25_cc_freq != 0)
              {
                if (opts->use_rigctl == 1)
                {
                  long int cur_freq = trunk_get_current_freq(opts, state);
                  if (cur_freq != tune_freq)
                  {
                    dmr_reset_blocks (opts, state);
                    dmr_csbk_clear_both_slots (state);
                    if (opts->setmod_bw != 0) SetModulation(opts->rigctl_sockfd, opts->setmod_bw);
                    SetFreq(opts->rigctl_sockfd, tune_freq);
                  }
                  else
                  {
                    trunk_update_filter_for_freq(tune_freq);
                  }
                  dmr_t3_grant_init_slot (state, tune_slot, 0, 49, 0, 0, 0, tune_tg, known_src);
                  dmr_t3_active_mark_logged (state, tune_lpcn, tune_slot, tune_tg);
                  state->p25_vc_freq[0] = state->p25_vc_freq[1] = tune_freq;
                  trunk_set_t3_vc_lpcn(tune_lpcn);
                  opts->p25_is_tuned = 1;
                  state->last_vc_sync_time = now;
                  state->last_t3_tune_time = now;
                  s_last_cm_tune_lpcn = tune_lpcn;
                  s_last_cm_tune_slot = tune_slot;
                  s_last_cm_tune_tg   = tune_tg;
                  s_last_cm_tune_time = now;
                  cm_did_tune = 1;
                }
                else if (opts->audio_in_type == 3)
                {
                  #ifdef USE_RTLSDR
                  uint32_t tempf = (uint32_t)tune_freq;
                  if (opts->rtlsdr_center_freq != tempf)
                  {
                    dmr_reset_blocks (opts, state);
                    dmr_csbk_clear_both_slots (state);
                    rtl_dev_tune (opts, tune_freq);
                  }
                  else
                  {
                    trunk_update_filter_for_freq(tune_freq);
                  }
                  dmr_t3_grant_init_slot (state, tune_slot, 0, 49, 0, 0, 0, tune_tg, known_src);
                  dmr_t3_active_mark_logged (state, tune_lpcn, tune_slot, tune_tg);
                  state->p25_vc_freq[0] = state->p25_vc_freq[1] = tune_freq;
                  trunk_set_t3_vc_lpcn(tune_lpcn);
                  opts->p25_is_tuned = 1;
                  state->last_vc_sync_time = now;
                  state->last_t3_tune_time = now;
                  s_last_cm_tune_lpcn = tune_lpcn;
                  s_last_cm_tune_slot = tune_slot;
                  s_last_cm_tune_tg   = tune_tg;
                  s_last_cm_tune_time = now;
                  cm_did_tune = 1;
                  #endif
                }
              }
            }
          }
        }

        if (cm_did_tune)
        {
          dmr_active_channel_reset_rows (state);
        }
        else
        {
          long int cur_f = trunk_get_current_freq(opts, state);
          if (cur_f == 0 && opts->p25_is_tuned == 0)
            cur_f = state->p25_cc_freq;
          uint16_t cc_lpcn = (uint16_t)trunk_get_t3_cc_lpcn(state);
          for (int e = 0; e < 2; e++)
          {
            if (cm_lpcn[e] == 0)
              continue;
            long int f = state->trunk_chan_map[cm_lpcn[e]];
            int on_cm_freq = ((cur_f != 0 && f != 0 && f == cur_f) ||
                              (opts->p25_is_tuned == 0 && cc_lpcn != 0 && cm_lpcn[e] == cc_lpcn));
            for (int s = 0; s < 2; s++)
            {
              if (cm_tg[e][s] == 0)
                continue;
              int recently_cleared = ((now - s_last_pclear_time) <= 2 &&
                                      cm_lpcn[e] == s_last_pclear_lpcn &&
                                      (s_last_pclear_slot == (uint8_t)s || s_last_pclear_slot == 2));
              if (recently_cleared)
                continue;
              if (on_cm_freq)
              {
                dmr_t3_active_mark_logged (state, cm_lpcn[e], (uint8_t)s, cm_tg[e][s]);
              }
              else if (!dmr_t3_active_is_logged (state, cm_lpcn[e], (uint8_t)s, cm_tg[e][s]))
              {
                if (dmr_log_t3_grant_event (opts, state, (uint8_t)s, cm_lpcn[e], 49, 0, 0, cm_tg[e][s], now))
                  dmr_t3_active_mark_logged (state, cm_lpcn[e], (uint8_t)s, cm_tg[e][s]);
              }
            }
          }
        }
      }

      //tier 2 csbks
      if (csbk_o == 4)
      {
        fprintf (stderr, "\n");
        fprintf (stderr, " Unit to Unit Voice Service Request (UU_V_Req) - ");

        uint8_t uu_so = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 7);
        uint32_t target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[32], 24);
        uint32_t source = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);
        fprintf (stderr, "Target [%d] - Source [%d] ", target, source);
        if (uu_so != 0)
        {
          fprintf (stderr, "SVC: %02X; ", uu_so);
          dmr_print_svc_opt_flags (uu_so, NULL, 0);
        }
      }

      if (csbk_o == 5)
      {
        fprintf (stderr, "\n");
        fprintf (stderr, " Unit to Unit Voice Service Answer Response (UU_Ans_Req) - ");

        uint8_t uu_so = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 7);
        uint8_t ans_rsp = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[24], 8);
        uint32_t target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[32], 24);
        uint32_t source = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);
        const char * ans_str = (ans_rsp == 0x20) ? "Proceed" :
                               (ans_rsp == 0x21) ? "Deny" :
                               (ans_rsp == 0x22) ? "Wait" : "Unknown";
        fprintf (stderr, "Target [%d] - Source [%d] - Answer: %s (%02X); ",
                 target, source, ans_str, ans_rsp);
        if (uu_so != 0)
        {
          fprintf (stderr, "SVC: %02X; ", uu_so);
          dmr_print_svc_opt_flags (uu_so, NULL, 0);
        }

      }

      if (csbk_o == 7)
      {
        fprintf (stderr, "\n");
        fprintf (stderr, " Channel Timing CSBK (CT_CSBK) ");
      }

      if (csbk_o == 38)
      {
        fprintf (stderr, "\n");
        fprintf (stderr, " Negative Acknowledgement Response (NACK_Rsp) - ");

        uint8_t src_type = cs_pdu_bits[17];
        uint8_t svc_type = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[18], 6);
        uint8_t nack_reason = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[24], 8);
        uint32_t target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[32], 24);
        uint32_t source = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);
        fprintf (stderr, "Target [%d] - Source [%d] - SrcType: %s; SvcType: %02X; Reason: %02X; ",
                 target, source, src_type ? "MS" : "BS", svc_type, nack_reason);

      }

      if (csbk_o == 56 && state->synctype == 33) //only run on MS Data sync pattern
      {
        fprintf (stderr, "\n");
        fprintf (stderr, " BS Outbound Activation (BS_Dwn_Act) - ");
        //Inbound CSBK only from MS source to 'wake' the repeater up (best that I understand)
        uint32_t target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[32], 24);
        uint32_t source = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);
        fprintf (stderr, "Target [%d] - Source [%d] ", target, source);
      }

      if (csbk_o == 61)
      {
        fprintf (stderr, "\n");
        fprintf (stderr, " Preamble CSBK - ");
        uint8_t content = cs_pdu_bits[16];
        uint8_t gi = cs_pdu_bits[17];
        uint8_t res = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[18], 6);
        uint8_t blocks = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[24], 8);
        uint32_t target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[32], 24);
        uint32_t source = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);
        UNUSED(res);

        uint8_t target_hash[24];
        uint8_t tg_hash = 0;

        if (gi == 0) fprintf (stderr, "Individual ");
        else fprintf (stderr, "Group ");

        if (content == 0) fprintf (stderr, "CSBK - ");
        else fprintf (stderr, "Data - ");

        // check to see if this is XPT
        if (strcmp (state->dmr_branding_sub, "XPT ") == 0)
        {
          //get 16-bit truncated target and source ids
          target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[40], 16); //40, or 32?
          source = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[64], 16); //56, or 64?

          //check to see if this is indiv data (private) then we will need to report the hash value as well
          if (gi == 0)
          {
            //the crc8 hash is the value represented in the Hytera XPT Site Status CSBK when dealing with indiv data
            for (int i = 0; i < 16; i++) target_hash[i] = cs_pdu_bits[40+i];
            tg_hash = crc8 (target_hash, 16);
            fprintf (stderr, "Source: %d - Target: %d - Hash: %d ", source, target, tg_hash);
          }
          else fprintf (stderr, "Source: %d - Target: %d ", source, target);

        }
        // check to see if this is Cap+
        else if (strcmp (state->dmr_branding_sub, "Cap+ ") == 0)
        {
          //truncate tg on group? or just on private/individual data?
          if (gi == 0) target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[40], 16);
          source = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[64], 16);
          int rest = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[60], 4);
          fprintf (stderr, "Source: %d - Target: %d - Rest LSN: %d ", source, target, rest);
          if (rest > 0)
          {
            state->dmr_rest_channel = rest;
            if (state->trunk_chan_map[rest] != 0)
              state->p25_cc_freq = state->trunk_chan_map[rest];
          }
        }
        else fprintf (stderr, "Source: %d - Target: %d ", source, target);

        fprintf (stderr, "Blocks: %d ", blocks);

        //if trunking and data calls allowed, convert preamble burst to a DATA header burst to hold p_clear or con+
        if (opts->p25_trunk == 1 && opts->trunk_tune_data_calls == 1)
        {
          if (state->currentslot == 0) state->dmrburstL = 6;
          else state->dmrburstR = 6;
        }

      }
      //end tier 2 csbks

    }

    //Reworked Cap+ for Multi Block FL PDU and Private/Data Calls
    //experimental, but seems to be working well
    if (csbk_fid == 0x10)
    {

      //Cap+ Something
      if (csbk_o == 0x3A)
      {
        //initial line break
        fprintf (stderr, "\n");
        fprintf (stderr, "%s", KYEL);

        uint8_t fl = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 2);
        uint8_t ts = cs_pdu_bits[18];
        uint8_t res = cs_pdu_bits[19];
        uint8_t rest_channel = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[20], 4);
        fprintf (stderr, " Capacity Plus CSBK 0x3A - FL: %d TS: %d RS: %d - Rest LSN: %d ", fl, ts, res, rest_channel);
        if (rest_channel > 0)
        {
          state->dmr_rest_channel = rest_channel;
          state->dmr_mfid = 0x10;
          sprintf (state->dmr_branding, "%s", "Motorola");
          sprintf (state->dmr_branding_sub, "%s", "Cap+ ");
          if (state->trunk_chan_map[rest_channel] != 0)
            state->p25_cc_freq = state->trunk_chan_map[rest_channel];
        }

        // 01:15:08 Sync: +DMR   slot1  [slot2] | Color Code=03 | CSBK
        // Capacity Plus Channel Status - FL: 3 TS: 1 RS: 0 - Rest Channel 1 - Single Block
        //   Ch1: Rest Ch2: Idle Ch3: Idle Ch4: Idle
        //   Ch5: Idle Ch6: Idle Ch7: Idle Ch8: Idle
        // DMR PDU Payload [BE][10][E1][00][00][00][00][00][00][00][4E][15]
        // 01:15:08 Sync: +DMR  [slot1]  slot2  | Color Code=03 | CSBK

        //FL, TS, and Rest Channel seem to be in same location as the Channel Status CSBK
        //other values are currently unknown
        // DMR PDU Payload [BA][10][C1][3B][61][11][51][00][00][00][3D][D6]
        // 01:15:08 Sync: +DMR   slot1  [slot2] | Color Code=03 | CSBK
        // DMR PDU Payload [BA][10][E1][3B][61][11][51][00][00][00][46][BE]

      }

      //Cap+ Adjacent Sites
      if (csbk_o == 0x3B)
      {

        //initial line break
        fprintf (stderr, "\n");
        fprintf (stderr, "%s", KYEL);

        fprintf (stderr, " Capacity Plus Adjacent Sites\n  ");

        uint8_t nl[6]; //adjacet site numerical value
        uint8_t nr[6]; //adjacent site current rest channel
        memset (nl, 0, sizeof (nl));
        memset (nr, 0, sizeof (nr));

        state->dmr_t3_syscode = 0;
        state->dmr_neighbor_count = 0;
        for (int i = 0; i < 6; i++)
        {
          nl[i] = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[32+(i*8)], 4);
          nr[i] = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[36+(i*8)], 4);
          if (nl[i])
          {
            fprintf (stderr, "Site: %d Rest: %d; ", nl[i], nr[i]);
            dmr_add_neighbor (state, nl[i]);
          }
        }
        state->last_active_time = time(NULL);

      }

      //Cap+ Channel Status -- Expanded for up to 16 LSNs and private voice/data calls and dual slot csbk_bit storage
      if (csbk_o == 0x3E)
      {

        //initial line break
        fprintf (stderr, "\n");
        fprintf (stderr, "%s", KYEL);

        uint8_t fl = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 2);
        uint8_t ts = cs_pdu_bits[18];  //timeslot this PDU occurs in
        uint8_t res = cs_pdu_bits[19]; //unknown or unused bit value
        uint8_t rest_channel = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[20], 4);
        uint8_t group_tally = 0; //set this to the number of active channels tallied
        uint8_t bank_one = 0;
        uint8_t bank_two = 0;
        uint8_t pbank_one = 0;
        uint8_t pbank_two = 0;
        uint8_t block_num = state->cap_plus_block_num[ts];
        uint8_t pdflag = 0;
        uint8_t start = 8;
        uint8_t end = 8;
        uint8_t fl_bytes = 0;
        uint8_t ch[24]; //one bit per channel
        uint8_t pch[24]; //private or data call channel bits
        uint16_t tg = 0;
        int i, j;
        //tg and channel info for trunking purposes
        uint16_t g_tg[16];
        uint16_t p_tg[16];
        uint16_t t_tg[24];
        uint16_t empty[24]; //used to evaluate whether or not all channels are idle or not with memcmp after loading
        int cap_complete = 0;
        int hdr_ok = 1;
        const int cap_max_bits = (int)sizeof(state->cap_plus_csbk_bits[ts]);

        //sanity check
        if (block_num > 6)
        {
          state->cap_plus_block_num[ts] = 6;
          block_num = 6;
        }

        //init some arrays
        memset (g_tg, 0, sizeof(g_tg));
        memset (p_tg, 0, sizeof(p_tg));
        memset (t_tg, 0, sizeof(t_tg));
        memset (ch, 0, sizeof(ch));
        memset (pch, 0, sizeof(pch));
        memset (empty, 0, sizeof(empty));

        //treating FL as a form of LCSS
        if (fl == 2 || fl == 3) //initial or single block (fl2 or fl3)
        {
          //NOTE: this has been changed to store per slot
          memset (state->cap_plus_csbk_bits[ts], 0, sizeof(state->cap_plus_csbk_bits[ts]));
          for (i = 0; i < 10*8; i++) state->cap_plus_csbk_bits[ts][i] = cs_pdu_bits[i];
          state->cap_plus_block_num[ts] = 0;
          block_num = 0;
          s_cap_mbc_valid[ts] = (fl == 2) ? 1 : 0;
          cap_complete = (fl == 3) ? 1 : 0;
        }
        else //appended block (fl 0) or final block (fl 1)
        {
          hdr_ok = (s_cap_mbc_valid[ts] == 1 &&
                    (uint16_t)ConvertBitIntoBytes(&state->cap_plus_csbk_bits[ts][0], 16) == 0xBE10);
          if (hdr_ok && (80 + (7*8*block_num) + 56) <= cap_max_bits)
          {
            for (i = 0; i < 7*8; i++) state->cap_plus_csbk_bits[ts][i+80+(7*8*block_num)] = cs_pdu_bits[i+24];
            block_num++;
            state->cap_plus_block_num[ts]++;
            if (fl == 1)
            {
              s_cap_mbc_valid[ts] = 0;
              cap_complete = 1;
            }
          }
          else
          {
            if (fl == 1)
            {
              s_cap_mbc_valid[ts] = 0;
              memset (state->cap_plus_csbk_bits[ts], 0, sizeof(state->cap_plus_csbk_bits[ts]));
              state->cap_plus_block_num[ts] = 0;
            }
            cap_complete = 0;
          }
        }

        //move assignment until later when evaluating for a p_clear condition
        static long int s_cap_cur_freq[2] = {0, 0};
        if (fl == 2 || fl == 3)
          s_cap_cur_freq[ts & 1] = trunk_get_current_freq(opts, state);
        if (rest_channel > 0)
        {
          if (rest_channel != state->dmr_rest_channel)
            state->dmr_rest_channel = rest_channel;

          //assign to cc freq to follow during no sync
          if (state->trunk_chan_map[rest_channel] != 0)
            state->p25_cc_freq = state->trunk_chan_map[rest_channel];
        }

        fprintf (stderr, " Capacity Plus Channel Status - FL: %d TS: %d RS: %d - Rest LSN: %d", fl, ts, res, rest_channel);
        if (fl == 0) fprintf (stderr, " - Appended Block"); //have not yet observed a system use this fl value
        if (fl == 1) fprintf (stderr, " - Final Block");
        if (fl == 2) fprintf (stderr, " - Initial Block");
        if (fl == 3) fprintf (stderr, " - Single Block");
        if ((fl == 0 || fl == 1) && !hdr_ok) fprintf (stderr, " (Orphaned)");

        if (cap_complete)
        {
          int bit_pos = 24;

          //look at each group channel bit -- run this PRIOR to checking private or data calls
          //or else we will overwrite the ch bits we set below for private calls with zeroes
          //these also seem to indicate data calls -- can't find a distinction of which is which
          bank_one = (uint8_t)ConvertBitIntoBytes(&state->cap_plus_csbk_bits[ts][bit_pos], 8);
          bit_pos += 8;
          for (i = 0; i < 8; i++)
          {
            ch[i] = state->cap_plus_csbk_bits[ts][24+i];
            if (ch[i] == 1)
            {
              if (bit_pos + 8 <= cap_max_bits)
                g_tg[i] = (uint16_t)ConvertBitIntoBytes(&state->cap_plus_csbk_bits[ts][bit_pos], 8);
              bit_pos += 8;
              group_tally++;
            }
          }

          //Expanded to cover larger systems up to 16 slots.
          int b2_map_pos = bit_pos;
          bank_two = (b2_map_pos + 8 <= cap_max_bits)
                   ? (uint8_t)ConvertBitIntoBytes(&state->cap_plus_csbk_bits[ts][b2_map_pos], 8)
                   : 0;
          bit_pos += 8;
          if (bank_two)
          {
            for (i = 0; i < 8; i++)
            {
              ch[i+8] = state->cap_plus_csbk_bits[ts][b2_map_pos+i];
              if (ch[i+8] == 1)
              {
                if (bit_pos + 8 <= cap_max_bits)
                  g_tg[i+8] = (uint16_t)ConvertBitIntoBytes(&state->cap_plus_csbk_bits[ts][bit_pos], 8);
                bit_pos += 8;
                group_tally++;
              }
            }
          }

          //check flag for appended private and/or data calls
          pdflag = (bit_pos + 8 <= cap_max_bits)
                 ? (uint8_t)ConvertBitIntoBytes(&state->cap_plus_csbk_bits[ts][bit_pos], 8)
                 : 0;
          bit_pos += 8;

          if (pdflag != 0)
          {
            //check for private activity on LSNs 1-8
            int p1_map_pos = bit_pos;
            pbank_one = (p1_map_pos + 8 <= cap_max_bits)
                      ? (uint8_t)ConvertBitIntoBytes(&state->cap_plus_csbk_bits[ts][p1_map_pos], 8)
                      : 0;
            bit_pos += 8;
            if (pbank_one)
            {
              fprintf (stderr, "\n Bank One F%02X Private or Data Call(s) - ", pdflag);
              for (i = 0; i < 8; i++)
              {
                pch[i] = state->cap_plus_csbk_bits[ts][p1_map_pos+i];
                if (pch[i] == 1)
                {
                  p_tg[i] = (bit_pos + 16 <= cap_max_bits)
                          ? (uint16_t)ConvertBitIntoBytes(&state->cap_plus_csbk_bits[ts][bit_pos], 16)
                          : 0;
                  bit_pos += 16;
                  fprintf (stderr, " LSN %02d: TGT %d;", i+1, p_tg[i]);
                }
              }
            }

            //check for private activity on LSNs 9-16 (directly follows Bank 1 targets)
            int p2_map_pos = bit_pos;
            pbank_two = (p2_map_pos + 8 <= cap_max_bits)
                      ? (uint8_t)ConvertBitIntoBytes(&state->cap_plus_csbk_bits[ts][p2_map_pos], 8)
                      : 0;
            bit_pos += 8;
            if (pbank_two)
            {
              fprintf (stderr, "\n Bank Two F%02X Private or Data Call(s) - ", pdflag);
              for (i = 0; i < 8; i++)
              {
                pch[i+8] = state->cap_plus_csbk_bits[ts][p2_map_pos+i];
                if (pch[i+8] == 1)
                {
                  p_tg[i+8] = (bit_pos + 16 <= cap_max_bits)
                            ? (uint16_t)ConvertBitIntoBytes(&state->cap_plus_csbk_bits[ts][bit_pos], 16)
                            : 0;
                  bit_pos += 16;
                  fprintf (stderr, " LSN %02d: TGT %d;", i+9, p_tg[i+8]);
                }
              }
            }
          }

          fprintf (stderr, "\n  ");

          uint8_t disp_b1 = bank_one | pbank_one;
          uint8_t disp_b2 = bank_two | pbank_two;

          //NOTE: New start and end positions will allow the status print
          //to contract and expand as needed to prevent this from
          //being a four liner CSBK when there is little or no activity

          //start position;
          start = 0;
          if (disp_b1 & 0xF0) start = 0;
          else if (rest_channel < 5) start = 0;

          else if (disp_b1 & 0xF) start = 4;
          else if (rest_channel > 4 && rest_channel < 9) start = 4;

          else if (disp_b2 & 0xF0) start = 8;
          else if (rest_channel > 8 && rest_channel < 13) start = 8;

          else if (disp_b2 & 0xF) start = 12;
          else if (rest_channel > 12) start = 12;

          //end position;
          end = 16;
          if (disp_b2 & 0xF) end = 16;
          else if (rest_channel > 12) end = 16;

          else if (disp_b2 & 0xF0) end = 12;
          else if (rest_channel > 8 && rest_channel < 13) end = 12;

          else if (disp_b1 & 0xF) end = 8;
          else if (rest_channel > 4 && rest_channel < 9) end = 8;

          else if (disp_b1 & 0xF0) end = 4;
          else if (rest_channel < 5) end = 4;

          for (i = 0; i < 16; i++)
          {
            if (ch[i] == 1 && opts->trunk_tune_group_calls == 1)
              t_tg[i] = g_tg[i];
            else if (pch[i] == 1 && opts->trunk_tune_private_calls == 1)
              t_tg[i] = p_tg[i];
          }

          //start parsing info and listing active LSNs
          for (i = start; i < end; i++)
          {
            if (start < 1 && i == 4)
              fprintf (stderr, "\n  ");

            if (start < 5 && i == 8)
              fprintf (stderr, "\n  ");

            if (start < 9 && i == 12)
              fprintf (stderr, "\n  ");

            fprintf (stderr, "LSN %02d: ", i+1);
            if (ch[i] == 1) //group voice channels
            {
              tg = g_tg[i];
              if (tg != 0) fprintf (stderr, "%5d;  ", tg);
              else fprintf (stderr, "Group;  ");
            }
            else if (pch[i] == 1) //private or data channels
            {
              tg = p_tg[i];
              if (tg != 0) fprintf (stderr, "%5d;  ", tg);
              else fprintf (stderr, " P||D;  ");
            }
            else if (i+1 == rest_channel) fprintf (stderr, " Rest;  ");
            else fprintf (stderr, " Idle;  ");
          }

          //update active channels in-place with debounce so brief TS0/TS1 skew or short frames don't blink
          if (!s_cap_active_present)
          {
            dmr_active_channel_reset_rows (state);
            s_cap_active_present = 1;
          }
          state->last_active_time = time(NULL);

          struct timespec cap_now_ts = {0, 0};
          int have_cap_ts = (clock_gettime(CLOCK_MONOTONIC, &cap_now_ts) == 0);

          for (i = 0; i < 16; i++)
          {
            int row = i + 1;
            int show_row = 0;

            if (ch[i] == 1)
            {
              s_cap_priv_active[row] = 0;
              s_cap_priv_tg[row] = 0;
              s_cap_priv_miss[row] = 0;
              show_row = 1;
            }
            else if (pch[i] == 1)
            {
              s_cap_priv_miss[row] = 0;
              if (!s_cap_priv_active[row] || s_cap_priv_tg[row] != p_tg[i])
              {
                s_cap_priv_active[row] = 1;
                s_cap_priv_tg[row] = p_tg[i];
                s_cap_priv_start[row] = cap_now_ts;
              }
              else if (have_cap_ts)
              {
                long elapsed_ms = (cap_now_ts.tv_sec - s_cap_priv_start[row].tv_sec) * 1000L
                                + (cap_now_ts.tv_nsec - s_cap_priv_start[row].tv_nsec) / 1000000L;
                if (elapsed_ms >= DMR_CAP_PRIV_MIN_MS)
                  show_row = 1;
              }
              else
              {
                show_row = 1;
              }
            }
            else if (s_cap_priv_active[row])
            {
              s_cap_priv_miss[row]++;
              if (s_cap_priv_miss[row] >= 3)
              {
                s_cap_priv_active[row] = 0;
                s_cap_priv_tg[row] = 0;
                s_cap_priv_miss[row] = 0;
              }
            }

            if (show_row)
            {
              uint8_t is_group = (ch[i] == 1);
              uint16_t act_tg = is_group ? g_tg[i] : p_tg[i];
              const char * alias = NULL;
              if (act_tg != 0)
              {
                for (int g = 0; g < state->group_tally; g++)
                {
                  if (state->group_array[g].groupNumber == act_tg &&
                      state->group_array[g].groupName[0] != '\0')
                  {
                    alias = state->group_array[g].groupName;
                    break;
                  }
                }
              }
              if (act_tg != 0)
              {
                if (alias != NULL)
                  snprintf (state->active_channel[row], sizeof(state->active_channel[row]),
                            "Active Ch: LSN %2d %-9s TGT: %u [%.64s]",
                            row, is_group ? "Group" : "Private", act_tg, alias);
                else
                  snprintf (state->active_channel[row], sizeof(state->active_channel[row]),
                            "Active Ch: LSN %2d %-9s TGT: %u",
                            row, is_group ? "Group" : "Private", act_tg);
              }
              else
              {
                snprintf (state->active_channel[row], sizeof(state->active_channel[row]),
                          "Active Ch: LSN %2d %s", row, is_group ? "Group" : "Priv/Data");
              }
              s_cap_active_miss[row] = 0;
            }
            else if (state->active_channel[row][0] != '\0')
            {
              s_cap_active_miss[row]++;
              if (s_cap_active_miss[row] >= DMR_CAP_ACTIVE_MISS_LIMIT)
              {
                state->active_channel[row][0] = '\0';
                s_cap_active_miss[row] = 0;
              }
            }
          }


          state->dmr_mfid = 0x10;
          sprintf (state->dmr_branding, "%s", "Motorola");
          sprintf (state->dmr_branding_sub, "%s", "Cap+ ");

          //nullify any previous non-Cap+ site_parm data
          if (strncmp(state->dmr_site_parms, "Site ", 5) != 0)
            sprintf(state->dmr_site_parms, "%s", "");

          fprintf (stderr, "%s", KNRM);

          //Skip tuning group calls if group calls are disabled -- moved to individual switches in the parsing phase
          // if (opts->trunk_tune_group_calls == 0) goto SKIPCAP;

          //Test allowing a group in the white list to preempt a call in progress and tune to a white listed call
          if (opts->trunk_use_allow_list == 1) state->last_vc_sync_time = 0;

          //Test allowing a tg hold to pre-empt a call in progress and tune to the hold TG
          if (state->tg_hold != 0) state->last_vc_sync_time = 0;

          //if no activity in this window
          if ( (time(NULL) - state->last_vc_sync_time) > 2 ) //may use last_cc_sync_time instead
            rotate_symbol_out_file(opts, state);

          //TODO: Consider a method to allow moving the frequency to the rest channel
          //when a TG hold is specified but nether slot carries the TG on Hold;
          //CODED: using slow link control for that

          //filter blocked/locked-out talkgroups so they do not count as busy or prevent returning to Rest LSN
          for (j = 0; j < 16; j++)
          {
            if (t_tg[j] == 0) continue;

            char mode[8]; //allow, block, digital, enc, etc
            sprintf (mode, "%s", "");

            //if we are using allow/whitelist mode, then write 'B' to mode for block
            //comparison below will look for an 'A' to write to mode if it is allowed
            if (opts->trunk_use_allow_list == 1) sprintf (mode, "%s", "B");

            for (int i = 0; i < state->group_tally; i++)
            {
              if (state->group_array[i].groupNumber == t_tg[j])
              {
                fprintf (stderr, " [%s]", state->group_array[i].groupName);
                strcpy (mode, state->group_array[i].groupMode);
                break;
              }
            }

            //TG hold on DMR Cap+ -- block non-matching target, allow matching target
            if (state->tg_hold != 0 && state->tg_hold != t_tg[j]) sprintf (mode, "%s", "B");
            if (state->tg_hold != 0 && state->tg_hold == t_tg[j]) sprintf (mode, "%s", "A");

            if (strcmp(mode, "B") == 0 || strcmp(mode, "DE") == 0)
              t_tg[j] = 0;
          }

          //don't tune if vc on the current channel
          if ( (time(NULL) - state->last_vc_sync_time > 2) )
          {
            for (j = 0; j < 16; j++) //go through the channels stored looking for active ones to tune to
            {
              //without priority, this will tune the first one it finds (if group isn't blocked)
              if (t_tg[j] != 0 && state->p25_cc_freq != 0 && opts->p25_trunk == 1)
              {
                //debug print for tuning verification
                // fprintf (stderr, "\n LSN/TG to tune to: %d - %d", j+1, t_tg[j]);

                if (state->trunk_chan_map[j+1] != 0) //if we have a valid frequency
                {
                  //RIGCTL
                  if (opts->use_rigctl == 1)
                  {

                    //may need the code below to TG hold (just in case SLC comes before a VLC or a VC6 EMB and immediately goes to the rest channel)
                    //disable or tweak code below if these are reversed somehow or causes issues
                    if (state->tg_hold != 0)
                    {
                      if ( (j&1) == 0 ) //slot 1 LSN
                      {
                        state->lasttg = t_tg[j];
                        // state->lastsrc = source;
                      }
                      else //slot 2 LSN
                      {
                        state->lasttgR = t_tg[j];
                        // state->lastsrcR = source;
                      }
                      //end TG set on tune
                    }
                    long int cur_freq = trunk_get_current_freq(opts, state);
                    if (cur_freq != state->trunk_chan_map[j+1])
                    {
                      dmr_reset_blocks (opts, state); //reset all block gathering since we are tuning away from current frequency
                      if (opts->setmod_bw != 0 ) SetModulation(opts->rigctl_sockfd, opts->setmod_bw);
                      SetFreq(opts->rigctl_sockfd, state->trunk_chan_map[j+1]);
                    }
                    else
                    {
                      trunk_update_filter_for_freq(state->trunk_chan_map[j+1]);
                    }
                    state->gi[j & 1] = (ch[j] == 1) ? 0 : 1;
                    state->p25_vc_freq[0] = state->p25_vc_freq[1] = state->trunk_chan_map[j+1];
                    opts->p25_is_tuned = 1; //set to 1 to set as currently tuned so we don't keep tuning nonstop
                    state->last_vc_sync_time = time(NULL);
                    j = 16; //break loop
                  }

                  //rtl
                  else if (opts->audio_in_type == 3)
                  {
                    #ifdef USE_RTLSDR

                    //may need the code below to TG hold (just in case SLC comes before a VLC or a VC6 EMB and immediately goes to the rest channel)
                    //disable or tweak code below if these are reversed somehow or causes issues
                    if (state->tg_hold != 0)
                    {
                      if ( (j&1) == 0 ) //slot 1 LSN
                      {
                        state->lasttg = t_tg[j];
                        // state->lastsrc = source;
                      }
                      else //slot 2 LSN
                      {
                        state->lasttgR = t_tg[j];
                        // state->lastsrcR = source;
                      }
                      //end TG set on tune
                    }
                    uint32_t temp = (uint32_t)state->trunk_chan_map[j+1];
                    if (opts->rtlsdr_center_freq != temp)
                    {
                      dmr_reset_blocks (opts, state); //reset all block gathering since we are tuning away from current frequency
                      rtl_dev_tune (opts, state->trunk_chan_map[j+1]); //unlike rigctl, using this actually interrupts signal decodes (rtl_clean_queue)
                      //debug print for tuning verification
                      // fprintf (stderr, "\n RTL LSN/TG to tune to: %d - %d", j+1, t_tg[j]);
                    }
                    else
                    {
                      trunk_update_filter_for_freq(state->trunk_chan_map[j+1]);
                    }
                    // else fprintf (stderr, "\n DONT RTL LSN/TG to tune to: %d - %d", j+1, t_tg[j]); //debug
                    state->gi[j & 1] = (ch[j] == 1) ? 0 : 1;
                    state->p25_vc_freq[0] = state->p25_vc_freq[1] = state->trunk_chan_map[j+1];
                    opts->p25_is_tuned = 1;
                    state->last_vc_sync_time = time(NULL);
                    j = 16; //break loop
                    #endif
                  }

                }
              }

            }
          } //end tuning

          // SKIPCAP: ;
          //debug print
          if (fl == 1 && opts->payload == 1)
          {
            fprintf (stderr, "%s\n", KYEL);
            fprintf (stderr, " CAP+ Multi Block PDU \n  ");
            fl_bytes = 0;
            int total_bytes = 10 + (block_num * 7);
            if (total_bytes > cap_max_bits / 8)
              total_bytes = cap_max_bits / 8;
            for (i = 0; i < total_bytes; i++)
            {
              fl_bytes = (uint8_t)ConvertBitIntoBytes(&state->cap_plus_csbk_bits[ts][i*8], 8);
              fprintf (stderr, "[%02X]", fl_bytes);
              if (i == 17 || i == 35) fprintf (stderr, "\n  ");
            }
            fprintf (stderr, "%s", KNRM);
          }
          memset (state->cap_plus_csbk_bits[ts], 0, sizeof(state->cap_plus_csbk_bits[ts]));
          state->cap_plus_block_num[ts] = 0;

          //check here to see if we can go to the current rest lsn, if not already on it
          int busy = memcmp (empty, t_tg, sizeof(empty));
          if (!busy)
            trunk_set_cap_plus_cur_lsn(rest_channel);

          long int saved_cap_cur_freq = s_cap_cur_freq[ts & 1];
          s_cap_cur_freq[ts & 1] = 0;

          //testing (don't keep setting on quick data call LSN flip flops but same frequency for rest channel, other misc conditions)
          // if (!busy && rest_channel != state->dmr_rest_channel && opts->p25_trunk == 1 && state->p25_cc_freq != state->trunk_chan_map[rest_channel])
          if (!busy && opts->p25_trunk == 1 && rest_channel > 0 && state->trunk_chan_map[rest_channel] != 0)
          {
            long int cap_cur_freq = (saved_cap_cur_freq != 0) ? saved_cap_cur_freq : trunk_get_current_freq(opts, state);
            if (cap_cur_freq != 0 && cap_cur_freq != state->trunk_chan_map[rest_channel])
            {
              //update frequency
              state->p25_cc_freq = state->trunk_chan_map[rest_channel];

              //Craft a fake CSBK pdu send it to run as a p_clear to go to rest channel if its available (no calls currently)
              uint8_t dummy[12]; uint8_t dbits[96]; memset (dummy, 0, sizeof(dummy)); memset (dbits, 0, sizeof(dbits)); dummy[0] = 46; dummy[1] = 253;
              dmr_cspdu (opts, state, dbits, dummy, 1, 0);
            }
            else if (state->p25_vc_freq[0] != 0 || opts->p25_is_tuned == 1)
            {
              state->p25_vc_freq[0] = state->p25_vc_freq[1] = 0;
              if ((time(NULL) - state->last_vc_sync_time) > opts->trunk_hangtime &&
                  state->dmrburstL != 16 && state->dmrburstR != 16)
                opts->p25_is_tuned = 0;
            }
          }

        } //if (cap_complete)
      } //opcode == 0x3E

    } //fid == 0x10

    //Connect+ Section
    if (csbk_fid == 0x06)
    {
      //Con+ Adjacent Site
      if (csbk_o == 0x01)
      {
        uint8_t nb1 = cs_pdu[2] & 0x3F;
        uint8_t nb2 = cs_pdu[3] & 0x3F;
        uint8_t nb3 = cs_pdu[4] & 0x3F;
        uint8_t nb4 = cs_pdu[5] & 0x3F;
        uint8_t nb5 = cs_pdu[6] & 0x3F;

        //initial line break
        fprintf (stderr, "\n");
        fprintf (stderr, "%s", KYEL);
        fprintf (stderr, " Connect Plus Adjacent Sites:");
        if (nb1 != 0) fprintf (stderr, " %d;", nb1);
        if (nb2 != 0) fprintf (stderr, " %d;", nb2);
        if (nb3 != 0) fprintf (stderr, " %d;", nb3);
        if (nb4 != 0) fprintf (stderr, " %d;", nb4);
        if (nb5 != 0) fprintf (stderr, " %d;", nb5);
        if (nb1 == 0) fprintf (stderr, " None Listed;");
        state->dmr_t3_syscode = 0;
        state->dmr_neighbor_count = 0;
        dmr_add_neighbor (state, nb1);
        dmr_add_neighbor (state, nb2);
        dmr_add_neighbor (state, nb3);
        dmr_add_neighbor (state, nb4);
        dmr_add_neighbor (state, nb5);
        state->last_active_time = time(NULL);
        state->dmr_mfid = 0x06;
        sprintf (state->dmr_branding, "%s", "Motorola");
        sprintf(state->dmr_branding_sub, "Con+ ");

      }

      //Con+ Voice Channel Grant
      if (csbk_o == 0x03)
      {

        /*
        //this was a private voice call, see if a bit is flipped somewhere to indicate as much, octet 9?
        Connect Plus Voice Channel Grant; Target: 118903; Source: 151015; LCN: 1; TS: 0;
        DMR PDU Payload [83][06][02][4D][E7][01][D0][77][10][03][C2][CA] <-- 03 private?

        //this was from a group voice call
        18:06:29 Sync: +DMR  [slot1]  slot2  | Color Code=01 | CSBK
        Connect Plus Voice Channel Grant; Target: 1216; Source: 113114; LCN: 2; TS: 1;
        DMR PDU Payload [83][06][01][B9][DA][00][04][C0][28][02][AA][FF] <--02 group?

        //the 02/03 holds up on a few samples from different systems, also seems
        //that encrypted calls are not bit flipped here
        */

        //initial line break
        fprintf (stderr, "\n");
        uint32_t srcAddr = ( (cs_pdu[2] << 16) + (cs_pdu[3] << 8) + cs_pdu[4] );
        uint32_t grpAddr = ( (cs_pdu[5] << 16) + (cs_pdu[6] << 8) + cs_pdu[7] );
        uint8_t  lcn     = ( (cs_pdu[8] & 0xF0 ) >> 4 );
        uint8_t  tslot   = ( (cs_pdu[8] & 0x08 ) >> 3 ) & 1;
        uint8_t  opt     =    cs_pdu[9]; //call options? May only be a few of the LSB honestly
        fprintf (stderr, "%s", KYEL);
        if (opt == 2)
          fprintf (stderr, " Connect Plus Group Voice Channel Grant;");
        else if (opt == 3)
          fprintf (stderr, " Connect Plus Private Voice Channel Grant;");
        else fprintf (stderr, " Connect Plus Unknown %02X Channel Grant;", opt);
        fprintf (stderr, " Target: %d; Source: %d; LCN: %d; TS: %d;", grpAddr, srcAddr, lcn, tslot+1);
        // fprintf (stderr, " OPT: %02X;", opt); //debug
        state->dmr_mfid = 0x06;
        sprintf (state->dmr_branding, "%s", "Motorola");
        sprintf(state->dmr_branding_sub, "Con+ ");

        //add active channel string for display
        dmr_t3_active_clear_if_present (state);
        if (opt == 2)
          sprintf (state->active_channel[tslot], "Active Group Ch: %d TG: %d; ", lcn, grpAddr);
        else if (opt == 3)
          sprintf (state->active_channel[tslot], "Active Private Ch: %d TG: %d; ", lcn, grpAddr);
        else //generic channel of unknown type
          sprintf (state->active_channel[tslot], "Active OPT %02X Ch: %d TG: %d; ", opt, lcn, grpAddr);

        state->last_active_time = time(NULL);

        //Skip tuning group calls if group calls are disabled
        if (opts->trunk_tune_group_calls == 0 && opt == 2) goto SKIPCON;

        //Skip tuning private calls if private calls are disabled
        if (opts->trunk_tune_private_calls == 0 && opt == 3) goto SKIPCON;

        //skip tuning if opt is not 2 or 3 (assume group call option)
        if (opts->trunk_tune_group_calls == 0 && opt != 2 && opt != 3) goto SKIPCON;

        //NOTE: Only set CC Frequency from SLC since it will tell us
        //when we are on a control channel, and not a payload channel,
        //these CSBKs can come on payload channels as well and cause
        //FME to return to a dead air channel and start searching

        //shim in here for ncurses freq display when not trunking (playback, not live)
        if (opts->p25_trunk == 0 && state->trunk_chan_map[lcn] != 0)
        {
          //just set to both for now, could go on tslot later
          state->p25_vc_freq[0] = state->trunk_chan_map[lcn];
          state->p25_vc_freq[1] = state->trunk_chan_map[lcn];
        }

        //if tg hold is specified and matches target, allow for a call pre-emption by nullifying the last vc sync time
        if (state->tg_hold != 0 && state->tg_hold == grpAddr)
          state->last_vc_sync_time = 0;

        char mode[8]; //allow, block, digital, enc, etc
        sprintf (mode, "%s", "");

        //if we are using allow/whitelist mode, then write 'B' to mode for block
        //comparison below will look for an 'A' to write to mode if it is allowed
        if (opts->trunk_use_allow_list == 1) sprintf (mode, "%s", "B");

        for (int i = 0; i < state->group_tally; i++)
        {
          if (state->group_array[i].groupNumber == grpAddr)
          {
            fprintf (stderr, " [%s]", state->group_array[i].groupName);
            strcpy (mode, state->group_array[i].groupMode);
            break;
          }
        }

        //TG hold on DMR Con+ -- block non-matching target, allow matching target
        if (state->tg_hold != 0 && state->tg_hold != grpAddr) sprintf (mode, "%s", "B");
        if (state->tg_hold != 0 && state->tg_hold == grpAddr) sprintf (mode, "%s", "A");

        //TG Hop if the target here matches the target currently listening to (may be better to just rely on TG hold for this)
        // if (state->currentslot == 1 && state->lasttg  == grpAddr) state->last_vc_sync_time = 0; //opposite slot if TLC in current
        // if (state->currentslot == 0 && state->lasttgR == grpAddr) state->last_vc_sync_time = 0; //opposite slot if TLC in current

        time_t waitsec = 2;
        if (opts->trunk_tune_data_calls == 1)
          waitsec = 4; //increase time for a data call before hopping

        //don't tune if currently a vc on the control channel, but allow hopping form one VC to another VC if the former is in long TLC/Idle mode
        if ( (opts->trunk_tune_group_calls == 1) && (time(NULL) - state->last_vc_sync_time > waitsec) )
        {

          if (state->p25_cc_freq != 0 && opts->p25_trunk == 1 && (strcmp(mode, "B") != 0) && (strcmp(mode, "DE") != 0) )
          {
            if (state->trunk_chan_map[lcn] != 0) //if we have a valid frequency
            {
              //RIGCTL
              if (opts->use_rigctl == 1)
              {
                if (opts->setmod_bw != 0 ) SetModulation(opts->rigctl_sockfd, opts->setmod_bw);
                SetFreq(opts->rigctl_sockfd, state->trunk_chan_map[lcn]);
                state->p25_vc_freq[0] = state->p25_vc_freq[1] = state->trunk_chan_map[lcn];
                opts->p25_is_tuned = 1; //set to 1 to set as currently tuned so we don't keep tuning nonstop
                state->is_con_plus = 1; //flag on
                state->last_vc_sync_time = time(NULL); //bugfix: set sync here so we don't immediately tune back to CC constantly.
                dmr_reset_blocks (opts, state); //reset all block gathering since we are tuning away
                state->gi[tslot] = (opt == 3) ? 1 : 0;
              }

              //rtl
              else if (opts->audio_in_type == 3)
              {
                #ifdef USE_RTLSDR
                rtl_dev_tune (opts, state->trunk_chan_map[lcn]);
                state->p25_vc_freq[0] = state->p25_vc_freq[1] = state->trunk_chan_map[lcn];
                opts->p25_is_tuned = 1;
                state->is_con_plus = 1; //flag on
                state->last_vc_sync_time = time(NULL); //bugfix: set sync here so we don't immediately tune back to CC constantly.
                dmr_reset_blocks (opts, state); //reset all block gathering since we are tuning away
                state->gi[tslot] = (opt == 3) ? 1 : 0;
                #endif
              }

            }
          }
        }

        SKIPCON: ; //do nothing

      }

      //Con+ Data Channel Grant
      if (csbk_o == 0x06)
      {

        //initial line break
        fprintf (stderr, "\n");
        uint32_t dtarget = ( (cs_pdu[2] << 16) + (cs_pdu[3] << 8) + cs_pdu[4] );
        uint8_t  lcn     = ( (cs_pdu[5] & 0xF0 ) >> 4 );
        uint8_t  tslot   = ( (cs_pdu[5] & 0x08 ) >> 3 ) & 1;
        fprintf (stderr, "%s", KYEL);
        fprintf (stderr, " Connect Plus Data Channel Grant;");
        fprintf (stderr, " Target: %d; LCN: %d; TS: %d;", dtarget, lcn, tslot+1);
        state->dmr_mfid = 0x06;
        sprintf (state->dmr_branding, "%s", "Motorola");
        sprintf(state->dmr_branding_sub, "Con+ ");

        //Skip tuning data calls if data calls are disabled
        if (opts->trunk_tune_data_calls == 0) goto SKIPCOND;

        //add active channel string for display
        dmr_t3_active_clear_if_present (state);
        sprintf (state->active_channel[tslot], "Active Data Ch: %d TG: %d; ", lcn, dtarget);
        state->last_active_time = time(NULL);

        //NOTE: Only set CC Frequency from SLC since it will tell us
        //when we are on a control channel, and not a payload channel,
        //these CSBKs can come on payload channels as well and cause
        //FME to return to a dead air channel and start searching

        //shim in here for ncurses freq display when not trunking (playback, not live)
        if (opts->p25_trunk == 0 && state->trunk_chan_map[lcn] != 0)
        {
          //just set to both for now, could go on tslot later
          state->p25_vc_freq[0] = state->trunk_chan_map[lcn];
          state->p25_vc_freq[1] = state->trunk_chan_map[lcn];
        }

        //if tg hold is specified and matches target, allow for a call pre-emption by nullifying the last vc sync time
        // if (state->tg_hold != 0 && state->tg_hold == dtarget)
        //   state->last_vc_sync_time = 0;

        char mode[8]; //allow, block, digital, enc, etc
        sprintf (mode, "%s", "");

        //if we are using allow/whitelist mode, then write 'B' to mode for block
        //comparison below will look for an 'A' to write to mode if it is allowed
        if (opts->trunk_use_allow_list == 1) sprintf (mode, "%s", "B");

        for (int i = 0; i < state->group_tally; i++)
        {
          if (state->group_array[i].groupNumber == dtarget)
          {
            fprintf (stderr, " [%s]", state->group_array[i].groupName);
            strcpy (mode, state->group_array[i].groupMode);
            break;
          }
        }

        //TG hold on DMR Con+ -- block non-matching target, allow matching target
        // if (state->tg_hold != 0 && state->tg_hold != dtarget) sprintf (mode, "%s", "B");
        // if (state->tg_hold != 0 && state->tg_hold == dtarget) sprintf (mode, "%s", "A");

        //don't tune if currently a vc on the control channel
        if ( (opts->trunk_tune_data_calls == 1) && (time(NULL) - state->last_vc_sync_time > 2) )
        {

          if (state->p25_cc_freq != 0 && opts->p25_trunk == 1 && (strcmp(mode, "B") != 0) && (strcmp(mode, "DE") != 0) )
          {
            if (state->trunk_chan_map[lcn] != 0) //if we have a valid frequency
            {
              //RIGCTL
              if (opts->use_rigctl == 1)
              {
                if (opts->setmod_bw != 0 ) SetModulation(opts->rigctl_sockfd, opts->setmod_bw);
                SetFreq(opts->rigctl_sockfd, state->trunk_chan_map[lcn]);
                state->p25_vc_freq[0] = state->p25_vc_freq[1] = state->trunk_chan_map[lcn];
                opts->p25_is_tuned = 1; //set to 1 to set as currently tuned so we don't keep tuning nonstop
                state->is_con_plus = 1; //flag on
                state->last_vc_sync_time = time(NULL); //bugfix: set sync here so we don't immediately tune back to CC constantly.
                dmr_reset_blocks (opts, state); //reset all block gathering since we are tuning away
              }

              //rtl
              else if (opts->audio_in_type == 3)
              {
                #ifdef USE_RTLSDR
                rtl_dev_tune (opts, state->trunk_chan_map[lcn]);
                state->p25_vc_freq[0] = state->p25_vc_freq[1] = state->trunk_chan_map[lcn];
                opts->p25_is_tuned = 1;
                state->is_con_plus = 1; //flag on
                state->last_vc_sync_time = time(NULL); //bugfix: set sync here so we don't immediately tune back to CC constantly.
                dmr_reset_blocks (opts, state); //reset all block gathering since we are tuning away
                #endif
              }

            }
          }
        }

        SKIPCOND: ; //do nothing

      }

      //Con+ Slot Termination
      if (csbk_o == 0x0C)
      {

        //initial line break
        fprintf (stderr, "\n");
        uint32_t ttarget = ( (cs_pdu[2] << 16) + (cs_pdu[3] << 8) + cs_pdu[4] );
        fprintf (stderr, "%s", KYEL);
        fprintf (stderr, " Connect Plus Slot Termination;");
        fprintf (stderr, " Target: %d;", ttarget);

        //Run p_clear to decide whether or not to return to the control channel
        uint8_t dummy[12]; uint8_t dbits[96]; memset (dummy, 0, sizeof(dummy)); memset (dbits, 0, sizeof(dbits)); dummy[0] = 46; dummy[1] = 12;
        dmr_cspdu (opts, state, dbits, dummy, 1, 0);

        state->dmr_mfid = 0x06;
        sprintf (state->dmr_branding, "%s", "Motorola");
        sprintf(state->dmr_branding_sub, "Con+ ");

      }

      fprintf (stderr, "%s", KNRM);

    } //end Connect+

    //Hytera XPT -- Experimental, but values now seem very consistent and trunking is working on systems tested with
    if (csbk_fid == 0x68)
    {
      //XPT Site Status
      if (csbk_o == 0x0A)
      {
        //initial line break
        fprintf (stderr, "\n");
        fprintf (stderr, "%s", KYEL);

        uint8_t xpt_ch[6]; //one tg/call per LSN
        uint16_t tg = 0;  //8-bit TG value or TGT hash
        int i, j;

        //tg and channel info for trunking purposes
        uint16_t t_tg[24];
        memset (t_tg, 0, sizeof(t_tg));

        uint8_t xpt_seq    = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[0], 2);  //this replaces the CSBK lb and pf
        uint8_t xpt_free   = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[16], 4); //free repeater channel LCN
        uint8_t xpt_bank   = 0;
        uint8_t xpt_lcn    = 1;

        //determine starting position of LCN value
        if (xpt_seq == 1) xpt_lcn = 4;
        if (xpt_seq == 2) xpt_lcn = 7;

        //determine xpt_bank value for LSN value/tuning
        if (xpt_seq) xpt_bank = xpt_seq*6;

        //get 2-bit status values for each 6 LSNs
        for (i = 0; i < 6; i++) xpt_ch[i] = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[20+(i*2)], 2);

        fprintf (stderr, " Hytera XPT Site Status - Free LCN: %d SN: %d", xpt_free, xpt_seq);

        //strings for active channels
        dmr_t3_active_clear_if_present (state);
        char xpt_active[32];
        if (xpt_seq == 0) sprintf (state->active_channel[0], "XPT "); //add initial if sequence 0
        else sprintf (state->active_channel[xpt_seq], "%s", ""); //blank current sequence to re-write to
        state->last_active_time = time(NULL);

        //Print List of LCN with LSN Activity
        for (i = 0; i < 6; i++)
        {
          //add LCN value for each LSN pair to help users differentiate between the two when seeing LCN on FLC
          if (i == 0 || i == 2 || i == 4)
          {
            fprintf (stderr, "\n LCN %d - ", xpt_lcn);
            xpt_lcn++;
          }

          //LSN value here is logical slot, in flco, we get the logical channel (which is the repeater, does not include the slot value)
          fprintf (stderr, "LSN %02d: ", i+xpt_bank+1); //swapped out xpt_bank for all (xpt_seq*6) to simplify things
          tg = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[i*8+32], 8);
          fprintf (stderr, "ST-%X", xpt_ch[i]); //status bits value 0,1,2, or 3
          if (tg != 0)                fprintf (stderr, " %03d;  ", tg);
          else
          {
            if (xpt_ch[i] == 3)         fprintf (stderr, " Null; "); //NULL on ST-3 indicates repeater is not active for the LSNs that would be covered by these bits
            else if (xpt_ch[i] == 2)    fprintf (stderr, " Priv; "); //This seems to be used on both private data and private voice calls, but occassionally has a 0 TGT value
            else if (xpt_ch[i] == 1)    fprintf (stderr, " Unk;  "); //the 01 value has not been observed as of yet
            else if (xpt_ch[i] == 0)    fprintf (stderr, " Idle; "); //Idle

            //if Priv/Unk occurs, add t_tg value to it for tuning purposes
            if (xpt_ch[i] == 2)
            {
              if (opts->trunk_tune_private_calls == 1) t_tg[i+xpt_bank] = 1;
            }

            //could cause erroneous tuning since we don't yet know what the unk value really is (never observed)
            if (xpt_ch[i] == 1)
            {
              // if (opts->trunk_tune_data_calls == 1) t_tg[i] = 1; //disabled
            }

          }

          // if (i == 2) fprintf (stderr, "\n ");

          //add values to trunking tg/channel potentials
          if (tg != 0) t_tg[i+xpt_bank] = tg;

          //concantenate string to active channels for ncurses display
          if (tg != 0)
          {
            if (xpt_ch[i] == 3) sprintf (xpt_active, "LSN: %d TG: %d; ", i+xpt_bank+1, tg);
            if (xpt_ch[i] == 2) sprintf (xpt_active, "LSN: %d PC: %d; ", i+xpt_bank+1, tg);
            //last two are unknown status but have an associated target value
            if (xpt_ch[i] == 1) sprintf (xpt_active, "LSN: %d UK: %d; ", i+xpt_bank+1, tg);
            if (xpt_ch[i] == 0) sprintf (xpt_active, "LSN: %d UK: %d; ", i+xpt_bank+1, tg);
            strcat (state->active_channel[xpt_seq], xpt_active); //add string to active channel seq
          }

        }

        //add string for ncurses terminal display
        sprintf (state->dmr_site_parms, "Free LCN - %d ", xpt_free);

        //assign to cc freq to follow during no sync
        long int ccfreq = 0;

        //if using rigctl we can set an unknown or updated cc frequency
        if (opts->use_rigctl == 1 )
        {
          ccfreq = trunk_get_current_freq (opts, state);
          if (ccfreq != 0)
          {
            trunk_set_cc_freq_if_valid (opts, state, ccfreq);
            if (opts->p25_trunk == 1)
              opts->p25_is_tuned = 1;
          }
        }

        //if using rtl input, we can ask for the current frequency tuned
        if (opts->audio_in_type == 3 )
        {
          ccfreq = (long int)opts->rtlsdr_center_freq;
          if (ccfreq != 0)
          {
            trunk_set_cc_freq_if_valid (opts, state, ccfreq);
            if (opts->p25_trunk == 1)
              opts->p25_is_tuned = 1;
          }

        }

        //Skip tuning calls if group calls are disabled
        if (opts->trunk_tune_group_calls == 0) goto SKIPXPT;

        //Test allowing a group in the white list to preempt a call in progress and tune to a white listed call
        if (opts->trunk_use_allow_list == 1) state->last_vc_sync_time = 0;

        //Test allowing a tg hold to pre-empt a call in progress and tune to the hold TG
        if (state->tg_hold != 0) state->last_vc_sync_time = 0;

        //if no activity in this window
        if ( (time(NULL) - state->last_vc_sync_time) > 2 ) //may use last_cc_sync_time instead
          rotate_symbol_out_file(opts, state);

        //TODO: Consider a method to allow moving the frequency to the free repeater channel
        //when a TG hold is specified but nether slot carries the TG on Hold;
        //CODED: using Free repeater in SLC to change over if required

        //don't tune if vc on the current channel
        if ( (time(NULL) - state->last_vc_sync_time) > 2 ) //parenthesis error fixed
        {
          for (j = 0; j < 6; j++) //go through the channels stored looking for active ones to tune to
          {
            char mode[8]; //allow, block, digital, enc, etc
            sprintf (mode, "%s", "");

            //if we are using allow/whitelist mode, then write 'B' to mode for block
            //comparison below will look for an 'A' to write to mode if it is allowed
            if (opts->trunk_use_allow_list == 1) sprintf (mode, "%s", "B");

            //this won't work properly on hashed TGT values
            //unless users load a TGT hash in a csv file
            //and hope it doesn't clash with other normal TG values
            for (int i = 0; i < state->group_tally; i++)
            {
              if (state->group_array[i].groupNumber == t_tg[j+xpt_bank])
              {
                fprintf (stderr, " [%s]", state->group_array[i].groupName);
                strcpy (mode, state->group_array[i].groupMode);
                break;
              }
            }

            //TG hold on DMR XPT -- block non-matching target, allow matching target
            if (state->tg_hold != 0 && state->tg_hold != t_tg[j+xpt_bank]) sprintf (mode, "%s", "B");
            if (state->tg_hold != 0 && state->tg_hold == t_tg[j+xpt_bank]) sprintf (mode, "%s", "A");

            //without priority, this will tune the first one it finds (if group isn't blocked)
            if (t_tg[j+xpt_bank] != 0 && state->p25_cc_freq != 0 && opts->p25_trunk == 1 && (strcmp(mode, "B") != 0) && (strcmp(mode, "DE") != 0))
            {
              //debug print for tuning verification
              fprintf (stderr, "\n LSN/TG to tune to: %d - %d", j+xpt_bank+1, t_tg[j+xpt_bank]);

              if (state->trunk_chan_map[j+xpt_bank+1] != 0) //if we have a valid frequency
              {
                //RIGCTL
                if (opts->use_rigctl == 1)
                {

                  //may need the code below to TG hold (just in case SLC comes before a VLC or a VC6 EMB and immediately goes to the free lcn channel)
                  //disable or tweak code below if these are reversed somehow or causes issues
                  if (state->tg_hold != 0)
                  {
                    if ( (j&1) == 0 ) //slot 1 LSN
                    {
                      state->lasttg = t_tg[j+xpt_bank];
                      // state->lastsrc = source;
                    }
                    else //slot 2 LSN
                    {
                      state->lasttgR = t_tg[j+xpt_bank];
                      // state->lastsrcR = source;
                    }
                    //end TG set on tune
                  }

                  //debug
                  // fprintf (stderr, " - Freq: %ld", state->trunk_chan_map[j+xpt_bank+1]);
                  long int cur_freq = trunk_get_current_freq(opts, state);
                  if (cur_freq != state->trunk_chan_map[j+xpt_bank+1])
                  {
                    dmr_reset_blocks (opts, state); //reset all block gathering since we are tuning away from current frequency
                    if (opts->setmod_bw != 0 ) SetModulation(opts->rigctl_sockfd, opts->setmod_bw);
                    SetFreq(opts->rigctl_sockfd, state->trunk_chan_map[j+xpt_bank+1]);
                  }
                  else
                  {
                    trunk_update_filter_for_freq(state->trunk_chan_map[j+xpt_bank+1]);
                  }
                  state->gi[j & 1] = 0;
                  state->p25_vc_freq[0] = state->p25_vc_freq[1] = state->trunk_chan_map[j+xpt_bank+1];
                  opts->p25_is_tuned = 1; //set to 1 to set as currently tuned so we don't keep tuning nonstop
                  state->last_vc_sync_time = time(NULL);
                  j = 11; //break loop
                }

                //rtl
                else if (opts->audio_in_type == 3)
                {
                  #ifdef USE_RTLSDR

                  //may need the code below to TG hold (just in case SLC comes before a VLC or a VC6 EMB and immediately goes to the free lcn channel)
                  //disable or tweak code below if these are reversed somehow or causes issues
                  if (state->tg_hold != 0)
                  {
                    if ( (j&1) == 0 ) //slot 1 LSN
                    {
                      state->lasttg = t_tg[j+xpt_bank];
                      // state->lastsrc = source;
                    }
                    else //slot 2 LSN
                    {
                      state->lasttgR = t_tg[j+xpt_bank];
                      // state->lastsrcR = source;
                    }
                    //end TG set on tune
                  }

                  uint32_t temp = (uint32_t)state->trunk_chan_map[j+xpt_bank+1];
                  if (opts->rtlsdr_center_freq != temp)
                  {
                    dmr_reset_blocks (opts, state); //reset all block gathering since we are tuning away from current frequency
                    rtl_dev_tune (opts, state->trunk_chan_map[j+xpt_bank+1]); //unlike rigctl, using this actually interrupts signal decodes (rtl_clean_queue)
                    //debug print for tuning verification
                    fprintf (stderr, " - Tune to Freq: %ld", state->trunk_chan_map[j+xpt_bank+1]);
                  }
                  else
                  {
                    trunk_update_filter_for_freq(state->trunk_chan_map[j+xpt_bank+1]);
                    fprintf (stderr, " - Dont Tune Freq: %ld", state->trunk_chan_map[j+xpt_bank+1]);
                  }
                  state->gi[j & 1] = 0;
                  state->p25_vc_freq[0] = state->p25_vc_freq[1] = state->trunk_chan_map[j+xpt_bank+1];
                  opts->p25_is_tuned = 1;
                  state->last_vc_sync_time = time(NULL);
                  j = 11; //break loop
                  #endif
                }

              }
            }

          }
        } //end tuning

        SKIPXPT: ;

        sprintf (state->dmr_branding_sub, "XPT ");

        //Notes: I had a few issues to fix in this CSBK, but it does appear that this is trunking on a few small systems now,
        //albeit a very quiet systems that makes it difficult to know for certain if every aspect is working correctly

        //Notes: I've set XPT to set a CC frequency to whichever frequency its tuned to currently and getting
        //this particular CSBK, if the status portion does work correctly, then it shouldn't matter which frequency it is on
        //as long as this CSBK comes in and we can tune to other repeater lcns if they have activity and the frequency mapping
        //is correct in the csv file, assuming the current frequency doesn't have voice activity

      } //end 0x0A

      //XPT Adjacent Site Information -- Have yet to find a consistent indication of 'current site' identification in any CSBK/FLC/SLC payload
      if (csbk_o == 0x0B)
      {

        //initial line break
        fprintf (stderr, "\n");
        fprintf (stderr, "%s", KYEL);

        int i;
        uint8_t xpt_site_id[4];
        uint8_t xpt_site_rp[4];
        uint8_t xpt_site_u1[4];
        uint8_t xpt_site_u2[4];
        UNUSED2(xpt_site_u1, xpt_site_u2);

        uint8_t xpt_sn = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[0], 2);

        for (i = 0; i < 4; i++)
        {
          xpt_site_id[i] = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[16+(i*16)], 5);
          xpt_site_u1[i] = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[21+(i*16)], 3);
          xpt_site_rp[i] = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[24+(i*16)], 4);
          xpt_site_u2[i] = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[28+(i*16)], 4);
        }

        if (xpt_sn == 0 || state->dmr_t3_syscode != 0)
          state->dmr_neighbor_count = 0;
        state->dmr_t3_syscode = 0;
        fprintf (stderr, " Hytera XPT CSBK 0x0B - SN: %d", xpt_sn);
        fprintf (stderr, "\n");
        fprintf (stderr, " XPT Adjacent ");
        for (i = 0; i < 4; i++)
        {
          if (xpt_site_id[i] != 0)
          {
            fprintf (stderr, "Site:%d Free:%d; ", xpt_site_id[i], xpt_site_rp[i]);
            // fprintf (stderr, "RS1: %d RS2: %d - ", xpt_site_u1[i], xpt_site_u2[i]); //debug
            dmr_add_neighbor (state, xpt_site_id[i]);
          }
        }
        state->last_active_time = time(NULL);
        sprintf (state->dmr_branding_sub, "XPT ");

      } //end 0x0B

    } //end Hytera XPT section

    //misc discovered but not uncovered CSBKs
    if (csbk_o == 41 && csbk_fid == 0x10)
    {
      //initial line break
      fprintf (stderr, "\n");
      fprintf (stderr, "%s", KYEL);
      fprintf (stderr, " Moto Data Channel: %02X; ", csbk_o);
      for (int i = 2; i < 10; i++)
        fprintf (stderr, "%02X ", cs_pdu[i]);

      //SDRTrunk suggest this could be a data channel revert announcement
      //I'm not even sure what a revert data channel is
      //Moto Unknown Data Opcode: 29; 00 00 00 39 04 FC 00 00
    }

  }

  fprintf (stderr, "%s", KNRM);

}

//translate special gateway identifier addresses
void dmr_gateway_identifier (uint32_t source, uint32_t target)
{
  int i;
  uint32_t id;
  for (i = 0; i < 2; i++)
  {
    if (i == 0) id = source;
    else id = target;
    if (id == 0xFFFEC0) fprintf (stderr, "PSTNI; ");
    if (id == 0xFFFEC1) fprintf (stderr, "PABXI; ");
    if (id == 0xFFFEC2) fprintf (stderr, "LINEI; ");
    if (id == 0xFFFEC3) fprintf (stderr, "IPI; ");
    if (id == 0xFFFEC4) fprintf (stderr, "SUPLI; ");
    if (id == 0xFFFEC5) fprintf (stderr, "SDMI; ");
    if (id == 0xFFFEC6) fprintf (stderr, "REGI; ");
    if (id == 0xFFFEC7) fprintf (stderr, "MSI; ");
    if (id == 0xFFFEC8) fprintf (stderr, "RESERVED; ");
    if (id == 0xFFFEC9) fprintf (stderr, "DIVERTI; ");
    if (id == 0xFFFECA) fprintf (stderr, "TSI; ");
    if (id == 0xFFFECB) fprintf (stderr, "DISPATI; ");
    if (id == 0xFFFECC) fprintf (stderr, "STUNI; ");
    if (id == 0xFFFECD) fprintf (stderr, "AUTHI; ");
    if (id == 0xFFFECE) fprintf (stderr, "GPI; ");
    if (id == 0xFFFECF) fprintf (stderr, "KILLI; ");
    if (id == 0xFFFED0) fprintf (stderr, "PSTNDI; ");
    if (id == 0xFFFED1) fprintf (stderr, "PABXDI; ");
    if (id == 0xFFFED2) fprintf (stderr, "LINEDI; ");
    if (id == 0xFFFED3) fprintf (stderr, "DISPATDI; ");
    if (id == 0xFFFED4) fprintf (stderr, "ALLMSI; ");
    if (id == 0xFFFED5) fprintf (stderr, "IPDI; ");
    if (id == 0xFFFED6) fprintf (stderr, "DGNAI; ");
    if (id == 0xFFFED7) fprintf (stderr, "TATTSI; ");
    if (id == 0xFFFFFD) fprintf (stderr, "ALLMSIDL; ");
    if (id == 0xFFFFFE) fprintf (stderr, "ALLMSIDZ; ");
    if (id == 0xFFFFFF) fprintf (stderr, "ALLMSID; ");

    //NOTE: Observed address values of 64250, or 0xFAFA have been observed
    //on some Moto Tier 2 and Cap+ Systems, and 0xFAFAFA has been observed
    //on some Moto Tier 3 (CapMax) Systems, unsure if these are unique to that
    //manufacturer, or not, usually associated with Data Headers and PDU Messages
  }
}

void dmr_decode_syscode(dsd_opts * opts, dsd_state * state, uint8_t * cs_pdu_bits, int csbk_fid, int type)
{
  //TODO: Probably return specific elements of just the aloha to its own area, and just do the syscode here
  //copy and paste code into here, use type to determine whether or not to set current site info or not
  uint8_t reserved = cs_pdu_bits[16];
  uint8_t tsccas = cs_pdu_bits[17];
  uint8_t sync = cs_pdu_bits[18];
  uint8_t version = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[19], 3); //Document Version Control
  uint8_t offset = cs_pdu_bits[22]; //0-TSCC uses aligned timing; 1-TSCC uses offset timing
  uint8_t active = cs_pdu_bits[23]; //Active_Connection
  uint8_t mask = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[24], 5);
  uint8_t sf = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[29], 2); //service function
  uint8_t nrandwait = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[31], 4);
  uint8_t regreq = cs_pdu_bits[35];
  uint8_t backoff = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[36], 4);
  UNUSED5(reserved, tsccas, sync, offset, active);
  UNUSED3(sf, nrandwait, backoff);

  //raw syscode (C_ALOHA uses bits 40..53; C_BCAST Adj/VoteNow uses bparms1 at bits 21..34 and local site at 40..53)
  uint16_t local_syscode = (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[40], 14);
  uint16_t syscode = (type == 0) ? local_syscode
                                 : (uint16_t)ConvertBitIntoBytes(&cs_pdu_bits[21], 14);

  if (type == 0 || (type == 1 && local_syscode != 0))
  {
    if (state->dmr_t3_syscode != local_syscode)
    {
      state->dmr_is_ras = 0;
      state->dmr_branding[0] = 0;
      state->dmr_neighbor_count = 0;
      if (csbk_fid == 0 || csbk_fid >= 253)
        state->dmr_mfid = -1;
      s_capmax_cc_seen = 0;
      s_capmax_status_slot = -1;
      s_capmax_slot_aloha_cnt = 0;
      s_capmax_non_status_csbks = 0;
    }
    state->dmr_t3_syscode = local_syscode;
    state->dmr_rest_channel = -1;
    if (state->dmr_branding_sub[0] != 0)
    {
      state->dmr_branding[0] = 0;
      state->dmr_branding_sub[0] = 0;
      if (csbk_fid == 0 || csbk_fid >= 253)
        state->dmr_mfid = -1;
    }
  }

  uint16_t net = 0;
  uint16_t site = 0;

  //DMR Location Area - DMRLA: tiny n 1-3; small 1-5; large 1-8; huge 1-10
  uint16_t n = 1;
  char model_str[8];
  char par_str[8]; //category A, B, AB, or reserved

  sprintf (model_str, "%s", " ");
  sprintf (par_str, "%s", "Res");

  dmr_parse_t3_syscode (syscode, &net, &site, &n, model_str, sizeof(model_str));
  if (type == 1 && local_syscode != 0)
    dmr_record_t3_syscode (state, local_syscode, 0);
  dmr_record_t3_syscode (state, syscode, (type != 0));

  //honestly can't say that this is accurate, just a guess
  uint8_t is_capmax = 0; //capmax(mot) flag
  if (csbk_fid == 0x10)
  {
    is_capmax = 1;
    if (type == 0)
      sprintf (state->dmr_branding, "%s", "Motorola");
    // sprintf (state->dmr_branding_sub, "%s", "CapMax ");
  }

  n = dmr_effective_t3_n (opts, state, syscode, n, is_capmax);

  uint16_t sub_mask = (n == 0) ? 0 : (uint16_t)((1U << n) - 1);

  uint8_t par = (uint8_t)ConvertBitIntoBytes(&cs_pdu_bits[54], 2);
  if (par == 1) sprintf (par_str, "%s", "A");
  if (par == 2) sprintf (par_str, "%s", "B");
  if (par == 3) sprintf (par_str, "%s", "AB");

  uint32_t target = (uint32_t)ConvertBitIntoBytes(&cs_pdu_bits[56], 24);

  if (type == 0)
  {
    //considering not adding a +1 to these values (will need to consult the docs first)
    //TODO: Change SLC to mirror this output format
    if (n != 0) fprintf (stderr, " C_ALOHA_SYS_PARMS: %s; Net ID: %d; Site ID: %d.%d; Cat: %s;", model_str, net+1, (site>>n)+1, (site & sub_mask)+1, par_str);
    else fprintf (stderr, " C_ALOHA_SYS_PARMS: %s; Net ID: %d; Site ID: %d;", model_str, net, site);
    fprintf (stderr, " SYS: %04X;", syscode); //#192
    if (is_capmax) fprintf (stderr, " Capacity Max");

    if (opts->payload == 1)
    {
      fprintf (stderr, "\n");
      if (reserved) fprintf (stderr, " Res: %04X;", reserved);
      if (tsccas) fprintf (stderr, " TSCCAS;");
      if (sync) fprintf (stderr, " Sync;");
      fprintf (stderr, " Ver: %d;", version);
      if (offset) fprintf (stderr, " Offset;");
      if (active) fprintf (stderr, " Active Connection;");
      fprintf (stderr, " SF: %d;", sf); //service function
      fprintf (stderr, " NR: %X;", nrandwait); //what is this for again?
      if (regreq) fprintf (stderr, " Reg Required;");
      fprintf (stderr, " Backoff: %X;", backoff);
      if (mask) fprintf (stderr, " Mask: %02X;", mask);
      if (target) fprintf (stderr, " MS: %d; ", target);
      dmr_gateway_identifier(0,target); //its either 0, or possibly ALLMSI, or a specifically targeted value?
    }

    //add string for ncurses terminal display
    dmr_refresh_t3_site_parms (opts, state);
  }

  if (type == 1)
  {
    dmr_add_neighbor (state, syscode);
    //NOTE: I just wrote bparms1 into the area where syscode is when it is an adj_site (or votenow site)
    if (n != 0) fprintf (stderr, " %s; Net ID: %d; Site ID: %d.%d;", model_str, net+1, (site>>n)+1, (site & sub_mask)+1); //par_string available here?
    else fprintf (stderr, " %s; Net ID: %d; Site ID: %d;", model_str, net, site);
    fprintf (stderr, " SYS: %04X;", syscode); //#192
    // if (is_capmax) fprintf (stderr, "Capacity Max ");
    dmr_refresh_t3_site_parms (opts, state);
  }

}