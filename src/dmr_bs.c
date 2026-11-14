/*-------------------------------------------------------------------------------
 * dmr_bs.c
 * DMR BS Voice Handling and Data Gathering Routines - "DMR STEREO"
 *
 * LWVMOBILE
 * 2022-12 DSD-FME Florida Man Edition
 *-----------------------------------------------------------------------------*/

#include "dsd.h"
#include "dmr_const.h"

// #define PRINT_AMBE72 //enable to view 72-bit AMBE codewords

static inline int dmr_bs_sync_match(const char *sync, const char *pat)
{
  int m = 0;
  for (int i = 0; i < 24; i++)
  {
    if (sync[i] == pat[i]) m++;
  }
  return (m >= 20);
}

//A subroutine for processing each TDMA frame individually to allow for
//processing voice and/or data on both BS slots (channels) simultaneously
void dmrBS (dsd_opts * opts, dsd_state * state)
{
  char * timestr = NULL;

  int i, dibit;
  char ambe_fr[4][24];
  char ambe_fr2[4][24];
  char ambe_fr3[4][24];

  //redundancy check (carrier signal loss event)
  char redundancyA[36];

  //memcpy of ambe_fr for late entry
  uint8_t m1[4][24];
  uint8_t m2[4][24];
  uint8_t m3[4][24];

  const int *w, *x, *y, *z;
  char sync[25];
  uint8_t syncdata[48];
  memset (syncdata, 0, sizeof(syncdata));

  uint8_t emb_pdu[16];
  memset (emb_pdu, 0, sizeof(emb_pdu));

  uint8_t emb_ok = 0;
  uint8_t tact_okay = 0;
  uint8_t cach_err = 0;
  UNUSED(cach_err);

  uint8_t internalslot;
  uint8_t vc1;
  uint8_t vc2;

  //assign as nonsensical numbers
  uint8_t cc = 25;
  uint8_t power = 9; //power and pre-emption indicator
  uint8_t lcss = 9;
  UNUSED2(cc, lcss);

  //would be ideal to grab all dibits and break them into bits to pass to new data handler?
  uint8_t dummy_bits[196];
  memset (dummy_bits, 0, sizeof(dummy_bits));

  //Init slot lights
  sprintf (state->slot1light, " slot1 ");
  sprintf (state->slot2light, " slot2 ");

  //if coming from the bootsrap, then the slot will still be assigned the last value
  //we want to set only that vc value to 2, the other to 7
  if (state->currentslot == 0)
  {
    vc1 = 2;
    vc2 = 7;
  }
  if (state->currentslot == 1)
  {
    vc1 = 7;
    vc2 = 2;
  }

  uint8_t emb_lc_valid[2] = {0, 0};
  uint8_t emb_bad_count[2] = {0, 0};
  uint8_t vc_cc_confirmed[2] = {0, 0};
  uint8_t slot_cc_rejected[2] = {0, 0};
  if (state->currentslot == 0) emb_lc_valid[0] = 1;
  if (state->currentslot == 1) emb_lc_valid[1] = 1;
  if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 &&
      state->color_code_ok == 1 && (int)state->color_code == opts->dmr_cc_filter)
  {
    if (state->currentslot == 0 && (state->dmrburstL == 0 || state->dmrburstL == 1 || state->dmrburstL == 16 || state->lasttg != 0))
      vc_cc_confirmed[0] = 1;
    if (state->currentslot == 1 && (state->dmrburstR == 0 || state->dmrburstR == 1 || state->dmrburstR == 16 || state->lasttgR != 0))
      vc_cc_confirmed[1] = 1;
  }

  short int loop = 1;
  short int skipcount = 0;
  uint8_t expected_slot = (state->currentslot == 0) ? 1 : 0;

  //cach
  uint8_t cachdata[25];
  int cachInterleave[24] =
  {0, 7, 8, 9, 1, 10,
   11, 12, 2, 13, 14,
   15, 3, 16, 4, 17, 18,
   19, 5, 20, 21, 22, 6, 23
  };

  //cach tact bits
  uint8_t tact_bits[7];

  //Run Loop while the getting is good
  while (loop == 1) {

  if (exitflag == 1)
  {
    cleanupAndExit (opts, state);
  }

  timestr = getTimeC();

  memset (ambe_fr, 0, sizeof(ambe_fr));
  memset (ambe_fr2, 0, sizeof(ambe_fr2));
  memset (ambe_fr3, 0, sizeof(ambe_fr3));
  memset (emb_pdu, 0, sizeof(emb_pdu));
  memset (syncdata, 0, sizeof(syncdata));

  internalslot = -1; //reset here so we know if this value is being set properly
  for(i = 0; i < 12; i++)
  {
    dibit = getDibit(opts, state);
    state->dmr_stereo_payload[i] = dibit;

    cachdata[cachInterleave[(i*2)]]   = (1 & (dibit >> 1)); // bit 1
    cachdata[cachInterleave[(i*2)+1]] = (1 & dibit);       // bit 0
  }

  for (i = 0; i < 7; i++)
  {
    tact_bits[i] = cachdata[i];
  }

  //disabled for dmr_cach testing
  tact_okay = 0;
  if ( Hamming_7_4_decode (tact_bits) ) tact_okay = 1;
  int cach_tact_bad = (tact_okay != 1 || tact_bits[1] != expected_slot);
  if (cach_tact_bad)
  {
    state->dmr_cach_counter = 0;
    if (vc1 > 6 && vc2 > 6)
    {
      tact_okay = 0;
      goto END;
    }
    tact_okay = 1;
    internalslot = state->currentslot = expected_slot;
  }
  else
  {
    internalslot = state->currentslot = tact_bits[1];
  }

  //Setup for first AMBE Frame
  //Interleave Schedule
  w = rW;
  x = rX;
  y = rY;
  z = rZ;

  //First AMBE Frame, Full 36
  for(i = 0; i < 36; i++)
  {
    dibit = getDibit(opts, state);
    state->dmr_stereo_payload[i+12] = dibit;
    redundancyA[i] = dibit;

    ambe_fr[*w][*x] = (1 & (dibit >> 1)); // bit 1
    ambe_fr[*y][*z] = (1 & dibit);        // bit 0

    w++;
    x++;
    y++;
    z++;

  }

  //check for flatline input if caught in a 'no carrier' loop
  int flatline = 1;
  for (i = 1; i < 36; i++)
  {
    if (redundancyA[i] != redundancyA[0])
    {
      flatline = 0;
      break;
    }
  }
  if (flatline)
  {
    goto END;
  }

  //Setup for Second AMBE Frame
  //Interleave Schedule
  w = rW;
  x = rX;
  y = rY;
  z = rZ;

  //Second AMBE Frame, First Half 18 dibits just before Sync or EmbeddedSignalling
  for(i = 0; i < 18; i++)
  {
    dibit = getDibit(opts, state);
    state->dmr_stereo_payload[i+48] = dibit;
    ambe_fr2[*w][*x] = (1 & (dibit >> 1)); // bit 1
    ambe_fr2[*y][*z] = (1 & dibit);        // bit 0

    w++;
    x++;
    y++;
    z++;

  }

  //signaling data or sync
  for(i = 0; i < 24; i++)
  {
    dibit = getDibit(opts, state);
    state->dmr_stereo_payload[i+66] = dibit;
    sync[i] = (dibit | 1) + 48; //Sync Burst String as ones and threes

    syncdata[(2*i)]   = (1 & (dibit >> 1));  // bit 1
    syncdata[(2*i)+1] = (1 & dibit);         // bit 0

    //embedded link control
    if(internalslot == 0 && vc1 > 1 && vc1 < 7) //grab on vc1 values 2-5 B C D E, and F
    {
      state->dmr_embedded_signalling[internalslot][vc1-1][i*2]   = (1 & (dibit >> 1)); // bit 1
      state->dmr_embedded_signalling[internalslot][vc1-1][i*2+1] = (1 & dibit); // bit 0
    }

    if(internalslot == 1 && vc2 > 1 && vc2 < 7) //grab on vc2 values 2-5 B C D E, and F
    {
      state->dmr_embedded_signalling[internalslot][vc2-1][i*2]   = (1 & (dibit >> 1)); // bit 1
      state->dmr_embedded_signalling[internalslot][vc2-1][i*2+1] = (1 & dibit); // bit 0
    }

  }
  sync[24] = 0;

  for(i = 0; i < 8; i++) emb_pdu[i + 0] = syncdata[i];
  for(i = 0; i < 8; i++) emb_pdu[i + 8] = syncdata[i + 40];

  //Continue Second AMBE Frame, 18 after Sync or EmbeddedSignalling
  for(i = 0; i < 18; i++)
  {
    dibit = getDibit(opts, state);
    state->dmr_stereo_payload[i+90] = dibit;
    ambe_fr2[*w][*x] = (1 & (dibit >> 1)); // bit 1
    ambe_fr2[*y][*z] = (1 & dibit);        // bit 0

    w++;
    x++;
    y++;
    z++;

  }

  //Setup for Third AMBE Frame
  //Interleave Schedule
  w = rW;
  x = rX;
  y = rY;
  z = rZ;

  //Third AMBE Frame, Full 36
  for(i = 0; i < 36; i++)
  {
    dibit = getDibit(opts, state);
    state->dmr_stereo_payload[i+108] = dibit;
    ambe_fr3[*w][*x] = (1 & (dibit >> 1)); // bit 1
    ambe_fr3[*y][*z] = (1 & dibit);        // bit 0

    w++;
    x++;
    y++;
    z++;

  }

  //reset vc counters to 1 if new voice sync frame on each slot
  if ( dmr_bs_sync_match (sync, DMR_BS_VOICE_SYNC) )
  {
    if (internalslot == 0) { vc1 = 1; emb_lc_valid[0] = 1; emb_bad_count[0] = 0; }
    if (internalslot == 1) { vc2 = 1; emb_lc_valid[1] = 1; emb_bad_count[1] = 0; }
    dmr_alg_reset (opts, state);
  }

  //check for sync pattern here after collected the rest of the payload, decide what to do with it
  if ( dmr_bs_sync_match (sync, DMR_BS_DATA_SYNC) )
  {
    uint8_t prev_vc1 = vc1;
    uint8_t prev_vc2 = vc2;
    if (internalslot == 0)
    {
      vc1 = 7; //set to 7 so we can see that we should not be on a VC unless a framesync comes in for it first
      memset (state->f_l4, 0, sizeof(state->f_l4));
      memset (state->s_l4, 0, sizeof(state->s_l4));
      memset (state->s_l4u, 0, sizeof(state->s_l4u));

      //close MBEout file - slot 1
      if (opts->mbe_out_f != NULL) closeMbeOutFile (opts, state);
    }
    if (internalslot == 1)
    {
      vc2 = 7; //set to 7 so we can see that we should not be on a VC unless a framesync comes in for it first
      memset (state->f_r4, 0, sizeof(state->f_r4));
      memset (state->s_r4, 0, sizeof(state->s_r4));
      memset (state->s_r4u, 0, sizeof(state->s_r4u));

      //close MBEout file - slot 2
      if (opts->mbe_out_fR != NULL) closeMbeOutFileR (opts, state);

    }
    dmr_data_sync (opts, state);
    if (state->color_code_ok == 0 && state->dmr_color_code == 16)
    {
      cach_err = 0;
      tact_okay = 1;
      emb_ok = 1;
      goto END;
    }
    if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 &&
        state->color_code_ok == 1 && (int)state->color_code == opts->dmr_cc_filter)
    {
      slot_cc_rejected[internalslot] = 0;
      if (internalslot == 0)
        vc_cc_confirmed[0] = (state->dmrburstL == 0 || state->dmrburstL == 1) ? 1 : 0;
      else
        vc_cc_confirmed[1] = (state->dmrburstR == 0 || state->dmrburstR == 1) ? 1 : 0;
    }
    if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 &&
        state->color_code_ok == 1 && (int)state->color_code != opts->dmr_cc_filter)
    {
      if (internalslot == 0 && prev_vc1 <= 6)
      {
        vc1 = prev_vc1 + 1;
        emb_lc_valid[0] = 0;
        state->dmrburstL = 16;
        goto SKIP;
      }
      if (internalslot == 1 && prev_vc2 <= 6)
      {
        vc2 = prev_vc2 + 1;
        emb_lc_valid[1] = 0;
        state->dmrburstR = 16;
        goto SKIP;
      }
      slot_cc_rejected[internalslot] = 1;
      vc_cc_confirmed[internalslot] = 0;
      if (prev_vc1 > 6 && prev_vc2 > 6)
      {
        cach_err = 0;
        tact_okay = 1;
        emb_ok = 1;
        goto END;
      }
      long int cur_f = ((opts->p25_trunk == 1 || opts->scanner_mode == 1) && opts->p25_is_tuned == 0 && state->lcn_freq_count > 0)
                       ? trunk_get_current_freq(opts, state) : 0;
      int not_on_imported = (cur_f > 0 && !trunk_is_imported_freq(state, cur_f));
      int on_cap_plus_rest = (state->dmr_rest_channel != -1 &&
                              (opts->p25_is_tuned == 0 || state->p25_vc_freq[0] == 0 || state->p25_vc_freq[0] == state->p25_cc_freq));
      time_t hangtime_limit = on_cap_plus_rest
                              ? (opts->trunk_hangtime + 4)
                              : opts->trunk_hangtime;
      int can_hunt_or_return = (opts->scanner_mode == 1) ||
                               (opts->p25_trunk == 1 && (opts->p25_is_tuned == 1 || state->dmr_rest_channel != -1 || state->lcn_freq_count > 0 || state->p25_cc_freq != 0));
      if (not_on_imported || (can_hunt_or_return && ((time(NULL) - state->last_cc_sync_time) > hangtime_limit)))
      {
        cach_err = 0;
        tact_okay = 1;
        emb_ok = 1;
        goto END;
      }
    }
    skipcount++;
    goto SKIP;
  }

  //ETSI TS 102 361-1 V2.6.1 Table E.2 and E.4
  //check for a rc burst with bptc or 34 rate data in it (testing only)
  // #define RC_TESTING //disable if not in use
  #ifdef RC_TESTING

  //skip the vc counter, just look at the QR and P/Pi if not voice or data sync pattern
  if ( (strcmp (sync, DMR_BS_DATA_SYNC) != 0) && (strcmp (sync, DMR_BS_VOICE_SYNC) != 0) )
  {

    //Golay_20_8_decode FEC for the burst type (SlotType)
    unsigned char SlotType[20];
    memset (SlotType, 0, sizeof(SlotType));
    uint8_t k = 61;
    for (uint8_t i = 0; i < 5; i++)
    {
      SlotType[(i*2)+0] = (state->dmr_stereo_payload[k+0] >> 1) & 1;
      SlotType[(i*2)+1] = (state->dmr_stereo_payload[k++] >> 0) & 1;
    }
    k = 90;
    for (uint8_t i = 0; i < 5; i++)
    {
      SlotType[(i*2)+10] = (state->dmr_stereo_payload[k+0] >> 1) & 1;
      SlotType[(i*2)+11] = (state->dmr_stereo_payload[k++] >> 0) & 1;
    }

    //if the QR FEC is good, tact/cach FEC is good, and slot type burst FEC is good, and the P/PI bit is on for RC
    //NOTE: This can still trigger on bad signal when it should be a data sync pattern but signal drops out or
    //occassionally on p_clear with trunking tuner logic active and partial stale dibits in the buffer
    if (QR_16_7_6_decode(emb_pdu) && emb_pdu[4] && tact_okay == 1) //during voice test
    {

      if (Golay_20_8_decode(SlotType))
      {
        fprintf (stderr,"%s ", timestr);
        if (opts->inverted_dmr == 0)
          fprintf (stderr,"Sync: +RC   ");
        else fprintf (stderr,"Sync: -RC   ");

        dmr_data_sync (opts, state);
      }

      for (i = 0; i < 48; i++)
        state->dmr_embedded_signalling[internalslot][5][i] = syncdata[i];

      dmr_sbrc (opts, state, emb_pdu[4]);

      emb_ok = 1;

      //give an audio cue when this happens (low, high)
      beeper (opts, state, internalslot, 40, 86, 3);
      beeper (opts, state, internalslot, 80, 86, 3);

      //put into Event History
      state->event_history_s[0].Event_History_Items[internalslot].color_pair = 4;
      watchdog_event_datacall (opts, state, 0, 0, "DMR Reverse Channel P/PI Indicator On (FEC Okay);", internalslot);
      push_event_history (&state->event_history_s[internalslot]);
      init_event_history (&state->event_history_s[internalslot], 0, 1);

      if (Golay_20_8_decode(SlotType))
      {
        skipcount++;
        goto SKIP;
      }

    }

  }
  #endif //RC_TESTING

  //check to see if we are expecting a VC at this point vc > 7
  if (!dmr_bs_sync_match (sync, DMR_BS_DATA_SYNC) && internalslot == 0 && vc1 > 6)
  {
    //if we joined mid-superframe or missed VC1 sync on slot 1, check if EMB is valid
    if (tact_okay == 1 && QR_16_7_6_decode(emb_pdu))
    {
      uint8_t emb_cc = ((emb_pdu[0] << 3) + (emb_pdu[1] << 2) + (emb_pdu[2] << 1) + emb_pdu[3]);
      uint8_t emb_lcss = ((emb_pdu[5] << 1) + emb_pdu[6]);
      if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 && emb_cc != (uint8_t)opts->dmr_cc_filter)
      {
        trunk_check_rigctl_freq(opts, state);
      }
      int cc_match = (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15)
                     ? (emb_cc == (uint8_t)opts->dmr_cc_filter)
                     : (state->dmr_color_code <= 15 && emb_cc == state->dmr_color_code);
      if (!cc_match && trunk_is_stale_retune_burst((int)emb_cc))
      {
        if (state->dmr_unconfirmed_sync == 1)
        {
          state->last_cc_sync_time = state->dmr_unconfirmed_cc_sync_time;
          state->last_vc_sync_time = state->dmr_unconfirmed_vc_sync_time;
          state->dmr_unconfirmed_sync = 0;
        }
        cach_err = 0;
        tact_okay = 1;
        emb_ok = 1;
        goto END;
      }
      (void)trunk_is_stale_retune_burst((int)emb_cc);
      state->color_code = emb_cc;
      if (cc_match || opts->dmr_cc_filter < 0 || opts->dmr_cc_filter > 15 || state->dmr_color_code != (unsigned int)opts->dmr_cc_filter)
        state->dmr_color_code = emb_cc;
      state->color_code_ok = 1;
      if (cc_match)
      {
        emb_bad_count[0] = 0;
        slot_cc_rejected[0] = 0;
        state->dmr_unconfirmed_sync = 0;
        state->last_cc_sync_time = time(NULL);
        if (emb_lcss == 1)
        {
          vc1 = 2;
          emb_lc_valid[0] = 1;
          for (i = 0; i < 48; i++)
            state->dmr_embedded_signalling[0][1][i] = syncdata[i];
          if (state->DMRvcL == 0) state->DMRvcL = 3;
          if (state->dropL == 256) state->dropL = 277;
        }
        else if (emb_lcss == 3)
        {
          vc1 = 4;
          emb_lc_valid[0] = 0;
          if (state->DMRvcL == 0) state->DMRvcL = 9;
          if (state->dropL == 256) state->dropL = 298;
        }
        else if (emb_lcss == 2)
        {
          vc1 = 5;
          emb_lc_valid[0] = 0;
          if (state->DMRvcL == 0) state->DMRvcL = 12;
          if (state->dropL == 256) state->dropL = 319;
        }
        else if (emb_lcss == 0 && state->dmrburstL == 16)
        {
          vc1 = 6;
          emb_lc_valid[0] = 0;
          if (state->DMRvcL == 0) state->DMRvcL = 15;
          if (state->dropL == 256) state->dropL = 340;
        }
        if (vc1 <= 6)
          vc_cc_confirmed[0] = 1;
      }
      else if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15)
      {
        if (state->dmr_unconfirmed_sync == 1)
        {
          state->last_cc_sync_time = state->dmr_unconfirmed_cc_sync_time;
          state->last_vc_sync_time = state->dmr_unconfirmed_vc_sync_time;
          state->dmr_unconfirmed_sync = 0;
        }
        memset (state->f_l4, 0, sizeof(state->f_l4));
        memset (state->s_l4, 0, sizeof(state->s_l4));
        memset (state->s_l4u, 0, sizeof(state->s_l4u));
        slot_cc_rejected[0] = 1;
        vc_cc_confirmed[0] = 0;
        if (vc2 > 6)
        {
          cach_err = 0;
          tact_okay = 1;
          emb_ok = 1;
          goto END;
        }
        goto SKIP;
      }
    }
    if (vc1 > 6)
    {
      if (trunk_is_stale_retune_burst(-1))
      {
        cach_err = 0;
        tact_okay = 1;
        emb_ok = 1;
        goto END;
      }
      if (state->dmrburstL == 16) state->dmrburstL = 17;
      fprintf (stderr,"%s ", timestr);

      //simplifying things
      // char polarity[3];
      char light[18];

      sprintf (light, "%s", " [SLOT1]  slot2  ");
      fprintf (stderr,"Sync:  DMR %s", light);
      fprintf (stderr, "%s", KCYN);
      fprintf (stderr, "| Frame Sync Err: %d", vc1);
      fprintf (stderr, "%s", KNRM);
      fprintf (stderr, "\n");
      vc1++;
      //only trigger a resync if the opposite slot is not actively decoding voice
      if ( vc1 > 13 && vc2 > 6 ) goto END;
      if ( vc1 > 13 ) vc1 = 13;
      goto SKIP;
    }
  }

  //check to see if we are expecting a VC at this point vc > 7
  if (!dmr_bs_sync_match (sync, DMR_BS_DATA_SYNC) && internalslot == 1 && vc2 > 6)
  {
    //if we joined mid-superframe or missed VC1 sync on slot 2, check if EMB is valid
    if (tact_okay == 1 && QR_16_7_6_decode(emb_pdu))
    {
      uint8_t emb_cc = ((emb_pdu[0] << 3) + (emb_pdu[1] << 2) + (emb_pdu[2] << 1) + emb_pdu[3]);
      uint8_t emb_lcss = ((emb_pdu[5] << 1) + emb_pdu[6]);
      if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 && emb_cc != (uint8_t)opts->dmr_cc_filter)
      {
        trunk_check_rigctl_freq(opts, state);
      }
      int cc_match = (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15)
                     ? (emb_cc == (uint8_t)opts->dmr_cc_filter)
                     : (state->dmr_color_code <= 15 && emb_cc == state->dmr_color_code);
      if (!cc_match && trunk_is_stale_retune_burst((int)emb_cc))
      {
        if (state->dmr_unconfirmed_sync == 1)
        {
          state->last_cc_sync_time = state->dmr_unconfirmed_cc_sync_time;
          state->last_vc_sync_time = state->dmr_unconfirmed_vc_sync_time;
          state->dmr_unconfirmed_sync = 0;
        }
        cach_err = 0;
        tact_okay = 1;
        emb_ok = 1;
        goto END;
      }
      (void)trunk_is_stale_retune_burst((int)emb_cc);
      state->color_code = emb_cc;
      if (cc_match || opts->dmr_cc_filter < 0 || opts->dmr_cc_filter > 15 || state->dmr_color_code != (unsigned int)opts->dmr_cc_filter)
        state->dmr_color_code = emb_cc;
      state->color_code_ok = 1;
      if (cc_match)
      {
        emb_bad_count[1] = 0;
        slot_cc_rejected[1] = 0;
        state->dmr_unconfirmed_sync = 0;
        state->last_cc_sync_time = time(NULL);
        if (emb_lcss == 1)
        {
          vc2 = 2;
          emb_lc_valid[1] = 1;
          for (i = 0; i < 48; i++)
            state->dmr_embedded_signalling[1][1][i] = syncdata[i];
          if (state->DMRvcR == 0) state->DMRvcR = 3;
          if (state->dropR == 256) state->dropR = 277;
        }
        else if (emb_lcss == 3)
        {
          vc2 = 4;
          emb_lc_valid[1] = 0;
          if (state->DMRvcR == 0) state->DMRvcR = 9;
          if (state->dropR == 256) state->dropR = 298;
        }
        else if (emb_lcss == 2)
        {
          vc2 = 5;
          emb_lc_valid[1] = 0;
          if (state->DMRvcR == 0) state->DMRvcR = 12;
          if (state->dropR == 256) state->dropR = 319;
        }
        else if (emb_lcss == 0 && state->dmrburstR == 16)
        {
          vc2 = 6;
          emb_lc_valid[1] = 0;
          if (state->DMRvcR == 0) state->DMRvcR = 15;
          if (state->dropR == 256) state->dropR = 340;
        }
        if (vc2 <= 6)
          vc_cc_confirmed[1] = 1;
      }
      else if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15)
      {
        if (state->dmr_unconfirmed_sync == 1)
        {
          state->last_cc_sync_time = state->dmr_unconfirmed_cc_sync_time;
          state->last_vc_sync_time = state->dmr_unconfirmed_vc_sync_time;
          state->dmr_unconfirmed_sync = 0;
        }
        memset (state->f_r4, 0, sizeof(state->f_r4));
        memset (state->s_r4, 0, sizeof(state->s_r4));
        memset (state->s_r4u, 0, sizeof(state->s_r4u));
        slot_cc_rejected[1] = 1;
        vc_cc_confirmed[1] = 0;
        if (vc1 > 6)
        {
          cach_err = 0;
          tact_okay = 1;
          emb_ok = 1;
          goto END;
        }
        goto SKIP;
      }
    }
    if (vc2 > 6)
    {
      if (trunk_is_stale_retune_burst(-1))
      {
        cach_err = 0;
        tact_okay = 1;
        emb_ok = 1;
        goto END;
      }
      if (state->dmrburstR == 16) state->dmrburstR = 17;
      fprintf (stderr,"%s ", timestr);

      //simplifying things
      // char polarity[3];
      char light[18];

      sprintf (light, "%s", "  slot1  [SLOT2] ");
      fprintf (stderr,"Sync:  DMR %s", light);
      fprintf (stderr, "%s", KCYN);
      fprintf (stderr, "| Frame Sync Err: %d", vc2);
      fprintf (stderr, "%s", KNRM);
      fprintf (stderr, "\n");
      vc2++;
      //only trigger a resync if the opposite slot is not actively decoding voice
      if ( vc2 > 13 && vc1 > 6 ) goto END;
      if ( vc2 > 13 ) vc2 = 13;
      goto SKIP;
    }
  }

  //only play voice on no data sync, and VC values are within expected values 1-6
  if (!dmr_bs_sync_match (sync, DMR_BS_DATA_SYNC)) //we already have a tact ecc check, so we won't get here without that, see if there is any other eccs we can run just to make sure
  {

    //check the embedded signalling, if bad at this point, we probably aren't quite in sync
    if (dmr_bs_sync_match (sync, DMR_BS_VOICE_SYNC))
    {
      emb_ok = 1;
      emb_bad_count[internalslot] = 0;
      if (slot_cc_rejected[internalslot] == 0)
        state->color_code = state->dmr_color_code;
    }
    else if (QR_16_7_6_decode(emb_pdu))
    {
      cc = ((emb_pdu[0] << 3) + (emb_pdu[1] << 2) + (emb_pdu[2] << 1) + emb_pdu[3]);
      power = emb_pdu[4];
      lcss = ((emb_pdu[5] << 1) + emb_pdu[6]);
      if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 && (int)cc != opts->dmr_cc_filter)
      {
        trunk_check_rigctl_freq(opts, state);
      }
      if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 && (int)cc != opts->dmr_cc_filter &&
          trunk_is_stale_retune_burst((int)cc))
      {
        if (state->dmr_unconfirmed_sync == 1)
        {
          state->last_cc_sync_time = state->dmr_unconfirmed_cc_sync_time;
          state->last_vc_sync_time = state->dmr_unconfirmed_vc_sync_time;
          state->dmr_unconfirmed_sync = 0;
        }
        cach_err = 0;
        tact_okay = 1;
        emb_ok = 1;
        goto END;
      }
      (void)trunk_is_stale_retune_burst((int)cc);
      if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 && (int)cc != opts->dmr_cc_filter)
      {
        if (vc_cc_confirmed[internalslot] == 1)
        {
          emb_bad_count[internalslot]++;
          emb_lc_valid[internalslot] = 0;
          if (emb_bad_count[internalslot] == 1)
          {
            //tolerate a single collision-corrupted EMB mid-superframe on an already-confirmed call
            emb_ok = 1;
            power = 9;
            lcss = 9;
            state->color_code = state->dmr_color_code;
            state->color_code_ok = 1;
          }
          else
          {
            emb_ok = 1;
            vc_cc_confirmed[internalslot] = 0;
            slot_cc_rejected[internalslot] = 1;
            state->color_code = cc;
            state->color_code_ok = 1;
          }
        }
        else
        {
          emb_ok = 1;
          slot_cc_rejected[internalslot] = 1;
          state->color_code = cc;
          state->color_code_ok = 1;
        }
      }
      else
      {
        emb_ok = 1;
        emb_bad_count[internalslot] = 0;
        vc_cc_confirmed[internalslot] = 1;
        slot_cc_rejected[internalslot] = 0;
        state->color_code = cc;
        state->dmr_color_code = cc;
        state->color_code_ok = 1;
      }
    }
    else
    {
      emb_bad_count[internalslot]++;
      emb_lc_valid[internalslot] = 0;
      if (emb_bad_count[internalslot] == 1)
      {
        //tolerate a single corrupted EMB mid-superframe so co-channel interference doesn't abort the superframe
        emb_ok = 1;
        power = 9;
        lcss = 9;
        state->color_code = state->dmr_color_code;
      }
      else
      {
        emb_ok = 0;
        if (internalslot == 0 && vc2 <= 6)
        {
          if (state->dmrburstL == 16) state->dmrburstL = 17;
          vc1 = 7;
          emb_ok = 1;
          memset (state->f_l4, 0, sizeof(state->f_l4));
          memset (state->s_l4, 0, sizeof(state->s_l4));
          memset (state->s_l4u, 0, sizeof(state->s_l4u));
          goto SKIP;
        }
        if (internalslot == 1 && vc1 <= 6)
        {
          if (state->dmrburstR == 16) state->dmrburstR = 17;
          vc2 = 7;
          emb_ok = 1;
          memset (state->f_r4, 0, sizeof(state->f_r4));
          memset (state->s_r4, 0, sizeof(state->s_r4));
          memset (state->s_r4u, 0, sizeof(state->s_r4u));
          goto SKIP;
        }
        goto END; //fprintf (stderr, "EMB BAD? ");
      }
    }

    fprintf (stderr,"%s ", timestr);

    //simplifying things
    char polarity[3];
    char light[18];
    uint8_t vc;

    if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 &&
        state->color_code_ok == 1 && (int)state->color_code != opts->dmr_cc_filter)
    {
      trunk_check_rigctl_freq(opts, state);
    }

    if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 &&
        state->color_code_ok == 1 && (int)state->color_code != opts->dmr_cc_filter)
    {
      if (state->dmr_unconfirmed_sync == 1)
      {
        state->last_cc_sync_time = state->dmr_unconfirmed_cc_sync_time;
        state->last_vc_sync_time = state->dmr_unconfirmed_vc_sync_time;
        state->dmr_unconfirmed_sync = 0;
      }
      emb_lc_valid[internalslot] = 0;
      vc_cc_confirmed[internalslot] = 0;
      slot_cc_rejected[internalslot] = 1;
      state->dmr_cach_counter = 0;
      if (opts->inverted_dmr == 0) sprintf (polarity, "%s", "+");
      else sprintf (polarity, "%s", "-");
      if (internalslot == 0)
      {
        if (state->dmrburstL == 16) state->dmrburstL = 17;
        if (opts->mbe_out_f != NULL) closeMbeOutFile (opts, state);
        memset (state->f_l4, 0, sizeof(state->f_l4));
        memset (state->s_l4, 0, sizeof(state->s_l4));
        memset (state->s_l4u, 0, sizeof(state->s_l4u));
        memset (state->audio_out_temp_buf, 0, sizeof(state->audio_out_temp_buf));
        memset (state->s_l, 0, sizeof(state->s_l));
        memset (state->s_lu, 0, sizeof(state->s_lu));
        vc = vc1;
        sprintf (light, "%s", " [SLOT1]  slot2  ");
        vc1 = 7;
      }
      else
      {
        if (state->dmrburstR == 16) state->dmrburstR = 17;
        if (opts->mbe_out_fR != NULL) closeMbeOutFileR (opts, state);
        memset (state->f_r4, 0, sizeof(state->f_r4));
        memset (state->s_r4, 0, sizeof(state->s_r4));
        memset (state->s_r4u, 0, sizeof(state->s_r4u));
        memset (state->audio_out_temp_bufR, 0, sizeof(state->audio_out_temp_bufR));
        memset (state->s_r, 0, sizeof(state->s_r));
        memset (state->s_ru, 0, sizeof(state->s_ru));
        vc = vc2;
        sprintf (light, "%s", "  slot1  [SLOT2] ");
        vc2 = 7;
      }
      if (opts->dmr_le != 2)
      {
        memcpy (m1, ambe_fr, sizeof(m1));
        memcpy (m2, ambe_fr2, sizeof(m2));
        memcpy (m3, ambe_fr3, sizeof(m3));
        dmr_late_entry_mi_fragment (opts, state, vc%7, m1, m2, m3);
      }
      if (state->color_code != 16)
        fprintf (stderr,"Sync: %sDMR %s| Color Code=%02d | VC%d [CC FILTER: %02d]\n", polarity, light, state->color_code, vc, opts->dmr_cc_filter);
      else fprintf (stderr,"Sync: %sDMR %s| Color Code=XX | VC%d [CC FILTER: %02d]\n", polarity, light, vc, opts->dmr_cc_filter);

      cach_err = 0;
      tact_okay = 1;
      emb_ok = 1;
      cc = 25;
      power = 9;
      lcss = 9;
      if ( vc1 > 6 && vc2 > 6 ) goto END;
      long int cur_f = ((opts->p25_trunk == 1 || opts->scanner_mode == 1) && opts->p25_is_tuned == 0 && state->lcn_freq_count > 0)
                       ? trunk_get_current_freq(opts, state) : 0;
      int not_on_imported = (cur_f > 0 && !trunk_is_imported_freq(state, cur_f));
      int on_cap_plus_rest = (state->dmr_rest_channel != -1 &&
                              (opts->p25_is_tuned == 0 || state->p25_vc_freq[0] == 0 || state->p25_vc_freq[0] == state->p25_cc_freq));
      time_t hangtime_limit = on_cap_plus_rest
                              ? (opts->trunk_hangtime + 4)
                              : opts->trunk_hangtime;
      int can_hunt_or_return = (opts->scanner_mode == 1) ||
                               (opts->p25_trunk == 1 && (opts->p25_is_tuned == 1 || state->dmr_rest_channel != -1 || state->lcn_freq_count > 0 || state->p25_cc_freq != 0));
      if ( not_on_imported || (can_hunt_or_return && ((time(NULL) - state->last_cc_sync_time) > hangtime_limit)) ) goto END;
      goto SKIP;
    }

    skipcount = 0; //reset skip count if processing voice frames

    if (internalslot == 0)
    {
      state->dmrburstL = 16;
      state->dmr_hangtime[0] = 0;
      //open MBEout file - slot 1
      if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_f == NULL)) openMbeOutFile (opts, state);
      vc = vc1;
      sprintf (light, "%s", " [SLOT1]  slot2  ");
    }
    else
    {
      state->dmrburstR = 16;
      state->dmr_hangtime[1] = 0;
      //open MBEout file - slot 2
      if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_fR == NULL)) openMbeOutFileR (opts, state);
      vc = vc2;
      sprintf (light, "%s", "  slot1  [SLOT2] ");
    }
    if (opts->inverted_dmr == 0) sprintf (polarity, "%s", "+");
    else sprintf (polarity, "%s", "-");
    if (state->dmr_color_code != 16)
      fprintf (stderr,"Sync: %sDMR %s| Color Code=%02d | VC%d ", polarity, light, state->dmr_color_code, vc);
    else fprintf (stderr,"Sync: %sDMR %s| Color Code=XX | VC%d ", polarity, light, vc);

    if (internalslot == 0 && vc1 == 6)
    {
      if (state->hota_enc_check && state->forced_alg_id > 1)
      {
        state->dmr_so = 0x40;
        state->payload_algid = state->forced_alg_id;
        state->payload_keyid = 0xFF;
      }
      //this needs to run prior to embedded link control
      if (state->payload_algid == 0x02)
        hytera_enhanced_alg_refresh(state);
      //process embedded link control
      if (emb_lc_valid[0] == 1)
      {
        fprintf (stderr, "\n");
        dmr_data_burst_handler(opts, state, (uint8_t *)dummy_bits, 0xEB);
        if (state->color_code_ok == 0)
        {
          cach_err = 0;
          tact_okay = 1;
          emb_ok = 1;
          goto END;
        }
      }
      //check the single burst/reverse channel opportunity -- moved
      // dmr_sbrc (opts, state, power);

    }

    if (internalslot == 1 && vc2 == 6)
    {
      if (state->hota_enc_check && state->forced_alg_id > 1)
      {
        state->dmr_soR = 0x40;
        state->payload_algidR = state->forced_alg_id;
        state->payload_keyidR = 0xFF;
      }
      //this needs to run prior to embedded link control
      if (state->payload_algidR == 0x02)
        hytera_enhanced_alg_refresh(state);
      //process embedded link control
      if (emb_lc_valid[1] == 1)
      {
        fprintf (stderr, "\n");
        dmr_data_burst_handler(opts, state, (uint8_t *)dummy_bits, 0xEB);
        if (state->color_code_ok == 0)
        {
          cach_err = 0;
          tact_okay = 1;
          emb_ok = 1;
          goto END;
        }
      }
      //check the single burst/reverse channel opportunity -- moved
      // dmr_sbrc (opts, state, power);

    }

    if (opts->payload == 1) fprintf (stderr, "\n"); //extra line break necessary here

    //copy ambe_fr frames first, running process mbe will correct them,
    //but this also leads to issues extracting good le mi values when
    //we go to do correction on them there too
    memcpy (m1, ambe_fr, sizeof(m1));
    memcpy (m2, ambe_fr2, sizeof(m2));
    memcpy (m3, ambe_fr3, sizeof(m3));

    if (state->tyt_bp == 1)
    {
      tyt16_ambe2_codeword_keystream(state, ambe_fr, 0);
      tyt16_ambe2_codeword_keystream(state, ambe_fr2, 1);
      tyt16_ambe2_codeword_keystream(state, ambe_fr3, 0);
    }

    if (state->csi_ee == 1)
    {
      csi72_ambe2_codeword_keystream(state, ambe_fr);
      csi72_ambe2_codeword_keystream(state, ambe_fr2);
      csi72_ambe2_codeword_keystream(state, ambe_fr3);
    }

    if (state->vtx256_ekey != 0)
    {
      vtx256_ambe2_interleave_inversion(ambe_fr);
      vtx256_ambe2_interleave_inversion(ambe_fr2);
      vtx256_ambe2_interleave_inversion(ambe_fr3);
    }

    #ifdef PRINT_AMBE72
    ambe2_codeword_print_i(opts, ambe_fr);
    ambe2_codeword_print_i(opts, ambe_fr2);
    ambe2_codeword_print_i(opts, ambe_fr3);
    #endif

    processMbeFrame (opts, state, NULL, ambe_fr, NULL);
    if(internalslot == 0)
    {
      memcpy(state->f_l4[0], state->audio_out_temp_buf, sizeof(state->audio_out_temp_buf));
      memcpy(state->s_l4[0], state->s_l, sizeof(state->s_l));
      memcpy(state->s_l4u[0], state->s_lu, sizeof(state->s_lu));
    }

    else
    {
      memcpy(state->f_r4[0], state->audio_out_temp_bufR, sizeof(state->audio_out_temp_bufR));
      memcpy(state->s_r4[0], state->s_r, sizeof(state->s_r));
      memcpy(state->s_r4u[0], state->s_ru, sizeof(state->s_ru));
    }


    processMbeFrame (opts, state, NULL, ambe_fr2, NULL);
    if(internalslot == 0)
    {
      memcpy(state->f_l4[1], state->audio_out_temp_buf, sizeof(state->audio_out_temp_buf));
      memcpy(state->s_l4[1], state->s_l, sizeof(state->s_l));
      memcpy(state->s_l4u[1], state->s_lu, sizeof(state->s_lu));
    }

    else
    {
      memcpy(state->f_r4[1], state->audio_out_temp_bufR, sizeof(state->audio_out_temp_bufR));
      memcpy(state->s_r4[1], state->s_r, sizeof(state->s_r));
      memcpy(state->s_r4u[1], state->s_ru, sizeof(state->s_ru));
    }

    processMbeFrame (opts, state, NULL, ambe_fr3, NULL);
    if(internalslot == 0)
    {
      memcpy(state->f_l4[2], state->audio_out_temp_buf, sizeof(state->audio_out_temp_buf));
      memcpy(state->s_l4[2], state->s_l, sizeof(state->s_l));
      memcpy(state->s_l4u[2], state->s_lu, sizeof(state->s_lu));
    }

    else
    {
      memcpy(state->f_r4[2], state->audio_out_temp_bufR, sizeof(state->audio_out_temp_bufR));
      memcpy(state->s_r4[2], state->s_r, sizeof(state->s_r));
      memcpy(state->s_r4u[2], state->s_ru, sizeof(state->s_ru));
    }


    //'DSP' output to file -- run before sbrc
    if (opts->use_dsp_output == 1)
    {
      FILE * pFile; //file pointer
      pFile = fopen (opts->dsp_out_file, "a");
      fprintf (pFile, "\n%d 98 ", internalslot+1); //'98' is CACH designation value
      for (i = 0; i < 6; i++) //3 byte CACH
      {
        int cach_byte = (state->dmr_stereo_payload[i*2] << 2) | state->dmr_stereo_payload[i*2 + 1];
        fprintf (pFile, "%X", cach_byte);
      }
      fprintf (pFile, "\n%d 10 ", internalslot+1); //0x10 for voice burst
      for (i = 6; i < 72; i++) //33 bytes, no CACH
      {
        int dsp_byte = (state->dmr_stereo_payload[i*2] << 2) | state->dmr_stereo_payload[i*2 + 1];
        fprintf (pFile, "%X", dsp_byte);
      }
      fclose (pFile);
    }

    //run sbrc here to look for the late entry key and alg after we observe potential errors in VC6
    if (internalslot == 0 && vc1 == 6) dmr_sbrc (opts, state, power);
    if (internalslot == 1 && vc2 == 6) dmr_sbrc (opts, state, power);

    if (!cach_tact_bad)
      cach_err = dmr_cach (opts, state, cachdata);
    if (opts->payload == 0) fprintf (stderr, "\n");

    // run alg refresh after vc6 ambe processing
    if (internalslot == 0 && vc1 == 6) dmr_alg_refresh (opts, state);
    if (internalslot == 1 && vc2 == 6) dmr_alg_refresh (opts, state);

    if (internalslot == 0 && vc1 == 6) state->static_ks_counter[0] = 0;
    if (internalslot == 1 && vc2 == 6) state->static_ks_counter[1] = 0;

    if (opts->dmr_le != 2) //if not Hytera Enhanced
      dmr_late_entry_mi_fragment (opts, state, vc%7, m1, m2, m3);

    //increment the vc counters
    if (internalslot == 0) vc1++;
    if (internalslot == 1) vc2++;

    //update cc and vc sync time
    if (state->color_code_ok == 1 || opts->dmr_cc_filter < 0 || opts->dmr_cc_filter > 15)
    {
      state->dmr_unconfirmed_sync = 0;
    }
    else if (state->dmr_unconfirmed_sync == 0)
    {
      state->dmr_unconfirmed_cc_sync_time = state->last_cc_sync_time;
      state->dmr_unconfirmed_vc_sync_time = state->last_vc_sync_time;
      state->dmr_unconfirmed_sync = 1;
    }
    state->last_cc_sync_time = time(NULL);
    if (!trunk_is_dmr_slot_blocked(opts, state, internalslot))
    {
      state->last_vc_sync_time = time(NULL);
    }
    else if (opts->p25_trunk == 1 && (opts->p25_is_tuned == 1 || state->dmr_rest_channel != -1) && state->p25_cc_freq != 0)
    {
      long int cur_freq = trunk_get_current_freq(opts, state);
      if ((cur_freq != 0 && cur_freq != state->p25_cc_freq) || (cur_freq == 0 && opts->p25_is_tuned == 1))
      {
        uint8_t dummy[12]; uint8_t dbits[96];
        memset (dummy, 0, sizeof(dummy));
        memset (dbits, 0, sizeof(dbits));
        dummy[0] = 46; dummy[1] = 255;
        dmr_cspdu (opts, state, dbits, dummy, 1, 0);
        if (state->color_code_ok == 0)
        {
          cach_err = 0;
          tact_okay = 1;
          emb_ok = 1;
          goto END;
        }
      }
    }

    //reset err checks
    cach_err = 1;
    tact_okay = 0;
    emb_ok = 0;

    //reset emb components
    cc = 25;
    power = 9;
    lcss = 9;

    //Extra safeguards to break loop
    // if ( (vc1 > 7 && vc2 > 7) ) goto END;
    if ( vc1 > 6 && vc2 > 6 && (vc1 > 14 || vc2 > 14) ) goto END;

  }

  SKIP:

  //both working now, will need support added for ENC audio and no key
  //NOTE: We want this to play regardless of whether the slot is voice or data, to play silence in one slot and voice in the second, or voices in both
  if (opts->dmr_cc_filter < 0 || opts->dmr_cc_filter > 15 ||
      state->color_code_ok == 0 || (int)state->dmr_color_code == opts->dmr_cc_filter ||
      state->dmrburstL == 16 || state->dmrburstR == 16)
  {
    if (internalslot == 1 && opts->floating_point == 1 && opts->pulse_digi_rate_out == 8000)
      playSynthesizedVoiceFS3 (opts, state); //Float Stereo Mix 3v2
    if (internalslot == 1 && opts->floating_point == 0 && opts->pulse_digi_rate_out == 8000)
      playSynthesizedVoiceSS3 (opts, state); //Short Stereo Mix 3v2
  }

  if (skipcount > 3) //after 3 onsecutive data frames, drop back to getFrameSync and process with dmr_data_sync (need one more in order to push last voice on slot 2 only voice)
  {
    //set tests to all good so we don't get a bogus/redundant voice error
    cach_err = 0;
    tact_okay = 1;
    emb_ok = 1;
    goto END;
  }

  //since we are in a while loop, run ncursesPrinter here.
  if (opts->use_ncurses_terminal == 1)
  {
    ncursesPrinter(opts, state);
  }

  //slot 1
  watchdog_event_history(opts, state, 0);
  watchdog_event_current(opts, state, 0);

  //slot 2 for TDMA systems
  watchdog_event_history(opts, state, 1);
  watchdog_event_current(opts, state, 1);

  //
  if (timestr != NULL)
  {
    free (timestr);
    timestr = NULL;
  }

  expected_slot ^= 1;

 } // while loop

 END:
 if (state->dmr_unconfirmed_sync == 1)
 {
   state->last_cc_sync_time = state->dmr_unconfirmed_cc_sync_time;
   state->last_vc_sync_time = state->dmr_unconfirmed_vc_sync_time;
   state->dmr_unconfirmed_sync = 0;
 }
 state->dmr_stereo = 0;
 state->errs = 0;
 state->errs2 = 0;
 state->errs2R = 0;
 state->errs2 = 0;

 //close any open MBEout files
 if (opts->mbe_out_f != NULL) closeMbeOutFile (opts, state);
 if (opts->mbe_out_fR != NULL) closeMbeOutFileR (opts, state);

 //clear out any stale audio storage buffers
 memset (state->f_l4, 0.0f, sizeof(state->f_l4));
 memset (state->f_r4, 0.0f, sizeof(state->f_r4));
 memset (state->s_l4, 0, sizeof(state->s_l4));
 memset (state->s_r4, 0, sizeof(state->s_r4));

 //if we have a tact or emb err, then produce sync pattern/err message
 if (tact_okay != 1 || emb_ok != 1)
 {
  if (state->dmr_unconfirmed_sync == 1)
  {
    state->last_cc_sync_time = state->dmr_unconfirmed_cc_sync_time;
    state->last_vc_sync_time = state->dmr_unconfirmed_vc_sync_time;
    state->dmr_unconfirmed_sync = 0;
  }
  state->color_code_ok = 0;
  if (!trunk_is_stale_retune_burst(-1))
  {
    fprintf (stderr,"%s ", timestr);
    fprintf (stderr,"Sync:  DMR                  ");
    fprintf (stderr, "%s", KRED);
    fprintf (stderr, "| VOICE CACH/EMB ERR");
    fprintf (stderr, "%s", KNRM);
    fprintf (stderr, "\n");
  }
  // Do not run dmr_alg_refresh() here: END is reached when the opposite idle slot
  // hits Frame Sync Err (vc > 13) right after a PI header was decoded, or mid-superframe,
  // which would prematurely step the MI one superframe ahead of the transmitter!

  //failsafe to reset all data header and blocks when bad tact or emb
  dmr_reset_blocks (opts, state);

 }

 //
  if (timestr != NULL)
  {
    free (timestr);
    timestr = NULL;
  }

  //reset static ks counter
  state->static_ks_counter[0] = 0;
  state->static_ks_counter[1] = 0;

}

//Process buffered half frame and 2nd half and then jump to full BS decoding
void dmrBSBootstrap (dsd_opts * opts, dsd_state * state)
{
  int i, dibit;
  int *dibit_p;
  char ambe_fr[4][24];
  char ambe_fr2[4][24];
  char ambe_fr3[4][24];

  memset (ambe_fr, 0, sizeof(ambe_fr));
  memset (ambe_fr2, 0, sizeof(ambe_fr2));
  memset (ambe_fr3, 0, sizeof(ambe_fr3));

  //memcpy of ambe_fr for late entry
  uint8_t m1[4][24];
  uint8_t m2[4][24];
  uint8_t m3[4][24];

  const int *w, *x, *y, *z;
  char sync[25];
  uint8_t tact_okay = 0;
  uint8_t cach_err = 0;
  uint8_t sync_okay = 1;
  UNUSED(cach_err);

  uint8_t internalslot;

  uint8_t cachdata[25];
  int cachInterleave[24] =
  {0, 7, 8, 9, 1, 10,
  11, 12, 2, 13, 14,
  15, 3, 16, 4, 17, 18,
  19, 5, 20, 21, 22, 6, 23
  };

  char * timestr = getTimeC();

  //payload buffer
  //CACH + First Half Payload + Sync = 12 + 54 + 24
  dibit_p = state->dmr_payload_p - 90;
  for (i = 0; i < 90; i++) //90
  {
    dibit = *dibit_p;
    dibit_p++;
    if(opts->inverted_dmr == 1) dibit = (dibit ^ 2) & 3;
    state->dmr_stereo_payload[i] = dibit;
  }

  for(i = 0; i < 12; i++)
  {
    dibit = state->dmr_stereo_payload[i];
    cachdata[cachInterleave[(i*2)]]   = (1 & (dibit >> 1)); // bit 1
    cachdata[cachInterleave[(i*2)+1]] = (1 & dibit);       // bit 0
  }

  //cach tact bits
  uint8_t tact_bits[7];
  for (i = 0; i < 7; i++)
  {
    tact_bits[i] = cachdata[i];
  }

  //decode and correct tact and compare
  if ( Hamming_7_4_decode (tact_bits) ) tact_okay = 1;
  if (tact_okay != 1) goto END;

  internalslot = state->currentslot = tact_bits[1];

  //reset static ks counter
  state->static_ks_counter[internalslot] = 0;

  //Setup for first AMBE Frame

  //Interleave Schedule
  w = rW;
  x = rX;
  y = rY;
  z = rZ;

  //First AMBE Frame, Full 36
  for(i = 0; i < 36; i++)
  {
    dibit = state->dmr_stereo_payload[i+12];
    ambe_fr[*w][*x] = (1 & (dibit >> 1)); // bit 1
    ambe_fr[*y][*z] = (1 & dibit);        // bit 0

    w++;
    x++;
    y++;
    z++;

  }

  //Setup for Second AMBE Frame

  //Interleave Schedule
  w = rW;
  x = rX;
  y = rY;
  z = rZ;

  //Second AMBE Frame, First Half 18 dibits just before Sync or EmbeddedSignalling
  for(i = 0; i < 18; i++)
  {
    dibit = state->dmr_stereo_payload[i+48];
    ambe_fr2[*w][*x] = (1 & (dibit >> 1)); // bit 1
    ambe_fr2[*y][*z] = (1 & dibit);        // bit 0

    w++;
    x++;
    y++;
    z++;

  }

  // signaling data or sync, just redo it
  for(i = 0; i < 24; i++)
  {
    dibit = state->dmr_stereo_payload[i+66];
    sync[i] = (dibit | 1) + 48;
  }
  sync[24] = 0;

  if ( !dmr_bs_sync_match (sync, DMR_BS_VOICE_SYNC) )
  {
    sync_okay = 0;
    goto END;
  }

  //Continue Second AMBE Frame, 18 after Sync or EmbeddedSignalling
  for(i = 0; i < 18; i++)
  {
    dibit = getDibit(opts, state);
    state->dmr_stereo_payload[i+90] = dibit;
    ambe_fr2[*w][*x] = (1 & (dibit >> 1)); // bit 1
    ambe_fr2[*y][*z] = (1 & dibit);        // bit 0

    w++;
    x++;
    y++;
    z++;

  }

  //Setup for Third AMBE Frame

  //Interleave Schedule
  w = rW;
  x = rX;
  y = rY;
  z = rZ;

  //Third AMBE Frame, Full 36
  for(i = 0; i < 36; i++)
  {
    dibit = getDibit(opts, state);
    state->dmr_stereo_payload[i+108] = dibit;
    ambe_fr3[*w][*x] = (1 & (dibit >> 1)); // bit 1
    ambe_fr3[*y][*z] = (1 & dibit);        // bit 0

    w++;
    x++;
    y++;
    z++;

  }

  //'DSP' output to file
  if (opts->use_dsp_output == 1)
  {
    FILE * pFile; //file pointer
    pFile = fopen (opts->dsp_out_file, "a");
    fprintf (pFile, "\n%d 98 ", internalslot+1); //'98' is CACH designation value
    for (i = 0; i < 6; i++) //3 byte CACH
    {
      int cach_byte = (state->dmr_stereo_payload[i*2] << 2) | state->dmr_stereo_payload[i*2 + 1];
      fprintf (pFile, "%X", cach_byte);
    }
    fprintf (pFile, "\n%d 10 ", internalslot+1); //0x10 for "voice burst"
    for (i = 6; i < 72; i++) //33 bytes, no CACH
    {
      int dsp_byte = (state->dmr_stereo_payload[i*2] << 2) | state->dmr_stereo_payload[i*2 + 1];
      fprintf (pFile, "%X", dsp_byte);
    }
    fclose (pFile);
  }

  fprintf (stderr,"%s ", timestr);
  char polarity[3];
  char light[18];

  if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 &&
      state->color_code_ok == 1 && (int)state->color_code != opts->dmr_cc_filter)
  {
    trunk_check_rigctl_freq(opts, state);
  }

  if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 &&
      state->color_code_ok == 1 && (int)state->color_code != opts->dmr_cc_filter)
  {
    if (state->dmr_unconfirmed_sync == 1)
    {
      state->last_cc_sync_time = state->dmr_unconfirmed_cc_sync_time;
      state->last_vc_sync_time = state->dmr_unconfirmed_vc_sync_time;
      state->dmr_unconfirmed_sync = 0;
    }
    state->dmr_cach_counter = 0;
    if (opts->inverted_dmr == 0) sprintf (polarity, "%s", "+");
    else sprintf (polarity, "%s", "-");
    if (state->currentslot == 0)
    {
      if (state->dmrburstL == 16) state->dmrburstL = 17;
      if (opts->mbe_out_f != NULL) closeMbeOutFile (opts, state);
      sprintf (light, "%s", " [SLOT1]  slot2  ");
      memset (state->f_l4, 0, sizeof(state->f_l4));
      memset (state->s_l4, 0, sizeof(state->s_l4));
      memset (state->s_l4u, 0, sizeof(state->s_l4u));
      memset (state->audio_out_temp_buf, 0, sizeof(state->audio_out_temp_buf));
      memset (state->s_l, 0, sizeof(state->s_l));
      memset (state->s_lu, 0, sizeof(state->s_lu));
    }
    else
    {
      if (state->dmrburstR == 16) state->dmrburstR = 17;
      if (opts->mbe_out_fR != NULL) closeMbeOutFileR (opts, state);
      sprintf (light, "%s", "  slot1  [SLOT2] ");
      memset (state->f_r4, 0, sizeof(state->f_r4));
      memset (state->s_r4, 0, sizeof(state->s_r4));
      memset (state->s_r4u, 0, sizeof(state->s_r4u));
      memset (state->audio_out_temp_bufR, 0, sizeof(state->audio_out_temp_bufR));
      memset (state->s_r, 0, sizeof(state->s_r));
      memset (state->s_ru, 0, sizeof(state->s_ru));
    }
    dmr_alg_reset (opts, state);
    if (opts->dmr_le != 2)
    {
      memcpy (m1, ambe_fr, sizeof(m1));
      memcpy (m2, ambe_fr2, sizeof(m2));
      memcpy (m3, ambe_fr3, sizeof(m3));
      dmr_late_entry_mi_fragment (opts, state, 1, m1, m2, m3);
    }
    if (state->color_code != 16)
      fprintf (stderr,"Sync: %sDMR %s| Color Code=%02d | VC1* [CC FILTER: %02d]\n", polarity, light, state->color_code, opts->dmr_cc_filter);
    else fprintf (stderr,"Sync: %sDMR %s| Color Code=XX | VC1* [CC FILTER: %02d]\n", polarity, light, opts->dmr_cc_filter);
    tact_okay = 1;
    sync_okay = 1;
    goto END;
  }

  if (state->currentslot == 0)
  {
    state->dmrburstL = 16;
    state->dmr_hangtime[0] = 0;
    sprintf (light, "%s", " [SLOT1]  slot2  ");
    //open MBEout file - slot 1
    if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_f == NULL)) openMbeOutFile (opts, state);
  }
  else
  {
    state->dmrburstR = 16;
    state->dmr_hangtime[1] = 0;
    sprintf (light, "%s", "  slot1  [SLOT2] ");
    //open MBEout file - slot 2
    if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_fR == NULL)) openMbeOutFileR (opts, state);
  }
  if (opts->inverted_dmr == 0) sprintf (polarity, "%s", "+");
  else sprintf (polarity, "%s", "-");
  if (state->dmr_color_code != 16)
    fprintf (stderr,"Sync: %sDMR %s| Color Code=%02d | VC1*", polarity, light, state->dmr_color_code);
  else fprintf (stderr,"Sync: %sDMR %s| Color Code=XX | VC1*", polarity, light);

  dmr_alg_reset (opts, state);

  //copy ambe_fr frames first, running process mbe will correct them,
  //but this also leads to issues extracting good le mi values when
  //we go to do correction on them there too
  memcpy (m1, ambe_fr, sizeof(m1));
  memcpy (m2, ambe_fr2, sizeof(m2));
  memcpy (m3, ambe_fr3, sizeof(m3));

  if (state->tyt_bp == 1)
  {
    tyt16_ambe2_codeword_keystream(state, ambe_fr, 0);
    tyt16_ambe2_codeword_keystream(state, ambe_fr2, 1);
    tyt16_ambe2_codeword_keystream(state, ambe_fr3, 0);
  }

  if (state->csi_ee == 1)
  {
    csi72_ambe2_codeword_keystream(state, ambe_fr);
    csi72_ambe2_codeword_keystream(state, ambe_fr2);
    csi72_ambe2_codeword_keystream(state, ambe_fr3);
  }

  if (state->vtx256_ekey != 0)
  {
    vtx256_ambe2_interleave_inversion(ambe_fr);
    vtx256_ambe2_interleave_inversion(ambe_fr2);
    vtx256_ambe2_interleave_inversion(ambe_fr3);
  }

  #ifdef PRINT_AMBE72
  ambe2_codeword_print_i(opts, ambe_fr);
  ambe2_codeword_print_i(opts, ambe_fr2);
  ambe2_codeword_print_i(opts, ambe_fr3);
  #endif

  if (opts->payload == 1) fprintf (stderr, "\n"); //extra line break necessary here
  // processMbeFrame (opts, state, NULL, ambe_fr, NULL);
  // processMbeFrame (opts, state, NULL, ambe_fr2, NULL);
  // processMbeFrame (opts, state, NULL, ambe_fr3, NULL);

  processMbeFrame (opts, state, NULL, ambe_fr, NULL);
  if(internalslot == 0)
  {
    memcpy(state->f_l4[0], state->audio_out_temp_buf, sizeof(state->audio_out_temp_buf));
    memcpy(state->s_l4[0], state->s_l, sizeof(state->s_l));
    memcpy(state->s_l4u[0], state->s_lu, sizeof(state->s_lu));
  }

  else
  {
    memcpy(state->f_r4[0], state->audio_out_temp_bufR, sizeof(state->audio_out_temp_bufR));
    memcpy(state->s_r4[0], state->s_r, sizeof(state->s_r));
    memcpy(state->s_r4u[0], state->s_ru, sizeof(state->s_ru));
  }


  processMbeFrame (opts, state, NULL, ambe_fr2, NULL);
  if(internalslot == 0)
  {
    memcpy(state->f_l4[1], state->audio_out_temp_buf, sizeof(state->audio_out_temp_buf));
    memcpy(state->s_l4[1], state->s_l, sizeof(state->s_l));
    memcpy(state->s_l4u[1], state->s_lu, sizeof(state->s_lu));
  }

  else
  {
    memcpy(state->f_r4[1], state->audio_out_temp_bufR, sizeof(state->audio_out_temp_bufR));
    memcpy(state->s_r4[1], state->s_r, sizeof(state->s_r));
    memcpy(state->s_r4u[1], state->s_ru, sizeof(state->s_ru));
  }

  processMbeFrame (opts, state, NULL, ambe_fr3, NULL);
  if(internalslot == 0)
  {
    memcpy(state->f_l4[2], state->audio_out_temp_buf, sizeof(state->audio_out_temp_buf));
    memcpy(state->s_l4[2], state->s_l, sizeof(state->s_l));
    memcpy(state->s_l4u[2], state->s_lu, sizeof(state->s_lu));
  }

  else
  {
    memcpy(state->f_r4[2], state->audio_out_temp_bufR, sizeof(state->audio_out_temp_bufR));
    memcpy(state->s_r4[2], state->s_r, sizeof(state->s_r));
    memcpy(state->s_r4u[2], state->s_ru, sizeof(state->s_ru));
  }

  //collect the mi fragment
  if (opts->dmr_le != 2) //if not Hytera Enhanced
    dmr_late_entry_mi_fragment (opts, state, 1, m1, m2, m3);

  cach_err = dmr_cach (opts, state, cachdata);
  if (opts->payload == 0) fprintf (stderr, "\n");

  if (opts->dmr_cc_filter >= 0 && opts->dmr_cc_filter <= 15 &&
      state->color_code_ok == 0 && state->dmr_unconfirmed_sync == 0)
  {
    state->dmr_unconfirmed_cc_sync_time = state->last_cc_sync_time;
    state->dmr_unconfirmed_vc_sync_time = state->last_vc_sync_time;
    state->dmr_unconfirmed_sync = 1;
  }
  state->last_cc_sync_time = time(NULL);
  if (!trunk_is_dmr_slot_blocked(opts, state, internalslot))
  {
    state->last_vc_sync_time = time(NULL);
  }
  else if (opts->p25_trunk == 1 && (opts->p25_is_tuned == 1 || state->dmr_rest_channel != -1) && state->p25_cc_freq != 0)
  {
    long int cur_freq = trunk_get_current_freq(opts, state);
    if ((cur_freq != 0 && cur_freq != state->p25_cc_freq) || (cur_freq == 0 && opts->p25_is_tuned == 1))
    {
      uint8_t dummy[12]; uint8_t dbits[96];
      memset (dummy, 0, sizeof(dummy));
      memset (dbits, 0, sizeof(dbits));
      dummy[0] = 46; dummy[1] = 255;
      dmr_cspdu (opts, state, dbits, dummy, 1, 0);
      if (state->color_code_ok == 0)
      {
        tact_okay = 1;
        sync_okay = 1;
        goto END;
      }
    }
  }

  //NOTE: Only play on slot 1, if slot 0, then it'll play after the next TDMA frame in the BS loop instead
  if (opts->dmr_cc_filter < 0 || opts->dmr_cc_filter > 15 ||
      state->color_code_ok == 0 || (int)state->dmr_color_code == opts->dmr_cc_filter ||
      state->dmrburstL == 16 || state->dmrburstR == 16)
  {
    if (internalslot == 1 && opts->floating_point == 1 && opts->pulse_digi_rate_out == 8000)
      playSynthesizedVoiceFS3 (opts, state); //Float Stereo Mix 3v2
    if (internalslot == 1 && opts->floating_point == 0 && opts->pulse_digi_rate_out == 8000)
      playSynthesizedVoiceSS3 (opts, state); //Short Stereo Mix 3v2
  }

  dmrBS (opts, state); //bootstrap into full TDMA frame for BS mode
  END:
  //if we have a tact err, then produce sync pattern/err message
  if (tact_okay != 1 || sync_okay != 1)
  {
    if (state->dmr_unconfirmed_sync == 1)
    {
      state->last_cc_sync_time = state->dmr_unconfirmed_cc_sync_time;
      state->last_vc_sync_time = state->dmr_unconfirmed_vc_sync_time;
      state->dmr_unconfirmed_sync = 0;
    }
    state->color_code_ok = 0;
    if (!trunk_is_stale_retune_burst(-1))
    {
      fprintf (stderr,"%s ", timestr);
      fprintf (stderr,"Sync:  DMR                  ");
      fprintf (stderr, "%s", KRED);
      fprintf (stderr, "| VOICE CACH/SYNC ERR");
      fprintf (stderr, "%s", KNRM);
      fprintf (stderr, "\n");
    }

    //failsafe to reset all data header and blocks when bad tact or emb
    dmr_reset_blocks (opts, state);
  }

  if (timestr != NULL)
  {
    free (timestr);
    timestr = NULL;
  }

}
