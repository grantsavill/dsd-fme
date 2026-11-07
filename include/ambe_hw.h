#ifndef AMBE_HW_H
#define AMBE_HW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <mbelib.h>

int ambe_hw_init(const void *opts, const void *state);
void ambe_hw_close(void);
int ambe_hw_toggle(const void *opts, const void *state);
void ambe_hw_release_lock(void);
void ambe_hw_reset_slot(int slot);
int ambe_hw_is_active(void);
int ambe_hw_single_channel_slot(void);
int ambe_hw_is_slot_muted(int slot);
int ambe_hw_was_frame_muted(int slot);
int ambe_hw_is_slot_hw_decoded(int slot);
const char *ambe_hw_status_str(void);
void ambe_hw_reset_call(int slot);
void ambe_hw_record_ber(int slot, int err_bits, int total_bits);
float ambe_hw_get_slot_ber(int slot);
const char *ambe_hw_slot_status_str(int slot);

void ambe_hw_initMbeParms(mbe_parms *cur_mp, mbe_parms *prev_mp, mbe_parms *prev_mp_enhanced);
int ambe_hw_eccImbe7200x4400Data(char imbe_fr[8][23], char *imbe_d);
int ambe_hw_eccAmbe3600x2450C0(char ambe_fr[4][24]);
int ambe_hw_eccAmbe3600x2450Data(char ambe_fr[4][24], char *ambe_d);

void ambe_hw_processAmbe3600x2450Framef(float *aout_buf, int *errs, int *errs2, char *err_str,
                                        char ambe_fr[4][24], char ambe_d[49], mbe_parms *cur_mp,
                                        mbe_parms *prev_mp, mbe_parms *prev_mp_enhanced, int uvquality);
void ambe_hw_processAmbe2450Dataf(float *aout_buf, int *errs, int *errs2, char *err_str,
                                  char ambe_d[49], mbe_parms *cur_mp, mbe_parms *prev_mp,
                                  mbe_parms *prev_mp_enhanced, int uvquality);

void ambe_hw_processAmbe3600x2400Framef(float *aout_buf, int *errs, int *errs2, char *err_str,
                                        char ambe_fr[4][24], char ambe_d[49], mbe_parms *cur_mp,
                                        mbe_parms *prev_mp, mbe_parms *prev_mp_enhanced, int uvquality);

#ifdef __cplusplus
}
#endif

#ifndef AMBE_HW_IMPLEMENTATION
#define mbe_initMbeParms ambe_hw_initMbeParms
#define mbe_eccImbe7200x4400Data ambe_hw_eccImbe7200x4400Data
#define mbe_eccAmbe3600x2450C0 ambe_hw_eccAmbe3600x2450C0
#define mbe_processAmbe3600x2450Framef ambe_hw_processAmbe3600x2450Framef
#define mbe_eccAmbe3600x2450Data ambe_hw_eccAmbe3600x2450Data
#define mbe_processAmbe2450Dataf ambe_hw_processAmbe2450Dataf
#define mbe_processAmbe3600x2400Framef ambe_hw_processAmbe3600x2400Framef
#endif

#endif // AMBE_HW_H
