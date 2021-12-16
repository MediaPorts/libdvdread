/*
 * Copyright (C) 2000, 2001, 2002, 2003
 *               Björn Englund <d4bjorn@dtek.chalmers.se>,
 *               Håkan Hjort <d95hjort@dtek.chalmers.se>
 *
 * This file is part of libdvdread.
 *
 * libdvdread is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * libdvdread is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with libdvdread; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <assert.h>

#include "bswap.h"
#include "dvdread/ifo_types.h"
#include "dvdread/ifo_read.h"
#include "dvdread/dvd_reader.h"
#include "dvdread_internal.h"
#include "dvdread/bitreader.h"

#ifndef DVD_BLOCK_LEN
#define DVD_BLOCK_LEN 2048
#endif

#define PRIV(a) container_of(a, struct ifo_handle_private_s, handle)

#define CHECK_VALUE(arg)\
  if(!(arg)) {\
    Log1(ifop->ctx, "CHECK_VALUE failed in %s:%i for %s",\
                __FILE__, __LINE__, # arg );\
  }

#ifndef NDEBUG
static inline char * makehexdump(const uint8_t *p_CZ, size_t i_CZ)
{
  char *alloc = malloc(i_CZ * 2 + 1);
  if(alloc)
  {
    *alloc = 0;
    for(size_t i = 0; i < i_CZ; i++)
        sprintf(&alloc[i*2], "%02x", *((uint8_t*)&p_CZ[i]));
  }
  return alloc;
}
#define CHECK_ZERO0(arg)                                                \
  if(arg != 0) {                                                        \
    Log1(ifop->ctx, "Zero check failed in %s:%i\n    for %s = 0x%x",    \
            __FILE__, __LINE__, # arg, arg);                            \
  }
#define CHECK_ZERO(arg)                                                 \
  if(memcmp(my_friendly_zeros, &arg, sizeof(arg))) {                    \
    char *dump = makehexdump((const uint8_t *)&arg, sizeof(arg));       \
    Log0(ifop->ctx, "Zero check failed in %s:%i for %s : 0x%s",         \
            __FILE__, __LINE__, # arg, dump );                          \
    free(dump);                                                         \
  }
static const uint8_t my_friendly_zeros[2048];
#else
#define CHECK_ZERO0(arg) (void)(arg)
#define CHECK_ZERO(arg) (void)(arg)
#endif


/* Prototypes for internal functions */
static int ifoRead_VMG(ifo_handle_t *ifofile);
static int ifoRead_VTS(ifo_handle_t *ifofile);
static int ifoRead_PGC(ifo_handle_t *ifofile, pgc_t *pgc, unsigned int offset);
static int ifoRead_PGC_COMMAND_TBL(ifo_handle_t *ifofile,
                                   pgc_command_tbl_t *cmd_tbl,
                                   unsigned int offset);
static int ifoRead_PGC_PROGRAM_MAP(ifo_handle_t *ifofile,
                                   pgc_program_map_t *program_map,
                                   unsigned int nr, unsigned int offset);
static int ifoRead_CELL_PLAYBACK_TBL(ifo_handle_t *ifofile,
                                     cell_playback_t *cell_playback,
                                     unsigned int nr, unsigned int offset);
static int ifoRead_CELL_POSITION_TBL(ifo_handle_t *ifofile,
                                     cell_position_t *cell_position,
                                     unsigned int nr, unsigned int offset);
static int ifoRead_VTS_ATTRIBUTES(ifo_handle_t *ifofile,
                                  vts_attributes_t *vts_attributes,
                                  unsigned int offset);
static int ifoRead_C_ADT_internal(ifo_handle_t *ifofile, c_adt_t *c_adt,
                                  unsigned int sector);
static int ifoRead_VOBU_ADMAP_internal(ifo_handle_t *ifofile,
                                       vobu_admap_t *vobu_admap,
                                       unsigned int sector);
static int ifoRead_PGCIT_internal(ifo_handle_t *ifofile, pgcit_t *pgcit,
                                  unsigned int offset);

static void ifoFree_PGC(pgc_t **pgc);
static void ifoFree_PGC_COMMAND_TBL(pgc_command_tbl_t *cmd_tbl);
static void ifoFree_PGCIT_internal(pgcit_t **pgcit);

static inline int DVDFileSeekForce_( dvd_file_t *dvd_file, uint32_t offset, int force_size ) {
  return (DVDFileSeekForce(dvd_file, (int)offset, force_size) == (int)offset);
}

static inline int DVDFileSeek_( dvd_file_t *dvd_file, uint32_t offset ) {
  return (DVDFileSeek(dvd_file, (int)offset) == (int)offset);
}

typedef struct {
    union {
        const uint8_t *rbuf;
        char *wbuf;
    };
    size_t buflen;
} buf_reader;

static void SkipBuf(buf_reader *b, size_t octets)
{
    assert(b->buflen >= octets);
    b->rbuf += octets;
    b->buflen -= octets;
}

#define SkipZeroBuf(b,octets)                                           \
  assert((b)->buflen >= octets);                                        \
  if(memcmp(my_friendly_zeros, (b)->rbuf, octets)) {                    \
    char *dump = makehexdump((b)->rbuf, octets);                        \
    Log0(ifop->ctx, "Zero check failed in %s:%i : 0x%s",                \
            __FILE__, __LINE__, dump );                                 \
    free(dump);                                                         \
  }                                                                     \
  SkipBuf(b, octets);

static void ReadBuf64(buf_reader *b, uint64_t *buf64)
{
    memcpy(buf64, b->rbuf, 8);
    SkipBuf(b, 8);
    B2N_64(*buf64);
}

static void ReadBuf32(buf_reader *b, uint32_t *buf32)
{
    memcpy(buf32, b->rbuf, 4);
    SkipBuf(b, 4);
    B2N_32(*buf32);
}

static void ReadBuf16(buf_reader *b, uint16_t *buf16)
{
    memcpy(buf16, b->rbuf, 2);
    SkipBuf(b, 2);
    B2N_16(*buf16);
}

static void ReadBuf8(buf_reader *b, uint8_t *buf8)
{
    memcpy(buf8, b->rbuf, 1);
    SkipBuf(b, 1);
}

static void ReadBufData(buf_reader *b, void *str, size_t size)
{
    assert(b->buflen >= size);
    memcpy(str, b->rbuf, size);
    SkipBuf(b, size);
}

static void ReadBufTime(buf_reader *b, dvd_time_t *time)
{
    ReadBuf8(b, &time->hour);
    ReadBuf8(b, &time->minute);
    ReadBuf8(b, &time->second);
    ReadBuf8(b, &time->frame_u);
}

static void read_video_attr_(buf_reader *b, video_attr_t *va) {
  getbits_state_t state;

  dvdread_getbits_init(&state, b->rbuf);
  va->mpeg_version = dvdread_getbits(&state, 2);
  va->video_format = dvdread_getbits(&state, 2);
  va->display_aspect_ratio = dvdread_getbits(&state, 2);
  va->permitted_df = dvdread_getbits(&state, 2);
  va->line21_cc_1 = dvdread_getbits(&state, 1);
  va->line21_cc_2 = dvdread_getbits(&state, 1);
  va->unknown1 = dvdread_getbits(&state, 1);
  va->bit_rate = dvdread_getbits(&state, 1);
  va->picture_size = dvdread_getbits(&state, 2);
  va->letterboxed = dvdread_getbits(&state, 1);
  va->film_mode = dvdread_getbits(&state, 1);
  SkipBuf(b, VIDEO_ATTR_SIZE);
}
static void read_video_attr(video_attr_t *va) {
  char buf[VIDEO_ATTR_SIZE];
  buf_reader b;
  b.wbuf = buf;
  b.buflen = VIDEO_ATTR_SIZE;
  memcpy(b.wbuf, va, VIDEO_ATTR_SIZE);
  read_video_attr_(&b, va);
}

static void read_audio_attr_(buf_reader *b, audio_attr_t *aa) {
  getbits_state_t state;

  dvdread_getbits_init(&state, b->rbuf);
  aa->audio_format = dvdread_getbits(&state, 3);
  aa->multichannel_extension = dvdread_getbits(&state, 1);
  aa->lang_type = dvdread_getbits(&state, 2);
  aa->application_mode = dvdread_getbits(&state, 2);
  aa->quantization = dvdread_getbits(&state, 2);
  aa->sample_frequency = dvdread_getbits(&state, 2);
  aa->unknown1 = dvdread_getbits(&state, 1);
  aa->channels = dvdread_getbits(&state, 3);
  aa->lang_code = dvdread_getbits(&state, 16);
  aa->lang_extension = dvdread_getbits(&state, 8);
  aa->code_extension = dvdread_getbits(&state, 8);
  aa->unknown3 = dvdread_getbits(&state, 8);
  aa->app_info.karaoke.unknown4 = dvdread_getbits(&state, 1);
  aa->app_info.karaoke.channel_assignment = dvdread_getbits(&state, 3);
  aa->app_info.karaoke.version = dvdread_getbits(&state, 2);
  aa->app_info.karaoke.mc_intro = dvdread_getbits(&state, 1);
  aa->app_info.karaoke.mode = dvdread_getbits(&state, 1);
  SkipBuf(b, AUDIO_ATTR_SIZE);
}
static void read_audio_attr(audio_attr_t *aa) {
  char buf[AUDIO_ATTR_SIZE];
  buf_reader b;
  b.wbuf = buf;
  b.buflen = AUDIO_ATTR_SIZE;
  memcpy(b.wbuf, aa, AUDIO_ATTR_SIZE);
  read_audio_attr_(&b, aa);
}

static void read_multichannel_ext_(buf_reader *b, struct ifo_handle_private_s *ifop,
                                  multichannel_ext_t *me) {
  getbits_state_t state;

  dvdread_getbits_init(&state, b->rbuf);
  me->zero1 = dvdread_getbits(&state, 7);
  me->ach0_gme = dvdread_getbits(&state, 1);
  me->zero2 = dvdread_getbits(&state, 7);
  me->ach1_gme = dvdread_getbits(&state, 1);
  me->zero3 = dvdread_getbits(&state, 4);
  me->ach2_gv1e = dvdread_getbits(&state, 1);
  me->ach2_gv2e = dvdread_getbits(&state, 1);
  me->ach2_gm1e = dvdread_getbits(&state, 1);
  me->ach2_gm2e = dvdread_getbits(&state, 1);
  me->zero4 = dvdread_getbits(&state, 4);
  me->ach3_gv1e = dvdread_getbits(&state, 1);
  me->ach3_gv2e = dvdread_getbits(&state, 1);
  me->ach3_gmAe = dvdread_getbits(&state, 1);
  me->ach3_se2e = dvdread_getbits(&state, 1);
  me->zero5 = dvdread_getbits(&state, 4);
  me->ach4_gv1e = dvdread_getbits(&state, 1);
  me->ach4_gv2e = dvdread_getbits(&state, 1);
  me->ach4_gmBe = dvdread_getbits(&state, 1);
  me->ach4_seBe = dvdread_getbits(&state, 1);
  SkipBuf(b, 5);
  SkipZeroBuf(b, MULTICHANNEL_EXT_SIZE - 5);
}
static void read_multichannel_ext(multichannel_ext_t *me, struct ifo_handle_private_s *ifop) {
  char buf[MULTICHANNEL_EXT_SIZE];
  buf_reader b;
  b.wbuf = buf;
  b.buflen = MULTICHANNEL_EXT_SIZE;
  memcpy(b.wbuf, me, MULTICHANNEL_EXT_SIZE);
  read_multichannel_ext_(&b, ifop, me);
}

static void read_subp_attr_(buf_reader *b, subp_attr_t *sa) {
  getbits_state_t state;

  dvdread_getbits_init(&state, b->rbuf);
  sa->code_mode = dvdread_getbits(&state, 3);
  sa->zero1 = dvdread_getbits(&state, 3);
  sa->type = dvdread_getbits(&state, 2);
  sa->zero2 = dvdread_getbits(&state, 8);
  sa->lang_code = dvdread_getbits(&state, 16);
  sa->lang_extension = dvdread_getbits(&state, 8);
  sa->code_extension = dvdread_getbits(&state, 8);
  SkipBuf(b, SUBP_ATTR_SIZE);
}
static void read_subp_attr(subp_attr_t *sa) {
  char buf[SUBP_ATTR_SIZE];
  buf_reader b;
  b.wbuf = buf;
  b.buflen = SUBP_ATTR_SIZE;
  memcpy(b.wbuf, sa, SUBP_ATTR_SIZE);
  read_subp_attr_(&b, sa);
}

static void read_user_ops_(buf_reader *b, user_ops_t *uo) {
  getbits_state_t state;

  dvdread_getbits_init(&state, b->rbuf);
  uo->zero                           = dvdread_getbits(&state, 7);
  uo->video_pres_mode_change         = dvdread_getbits(&state, 1);
  uo->karaoke_audio_pres_mode_change = dvdread_getbits(&state, 1);
  uo->angle_change                   = dvdread_getbits(&state, 1);
  uo->subpic_stream_change           = dvdread_getbits(&state, 1);
  uo->audio_stream_change            = dvdread_getbits(&state, 1);
  uo->pause_on                       = dvdread_getbits(&state, 1);
  uo->still_off                      = dvdread_getbits(&state, 1);
  uo->button_select_or_activate      = dvdread_getbits(&state, 1);
  uo->resume                         = dvdread_getbits(&state, 1);
  uo->chapter_menu_call              = dvdread_getbits(&state, 1);
  uo->angle_menu_call                = dvdread_getbits(&state, 1);
  uo->audio_menu_call                = dvdread_getbits(&state, 1);
  uo->subpic_menu_call               = dvdread_getbits(&state, 1);
  uo->root_menu_call                 = dvdread_getbits(&state, 1);
  uo->title_menu_call                = dvdread_getbits(&state, 1);
  uo->backward_scan                  = dvdread_getbits(&state, 1);
  uo->forward_scan                   = dvdread_getbits(&state, 1);
  uo->next_pg_search                 = dvdread_getbits(&state, 1);
  uo->prev_or_top_pg_search          = dvdread_getbits(&state, 1);
  uo->time_or_chapter_search         = dvdread_getbits(&state, 1);
  uo->go_up                          = dvdread_getbits(&state, 1);
  uo->stop                           = dvdread_getbits(&state, 1);
  uo->title_play                     = dvdread_getbits(&state, 1);
  uo->chapter_search_or_play         = dvdread_getbits(&state, 1);
  uo->title_or_time_play             = dvdread_getbits(&state, 1);
  SkipBuf(b, USER_OPS_SIZE);
}
static void read_user_ops(user_ops_t *uo) {
  char buf[USER_OPS_SIZE];
  buf_reader b;
  b.wbuf = buf;
  b.buflen = USER_OPS_SIZE;
  memcpy(b.wbuf, uo, USER_OPS_SIZE);
  read_user_ops_(&b, uo);
}

static void read_pgci_srp_(buf_reader *b, pgci_srp_t *ps) {
  getbits_state_t state;

  dvdread_getbits_init(&state, b->rbuf);
  ps->entry_id                       = dvdread_getbits(&state, 8);
  ps->block_mode                     = dvdread_getbits(&state, 2);
  ps->block_type                     = dvdread_getbits(&state, 2);
  ps->zero_1                         = dvdread_getbits(&state, 4);
  ps->ptl_id_mask                    = dvdread_getbits(&state, 16);
  ps->pgc_start_byte                 = dvdread_getbits(&state, 32);
  SkipBuf(b, PGCI_SRP_SIZE);
}
static void read_pgci_srp(pgci_srp_t *ps) {
  char buf[PGCI_SRP_SIZE];
  buf_reader b;
  b.wbuf = buf;
  b.buflen = PGCI_SRP_SIZE;
  memcpy(b.wbuf, ps, PGCI_SRP_SIZE);
  read_pgci_srp_(&b, ps);
}

static void read_cell_playback_(buf_reader *b, cell_playback_t *cp) {
  getbits_state_t state;

  dvdread_getbits_init(&state, b->rbuf);
  cp->block_mode                      = dvdread_getbits(&state, 2);
  cp->block_type                      = dvdread_getbits(&state, 2);
  cp->seamless_play                   = dvdread_getbits(&state, 1);
  cp->interleaved                     = dvdread_getbits(&state, 1);
  cp->stc_discontinuity               = dvdread_getbits(&state, 1);
  cp->seamless_angle                  = dvdread_getbits(&state, 1);
  cp->zero_1                          = dvdread_getbits(&state, 1);
  cp->playback_mode                   = dvdread_getbits(&state, 1);
  cp->restricted                      = dvdread_getbits(&state, 1);
  cp->cell_type                       = dvdread_getbits(&state, 5);
  cp->still_time                      = dvdread_getbits(&state, 8);
  cp->cell_cmd_nr                     = dvdread_getbits(&state, 8);

  cp->playback_time.hour              = dvdread_getbits(&state, 8);
  cp->playback_time.minute            = dvdread_getbits(&state, 8);
  cp->playback_time.second            = dvdread_getbits(&state, 8);
  cp->playback_time.frame_u           = dvdread_getbits(&state, 8);

  cp->first_sector                    = dvdread_getbits(&state, 32);
  cp->first_ilvu_end_sector           = dvdread_getbits(&state, 32);
  cp->last_vobu_start_sector          = dvdread_getbits(&state, 32);
  cp->last_sector                     = dvdread_getbits(&state, 32);
  SkipBuf(b, CELL_PLAYBACK_SIZE);
}
static void read_cell_playback(cell_playback_t *cp) {
  char buf[CELL_PLAYBACK_SIZE];
  buf_reader b;
  b.wbuf = buf;
  b.buflen = CELL_PLAYBACK_SIZE;
  memcpy(b.wbuf, cp, CELL_PLAYBACK_SIZE);
  read_cell_playback_(&b, cp);
}

static void read_playback_type_(buf_reader *b, playback_type_t *pt) {
  getbits_state_t state;

  dvdread_getbits_init(&state, b->rbuf);
  pt->zero_1                          = dvdread_getbits(&state, 1);
  pt->multi_or_random_pgc_title       = dvdread_getbits(&state, 1);
  pt->jlc_exists_in_cell_cmd          = dvdread_getbits(&state, 1);
  pt->jlc_exists_in_prepost_cmd       = dvdread_getbits(&state, 1);
  pt->jlc_exists_in_button_cmd        = dvdread_getbits(&state, 1);
  pt->jlc_exists_in_tt_dom            = dvdread_getbits(&state, 1);
  pt->chapter_search_or_play          = dvdread_getbits(&state, 1);
  pt->title_or_time_play              = dvdread_getbits(&state, 1);
  SkipBuf(b, PLAYBACK_TYPE_SIZE);
}
static void read_playback_type(playback_type_t *pt) {
  char buf[PLAYBACK_TYPE_SIZE];
  buf_reader b;
  b.wbuf = buf;
  b.buflen = PLAYBACK_TYPE_SIZE;
  memcpy(b.wbuf, pt, PLAYBACK_TYPE_SIZE);
  read_playback_type_(&b, pt);
}

static void free_ptl_mait(ptl_mait_t* ptl_mait, int num_entries) {
  int i;
  for (i = 0; i < num_entries; i++)
    free(ptl_mait->countries[i].pf_ptl_mai);

  free(ptl_mait->countries);
  free(ptl_mait);
}

static ifo_handle_t *ifoOpenFileOrBackup(dvd_reader_t *ctx, int title,
                                         int backup) {
  struct ifo_handle_private_s *ifop;
  dvd_read_domain_t domain = backup ? DVD_READ_INFO_BACKUP_FILE
                                    : DVD_READ_INFO_FILE;
  char ifo_filename[13];

  ifop = calloc(1, sizeof(*ifop));
  if(!ifop)
    return NULL;

  ifop->ctx = ctx;
  ifop->file = DVDOpenFile(ctx, title, domain);
  if(!ifop->file)
  {
      free(ifop);
      return NULL;
  }

  if (title)
    snprintf(ifo_filename, 13, "VTS_%02d_0.%s", title, backup ? "BUP" : "IFO");
  else
    snprintf(ifo_filename, 13, "VIDEO_TS.%s", backup ? "BUP" : "IFO");

  if(!ifop->file) {
    Log1(ctx, "Can't open file %s.", ifo_filename);
    free(ifop);
    return NULL;
  }

  ifo_handle_t *ifofile = &ifop->handle;
  /* First check if this is a VMGI file. */
  if(ifoRead_VMG(ifofile)) {

    /* These are both mandatory. */
    if(!ifoRead_FP_PGC(ifofile) || !ifoRead_TT_SRPT(ifofile))
      goto ifoOpen_fail;

    ifoRead_PGCI_UT(ifofile);
    ifoRead_PTL_MAIT(ifofile);

    /* This is also mandatory. */
    if(!ifoRead_VTS_ATRT(ifofile))
      goto ifoOpen_fail;

    ifoRead_TXTDT_MGI(ifofile);
    ifoRead_C_ADT(ifofile);
    ifoRead_VOBU_ADMAP(ifofile);

    return ifofile;
  }

  if(ifoRead_VTS(ifofile)) {

    if(!ifoRead_VTS_PTT_SRPT(ifofile) || !ifoRead_PGCIT(ifofile))
      goto ifoOpen_fail;

    ifoRead_PGCI_UT(ifofile);
    ifoRead_VTS_TMAPT(ifofile);
    ifoRead_C_ADT(ifofile);
    ifoRead_VOBU_ADMAP(ifofile);

    if(!ifoRead_TITLE_C_ADT(ifofile) || !ifoRead_TITLE_VOBU_ADMAP(ifofile))
      goto ifoOpen_fail;

    return ifofile;
  }

ifoOpen_fail:
  Log1(ctx, "Invalid IFO for title %d (%s).", title, ifo_filename);
  ifoClose(ifofile);
  return NULL;
}

static void ifoSetBupFlag(dvd_reader_t *ctx, int title)
{
    if(title > 63)
        ctx->ifoBUPflags[0] |= 1 << (title - 64);
    else
        ctx->ifoBUPflags[1] |= 1 << title;
}

static int ifoGetBupFlag(const dvd_reader_t *ctx, int title)
{
    int bupflag;
    if(title > 63)
        bupflag = !! (ctx->ifoBUPflags[0] & (1 << (title - 64)));
    else
        bupflag = !! (ctx->ifoBUPflags[1] & (1 << title));
    return bupflag;
}

ifo_handle_t *ifoOpen(dvd_reader_t *ctx, int title) {
  ifo_handle_t *ifofile;
  int bupflag = ifoGetBupFlag(ctx, title);

  ifofile = ifoOpenFileOrBackup(ctx, title, bupflag);
  if(!ifofile) /* Try backup */
  {
      ifofile = ifoOpenFileOrBackup(ctx, title, 1);
      if(ifofile && !bupflag)
          ifoSetBupFlag(ctx, title);
  }
  return ifofile;
}

ifo_handle_t *ifoOpenVMGI(dvd_reader_t *ctx) {
  struct ifo_handle_private_s *ifop;

  for(int backup = ifoGetBupFlag(ctx, 0); backup <= 1; backup++)
  {
    ifop = calloc(1, sizeof(*ifop));
    if(!ifop)
      return NULL;

    const dvd_read_domain_t domain = backup ? DVD_READ_INFO_BACKUP_FILE
                                            : DVD_READ_INFO_FILE;
    const char *ext = backup ? "BUP" : "IFO";

    ifop->ctx = ctx;
    ifop->file = DVDOpenFile(ctx, 0, domain);
    if(!ifop->file) { /* Should really catch any error */
      Log1(ctx, "Can't open file VIDEO_TS.%s.", ext);
      free(ifop);
      return NULL;
    }

    if(ifoRead_VMG(&ifop->handle))
      return &ifop->handle;

    Log1(ctx, "ifoOpenVMGI(): Invalid main menu IFO (VIDEO_TS.%s).", ext);
    ifoClose(&ifop->handle);
  }
  return NULL;
}


ifo_handle_t *ifoOpenVTSI(dvd_reader_t *ctx, int title) {
  struct ifo_handle_private_s *ifop;

  if(title <= 0 || title > 99) {
    Log1(ctx, "ifoOpenVTSI invalid title (%d).", title);
    return NULL;
  }

  for(int backup = ifoGetBupFlag(ctx, title); backup <= 1; backup++)
  {
    ifop = calloc(1, sizeof(*ifop));
    if(!ifop)
      return NULL;

    const dvd_read_domain_t domain = backup ? DVD_READ_INFO_BACKUP_FILE
                                            : DVD_READ_INFO_FILE;
    const char *ext = backup ? "BUP" : "IFO";
    ifop->ctx = ctx;
    ifop->file = DVDOpenFile(ctx, title, domain);
    /* Should really catch any error */
    if(!ifop->file) {
      Log1(ctx, "Can't open file VTS_%02d_0.%s.", title, ext);
      free(ifop);
      continue;
    }

    if(ifoRead_VTS(&ifop->handle) && ifop->handle.vtsi_mat)
      return &ifop->handle;

    Log1(ctx, "Invalid IFO for title %d (VTS_%02d_0.%s).",
            title, title, ext);
    ifoClose(&ifop->handle);
  }

  return NULL;
}


void ifoClose(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  ifoFree_VOBU_ADMAP(ifofile);
  ifoFree_TITLE_VOBU_ADMAP(ifofile);
  ifoFree_C_ADT(ifofile);
  ifoFree_TITLE_C_ADT(ifofile);
  ifoFree_TXTDT_MGI(ifofile);
  ifoFree_VTS_ATRT(ifofile);
  ifoFree_PTL_MAIT(ifofile);
  ifoFree_PGCI_UT(ifofile);
  ifoFree_TT_SRPT(ifofile);
  ifoFree_FP_PGC(ifofile);
  ifoFree_PGCIT(ifofile);
  ifoFree_VTS_PTT_SRPT(ifofile);
  ifoFree_VTS_TMAPT(ifofile);

  if(ifofile->vmgi_mat)
    free(ifofile->vmgi_mat);

  if(ifofile->vtsi_mat)
    free(ifofile->vtsi_mat);

  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  DVDCloseFile(ifop->file);
  free(ifop);
}


static int ifoRead_VMG(ifo_handle_t *ifofile) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  vmgi_mat_t *vmgi_mat;
  char buf[VMGI_MAT_SIZE];
  buf_reader b;

  vmgi_mat = calloc(1, sizeof(vmgi_mat_t));
  if(!vmgi_mat)
    return 0;

  ifofile->vmgi_mat = vmgi_mat;

  if(!DVDFileSeek_(ifop->file, 0)) {
    free(ifofile->vmgi_mat);
    ifofile->vmgi_mat = NULL;
    return 0;
  }

  b.wbuf = buf;
  b.buflen = VMGI_MAT_SIZE;

  if(!DVDReadBytes(ifop->file, b.wbuf, b.buflen)) {
    free(ifofile->vmgi_mat);
    ifofile->vmgi_mat = NULL;
    return 0;
  }

  ReadBufData(&b, vmgi_mat->vmg_identifier, 12);
  if(strncmp("DVDVIDEO-VMG", vmgi_mat->vmg_identifier, 12) != 0) {
    free(ifofile->vmgi_mat);
    ifofile->vmgi_mat = NULL;
    return 0;
  }

  ReadBuf32(&b, &vmgi_mat->vmg_last_sector);
  SkipZeroBuf(&b, 12);
  ReadBuf32(&b, &vmgi_mat->vmgi_last_sector);
  SkipZeroBuf(&b, 1);
  ReadBuf8(&b, &vmgi_mat->specification_version);
  ReadBuf32(&b, &vmgi_mat->vmg_category);
  ReadBuf16(&b, &vmgi_mat->vmg_nr_of_volumes);
  ReadBuf16(&b, &vmgi_mat->vmg_this_volume_nr);
  ReadBuf8(&b, &vmgi_mat->disc_side);
  /* DVDs created by VDR-to-DVD device LG RC590M violate the following check with
   * vmgi_mat->zero_3 = 0x00000000010000000000000000000000000000. */
  SkipZeroBuf(&b, 19);
  ReadBuf16(&b, &vmgi_mat->vmg_nr_of_title_sets);
  ReadBufData(&b, vmgi_mat->provider_identifier, 32);
  ReadBuf64(&b, &vmgi_mat->vmg_pos_code);
  SkipZeroBuf(&b, 24);
  ReadBuf32(&b, &vmgi_mat->vmgi_last_byte);
  ReadBuf32(&b, &vmgi_mat->first_play_pgc);
  SkipZeroBuf(&b, 56);
  ReadBuf32(&b, &vmgi_mat->vmgm_vobs);
  ReadBuf32(&b, &vmgi_mat->tt_srpt);
  ReadBuf32(&b, &vmgi_mat->vmgm_pgci_ut);
  ReadBuf32(&b, &vmgi_mat->ptl_mait);
  ReadBuf32(&b, &vmgi_mat->vts_atrt);
  ReadBuf32(&b, &vmgi_mat->txtdt_mgi);
  ReadBuf32(&b, &vmgi_mat->vmgm_c_adt);
  ReadBuf32(&b, &vmgi_mat->vmgm_vobu_admap);
  SkipZeroBuf(&b, 32);
  read_video_attr_(&b, &vmgi_mat->vmgm_video_attr);
  SkipZeroBuf(&b, 1);
  ReadBuf8(&b, &vmgi_mat->nr_of_vmgm_audio_streams);
  read_audio_attr_(&b, &vmgi_mat->vmgm_audio_attr);
  SkipZeroBuf(&b, AUDIO_ATTR_SIZE * 7);
  SkipZeroBuf(&b, 17);
  ReadBuf8(&b, &vmgi_mat->nr_of_vmgm_subp_streams);
  read_subp_attr_(&b, &vmgi_mat->vmgm_subp_attr);
  SkipZeroBuf(&b, SUBP_ATTR_SIZE * 27);


  CHECK_VALUE(vmgi_mat->vmg_last_sector != 0);
  CHECK_VALUE(vmgi_mat->vmgi_last_sector != 0);
  CHECK_VALUE(vmgi_mat->vmgi_last_sector * 2 <= vmgi_mat->vmg_last_sector);
  CHECK_VALUE(vmgi_mat->vmgi_last_sector * 2 <= vmgi_mat->vmg_last_sector);
  CHECK_VALUE(vmgi_mat->vmg_nr_of_volumes != 0);
  CHECK_VALUE(vmgi_mat->vmg_this_volume_nr != 0);
  CHECK_VALUE(vmgi_mat->vmg_this_volume_nr <= vmgi_mat->vmg_nr_of_volumes);
  CHECK_VALUE(vmgi_mat->disc_side == 1 || vmgi_mat->disc_side == 2);
  CHECK_VALUE(vmgi_mat->vmg_nr_of_title_sets != 0);
  CHECK_VALUE(vmgi_mat->vmgi_last_byte >= 341);
  CHECK_VALUE(vmgi_mat->vmgi_last_byte / DVD_BLOCK_LEN <=
              vmgi_mat->vmgi_last_sector);
  /* It seems that first_play_pgc is optional. */
  CHECK_VALUE(vmgi_mat->first_play_pgc < vmgi_mat->vmgi_last_byte);
  CHECK_VALUE(vmgi_mat->vmgm_vobs == 0 ||
              (vmgi_mat->vmgm_vobs > vmgi_mat->vmgi_last_sector &&
               vmgi_mat->vmgm_vobs < vmgi_mat->vmg_last_sector));
  CHECK_VALUE(vmgi_mat->tt_srpt <= vmgi_mat->vmgi_last_sector);
  CHECK_VALUE(vmgi_mat->vmgm_pgci_ut <= vmgi_mat->vmgi_last_sector);
  CHECK_VALUE(vmgi_mat->ptl_mait <= vmgi_mat->vmgi_last_sector);
  CHECK_VALUE(vmgi_mat->vts_atrt <= vmgi_mat->vmgi_last_sector);
  CHECK_VALUE(vmgi_mat->txtdt_mgi <= vmgi_mat->vmgi_last_sector);
  CHECK_VALUE(vmgi_mat->vmgm_c_adt <= vmgi_mat->vmgi_last_sector);
  CHECK_VALUE(vmgi_mat->vmgm_vobu_admap <= vmgi_mat->vmgi_last_sector);

  CHECK_VALUE(vmgi_mat->nr_of_vmgm_audio_streams <= 1);
  CHECK_VALUE(vmgi_mat->nr_of_vmgm_subp_streams <= 1);

  return 1;
}


static int ifoRead_VTS(ifo_handle_t *ifofile) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  vtsi_mat_t *vtsi_mat;
  int i;
  char buf[VTSI_MAT_SIZE];
  buf_reader b;

  vtsi_mat = calloc(1, sizeof(*vtsi_mat));
  if(!vtsi_mat)
    return 0;

  ifofile->vtsi_mat = vtsi_mat;

  if(!DVDFileSeek_(ifop->file, 0)) {
    free(ifofile->vtsi_mat);
    ifofile->vtsi_mat = NULL;
    return 0;
  }

  b.wbuf = buf;
  b.buflen = VTSI_MAT_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
    free(ifofile->vtsi_mat);
    ifofile->vtsi_mat = NULL;
    return 0;
  }

  ReadBufData(&b, vtsi_mat->vts_identifier, 12);

  if(strncmp("DVDVIDEO-VTS", vtsi_mat->vts_identifier, 12) != 0) {
    free(ifofile->vtsi_mat);
    ifofile->vtsi_mat = NULL;
    return 0;
  }

  ReadBuf32(&b, &vtsi_mat->vts_last_sector);
  SkipZeroBuf(&b, 12);
  ReadBuf32(&b, &vtsi_mat->vtsi_last_sector);
  SkipZeroBuf(&b, 1);
  ReadBuf8(&b, &vtsi_mat->specification_version);
  ReadBuf32(&b, &vtsi_mat->vts_category);
  SkipZeroBuf(&b, 90);
  ReadBuf32(&b, &vtsi_mat->vtsi_last_byte);
  SkipZeroBuf(&b, 60);
  ReadBuf32(&b, &vtsi_mat->vtsm_vobs);
  ReadBuf32(&b, &vtsi_mat->vtstt_vobs);
  ReadBuf32(&b, &vtsi_mat->vts_ptt_srpt);
  ReadBuf32(&b, &vtsi_mat->vts_pgcit);
  ReadBuf32(&b, &vtsi_mat->vtsm_pgci_ut);
  ReadBuf32(&b, &vtsi_mat->vts_tmapt);
  ReadBuf32(&b, &vtsi_mat->vtsm_c_adt);
  ReadBuf32(&b, &vtsi_mat->vtsm_vobu_admap);
  ReadBuf32(&b, &vtsi_mat->vts_c_adt);
  ReadBuf32(&b, &vtsi_mat->vts_vobu_admap);
  SkipZeroBuf(&b, 24);
  read_video_attr_(&b, &vtsi_mat->vtsm_video_attr);
  SkipZeroBuf(&b, 1);
  ReadBuf8(&b, &vtsi_mat->nr_of_vtsm_audio_streams);
  read_audio_attr_(&b, &vtsi_mat->vtsm_audio_attr);
  SkipZeroBuf(&b, AUDIO_ATTR_SIZE * 7);
  SkipZeroBuf(&b, 17);
  ReadBuf8(&b, &vtsi_mat->nr_of_vtsm_subp_streams);
  read_subp_attr_(&b, &vtsi_mat->vtsm_subp_attr);
  SkipZeroBuf(&b, SUBP_ATTR_SIZE * 27);
  SkipZeroBuf(&b, 2);
  read_video_attr_(&b, &vtsi_mat->vts_video_attr);
  SkipZeroBuf(&b, 1);
  ReadBuf8(&b, &vtsi_mat->nr_of_vts_audio_streams);
  for(i=0; i<8; i++)
    read_audio_attr_(&b, &vtsi_mat->vts_audio_attr[i]);
  SkipZeroBuf(&b, 17);
  ReadBuf8(&b, &vtsi_mat->nr_of_vts_subp_streams);
  for(i=0; i<32; i++)
    read_subp_attr_(&b, &vtsi_mat->vts_subp_attr[i]);
  SkipZeroBuf(&b, 2);

  CHECK_VALUE(vtsi_mat->vtsi_last_sector*2 <= vtsi_mat->vts_last_sector);
  CHECK_VALUE(vtsi_mat->vtsi_last_byte/DVD_BLOCK_LEN <= vtsi_mat->vtsi_last_sector);
  CHECK_VALUE(vtsi_mat->vtsm_vobs == 0 ||
              (vtsi_mat->vtsm_vobs > vtsi_mat->vtsi_last_sector &&
               vtsi_mat->vtsm_vobs < vtsi_mat->vts_last_sector));
  CHECK_VALUE(vtsi_mat->vtstt_vobs == 0 ||
              (vtsi_mat->vtstt_vobs > vtsi_mat->vtsi_last_sector &&
               vtsi_mat->vtstt_vobs < vtsi_mat->vts_last_sector));
  CHECK_VALUE(vtsi_mat->vts_ptt_srpt <= vtsi_mat->vtsi_last_sector);
  CHECK_VALUE(vtsi_mat->vts_pgcit <= vtsi_mat->vtsi_last_sector);
  CHECK_VALUE(vtsi_mat->vtsm_pgci_ut <= vtsi_mat->vtsi_last_sector);
  CHECK_VALUE(vtsi_mat->vts_tmapt <= vtsi_mat->vtsi_last_sector);
  CHECK_VALUE(vtsi_mat->vtsm_c_adt <= vtsi_mat->vtsi_last_sector);
  CHECK_VALUE(vtsi_mat->vtsm_vobu_admap <= vtsi_mat->vtsi_last_sector);
  CHECK_VALUE(vtsi_mat->vts_c_adt <= vtsi_mat->vtsi_last_sector);
  CHECK_VALUE(vtsi_mat->vts_vobu_admap <= vtsi_mat->vtsi_last_sector);

  CHECK_VALUE(vtsi_mat->nr_of_vtsm_audio_streams <= 1);
  CHECK_VALUE(vtsi_mat->nr_of_vtsm_subp_streams <= 1);

  CHECK_VALUE(vtsi_mat->nr_of_vts_audio_streams <= 8);
  for(i = vtsi_mat->nr_of_vts_audio_streams; i < 8; i++)
    CHECK_ZERO(vtsi_mat->vts_audio_attr[i]);

  CHECK_VALUE(vtsi_mat->nr_of_vts_subp_streams <= 32);
  for(i = vtsi_mat->nr_of_vts_subp_streams; i < 32; i++)
    CHECK_ZERO(vtsi_mat->vts_subp_attr[i]);

  for(i = 0; i < 8; i++) {
    read_multichannel_ext_(&b, ifop, &vtsi_mat->vts_mu_audio_attr[i]);
    CHECK_ZERO0(vtsi_mat->vts_mu_audio_attr[i].zero1);
    CHECK_ZERO0(vtsi_mat->vts_mu_audio_attr[i].zero2);
    CHECK_ZERO0(vtsi_mat->vts_mu_audio_attr[i].zero3);
    CHECK_ZERO0(vtsi_mat->vts_mu_audio_attr[i].zero4);
    CHECK_ZERO0(vtsi_mat->vts_mu_audio_attr[i].zero5);
  }

  return 1;
}


static int ifoRead_PGC_COMMAND_TBL(ifo_handle_t *ifofile,
                                   pgc_command_tbl_t *cmd_tbl,
                                   unsigned int offset) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  char buf[PGC_COMMAND_TBL_SIZE];
  unsigned i;
  buf_reader b;

  if(!DVDFileSeek_(ifop->file, offset))
    return 0;

  b.wbuf = buf;
  b.buflen = PGC_COMMAND_TBL_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen)))
    return 0;

  ReadBuf16(&b, &cmd_tbl->nr_of_pre);
  ReadBuf16(&b, &cmd_tbl->nr_of_post);
  ReadBuf16(&b, &cmd_tbl->nr_of_cell);
  SkipBuf(&b, 2);
  B2N_16(cmd_tbl->last_byte);

  CHECK_VALUE(cmd_tbl->nr_of_pre + cmd_tbl->nr_of_post + cmd_tbl->nr_of_cell<= 255);
  CHECK_VALUE((cmd_tbl->nr_of_pre + cmd_tbl->nr_of_post + cmd_tbl->nr_of_cell) * COMMAND_DATA_SIZE
              + PGC_COMMAND_TBL_SIZE <= cmd_tbl->last_byte + 1);

  if(cmd_tbl->nr_of_pre != 0) {
    unsigned int pre_cmds_size  = cmd_tbl->nr_of_pre * COMMAND_DATA_SIZE;
    char bufpre[pre_cmds_size];
    cmd_tbl->pre_cmds = malloc(pre_cmds_size);
    if(!cmd_tbl->pre_cmds)
      return 0;

    b.wbuf = bufpre;
    b.buflen = pre_cmds_size;

    if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
      free(cmd_tbl->pre_cmds);
      return 0;
    }

    for (i=0; i<cmd_tbl->nr_of_pre; i++)
        ReadBufData(&b, cmd_tbl->pre_cmds[i].bytes, 8);
  }

  if(cmd_tbl->nr_of_post != 0) {
    unsigned int post_cmds_size = cmd_tbl->nr_of_post * COMMAND_DATA_SIZE;
    char bufpost[post_cmds_size];
    cmd_tbl->post_cmds = malloc(post_cmds_size);
    if(!cmd_tbl->post_cmds) {
      if(cmd_tbl->pre_cmds)
        free(cmd_tbl->pre_cmds);
      return 0;
    }

    b.wbuf = bufpost;
    b.buflen = post_cmds_size;

    if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
      if(cmd_tbl->pre_cmds)
        free(cmd_tbl->pre_cmds);
      free(cmd_tbl->post_cmds);
      return 0;
    }

    for (i=0; i<cmd_tbl->nr_of_post; i++)
        ReadBufData(&b, cmd_tbl->post_cmds[i].bytes, 8);
  }

  if(cmd_tbl->nr_of_cell != 0) {
    unsigned int cell_cmds_size = cmd_tbl->nr_of_cell * COMMAND_DATA_SIZE;
    char bufcell[cell_cmds_size];
    cmd_tbl->cell_cmds = malloc(cell_cmds_size);
    if(!cmd_tbl->cell_cmds) {
      if(cmd_tbl->pre_cmds)
        free(cmd_tbl->pre_cmds);
      if(cmd_tbl->post_cmds)
        free(cmd_tbl->post_cmds);
      return 0;
    }

    b.wbuf = bufcell;
    b.buflen = cell_cmds_size;

    if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
      if(cmd_tbl->pre_cmds)
        free(cmd_tbl->pre_cmds);
      if(cmd_tbl->post_cmds)
        free(cmd_tbl->post_cmds);
      free(cmd_tbl->cell_cmds);
      return 0;
    }

    for (i=0; i<cmd_tbl->nr_of_cell; i++)
        ReadBufData(&b, cmd_tbl->cell_cmds[i].bytes, 8);
  }

  /*
   * Make a run over all the commands and see that we can interpret them all?
   */
  return 1;
}


static void ifoFree_PGC_COMMAND_TBL(pgc_command_tbl_t *cmd_tbl) {
  if(cmd_tbl) {
    if(cmd_tbl->nr_of_pre && cmd_tbl->pre_cmds)
      free(cmd_tbl->pre_cmds);
    if(cmd_tbl->nr_of_post && cmd_tbl->post_cmds)
      free(cmd_tbl->post_cmds);
    if(cmd_tbl->nr_of_cell && cmd_tbl->cell_cmds)
      free(cmd_tbl->cell_cmds);
    free(cmd_tbl);
  }
}

static int ifoRead_PGC_PROGRAM_MAP(ifo_handle_t *ifofile,
                                   pgc_program_map_t *program_map,
                                   unsigned int nr, unsigned int offset) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  unsigned int size = nr * sizeof(pgc_program_map_t);
  buf_reader b;

  if(!DVDFileSeek_(ifop->file, offset))
    return 0;

//   static_assert(sizeof(pgc_program_map_t)==1,"odd uint8_t size");
  b.wbuf = (char*)program_map;
  b.buflen = size;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen)))
    return 0;

  return 1;
}

static int ifoRead_CELL_PLAYBACK_TBL(ifo_handle_t *ifofile,
                                     cell_playback_t *cell_playback,
                                     unsigned int nr, unsigned int offset) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  unsigned int i;
  char buf[nr * CELL_PLAYBACK_SIZE];
  buf_reader b;

  if(!DVDFileSeek_(ifop->file, offset))
    return 0;

  b.wbuf = buf;
  b.buflen = nr * CELL_PLAYBACK_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen)))
    return 0;

  for(i = 0; i < nr; i++) {
    read_cell_playback_(&b, &cell_playback[i]);
    /* Changed < to <= because this was false in the movie 'Pi'. */
    CHECK_VALUE(cell_playback[i].last_vobu_start_sector <=
                cell_playback[i].last_sector);
    CHECK_VALUE(cell_playback[i].first_sector <=
                cell_playback[i].last_vobu_start_sector);
  }

  return 1;
}


static int ifoRead_CELL_POSITION_TBL(ifo_handle_t *ifofile,
                                     cell_position_t *cell_position,
                                     unsigned int nr, unsigned int offset) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  unsigned int i;
  unsigned int size = nr * CELL_POSITION_SIZE;
  char buf[size];
  buf_reader b;

  if(!DVDFileSeek_(ifop->file, offset))
    return 0;

  b.wbuf = buf;
  b.buflen = size;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen)))
    return 0;

  for(i = 0; i < nr; i++) {
    ReadBuf16(&b, &cell_position[i].vob_id_nr);
    SkipZeroBuf(&b, 1);
    ReadBuf8(&b, &cell_position[i].cell_nr);
  }

  return 1;
}

static int ifoRead_PGC(ifo_handle_t *ifofile, pgc_t *pgc, unsigned int offset) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  unsigned int i;
  char buf[PGC_SIZE];
  buf_reader b;

  if(!DVDFileSeek_(ifop->file, offset))
    return 0;

  b.wbuf = buf;
  b.buflen = PGC_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen)))
    return 0;

  SkipZeroBuf(&b, 2);
  ReadBuf8(&b, &pgc->nr_of_programs);
  ReadBuf8(&b, &pgc->nr_of_cells);
  ReadBufTime(&b, &pgc->playback_time);
  read_user_ops_(&b, &pgc->prohibited_ops);
  for(i = 0; i < 8; i++)
    ReadBuf16(&b, &pgc->audio_control[i]);
  for(i = 0; i < 32; i++)
    ReadBuf32(&b, &pgc->subp_control[i]);
  ReadBuf16(&b, &pgc->next_pgc_nr);
  ReadBuf16(&b, &pgc->prev_pgc_nr);
  ReadBuf16(&b, &pgc->goup_pgc_nr);
  ReadBuf8(&b, &pgc->pg_playback_mode);
  ReadBuf8(&b, &pgc->still_time);
  for(i = 0; i < 16; i++)
    ReadBuf32(&b, &pgc->palette[i]);
  ReadBuf16(&b, &pgc->command_tbl_offset);
  ReadBuf16(&b, &pgc->program_map_offset);
  ReadBuf16(&b, &pgc->cell_playback_offset);
  ReadBuf16(&b, &pgc->cell_position_offset);

  CHECK_VALUE(pgc->nr_of_programs <= pgc->nr_of_cells);

  /* verify time (look at print_time) */
  for(i = 0; i < 8; i++)
    if(!(pgc->audio_control[i] & 0x8000)) /* The 'is present' bit */
      CHECK_ZERO(pgc->audio_control[i]);
  for(i = 0; i < 32; i++)
    if(!(pgc->subp_control[i] & 0x80000000)) /* The 'is present' bit */
      CHECK_ZERO(pgc->subp_control[i]);

  /* Check that time is 0:0:0:0 also if nr_of_programs == 0 */
  if(pgc->nr_of_programs == 0) {
    CHECK_ZERO(pgc->still_time);
    CHECK_ZERO(pgc->pg_playback_mode); /* ?? */
    CHECK_VALUE(pgc->program_map_offset == 0);
    CHECK_VALUE(pgc->cell_playback_offset == 0);
    CHECK_VALUE(pgc->cell_position_offset == 0);
  } else {
    CHECK_VALUE(pgc->program_map_offset != 0);
    CHECK_VALUE(pgc->cell_playback_offset != 0);
    CHECK_VALUE(pgc->cell_position_offset != 0);
  }

  if(pgc->command_tbl_offset != 0) {
    pgc->command_tbl = calloc(1, sizeof(pgc_command_tbl_t));
    if(!pgc->command_tbl)
      return 0;

    if(!ifoRead_PGC_COMMAND_TBL(ifofile, pgc->command_tbl,
                                offset + pgc->command_tbl_offset)) {
      return 0;
    }
  } else {
    pgc->command_tbl = NULL;
  }

  if(pgc->program_map_offset != 0 && pgc->nr_of_programs>0) {
    pgc->program_map = calloc(pgc->nr_of_programs, sizeof(pgc_program_map_t));
    if(!pgc->program_map) {
      return 0;
    }
    if(!ifoRead_PGC_PROGRAM_MAP(ifofile, pgc->program_map,pgc->nr_of_programs,
                                offset + pgc->program_map_offset)) {
      return 0;
    }
  } else {
    pgc->program_map = NULL;
  }

  if(pgc->cell_playback_offset != 0 && pgc->nr_of_cells>0) {
    pgc->cell_playback = calloc(pgc->nr_of_cells, sizeof(cell_playback_t));
    if(!pgc->cell_playback) {
      return 0;
    }
    if(!ifoRead_CELL_PLAYBACK_TBL(ifofile, pgc->cell_playback,
                                  pgc->nr_of_cells,
                                  offset + pgc->cell_playback_offset)) {
      return 0;
    }
  } else {
    pgc->cell_playback = NULL;
  }

  if(pgc->cell_position_offset != 0 && pgc->nr_of_cells>0) {
    pgc->cell_position = calloc(pgc->nr_of_cells, sizeof(cell_position_t));
    if(!pgc->cell_position) {
      return 0;
    }
    if(!ifoRead_CELL_POSITION_TBL(ifofile, pgc->cell_position,
                                  pgc->nr_of_cells,
                                  offset + pgc->cell_position_offset)) {
      return 0;
    }
  } else {
    pgc->cell_position = NULL;
  }

  return 1;
}

int ifoRead_FP_PGC(ifo_handle_t *ifofile) {

  if(!ifofile)
    return 0;

  if(!ifofile->vmgi_mat)
    return 0;

  /* It seems that first_play_pgc is optional after all. */
  ifofile->first_play_pgc = NULL;
  if(!ifofile->vmgi_mat->first_play_pgc)
    return 1;

  ifofile->first_play_pgc = calloc(1, sizeof(pgc_t));
  if(!ifofile->first_play_pgc)
    return 0;

  ifofile->first_play_pgc->ref_count = 1;
  if(!ifoRead_PGC(ifofile, ifofile->first_play_pgc,
                  ifofile->vmgi_mat->first_play_pgc)) {
    ifoFree_PGC(&ifofile->first_play_pgc);
    return 0;
  }

  return 1;
}

static void ifoFree_PGC(pgc_t **pgc) {
  if(pgc && *pgc && (--(*pgc)->ref_count) <= 0) {
    ifoFree_PGC_COMMAND_TBL((*pgc)->command_tbl);
    if((*pgc)->program_map)
      free((*pgc)->program_map);
    if((*pgc)->cell_playback)
      free((*pgc)->cell_playback);
    if((*pgc)->cell_position)
      free((*pgc)->cell_position);
    free(*pgc);
  }
  if (pgc) {
    *pgc = NULL;
  }
}

void ifoFree_FP_PGC(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  if(ifofile->first_play_pgc) {
    ifoFree_PGC(&ifofile->first_play_pgc);
  }
}


int ifoRead_TT_SRPT(ifo_handle_t *ifofile) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  tt_srpt_t *tt_srpt;
  unsigned int i;
  size_t info_length;
  char buf[TT_SRPT_SIZE];
  buf_reader b;

  if(!ifofile)
    return 0;

  if(!ifofile->vmgi_mat)
    return 0;

  if(ifofile->vmgi_mat->tt_srpt == 0) /* mandatory */
    return 0;

  if(!DVDFileSeek_(ifop->file, ifofile->vmgi_mat->tt_srpt * DVD_BLOCK_LEN))
    return 0;

  tt_srpt = calloc(1, sizeof(tt_srpt_t));
  if(!tt_srpt)
    return 0;

  ifofile->tt_srpt = tt_srpt;

  b.wbuf = buf;
  b.buflen = TT_SRPT_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
    Log0(ifop->ctx, "Unable to read read TT_SRPT.");
    free(tt_srpt);
    return 0;
  }

  ReadBuf16(&b, &tt_srpt->nr_of_srpts);
  SkipZeroBuf(&b, 2);
  ReadBuf32(&b, &tt_srpt->last_byte);

  /* E-One releases don't fill this field */
  if(tt_srpt->last_byte == 0) {
    tt_srpt->last_byte = tt_srpt->nr_of_srpts * TITLE_INFO_SIZE - 1 + TT_SRPT_SIZE;
  }
  info_length = tt_srpt->last_byte + 1 - TT_SRPT_SIZE;

  tt_srpt->title = calloc(tt_srpt->nr_of_srpts, sizeof(*tt_srpt->title));
  if(!tt_srpt->title) {
    free(tt_srpt);
    ifofile->tt_srpt = NULL;
    return 0;
  }

  char titleBuf[info_length];
  b.wbuf = titleBuf;
  b.buflen = info_length;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
    Log0(ifop->ctx, "libdvdread: Unable to read read TT_SRPT.");
    ifoFree_TT_SRPT(ifofile);
    return 0;
  }

  if(tt_srpt->nr_of_srpts>info_length/TITLE_INFO_SIZE){
    Log1(ifop->ctx, "data mismatch: info_length (%zd)!= nr_of_srpts (%d). Truncating.",
            info_length/TITLE_INFO_SIZE,tt_srpt->nr_of_srpts);
    tt_srpt->nr_of_srpts=info_length/TITLE_INFO_SIZE;
  }

  CHECK_VALUE(tt_srpt->nr_of_srpts != 0);
  CHECK_VALUE(tt_srpt->nr_of_srpts < 100); /* ?? */
  CHECK_VALUE(tt_srpt->nr_of_srpts * TITLE_INFO_SIZE <= info_length);

  for(i = 0; i < tt_srpt->nr_of_srpts; i++) {
    read_playback_type_(&b, &tt_srpt->title[i].pb_ty);
    ReadBuf8(&b, &tt_srpt->title[i].nr_of_angles);
    ReadBuf16(&b, &tt_srpt->title[i].nr_of_ptts);
    ReadBuf16(&b, &tt_srpt->title[i].parental_id);
    ReadBuf8(&b, &tt_srpt->title[i].title_set_nr);
    ReadBuf8(&b, &tt_srpt->title[i].vts_ttn);
    ReadBuf32(&b, &tt_srpt->title[i].title_set_sector);
    CHECK_VALUE(tt_srpt->title[i].pb_ty.zero_1 == 0);
    CHECK_VALUE(tt_srpt->title[i].nr_of_angles != 0);
    CHECK_VALUE(tt_srpt->title[i].nr_of_angles < 10);
    /* CHECK_VALUE(tt_srpt->title[i].nr_of_ptts != 0); */
    /* XXX: this assertion breaks Ghostbusters: */
    CHECK_VALUE(tt_srpt->title[i].nr_of_ptts < 1000); /* ?? */
    CHECK_VALUE(tt_srpt->title[i].title_set_nr != 0);
    CHECK_VALUE(tt_srpt->title[i].title_set_nr < 100); /* ?? */
    CHECK_VALUE(tt_srpt->title[i].vts_ttn != 0);
    CHECK_VALUE(tt_srpt->title[i].vts_ttn < 100); /* ?? */
    /* CHECK_VALUE(tt_srpt->title[i].title_set_sector != 0); */
  }

  /* Make this a function */
#if 0
  if(memcmp((uint8_t *)tt_srpt->title +
            tt_srpt->nr_of_srpts * TITLE_INFO_SIZE,
            my_friendly_zeros,
            info_length - tt_srpt->nr_of_srpts * TITLE_INFO_SIZE)) {
    Log1(ifop->ctx, "VMG_PTT_SRPT slack is != 0, ");
    hexdump((uint8_t *)tt_srpt->title +
            tt_srpt->nr_of_srpts * TITLE_INFO_SIZE,
            info_length - tt_srpt->nr_of_srpts * TITLE_INFO_SIZE);
  }
#endif

  return 1;
}


void ifoFree_TT_SRPT(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  if(ifofile->tt_srpt) {
    free(ifofile->tt_srpt->title);
    ifofile->tt_srpt->title = NULL;
    free(ifofile->tt_srpt);
    ifofile->tt_srpt = NULL;
  }
}


int ifoRead_VTS_PTT_SRPT(ifo_handle_t *ifofile) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  vts_ptt_srpt_t *vts_ptt_srpt = NULL;
  int info_length, i, j;
  char *data = NULL;
  char buf[VTS_PTT_SRPT_SIZE];
  buf_reader b;

  if(!ifofile)
    return 0;

  if(!ifofile->vtsi_mat)
    return 0;

  if(ifofile->vtsi_mat->vts_ptt_srpt == 0) /* mandatory */
    return 0;

  if(!DVDFileSeek_(ifop->file,
                   ifofile->vtsi_mat->vts_ptt_srpt * DVD_BLOCK_LEN))
    return 0;

  vts_ptt_srpt = calloc(1, sizeof(vts_ptt_srpt_t));
  if(!vts_ptt_srpt)
    return 0;

  vts_ptt_srpt->title = NULL;
  ifofile->vts_ptt_srpt = vts_ptt_srpt;

  b.wbuf = buf;
  b.buflen = VTS_PTT_SRPT_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
    Log0(ifop->ctx, "Unable to read PTT search table.");
    goto fail;
  }

  ReadBuf16(&b, &vts_ptt_srpt->nr_of_srpts);
  SkipZeroBuf(&b, 2);
  ReadBuf32(&b, &vts_ptt_srpt->last_byte);

  CHECK_VALUE(vts_ptt_srpt->nr_of_srpts != 0);
  CHECK_VALUE(vts_ptt_srpt->nr_of_srpts < 100); /* ?? */

  /* E-One releases don't fill this field */
  if(vts_ptt_srpt->last_byte == 0) {
    vts_ptt_srpt->last_byte  = vts_ptt_srpt->nr_of_srpts * sizeof(uint32_t) - 1 + VTS_PTT_SRPT_SIZE;
  }
  info_length = vts_ptt_srpt->last_byte + 1 - VTS_PTT_SRPT_SIZE;
  if(vts_ptt_srpt->nr_of_srpts > info_length / sizeof(uint32_t)) {
    Log0(ifop->ctx, "PTT search table too small.");
    goto fail;
  }

  if(vts_ptt_srpt->nr_of_srpts == 0) {
    Log0(ifop->ctx, "Zero entries in PTT search table.");
    goto fail;
  }

  vts_ptt_srpt->ttu_offset = calloc(1, info_length);
  if(!vts_ptt_srpt->ttu_offset)
    goto fail;

  data = (char*)vts_ptt_srpt->ttu_offset;

  b.wbuf = data;
  b.buflen = info_length;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
    Log0(ifop->ctx, "Unable to read PTT search table.");
    goto fail;
  }

  for(i = 0; i < vts_ptt_srpt->nr_of_srpts; i++) {
    /* Transformers 3 has PTT start bytes that point outside the SRPT PTT */
    uint32_t start;
    ReadBuf32(&b, &start);
    if(start + PTT_INFO_SIZE > vts_ptt_srpt->last_byte + 1) {
      /* don't mess with any bytes beyond the end of the allocation */
      vts_ptt_srpt->nr_of_srpts = i;
      break;
    }
    data[i] = start;
    /* assert(data[i] + PTT_INFO_SIZE <= vts_ptt_srpt->last_byte + 1);
       Magic Knight Rayearth Daybreak is mastered very strange and has
       Titles with 0 PTTs. They all have a data[i] offsets beyond the end of
       of the vts_ptt_srpt structure. */
    CHECK_VALUE(data[i] + PTT_INFO_SIZE <= vts_ptt_srpt->last_byte + 1 + 4);
  }

  vts_ptt_srpt->title = calloc(vts_ptt_srpt->nr_of_srpts, sizeof(ttu_t));
  if(!vts_ptt_srpt->title)
    goto fail;

  for(i = 0; i < vts_ptt_srpt->nr_of_srpts; i++) {
    int n;
    if(i < vts_ptt_srpt->nr_of_srpts - 1)
      n = (data[i+1] - data[i]);
    else
      n = (vts_ptt_srpt->last_byte + 1 - data[i]);

    /* assert(n > 0 && (n % 4) == 0);
       Magic Knight Rayearth Daybreak is mastered very strange and has
       Titles with 0 PTTs. */
    if(n < 0) n = 0;

    /* DVDs created by the VDR-to-DVD device LG RC590M violate the following requirement */
    CHECK_VALUE(n % 4 == 0);

    vts_ptt_srpt->title[i].nr_of_ptts = n / 4;
    vts_ptt_srpt->title[i].ptt = calloc(n / 4, sizeof(ptt_info_t));
    if(!vts_ptt_srpt->title[i].ptt) {
      for(n = 0; n < i; n++)
        free(vts_ptt_srpt->title[n].ptt);

      goto fail;
    }
    for(j = 0; j < vts_ptt_srpt->title[i].nr_of_ptts; j++) {
      /* The assert placed here because of Magic Knight Rayearth Daybreak */
      CHECK_VALUE(data[i] + PTT_INFO_SIZE <= vts_ptt_srpt->last_byte + 1);
      vts_ptt_srpt->title[i].ptt[j].pgcn
        = *(uint16_t*)(((char *)data) + data[i] + 4*j - VTS_PTT_SRPT_SIZE);
      vts_ptt_srpt->title[i].ptt[j].pgn
        = *(uint16_t*)(((char *)data) + data[i] + 4*j + 2 - VTS_PTT_SRPT_SIZE);
    }
  }

  for(i = 0; i < vts_ptt_srpt->nr_of_srpts; i++) {
    for(j = 0; j < vts_ptt_srpt->title[i].nr_of_ptts; j++) {
      B2N_16(vts_ptt_srpt->title[i].ptt[j].pgcn);
      B2N_16(vts_ptt_srpt->title[i].ptt[j].pgn);
    }
  }

  for(i = 0; i < vts_ptt_srpt->nr_of_srpts; i++) {
    CHECK_VALUE(vts_ptt_srpt->title[i].nr_of_ptts < 1000); /* ?? */
    for(j = 0; j < vts_ptt_srpt->title[i].nr_of_ptts; j++) {
      CHECK_VALUE(vts_ptt_srpt->title[i].ptt[j].pgcn != 0 );
      CHECK_VALUE(vts_ptt_srpt->title[i].ptt[j].pgcn < 1000); /* ?? */
      CHECK_VALUE(vts_ptt_srpt->title[i].ptt[j].pgn != 0);
      CHECK_VALUE(vts_ptt_srpt->title[i].ptt[j].pgn < 100); /* ?? */
      //don't abort here. E-One DVDs contain PTT with pgcn or pgn == 0
    }
  }

  return 1;

fail:
  free(vts_ptt_srpt->ttu_offset);
  ifofile->vts_ptt_srpt = NULL;
  free(vts_ptt_srpt->title);
  free(vts_ptt_srpt);
  return 0;
}


void ifoFree_VTS_PTT_SRPT(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  if(ifofile->vts_ptt_srpt) {
    int i;
    for(i = 0; i < ifofile->vts_ptt_srpt->nr_of_srpts; i++)
      free(ifofile->vts_ptt_srpt->title[i].ptt);
    free(ifofile->vts_ptt_srpt->ttu_offset);
    free(ifofile->vts_ptt_srpt->title);
    free(ifofile->vts_ptt_srpt);
    ifofile->vts_ptt_srpt = 0;
  }
}


int ifoRead_PTL_MAIT(ifo_handle_t *ifofile) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  ptl_mait_t *ptl_mait;
  int info_length;
  unsigned int i, j;
  char buf[PTL_MAIT_SIZE];
  buf_reader b;

  if(!ifofile)
    return 0;

  if(!ifofile->vmgi_mat)
    return 0;

  if(!ifofile->vmgi_mat->ptl_mait)
    return 1;

  if(!DVDFileSeek_(ifop->file, ifofile->vmgi_mat->ptl_mait * DVD_BLOCK_LEN))
    return 0;

  ptl_mait = calloc(1, sizeof(ptl_mait_t));
  if(!ptl_mait)
    return 0;

  ifofile->ptl_mait = ptl_mait;

  b.wbuf = buf;
  b.buflen = PTL_MAIT_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
    free(ptl_mait);
    ifofile->ptl_mait = NULL;
    return 0;
  }

  ReadBuf16(&b, &ptl_mait->nr_of_countries);
  ReadBuf16(&b, &ptl_mait->nr_of_vtss);
  ReadBuf32(&b, &ptl_mait->last_byte);

  CHECK_VALUE(ptl_mait->nr_of_countries != 0);
  CHECK_VALUE(ptl_mait->nr_of_countries < 100); /* ?? */
  CHECK_VALUE(ptl_mait->nr_of_vtss != 0);
  CHECK_VALUE(ptl_mait->nr_of_vtss < 100); /* ?? */
  CHECK_VALUE(ptl_mait->nr_of_countries * PTL_MAIT_COUNTRY_SIZE
              <= ptl_mait->last_byte + 1 - PTL_MAIT_SIZE);

  info_length = ptl_mait->nr_of_countries * sizeof(ptl_mait_country_t);
  ptl_mait->countries = calloc(1, info_length);
  if(!ptl_mait->countries) {
    free(ptl_mait);
    ifofile->ptl_mait = NULL;
    return 0;
  }
  for(i = 0; i < ptl_mait->nr_of_countries; i++) {
    ptl_mait->countries[i].pf_ptl_mai = NULL;
  }

  for(i = 0; i < ptl_mait->nr_of_countries; i++) {
    char bufcountry[PTL_MAIT_COUNTRY_SIZE];

    b.wbuf = bufcountry;
    b.buflen = PTL_MAIT_COUNTRY_SIZE;

    if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
      Log0(ifop->ctx, "Unable to read PTL_MAIT.");
      free(ptl_mait->countries);
      free(ptl_mait);
      ifofile->ptl_mait = NULL;
      return 0;
    }

    ReadBuf16(&b, &ptl_mait->countries[i].country_code);
    SkipZeroBuf(&b, 2);
    ReadBuf16(&b, &ptl_mait->countries[i].pf_ptl_mai_start_byte);
    SkipZeroBuf(&b, 2);
  }

  for(i = 0; i < ptl_mait->nr_of_countries; i++) {
    CHECK_VALUE(ptl_mait->countries[i].pf_ptl_mai_start_byte
                + sizeof(pf_level_t) * (ptl_mait->nr_of_vtss + 1) <= ptl_mait->last_byte + 1);
  }

  for(i = 0; i < ptl_mait->nr_of_countries; i++) {
    uint16_t *pf_temp;

    if(!DVDFileSeek_(ifop->file,
                     ifofile->vmgi_mat->ptl_mait * DVD_BLOCK_LEN
                     + ptl_mait->countries[i].pf_ptl_mai_start_byte)) {
      Log0(ifop->ctx, "Unable to seek PTL_MAIT table at index %d.",i);
      free(ptl_mait->countries);
      free(ptl_mait);
      ifofile->ptl_mait = NULL;
      return 0;
    }
    info_length = (ptl_mait->nr_of_vtss + 1) * sizeof(pf_level_t);
    pf_temp = calloc(1, info_length);
    if(!pf_temp) {
      free_ptl_mait(ptl_mait, i);
      ifofile->ptl_mait = NULL;
      return 0;
    }

    b.wbuf = pf_temp;
    b.buflen = info_length;

    if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
      Log0(ifop->ctx, "Unable to read PTL_MAIT table at index %d.",i);
      free(pf_temp);
      free_ptl_mait(ptl_mait, i);
      ifofile->ptl_mait = NULL;
      return 0;
    }
    for (j = 0; j < ((ptl_mait->nr_of_vtss + 1U) * 8U); j++) {
      B2N_16(pf_temp[j]);
    }
    ptl_mait->countries[i].pf_ptl_mai = calloc(1, info_length);
    if(!ptl_mait->countries[i].pf_ptl_mai) {
      free(pf_temp);
      free_ptl_mait(ptl_mait, i);
      ifofile->ptl_mait = NULL;
      return 0;
    }
    { /* Transpose the array so we can use C indexing. */
      int level, vts;
      for(level = 0; level < PTL_MAIT_NUM_LEVEL; level++) {
        for(vts = 0; vts <= ptl_mait->nr_of_vtss; vts++) {
          ptl_mait->countries[i].pf_ptl_mai[vts][level] =
            pf_temp[(7-level)*(ptl_mait->nr_of_vtss+1) + vts];
        }
      }
      free(pf_temp);
    }
  }
  return 1;
}

void ifoFree_PTL_MAIT(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  if(ifofile->ptl_mait) {
    unsigned int i;

    for(i = 0; i < ifofile->ptl_mait->nr_of_countries; i++) {
      free(ifofile->ptl_mait->countries[i].pf_ptl_mai);
    }
    free(ifofile->ptl_mait->countries);
    free(ifofile->ptl_mait);
    ifofile->ptl_mait = NULL;
  }
}

int ifoRead_VTS_TMAPT(ifo_handle_t *ifofile) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  vts_tmapt_t *vts_tmapt;
  uint32_t *vts_tmap_srp;
  unsigned int offset;
  int info_length;
  unsigned int i, j;
  char buf[VTS_TMAPT_SIZE];
  buf_reader b;

  if(!ifofile)
    return 0;

  if(!ifofile->vtsi_mat)
    return 0;

  if(ifofile->vtsi_mat->vts_tmapt == 0) {
    ifofile->vts_tmapt = NULL;
    return 1;
  }

  offset = ifofile->vtsi_mat->vts_tmapt * DVD_BLOCK_LEN;

  if(!DVDFileSeek_(ifop->file, offset))
    return 0;

  vts_tmapt = calloc(1, sizeof(vts_tmapt_t));
  if(!vts_tmapt)
    return 0;

  ifofile->vts_tmapt = vts_tmapt;

  b.wbuf = buf;
  b.buflen = VTS_TMAPT_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
    Log0(ifop->ctx, "Unable to read VTS_TMAPT.");
    free(vts_tmapt);
    ifofile->vts_tmapt = NULL;
    return 0;
  }

  ReadBuf16(&b, &vts_tmapt->nr_of_tmaps);
  SkipZeroBuf(&b, 2);
  ReadBuf32(&b, &vts_tmapt->last_byte);

  info_length = vts_tmapt->nr_of_tmaps * sizeof(*vts_tmap_srp);

  vts_tmap_srp = calloc(1, info_length);
  if(!vts_tmap_srp) {
    free(vts_tmapt);
    ifofile->vts_tmapt = NULL;
    return 0;
  }

  vts_tmapt->tmap_offset = vts_tmap_srp;

  char bufsrp[info_length];
  b.wbuf = bufsrp;
  b.buflen = info_length;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
    Log0(ifop->ctx, "Unable to read VTS_TMAPT.");
    free(vts_tmap_srp);
    free(vts_tmapt);
    ifofile->vts_tmapt = NULL;
    return 0;
  }

  for (i = 0; i < vts_tmapt->nr_of_tmaps; i++) {
    ReadBuf32(&b, &vts_tmap_srp[i]);
  }

  vts_tmapt->tmap = calloc(vts_tmapt->nr_of_tmaps, sizeof(vts_tmap_t));
  if(!vts_tmapt->tmap) {
    free(vts_tmap_srp);
    free(vts_tmapt);
    ifofile->vts_tmapt = NULL;
    return 0;
  }

  for(i = 0; i < vts_tmapt->nr_of_tmaps; i++) {
    char buftmap[VTS_TMAP_SIZE];

    b.wbuf = buftmap;
    b.buflen = VTS_TMAP_SIZE;

    if(!DVDFileSeek_(ifop->file, offset + vts_tmap_srp[i])) {
      ifoFree_VTS_TMAPT(ifofile);
      return 0;
    }

    if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
      Log0(ifop->ctx, "Unable to read VTS_TMAP.");
      ifoFree_VTS_TMAPT(ifofile);
      return 0;
    }

    ReadBuf8(&b, &vts_tmapt->tmap[i].tmu);
    SkipZeroBuf(&b, 1);
    ReadBuf16(&b, &vts_tmapt->tmap[i].nr_of_entries);

    if(vts_tmapt->tmap[i].nr_of_entries == 0) { /* Early out if zero entries */
      vts_tmapt->tmap[i].map_ent = NULL;
      continue;
    }

    info_length = vts_tmapt->tmap[i].nr_of_entries * sizeof(map_ent_t);

    vts_tmapt->tmap[i].map_ent = calloc(1, info_length);
    if(!vts_tmapt->tmap[i].map_ent) {
      ifoFree_VTS_TMAPT(ifofile);
      return 0;
    }

    char bufmaps[info_length];
    b.wbuf = bufmaps;
    b.buflen = info_length;

    if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
      Log0(ifop->ctx, "Unable to read VTS_TMAP_ENT.");
      ifoFree_VTS_TMAPT(ifofile);
      return 0;
    }

    for(j = 0; j < vts_tmapt->tmap[i].nr_of_entries; j++)
      ReadBuf32(&b, &vts_tmapt->tmap[i].map_ent[j]);
  }

  return 1;
}

void ifoFree_VTS_TMAPT(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  if(ifofile->vts_tmapt) {
    unsigned int i;

    for(i = 0; i < ifofile->vts_tmapt->nr_of_tmaps; i++)
      if(ifofile->vts_tmapt->tmap[i].map_ent)
        free(ifofile->vts_tmapt->tmap[i].map_ent);
    free(ifofile->vts_tmapt->tmap);
    free(ifofile->vts_tmapt->tmap_offset);
    free(ifofile->vts_tmapt);
    ifofile->vts_tmapt = NULL;
  }
}


int ifoRead_TITLE_C_ADT(ifo_handle_t *ifofile) {

  if(!ifofile)
    return 0;

  if(!ifofile->vtsi_mat)
    return 0;

  if(ifofile->vtsi_mat->vts_c_adt == 0) /* mandatory */
    return 0;

  ifofile->vts_c_adt = calloc(1, sizeof(c_adt_t));
  if(!ifofile->vts_c_adt)
    return 0;

  if(!ifoRead_C_ADT_internal(ifofile, ifofile->vts_c_adt,
                             ifofile->vtsi_mat->vts_c_adt)) {
    free(ifofile->vts_c_adt);
    ifofile->vts_c_adt = NULL;
    return 0;
  }

  return 1;
}

int ifoRead_C_ADT(ifo_handle_t *ifofile) {
  unsigned int sector;

  if(!ifofile)
    return 0;

  if(ifofile->vmgi_mat) {
    if(ifofile->vmgi_mat->vmgm_c_adt == 0)
      return 1;
    sector = ifofile->vmgi_mat->vmgm_c_adt;
  } else if(ifofile->vtsi_mat) {
    if(ifofile->vtsi_mat->vtsm_c_adt == 0)
      return 1;
    sector = ifofile->vtsi_mat->vtsm_c_adt;
  } else {
    return 0;
  }

  ifofile->menu_c_adt = calloc(1, sizeof(c_adt_t));
  if(!ifofile->menu_c_adt)
    return 0;

  if(!ifoRead_C_ADT_internal(ifofile, ifofile->menu_c_adt, sector)) {
    free(ifofile->menu_c_adt);
    ifofile->menu_c_adt = NULL;
    return 0;
  }

  return 1;
}

static int ifoRead_C_ADT_internal(ifo_handle_t *ifofile,
                                  c_adt_t *c_adt, unsigned int sector) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  size_t i, info_length;
  char buf[C_ADT_SIZE];
  buf_reader b;

  if(!DVDFileSeek_(ifop->file, sector * DVD_BLOCK_LEN))
    return 0;

  b.wbuf = buf;
  b.buflen = C_ADT_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen)))
    return 0;

  ReadBuf16(&b, &c_adt->nr_of_vobs);
  SkipZeroBuf(&b, 2);
  ReadBuf32(&b, &c_adt->last_byte);

  if(c_adt->last_byte + 1 < C_ADT_SIZE)
    return 0;

  info_length = c_adt->last_byte + 1 - C_ADT_SIZE;

  /* assert(c_adt->nr_of_vobs > 0);
     Magic Knight Rayearth Daybreak is mastered very strange and has
     Titles with a VOBS that has no cells. */
  CHECK_VALUE(info_length % CELL_ADDR_SIZE == 0);

  /* assert(info_length / CELL_ADDR_SIZE >= c_adt->nr_of_vobs);
     Enemy of the State region 2 (de) has Titles where nr_of_vobs field
     is to high, they high ones are never referenced though. */
  if(info_length / CELL_ADDR_SIZE < c_adt->nr_of_vobs) {
    Log1(ifop->ctx, "C_ADT nr_of_vobs > available info entries");
    c_adt->nr_of_vobs = info_length / CELL_ADDR_SIZE;
  }

  c_adt->cell_adr_table = calloc(1, info_length);
  if(!c_adt->cell_adr_table)
    return 0;

  if(info_length)
  {
    char bufcells[info_length];
    b.wbuf = bufcells;
    b.buflen = info_length;

    if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
      free(c_adt->cell_adr_table);
      return 0;
    }

    for(i = 0; i < info_length/CELL_ADDR_SIZE; i++) {
      ReadBuf16(&b, &c_adt->cell_adr_table[i].vob_id);
      ReadBuf8(&b, &c_adt->cell_adr_table[i].cell_id);
      SkipZeroBuf(&b, 1);
      ReadBuf32(&b, &c_adt->cell_adr_table[i].start_sector);
      ReadBuf32(&b, &c_adt->cell_adr_table[i].last_sector);

      CHECK_VALUE(c_adt->cell_adr_table[i].vob_id > 0);
      CHECK_VALUE(c_adt->cell_adr_table[i].vob_id <= c_adt->nr_of_vobs);
      CHECK_VALUE(c_adt->cell_adr_table[i].cell_id > 0);
      CHECK_VALUE(c_adt->cell_adr_table[i].start_sector <
                  c_adt->cell_adr_table[i].last_sector);
    }
  }

  return 1;
}


static void ifoFree_C_ADT_internal(c_adt_t *c_adt) {
  if(c_adt) {
    free(c_adt->cell_adr_table);
    free(c_adt);
  }
}

void ifoFree_C_ADT(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  ifoFree_C_ADT_internal(ifofile->menu_c_adt);
  ifofile->menu_c_adt = NULL;
}

void ifoFree_TITLE_C_ADT(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  ifoFree_C_ADT_internal(ifofile->vts_c_adt);
  ifofile->vts_c_adt = NULL;
}

int ifoRead_TITLE_VOBU_ADMAP(ifo_handle_t *ifofile) {
  if(!ifofile)
    return 0;

  if(!ifofile->vtsi_mat)
    return 0;

  if(ifofile->vtsi_mat->vts_vobu_admap == 0) /* mandatory */
    return 0;

  ifofile->vts_vobu_admap = calloc(1, sizeof(vobu_admap_t));
  if(!ifofile->vts_vobu_admap)
    return 0;

  if(!ifoRead_VOBU_ADMAP_internal(ifofile, ifofile->vts_vobu_admap,
                                  ifofile->vtsi_mat->vts_vobu_admap)) {
    free(ifofile->vts_vobu_admap);
    ifofile->vts_vobu_admap = NULL;
    return 0;
  }

  return 1;
}

int ifoRead_VOBU_ADMAP(ifo_handle_t *ifofile) {
  unsigned int sector;

  if(!ifofile)
    return 0;

  if(ifofile->vmgi_mat) {
    if(ifofile->vmgi_mat->vmgm_vobu_admap == 0)
      return 1;
    sector = ifofile->vmgi_mat->vmgm_vobu_admap;
  } else if(ifofile->vtsi_mat) {
    if(ifofile->vtsi_mat->vtsm_vobu_admap == 0)
      return 1;
    sector = ifofile->vtsi_mat->vtsm_vobu_admap;
  } else {
    return 0;
  }

  ifofile->menu_vobu_admap = calloc(1, sizeof(vobu_admap_t));
  if(!ifofile->menu_vobu_admap)
    return 0;

  if(!ifoRead_VOBU_ADMAP_internal(ifofile, ifofile->menu_vobu_admap, sector)) {
    free(ifofile->menu_vobu_admap);
    ifofile->menu_vobu_admap = NULL;
    return 0;
  }

  return 1;
}

static int ifoRead_VOBU_ADMAP_internal(ifo_handle_t *ifofile,
                                       vobu_admap_t *vobu_admap,
                                       unsigned int sector) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  unsigned int i;
  int info_length;
  char buf[VOBU_ADMAP_SIZE];
  buf_reader b;

  if(!DVDFileSeekForce_(ifop->file, sector * DVD_BLOCK_LEN, sector))
    return 0;

  b.wbuf = buf;
  b.buflen = VOBU_ADMAP_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen)))
    return 0;

  ReadBuf32(&b, &vobu_admap->last_byte);

  info_length = vobu_admap->last_byte + 1 - VOBU_ADMAP_SIZE;
  /* assert(info_length > 0);
     Magic Knight Rayearth Daybreak is mastered very strange and has
     Titles with a VOBS that has no VOBUs. */
  CHECK_VALUE(info_length % sizeof(uint32_t) == 0);

  vobu_admap->vobu_start_sectors = calloc(1, info_length);
  if(!vobu_admap->vobu_start_sectors) {
    return 0;
  }
  if(info_length)
  {
    char bufsect[info_length];
    b.wbuf = bufsect;
    b.buflen = info_length;
    if (!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
      free(vobu_admap->vobu_start_sectors);
      return 0;
    }

    for(i = 0; i < info_length/sizeof(uint32_t); i++)
      ReadBuf32(&b, &vobu_admap->vobu_start_sectors[i]);
  }

  return 1;
}


static void ifoFree_VOBU_ADMAP_internal(vobu_admap_t *vobu_admap) {
  if(vobu_admap) {
    free(vobu_admap->vobu_start_sectors);
    free(vobu_admap);
  }
}

void ifoFree_VOBU_ADMAP(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  ifoFree_VOBU_ADMAP_internal(ifofile->menu_vobu_admap);
  ifofile->menu_vobu_admap = NULL;
}

void ifoFree_TITLE_VOBU_ADMAP(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  ifoFree_VOBU_ADMAP_internal(ifofile->vts_vobu_admap);
  ifofile->vts_vobu_admap = NULL;
}

int ifoRead_PGCIT(ifo_handle_t *ifofile) {

  if(!ifofile)
    return 0;

  if(!ifofile->vtsi_mat)
    return 0;

  if(ifofile->vtsi_mat->vts_pgcit == 0) /* mandatory */
    return 0;

  ifofile->vts_pgcit = calloc(1, sizeof(pgcit_t));
  if(!ifofile->vts_pgcit)
    return 0;

  ifofile->vts_pgcit->ref_count = 1;
  if(!ifoRead_PGCIT_internal(ifofile, ifofile->vts_pgcit,
                             ifofile->vtsi_mat->vts_pgcit * DVD_BLOCK_LEN)) {
    free(ifofile->vts_pgcit);
    ifofile->vts_pgcit = NULL;
    return 0;
  }

  return 1;
}

static int find_dup_pgc(pgci_srp_t *pgci_srp, uint32_t start_byte, int count) {
  int i;

  for(i = 0; i < count; i++) {
    if(pgci_srp[i].pgc_start_byte == start_byte) {
      return i;
    }
  }
  return -1;
}

static int ifoRead_PGCIT_internal(ifo_handle_t *ifofile, pgcit_t *pgcit,
                                  unsigned int offset) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  int i, info_length;
  char *data;
  char buf[PGCIT_SIZE];
  buf_reader b;

  if(!DVDFileSeek_(ifop->file, offset))
    return 0;

  b.wbuf = buf;
  b.buflen = PGCIT_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen)))
    return 0;

  ReadBuf16(&b, &pgcit->nr_of_pgci_srp);
  SkipZeroBuf(&b, 2);
  ReadBuf32(&b, &pgcit->last_byte);

  /* assert(pgcit->nr_of_pgci_srp != 0);
     Magic Knight Rayearth Daybreak is mastered very strange and has
     Titles with 0 PTTs. */
  CHECK_VALUE(pgcit->nr_of_pgci_srp < 10000); /* ?? seen max of 1338 */

  if (pgcit->nr_of_pgci_srp == 0) {
    pgcit->pgci_srp = NULL;
    return 1;
  }

  info_length = pgcit->nr_of_pgci_srp * PGCI_SRP_SIZE;
  data = calloc(1, info_length);
  if(!data)
    return 0;

  b.wbuf = data;
  b.buflen = info_length;

  if(info_length && !(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
    free(data);
    return 0;
  }

  pgcit->pgci_srp = calloc(pgcit->nr_of_pgci_srp, sizeof(pgci_srp_t));
  if(!pgcit->pgci_srp) {
    free(data);
    return 0;
  }
  for(i = 0; i < pgcit->nr_of_pgci_srp; i++) {
    read_pgci_srp_(&b, &pgcit->pgci_srp[i]);
    CHECK_VALUE(pgcit->pgci_srp[i].zero_1 == 0);
  }
  free(data);

  for(i = 0; i < pgcit->nr_of_pgci_srp; i++)
    CHECK_VALUE(pgcit->pgci_srp[i].pgc_start_byte + PGC_SIZE <= pgcit->last_byte+1);

  for(i = 0; i < pgcit->nr_of_pgci_srp; i++) {
    int dup;
    if((dup = find_dup_pgc(pgcit->pgci_srp, pgcit->pgci_srp[i].pgc_start_byte, i)) >= 0) {
      pgcit->pgci_srp[i].pgc = pgcit->pgci_srp[dup].pgc;
      pgcit->pgci_srp[i].pgc->ref_count++;
      continue;
    }
    pgcit->pgci_srp[i].pgc = calloc(1, sizeof(pgc_t));
    if(!pgcit->pgci_srp[i].pgc) {
      int j;
      for(j = 0; j < i; j++) {
        ifoFree_PGC(&pgcit->pgci_srp[j].pgc);
      }
      goto fail;
    }
    pgcit->pgci_srp[i].pgc->ref_count = 1;
    if(!ifoRead_PGC(ifofile, pgcit->pgci_srp[i].pgc,
                    offset + pgcit->pgci_srp[i].pgc_start_byte)) {
      Log0(ifop->ctx, "Unable to read invalid PCG");
      //E-One releases provide bogus PGC, ie: out of bound start_byte
      free(pgcit->pgci_srp[i].pgc);
      pgcit->pgci_srp[i].pgc = NULL;
    }
  }

  return 1;
fail:
  free(pgcit->pgci_srp);
  pgcit->pgci_srp = NULL;
  return 0;
}

static void ifoFree_PGCIT_internal(pgcit_t **pgcit) {
  if(pgcit && *pgcit && (--(*pgcit)->ref_count <= 0)) {
    int i;
    for(i = 0; i < (*pgcit)->nr_of_pgci_srp; i++)
    {
      ifoFree_PGC(&(*pgcit)->pgci_srp[i].pgc);
    }
    free((*pgcit)->pgci_srp);
    free(*pgcit);
  }
  if (pgcit)
    *pgcit = NULL;
}

void ifoFree_PGCIT(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  if(ifofile->vts_pgcit) {
    ifoFree_PGCIT_internal(&ifofile->vts_pgcit);
  }
}

static int find_dup_lut(pgci_lu_t *lu, uint32_t start_byte, int count) {
  int i;

  for(i = 0; i < count; i++) {
    if(lu[i].lang_start_byte == start_byte) {
      return i;
    }
  }
  return -1;
}

int ifoRead_PGCI_UT(ifo_handle_t *ifofile) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  pgci_ut_t *pgci_ut;
  unsigned int sector;
  unsigned int i;
  int info_length;
  char *data;
  char buf[PGCI_UT_SIZE];
  buf_reader b;

  if(!ifofile)
    return 0;

  if(ifofile->vmgi_mat) {
    if(ifofile->vmgi_mat->vmgm_pgci_ut == 0)
      return 1;
    sector = ifofile->vmgi_mat->vmgm_pgci_ut;
  } else if(ifofile->vtsi_mat) {
    if(ifofile->vtsi_mat->vtsm_pgci_ut == 0)
      return 1;
    sector = ifofile->vtsi_mat->vtsm_pgci_ut;
  } else {
    return 0;
  }

  ifofile->pgci_ut = calloc(1, sizeof(pgci_ut_t));
  if(!ifofile->pgci_ut)
    return 0;

  if(!DVDFileSeek_(ifop->file, sector * DVD_BLOCK_LEN)) {
    free(ifofile->pgci_ut);
    ifofile->pgci_ut = NULL;
    return 0;
  }

  b.wbuf = buf;
  b.buflen = PGCI_UT_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
    free(ifofile->pgci_ut);
    ifofile->pgci_ut = NULL;
    return 0;
  }

  pgci_ut = ifofile->pgci_ut;

  ReadBuf16(&b, &pgci_ut->nr_of_lus);
  SkipZeroBuf(&b, 2);
  ReadBuf32(&b, &pgci_ut->last_byte);

  CHECK_VALUE(pgci_ut->nr_of_lus != 0);
  CHECK_VALUE(pgci_ut->nr_of_lus < 100); /* ?? 3-4 ? */
  CHECK_VALUE((uint32_t)pgci_ut->nr_of_lus * PGCI_LU_SIZE < pgci_ut->last_byte);

  info_length = pgci_ut->nr_of_lus * PGCI_LU_SIZE;
  data = calloc(1, info_length);
  if(!data) {
    free(pgci_ut);
    ifofile->pgci_ut = NULL;
    return 0;
  }

  b.wbuf = data;
  b.buflen = info_length;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen))) {
    free(data);
    free(pgci_ut);
    ifofile->pgci_ut = NULL;
    return 0;
  }

  pgci_ut->lu = calloc(pgci_ut->nr_of_lus, sizeof(pgci_lu_t));
  if(!pgci_ut->lu) {
    free(data);
    free(pgci_ut);
    ifofile->pgci_ut = NULL;
    return 0;
  }
  for(i = 0; i < pgci_ut->nr_of_lus; i++) {
    ReadBuf16(&b, &pgci_ut->lu[i].lang_code);
    ReadBuf8(&b, &pgci_ut->lu[i].lang_extension);
    ReadBuf8(&b, &pgci_ut->lu[i].exists);
    ReadBuf32(&b, &pgci_ut->lu[i].lang_start_byte);
  }
  free(data);

  for(i = 0; i < pgci_ut->nr_of_lus; i++) {
    /* Maybe this is only defined for v1.1 and later titles? */
    /* If the bits in 'lu[i].exists' are enumerated abcd efgh then:
       VTS_x_yy.IFO        VIDEO_TS.IFO
       a == 0x83 "Root"         0x82 "Title"
       b == 0x84 "Subpicture"
       c == 0x85 "Audio"
       d == 0x86 "Angle"
       e == 0x87 "PTT"
    */
    CHECK_VALUE((pgci_ut->lu[i].exists & 0x07) == 0);
  }

  for(i = 0; i < pgci_ut->nr_of_lus; i++) {
    int dup;
    if((dup = find_dup_lut(pgci_ut->lu, pgci_ut->lu[i].lang_start_byte, i)) >= 0) {
      pgci_ut->lu[i].pgcit = pgci_ut->lu[dup].pgcit;
      pgci_ut->lu[i].pgcit->ref_count++;
      continue;
    }
    pgci_ut->lu[i].pgcit = calloc(1, sizeof(pgcit_t));
    if(!pgci_ut->lu[i].pgcit) {
      unsigned int j;
      for(j = 0; j < i; j++) {
        ifoFree_PGCIT_internal(&pgci_ut->lu[j].pgcit);
      }
      free(pgci_ut->lu);
      free(pgci_ut);
      ifofile->pgci_ut = NULL;
      return 0;
    }
    pgci_ut->lu[i].pgcit->ref_count = 1;
    if(!ifoRead_PGCIT_internal(ifofile, pgci_ut->lu[i].pgcit,
                               sector * DVD_BLOCK_LEN
                               + pgci_ut->lu[i].lang_start_byte)) {
      unsigned int j;
      for(j = 0; j <= i; j++) {
        ifoFree_PGCIT_internal(&pgci_ut->lu[j].pgcit);
      }
      free(pgci_ut->lu);
      free(pgci_ut);
      ifofile->pgci_ut = NULL;
      return 0;
    }
    /* FIXME: Iterate and verify that all menus that should exists accordingly
     * to pgci_ut->lu[i].exists really do? */
  }

  return 1;
}


void ifoFree_PGCI_UT(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  if(ifofile->pgci_ut) {
    unsigned int i;

    for(i = 0; i < ifofile->pgci_ut->nr_of_lus; i++) {
      ifoFree_PGCIT_internal(&ifofile->pgci_ut->lu[i].pgcit);
    }
    free(ifofile->pgci_ut->lu);
    free(ifofile->pgci_ut);
    ifofile->pgci_ut = NULL;
  }
}

static int ifoRead_VTS_ATTRIBUTES(ifo_handle_t *ifofile,
                                  vts_attributes_t *vts_attributes,
                                  unsigned int offset) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  unsigned int i;
  char buf[VTS_ATTRIBUTES_SIZE];
  buf_reader b;

  if(!DVDFileSeek_(ifop->file, offset))
    return 0;

  b.wbuf = buf;
  b.buflen = VTS_ATTRIBUTES_SIZE;

  if(!(DVDReadBytes(ifop->file, b.wbuf, b.buflen)))
    return 0;

  ReadBuf32(&b, &vts_attributes->last_byte);
  ReadBuf32(&b, &vts_attributes->vts_cat);
  read_video_attr_(&b, &vts_attributes->vtsm_vobs_attr);
  SkipZeroBuf(&b, 1);
  ReadBuf8(&b, &vts_attributes->nr_of_vtsm_audio_streams);
  read_audio_attr_(&b, &vts_attributes->vtsm_audio_attr);
  SkipZeroBuf(&b, AUDIO_ATTR_SIZE * 7);
  SkipZeroBuf(&b, 16);
  SkipZeroBuf(&b, 1);
  ReadBuf8(&b, &vts_attributes->nr_of_vtsm_subp_streams);
  read_subp_attr_(&b, &vts_attributes->vtsm_subp_attr);
  SkipZeroBuf(&b, SUBP_ATTR_SIZE * 27);
  SkipZeroBuf(&b, 2);
  read_video_attr_(&b, &vts_attributes->vtstt_vobs_video_attr);
  SkipZeroBuf(&b, 1);
  ReadBuf8(&b, &vts_attributes->nr_of_vtstt_audio_streams);
  for(i=0; i<8; i++)
    read_audio_attr_(&b, &vts_attributes->vtstt_audio_attr[i]);
  SkipZeroBuf(&b, 16);
  SkipZeroBuf(&b, 1);
  ReadBuf8(&b, &vts_attributes->nr_of_vtstt_subp_streams);
  for(i=0; i<32; i++)
    read_subp_attr_(&b, &vts_attributes->vtstt_subp_attr[i]);

  CHECK_VALUE(vts_attributes->nr_of_vtsm_audio_streams <= 1);
  CHECK_VALUE(vts_attributes->nr_of_vtsm_subp_streams <= 1);
  CHECK_VALUE(vts_attributes->nr_of_vtstt_audio_streams <= 8);
  for(i = vts_attributes->nr_of_vtstt_audio_streams; i < 8; i++)
    CHECK_ZERO(vts_attributes->vtstt_audio_attr[i]);
  CHECK_VALUE(vts_attributes->nr_of_vtstt_subp_streams <= 32);
  {
    unsigned int nr_coded;
    CHECK_VALUE(vts_attributes->last_byte + 1 >= VTS_ATTRIBUTES_MIN_SIZE);
    nr_coded = (vts_attributes->last_byte + 1 - VTS_ATTRIBUTES_MIN_SIZE)/SUBP_ATTR_SIZE;
    /* This is often nr_coded = 70, how do you know how many there really are? */
    if(nr_coded > 32) { /* We haven't read more from disk/file anyway */
      nr_coded = 32;
    }
    CHECK_VALUE(vts_attributes->nr_of_vtstt_subp_streams <= nr_coded);
    for(i = vts_attributes->nr_of_vtstt_subp_streams; i < nr_coded; i++)
      CHECK_ZERO(vts_attributes->vtstt_subp_attr[i]);
  }

  return 1;
}



int ifoRead_VTS_ATRT(ifo_handle_t *ifofile) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  vts_atrt_t *vts_atrt;
  unsigned int i, info_length, sector;
  uint32_t *data;

  if(!ifofile)
    return 0;

  if(!ifofile->vmgi_mat)
    return 0;

  if(ifofile->vmgi_mat->vts_atrt == 0) /* mandatory */
    return 0;

  sector = ifofile->vmgi_mat->vts_atrt;
  if(!DVDFileSeek_(ifop->file, sector * DVD_BLOCK_LEN))
    return 0;

  vts_atrt = calloc(1, sizeof(vts_atrt_t));
  if(!vts_atrt)
    return 0;

  ifofile->vts_atrt = vts_atrt;

  if(!(DVDReadBytes(ifop->file, vts_atrt, VTS_ATRT_SIZE))) {
    free(vts_atrt);
    ifofile->vts_atrt = NULL;
    return 0;
  }

  B2N_16(vts_atrt->nr_of_vtss);
  B2N_32(vts_atrt->last_byte);

  CHECK_ZERO(vts_atrt->zero_1);
  CHECK_VALUE(vts_atrt->nr_of_vtss != 0);
  CHECK_VALUE(vts_atrt->nr_of_vtss < 100); /* ?? */
  CHECK_VALUE((uint32_t)vts_atrt->nr_of_vtss * (4 + VTS_ATTRIBUTES_MIN_SIZE) +
              VTS_ATRT_SIZE < vts_atrt->last_byte + 1);

  info_length = vts_atrt->nr_of_vtss * sizeof(uint32_t);
  data = calloc(1, info_length);
  if(!data) {
    free(vts_atrt);
    ifofile->vts_atrt = NULL;
    return 0;
  }

  vts_atrt->vts_atrt_offsets = data;

  if(!(DVDReadBytes(ifop->file, data, info_length))) {
    free(data);
    free(vts_atrt);
    ifofile->vts_atrt = NULL;
    return 0;
  }

  for(i = 0; i < vts_atrt->nr_of_vtss; i++) {
    B2N_32(data[i]);
    CHECK_VALUE(data[i] + VTS_ATTRIBUTES_MIN_SIZE < vts_atrt->last_byte + 1);
  }

  info_length = vts_atrt->nr_of_vtss * sizeof(vts_attributes_t);
  vts_atrt->vts = calloc(1, info_length);
  if(!vts_atrt->vts) {
    free(data);
    free(vts_atrt);
    ifofile->vts_atrt = NULL;
    return 0;
  }
  for(i = 0; i < vts_atrt->nr_of_vtss; i++) {
    unsigned int offset = data[i];
    if(!ifoRead_VTS_ATTRIBUTES(ifofile, &(vts_atrt->vts[i]),
                               (sector * DVD_BLOCK_LEN) + offset)) {
      free(data);
      free(vts_atrt);
      ifofile->vts_atrt = NULL;
      return 0;
    }

    /* This assert can't be in ifoRead_VTS_ATTRIBUTES */
    CHECK_VALUE(offset + vts_atrt->vts[i].last_byte <= vts_atrt->last_byte + 1);
    /* Is this check correct? */
  }

  return 1;
}


void ifoFree_VTS_ATRT(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  if(ifofile->vts_atrt) {
    free(ifofile->vts_atrt->vts);
    free(ifofile->vts_atrt->vts_atrt_offsets);
    free(ifofile->vts_atrt);
    ifofile->vts_atrt = NULL;
  }
}


int ifoRead_TXTDT_MGI(ifo_handle_t *ifofile) {
  struct ifo_handle_private_s *ifop = PRIV(ifofile);
  txtdt_mgi_t *txtdt_mgi;

  if(!ifofile)
    return 0;

  if(!ifofile->vmgi_mat)
    return 0;

  /* Return successfully if there is nothing to read. */
  if(ifofile->vmgi_mat->txtdt_mgi == 0)
    return 1;

  if(!DVDFileSeek_(ifop->file,
                   ifofile->vmgi_mat->txtdt_mgi * DVD_BLOCK_LEN))
    return 0;

  txtdt_mgi = calloc(1, sizeof(txtdt_mgi_t));
  if(!txtdt_mgi) {
    return 0;
  }
  ifofile->txtdt_mgi = txtdt_mgi;

  if(!(DVDReadBytes(ifop->file, txtdt_mgi, TXTDT_MGI_SIZE))) {
    Log0(ifop->ctx, "Unable to read TXTDT_MGI.");
    free(txtdt_mgi);
    ifofile->txtdt_mgi = NULL;
    return 0;
  }

  /* Log1(ifop->ctx, "-- Not done yet --\n"); */
  return 1;
}

void ifoFree_TXTDT_MGI(ifo_handle_t *ifofile) {
  if(!ifofile)
    return;

  if(ifofile->txtdt_mgi) {
    free(ifofile->txtdt_mgi);
    ifofile->txtdt_mgi = NULL;
  }
}
