#include <notcurses/notcurses.h>

struct moboctx {
  int toprpm;     // top ek/noctua mix
  int noctuarpm;  // bottom noctuae
  int phantekrpm; // bottommost phanteks
};

// fill in canned values for testing
static void
canned_values(struct moboctx* mctx){
  mctx->toprpm = 2000;
  mctx->noctuarpm = 1500;
  mctx->phantekrpm = 1800;
}

// update the various labels on the mobo side, but do not rerender
static int
update_mobo(struct ncplane* n, const struct moboctx* mctx){
  int dimy, dimx;
  ncplane_dim_yx(n, &dimy, &dimx);
  // FIXME kill off all these literal positions
  if(ncplane_printf_yx(n, 11, 20, "%d           %d           %d", mctx->toprpm, mctx->toprpm, mctx->toprpm) < 0){
    return -1;
  }
  if(ncplane_printf_yx(n, 54, 20, "%d           %d", mctx->noctuarpm, mctx->noctuarpm) < 0){
    return -1;
  }
  if(ncplane_printf_yx(n, 60, 20, "%d           %d", mctx->phantekrpm, mctx->phantekrpm) < 0){
    return -1;
  }
  return 0;
}

int main(void){
  struct moboctx mctx = {0};
  canned_values(&mctx); // FIXME
  struct notcurses* nc = notcurses_init(NULL, NULL);
  if(nc == NULL){
    exit(EXIT_FAILURE);
  }
  struct ncvisual* mobo = ncvisual_from_file("counterforce-mobo.jpg");
  if(mobo == NULL){
    notcurses_stop(nc);
    return EXIT_FAILURE;
  }
  struct ncvisual* psu = ncvisual_from_file("counterforce-psu.jpg");
  if(psu == NULL){
    notcurses_stop(nc);
    return EXIT_FAILURE;
  }
  int dimy, dimx;
  struct ncplane* std = notcurses_stddim_yx(nc, &dimy, &dimx);
  struct ncplane_options mpopts = {0};
  mpopts.cols = dimx / 2;
  mpopts.rows = dimy;
  struct ncvisual_options mopts = {0};
  mopts.blitter = NCBLIT_PIXEL;
  mopts.scaling = NCSCALE_STRETCH;
  mopts.flags = NCVISUAL_OPTION_CHILDPLANE;
  if((mopts.n = ncplane_create(std, &mpopts)) == NULL){
    notcurses_stop(nc);
    return EXIT_FAILURE;
  }
  nccell bgc = NCCELL_TRIVIAL_INITIALIZER;
  nccell_set_fg_alpha(&bgc, NCALPHA_TRANSPARENT);
  nccell_set_bg_alpha(&bgc, NCALPHA_TRANSPARENT);
  ncplane_set_base_cell(mopts.n, &bgc);
  nccell_release(mopts.n, &bgc);
  ncvgeom mgeom;
  if(ncvisual_geom(nc, mobo, &mopts, &mgeom)){
    notcurses_stop(nc);
    return EXIT_FAILURE;
  }
  struct ncplane* mobop = ncvisual_blit(nc, mobo, &mopts);
  if(mobop == NULL){
    notcurses_stop(nc);
    return EXIT_FAILURE;
  }
  ncplane_move_above(mopts.n, mobop);
  struct ncplane_options ppopts = {0};
  ppopts.cols = dimx / 2;
  ppopts.rows = dimy;
  ppopts.flags = NCPLANE_OPTION_HORALIGNED;
  ppopts.x = NCALIGN_RIGHT;
  struct ncvisual_options popts = {0};
  popts.blitter = NCBLIT_PIXEL;
  popts.scaling = NCSCALE_STRETCH;
  popts.flags = NCVISUAL_OPTION_CHILDPLANE;
  if((popts.n = ncplane_create(std, &ppopts)) == NULL){
    notcurses_stop(nc);
    return EXIT_FAILURE;
  }
  ncvgeom pgeom;
  if(ncvisual_geom(nc, psu, &popts, &pgeom)){
    notcurses_stop(nc);
    return EXIT_FAILURE;
  }
  struct ncplane* psup = ncvisual_blit(nc, psu, &popts);
  if(psup == NULL){
    notcurses_stop(nc);
    return EXIT_FAILURE;
  }
  ncinput ni;
  uint32_t k;
  do{
    if(update_mobo(mopts.n, &mctx)){
      notcurses_stop(nc);
      return EXIT_FAILURE;
    }
    if(notcurses_render(nc)){
      notcurses_stop(nc);
      return EXIT_FAILURE;
    }
    if(k == 'q'){
      notcurses_debug(nc, stderr);
      notcurses_stop(nc);
      return EXIT_SUCCESS;
    }
    // FIXME check for updates, along with stdin
  }while((k = notcurses_get(nc, NULL, &ni)) >= 0);
  notcurses_stop(nc);
  return EXIT_FAILURE;
}
