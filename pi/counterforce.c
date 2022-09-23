#include <notcurses/notcurses.h>

int main(void){
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
  if((mopts.n = ncplane_create(std, &mpopts)) == NULL){
    notcurses_stop(nc);
    return EXIT_FAILURE;
  }
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
  struct ncplane_options ppopts = {0};
  ppopts.cols = dimx / 2;
  ppopts.rows = dimy;
  ppopts.flags = NCPLANE_OPTION_HORALIGNED;
  ppopts.x = NCALIGN_RIGHT;
  struct ncvisual_options popts = {0};
  popts.blitter = NCBLIT_PIXEL;
  popts.scaling = NCSCALE_STRETCH;
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
  if(notcurses_render(nc)){
    notcurses_stop(nc);
    return EXIT_FAILURE;
  }
  ncinput ni;
  uint32_t k;
  while((k = notcurses_get(nc, NULL, &ni)) >= 0){
    if(k == 'q'){
      notcurses_stop(nc);
      return EXIT_SUCCESS;
    }
  }
  notcurses_stop(nc);
  return EXIT_FAILURE;
}
