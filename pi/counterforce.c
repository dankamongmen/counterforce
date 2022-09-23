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
  struct ncvisual_options mopts = {0};
  mopts.blitter = NCBLIT_PIXEL;
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
  struct ncvisual_options popts = {0};
  popts.blitter = NCBLIT_PIXEL;
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
  sleep(4);
  // FIXME loop, showing data
  notcurses_stop(nc);
  return EXIT_SUCCESS;
}
