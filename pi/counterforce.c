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
  if(psu == NULL){
    notcurses_stop(nc);
    return EXIT_FAILURE;
  }
  struct ncvisual* psu = ncvisual_from_file("counterforce-psu.jpg");
  // FIXME render them
  // FIXME loop, showing data
  notcurses_stop(nc);
  return EXIT_SUCCESS;
}
