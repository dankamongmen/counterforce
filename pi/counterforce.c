#include <notcurses/notcurses.h>

int main(void){
  struct notcurses* nc = notcurses_init(NULL, NULL);
  if(nc == NULL){
    exit(EXIT_FAILURE);
  }
  notcurses_stop(nc);
  return EXIT_SUCCESS;
}
