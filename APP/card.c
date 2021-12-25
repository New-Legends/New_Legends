#include "card.h"

int a=0;

void card_task(void const * argument)
{
  card.init = card_init;
  card.init();
  while(1)
  {
    card.measure();
    card.set_mode();
    card.control();
    card.can_send();
  }
}
