#include <stdio.h>

#include "polifitgsl.cpp"

#define NP 11
double x[] = {0,1,2,3,4,5,6,7};
double y[] = {85,102,119,135,151,168,184,201};

#define DEGREE 3
double coeff[DEGREE];

int main()
{
  int i;

  polynomialfit(NP, DEGREE, x, y, coeff);
  for(i=0; i < DEGREE; i++) {
    printf("%lf\n", coeff[i]);
  }
  return 0;
}