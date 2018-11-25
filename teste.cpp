#include <stdio.h>

#include "polifitgsl.cpp"

#define NP 11
double x[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
double y[] = {98.02, 98.01, 98.01, 98.02, 97.98, 97.97, 97.96, 97.94, 97.96, 97.96, 97.97, 97.97, 97.94, 97.94, 97.94, 97.92, 97.96, 97.9, 97.85, 97.9};

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