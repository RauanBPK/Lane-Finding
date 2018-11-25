
#include <gsl/gsl_multifit.h>
#include <stdbool.h>
#include <math.h>

bool findQuadCoefficients(double timeArray[], double valueArray[], double *coef, double &critPoint, int PointsNum){
    const double S00=PointsNum;//points number
    double S40=0, S10=0, S20=0, S30=0, S01=0, S11=0, S21 = 0;
//    const double MINvalue = valueArray[0];
//    const double MINtime = timeArray[0];
    for (int i=0; i<PointsNum; i++ ){
        double value = valueArray[i]; //  - MINvalue); //normalizing
//      cout << "i=" << i << " index=" << index << " value=" << value << endl;

        int index = timeArray[i]; //  - MINtime;
        int index2 = index * index;
        int index3 = index2 * index;
        int index4 = index3 * index;

        S40+= index4;
        S30+= index3; 
        S20+= index2;
        S10+= index;

        S01 += value;
        S11 += value*index;
        S21 += value*index2;
    }

    double S20squared = S20*S20;

    //minors M_ij=M_ji
    double M11 = S20*S00 - S10*S10;
    double M21 = S30*S00 - S20*S10;
    double M22 = S40*S00 - S20squared;
    double M31 = S30*S10 - S20squared;

    double M32 = S40*S10 - S20*S30;
//  double M33 = S40*S20 - pow(S30,2);

    double discriminant = S40*M11 - S30*M21 + S20*M31;
//    printf("discriminant :%lf\n", discriminant);
    if (abs(discriminant) < .00000000001) return  false;

    double Da = S21*M11
               -S11*M21
               +S01*M31;
    coef[2] = Da/discriminant;
//  cout << "discriminant=" << discriminant;
//  cout << " Da=" << Da;

    double Db = -S21*M21
                +S11*M22
                -S01*M32;
    coef[1] = Db/discriminant;
//  cout << " Db=" << Db << endl;

    double Dc =   S40*(S20*S01 - S10*S11) 
                - S30*(S30*S01 - S10*S21) 
                + S20*(S30*S11 - S20*S21);
    coef[0] = Dc/discriminant;
//    printf("c=%lf\n", c);

    critPoint = -Db/(2*Da); // + MINtime; //-b/(2*a)= -Db/discriminant / (2*(Da/discriminant)) = -Db/(2*Da);

    return true;
}

bool polynomialfit(int obs, int degree, 
           double *dx, double *dy, double *store) /* n, p */
{
  gsl_multifit_linear_workspace *ws;
  gsl_matrix *cov, *X;
  gsl_vector *y, *c;
  double chisq;

  int i, j;

  X = gsl_matrix_alloc(obs, degree);
  y = gsl_vector_alloc(obs);
  c = gsl_vector_alloc(degree);
  cov = gsl_matrix_alloc(degree, degree);

  for(i=0; i < obs; i++) {
    for(j=0; j < degree; j++) {
      gsl_matrix_set(X, i, j, pow(dx[i], j));
    }
    gsl_vector_set(y, i, dy[i]);
  }

  ws = gsl_multifit_linear_alloc(obs, degree);
  gsl_multifit_linear(X, y, c, cov, &chisq, ws);

  /* store result ... */
  for(i=0; i < degree; i++)
  {
    store[i] = gsl_vector_get(c, i);
  }

  gsl_multifit_linear_free(ws);
  gsl_matrix_free(X);
  gsl_matrix_free(cov);
  gsl_vector_free(y);
  gsl_vector_free(c);
  return true; /* we do not "analyse" the result (cov matrix mainly)
          to know if the fit is "good" */
}
