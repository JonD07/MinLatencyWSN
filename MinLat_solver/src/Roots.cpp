#include "../inc/Roots.h"


Roots::Roots() {
	root1 = 0;
	root2 = 0;
	imaginary = false;
}


void Roots::findRoots(double a, double b, double c) {
	double discriminant, realPart, imaginaryPart, x1, x2;
	if (isZero(a)) {
		fprintf(stderr, "ERROR : Roots::findRoots() Requested to find root of non-quadratic\n");
		exit(1);
	}
	else {
		discriminant = b*b - 4*a*c;
		if (discriminant > 0) {
			x1 = (-b + sqrt(discriminant)) / (2*a);
			x2 = (-b - sqrt(discriminant)) / (2*a);

			if(DEBUG_ROOTS) {
				printf("Roots are real and different.\n");
				printf("Root 1 = %f\n", x1);
				printf("Root 2 = %f\n", x2);
			}

			root1 = x1;
			root2 = x2;
			imaginary = false;
		}
		else if(discriminant == 0) {
			if(DEBUG_ROOTS)
				printf("Roots are real and same.\n");
			x1 = (-b + sqrt(discriminant)) / (2*a);
			if(DEBUG_ROOTS)
				printf("Root 1 = Root 2 = %f\n", x1);

			root1 = root2 = x1;
			imaginary = false;
		}
		else {
			realPart = -b/(2*a);
			imaginaryPart = sqrt(-discriminant)/(2*a);

			if(DEBUG_ROOTS) {
				printf("Roots are complex and different.\n");
				printf("Root 1 = %f + %fi\n", realPart, imaginaryPart);
				printf("Root 1 = %f - %fi\n", realPart, imaginaryPart);
			}

			root1 = realPart;
			root2 = imaginaryPart;
			imaginary = true;
		}
	}
}

