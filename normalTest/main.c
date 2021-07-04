#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>

#include"data.h"
#include"normaldistribution.h"

int main(void)
{
    puts("-----------------------");
    puts("| codedrome.com       |");
    puts("| Normal Distribution |");
    puts("-----------------------\n");

    int data[138];

    populatedata(data);

    probDist* pd = normalDistributionCreate();

    normalDistributionCalculate(data, 138, pd);

    normalDistributionPrint(pd);

    normalDistributionFree(pd);

    return EXIT_SUCCESS;
}
