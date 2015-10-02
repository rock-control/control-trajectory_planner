#include<iostream>
#include<fstream>
#include<stdio.h>

//#include <boost/test/unit_test.hpp>
#include <trajectory_planner/bezier_curve/BezierCurve.hpp>
//#include <base/logging/logging_printf_style.h>
#include <Eigen/Core>
#include <boost/concept_check.hpp>
#include <Eigen/Dense>

using namespace trajectory_planner;


int main(int argc, char* argv[])
{
    printf("\nUsage: ./test_interpolator input_file.txt outputfile.txt ...\n\n"
           "Returns the bezier curve given the input points\n"
           "The output file will be saved in the binary folder\n\n");

    if(argc != 3)
    {
        printf("Please give the input filename along with its path and a output file name.\n");
        return 1;
    }

    BezierCurve b;
    int num_intervals, ndata;
    ndata=0;
    float data1, data2, data3;
    Eigen::MatrixXd points1(100,3);

    std::cout<<argc<<std::endl;

    FILE * fp_in=fopen(argv[1], "r") ;        // Opening the input file to read

    if (fp_in == NULL)
    {
        std::cout << "The inputdata file cannot be opened to read" << std::endl;
        std::cout << std::endl;
    }
    do
    {
        fscanf(fp_in, "%f %f %f\n", &data1, &data2, &data3);
        //cout<<data1<<data2<<data3<<endl;
        points1(ndata,0)=data1;
        points1(ndata,1)=data2;
        points1(ndata,2)=data3;
        ndata = ndata + 1;
    }while (!feof(fp_in));

    Eigen::MatrixXd points(ndata,3);
    for(int i=0; i<ndata; i++)
    {
        points(i,0)=points1(i,0);
        points(i,1)=points1(i,1);
        points(i,2)=points1(i,2);
    }

    std::cout<<points<<std::endl;
    num_intervals=20;

    int no_pts=int(points.rows())-1;
    int dim= int(points.cols());

    Eigen::MatrixXd bezier_curve(no_pts*num_intervals+1,dim);
    bezier_curve=b.calc_bezier_curve(points,num_intervals);

    std::ofstream bezier_file(argv[2]);
    if (bezier_file.is_open())
    {
        for(int i=0; i<no_pts*num_intervals+1; i++)
        {
            for(int j=0; j<dim; j++)
            {
                bezier_file<<bezier_curve(i,j)<< " ";
            }
            bezier_file<<std::endl;
        }
    }

    bezier_file.close();

    return 0;

}
