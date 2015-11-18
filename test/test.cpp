#include<iostream>
#include<fstream>
#include<stdio.h>

//#include <boost/test/unit_test.hpp>
#include "trajectory_planner/bezier_curve/BezierCurve.hpp"
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
        fscanf(fp_in, "%f \n", &data1);
        //cout<<data1<<data2<<data3<<endl;
        points1(ndata,0)=data1;
        
        ndata = ndata + 1;
    }while (!feof(fp_in));

    base::MatrixXd points(ndata,1);
    for(int i=0; i<ndata; i++)
    {
        points(i,0)=points1(i,0);
       
    }

    std::cout<<points<<std::endl;
    
    std::cout<<"Enter no of intervals= "<<std::endl;
    std::cin>>num_intervals;

    double max_value=points.row(0).maxCoeff();
    std::cout<<"max value= "<<max_value<<std::endl;
    
   
    check_bezier_curve(points);
    int no_pts=int(points.rows());
    int dim= int(points.cols());
    base::MatrixXd bezier_curve((no_pts-1)*num_intervals+1,dim);
   
    BezierCurve b(no_pts, dim);
    base ::VectorXd num_intervals_vec(no_pts-1);
    num_intervals_vec.fill(num_intervals);
    int cond;
    std::cout<<"Enter 0 for end accelerations to be zero and 1 for velocities to be zero "<<std::endl;
    std::cin>>cond;
    b.read_input_data(points, num_intervals_vec,cond);
    bezier_curve=b.calc_bezier_curve(points,num_intervals);
       
    base::MatrixXd b_curve_deriv(int(num_intervals_vec.sum())+1, dim);
    array_3D bcoeffs_deriv=b.comp_bezier_coeff_deriv(b.m_bCoeffs);
    b.comp_bezier_curve(bcoeffs_deriv, num_intervals_vec,b_curve_deriv, 0);
    
    base::MatrixXd b_curve_deriv2(int(num_intervals_vec.sum())+1, dim);
    array_3D bcoeffs_deriv2=b.comp_bezier_coeff_deriv(bcoeffs_deriv);
    b.comp_bezier_curve(bcoeffs_deriv2, num_intervals_vec,b_curve_deriv2, 0);

    
    base::MatrixXd b_curve_deriv3(int(num_intervals_vec.sum())+1, dim);
    b.comp_bezier_curve_deriv(bezier_curve,num_intervals, b_curve_deriv3);
    
    base::MatrixXd b_curve_deriv4(int(num_intervals_vec.sum())+1, dim);
    b.comp_bezier_curve_deriv(b_curve_deriv3,num_intervals, b_curve_deriv4);
    
    std::ofstream bezier_file(argv[2]);
    if (bezier_file.is_open())
    {
        for(int i=0; i<(no_pts-1)*num_intervals+1; i++)
        {
            for(int j=0; j<dim; j++)
            {
                bezier_file<<bezier_curve(i,j)<< " ";
            }
            bezier_file<<std::endl;
        }
    }
    bezier_file.close();
   
    std::ofstream bezier_file1("output2.txt");
    if (bezier_file1.is_open())
    {
      for(int i=0; i<(no_pts-1)*num_intervals+1; i++)
      {
	for(int j=0; j<dim; j++)
	{
	  bezier_file1<<b_curve_deriv(i,j)<< " ";
	}
	bezier_file1<<std::endl;
      }
    }
    bezier_file1.close(); 
    
    std::ofstream bezier_file2("output3.txt");
    if (bezier_file2.is_open())
    {
      for(int i=0; i<(no_pts-1)*num_intervals+1; i++)
      {
	for(int j=0; j<dim; j++)
	{
	  bezier_file2<<b_curve_deriv2(i,j)<< " ";
	}
	bezier_file2<<std::endl;
      }
    }
    bezier_file2.close(); 
    
    std::ofstream bezier_file3("output4.txt");
    if (bezier_file3.is_open())
    {
      for(int i=0; i<(no_pts-1)*num_intervals+1; i++)
      {
	for(int j=0; j<dim; j++)
	{
	  bezier_file3<<b_curve_deriv3(i,j)<< " ";
	}
	bezier_file3<<std::endl;
      }
    }
    bezier_file3.close(); 
    
    std::ofstream bezier_file4("output5.txt");
    if (bezier_file4.is_open())
    {
      for(int i=0; i<(no_pts-1)*num_intervals+1; i++)
      {
	for(int j=0; j<dim; j++)
	{
	  bezier_file4<<b_curve_deriv4(i,j)<< " ";
	}
	bezier_file4<<std::endl;
      }
    }
    bezier_file4.close(); 
    
   std::ofstream bezier_coeff_file("bcoeffs_deriv.txt");
   if (bezier_coeff_file.is_open())
   {
     for(int i=0; i<(no_pts-1); i++)
     {
       for(int j=0; j<2; j++)
       {
	 bezier_coeff_file<<bcoeffs_deriv2[0][i][j]<< " ";
       }
       bezier_coeff_file<<std::endl;
     }
     std::cout<<" "<<std::endl;
     for(int i=0; i<(no_pts-1); i++)
     {
       for(int j=0; j<3; j++)
       {
	 bezier_coeff_file<<bcoeffs_deriv[0][i][j]<< " ";
       }
       bezier_coeff_file<<std::endl;
     }
     std::cout<<" "<<std::endl;
     for(int i=0; i<(no_pts-1); i++)
     {
       for(int j=0; j<4; j++)
       {
	 bezier_coeff_file<<b.m_bCoeffs[0][i][j]<< " ";
       }
       bezier_coeff_file<<std::endl;
     }
   } 
   
   bezier_coeff_file.close();
   
    return 0;

}
