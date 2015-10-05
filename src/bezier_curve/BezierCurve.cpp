#include <stdio.h>
#include <stdlib.h>
#include<iostream>
#include "BezierCurve.hpp"

namespace trajectory_planner
{

    BezierCurve::BezierCurve()
    {

    }

    BezierCurve::~BezierCurve()
    {

    }
    
    void BezierCurve::check_bezier_curve(base::MatrixXd & pts)
    {
      int no_pts_input=int(pts.rows());
      int dim_input=int(pts.cols());
      
      switch (no_pts_input)
      {
	case 0:
	{
	  
	  std::cout<<"Points matrix empty. Needs atleast two points"<<std::endl;
	  break;
	}  
	case 1:
	{ 
	  std::cout<<"Points matrix has only one point. Needs atleast two points"<<std::endl;
	  break;
	}  
	case 2:
	{
	  std::cout<<"Adding intermediate points for linear interpolation"<<std::endl;
	  base::MatrixXd pts_temp=pts;
	  pts.resize(4,dim_input);
	  pts.row(0)=pts_temp.row(0);
	  pts.row(1)=0.3333*pts_temp.row(0)+0.667*pts_temp.row(1);
	  pts.row(2)=0.6667*pts_temp.row(0)+0.333*pts_temp.row(1);
	  pts.row(3)=pts_temp.row(1);
	  break;
	}  
	default:
	{
	  std::cout<<"Calculating bezier curve"<<std::endl;
	}
      }
    }

    base::MatrixXd BezierCurve::calc_bezier_curve(const base::MatrixXd & pts_in, const int& num_int)
    {
        base::MatrixXd pts=pts_in;
        this->check_bezier_curve(pts);
        Eigen::MatrixXd points=pts;
        no_pts=int(points.rows())-1;
        dim= int(points.cols());
        condition=0;
        num_intervals=num_int;

        Eigen::MatrixXd deBoor_points(no_pts+3,dim);
        deBoor_points=this->comp_deBoor_controlpts(points, condition);
        array_3D b_coeffs(boost::extents[dim][no_pts][4]);

        b_coeffs=this->comp_bezier_coeff(points, deBoor_points);

        Eigen::MatrixXd b_curve(no_pts*num_intervals+1,dim);
        b_curve=this->comp_bezier_curve(b_coeffs, num_intervals);


        base::MatrixXd b_curve1=b_curve;
        return b_curve1;

    }

    Eigen::MatrixXd BezierCurve::comp_deBoor_controlpts(const base::MatrixXd & pts, const int& condition)
        {
          Eigen::MatrixXd points=pts;

          int no_d_pts=no_pts+3;
          //std::cout<<no_d_pts<<std::endl;

          Eigen::MatrixXd A=Eigen::MatrixXd::Zero(1, 1);
          if (condition==0) //natural
          {
            if (no_pts>2)
            {
              A=Eigen::MatrixXd::Zero(no_pts-1, no_pts-1);
              A(0,0)=4;
              A(0,1)=1;
              A(no_pts-2, no_pts-3)=1;
              A(no_pts-2, no_pts-2)=4;

            }
            else
            {
              A(0,0)=4;
            }
          }
         else
         {
           A(0,0)=4;
           std::cout<<"Other conditions will be added later"<<std::endl;
         }

         for(int i=1;i<no_pts-2;i++)
         {
           A(i, i-1)=1;
           A(i, i)=4;
           A(i, i+1)=1;
         }
            //std::cout<<A<<std::endl;
             //Compute deBoor Control Points from A Matrix

        Eigen::MatrixXd d_pts=Eigen::MatrixXd::Zero(no_pts+3, dim);

        for(int col=0; col<dim;col++)
        {
          Eigen::MatrixXd x=Eigen::MatrixXd::Zero(no_pts-1, 1);

          //std::cout<<points(0,0)<<std::endl;

          if(condition==0)
          {
            x(no_pts-2, 0)=6*points(no_pts-1, col)-points(no_pts, col);
            x(0,0)=6*points(1,col)-points(0,col);
          }

          else
          {
            std::cout<<"Will be added later"<<std::endl;
          }
          if(no_pts>3)
          {
            for(int i=0; i<no_pts-3; i++)
            {
            //x.block(1, 0, no_pts-3+1, 1)=6*points.block(2, col, no_pts-2+1-2+1, 1);

              x(1+i,0)=6*points(2+i, col);

            }
          }

           Eigen::MatrixXd temp=A.fullPivHouseholderQr().solve(x);
           Eigen::MatrixXd temp2=temp.transpose();

           d_pts.block(2,col,no_pts+1-2, 1)=temp;

           d_pts(0,col)=points(0,col);
           d_pts(no_d_pts-1, col)=points(no_pts, col);

             }

          if(condition==0)
        {
          d_pts.row(1)=0.666*points.row(0)+0.333*d_pts.row(2);
          d_pts.row(no_pts+1)=0.333*d_pts.row(no_d_pts-3)+0.666*points.row(no_pts);
        }
        else
        {
          std::cout<<"Will be added later"<<std::endl;
        }

        /*std::ofstream bezier_file("deBoor.txt");
              if (bezier_file.is_open())
              {
                for(int i=0; i<no_pts+3; i++)
                {
              for(int j=0; j<dim; j++)
              {
            bezier_file<<d_pts(i,j)<< " ";
              }
              bezier_file<<std::endl;
            }
          }

          bezier_file.close();*/

          return d_pts;
        }//End of comp_deBoor_controlpts


    template <typename Derived>
            Eigen::MatrixXd BezierCurve::comp_cubic_spline_pt(const Eigen::EigenBase< Derived >& b_coeff, double t)
        {
          Eigen::MatrixXd spline_pt(1,dim);
          Eigen::MatrixXd b_coeff_set=b_coeff;

          for(int i=0; i<dim;i++)
          {
            spline_pt(0,i)=(pow(1-t,3)*b_coeff_set(i,0)+3*pow(1-t,2)*t*b_coeff_set(i,1)+3*(1-t)*pow(t,2)*b_coeff_set(i,2)+pow(t,3)*b_coeff_set(i,3));

          }
          return spline_pt;
        }


        Eigen::MatrixXd BezierCurve::comp_bezier_pt(array_3D b_coeffs, const int& b_index, double t)
        {
          Eigen::MatrixXd b_point(1,dim);
          Eigen::MatrixXd b_coeff_set(dim, 4);
          if(b_index<=0)
          {
            for(int i=0; i<dim; i++)
            {
              b_point(0,i)=b_coeffs[i][0][0];

            }
          }
          if(b_index>=no_pts)
          {
            for(int i=0; i<dim; i++)
            {
              b_point(0,i)=b_coeffs[i][no_pts-1][3];
            }
          }

          if(b_index>0 && b_index<no_pts)
          {
            if(t<0.0)
              t=0.0;
            if(t>1.0)
              t=1.0;

            for(int i=0; i<dim; i++)
            {
              b_coeff_set(i,0)=b_coeffs[i][b_index-1][0];
              b_coeff_set(i,1)=b_coeffs[i][b_index-1][1];
              b_coeff_set(i,2)=b_coeffs[i][b_index-1][2];
              b_coeff_set(i,3)=b_coeffs[i][b_index-1][3];
            }

            b_point=this->comp_cubic_spline_pt(b_coeff_set, t);
          }
           return b_point;

        }


        Eigen::MatrixXd BezierCurve::comp_bezier_curve(array_3D b_coeffs, const int& num_intervals)
        {

         double interval= 1.0/num_intervals;
         //int dim=b_coeffs.shape()[0];
         //int no_pts=b_coeffs.shape()[1];


         base::MatrixXd b_curve(no_pts*num_intervals+1,dim);

         for(int i=0; i<dim; i++)
         {
           b_curve(0,i)=b_coeffs[i][0][0];
           b_curve(0,i)=b_coeffs[i][0][0];
           b_curve(0,i)=b_coeffs[i][0][0];
         }

         for(int curr_b_pt=0;curr_b_pt<no_pts;curr_b_pt++)
         {
           Eigen::MatrixXd b_coeff_set(dim, 4);

           for(int i=0; i<dim; i++)
           {
             b_coeff_set(i,0)=b_coeffs[i][curr_b_pt][0];
             b_coeff_set(i,1)=b_coeffs[i][curr_b_pt][1];
             b_coeff_set(i,2)=b_coeffs[i][curr_b_pt][2];
             b_coeff_set(i,3)=b_coeffs[i][curr_b_pt][3];
           }


           Eigen::VectorXd t(num_intervals);
           t=Eigen::VectorXd::LinSpaced(num_intervals,interval,1);

           for(int iteration=0; iteration<num_intervals; iteration++)
           {
             b_curve.row(curr_b_pt*num_intervals+iteration+1)=this->comp_cubic_spline_pt(b_coeff_set,t(iteration));

           }


         }
          return b_curve;
            }

        //Computes the bezier coefficients given the trajectory points and the de Boor control points
        template <typename Derived>
        array_3D BezierCurve::comp_bezier_coeff(const Eigen::EigenBase< Derived > & pts, const Eigen::EigenBase< Derived >& d_points)
        {
            Eigen::MatrixXd points=pts;
            Eigen::MatrixXd d_pts=d_points;

            int no_d_pts=no_pts+3;

            array_3D b_coeffs(boost::extents[dim][no_pts][4]);

            for(int i=0; i<no_pts;i++)
            {
                int points_i=i+1;
                int d_pts_i=i+2;
                if(i==0)
                {
                    for(int axis_pos=0;axis_pos<dim; axis_pos++)
                    {
                        b_coeffs[axis_pos][i][0]=points(points_i-1, axis_pos);
                        b_coeffs[axis_pos][i][1]=d_pts(d_pts_i-1, axis_pos);
                        b_coeffs[axis_pos][i][2]=0.5*d_pts(d_pts_i-1, axis_pos)+0.5*d_pts(d_pts_i,axis_pos);
                        b_coeffs[axis_pos][i][3]=points(points_i, axis_pos);
                    }
                }
                if(i==no_pts-1)
                {
                    for(int axis_pos=0;axis_pos<dim; axis_pos++)
                    {
                        b_coeffs[axis_pos][i][0]=points(points_i-1, axis_pos);
                        b_coeffs[axis_pos][i][1]=0.5*d_pts(d_pts_i-1, axis_pos)+0.5*d_pts(d_pts_i,axis_pos);
                        b_coeffs[axis_pos][i][2]=d_pts(d_pts_i, axis_pos);
                        b_coeffs[axis_pos][i][3]=points(points_i, axis_pos);
                    }

                }

                if(i!=0&&i!=no_pts-1)
                {
                    for(int axis_pos=0;axis_pos<dim; axis_pos++)
                    {
                        b_coeffs[axis_pos][i][0]=points(points_i-1, axis_pos);
                        b_coeffs[axis_pos][i][1]=0.666*d_pts(d_pts_i-1, axis_pos)+0.333*d_pts(d_pts_i,axis_pos);
                        b_coeffs[axis_pos][i][2]=0.333*d_pts(d_pts_i-1, axis_pos)+0.666*d_pts(d_pts_i,axis_pos);
                        b_coeffs[axis_pos][i][3]=points(points_i, axis_pos);
                    }
                }

                /* std::ofstream bezier_file("b_coeffs.txt");
                if (bezier_file.is_open())
                {
                for(int i=0; i<no_pts; i++)
                {
                for(int j=0; j<dim; j++)
                {
                for(int k=0; k<4;k++)
                {bezier_file<<b_coeffs[j][i][k]<< " ";
                }bezier_file<<std::endl;
                }
                bezier_file<<std::endl;
                }
                }*/

            }//end of for loop

            return b_coeffs;
        } //end of comp_bezier_coeff


}
