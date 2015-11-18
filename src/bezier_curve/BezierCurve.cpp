#include <stdio.h>
#include <stdlib.h>
#include<iostream>
#include "BezierCurve.hpp"



namespace trajectory_planner
{
unsigned factorial(unsigned n)
{
  if(n==0)
    return 1;
  if(n==1)
    return n;
  else
    return n*factorial(n-1);
}

  
  
  
  void check_bezier_curve(base::MatrixXd & pts)
  {
    
    std::cout<<"check_bezier_curve"<<std::endl;
    
    
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
	pts.row(1)=0.67*pts_temp.row(0)+0.33*pts_temp.row(1);
	pts.row(2)=0.33*pts_temp.row(0)+0.67*pts_temp.row(1);
	pts.row(3)=pts_temp.row(1);
	break;
      }  
      default:
      {
	std::cout<<"Calculating bezier curve"<<std::endl;
      }
    }
  }
  
  BezierCurve::BezierCurve():m_const(0),m_condition(0),
  m_bCoeffs(boost::extents[3][10][4])
  {
       
    std::cout<<"construct"<<std::endl;
    for(int j=0; j<3; j++)
    {
      std::cout<<m_bCoeffs.shape()[j]<<std::endl;
    }
    
  }
  
  BezierCurve::BezierCurve(int no_points, int dimension):m_const(1),m_condition(0),
  m_noPts(no_points-1), m_dim(dimension), 
  m_bCoeffs(boost::extents[dimension][no_points-1][4]),
  m_trajPoints(no_points, dimension),
  m_deBoorPoints(no_points+2, dimension)
  {

    std::cout<<"construct"<<std::endl;
   
    for(int j=0; j<3; j++)
    {
      std::cout<<m_bCoeffs.shape()[j]<<std::endl;
    }
  }
  
  BezierCurve::BezierCurve(int no_points, int dimension, int condition):m_const(1),m_condition(condition),
  m_noPts(no_points-1), m_dim(dimension), 
  m_bCoeffs(boost::extents[dimension][no_points-1][4]),
  m_trajPoints(no_points, dimension),
  m_deBoorPoints(no_points+2, dimension), 
  m_bCurve((no_points-1)*100+1,m_dim)
  {
    
    std::cout<<"construct"<<std::endl;
    
    for(int j=0; j<3; j++)
    {
      std::cout<<m_bCoeffs.shape()[j]<<std::endl;
    }
  }
  
  BezierCurve::BezierCurve(int no_points, int dimension, int condition, int order):m_const(1),m_condition(condition),
  m_noPts(no_points-1), m_dim(dimension), 
  m_bCoeffs(boost::extents[dimension][no_points-1][order]),
  m_trajPoints(no_points, dimension),
  m_deBoorPoints(no_points+2, dimension),
  m_bCurve((no_points-1)*100+1,m_dim)
  {
    
    std::cout<<"construct"<<std::endl;
    
    for(int j=0; j<3; j++)
    {
      std::cout<<m_bCoeffs.shape()[j]<<std::endl;
    }
  }


    BezierCurve::~BezierCurve()
    {
      
      std::cout<<"dest"<<std::endl;
      for(int j=0; j<3; j++)
      {
	std::cout<<m_bCoeffs.shape()[j]<<std::endl;
      }

    }
    void BezierCurve::initialise_data(int noTrajPts, int dim, double period, double order, double end_condition)
    {
      m_numIntervals=1/period;
      m_condition=end_condition;
      m_dim=dim;      
      m_noPts=noTrajPts-1;
      
      m_deBoorPoints.resize(m_noPts+3, m_dim);
      m_bCoeffs.resize(boost::extents[m_dim][m_noPts][order]);
     // m_bCurve.resize(m_noPts*m_numIntervals+1,m_dim);
      
      std::cout<<"initialise bCurve rows= "<<m_bCurve.rows()<<std::endl;
      
    }
    
    void BezierCurve::read_input_data(const base::MatrixXd & pts_in, const base::VectorXd num_int, int condition)
    {
      m_trajPoints=pts_in;
      m_numIntervals_vec=num_int;
      m_condition=condition;
      if(m_const==0)
      {
	m_noPts=m_trajPoints.rows()-1;
	m_dim=m_trajPoints.cols();
	m_deBoorPoints.resize(m_noPts+3, m_dim);
	m_bCoeffs.resize(boost::extents[m_dim][m_noPts][4]);
	
      }
    }
    
    
    void BezierCurve::read_input_data(const base::MatrixXd & pts_in, int num_int, int condition)
    {
      m_trajPoints=pts_in;
      m_numIntervals=num_int;
      m_condition=condition;
      if(m_const==0)
      {
	m_noPts=m_trajPoints.rows()-1;
	m_dim=m_trajPoints.cols();
	m_deBoorPoints.resize(m_noPts+3, m_dim);
	m_bCoeffs.resize(boost::extents[m_dim][m_noPts][4]);
      }
    }
    
    base::MatrixXd BezierCurve::calc_bezier_curve(const base::MatrixXd & pts_in, const int& num_int)
    {
      std::cout<<"size"<<std::endl;
      for(int j=0; j<3; j++)
      {
	std::cout<<m_bCoeffs.shape()[j]<<std::endl;
	
      }
      m_trajPoints=pts_in;
      m_numIntervals=num_int;
      if(m_const==0)
      {
	m_noPts=m_trajPoints.rows()-1;
	m_dim=m_trajPoints.cols();
	m_deBoorPoints.resize(m_noPts+3, m_dim);
	m_bCoeffs.resize(boost::extents[m_dim][m_noPts][4]);
      }
        	
      base::MatrixXd b_curve1(m_noPts*m_numIntervals+1,m_dim);	
      this->comp_deBoor_controlpts();
      this->comp_bezier_coeff();
      std::cout<<"comp bezier coeff"<<std::endl;
      b_curve1=this->comp_bezier_curve(m_numIntervals);
      std::cout<<"comp bezier curve"<<std::endl;
      m_bCurve=b_curve1;
      return m_bCurve;

    }

    void BezierCurve::comp_deBoor_controlpts()
        {
          //Eigen::MatrixXd points=pts;

          int no_d_pts=m_noPts+3;
          //std::cout<<no_d_pts<<std::endl;

          Eigen::MatrixXd A=Eigen::MatrixXd::Zero(1, 1);
	  switch(m_condition)
	  {
	    case 0: //end acceleration=0
            {
	      std::cout<<"End condition= "<<m_condition<<std::endl;
	      if (m_noPts>2)
              {
		A=Eigen::MatrixXd::Zero(m_noPts-1, m_noPts-1);
                A(0,0)=4;
                A(0,1)=1;
                A(m_noPts-2, m_noPts-3)=1;
                A(m_noPts-2, m_noPts-2)=4;
                break;
              }
              else
              {
                A(0,0)=4;
	      }
            }
	    case 1: //end velocities=0
	    {
	      if (m_noPts>2)
	      {
		A=Eigen::MatrixXd::Zero(m_noPts-1, m_noPts-1);
		A(0,0)=7/2;
		A(0,1)=1;
		A(m_noPts-2, m_noPts-3)=1;
		A(m_noPts-2, m_noPts-2)=7/2;
		break;
		
	      }
	      else
	      {
		A(0,0)=4;
	      }
	    }
	    
	    default:
	    {
	      std::cout<<"No condition provided"<<std::endl;
	    }
	  }
           
         for(int i=1;i<m_noPts-2;i++)
         {
           A(i, i-1)=1;
           A(i, i)=4;
           A(i, i+1)=1;
         }
            //std::cout<<A<<std::endl;
             //Compute deBoor Control Points from A Matrix

        m_deBoorPoints=Eigen::MatrixXd::Zero(m_noPts+3, m_dim);

        for(int col=0; col<m_dim;col++)
        {
          Eigen::MatrixXd x=Eigen::MatrixXd::Zero(m_noPts-1, 1);

          //std::cout<<m_trajPoints(0,0)<<std::endl;
          switch(m_condition)
	  {
	    case 0:
            {
              x(m_noPts-2, 0)=6*m_trajPoints(m_noPts-1, col)-m_trajPoints(m_noPts, col);
              x(0,0)=6*m_trajPoints(1,col)-m_trajPoints(0,col);
	      break;
            }

	    case 1:
            {
	      x(m_noPts-2, 0)=6*m_trajPoints(m_noPts-1, col)-m_trajPoints(m_noPts, col);
	      x(0,0)=6*m_trajPoints(1,col)-m_trajPoints(0,col);
	      break;
            }
	    default:
	    {
	    }
	  }
          if(m_noPts>3)
          {
            for(int i=0; i<m_noPts-3; i++)
            {
            //x.block(1, 0, m_noPts-3+1, 1)=6*m_trajPoints.block(2, col, m_noPts-2+1-2+1, 1);

              x(1+i,0)=6*m_trajPoints(2+i, col);

            }
          }

           Eigen::MatrixXd temp=A.fullPivHouseholderQr().solve(x);
           Eigen::MatrixXd temp2=temp.transpose();

           m_deBoorPoints.block(2,col,m_noPts+1-2, 1)=temp;

           m_deBoorPoints(0,col)=m_trajPoints(0,col);
           m_deBoorPoints(no_d_pts-1, col)=m_trajPoints(m_noPts, col);

        }
        
        switch(m_condition)
	{
	  case 0:
	  {
	    m_deBoorPoints.row(1)=0.666*m_trajPoints.row(0)+0.333*m_deBoorPoints.row(2);
            m_deBoorPoints.row(m_noPts+1)=0.333*m_deBoorPoints.row(no_d_pts-3)+0.666*m_trajPoints.row(m_noPts);
	    break;
          }
	  case 1:
	  {
	    m_deBoorPoints.row(1)=m_trajPoints.row(0);
	    m_deBoorPoints.row(m_noPts+1)=m_trajPoints.row(m_noPts);
	    break;
	  }
	  default:
          {
          std::cout<<"Condition not added"<<std::endl;
          }
	}

        /*std::ofstream bezier_file("deBoor.txt");
              if (bezier_file.is_open())
              {
                for(int i=0; i<m_noPts+3; i++)
                {
              for(int j=0; j<m_dim; j++)
              {
            bezier_file<<m_deBoorPoints(i,j)<< " ";
              }
              bezier_file<<std::endl;
            }
          }

          bezier_file.close();*/
	//std::cout<<"deboor="<<m_deBoorPoints<<std::endl;
      }//End of comp_deBoor_controlpts


    
        base::MatrixXd BezierCurve::comp_cubic_spline_pt(const base::MatrixXd& b_coeff_set, double t)
        {
          base::MatrixXd spline_pt(1,m_dim);
          

          for(int i=0; i<m_dim;i++)
          {
            spline_pt(0,i)=(pow(1-t,3)*b_coeff_set(i,0)+3*pow(1-t,2)*t*b_coeff_set(i,1)+3*(1-t)*pow(t,2)*b_coeff_set(i,2)+pow(t,3)*b_coeff_set(i,3));

          }
          return spline_pt;
        }
        
        base::MatrixXd BezierCurve::comp_spline_pt(const base::MatrixXd& b_coeff_set, double t)
	{
	  base::MatrixXd spline_pt(1,m_dim);
	  spline_pt.row(0).fill(0);
	  unsigned degree=unsigned(b_coeff_set.cols())-1;
	  double sum=0;
	  
	  for(int i=0; i<m_dim;i++)
	  {
	    sum=0;
	    for(unsigned j=0;j<degree+1; j++)
	    {
	      
	      sum=sum+factorial(degree)/(factorial(j)*factorial(degree-j))*pow(1-t,degree-j)*pow(t,j)*b_coeff_set(i,j);
	    }
	    spline_pt(0,i)=sum;
	  }
	  //std::cout<<"spline_pt= "<<spline_pt<<std::endl;
	  return spline_pt;
	}
   

        base::MatrixXd BezierCurve::comp_bezier_pt(const int& b_index, double t)
        {
          base::MatrixXd b_point(1,m_dim);
          base::MatrixXd b_coeff_set(m_dim, 4);
          if(b_index<=0)
          {
            for(int i=0; i<m_dim; i++)
            {
              b_point(0,i)=m_bCoeffs[i][0][0];

            }
          }
          if(b_index>=m_noPts)
          {
            for(int i=0; i<m_dim; i++)
            {
              b_point(0,i)=m_bCoeffs[i][m_noPts-1][3];
            }
          }

          if(b_index>0 && b_index<m_noPts)
          {
            if(t<0.0)
              t=0.0;
            if(t>1.0)
              t=1.0;

            for(int i=0; i<m_dim; i++)
            {
              b_coeff_set(i,0)=m_bCoeffs[i][b_index-1][0];
              b_coeff_set(i,1)=m_bCoeffs[i][b_index-1][1];
              b_coeff_set(i,2)=m_bCoeffs[i][b_index-1][2];
              b_coeff_set(i,3)=m_bCoeffs[i][b_index-1][3];
            }

            b_point=this->comp_cubic_spline_pt(b_coeff_set, t);
          }
           return b_point;

        }


        base::MatrixXd BezierCurve::comp_bezier_curve(int num_intervals)
        {

         double interval=1/num_intervals;
         //int m_dim=m_bCoeffs.shape()[0];
         //int m_noPts=m_bCoeffs.shape()[1];
        
	// std::cout<<"b-curve num_intervals= "<<num_intervals<<std::endl;	

	 base::MatrixXd b_curve(m_noPts*num_intervals+1,m_dim);
	// std::cout<<"b-curve rows= "<<m_bCurve.rows()<<std::endl;	

         for(int i=0; i<m_dim; i++)
         {
           b_curve(0,i)=m_bCoeffs[i][0][0];
         }

         for(int curr_b_pt=0;curr_b_pt<m_noPts;curr_b_pt++)
         {
	  base::MatrixXd b_coeff_set(m_dim, 4);

           for(int i=0; i<m_dim; i++)
           {
	     for(int j=0;j<m_bCoeffs.shape()[2]; j++)
	     {
	       b_coeff_set(i,j)=m_bCoeffs[i][curr_b_pt][j];
	     }
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
        
       void BezierCurve::comp_bezier_curve()
	{
	  
	  double interval=1/m_numIntervals;
	  //int m_dim=m_bCoeffs.shape()[0];
	  //int m_noPts=m_bCoeffs.shape()[1];
	  
	//  std::cout<<"bCurve num_intervals= "<<m_numIntervals<<std::endl;	
	  
	  m_bCurve.resize(m_noPts*m_numIntervals+1,m_dim);
	//  std::cout<<"bCurve rows= "<<m_bCurve.rows()<<std::endl;	
	  
	  for(int i=0; i<m_dim; i++)
	  {
	    m_bCurve(0,i)=m_bCoeffs[i][0][0];
	  }
	  
	  for(int curr_b_pt=0;curr_b_pt<m_noPts;curr_b_pt++)
	  {
	    base::MatrixXd b_coeff_set(m_dim, 4);
	    
	    for(int i=0; i<m_dim; i++)
	    {
	      for(int j=0;j<m_bCoeffs.shape()[2]; j++)
	      {
		b_coeff_set(i,j)=m_bCoeffs[i][curr_b_pt][j];
	      }
	    }
	    
	    Eigen::VectorXd t(m_numIntervals);
	    t=Eigen::VectorXd::LinSpaced(m_numIntervals,interval,1);
	    
	    for(int iteration=0; iteration<m_numIntervals; iteration++)
	    {
	      m_bCurve.row(curr_b_pt*m_numIntervals+iteration+1)=this->comp_cubic_spline_pt(b_coeff_set,t(iteration));
	    }
	    
	  }

	  
	}

        //Computes the bezier coefficients given the trajectory points and the de Boor control points
       void BezierCurve::comp_bezier_coeff()
        {
           // Eigen::MatrixXd points=pts;
            //Eigen::MatrixXd d_pts=d_points;

           // int no_d_pts=m_noPts+3;

            //array_3D m_bCoeffs(boost::extents[m_dim][m_noPts][4]);
	    
	   
	    std::cout<<"b-coeffs"<<std::endl;	
	    for(int j=0; j<3; j++)
	    {
	      std::cout<<m_bCoeffs.shape()[j]<<std::endl;
	    }
            for(int i=0; i<m_noPts;i++)
            {
                int points_i=i+1;
                int d_pts_i=i+2;
                if(i==0)
                {
                    for(int axis_pos=0;axis_pos<m_dim; axis_pos++)
                    {
                        m_bCoeffs[axis_pos][i][0]=m_trajPoints(points_i-1, axis_pos);
                        m_bCoeffs[axis_pos][i][1]=m_deBoorPoints(d_pts_i-1, axis_pos);
                        m_bCoeffs[axis_pos][i][2]=0.5*m_deBoorPoints(d_pts_i-1, axis_pos)+0.5*m_deBoorPoints(d_pts_i,axis_pos);
                        m_bCoeffs[axis_pos][i][3]=m_trajPoints(points_i, axis_pos);
                    }
                }
                
                if(i==m_noPts-1)
                {
                    for(int axis_pos=0;axis_pos<m_dim; axis_pos++)
                    {
                        m_bCoeffs[axis_pos][i][0]=m_trajPoints(points_i-1, axis_pos);
                        m_bCoeffs[axis_pos][i][1]=0.5*m_deBoorPoints(d_pts_i-1, axis_pos)+0.5*m_deBoorPoints(d_pts_i,axis_pos);
                        m_bCoeffs[axis_pos][i][2]=m_deBoorPoints(d_pts_i, axis_pos);
                        m_bCoeffs[axis_pos][i][3]=m_trajPoints(points_i, axis_pos);
                    }

                }

                if(i!=0&&i!=m_noPts-1)
                {
                    for(int axis_pos=0;axis_pos<m_dim; axis_pos++)
                    {
                        m_bCoeffs[axis_pos][i][0]=m_trajPoints(points_i-1, axis_pos);
                        m_bCoeffs[axis_pos][i][1]=0.666*m_deBoorPoints(d_pts_i-1, axis_pos)+0.333*m_deBoorPoints(d_pts_i,axis_pos);
                        m_bCoeffs[axis_pos][i][2]=0.333*m_deBoorPoints(d_pts_i-1, axis_pos)+0.666*m_deBoorPoints(d_pts_i,axis_pos);
                        m_bCoeffs[axis_pos][i][3]=m_trajPoints(points_i, axis_pos);
                    }
                }
               // std::cout<<"end coeffs "<<i<<std::endl;
		
                /* std::ofstream bezier_file("m_bCoeffs.txt");
                if (bezier_file.is_open())
                {
                for(int i=0; i<m_noPts; i++)
                {
                for(int j=0; j<m_dim; j++)
                {
                for(int k=0; k<4;k++)
                {bezier_file<<m_bCoeffs[j][i][k]<< " ";
                }bezier_file<<std::endl;
                }
                bezier_file<<std::endl;
                }
                }*/
           // std::cout<<"bcoeffs loop no ="<<i<<std::endl;
            }//end of for loop
           // std::cout<<"end coeffs "<<std::endl;
	    for(int j=0; j<3; j++)
	    {
	      std::cout<<m_bCoeffs.shape()[j]<<std::endl;
	    }
           // return m_bCoeffs;
        } //end of comp_bezier_coeff
        
        
        base::MatrixXd BezierCurve::comp_bezier_curve(base::VectorXd& num_intervals)
	{
	  
	  
	  //int m_dim=m_bCoeffs.shape()[0];
	  //int m_noPts=m_bCoeffs.shape()[1];
	  
	  
	  base::MatrixXd b_curve(int(num_intervals.sum())+1,m_dim);
	  b_curve.fill(0);
	  for(int i=0; i<m_dim; i++)
	  {
	    b_curve(0,i)=m_bCoeffs[i][0][0];
	  }
	  
	  int ctr=0;
	  
	  
	  Eigen::MatrixXd b_coeff_set(m_dim, m_bCoeffs.shape()[2]);
	  for(int curr_b_pt=0;curr_b_pt<m_noPts;curr_b_pt++)
	  {
	   
	    
	    for(int i=0; i<m_dim; i++)
	    {
	      for(int j=0;j<m_bCoeffs.shape()[2]; j++)
	      {
		b_coeff_set(i,j)=m_bCoeffs[i][curr_b_pt][j];
	      }
	    }
	   // if(curr_b_pt==0)
	  //  std::cout<<"b coeffset= "<<b_coeff_set<<std::endl;
	    
	    double interval= 1.0/num_intervals(curr_b_pt);
	    Eigen::VectorXd t(num_intervals(curr_b_pt));
	    t=Eigen::VectorXd::LinSpaced(num_intervals(curr_b_pt),interval,1);
	   
	    for(int iteration=0; iteration<int(num_intervals(curr_b_pt)); iteration++)
	    {
	      // std::cout<<"iteration= "<<iteration<<std::endl;
	      b_curve.row(ctr+1)=this->comp_spline_pt(b_coeff_set,t(iteration));
	      // std::cout<<b_curve.row(num_intervals(curr_b_pt)+iteration+1)<<std::endl;
	      ctr=ctr+1;
	    }
	  }
	  return b_curve;
	}
	
	void BezierCurve::comp_bezier_curve(const array_3D& bCoeffs, base::VectorXd& num_intervals,base::MatrixXd& b_curve, int pt)
	{
	 
	  int order=bCoeffs.shape()[2];
	  int noPts=bCoeffs.shape()[1];
	  b_curve.fill(0);
	  
	  if(pt==0)	  
	  {
	    for(int i=0; i<m_dim; i++)
	    {
	      b_curve(0,i)=bCoeffs[i][0][0];
	    }
	  }
	  int ctr=0;
	  for(int curr_b_pt=pt;curr_b_pt<pt+noPts;curr_b_pt++)
	  {
	   
	    Eigen::MatrixXd b_coeff_set(m_dim, order);
	  //  std::cout<<curr_b_pt<<std::endl;
	    for(int i=0; i<m_dim; i++)
	    {
	      for(int j=0;j<order; j++)
	      {
		b_coeff_set(i,j)=bCoeffs[i][curr_b_pt][j];
	      }
	    }
	   
	    double interval= 1.0/num_intervals(curr_b_pt);
	    std::cout<<interval<<std::endl;
	    Eigen::VectorXd t(num_intervals(curr_b_pt));
	    t=Eigen::VectorXd::LinSpaced(num_intervals(curr_b_pt),interval,1);
	    std::cout<<t<<std::endl;
	    
	    for(int iteration=0; iteration<int(num_intervals(curr_b_pt)); iteration++)
	    {
	     // std::cout<<"iteration= "<<iteration<<std::endl;
	      b_curve.row(ctr+1)=this->comp_spline_pt(b_coeff_set,t(iteration));
	     // std::cout<<b_curve.row(num_intervals(curr_b_pt)+iteration+1)<<std::endl;
	      ctr=ctr+1;
	    }
	    //std::cout<<b_curve<<std::endl;
	  }
	  
	 
	 
	}
	
	array_3D BezierCurve::comp_bezier_coeff_deriv(array_3D bCoeffs)
	{
	  int degree=bCoeffs.shape()[2]-1;
	  int dim=bCoeffs.shape()[0];
	  int noPts=bCoeffs.shape()[1];
	  
	  if (degree==0)
	    return bCoeffs;
	  else
	  {
	    array_3D bCoeffs_deriv(boost::extents[dim][noPts][degree]);
	    
	    for(int i=0;i<dim; i++)
	    {
	      for(int j=0;j<noPts; j++)
	      {
		for(int k=0;k<degree;k++)
		{
		  //std::cout<<bCoeffs[i][j][k]<<" ";
		  bCoeffs_deriv[i][j][k]=(degree)*(bCoeffs[i][j][k+1]-bCoeffs[i][j][k]);
		}
	//	std::cout<<bCoeffs[i][j][degree]<<" ";
		//std::cout<<""<<std::endl;
	      }  
	      //std::cout<<""<<std::endl;
	      //std::cout<<""<<std::endl;
	    }
	    return bCoeffs_deriv;
	  }
	  
	}
	
	array_3D BezierCurve::comp_bezier_coeff_start(array_3D bCoeffs)
	{
	  int dim=bCoeffs.shape()[0];
	  array_3D bCoeffs_start(boost::extents[dim][1][6]);
	  
	  for(int i=0;i<dim;i++)
	  {
	    bCoeffs_start[i][0][0]=bCoeffs[i][0][0];
	    bCoeffs_start[i][0][1]=bCoeffs[i][0][0];
	    bCoeffs_start[i][0][2]=bCoeffs[i][0][0];
	    bCoeffs_start[i][0][3]=(bCoeffs[i][0][3]+6*bCoeffs[i][0][2]+3*bCoeffs[i][0][1])/10;
	    bCoeffs_start[i][0][4]=(2*bCoeffs[i][0][3]+3*bCoeffs[i][0][2])/5;
	    bCoeffs_start[i][0][5]=bCoeffs[i][0][3];
	  }
	  
	  return bCoeffs_start;
	}
	
	
	array_3D BezierCurve::comp_bezier_coeff_end(array_3D bCoeffs)
	{
	  int dim=bCoeffs.shape()[0];
	  int noPts=bCoeffs.shape()[1];
	  array_3D bCoeffs_end(boost::extents[dim][1][6]);
	  
	  for(int i=0;i<dim;i++)
	  {
	    bCoeffs_end[i][0][0]=bCoeffs[i][noPts-1][0];
	    bCoeffs_end[i][0][1]=(3*bCoeffs[i][noPts-1][1]+2*bCoeffs[i][noPts-1][0])/5;
	    bCoeffs_end[i][0][2]=(bCoeffs[i][noPts-1][0]+6*bCoeffs[i][noPts-1][1]+3*bCoeffs[i][noPts-1][2])/10;
	    bCoeffs_end[i][0][3]=bCoeffs[i][noPts-1][3];
	    bCoeffs_end[i][0][4]=bCoeffs[i][noPts-1][3];
	    bCoeffs_end[i][0][5]=bCoeffs[i][noPts-1][3];
	  }	
	  return bCoeffs_end;
	}
	
	void BezierCurve::comp_bezier_curve_deriv(const base::MatrixXd& bezierCurve, int interval, base::MatrixXd& bezierCurve_deriv)
	{
	  bezierCurve_deriv.resize(bezierCurve.rows(), bezierCurve.cols());
	  for(int i=0; i<int(bezierCurve.rows()-1);i++)
	  {
	    bezierCurve_deriv.row(i)=(bezierCurve.row(i+1)-bezierCurve.row(i))*interval;
	  }
	  bezierCurve_deriv.row(int(bezierCurve.rows()-1))=bezierCurve_deriv.row(int(bezierCurve.rows()-1)-1);
	}
	
	
	void BezierCurve::add_points(const base::VectorXi& newPoints)
	{
	  base::MatrixXd traj_temp(m_trajPoints.rows(), m_dim);
	  traj_temp=m_trajPoints;
	  m_trajPoints.resize(m_trajPoints.rows()+int(newPoints.sum()), m_dim);
	  //std::cout<<traj_temp<<std::endl;
	 // std::cout<< m_trajPoints<<std::endl;
	  
	  int ctr=0;
	  for(int i=0; i<newPoints.size();i++)
	  {
	    m_trajPoints.row(ctr)=traj_temp.row(i);
	  //  std::cout<<"i="<<i<<std::endl;
	    //std::cout<<"traj_temp.row(i)="<<traj_temp.row(i)<<std::endl;
	    if(newPoints(i)>0)
	    {
	      for(int j=1; j<int(newPoints(i))+1;j++)
	      {
		ctr=ctr+1;
		m_trajPoints.row(ctr)=((newPoints(i)+1-j)*traj_temp.row(i)+j*traj_temp.row(i+1))/(newPoints(i)+1);
	      }
	    }
	    
	    ctr=ctr+1;
	   // std::cout<<"ctr="<<ctr<<std::endl;
	    //std::cout<<"i="<<i<<std::endl;
	    //std::cout<<" m_jointsPos_curve.m_trajPoints"<<std::endl;
	    //std::cout<< m_jointsPos_curve.m_trajPoints<<std::endl;
	  }
	  m_trajPoints.row(ctr)=traj_temp.row(traj_temp.rows()-1);
	  //std::cout<<" m_jointsPos_curve.m_trajPoints"<<std::endl;
	  //std::cout<< m_trajPoints<<std::endl;
	}
}
