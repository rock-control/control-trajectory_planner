#include"joints_trajectory.hpp"
namespace trajectory_planner
{

  joints_trajectory::joints_trajectory():m_const(0), m_dim(0),hasTime(0),m_speedLevel(75), m_timeDesired(-1), m_accMax(5), m_accMin(-5), m_velMax(1.0), m_velMin(-1.0)
  {
  }
  joints_trajectory::joints_trajectory(base::JointsTrajectory joints_traj_in, base::JointLimits limits, double period):m_const(1),hasTime(0),m_order(4),m_speedLevel(75),m_period(period),m_timeDesired(-1), m_accMax(5), m_accMin(-5), m_velMax(1.0), m_velMin(-1.0),
  m_jointsLimits(limits), m_jointsPos_curve(joints_traj_in.getTimeSteps(),joints_traj_in.getNumberOfJoints()),  m_jointsVel_curve(joints_traj_in.getTimeSteps(),joints_traj_in.getNumberOfJoints()), 
  m_jointsAcc_curve(joints_traj_in.getTimeSteps(),joints_traj_in.getNumberOfJoints())
  {
    
    m_dim=joints_traj_in.getNumberOfJoints();
    m_noTrajPts=joints_traj_in.getTimeSteps();
    m_timeSteps.resize(m_noTrajPts);
    m_numIntervals.resize(m_noTrajPts-1);
    
  }
  
  joints_trajectory::joints_trajectory(base::JointsTrajectory joints_traj_in, base::JointLimits limits, double period, double total_desired_time):m_const(2),hasTime(0),m_order(4),m_speedLevel(75),m_period(period),m_timeDesired(total_desired_time),
  m_accMax(5), m_accMin(-5), m_velMax(1.0), m_velMin(-1.0),m_jointsLimits(limits), m_jointsPos_curve(joints_traj_in.getTimeSteps(),joints_traj_in.getNumberOfJoints()),  m_jointsVel_curve(joints_traj_in.getTimeSteps(),joints_traj_in.getNumberOfJoints()), 
  m_jointsAcc_curve(joints_traj_in.getTimeSteps(),joints_traj_in.getNumberOfJoints())
  {
    
    m_dim=joints_traj_in.getNumberOfJoints();
    m_noTrajPts=joints_traj_in.getTimeSteps();
    m_timeSteps.resize(m_noTrajPts);
    m_numIntervals.resize(m_noTrajPts-1);
    
  }
  
  joints_trajectory::~joints_trajectory()
  {
    
  }
  
  void joints_trajectory::initialise_data(const base::JointsTrajectory& joints_traj_in, const base::JointLimits& limits,  double period)
  {
    if(m_const==0)
    {
      
      m_jointsLimits=limits;
      m_period=period;
      m_dim=joints_traj_in.getNumberOfJoints();
      m_noTrajPts=joints_traj_in.elements[0].size();
      m_timeSteps.resize(m_noTrajPts);
      m_numIntervals.resize(m_noTrajPts-1);   
      m_numIntervals.fill(ceil(1/m_period));
      m_order=4;
      
      if(joints_traj_in.isTimed())
      {
	hasTime=1;
      }  
    }
    
    map_jointsTraj2baseMatrix(joints_traj_in);
       
  }
  
  void joints_trajectory::initialise_data(const base::JointLimits& limits,  double period, double acc, int speed_level, int end_condition)
  {
    if(m_const==0)
    {
      
      m_jointsLimits=limits;
      m_period=period;
      m_accMax=acc;
      m_accMin=-acc;
      m_speedLevel=speed_level;
      m_endCondition=end_condition;
      
      
    }  
    
  }
  
  //wrapper function that takes sparse trajectory and outputs interpolated one 
  
  base::JointsTrajectory joints_trajectory::calc_joint_traj_bezier(const base::JointsTrajectory& joints_traj_in)
  {
    
    m_dim=joints_traj_in.getNumberOfJoints();
    std::cout<<"start0  "<<m_dim<<std::endl;
    m_noTrajPts=joints_traj_in.getTimeSteps();
    std::cout<<"start1  "<<m_noTrajPts<<std::endl;
    m_timeSteps.resize(m_noTrajPts);
    m_numIntervals.resize(m_noTrajPts-1);   
    m_numIntervals.fill(ceil(1/m_period));
    m_order=4;
    
    if(joints_traj_in.isTimed())
    {
      hasTime=1;
    }  
    std::cout<<"start1"<<std::endl;
    
    m_jointsPos_curve.initialise_data(m_noTrajPts, m_dim, m_period, 4, m_endCondition);
    m_jointsVel_curve.initialise_data(m_noTrajPts, m_dim, m_period, 4, m_endCondition);
    m_jointsAcc_curve.initialise_data(m_noTrajPts, m_dim, m_period, 4, m_endCondition);
    
    map_jointsTraj2baseMatrix(joints_traj_in);
    // m_jointsPos_curve.initialise_data();
    std::cout<<"start2"<<std::endl;
    int addPts_flag=0;
    double time_interval=1;
    int do_ctr=0;  
    int ctr_lim=1;
   // std::cout<<"Enter time optim counter"<<std::endl;
   // std::cin>>ctr_lim;
    
    base::VectorXi newPoints(m_numIntervals.size());
    newPoints.fill(0);
    
    std::cout<<"start"<<std::endl;
    
    
    m_jointsPos_curve.comp_deBoor_controlpts();
    m_jointsPos_curve.comp_bezier_coeff();
    // std::cout<<"m_jointsPos_curve.comp_bezier_coeff"<<std::endl;
    m_jointsPos_curve.comp_bezier_curve();
    std::cout<<"m_jointsPos_curve.comp_bezier_curve1"<<std::endl;
    int posLimit_exceeded=0;
    posLimit_exceeded=check_position(newPoints);
    //std::cout<<"newPoints1="<<newPoints<<std::endl;
    if(posLimit_exceeded)
    {
      
      std::cout<<"posLimit_exceeded"<<std::endl;
      
      m_jointsPos_curve.add_points(newPoints);
      newPoints.resize(newPoints.size()+newPoints.sum());
      newPoints.fill(0);
      m_numIntervals.resize(newPoints.size());
      // std::cout<<"m_numIntervals.size="<<m_numIntervals.size()<<std::endl;
      m_noTrajPts=m_jointsPos_curve.m_trajPoints.rows();
      //  std::cout<<"m_noTrajPts.size="<<m_noTrajPts<<std::endl;
    }
    
    do
    {
      std::cout<<"entering do"<<std::endl;
      if(addPts_flag)	
      {
	m_jointsPos_curve.add_points(newPoints);  
	newPoints.resize(newPoints.size()+newPoints.sum());
	newPoints.fill(0);
	m_numIntervals.resize(newPoints.size());
	//std::cout<<"m_numIntervals.size="<<m_numIntervals.size()<<std::endl;
	m_noTrajPts=m_jointsPos_curve.m_trajPoints.rows();
	//std::cout<<"m_noTrajPts.size="<<m_noTrajPts<<std::endl;
      }
      m_numIntervals.fill(ceil(time_interval/m_period));
      m_jointsPos_curve.m_bCurve.resize(int(m_numIntervals.sum()+1),m_dim);
      m_jointsVel_curve.m_bCurve.resize(int(m_numIntervals.sum()+1),m_dim);
      m_jointsAcc_curve.m_bCurve.resize(int(m_numIntervals.sum()+1),m_dim);
      m_jointsPos_curve.initialise_data(m_noTrajPts, m_dim,m_period,m_order,1);
      std::cout<<"m_jointsPos_curve.initialise_data"<<std::endl;
      m_jointsVel_curve.initialise_data(m_noTrajPts, m_dim,m_period,m_order-1,0);
      m_jointsAcc_curve.initialise_data(m_noTrajPts, m_dim,m_period,m_order-2,0);
      m_jointsPos_curve.comp_deBoor_controlpts();
      m_jointsPos_curve.comp_bezier_coeff();
      std::cout<<"m_jointsPos_curve.comp_bezier_coeff"<<std::endl;
      
      m_jointsPos_curve.m_bCurve=m_jointsPos_curve.comp_bezier_curve(m_numIntervals);
      
      std::cout<<"m_jointsPos_curve.comp_bezier_curve"<<std::endl;
    //  write_data(m_jointsPos_curve.m_bCurve, "pos_bCurve0.txt");
      m_jointsVel_curve.m_bCoeffs= m_jointsPos_curve.comp_bezier_coeff_deriv(m_jointsPos_curve.m_bCoeffs);
      
      m_jointsAcc_curve.m_bCoeffs.resize(boost::extents[m_jointsPos_curve.m_bCoeffs.shape()[0]][m_jointsPos_curve.m_bCoeffs.shape()[1]][m_jointsPos_curve.m_bCoeffs.shape()[2]-2]);
      m_jointsAcc_curve.m_bCoeffs= m_jointsPos_curve.comp_bezier_coeff_deriv(m_jointsVel_curve.m_bCoeffs);
      
      //m_jointsVel_curve.comp_bezier_curve_deriv(m_jointsPos_curve.m_bCurve, 1/m_period, m_jointsVel_curve.m_bCurve);
      m_jointsVel_curve.m_bCurve= m_jointsVel_curve.comp_bezier_curve(m_numIntervals)/time_interval;
      //write_data(m_jointsVel_curve.m_bCurve, "vel_bCurve0_deriv.txt");
      
      //   m_jointsAcc_curve.comp_bezier_curve_deriv(m_jointsVel_curve.m_bCurve, 1/m_period, m_jointsAcc_curve.m_bCurve);
      // m_jointsAcc_curve.m_bCurve.row(0).fill(0);
      // m_jointsAcc_curve.m_bCurve.row(int(m_jointsAcc_curve.m_bCurve.rows())-1).fill(0);
      // m_jointsAcc_curve.comp_bezier_curve();
      m_jointsAcc_curve.m_bCurve= m_jointsAcc_curve.comp_bezier_curve(m_numIntervals)/(time_interval*time_interval);
    //  write_data(m_jointsAcc_curve.m_bCurve, "acc_bCurve0_deriv.txt");
      
      time_interval=synchronise_time(m_jointsVel_curve.m_bCurve, m_jointsAcc_curve.m_bCurve, time_interval);
      std::cout<<"synchronise_time"<<std::endl;
      // std::cout<<m_numIntervals<<std::endl;
      
      addPts_flag=check_time_reduction(newPoints);
      std::cout<<"Do While loop"<<do_ctr<<std::endl;
      do_ctr=do_ctr+1;
      
    }
    while(time_interval*m_numIntervals.size()>m_timeDesired &&(do_ctr<ctr_lim) && addPts_flag);
    m_numIntervals.fill(ceil(time_interval/m_period));
    
    m_jointsPos_curve.m_bCurve.resize(int(m_numIntervals.sum()+1),m_dim);
    m_jointsVel_curve.m_bCurve.resize(int(m_numIntervals.sum()+1),m_dim);
    m_jointsAcc_curve.m_bCurve.resize(int(m_numIntervals.sum()+1),m_dim);
    
    m_jointsPos_curve.m_bCurve=m_jointsPos_curve.comp_bezier_curve(m_numIntervals);
   // write_data(m_jointsPos_curve.m_bCurve, "pos_bCurve.txt");
    
    double T= m_period*m_numIntervals(0);
    
    m_jointsVel_curve.m_bCurve= m_jointsVel_curve.comp_bezier_curve(m_numIntervals)/T;
   // write_data(m_jointsVel_curve.m_bCurve, "vel_bCurve.txt");
    
    m_jointsAcc_curve.m_bCurve= m_jointsAcc_curve.comp_bezier_curve(m_numIntervals)/(T*T);
   // write_data(m_jointsAcc_curve.m_bCurve, "acc_bCurve.txt");
    
    base::VectorXd time(m_jointsPos_curve.m_bCurve.rows(),1);
    for(int i=0; i<int(time.size());i++)
    {
      time(i)=i*m_period;
    }
  //  write_data(time, "time.txt");
  //  write_data(m_jointsPos_curve.m_trajPoints, "final_trajpoints.txt");
    
    base::JointsTrajectory jointsTraj_out;
    map_baseMatrix2jointsTraj(jointsTraj_out);    
    return jointsTraj_out;
  }
  
  base::JointsTrajectory joints_trajectory::calc_joint_traj_bezier(const base::JointsTrajectory& joints_traj_in, const base::JointLimits& limits,  double period, double acc_max, int end_condition, double time_desired)
  {
    m_speedLevel=100;
    initialise_data(joints_traj_in,limits,period);
    // m_jointsPos_curve.initialise_data();
   
    int addPts_flag=0;
    double time_interval=1;
    int do_ctr=0;  
    int ctr_lim;
    std::cout<<"Enter time optim counter"<<std::endl;
    std::cin>>ctr_lim;
    
    base::VectorXi newPoints(m_numIntervals.size());
    newPoints.fill(0);
    
    m_jointsPos_curve.m_numIntervals=1/m_period;
    m_jointsPos_curve.m_condition=1;
    m_jointsPos_curve.comp_deBoor_controlpts();
    m_jointsPos_curve.comp_bezier_coeff();
   // std::cout<<"m_jointsPos_curve.comp_bezier_coeff"<<std::endl;
    m_jointsPos_curve.comp_bezier_curve();
    std::cout<<"m_jointsPos_curve.comp_bezier_curve1"<<std::endl;
    int posLimit_exceeded=0;
    posLimit_exceeded=check_position(newPoints);
    //std::cout<<"newPoints1="<<newPoints<<std::endl;
    if(posLimit_exceeded)
    {
      
      std::cout<<"posLimit_exceeded"<<std::endl;
      
      m_jointsPos_curve.add_points(newPoints);
      newPoints.resize(newPoints.size()+newPoints.sum());
      newPoints.fill(0);
      m_numIntervals.resize(newPoints.size());
     // std::cout<<"m_numIntervals.size="<<m_numIntervals.size()<<std::endl;
      m_noTrajPts=m_jointsPos_curve.m_trajPoints.rows();
    //  std::cout<<"m_noTrajPts.size="<<m_noTrajPts<<std::endl;
    }
    
    do
    {
      std::cout<<"entering do"<<std::endl;
      if(addPts_flag)	
      {
	m_jointsPos_curve.add_points(newPoints);  
	newPoints.resize(newPoints.size()+newPoints.sum());
	newPoints.fill(0);
	m_numIntervals.resize(newPoints.size());
	//std::cout<<"m_numIntervals.size="<<m_numIntervals.size()<<std::endl;
	m_noTrajPts=m_jointsPos_curve.m_trajPoints.rows();
	//std::cout<<"m_noTrajPts.size="<<m_noTrajPts<<std::endl;
      }
      m_numIntervals.fill(ceil(time_interval/m_period));
      m_jointsPos_curve.m_bCurve.resize(int(m_numIntervals.sum()+1),m_dim);
      m_jointsVel_curve.m_bCurve.resize(int(m_numIntervals.sum()+1),m_dim);
      m_jointsAcc_curve.m_bCurve.resize(int(m_numIntervals.sum()+1),m_dim);
      m_jointsPos_curve.initialise_data(m_noTrajPts, m_dim,m_period,m_order,1);
      std::cout<<"m_jointsPos_curve.initialise_data"<<std::endl;
      m_jointsVel_curve.initialise_data(m_noTrajPts, m_dim,m_period,m_order-1,0);
      m_jointsAcc_curve.initialise_data(m_noTrajPts, m_dim,m_period,m_order-2,0);
      m_jointsPos_curve.comp_deBoor_controlpts();
      m_jointsPos_curve.comp_bezier_coeff();
      std::cout<<"m_jointsPos_curve.comp_bezier_coeff"<<std::endl;
            
      m_jointsPos_curve.m_bCurve=m_jointsPos_curve.comp_bezier_curve(m_numIntervals);
    
      std::cout<<"m_jointsPos_curve.comp_bezier_curve"<<std::endl;
      write_data(m_jointsPos_curve.m_bCurve, "pos_bCurve0.txt");
      m_jointsVel_curve.m_bCoeffs= m_jointsPos_curve.comp_bezier_coeff_deriv(m_jointsPos_curve.m_bCoeffs);
      
      m_jointsAcc_curve.m_bCoeffs.resize(boost::extents[m_jointsPos_curve.m_bCoeffs.shape()[0]][m_jointsPos_curve.m_bCoeffs.shape()[1]][m_jointsPos_curve.m_bCoeffs.shape()[2]-2]);
      m_jointsAcc_curve.m_bCoeffs= m_jointsPos_curve.comp_bezier_coeff_deriv(m_jointsVel_curve.m_bCoeffs);
      
      //m_jointsVel_curve.comp_bezier_curve_deriv(m_jointsPos_curve.m_bCurve, 1/m_period, m_jointsVel_curve.m_bCurve);
      m_jointsVel_curve.m_bCurve= m_jointsVel_curve.comp_bezier_curve(m_numIntervals)/time_interval;
      write_data(m_jointsVel_curve.m_bCurve, "vel_bCurve0_deriv.txt");
      
   //   m_jointsAcc_curve.comp_bezier_curve_deriv(m_jointsVel_curve.m_bCurve, 1/m_period, m_jointsAcc_curve.m_bCurve);
     // m_jointsAcc_curve.m_bCurve.row(0).fill(0);
     // m_jointsAcc_curve.m_bCurve.row(int(m_jointsAcc_curve.m_bCurve.rows())-1).fill(0);
     // m_jointsAcc_curve.comp_bezier_curve();
      m_jointsAcc_curve.m_bCurve= m_jointsAcc_curve.comp_bezier_curve(m_numIntervals)/(time_interval*time_interval);
      write_data(m_jointsAcc_curve.m_bCurve, "acc_bCurve0_deriv.txt");
      
      time_interval=synchronise_time(m_jointsVel_curve.m_bCurve, m_jointsAcc_curve.m_bCurve, time_interval);
      std::cout<<"synchronise_time"<<std::endl;
     // std::cout<<m_numIntervals<<std::endl;

      addPts_flag=check_time_reduction(newPoints);
      std::cout<<"Do While loop"<<do_ctr<<std::endl;
      do_ctr=do_ctr+1;
   
    }
    while(time_interval*m_numIntervals.size()>m_timeDesired &&(do_ctr<ctr_lim) && addPts_flag);
    m_numIntervals.fill(ceil(time_interval/m_period));
    
    m_jointsPos_curve.m_bCurve.resize(int(m_numIntervals.sum()+1),m_dim);
    m_jointsVel_curve.m_bCurve.resize(int(m_numIntervals.sum()+1),m_dim);
    m_jointsAcc_curve.m_bCurve.resize(int(m_numIntervals.sum()+1),m_dim);
    
    m_jointsPos_curve.m_bCurve=m_jointsPos_curve.comp_bezier_curve(m_numIntervals);
    write_data(m_jointsPos_curve.m_bCurve, "pos_bCurve.txt");
    
    double T= m_period*m_numIntervals(0);
    
    m_jointsVel_curve.m_bCurve= m_jointsVel_curve.comp_bezier_curve(m_numIntervals)/T;
    write_data(m_jointsVel_curve.m_bCurve, "vel_bCurve.txt");
    
    m_jointsAcc_curve.m_bCurve= m_jointsAcc_curve.comp_bezier_curve(m_numIntervals)/(T*T);
    write_data(m_jointsAcc_curve.m_bCurve, "acc_bCurve.txt");
       
    base::VectorXd time(m_jointsPos_curve.m_bCurve.rows(),1);
    for(int i=0; i<int(time.size());i++)
    {
      time(i)=i*m_period;
    }
    write_data(time, "time.txt");
    write_data(m_jointsPos_curve.m_trajPoints, "final_trajpoints.txt");
    
    base::JointsTrajectory jointsTraj_out;
    map_baseMatrix2jointsTraj(jointsTraj_out);    
    return jointsTraj_out;
  }
  
  double joints_trajectory::synchronise_time(const base::MatrixXd& bCurve_vel, const base::MatrixXd& bCurve_acc, double time_interval)
  {
    int interval=m_numIntervals(0);
    m_timeSteps(0)=0;
    base::MatrixXd block_curve_vel(interval, m_dim);
    base::MatrixXd block_curve_acc(interval, m_dim);
    double tmax=0;
    base::VectorXd tmin(4);
    tmin.fill(0);
  //  std::cout<<"rows= "<<bCurve_vel.rows()<<std::endl;
  //  std::cout<<"cols= "<<bCurve_vel.cols()<<std::endl;
    
   
    
    for(int i=0; i<m_numIntervals.size();i++)
    {
      tmax=0;
     // std::cout<<"section begin "<<i<<std::endl;
     
      for(int j=0; j<m_dim; j++)
      {
	//std::cout<<"joint "<<j<<std::endl;
	block_curve_vel=bCurve_vel.block(i*interval,j,interval,1);
	block_curve_acc=bCurve_acc.block(i*interval,j,interval,1);
	
	if(m_jointsLimits.elements[j].max.hasSpeed())
	  m_velMax=m_speedLevel/100*m_jointsLimits.elements[j].max.speed;
	  
	if(m_jointsLimits.elements[j].min.hasSpeed())
	  m_velMin=m_speedLevel/100*m_jointsLimits.elements[j].min.speed;
	if(m_jointsLimits.elements[j].max.hasAcceleration())
	  m_accMax=m_jointsLimits.elements[j].max.acceleration;
	
	if(m_jointsLimits.elements[j].min.hasAcceleration())
	  m_accMin=m_jointsLimits.elements[j].min.acceleration;
	
	//std::cout<<"max vel "<<block_curve_vel.maxCoeff()<<std::endl;
	//std::cout<<"min vel "<<block_curve_vel.minCoeff()<<std::endl;
	tmin(0)=fabs(block_curve_vel.maxCoeff()/m_velMax)*time_interval;
	tmin(1)=fabs(block_curve_vel.minCoeff()/m_velMin)*time_interval;
	tmin(2)=sqrt(fabs(block_curve_acc.maxCoeff()/m_accMax))*time_interval;
	tmin(3)=sqrt(fabs(block_curve_acc.minCoeff()/m_accMax))*time_interval;
	
	//std::cout<<"tmin "<<j<<"= "<<tmin<<std::endl;
	tmax=std::max(tmax,tmin.maxCoeff());
      }
     // std::cout<<"section tmax"<<i<<" "<<tmax<<std::endl;
      m_numIntervals(i)=ceil(tmax/m_period);
      tmax=m_numIntervals(i)*m_period;
     // m_timeSteps(i+1)=m_timeSteps(i)+tmax;
    }
    int r,c;
    
    tmax=m_numIntervals.maxCoeff(&r, &c)*m_period;
    
   
   return tmax;
     
  
  }
  
  // checks if the positions ae within joint limits

  int joints_trajectory::check_position(base::VectorXi& newPoints)
  {
    int interval=1/m_period;
    base::MatrixXd block_curve_pos(interval, m_dim);
    
    for(int i=0; i<m_noTrajPts-1;i++)
    {
     // std::cout<<"section begin "<<i<<std::endl;
      for(int j=0; j<m_dim; j++)
      {
	block_curve_pos=m_jointsPos_curve.m_bCurve.block(i*interval,j,interval,1);
	if((block_curve_pos.maxCoeff()>m_jointsLimits.elements[j].max.position) || (block_curve_pos.minCoeff()<m_jointsLimits.elements[j].min.position))
	{
	  newPoints(i)=1;
	  break;
	}
      }
    }
    
    return newPoints.sum();
    
  }
  
  //check if there is possibility for time reduction
  
  int joints_trajectory::check_time_reduction(base::VectorXi& newPoints)
  {
    base::VectorXd time(m_numIntervals.size());
    time=(m_numIntervals)*m_period;
   // std::cout<<"time= "<<time<<std::endl;
    time.sort();
   // std::cout<<"time sorted?= "<<time<<std::endl;
    
    int median_idx=floor(time.size()/2);
    double median=time(median_idx);
    time=(m_numIntervals)*m_period;
    double minimum=time.minCoeff();
   // std::cout<<"time= "<<time<<std::endl;
   // std::cout<<"median= "<<median<<std::endl;
   // std::cout<<"medianindx= "<<median_idx<<std::endl;
    newPoints.resize(time.size());
    int result=0;
    for(int i=0; i<newPoints.size(); i++)
    {
      newPoints(i)=fmax(ceil(time(i)/median)-1, 0);
      result=result+newPoints(i);
      //
    }
    //std::cout<<"newPoints= "<<newPoints<<std::endl;
    return result;
  }
  
  //converts joints trajectory to base matrix form
  void joints_trajectory::map_jointsTraj2baseMatrix(const base::JointsTrajectory& jointsTraj_in)
  {
    if(jointsTraj_in.elements[0][0].hasPosition())
      {
	for(int i=0; i<m_dim;i++)
	{
	  for(int j=0; j<m_noTrajPts;j++)
	  {
	    m_jointsPos_curve.m_trajPoints(j,i)=jointsTraj_in.elements[i][j].position;
	  }     
	}
      }
      
      if(jointsTraj_in.elements[0][0].hasSpeed())
      {
	for(int i=0; i<m_dim;i++)
	{
	  for(int j=0; j<m_noTrajPts;j++)
	  {
	    m_jointsVel_curve.m_trajPoints(j,i)=jointsTraj_in.elements[i][j].speed;
	  }     
	}
	
      }
      
      if(jointsTraj_in.elements[0][0].hasAcceleration())
      {
	for(int i=0; i<m_dim;i++)
	{
	  for(int j=0; j<m_noTrajPts;j++)
	  {
	    m_jointsAcc_curve.m_trajPoints(j,i)=jointsTraj_in.elements[i][j].acceleration;
	  } 
	 
	}
      }
      
    
    if(hasTime==1)
    {
      for(int i=0; i<m_noTrajPts;i++)
      {
	m_timeSteps(i)=jointsTraj_in.times[i].toSeconds();
      } 
    }
  }
  // convert interpolated bezier curve in base matrix form to base::JointsTrajectory
  void joints_trajectory::map_baseMatrix2jointsTraj(base::JointsTrajectory& jointsTraj_out)
  {
    jointsTraj_out.resize(m_dim,m_jointsPos_curve.m_bCurve.rows());
    for(int i=0; i<m_dim;i++)
    {
      for(int j=0; j<int(m_jointsPos_curve.m_bCurve.rows());j++)
      {
        jointsTraj_out.elements[i][j].position=m_jointsPos_curve.m_bCurve(j,i);
	jointsTraj_out.elements[i][j].speed=m_jointsVel_curve.m_bCurve(j,i);
	jointsTraj_out.elements[i][j].acceleration=m_jointsAcc_curve.m_bCurve(j,i);
	
	if(i==0)
	{
	  jointsTraj_out.times[j].fromSeconds(j*m_period);
	}
      }     
    }
    
  }
  
 
  
  void joints_trajectory::correct_knotacc()
  {
    int order=m_jointsAcc_curve.m_bCoeffs.shape()[2];
    int noPts=m_jointsAcc_curve.m_bCoeffs.shape()[1];
    std::cout<<"correct_knotacc npts= "<<noPts<<std::endl;
    int sum=0;
    m_jointsAcc_curve.m_bCurve.row(0).fill(0);
    for(int i=0; i<m_noTrajPts-1; i++)
    {
      sum=sum+int(m_numIntervals(i));
      std::cout<<"sum= "<<sum<<std::endl;
      for(int j=0; j<m_dim; j++)
      {
	std::cout<<"j= "<<j<<std::endl;
      	m_jointsAcc_curve.m_bCurve(sum-2,j)=m_jointsAcc_curve.m_bCoeffs[j][i][order-1];
      }
    
    }
  }
  
  //write data to a txt file from a base matrix

  void joints_trajectory::write_data(const base::MatrixXd& bCurve, const char*filename)
  {
    
    std::ofstream file(filename);
   // std::cout<<"rows= "<<bCurve.rows()<<std::endl;
   // std::cout<<"cols= "<<bCurve.cols()<<std::endl;
    if (file.is_open())
    {
      for(int i=0; i<bCurve.rows(); i++)
      {
	for(int j=0; j<bCurve.cols(); j++)
	{
	  file<<bCurve(i,j)<< " ";
	}
	file<<std::endl;
      }
    }
    file.close();
  }
  
}
