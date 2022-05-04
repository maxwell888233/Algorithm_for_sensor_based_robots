#include "assignment3_context.h"
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <vector>
#include <iostream>
#include <algorithm>


// utility function to test for a collision
bool CS436Context::is_colliding( const vertex& q ) const {
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q );

  if( getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
    { return true; }
  else
    { return false; }
}

// utility function to interpolate between two configurations
CS436Context::vertex CS436Context::interpolate( const CS436Context::vertex& qA,
						const CS436Context::vertex& qB,
						double t ){
  CS436Context::vertex qt( qA.size(), 0.0 );
  for( std::size_t i=0; i<qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
  return qt;
}

CS436Context::CS436Context( const robot_model::RobotModelConstPtr& robotmodel,
			    const std::string& name, 
			    const std::string& group, 
			    const ros::NodeHandle& nh ) :
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){}

CS436Context::~CS436Context(){}


// TODO
CS436Context::index CS436Context::select_config_from_tree( const std::vector<CS436Context::weight>& w ){
  CS436Context::index i;

  // TODO
  // Use the weights to return the index of a configuration in the tree
  std::discrete_distribution<size_t> dist ( std::begin(w), std::end(w) );
  std::random_device rd;
  std::mt19937 gen(rd());
  i = dist(gen);

  return i;
}


// TODO
CS436Context::vertex CS436Context::sample_nearby( const CS436Context::vertex& q ){
  CS436Context::vertex q_rand;

  // TODO
  // Generate a random configuration near q
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> d(-0.4,0.4);
  vertex error;
  double rand;
  bool collide; 
  do {
    for ( std::size_t i=0; i < 6; i++)
  {
    do{
      rand = d(gen);
    }
    while (rand<-3.14 || rand>3.14);
    // std::cout<<rand<<std::endl;
    error.push_back(rand);
    q_rand.push_back(q[i] + error[i]);
    // std::cout<<is_colliding(q_rand)<<std::endl;
  }
  collide = is_colliding(q_rand);
  // std::cout<<collide<<std::endl;
  }
  while (collide);

  return q_rand;
}

// TODO
bool CS436Context::is_local_path_collision_free( const CS436Context::vertex& q,
						                                     const CS436Context::vertex& q_rand ){
  bool collision_free;
  
  // return true/false if the local path between q and q_rand is collision free
  std::vector<double> qt( q.size(), 0.0 );
  for ( double i=0; i <= 1; i = i + 0.05)
  {
    std::cout<<"start collision check\n";
    qt = interpolate(q,q_rand,i);
    if (is_colliding(qt))
    {
    collision_free = false;
    break;
    }
    collision_free = true;
  }
  std::cout<<collision_free<<std::endl;
  return collision_free;
}


// TODO
CS436Context::path CS436Context::search_path( const std::vector<vertex>& V,
 					                                    const index& idx_init,
					                                    const index& idx_goal,
                                              const std::vector<index>& map){
//map is the connection between nodes, i.e the structure of tree, dim=1*n
//V is the joints configuration of each node, dim=6*n
  CS436Context::path P;
  
  // TODO
  // find and return a path (ordered sequence of configurations) between the indices
  std::size_t idx_end;
  std::size_t idx_form;

  P.push_back(V[idx_goal]);

  // while idx_init is not in p
  while (P.back() != V[idx_init]){
    std::vector<double> end_joints = P.back(); // idx_end is the last node of current path
   
    for ( std::size_t i=0; i<P.size(); i++)
    {
      if (end_joints == V[i])
      {
        idx_end = i; // to get the index of the end_joints
      }
    }
    idx_form = map[idx_end]; // to get the idx_form from map, map[idx_end]={idx_form}
    P.push_back(V[idx_form]);
  }

  std::reverse(P.begin(), P.end());
  return P;
}

// TODO
CS436Context::path CS436Context::est( const CS436Context::vertex& q_init,
 				                              const CS436Context::vertex& q_goal ){
  CS436Context::path P;

  // TODO implement EST algorithm and return the path (an ordered sequence of configurations).
    // std::vector<double> vertex;
    std::vector<double> q_left;
    std::vector<double> q_right;
    CS436Context::path P_left;
    CS436Context::path P_right;
    std::vector<double> weight_left;
    std::vector<double> weight_right;
    std::size_t i_left;
    std::size_t i_right;
    std::vector<index> map_left;
    std::vector<index> map_right;
    std::vector<vertex> V_left;
    std::vector<vertex> V_right;
    std::size_t idx_left_end;
    std::size_t idx_right_end;
    std::vector<double> q_rand_left;
    std::vector<double> q_rand_right;
    std::vector<double> distance_vector_left;
    std::vector<double> distance_vector_right;
    std::vector<double> distance_end_vector;
    std::size_t idx_init; // q_init index of V_left
    std::size_t idx_goal; // q_goal index of V_right
    bool local;
    bool global;
    double norm;
    bool norm_ok;

    // initialize V weight
    V_left.push_back(q_init);
    V_right.push_back(q_goal); // V_right is inverse put ,take care!!!!!!!!!!!
    weight_left.push_back(1.0);
    weight_right.push_back(1.0);

    do{
    i_left = select_config_from_tree(weight_left);
    i_right = select_config_from_tree(weight_right); // to get the selected index from weight;

    ////////////////////////////////////////////////////
    std::cout<<"i_left :"<<i_left<<std::endl;
    std::cout<<"i_right :"<<i_right<<std::endl;
    ////////////////////////////////////////////////////

    q_left = V_left[i_left];
    q_right = V_right[i_right]; // corresponding joints configuration;

    //////////////////////////////////////////////////////
    for (size_t i = 0; i<6; i++)
    {
      std::cout<<"q_left :"<<i<<" "<<q_left[i]<<std::endl;
    }
    ////////////////////////////////////////////////////

    do{
      q_rand_left = sample_nearby(q_left);
      q_rand_right = sample_nearby(q_right); // to get the random nearby sample from selected vertex;
    
    ///////////////////////////////////////////////////////
    for (size_t i = 0; i<6; i++)
    {
      std::cout<<"q_rand_left :"<<i<<" "<<q_rand_left[i]<<std::endl;
    }  
    ///////////////////////////////////////////////////////

      local = is_local_path_collision_free(q_left,q_rand_left) && is_local_path_collision_free(q_right,q_rand_right);
      
      ////////////////////////////////////////////////////////
      std::cout<<local<<std::endl;
      ////////////////////////////////////////////////////////

      // to check if both q_rand is collision free to selected node, true is both collision free    
    }
    while(!local);

    // to update w V map
    V_left.push_back(q_left);
    V_right.push_back(q_right);

    map_left.push_back(i_left);
    map_right.push_back(i_right);

    /////////////////////////////////////////////////////////////////
    std::cout<<"map_left.size is "<<map_left.size()<<std::endl;
    std::cout<<"map_right.size is "<<map_right.size()<<std::endl;
    std::cout<<"map_left[0] is "<<map_left[0]<<std::endl;
    //////////////////////////////////////////////////////////////////

    // for (int i = 0; i<map_left.size(); i++)
    // {
    //   for (int j = 0; j<map_left.size(); j++)
    //   {
    //     for (int m = 0; m<6; m++)
    //     {
    //       distance_vector_left[m] = V_left[i][m] - V_left[j][m];

    //       /////////////////////////////////////////////////////////////
    //       std::cout<<"V_left[i][m] is "<<V_left[i][m]<<std::endl;
    //       std::cout<<distance_vector_left[m]<<std::endl;
    //       /////////////////////////////////////////////////////////////
    //     }
    //     norm  = 0;
    //     for (int k = 0; k<distance_vector_left.size(); k++)
    //     {
    //       norm = norm + distance_vector_left[k] * distance_vector_left[k];
    //     }
    //     if (norm<2.0) ///////adjustable
    //     {
    weight_left.push_back(0.5);
    weight_left[i_left] = 1/((1/weight_left[i_left])+1);
    
    ////////////////////////////////////////////////////////
    for (int i = 0; i<weight_left.size(); i++)
    {
        std::cout<<"weight_left["<<i<<"] is "<<weight_left[i]<<std::endl;
    }
    ///////////////////////////////////////////////////////
    

    //     }
    //   }
    // }

    // for (int i = 0; i<map_right.size(); i++)
    // {
    //   for (int j = 0; j<map_right.size(); j++)
    //   {
    //     for (int m = 0; m<6; m++)
    //     {
    //       distance_vector_right[m] = V_right[i][m] - V_right[j][m];
    //     }
    //     norm = 0;
    //     for (int k = 0; k<distance_vector_right.size(); k++)
    //     {
    //       norm = norm + distance_vector_right[k] * distance_vector_right[k];
    //     }
    //     if (norm<2.0) ///////adjustable
    //     {
    weight_right.push_back(0.5);
    weight_right[i_right] = 1/((1/weight_right[i_right])+1);

    ///////////////////////////////////////////////////////////////////
    for (int i = 0; i<weight_right.size(); i++)
    {
        std::cout<<"weight_right["<<i<<"] is "<<weight_right[i]<<std::endl;
    }
    ///////////////////////////////////////////////////////////////////

    //     }
    //   }
    // }

    for (int i = 0; i<V_left.size(); i++)
    {
      for(int j = 0; j<V_right.size(); j++)
      {
        global = is_local_path_collision_free(V_left[i],V_right[j]);
        if (global)
        {
          idx_left_end = i;
          idx_right_end = j;
          
          ///////////////////////////////////////////////
          std::cout<<"global is true"<<std::endl;
          std::cout<<"idx_left_end is "<<i<<std::endl;
          std::cout<<"idx_right_end is "<<j<<std::endl;
          std::cout<<"V_left[idx_left_end][0] is "<<V_left[idx_left_end][0]<<std::endl;
          std::cout<<"size is "<<V_left[idx_left_end].size()<<std::endl;
          ///////////////////////////////////////////////

          break;
        }
      }
      norm = 0;

      /////////////////////////////////////////////////////////////
      for (int i = 0; i<6; i++)
      {
        std::cout<<V_left[idx_left_end][i]<<std::endl;
      }
      for (int i = 0; i<6; i++)
      {
        std::cout<<V_right[idx_right_end][i]<<std::endl;
      }      
      /////////////////////////////////////////////////////////////

      for (int k = 0; k<6; k++)
      {
        distance_end_vector.push_back(V_left[idx_left_end][k] - V_right[idx_right_end][k]);
        norm = norm + distance_end_vector[k] * distance_end_vector[k];

        ////////////////////////////////////////////////////////////////////////////////
        std::cout<<"V_left[idx_left_end][k] - V_right[idx_right_end][k] is "<<V_left[idx_left_end][k]<<" - "<<V_right[idx_right_end][k]<<std::endl;
        std::cout<<"distance_end_vector[k] is "<<distance_end_vector[k]<<std::endl;
        std::cout<<"norm is "<<norm<<std::endl;
        ////////////////////////////////////////////////////////////////////////////////        
      }
      if (norm < 3)
      {
        break;
      }
    }
      if (norm < 3)
      {
        norm_ok = true;
      }
    }
    while(!global && !norm_ok);

    for (int i = 0; i<V_left.size(); i++)
    {
      if (V_left[i] == q_init)
      {
        idx_init = i;

        ///////////////////////////////////////////
        std::cout<<"idx_init is "<<i<<std::endl;
        /////////////////////////////////////////////
      }
    }
    for (int i = 0; i<V_right.size(); i++)
    {
      if (V_right[i] == q_goal)
      {
        idx_goal = i;
        //////////////////////////////////////////////
        std::cout<<"idx_goal is "<<i<<std::endl;
        ////////////////////////////////////////////////
      }
    }
      
    P_left = search_path(V_left,idx_init,idx_left_end,map_left);
    P_right = search_path(V_right,idx_right_end,idx_goal,map_right);
    for (size_t i = 0; i<P_right.size(); i++)
    {
      P_left.push_back(P_right[i]);
    }
    P = P_left;

    return P;
}



// This is the method that is called each time a plan is requested
// You should not need to modify code below this (but it's ok of you absolutely need to).
bool CS436Context::solve( planning_interface::MotionPlanResponse &res ){

  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, getGroupName()));
  res.trajectory_->clear();

  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  vertex q_init, q_goal;
  for( size_t i=0; i<robotmodel->getVariableCount(); i++ ){
    q_goal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    q_init.push_back(request_.start_state.joint_state.position[i]);
  }

  // start the timer
  ros::Time begin = ros::Time::now();

  path P = est( q_init, q_goal );

  // end the timer
  ros::Time end = ros::Time::now();

  // The rest is to fill in the animation. You can ignore this part.
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q_init );

  for( std::size_t i=1; i<P.size(); i++ ){
    for( double t=0.0; t<=1.0; t+=0.01 ){
      vertex q = interpolate( P[i-1], P[i], t );
      robotstate.setJointGroupPositions( "manipulator", q );
      res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
    }
  }
  
  // set the planning time
  ros::Duration duration = end-begin;
  res.planning_time_ = duration.toSec();
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
  
}

bool CS436Context::solve( planning_interface::MotionPlanDetailedResponse &res )
{ return true; }

void CS436Context::clear(){}

bool CS436Context::terminate(){return true;}
