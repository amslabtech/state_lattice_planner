#include "trajectory_generation/boundary_state.h"
#include "math.h"


void generateUniformBoundaryStates(ShapeParameter p_ss, geometry_msgs::PoseStamped x_i, 
									geometry_msgs::PoseArray& boundary_state)
{
	float psi_i = tf::getYaw(x_i.pose.orientation);
	geometry_msgs::Pose bound;
	for(int i=0; i<p_ss.n_p; i++){
		for(int j=0; j<p_ss.n_h; j++){
			float alph = p_ss.alph_min + (p_ss.alph_max-p_ss.alph_min)*i/(p_ss.n_p-1);
			bound.position.x = x_i.pose.position.x + p_ss.d*cos(alph + psi_i);
			bound.position.y = x_i.pose.position.y + p_ss.d*sin(alph + psi_i);
			float psi_f = psi_i + p_ss.psi_min + ((p_ss.psi_max - p_ss.psi_min)*j/(p_ss.n_h - 1)) + alph;
			bound.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,psi_f);
			boundary_state.poses.push_back(bound);
		}
	}
	
}

void generateFinalBoundaryStates(ShapeParameter p_ss, int num,
									geometry_msgs::PoseStamped x_i, 
									geometry_msgs::PoseStamped x_d, 
									geometry_msgs::PoseArray& boundary_state)
{
	float psi_i = tf::getYaw(x_i.pose.orientation);
	geometry_msgs::Pose bound;
	for(int j=0; j<num; j++){
		bound.position.x = x_d.pose.position.x;
		bound.position.y = x_d.pose.position.y;
		float psi_f = psi_i + p_ss.psi_min + ((p_ss.psi_max - p_ss.psi_min)*j/(num - 1));
		bound.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,psi_f);
		boundary_state.poses.push_back(bound);
	}
}


///////////////////////////////////////////////////////
/////---generateGloballyGuidedBoundaryStates----///////
///////////////////////////////////////////////////////
float navFunc(float x, float y, geometry_msgs::PoseStamped x_d)
{
	float dx = x - x_d.pose.position.x;
	float dy = y - x_d.pose.position.y;
	return sqrt(fabs(dx) + fabs(dy));
}

float listSum(list<float> a)
{
	float sum = 0;
	list<float>::iterator it = a.begin();
	for(it = a.begin(); it != a.end(); ++it ){
		sum+=(*it);
	}
	return sum;
}

float findIntersectionCnav(ShapeParameter p_ss, list<float> cnav, float val)
{
	float integ = 0;
	int count = 0;
	list<float>::iterator it = cnav.begin();
	while((it != cnav.end())){
		integ += (*it);
		if(integ > val)	break;
		++it;
		count++;
	}
	//cout<<"count = "<<count<<"\tval = "<<val<<"\tinteg="<<integ<<endl;
	float alph = p_ss.alph_min + (count-1)*(p_ss.alph_max-p_ss.alph_min)/(p_ss.n_s-1);
	return alph;
}

void generateGloballyGuidedBoundaryStates(ShapeParameter p_ss, 
											geometry_msgs::PoseStamped x_i, 
											geometry_msgs::PoseStamped x_d, 
											geometry_msgs::PoseArray& boundary_state)
{
	float psi_i = tf::getYaw(x_i.pose.orientation);
	list<float> cnav;
	for(int i=0; i<p_ss.n_s; i++){
		float alph_s = p_ss.alph_min + (p_ss.alph_max-p_ss.alph_min)*i/(p_ss.n_s-1);
		float nav_x = x_i.pose.position.x + p_ss.d*cos(alph_s + psi_i);
		float nav_y = x_i.pose.position.y + p_ss.d*sin(alph_s + psi_i);
		cnav.push_back(navFunc(nav_x, nav_y, x_d));
	}
	
	float cnav_sum = listSum(cnav);
	list<float>::iterator max_it = max_element( cnav.begin(), cnav.end() );
	float cnav_max = (*max_it);
	//cout<<"max="<<cnav_max<<endl;
	
	list<float>::iterator it = cnav.begin();
	list<float> cnav2;
	++it;
	for(int i=1; i<p_ss.n_s; i++){
		cnav2.push_back((cnav_max - (*it))/(cnav_max*p_ss.n_s - cnav_sum));
		++it;
	}
	
	for(int i=0; i<p_ss.n_p; i++){
		for(int j=0; j<p_ss.n_h; j++){
			geometry_msgs::Pose bound;
			float alph = findIntersectionCnav(p_ss, cnav2, (float)i/((float)p_ss.n_p-1));
			bound.position.x = x_i.pose.position.x + p_ss.d*cos(alph + psi_i);
			bound.position.y = x_i.pose.position.y + p_ss.d*sin(alph + psi_i);
			float psi_f = psi_i + p_ss.psi_min + ((p_ss.psi_max - p_ss.psi_min)*j/(p_ss.n_h - 1)) + alph;
			bound.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,psi_f);
			boundary_state.poses.push_back(bound);
		}
	}
}




///////////////////////////////////////////////////////
////----Generate Lane Guided Boundary States-----//////
///////////////////////////////////////////////////////
void homoTrance(geometry_msgs::PoseStamped x_i, geometry_msgs::PoseStamped& x_center)
{
	float Px = x_center.pose.position.x - x_i.pose.position.x;
	float Py = x_center.pose.position.y - x_i.pose.position.y;
	
	float yaw = tf::getYaw(x_i.pose.orientation);
	float C = cos(yaw);
	float S = sin(yaw);
	
	x_center.pose.position.x = Px*C + Py*S;
	x_center.pose.position.y = Py*C - Px*S;
}


void computeStateAtDistanceAlongLane(nav_msgs::Path l_center, float d, float l_heading, geometry_msgs::PoseStamped x_i,
										geometry_msgs::PoseStamped& x_center)
{
	u_int num = l_center.poses.size();
	cout<<"n"<<num<<endl;
	float dist_sum;
	geometry_msgs::PoseStamped pre_x_center;
	pre_x_center.pose.position.x = l_center.poses[num/2].pose.position.x;
	pre_x_center.pose.position.y = l_center.poses[num/2].pose.position.y;
	homoTrance(x_i, pre_x_center);
	
	//20131001コメントアウト
	for(int i=num/2; i<num; i++){
		//x_center.pose.position.x = l_center.poses[i].pose.position.x;
		//x_center.pose.position.y = l_center.poses[i].pose.position.y;
		
		x_center.pose.position.x = l_center.poses[4+num/2].pose.position.x;
		x_center.pose.position.y = l_center.poses[4+num/2].pose.position.y;
		homoTrance(x_i, x_center);
		
		float dx = x_center.pose.position.x - pre_x_center.pose.position.x;
		float dy = x_center.pose.position.y - pre_x_center.pose.position.y;
		dist_sum += sqrt(dx*dx + dy*dy);
		if(dist_sum > d){
			break;
		}
	}
	
	//x_center.pose.position.x = l_center.poses[5+num/2].pose.position.x;
	//x_center.pose.position.y = l_center.poses[5+num/2].pose.position.y;
	
	x_center.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,l_heading);
	
}

void computeStateAtDistanceAlongLane2(geometry_msgs::PoseStamped x_d, float d, float l_heading, geometry_msgs::PoseStamped x_i,
										geometry_msgs::PoseStamped& x_center)
{
	x_center = x_d;
	homoTrance(x_i, x_center);
	//x_center.pose.position.x = l_center.poses[5+num/2].pose.position.x;
	//x_center.pose.position.y = l_center.poses[5+num/2].pose.position.y;
	
	x_center.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,l_heading);
	
}


void generateLaneGuidedBoundaryStates(LGShapeParameter p_ss, geometry_msgs::PoseStamped x_i, 
									geometry_msgs::PoseArray& boundary_state)
{
	float psi_i = tf::getYaw(x_i.pose.orientation);
	geometry_msgs::Pose bound;
	geometry_msgs::PoseStamped x_center;
	computeStateAtDistanceAlongLane(p_ss.l_center, p_ss.d, p_ss.l_heading, x_i, x_center); 
	for(int i=0; i<p_ss.n_p; i++){
		for(int j=0; j<p_ss.n_l; j++){
			
			int n = i*p_ss.n_l + j;
			
			float delta = -0.5*(p_ss.l_width - p_ss.v_width) + (p_ss.l_width - p_ss.v_width) * i/(p_ss.n_p - 1);
			
			float psi_center = tf::getYaw(x_center.pose.orientation);
			bound.position.x = x_center.pose.position.x - delta*sin(psi_center);
			bound.position.y = x_center.pose.position.y + delta*cos(psi_center);
			float psi_f = p_ss.l_heading;
			bound.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,psi_f);
			boundary_state.poses.push_back(bound);
		}
	}
}

void generateLaneGuidedBoundaryStates2(LGShapeParameter p_ss, geometry_msgs::PoseStamped x_i, 
									geometry_msgs::PoseStamped x_d, 
									geometry_msgs::PoseArray& boundary_state)
{
	float psi_i = tf::getYaw(x_i.pose.orientation);
	geometry_msgs::Pose bound;
	geometry_msgs::PoseStamped x_center;
	computeStateAtDistanceAlongLane2(x_d, p_ss.d, p_ss.l_heading, x_i, x_center); 
	for(int i=0; i<p_ss.n_p; i++){
		for(int j=0; j<p_ss.n_l; j++){
			
			int n = i*p_ss.n_l + j;
			
			float delta = -0.5*(p_ss.l_width - p_ss.v_width) + (p_ss.l_width - p_ss.v_width) * i/(p_ss.n_p - 1);
			
			float psi_center = tf::getYaw(x_center.pose.orientation);
			bound.position.x = x_center.pose.position.x - delta*sin(psi_center);
			bound.position.y = x_center.pose.position.y + delta*cos(psi_center);
			float psi_f = p_ss.l_heading;
			bound.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,psi_f);
			boundary_state.poses.push_back(bound);
		}
	}
}
