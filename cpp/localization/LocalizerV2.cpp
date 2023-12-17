#include "LocalizerV2.h"
#include "math.h"
#include "iostream"
#include "World.h"

using namespace std;

static World& world = SWorld::getInstance();


/**
 *  Compute 3D position and 3D orientation
 * */
void LocalizerV2::run(){

	Field& fd = SField::getInstance();

	stats_change_state(RUNNING);

	//------------------ WORKFLOW: 0

	_is_uptodate = false;
	_is_head_z_uptodate = false;
	_steps_since_last_update++;

	prelim_reset(); //reset preliminary transformation matrix

	fd.update(); //update visible collections
	int lines_no = fd.list_segments.size();
	int landmarks_no = fd.list_landmarks.size();

	if( (landmarks_no == 0 && lines_no < 2) || (lines_no == 0) ){   
		if(lines_no==0 && landmarks_no==0){ stats_change_state(BLIND);   } else { stats_change_state(MINFAIL);   }
		return;
	}

	//------------------ WORKFLOW: 1-2

	if( ! find_z_axis_orient_vec() ){ return; }

	//------------------ WORKFLOW: 3-4

	if(!(  landmarks_no >1 ? find_xy() : guess_xy()  )){ return; }

	//------------------ Update public variables

	commit_everything();

	stats_change_state(DONE);

	//------------------ Statistics

	//Ball position stats
	if(world.ball_seen){
		counter_ball += stats_sample_position_error(prelimHeadToField * world.ball_rel_pos_cart,
			world.ball_cheat_abs_cart_pos, errorSum_ball);
	}

	//print_report(); //uncomment to enable report (average errors + solution analysis)

}



//=================================================================================================
//=================================================================================== GSL Utilities
//=================================================================================================


void add_gsl_regression_sample(gsl_matrix* m, gsl_vector* v, int sample_no, const Vector3f& relativeCoord, double absoluteCoord, double translCoeffMult=1){

	gsl_matrix_set(m, sample_no, 0, relativeCoord.x);
	gsl_matrix_set(m, sample_no, 1, relativeCoord.y);
	gsl_matrix_set(m, sample_no, 2, relativeCoord.z); 
	gsl_matrix_set(m, sample_no, 3, translCoeffMult); 
	gsl_vector_set(v, sample_no, absoluteCoord );

}

template<std::size_t SIZE>
gsl_vector* create_gsl_vector(const std::array<double, SIZE> &content){

	gsl_vector* v = gsl_vector_alloc (SIZE);
	
	for(int i=0; i<SIZE; i++){
		gsl_vector_set(v, i, content[i]);
	}
	return v;
}


//=================================================================================================
//================================================================================== Math utilities
//=================================================================================================


/**
 * Get unit vector on plane Z=0, perpendicular to given vector
 * Mathematically this is (0,0,1)x(vec)/|vec| with some additional checks
 */
Vector get_ground_unit_vec_perpendicular_to(const Vector3f& vec){

	float gx = 1, gy = 0; //rotation axis unit vector, default:(1,0,0)
	const float aux = sqrtf(vec.x*vec.x + vec.y*vec.y); //rotation axis length before becoming a unit vector

	if(aux > 0){ //if vec is (0,0,1) or (0,0,-1) we keep the default rotation axis
		gx = -vec.y / aux;
		gy = vec.x / aux;
	}

	return Vector(gx,gy);
}


/**
 * Rotate vector around ground axis defined as u=(0,0,1)x(Zvec)/|Zvec| 
 * Direction of rotation from Zvec to (0,0,1)
 * To invert the rotation direction, invert Zvec.x and Zvec.y 
 * @param v vector to be rotated
 * @param Zvec unit normal vector of rotated ground plane
 */
Vector3f fast_rotate_around_ground_axis(Vector3f v, Vector3f Zvec){

	Vector u = get_ground_unit_vec_perpendicular_to(Zvec);

	//Angle between unit normal vector of original plane and unit normal vector of rotated plane:
	//cos(a) = (ov.rv)/(|ov||rv|) = ((0,0,1).(rvx,rvy,rvz))/(1*1) = rvz
	float& cos_a = Zvec.z; 
	//assert(cos_a <= 1);
	if(cos_a > 1) cos_a = 1; //Fix: it happens rarely, no cause was yet detected
	float sin_a = -sqrtf(1 - cos_a*cos_a); //invert sin_a to invert a (direction was defined in method description)

	const float i_cos_a = 1-cos_a;
	const float uxuy_i = u.x*u.y*i_cos_a;
	const float uxux_i = u.x*u.x*i_cos_a;
	const float uyuy_i = u.y*u.y*i_cos_a;
	const float uxsin_a = u.x*sin_a;
	const float uysin_a = u.y*sin_a;

	float x = (cos_a  + uxux_i ) * v.x + uxuy_i * v.y + uysin_a * v.z;
	float y = uxuy_i * v.x + (cos_a  + uyuy_i ) * v.y - uxsin_a * v.z;
	float z = - uysin_a * v.x + uxsin_a * v.y + cos_a * v.z;

	return Vector3f(x,y,z);

}


/**
 * Compute Xvec and Yvec from Zvec and angle
 * @param Zvec  input z-axis orientation vector
 * @param angle input rotation of Xvec around Zvec (rads)
 * @param Xvec  output x-axis orientation vector 
 * @param Yvec  output y-axis orientation vector
 */
void fast_compute_XYvec_from_Zvec(const Vector3f& Zvec, float agent_angle, Vector3f& Xvec, Vector3f& Yvec){
	/**
	 * There are two coordinate systems being considered in this method:
	 * - The actual agent's vision (RELATIVE system -> RELsys)
	 * - A rotated perspective where the agent's seen Zvec is the real Zvec (ROTATED system -> ROTsys)
	 * 		(the agent's optical axis / line of sight is parallel to ground ) 
	 * 
	 * SUMMARY:
	 * 		I provide an angle which defines the agent's rotation around Zvec (in the ROTsys)
	 * 		E.g. suppose the agent is rotated 5deg, then in the ROTsys, the agent sees Xvec as being rotated -5deg
	 * 		I then rotate Xvec to the RELsys, and compute the Yvec using cross product
	 * 
	 * STEPS:
	 * 1st. Compute ROTsys, by finding rotation of plane defined by normal vector Zvec, in relation to seen XY plane 
	 * (whose seen normal vector is (0,0,1)). We need: axis of rotation (unit vector lying on XY plane) and angle (rads):
	 * 		rotation axis:  
	 * 			u = (0,0,1)x(Zvec) (vector perpendicular to XY plane and Zvec)
	 * 			  = (-ZvecY,ZvecX,0) (rotation will be counterclockwise when u points towards the observer)
	 * 							     (so a negative angle will bring the agent to the ROTsys)
	 * 		angle between Zvec and (0,0,1): 
	 * 			a = acos( ((0,0,1).(Zvec)) / (|(0,0,1)|*|Zvec|) )
	 *      	  =	acos( ((0,0,1).(Zvec)) )
	 * 			  =	acos( ZvecZ )
	 * 
	 * 2nd. Establish Xvec in ROTsys:
	 * 		Let agent_angle be the agent's angle. Then Xvec's angle is (b=-agent_angle).
	 * 			Xvec = (cos(b),sin(b),0)
	 * 
	 * 3rd. Rotate Xvec to RELsys:
	 * 		Let R be the rotation matrix that rotates from ROTsys to RELsys (positive angle using u):
	 * 		Xvec = R * Xvec
 	 * 		     = R * (cos(b),sin(b),0)
	 *           = (R00 * cos(b) + R01 * sin(b), R10 * cos(b) + R11 * sin(b), R20 * cos(b) + R21 * sin(b))
	 * 		where R is: (rotation matrix from axis and angle https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle)
	 * 			R00 = cos(a) + ux*ux(1-cos(a))    R01 = ux*uy(1-cos(a))
	 * 			R10 = uy*ux(1-cos(a))             R11 = cos(a) + uy*uy(1-cos(a))
	 *      	R20 = -uy*sin(a)                  R21 = ux*sin(a)
	 * 		so Xvec becomes:
	 * 			XvecX = cos(a)*cos(b) + (1-cos(a))*(ux*ux*cos(b) + ux*uy*sin(b))
	 * 			XvecY = cos(a)*sin(b) + (1-cos(a))*(uy*uy*sin(b) + ux*uy*cos(b))
	 * 			XvecZ = sin(a)*(ux*sin(b) - uy*cos(b))
	 * 
	 * 4th. To find Yvec we have two options:
	 * 		A. add pi/2 to b and compute Yvec with the same expression used for Xvec
	 * 		B. Yvec = Zvec x Xvec  (specifically in this order for original coordinate system)
	 */

	Vector u = get_ground_unit_vec_perpendicular_to(Zvec);

	const float& cos_a = Zvec.z; 
	const float sin_a = sqrtf(1 - cos_a*cos_a);
	const float uxuy = u.x * u.y;
	const float b = -agent_angle; //Xvec's angle
	const float cos_b = cosf(b);
	const float sin_b = sinf(b);
	const float i_cos_a = 1-cos_a;

	Xvec.x = cos_a * cos_b + i_cos_a * ( u.x*u.x*cos_b + uxuy*sin_b );
	Xvec.y = cos_a * sin_b + i_cos_a * ( u.y*u.y*sin_b + uxuy*cos_b );
	Xvec.z = sin_a * ( u.x*sin_b - u.y*cos_b );

	Yvec = Zvec.crossProduct(Xvec); //Using original coordinate system
}


//=================================================================================================
//================================================================================== Main algorithm
//=================================================================================================

/**
 * WORKFLOW: 1
 * See workflow description on header
 */ 
bool LocalizerV2::find_z_axis_orient_vec(){

	Field& fd = SField::getInstance();

	const int goalNo = fd.list_landmarks_goalposts.size();

	if(fd.non_collinear_ground_markers >= 3){
		fit_ground_plane();
		return true;
	}

	/**
	 * At this point we have > 0 lines. Having 1 line is the most common option.
	 * But having 2 lines is also possible if at least one of them is too short to create 2 reference points.
	 * But those 2 lines can be both too short, which would not be ideal for the algorithms below.
	 * Steps: find largest line, check if it is large enough
	 */
	const Line6f* l = &fd.list_segments.front();
	if(fd.list_segments.size() > 1){
		for(const auto& ll: fd.list_segments){
			if(ll.length > l->length) l = &ll;
		}
	}

	if(l->length < 1){ //larger line is too short
		stats_change_state(FAILzLine);
		return false;
	}

	if(goalNo == 0){ //Case e.g.: 1 corner and 1 line (or 2 short lines)
		stats_change_state(FAILzNOgoal);
		return false;
	}
		

	//------------------------------ Prepare variables for solution A & B

	// Get any crossbar point
	Vector3f crossbar_pt;
	auto& glist = fd.list_landmarks_goalposts;
	if(goalNo == 1){
		crossbar_pt = glist.front().relPosCart;
	}else{
		if(glist[0].absPos.x == glist[1].absPos.x){
			crossbar_pt = Vector3f::determineMidpoint( glist[0].relPosCart, glist[1].relPosCart);
		}else{
			//extremely rare: there are other solutions when goalNo>2 but cost/benefit not worth it
			crossbar_pt = glist.front().relPosCart; 
		}
	}

	//------------------------------ Identify and apply solution A & B

	//Get line point closest to crossbar point
	Vector3f p = l->linePointClosestToCartPoint( crossbar_pt );
	Vector3f possibleZvec = crossbar_pt - p;
	float possibleZvecLength = possibleZvec.length();

	if(fabsf(possibleZvecLength - 0.8) < 0.05){ //Solution A & B
		Vector3f unit_zvec = possibleZvec / possibleZvecLength;

		// save as the new z axis orientation vector
		prelimHeadToField.set(2,0,unit_zvec.x);
		prelimHeadToField.set(2,1,unit_zvec.y);
		prelimHeadToField.set(2,2,unit_zvec.z);

		find_z(unit_zvec);
		return true;
	}

	//------------------------------ Apply solution C

	Vector3f crossbar_left_vec, crossbar_midp; //this crossbar vector points left if seen from the midfield (this is important for the cross product)

	const auto& goal_mm = Field::list_8_landmarks::goal_mm;
	const auto& goal_mp = Field::list_8_landmarks::goal_mp;
	const auto& goal_pm = Field::list_8_landmarks::goal_pm;
	const auto& goal_pp = Field::list_8_landmarks::goal_pp;

	if(                     goal_mm.visible   && goal_mp.visible){
		crossbar_left_vec = goal_mm.relPosCart - goal_mp.relPosCart;
		crossbar_midp =    (goal_mm.relPosCart + goal_mp.relPosCart)/2;
	}else if(               goal_pp.visible   && goal_pm.visible){
		crossbar_left_vec = goal_pp.relPosCart - goal_pm.relPosCart;
		crossbar_midp =    (goal_pp.relPosCart + goal_pm.relPosCart)/2;
	}

	/**
	 * Check if angle between line and crossbar is between 45deg and 135deg
	 * 		45deg < acos(line.crossbar / |line||crossbar|) < 135deg  <=>
	 * 		| line.crossbar / |line||crossbar| | < cos(45deg)        <=>
	 * 		| line.crossbar | < cos(45deg) * |line| * ~2.1           <=>
	 * 		| line.crossbar | < 1.485 * |line|
	 */
	Vector3f lvec = l->get_cart_vector();
	if( goalNo > 1 && fabsf(lvec.innerProduct(crossbar_left_vec)) < 1.485 * l->length ){
		Vector3f Zvec;
		if(l->startc.dist(crossbar_midp) > l->endc.dist(crossbar_midp)){
			Zvec = lvec.crossProduct(crossbar_left_vec);
		}else{
			Zvec = crossbar_left_vec.crossProduct(lvec);
		}
		Zvec = Zvec.normalize(); // get unit vector

		// save as the new z axis orientation vector
		prelimHeadToField.set(2,0,Zvec.x);
		prelimHeadToField.set(2,1,Zvec.y);
		prelimHeadToField.set(2,2,Zvec.z);

		find_z(Zvec);
		return true;
	}

	

	stats_change_state(FAILz);
	return false; //no solution was found
}

/**
 * Find the best fitting ground plane's normal vector using Singular Value Decomposition
 * Also compute the agent's height based on the ground references' centroid
 * Dependency: at least 3 ground references
 */
void LocalizerV2::fit_ground_plane(){

	Field& fd = SField::getInstance();

	const auto& ground_markers = fd.list_weighted_ground_markers;
	const int ground_m_size = ground_markers.size();

	//------------------------------------ Compute groundmarks plane (if we have at least 3 groundmarks)

	gsl_matrix *A, *V;
	gsl_vector *S, *work;

	A = gsl_matrix_alloc(ground_m_size, 3); 
	V = gsl_matrix_alloc(3, 3); 
	S = gsl_vector_alloc(3); 
	work = gsl_vector_alloc(3);

	// Find the centroid
	Vector3f centroid(0,0,0);
	for(const auto& g : ground_markers){ // Insert all weighted groundmarks in matrix
		centroid += g.relPosCart;
	}
	centroid /= (float)ground_m_size;

	// Insert all groundmarks in matrix after subtracting the centroid
	for(int i=0; i<ground_m_size; i++){ 
		gsl_matrix_set(A,i,0, ground_markers[i].relPosCart.x - centroid.x );
		gsl_matrix_set(A,i,1, ground_markers[i].relPosCart.y - centroid.y );
		gsl_matrix_set(A,i,2, ground_markers[i].relPosCart.z - centroid.z );
	} 

	// Singular value decomposition to find best fitting plane
	gsl_linalg_SV_decomp(A,V,S,work);

	// plane: ax + by + cz = d
	double a = gsl_matrix_get(V,0,2);
	double b = gsl_matrix_get(V,1,2);
	double c = gsl_matrix_get(V,2,2);

	/**
	 * Plane equation ax + by + cz = d
	 * therefore, considering the centroid(x,y,z) and normalvec(a,b,c):
	 *  d = ax + by + cz
	 *    = normalvec . centroid
	 */
	double d = a*centroid.x + b*centroid.y + c*centroid.z;

	//Note: |d| is an estimate of the agent's height, but we can do better by including aerial references in the prediction later

	gsl_matrix_free (A);
	gsl_matrix_free (V);
	gsl_vector_free (S);
	gsl_vector_free (work);

	/**
	 * Unfortunately, the normal vector doesn't always point up
	 * The plane is defined by (ax + by + cz - d = 0)
	 * Replacing [x,y,z] by a random point p, the sign of (ax + by + cz - d) gives us p's side
	 * Proof that the sign of (ax + by + cz - d) is positive for (rnd_ground_pt(i,j,k) + normal vector(a,b,c)):
	 *		ax + by + cz - d  =  
	*		a(i+a) + b(j+b) + c(k+c) - (a*i + b*j + c*k)  =
	*		ai + aa + bj + bb + ck + cc - ai - bj - ck  =
	*		aa + bb + cc  (which is always positive and equal to 1 because it is a unit vector) 
	* Since the agent is always above the field, we only need to make sure it's on the same side as the normal vector
	* To do that, we check if the origin of the relative coordinate system (agent's head) gives a positive sign for (ax + by + cz - d)
	* 		ax + by + cz - d  > 0   (replacing x=0, y=0, z=0)
	* 		d < 0
	* 
	* However, if the agent is lying down, the optimized plane may be slightly above its head due to vision errors
	* So, if we have a better reference point, such as a goal post, we use it
	*/

	if(!fd.list_landmarks_goalposts.empty()){ //If there are visible goal posts
		const Vector3f& aerialpt = fd.list_landmarks_goalposts.front().relPosCart; //random aerial point (goal post)
		if( a*aerialpt.x + b*aerialpt.y + c*aerialpt.z < d ){ //the goalpost is on the negative side, so we invert the normal vector
			a=-a;    b=-b;   c=-c;
		}
	}else{ //If there are no visible goal posts, we rely on the agent's head
		if(d > 0){// the normal vectors points down, so we invert it (we ignore 'd' from this point forward)
			a=-a;    b=-b;   c=-c;
		}
	}


	// save the ground plane's normal vector as the new z axis orientation vector
	prelimHeadToField.set(2,0,a);
	prelimHeadToField.set(2,1,b);
	prelimHeadToField.set(2,2,c);

	// compute the agent's height
	float h = max(  -centroid.x*a - centroid.y*b - centroid.z*c  ,0.064);
	prelimHeadToField.set(2,3, h ); 

	//Set public independent coordinate z (Not a problem: may be out of sync with transf matrix)
	last_z = final_z;
	final_z = h;
	_is_head_z_uptodate = true;

}

/**
 * Compute translation in z (height)
 * Note: Apparently there's no real benefit in involving goalposts (weighted or not), only when the
 * 		 visible objects are below 5/6, and even then the difference is minimal. 
 */
void LocalizerV2::find_z(const Vector3f& Zvec){

	Field& fd = SField::getInstance();

	Vector3f zsum;
	for(const auto& g: fd.list_weighted_ground_markers){
		zsum += g.relPosCart;
	}

	//Minimum height: 0.064m
	float z =  max(  -(zsum/fd.list_weighted_ground_markers.size()).innerProduct(Zvec)  ,0.064f);

	prelimHeadToField.set( 2,3,z ); 

	//Set public independent coordinate z (Not a problem: may be out of sync with transf matrix)
	last_z = final_z;
	final_z = z;
	_is_head_z_uptodate = true;

}


/**
 * Computes mapping error using distance probabilities
 * @return negative log of ["normalized" probability = (p1*p2*p3*...*pn)^(1/n)]
 */
double LocalizerV2::map_error_logprob(const gsl_vector *v, void *params){

	float angle;
	Field& fd = SField::getInstance();

	//Get angle from optimization vector, or from params (as a constant)
	if(v->size == 3){
		angle = gsl_vector_get(v,2);
	}else{
		angle = *(float *)params;
	}

	Matrix4D& transfMat = SLocalizerV2::getInstance().prelimHeadToField;
	Vector3f Zvec(transfMat.get(2,0), transfMat.get(2,1), transfMat.get(2,2));
	
	Vector3f Xvec, Yvec;
	fast_compute_XYvec_from_Zvec(Zvec, angle, Xvec, Yvec );

	//These are the transformation coefficients that are being optimized
	transfMat.set(0,0,Xvec.x);
	transfMat.set(0,1,Xvec.y);
	transfMat.set(0,2,Xvec.z);
	transfMat.set(0,3,gsl_vector_get(v, 0));
	transfMat.set(1,0,Yvec.x);
	transfMat.set(1,1,Yvec.y);
	transfMat.set(1,2,Yvec.z);
	transfMat.set(1,3,gsl_vector_get(v, 1));

	Matrix4D inverseTransMat = transfMat.inverse_tranformation_matrix();


	double total_logprob = 0;
	int total_err_cnt =0;
	
	//Add log probability of unknown markers (with known corresponding field segment)
	for(const auto& u : fd.list_unknown_markers){


		//We know the closest field segment, so we can bring it to the agent's frame
		Vector3f rel_field_s_start = inverseTransMat * u.fieldSeg->point[0]->get_vector();
		Vector3f rel_field_s_end =   inverseTransMat * u.fieldSeg->point[1]->get_vector();

		Line6f rel_field_s(rel_field_s_start, rel_field_s_end, u.fieldSeg->length); //Convert to Line6f

		Vector3f closest_polar_pt = rel_field_s.segmentPointClosestToCartPoint(u.relPosCart).toPolar();

		total_logprob += FieldNoise::log_prob_r(closest_polar_pt.x, u.relPosPolar.x);
		total_logprob += FieldNoise::log_prob_h(closest_polar_pt.y, u.relPosPolar.y);
		total_logprob += FieldNoise::log_prob_v(closest_polar_pt.z, u.relPosPolar.z);
		total_err_cnt++;
			
	}

	//Add log probability of known markers
	for(const auto& k : fd.list_known_markers){

		//Bring marker to agent's frame
		Vector3f rel_k = (inverseTransMat * k.absPos.get_vector()).toPolar();

		total_logprob += FieldNoise::log_prob_r(rel_k.x, k.relPosPolar.x);
		total_logprob += FieldNoise::log_prob_h(rel_k.y, k.relPosPolar.y);
		total_logprob += FieldNoise::log_prob_v(rel_k.z, k.relPosPolar.z);
		total_err_cnt++;

	}

	//return log of "normalized" probability = (p1*p2*p3*...*pn)^(1/n)
	//negative because the optimization method minimizes the loss function
	double logNormProb = -total_logprob / total_err_cnt; 

	if(!gsl_finite(logNormProb)) return 1e6; //fix

	return logNormProb;

}



/**
 * Computes mapping error using 2d euclidian distances
 * @return average distance
 */
double LocalizerV2::map_error_2d(const gsl_vector *v, void *params){

	float angle;
	Field& fd = SField::getInstance();

	//Get angle from optimization vector, or from params (as a constant)
	if(v->size == 3){
		angle = gsl_vector_get(v,2);
	}else{
		angle = *(float *)params;
	}

	Matrix4D& transfMat = SLocalizerV2::getInstance().prelimHeadToField;
	Vector3f Zvec(transfMat.get(2,0), transfMat.get(2,1), transfMat.get(2,2));
	
	Vector3f Xvec, Yvec;
	fast_compute_XYvec_from_Zvec(Zvec, angle, Xvec, Yvec );


	//These are the transformation coefficients that are being optimized
	transfMat.set(0,0,Xvec.x);
	transfMat.set(0,1,Xvec.y);
	transfMat.set(0,2,Xvec.z);
	transfMat.set(0,3,gsl_vector_get(v, 0));
	transfMat.set(1,0,Yvec.x);
	transfMat.set(1,1,Yvec.y);
	transfMat.set(1,2,Yvec.z);
	transfMat.set(1,3,gsl_vector_get(v, 1));


	float total_err = 0;
	int total_err_cnt =0;
	for(const Line6f& l : fd.list_segments){
 
		//Compute line absolute coordinates according to current transformation
		Vector3f ls = transfMat * l.startc; 
		Vector3f le = transfMat * l.endc; 

		//Compute line angle and establish a tolerance
		float l_angle = 0;
		float l_angle_tolerance = 10; //default full tolerance (no point in being larger than pi/2 but no harm either)

		if(l.length > 0.8){
			//This is the easy case: find the angle and establish a small tolerance (which allows some visual rotation attempts)
			l_angle = atan2f(le.y - ls.y, le.x - ls.x);
			if(l_angle < 0) { l_angle += 3.14159265f; } //this is a line, not a vector, so positive angles are enough
			l_angle_tolerance = 0.35f; //20 degrees

		} else if(fd.list_segments.size() <= 3) {
			//It gets to a point where the cost/benefit is not very inviting. If there are many lines (>3),
			//the small ones are not as decisive for the mapping error. Otherwise, we proceed:

			//If the small line is touching a big line, they have different orientations (it's a characteristic from the field lines) 

			for(const Line6f& lbig : fd.list_segments){
				if(lbig.length < 2 || &lbig == &l ) continue; //check if line is big and different from current

				if(lbig.segmentDistToSegment(l)<0.5){
					//this would only generate false positives with the halfway line and 4 ring lines (if enough vision error)
					//but even then, their orientation would be very different, so the method still holds

					//---------- get angle perpendicular to bigline (that is either the small line's angle, or at least close enough)

					//get bigline angle
					Vector3f lbigs = transfMat * lbig.startc; 
					Vector3f lbige = transfMat * lbig.endc; 
					l_angle = atan2f(lbige.y - lbigs.y, lbige.x - lbigs.x);

					// add 90deg while keeping the angle between 0-180deg (same logic used when l.length > 0.8)
					if     (l_angle < -1.57079632f){ l_angle += 4.71238898f; } //Q3 -> add pi*3/2
					else if(l_angle < 0           ){ l_angle += 1.57079632f; } //Q4 -> add pi/2
					else if(l_angle < 1.57079632f ){ l_angle += 1.57079632f; } //Q1 -> add pi/2
					else                           { l_angle -= 1.57079632f; } //Q2 -> subtract pi/2

					//This large tolerance means that this small line can be matched with almost everything except perpendicular lines
					l_angle_tolerance = 1.22f; //70 deg tolerance
					break; //end search for close big lines
				}
			}
		}


		//this default error of 1e6f is applied when there is no match (which means the transf. matrix's Xvec/Yvec are wrong)
		float min_err = 1e6f;
		for(const auto& s : Field::cFieldLineSegments::list){ //find distance to closest field line

			//Skip field line if seen line is substantially larger
			if( l.length > (s.length + 0.7) ){ continue; }

			//Skip field line if orientation does not match
			float angle_difference = fabsf(l_angle - s.angle);
			if(angle_difference > 1.57079632f) angle_difference = 3.14159265f - angle_difference;
			if(angle_difference > l_angle_tolerance) continue;
			
			//Error is the sum of the distance of a single line segment to both endpoints of seen line
			float err = Field::fieldLineSegmentDistToCart2DPoint(s,ls.to2d());
			if(err < min_err) err += Field::fieldLineSegmentDistToCart2DPoint(s,le.to2d());

			if(err < min_err) min_err = err;
		}

		total_err += min_err;
		total_err_cnt+=2; //a line has 2 points, double the weight of a single landmark

	}


	for(const Field::sMarker& m : fd.list_landmarks){

		Vector3f lpt = transfMat * m.relPosCart; //compute absolute coordinates according to current transformation

		float err = lpt.to2d().getDistanceTo(Vector(m.absPos.x,m.absPos.y));
		total_err += err > 0.5 ? err * 100 : err;
		total_err_cnt++;

	}

	double avg_error = total_err / total_err_cnt;

	if(!gsl_finite(avg_error)) return 1e6; //fix

	return avg_error; //return average error
}




/**
 * Apply fine tuning directly on the prelimHeadToField matrix
 * 1st - improve map fitting
 * 2nd - identify line segments and their endpoints
 * 3rd - fine tune again using known markers
 * @param initial_angle initial angle of Xvec around Zvec
 * @param initial_x initial translation in x
 * @param initial_y initial translation in y
 */
bool LocalizerV2::fine_tune(float initial_angle, float initial_x, float initial_y){

	Field& fd = SField::getInstance();

	//Statistics before fine tune
	counter_fineTune += stats_sample_position_error(Vector3f(initial_x,initial_y,prelimHeadToField.get(11)), world.my_cheat_abs_cart_pos, errorSum_fineTune_before);

	//Fine tune, changing the initial parameters directly
	if(!fine_tune_aux(initial_angle, initial_x, initial_y, false)) return false;
	
	//Statistics for 1st fine tune
	stats_sample_position_error(Vector3f(initial_x,initial_y,prelimHeadToField.get(11)), world.my_cheat_abs_cart_pos, errorSum_fineTune_euclidianDist);

	//Identify new markers 
	fd.update_from_transformation(prelimHeadToField);

	//Probabilistic fine tune
	fine_tune_aux(initial_angle, initial_x, initial_y, true);

	//Statistics for 2nd fine tune
	stats_sample_position_error(prelimHeadToField.toVector3f(), world.my_cheat_abs_cart_pos, errorSum_fineTune_probabilistic);
	
	//Update unknown markers absolute position based on refined transformation matrix
	fd.update_unknown_markers(prelimHeadToField);

	return true;
}


/**
 * Apply fine tuning:
 * - directly on: initial_angle, initial_x, initial_y (if use_probabilities == false)
 * - directly on the prelimHeadToField matrix using probabilities (if use_probabilities == true)
 * @param initial_angle initial angle of Xvec around Zvec
 * @param initial_x initial translation in x
 * @param initial_y initial translation in y
 */
bool LocalizerV2::fine_tune_aux(float &initial_angle, float &initial_x, float &initial_y, bool use_probabilities){

	int status, iter=0;
	gsl_vector* x =  create_gsl_vector<3>({initial_x, initial_y, initial_angle}); // Initial transformation 
	gsl_vector* ss = create_gsl_vector<3>({0.02, 0.02, 0.03});                    // Set initial step sizes 
	gsl_multimin_function minex_func = {map_error_2d, 3, nullptr};                // error func, variables no., params
	if(use_probabilities) minex_func.f = map_error_logprob;				          // probablity-based error function

	const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;   // algorithm type
	gsl_multimin_fminimizer *s = gsl_multimin_fminimizer_alloc (T, 3);            // allocate workspace
  	gsl_multimin_fminimizer_set (s, &minex_func, x, ss);                          // set workspace

	float best_x, best_y, best_ang;


  	do{
		iter++;
		status = gsl_multimin_fminimizer_iterate(s);

		//*s holds the best solution, not the last solution
		best_x = gsl_vector_get (s->x, 0);
		best_y = gsl_vector_get (s->x, 1);
		best_ang = gsl_vector_get (s->x, 2);

		if (status) break;

		double size = gsl_multimin_fminimizer_size (s); //minimizer-specific characteristic size
		status = gsl_multimin_test_size (size, 1e-3); //This size can be used as a stopping criteria, as the simplex contracts itself near the minimum

    }
	while ((status == GSL_CONTINUE || use_probabilities) && iter < 40);

	float best_map_error = s->fval;

	gsl_vector_free(x);
	gsl_vector_free(ss);
	gsl_multimin_fminimizer_free (s);

	if(!use_probabilities){
		if(best_map_error > 0.10){
			stats_change_state(FAILtune);
			return false;
		}else{
			initial_angle = best_ang;
			initial_x = best_x;
			initial_y = best_y;
			return true;
		}
	}

	/**
	 * At this point, use_probabilities is true
	 * Note: The transformations are directly tested on prelimHeadToField but it currently
	 * holds the last test, so we set it manually here to the best found solution
	 */

	//Convert angle into Xvec and Yvec
	Vector3f Zvec(prelimHeadToField.get(2,0), prelimHeadToField.get(2,1), prelimHeadToField.get(2,2));
	Vector3f Xvec, Yvec;
	fast_compute_XYvec_from_Zvec(Zvec, best_ang, Xvec, Yvec );

	prelimHeadToField.set(0,0, Xvec.x);
	prelimHeadToField.set(0,1, Xvec.y);
	prelimHeadToField.set(0,2, Xvec.z);
	prelimHeadToField.set(0,3, best_x);
	prelimHeadToField.set(1,0, Yvec.x);
	prelimHeadToField.set(1,1, Yvec.y);
	prelimHeadToField.set(1,2, Yvec.z);
	prelimHeadToField.set(1,3, best_y);

	return true;
}

/**
 * Find XY translation/rotation
 * A unique solution is guaranteed if Zvec points in the right direction
 * Requirement: 2 visible landmarks
 */
bool LocalizerV2::find_xy(){

	Field& fd = SField::getInstance();

	Vector3f Zvec(prelimHeadToField.get(2,0), prelimHeadToField.get(2,1), prelimHeadToField.get(2,2));

	Field::sMarker *m1 = nullptr, *m2 = nullptr;

	//Get as many corners as possible
	if(fd.list_landmarks_corners.size()>1){
		m1 = &fd.list_landmarks_corners[0];
		m2 = &fd.list_landmarks_corners[1];
	}else if(fd.list_landmarks_corners.size()==1){
		m1 = &fd.list_landmarks_corners[0];
		m2 = &fd.list_landmarks_goalposts[0];
	}else{
		m1 = &fd.list_landmarks_goalposts[0];
		m2 = &fd.list_landmarks_goalposts[1];
	}

	Vector3f realVec(m2->absPos.x - m1->absPos.x,  m2->absPos.y - m1->absPos.y,  m2->absPos.z - m1->absPos.z);
	float real_angle = atan2f(realVec.y, realVec.x); //angle of real vector

	Vector3f seenVec(m2->relPosCart - m1->relPosCart);
	Vector3f rotated_abs_vec = fast_rotate_around_ground_axis(seenVec, Zvec);
	float seen_angle = atan2f(rotated_abs_vec.y, rotated_abs_vec.x); //angle of real vector
	
	float AgentAngle = real_angle - seen_angle; //no normalization is needed

	
	Vector3f Xvec, Yvec;
	fast_compute_XYvec_from_Zvec(Zvec, AgentAngle, Xvec, Yvec );

	/**
	 * Let m be a landmark, rel:(mx,my,mz), abs:(mabsx, mabsy, mabsz)
	 * XvecX*mx + XvecY*my + XvecZ*mz + AgentX = mabsx
	 * AgentX = mabsx - (XvecX*mx + XvecY*my + XvecZ*mz)
	 * AgentX = mabsx - (XvecX . m)
	 * 
	 * Generalizing for N estimates:
	 * AgentX = sum( mabsx - (XvecX . m) )/N
	 */
	float initial_x = 0, initial_y = 0;
	for(const Field::sMarker& m : fd.list_landmarks){
		initial_x += m.absPos.x - Xvec.innerProduct(m.relPosCart);
		initial_y += m.absPos.y - Yvec.innerProduct(m.relPosCart);
	}

	initial_x /= fd.list_landmarks.size();
	initial_y /= fd.list_landmarks.size();


	return fine_tune(AgentAngle, initial_x, initial_y);

}

bool LocalizerV2::guess_xy(){
	Field& fd = SField::getInstance();

	//Get Zvec from previous steps
	Vector3f Zvec(prelimHeadToField.get(2,0), prelimHeadToField.get(2,1), prelimHeadToField.get(2,2));
	Vector last_known_position(head_position.x, head_position.y);

	//------------------------------------------------------------ Get longest line and use it as X or Y vector

	const Line6f* longestLine = &fd.list_segments.front();
	for(const Line6f& l : fd.list_segments){
		if(l.length > longestLine->length) longestLine = &l;
	}

	if(longestLine->length < 1.6){
		stats_change_state(FAILguessLine);
		return false; //largest line is too short, it could be mistaken for a ring line
	}

	//Rotate line to real ground plane, where it loses the 3rd dimension
	Vector3f longestLineVec = longestLine->endc - longestLine->startc;
	Vector3f rotated_abs_line = fast_rotate_around_ground_axis(longestLineVec, Zvec);

	//The line can be aligned with X or Y, positively or negatively (these angles don't need to be normalized) 
	float fixed_angle[4];
	fixed_angle[0] = -atan2f(rotated_abs_line.y,rotated_abs_line.x); //if longestLineVec is Xvec
	fixed_angle[1] = fixed_angle[0] + 3.14159265f; //if longestLineVec is -Xvec
	fixed_angle[2] = fixed_angle[0] + 1.57079633f; //if longestLineVec is Yvec
	fixed_angle[3] = fixed_angle[0] - 1.57079633f; //if longestLineVec is -Yvec

	//------------------------------------------------------------ Get initial translation

	//if we see 1 landmark, we use it, if not, we get the last position

	float initial_x[4], initial_y[4];
	bool noLandmarks = fd.list_landmarks.empty();

	if(noLandmarks){

		for(int i=0; i<4; i++){
			initial_x[i] = last_known_position.x;
			initial_y[i] = last_known_position.y;
		}

	} else {

		Vector3f Xvec = longestLineVec / longestLine->length;
		Vector3f Yvec(Zvec.crossProduct(Xvec));

		/**
		 * Let m be a landmark, rel:(mx,my,mz), abs:(mabsx, mabsy, mabsz)
		 * XvecX*mx + XvecY*my + XvecZ*mz + AgentX = mabsx
		 * AgentX = mabsx - (XvecX*mx + XvecY*my + XvecZ*mz)
		 * AgentX = mabsx - (XvecX . m)
		 */

		const Field::sMarker& m = fd.list_landmarks.front();
		const float x_aux = Xvec.innerProduct(m.relPosCart);
		const float y_aux = Yvec.innerProduct(m.relPosCart);

		initial_x[0] = m.absPos.x - x_aux;
		initial_y[0] = m.absPos.y - y_aux;
		initial_x[1] = m.absPos.x + x_aux; //2nd version: X is inverted
		initial_y[1] = m.absPos.y + y_aux; //2nd version: Y is inverted
		initial_x[2] = m.absPos.x + y_aux; //3rd version: X is inverted Y
		initial_y[2] = m.absPos.y - x_aux; //3rd version: Y is X
		initial_x[3] = m.absPos.x - y_aux; //4th version: X is Y
		initial_y[3] = m.absPos.y + x_aux; //4th version: Y is inverted X

	}
	

	//------------------------------------------------------------ Optimize XY rotation for each possible orientation


	const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;
	gsl_multimin_fminimizer *s[4] = {nullptr,nullptr,nullptr,nullptr};
	gsl_vector *ss[4], *x[4];
	gsl_multimin_function minex_func[4];

	size_t iter = 0;
	int status;
	double size;

	for(int i=0; i<4; i++){
		x[i]  = create_gsl_vector<2>({initial_x[i], initial_y[i]}); // Initial transformation 
		ss[i] = create_gsl_vector<2>({1, 1}); //Set initial step sizes to 1

		/* Initialize method */
		minex_func[i].n = 2;
		minex_func[i].f = map_error_2d;
		minex_func[i].params = &fixed_angle[i];	

		s[i] = gsl_multimin_fminimizer_alloc (T, 2);
  		gsl_multimin_fminimizer_set (s[i], &minex_func[i], x[i], ss[i]);
	}

	/* start iterating */
	bool running[4] = {true,true,true,true};
	float current_error[4] = {1e6,1e6,1e6,1e6};
	float lowest_error = 1e6;
	Vector best_xy[4];
	const int maximum_iterations = 50;
	bool plausible_solution[4] = {false,false,false,false};
  	do{
		iter++;
		for(int i=0; i<4; i++){
			if(!running[i]) continue;

			status = gsl_multimin_fminimizer_iterate(s[i]);

			current_error[i] = s[i]->fval;
			if(current_error[i] < lowest_error) lowest_error = current_error[i];

			// Possible errors: 
			// GSL_ERROR ("incompatible size of x", GSL_EINVAL); This should only be a concern during code design
			// GSL_ERROR ("contraction failed", GSL_EFAILED); Evaluation function produced non finite value
			if (status) { 
				running[i]=false; //This is not a valid solution
				continue; 
			}

			size = gsl_multimin_fminimizer_size (s[i]); //minimizer-specific characteristic size
			status = gsl_multimin_test_size (size, 1e-2); //This size can be used as a stopping criteria, as the simplex contracts itself near the minimum

			if(status != GSL_CONTINUE || (lowest_error * 50 < current_error[i])) { //finished or aborted
				best_xy[i].x = gsl_vector_get (s[i]->x, 0);
				best_xy[i].y = gsl_vector_get (s[i]->x, 1);
				running[i]=false; 
				plausible_solution[i]=(status == GSL_SUCCESS); //only valid if it converged to local minimum
				continue; 
			} 


		}	


    } while (iter < maximum_iterations && (running[0] || running[1] || running[2] || running[3]));

	for(int i=0; i<4; i++){
		gsl_vector_free(x[i]);
		gsl_vector_free(ss[i]);
		gsl_multimin_fminimizer_free (s[i]);
	}


	//At this point, a solution is plausible if it converged to a local minimum
	//So, we apply the remaining criteria for plausiblity
	int plausible_count = 0;
	int last_i;
	for(int i=0; i<4; i++){
		if(!plausible_solution[i]) continue;
		bool isDistanceOk = (!noLandmarks) || last_known_position.getDistanceTo(best_xy[i]) < 0.5; //  distance to last known position
		if(current_error[i] < 0.12 && isDistanceOk){ // mapping error  
			plausible_count++; 
			last_i = i;
		}
	}

	// If there is 1 landmark, and multiple options, the distance to last known pos is now used to eliminate candidates
	if(!noLandmarks && plausible_count>1){
		plausible_count = 0;
		for(int i=0; i<4; i++){
			if(plausible_solution[i] && last_known_position.getDistanceTo(best_xy[i]) < 0.5){ // distance to last known position
				plausible_count++; 
				last_i = i;
			}
		}
	}

	//Check if best solution is good if all others are not even plausible
	if(plausible_count==0){
		stats_change_state(FAILguessNone);
		return false; 
	}else if(plausible_count>1){
		stats_change_state(FAILguessMany);
		return false;
	}else if(current_error[last_i] > 0.06 || (noLandmarks && last_known_position.getDistanceTo(best_xy[last_i]) > 0.3)){ // mapping error  /  distance to last known position
		stats_change_state(FAILguessTest);
		return false;
	}

	return fine_tune(fixed_angle[last_i],best_xy[last_i].x, best_xy[last_i].y);
	
}



/**
 * Called to update every public variable (rotation + translation)
 */
void LocalizerV2::commit_everything(){

	final_headTofieldTransform = prelimHeadToField; //Full transformation (relative to absolute)

	final_headTofieldTransform.inverse_tranformation_matrix( final_fieldToheadTransform ); //Full transformation (absolute to relative)

	for(int i=0; i<3; i++){ 
		
		//Rotation  (relative to absolute)
		final_headTofieldRotate.set(i  ,final_headTofieldTransform.get(i  )); //Copy rotation line 1
		final_headTofieldRotate.set(i+4,final_headTofieldTransform.get(i+4)); //Copy rotation line 2
		final_headTofieldRotate.set(i+8,final_headTofieldTransform.get(i+8)); //Copy rotation line 3

		//Rotation  (absolute to relative)
		final_fieldToheadRotate.set(i  ,final_fieldToheadTransform.get(i  )); //Copy rotation line 1
		final_fieldToheadRotate.set(i+4,final_fieldToheadTransform.get(i+4)); //Copy rotation line 2
		final_fieldToheadRotate.set(i+8,final_fieldToheadTransform.get(i+8)); //Copy rotation line 3
	}

	final_translation = final_headTofieldTransform.toVector3f();

	_is_uptodate = true;

	_steps_since_last_update = 0;

	//Add current 3D position to history
	position_history[position_history_ptr++] = final_translation;
	if(position_history_ptr >= position_history.size()) position_history_ptr=0;

}


Vector3f LocalizerV2::relativeToAbsoluteCoordinates(const Vector3f relativeCoordinates) const{
	return headTofieldTransform * relativeCoordinates;
}

Vector3f LocalizerV2::absoluteToRelativeCoordinates(const Vector3f absoluteCoordinates) const{
	return fieldToheadTransform * absoluteCoordinates;
}



/**
 * Reset preliminary matrix with a constant for the first 3 rows
 */
void LocalizerV2::prelim_reset(){
	//Since it was initialized as identity matrix we never need to change the last row
	for(int i=0; i<12; i++){
		prelimHeadToField.set(i,9999); //flag unchanged cells with a constant
	}
}

//=================================================================================================
//=============================================================================== useful statistics
//=================================================================================================

Vector3f LocalizerV2::get_velocity(unsigned int n) const{
	//assert(n > 0 && n < position_history.size() && "LocalizerV2::get_velocity(unsigned int n) -> n must be between 1 and 9!");

	int l = position_history.size() - 1;

	Vector3f current_pos = position_history[(position_history_ptr + l)     % position_history.size()];
	Vector3f last_pos    = position_history[(position_history_ptr + l - n) % position_history.size()];
	
	return current_pos - last_pos;
}

//=================================================================================================
//================================================================================ debug statistics
//=================================================================================================


void LocalizerV2::print_report() const{

	if(counter_fineTune == 0){
		cout << "LocalizerV2 Report - Check if the server is providing cheat data.\n";
		return;
	}

	if(counter_fineTune < 2) return; //otherwise, c-1=0
	const int &c = counter_fineTune;
	const int &cb = counter_ball;
	const int c1 = c-1;
	const int cb1 = cb-1;

	const double* ptr = errorSum_fineTune_before;
	float e1_2d_var = (ptr[4] - (ptr[3]*ptr[3]) / c) / c1;
	float e1_3d_var = (ptr[6] - (ptr[5]*ptr[5]) / c) / c1;
	float e1[] = { ptr[3]/c, sqrt(e1_2d_var), ptr[5]/c, sqrt(e1_3d_var), ptr[0]/c, ptr[1]/c, ptr[2]/c };

	ptr = errorSum_fineTune_euclidianDist;
	float e2_2d_var = (ptr[4] - (ptr[3]*ptr[3]) / c) / c1;
	float e2_3d_var = (ptr[6] - (ptr[5]*ptr[5]) / c) / c1;
	float e2[] = { ptr[3]/c, sqrt(e2_2d_var), ptr[5]/c, sqrt(e2_3d_var), ptr[0]/c, ptr[1]/c, ptr[2]/c };

	ptr = errorSum_fineTune_probabilistic;
	float e3_2d_var = (ptr[4] - (ptr[3]*ptr[3]) / c) / c1;
	float e3_3d_var = (ptr[6] - (ptr[5]*ptr[5]) / c) / c1;
	float e3[] = { ptr[3]/c, sqrt(e3_2d_var), ptr[5]/c, sqrt(e3_3d_var), ptr[0]/c, ptr[1]/c, ptr[2]/c };

	ptr = errorSum_ball;
	float e4_2d_var=0, e4_3d_var=0;
	if(cb1 > 0){
		e4_2d_var = (ptr[4] - (ptr[3]*ptr[3]) / cb) / cb1;
		e4_3d_var = (ptr[6] - (ptr[5]*ptr[5]) / cb) / cb1;
	}
	float e4[] = { ptr[3]/cb, sqrt(e4_2d_var), ptr[5]/cb, sqrt(e4_3d_var), ptr[0]/cb, ptr[1]/cb, ptr[2]/cb };

	const int* st = state_counter;
	printf("---------------------------------- LocalizerV2 Report ----------------------------------\n");
	printf("SAMPLING STAGE              2D-MAE  2D-STD  3D-MAE  3D-STD   x-MBE    y-MBE    z-MBE\n");
	printf("Before fine-tune:           %.4f  %.4f  %.4f  %.4f  %7.4f  %7.4f  %7.4f\n",   e1[0],e1[1],e1[2],e1[3],e1[4],e1[5],e1[6]);
	printf("After Euclidian dist. fit:  %.4f  %.4f  %.4f  %.4f  %7.4f  %7.4f  %7.4f\n",   e2[0],e2[1],e2[2],e2[3],e2[4],e2[5],e2[6]);
	printf("After probabilistic fit:    %.4f  %.4f  %.4f  %.4f  %7.4f  %7.4f  %7.4f\n",   e3[0],e3[1],e3[2],e3[3],e3[4],e3[5],e3[6]);
	printf("Ball:                       %.4f  %.4f  %.4f  %.4f  %7.4f  %7.4f  %7.4f\n\n", e4[0],e4[1],e4[2],e4[3],e4[4],e4[5],e4[6]);
	printf("* MBE(Mean Bias Error) MAE(Mean Abs Error) STD(Standard Deviation)\n");
	printf("* Note: the cheat positions should be active in server (preferably with >2 decimal places)\n\n");
	printf("------------------LocalizerV2::run calls analysis:\n");
	printf("- Total:               %i \n", st[RUNNING]);
	printf("- Successful:          %i \n", st[DONE]);
	printf("- Blind agent:         %i \n", st[BLIND]);
	printf("- Almost blind:        %i \n", st[MINFAIL] + st[FAILzNOgoal] + st[FAILzLine] + st[FAILz]);
	printf("- Guess location fail: %i \n", st[FAILguessLine] + st[FAILguessNone] + st[FAILguessMany] + st[FAILguessTest]);
	printf("--- Lines too short:   %i \n", st[FAILguessLine]);
	printf("--- No solution:       %i \n", st[FAILguessNone]);
	printf("--- >1 solution:       %i \n", st[FAILguessMany]);
	printf("--- Weak solution:     %i \n", st[FAILguessTest]);
	printf("- Eucl. tune fail:     %i \n", st[FAILtune]); //Euclidian distance tune error above 6cm
	printf("----------------------------------------------------------------------------------------\n");

}

void LocalizerV2::stats_reset(){

	counter_fineTune = 0;
	for(int i=0; i<sizeof(errorSum_fineTune_before)/sizeof(errorSum_fineTune_before[0]); i++){
		errorSum_fineTune_before[i] = 0;
		errorSum_fineTune_euclidianDist[i] = 0;
		errorSum_fineTune_probabilistic[i] = 0;
	}

	for(int i=0; i<STATE::ENUMSIZE; i++){
		state_counter[i] = 0;
	}

}

void LocalizerV2::stats_change_state(enum STATE s){

	state_counter[s]++;
	state = s;

}

/**
 * Take a sample of the current position error 
 * @param position estimated agent's position
 * @param cheat actual position provided by server
 * @param error_placeholder variable where result is stored
 */
int LocalizerV2::stats_sample_position_error(const Vector3f sample, const Vector3f& cheat, double error_placeholder[]){

	if(world.my_cheat_abs_cart_pos == Vector3f(0,0,0)) return 0;

	double x_err = sample.x - cheat.x;
	double y_err = sample.y - cheat.y;
	double z_err = sample.z - cheat.z;
	double xx_err = x_err * x_err; 
	double yy_err = y_err * y_err;
	double zz_err = z_err * z_err;

	error_placeholder[0] += x_err;
	error_placeholder[1] += y_err;
	error_placeholder[2] += z_err;

	double sq_err_2d = xx_err + yy_err;
	error_placeholder[3] += sqrt(sq_err_2d);
	error_placeholder[4] += sq_err_2d;

	double sq_err_3d = xx_err + yy_err + zz_err;
	error_placeholder[5] += sqrt(sq_err_3d);
	error_placeholder[6] += sq_err_3d;

	return 1;
}

