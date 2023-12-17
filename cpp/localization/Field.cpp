#include "Field.h"
#include "RobovizLogger.h"
#include "World.h"


static World& world = SWorld::getInstance();

//=================================================================================================
//=========================================================================== constexpr definitions
//=================================================================================================

decltype(Field::cRingLineLength) constexpr Field::cRingLineLength;
decltype(Field::cPenaltyBoxDistX) constexpr Field::cPenaltyBoxDistX;
decltype(Field::cHalfPenaltyWidth) constexpr Field::cHalfPenaltyWidth;
decltype(Field::cHalfGoalWidth) constexpr Field::cHalfGoalWidth;
decltype(Field::cHalfFielfLength) constexpr Field::cHalfFielfLength;
decltype(Field::cGoalWidth) constexpr Field::cGoalWidth;
decltype(Field::cGoalDepth) constexpr Field::cGoalDepth;
decltype(Field::cGoalHeight) constexpr Field::cGoalHeight;
decltype(Field::cFieldLength) constexpr Field::cFieldLength;
decltype(Field::cFieldWidth) constexpr Field::cFieldWidth;
decltype(Field::cPenaltyLength) constexpr Field::cPenaltyLength;
decltype(Field::cPenaltyWidth) constexpr Field::cPenaltyWidth;
decltype(Field::cFieldLineSegments::list) constexpr Field::cFieldLineSegments::list;
decltype(Field::cFieldPoints::list) constexpr Field::cFieldPoints::list;

//non-constexpr definitions
decltype(Field::list_8_landmarks::list) Field::list_8_landmarks::list;

//=================================================================================================
//=============================================================================== Drawing utilities
//=================================================================================================


/**
 * Draw estimates of all visible lines, markers, self position and ball 
 * */

void Field::draw_visible(const Matrix4D& headToFieldT, bool is_right_side) const{

    if(is_right_side){
        return draw_visible_switch(headToFieldT);
    }

    string draw_name = "localization";

    RobovizLogger* roboviz = RobovizLogger::Instance();
    roboviz->init(); // only initialized when draw_visible is called (only happens once)
    
    //--------------------------------- Print all lines, whether they were identified or not
    for(const Line6f& l : list_segments) {

        Vector3f s = headToFieldT * l.startc;
        Vector3f e = headToFieldT * l.endc;

        roboviz->drawLine(s.x, s.y, s.z, e.x, e.y, e.z, 1, 0.8,0,0, &draw_name);
    }  

    //--------------------------------- Print identified line segments, with their fixed abs coordinates

    for(const auto& s : list_known_segments){

		Vector3f mid = Vector3f::determineMidpoint(s.point[0].absPos.get_vector(), s.point[1].absPos.get_vector());
        
        string line_name(s.fieldSegment->name);
		roboviz->drawAnnotation(&line_name,mid.x,mid.y,mid.z, 0,1,0,&draw_name);
        roboviz->drawLine(s.point[0].absPos.x, s.point[0].absPos.y, s.point[0].absPos.z, 
                          s.point[1].absPos.x, s.point[1].absPos.y, s.point[1].absPos.z, 3, 0,0.8,0, &draw_name);
	}

    for(const auto& m : list_known_markers){
        string line_name(m.fieldPt->name);
		roboviz->drawAnnotation(&line_name,m.absPos.x, m.absPos.y, m.absPos.z+1, 1,0,0,&draw_name);
        roboviz->drawLine(m.absPos.x, m.absPos.y, m.absPos.z, 
                          m.absPos.x, m.absPos.y, m.absPos.z+0.5, 1, 0.8,0.8,0.8, &draw_name);
	}
    for(const auto& m : list_unknown_markers){
        string line_name = "!";
		roboviz->drawAnnotation(&line_name,m.absPos.x, m.absPos.y, m.absPos.z+1, 1,0,0,&draw_name);
        roboviz->drawLine(m.absPos.x, m.absPos.y, m.absPos.z, 
                          m.absPos.x, m.absPos.y, m.absPos.z+0.5, 1, 0.8,0.8,0.8, &draw_name);
	}

    //--------------------------------- Draw player and ball arrows

    Vector3f me = headToFieldT.toVector3f();

    roboviz->drawLine(me.x, me.y, me.z, me.x,     me.y, me.z+0.5, 2,1,0,0,&draw_name);
    roboviz->drawLine(me.x, me.y, me.z, me.x-0.2, me.y, me.z+0.2, 2,1,0,0,&draw_name);
    roboviz->drawLine(me.x, me.y, me.z, me.x+0.2, me.y, me.z+0.2, 2,1,0,0,&draw_name);
    
    //There is no need to draw the ball position here (but it works well)

    /*static Vector3f last_known_ball_pos = Vector3f();
    if(world.ball_seen){
        last_known_ball_pos = headToFieldT * world.ball_rel_pos_cart;
    }

    Vector3f &b = last_known_ball_pos;

    roboviz->drawLine(b.x, b.y, b.z, b.x,     b.y, b.z+0.5, 2,1,1,0,&draw_name);
    roboviz->drawLine(b.x, b.y, b.z, b.x-0.2, b.y, b.z+0.2, 2,1,1,0,&draw_name);
    roboviz->drawLine(b.x, b.y, b.z, b.x+0.2, b.y, b.z+0.2, 2,1,1,0,&draw_name);*/

    roboviz->swapBuffers(&draw_name);


}

/**
 * Draw estimates of all visible lines, markers, self position and ball, but switch field sides
 * */

void Field::draw_visible_switch(const Matrix4D& headToFieldT) const{

    string draw_name = "localization";

    RobovizLogger* roboviz = RobovizLogger::Instance();
    roboviz->init(); // only initialized when draw_visible is called (only happens once)
    
    //--------------------------------- Print all lines, whether they were identified or not
    for(const Line6f& l : list_segments) {

        Vector3f s = headToFieldT * l.startc;
        Vector3f e = headToFieldT * l.endc;

        roboviz->drawLine(-s.x, -s.y, s.z, -e.x, -e.y, e.z, 1, 0.8,0,0, &draw_name);
    }  

    //--------------------------------- Print identified line segments, with their fixed abs coordinates

    for(const auto& s : list_known_segments){

		Vector3f mid = Vector3f::determineMidpoint(s.point[0].absPos.get_vector(), s.point[1].absPos.get_vector());
        
        string line_name(s.fieldSegment->name);
		roboviz->drawAnnotation(&line_name,-mid.x,-mid.y,mid.z, 0,1,0,&draw_name);
        roboviz->drawLine(-s.point[0].absPos.x, -s.point[0].absPos.y, s.point[0].absPos.z, 
                          -s.point[1].absPos.x, -s.point[1].absPos.y, s.point[1].absPos.z, 3, 0,0.8,0, &draw_name);
	}

    for(const auto& m : list_known_markers){
        string line_name(m.fieldPt->name);
		roboviz->drawAnnotation(&line_name,-m.absPos.x, -m.absPos.y, m.absPos.z+1, 1,0,0,&draw_name);
        roboviz->drawLine(-m.absPos.x, -m.absPos.y, m.absPos.z, 
                          -m.absPos.x, -m.absPos.y, m.absPos.z+0.5, 1, 0.8,0.8,0.8, &draw_name);
	}
    for(const auto& m : list_unknown_markers){
        string line_name = "!";
		roboviz->drawAnnotation(&line_name,-m.absPos.x, -m.absPos.y, m.absPos.z+1, 1,0,0,&draw_name);
        roboviz->drawLine(-m.absPos.x, -m.absPos.y, m.absPos.z, 
                          -m.absPos.x, -m.absPos.y, m.absPos.z+0.5, 1, 0.8,0.8,0.8, &draw_name);
	}

    //--------------------------------- Draw player and ball arrows

    Vector3f me = headToFieldT.toVector3f();

    roboviz->drawLine(-me.x, -me.y, me.z, -me.x,     -me.y, me.z+0.5, 2,1,0,0,&draw_name);
    roboviz->drawLine(-me.x, -me.y, me.z, -me.x+0.2, -me.y, me.z+0.2, 2,1,0,0,&draw_name);
    roboviz->drawLine(-me.x, -me.y, me.z, -me.x-0.2, -me.y, me.z+0.2, 2,1,0,0,&draw_name);
    
    //There is no need to draw the ball position here (but it works well)

    /*static Vector3f last_known_ball_pos = Vector3f();
    if(world.ball_seen){
        last_known_ball_pos = headToFieldT * world.ball_rel_pos_cart;
    }

    Vector3f &b = last_known_ball_pos;

    roboviz->drawLine(b.x, b.y, b.z, b.x,     b.y, b.z+0.5, 2,1,1,0,&draw_name);
    roboviz->drawLine(b.x, b.y, b.z, b.x-0.2, b.y, b.z+0.2, 2,1,1,0,&draw_name);
    roboviz->drawLine(b.x, b.y, b.z, b.x+0.2, b.y, b.z+0.2, 2,1,1,0,&draw_name);*/

    roboviz->swapBuffers(&draw_name);


}


//=================================================================================================
//==================================================================== Refresh / Identify / Collect
//=================================================================================================


/**
 * Gather markers with known absolute z: line endpoints + foot contact points + [toe contact points]
 **/
void Field::gather_ground_markers(){

    /**
     * Add NAO's feet ground contact points to zmarks
     * Dependency: the agent's feet must be touching the ground
     * Flaws: 
     * - if the feet are touching other players or the ball (may be solved if it is problematic)
     * - robot 4 may be touching the ground with its toes and they are currently ignored
     **/

	for(int i=0; i<2; i++){
		if( world.foot_touch[i] ){ //if this foot is touching the ground

			//Vector3f contactAux = world.foot_contact_pt[i]; //contact point using strange coordinate system
			//Vector3f contactpt = Vector3f(contactAux.y,-contactAux.x,contactAux.z); // fix coordinate system for both feet

            Vector3f relPos = world.foot_contact_rel_pos[i];
            list_feet_contact_points.emplace_back( sVector3d({0,0,0}), relPos.toPolar(), relPos);
            list_ground_markers.emplace_back( sVector3d({0,0,0}), relPos.toPolar(), relPos);
		}
	}

    //Deactivated since it did not produce better results, even when both feet are floating (it was only better when the robot fell)
    /*const Types::BodyParts toePart[2] = {Types::ilLToe, Types::ilRToe};
    for(int i=0; i<2; i++){
		if( agent::model->getToeTouch(feetSide[i]) ){ //if this foot is touching the ground

			Vector3f contactAux = agent::model->getToeContact(feetSide[i]); //contact point using strange coordinate system
			Vector3f contactpt = Vector3f(contactAux.y,-contactAux.x,contactAux.z); // fix coordinate system for both feet
            Vector3f relPos = agent::model->getRelPositionOfBodyPoint(agent::model->getBodyPart(toePart[i]),contactpt);

            if(agent::model->getFootTouch(Types::iLeft) == false && agent::model->getFootTouch(Types::iRight) == false){
                zmarks.emplace_back( 0,0,0,relPos );
            }
		}
	}*/


    //Add all line endings to ground markers
    for(const Line6f& l : list_segments){
        list_ground_markers.emplace_back( sVector3d({0,0,0}), l.startp, l.startc);
        list_ground_markers.emplace_back( sVector3d({0,0,0}), l.endp, l.endc);
    }

    non_collinear_ground_markers = list_ground_markers.size(); //Excludes corner flags

    //Add corner flags
    for(const auto& c : list_landmarks_corners){
        list_ground_markers.emplace_back( sVector3d({0,0,0}), c.relPosPolar, c.relPosCart);
    }
    

    /**
     * All the polar coordinates' errors are dependent on the distance
     * Var[distance error] = Var[ed*d/100] + Var[er]            (ed-error distance, er-error rounding)
     * Var[distance error] = (d/100)^2 * Var[ed] + Var[er] 
     * Their importance will be given by Inverse-variance weighting
     * 
     * Application:
     * repetition = max(int(k*(1/var)),1), where k=1/1500
     * repetitions for    1 meter:  71
     * repetitions for    2 meters: 55
     * repetitions for >=19 meters:  1
     */

    for(const auto& g : list_ground_markers){
        float var = pow(g.relPosPolar.x / 100.f,2) * var_distance + var_round_hundredth;
        float w = 1.f/(1500.f*var); //weight = (1/var)*k   where k is a constant to transform the large weights into number of repetitions
        int repetitions = max(int(w),1);

        list_weighted_ground_markers.insert(list_weighted_ground_markers.end(), repetitions, g);
    }
}


/**
 * Update markers after visual step
 * Marks attributes: abs(x,y,z), rel(r,h,v)
 * Possible markers:
 *  - 8 landmarks
 *  - line endings:
 *      - 8  @ field corners
 *      - 12 @ penalty box corners
 *      - 20 @ center ring
 *      - 2  @ halfway line
 *  - 8 noisy estimates from corner-originated lines 
 * */
void Field::update(){
   
    //no need to reserve space since these vectors will expand mostly in the first cycles
    list_segments.clear();
    list_landmarks.clear();
    list_landmarks_corners.clear();
    list_landmarks_goalposts.clear();
    list_feet_contact_points.clear();
    list_known_markers.clear();
    list_unknown_markers.clear();
    list_known_segments.clear();
    list_ground_markers.clear();
    list_weighted_ground_markers.clear();

    //----------------------------------------- Pre-processing: prepare landmark lists

    for(int i=0; i<8; i++){
        sFixedMarker *l8;
        const sFieldPoint *fp;
        World::sLMark *l = &world.landmark[i];
        if     (l->pos.x == -15 && l->pos.y == -10) {l8 = &list_8_landmarks::_corner_mm; fp = &cFieldPoints::corner_mm;}
        else if(l->pos.x == -15 && l->pos.y == +10) {l8 = &list_8_landmarks::_corner_mp; fp = &cFieldPoints::corner_mp;}
        else if(l->pos.x == +15 && l->pos.y == -10) {l8 = &list_8_landmarks::_corner_pm; fp = &cFieldPoints::corner_pm;}
        else if(l->pos.x == +15 && l->pos.y == +10) {l8 = &list_8_landmarks::_corner_pp; fp = &cFieldPoints::corner_pp;}
        else if(l->pos.x == -15 && l->pos.y < 0)    {l8 = &list_8_landmarks::_goal_mm;   fp = &cFieldPoints::goal_mm;  }
        else if(l->pos.x == -15 && l->pos.y > 0)    {l8 = &list_8_landmarks::_goal_mp;   fp = &cFieldPoints::goal_mp;  }
        else if(l->pos.x == +15 && l->pos.y < 0)    {l8 = &list_8_landmarks::_goal_pm;   fp = &cFieldPoints::goal_pm;  }
        else if(l->pos.x == +15 && l->pos.y > 0)    {l8 = &list_8_landmarks::_goal_pp;   fp = &cFieldPoints::goal_pp;  }
        else{ return; } 

        if(l->seen){   
            l8->set_relPos(l->rel_pos);
            l8->visible = true;

            sMarker seen_mark(fp, l->rel_pos);
            list_landmarks.push_back(seen_mark);
            list_known_markers.push_back(seen_mark);

            if (l->isCorner){ list_landmarks_corners.push_back(  seen_mark); }
            else { list_landmarks_goalposts.push_back(seen_mark); }
        }else{
            l8->visible = false;
        }
    }

    //----------------------------------------- Pre-processing: prepare lines and landmarks' coordinates sign

    for(const auto& l : world.lines_polar) {
        list_segments.emplace_back(l.start, l.end); 
    }

    //----------------------------------------- Gather markers with known absolute z: line endpoints + foot contact points

    gather_ground_markers();

}




    



void Field::update_from_transformation(const Matrix4D& tmatrix){

    /**
     * Identify segments based on transformation matrix
     * 
     * Identification starts from longest to shortest line
     * The visible line segment is identified if there is only 1 close field line
     * If there is more than 1 close field line and all but 1 were already taken, it is still identified 
     */

    //----------------------------------------------- get lines ordered from largest to shortest
    vector<Line6f*> lines_descending_length;
    for(auto& l : list_segments){
        lines_descending_length.push_back(&l);
    }

    //Sort from largest to smallest radius
    sort(lines_descending_length.begin(),lines_descending_length.end(), 
        [](const Line6f* a, const Line6f* b) { return (a->length > b->length); });

    //----------------------------------------------- identify lines

    for(const Line6f* l : lines_descending_length){
        Vector3f l_abs[2] = {tmatrix * l->startc, tmatrix * l->endc}; 

        float l_angle = atan2f(l_abs[1].y - l_abs[0].y, l_abs[1].x - l_abs[0].x);

        const float min_err = 0.3; //maximum allowed distance (startdist + enddist < 0.3m)
        const sFieldSegment* best_line = nullptr;
		for(const auto& s : cFieldLineSegments::list){ //find distance to closest field line

			//Skip field line if seen line is substantially larger
			if( l->length > (s.length + 0.7) ){ continue; }

			//Skip field line if orientation does not match
            float line_angle_difference = normalize_line_angle_rad(l_angle - s.angle);
			if(line_angle_difference > 0.26) continue; //tolerance 15deg

            //Skip field line if it was already identified
            bool already_identified = false;
            for(const auto& k : list_known_segments){
                if(k.fieldSegment == &s){ already_identified=true; break; }
            }
            if(already_identified) continue;
			
			//Error is the sum of the distance of a single line segment to both endpoints of seen line
			float err = fieldLineSegmentDistToCart2DPoint(s,l_abs[0].to2d());
			if(err < min_err) err += fieldLineSegmentDistToCart2DPoint(s,l_abs[1].to2d());

			if(err < min_err){
                if(best_line == nullptr){ best_line = &s; } //Save the field line for now (others may emerge)
                else{
                    best_line = nullptr; //Two close field lines, none of which was taken yet, so abort
                    break;
                }
            } 
		}

        if(best_line != nullptr){

            //-------------- Fix the seen line's start<->end order to match the corresponding field line

            int l_index[2] = {0,1}; //line index of [0]start and [1]end points

            if(normalize_vector_angle_rad(l_angle - best_line->angle) > 1.57079633f){ // they point in opposite directions
                l_index[0] = 1;
                l_index[1] = 0;
            }

            const Vector3f *lAbs[2]  = {&l_abs[l_index[0]],           &l_abs[l_index[1]]};
            const Vector3f *lRelP[2] = {&l->get_polar_pt(l_index[0]), &l->get_polar_pt(l_index[1])};
            const Vector3f *lRelC[2] = {&l->get_cart_pt(l_index[0]),  &l->get_cart_pt(l_index[1])};

            //-------------- Fix the absolute coordinates with field information

            bool isInFoV[2] = {false,false};

            //1st: recognize endpoints as field points (& save known markers)

            /**
             * //---------------------------------------------- General solution
             * All points reasonably within the FoV are identified
             * Noise applied horizontally sigma=0.1225,  Pr[-0.5<x<0.5]=0.99996
             * Noise applied vertically   sigma=0.1480,  Pr[-0.5<x<0.5]=0.99928
             * 
             * Warning: the server limits the focal distance to 10cm (which is enforced in the x cartesian coordinate)
             *          current solution: lRelC[i].x > 0.2
             * 
             * Warning 2: sometimes the error of phi and theta is larger than expected, producing points like 
             *            (rel polar: 0.57,-36.36,-54.41) (rel cart: 0.267,-0.1967,-0.4635)
             *            which goes with the theory that the FoV is actually defined as a cone (a bit unrealistic though)
             *            current solution: cone_angle < hor_FoV-5
             */

            for( int i=0; i<2; i++){
                float cone_angle = acosf(lRelC[i]->x / lRelP[i]->x); //angle between vector and (1,0,0)
                const float max_cone_angle = (cHalfHorizontalFoV-5)*M_PI/180;

                if(cone_angle < max_cone_angle && lRelC[i]->x > 0.2){
                    list_known_markers.emplace_back(best_line->point[i], *lRelP[i], *lRelC[i]);
                    isInFoV[i] = true;
                }
            }

            //2nd: use real coordinates if point was recognized, otherwise push it to a valid position (& save segment and unknown markers)
            
            const Line6f field_line(best_line->point[0]->get_vector(), best_line->point[1]->get_vector(), best_line->length);

            sVector3d l_pt_d[2]; //final line segment points (double precision floating-points)

            for( int i=0; i<2; i++){
                if(isInFoV[i]){
                    l_pt_d[i] = best_line->point[i]->pt; //set recognized point's abs coordinates
                }else{
                    Vector3f p = field_line.segmentPointClosestToCartPoint(*lAbs[i]); //push point to closest valid position
                    l_pt_d[i].set(p); //set unknown point's estimated coordinates
                    list_unknown_markers.emplace_back(best_line, l_pt_d[i], *lRelP[i], *lRelC[i]);
                }
            }

            //-------------- Save identified line segment
            list_known_segments.emplace_back(sMarker(l_pt_d[0],*lRelP[0],*lRelC[0]),sMarker(l_pt_d[1],*lRelP[1],*lRelC[1]), l->length, best_line);

        }
    }
}

void Field::update_unknown_markers(const Matrix4D& tmatrix){

    for(auto& u : list_unknown_markers){

        //Transform marker to world frame
        Vector3f raw_abs_pos = tmatrix * u.relPosCart;

        //Push marker to existing field segment
        const Line6f field_seg( u.fieldSeg->point[0]->get_vector(),  u.fieldSeg->point[1]->get_vector(),  u.fieldSeg->length);
        Vector3f fixed_abs_pos = field_seg.segmentPointClosestToCartPoint(raw_abs_pos); //push point to closest valid position

        u.absPos = sVector3d({fixed_abs_pos.x, fixed_abs_pos.y, fixed_abs_pos.z});
    }
}


//=================================================================================================
//================================================================================== Math utilities
//=================================================================================================


/**
 * Field lines are on the ground (z=0), so the method is simplified
 */
float Field::fieldLineSegmentDistToCartPoint(const sFieldSegment& fLine, const Vector3f& cp){

	//Line segment vector (start -> end)
	float vx = fLine.point[1]->pt.x - fLine.point[0]->pt.x; 
	float vy = fLine.point[1]->pt.y - fLine.point[0]->pt.y;

	Vector3f w1(cp.x - fLine.point[0]->pt.x, cp.y - fLine.point[0]->pt.y, cp.z); // vector: (segment start -> point)
	if (w1.x * vx + w1.y * vy <= 0)  
		return w1.length();// if angle between vectors is >=90deg, we return the distance to segment start

	Vector3f w2(cp.x - fLine.point[1]->pt.x, cp.y - fLine.point[1]->pt.y, cp.z); // vector: (segment end -> point)
	if (w2.x * vx + w2.y * vy >= 0)  
		return w2.length(); //if angle between vectors is <=90deg, we return the distance to segment end

	Vector3f v_cross_w1(vy * w1.z, - vx * w1.z, vx * w1.y - vy * w1.x);
	return v_cross_w1.length() / fLine.length; //distance line to point (area of parallelogram divided by base gives height)

}

/**
 * Field lines are on the ground (z=0), so the method is simplified
 */
float Field::fieldLineSegmentDistToCart2DPoint(const sFieldSegment& fLine, const Vector& cp){

	const Vector segment_start(fLine.point[0]->pt.x, fLine.point[0]->pt.y);
	const Vector segment_end(  fLine.point[1]->pt.x, fLine.point[1]->pt.y  );

	//Line segment vector (start -> end)
	Vector v(segment_end-segment_start);

	Vector w1(cp - segment_start);// vector: (segment start -> point)

	if(w1.innerProduct(v) <= 0)
		return w1.length();// if angle between vectors is >=90deg, we return the distance to segment start

	Vector w2(cp - segment_end); // vector: (segment end -> point)
	if(w2.innerProduct(v) >= 0)  
		return w2.length(); //if angle between vectors is <=90deg, we return the distance to segment end

	return fabsf(v.crossProduct(w1)) / fLine.length; //distance line to point (area of parallelogram divided by base gives height)
}