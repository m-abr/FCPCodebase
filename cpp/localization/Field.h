/**
 * FILENAME:     Field
 * DESCRIPTION:  Field map
 * AUTHOR:       Miguel Abreu (m.abreu@fe.up.pt)
 * DATE:         2021
 */

#pragma once
#include "Vector3f.h"
#include "Singleton.h"
#include "Matrix4D.h"
#include "Line6f.h"
#include <vector>
#include <array>

using namespace std;


class Field {
    friend class Singleton<Field>;

private:

    Field(){};
    void gather_ground_markers();

public:

//=================================================================================================
//====================================================================================== Structures
//=================================================================================================

    struct sVector3d {
        double x,y,z;

        //sVector3d(const Vector3f& v) : x(v.x), y(v.y), z(v.z) {} 

        Vector3f get_vector() const {
            return Vector3f(x,y,z);
        }

        void set(const sVector3d &pt){
            x=pt.x; y=pt.y; z=pt.z;
        }

        void set(const Vector3f &pt){
            x=pt.x; y=pt.y; z=pt.z;
        }

        float dist(const Vector3f &other) const{
            float dx = x-other.x;
            float dy = y-other.y;
            float dz = z-other.z;
            return sqrtf(dx*dx+dy*dy+dz*dz);
        }

    };

    struct sFieldPoint {
        const sVector3d pt;
        const char name[10];

        Vector3f get_vector() const {
            return Vector3f(pt.x,pt.y,pt.z);
        }
    };

    struct sFieldSegment {
        const sFieldPoint * const point[2];
        const double length;
        const double angle;
        const char name[8];
    };


    struct sMarker {
        /**
         * Estimated absolute position based on the transformation matrix and field knowledge
         */
        sVector3d absPos;

        /**
         * Pointer to corresponding field point (if reasonably inside the FoV)
         * The coordinates are the same as "absPos" but it provides other features:
         * - Name of the field point
         * - Knowledge that this marker corresponds to a field point (nullptr otherwise)
         * - The address of the fieldPt may be compared with field segment endpoints
         */
        const sFieldPoint *fieldPt = nullptr;

        /**
         * Pointer to corresponding field segment
         * This variable is currently set only for unknown markers 
         * (i.e. those which are known to belong to a field line, but whose field point is unknown)
         */
        const sFieldSegment *fieldSeg = nullptr;

        Vector3f relPosPolar;
        Vector3f relPosCart;

        /**
         * Default constructor
         */
        sMarker() : absPos({0,0,0}), relPosPolar(Vector3f()), relPosCart(Vector3f()) {};

        /**
         * Constructor with absolute position and relative polar coordinates (the cartesian version is computed)
         */
        sMarker(const sVector3d& absPos_, const Vector3f& relPosPolar_) 
            : absPos(absPos_), relPosPolar(relPosPolar_), relPosCart(relPosPolar_.toCartesian()) {};

        /**
         * Constructor with field point and relative polar coordinates (the cartesian version is computed)
         */
        sMarker(const sFieldPoint* fieldPt_, const Vector3f& relPosPolar_) 
            : absPos(fieldPt_->pt), fieldPt(fieldPt_), relPosPolar(relPosPolar_), relPosCart(relPosPolar_.toCartesian()) {};

        /**
         * Constructor with float absolute position and relative polar coordinates (the cartesian version is computed)
         */
        sMarker(const Vector3f& absPos_, const Vector3f& relPosPolar_) 
            : absPos(sVector3d({absPos_.x,absPos_.y,absPos_.z})), relPosPolar(relPosPolar_), relPosCart(relPosPolar_.toCartesian()) {};

        /**
         * Constructor with absolute position, relative polar & cartesian coordinates
         */
        sMarker(const sVector3d& absPos_, const Vector3f& relPosPolar_, const Vector3f& relPosCart_) 
            : absPos(absPos_), relPosPolar(relPosPolar_), relPosCart(relPosCart_) {};

        /**
         * Constructor with field segment, absolute position, relative polar & cartesian coordinates (e.g. unknown marker)
         */
        sMarker(const sFieldSegment* fieldSeg_, const sVector3d& absPos_, const Vector3f& relPosPolar_, const Vector3f& relPosCart_) 
            : fieldSeg(fieldSeg_), absPos(absPos_), relPosPolar(relPosPolar_), relPosCart(relPosCart_) {};

        /**
         * Constructor with field point, relative polar & cartesian coordinates 
         */
        sMarker(const sFieldPoint* fieldPt_, const Vector3f& relPosPolar_, const Vector3f& relPosCart_) 
            : absPos(fieldPt_->pt), fieldPt(fieldPt_), relPosPolar(relPosPolar_), relPosCart(relPosCart_) {};


    };

    struct sSegment {

        /**
         * Order of start and end is the same as the corresponding fieldSegment
         * [0]-start, [1]-end
         */
        sMarker point[2];

        float length; //visible segment length
        const sFieldSegment* fieldSegment; //Corresponding field segment if we had full visibility

        /**
         * Constructor
         */
        sSegment(const sMarker& start, const sMarker& end, float length_, const sFieldSegment* fieldSegment_) 
                : point{start,end}, length(length_), fieldSegment(fieldSegment_) {};

    };

    struct sFixedMarker {
        bool visible;
        Vector3f relPosPolar;
        Vector3f relPosCart;

        /**
         * Default constructor
         */
        sFixedMarker() : relPosPolar(Vector3f()), relPosCart(Vector3f()), visible(false) {};

        void set_relPos(Vector3f relPosPolar_){
            relPosPolar = relPosPolar_;
            relPosCart =  relPosPolar_.toCartesian();
        }

    };

//=================================================================================================
//================================================================================= Field constants
//=================================================================================================

    /**
     * Constant field dimensions
	 */
    static constexpr double cFieldLength = 30.0, cFieldWidth = 20.0, cPenaltyLength = 1.8, cPenaltyWidth = 6.0;
	static constexpr double cGoalWidth = 2.1, cGoalDepth = 0.6, cGoalHeight = 0.8;

	static constexpr double cHalfFielfLength = cFieldLength/2.0, cHalfFieldWidth = cFieldWidth/2.0;
	static constexpr double cHalfGoalWidth = cGoalWidth/2.0, cHalfPenaltyLength = cPenaltyLength/2.0;
	static constexpr double cHalfPenaltyWidth = cPenaltyWidth/2.0;

	static constexpr double cPenaltyBoxDistX = cHalfFielfLength-cPenaltyLength;
    static constexpr double cRingLineLength = 1.2360679774997897;

    static constexpr float cHalfHorizontalFoV = 60;
    static constexpr float cHalfVerticalFoV = 60;

    static constexpr float stdev_distance = 0.0965; //st. deviation of error ed  (distance error=d/100*ed)
    static constexpr float var_distance = 0.00931225; //   variance of error ed  (distance error=d/100*ed)
    static constexpr float var_round_hundredth = 0.01*0.01/12; //variance of uniformly distributed random variable [-0.005,0.005]


    class cFieldPoints{
        public:
        /**
         * Constant list of field points
         * Notation
         * "PT1-.-PT2" midpoint between PT1 and PT2 (2D/3D)
         * "PT1-PT2" point between PT1 and PT2 (in 2D only)
         */
        static constexpr std::array<sFieldPoint,28> list {{
            {-cHalfFielfLength,-cHalfGoalWidth, cGoalHeight, "post--"}, {-cHalfFielfLength, cHalfGoalWidth, cGoalHeight, "post-+"}, //Goalposts x<0
            { cHalfFielfLength,-cHalfGoalWidth, cGoalHeight, "post+-"}, { cHalfFielfLength, cHalfGoalWidth, cGoalHeight, "post++"}, //Goalposts x>0
            {-cHalfFielfLength,-cHalfFieldWidth,0, "corner--"}, {-cHalfFielfLength, cHalfFieldWidth,0, "corner-+"}, //Corners x<0
            { cHalfFielfLength,-cHalfFieldWidth,0, "corner+-"}, { cHalfFielfLength, cHalfFieldWidth,0, "corner++"}, //Corners x>0
            {0,-cHalfFieldWidth, 0, "halfway-"}, // Halfway line ending y<0
            {0, cHalfFieldWidth, 0, "halfway+"}, // Halfway line ending y>0
            {-cHalfFielfLength, -cHalfPenaltyWidth, 0, "boxBack--"}, {-cHalfFielfLength,  cHalfPenaltyWidth, 0, "boxBack-+"}, //Penalty box goal line corner x<0
            { cHalfFielfLength, -cHalfPenaltyWidth, 0, "boxBack+-"}, { cHalfFielfLength,  cHalfPenaltyWidth, 0, "boxBack++"}, //Penalty box goal line corner x>0
            {-cPenaltyBoxDistX, -cHalfPenaltyWidth, 0, "boxFrnt--"}, {-cPenaltyBoxDistX,  cHalfPenaltyWidth, 0, "boxFrnt-+"}, //Penalty box front corner x<0
            { cPenaltyBoxDistX, -cHalfPenaltyWidth, 0, "boxFrnt+-"}, { cPenaltyBoxDistX,  cHalfPenaltyWidth, 0, "boxFrnt++"}, //Penalty box front corner x>0
            {2, 0, 0, "r0"},                                       { 1.6180339887498948,   1.1755705045849463, 0, "r36" }, //(18,19) Ring 0/36 deg
            {0.61803398874989485,  1.9021130325903071, 0, "r72" }, {-0.61803398874989485,  1.9021130325903071, 0, "r108"}, //(20,21) Ring 72/108 deg
            {-1.6180339887498948,  1.1755705045849463, 0, "r144"}, {-2, 0, 0, "r180"},                                     //(22,23) Ring 144/180 deg
            {-1.6180339887498948, -1.1755705045849463, 0, "r216"}, {-0.61803398874989485, -1.9021130325903071, 0, "r252"}, //(24,25) Ring 216/252 deg
            {0.61803398874989485, -1.9021130325903071, 0, "r288"}, { 1.6180339887498948,  -1.1755705045849463, 0, "r324"}  //(26,27) Ring 288/324 deg
        }};

        static constexpr const sFieldPoint &goal_mm = list[0];   //Goalpost x<0 y<0
        static constexpr const sFieldPoint &goal_mp = list[1]; //Goalpost x<0 y>0
        static constexpr const sFieldPoint &goal_pm = list[2]; //Goalpost x>0 y<0
        static constexpr const sFieldPoint &goal_pp = list[3]; //Goalpost x>0 y>0

        static constexpr const sFieldPoint &corner_mm = list[4]; //Corner x<0 y<0
        static constexpr const sFieldPoint &corner_mp = list[5]; //Corner x<0 y>0
        static constexpr const sFieldPoint &corner_pm = list[6]; //Corner x>0 y<0
        static constexpr const sFieldPoint &corner_pp = list[7]; //Corner x>0 y>0

        static constexpr const sFieldPoint &halfway_m = list[8]; //Halfway line ending y<0
        static constexpr const sFieldPoint &halfway_p = list[9]; //Halfway line ending y>0

        static constexpr const sFieldPoint &boxgoal_mm = list[10]; //Penalty box goal line corner x<0 y<0
        static constexpr const sFieldPoint &boxgoal_mp = list[11]; //Penalty box goal line corner x<0 y>0
        static constexpr const sFieldPoint &boxgoal_pm = list[12]; //Penalty box goal line corner x>0 y<0
        static constexpr const sFieldPoint &boxgoal_pp = list[13]; //Penalty box goal line corner x>0 y>0

        static constexpr const sFieldPoint &box_mm = list[14]; //Penalty box front corner x<0 y<0
        static constexpr const sFieldPoint &box_mp = list[15]; //Penalty box front corner x<0 y>0
        static constexpr const sFieldPoint &box_pm = list[16]; //Penalty box front corner x>0 y<0
        static constexpr const sFieldPoint &box_pp = list[17]; //Penalty box front corner x>0 y>0

        static constexpr const sFieldPoint *rings = &list[18]; //iterator for 10 ring points

    };



    /**
     * Constant list of field line segments
	 * Each line segment has 3 characteristics: {startc, endc, length, angle, print name},
     * The angle is always positive, in [0,180[, and corresponds to the vector defined by (end-start)
     * The print name should be used for printing purposes only, 
     *      since the line segment can be identified by its constant index or address
	 */

    class cFieldLineSegments{
        public:
        static constexpr double c0deg = 0, c36deg = 0.62831853071795865, c72deg = 1.2566370614359173;
        static constexpr double c90deg = 1.5707963267948966, c108deg = 1.8849555921538759, c144deg = 2.5132741228718346;

        static constexpr std::array<sFieldSegment,21> list {{    
            {&cFieldPoints::corner_mm,  &cFieldPoints::corner_pm,  cFieldLength,    c0deg  , "side-"},  // Sideline y<0
            {&cFieldPoints::corner_mp,  &cFieldPoints::corner_pp,  cFieldLength,    c0deg  , "side+"},  // Sideline y>0
            {&cFieldPoints::corner_mm,  &cFieldPoints::corner_mp,  cFieldWidth,     c90deg , "goal-"},  // Goal line x<0
            {&cFieldPoints::corner_pm,  &cFieldPoints::corner_pp,  cFieldWidth,     c90deg , "goal+"},  // Goal line x>0
            {&cFieldPoints::halfway_m,  &cFieldPoints::halfway_p,  cFieldWidth,     c90deg , "halfway"},// Halfway line 
            {&cFieldPoints::boxgoal_mm, &cFieldPoints::box_mm,     cPenaltyLength,  c0deg  , "box--"},  // Penalty box sideline x<0 y<0
            {&cFieldPoints::boxgoal_mp, &cFieldPoints::box_mp,     cPenaltyLength,  c0deg  , "box-+"},  // Penalty box sideline x<0 y>0
            {&cFieldPoints::box_pm,     &cFieldPoints::boxgoal_pm, cPenaltyLength,  c0deg  , "box+-"},  // Penalty box sideline x>0 y<0
            {&cFieldPoints::box_pp,     &cFieldPoints::boxgoal_pp, cPenaltyLength,  c0deg  , "box++"},  // Penalty box sideline x>0 y>0
            {&cFieldPoints::box_mm,     &cFieldPoints::box_mp,     cPenaltyWidth,   c90deg , "box-"},   // Penalty box front line x<0
            {&cFieldPoints::box_pm,     &cFieldPoints::box_pp,     cPenaltyWidth,   c90deg , "box+"},   // Penalty box front line x>0
            {&cFieldPoints::rings[0],   &cFieldPoints::rings[1],   cRingLineLength, c108deg, "rL0"}, // Ring line 0   -> 36 
            {&cFieldPoints::rings[1],   &cFieldPoints::rings[2],   cRingLineLength, c144deg, "rL1"}, // Ring line 36  -> 72 
            {&cFieldPoints::rings[3],   &cFieldPoints::rings[2],   cRingLineLength, c0deg  , "rL2"}, // Ring line 72  <- 108
            {&cFieldPoints::rings[4],   &cFieldPoints::rings[3],   cRingLineLength, c36deg , "rL3"}, // Ring line 108 <- 144
            {&cFieldPoints::rings[5],   &cFieldPoints::rings[4],   cRingLineLength, c72deg , "rL4"}, // Ring line 144 <- 180
            {&cFieldPoints::rings[6],   &cFieldPoints::rings[5],   cRingLineLength, c108deg, "rL5"}, // Ring line 180 <- 216
            {&cFieldPoints::rings[7],   &cFieldPoints::rings[6],   cRingLineLength, c144deg, "rL6"}, // Ring line 216 <- 252
            {&cFieldPoints::rings[7],   &cFieldPoints::rings[8],   cRingLineLength, c0deg  , "rL7"}, // Ring line 252 -> 288
            {&cFieldPoints::rings[8],   &cFieldPoints::rings[9],   cRingLineLength, c36deg , "rL8"}, // Ring line 288 -> 324
            {&cFieldPoints::rings[9],   &cFieldPoints::rings[0],   cRingLineLength, c72deg , "rL9"}  // Ring line 324 -> 0
        }};

        static constexpr const sFieldSegment &side_m = list[0]; // Sideline y<0
        static constexpr const sFieldSegment &side_p = list[1]; // Sideline y>0
        static constexpr const sFieldSegment &goal_m = list[2]; // Goal line x<0
        static constexpr const sFieldSegment &goal_p = list[3]; // Goal line x>0

        static constexpr const sFieldSegment &halfway = list[4]; //Halfway line

        static constexpr const sFieldSegment &box_mm = list[5]; // Penalty box sideline x<0 y<0
        static constexpr const sFieldSegment &box_mp = list[6]; // Penalty box sideline x<0 y>0
        static constexpr const sFieldSegment &box_pm = list[7]; // Penalty box sideline x>0 y<0
        static constexpr const sFieldSegment &box_pp = list[8]; // Penalty box sideline x>0 y>0

        static constexpr const sFieldSegment &box_m = list[9];  // Penalty box front line x<0
        static constexpr const sFieldSegment &box_p = list[10]; // Penalty box front line x>0

        static constexpr const sFieldSegment *rings = &list[11]; //iterator for 10 ring lines

    };



    sSegment* get_known_segment(const sFieldSegment &id){
        for( auto& s : list_known_segments){
            if(s.fieldSegment == &id) return &s;
        }
        return nullptr;
    }

//=================================================================================================
//================================================================================= Control methods
//=================================================================================================
    
    /**
     * Update markers, based on existing landmarks and lines
     */
    void update();

    /**
     * Update markers, based on transformation matrix and existing lines
     */
    void update_from_transformation(const Matrix4D& tmatrix);

    /**
     * Update the absolute position of unknown markers, based on transformation matrix and existing lines
     */
    void update_unknown_markers(const Matrix4D& tmatrix);

    /**
     * Draw estimates of all visible lines, markers, self position and ball 
     */
    void draw_visible(const Matrix4D& headToFieldT, bool is_right_side) const;

    /**
     * Draw estimates of all visible lines, markers, self position and ball, but switch field sides
     */
    void draw_visible_switch(const Matrix4D& headToFieldT) const;


//=================================================================================================
//============================================================================= Visible collections
//=================================================================================================


    /**
     * Visible landmarks: corners + goalposts
     */
    vector<sMarker> list_landmarks;

    /**
     * Visible corners
     */
    vector<sMarker> list_landmarks_corners;

    /**
     * Visible goalposts
     */
    vector<sMarker> list_landmarks_goalposts;
    
    /**
     * Identified visible line segments
     * Their start and endpoints' order is the same as the corresponding field segment to which they point
     */
    vector<sSegment> list_known_segments;
    
    /**
     * Identified visible line segment endpoints + landmarks
     * Each marker has a reference to the corresponding field point
     */
    vector<sMarker> list_known_markers;

    /**
     * Endpoints (of identified visible line segments) whose corresponding field point is unknown
     * Each marker has a reference to the corresponding field segment
     * Note: list_known_markers + list_unknown_markers excludes points from unknown line segments
     */
    vector<sMarker> list_unknown_markers;

    /**
     * Visible line endpoints + foot contact points + corner flags (the absolute position is always (0,0,0))
     */
    vector<sMarker> list_ground_markers;

    /**
     * Number of visible non-collinear (line endpoints + foot contact points)
     * Note: collinearity between lines is impossible; between feet<->lines it is possible but unlikely
     */
    int non_collinear_ground_markers;

    /**
     * Same as list_ground_markers but closer points are repeated more often (proportional to distance)
     */
    vector<sMarker> list_weighted_ground_markers;

    /**
     * Feet contact points
     */
    vector<sMarker> list_feet_contact_points;

    /**
     * Visible line segments
     */
    vector<Line6f> list_segments;

    /**
     * Redundant list of all 8 landmarks' relative cartesian coordinates (to speed up lookups)
     * It's different from world.landmarks since it is ordered by position, not by name, and it holds the cartesian relPos
     * (this ordering difference is important when the teams switch sides)
     */

    class list_8_landmarks{
        friend class Field;
        private:
            static sFixedMarker list[8];
            //static std::array<sFixedMarker,8> list;
            static constexpr sFixedMarker &_corner_mm = list[0];
            static constexpr sFixedMarker &_corner_mp = list[1];
            static constexpr sFixedMarker &_corner_pm = list[2];
            static constexpr sFixedMarker &_corner_pp = list[3];
            static constexpr sFixedMarker &_goal_mm   = list[4];
            static constexpr sFixedMarker &_goal_mp   = list[5];
            static constexpr sFixedMarker &_goal_pm   = list[6];
            static constexpr sFixedMarker &_goal_pp   = list[7];
        public:
            static constexpr const sFixedMarker &corner_mm = list[0];
            static constexpr const sFixedMarker &corner_mp = list[1];
            static constexpr const sFixedMarker &corner_pm = list[2];
            static constexpr const sFixedMarker &corner_pp = list[3];
            static constexpr const sFixedMarker &goal_mm   = list[4];
            static constexpr const sFixedMarker &goal_mp   = list[5];
            static constexpr const sFixedMarker &goal_pm   = list[6];
            static constexpr const sFixedMarker &goal_pp   = list[7];
    };



//=================================================================================================
//================================================================================== Math Utilities
//=================================================================================================

    /**
     * Compute 3D distance between field line segment and cartesian point
     * Field lines are on the ground (z=0), so the method is simplified
     */
    static float fieldLineSegmentDistToCartPoint(const sFieldSegment& fLine, const Vector3f& cp);

    /**
     * Compute 2D distance between field line segment and cartesian point
     * Field lines are on the ground (z=0), so the method is simplified
     */
    static float fieldLineSegmentDistToCart2DPoint(const sFieldSegment& fLine, const Vector& cp);

    /**
     * Normalize angle between 2 lines
     * @return angle between 0 and 90 deg
     */
    static inline float normalize_line_angle_deg(float deg){
        return 90.f-fabsf(fmodf(fabsf(deg), 180.f) - 90.f);
    }

    /**
     * Normalize angle between 2 lines
     * @return angle between 0 and pi/2 rad
     */
    static inline float normalize_line_angle_rad(float rad){
        return 1.57079633f-fabsf(fmod(fabsf(rad), 3.14159265f) - 1.57079633f);
    }

    /**
     * Normalize angle between 2 vectors
     * @return angle between 0 and 180 deg
     */
    static inline float normalize_vector_angle_deg(float deg){
        return 180.f-fabsf(fmodf(fabsf(deg), 360.f) - 180.f);
    }

    /**
     * Normalize angle between 2 vectors
     * @return angle between 0 and pi rad
     */
    static inline float normalize_vector_angle_rad(float rad){
        return 3.14159265f-fabsf(fmod(fabsf(rad), 6.28318531f) - 3.14159265f);
    }

};

typedef Singleton<Field> SField;