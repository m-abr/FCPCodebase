/**
 * FILENAME:     LocalizerV2
 * DESCRIPTION:  main 6D localization algorithm
 * AUTHOR:       Miguel Abreu (m.abreu@fe.up.pt)
 * DATE:         2021
 * 
 * ===================================================================================
 * WORKFLOW
 * ===================================================================================
 * 
 * References can be obtained from:
 * - landmarks (which are identified by the server and can be corner flags or goal posts)
 * - line segments (which are always on the ground) (hereinafter referred to as lines)
 * - feet contact points (we are assuming the contact point is on the ground plane)
 * 
 * WARNING:
 * When partial information is available, it is used to change the head position (e.g. translation in x/y/z may be updated
 * without visual information). However, the transformation matrix (including the translation) are not changed, because 
 * this would affect the perceived position of previously seen objects. For this reason, the worldstate should not rely 
 * on the head position to convert relative to absolute coordinates. Instead, it should only use transformation matrix, 
 * or internal conversion methods. The head position can still be used for other purposes, such as machine learning.
 * 
 * -----------------------------------------------------------------------------------
 * 0. Perform basic tasks and checks
 * 
 *      Excluded scenarios:
 *          0 landmarks & <2 lines - it's a requirement from step 1
 *          0 lines - step 1 allows this if there are 3 ground refs but the only marginally common
 *                    option would be 2 feet and a corner (which is undesirable)
 *          
 * -----------------------------------------------------------------------------------
 * 1. Find the Z axis orientation vector (and add it to the preliminary transformation matrix):
 * 		1.1. there are >= 3 noncollinear ground references (z=0)
 * 			ASSUMPTION: the ground references are not collinear. Why? 
 * 						If we see 2 lines their endpoints are never collinear. 
 *  					If we see one and we are on top of it, the feet contact points can cause collinearity but it is very rare.
 * 			SOLUTION: Find the best fitting ground plane's normal vector using Singular Value Decomposition
 * 
 * 		1.2. there are < 3 noncollinear ground references (z=0)
 * 
 * 			Possible combinations: 
 * 			If there is 1 corner flag, either we have >= 3 ground references, or it is impossible.
 * 			So, below, we assume there are 0 corner flags.
 *
 * 						 |	0 lines + 0/1/2 feet  |  1 line + 0 feet |
 *      	-------------|------------------------|------------------|
 * 			0 goalposts  |         -----          |       -----      | (Only 1 line: there is no way of identifying it)
 * 			1 goalpost	 |         -----          |        A,C       | (1 goalpost and 0/1/2 feet: infinite solutions)
 * 			2 goalposts  |           *            |        B,C       |
 * 			
 * 		
 * 			If it sees 1 or 2 goalposts and only 1 line, we assume for A & B that it is the endline (aka goal line)
 * 		
 * 			SOLUTIONS:
 * 			1.2.A. IF IT IS THE GOALLINE. Find the line's nearest point (p) to the goalpost (g), Zvec = (g-p) / |Zvec|
 * 			1.2.B. IF IT IS THE GOALLINE. Find the line's nearest point (p) to the goalposts (g1,g2) midpoint (m), Zvec = (m-p) / |Zvec|
 * 				   (This solution is more accurate than using only 1 goalpost. Even m is more accurate, on average, than g1 or g2.)  
 *          1.2.C. IF IT IS NOT THE GOALLINE. There are 3 options: 
 *                  I - There are 2 goalposts (crossbar line) and an orthogonal line:
 *                      Zvec = crossbar x line (or) line x crossbar (depending of the direction of both vectors)
 *                  II - Other situation if the z translation coordinate was externally provided through machine learning:
 *                      Find any horizontal line (e.g. line between 2 goalposts, or ground line)
 *                      Let M be any mark with known absolute z, and let Z be the externally provided z coordinate:
 *                      Find Zvec such that (HorLine.Zvec=0) and (Zvec.Mrel=Mabsz-Z) 
 *                  III - If z was not provided:
 *                      Skip to last step.
 * 			1.2.*. This scenario was tested and it is not valid. In certain body positions there are two solutions, and even though 
 *                 one is correct and generally yields lower error, the other one is a local optimum outside the field. One could 
 *                 exclude the out-of-field solution with some mildly expensive modifications to the optimization's error function,
 *                 but the out-of-field scenario is not unrealistic, so this is not the way. Adding an external z source could help
 *                 increasing the error of the wrong local optimum, but it may not be enough. Another shortcoming of this scenario is
 *                 when we see the goalposts from the opposite goal, creating large visual error. 
 * 		
 * 		1.3. impossible / not implemented: in this case skip to last step
 * 
 * -----------------------------------------------------------------------------------		
 * 2. Compute z:
 * 		
 * 		Here's what we know about the transformation matrix so far:
 * 		| -  | -  | -  | - |
 *      | -  | -  | -  | - |
 * 		| zx | zy | zz | ? | We want to know the translation in z
 * 		| 0  | 0  | 0  | 1 |
 * 
 * 		Given a random point (p) with known relative coordinates and known absolute z coordinate, 
 * 		we can find the translation in z:
 * 			p.relx * zx + p.rely * zy + p.relz * zz + ? = p.absz
 * 
 * 		If we do this for every point, we can then average z
 * 
 * -----------------------------------------------------------------------------------		
 * 3. Compute a rough estimate for entire transformation (2 first rows):
 * 
 * 		Solution quality for possible combinations:
 * 
 * 		short line (length < 0.5m) *hard to detect orientation, *generated displacement error is insuficient for optimization
 * 		long line (length >= 0.5m)
 * 		
 * 		                 |	0 landmarks  |  1 goalpost  |  1 corner  | >= 2 landmarks |
 *      -----------------|---------------|--------------|------------|----------------|
 * 		0 long lines     |      ---      |      ---     |     ---    |        A       |
 * 		1 long line      |      ---      |       B+     |      B     |        A       |
 * 		2 long lines     |       B       |       B+     |      B++   |        A       |
 * 
 * 		SOLUTIONS:
 * 		A - the solution is unique
 *          STEPS:
 * 			    - Compute the X-axis and Y-axis orientation from 2 landmarks
 *              - Average the translation for every visible landmark
 *              - Fine-tune XY translation/rotation
 * 		B - there is more than 1 solution, so we use the last known position as reference
 * 			Minimum Requirements:
 * 				- longest line must be >1.6m so that we can extract the orientation (hor/ver) while not being mistaken for a ring line
 * 				- there is exactly 1 plausible solution
 * 			Notes:
 * 				B+  (the solution is rarely unique)
 * 				B++ (the solution is virtually unique but cannot be computed with the algorithm for A scenarios)
 * 			STEPS:
 * 				- Find 4 possible orientations based on the longest line (which should be either aligned with X or Y)
 *              - Determine reasonable initial translation:
 *                  - If the agent sees 1 landmark: compute XY translation for each of the 4 orientations based on that 1 landmark
 *                  - If the agent sees 0 landmarks: use last known position
 *                  Note: Why not use always last known position if that is a criterion in the end? 
 *                        Because in this stage it would only delay the optimization.
 * 				- Optimize the X/Y translation for every possible orientation 
 *              - Perform quality assessment
 * 			        Plausible solution if:
 *                      - Optimization converged to local minimum
 * 				        - Distance to last known position <50cm (applicable if no visible landmarks)
 * 				        - Mapping error <0.12m/point
 * 				        - Given the agent's FOV, inverse mapping error <0.2m/point (disabled in V2)
 *                      NOTE: If there is 1 landmark, plausibility is defined by mapping errors, not distance to last known pos. So if
 *                      one guess has the only acceptable mapping error, but is the farthest from previous position, it is still chosen.
 *                      However, if >1 guess has acceptable mapping error, the 0.5m threshold is used to eliminate candidates.
 *                  Likely if:
 *                      - Plausible
 *                      - Distance to last known position <30cm (applicable if no visible landmarks)
 *                      - Mapping error <0.06m/point
 *                      - Given the agent's FOV, inverse mapping error <0.1m/point (not currently active)
 *              - Choose likely solution if all others are not even plausible
 *              - Fine-tune XY translation/rotation
 * 
 * -----------------------------------------------------------------------------------
 * 4. Identify visible elements and perform 2nd fine tune based on distance probabilites
 *              
 * -----------------------------------------------------------------------------------
 * Last step. Analyze preliminary transformation matrix to update final matrices
 * 
 * 		For the reasons stated in the beginning (see warning), if the preliminary matrix was not entirely set, the
 * 		actual transformation matrix will not be changed. But the head position will always reflect the latest changes.
 *
 * 
 * ===================================================================================
 * LOCALIZATION BASED ON PROBABILITY DENSITIES
 * ===================================================================================
 * ================================PROBABILITY DENSITY================================
 * 
 * For 1 distance measurement from RCSSSERVER3D:
 * 
 * Error E = d/100 * A~N(0,0.0965^2) + B~U(-0.005,0.005)
 * PDF[d/100 * A](w) = PDF[N(0,(d/100 * 0.0965)^2)](w)
 * PDF[E](w) = PDF[N(0,(d/100 * 0.0965)^2)](w) convoluted with PDF[U(-0.005,0.005)](w)
 * 
 * where d is the distance from a given [p]oint (px,py,pz) to the [o]bject (ox,oy,oz)
 * and w is the [m]easurement error: w = d-measurement = sqrt((px-ox)^2+(py-oy)^2+(pz-oz)^2) - measurement
 * 
 * PDF[E](w) -> PDF[E](p,o,m)
 * ---------------------------------------------------------------
 * For n independent measurements:
 * 
 * PDF[En](p,on,mn) = PDF[E1](p,o1,m1) * PDF[E2](p,o2,m2) * PDF[E3](p,o3,m3) * ...
 * ---------------------------------------------------------------
 * Adding z estimation:
 *
 * PDF[zE](wz) =  PDF[N(mean,std^2)](wz) 
 * where wz is the zError = estz - pz
 * 
 * PDF[zE](wz) -> PDF[zE](pz,estz)
 * PDF[En](p,on,mn,estz) =  PDF[En](p,on,mn) * PDF[zE](pz,estz)
 * ===================================================================================
 * =====================================GRADIENT======================================
 * 
 * Grad(PDF[En](p,on,mn,estz)) wrt p = Grad( PDF[E1](p,o1,m1) * ... * PDF[E2](p,on,mn) * PDF[zE](pz,estz)) wrt {px,py,pz}
 * 
 * Generalizing the product rule for n factors, we have:
 * 
 * Grad(PDF[En](p,on,mn)) wrt p = sum(gradPDF[Ei] / PDF[Ei]) * prod(PDF[Ei])
 * Grad(PDF[En](p,on,mn)) wrt p = sum(gradPDF[Ei] / PDF[Ei]) * PDF[En](p,on,mn)
 * 
 * note that: gradPDF[zE](pz,estz) wrt {px,py,pz} = {0,0,d/d_pz}
 * ===================================================================================
 * */

#pragma once
#include "Singleton.h"
#include "Field.h"
#include "Matrix4D.h"
#include "FieldNoise.h"

#include <gsl/gsl_multifit.h> //Linear least-squares fitting
#include <gsl/gsl_linalg.h>   //Singular value decomposition
#include <gsl/gsl_multimin.h> //Multidimensional minimization

class LocalizerV2 {
    friend class Singleton<LocalizerV2>;

public:
 
    /**
     * Compute 3D position and 3D orientation
     * sets "is_uptodate" to true if there is new information available (rotation+translation)
     * If no new information is available, the last known position is provided
     */
    void run();

    /**
     * Print report (average errors + solution analysis)
     */
    void print_report() const;

    /**
     * Transformation matrices
     * They are initialized as 4x4 identity matrices
     */
    const Matrix4D &headTofieldTransform = final_headTofieldTransform; // rotation + translation
    const Matrix4D &headTofieldRotate    = final_headTofieldRotate; // rotation
    const Matrix4D &fieldToheadTransform = final_fieldToheadTransform; // rotation + translation
    const Matrix4D &fieldToheadRotate    = final_fieldToheadRotate; // rotation

    /**
     * Head position
     * translation part of headTofieldTransform
     */
    const Vector3f &head_position = final_translation;

    /**
     * True if head_position and the transformation matrices are up to date
     * (false if this is not a visual step, or not enough elements are visible)
     */
    const bool &is_uptodate = _is_uptodate;

    /**
     * Number of simulation steps since last update (see is_uptodate)
     * If (is_uptodate==true) then "steps_since_last_update" is zero
     */
    const unsigned int &steps_since_last_update = _steps_since_last_update;

    /**
     * Head z coordinate
     * This variable is often equivalent to head_position.z, but sometimes it differs.
     * There are situations in which the rotation and translation cannot be computed, 
     * but the z-coordinate can still be found through vision.
     * It should be used in applications which rely on z as an independent coordinate,
     * such as detecting if the robot has fallen, or as machine learning observations.
     * It should not be used for 3D transformations.
     */
    const float &head_z = final_z;

    /**
     * Since head_z can be computed in situations where self-location is impossible,
     * this variable is set to True when head_z is up to date
     */
    const bool &is_head_z_uptodate = _is_head_z_uptodate;
    
    /**
     * Transform relative to absolute coordinates using headTofieldTransform
     * @return absolute coordinates
     */
    Vector3f relativeToAbsoluteCoordinates(const Vector3f relativeCoordinates) const;

    /**
     * Transform absolute to relative coordinates using fieldToheadTransform
     * @return relative coordinates
     */
    Vector3f absoluteToRelativeCoordinates(const Vector3f absoluteCoordinates) const;

    /**
     * Get 3D velocity (based on last n 3D positions)
     * @param n number of last positions to evaluate (min 1, max 9)
     * Example for n=3:
     *      current position: p0 (current time step)
     *      last position:    p1 (typically* 3 time steps ago)
     *      position before:  p2 (typically* 6 time steps ago)
     *      position before:  p3 (typically* 9 time steps ago)
     *      RETURN value:     p0-p3
     * *Note: number of actual time steps depends on server configuration and whether 
     *        the agent was able to self-locate on the last n visual steps
     * @return 3D velocity vector
     */
    Vector3f get_velocity(unsigned int n) const;

    /**
     * Get last known head z coordinate
     * Note: this variable is based on head_z. It can be used as an independent coordinate,
     *       but it should not be used for 3D transformations, as it may be out of sync with
     *       the x and y coordinates. (see head_z)
     * @return last known head z coordinate
     */
    float get_last_head_z() const {return last_z;}


private:
    
    //=================================================================================================
    //============================================================================ main private methods
    //=================================================================================================

    bool find_z_axis_orient_vec(); //returns true if successful
    void fit_ground_plane();

    void find_z(const Vector3f& Zvec);
    bool find_xy();
    bool guess_xy();

    bool fine_tune_aux(float &initial_angle, float &initial_x, float &initial_y, bool use_probabilities);
    bool fine_tune(float initial_angle, float initial_x, float initial_y);

    static double map_error_logprob(const gsl_vector *v, void *params);
    static double map_error_2d(const gsl_vector *v, void *params);

    void commit_everything();


    //=================================================================================================
    //=================================================================== private transformation matrix
    //=================================================================================================

    //PRELIMINARY MATRIX - where all operations are stored
    //if the algorithm is not successful, the final matrix is not modified
    void prelim_reset();
    Matrix4D prelimHeadToField = Matrix4D(); //initialized as identity matrix

    //FINAL MATRIX - the user has access to a public const reference of the variables below
    Matrix4D final_headTofieldTransform; // rotation + translation
    Matrix4D final_headTofieldRotate;    // rotation
    Matrix4D final_fieldToheadTransform; // rotation + translation
    Matrix4D final_fieldToheadRotate;    // rotation

    Vector3f final_translation; //translation

    float final_z; //independent z translation (may be updated more often)

    //=================================================================================================
    //=============================================================================== useful statistics
    //=================================================================================================

    std::array<Vector3f, 10> position_history;
    unsigned int position_history_ptr = 0;

    float last_z = 0.5; 

    unsigned int _steps_since_last_update = 0;
    bool _is_uptodate = false;
    bool _is_head_z_uptodate = false;

    //=================================================================================================
    //================================================================================ debug statistics
    //=================================================================================================

    int stats_sample_position_error(const Vector3f sample, const Vector3f& cheat, double error_placeholder[]);
    void stats_reset();
    
    double errorSum_fineTune_before[7] = {0};        //[0,1,2]- xyz err sum, [3]-2D err sum, [4]-2D err sq sum, [5]-3D err sum, [6]-3D err sq sum
    double errorSum_fineTune_euclidianDist[7] = {0}; //[0,1,2]- xyz err sum, [3]-2D err sum, [4]-2D err sq sum, [5]-3D err sum, [6]-3D err sq sum
    double errorSum_fineTune_probabilistic[7] = {0}; //[0,1,2]- xyz err sum, [3]-2D err sum, [4]-2D err sq sum, [5]-3D err sum, [6]-3D err sq sum
    double errorSum_ball[7] = {0};                   //[0,1,2]- xyz err sum, [3]-2D err sum, [4]-2D err sq sum, [5]-3D err sum, [6]-3D err sq sum
    int counter_fineTune = 0;
    int counter_ball = 0;

    enum STATE{NONE, RUNNING, MINFAIL, BLIND, FAILzNOgoal, FAILzLine, FAILz, FAILtune, FAILguessLine, FAILguessNone, FAILguessMany, FAILguessTest, DONE, ENUMSIZE};
    STATE state = NONE;

    void stats_change_state(enum STATE s);
    int state_counter[STATE::ENUMSIZE] = {0};

};


typedef Singleton<LocalizerV2> SLocalizerV2;