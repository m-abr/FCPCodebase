#include "a_star.h"
#include "expansion_groups.h"
#include <cmath>
#include <algorithm>
#include <chrono>
#define SQRT2 1.414213562373095f
#define LINES 321
#define COLS 221
#define MAX_RADIUS 5  // obstacle max radius in meters
#define IN_GOAL_LINE 312 // target line when go_to_goal is 'true' (312 -> 15.2m)
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

/*
Map dimensions: 32m*22m 
col 0  ...  col 220
                   line 0
 --- our goal ---  line 1
 |              |  line 2        
 |              |  line 3
 |--------------|  ...
 |              |  line 317
 |              |  line 318
 -- their goal --  line 319
                   line 320   

[(H)ard wall: -3, (S)oft wall: -2, (E)mpty: 0, 0 < Cost < inf]                   
*/

// build board cost statically
#define H27 -3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3
#define S11 -2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2
#define S19 S11,-2,-2,-2,-2,-2,-2,-2,-2
#define S97 S11,S11,S11,S11,S11,S11,S11,S11,-2,-2,-2,-2,-2,-2,-2,-2,-2
#define S98 S11,S11,S11,S11,S11,S11,S11,S11,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2
#define S221 S98,S98,S11,S11,-2,-2,-2
#define E19 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
#define E197 E19,E19,E19,E19,E19,E19,E19,E19,E19,E19,0,0,0,0,0,0,0
#define L0 S221                              // Line 0: soft W
#define L0_1 L0,L0
#define L2 S97,H27,S97                       // Line 2: soft W, (goal post, back net, goal post), soft W
#define L2_5 L2,L2,L2,L2
#define L6 S98,-3,-3,-3,S19,-3,-3,-3,S98     // Line 6: soft W, goal post, soft W, goal post, soft W
#define L6_10 L6,L6,L6,L6,L6
#define L11 S98,-2,-3,-3,S19,-3,-3,-2,S98  // Line 11:soft W, empty field, goal post, soft W, goal post,empty field, soft W
#define L12 S11,-2,E197,-2,S11                     // Line 12:soft W, empty field, soft W

#define L12x33 L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12,L12
#define LIN12_308 L12x33,L12x33,L12x33,L12x33,L12x33,L12x33,L12x33,L12x33,L12x33

#define L309 S98,-2,-3,-3,E19,-3,-3,-2,S98  // Line 309: soft W, empty field, goal post, empty field, goal post,empty field, soft W
#define L310 S98,-3,-3,-3,E19,-3,-3,-3,S98    // Line 310: soft W, goal post, inside goal, goal post, soft W
#define L310_314 L310,L310,L310,L310,L310

using std::min;
using std::max;


#define MIN min_node
Node* min_node; // non-expanded node with lowest predicted total cost (f)

namespace open{

    Node* insert(Node* new_node, Node* root) {

        new_node->left = nullptr;
        new_node->right = nullptr;
        
        // Empty BST, return without saving min_node
        if(root == nullptr){
            new_node->up = nullptr;
            MIN = new_node; // save min node for fast access
            return new_node;
        }

        // If new_node is the new min node
        if(new_node->f < MIN->f){
            MIN->left = new_node;
            new_node->up = MIN;
            MIN = new_node;
            return root;
        }

        Node* node = root;
        float key = new_node->f;

        while(true){
            if (key < node->f)
                if(node->left == nullptr){
                    node->left = new_node;
                    break;
                }else{
                    node = node->left;
                }
            else{
                if(node->right == nullptr){                
                    node->right = new_node;
                    break;
                }else{
                    node = node->right;
                }    
            }
        }

        new_node->up = node;
        return root;
    }


    // Remove min node
    Node* pop(Node* root) {

        // Minimum node can have right child, but not left child
        if (MIN->right == nullptr){
            if(MIN == root){             //------(A)------ min node is root and has no children
                return nullptr;          // BST is empty
            } 
            MIN->up->left = nullptr;     //------(B)------ min node has no children but has parent
            MIN = MIN->up;
        }else{
            if(MIN == root){             //------(C)------ min node is root and has right child
                MIN = MIN->right;
                root = MIN;
                root->up = nullptr;

                while(MIN->left != nullptr){  // update new min node
                    MIN = MIN->left;
                }

                return root;                  // right child is now root
            } 
            MIN->right->up = MIN->up;         //------(D)------ min node has right child and parent
            MIN->up->left = MIN->right;      

            MIN = MIN->right;
            while(MIN->left != nullptr){      // update new min node
                MIN = MIN->left;
            }
        }

        return root;
    }


    // Remove specific node
    Node* delete_node(Node* node, Node* root) {

        if(node == MIN){ // remove min node   
            return pop(root);
        }

        if(node->left==nullptr and node->right==nullptr){  //------(A)------ node has no children (it can't be root, otherwise it would be min node)
            // Redirect incoming connection
            if(node->up->left == node){  node->up->left  = nullptr; }
            else{                        node->up->right = nullptr; }
            
        }else if(node->left==nullptr){                    //------(B)------ node has right child (it can't be root, otherwise it would be min node)
            // Redirect incoming connections
            node->right->up = node->up;
            if(node->up->left == node){ node->up->left  = node->right; }
            else{                       node->up->right = node->right; }

        }else if(node->right==nullptr){                   //------(C)------ node has left child
            if(node == root){
                node->left->up = nullptr;
                return node->left; // left child becomes root
            }

            // Redirect incoming connections (if not root)
            node->left->up = node->up;
            if(node->up->left == node){ node->up->left  = node->left; }
            else{                       node->up->right = node->left; }

        }else{                                            //------(D)------ node has 2 children
            Node *successor = node->right;
            if(successor->left == nullptr){               //----- if successor is the node's right child (successor has no left child)

                //-------------- successor replaces node

                // Outgoing connections (successor's right child is not changed)
                successor->left = node->left;
                successor->up = node->up; // if node is root this is also ok

                // Incoming connections
                node->left->up = successor;

                if(node == root){ return successor; } // successor becomes root

                // Incoming connections (if not root)
                if(node->up->left == node){ node->up->left  = successor; }
                else{                       node->up->right = successor; }
                
            }else{                                        //------ if successor is deeper (successor has no left child, and is itself a left child)
                do{
                    successor = successor->left;
                }while(successor->left != nullptr);

                //-------------- Remove successor by redirecting its incoming connections
                if(successor->right==nullptr){ // no children
                    successor->up->left  = nullptr; 
                }else{
                    successor->up->left  = successor->right; 
                    successor->right->up = successor->up;
                }

                //-------------- successor replaces node

                // Outgoing connections
                successor->left = node->left;
                successor->right = node->right;
                successor->up = node->up; // if node is root this is also ok
                
                // Incoming connections
                node->left->up = successor;
                node->right->up = successor;

                if(node == root){ return successor; } // successor becomes root

                // Incoming connections (if not root)
                if(node->up->left == node){ node->up->left  = successor; }
                else{                       node->up->right = successor; }
            }
        }

        return root; 
    }

    // Inorder Traversal
    // void inorder(Node* root, Node* board) {
    //     if (root != nullptr) {
    //         // Traverse left
    //         inorder(root->left, board);

    //         // Traverse root
    //         std::cout << (root-board)/COLS << " " << (root-board)%COLS << " -> ";

    //         // Traverse right
    //         inorder(root->right, board);

    //     }
    //     return;
    // }
}




inline int x_to_line(float x){
    return int(fmaxf(0.f, fminf(10*x+160, 320.f)) + 0.5f);
}

inline int y_to_col(float y){
    return int(fmaxf(0.f, fminf(10*y+110, 220.f)) + 0.5f);
}

inline float  diagonal_distance(bool go_to_goal, int line, int col, int end_l, int end_c){
    // diagonal distance - adapted from http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
    int dl, dc;
    if( go_to_goal ){
        dl = abs(IN_GOAL_LINE - line);
        if (col>119)      { dc = col-119; }
        else if (col<101) { dc = 101-col; }
        else              { dc = 0;       } 
    }else{
        dl = abs(line - end_l);
        dc = abs(col - end_c);
    }
    return (dl + dc) - 0.585786437626905f * min(dl,dc); 
}

inline Node* expand_child(Node* open_root, float cost, float wall_index, Node* curr_node, Node* board, int pos, int state, 
                          bool go_to_goal, int line, int col, int end_l, int end_c, unsigned int* node_state, float extra ){
    // child can be as inaccessible as current pos (but there is a cost penalty to avoid inaccessible paths)
    if(cost <= wall_index){
        cost = 100.f;
    }

    // g (min cost from start to n) 
    float g = curr_node->g + extra + std::fmaxf(0.f,cost); // current cost + child distance

    Node* child = &board[pos];

    // if child is already in the open set
    if (state){
        if (g >= child->g){
            return open_root; // if not an improvement, we discard the new child
        }else{
            open_root = open::delete_node(child, open_root); // if it is an improvement: remove reference, update it, add it again in correct order
        }
    }else{
        node_state[pos] = 1;
    }

    // f (prediction of min total cost passing through n)
    float f = g + diagonal_distance(go_to_goal,line,col,end_l,end_c); 

    child->g = g;
    child->f = f;
    child->parent = curr_node;
    return open::insert(child, open_root);
}


float final_path[2050];
int final_path_size;

inline void build_final_path(Node* const best_node, const Node* board, float status, const bool override_end=false, const float end_x=0, const float end_y=0){
    // Node* pt = best_node;
    // while( pt != nullptr ){
    //     int pos = pt - board;
    //     board_cost[pos] = -4;
    //     pt = pt->parent;
    // }
    // std::cout << "\n";
    // for(int l=l_min; l<=l_max; l++){
    //     for(int c=c_min; c<=c_max; c++){
    //         //[(H)ard wall: -3, (S)oft wall: -2, (E)mpty: 0, 0 < Cost < inf]
    //         //if      (board[l][c].closed) std::cout << "o";
    //         if (board_cost[l*COLS+c] == -3) std::cout << "h";
    //         else if (board_cost[l*COLS+c] == -2) std::cout << "s";
    //         else if (board_cost[l*COLS+c] == -4) std::cout << ".";
    //         else if (board_cost[l*COLS+c] == -1) std::cout << "g";
    //         else if (board_cost[l*COLS+c] ==  0 and node_state[l*COLS+c]==2) std::cout << "o";
    //         else if (board_cost[l*COLS+c] ==  0) std::cout << " ";
    //         //ele ainda nao sabe ler hard walls
    //         else std::cout << int(board_cost[l*COLS+c]+0.5f);
    //     }
    //     std::cout << "\n";
    // }

    // Using 'current_node' would suffice if A* reaches the objective (but 'best_node' works with impossible paths or timeout)
    Node* ptr = best_node;
    int counter=0;
    do{
        ptr = ptr->parent;
        counter++;
    }while( ptr != nullptr );

    final_path_size = min(counter*2,2048);
    
    ptr = best_node;
    int i = final_path_size-1;

    // if enabled, replace end point with correct coordinates instead of discrete version
    if(override_end){ 
        final_path[i--] = end_y;
        final_path[i--] = end_x;
        ptr = ptr->parent;
    }

    for(; i>0;){
        final_path[i--] = ((ptr-board) % COLS)/10.f-11.f; // y
        final_path[i--] = ((ptr-board) / COLS)/10.f-16.f; // x
        ptr = ptr->parent;
    }    

    // add status (& increment path size)
    final_path[final_path_size++] = status; // 0-success, 1-timeout, 2-impossible, 3-no obstacles(this one is not done in this function)

    // add cost (& increment path size)
    final_path[final_path_size++] = best_node->g / 10.f; // min. A* cost from start to best_node

}


/**
 * @brief Returns true if line segment 'ab' intersects either goal (considering the unreachable area)
 * - This function assumes that 'a' and 'b' are two points outside the unreachable goal area
 * - Therefore, 'ab' must enter and exit the goal unreachable area
 * - To detect this, we consider only the intersection of 'ab' and the goal outer borders (back+sides)
 * - The front should already be covered by independent goal posts checks
 */
inline bool does_intersect_any_goal(float a_x, float a_y, float b_x, float b_y){
    
    float ab_x = b_x - a_x;
    float ab_y = b_y - a_y;
    float k;

    if(ab_x != 0){ // Check if 'ab' and goal back is noncollinear (collinear intersections are ignored)

        k = (15.75-a_x) / ab_x;  // a_x + ab_x*k = 15.75
        if (k >= 0 and k <= 1 and fabsf(a_y + ab_y * k) <= 1.25){ // collision (intersection_y = a_y + ab_y*k)
            return true;
        }

        k = (-15.75-a_x) / ab_x;  // a_x + ab_x*k = -15.75
        if (k >= 0 and k <= 1 and fabsf(a_y + ab_y * k) <= 1.25){ // collision (intersection_y = a_y + ab_y*k)
            return true;
        }
    }

    if(ab_y != 0){ // Check if 'ab' and goal sides are noncollinear (collinear intersections are ignored)

        k = (1.25-a_y) / ab_y; // a_y + ab_y*k = 1.25
        if( k >= 0 and k <= 1){
            float intersection_x_abs = fabsf(a_x + ab_x * k);
            if( intersection_x_abs >= 15 and intersection_x_abs <= 15.75 ){ // check one side for both goals at same time
                return true;
            }
        }

        k = (-1.25-a_y) / ab_y; // a_y + ab_y*k = -1.25
        if( k >= 0 and k <= 1){
            float intersection_x_abs = fabsf(a_x + ab_x * k);
            if( intersection_x_abs >= 15 and intersection_x_abs <= 15.75 ){ // check one side for both goals at same time
                return true;
            }
        }
    }

    return false;
}


/**
 * @brief Add space cushion near the midlines and endlines
 */
inline void add_space_cushion(float board_cost[]){

    #define CUSHION_WIDTH 6

    // opponent goal line
    for(int i=0; i<CUSHION_WIDTH; i++){
        int ii = (i+12)*COLS;
        for(int j=12+i; j<209-i; j++){
            board_cost[ii+j] = CUSHION_WIDTH-i;
        }
    }

    // our goal line
    for(int i=0; i<CUSHION_WIDTH; i++){
        int ii = (308-i)*COLS;
        for(int j=12+i; j<99; j++){
            board_cost[ii+j] = CUSHION_WIDTH-i;
            board_cost[ii+220-j] = CUSHION_WIDTH-i;
        }
    }

    // sidelines
    for(int i=0; i<CUSHION_WIDTH; i++){
        for(int j=(13+i)*COLS; j<(308-i)*COLS; j+=COLS){
            board_cost[j+i+12] = CUSHION_WIDTH-i;
            board_cost[j-i+208] = CUSHION_WIDTH-i;
        }
    }
    
}



/**
 * @brief This function checks if a straight path from start to end is obstructed by obstacles
 * Returning 'false' means that we do not need A* to solve this problem
 * 
 * Normal case (start != end):
 *      The path is obstructed if it intersects any hard or soft circumference
 * Special case (start == end):
 *      The path is obstructed if start is inside any hard circumference
 */
bool is_path_obstructed(float start_x, float start_y, float end_x, float end_y, float given_obstacles[], 
                        int given_obst_size, bool go_to_goal, int wall_index, float board_cost[]){


    // Restrict start coordinates to map
    start_x = max(-16.f, min(start_x, 16.f));
    start_y = max(-11.f, min(start_y, 11.f));

    int s_lin = x_to_line(start_x);
    int e_lin = x_to_line(end_x);
    int s_col = y_to_col(start_y);
    int e_col = y_to_col(end_y);
    float s_cost = board_cost[s_lin*COLS+s_col];
    float e_cost = board_cost[e_lin*COLS+e_col];

    // Path is obvious if start/end are is same or adjacent cells
    bool is_near = abs(s_lin - e_lin) <= 1 and abs(s_col - e_col) <= 1;

    // Let A* handle it if the start position is unreachable or (out of bounds is not allowed && is nearly out of bounds && not near end)
    if(s_cost <= wall_index or (!is_near and s_cost > 0)){
        return true;
    }


    if (go_to_goal){ // This is a safe target. If it generates a collision with any goal post, we use A* instead.
        end_x = 15.2;
        end_y = max(-0.8f, min(start_y, 0.8f));
    }else{ // Restrict end coordinates to map
        end_x = max(-16.f, min(end_x, 16.f));
        end_y = max(-11.f, min(end_y, 11.f));

        // Let A* handle it if the end position is unreachable or (is nearly out of bounds && out of bounds is not allowed && not near end)
        if(e_cost <= wall_index or (!is_near and e_cost > 0)){
            return true;
        }
    }


    /**
     * Check if path intersects either goal (considering the unreachable area)
     * - at this point we know that 'start' and 'end' are reachable
     * - Therefore, the path must enter and exit the goal unreachable area for an intersection to exist
     * - To detect this, we consider only the intersection of the path and the goal outer borders (back+sides)
     * - The front is covered next by goal posts checks
     */
    if (does_intersect_any_goal(start_x, start_y, end_x, end_y)) {
        return true;
    }

    /**
     * ----------------------- List all obstacles: given obstacles + goal posts
     * note that including the goal posts in the given obstacles is not a bad idea since the default 
     * goal posts we add here provide no space cushion (which may be needed if the robot is not precise)
     * values explanation:
     * - goal post location (tested in simulator, collision with robot, robot is sideways to the goal post and arms are close to body)
     * - hard radius (tested in the same way, robot collides when closer than 0.15m from goal post border)
     *      post radius 0.02 + agent radius 0.15 = 0.17
     * - largest radius (equivalent to hard radius, since there is no soft radius)
     */

    float obst[given_obst_size*4/5+16] = {
         15.02,  1.07, 0.17, 0.17,
         15.02, -1.07, 0.17, 0.17,
        -15.02,  1.07, 0.17, 0.17,
        -15.02, -1.07, 0.17, 0.17
    }; // x, y, hard radius, largest radius


    int obst_size = 16;

    for(int i=0; i<given_obst_size; i+=5){
        obst[obst_size++] = given_obstacles[i];                                                       // x
        obst[obst_size++] = given_obstacles[i+1];                                                     // y
        obst[obst_size++] = fmaxf( 0, fminf(given_obstacles[i+2], MAX_RADIUS) );                      // hard radius
        obst[obst_size++] = fmaxf( 0, fminf(max(given_obstacles[i+2], given_obstacles[i+3]), MAX_RADIUS) ); // largest radius
    }

    //------------------------ Special case (start ~= end): the path is obstructed if start or end are inside any hard circumference

    if( is_near ){
        for(int ob=0; ob<obst_size; ob+=4){
            float c_x = obst[ob];   // obstacle center
            float c_y = obst[ob+1]; // obstacle center
            float hard_radius = obst[ob+2]; // hard radius
            float r_sq = hard_radius * hard_radius; // squared radius

            float sc_x = c_x - start_x; 
            float sc_y = c_y - start_y;
            float ec_x = c_x - end_x; 
            float ec_y = c_y - end_y;

            if(sc_x*sc_x + sc_y*sc_y <= r_sq  or  ec_x*ec_x + ec_y*ec_y <= r_sq){ // check distance: center<->start center<->end
                return true;
            }
        }
    }else{

        //-------------------- Normal case (start !~= end): the path is obstructed if it intersects any hard or soft circumference

        // for each obstacle: check if circle intersects line segment (start-end)
        for(int ob=0; ob<obst_size; ob+=4){
            float c_x = obst[ob];   // obstacle center
            float c_y = obst[ob+1]; // obstacle center
            float largest_radius = obst[ob+3]; // largest radius
            float r_sq = largest_radius * largest_radius; // squared radius

            float sc_x = c_x - start_x; 
            float sc_y = c_y - start_y; 
            float se_x = end_x - start_x;
            float se_y = end_y - start_y;

            float sc_proj_scale = (sc_x*se_x + sc_y*se_y) / (se_x*se_x + se_y*se_y);  // scale = projection length / target vector length
            float sc_proj_x = se_x * sc_proj_scale; // projection of start->center onto start->end
            float sc_proj_y = se_y * sc_proj_scale; // projection of start->center onto start->end

            // check if projection falls on top of trajectory (start->projection = k * start->end)
            float k = abs(se_x)>abs(se_y) ? sc_proj_x/se_x : sc_proj_y/se_y; // we use the largest dimension of start->end to avoid division by 0

            if(k <= 0){
                if(sc_x*sc_x + sc_y*sc_y <= r_sq){ // check distance: center<->start
                    return true;
                }
            }else if(k >= 1){
                float ec_x = c_x - end_x;
                float ec_y = c_y - end_y;
                if(ec_x*ec_x + ec_y*ec_y <= r_sq){ // check distance: center<->end
                    return true;
                }
            }else{
                float proj_c_x = c_x - (sc_proj_x + start_x);
                float proj_c_y = c_y - (sc_proj_y + start_y);
                if(proj_c_x*proj_c_x + proj_c_y*proj_c_y <= r_sq){ // check distance: center<->projection
                    return true;
                }
            }
        }
    }

    float path_x = end_x - start_x;
    float path_y = end_y - start_y;

    final_path_size = 6;
    final_path[0] = start_x;
    final_path[1] = start_y;
    final_path[2] = end_x;
    final_path[3] = end_y;
    final_path[4] = 3; // status: 3-no obstacles
    final_path[5] = sqrtf(path_x*path_x+path_y*path_y) + max(0.f, e_cost/10.f); // min. A* cost from start to end (e_cost is added even if start==end to help debug cell costs)

    return false; // no obstruction was found
}

// opponent players + active player + restricted areas (from referee)
// data: 
// [start x][start y]
// [allow out of bounds?][go to goal?]
// [optional target x][optional target y]
// [timeout]
// [x][y][hard radius][soft radius][force]
void astar(float params[], int params_size){

    auto t1 = high_resolution_clock::now();

    const float s_x = params[0]; // start x
    const float s_y = params[1]; // start y
    const bool allow_out_of_bounds = params[2];
    const int wall_index = allow_out_of_bounds ? -3 : -2; // (cost <= wall_index) means 'unreachable'
    const bool go_to_goal = params[3];
    const float opt_t_x = params[4]; // optional target x
    const float opt_t_y = params[5]; // optional target y
    const int timeout_us = params[6];
    float* obstacles = &params[7];
    int obst_size = params_size-7; // size of obstacles array

    //======================================================== Populate board 0: add field layout
    float board_cost[LINES*COLS] = {L0_1,L2_5,L6_10,L11,LIN12_308,L309,L310_314,L2_5,L0_1};

    if (!allow_out_of_bounds){ // add cost to getting near sideline or endline (except near goal)
        add_space_cushion(board_cost);
    }

    //======================================================== Check if path is obstructed

    if (!is_path_obstructed(s_x, s_y, opt_t_x, opt_t_y, obstacles, obst_size, go_to_goal, wall_index, board_cost)){
        return; // return if path is not obstructed
    }
    
    //======================================================== Define board basics (start, end, limits)

    // if the start point is out of field, it is brought in
    const int start_l = x_to_line(s_x);
    const int start_c = y_to_col(s_y);
    const int start_pos = start_l * COLS + start_c;

    // define objective (go to goal or a specific point)
    int end_l, end_c;
    if(!go_to_goal){
        end_l = x_to_line(opt_t_x);
        end_c = y_to_col(opt_t_y);
    }else{
        end_l = IN_GOAL_LINE;
    }

    // define board limits considering the initial and final positions (and obstacles in the next section, and goals after that)
    int l_min = min(start_l, end_l);
    int l_max = max(start_l, end_l);
    int c_min, c_max;
    if(go_to_goal){
        c_min = min(start_c,119);
        c_max = max(start_c,101);
    }else{
        c_min = min(start_c, end_c);
        c_max = max(start_c, end_c);
    } 

    if (!allow_out_of_bounds){ // workspace must contain a bit of empty field if out of bounds is not allowed
        l_min = min(l_min, 306);
        l_max = max(14, l_max);
        c_min = min(c_min, 206);
        c_max = max(14, c_max);
    }

    //======================================================== Initialize A*

    Node* open_root = nullptr;
    Node board[LINES*COLS];
    unsigned int node_state[LINES*COLS] = {0}; //0-unknown, 1-open, 2-closed

    //======================================================== Populate board 1: convert obstacles to cost
    for(int ob=0; ob<obst_size; ob+=5){
        int lin = x_to_line(obstacles[ob]);
        int col = y_to_col(obstacles[ob+1]);
        float hard_radius = fmaxf( 0, fminf(obstacles[ob+2], MAX_RADIUS) );
        float soft_radius = fmaxf( 0, fminf(obstacles[ob+3], MAX_RADIUS) );
        float force = obstacles[ob+4];
        float f_per_m = force / soft_radius; // force per meter

        int max_r = int( fmaxf(hard_radius, soft_radius)*10.f+1e-4 ); // add epsilon to avoid potential rounding error in expansion groups
        l_min = min(l_min,  lin - max_r - 1  );
        l_max = max(l_max,  lin + max_r + 1  );
        c_min = min(c_min,  col - max_r - 1  );
        c_max = max(c_max,  col + max_r + 1  );

        //=============================================================== hard radius
        int i=0;
        for(; i<expansion_positions_no and expansion_pos_dist[i] <= hard_radius; i++){
            int l = lin + expansion_pos_l[i];
            int c = col + expansion_pos_c[i];
            if(l>=0 and c>=0 and l<LINES and c<COLS){
                board_cost[l*COLS+c] = -3;
            }
        }

        //=============================================================== soft radius

        for(; i<expansion_positions_no and expansion_pos_dist[i] <= soft_radius; i++){
            int l = lin + expansion_pos_l[i];
            int c = col + expansion_pos_c[i];
            int p = l*COLS+c;
            float cost = board_cost[p];
            float fr = force-(f_per_m * expansion_pos_dist[i]);

            if(l>=0 and c>=0 and l<LINES and c<COLS and cost > wall_index and cost < fr){
                board_cost[p] = fr;
            }
        }
    }

    // adjust board limits if working area overlaps goal area (which includes walking margin)
    if (c_max > 96 and c_min < 124){ // Otherwise it does not overlap any goal
        if (l_max > 1 and l_min < 12 ){ // Overlaps our goal
            l_max = max(12,l_max);      // Extend working area to include our goal
            l_min = min(l_min,1);
            c_max = max(124,c_max);
            c_min = min(c_min,96);
        }
        if (l_max > 308 and l_min < 319 ){  // Overlaps their goal
            l_max = max(319,l_max);         // Extend working area to include their goal
            l_min = min(l_min,308);
            c_max = max(124,c_max);
            c_min = min(c_min,96);
        }
    }


    //======================================================== Populate board 2: add objective
    // Explanation: if objective is not accessible we do not add it to the map, although its reference still exists.
    // Therefore, we know how far we are from that reference, but it cannot be reached.
    // Even if we are on top of the objective, the idea is to get away, to the nearest valid position.
    if(!go_to_goal){
        float *end_cost = &board_cost[end_l*COLS+end_c];
        if(*end_cost > wall_index){
            *end_cost = -1;
        }
    }else{
        for(int i=IN_GOAL_LINE*COLS+101; i<=IN_GOAL_LINE*COLS+119; i++){
            if(board_cost[i] > wall_index){
                board_cost[i] = -1;
            }
        }
    }

    // add board limits as an additional restriction to workspace
    l_min = max(0, l_min);
    l_max = min(l_max, 320);
    c_min = max(0, c_min);
    c_max = min(c_max, 220);
    
    // add start node to open list (it will be closed right away, so there is not need to set it as open)
    board[start_pos].g = 0; // This is needed to compute the cost of child nodes, but f is not needed because there are no comparisons with other nodes in the open BST
    board[start_pos].parent = nullptr; //This is where the path ends
    open_root = open::insert(&board[start_pos], open_root);
    int measure_timeout=0;
    Node* best_node = &board[start_pos]; // save best node based on distance to goal (useful if impossible/timeout to get best path)

    float best_node_dist = std::numeric_limits<float>::max(); // infinite distance if start is itself unreachable
    if(board_cost[start_pos] > wall_index){
        best_node_dist = diagonal_distance(go_to_goal,start_l,start_c,end_l,end_c);
    }
    
    
   
    //======================================================== A* algorithm
    while (open_root != nullptr){

        // Get next best node (lowest predicted total cost (f))
        Node* curr_node = min_node;
        const int curr_pos = curr_node - board; 
        const int curr_line = curr_pos / COLS;
        const int curr_col  = curr_pos % COLS;
        const float curr_cost = board_cost[curr_pos];
        measure_timeout = (measure_timeout+1) & 31; // check timeout at every 32 iterations

        // save best node based on distance to goal (useful if impossible/timeout)
        if(curr_cost > wall_index){ 
            float dd = diagonal_distance(go_to_goal,curr_line,curr_col,end_l,end_c);
            if(best_node_dist > dd){
                best_node = curr_node;
                best_node_dist = dd;
            }
        }


        open_root = open::pop(open_root);
        node_state[curr_pos] = 2;

        // Check if we reached objective
        if( curr_cost == -1 ){
            // replace end point with correct coordinates instead of discrete version if the optional target was defined (not going to goal)
            build_final_path(best_node, board, 0, !go_to_goal, opt_t_x, opt_t_y);
            return;
        }
        if( measure_timeout==0 and duration_cast<microseconds>(high_resolution_clock::now() - t1).count() > timeout_us ){         
            build_final_path(best_node, board, 1);
            return;
        }

        // Expand child nodes
        bool rcol_ok = curr_col < c_max;
        bool lcol_ok = curr_col > c_min;

        if(curr_line > l_min){
            int line = curr_line - 1;
            int col  = curr_col  - 1;
            int pos = curr_pos - COLS - 1;
            float cost = board_cost[pos]; 
            int state = node_state[pos];

            // check if not an obstacle and if node is not closed (child can be as inaccessible as current pos)
            if (state!=2 and !(cost <= wall_index and cost < curr_cost) and lcol_ok){
                open_root = expand_child(open_root,cost,wall_index,curr_node,board,pos,state,go_to_goal,line,col,end_l,end_c,node_state,SQRT2);
            }

            col++;
            pos++;
            cost = board_cost[pos];
            state = node_state[pos];

            // check if not an obstacle and if node is not closed (child can be as inaccessible as current pos)
            if (state!=2 and !(cost <= wall_index and cost < curr_cost)){
                open_root = expand_child(open_root,cost,wall_index,curr_node,board,pos,state,go_to_goal,line,col,end_l,end_c,node_state,1);
            }

            col++;
            pos++;
            cost = board_cost[pos];
            state = node_state[pos];

            // check if not an obstacle and if node is not closed (child can be as inaccessible as current pos)
            if (state!=2 and !(cost <= wall_index and cost < curr_cost) and rcol_ok){
                open_root = expand_child(open_root,cost,wall_index,curr_node,board,pos,state,go_to_goal,line,col,end_l,end_c,node_state,SQRT2);
            }

        }
        
        if(curr_line < l_max){
            int line = curr_line + 1;
            int col  = curr_col  - 1;
            int pos = curr_pos + COLS - 1;
            float cost = board_cost[pos]; 
            int state = node_state[pos];

            // check if not an obstacle and if node is not closed (child can be as inaccessible as current pos)
            if (state!=2 and !(cost <= wall_index and cost < curr_cost) and lcol_ok){
                open_root = expand_child(open_root,cost,wall_index,curr_node,board,pos,state,go_to_goal,line,col,end_l,end_c,node_state,SQRT2);
            }

            col++;
            pos++;
            cost = board_cost[pos];
            state = node_state[pos];

            // check if not an obstacle and if node is not closed (child can be as inaccessible as current pos)
            if (state!=2 and !(cost <= wall_index and cost < curr_cost)){
                open_root = expand_child(open_root,cost,wall_index,curr_node,board,pos,state,go_to_goal,line,col,end_l,end_c,node_state,1);
            }

            col++;
            pos++;
            cost = board_cost[pos];
            state = node_state[pos];

            // check if not an obstacle and if node is not closed (child can be as inaccessible as current pos)
            if (state!=2 and !(cost <= wall_index and cost < curr_cost) and rcol_ok){
                open_root = expand_child(open_root,cost,wall_index,curr_node,board,pos,state,go_to_goal,line,col,end_l,end_c,node_state,SQRT2);
            }
        }


        {
            int col = curr_col - 1;
            int pos = curr_pos - 1;
            float cost = board_cost[pos]; 
            int state = node_state[pos];

            // check if not an obstacle and if node is not closed (child can be as inaccessible as current pos)
            if (state!=2 and !(cost <= wall_index and cost < curr_cost) and lcol_ok){
                open_root = expand_child(open_root,cost,wall_index,curr_node,board,pos,state,go_to_goal,curr_line,col,end_l,end_c,node_state,1);
            }

            col+=2;
            pos+=2;
            cost = board_cost[pos];
            state = node_state[pos];


            // check if not an obstacle and if node is not closed (child can be as inaccessible as current pos)
            if (state!=2 and !(cost <= wall_index and cost < curr_cost) and rcol_ok){
                open_root = expand_child(open_root,cost,wall_index,curr_node,board,pos,state,go_to_goal,curr_line,col,end_l,end_c,node_state,1);
            }
        }   

 
    }
    
    build_final_path(best_node, board, 2);
    return;
}