/**
 * FILENAME:     World
 * DESCRIPTION:  World data from Python
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


class World {
    friend class Singleton<World>;

private:

    World(){};

public:

    //Feet variables: (0) left,  (1) right
    bool foot_touch[2]; // is foot touching ground
    Vector3f foot_contact_rel_pos[2]; // foot_transform * translation(foot_contact_pt)
    
    bool ball_seen;
    Vector3f ball_rel_pos_cart;
    Vector3f ball_cheat_abs_cart_pos;
    Vector3f my_cheat_abs_cart_pos;
    
    struct sLMark {
        bool seen; 
        bool isCorner;
        Vector3f pos;
        Vector3f rel_pos;
    };
    
    sLMark landmark[8];
    
    struct sLine {
        Vector3f start, end;  
        sLine(const Vector3f& s, const Vector3f& e) : start(s), end(e) {}; 
    };
    
    vector<sLine> lines_polar;
    

};

typedef Singleton<World> SWorld;