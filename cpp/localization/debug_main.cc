#include <iostream>
#include "Geometry.h"
#include "Vector3f.h"
#include "Matrix4D.h"
#include "FieldNoise.h"
#include "Line6f.h"
#include "World.h"
#include "Field.h"
#include "LocalizerV2.h"

using namespace std;

static LocalizerV2& loc = SLocalizerV2::getInstance();

void print_python_data(){

    static World &world = SWorld::getInstance();

    cout << "Foot touch: " << world.foot_touch[0] << " " << world.foot_touch[1] << endl;
    cout << "LFoot contact rpos: " << world.foot_contact_rel_pos[0].x << " " << world.foot_contact_rel_pos[0].y << " " << world.foot_contact_rel_pos[0].z << endl;
    cout << "RFoot contact rpos: " << world.foot_contact_rel_pos[1].x << " " << world.foot_contact_rel_pos[1].y << " " << world.foot_contact_rel_pos[1].z << endl;
    cout << "Ball seen: " << world.ball_seen << endl;
    cout << "Ball rpos cart: " << world.ball_rel_pos_cart.x << " " << world.ball_rel_pos_cart.y << " " << world.ball_rel_pos_cart.z << endl;
    cout << "Ball cheat: " << world.ball_cheat_abs_cart_pos.x << " " << world.ball_cheat_abs_cart_pos.y << " " << world.ball_cheat_abs_cart_pos.z << endl;
    cout << "Me cheat: " << world.my_cheat_abs_cart_pos.x << " " << world.my_cheat_abs_cart_pos.y << " " << world.my_cheat_abs_cart_pos.z << endl;
    
    for(int i=0; i<8; i++){
        cout << "Landmark " << i << ": " <<
        world.landmark[i].seen << " " <<
        world.landmark[i].isCorner << " " <<
        world.landmark[i].pos.x << " " <<
        world.landmark[i].pos.y << " " <<
        world.landmark[i].pos.z << " " <<
        world.landmark[i].rel_pos.x << " " <<
        world.landmark[i].rel_pos.y << " " <<
        world.landmark[i].rel_pos.z << endl;
    }

    for(int i=0; i<world.lines_polar.size(); i++){
        cout << "Line " << i << ": " <<
        world.lines_polar[i].start.x << " " << 
        world.lines_polar[i].start.y << " " << 
        world.lines_polar[i].start.z << " " << 
        world.lines_polar[i].end.x << " " << 
        world.lines_polar[i].end.y << " " << 
        world.lines_polar[i].end.z << endl;
    }
}

float *compute(bool lfoot_touch, bool rfoot_touch, 
            double feet_contact[],
            bool ball_seen, double ball_pos[],
            double me_pos[],
            double landmarks[],
            double lines[],
            int lines_no){

    // ================================================= 1. Parse data
    
    static World &world = SWorld::getInstance();
    world.foot_touch[0] = lfoot_touch;
    world.foot_touch[1] = rfoot_touch;

    //Structure of feet_contact {lfoot_contact_pt, rfoot_contact_pt, lfoot_contact_rel_pos, rfoot_contact_rel_pos}

    world.foot_contact_rel_pos[0].x = feet_contact[0];
    world.foot_contact_rel_pos[0].y = feet_contact[1];
    world.foot_contact_rel_pos[0].z = feet_contact[2];
    world.foot_contact_rel_pos[1].x = feet_contact[3];
    world.foot_contact_rel_pos[1].y = feet_contact[4];
    world.foot_contact_rel_pos[1].z = feet_contact[5];

    world.ball_seen = ball_seen;

    //Structure of ball_pos {ball_rel_pos_cart, ball_cheat_abs_cart_pos}

    world.ball_rel_pos_cart.x = ball_pos[0];
    world.ball_rel_pos_cart.y = ball_pos[1];
    world.ball_rel_pos_cart.z = ball_pos[2];
    world.ball_cheat_abs_cart_pos.x = ball_pos[3];
    world.ball_cheat_abs_cart_pos.y = ball_pos[4];
    world.ball_cheat_abs_cart_pos.z = ball_pos[5];
    
    world.my_cheat_abs_cart_pos.x = me_pos[0];
    world.my_cheat_abs_cart_pos.y = me_pos[1];
    world.my_cheat_abs_cart_pos.z = me_pos[2];

    for(int i=0; i<8; i++){
        world.landmark[i].seen = (bool) landmarks[0];
        world.landmark[i].isCorner = (bool) landmarks[1];
        world.landmark[i].pos.x = landmarks[2];
        world.landmark[i].pos.y = landmarks[3];
        world.landmark[i].pos.z = landmarks[4];
        world.landmark[i].rel_pos.x = landmarks[5];
        world.landmark[i].rel_pos.y = landmarks[6];
        world.landmark[i].rel_pos.z = landmarks[7];
        landmarks += 8;
    }


    world.lines_polar.clear();

    for(int i=0; i<lines_no; i++){
        Vector3f s(lines[0],lines[1],lines[2]);
        Vector3f e(lines[3],lines[4],lines[5]);
        world.lines_polar.emplace_back(s, e); 
        lines += 6;
    }

    print_python_data();
    
    // ================================================= 2. Compute 6D pose

    loc.run(); 
    
    // ================================================= 3. Prepare data to return
    
    float retval[35];
    float *ptr = retval;

    for(int i=0; i<16; i++){
        ptr[i] = loc.headTofieldTransform.content[i];
    }
    ptr += 16;
    for(int i=0; i<16; i++){
        ptr[i] = loc.fieldToheadTransform.content[i];
    }
    ptr += 16;

    ptr[0] = (float) loc.is_uptodate;
    ptr[1] = loc.head_z;
    ptr[2] = (float) loc.is_head_z_uptodate;


    return retval;
}

void print_report(){
    loc.print_report();
}

void draw_visible_elements(bool is_right_side){
    Field& fd = SField::getInstance();
    fd.draw_visible(loc.headTofieldTransform, is_right_side);
}

int main(){

    double feet_contact[] = {0.02668597,  0.055     , -0.49031584,  0.02668597, -0.055     , -0.49031584};
    double ball_pos[] =     {22.3917517 ,  4.91904904, -0.44419865, -0.        , -0.        , 0.04 };
    double me_pos[] =       {-22.8 ,  -2.44,   0.48};
    double landmarks[] =    { 1.  ,   1.  , -15.  , -10.  ,   0.  ,  10.88, -37.74,  -2.42,
                              0.  ,   1.  , -15.  ,  10.  ,   0.  ,   0.  ,   0.  ,   0.  ,
                              1.  ,   1.  ,  15.  , -10.  ,   0.  ,  38.56,  -4.9 ,  -0.66,
                              1.  ,   1.  ,  15.  ,  10.  ,   0.  ,  39.75,  24.4 ,  -0.7 ,
                              1.  ,   0.  , -15.  ,  -1.05,   0.8 ,   7.94,  16.31,   2.42,
                              1.  ,   0.  , -15.  ,   1.05,   0.8 ,   8.55,  30.15,   2.11,
                              1.  ,   0.  ,  15.  ,  -1.05,   0.8 ,  37.82,   8.16,   0.5 ,
                              1.  ,   0.  ,  15.  ,   1.05,   0.8 ,  37.94,  11.77,   0.44 };
    double lines[] =        { 25.95,  35.02,  -1.14,  24.02, -12.26,  -1.12,
                              13.18,  59.93,  -2.11,  10.87, -37.8 ,  -2.69,
                              39.78,  24.32,  -0.75,  38.64,  -5.05,  -0.67,
                              10.89, -37.56,  -2.6 ,  38.52,  -5.24,  -0.68,
                              15.44,  59.85,  -1.87,  39.76,  24.77,  -0.88,
                               9.62,   3.24,  -2.67,  11.02,  36.02,  -2.54,
                               9.63,   2.82,  -3.16,   7.82,   2.14,  -3.67,
                              11.02,  36.09,  -2.61,   9.51,  41.19,  -2.94,
                              36.03,   5.33,  -0.66,  36.46,  14.9 ,  -0.74,
                              35.94,   5.43,  -0.72,  37.81,   5.26,  -0.73,
                              36.42,  14.72,  -0.83,  38.16,  14.68,  -0.85,
                              20.93,  13.26,  -1.33,  21.25,   9.66,  -1.15,
                              21.21,   9.75,  -1.6 ,  22.18,   7.95,  -1.19,
                              22.21,   7.94,  -1.17,  23.43,   7.82,  -1.11,
                              23.38,   7.55,  -1.18,  24.42,   9.47,  -1.16,
                              24.43,   9.37,  -1.25,  24.9 ,  11.72,  -0.98,
                              24.89,  11.73,  -1.2 ,  24.68,  14.54,  -1.05,
                              24.7 ,  14.85,  -1.06,  23.85,  16.63,  -1.1 ,
                              23.82,  16.53,  -1.14,  22.61,  17.14,  -1.32,
                              22.65,  17.53,  -1.23,  21.5 ,  16.19,  -1.34,
                              21.49,  15.92,  -1.32,  20.95,  13.07,  -1.32 };

    int lines_no = sizeof(lines)/sizeof(lines[0])/6;

    compute(true, // lfoot_touch
            true, // rfoot_touch
            feet_contact,
            true, // ball_seen
            ball_pos, 
            me_pos,
            landmarks,
            lines,
            lines_no);

}


