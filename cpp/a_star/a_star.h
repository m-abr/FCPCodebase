#pragma once

/**
 * FILENAME:     a_star.h
 * DESCRIPTION:  custom A* pathfinding implementation, optimized for the soccer environment
 * AUTHOR:       Miguel Abreu (m.abreu@fe.up.pt)
 * DATE:         2022
 */

struct Node{

    //------------- BST parameters
    Node* left;
    Node* right;
    Node* up;

    //------------- A* parameters 
    Node* parent;
    float g;
    float f;

};

extern void astar(float params[], int params_size);
extern float final_path[2050];
extern int final_path_size;