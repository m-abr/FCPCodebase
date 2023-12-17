/**
 * FILENAME:     Line6f
 * DESCRIPTION:  Simple line (segment) class
 * AUTHOR:       Miguel Abreu (m.abreu@fe.up.pt)
 * DATE:         2021
 * 
 * Immutable Class for:
 * 3D Line composed of 2 Vector3f points
 * Optimized for:
 * - regular access to cartesian coordinates
 * - regular access to line segment length
 **/

#pragma once
#include "Vector3f.h"


class Line6f {
public:

    /**
     * Polar start coordinate
     */
    const Vector3f startp;

    /**
     * Polar end coordinate
     */
    const Vector3f endp;

    /**
     * Cartesian start coordinate
     */
    const Vector3f startc;

    /**
     * Cartesian end coordinate
     */
    const Vector3f endc;

    /**
     * Length of line segment limited by (start,end)
     */
    const float length;

    /**
     * Constructor
     * @param polar_s polar start point
     * @param polar_e polar end point
     */
    Line6f(const Vector3f &polar_s, const Vector3f &polar_e);

    /**
     * Constructor
     * @param cart_s cartesian start point
     * @param cart_e cartesian end point
     * @param length line segment length
     */
    Line6f(const Vector3f &cart_s, const Vector3f &cart_e, float length);

    /**
     * Copy Constructor
     * @param obj another line
     */
    Line6f(const Line6f &obj);

    /**
     * Find point in line defined by (start,end) closest to given point
     * @param cp cartesian point
     * @return point in this infinite line closest to given point
     */
    Vector3f linePointClosestToCartPoint(const Vector3f &cp) const;

    /**
     * Find point in line defined by (start,end) closest to given point
     * @param pp polar point
     * @return point in this infinite line closest to given point
     */
    Vector3f linePointClosestToPolarPoint(const Vector3f &pp) const;

    /**
     * Find distance between line defined by (start,end) and given point
     * @param cp cartesian point
     * @return distance between line defined by (start,end) and given point
     */
    float lineDistToCartPoint(const Vector3f &cp) const;

    /**
     * Find distance between line defined by (start,end) and given point
     * @param pp polar point
     * @return distance between line defined by (start,end) and given point
     */
    float lineDistToPolarPoint(const Vector3f &pp) const;

    /**
     * Find distance between line defined by (start,end) and given line
     * @param l line
     * @return distance between line defined by (start,end) and given line
     */
    float lineDistToLine(const Line6f &l) const;

    /**
     * Find point in line defined by (start,end) closest to given point
     * @param cp cartesian point
     * @return point in this infinite line closest to given point
     */
    Vector3f segmentPointClosestToCartPoint(const Vector3f &cp) const;

    /**
     * Find point in line defined by (start,end) closest to given point
     * @param pp polar point
     * @return point in this infinite line closest to given point
     */
    Vector3f segmentPointClosestToPolarPoint(const Vector3f &pp) const;

    /**
     * Find distance between line segment limited by (start,end) and given point
     * @param cp cartesian point
     * @return distance between line segment limited by (start,end) and given point
     */
    float segmentDistToCartPoint(const Vector3f &cp) const;

    /**
     * Find distance between line segment limited by (start,end) and given point
     * @param pp polar point
     * @return distance between line segment limited by (start,end) and given point
     */    
    float segmentDistToPolarPoint(const Vector3f &pp) const;

    /**
     * Find distance between line segment limited by (start,end) and given line segment 
     * @param other line segment
     * @return distance between line segment limited by (start,end) and given line segment 
     */
    float segmentDistToSegment(const Line6f &other) const;

    /**
     * Find midpoint of line segment limited by (start,end)
     * @return cartesian midpoint of line segment limited by (start,end)
     */
    Vector3f midPointCart() const;

    /**
     * Find midpoint of line segment limited by (start,end)
     * @return polar midpoint of line segment limited by (start,end)
     */
    Vector3f midPointPolar() const;

    /**
     * Operator ==
     * @param other line
     * @return true if both lines are the same
     */
    bool  operator==(const Line6f& other) const;

    /**
     * Get polar line ending by index
     * @param index (0)->start or (1)->end
     * @return polar line ending according to given index
     */
    const Vector3f &get_polar_pt(const int index) const;

    /**
     * Get cartesian line ending by index
     * @param index (0)->start or (1)->end
     * @return cartesian line ending according to given index
     */
    const Vector3f &get_cart_pt(const int index) const;   

    /**
     * Get polar vector (end-start)
     * @return polar vector (end-start)
     */
    Vector3f get_polar_vector() const;

    /**
     * Get cartesian vector (end-start)
     * @return cartesian vector (end-start)
     */
    Vector3f get_cart_vector() const;



        
};