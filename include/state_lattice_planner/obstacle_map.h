#ifndef __OBSTACLE_MAP_H
#define __OBSTACLE_MAP_H

#include <iostream>
#include <vector>

namespace state_lattice_planner
{

 //
 //            x
 //            ^
 //            | height
 //            |
 //    y <- - -
 //       width
 //

template<typename ELEMENT_TYPE>
class ObstacleMap
{
public:
    ObstacleMap(void);
    unsigned int get_index_from_xy(double, double) const;
    void set_shape(unsigned int, unsigned int, double);
    double get_resolution(void) const;

    std::vector<ELEMENT_TYPE> data;
protected:
    unsigned int width;// cells
    unsigned int height;// cells
    double resolution;// m/cell
    double origin_x;// m
    double origin_y;// m
};

template<typename ELEMENT_TYPE>
ObstacleMap<ELEMENT_TYPE>::ObstacleMap(void)
{
    width = 0;
    height = 0;
    resolution = 0.0;
    origin_x = 0.0;
    origin_y = 0.0;
    data.clear();
}

template<typename ELEMENT_TYPE>
unsigned int ObstacleMap<ELEMENT_TYPE>::get_index_from_xy(double x, double y) const
{
    unsigned int index = 0;
    index = std::round((x - origin_x) / resolution) + std::round((y - origin_y) / resolution) * height;
    return index;
}

template<typename ELEMENT_TYPE>
void ObstacleMap<ELEMENT_TYPE>::set_shape(unsigned int width_, unsigned int height_, double resolution_)
{
    if(width_ > 0 && height_ > 0){
        width = width_;
        height = height_;
    }else{
        std::cout << "\033[31mvalue error: width and height must be > 0\033[0m" << std::endl;
        return;
    }
    if(resolution_ > 0.0){
        resolution = resolution_;
    }else{
        std::cout << "\033[31mvalue error: resolution must be > 0.0 m\033[0m" << std::endl;
        return;
    }
    // origin is in the lower right-hand corner of the map.
    origin_x = -static_cast<double>(height) * 0.5 * resolution;
    origin_y = -static_cast<double>(width) * 0.5 * resolution;
}

template<typename ELEMENT_TYPE>
double ObstacleMap<ELEMENT_TYPE>::get_resolution(void) const
{
    return resolution;
}

}

#endif// __OBSTACLE_MAP_H
