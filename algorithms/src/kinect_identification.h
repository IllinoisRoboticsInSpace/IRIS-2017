// KINECT OBSTACLE DETECTION MODULE
// andres.r.reina@gmail.com
// singhrohit2@hotmail.com
// IRIS at UIUC 2015 **/

/**======================**/
/**TELLS US THE STEEPNESS FACTOR OF A PIECE OF TERRAIN BY COMPARING THE HEIGHT OF ADJACENT PIECES**/
/**======================**/
template <typename T>
int f_isSteep(T origin, T up, T left, T down, T right, T tolerance)//written by Max Archer 9/17/2014
{
    if(origin!=map_defaultValue)
    {
        if(up==map_defaultValue)
            up = origin;
        if(left==map_defaultValue)
            left = origin;
        if(down==map_defaultValue)
            down = origin;
        if(right==map_defaultValue)
            right = origin;

        if(up-origin >= tolerance || origin-up >= tolerance)
            return map_occupied;
        else if(left-origin >= tolerance || origin-left >= tolerance)
            return map_occupied;
        else if(down-origin >= tolerance || origin-down >= tolerance)
            return map_occupied;
        else if(right-origin >= tolerance || origin-right >= tolerance)
            return map_occupied;
    }
    return map_unoccupied;
}

/**======================**/
/**Uses the input map to produce a gradient of this map**/
/**======================**/
void obstacle_identification(MATRIX & output, const MATRIX& input, const float tolerance)//takes a map and gives it the gradient data
{
    
    //const float tolerance = 0.5f;
    for(int y = input.yllim()+1; y < input.yhlim()-1; ++y)
    {
        for(int x = input.xllim()+1; x < input.xhlim()-1; ++x)
        {
            if(input(x,y) != map_defaultValue)//get the point, and all points around it!
                output(x,y) = f_isSteep(input(x  , y  ),
                                        input(x  , y+1),
                                        input(x  , y-1),
                                        input(x-1, y  ),
                                        input(x+1, y  ),
                                        tolerance);
        }
    }
}
