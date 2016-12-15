// KINECT OBSTACLE DETECTION MODULE
// andres.r.reina@gmail.com
// singhrohit2@hotmail.com
// IRIS at UIUC 2016 **/


const float cellStepTolerance = 0.5;//fraction of a cells size that a cell
//can change in height and will be marked as steep afterward

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

void getGradientMap(const MATRIX& height, MATRIX& gradient){
    int xMin = height.xllim();
    int xMax = height.xhlim();
    int yMin = height.yllim();
    int yMax = height.yhlim();
    for(int x=xMin;x<=xMax;x++){
        for(int y=yMin;y<=yMax;y++){
            if (height(x, y)!=map_defaultValue){
                float minGrad = 0;
                for(int i=-1;i<=1;i++){
                    for(int j=-1;j<=1;j++){
                        if(x+i>=xMin && x+i<=xMax && y+j>=yMin && y+j<= yMax&& height(x+i, y+j)!=map_defaultValue){
                            minGrad = fmax(minGrad,fabs(height(x+i,y+j)-height(x,y)));
                        }
                    }
                }
                gradient(x,y) = minGrad;
            }else{
                gradient(x,y) = map_defaultValue;
            }
        }
    }
}





void blur(const MATRIX& source, MATRIX& output){
    int xMin = source.xllim();
    int xMax = source.xhlim();
    int yMin = source.yllim();
    int yMax = source.yhlim();
    for(int x=xMin;x<=xMax;x++){
        for(int y=yMin;y<=yMax;y++){
            if(source(x,y)!=map_defaultValue){
                float cell = 0;
                float counter = 0;
                for(int i=-2;i<=2;i++){
                    for(int j=-2;j<=2;j++){
                        if(x+i>=xMin && x+i<=xMax && y+j>=yMin && y+j<= yMax && source(x+i,y+j)!=map_defaultValue){
                            cell+=source(x+i, y+j);
                            counter+=1;
                        }
                    }
                }
                cell /= counter;
                output(x,y)=cell;
            }else{
                output(x,y)=map_defaultValue;
            }
        }
    }
}


/**======================**/
/**Uses the input map to produce a gradient of this map**/
/**======================**/
void obstacle_identification(MATRIX & output, const MATRIX& input, MATRIX& temporary_matrix)//takes a map and gives it the gradient data
{
    getGradientMap(input,output);
    blur(output,temporary_matrix);
    //const float tolerance = 0.5f;
    for(int y = input.yllim()+1; y < input.yhlim()-1; ++y){
        for(int x = input.xllim()+1; x < input.xhlim()-1; ++x){
           output(x,y) = temporary_matrix(x,y) == map_defaultValue ? map_defaultValue : temporary_matrix(x,y)>cellStepTolerance ? 1 : 0;
        }
    }
}
