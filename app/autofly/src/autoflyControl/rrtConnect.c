#include <stdlib.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include <math.h>
#include "debug.h"
#include "task.h"

#include "rrtConnect.h"
#include "auxiliary_tool.h"

void planning(coordinate_t* X_start, coordinate_t* X_end, octoTree_t *octoTree, octoMap_t *octoMap,array_t* result)
{
    DEBUG_PRINT("Start planning\n");
    //char* filename = "./assets/rrtPath.csv";
    //FILE *fp = fopen(filename, "w");
    // fprintf(fp, "%d,%d,%d,", X_start.x, X_start.y, X_start.z);
    // fprintf(fp, "%d,%d,%d,0\n", X_end.x, X_end.y, X_end.z);
    result->len = 0;
    if (caldistance(X_start, X_end) < MIN_DISTANCE)
        return;
    array_t current1,current2;
    // DEBUG_PRINT("2\n");
    current1.len = 0;
    current2.len = 0;
    addToArray_coordinate(&current1, X_start, -1);
    addToArray_coordinate(&current2, X_end, -1);
    vertex_t X_rand, X_new_1,X_new_2;
    short near_index_1 = -1,near_index_2 = -1;
    vertex_t *X_near_1,*X_near_2;
    vertex_t *X_connect_1 = &current1.arr[0],*X_connect_2 = &current2.arr[0];
    for (int i = 0; i < ITER_MAX; ++i)
    {
        vTaskDelay(200);
        DEBUG_PRINT("i:%d\n",i);
        // printf("probability_end: %d",octoTreeGetLogProbability(octoTree, octoMap, &X_end));
        generate_random_node(&X_rand.loc);
        // printf("X_rand %d,%d,%d\n",X_rand.loc.x,X_rand.loc.y,X_rand.loc.z);
        near_index_1 = find_nearest_neighbor(&X_rand.loc, &current1);
        near_index_2 = find_nearest_neighbor(&X_rand.loc, &current2);
        X_near_1 = &current1.arr[near_index_1];
        X_near_2 = &current2.arr[near_index_2];
        if(caldistance(&X_near_1->loc,&X_rand.loc)<0.5 || caldistance(&X_near_2->loc,&X_rand.loc)<0.5)
            continue;
        // printf("X_near %d,%d,%d\n",X_near->loc.x,X_near->loc.y,X_near->loc.z);
        steer(&X_near_1->loc, &X_rand.loc, &X_new_1);
        steer(&X_near_2->loc, &X_rand.loc, &X_new_2);
        // printf("X_end %d,%d,%d\n",X_end.x,X_end.y,X_end.z);
        if (obstaclefree(octoTree, octoMap, X_near_1->loc, X_new_1.loc))
        {
            X_new_1.index_parent = near_index_1;
            // fprintf(fp, "%d,%d,%d,", X_new_1.loc.x, X_new_1.loc.y, X_new_1.loc.z);
            // fprintf(fp, "%d,%d,%d,1\n", X_new_1.parent->loc.x, X_new_1.parent->loc.y, X_new_1.parent->loc.z);
            if (!addToArray_vertex(&current1, &X_new_1))
                break;
            // DEBUG_PRINT("addToRrray current1, Point:%d,%d,%d\n",X_new_1.loc.x,X_new_1.loc.y,X_new_1.loc.z);
            X_connect_2 = &current2.arr[find_nearest_neighbor(&X_new_1.loc, &current2)];
            if (caldistance(&X_new_1.loc, &X_connect_2->loc) <= MIN_DISTANCE)
            {
                X_connect_1 = &X_new_1;
                break;
            }
        }
        else
        {
            //DEBUG_PRINT("[rrtC]obstaclefree false\n");
        }

        if (obstaclefree(octoTree, octoMap, X_near_2->loc, X_new_2.loc))
        {
            // DEBUG_PRINT("obstaclefree true\n");
            X_new_2.index_parent = near_index_2;
            // fprintf(fp, "%d,%d,%d,", X_new_2.loc.x, X_new_2.loc.y, X_new_2.loc.z);
            // fprintf(fp, "%d,%d,%d,-1\n", X_new_2.parent->loc.x, X_new_2.parent->loc.y, X_new_2.parent->loc.z);
            if (!addToArray_vertex(&current2, &X_new_2))
                break;
            // DEBUG_PRINT("addToRrray current2, Point:%d,%d,%d\n",X_new_2.loc.x,X_new_2.loc.y,X_new_2.loc.z);
            X_connect_1 = &current1.arr[find_nearest_neighbor(&X_new_2.loc, &current1)];
            if (caldistance(&X_new_2.loc, &X_connect_1->loc) <= MIN_DISTANCE)
            {
                X_connect_2 = &X_new_2;
                break;
            }
        }
        else
        {
            //DEBUG_PRINT("[rrtC]obstaclefree false\n");
        }
    }
    // DEBUG_PRINT("4\n");
    if (caldistance(&X_connect_1->loc, &X_connect_2->loc) <= MIN_DISTANCE)
    {
        // printf("c1:(%d,%d,%d),c2:(%d,%d,%d)\n",X_connect_1->loc.x,X_connect_1->loc.y,X_connect_1->loc.z,X_connect_2->loc.x,X_connect_2->loc.y,X_connect_2->loc.z);
        //DEBUG_PRINT("put path\n");
        vertex_t *p = X_connect_1;
        while (p != NULL)
        {
            addToArray_vertex(result, p);
            if(p->index_parent == -1)
                break;
            p = &current1.arr[p->index_parent];
        }
        //reverse
        for (int i = 0; i < result->len/2; ++i)
        {
            vertex_t temp = result->arr[i];
            // printf("p1:(%d,%d,%d)\n",temp.loc.x,temp.loc.y,temp.loc.z);
            result->arr[i] = result->arr[result->len - i - 1];
            result->arr[result->len - i - 1] = temp;
        }

        p = X_connect_2;
        while (p != NULL)
        {
            // printf("p2:(%d,%d,%d)\n",p->loc.x,p->loc.y,p->loc.z);
            addToArray_vertex(result, p);
            if(p->index_parent == -1)
                break;
            p = &current2.arr[p->index_parent];
        }
        for(int i=1;i<result->len;i++)
            result->arr[i].index_parent = i-1;
    }
    DEBUG_PRINT("planning end\n");
    //writefile(&current1,filename,1);
    //writefile(&current2,filename,-1);
    //writefile(&result,filename,2);
    //fclose(fp);
    // return result;
}

void generate_random_node(coordinate_t *X_rand)
{
    // DEBUG_PRINT("generate_random_node start\n");
    X_rand->x = rand() % WIDTH_X;
    X_rand->y = rand() % WIDTH_Y;
    X_rand->z = rand() % WIDTH_Z;
    // DEBUG_PRINT("generate_random_node end\n");
}

short find_nearest_neighbor(coordinate_t *X_rand, array_t *current)
{
    // DEBUG_PRINT("find_nearest_neighbor start\n");
    short len = current->len;
    short min = 0;
    double min_distance = caldistance(X_rand, &(current->arr[0].loc));
    double distance = 0;
    for (short i = 1; i < len; ++i)
    {
        distance = caldistance(X_rand, &(current->arr[i].loc));
        if (distance < min_distance)
        {
            min_distance = distance;
            min = i;
        }
    }
    //printf("len:%d,min:%d\n", len, min);
    // DEBUG_PRINT("find_nearest_neighbor end\n");
    return min;
}

void steer(coordinate_t *X_near, coordinate_t *X_rand,vertex_t* X_new)
{
    // DEBUG_PRINT("steer start\n");
    double length = caldistance(X_near, X_rand);
    X_new->loc.x = fmax(0,fmin(X_near->x + (X_rand->x - X_near->x) * (double)STRIDE / length,WIDTH_X));
    X_new->loc.y = fmax(0,fmin(X_near->y + (X_rand->y - X_near->y) * (double)STRIDE / length,WIDTH_Y));
    X_new->loc.z = fmax(0,fmin(X_near->z + (X_rand->z - X_near->z) * (double)STRIDE / length,WIDTH_Z));
    // DEBUG_PRINT("steer end\n");
}

bool obstaclefree(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t start, coordinate_t end)
{
    // DEBUG_PRINT("obstaclefree start\n");
    float d = caldistance(&start, &end);
    // float dx = TREE_RESOLUTION * (end.x - start.x) / d;
    // float dy = TREE_RESOLUTION * (end.y - start.y) / d;
    // float dz = TREE_RESOLUTION * (end.z - start.z) / d;
    float dx = MIN_DISTANCE * (end.x - start.x) / d;
    float dy = MIN_DISTANCE * (end.y - start.y) / d;
    float dz = MIN_DISTANCE * (end.z - start.z) / d;
    int sgl_x = dx > 0 ? 1 : -1;
    int sgl_y = dy > 0 ? 1 : -1;
    int sgl_z = dz > 0 ? 1 : -1;
    int dx_i = dx;
    int dy_i = dy;
    int dz_i = dz;
    float dif_x = (dx - dx_i);
    float dif_y = (dy - dy_i);
    float dif_z = (dz - dz_i);
    // printf("dx:%f,dy:%f,dz:%f\n", dx, dy, dz);
    //printf("sgl_x %d,sgl_y %d,sgl_z %d\n", sgl_x, sgl_y, sgl_z);
    //printf("dx_i %d,dy_i %d,dz_i %d\n", dx_i, dy_i, dz_i);
    // printf("dif_x %f,dif_y %f,dif_z %f\n", dif_x, dif_y, dif_z);
    float error_x = 0;
    float error_y = 0;
    float error_z = 0;

    coordinate_t point = start;
    uint8_t probability = octoTreeGetLogProbability(octoTree, octoMap, &end);
    if(probability != LOG_ODDS_FREE)
        return false;
    // for x in range(startPoint[0], endPoint[0], step[0]):
    while (caldistance(&point, &end) > MIN_DISTANCE)
    {
        probability = 3;
        point.x += dx_i;
        error_x += dif_x;
        if (error_x >=(float)0.5 || error_x <= (float)-0.5)
        {
            point.x += sgl_x;
            error_x -= sgl_x;
        }
        point.y += dy_i;
        error_y += dif_y;
        if (error_y >= (float)0.5 || error_y <= (float)-0.5)
        {
            point.y += sgl_y;
            error_y -= sgl_y;
        }
        point.z += dz_i;
        error_z += dif_z;
        if (error_z >= (float)0.5 || error_z <= (float)-0.5)
        {
            point.z += sgl_z;
            error_z -= sgl_z;
        }

        probability = octoTreeGetLogProbability(octoTree, octoMap, &point);
        //printf("d:%f,dx:%f,dy:%f,dz:%f\n",d,dx,dy,dz);
        // printf("point:%d,%d,%d , probability:%d \n",point.x,point.y,point.z,probability);
        // printf("start:%d,%d,%d , end:%d,%d,%d \n",start.x,start.y,start.z,end.x,end.y,end.z);
        if (probability != LOG_ODDS_FREE){
            // DEBUG_PRINT("obstaclefree false end\n");
            return false;
        }
    }
    // DEBUG_PRINT("obstaclefree true end\n");
    return true;
}

BOOL addToArray_vertex(array_t *array, vertex_t* element)
{
    if (array->len >= MAX_ARRAY_SIZE)
    {
        //DEBUG_PRINT("[rrtC]Array is full\n");
        return FALSE;
    }
    array->arr[array->len].loc.x = element->loc.x;
    array->arr[array->len].loc.y = element->loc.y;
    array->arr[array->len].loc.z = element->loc.z;
    array->arr[array->len].index_parent = element->index_parent;
    array->len++;
    return TRUE;
}

BOOL addToArray_coordinate(array_t *array, coordinate_t* element, short index_p)
{
    if (array->len >= MAX_ARRAY_SIZE)
    {
        //DEBUG_PRINT("[rrtC]Array is full\n");
        return FALSE;
    }
    array->arr[array->len].loc.x = element->x;
    array->arr[array->len].loc.y = element->y;
    array->arr[array->len].loc.z = element->z;
    array->arr[array->len].index_parent = index_p;
    array->len++;
    return TRUE;
}

// void writefile(array_t* array, char* filename,int flag){
//     FILE* fp = fopen(filename, "a");
//     fprintf(fp,"%d\n",array->len);
//     for(int i=1;i<array->len;i++){
//         fprintf(fp, "%d,%d,%d", array->arr[i].loc.x, array->arr[i].loc.y, array->arr[i].loc.z);
//         fprintf(fp, ",%d,%d,%d", array->arr[i].parent->loc.x, array->arr[i].parent->loc.y, array->arr[i].parent->loc.z);  
//         fprintf(fp,",%d\n",flag);
//     }
//     fclose(fp);
//     return;
// }