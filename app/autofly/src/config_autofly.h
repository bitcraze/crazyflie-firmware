// Autofly_Main config
#define OFFSET_X 50
#define OFFSET_Y 50
#define OFFSET_Z 0
#define TOP 65 //obstacle top
#define BOTTOM 25 //obstacle bottom
#define AVOID_DISTANCE 15 // obstacle avoidance distance
#define DIRECTION_AWARD 1.2 // factor of direction reward 
#define DISCIPLINE 0 // Penalty number without information entropy
#define INCOME_INFO_TIMES 1 // coefficient of information entropy
#define COST_PRUNE_TIMES 1 // coefficient of pruning benefit
#define MAX_LOOP 5  // The number of judgments that trigger jumping out of the local optimum
#define WINDOW_SIZE 30 
#define MAXRUN 300
#define RELIABILITY_DISTANCE 24

#define WAIT_DELAY 300
#define LOOP_DELAY 50
#define PROBABILITY_MEM(octomap) (double)octomap->octoNodeSet->length / NODE_SET_SIZE
#define PRESENT_JUMP 2
#define SPEED 0.25

#define MAPPING_DIF 100
#define EXPLORE_DIF 300
#define TERMINATE_DIF 500
#define MAPPING_MAX 100
#define EXPLORE_MAX MAXRUN*2

//octomap config
#define TREE_CENTER_X 128
#define TREE_CENTER_Y 128
#define TREE_CENTER_Z 128
#define WIDTH_X TREE_CENTER_X * 2
#define WIDTH_Y TREE_CENTER_Y * 2
#define WIDTH_Z TREE_CENTER_Z * 2
#define TREE_RESOLUTION 4
#define TREE_MAX_DEPTH 6
#define NODE_SET_SIZE 3000

#define LOG_ODDS_OCCUPIED 6
#define LOG_ODDS_FREE 0
#define LOG_ODDS_UNKNOWN 3
#define LOG_ODDS_OCCUPIED_FLAG 1
#define LOG_ODDS_FREE_FLAG 0
#define LOG_ODDS_DIFF_STEP 3

//auxiliary_tool config
#define SENSOR_TH 300
#define P_GLOBAL 0.5
#define MIN_OCCUPIED 5
#define MAX_NOT_OCCUPIED 2

//rrtConnect config
#define ITER_MAX 200
#define MAX_ARRAY_SIZE ITER_MAX
#if TREE_RESOLUTION >= 2
    #define MIN_DISTANCE TREE_RESOLUTION/2
#else
    #define MIN_DISTANCE 1
#endif
#define STRIDE 8