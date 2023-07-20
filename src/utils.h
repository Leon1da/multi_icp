# include "defs.h"

bool estimate_normal(Vector2dVector &points, IntVector& indices, double &angle);
bool estimate_normal(Vector2dVector &points, IntVector& indices, Vector2d &normal);




enum ARGS_LABEL{
    DATASET_PATH_LABEL,
    POINTS_KDTREE_DIM_LABEL,
    POSES_KDTREE_DIM_LABEL
};

enum ARGS_DEFAULT{
    DATASET_PATH_DEFAULT,
    POINTS_KDTREE_DIM_DEFAULT,
    POSES_KDTREE_DIM_DEFAULT
};


// class ARGS{
//     public:
//         args
// }

// class CONFIG {
//     public:
//         inline static const string DATASET_PATH = "/home/leonardo/multi_icp/dataset/dataset_test.txt";
//         inline static const int POINTS_KDTREE_DIM = 2;
//         inline static const int POSES_KDTREE_DIM = 2;
// };