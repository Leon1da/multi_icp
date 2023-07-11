#include "defs.h"


class DatasetRecord{
    private:
        float _ts;
        Vector3d _pose;
        Vector3d _offset;
        float _range_min;
        float _range_max;
        float _angle_min;
        float _angle_max;
        size_t _n_beams;
        Eigen::VectorXd _values;
    
    public:
        DatasetRecord(float ts, Vector3d pose, Vector3d offset, float range_min, float range_max, float angle_min, float angle_max, size_t n_beams, Eigen::VectorXd values);
        ~DatasetRecord();

        float ts();
        Vector3d pose();
        Vector3d offset();
        float range_min();
        float range_max();
        float angle_min();
        float angle_max();
        size_t n_beams();
        Eigen::VectorXd values();
    

        


        
};

class Dataset{
    private:
        string _filename;
        vector<DatasetRecord> _records;
        size_t _num_records;
        
    public:
        Dataset(string filename);
        ~Dataset();
        DatasetRecord decode_line(string line);
        vector<DatasetRecord> records();
        size_t num_records();        
        void load_data(Vector3dVector& poses, Vector3dVector& sensor_poses, Vector2dVector& points, vector<vector<MapPoint>>& map, size_t from_records, size_t num_records_to_load, double min_pose_distance_threshold=DBL_MIN);

    
};

