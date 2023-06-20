#include "defs.h"


class DatasetRecord{
    private:
        float _ts;
        vector<float> _pose;
        vector<float> _offset;
        float _range_min;
        float _range_max;
        float _angle_min;
        float _angle_max;
        size_t _n_beams;
        vector<float> _values;
    
    public:
        DatasetRecord(float ts, vector<float> pose, vector<float> offset, float range_min, float range_max, float angle_min, float angle_max, size_t n_beams, vector<float> values);
        ~DatasetRecord();

        float ts();
        vector<float> pose();
        vector<float> offset();
        float range_min();
        float range_max();
        float angle_min();
        float angle_max();
        size_t n_beams();
        vector<float> values();
    

        


        
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
        void load_data(Vector3fVector &poses, Vector3fVector &sensor_poses, vector<Vector2fVector> &points, IntPairVector& pose_point_correspondences, vector<vector<int>>& valid_points, size_t from_records, size_t num_records_to_load);

    
};

