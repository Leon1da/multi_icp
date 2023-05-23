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
        int _n_beams;
        vector<float> _values;
    
    public:
        DatasetRecord(float ts, vector<float> pose, vector<float> offset, float range_min, float range_max, float angle_min, float angle_max, int n_beams, vector<float> values);
        ~DatasetRecord();

        float ts();
        vector<float> pose();
        vector<float> offset();
        float range_min();
        float range_max();
        float angle_min();
        float angle_max();
        int n_beams();
        vector<float> values();
    

        


        
};

class Dataset{
    private:
        string _filename;
        vector<DatasetRecord> _records;

    public:
        Dataset(string filename);
        ~Dataset();
        DatasetRecord decode_line(string line);
        vector<DatasetRecord> records();

};


Pose2D record_to_pose(DatasetRecord record);
vector<Point2D> record_to_points(DatasetRecord record);
