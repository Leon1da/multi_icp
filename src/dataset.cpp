#include "defs.h"

class DatasetRecord {
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

DatasetRecord::DatasetRecord(float ts, vector<float> pose, vector<float> offset, float range_min, float range_max, float angle_min, float angle_max, int n_beams, vector<float> values){
        _ts = ts;
        _pose = pose;
        _offset = offset;
        _range_min = range_min;
        _range_max = range_max;
        _angle_min = angle_min;
        _angle_max = angle_max;
        _n_beams = n_beams;
        _values = values;           
}

DatasetRecord::~DatasetRecord(){

}

float DatasetRecord::ts(){
    return _ts;
}

vector<float> DatasetRecord::pose(){
    return _pose;
}

vector<float> DatasetRecord::offset(){
    return _offset;
}

float DatasetRecord::range_min(){
    return _range_min;
}

float DatasetRecord::range_max(){
    return _range_max;
}

float DatasetRecord::angle_min(){
    return _angle_min;
}

float DatasetRecord::angle_max(){
    return _angle_max;
}

int DatasetRecord::n_beams(){
    return _n_beams;
}

vector<float> DatasetRecord::values(){
    return _values;
}



class Dataset
{
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
        void load_data(Vector3fVector &poses, Vector2fVector &points, IntPairVector &pose_point_correspondences, size_t from_records, size_t to_records);


};

Dataset::Dataset(string filename)
{
    _filename = filename;
    _num_records = 0;

    cout << "Reading dataset.." << endl;
    cout << "Dataset file path: " << filename << endl;

    string line;
    ifstream file(filename);
        
    while (getline(file, line)) {
    
        DatasetRecord record = decode_line(line);
        _records.push_back(record);
        _num_records = _num_records + 1;
        
    }

    // Close the file
    file.close();

    cout << "Reading dataset complete." << endl << endl;

}

Dataset::~Dataset()
{
}

vector<DatasetRecord> Dataset::records(){
    return _records;
}

size_t Dataset::num_records(){
    return _num_records;
}

DatasetRecord Dataset::decode_line(string line){

    
    size_t pos_ts = line.find("ts");
    size_t pos_pose = line.find("pose");
    size_t pos_offset = line.find("offset");
    size_t pos_range_min = line.find("range_min");
    size_t pos_range_max = line.find("range_max");
    size_t pos_angle_min = line.find("angle_min");
    size_t pos_angle_max = line.find("angle_max");
    size_t pos_n_beams = line.find("n_beams");
    size_t pos_values = line.find("values");
    
    double ts = stod(line.substr(pos_ts + 4, pos_pose - pos_ts - 4));
    
    
    vector<float> pose;
    std::istringstream pose_string(line.substr(pos_pose + 6, pos_offset - pos_pose - 6));
    copy(std::istream_iterator<float>(pose_string), std::istream_iterator<float>(), std::back_inserter(pose));
    
    vector<float> offset;
    std::istringstream offset_string(line.substr(pos_offset + 8, pos_range_min - pos_offset - 8));
    copy(std::istream_iterator<float>(offset_string), std::istream_iterator<float>(), std::back_inserter(offset));


    float range_min = stof(line.substr(pos_range_min + 11, pos_range_max - pos_range_min - 11));
    float range_max = stof(line.substr(pos_range_max + 11, pos_angle_min - pos_range_max - 11));
    float angle_min = stof(line.substr(pos_angle_min + 11, pos_angle_max - pos_angle_min - 11));
    float angle_max = stof(line.substr(pos_angle_max + 11, pos_n_beams - pos_angle_max - 11));
    int n_beams = stoi(line.substr(pos_n_beams + 9, pos_values - pos_n_beams - 9));
    
    vector<float> values;
    std::istringstream values_string(line.substr(pos_values + 8));
    copy(std::istream_iterator<float>(values_string), std::istream_iterator<float>(), std::back_inserter(values));

    DatasetRecord record(ts, pose, offset, range_min, range_max, angle_min, angle_max, n_beams, values);
    return record;
}

void Dataset::load_data(Vector3fVector &poses, Vector2fVector &points, IntPairVector &pose_point_correspondences, size_t from_records=0, size_t num_records_to_load=0){

    
    size_t num_poses = num_records();

    if (from_records < 0 || num_records_to_load < 0) {
        cout << "from_records < 0 || num_records_to_load < 0" << endl;
        return;
    }

    if (from_records >= num_poses || num_records_to_load >= num_poses){
        cout << "from_records > num_poses || num_records_to_load > num_poses" << endl;
        return;
    }

    if (from_records + num_records_to_load > num_poses){
        cout << "from_records + num_records_to_load > num_poses" << endl;
        return;
    }

    if (num_records_to_load == 0) {
        num_records_to_load = num_poses - from_records;
    }

    cout << "from_records: " << from_records << " num_records_to_load: " << num_records_to_load << endl;
    cout << "num_records: " << num_poses << endl;
   
    poses.resize(num_records_to_load);
    
    Eigen::Vector3f pose;
    Eigen::Vector2f point;
    Vector2fVector valid_points;
    IntPairVector pose_valid_point_correspondences;

    int total_beams = 0;
    float angle_total_range, beams_angle_delta, point_x, point_y, value;
    for (size_t pose_index = 0; pose_index < num_records_to_load; pose_index++)
    {
        DatasetRecord record = records()[from_records + pose_index];
        pose << record.pose()[0], record.pose()[1], record.pose()[2];
        poses[pose_index] = pose;

        angle_total_range = abs(record.angle_min()) + abs(record.angle_max());
        beams_angle_delta = angle_total_range / record.n_beams();

        for(int beam_index=0; beam_index<record.n_beams(); beam_index++){
            value = record.values()[beam_index];
            if (value <= record.range_min() || value >= record.range_max()) continue; // out of range (sensor reading is too far or too close)
            point_x = pose.x() + value * cos(pose.z() + record.angle_min() + beams_angle_delta * beam_index ); 
            point_y = pose.y() + value * sin(pose.z() + record.angle_min() + beams_angle_delta * beam_index ); 
            point << point_x, point_y;
            valid_points.push_back(point);
            pose_valid_point_correspondences.push_back(IntPair(pose_index, total_beams));
            total_beams++;
        }

        
    }
    
    size_t num_points = valid_points.size();
    points.resize(num_points);
    pose_point_correspondences.resize(num_points);
    for (size_t point_index = 0; point_index < num_points; point_index++)
    {
        points[point_index] = valid_points[point_index];
        pose_point_correspondences[point_index] = pose_valid_point_correspondences[point_index];
    }
    
    
       
}

