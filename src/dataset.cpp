#include "defs.h"

class DatasetRecord {
    
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

DatasetRecord::DatasetRecord(float ts, Vector3d pose, Vector3d offset, float range_min, float range_max, float angle_min, float angle_max, size_t n_beams, Eigen::VectorXd values){
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

Vector3d DatasetRecord::pose(){
    return _pose;
}

Vector3d DatasetRecord::offset(){
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

size_t DatasetRecord::n_beams(){
    return _n_beams;
}

Eigen::VectorXd DatasetRecord::values(){
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
        void load_data(Vector3dVector& poses, Vector3dVector& sensor_poses, Vector2dVector& points, vector<vector<MapPoint>>& map, size_t from_records, size_t num_records_to_load, double min_pose_distance_threshold=DBL_MIN, double min_point_distance_threshold = DBL_MIN);


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
    
    
    // vector<double> pose;
    // std::istringstream pose_string(line.substr(pos_pose + 6, pos_offset - pos_pose - 6));
    // copy(std::istream_iterator<double>(pose_string), std::istream_iterator<double>(), std::back_inserter(pose));
    
    // vector<double> offset;
    // std::istringstream offset_string(line.substr(pos_offset + 8, pos_range_min - pos_offset - 8));
    // copy(std::istream_iterator<double>(offset_string), std::istream_iterator<double>(), std::back_inserter(offset));

    vector<double> pose_vector;
    std::istringstream pose_string(line.substr(pos_pose + 6, pos_offset - pos_pose - 6));
    copy(std::istream_iterator<double>(pose_string), std::istream_iterator<double>(), std::back_inserter(pose_vector));
    Vector3d pose = Vector3d(pose_vector[0], pose_vector[1], pose_vector[2]);
    
    vector<double> offset_vector;
    std::istringstream offset_string(line.substr(pos_offset + 8, pos_range_min - pos_offset - 8));
    copy(std::istream_iterator<double>(offset_string), std::istream_iterator<double>(), std::back_inserter(offset_vector));
    Vector3d offset = Vector3d(offset_vector[0], offset_vector[1], offset_vector[2]);
    

    float range_min = stof(line.substr(pos_range_min + 11, pos_range_max - pos_range_min - 11));
    float range_max = stof(line.substr(pos_range_max + 11, pos_angle_min - pos_range_max - 11));
    float angle_min = stof(line.substr(pos_angle_min + 11, pos_angle_max - pos_angle_min - 11));
    float angle_max = stof(line.substr(pos_angle_max + 11, pos_n_beams - pos_angle_max - 11));
    size_t n_beams = stoi(line.substr(pos_n_beams + 9, pos_values - pos_n_beams - 9));
    
    // vector<double> values;
    // std::istringstream values_string(line.substr(pos_values + 8));
    // copy(std::istream_iterator<double>(values_string), std::istream_iterator<double>(), std::back_inserter(values));
    
    vector<double> values_vector;
    std::istringstream values_string(line.substr(pos_values + 8));
    copy(std::istream_iterator<double>(values_string), std::istream_iterator<double>(), std::back_inserter(values_vector));
    
    Eigen::VectorXd values(n_beams);
    for (size_t value_index = 0; value_index < n_beams; value_index++) values[value_index] = values_vector[value_index];
    

    DatasetRecord record(ts, pose, offset, range_min, range_max, angle_min, angle_max, n_beams, values);
    return record;
}

void Dataset::load_data(Vector3dVector& poses, Vector3dVector& sensor_poses, Vector2dVector& points, vector<vector<MapPoint>>& map, size_t from_records, size_t num_records_to_load, double min_pose_distance_threshold, double min_point_distance_threshold){

    
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
   
    
    int total_points = 0;
    int total_poses = 0;
    
    Eigen::Vector3d ref_pose = records()[from_records].pose();
    double ref_distance = DBL_MAX;
    
    for (size_t pose_index = 0; pose_index < num_records_to_load; pose_index++)
    {
        // read record
        DatasetRecord record = records()[from_records + pose_index];
        
        // read and save pose
        Eigen::Vector3d pose = record.pose();
        
        // drop poses that are too near the last added one!
        Eigen::Vector2d diff = Eigen::Vector2d((ref_pose - pose).block(0, 0, 2, 1));
        ref_distance += diff.norm();
        if (ref_distance < min_pose_distance_threshold) continue;

        ref_pose = pose;
        ref_distance = 0;

        // cout << "Loading pose # [ " <<  pose_index << "] " << pose.transpose() << endl;
        
        Eigen::Vector3d sensor_pose = record.offset();
        
        poses.push_back(pose);
        sensor_poses.push_back(sensor_pose);

        // read and save scanned points
        float angle_min = record.angle_min();
        float angle_max = record.angle_max();
        float angle_total = abs(angle_max - angle_min);
        float angle_offset = angle_total / record.n_beams();
        size_t total_beams = record.n_beams();

        Eigen::VectorXd values = record.values();

        vector<MapPoint> map_pose;
        
        int pose_total_points = 0;
        Vector2d last_point;
        float angle = angle_min - angle_offset;
        for (size_t beam_index = 0; beam_index < total_beams; beam_index++)
        {   
            
            angle = angle + angle_offset;
            
            float value = values[beam_index];
            // float value = record.values()[beam_index];

            Vector2d point;
            point << value * cos(angle), value * sin(angle);

            if (pose_total_points && (point - last_point).norm() < min_point_distance_threshold) continue;
            
            if (value > record.range_min() && value < record.range_max()) {
                points.push_back(point);
                
                MapPoint map_point;
                map_point.set_pose_index(total_poses);
                map_point.set_point_index(total_points);
                map_pose.push_back(map_point);

                last_point = point;

                total_points++;
                pose_total_points++;

                
            }

        }
        
        
        total_poses++;
        map.push_back(map_pose);
        
        
    }
        
}

