// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// Author of Lilo: Edison Velasco 
// Email evs25@alu.ua.es

#ifndef _LIDAR_H_
#define _LIDAR_H_

//define lidar parameter

namespace lidar{

class Lidar
{
    public:
        Lidar();

        void setScanPeriod(double scan_period_in);
        void setLines(double num_lines_in);
        void setVerticalAngle(double velodyne_height_in);
        void setValidationAngle(bool validation_height_in);
        void setVerticalResolution(double vertical_angle_resolution_in);
        void setMaxDistance(double max_distance_in);
        void setMinDistance(double min_distance_in);

    	double max_distance;
        double min_distance;
        int num_lines;
        double scan_period;
        int points_per_line;
        double horizontal_angle_resolution;
        double horizontal_angle;
        double vertical_angle_resolution;
        double velodyne_height;
        bool validation_height;
};


}


#endif // _LIDAR_H_

