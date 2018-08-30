#include <QCoreApplication>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#define _USE_MATH_DEFINES

using namespace std;
using namespace pcl;
using namespace pcl::io;

int offset_depth_image(vector<vector<unsigned short>> &depth_image,
                       unsigned int vertical_resolution,
                       unsigned int horizontal_resolution,
                       unsigned int number_of_vertical_curves,
                       unsigned int number_of_horizontal_curves,
                       bool continuous_bool,
                       bool curved_bool,
                       int offset,
                       float focal_length)
{
    double vertical_curve_step = ((1.0 * M_PI) * number_of_vertical_curves) / vertical_resolution;
    double horizontal_curve_step = ((1.0 * M_PI) * number_of_horizontal_curves) / horizontal_resolution;

    for(unsigned long i = 0; i < depth_image.size(); ++i)
    {
        for(unsigned int j = 0; j < vertical_resolution; ++j)
        {
            for(unsigned int k = 0; k < horizontal_resolution; ++k)
            {
                if(continuous_bool)
                {
                    depth_image[i][(horizontal_resolution * j) + k] *= (i + 1);
                }
                else
                {
                    if(curved_bool)
                    {
                        depth_image[i][(horizontal_resolution * j) + k] = static_cast<unsigned short>(floor(depth_image[i][(horizontal_resolution * j) + k] *
                                fabs((sin(vertical_curve_step * j) + sin(horizontal_curve_step * k)) / 2.0)));
                    }
                }
            }
        }
    }

    return 1;
}

int write_depth_to_file(string &output_path, vector<vector<unsigned short>> &depth_image)
{
    for(unsigned long i = 0; i < depth_image.size(); ++i)
    {
        string depth_test_output_path = output_path + "/depth_test_" + to_string(i) + ".bin";

        ofstream depth_stream(depth_test_output_path, ios::out | ios::binary);

        for(unsigned long j = 0; j < depth_image[i].size(); ++j)
        {
            depth_stream.write(reinterpret_cast<char *>(&depth_image[i][j]), sizeof(unsigned short));
        }

        depth_stream.flush();
        depth_stream.close();
    }

    return 1;
}

int write_header_to_file(string &output_path, vector<vector<unsigned short>> &depth_image, unsigned int horizontal_resolution, unsigned int vertical_resolution)
{
    for(unsigned long i = 0; i < depth_image.size(); ++i)
    {
        ofstream header_stream(output_path + "/header_test_" + to_string(i) + ".kpclp", ios::out);

        header_stream << "kpclp_header_version=0.1" << endl;
        header_stream << "data_type=u" << endl;
        header_stream << "data_size=16" << endl;
        header_stream << "data_dimensions=1" << endl;
        header_stream << "data_resolution=" << to_string(horizontal_resolution) << "," << to_string(vertical_resolution) << endl;
        header_stream << "data_path=" + output_path + "/depth_test_" + to_string(i) + ".bin" << endl;
        header_stream << "epoch_timestamp=" << to_string(i) << endl;
        header_stream << "kinect_timestamp=" << to_string(i) << endl;
        header_stream << "kpclp_header_status=end" << endl;

        header_stream.flush();
        header_stream.close();
    }

    return 1;
}

int depth_image_to_point_cloud(vector<PointCloud<PointXYZ>> &point_cloud,
                               vector<vector<unsigned short>> &depth_image,
                               unsigned int horizontal_resolution,
                               unsigned int vertical_resolution,
                               int offset,
                               float focal_length)
{
    for(unsigned long i = 0; i < point_cloud.size(); ++i)
    {
        point_cloud[i].width = horizontal_resolution * vertical_resolution;
        point_cloud[i].height = 1;
        point_cloud[i].is_dense = false;
        point_cloud[i].points.resize(horizontal_resolution * vertical_resolution);

        for(unsigned int j = 0; j < vertical_resolution; ++j)
        {
            for(unsigned int k = 0; k < horizontal_resolution; ++k)
            {
                float x = (k - (vertical_resolution / 2.0f)) * (depth_image[i][(horizontal_resolution * j) + k] - offset) * focal_length;

                float y = (j - (horizontal_resolution / 2.0f)) * (depth_image[i][(horizontal_resolution * j) + k] - offset) * focal_length;

                float z = depth_image[i][(horizontal_resolution * j) + k];

                point_cloud[i].points[(horizontal_resolution * j) + k].x = x;
                point_cloud[i].points[(horizontal_resolution * j) + k].y = y;
                point_cloud[i].points[(horizontal_resolution * j) + k].z = z;
            }
        }
    }

    return 1;
}

int offset_point_cloud(vector<PointCloud<PointXYZ>> &point_cloud,
                       float rotation_x,
                       float rotation_y,
                       float rotation_z,
                       float translation_x,
                       float translation_y,
                       float translation_z)
{
    for(unsigned long i = 0; i < point_cloud.size(); ++i)
    {
        Eigen::AngleAxisf rotation_x_matrix((rotation_x / point_cloud.size()) * i, Eigen::Vector3f::UnitX());

        Eigen::AngleAxisf rotation_y_matrix((rotation_y / point_cloud.size()) * i, Eigen::Vector3f::UnitY());

        Eigen::AngleAxisf rotation_z_matrix((rotation_z / point_cloud.size()) * i, Eigen::Vector3f::UnitZ());

        Eigen::Translation3f translation_matrix((translation_x / point_cloud.size()) * i,
                                                (translation_y / point_cloud.size()) * i,
                                                (translation_z / point_cloud.size()) * i);

        Eigen::Matrix4f transformation = (rotation_x_matrix * rotation_y_matrix * rotation_z_matrix * translation_matrix).matrix();

        transformPointCloud(point_cloud[i], point_cloud[i], transformation);
    }

    return 1;
}

int write_point_cloud_to_file(string &output_path, vector<PointCloud<PointXYZ>> &point_cloud)
{
    for(unsigned long i = 0; i < point_cloud.size(); ++i)
    {
        savePCDFileASCII(output_path + "/point_cloud_test_" + to_string(i) + ".pcd", point_cloud[i]);
    }

    return 1;
}

int kinect_point_cloud_test_main()
{
    unsigned char configure = 0;
    string output_path = "/home/nikos/Documents/KinectPointCloudTest-output";
    unsigned long iterations = 30;
    unsigned int horizontal_resolution = 640;
    unsigned int vertical_resolution = 480;
    unsigned short iteration_offset = 1000;
    unsigned int number_of_vertical_curves = 2;
    unsigned int number_of_horizontal_curves = 3;
    unsigned char continuous = 0;
    bool continuous_bool = false;
    unsigned char curved = 1;
    bool curved_bool = true;
    int offset = -10;
    float focal_length = 0.0021f;
    float rotation_x = 0.25f * static_cast<float>(M_PI);
    float rotation_y = 0.5f * static_cast<float>(M_PI);
    float rotation_z = 0.1f * static_cast<float>(M_PI);
    float translation_x = -100.0f;
    float translation_y = 50.5f;
    float translation_z = 100.0f;


    cout << "Configure inputs? (0/1): ";
    cin >> configure;

    if(configure == 1)
    {
        cout << "Please enter outut_path: ";
        cin >> output_path;

        cout << "Please enter number of iterations: ";
        cin >> iterations;

        cout << "Please enter offset per iteration: ";
        cin >> iteration_offset;

        cout << "Please enter horizontal resolution: ";
        cin >> horizontal_resolution;

        cout << "Please enter vertical resolution: ";
        cin >> vertical_resolution;

        cout << "Please enter number of vertical curves: ";
        cin >> number_of_vertical_curves;

        cout << "Please enter number of horizontal curves: ";
        cin >> number_of_horizontal_curves;

        cout << "Continuous iteration offset? (0/1): ";
        cin >> continuous;

        cout << "Curved iteration offset? (0/1): ";
        cin >> curved;

        cout << "Please enter offset: ";
        cin >> offset;

        cout << "Please enter focal length: ";
        cin >> focal_length;

        cout << "Please the x component of the rotation: ";
        cin >> rotation_x;

        cout << "Please the y component of the rotation: ";
        cin >> rotation_y;

        cout << "Please the z component of the rotation: ";
        cin >> rotation_z;

        cout << "Please enter the x component of the translation: ";
        cin >> translation_x;

        cout << "Please enter the y component of the translation: ";
        cin >> translation_y;

        cout << "Please enter the z component of the translation: ";
        cin >> translation_z;
    }

    if(continuous == 1)
    {
        continuous_bool = true;
    }
    else
    {
        continuous_bool = false;
    }

    if(curved == 1)
    {
        curved_bool = true;
    }
    else
    {
        curved_bool = false;
    }

    vector<vector<unsigned short>> depth_image(iterations, vector<unsigned short>(static_cast<unsigned long>(horizontal_resolution * vertical_resolution), iteration_offset));

    vector<PointCloud<PointXYZ>> point_cloud(iterations, PointCloud<PointXYZ>());

    offset_depth_image(depth_image, vertical_resolution, horizontal_resolution, number_of_vertical_curves, number_of_horizontal_curves, continuous_bool, curved_bool, offset, focal_length);

    write_depth_to_file(output_path, depth_image);

    write_header_to_file(output_path, depth_image, horizontal_resolution, vertical_resolution);

    depth_image_to_point_cloud(point_cloud, depth_image, horizontal_resolution, vertical_resolution, offset, focal_length);

    offset_point_cloud(point_cloud, rotation_x, rotation_y, rotation_z, translation_x, translation_y, translation_z);

    write_point_cloud_to_file(output_path, point_cloud);

    cout << "Done" << endl;

    return 1;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    kinect_point_cloud_test_main();

    return a.exec();
}
