#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis,float angle)
{
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f N = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();

    N<< 0,-axis[2],axis[1],
        axis[2],0,-axis[0],
        -axis[1],axis[0],0;
    
    R=cos(angle/180.0)*I+(1-cos(angle/180.0))*axis*axis.transpose()+sin(angle/180.0)*N;

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result.block(0,0,3,3)=R;
    return result;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}



Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    model<< cos(rotation_angle/180.0),-sin(rotation_angle/180.0),0,0,
            sin(rotation_angle/180.0),cos(rotation_angle/180.0),0,0,
            0,0,1,0,
            0,0,0,1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f squish = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    //squish挤压,根据相似三角形算出来

    float top=tan(eye_fov/2.0)*abs(zNear);//平截头体近平面上界的y坐标
    float right=aspect_ratio*top;
    float left=-right;
    float bottom=-top;

    squish<< zNear,0,0,0,
             0,zNear,0,0,
             0,0,zNear+zFar,-zNear*zFar,
             0,0,1,0;
    
    trans << 1,0,0,-(right+left)/2,
            0,1,0,-(top+bottom)/2,
            0,0,1,-(zNear+zFar)/2,
            0,0,0,1;   

    scale << 2.0/(right-left),0,0,0,
            0,2.0/(top-bottom),0,0,
            0,0,2.0/(zNear-zFar),0,
            0,0,0,1;  

    projection=scale*trans*squish;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }


    rst::rasterizer r(700, 700);//定义光栅化器

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    //输入旋转轴
    Eigen::Vector3f axis;
    cout<<"please input the vector3 of your want axis(split by ','):";
    string array[3];
    char * s;

    cin>>s;
    const char *sep=",";
    char *p;
    p=strtok(s,sep);
    int i=0;
    while (p)
    {
        array[i++].append(p);
        p=strtok(NULL,sep);
    }
    
    axis[0]=atoi(array[0].c_str());
    axis[1]=atoi(array[1].c_str());
    axis[2]=atoi(array[2].c_str());



    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //get_rotation(axis,angle);

        r.set_model(get_rotation(axis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_rotation(axis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
