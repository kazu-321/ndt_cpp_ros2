// Copyright 2024 Taiga Takano
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstddef>
#include <iostream>
#include <fstream>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <vector>
#include <sstream>
#include <cmath>
#include <chrono>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "ndt_cpp_ros2/flatkdtree.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

struct point2{
    float x, y;
};

template <std::size_t I>
struct kdtree::trait::access<point2, I> {
    static auto get(const point2 &p) -> float
    {
        return I == 0 ? p.x : p.y;
    }
};

template <>
struct kdtree::trait::dimension<point2> {
    static constexpr std::size_t value = 2;
};

struct point3{
    float x, y, z;
};

struct mat2x2{
    float a, b;
    float c, d;
};

struct mat3x3{
    float a, b, c;
    float d, e, f;
    float g, h, i;
};

struct ndtpoint2 {
    point2 mean;
    mat2x2 cov;
};

template <std::size_t I>
struct kdtree::trait::access<ndtpoint2, I> {
    static auto get(const ndtpoint2 &p) -> float
    {
        return I == 0 ? p.mean.x : p.mean.y;
    }
};

template <>
struct kdtree::trait::dimension<ndtpoint2> {
    static constexpr std::size_t value = 2;
};

mat3x3 makeTransformationMatrix(const float& tx, const float& ty, const float& theta) {
    mat3x3 mat = {
        cosf(theta), sinf(theta) * -1.0f, tx,
        sinf(theta), cosf(theta)        , ty,
        0.0f, 0.0f, 1.0f
    };
    return mat;
}

void transformPointsZeroCopy(const mat3x3& mat, std::vector<point2>& points) {
    point2 transformedPoint;

    for (auto& point : points) {
        transformedPoint.x = mat.a * point.x + mat.b * point.y + mat.c;
        transformedPoint.y = mat.d * point.x + mat.e * point.y + mat.f;
        point.x = transformedPoint.x;
        point.y = transformedPoint.y;
    }
}

mat3x3 multiplyMatrices3x3x2(const mat3x3& mat1, const mat3x3& mat2) {
    mat3x3 result;
    result.a = mat1.a * mat2.a + mat1.b * mat2.d + mat1.c * mat2.g;
    result.b = mat1.a * mat2.b + mat1.b * mat2.e + mat1.c * mat2.h;
    result.c = mat1.a * mat2.c + mat1.b * mat2.f + mat1.c * mat2.i;

    result.d = mat1.d * mat2.a + mat1.e * mat2.d + mat1.f * mat2.g;
    result.e = mat1.d * mat2.b + mat1.e * mat2.e + mat1.f * mat2.h;
    result.f = mat1.d * mat2.c + mat1.e * mat2.f + mat1.f * mat2.i;

    result.g = mat1.g * mat2.a + mat1.h * mat2.d + mat1.i * mat2.g;
    result.h = mat1.g * mat2.b + mat1.h * mat2.e + mat1.i * mat2.h;
    result.i = mat1.g * mat2.c + mat1.h * mat2.f + mat1.i * mat2.i;

    return result;
}

point3 multiplyMatrixPoint3(const mat3x3& mat, const point3& vec) {
    point3 result;
    result.x = mat.a * vec.x + mat.b * vec.y + mat.c * vec.z;
    result.y = mat.d * vec.x + mat.e * vec.y + mat.f * vec.z;
    result.z = mat.g * vec.x + mat.h * vec.y + mat.i * vec.z;
    return result;
}

float multtiplyPowPoint3(const point3& vec){
    return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
}

void addMat3x3(mat3x3& mat1, const mat3x3& mat2){
    mat1.a += mat2.a;
    mat1.b += mat2.b;
    mat1.c += mat2.c;

    mat1.d += mat2.d;
    mat1.e += mat2.e;
    mat1.f += mat2.f;

    mat1.g += mat2.g;
    mat1.h += mat2.h;
    mat1.i += mat2.i;
}

void addPoint3(point3& point1, const point3& point2){
    point1.x += point2.x;
    point1.y += point2.y;
    point1.z += point2.z;
}

point3 solve3x3(const mat3x3& m, const point3& p) {
    float A[3][4] = {
        {m.a, m.b, m.c, p.x},
        {m.d, m.e, m.f, p.y},
        {m.g, m.h, m.i, p.z}
    };

    int n = 3;

    for (int i = 0; i < n; i++) {
        // Pivot選択
        float maxEl = std::abs(A[i][i]);
        int maxRow = i;
        for (int k = i+1; k < n; k++) {
            if (std::abs(A[k][i]) > maxEl) {
                maxEl = std::abs(A[k][i]);
                maxRow = k;
            }
        }

        // Pivotのある行を交換
        for (int k = i; k < n+1;k++) {
            float tmp = A[maxRow][k];
            A[maxRow][k] = A[i][k];
            A[i][k] = tmp;
        }

        // すべての行について消去を行う
        for (int k = i+1; k < n; k++) {
            float c = -A[k][i] / A[i][i];
            for (int j = i; j < n+1; j++) {
                if (i == j) {
                    A[k][j] = 0.0f;
                } else {
                    A[k][j] += c * A[i][j];
                }
            }
        }
    }

    // 解の計算 (後退代入)
    point3 solution;
    solution.z = A[2][3] / A[2][2];
    solution.y = (A[1][3] - A[1][2] * solution.z) / A[1][1];
    solution.x = (A[0][3] - A[0][2] * solution.z - A[0][1] * solution.y) / A[0][0];

    return solution;
}

mat3x3 expmap(const point3& point){
    auto t = point.z;
    auto c = cosf(t);
    auto s = sinf(t);

    mat2x2 R {
        c, s * -1.0f,
        s, c
    };

    mat3x3 T {
        R.a, R.b, point.x,
        R.c, R.d, point.y,
        0.0f, 0.0f, 1.0f
    };

    return T;
}

point2 transformPointCopy(const mat3x3& mat, const point2& point) {
    point2 transformedPoint;

    transformedPoint.x = mat.a * point.x + mat.b * point.y + mat.c;
    transformedPoint.y = mat.d * point.x + mat.e * point.y + mat.f;

    return transformedPoint;
}

mat3x3 inverse3x3Copy(const mat3x3& mat){
    const auto a = 1.0 / (
        mat.a * mat.e * mat.i +
        mat.b * mat.f * mat.g +
        mat.c * mat.d * mat.h -
        mat.c * mat.e * mat.g -
        mat.b * mat.d * mat.i -
        mat.a * mat.f * mat.h
        );

    mat3x3 inv_mat;
    inv_mat.a = mat.e * mat.i - mat.f * mat.h;
    inv_mat.b = mat.b * mat.i - mat.c * mat.h;
    inv_mat.c = mat.b * mat.f - mat.c * mat.e;

    inv_mat.d = mat.d * mat.i - mat.f * mat.g;
    inv_mat.e = mat.a * mat.i - mat.c * mat.g;
    inv_mat.f = mat.a * mat.f - mat.c * mat.d;

    inv_mat.g = mat.d * mat.h - mat.e * mat.g;
    inv_mat.h = mat.a * mat.h - mat.b * mat.g;
    inv_mat.i = mat.a * mat.e - mat.b * mat.d;


    inv_mat.a = inv_mat.a * a;
    inv_mat.b = inv_mat.b * a * -1.0f;
    inv_mat.c = inv_mat.c * a;

    inv_mat.d = inv_mat.d * a * -1.0f;
    inv_mat.e = inv_mat.e * a;
    inv_mat.f = inv_mat.f * a * -1.0f;

    inv_mat.g = inv_mat.g * a;
    inv_mat.h = inv_mat.h * a * -1.0f;
    inv_mat.i = inv_mat.i * a;

    return inv_mat;
}

point2 skewd(const point2& input_point){
    const point2 skewd_point {
        input_point.y,
        input_point.x * -1.0f
    };
    return skewd_point;
}

mat3x3 transpose(const mat3x3& input_mat){
    const mat3x3 transpose_mat{
        input_mat.a, input_mat.d, input_mat.g,
        input_mat.b, input_mat.e, input_mat.h,
        input_mat.c, input_mat.f, input_mat.i
    };
    return transpose_mat;
}

point2 compute_mean(const std::vector<point2>& points){
    point2 mean;
    mean.x = 0.0f;
    mean.y = 0.0f;
    for(const auto& point : points){
        mean.x += point.x;
        mean.y += point.y;
    }
    mean.x = mean.x / (float)points.size();
    mean.y = mean.y / (float)points.size();
    return mean;
}

mat2x2 compute_covariance(const std::vector<point2>& points, const point2& mean){
    auto point_size = points.size();
    auto vxx = 0.0f;
    auto vxy = 0.0f;
    auto vyy = 0.0f;

    for(const auto& point : points){
        const auto dx = point.x - mean.x;
        const auto dy = point.y - mean.y;
        vxx += dx * dx;
        vxy += dx * dy;
        vyy += dy * dy;
    }

    mat2x2 cov;
    cov.a = vxx / point_size;
    cov.b = vxy / point_size;
    cov.c = vxy / point_size;
    cov.d = vyy / point_size;
    return cov;
}

void compute_ndt_points(std::vector<point2>& points, std::vector<ndtpoint2> &results){
    auto N = 20;

    const auto point_size = points.size();

    kdtree::construct(points.begin(), points.end());
    std::vector<point2> result_points(N);
    std::vector<float> result_distances(N);

    std::vector<mat2x2> covs(point_size);
    results.resize(point_size);

    for(std::size_t i = 0; i < point_size; i++) {
        kdtree::search_knn(points.begin(), points.end(), result_points.begin(), result_distances.begin(), N, points[i]);
        const auto mean = compute_mean(result_points);
        const auto cov = compute_covariance(result_points, mean);
        results[i] = {mean, cov};
    }
}

void ndt_scan_matching(mat3x3& trans_mat, const std::vector<point2>& source_points, std::vector<ndtpoint2>& target_points){
    const size_t max_iter_num = 50;
    const float max_distance2 = 3.0f * 3.0f;

    const size_t target_points_size = target_points.size();
    const size_t source_points_size = source_points.size();

    kdtree::construct(target_points.begin(), target_points.end());
    for(size_t iter = 0; iter < max_iter_num; iter++){
        mat3x3 H_Mat {
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f
        };

        point3 b_Point {
            0.0f, 0.0f, 0.0f
        };

        for(auto point_iter = 0; point_iter < source_points_size; point_iter += 10){
            ndtpoint2 query_point = {transformPointCopy(trans_mat, source_points[point_iter]), {}};
            ndtpoint2 target_point;
            float target_distance;
            kdtree::search_knn(target_points.begin(), target_points.end(), &target_point, &target_distance, 1, query_point);

            if(target_distance > max_distance2){continue;}

            const auto identity_plus_cov = mat3x3{
                target_point.cov.a, target_point.cov.b, 0.0f,
                target_point.cov.c, target_point.cov.d, 0.0f,
                0.0f, 0.0f, 1.0f
            };

            const mat3x3 target_cov_inv = inverse3x3Copy(identity_plus_cov); //IM


            const auto error = point3{
                target_point.mean.x - query_point.mean.x,
                target_point.mean.y - query_point.mean.y,
                0.0f
            };

            const point2 v_point = transformPointCopy(trans_mat, skewd(source_points[point_iter]));

            const auto mat_J = mat3x3{
                trans_mat.a * -1.0f, trans_mat.b * -1.0f, v_point.x,
                trans_mat.d * -1.0f, trans_mat.e * -1.0f, v_point.y,
                trans_mat.g * -1.0f, trans_mat.h * -1.0f, trans_mat.i * -1.0f
            };

            const mat3x3 mat_J_T = transpose(mat_J);

            addMat3x3(H_Mat, multiplyMatrices3x3x2(mat_J_T, multiplyMatrices3x3x2(target_cov_inv, mat_J)));

            addPoint3(b_Point, multiplyMatrixPoint3(mat_J_T, multiplyMatrixPoint3(target_cov_inv, error)));

        }
        b_Point.x *= -1.0;
        b_Point.y *= -1.0;
        b_Point.z *= -1.0;
        const point3 delta = solve3x3(H_Mat,b_Point);
        trans_mat = multiplyMatrices3x3x2(trans_mat, expmap(delta));

        if(multtiplyPowPoint3(delta) < 1e-4){
            std::cout << "END NDT. ITER: " << iter << std::endl;
            break;
        }
    }
}

void writePointsToSVG(const std::vector<point2>& point_1, const std::vector<point2>& point_2, const std::string& file_name) {
    std::ofstream file(file_name);
    if (!file.is_open()) {
        std::cerr << "Cannot open file for writing." << std::endl;
        return;
    }

    file << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"500\" height=\"500\">\n";
    file << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";

    for (const auto& point : point_1) {
        file << "<circle cx=\"" << point.x * 10.0 + 200.0 << "\" cy=\"" << point.y * 10.0 + 200.0 << "\" r=\"1\" fill=\"red\" />\n";
    }

    for (const auto& point : point_2) {
        file << "<circle cx=\"" << point.x * 10.0 + 200.0 << "\" cy=\"" << point.y * 10.0 + 200.0 << "\" r=\"1\" fill=\"black\" />\n";
    }

    file << "</svg>\n";
    file.close();
}



class ndt_cpp_ros2 : public rclcpp::Node
{
public:
    ndt_cpp_ros2():Node("ndt_cpp_ros2")
    {
				// パラメータの宣言
				// tfのフレーム名
				this->declare_parameter("map_frame_id", "map");	
				this->declare_parameter("odom_frame_id", "odom");
				this->declare_parameter("base_frame_id", "base_link");
				this->declare_parameter("laser_frame_id", "laser");

				// mapからodomへの初期位置
				this->declare_parameter("initial_pose_x", 0.0);
				this->declare_parameter("initial_pose_y", 0.0);
				this->declare_parameter("initial_pose_yaw", 0.0);

				// LiDARが逆さまならtrue
				this->declare_parameter("reverse_scan", false);

				// 処理周期
				this->declare_parameter("frequency_millisec", 100);
				
				// tfのbroadcaster, listener, buffer
				tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
				tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
				tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(), std::bind(&ndt_cpp_ros2::scan_callback, this, std::placeholders::_1));
        map_sub_ =this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&ndt_cpp_ros2::map_callback, this, std::placeholders::_1));
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("output/pose", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(this->get_parameter("frequency").as_int()), std::bind(&ndt_cpp_ros2::timer_callback, this));

        old_point.x = 0.0;
        old_point.y = 0.0;
        old_point.z = 0.0;
				is_first = true;

				RCLCPP_INFO(this->get_logger(),"[param]reverse_scan: %b",this->get_parameter("reverse_scan").as_bool());
    }

private:
    std::vector<point2> map_points;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
		std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
		std::shared_ptr<tf2_ros::TransformListener> tfListener_;
		std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
    std::vector<point2> transformed_scan_points;
    rclcpp::TimerBase::SharedPtr timer_;
		bool is_first;

    void timer_callback()
    {       
        if(map_points.empty() || transformed_scan_points.empty()){
					RCLCPP_INFO(this->get_logger(),"No map data or scan data received yet");
            return;
        }

				std::string mapFrameRel = this->get_parameter("map_frame_id").as_string();
				std::string odomFrameRel = this->get_parameter("odom_frame_id").as_string();
				std::string baseFrameRel = this->get_parameter("base_frame_id").as_string();
				std::string laserFrameRel = this->get_parameter("laser_frame_id").as_string();

				geometry_msgs::msg::TransformStamped mapToLaser;

				if(is_first){
					geometry_msgs::msg::TransformStamped transformStamped;
					transformStamped.header.stamp = this->now();
					transformStamped.header.frame_id = "map";
					transformStamped.child_frame_id = "odom";
					transformStamped.transform.translation.x = this->get_parameter("initial_pose_x").as_double();
					transformStamped.transform.translation.y = this->get_parameter("initial_pose_y").as_double();
					transformStamped.transform.translation.z = 0.0;
					tf2::Quaternion q;
					q.setRPY(0, 0, this->get_parameter("initial_pose_yaw").as_double());
					transformStamped.transform.rotation = tf2::toMsg(q);

					tfBroadcaster_->sendTransform(transformStamped);
					try {
						mapToLaser = tfBuffer_->lookupTransform(
							mapFrameRel, laserFrameRel,
							tf2::TimePointZero);
					} catch (const tf2::TransformException & ex) {
						RCLCPP_INFO(
							this->get_logger(), "Could not transform %s to %s: %s",
							mapFrameRel.c_str(), laserFrameRel.c_str(), ex.what());
						return;
					}
					is_first = false;
				}
				else{
					try {
						mapToLaser = tfBuffer_->lookupTransform(
							mapFrameRel, laserFrameRel,
							tf2::TimePointZero);
					} catch (const tf2::TransformException & ex) {
						RCLCPP_INFO(
							this->get_logger(), "Could not transform %s to %s: %s",
							mapFrameRel.c_str(), laserFrameRel.c_str(), ex.what());
						return;
					}
				}
        auto trans_mat1 = makeTransformationMatrix(mapToLaser.transform.translation.x, mapToLaser.transform.translation.y, mapToLaser.transform.rotation.z);
        auto ndt_points = std::vector<ndtpoint2>();

        compute_ndt_points(map_points, ndt_points);
        ndt_scan_matching(trans_mat1, transformed_scan_points, ndt_points);

        float x=trans_mat1.c;
        float y=trans_mat1.f;
        float theta=std::atan(trans_mat1.d/trans_mat1.a);
        RCLCPP_INFO(this->get_logger(),"x:%f, y:%f, theta:%f",x,y,theta);

				geometry_msgs::msg::TransformStamped resultTransform;
				resultTransform.transform.translation.x = x;
				resultTransform.transform.translation.y = y;
				tf2::Quaternion q;
				q.setRPY(0.0,0.0,theta);
				resultTransform.transform.rotation = tf2::toMsg(q);

				// odom->base_linkの変換を取得
				geometry_msgs::msg::TransformStamped odomToLaser;
				try {
					odomToLaser = tfBuffer_->lookupTransform(
						odomFrameRel, laserFrameRel,
						tf2::TimePointZero);
				} catch (const tf2::TransformException & ex) {
					RCLCPP_INFO(
						this->get_logger(), "Could not transform %s to %s: %s",
						odomFrameRel.c_str(), laserFrameRel.c_str(), ex.what());
					return;
				}

				tf2::Transform odomToLaserTF2,resultTF2;
				tf2::fromMsg(odomToLaser.transform, odomToLaserTF2);	
				tf2::fromMsg(resultTransform.transform, resultTF2);	

				// LiDARが逆さまの時の対応。yawのみを取得してodom->base_linkの変換の回転を作成
				float yaw = tf2::getYaw(odomToLaserTF2.getRotation());
				tf2::Quaternion odomToLaserTF2Rot;
				odomToLaserTF2Rot.setRPY(0.0, 0.0, yaw);
				odomToLaserTF2.setRotation(odomToLaserTF2Rot);

				resultTF2 = resultTF2 * odomToLaserTF2.inverse();

				geometry_msgs::msg::TransformStamped transformStamped;
				transformStamped.header.stamp = this->now();
				transformStamped.header.frame_id = "map";
				transformStamped.child_frame_id = "odom";
				transformStamped.transform = tf2::toMsg(resultTF2);

				tfBroadcaster_->sendTransform(transformStamped);

				geometry_msgs::msg::PoseStamped pose;
				pose.header.stamp = this->now();
				pose.header.frame_id = "map";
				pose.pose.position.x = x;
				pose.pose.position.y = y;
				pose.pose.position.z = 0.0;
				pose.pose.orientation.x = 0.0;
				pose.pose.orientation.y = 0.0;
				pose.pose.orientation.z = sin(theta/2);
				pose.pose.orientation.w = cos(theta/2);
				pose_pub_->publish(pose);

    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        transformed_scan_points.clear();
        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
            {
                continue;
            }
            point2 point;
						if(this->get_parameter("reverse_scan").as_bool()){
							double angle = msg->angle_max - msg->angle_increment * i;

							if(abs(angle)>1.569){
								continue;
							}

							point.x = msg->ranges[i] * cosf(angle);
							point.y = msg->ranges[i] * sinf(angle);

						}else{
							point.x = msg->ranges[i] * cosf(msg->angle_min + msg->angle_increment * i - M_PI/2);
							point.y = msg->ranges[i] * sinf(msg->angle_min + msg->angle_increment * i - M_PI/2);
						}
            transformed_scan_points.push_back(point);    
        }
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_points.clear();
        for (size_t i = 0; i < msg->data.size(); i++)
        {
            if (msg->data[i] == 100)
            {
                point2 point;
                point.x = (i % msg->info.width) * msg->info.resolution+msg->info.origin.position.x;
                point.y = (i / msg->info.width) * msg->info.resolution+msg->info.origin.position.y;
                map_points.push_back(point);
            }
        }

    }

    double get_yaw(const geometry_msgs::msg::Quaternion &q){return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));}
    point3 old_point;

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ndt_cpp_ros2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
