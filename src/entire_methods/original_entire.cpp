#pragma once
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "../models/envParams.cpp"
#include "../models/hyperParams.cpp"
#include "../models/lidarParams.cpp"

using namespace std;

struct UnionFind
{
    int d[1000 * 1000];
    UnionFind(int n = 0)
    {
        for (int i = 0; i < n; i++)
            d[i] = -1;
    }
    int root(int x)
    {
        if (d[x] < 0)
            return x;
        return d[x] = root(d[x]);
    }
    bool unite(int x, int y)
    {
        x = root(x);
        y = root(y);
        if (x == y)
            return false;
        if (d[x] > d[y])
            swap(x, y);
        d[x] += d[y];
        d[y] = x;
        return true;
    }
    bool same(int x, int y) { return root(x) == root(y); }
    int size(int x) { return -d[root(x)]; }
};

class Graph
{
    vector<tuple<double, int, int>> edges;
    int length;

    double get_diff(cv::Vec3b &a, cv::Vec3b &b)
    {
        double diff = 0;
        for (int i = 0; i < 3; i++)
        {
            diff += (a[i] - b[i]) * (a[i] - b[i]);
        }
        diff = sqrt(diff);
        return diff;
    }

    double get_threshold(double k, int size)
    {
        return 1.0 * k / size;
    }

public:
    Graph(cv::Mat *img)
    {
        length = img->rows * img->cols;
        int dx[] = {1, 0, 0, -1};
        int dy[] = {0, 1, -1, 0};
        for (int i = 0; i < img->rows; i++)
        {
            cv::Vec3b *row = img->ptr<cv::Vec3b>(i);
            for (int j = 0; j < img->cols; j++)
            {
                for (int k = 0; k < 2; k++)
                {
                    int to_x = j + dx[k];
                    int to_y = i + dy[k];
                    if (0 <= to_x && to_x < img->cols && 0 <= to_y && to_y < img->rows)
                    {
                        double diff = get_diff(row[j], img->at<cv::Vec3b>(to_y, to_x));
                        edges.emplace_back(diff, i * img->cols + j, to_y * img->cols + to_x);
                    }
                }
            }
        }
    }

    shared_ptr<UnionFind> segmentate(double k)
    {
        auto unionFind = make_shared<UnionFind>(length);
        int edge_len = edges.size();
        double *thresholds = new double[length];
        for (int i = 0; i < length; i++)
        {
            thresholds[i] = get_threshold(k, 1);
        }

        double diff_max = 0;
        double diff_min = 1000000;
        for (int i = 0; i < edge_len; i++)
        {
            double diff = get<0>(edges[i]);
            diff_max = max(diff_max, diff);
            diff_min = min(diff_min, diff);
        }
        int bucket_len = length;
        vector<int> *bucket = new vector<int>[bucket_len + 1];
        for (int i = 0; i < edge_len; i++)
        {
            int diff_level = (int)(bucket_len * (get<0>(edges[i]) - diff_min) / (diff_max - diff_min));
            bucket[diff_level].emplace_back(i);
        }

        for (int i = 0; i <= bucket_len; i++)
        {
            for (int j = 0; j < bucket[i].size(); j++)
            {
                double diff = get<0>(edges[bucket[i][j]]);
                int from = get<1>(edges[bucket[i][j]]);
                int to = get<2>(edges[bucket[i][j]]);

                from = unionFind->root(from);
                to = unionFind->root(to);

                if (from == to)
                {
                    continue;
                }

                if (diff <= min(thresholds[from], thresholds[to]))
                {
                    unionFind->unite(from, to);
                    int root = unionFind->root(from);
                    thresholds[root] = diff + get_threshold(k, unionFind->size(root));
                }
            }
        }
        delete[] thresholds;
        delete[] bucket;

        /*
        sort(edges.begin(), edges.end());
        cout << "Sort:" << chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - startTime).count() << "ms" << endl;
        for (int i = 0; i < edge_len; i++)
        {
            double diff = get<0>(edges[i]);
            int from = get<1>(edges[i]);
            int to = get<2>(edges[i]);

            from = unionFind->root(from);
            to = unionFind->root(to);

            if (from == to)
            {
                continue;
            }

            if (diff <= min(thresholds[from], thresholds[to]))
            {
                unionFind->unite(from, to);
                int root = unionFind->root(from);
                thresholds[root] = diff + get_threshold(k, unionFind->size(root));
            }
        }
        */

        return unionFind;
    }
};

void original_entire(vector<vector<double>> &grid, EnvParams &envParams, HyperParams &hyperParams, LidarParams &lidarParams, cv::Mat &img, int horizon_offset, vector<vector<Eigen::Vector3d>> &color_grid)
{
    vector<vector<double>> noise_removed;
    //remove_noise_2d(grid, noise_removed, lidarParams, 0.01, 1);
    //grid = noise_removed;

    //remove_noise(grid, noise_removed, lidarParams);

    vector<vector<vector<int>>> image_positions(lidarParams.height, vector<vector<int>>(lidarParams.width, vector<int>(2, 0)));
    {
        // Calibration
        double rollVal = (envParams.roll - 500) / 1000.0;
        double pitchVal = (envParams.pitch - 500) / 1000.0;
        double yawVal = (envParams.yaw - 500) / 1000.0;
        double dx = (envParams.X - 500) / 100.0;
        double dy = (envParams.Y - 500) / 100.0;
        double dz = (envParams.Z - 500) / 100.0;

        Eigen::MatrixXd calibration_mtx(3, 4);
        calibration_mtx << cos(yawVal) * cos(pitchVal), cos(yawVal) * sin(pitchVal) * sin(rollVal) - sin(yawVal) * cos(rollVal), cos(yawVal) * sin(pitchVal) * cos(rollVal) + sin(yawVal) * sin(rollVal), dx,
            sin(yawVal) * cos(pitchVal), sin(yawVal) * sin(pitchVal) * sin(rollVal) + cos(yawVal) * cos(rollVal), sin(yawVal) * sin(pitchVal) * cos(rollVal) - cos(yawVal) * sin(rollVal), dy,
            -sin(pitchVal), cos(pitchVal) * sin(rollVal), cos(pitchVal) * cos(rollVal), dz;
        for (int i = 0; i < lidarParams.height; i++)
        {
            for (int j = 0; j < lidarParams.width; j++)
            {
                int j2 = (j - horizon_offset + lidarParams.width) % lidarParams.width;
                double horizon_angle = lidarParams.horizon_res * j2;
                if (horizon_angle <= 90 || horizon_angle >= 270)
                {
                    continue;
                }

                double vertical_angle = lidarParams.vertical_res * i - lidarParams.bottom_angle;
                double r = 10; // rを仮定しても計算に必要な角度情報は変わらない
                double rawX = r * sin(horizon_angle * M_PI / 180);
                double rawY = -r * tan(vertical_angle * M_PI / 180);
                double rawZ = -r * cos(horizon_angle * M_PI / 180);

                double x = calibration_mtx(0, 0) * rawX + calibration_mtx(0, 1) * rawY + calibration_mtx(0, 2) * rawZ + calibration_mtx(0, 3);
                double y = calibration_mtx(1, 0) * rawX + calibration_mtx(1, 1) * rawY + calibration_mtx(1, 2) * rawZ + calibration_mtx(1, 3);
                double z = calibration_mtx(2, 0) * rawX + calibration_mtx(2, 1) * rawY + calibration_mtx(2, 2) * rawZ + calibration_mtx(2, 3);

                int u = (int)(envParams.width / 2 + envParams.f_xy * x / z);
                int v = (int)(envParams.height / 2 + envParams.f_xy * y / z);
                if (0 <= u && u < envParams.width && 0 <= v && v < envParams.height)
                {
                    image_positions[i][j][0] = v;
                    image_positions[i][j][1] = u;
                    cv::Vec3b c = img.at<cv::Vec3b>(v, u);
                    color_grid[i][j] = Eigen::Vector3d(c[0] / 255.0, c[1] / 255.0, c[2] / 255.0);
                }
            }
        }
    }

    shared_ptr<UnionFind> color_segments;
    {
        // Segmentation
        Graph graph(&img);
        color_segments = graph.segmentate(hyperParams.original_color_segment_k);
    }

    vector<vector<double>> interpolated_grid(lidarParams.height, vector<double>(lidarParams.width, 0));
    {
        // Enhancement
        for (int i = 0; i < lidarParams.height; i++)
        {
            for (int j = 0; j < lidarParams.width; j++)
            {
                if (grid[i][j] > 0)
                {
                    // すでに存在する点はそのまま入れる
                    interpolated_grid[i][j] = grid[i][j];
                    continue;
                }

                int img_u = image_positions[i][j][1];
                int img_v = image_positions[i][j][0];
                if (img_u == 0 && img_v == 0)
                {
                    continue;
                }
                double coef = 0;
                double val = 0;

                cv::Vec3b d0 = img.at<cv::Vec3b>(img_v, img_u);
                int r0 = color_segments->root(img_v * envParams.width + img_u);

                for (int ii = 0; ii < hyperParams.original_r; ii++)
                {
                    for (int jj = 0; jj < hyperParams.original_r; jj++)
                    {
                        int dy = ii - hyperParams.original_r / 2;
                        int dx = jj - hyperParams.original_r / 2;
                        if (i + dy < 0 || i + dy >= lidarParams.height)
                        {
                            continue;
                        }
                        if (grid[i + dy][(j + dx + lidarParams.width) % lidarParams.width] == 0)
                        {
                            continue;
                        }

                        int img_u_tmp = image_positions[i + dy][(j + dx + lidarParams.width) % lidarParams.width][1];
                        int img_v_tmp = image_positions[i + dy][(j + dx + lidarParams.width) % lidarParams.width][0];
                        if (img_u_tmp == 0 && img_v_tmp == 0)
                        {
                            continue;
                        }

                        cv::Vec3b d1 = img.at<cv::Vec3b>(img_v_tmp, img_u_tmp);
                        int r1 = color_segments->root(img_v_tmp * envParams.width + img_u_tmp);
                        double tmp = exp(-(dx * dx + dy * dy) / 2 / hyperParams.original_sigma_s / hyperParams.original_sigma_s); // * exp(-cv::norm(d0 - d1) / 2 / hyperParams.original_sigma_r / hyperParams.original_sigma_r);
                        if (r1 != r0)
                        {
                            tmp *= hyperParams.original_coef_s;
                            //tmp = 0;
                        }
                        if (grid[i + dy][(j + dx + lidarParams.width) % lidarParams.width] == 0)
                        {
                            val += tmp * 1e2;
                        }
                        else
                        {
                            val += tmp * grid[i + dy][(j + dx + lidarParams.width) % lidarParams.width];
                        }
                        coef += tmp;
                    }
                }
                if (coef > 0 /* some threshold */)
                {
                    interpolated_grid[i][j] = val / coef;
                }
            }
        }
    }

    //remove_noise(interpolated_grid, noise_removed, lidarParams, 0.001, 2);
    //interpolated_grid = noise_removed;

    {
        // Apply
        for (int i = 0; i < lidarParams.height; i++)
        {
            for (int j = 0; j < lidarParams.width; j++)
            {
                if (grid[i][j])
                {
                    continue;
                }
                grid[i][j] = interpolated_grid[i][j];
            }
        }
    }
}