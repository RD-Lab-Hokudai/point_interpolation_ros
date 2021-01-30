#pragma once
#include <vector>
#include <chrono>

#include <opencv2/opencv.hpp>
#include "./UnionFind.cpp"

using namespace std;

struct Edge
{
    int node1;
    int node2;
    double cost;

    Edge(int n1, int n2, double c) : node1(n1), node2(n2), cost(c) {}
};

class SegmentationGraph
{
    vector<Edge> edges;
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
    SegmentationGraph(cv::Mat *img)
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
                        edges.emplace_back(i * img->cols + j, to_y * img->cols + to_x, diff);
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
            double diff = edges[i].cost;
            diff_max = max(diff_max, diff);
            diff_min = min(diff_min, diff);
        }
        int bucket_len = length;
        vector<int> *bucket = new vector<int>[bucket_len + 1];
        for (int i = 0; i < edge_len; i++)
        {
            int diff_level = (int)(bucket_len * (edges[i].cost - diff_min) / (diff_max - diff_min));
            bucket[diff_level].emplace_back(i);
        }

        for (int i = 0; i <= bucket_len; i++)
        {
            for (int j = 0; j < bucket[i].size(); j++)
            {
                double diff = edges[bucket[i][j]].cost;
                int from = edges[bucket[i][j]].node1;
                int to = edges[bucket[i][j]].node2;

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

        //cout << "Segmentation end" << endl;
        //cout << chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - startTime).count() << "ms" << endl;

        return unionFind;
    }
};