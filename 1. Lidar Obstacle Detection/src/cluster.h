
// Quiz on implementing simple RANSAC line fitting

#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <chrono>
#include <string>

inline void clusterHelper(int index, const std::vector<std::vector<float>>& points, std::vector<bool>& processed, std::vector<int>& cluster, KdTree* tree, float distanceTol)
{
 	processed[index] = true;
	cluster.push_back(index);

	std::vector<int> neighbors = tree->search(points[index], distanceTol);

	for(int pointIndex: neighbors)
	{
		if(!processed[pointIndex])
		{
			clusterHelper(pointIndex, points, processed, cluster, tree, distanceTol); 
		}
	}
}

inline std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	for(int i = 0; i < points.size(); i++)
	{
		if(!processed[i])
		{
			std::vector<int> cluster;
			clusterHelper(i, points, processed, cluster, tree, distanceTol);
			clusters.push_back(cluster);
		}
	}
 
	return clusters;
}

#endif