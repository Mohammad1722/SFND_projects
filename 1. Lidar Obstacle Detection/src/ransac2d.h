
// Quiz on implementing simple RANSAC line fitting

#ifndef RANSAC_H_
#define RANSAC_H_

#include <unordered_set>

template<typename PointT> 
inline std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	int num_points = cloud->points.size();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
  
	for(size_t i = 0; i < maxIterations; i++)
	{
		PointT pt1, pt2, pt3;

		std::unordered_set<int> inliersTemp;

		while(inliersTemp.size() < 3)
		{
			inliersTemp.insert(rand() % num_points);
		}

		auto it = inliersTemp.begin();
		pt1 = cloud->points[*it];
		it++;
		pt2 = cloud->points[*it];
		it++;
		pt3 = cloud->points[*it];
      
		double A = ((pt2.y - pt1.y) * (pt3.z - pt1.z)) - ((pt2.z - pt1.z) * (pt3.y - pt1.y));
		double B = ((pt2.z - pt1.z) * (pt3.x - pt1.x)) - ((pt2.x - pt1.x) * (pt3.z - pt1.z));
		double C = ((pt2.x - pt1.x) * (pt3.y - pt1.y)) - ((pt2.y - pt1.y) * (pt3.x - pt1.x));
		double D = -(A * pt1.x + B * pt1.y + C * pt1.z);
		
		for(size_t j = 0; j < num_points; j++)
		{
			if(inliersTemp.count(j) > 0) 
				continue;

			auto other = cloud->points[j];
          
			double distance = fabs(A * other.x + B * other.y + C * other.z + D) / sqrt(A*A + B*B + C*C);

			if(distance <= distanceTol)
			{
				inliersTemp.insert(j);
			}
		}

		if(inliersTemp.size() > inliersResult.size())
		{
			inliersResult = inliersTemp;
		}
	}

	return inliersResult;
}

#endif
