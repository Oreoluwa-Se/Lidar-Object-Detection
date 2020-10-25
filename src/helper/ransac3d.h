#ifndef _RANSAC_3D_H_
#define _RANSAC_3D_H_

#include <unordered_set>

template <typename PointT>
class Ransac3D{

	public:
		// constructor
		Ransac3D();

		// desctructor
		~Ransac3D();
 	
		static std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, const int maxIterations, const float distanceTol){
			
			// storage for best inliers
			std::unordered_set<int> inliersResult;
			srand(time(NULL));

			// size of the vector
			int cloud_size = cloud->points.size();

			// array to hold x,y,z coordinates for 3 points
			float line_points[9];
			
			// For max iterations
			for (int index = 0; index < maxIterations; index++){
				std::unordered_set<int> inliers;

				int i_ = 0;
				// Randomly extract two random numbers between 0 and cloud size -1
				while(inliers.size() < 3){
					int genNum = rand()%cloud_size;
					inliers.insert(genNum);
					line_points[i_] = cloud->points[genNum].x;
					line_points[i_ + 1] = cloud->points[genNum].y;
					line_points[i_ + 2] = cloud->points[genNum].z;
					i_+ 3;
				}

				// calculate constants for distance equation betweem point 1 and 2
				float v1 [3] = {line_points[3] - line_points[0], 
					line_points[4] - line_points[1], line_points[5] - line_points[2]};
				// calculate constants for distance equation betweem point 1 and 3
				float v2 [3] = {line_points[6] - line_points[0], 
					line_points[7] - line_points[1], line_points[8] - line_points[2]};

				// find the normal vector
				float dist_cross[3] = {v1[1]*v2[2] -v1[2]*v2[1], 
					v1[2]*v2[0] -v1[0]*v2[2], v1[0]*v2[1] -v1[1]*v2[0]};		

				// calculate the constants for a, b, c, d .. cross product
				float a, b, c, d;
				a = dist_cross[0];
				b = dist_cross[1];
				c = dist_cross[2];
				d = -1*(a*line_points[0] + b*line_points[1] + c*line_points[2]);

				// loop through the remaining points 
				for (int rem_indx = 0; rem_indx < cloud->points.size(); rem_indx++){
					// only check if current position is not in the inliers
					// set
					if(inliers.count(rem_indx) == 0){
						// create a point
						PointT point = cloud->points[rem_indx];

						// Measure distance between every point and fitted line
						float d_ = fabs(a*point.x + b*point.y + c*point.z + d)/sqrt(a*a + b*b + c*c);

						// if the distance is less than the given distacne
						if (d_ <= distanceTol){
							inliers.insert(rem_indx);
						}
					}
				}

				// if the size of the inlier is greater than result.. swap
				if (inliers.size() > inliersResult.size()){
					inliersResult = inliers;
				}

			} 

			// Return indicies of inliers from fitted line with most inliers
			
			return inliersResult;

	}
};
#endif
// End of ransac 3d