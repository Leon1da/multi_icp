#ifndef __KDTREE_H__
#define __KDTREE_H__

#include "defs.h"
#include <vector>
#include <numeric>
#include <algorithm>
#include <exception>
#include <functional>

namespace kdt
{

	/** @brief Bounded priority queue.
	*/
	template <class T, class Compare = std::less<T>>
	class BoundedPriorityQueue
		{
		public:

			BoundedPriorityQueue() = delete;
			BoundedPriorityQueue(size_t bound) : bound_(bound) { elements_.reserve(bound + 1); };

			void push(const T& val)
			{
				auto it = std::find_if(std::begin(elements_), std::end(elements_),
					[&](const T& element){ return Compare()(val, element); });
				elements_.insert(it, val);

				if (elements_.size() > bound_)
					elements_.resize(bound_);
			}

			const T& back() const { return elements_.back(); };
			const T& operator[](size_t index) const { return elements_[index]; }
			size_t size() const { return elements_.size(); }

		private:
			size_t bound_;
			std::vector<T> elements_;
		};

	/** @brief k-d tree class.
	*/
	class KDTree{
		public:
			/** @brief The constructors.
			*/
			KDTree() : root_(nullptr) {};
			KDTree(const Vector2fVector& points) : root_(nullptr) { build(points); }

			/** @brief The destructor.
			*/
			~KDTree() { clear(); }

			/** @brief Re-builds k-d tree.
			*/
			void build(const Vector2fVector& points)
			{
				clear();

				points_ = points;

				std::vector<int> indices(points.size());
				std::iota(std::begin(indices), std::end(indices), 0);

				root_ = buildRecursive(indices.data(), (int)points.size(), 0);
			}

			/** @brief Clears k-d tree.
			*/
			void clear()
			{ 
				clearRecursive(root_);
				root_ = nullptr;
				points_.clear();
			}

			/** @brief Validates k-d tree.
			*/
			bool validate() const
			{
				try
				{
					validateRecursive(root_, 0);
				}
				catch (const Exception&)
				{
					return false;
				}

				return true;
			}

			/** @brief Searches the nearest neighbor.
			*/
			int nnSearch(const Eigen::Vector2f& query, double* minDist = nullptr) const
			{
				int guess;
				double _minDist = std::numeric_limits<double>::max();

				nnSearchRecursive(query, root_, &guess, &_minDist);

				if (minDist)
					*minDist = _minDist;

				return guess;
			}

			/** @brief Searches k-nearest neighbors.
			*/
			std::vector<int> knnSearch(const Eigen::Vector2f& query, int k) const
			{
				KnnQueue queue(k);
				knnSearchRecursive(query, root_, queue, k);
				
				std::vector<int> indices(queue.size());
				for (size_t i = 0; i < queue.size(); i++)
					indices[i] = queue[i].second;

				return indices;
			}

			/** @brief Searches neighbors within radius.
			*/
			std::vector<int> radiusSearch(const Eigen::Vector2f& query, double radius) const
			{
				std::vector<int> indices;
				radiusSearchRecursive(query, root_, indices, radius);
				return indices;
			}


			/// @brief apply an isometry transformation to all the tree points 
			/// @param transformation: the transformation to be applied
			void transform(const Eigen::Isometry2f transformation) {
				for (size_t point_index = 0; point_index < points_.size(); point_index++)
				{
					Eigen::Vector2f point_ = transformation * points_[point_index];
					points_[point_index] = point_;
				}
				
			}

		private:

			/** @brief k-d tree node.
			*/
			struct Node
			{
				int idx;       //!< index to the original point
				Node* next[2]; //!< pointers to the child nodes
				int axis;      //!< dimension's axis

				Node() : idx(-1), axis(-1) { next[0] = next[1] = nullptr; }
			};

			/** @brief k-d tree exception.
			*/
			class Exception : public std::exception { using std::exception::exception; };

			
			/** @brief Priority queue of <distance, index> pair.
			*/
			using KnnQueue = BoundedPriorityQueue<std::pair<double, int>>;

			/** @brief Builds k-d tree recursively.
			*/
			Node* buildRecursive(int* indices, int npoints, int depth)
			{
				if (npoints <= 0)
					return nullptr;

				// const int axis = depth % PointT::DIM;
				const int axis = depth % 2;
				const int mid = (npoints - 1) / 2;

				std::nth_element(indices, indices + mid, indices + npoints, [&](int lhs, int rhs)
				{
					return points_[lhs][axis] < points_[rhs][axis];
				});

				Node* node = new Node();
				node->idx = indices[mid];
				node->axis = axis;

				node->next[0] = buildRecursive(indices, mid, depth + 1);
				node->next[1] = buildRecursive(indices + mid + 1, npoints - mid - 1, depth + 1);

				return node;
			}

			/** @brief Clears k-d tree recursively.
			*/
			void clearRecursive(Node* node)
			{
				if (node == nullptr)
					return;

				if (node->next[0])
					clearRecursive(node->next[0]);

				if (node->next[1])
					clearRecursive(node->next[1]);

				delete node;
			}

			/** @brief Validates k-d tree recursively.
			*/
			void validateRecursive(const Node* node, int depth) const
			{
				if (node == nullptr)
					return;

				const int axis = node->axis;
				const Node* node0 = node->next[0];
				const Node* node1 = node->next[1];

				if (node0 && node1)
				{
					if (points_[node->idx][axis] < points_[node0->idx][axis])
						throw Exception();

					if (points_[node->idx][axis] > points_[node1->idx][axis])
						throw Exception();
				}

				if (node0)
					validateRecursive(node0, depth + 1);

				if (node1)
					validateRecursive(node1, depth + 1);
			}

			static double distance(const Eigen::Vector2f& p, const Eigen::Vector2f& q)
			{
				
				// double dist = 0;
				// for (size_t i = 0; i < PointT::DIM; i++)
				// 	dist += (p[i] - q[i]) * (p[i] - q[i]);
				// return sqrt(dist);
				Eigen::Vector2f diff = p - q;
				return sqrt(diff.x() * diff.x() + diff.y() * diff.y());
			}

			/** @brief Searches the nearest neighbor recursively.
			*/
			void nnSearchRecursive(const Eigen::Vector2f& query, const Node* node, int *guess, double *minDist) const
			{
				if (node == nullptr)
					return;

				const Eigen::Vector2f& train = points_[node->idx];

				const double dist = distance(query, train);
				if (dist < *minDist)
				{
					*minDist = dist;
					*guess = node->idx;
				}

				const int axis = node->axis;
				const int dir = query[axis] < train[axis] ? 0 : 1;
				nnSearchRecursive(query, node->next[dir], guess, minDist);

				const double diff = fabs(query[axis] - train[axis]);
				if (diff < *minDist)
					nnSearchRecursive(query, node->next[!dir], guess, minDist);
			}

			/** @brief Searches k-nearest neighbors recursively.
			*/
			void knnSearchRecursive(const Eigen::Vector2f& query, const Node* node, KnnQueue& queue, int k) const
			{
				if (node == nullptr)
					return;

				const Eigen::Vector2f& train = points_[node->idx];

				const double dist = distance(query, train);
				queue.push(std::make_pair(dist, node->idx));

				const int axis = node->axis;
				const int dir = query[axis] < train[axis] ? 0 : 1;
				knnSearchRecursive(query, node->next[dir], queue, k);

				const double diff = fabs(query[axis] - train[axis]);
				if ((int)queue.size() < k || diff < queue.back().first)
					knnSearchRecursive(query, node->next[!dir], queue, k);
			}

			/** @brief Searches neighbors within radius.
			*/
			void radiusSearchRecursive(const Eigen::Vector2f& query, const Node* node, std::vector<int>& indices, double radius) const
			{
				if (node == nullptr)
					return;

				const Eigen::Vector2f& train = points_[node->idx];

				const double dist = distance(query, train);
				if (dist < radius)
					indices.push_back(node->idx);

				const int axis = node->axis;
				const int dir = query[axis] < train[axis] ? 0 : 1;
				radiusSearchRecursive(query, node->next[dir], indices, radius);

				const double diff = fabs(query[axis] - train[axis]);
				if (diff < radius)
					radiusSearchRecursive(query, node->next[!dir], indices, radius);
			}

			Node* root_;                 //!< root node
			Vector2fVector points_; //!< points
			
	};
	

	/// @brief CorrespondenceFinder class
	class CorrespondenceFinder{

		private:
		
			IntPairVector* _pose_point_correspondences;
			vector<Vector2fVector>* _points;
			Vector3fVector* _poses;
			
			vector<kdt::KDTree> _kdtrees;
			

		public:

			CorrespondenceFinder(){};

			~CorrespondenceFinder(){};

			void init(Vector3fVector &poses, vector<Vector2fVector> &points, IntPairVector &pose_point_correspondences){

				cout << "building kdtrees.." << endl;
				_pose_point_correspondences = &pose_point_correspondences;
				_points = &points;
				_poses = &poses;
				
				size_t num_poses = poses.size();
				_kdtrees.resize(num_poses);

				for (size_t pose_index = 0; pose_index < num_poses; pose_index++)
				{
					_kdtrees[pose_index].build(points[pose_index]);
					cout << '\r' << "tree " << pose_index + 1 << "/" << num_poses << " built." << flush;
			
				}
				
				cout << endl << "building kdtrees complete." << endl;

			}

			
			/// @brief find the point correspondences in the current pose (local to the tree)
			/// @param pose_point_pair 
			/// @param num_neighbors 
			/// @param neighbors 
			bool find_local_neighbors(IntPair& pose_point_pair, int num_neighbors, IntPairVector& neighbors){
				int pose_index = pose_point_pair.first;
				int point_index = pose_point_pair.second;

				Vector2f query_point((*_points)[pose_index][point_index]);

				vector<int> indices = _kdtrees[pose_index].knnSearch(query_point, num_neighbors); // one occurence will be the point itself
				
				for (auto id : indices) neighbors.push_back(IntPair(pose_index, id));

				return true;
					
			}

			/// @brief find the point correspondences in all the poses apart the current
			/// @param pose_point_pair 
			/// @param neighbor
			bool find_global_neighbor(IntPair& pose_point_pair, IntPair& neighbor){
				
				size_t pose_index = pose_point_pair.first;
				size_t point_index = pose_point_pair.second;

				Eigen::Isometry2f query_point_pose = v2t((*_poses)[pose_index]);
				Eigen::Vector2f query_point = (*_points)[pose_index][point_index];

				size_t window_size = 20;
				size_t mid_win_size = window_size / 2;

				// float min_distance = MAXFLOAT;
				float threshold = 0.1;
				float min_distance = threshold;
				for (size_t wind_index = 0; wind_index < mid_win_size * 2; wind_index++)
				{
					size_t tree_index = pose_index - mid_win_size + wind_index;
					if (tree_index != pose_index && tree_index >= 0 && tree_index < _poses->size())
					{	
						Eigen::Isometry2f tree_pose = v2t((*_poses)[tree_index]);
						Eigen::Vector2f query_point_in_tree_pose = tree_pose.inverse() * query_point_pose * query_point;
						vector<int> found_indices = _kdtrees[tree_index].knnSearch(query_point_in_tree_pose, 1);
						float dist = ((*_points)[tree_index][found_indices[0]] - query_point_in_tree_pose).norm();
						if (min_distance > dist){
							neighbor = IntPair(tree_index, found_indices[0]);
							min_distance = dist;
						}
					}
					
					
				}
				

				if (min_distance != threshold) return true;
				else return false;
				
				
					
			}
			
		};
	

} // kdt

#endif // !__KDTREE_H__