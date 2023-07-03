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
			KDTree(const VectorXfVector& points, const int dim) : root_(nullptr) { build(points, dim); }

			/** @brief The destructor.
			*/
			~KDTree() { clear(); }

			/** @brief Re-builds k-d tree.
			*/
			void build(const VectorXfVector& points,const int dim)
			{
				clear();

				points_ = points;
				dim_ = dim;

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
				dim_ = 0;
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

			void nnSearch(const Eigen::VectorXf& query, pair<double, int>& out) const 
			{
				out.first = std::numeric_limits<double>::max();
				out.second = -1;
				
				nnSearchRecursive(query, root_, out);

			}

			int nnSearch(const Eigen::VectorXf& query, double* minDist = nullptr) const
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
			void knnSearch(const Eigen::VectorXf& query, int k, vector<pair<double, int>>& out) const
			{
				KnnQueue queue(k);
				knnSearchRecursive(query, root_, queue, k);

				for (size_t i = 0; i < queue.size(); i++) out.push_back(queue[i]);
				
			}

			// TODO use unbounded priority queue !!!

			/** @brief Searches neighbors within radius.
			*/
			void radiusSearch(const Eigen::VectorXf& query, double radius, vector<pair<double, int>>& out) const
			{
				// std::vector<int> indices;
				// radiusSearchRecursive(query, root_, indices, radius);
				int max_size = 10;
				KnnQueue queue(max_size);
				radiusSearchRecursive(query, root_, queue, radius);
				
				for (size_t i = 0; i < queue.size(); i++) out.push_back(queue[i]);
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
				const int axis = depth % dim_;
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

			static double distance(const Eigen::VectorXf& p, const Eigen::VectorXf& q)
			{
				
				Eigen::Vector2f diff = p - q;
				return diff.norm();
			}

			/** @brief Searches the nearest neighbor recursively.
			*/
			void nnSearchRecursive(const Eigen::VectorXf& query, const Node* node, pair<double, int>& out) const
			{
				if (node == nullptr)
					return;

				const Eigen::Vector2f& train = points_[node->idx];

				const double dist = distance(query, train);
				
				if (dist < out.first)
				{
					out.first = dist;
					out.second = node->idx;
				}

				const int axis = node->axis;
				const int dir = query[axis] < train[axis] ? 0 : 1;
				nnSearchRecursive(query, node->next[dir], out);

				const double diff = fabs(query[axis] - train[axis]);
				if (diff < out.first)
					nnSearchRecursive(query, node->next[!dir], out);
			}

			void nnSearchRecursive(const Eigen::VectorXf& query, const Node* node, int *guess, double *minDist) const
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
			void knnSearchRecursive(const Eigen::VectorXf& query, const Node* node, KnnQueue& queue, int k) const
			{
				if (node == nullptr)
					return;

				const Eigen::VectorXf& train = points_[node->idx];

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

		
			// void radiusSearchRecursive(const Eigen::Vector2f& query, const Node* node, std::vector<int>& indices, double radius) const
			void radiusSearchRecursive(const Eigen::VectorXf& query, const Node* node, KnnQueue& queue, double radius) const
			{
				if (node == nullptr)
					return;

				const Eigen::VectorXf& train = points_[node->idx];

				const double dist = distance(query, train);
				if (dist < radius)
					queue.push(std::make_pair(dist, node->idx));

				const int axis = node->axis;
				const int dir = query[axis] < train[axis] ? 0 : 1;
				radiusSearchRecursive(query, node->next[dir], queue, radius);

				const double diff = fabs(query[axis] - train[axis]);
				if (diff < radius)
					radiusSearchRecursive(query, node->next[!dir], queue, radius);
			}

			Node* root_;                //!< root node
			VectorXfVector points_; 	//!< points
			int dim_;					//!< dimension
			
	};
	

	/// @brief CorrespondenceFinder class
	class CorrespondenceFinder{

		private:
		

			vector<vector<MapPoint>>* _map;
			Vector2fVector* _points;
			Vector3fVector* _poses;
			
			vector<kdt::KDTree> _points_2dtrees;
			vector<kdt::KDTree> _points_4dtrees;

			kdt::KDTree _poses_2dtrees;
			kdt::KDTree _poses_4dtrees;
			

			

		public:

			CorrespondenceFinder(){};

			~CorrespondenceFinder(){};

			void init(Vector3fVector &poses, Vector2fVector &points){

				
				_points = &points;
				_poses = &poses;

				size_t num_poses = poses.size();
				
				_points_2dtrees.resize(num_poses);
				_points_4dtrees.resize(num_poses);

			}

			void load_points_2dtree(int pose_index, IntVector points_indices){
				VectorXfVector tree_points;
				for (size_t i = 0; i < points_indices.size(); i++)
				{
					tree_points.push_back((*_points)[points_indices[i]]);
				}
				_points_2dtrees[pose_index].build(tree_points, 2);

			}

			bool find_point_neighbors(int tree_index, int pose_index, int point_index, int num_neighbors, vector<pair<double, int>>& neighbors){
				Vector2f point = (*_points)[point_index];
				Vector3f pose = (*_poses)[pose_index];

				Vector3f tree_pose = (*_poses)[tree_index];

				Vector2f query_point = v2t(tree_pose).inverse() * v2t(pose) * point; // point in tree coordinates
				
				_points_2dtrees[tree_index].knnSearch(query_point, num_neighbors, neighbors);
				
				if (!neighbors.size()) return false;
			
				return true;
			}
			
			bool find_point_neighbors(int tree_index, int pose_index, int point_index, double radius, vector<pair<double, int>>& neighbors){
				Vector2f point = (*_points)[point_index];
				Vector3f pose = (*_poses)[pose_index];

				Vector3f tree_pose = (*_poses)[tree_index];

				Vector2f query_point = v2t(tree_pose).inverse() * v2t(pose) * point; // point in tree coordinates
				
				_points_2dtrees[tree_index].radiusSearch(query_point, radius, neighbors);
				
				if (!neighbors.size()) return false;
				
				return true;
			}

			bool find_point_neighbor(int tree_index, int pose_index, int point_index, pair<double, int>& neighbor, double min_distance = MAXFLOAT){
				Vector2f point = (*_points)[point_index];
				Vector3f pose = (*_poses)[pose_index];

				Vector3f tree_pose = (*_poses)[tree_index];

				Vector2f query_point = v2t(tree_pose).inverse() * v2t(pose) * point; // point in tree coordinates

				_points_2dtrees[tree_index].nnSearch(query_point, neighbor);

				if (neighbor.first > min_distance || neighbor.second == -1) return false;	
							
				return true;

			}

			void load_poses_2dtree(){
				VectorXfVector tree_poses;
				for (size_t pose_index = 0; pose_index < _poses->size(); pose_index++)
				{
					tree_poses.push_back((*_poses)[pose_index]);
				}
				_poses_2dtrees.build(tree_poses, 2);
				
			}

			bool find_pose_neighbors(int pose_index, int num_neighbors, IntVector& neighbors){
				Vector3f query_pose = (*_poses)[pose_index];
				
				vector<pair<double, int>> pose_neighbors;
				_poses_2dtrees.knnSearch(query_pose, num_neighbors, pose_neighbors);
				
				if (!pose_neighbors.size()) return false;	

				for (size_t i = 0; i < pose_neighbors.size(); i++)
				{
					neighbors.push_back(pose_neighbors[i].second);
				}
							
				return true;

			}
			
			bool find_pose_neighbors(int pose_index, double radius, IntVector& neighbors){
				Vector3f query_pose = (*_poses)[pose_index];
				
				vector<pair<double, int>> pose_neighbors;
				_poses_2dtrees.radiusSearch(query_pose, radius, pose_neighbors);
				
				if (!pose_neighbors.size()) return false;	

				for (size_t i = 0; i < pose_neighbors.size(); i++)
				{
					neighbors.push_back(pose_neighbors[i].second);
				}
							
				return true;

			}

			bool find_pose_neighbor(int pose_index, pair<double, int>& neighbor, double min_distance = MAXFLOAT){
				Vector3f query_pose = (*_poses)[pose_index];
				
				_poses_2dtrees.nnSearch(query_pose, neighbor);

				if (neighbor.first > min_distance || neighbor.second == -1) return false;					
				return true;

			}
	};

} // kdt

#endif // !__KDTREE_H__