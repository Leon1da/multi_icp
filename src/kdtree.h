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
			KDTree(const VectorXdVector& points, const int dim) : root_(nullptr) { build(points, dim); }

			/** @brief The destructor.
			*/
			~KDTree() { clear(); }

			/** @brief Re-builds k-d tree.
			*/
			void build(const VectorXdVector& points, const int dim)
			{
				clear();

				points_ = points;
				dim_ = dim;

				std::vector<int> indices(points.size());
				std::iota(std::begin(indices), std::end(indices), 0);

				root_ = buildRecursive(indices.data(), (int) points.size(), 0);
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

			void nnSearch(const Eigen::VectorXd& query, pair<double, int>& out) const 
			{
				out.first = std::numeric_limits<double>::max();
				out.second = -1;
				
				nnSearchRecursive(query, root_, out);

			}

			int nnSearch(const Eigen::VectorXd& query, double* minDist = nullptr) const
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
			void knnSearch(const Eigen::VectorXd& query, int k, vector<pair<double, int>>& out) const
			{
				KnnQueue queue(k);
				knnSearchRecursive(query, root_, queue, k);

				for (size_t i = 0; i < queue.size(); i++) out.push_back(queue[i]);
				
			}

			// TODO use unbounded priority queue !!!

			/** @brief Searches neighbors within radius.
			*/
			void radiusSearch(const Eigen::VectorXd& query, double radius, vector<pair<double, int>>& out) const
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

			static double distance(const Eigen::VectorXd& p, const Eigen::VectorXd& q, int dim)
			{
				// cout << "distance ( p: " << p << " q: " << q << " dim: " << dim << " )" << endl;
				
				if (dim == 2) {
					Vector2d diff = p - q;
					// cout << " dim 2 diff: " << diff <<endl;
					return diff.norm();

				} else if (dim == 4) {
					// TODO implements different distance functions
					Vector4d diff = p - q;
					// return diff.norm();

					double alpha = 0.2;
					Vector2d pos = diff.block(0, 0, 2, 1);
					Vector2d vec = diff.block(2, 0, 2, 1);

					double norm = sqrt((1-alpha)*(pow(pos.x(), 2) + pow(pos.y(), 2)) + alpha * (pow(vec.x(), 2) + pow(vec.y(), 2)));
					return norm;

					
				} else {
					new runtime_error("Not implemented exception");
					return 0;
				}
			}

			/** @brief Searches the nearest neighbor recursively.
			*/
			void nnSearchRecursive(const Eigen::VectorXd& query, const Node* node, pair<double, int>& out) const
			{
				if (node == nullptr)
					return;

				const Eigen::VectorXd& train = points_[node->idx];

				const double dist = distance(query, train, dim_);
				
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

			void nnSearchRecursive(const Eigen::VectorXd& query, const Node* node, int *guess, double *minDist) const
			{
				if (node == nullptr)
					return;

				const Eigen::VectorXd& train = points_[node->idx];

				const double dist = distance(query, train, dim_);
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
			void knnSearchRecursive(const Eigen::VectorXd& query, const Node* node, KnnQueue& queue, int k) const
			{
				if (node == nullptr)
					return;

				const Eigen::VectorXd& train = points_[node->idx];

				const double dist = distance(query, train, dim_);
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

		
			// void radiusSearchRecursive(const Eigen::Vector2d& query, const Node* node, std::vector<int>& indices, double radius) const
			void radiusSearchRecursive(const Eigen::VectorXd& query, const Node* node, KnnQueue& queue, double radius) const
			{
				if (node == nullptr)
					return;

				const Eigen::VectorXd& train = points_[node->idx];

				const double dist = distance(query, train, dim_);
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
			VectorXdVector points_; 	//!< points
			int dim_;					//!< dimension
			
	};
	

	/// @brief CorrespondenceFinder class
	class CorrespondenceFinder{

		private:
		

			Vector2dVector* _points;
			Vector2dVector* _normals;
			Vector3dVector* _poses;
			
			vector<kdt::KDTree> _points_2dtrees;
			vector<kdt::KDTree> _points_4dtrees;

			kdt::KDTree _poses_2dtrees;
			kdt::KDTree _poses_4dtrees;
			

			

		public:

			CorrespondenceFinder(){};

			~CorrespondenceFinder(){};

			void init(Vector3dVector &poses, Vector2dVector &points){

				
				_points = &points;
				_poses = &poses;

				size_t num_poses = poses.size();
				
				_points_2dtrees.resize(num_poses);
				_points_4dtrees.resize(num_poses);

			}

			void init(Vector3dVector &poses, Vector2dVector &points, Vector2dVector &normals){

				
				_points = &points;
				_normals = &normals;
				_poses = &poses;

				size_t num_poses = poses.size();
				
				_points_2dtrees.resize(num_poses);
				_points_4dtrees.resize(num_poses);

			}

			void set_normals(Vector2dVector* normals){
				_normals = normals;
			}
			
			// points 2dtree
			void load_points_2dtree(int pose_index, IntVector points_indices){
				VectorXdVector tree_points;
				for (size_t i = 0; i < points_indices.size(); i++)
				{
					tree_points.push_back((*_points)[points_indices[i]]);
				}
				_points_2dtrees[pose_index].build(tree_points, 2);

			}

			bool find_point_neighbors(int tree_index, int pose_index, int point_index, int num_neighbors, vector<pair<double, int>>& neighbors){
				Vector2d point = (*_points)[point_index];
				Vector3d pose = (*_poses)[pose_index];

				Vector3d tree_pose = (*_poses)[tree_index];

				Vector2d query_point = v2t(tree_pose).inverse() * v2t(pose) * point; // point in tree coordinates
				
				_points_2dtrees[tree_index].knnSearch(query_point, num_neighbors, neighbors);
				
				if (!neighbors.size()) return false;
			
				return true;
			}

			bool find_point_neighbors(int tree_index, int pose_index, int point_index, double radius, vector<pair<double, int>>& neighbors){
				Vector2d point = (*_points)[point_index];
				Vector3d pose = (*_poses)[pose_index];

				Vector3d tree_pose = (*_poses)[tree_index];

				Vector2d query_point = v2t(tree_pose).inverse() * v2t(pose) * point; // point in tree coordinates
				
				_points_2dtrees[tree_index].radiusSearch(query_point, radius, neighbors);
				
				if (!neighbors.size()) return false;
				
				return true;
			}

			bool find_point_neighbor(int tree_index, int pose_index, int point_index, pair<double, int>& neighbor, double min_distance = MAXFLOAT){
				Vector2d point = (*_points)[point_index];
				Vector3d pose = (*_poses)[pose_index];
				Vector3d tree_pose = (*_poses)[tree_index];

				Vector2d query_point = v2t(tree_pose).inverse() * v2t(pose) * point; // point in tree coordinates

				_points_2dtrees[tree_index].nnSearch(query_point, neighbor);

				if (neighbor.first > min_distance || neighbor.second == -1) return false;	
							
				return true;

			}

			// points 4dtree
			void load_points_4dtree(int pose_index, IntVector points_indices, IntVector normals_indices){
				assert(points_indices.size() == normals_indices.size());
				
				VectorXdVector tree_points;
				for (size_t i = 0; i < points_indices.size(); i++)
				{
					Vector2d point = (*_points)[points_indices[i]];
					Vector2d normal = (*_normals)[normals_indices[i]];
					// Vector2d normal = Vector2d(0, 0);
					Vector4d tree_point = Vector4d(point.x(), point.y(), normal.x(), normal.y());
					tree_points.push_back(tree_point);
				}

				_points_4dtrees[pose_index].build(tree_points, 4);
				
			}

			bool find_point_neighbors(int tree_index, int pose_index, int point_index, int normal_index, int num_neighbors, vector<pair<double, int>>& neighbors){
				Vector3d pose = (*_poses)[pose_index];
				Vector3d tree_pose = (*_poses)[tree_index];

				Vector2d point = (*_points)[point_index];
				
				Vector2d point_in_tree_pose = v2t(tree_pose).inverse() * v2t(pose) * point; // point in tree coordinates
				
				Vector2d normal = (*_normals)[normal_index]; 
				// Vector2d world_normal = v2t(tree_pose).inverse() * v2t(pose) * normal; // normal in tree coordinates
				Vector2d normal_in_tree_pose = v2t(tree_pose).inverse().rotation() * v2t(pose).rotation() * normal; // normal in tree coordinates

				Vector4d query_point = Vector4d(point_in_tree_pose.x(), point_in_tree_pose.y(), normal_in_tree_pose.x(), normal_in_tree_pose.y());
				
				_points_4dtrees[tree_index].knnSearch(query_point, num_neighbors, neighbors);
				
				if (!neighbors.size()) return false;
			
				return true;
			}
				
			bool find_point_neighbors(int tree_index, int pose_index, int point_index, int normal_index, double radius, vector<pair<double, int>>& neighbors){
				
				Vector3d pose = (*_poses)[pose_index];
				Vector3d tree_pose = (*_poses)[tree_index];

				

				Vector2d point = (*_points)[point_index];
				Vector2d point_in_tree_pose = v2t(tree_pose).inverse() * v2t(pose) * point; // point in tree coordinates
				
				Vector2d normal = (*_normals)[normal_index];
				// Vector2d world_normal = v2t(tree_pose).inverse() * v2t(pose) * normal; // normal in tree coordinates
				Vector2d normal_in_tree_pose = v2t(tree_pose).inverse().rotation() * v2t(pose).rotation() * normal; // normal in tree coordinates

				Vector4d query_point = Vector4d(point_in_tree_pose.x(), point_in_tree_pose.y(), normal_in_tree_pose.x(), normal_in_tree_pose.y());
				_points_4dtrees[tree_index].radiusSearch(query_point, radius, neighbors);
				
				if (!neighbors.size()) return false;
				
				return true;
			}

			bool find_point_neighbor(int tree_index, int pose_index, int point_index, int normal_index, pair<double, int>& neighbor, double min_distance = MAXFLOAT){
				Vector2d point = (*_points)[point_index];
				Vector2d normal = (*_normals)[normal_index];
				Vector3d pose = (*_poses)[pose_index];
				Vector3d tree_pose = (*_poses)[tree_index];

				Vector2d tree_point = v2t(tree_pose).inverse() * v2t(pose) * point; // point in the reference tree coordinates
				Vector2d tree_normal = v2t(tree_pose).inverse().rotation() * v2t(pose).rotation() * normal; // normal in the reference tree coordinates
				// tree_normal = tree_point + tree_normal;
				// tree_normal = Vector2d(0, 0);
				
	
				Vector4d query_point = Vector4d(tree_point.x(), tree_point.y(), tree_normal.x(), tree_normal.y());
				_points_4dtrees[tree_index].nnSearch(query_point, neighbor);
				
				
				if (neighbor.first > min_distance || neighbor.second == -1) return false;	
							
				return true;

			}

			// poses 2dtree
			void load_poses_2dtree(){
				VectorXdVector tree_poses;
				for (size_t pose_index = 0; pose_index < _poses->size(); pose_index++)
				{
					tree_poses.push_back((*_poses)[pose_index]);
				}
				_poses_2dtrees.build(tree_poses, 2);
				
			}

			// poses 4dtree
			void load_poses_4dtree(){
				VectorXdVector tree_poses;
				for (size_t pose_index = 0; pose_index < _poses->size(); pose_index++)
				{
					Vector3d pose = (*_poses)[pose_index];
					Vector4d tree_pose = Vector4d(pose.x(), pose.y(), cos(pose.z()), sin(pose.z()));
					tree_poses.push_back(tree_pose);
				}
				_poses_4dtrees.build(tree_poses, 4);
				
			}

			bool find_pose_neighbors(int pose_index, int num_neighbors, IntVector& neighbors, int tree_dim = 2){
				Vector3d pose = (*_poses)[pose_index];
				vector<pair<double, int>> pose_neighbors;

				if (tree_dim == 2)
				{
					Vector2d query_pose = Vector2d(pose.x(), pose.y());
					_poses_2dtrees.knnSearch(query_pose, num_neighbors, pose_neighbors);
				}
				else if (tree_dim == 4)
				{
					Vector4d query_pose = Vector4d(pose.x(), pose.y(), cos(pose.z()), sin(pose.z()));
					_poses_4dtrees.knnSearch(query_pose, num_neighbors, pose_neighbors);
				}
					
				if (!pose_neighbors.size()) return false;	

				for (size_t i = 0; i < pose_neighbors.size(); i++)
				{
					neighbors.push_back(pose_neighbors[i].second);
				}
							
				return true;

			}
			
			bool find_pose_neighbors(int pose_index, double radius, IntVector& neighbors, int tree_dim = 2){
				
				Vector3d pose = (*_poses)[pose_index];
				vector<pair<double, int>> pose_neighbors;

				if (tree_dim == 2)
				{
					Vector2d query_pose = Vector2d(pose.x(), pose.y());
					_poses_2dtrees.radiusSearch(query_pose, radius, pose_neighbors);
				}
				else if (tree_dim == 4)
				{
					Vector4d query_pose = Vector4d(pose.x(), pose.y(), cos(pose.z()), sin(pose.z()));
					_poses_4dtrees.radiusSearch(query_pose, radius, pose_neighbors);
				}
				

				if (!pose_neighbors.size()) return false;	

				for (size_t i = 0; i < pose_neighbors.size(); i++)
				{
					neighbors.push_back(pose_neighbors[i].second);
				}
							
				return true;

			}

			bool find_pose_neighbor(int pose_index, pair<double, int>& neighbor, double min_distance = MAXFLOAT, int tree_dim = 2){
				Vector3d pose = (*_poses)[pose_index];
				
				if (tree_dim == 2)
				{
					Vector2d query_pose = Vector2d(pose.x(), pose.y());
					_poses_2dtrees.nnSearch(query_pose, neighbor);
				}
				else if (tree_dim == 4)
				{
					Vector4d query_pose = Vector4d(pose.x(), pose.y(), cos(pose.z()), sin(pose.z()));
					_poses_4dtrees.nnSearch(query_pose, neighbor);
				}
				
				if (neighbor.first > min_distance || neighbor.second == -1) return false;					
				return true;

			}

          
	};

} // kdt

#endif // !__KDTREE_H__