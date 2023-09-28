#include <memory>
#include "split.h"
#include "plane_side_predicate.h"
#include "eigen_covariance.h"
#include "brute_force_search.h"

template <typename IteratorType_>
class TreeNode_ {
public:
  using IteratorType = IteratorType_;
  using PointType = typename IteratorType_::value_type;
  using Scalar    = typename PointType::Scalar;
  static constexpr int Dim = PointType::RowsAtCompileTime;
  using CovarianceType = Eigen::Matrix<Scalar, Dim, Dim>;
  using ThisType  = TreeNode_<IteratorType>;
  using PtrType = std::unique_ptr<ThisType>;
  using AnswerType = std::vector<PointType*>;
  IntVector _indices;
  
  
  TreeNode_(
            IteratorType begin_,
            IteratorType end_,
            IntVector::iterator indices_begin_,
            IntVector::iterator indices_end_,
            int max_points_in_leaf=20):
    _begin(begin_),
    _end(end_)
  {
    CovarianceType cov;
    computeMeanAndCovariance(_mean, cov, _begin, _end);
    int num_points=std::distance(_begin, _end);
    if (num_points < max_points_in_leaf){
      _smallest_eigenvector = smallestEigenVector(cov);
      return;
    }
    
    _largest_eigenvector = largestEigenVector(cov);

    
    IntVector::iterator indices_middle;
    pair<IteratorType, IntVector::iterator> middle_pair = split(_begin, _end, indices_begin_, indices_end_, PlaneSidePredicate_<PointType>(_mean, _largest_eigenvector) );
    _left  = PtrType(new ThisType(_begin, middle_pair.first, indices_begin_, middle_pair.second, max_points_in_leaf) );
    _right = PtrType(new ThisType(middle_pair.first, _end, middle_pair.second, indices_end_, max_points_in_leaf) );
    
  }

  void getLeafPoints(AnswerType& answers, const PointType& query) {
      
      if (! _left && !_right) {
        for (auto it=_begin; it!=_end; ++it) {
          auto& p=*it;
          answers.push_back(&p);
        }
        return;
      }
      Scalar distance_from_split_plane = (query-_mean).dot(_largest_eigenvector);
      if (distance_from_split_plane<Scalar(0))
        _left->getLeafPoints(answers, query);
      else
        _right->getLeafPoints(answers, query);
      
  }

  void fastSearch(AnswerType& answers,
                  const PointType& query,
                  Scalar norm) {
    if (! _left && !_right) {
      bruteForceSearch(answers, _begin, _end, query, norm);
      return;
    }
    Scalar distance_from_split_plane =  (query-_mean).dot(_largest_eigenvector);
    if (distance_from_split_plane<Scalar(0))
      _left->fastSearch(answers,query,norm);
    else
      _right->fastSearch(answers,query,norm);
  }

  void fullSearch(AnswerType& answers,
                  const PointType& query,
                  Scalar norm) {
    if (!_left && !_right) {
      bruteForceSearch(answers, _begin, _end, query, norm);
      return;
    }
    Scalar distance_from_split_plane =  (query-_mean).dot(_largest_eigenvector);
    if (distance_from_split_plane < -norm ){
      _left->fullSearch(answers,query, norm);
    }
    else if (distance_from_split_plane > norm ){
      _right->fullSearch(answers,query,norm);
    }
    else {
      _left->fullSearch(answers,query, norm);
      _right->fullSearch(answers,query,norm);
    }
  }

  
  PointType* bestMatchFast(const PointType& query,
                           Scalar norm) {
    if (! _left && !_right) {
      return bruteForceBestMatch(_begin, _end, query, norm);
    }
    Scalar distance_from_split_plane =  (query-_mean).dot(_largest_eigenvector);
    if (distance_from_split_plane<Scalar(0))
      return _left->bestMatchFast(query, norm);
    else
      return _right->bestMatchFast(query, norm);
  }

  // this returns the closest point in the set,
  // among those whose distance from query is smaller than norm
  // doing a full search
  PointType* bestMatchFull(const PointType& query,
                           Scalar norm) {
    if (! _left && !_right) {
      return bruteForceBestMatch(_begin, _end, query, norm);
    }
    Scalar distance_from_split_plane =  (query-_mean).dot(_largest_eigenvector);

    if (distance_from_split_plane < -norm )
      return _left->bestMatchFull(query, norm);

    if (distance_from_split_plane > norm )
      return _right->bestMatchFull(query, norm);

    PointType* p_left  = _left->bestMatchFull(query, norm);
    PointType* p_right = _right->bestMatchFull(query, norm);
    Scalar d_left=norm*norm;
    Scalar d_right=norm*norm;
    if (p_left)
      d_left=((*p_left)-query).squaredNorm();
    if (p_right)
      d_right=((*p_right)-query).squaredNorm();
    if (p_left<p_right)
      return p_left;

    return p_right;
  }

  PointType _mean, _smallest_eigenvector, _largest_eigenvector;
  IteratorType _begin, _end;
  PtrType _left, _right;
};

