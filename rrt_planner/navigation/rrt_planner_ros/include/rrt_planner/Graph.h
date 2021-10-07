#include <vector>
#include <memory>
#include <cmath>

/**
 * A utility class to represent a 2D point
 */
class Point2D
{
public:
  Point2D(): x_(0), y_(0) {}
  Point2D(int x, int y): x_(x), y_(y) {}
  //Point2D(const Point2D &p): x_(p.x()), y_(p.y()){}

  double distance(Point2D &p){
    return (std::sqrt(std::pow(x_ - p.x(), 2) + std::pow(y_ - p.y(), 2)));
  }

  bool operator==(const Point2D &p) {
    return (x_ == p.x() && y_ == p.y());
  }

  bool operator!=(const Point2D &p) {
    return !(*this == p);
  }

  int x() const
  {
    return x_;
  }

  int y() const
  {
    return y_;
  }

  void x(int x)
  {
    x_ = x;
  }

  void y(int y)
  {
    y_ = y;
  }

private:
  int x_;
  int y_;
};

class Node {
  public:
    Node(){
      root_ = Point2D(0,0);
      links_.resize(0);
    };

    explicit Node(const Point2D &root){
      root_ = Point2D(root);
      links_.resize(0);
    }

  std::shared_ptr<Node> connect(const Point2D &link) {
    auto node = findNode(link);
    if (node == nullptr) {
      node.reset(new Node(link));
      links_.push_back(node);
    }
    return node;
  }

  std::shared_ptr<Node> connect(const std::shared_ptr<Node> &link) {
    auto node = findNode(link->getRoot());
    if (node == nullptr) {
      node = link;
      links_.push_back(node);
    }
    return node;
  }

  std::shared_ptr<Node> findNode(const Point2D &p) {
    for (auto &node : links_) {
      if (node->getRoot() == p)
        return node;
    }
    return nullptr;
  }

  Point2D getRoot() {
    return root_;
  }

  std::vector<std::shared_ptr<Node>> getLinks() {
    return links_;
  }

  bool operator==(const Node &n) {
    return (this->root_ == n.root_);
  }


  private:
    Point2D root_;
    std::vector<std::shared_ptr<Node>> links_;
};

class Graph {
  public:
    Graph();

    std::shared_ptr<Node> add(const Point2D&);

    void add(const Point2D &, const Point2D &);

    std::shared_ptr<Node> findNode(const Point2D &);

    std::shared_ptr<Node> findNode(const std::shared_ptr<Node> &);

    int getSize();
    std::vector<std::shared_ptr<Node>> getNodes();
    bool getPath(std::shared_ptr<Node>, std::shared_ptr<Node>, std::vector<Point2D>&);

  private:

  std::vector<std::shared_ptr<Node>> nodes_;
  int size_;

};