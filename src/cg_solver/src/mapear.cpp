#include <chrono>
#include <memory>
#include <queue>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <algorithm>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

using namespace std::chrono_literals;

struct Coord {
  int x;
  int y;

  bool operator==(const Coord & other) const {
    return x == other.x && y == other.y;
  }
};

namespace std {
  template<>
  struct hash<Coord> {
    std::size_t operator()(const Coord & c) const noexcept {
      // hash simples: combina x e y
      return (std::hash<int>()(c.x) << 1) ^ std::hash<int>()(c.y);
    }
  };
}

enum class CellState {
  Unknown,
  Free,
  Wall,
  Start,
  Goal
};

// ----------------- Nó para o ROS2 -----------------

class MapearNode : public rclcpp::Node {
public:
  MapearNode()
  : rclcpp::Node("mapear_node")
  {
    sensor_sub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
      "/culling_games/robot_sensors",
      10,
      std::bind(&MapearNode::sensor_callback, this, std::placeholders::_1)
    );

    move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

    current_ = {0, 0};
    start_   = current_;
    goal_    = {999999, 999999}; 

    map_[start_] = CellState::Start;
    visited_exploration_.insert(start_);
    dfs_stack_.push_back(start_);

    RCLCPP_INFO(get_logger(), "Node de mapeamento iniciado. Primeiro eu vou explorar TUDO  e depois faço o melhor caminho :) (Explorar tudo → BFS).");
  }

private:
  // --- ROS ---
  rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
  rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr            move_client_;

  // --- Mapa e estados ---
  std::unordered_map<Coord, CellState> map_;               
  std::unordered_set<Coord> visited_exploration_;
  Coord current_;
  Coord start_;
  Coord goal_;

  bool exploration_done_  = false; 
  bool executing_path_    = false; 

  std::vector<Coord> dfs_stack_;   

  std::vector<Coord> best_path_;
  std::size_t        path_index_ = 0;

  // ----------------- Helpers de célula -----------------

  static char cell_char_from_string(const std::string & s) {
    if (s.empty()) return 'b';
    return s[0];
  }

  static CellState cell_state_from_char(char c) {
    switch (c) {
      case 'b': return CellState::Wall;
      case 'f': return CellState::Free;
      case 'r': return CellState::Free; 
      case 't': return CellState::Goal;
      default:  return CellState::Unknown;
    }
  }

  void set_cell(const Coord & c, CellState s) {
    auto it = map_.find(c);
    if (it == map_.end()) {
      map_[c] = s;
    } else {
      if (it->second == CellState::Unknown || it->second == CellState::Free) {
        if (s == CellState::Goal || s == CellState::Wall) {
          it->second = s;
        }
      }
    }
  }

  CellState get_cell(const Coord & c) const {
    auto it = map_.find(c);
    if (it == map_.end()) return CellState::Unknown;
    return it->second;
  }

  // ----------------- Movimento físico -----------------

  void send_move(const std::string & direction) {
    if (!move_client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "Serviço /move_command ainda não disponível.");
      return;
    }

    auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    req->direction = direction;
    move_client_->async_send_request(req);
  }

  bool coord_to_direction(const Coord & from, const Coord & to, std::string & out_dir) {
    if (to.x == from.x && to.y == from.y - 1) {
      out_dir = "up";
    } else if (to.x == from.x && to.y == from.y + 1) {
      out_dir = "down";
    } else if (to.x == from.x - 1 && to.y == from.y) {
      out_dir = "left";
    } else if (to.x == from.x + 1 && to.y == from.y) {
      out_dir = "right";
    } else {
      return false;
    }
    return true;
  }

  // ----------------- Fase 3: seguir caminho ótimo -----------------

  void follow_path_step() {
    if (path_index_ >= best_path_.size()) {
      RCLCPP_INFO(get_logger(), "Chegamos ao goal seguindo o caminho ótimo.");
      executing_path_ = false;
      return;
    }

    Coord next = best_path_[path_index_];
    std::string dir;
    if (!coord_to_direction(current_, next, dir)) {
      RCLCPP_WARN(
        get_logger(),
        "Coordenadas inconsistentes no caminho ótimo: cur=(%d,%d) next=(%d,%d)",
        current_.x, current_.y, next.x, next.y
      );
      executing_path_ = false;
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Caminho ótimo: (%d,%d) -> (%d,%d) via %s",
      current_.x, current_.y, next.x, next.y, dir.c_str()
    );

    send_move(dir);
    current_ = next;
    ++path_index_;
  }

  // ----------------- Callback de sensores -----------------

  void sensor_callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
    if (executing_path_) {
      follow_path_step();
      return;
    }

    if (exploration_done_) {
      return;
    }

    Coord up    { current_.x,     current_.y - 1 };
    Coord down  { current_.x,     current_.y + 1 };
    Coord left  { current_.x - 1, current_.y     };
    Coord right { current_.x + 1, current_.y     };

    char up_c    = cell_char_from_string(msg->up);
    char down_c  = cell_char_from_string(msg->down);
    char left_c  = cell_char_from_string(msg->left);
    char right_c = cell_char_from_string(msg->right);

    CellState up_s    = cell_state_from_char(up_c);
    CellState down_s  = cell_state_from_char(down_c);
    CellState left_s  = cell_state_from_char(left_c);
    CellState right_s = cell_state_from_char(right_c);

    set_cell(up,    up_s);
    set_cell(down,  down_s);
    set_cell(left,  left_s);
    set_cell(right, right_s);

    if (up_s == CellState::Goal)    goal_ = up;
    if (down_s == CellState::Goal)  goal_ = down;
    if (left_s == CellState::Goal)  goal_ = left;
    if (right_s == CellState::Goal) goal_ = right;

    explore_step();
  }

  // ----------------- Fase 1: exploração DFS -----------------

  void add_candidate_if_free_and_unvisited(const Coord & c, const std::string & dir,
                                           std::vector<std::pair<Coord, std::string>> & out) {
    CellState st = get_cell(c);
    if (st == CellState::Free && !visited_exploration_.count(c)) {
      out.push_back({c, dir});
    }
  }

  void explore_step() {
    std::vector<std::pair<Coord, std::string>> candidates;

    Coord up    { current_.x,     current_.y - 1 };
    Coord down  { current_.x,     current_.y + 1 };
    Coord left  { current_.x - 1, current_.y     };
    Coord right { current_.x + 1, current_.y     };

    add_candidate_if_free_and_unvisited(up,    "up",    candidates);
    add_candidate_if_free_and_unvisited(right, "right", candidates);
    add_candidate_if_free_and_unvisited(down,  "down",  candidates);
    add_candidate_if_free_and_unvisited(left,  "left",  candidates);

    if (!candidates.empty()) {
      auto [next_coord, dir] = candidates.front();

      RCLCPP_INFO(
        get_logger(),
        "Explorando: (%d,%d) -> (%d,%d) via %s",
        current_.x, current_.y, next_coord.x, next_coord.y, dir.c_str()
      );

      send_move(dir);
      current_ = next_coord;
      visited_exploration_.insert(current_);
      dfs_stack_.push_back(current_);
      return;
    }

    if (dfs_stack_.size() <= 1) {
      exploration_done_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Exploração concluída. Voltou ao start=(%d,%d).",
        start_.x, start_.y
      );

      if (goal_.x == 999999 && goal_.y == 999999) {
        RCLCPP_WARN(get_logger(), "Goal nunca foi visto no mapa, não há caminho pra calcular.");
      } else {
        compute_best_path_bfs();
      }

      return;
    }

    Coord prev = dfs_stack_[dfs_stack_.size() - 2];
    std::string dir_back;
    if (!coord_to_direction(current_, prev, dir_back)) {
      RCLCPP_WARN(
        get_logger(),
        "Backtracking impossível: cur=(%d,%d) prev=(%d,%d)",
        current_.x, current_.y, prev.x, prev.y
      );
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Backtracking: (%d,%d) -> (%d,%d) via %s",
      current_.x, current_.y, prev.x, prev.y, dir_back.c_str()
    );

    send_move(dir_back);
    current_ = prev;
    dfs_stack_.pop_back();
  }

  // ----------------- Fase 2: BFS para achar caminho ótimo -----------------

  void compute_best_path_bfs() {
    std::queue<Coord> q;
    std::unordered_set<Coord> visited;
    std::unordered_map<Coord, Coord> parent;

    q.push(start_);
    visited.insert(start_);

    auto enqueue_if_walkable = [&](const Coord & from, const Coord & c) {
      CellState st = get_cell(c);
      if (st == CellState::Wall || st == CellState::Unknown) {
        return; 
      }
      if (!visited.count(c)) {
        visited.insert(c);
        parent[c] = from;
        q.push(c);
      }
    };

    bool reached_goal = false;

    while (!q.empty()) {
      Coord cur = q.front();
      q.pop();

      if (cur == goal_) {
        reached_goal = true;
        break;
      }

      Coord up    { cur.x,     cur.y - 1 };
      Coord down  { cur.x,     cur.y + 1 };
      Coord left  { cur.x - 1, cur.y     };
      Coord right { cur.x + 1, cur.y     };

      enqueue_if_walkable(cur, up);
      enqueue_if_walkable(cur, down);
      enqueue_if_walkable(cur, left);
      enqueue_if_walkable(cur, right);
    }

    if (!reached_goal) {
      RCLCPP_ERROR(
        get_logger(),
        "BFS não encontrou caminho de start=(%d,%d) até goal=(%d,%d).",
        start_.x, start_.y, goal_.x, goal_.y
      );
      return;
    }

    std::vector<Coord> path;
    Coord cur = goal_;
    while (!(cur == start_)) {
      path.push_back(cur);
      cur = parent[cur];
    }
    std::reverse(path.begin(), path.end());

    best_path_  = path;
    path_index_ = 0;
    executing_path_ = true;
    current_ = start_; 

    RCLCPP_INFO(
      get_logger(),
      "Caminho ótimo encontrado (BFS) com %zu passos de start=(%d,%d) até goal=(%d,%d).",
      best_path_.size(), start_.x, start_.y, goal_.x, goal_.y
    );
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapearNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
