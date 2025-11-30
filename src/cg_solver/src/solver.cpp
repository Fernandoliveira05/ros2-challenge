#include <chrono>
#include <memory>
#include <vector>
#include <queue>
#include <string>
#include <algorithm>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

using namespace std::chrono_literals;

struct Coord {
  int x;
  int y;
};

bool inside(int x, int y, int width, int height) {
  return x >= 0 && x < width && y >= 0 && y < height;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("getmap_solver");

  // 1) Cliente para /get_map
  auto map_client = node->create_client<cg_interfaces::srv::GetMap>("/get_map");

  if (!map_client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Servico /get_map nao esta disponivel.");
    rclcpp::shutdown();
    return 1;
  }

  auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
  auto future = map_client->async_send_request(req);

  if (rclcpp::spin_until_future_complete(node, future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Falha ao chamar /get_map.");
    rclcpp::shutdown();
    return 1;
  }

  auto resp = future.get();
  const auto & flat  = resp->occupancy_grid_flattened;
  const auto & shape = resp->occupancy_grid_shape;

  if (shape.size() < 2) {
    RCLCPP_ERROR(node->get_logger(), "occupancy_grid_shape invalido.");
    rclcpp::shutdown();
    return 1;
  }

  int height = shape[0];
  int width  = shape[1];

  std::vector<std::vector<char>> grid(height, std::vector<char>(width, 'b'));
  Coord start{-1, -1};
  Coord goal{-1, -1};

  // 3) Preenche grid, encontra start ('r') e goal ('t')
  for (int i = 0; i < static_cast<int>(flat.size()); ++i) {
    int y = i / width;
    int x = i % width;

    if (!inside(x, y, width, height)) continue;

    const std::string & cell_str = flat[i];
    char c = cell_str.empty() ? 'b' : cell_str[0];

    grid[y][x] = c;

    if (c == 'r') {
      start = {x, y};
    } else if (c == 't') {
      goal = {x, y};
    }
  }

  if (start.x == -1 || start.y == -1) {
    RCLCPP_ERROR(node->get_logger(), "Nao encontrei a posicao do robo ('r').");
    rclcpp::shutdown();
    return 1;
  }

  if (goal.x == -1 || goal.y == -1) {
    RCLCPP_ERROR(node->get_logger(), "Nao encontrei o objetivo ('t').");
    rclcpp::shutdown();
    return 1;
  }

  // 4) BFS para achar caminho mais curto (start -> goal)
  std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
  std::vector<std::vector<Coord>> parent(height, std::vector<Coord>(width, {-1, -1}));

  std::queue<Coord> q;
  q.push(start);
  visited[start.y][start.x] = true;

  bool found = false;

  while (!q.empty()) {
    Coord cur = q.front();
    q.pop();

    if (cur.x == goal.x && cur.y == goal.y) {
      found = true;
      break;
    }

    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    for (int k = 0; k < 4; ++k) {
      int nx = cur.x + dx[k];
      int ny = cur.y + dy[k];

      if (!inside(nx, ny, width, height)) continue;

      char cell = grid[ny][nx];
      if (cell == 'b') continue; // parede

      if (!visited[ny][nx]) {
        visited[ny][nx] = true;
        parent[ny][nx] = cur;
        q.push({nx, ny});
      }
    }
  }

  if (!found) {
    RCLCPP_ERROR(node->get_logger(), "BFS nao encontrou caminho ate o objetivo.");
    rclcpp::shutdown();
    return 1;
  }

  // 5) Reconstrói caminho (goal -> start) e inverte
  std::vector<Coord> path;
  {
    Coord cur = goal;
    while (!(cur.x == start.x && cur.y == start.y)) {
      path.push_back(cur);
      Coord p = parent[cur.y][cur.x];
      if (p.x == -1 && p.y == -1) {
        RCLCPP_ERROR(node->get_logger(), "Falha ao reconstruir caminho.");
        rclcpp::shutdown();
        return 1;
      }
      cur = p;
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
  }

  // 6) Converte caminho em direcoes ("up", "down", "left", "right")
  std::vector<std::string> directions;
  for (size_t i = 1; i < path.size(); ++i) {
    Coord a = path[i - 1];
    Coord b = path[i];

    if (b.x == a.x + 1 && b.y == a.y) {
      directions.push_back("right");
    } else if (b.x == a.x - 1 && b.y == a.y) {
      directions.push_back("left");
    } else if (b.x == a.x && b.y == a.y + 1) {
      directions.push_back("down");
    } else if (b.x == a.x && b.y == a.y - 1) {
      directions.push_back("up");
    }
  }

  // 7) Envia comandos de movimento para /move_command
  auto move_client = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

  if (!move_client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Servico /move_command nao esta disponivel.");
    rclcpp::shutdown();
    return 1;
  }

  for (const auto & dir : directions) {
    auto mreq = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    mreq->direction = dir;

    auto mfut = move_client->async_send_request(mreq);

    // Espera um pouco pra nao atropelar os comandos
    rclcpp::spin_until_future_complete(node, mfut, 2s);
    std::this_thread::sleep_for(100ms);
  }

  RCLCPP_INFO(node->get_logger(), "Sequencia de movimentos concluida.");
  rclcpp::shutdown();
  return 0;
}