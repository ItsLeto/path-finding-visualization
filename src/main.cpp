#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#include <cmath>
#include <fmt/format.h>
#include <limits>
#include <ranges>
#include <vector>

void out(const auto& what, int n = 1) {
    while (n-- > 0)
        std::cout << what;
}

class PathFinding : public olc::PixelGameEngine {
  public:
    PathFinding() {
        sAppName.assign("PathFinding");
        srand(static_cast<unsigned>(time(0)));
    }

  public:
    bool OnUserCreate() override {

        m_grid.reserve(ROWS * COLS);
        for (int row : std::views::iota(0, ROWS)) {
            for (int col : std::views::iota(0, COLS)) {
                m_grid.emplace_back(col, row
                                    // ,((rand() % 100) < 25)
                );
            }
        }

        for (int row : std::views::iota(0, ROWS)) {
            for (int col : std::views::iota(0, COLS)) {
                auto& node = m_grid.at(to_index(row, col));
                if (row > 0)
                    node.neighbours.push_back(&m_grid.at(to_index(row - 1, col)));
                if (col > 0)
                    node.neighbours.push_back(&m_grid.at(to_index(row, col - 1)));
                if (row < ROWS - 1)
                    node.neighbours.push_back(&m_grid.at(to_index(row + 1, col)));
                if (col < COLS - 1)
                    node.neighbours.push_back(&m_grid.at(to_index(row, col + 1)));
                if (row > 0 && col > 0)
                    node.neighbours.push_back(&m_grid.at(to_index(row - 1, col - 1)));
                if (row < ROWS - 1 && col < COLS - 1)
                    node.neighbours.push_back(&m_grid.at(to_index(row + 1, col + 1)));
                if (row < ROWS - 1 && col > 0)
                    node.neighbours.push_back(&m_grid.at(to_index(row + 1, col - 1)));
                if (row > 0 && col < COLS - 1)
                    node.neighbours.push_back(&m_grid.at(to_index(row - 1, col + 1)));
            }
        }

        node_start = &m_grid.at(to_index(1, 1));

        node_end = &m_grid.at(to_index(5, 10));

        solve_astar();

        return true;
    }

    bool OnUserUpdate(float elapsed_time) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        Clear(olc::BLACK);

        auto distance = [](auto n1, auto n2) { return std::sqrt((n1->x - n2->x) * (n1->x - n2->x) + (n1->y - n2->y) * (n1->y - n2->y)); };
        auto heuristic = [&](auto n1, auto n2) { return distance(n1, n2); };
        auto is_not_wall = [](auto n) { return !n->is_wall; };
        auto is_not_visited = [](auto n) { return !n->visited; };
        auto print_fscore = [](auto n) { fmt::print("{:.2f} ", n->f); };
        auto lowest_fscore = [&](auto n1, auto n2) { return n1->f > n2->f; };

        constexpr int NODE_SIZE{32};
        for (const auto& node : m_grid) {
            if (node.is_wall) {
                for (const auto& neighbrous : node.neighbours) {
                    if (!neighbrous->is_wall) {
                        continue;
                    }
                    const int ncx = (node.x * NODE_SIZE) + NODE_SIZE / 2;
                    const int ncy = (node.y * NODE_SIZE) + NODE_SIZE / 2;
                    const int neighbrous_cx = (neighbrous->x * NODE_SIZE) + NODE_SIZE / 2;
                    const int neighbrous_cy = (neighbrous->y * NODE_SIZE) + NODE_SIZE / 2;
                    DrawLine(ncx, ncy, neighbrous_cx, neighbrous_cy, olc::GREY);
                }
            } else {
                // fmt::println("node {} {}", node.x, node.y);
                for (const auto& neighbrous : node.neighbours | std::views::filter(is_not_wall)) {
                    if (neighbrous->is_wall) {
                        continue;
                    }
                    const int ncx = (node.x * NODE_SIZE) + NODE_SIZE / 2;
                    const int ncy = (node.y * NODE_SIZE) + NODE_SIZE / 2;
                    const int neighbrous_cx = (neighbrous->x * NODE_SIZE) + NODE_SIZE / 2;
                    const int neighbrous_cy = (neighbrous->y * NODE_SIZE) + NODE_SIZE / 2;
                    DrawLine(ncx, ncy, neighbrous_cx, neighbrous_cy, olc::BLUE);
                }
            }
        }

        for (const auto& node : m_grid) {
            draw_node(&node, (!node.is_wall) ? olc::Pixel(0, 0, 255, 128) : olc::Pixel(192, 192, 192));
            if (node.visited && !node.is_wall) {
                draw_node(&node, olc::BLUE);
            }
        }
        for (auto n : m_open_set) {
            draw_node(n, olc::MAGENTA);
        }
        draw_node(node_end, olc::GREEN);
        draw_node(node_start, olc::RED);

        olc::vi2d mouse_pos = GetMousePos();
        if (GetMouse(0).bReleased) {
            // fmt::println("Mouse One Pressed {} {}", mouse_pos.x, mouse_pos.y);
            if (GetKey(olc::Key::R).bHeld) {
                node_start = &m_grid.at(to_index(mouse_pos.y / NODE_SIZE, mouse_pos.x / NODE_SIZE));
                node_start->is_wall = false;
            } else if (GetKey(olc::Key::N).bHeld) {
                node_end = &m_grid.at(to_index(mouse_pos.y / NODE_SIZE, mouse_pos.x / NODE_SIZE));
                node_end->is_wall = false;
            } else {
                auto& node = m_grid.at(to_index(mouse_pos.y / NODE_SIZE, mouse_pos.x / NODE_SIZE));
                node.is_wall = !node.is_wall;
                node.visited = false;
            }
            solve_astar();
        } else if (GetMouse(0).bHeld) {
            auto& node = m_grid.at(to_index(mouse_pos.y / NODE_SIZE, mouse_pos.x / NODE_SIZE));
            node.is_wall = true;
            node.visited = false;
        } else if (GetMouse(1).bHeld) {
            auto& node = m_grid.at(to_index(mouse_pos.y / NODE_SIZE, mouse_pos.x / NODE_SIZE));
            node.is_wall = false;
            node.visited = false;
        }

        if (GetKey(olc::Key::X).bReleased) {
            for (auto& node : m_grid) {
                node.visited = false;
                node.f = INFINITY; /* std::numeric_limits<double>::max(); */
                node.g = INFINITY; /* std::numeric_limits<double>::max(); */
                node.parent = nullptr;
            }

            node_start->g = 0.0;
            node_start->f = heuristic(node_start, node_end);
            m_open_set.push_back(node_start);
        }

        if (!m_open_set.empty()) {
            if (!std::ranges::is_heap(m_open_set, lowest_fscore)) {
                std::ranges::make_heap(m_open_set, lowest_fscore);
            }

            Node* current = m_open_set.front();
            if (current == node_end) {
                m_open_set.clear();
                return true;
            }
            current->visited = true;
            std::ranges::pop_heap(m_open_set);
            m_open_set.pop_back();

            for (auto& neighbour : current->neighbours | std::views::filter(is_not_wall) | std::views::filter(is_not_visited)) {
                double tentative_gScore = current->g + distance(current, neighbour);
                if (tentative_gScore < neighbour->g) {
                    neighbour->parent = current;
                    neighbour->g = tentative_gScore;
                    neighbour->f = tentative_gScore + heuristic(neighbour, node_end);
                    if (!neighbour->visited && m_open_set.end() == std::ranges::find(m_open_set, neighbour)) {
                        m_open_set.push_back(neighbour);
                        std::ranges::push_heap(m_open_set, lowest_fscore);
                    }
                }
            }

            if (current != nullptr) {
                Node* n = current;
                // fmt::println("Path ------");
                while (n->parent != nullptr) {
                    // fmt::println("Node {} {}", n->x, n->y);
                    const int ncx = (n->x * NODE_SIZE) + NODE_SIZE / 2;
                    const int ncy = (n->y * NODE_SIZE) + NODE_SIZE / 2;
                    const int parent_cx = (n->parent->x * NODE_SIZE) + NODE_SIZE / 2;
                    const int parent_cy = (n->parent->y * NODE_SIZE) + NODE_SIZE / 2;
                    DrawLine(ncx, ncy, parent_cx, parent_cy, olc::YELLOW);
                    n = n->parent;
                }
            }
        } else {
            if (node_end != nullptr) {
                Node* n = node_end;
                // fmt::println("Path ------");
                while (n->parent != nullptr) {
                    // fmt::println("Node {} {}", n->x, n->y);
                    const int ncx = (n->x * NODE_SIZE) + NODE_SIZE / 2;
                    const int ncy = (n->y * NODE_SIZE) + NODE_SIZE / 2;
                    const int parent_cx = (n->parent->x * NODE_SIZE) + NODE_SIZE / 2;
                    const int parent_cy = (n->parent->y * NODE_SIZE) + NODE_SIZE / 2;
                    DrawLine(ncx, ncy, parent_cx, parent_cy, olc::YELLOW);
                    n = n->parent;
                }
            }
        }
        return true;
    }

    void solve_astar() {
        auto distance = [](auto n1, auto n2) { return std::sqrt((n1->x - n2->x) * (n1->x - n2->x) + (n1->y - n2->y) * (n1->y - n2->y)); };
        auto heuristic = [&](auto n1, auto n2) { return distance(n1, n2); };
        auto is_not_wall = [](auto n) { return !n->is_wall; };
        auto is_not_visited = [](auto n) { return !n->visited; };
        auto print_fscore = [](auto n) { fmt::print("{:.2f} ", n->f); };

        auto lowest_fscore = [&](auto n1, auto n2) { return n1->f > n2->f; };

        for (auto& node : m_grid) {
            node.visited = false;
            node.f = INFINITY;
            node.g = INFINITY;
            node.parent = nullptr;
        }

        Node* current = node_start;
        node_start->g = 0.0;
        node_start->f = heuristic(node_start, node_end);

        std::vector<Node*> open_set;
        open_set.push_back(node_start);
        while (!open_set.empty() && current != node_end) {
            if (!std::ranges::is_heap(open_set, lowest_fscore)) {
                std::ranges::make_heap(open_set, lowest_fscore);
            }

            current = open_set.front();
            current->visited = true;
            std::ranges::pop_heap(open_set);
            open_set.pop_back();

            for (auto& neighbour : current->neighbours | std::views::filter(is_not_wall) | std::views::filter(is_not_visited)) {
                double tentative_gScore = current->g + distance(current, neighbour);
                if (tentative_gScore < neighbour->g) {
                    neighbour->parent = current;
                    neighbour->g = tentative_gScore;
                    neighbour->f = tentative_gScore + heuristic(neighbour, node_end);
                    if (!neighbour->visited && open_set.end() == std::ranges::find(open_set, neighbour)) {
                        open_set.push_back(neighbour);
                        std::ranges::push_heap(open_set, lowest_fscore);
                    }
                }
            }
        }
    }

    int to_index(int row, int col) {
        return col + row * COLS;
    }

    struct Node {
        int x;
        int y;
        bool is_wall{false};
        bool visited{false};
        double g{INFINITY};
        double f{INFINITY};
        std::vector<Node*> neighbours;
        Node* parent{nullptr};
    };

    void draw_node(const Node* node, olc::Pixel color) {
        constexpr int NODE_SIZE{32};
        FillCircle((node->x * NODE_SIZE) + NODE_SIZE / 2, //
                   (node->y * NODE_SIZE) + NODE_SIZE / 2, //
                   NODE_SIZE / 2 - 8,                     //
                   color);
    }

    constexpr static int ROWS{240 * 4 / 32};
    constexpr static int COLS{256 * 4 / 32};
    std::vector<Node> m_grid;

    Node* node_start{nullptr};
    Node* node_end{nullptr};

    std::vector<Node*> m_open_set;
    std::vector<Node*> m_closed_set;
};

int main() {
    PathFinding demo;
    if (demo.Construct(256 * 4, 240 * 4, 1, 1)) {
        demo.Start();
    }
    return 0;
}
