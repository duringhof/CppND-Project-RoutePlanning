#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

using std::ifstream;
using std::string;
using std::cin;
using std::cout;
using std::endl;
using std::vector;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path) {
  ifstream is{path, std::ios::binary | std::ios::ate};
  if (!is)
    return std::nullopt;

  auto size = is.tellg();
  std::vector<std::byte> contents(size);

  is.seekg(0);
  is.read((char *)contents.data(), size);

  if (contents.empty())
    return std::nullopt;
  return std::move(contents);
}

int main(int argc, const char **argv)
{
  string osm_data_file = "";
  if (argc > 1) {
    for (int i = 1; i < argc; ++i)
      if (std::string_view{argv[i]} == "-f" && ++i < argc)
        osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
      cout << "Reading OpenStreetMap data from the following file: "
           << osm_data_file << endl;
      auto data = ReadFile(osm_data_file);
      if (!data)
        cout << "Failed to read." << endl;
      else
        osm_data = std::move(*data);
    }

    vector<float> ui_coords{-1.0, -1.0, -1.0, -1.0};
    vector<string> ui_coords_labels{"Start_X", "Start_Y", "End_X", "End_Y"};

    for (int i = 0; i < 4; i++) {
      while (0.0 > ui_coords[i] || ui_coords[i] > 100.0) {
        cout << "Please enter your " << ui_coords_labels[i] << " coordinate: ";
        cin >> ui_coords[i];
        if (0.0 > ui_coords[i] || ui_coords[i] > 100.0)
          cout << "That value is out of range !!! (pick a value between 0 and "
                  "100)"
               << endl;
      }
    }

    cout << "You picked: " << ui_coords[0] << ", " << ui_coords[1] << ", "
         << ui_coords[2] << ", " << ui_coords[3] << endl;

    float start_x = ui_coords[0];
    float start_y = ui_coords[1];
    float end_x = ui_coords[2];
    float end_y = ui_coords[3];

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
