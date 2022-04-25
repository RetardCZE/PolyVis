/**
 * File:   trivis_debug.cc
 *
 * Date:   29.11.2021
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */


#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "data_loading/load_map.h"
#include "drawing/drawing.h"
#include "drawing/random_colors.h"
#include "logging/logging.h"

#include "trivis/core/tri_vis.h"
#include "trivis/core/geom/robust_geometry.h"
#include "trivis/core/geom/generic_geom_types.h"
#include "trivis/core/utils/simple_clock.h"
#include "trivis/core/utils/clipper_utils.h"
#include "trivis/core/geom/generic_geom_utils.h"

#include "trivis/map_coverage/map_coverage.h"
#include "trivis/map_coverage/random_points.h"
#include "trivis/map_coverage/macs.h"

#ifndef INPUT_MAPS_DIR
#define INPUT_MAPS_DIR "."
#endif

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace tv = trivis;
namespace tvc = tv::core;
namespace tvg = tvc::geom;
namespace tve = trivis_examples;
namespace dr = tve::drawing;

/**
 * All program option variables and their default values should be defined here.
 * For each variable, there should be an option added in AddProgramOptions.
 */
struct ProgramOptionVariables {
    bool verbosity_silent = false;
    bool verbosity_quiet = false;
    bool verbosity_verbose = false;
    std::string input_map_name = "undefined";
    std::string input_map_extension = ".txt";
    std::string input_map_dir = INPUT_MAPS_DIR;
    std::string input_map_full_path;
    bool generate_map_coverage = false;
    bool test_bucketing = false;
    double map_scale = -1.0;
    double vis_radius = -1.0;
};

void AddProgramOptions(
    po::options_description &options_description,
    ProgramOptionVariables &pov
) {
    options_description.add_options()
        ("help,h", "Produce this help message. \n (*) Overwrites options: all.")
        ("verbose,v",
         po::bool_switch(&pov.verbosity_verbose)->default_value(pov.verbosity_verbose),
         "Log messages: fatal, error, warning, info, debug.")
        ("quiet,q",
         po::bool_switch(&pov.verbosity_quiet)->default_value(pov.verbosity_quiet),
         "Log messages: fatal, error, warning. \n (*) Overwrites options: verbose.")
        ("silent,s",
         po::bool_switch(&pov.verbosity_silent)->default_value(pov.verbosity_silent),
         "Log messages: fatal, error. \n (*) Overwrites options: verbose, quiet.")
        ("map-name,m",
         po::value(&pov.input_map_name)->default_value(pov.input_map_name),
         "Map name.")
        ("map-ext",
         po::value(&pov.input_map_extension)->default_value(pov.input_map_extension),
         "Map file extension.")
        ("map-dir",
         po::value(&pov.input_map_dir)->default_value(pov.input_map_dir),
         "Map file directory.")
        ("map",
         po::value(&pov.input_map_full_path)->default_value(pov.input_map_full_path),
         "Full path to the map file. \n (*) Overwrites options: map_name, map_ext, map_dir.")
        ("map-scale,s",
         po::value(&pov.map_scale),
         "Map scale (optional).")
        ("vis-radius,r",
         po::value(&pov.vis_radius)->default_value(pov.vis_radius),
         "Visibility radius (-1 ~ infinite).")
        ("map-coverage",
         po::bool_switch(&pov.generate_map_coverage)->default_value(pov.generate_map_coverage),
         "Regions covering the map will be generated.")
        ("test-bucketing",
         po::bool_switch(&pov.test_bucketing)->default_value(pov.test_bucketing),
         "Runs bucketing test procedure.");
}

tve::logging::severity_level GetSeverity(const ProgramOptionVariables &pov) {
    using namespace tve::logging;
    if (pov.verbosity_silent) {
        return severity_level::error;
    } else if (pov.verbosity_quiet) {
        return severity_level::warning;
    } else if (pov.verbosity_verbose) {
        return severity_level::debug;
    } else {
        return severity_level::info;
    }
}

/**
 *
 * Parses arguments and initializes logging.
 *
 * @param argc Number of arguments.
 * @param argv Array of arguments.
 * @param pov
 * @return Character 'e' if an exception occurred, 'h' if --help option, '0' else.
 */
char ParseProgramOptions(
    int argc,
    const char *const *argv,
    ProgramOptionVariables &pov
) {
    using namespace tve::logging;
    if (argc < 2) {
        InitLogging(severity_level::info);
        LOGF_FTL("No arguments provided. Use --help to see available options.");
        return 'e';
    }
    po::variables_map vm;
    po::options_description command_line_options;
    po::options_description options_description("General options");
    AddProgramOptions(options_description, pov);
    try {
        // Parse the command line arguments.
        command_line_options.add(options_description);
        po::store(po::parse_command_line(argc, argv, command_line_options), vm);
        po::notify(vm);
    } catch (const std::exception &e) {
        // If exception, log it and return 'e'.
        InitLogging(severity_level::info);
        LOGF_FTL("Error in parsing arguments: " << e.what() << ".");
        return 'e';
    }
    if (vm.count("help")) {
        // If '-h' or '--help' option, print the options and return 'h'.
        command_line_options.print(std::cout, 80);
        return 'h';
    }
    InitLogging(GetSeverity(pov));
    // Make input_file_name, input_file_extension, input_file_dir, and input_file_full_path consistent.
    if (pov.input_map_full_path.empty()) {
        pov.input_map_full_path = pov.input_map_dir + "/" + pov.input_map_name + (pov.input_map_extension.empty() ? "" : pov.input_map_extension);
    } else {
        auto aux = fs::path(pov.input_map_full_path);
        pov.input_map_name = fs::change_extension(aux, "").filename().string();
        pov.input_map_extension = aux.extension().string();
        pov.input_map_dir = aux.parent_path().string();
    }
    return '0';
}

void DrawDetailedRadialVisibilityRegion(
    const tve::drawing::MapDrawer &drawer,
    const tvg::RadialVisibilityRegion &vis_reg
) {
    drawer.DrawArc(vis_reg.seed, 0.30, 0.0, 2.0 * M_PI, 0.01, dr::kColorDarkOrange);
    drawer.DrawArc(vis_reg.seed, 0.20, 0.0, 2.0 * M_PI, 0.01, dr::kColorDarkOrange);
    drawer.DrawPoint(vis_reg.seed, 0.10, dr::kColorOrange);
    drawer.DrawPoint(vis_reg.seed, 0.09, dr::kColorDarkOrange);
    drawer.DrawPoint(vis_reg.seed, 0.08, dr::kColorOrange);
    drawer.DrawPoint(vis_reg.seed, 0.07, dr::kColorDarkOrange);
    drawer.DrawPoint(vis_reg.seed, 0.06, dr::kColorOrange);
    drawer.DrawPoint(vis_reg.seed, 0.05, dr::kColorDarkOrange);
    drawer.DrawPoint(vis_reg.seed, 0.04, dr::kColorOrange);
    drawer.DrawPoint(vis_reg.seed, 0.03, dr::kColorDarkOrange);
    drawer.DrawPoint(vis_reg.seed, 0.02, dr::kColorOrange);
    drawer.DrawPoint(vis_reg.seed, 0.01, dr::kColorDarkOrange);
    drawer.DrawArc(vis_reg.seed, vis_reg.radius, 0.0, 2.0 * M_PI, 0.05, dr::kColorMagenta, 0.1);
    if (vis_reg.vertices.empty()) {
        if (vis_reg.radius == 0.0) {
            drawer.DrawPoint(vis_reg.seed, 0.05, dr::kColorBlueViolet);
        } else {
            drawer.DrawArc(vis_reg.seed, vis_reg.radius, 0.0, 2.0 * M_PI, 0.05, dr::kColorDeepSkyBlue);
        }
    } else {
        for (int i_prev = static_cast<int>(vis_reg.vertices.size()) - 1, i = 0; i < vis_reg.vertices.size(); i_prev = i++) {
            const auto &vi_prev = vis_reg.vertices[i_prev];
            const auto &vi = vis_reg.vertices[i];
            if (vi.edge_flag > -1) {
                drawer.DrawLine(vi_prev.point, vi.point, 0.05, dr::kColorRed);
                drawer.DrawPoint(vi.point, 0.03, dr::kColorRed);
            } else if (vi.edge_flag == -1) {
                drawer.DrawLine(vi_prev.point, vi.point, 0.05, dr::kColorLimeGreen);
            } else if (vi.edge_flag == -2) {
                drawer.DrawLine(vi_prev.point, vi.point, 0.05, dr::kColorDeepSkyBlue, 0.1);
                auto a = vi.point - vis_reg.seed;
                auto b = vi_prev.point - vis_reg.seed;
                drawer.DrawArc(vis_reg.seed, vis_reg.radius, std::atan2(a.x, a.y) - M_PI_2, std::atan2(b.x, b.y) - M_PI_2, 0.05, dr::kColorDeepSkyBlue);
            } else if (vi.edge_flag == -3) {
                drawer.DrawLine(vi_prev.point, vi.point, 0.05, dr::kColorBlueViolet);
            } else {
                drawer.DrawLine(vi_prev.point, vi.point, 0.05, dr::kColorDeepSkyBlue);
            }
        }
        for (int i_prev = static_cast<int>(vis_reg.vertices.size()) - 1, i = 0; i < vis_reg.vertices.size(); i_prev = i++) {
            const auto &vi_prev = vis_reg.vertices[i_prev];
            const auto &vi = vis_reg.vertices[i];
            auto u = vi_prev.point - vi.point;
            double norm = u.Norm();
            u = u / u.Norm();
            std::string text = std::to_string(i);
            if (vi.edge_flag >= 0) {
                text += "-" + std::to_string(vi.edge_flag);
            }
            drawer.DrawText(text, vi.point + u * norm * 0.5, 0.03, dr::kColorMaroon);
        }
        for (int i_prev = static_cast<int>(vis_reg.vertices.size()) - 1, i = 0; i < vis_reg.vertices.size(); i_prev = i++) {
            const auto &vi_prev = vis_reg.vertices[i_prev];
            const auto &vi = vis_reg.vertices[i];
            if (vi.edge_flag > 0) {
                drawer.DrawPoint(vi.point, 0.03, dr::kColorRed);
            } else if (vi.edge_flag == -1) {
                drawer.DrawPoint(vi.point, 0.03, dr::kColorLimeGreen);
            } else if (vi.edge_flag == -2) {
                drawer.DrawPoint(vi.point, 0.03, dr::kColorDeepSkyBlue);
            } else if (vi.edge_flag == -3) {
                drawer.DrawPoint(vi.point, 0.03, dr::kColorBlueViolet);
            } else {
                drawer.DrawPoint(vi.point, 0.03, dr::kColorDeepSkyBlue);
            }
            drawer.DrawPoint(vi.point, 0.02, vi.vertex_flag == -1 ? dr::kColorLightSkyBlue : dr::kColorPink);
        }
        for (int i_prev = static_cast<int>(vis_reg.vertices.size()) - 1, i = 0; i < vis_reg.vertices.size(); i_prev = i++) {
            const auto &vi_prev = vis_reg.vertices[i_prev];
            const auto &vi = vis_reg.vertices[i];
            std::string text = std::to_string(i);
            if (vi.vertex_flag >= 0) {
                text += "-" + std::to_string(vi.vertex_flag);
            }
            drawer.DrawText(text, vi.point, 0.03, dr::kColorNavy);
        }
    }
}

void DrawDetailedVisibilityPolygonWithMap(
    const tve::drawing::MapDrawer &drawer,
    const tvc::TriVis &vis,
    std::optional<tvg::RadialVisibilityRegion> vis_reg = std::nullopt
) {
    drawer.DrawMap();
    drawer.DrawPlane(dr::kColorBlack);
    drawer.DrawBorders(dr::kColorWhite);
    drawer.DrawHoles(dr::kColorDimGray);
    if (vis_reg) {
        drawer.DrawPolygon(tvc::TriVis::ConvertToPolygon(*vis_reg), dr::kColorLightYellow);
    }
    drawer.DrawPolygons(vis.triangles(), 0.01, dr::kColorYellow);
    // for (int i = 0; i < vis.map().holes().size(); ++i) {
    //     drawer.DrawText(std::to_string(i), vis.map().holes()[i].front(), 1.0, dr::kColorDeepPink);
    // }
    for (int i = 0; i < vis.mesh().nodes.size(); ++i) {
        drawer.DrawText(std::to_string(i), vis.mesh().point(i), 0.03, dr::kColorNavy);
    }
    for (int i = 0; i < vis.mesh().edges.size(); ++i) {
        drawer.DrawText(std::to_string(i), (vis.mesh().point(vis.mesh().edges[i].nodes[0]) + vis.mesh().point(vis.mesh().edges[i].nodes[1])) / 2.0, 0.03, dr::kColorMaroon);
    }
    for (int i = 0; i < vis.mesh().triangles.size(); ++i) {
        const auto &tri = vis.mesh().triangles[i];
        drawer.DrawText(std::to_string(i), (vis.mesh().point(tri.nodes[0]) + vis.mesh().point(tri.nodes[1]) + vis.mesh().point(tri.nodes[2])) / 3.0, 0.03, dr::kColorDeepPink);
    }
}

void DrawMap(
    const tve::drawing::MapDrawer &drawer,
    const tvc::TriVis &vis
) {
    drawer.DrawMap();
    drawer.DrawPlane(dr::kColorBlack);
    drawer.DrawBorders(dr::kColorWhite);
    drawer.DrawHoles(dr::kColorDimGray);
    drawer.DrawPolygons(vis.triangles(), 0.01, dr::kColorYellow);
    // for (int i = 0; i < vis.map().holes().size(); ++i) {
    //     drawer.DrawText(std::to_string(i), vis.map().holes()[i].front(), 1.0, dr::kColorDeepPink);
    // }
    for (int i = 0; i < vis.mesh().nodes.size(); ++i) {
        drawer.DrawText(std::to_string(i), vis.mesh().point(i), 0.03, dr::kColorNavy);
    }
    for (int i = 0; i < vis.mesh().edges.size(); ++i) {
        drawer.DrawText(std::to_string(i), (vis.mesh().point(vis.mesh().edges[i].nodes[0]) + vis.mesh().point(vis.mesh().edges[i].nodes[1])) / 2.0, 0.03, dr::kColorMaroon);
    }
    for (int i = 0; i < vis.mesh().triangles.size(); ++i) {
        const auto &tri = vis.mesh().triangles[i];
        drawer.DrawText(std::to_string(i), (vis.mesh().point(tri.nodes[0]) + vis.mesh().point(tri.nodes[1]) + vis.mesh().point(tri.nodes[2])) / 3.0, 0.03, dr::kColorDeepPink);
    }
}

void TestBucketing(
    const tvc::TriVis &vis,
    int random_seed = 42,
    int n_samples_per_edge = 1000000,
    int n_random_samples_per_triangle = 1000000
) {
    std::mt19937 rng(random_seed);
    LOGF_INF("Testing node points ...");
    for (int node_id = 0; node_id < vis.mesh().nodes.size(); ++node_id) {
        LOGF_INF("Node: " << node_id << ".");
        const tvg::FPoint &node_p = vis.mesh().point(node_id);
        std::optional<int> found_tri_id = vis.bucketing().FindTriangle(node_p, vis.triangles());
        if (!found_tri_id) {
            LOGF_ERR("Node point " << node_p << " (node:" << node_id << ") was not found in any of the triangles!");
            continue;
        }
        bool ok = false;
        for (int tri_id: vis.mesh().nodes[node_id].triangles) {
            if (found_tri_id == tri_id) {
                ok = true;
                break;
            }
        }
        if (!ok) {
            LOGF_ERR("Node point " << node_p << " (node:" << node_id << ") was found incorrectly in triangle " << *found_tri_id << ".");
            continue;
        }
    }
    LOGF_INF("Testing edge points ...");
    for (int edge_id = 0; edge_id < vis.mesh().edges.size(); ++edge_id) {
        LOGF_INF("Edge: " << edge_id << ".");
        const auto &edge = vis.mesh().edges[edge_id];
        const tvg::FPoint &p0 = vis.mesh().point(edge.nodes[0]);
        const tvg::FPoint &p1 = vis.mesh().point(edge.nodes[1]);
        const tvg::FPoints edge_samples = tvg::SampleSegment2D(p0, p1, p0.DistanceTo(p1) / (n_samples_per_edge + 1), false, false);
        for (const auto &edge_p: edge_samples) {
            std::optional<int> found_tri_id = vis.bucketing().FindTriangle(edge_p, vis.triangles());
            if (!found_tri_id) {
                LOGF_ERR("Edge point " << edge_p << " (edge:" << edge_id << "," << edge.is_obstacle() << ") was not found in any of the triangles!");
                continue;
            }
            if ((edge.is_obstacle() && found_tri_id != edge.triangles[0]) || (!edge.is_obstacle() && found_tri_id != edge.triangles[0] && found_tri_id != edge.triangles[1])) {
                LOGF_ERR("Edge point " << edge_p << " (edge:" << edge_id << "," << edge.is_obstacle() << ") was found incorrectly in triangle " << *found_tri_id << ".");
                continue;
            }
        }
    }
    LOGF_INF("Testing random triangle points ...");
    for (int tri_id = 0; tri_id < vis.triangles().size(); ++tri_id) {
        LOGF_INF("Triangle: " << tri_id << ".");
        const auto &tri = vis.triangles()[tri_id];
        for (int i = 0; i < n_random_samples_per_triangle; ++i) {
            tvg::FPoint random_p = tv::map_coverage::UniformRandomPointInTriangle(tri, rng);
            if (tvg::PointTriangleRelation(random_p, tri[0], tri[1], tri[2]) != '1') {
                --i;
                continue;
            }
            std::optional<int> found_tri_id = vis.bucketing().FindTriangle(random_p, vis.triangles());
            if (!found_tri_id) {
                LOGF_ERR("Random point " << random_p << " (triangle:" << tri_id << ") was not found in any of the triangles!");
                continue;
            }
            if (found_tri_id != tri_id) {
                LOGF_ERR("Random point " << random_p << " (triangle:" << tri_id << ") was found incorrectly in triangle " << *found_tri_id << ".");
                continue;
            }
        }
    }
}

void GenerateRandomRadialVisibilityRegions() {
    std::string out_dir = "poly-circles-coverage";
    int random_seed = 42;
    int n_query_points_per_map = 1000;
    std::vector<std::pair<std::string, double>> map_names_and_scales = {
        {"empty", -1.0},
        {"complex2", -1.0},
        {"jari-huge", -1.0},
        {"potholes", -1.0},
        {"warehouse2", -1.0},
        {"jf-ta2", 0.01},
        {"jf-pb2", 0.01}
    };
    int total_query_count = 0;
    double total_time = 0.0;
    std::vector<double> radii = {15.0, 10.0, 5.0, 3.0, 2.0, 1.0};
    for (const auto &pair: map_names_and_scales) {
        const std::string &map_name = pair.first;
        double map_scale = pair.second;
        std::string map_full_path = std::string(INPUT_MAPS_DIR) + "/" + map_name + ".txt";
        tvc::TriVis vis;
        {   // Load map from file and move it to TriVis (without copying).
            tvg::PolyMap map;
            std::string load_msg = tve::data_loading::LoadPolyMapSafely(map_full_path, map, map_scale);
            if (load_msg != "ok") {
                LOGF_ERR("Error while loading map. " << load_msg);
                continue;
            }
            map.RemoveDuplicatePoints();
            map.RemoveCollinearPoints();
            vis.SetMap(std::move(map));
            // cannot use map anymore
        }
        vis.TriangulateMapConstrainedDelaunay();
        vis.FillBucketing();
        vis.OptimizeBuckets();

        auto drawer = tve::drawing::MakeMapDrawer(vis.map());

        auto map_points = vis.map().ToPoints();
        for (int i = 0; i < map_points.size(); ++i) {
            if (vis.mesh().nodes[i].point != map_points[i]) {
                std::cout << "err\n";
            }
        }

        // Save map and mesh.
        std::string map_out_dir = out_dir + "/" + map_name;
        fs::create_directories(map_out_dir);
        std::ofstream ofs;
        // Map to file.
        ofs.open(map_out_dir + "/map.txt");
        ofs.precision(std::numeric_limits<double>::max_digits10);
        ofs << std::fixed;
        ofs << "[NAME]\n";
        ofs << map_name << "\n";
        ofs << "\n[LIMITS]\n";
        ofs << vis.map().limits().x_min << " " << vis.map().limits().x_max << " " << vis.map().limits().y_min << " " << vis.map().limits().y_max << "\n";
        ofs << "\n[POINTS]\n";
        ofs << map_points.size() << "\n";
        for (int i = 0; i < map_points.size(); ++i) {
            const auto &p = map_points[i];
            ofs << i << " " << p.x << " " << p.y << "\n";
        }
        ofs << "\n[BORDER]\n";
        ofs << vis.map().border().size() << "\n";
        int count = 0;
        for (int i = 0; i < vis.map().border().size(); ++i) {
            ofs << i << " " << count++ << "\n";
        }
        ofs << "\n[NUM_HOLES]\n";
        ofs << vis.map().holes().size() << "\n";
        for (int h = 0; h < vis.map().holes().size(); ++h) {
            ofs << "\n[HOLE]\n";
            ofs << h << " " << vis.map().holes()[h].size() << "\n";
            for (int i = 0; i < vis.map().holes()[h].size(); ++i) {
                ofs << i << " " << count++ << "\n";
            }
        }
        ofs.close();

        // Mesh to file.
        ofs.open(map_out_dir + "/mesh.txt");
        ofs << "[NODES]\n";
        ofs << vis.mesh().nodes.size() << "\n";
        for (int i = 0; i < vis.mesh().nodes.size(); ++i) {
            const auto &node = vis.mesh().nodes[i];
            ofs << i << " " << i;
            ofs << " " << node.edges.size();
            for (int edge_id: node.edges) {
                ofs << " " << edge_id;
            }
            ofs << " " << node.triangles.size();
            for (int tri_id: node.triangles) {
                ofs << " " << tri_id;
            }
            ofs << "\n";
        }
        ofs << "\n[EDGES]\n";
        ofs << vis.mesh().edges.size() << "\n";
        for (int i = 0; i < vis.mesh().edges.size(); ++i) {
            const auto &edge = vis.mesh().edges[i];
            ofs << i << " " << edge.nodes[0] << " " << edge.nodes[1];
            ofs << " " << edge.triangles.size();
            for (int tri_id: edge.triangles) {
                ofs << " " << tri_id;
            }
            for (int opp_id: edge.opposites) {
                ofs << " " << opp_id;
            }
            ofs << "\n";
        }
        ofs << "\n[TRIANGLES]\n";
        ofs << vis.mesh().triangles.size() << "\n";
        for (int i = 0; i < vis.mesh().triangles.size(); ++i) {
            const auto &tri = vis.mesh().triangles[i];
            ofs << i << " " << tri.nodes[0] << " " << tri.nodes[1] << " " << tri.nodes[2] << " " << tri.edges[0] << " " << tri.edges[1] << " " << tri.edges[2] << "\n";
        }
        ofs.close();

        // Go over all radii.
        for (int i = 0; i < radii.size(); ++i) {
            double vis_radius = radii[i];

            std::cout << map_name << "-" << vis_radius << "\n";

            std::string r_map_out_dir = map_out_dir + "/vis-radius-" + std::to_string(radii[i]);
            fs::create_directories(r_map_out_dir);

            const auto &lim = vis.map().limits();
            ClipperLib::Paths clip_map = tv::map_coverage::ToClipper(vis.map());
            ClipperLib::Paths uncovered_region = clip_map;
            std::vector<tv::map_coverage::VisRegWithPolygonApprox> coverage;
            tvc::utils::SimpleClock clock;

            int dual_samp_iter = 100;
            int stop_max_iter = 1000000;
            double stop_uncoverage_ratio = 1e-4;
            double total_area = tv::map_coverage::ClipperArea(clip_map);
            double eps_collinear = 1e-6;
            double eps_min_edge_len = 1e-6;
            double eps_max_sample_angle = M_PI / 16.0;
            int rng_seed = 42;
            std::mt19937 rng(rng_seed);

            tv::map_coverage::CoverWithDualSamplingMCCS(
                vis, vis_radius / 2.0, dual_samp_iter, stop_max_iter, stop_uncoverage_ratio, total_area, eps_collinear, eps_min_edge_len, eps_max_sample_angle, coverage, uncovered_region, rng
            );

            std::cout << "Coverage size after dual-sampling: " << coverage.size() << "\n";

            double uncovered_area = tv::map_coverage::ClipperArea(uncovered_region);
            tv::map_coverage::FilterCoverage(vis_radius / 2.0, uncovered_area / total_area, stop_uncoverage_ratio, total_area, coverage);
            uncovered_region = clip_map;
            tv::map_coverage::ClipOff(coverage, uncovered_region);
            uncovered_area = tv::map_coverage::ClipperArea(uncovered_region);

            std::cout << "Coverage size after filtering: " << coverage.size() << ".\n";
            std::cout << "Percentage coverage: " << 100.0 * (1.0 - uncovered_area / total_area) << " %.\n";
            std::cout << "Time total: " << clock.TimeInSeconds() << " s.\n";
            total_time += clock.TimeInSeconds();

            auto colors = dr::RandomColors(static_cast<int>(coverage.size()));
            for (auto &c: colors) c = dr::CairoGeomDrawer::SaturateColor(c);

            drawer.OpenPDF(r_map_out_dir + "/coverage.pdf");
            drawer.DrawMap();
            for (int j = 0; j < coverage.size(); ++j) {
                auto reg = tv::map_coverage::SampleArcEdges(coverage[j].orig, M_PI / 180);
                auto polygon = tvc::TriVis::ConvertToPolygon(reg);
                drawer.DrawPolygon(polygon, colors[j], 0.5);
                drawer.DrawPolygon(polygon, 0.05, colors[j], 1.0);
            }
            drawer.Close();

            // Go over all query points.
            for (int j = 0; j < coverage.size(); ++j) {

                std::cout << map_name << "-" << vis_radius << "-" << j + 1 << "-" << coverage.size() << "\n";

                ++total_query_count;

                const auto &vis_reg = coverage[j].orig;

                ofs.open(r_map_out_dir + "/vis-region-" + std::to_string(j + 1) + ".txt");
                ofs.precision(std::numeric_limits<double>::max_digits10);
                ofs << std::fixed;
                ofs << "[SEED]\n";
                ofs << vis_reg.seed.x << " " << vis_reg.seed.y << " " << vis_reg.seed_id << "\n";
                ofs << "\n[RADIUS]\n";
                ofs << vis_reg.radius << "\n";
                ofs << "\n[VERTICES]\n";
                ofs << vis_reg.vertices.size() << "\n";
                for (int k = 0; k < vis_reg.vertices.size(); ++k) {
                    const auto &v = vis_reg.vertices[k];
                    ofs << k << " " << v.point.x << " " << v.point.y << " " << v.edge_flag << " " << v.vertex_flag << "\n";
                }
                ofs.close();

                // draw
                // drawer.OpenImage();
                drawer.OpenPDF(r_map_out_dir + "/vis-region-" + std::to_string(j + 1) + ".pdf");
                DrawDetailedVisibilityPolygonWithMap(drawer, vis);
                DrawDetailedRadialVisibilityRegion(drawer, vis_reg);
                // drawer.SaveToPng(r_map_out_dir + "/vis-region-" + std::to_string(j + 1) + ".png");
                drawer.Close();
            }

        }

    }

    std::cout << "Mean time: " << total_time / total_query_count << " s.\n";

}

bool LoadMapNewFormat(
    const std::string &file,
    tvg::FPolygons &polygons
) noexcept(false) {
    std::ifstream ifs(file.c_str());
    if (ifs.fail()) {
        return false;
    }
    std::string token;
    tvg::FPoints points;
    tvg::FPolygon border;
    tvg::FPolygons holes;
    while (!ifs.eof()) {
        ifs >> token;
        if (token == "[NAME]") {
            // can be ignored
            continue;
        }
        if (token == "[LIMITS]") {
            // can be ignored
            continue;
        }
        if (token == "[POINTS]") {
            int n_points;
            ifs >> n_points;
            points.reserve(n_points);
            int id;
            double x, y;
            for (int i = 0; i < n_points; ++i) {
                ifs >> id;
                ifs >> x;
                ifs >> y;
                points.emplace_back(x, y);
            }
            continue;
        }
        if (token == "[BORDER]") {
            int n_border;
            ifs >> n_border;
            border.reserve(n_border);
            int id;
            int point_id;
            for (int i = 0; i < n_border; ++i) {
                ifs >> id;
                ifs >> point_id;
                border.push_back(points[point_id]);
            }
            continue;
        }
        if (token == "[NUM_HOLES]") {
            int n_holes;
            ifs >> n_holes;
            holes.resize(n_holes);
            continue;
        }
        if (token == "[HOLE]") {
            int hole_id, n_hole;
            ifs >> hole_id;
            ifs >> n_hole;
            auto &hole = holes[hole_id];
            hole.reserve(n_hole);
            int id;
            int point_id;
            for (int i = 0; i < n_hole; ++i) {
                ifs >> id;
                ifs >> point_id;
                hole.push_back(points[point_id]);
            }
            continue;
        }
    }
    // Save to output.
    polygons.reserve(1 + holes.size());
    polygons.push_back(border);
    for (const auto &hole: holes) {
        polygons.push_back(hole);
    }
    return true;
}

void GenerateMapCoverage(
    const tvc::TriVis &vis,
    std::optional<double> vis_radius
) {
    if (vis_radius) {
        *vis_radius /= 2.0;
    }

    auto drawer = dr::MakeMapDrawer(vis.map());

    const auto &lim = vis.map().limits();
    ClipperLib::Paths clip_map = tv::map_coverage::ToClipper(vis.map());
    ClipperLib::Paths uncovered_region = clip_map;
    std::vector<tv::map_coverage::VisRegWithPolygonApprox> coverage;
    tvc::utils::SimpleClock clock;

    int dual_samp_iter = 100;
    int stop_max_iter = 10000;
    double stop_uncoverage_ratio = 1e-4;
    double total_area = tv::map_coverage::ClipperArea(clip_map);
    double eps_collinear = 1e-6;
    double eps_min_edge_len = 1e-6;
    double eps_max_sample_angle = M_PI / 16.0;
    int rng_seed = 42;
    std::mt19937 rng(rng_seed);

    tv::map_coverage::CoverWithDualSamplingMCCS(
        vis, vis_radius, dual_samp_iter, stop_max_iter, stop_uncoverage_ratio, total_area, eps_collinear, eps_min_edge_len, eps_max_sample_angle, coverage, uncovered_region, rng
    );

    std::cout << "Coverage size after dual-sampling: " << coverage.size() << "\n";

    double uncovered_area = tv::map_coverage::ClipperArea(uncovered_region);
    tv::map_coverage::FilterCoverage(vis_radius, uncovered_area / total_area, stop_uncoverage_ratio, total_area, coverage);
    uncovered_region = clip_map;
    tv::map_coverage::ClipOff(coverage, uncovered_region);
    uncovered_area = tv::map_coverage::ClipperArea(uncovered_region);

    std::cout << "Coverage size after filtering: " << coverage.size() << "\n";
    std::cout << "Percentage coverage: " << 100.0 * (1.0 - uncovered_area / total_area) << " s.\n";
    std::cout << "Time total: " << clock.TimeInSeconds() << "\n";


    auto colors = dr::RandomColors(static_cast<int>(coverage.size()));
    for (auto &c: colors) c = dr::CairoGeomDrawer::SaturateColor(c);

    drawer.OpenPDF("coverage.pdf");
    drawer.DrawMap();
    for (int i = 0; i < coverage.size(); ++i) {
        auto reg = tv::map_coverage::SampleArcEdges(coverage[i].orig, M_PI / 180);
        auto polygon = tvc::TriVis::ConvertToPolygon(reg);
        drawer.DrawPolygon(polygon, colors[i], 0.5);
        drawer.DrawPolygon(polygon, 0.05, colors[i], 1.0);
    }
    drawer.Close();

}

/**
 *
 *  ##########################################
 *  ## THIS IS THE MAIN BODY OF THE PROGRAM ##
 *  ##########################################
 *
 * @param pov ~ program option variables
 * @return exit code
 */
int MainBody(const ProgramOptionVariables &pov) {

    // GenerateRandomRadialVisibilityRegions();
    // return EXIT_SUCCESS;

    tvc::TriVis vis;
    {   // Load map from file and move it to TriVis (without copying).
        tvg::PolyMap map;
        std::string load_msg = tve::data_loading::LoadPolyMapSafely(pov.input_map_full_path, map, pov.map_scale);
        if (load_msg != "ok") {
            LOGF_FTL("Error while loading map. " << load_msg);
            return EXIT_FAILURE;
        }
        map.RemoveDuplicatePoints();
        map.RemoveCollinearPoints();
        vis.SetMap(std::move(map));
        // cannot use map anymore
    }
    vis.TriangulateMapConstrainedDelaunay();
    vis.FillBucketing();
    vis.OptimizeBuckets();

    if (pov.generate_map_coverage) {
        std::optional<double> vis_radius = std::nullopt;
        if (pov.vis_radius > 0.0) {
            vis_radius = pov.vis_radius;
        }
        GenerateMapCoverage(vis, vis_radius);
        return EXIT_SUCCESS;
    }

    if (pov.test_bucketing) {
        TestBucketing(vis);
        return EXIT_SUCCESS;
    }

    auto drawer = tve::drawing::MakeMapDrawer(vis.map());

    tvg::FPoint q;
    // q = tvg::MakePoint(10.5, 5.5);
    q = vis.mesh().point(0);
    double vis_radius = 100.0;

    int num_expansions = 0;
    tvg::AbstractVisibilityRegion abs_vis_reg;
    bool vis_reg_ok = vis.ComputeVisibilityRegion(q, vis_radius, abs_vis_reg, num_expansions);
    if (!vis_reg_ok || abs_vis_reg.segments.empty()) {
        LOGF_ERR("The visibility region from point " << q << " could not be computed!");
    }
    auto pre_vis_reg = vis.ConvertToVisibilityRegion(abs_vis_reg);
    tvg::RemoveAntennas(pre_vis_reg);
    auto vis_reg = tvc::TriVis::ConvertToRadialVisibilityRegion(vis_radius, pre_vis_reg);
    tvg::RemoveShortEdges(1e-6, vis_reg);

    tvc::utils::SimpleClock clock;
    auto macs = tv::map_coverage::MaxAreaConvexSubset(vis_reg, 1e-6, 1e-6);
    std::cout << "Time: " << clock.TimeInSeconds() << " s.\n";

    std::vector<int> reflex_vertices = tv::map_coverage::ReflexVertices(vis_reg, 1e-6);
    auto kernel = tv::map_coverage::Kernel(vis_reg, reflex_vertices, 1e-6, 1e-6); // todo: kernel is wrong for (12.0, 10.0), rad=5.0 on potholes
    // tvg::RadialVisibilityRegion kernel;
//
    // if (reflex_vertices.empty()) {
    //     kernel = vis_reg;
    // } else {
    //     const int max_id = static_cast<int>(vis_reg.vertices.size()) - 1;
    //     const int max_reflex_id = static_cast<int>(reflex_vertices.size()) - 1;
    //     kernel = vis_reg;
    //     for (int i = 0; i < reflex_vertices.size(); ++i) {
    //         int id_curr = reflex_vertices[i];
    //         int id_prev = (id_curr == 0) ? max_id : id_curr - 1;
    //         // Clip off half-plane given by prev, and curr.
    //         kernel = IntersectionWithLeftHalfPlane(kernel, vis_reg.vertices[id_prev].point, vis_reg.vertices[id_curr].point);
//
    //         drawer.OpenPDF("kernel" + std::to_string(i) + "a.pdf");
    //         DrawVisibilityPolygonWithMap(drawer, vis, pre_vis_reg);
    //         //DrawRadialVisibilityPolygon(drawer, vis_reg);
    //         DrawRadialVisibilityPolygon(drawer, kernel);
    //         drawer.Close();
//
    //         if (kernel.vertices.empty()) {
    //             // Return empty kernel.
    //             kernel.radius = 0.0;
    //             break;
    //         }
    //         int id_next = (id_curr == max_id) ? 0 : id_curr + 1;
    //         int id_reflex_next = (i == max_reflex_id) ? reflex_vertices[0] : reflex_vertices[i + 1];
    //         if (id_next != id_reflex_next) {
    //             // Only if the next is not reflex: clip off half-plane given by curr, and next.
    //             kernel = IntersectionWithLeftHalfPlane(kernel, vis_reg.vertices[id_curr].point, vis_reg.vertices[id_next].point);
//
    //             drawer.OpenPDF("kernel" + std::to_string(i) + "b.pdf");
    //             DrawVisibilityPolygonWithMap(drawer, vis, pre_vis_reg);
    //             //DrawRadialVisibilityPolygon(drawer, vis_reg);
    //             DrawRadialVisibilityPolygon(drawer, kernel);
    //             drawer.Close();
//
    //             if (kernel.vertices.empty()) {
    //                 // Return empty kernel.
    //                 kernel.radius = 0.0;
    //                 break;
    //             }
    //         }
    //     }
    // }

    std::vector<int> aux;
    tv::map_coverage::OrderReflexVertices(vis_reg, kernel, reflex_vertices, aux, 1e-6);

    drawer.OpenPDF("poly.pdf");
    DrawDetailedVisibilityPolygonWithMap(drawer, vis, pre_vis_reg);
    DrawDetailedRadialVisibilityRegion(drawer, vis_reg);
    drawer.Close();

    drawer.OpenPDF("macs.pdf");
    DrawDetailedVisibilityPolygonWithMap(drawer, vis, pre_vis_reg);
    // DrawRadialVisibilityPolygon(drawer, vis_reg);
    DrawDetailedRadialVisibilityRegion(drawer, macs);
    drawer.Close();

    drawer.OpenPDF("kernel.pdf");
    DrawDetailedVisibilityPolygonWithMap(drawer, vis, pre_vis_reg);
    DrawDetailedRadialVisibilityRegion(drawer, vis_reg);
    DrawDetailedRadialVisibilityRegion(drawer, kernel);
    for (int i = 0; i < reflex_vertices.size(); ++i) {
        const auto &p = vis_reg.vertices[reflex_vertices[i]].point;
        drawer.DrawPoint(p, 0.10, dr::kColorDeepPink);
        drawer.DrawPoint(p, 0.08, dr::kColorWhite);
        drawer.DrawPoint(p, 0.06, dr::kColorDeepPink);
        drawer.DrawPoint(p, 0.04, dr::kColorWhite);
        drawer.DrawPoint(p, 0.02, dr::kColorDeepPink);
        drawer.DrawText(std::to_string(i), p, 0.10);
    }
    for (int i = 0; i < kernel.vertices.size(); ++i) {
        const auto &v_prev = kernel.vertices[i == 0 ? kernel.vertices.size() - 1 : i - 1];
        const auto &v = kernel.vertices[i];
        const auto &v_next = kernel.vertices[i + 1 == kernel.vertices.size() ? 0 : i + 1];
        tvg::FPoint p, a, b;
        if (v.edge_flag == -2) {
            // Edge (v_prev, v) is an arc.
            tvg::FPoint u = kernel.seed - v.point;
            a = {u.y, -u.x};
        } else {
            // Edge (v_prev, v) is a segment.
            a = v.point - v_prev.point;
        }
        if (v_next.edge_flag == -2) {
            // Edge (v, v_next) is an arc.
            tvg::FPoint u = kernel.seed - v.point;
            b = {-u.y, u.x};
        } else {
            // Edge (v, v_next) is a segment.
            b = v.point - v_next.point;
        }
        a = a / a.Norm();
        b = b / b.Norm();
        p = (a + b) / 2.0;
        if (p.Norm() < 1e-6) {
            p = {a.y, -a.x};
        } else {
            p = p / p.Norm();
        }
        drawer.DrawLine(v.point, v.point + p, 0.05, dr::kColorBlack, 0.2);
    }
    // drawer.DrawLine({8.54, 9.956}, {9.54, 8.956}, 0.04, dr::kColorBlack);
    // drawer.DrawPoint({8.54, 9.956}, 0.05, dr::kColorBlack);
    drawer.Close();

    return EXIT_SUCCESS;
}

int main(
    int argc,
    const char *const *argv
) {
    ProgramOptionVariables pov;
    char c = ParseProgramOptions(argc, argv, pov);
    if (c == 'h') {
        return EXIT_SUCCESS;
    } else if (c == 'e') {
        return EXIT_FAILURE;
    } else {
        return MainBody(pov);
    }
}