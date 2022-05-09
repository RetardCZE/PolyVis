#include "fadeutils/polymap.h"

namespace fadeutils
{

void fail(const string& message)
{
    cerr << message << endl;
    exit(1);
}

void read_polys(istream& infile, vector<Polygon> &polygons)
{
    string header;
    int version;

    if (!(infile >> header))
    {
        fail("Error reading header");
    }
    if (header != "poly")
    {
        cerr << "Got header '" << header << "'" << endl;
        fail("Invalid header (expecting 'poly')");
    }

    if (!(infile >> version))
    {
        fail("Error getting version number");
    }
    if (version != 1)
    {
        cerr << "Got file with version " << version << endl;
        fail("Invalid version (expecting 1)");
    }

    int N;
    if (!(infile >> N))
    {
        fail("Error getting number of polys");
    }
    if (N < 1)
    {
        cerr << "Got " << N << "polys" << endl;
        fail("Invalid number of polys");
    }

    for (int i = 0; i < N; i++)
    {
        int M;
        if (!(infile >> M))
        {
            fail("Error parsing map (can't get number of points of poly)");
        }
        if (M < 3)
        {
            cerr << "Got " << N << "points" << endl;
            fail("Invalid number of points in poly");
        }
        Polygon cur_poly;
        for (int j = 0; j < M; j++)
        {
            double x, y;
            if (!(infile >> x >> y))
            {
                fail("Error parsing map (can't get point)");
            }
            cur_poly.push_back(Point2(x, y));
        }
        polygons.push_back(cur_poly);
    }

    int temp;
    if (infile >> temp)
    {
        fail("Error parsing map (read too much)");
    }
}

void load_map(const std::string &filename, vector<Polygon> &polygons) {
        std::ifstream ifs(filename.c_str());
        if (ifs.fail()) {
            std::cout << "File " << filename << " cannot be opened or found." << std::endl;
            exit(EXIT_FAILURE);
        }

        Polygon cur_poly;
        /// Load the map:
        std::string token;
        bool isBorder = false;
        double scale = 1.0;
        while (!ifs.eof()) {
            ifs >> token;
            if (token == "[SCALE]") {
                ifs >> scale;
            } else if (token == "[BORDER]") {
                isBorder = true;
            } else if (token == "[OBSTACLE]") {
                polygons.push_back(cur_poly);
                cur_poly.clear();
            } else {
                if (!ifs.eof()) {
                    double x, y;
                    x = stod(token) * scale;
                    ifs >> y;
                    y *= scale;
                    cur_poly.push_back(Point2(x, y));
                }
            }
        }
        /// Last obstacle:
        polygons.push_back(cur_poly);
    }

vector<ConstraintGraph2*> *create_constraint_graphs(const vector<Polygon> &polygons, Fade_2D &dt)
{
    vector<ConstraintGraph2*> *constraint_graphs = new vector<ConstraintGraph2*>;
    for (auto poly : polygons)
    {
        vector<Segment2> segments;
        for (int i = 0; i < ((int)poly.size()) - 1; i++)
        {
            const Point2& p0 = poly[i];
            const Point2& p1 = poly[i+1];
            segments.push_back(Segment2(p0, p1));
        }
        segments.push_back(Segment2(poly.back(), poly.front()));
        ConstraintGraph2 *cg = dt.createConstraint(segments, CIS_CONSTRAINED_DELAUNAY);
        constraint_graphs->push_back(cg);
    }
    return constraint_graphs;
}

Zone2 *create_traversable_zone(const vector<Polygon> &polygons, Fade_2D &dt)
{
    vector<ConstraintGraph2*> *cgs = create_constraint_graphs(polygons, dt);
    dt.applyConstraintsAndZones();

    #ifndef NDEBUG
    for (auto x : *cgs)
    {
        assert(x->isPolygon());
    }
    #endif

    vector<Zone2*> zones;
    zones.push_back(dt.createZone(nullptr, ZL_GLOBAL));
    for (auto x : *cgs)
    {
        zones.push_back(dt.createZone(x, ZL_INSIDE));
    }
    zones.push_back(dt.createZone(nullptr, ZL_GLOBAL));
    assert(!zones.empty());

    Zone2* traversable = zones.front();
    for (int i = 1; i < ((int)zones.size()); i++)
    {
        traversable = zoneSymmetricDifference(traversable, zones[i]);
    }
    // Free pointer to vector
    delete cgs;
    return traversable;
}

Zone2* create_traversable_zone_istream(istream& infile, Fade_2D &dt, vector<Polygon> &obstacles)
{
    vector<Polygon> polygons;
    read_polys(infile, polygons);
    if ((int)polygons.size() > 1) obstacles.insert(obstacles.end(), polygons.begin() + 1, polygons.end());
    return create_traversable_zone(polygons,dt);
}

Zone2 *create_traversable_zone_filename(const string &filename, Fade_2D &dt, vector<Polygon> &obstacles)
{
    vector<Polygon> polygons;
    load_map(filename, polygons);
    if ((int)polygons.size() > 1) obstacles.insert(obstacles.end(), polygons.begin() + 1, polygons.end());
    return create_traversable_zone(polygons,dt);
}

}
