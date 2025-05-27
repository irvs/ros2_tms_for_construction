#include <memory>
#include <iostream>
#include <stdint.h>
#include <fstream>
#include <limits>
#include <vector>

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/Options.hpp>

#include <png++/png.hpp>

using namespace std;

#define DEFAULT_WIDTH  2048
#define DEFAULT_HEIGHT 2048

std::map<std::string, std::string> parseArgs(int argc, char* argv[]) {
	std::map<std::string, std::string> args;

	for (int i = 1; i < argc; ++i) {
		std::string arg = argv[i];
		if (arg[0] == '-') {
			std::string key = arg.substr(1);
			if ((i + 1) < argc && argv[i + 1][0] != '-') {
				std::cout << key << ": " << argv[i + 1][0] << std::endl;
				args[key] = argv[++i];
			} else {
				args[key] = "true";
			}
		}
	}

	return args;
}

struct Point {
	double x, y, z;
//	int classification;
//	uint8_t intensity;
	uint8_t r, g, b;

	double distance2(const Point &p) const {
		double dx = p.x - x;
		double dy = p.y - y;
		double dz = p.z - z;
		return dx*dx + dy*dy + dz*dz;
	}

	double distance(const Point &p) const {
		return sqrt(distance2(p));
	}
};

class LasToHeightmap {
	pdal::LasReader las_reader;

	double offsetX;
	double offsetY;
	double offsetZ;
	double scaleX;
	double scaleY;

	int output_width;
	int output_height;
	/////
	double minX;
	double maxX;
	double minY;
	double maxY;
	double minZ;
	double maxZ;
	///


	void addPoint(double x, double y, double z, uint16_t r, uint16_t g, uint16_t b){
		x = (x - offsetX) * scaleX;
		y = (y - offsetY) * scaleY;
		z = (z - offsetZ);
		//intensity /= 256;

		/* Skip vegetation, "other", and noise */
		//if (classification == 3 || classification == 4 || classification == 7 || classification == 8)
		//	return;

		if ((int)x >= output_width)
			x = output_width - 1;
		if ((int)y >= output_height)
			y = output_height - 1;
		if (x < 0)
			x = 0;
		if (y < 0)
			y = 0;

		//if (intensity > 255)
		//	intensity = 255;

		//Point point = {x,y,z,classification,(uint8_t)intensity};
		//Point point = {x, y, z, r, g, b};

		//std::cout << "r: " << r << ", g: " << g << ", b: " << b << std::endl;

		// 16ビットから8ビットにスケーリング
		uint8_t r8 = std::min(static_cast<uint16_t>(r / 256), (uint16_t)255);
		uint8_t g8 = std::min(static_cast<uint16_t>(g / 256), (uint16_t)255);
		uint8_t b8 = std::min(static_cast<uint16_t>(b / 256), (uint16_t)255);
	
		Point point = {x, y, z, r8, g8, b8};

		pointsAt(x, y)->push_back(point);
	}

	public:

	// ゲッター関数を追加
    double getMinX() const { return minX; }
    double getMaxX() const { return maxX; }
    double getMinY() const { return minY; }
    double getMaxY() const { return maxY; }
    double getMinZ() const { return minZ; }
    double getMaxZ() const { return maxZ; }

	std::vector<Point> *pointMatrix;

	LasToHeightmap(int width, int height, pdal::Options &las_opts, double minZ_value, double minX_value, double maxX_value, double minY_value, double maxY_value) {
		output_width = width;
		output_height = height;
		las_reader.setOptions(las_opts);

		pointMatrix = new std::vector<Point>[width * height]();

		// コマンドライン引数から渡された minZ を保持
        minZ = minZ_value;
		minX = minX_value;
		maxX = maxX_value;
		minY = minY_value;
		maxY = maxY_value;
	}

	void perform() {
		pdal::PointTable table;
		las_reader.prepare(table);
		pdal::PointViewSet point_view_set = las_reader.execute(table);
		pdal::PointViewPtr point_view = *point_view_set.begin();
		pdal::Dimension::IdList dims = point_view->dims();
		pdal::LasHeader las_header = las_reader.header();

		std::cerr << "output: " << output_width << "x" << output_height << std::endl;

		maxZ = las_header.maxZ();
		//
		std::cerr << "X: " << minX << " to " << maxX << std::endl;
		std::cerr << "Y: " << minY << " to " << maxY << std::endl;
		std::cerr << "Z: " << minZ << " to " << maxZ << std::endl;


		offsetX = minX;//las_header.minX();
		offsetY = maxY;//las_header.maxY();

		//koko
		offsetZ = minZ;

		//scaleX = output_width/1000.0;
		//scaleY = -output_height/1000.0;
		//
		scaleX = output_width / (maxX - minX);//
		//scaleX = output_width / (las_header.maxX() - las_header.minX());//
		scaleY = -output_height / (maxY - minY);//
		//scaleY = -output_height / (las_header.maxY() - las_header.minY());//

		for (pdal::PointId idx = 0; idx < point_view->size(); ++idx) {
			using namespace pdal::Dimension;
			double x = point_view->getFieldAs<double>(Id::X, idx);
			double y = point_view->getFieldAs<double>(Id::Y, idx);
			double z = point_view->getFieldAs<double>(Id::Z, idx);

		//	int classification = point_view->getFieldAs<int>(Id::Classification, idx);
		//	int intensity = point_view->getFieldAs<int>(Id::Intensity, idx);
			uint16_t r = point_view->getFieldAs<uint16_t>(Id::Red, idx);
			uint16_t g = point_view->getFieldAs<uint16_t>(Id::Green, idx);
			uint16_t b = point_view->getFieldAs<uint16_t>(Id::Blue, idx);
			//addPoint(x, y, z, classification, intensity);
		//	std::cout << "r: " << r << ", g: " << g << ", b: " << b << std::endl;
			addPoint(x, y, z, r, g, b);
		}
	}

	std::vector<Point> *pointsAt(int x, int y) {
		if (x < 0 || x >= output_width || y < 0 || y >= output_height) {
			return NULL;
		} else {
			return &pointMatrix[y * output_width + x];
		}
	}

	/* Makes a "fake" point with some heuristics on the points around it */
	Point pointAt(int x, int y, int range=3) {
		Point point = {(double)x + 0.5, (double)y + 0.5, 0, 0, 0};
		std::vector<Point> neighbourPoints;
		neighbourPoints.reserve(2048);

		auto addPoints = [&](std::vector<Point> *v) {
			if (v) {
				std::copy(v->begin(), v->end(), std::back_inserter(neighbourPoints));
			}
		};

		for (int dy = -range; dy <= range; dy++) {
			for (int dx = -range; dx <= range; dx++) {
				addPoints(pointsAt(x+dx, y+dy));
			}
		}

		if(neighbourPoints.empty()) {
			return point;
		} else {
			std::nth_element(
					neighbourPoints.begin(),
					neighbourPoints.begin() + (neighbourPoints.size() / 2),
					neighbourPoints.end(),
					[](const Point &p1, const Point &p2) { return p1.z < p2.z; }
					);

			Point medianPoint = neighbourPoints[neighbourPoints.size() / 2];
			point.z = medianPoint.z;

			auto representativePoint = std::min_element(
					neighbourPoints.begin(), neighbourPoints.end(),
					[&](const Point &p1, const Point &p2) { return point.distance2(p1) < point.distance2(p2); }
					);
			if (representativePoint != neighbourPoints.end()) {
				/* If we can find a point near out median z point, use its intensity */
				//point.intensity = representativePoint->intensity;
				//point.intensity = representativePoint->intensity;
			} else {
				/* Otherwise, take the average */
				cerr << "this should never happen" << endl;
				//point.intensity = 255; /*DEBUG*/
			}

			//point.intensity = medianPoint.intensity;

			return point;
		}
	}

};

void processPointsToGrayImage(png::image<png::gray_pixel_16> &output_image, LasToHeightmap &lasToHeightmap, unsigned int width, unsigned int height) {
	for (unsigned int y = 0; y < height; y++) {
		for (unsigned int x = 0; x < width; x++) {
			Point p = lasToHeightmap.pointAt(x, y);
			double z = p.z;
			if (z < 0)
				z = 0;
			//koko
			unsigned short iz = z * 4096;
			output_image[y][x] = png::gray_pixel_16(iz);
		}
	}
}

void processPointsToRGBImage(png::image<png::rgb_pixel> &output_image, LasToHeightmap &lasToHeightmap, unsigned int width, unsigned int height) {
    for (unsigned int y = 0; y < height; y++) {
        for (unsigned int x = 0; x < width; x++) {
            std::vector<Point> *points = lasToHeightmap.pointsAt(x, y);
            if (points && !points->empty()) {
                Point p = points->front();
                // R, G, Bを画像に反映
                output_image[y][x] = png::rgb_pixel(p.r, p.g, p.b);
            } else {
                // 点がない場所は黒で塗りつぶし
                output_image[y][x] = png::rgb_pixel(0, 0, 0);
            }
        }
    }
}

int main(int argc, char *argv[]) {
	auto args = parseArgs(argc, argv);
	for (const auto& [key, value] : args) {
		std::cout << "-" << key << ": " << value << std::endl;
	}

	if (args.count("help") || args.count("h")) {
		std::cout << "Usage: " << argv[0] << " -i <input file: required> -o <output file: required> -W <width: default 2048> -H <height: default: 2048>" << std::endl;
		return 0;
	}
	
	if (!args.count("i")) {
		std::cout << "-i <input file> option is required" << std::endl;
		return 1;
	}
	std::string input_filename = args["i"];
	
	if (!args.count("o")) {
		std::cout << "-o <output file> option is required" << std::endl;
		return 1;
	}
	std::string output_filename = args["o"];
	std::string output_csv = args["elevation_csv"];
	float minX = std::stod(args["min_x"]);
	float maxX = std::stod(args["max_x"]);
	float minY = std::stod(args["min_y"]);
	float maxY = std::stod(args["max_y"]);
	float minZ = std::stod(args["min_z"]);

	unsigned int width = DEFAULT_WIDTH;
	unsigned int height = DEFAULT_HEIGHT;
	try {
		if (args.count("W")) {
			width = std::stoul(args["W"]);
		}
		if (args.count("H")) {
			height = std::stoul(args["H"]);
		}
	} catch (const std::invalid_argument &e) {
		std::cerr << "Error: Width and height must be valid integers." << std::endl;
		return 1;
	}
	
	bool is_gray = true;
	if (args.count("rgb")) {
		is_gray = false;
	}

	try {
		cout << "Collecting all points..." << endl;
		pdal::Option las_opt("filename", input_filename);
		pdal::Options las_opts;
		las_opts.add(las_opt);
		LasToHeightmap lasToHeightmap(width, height, las_opts, minZ, minX, maxX, minY, maxY);
		lasToHeightmap.perform();

		cout << "Creating heightmap..." << endl;
		if (is_gray) {
			png::image<png::gray_pixel_16> output_image(width, height);
			processPointsToGrayImage(output_image, lasToHeightmap, width, height);
			output_image.write(output_filename);
		} 
		else {
			png::image<png::rgb_pixel> output_image(width, height);
			processPointsToRGBImage(output_image, lasToHeightmap, width, height);
			output_image.write(output_filename);
		}

		////////
		// X, Y, Z の最小値と最大値をファイルに出力
		std::ofstream outFile(output_csv);
		if (outFile.is_open()) {
			outFile << "X_min,X_max,Y_min,Y_max,Z_min,Z_max\n";
			outFile << lasToHeightmap.getMinX() << "," << lasToHeightmap.getMaxX() << ","
					<< lasToHeightmap.getMinY() << "," << lasToHeightmap.getMaxY() << ","
					<< lasToHeightmap.getMinZ() << "," << lasToHeightmap.getMaxZ() << "\n";
			outFile.close();
			std::cout << "Elevation min/max data written to " << output_csv << std::endl;
		} 
		else {
    		std::cerr << "Error writing min/max data to file!" << std::endl;
		}

		//////////////

	} catch (const std::exception &e) {
		std::cerr << "ERROR: " << e.what() << std::endl;
		return 1;
	}

	return 0;
}
