#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sys/stat.h>
#include "CSF.h"
#include "XYZReader.h"

static bool fileExists(const std::string& path) {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0 && buffer.st_size > 0);
}

static std::string getIsoTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm;
#if defined(_WIN32)
    gmtime_s(&now_tm, &now_c);
#else
    gmtime_r(&now_c, &now_tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &now_tm);
    return std::string(buf);
}

static std::string getBasename(const std::string& path) {
    size_t lastSlash = path.find_last_of("/\\");
    if (lastSlash == std::string::npos) return path;
    return path.substr(lastSlash + 1);
}

static std::vector<std::string> splitString(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        if (!token.empty()) tokens.push_back(token);
    }
    return tokens;
}

static void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n\n"
              << "Options:\n"
              << "  --cloud <path>             Path to input point cloud (.txt / .xyz)\n"
              << "  --csv <path>               Output CSV file (default: csf_benchmark_results.csv)\n"
              << "  --resolutions <r1,r2,...>  Cloth resolution list (default: 0.2,0.5,1.0)\n"
              << "  --iterations <i1,i2,...>   Max iterations list (default: 500,1000)\n"
              << "  --rigidness <r1,r2,...>    Rigidness list (default: 3)\n"
              << "  --time_steps <t1,t2,...>   Time step list (default: 0.65)\n"
              << "  --class_threshold <val>    Classification threshold (default: 0.40)\n"
              << "  --slope_smooth <bool>      Enable slope smoothing (default: false)\n"
              << "  --help                     Show this help message\n";
}

int main(int argc, char* argv[]) {
    std::string cloudPath = "";
    std::string csvPath = "csf_benchmark_results.csv";
    std::string resStr = "0.2,0.5,1.0";
    std::string iterStr = "500";
    std::string rigidStr = "3";
    std::string timeStepStr = "0.65";
    double classThreshold = 0.40;
    bool slopeSmooth = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--cloud" && i + 1 < argc) {
            cloudPath = argv[++i];
        } else if (arg == "--csv" && i + 1 < argc) {
            csvPath = argv[++i];
        } else if (arg == "--resolutions" && i + 1 < argc) {
            resStr = argv[++i];
        } else if (arg == "--iterations" && i + 1 < argc) {
            iterStr = argv[++i];
        } else if (arg == "--rigidness" && i + 1 < argc) {
            rigidStr = argv[++i];
        } else if (arg == "--time_steps" && i + 1 < argc) {
            timeStepStr = argv[++i];
        } else if (arg == "--class_threshold" && i + 1 < argc) {
            classThreshold = std::stod(argv[++i]);
        } else if (arg == "--slope_smooth" && i + 1 < argc) {
            std::string val = argv[++i];
            slopeSmooth = (val == "true" || val == "1" || val == "True");
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
    }

    if (cloudPath.empty()) {
        std::cerr << "Error: --cloud <path> is required.\n";
        printUsage(argv[0]);
        return 1;
    }

    std::cout << "Loading point cloud: " << cloudPath << " ..." << std::endl;
    const auto loadStart = std::chrono::steady_clock::now();

    CSF baseCsf;
    baseCsf.readPointsFromFile(cloudPath);
    const size_t totalPoints = baseCsf.size();

    if (totalPoints == 0) {
        std::cerr << "Error: Could not read points or point cloud is empty: " << cloudPath << std::endl;
        return 1;
    }

    const double loadDuration = std::chrono::duration<double>(std::chrono::steady_clock::now() - loadStart).count();
    std::cout << "Loaded " << totalPoints << " points in " << std::fixed << std::setprecision(4)
              << loadDuration << " s." << std::endl;

    std::vector<double> resolutions;
    for (const auto& token : splitString(resStr, ',')) resolutions.push_back(std::stod(token));

    std::vector<int> iterationsList;
    for (const auto& token : splitString(iterStr, ',')) iterationsList.push_back(std::stoi(token));

    std::vector<int> rigidnessList;
    for (const auto& token : splitString(rigidStr, ',')) rigidnessList.push_back(std::stoi(token));

    std::vector<double> timeSteps;
    for (const auto& token : splitString(timeStepStr, ',')) timeSteps.push_back(std::stod(token));

    const std::string cloudName = getBasename(cloudPath);

    std::cout << "\n===================================================================================================\n";
    std::cout << "Starting Comparative Benchmark on: " << cloudName << " (" << totalPoints << " points)\n";
    std::cout << "CSV output: " << csvPath << "\n";
    std::cout << "===================================================================================================\n";
    std::cout << std::left
              << std::setw(8)  << "Resol"
              << std::setw(8)  << "IterMax"
              << std::setw(8)  << "IterDone"
              << std::setw(12) << "Particles"
              << std::setw(14) << "# Chao"
              << std::setw(14) << "# Teto"
              << std::setw(10) << "% Chao"
              << std::setw(12) << "Simul(s)"
              << std::setw(12) << "Total(s)"
              << std::setw(14) << "StopReason"
              << "\n";
    std::cout << "---------------------------------------------------------------------------------------------------\n";

    for (double res : resolutions) {
        for (int maxIter : iterationsList) {
            for (int rigid : rigidnessList) {
                for (double dt : timeSteps) {
                    CSF csf;
                    csf.setPointCloud(baseCsf.getPointCloud());
                    csf.params.cloth_resolution = res;
                    csf.params.interations = maxIter;
                    csf.params.rigidness = rigid;
                    csf.params.time_step = dt;
                    csf.params.class_threshold = classThreshold;
                    csf.params.bSloopSmooth = slopeSmooth;

                    std::vector<int> groundIndexes;
                    std::vector<int> nonGroundIndexes;

                    csf.setPointCloudName(cloudName);
                    csf.do_filtering(groundIndexes, nonGroundIndexes, false);

                    const auto& timing = csf.getLastTiming();
                    const size_t nGround = groundIndexes.size();
                    const size_t nNonGround = nonGroundIndexes.size();
                    const double pctGround = (totalPoints > 0) ? (nGround * 100.0 / totalPoints) : 0.0;
                    const std::string stopReason = timing.stopped_by_convergence ? "convergence" : "iteration_limit";

                    // Console table row
                    std::cout << std::left << std::fixed
                              << std::setw(8)  << std::setprecision(2) << res
                              << std::setw(8)  << maxIter
                              << std::setw(8)  << timing.executed_iterations
                              << std::setw(12) << timing.num_particles
                              << std::setw(14) << nGround
                              << std::setw(14) << nNonGround
                              << std::setw(10) << std::setprecision(2) << pctGround
                              << std::setw(12) << std::setprecision(3) << timing.simulation_s
                              << std::setw(12) << std::setprecision(3) << timing.total_filtering_s
                              << std::setw(14) << stopReason
                              << "\n";

                    // Write line to CSV using CSF library method
                    csf.logRunToCsv(csvPath, cloudName, nGround, nNonGround);
                }
            }
        }
    }

    std::cout << "===================================================================================================\n";
    std::cout << "Benchmark completed. All results appended to: " << csvPath << std::endl;
    return 0;
}
