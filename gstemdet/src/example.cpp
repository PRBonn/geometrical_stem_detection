/*!
  example.cpp
  Demo app for the leaf and stem detection package.

  @author Ferdinand Langer
  @author Leonard Mandtler
*/

#include <boost/filesystem.hpp>
#if defined(DEBUG_MODE) || defined(TIMER)
#include <chrono>
#endif
#include "InputParser.h"
#include "algorithm.h"
#include "stem.h"

using namespace gstemdet;

std::vector<std::string> getFilesInDirectory(const std::string &directory);

bool handleArguments(int argc, char *argv[]);
void printHelp();

int currentID;
int m;
std::string currentFile;
bool all;
bool drawOutput = false;
bool writeOutput = false;
bool use_com = false;
std::string path = "../data";
std::string mask_path = "/annotations/PNG/pixelwise/iMap";
std::string yaml_path = "/annotations/YAML";

int main(int argc, char *argv[]) {
#ifdef DEBUG_MODE
  std::cout << "Debug mode" << std::endl;
#endif
  if (handleArguments(argc, argv)) {
    std::vector<std::string> files = getFilesInDirectory(path + yaml_path);
#ifdef DEBUG_MODE
    int counter = 0;
    uint64 time = 0;
    std::cout << "Parameter" << std::endl;
    std::cout << "  data folder: " << path << std::endl;
    std::cout << "  all images: " << all << std::endl;
    std::cout << "  drawing: " << drawOutput << std::endl;
    std::cout << "  writing: " << writeOutput << std::endl;
    std::cout << "  use only com: " << use_com << std::endl;
    std::cout << "  closing: " << m << std::endl;
#endif
#ifdef TIMER
    int counter = 0;
    uint64 time = 0;
#endif
    if (all) {  // all images
      for (unsigned int i = 0; i < files.size(); ++i) {
        Algorithm stemdet;
        stemdet.readYAML(files[i]);
#if defined(DEBUG_MODE) || defined(TIMER)
        std::chrono::steady_clock::time_point begin =
            std::chrono::steady_clock::now();
#endif
        stemdet.LeavesWithConvexHull(path, mask_path, m, drawOutput,
                                     writeOutput, use_com);
#if defined(DEBUG_MODE) || defined(TIMER)
        std::chrono::steady_clock::time_point end =
            std::chrono::steady_clock::now();
        time +=
            std::chrono::duration_cast<std::chrono::microseconds>(end - begin)
                .count();
        ++counter;
#endif
      }
#if defined(DEBUG_MODE) || defined(TIMER)
      time /= counter;
      std::cout << "Mean of used Time (ns) = " << time;
#endif
    } else {  // Single image
      if (currentID != -1) {
        currentFile = files[currentID];
      }
#ifdef DEBUG_MODE
      std::cout << "current yaml: " << currentFile << std::endl;
#endif
      Algorithm stemdet;
      stemdet.readYAML(currentFile);
#ifdef DEBUG_MODE
      std::chrono::steady_clock::time_point begin =
          std::chrono::steady_clock::now();
#endif
      stemdet.LeavesWithConvexHull(path, mask_path, m, drawOutput, writeOutput,
                                   use_com);
#ifdef DEBUG_MODE
      std::chrono::steady_clock::time_point end =
          std::chrono::steady_clock::now();

      std::cout << "Time difference (ns) = "
                << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                         begin)
                       .count()
                << std::endl;
#endif
      // stemdet.LeavesWithConvexHull(path + "/" + files[currentID], 10,
      // drawOutput);
    }
    return 0;
  } else {
    return 1;
  }
}

std::vector<std::string> getFilesInDirectory(const std::string &directory) {
  std::vector<std::string> out;

  for (auto i = boost::filesystem::directory_iterator(directory);
       i != boost::filesystem::directory_iterator(); i++) {
    if (!is_directory(i->path())) {  // we eliminate directories
      const std::string full_file_name =
          directory + "/" + i->path().filename().string();
      out.push_back(full_file_name);
    } else {
      continue;
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

bool handleArguments(int argc, char *argv[]) {
  InputParser input(argc, argv);

  if (input.cmdOptionExists("-h")) {
    printHelp();
    return false;
  }

  if (input.cmdOptionExists("-i")) {
    std::string temp = input.getCmdOption("-i");
    currentID = atoi(temp.c_str());
  } else {
    currentID = 0;
  }

  if (input.cmdOptionExists("-a")) {
    all = true;
  } else {
    all = false;
  }

  if (input.cmdOptionExists("-d")) {
    drawOutput = true;
  } else {
    drawOutput = false;
  }

  if (input.cmdOptionExists("-w")) {
    writeOutput = true;
  } else {
    writeOutput = false;
  }

  if (input.cmdOptionExists("-c")) {
    use_com = true;
  } else {
    use_com = false;
  }

  if (input.cmdOptionExists("-p")) {
    currentID = -1;
    currentFile = path + yaml_path + "/" + input.getCmdOption("-p");
  }

  if (input.cmdOptionExists("-f")) {
    path = input.getCmdOption("-f");
  }

  if (input.cmdOptionExists("-m")) {
    std::string temp = input.getCmdOption("-m");
    m = atoi(temp.c_str());
  } else {
    m = 10;
  }

  return true;
}

void printHelp() {
  std::cout << "Demo app for the leaf and stem detection package." << std::endl;
  std::cout << "  @author Ferdinand Langer" << std::endl;
  std::cout << "  @author Leonard Mandtler" << std::endl;
  std::cout << std::endl;
  std::cout << "Available Options:" << std::endl;
  std::cout << "  -h        Opens this help dialogue." << std::endl;
  std::cout << "  -f [path] Path to data folder [path]. (default = ../data/)"
            << std::endl;
  std::cout << "  -i [idx]  Progress image number [idx]. (default i = 0)"
            << std::endl;
  std::cout << "  -p [file] Progress image with filename [file]" << std::endl;
  std::cout << "  -a        Progress all images." << std::endl;
  std::cout << "  -d        Activate Drawing." << std::endl;
  std::cout << "  -w        Activate Output to file" << std::endl;
  std::cout << "  -c        Write CoM to HDD" << std::endl;
  std::cout << "  -m [px]   Radius for Closing Operation (default m = 10)"
            << std::endl;
}
