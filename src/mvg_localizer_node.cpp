/**
 * Node to try and localize new images given a precomputed databse in MVG
 */
#include <ros/ros.h>

//MVG imports
#include "openMVG/sfm/sfm.hpp"
#include "openMVG/system/timer.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include "openMVG/image/image.hpp"
#include "openMVG/features/features.hpp"
#include "openMVG/matching/matcher_brute_force.hpp"
#include "openMVG/matching/matcher_kdtree_flann.hpp"
#include "openMVG/matching/matching_filters.hpp"
#include "openMVG_dependencies/nonFree/sift/SIFT_describer.hpp"

using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::image;

int main(int argc, char **argv) {
  ros::init(argc, argv, "mvg_localizer_node");
  ros::NodeHandle n;
  ROS_INFO("Entered node!");
  using namespace std;
  std::cout << "Compute Structure from the provided poses" << std::endl;

  CmdLine cmd;

  std::string sSfM_Data_Filename;
  std::string sMatchesDir;
  std::string sMatchFile;
  std::string sOutFile = "";

  cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
  cmd.add(make_option('m', sMatchesDir, "match_dir"));
  cmd.add(make_option('f', sMatchFile, "match_file"));

  try {
    if (argc == 1)
      throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr
        << "Usage: "
        << argv[0]
        << '\n'
        << "[-i|--input_file] path to a SfM_Data scene\n"
        << "[-m|--match_dir] path to the features and descriptor that "
        << " corresponds to the provided SfM_Data scene\n"
        << "[-f|--match_file] (opt.) path to a matches file (used pairs will be used)\n"
        << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename,
            ESfM_Data(VIEWS | INTRINSICS | EXTRINSICS))) {
    std::cerr << std::endl << "The input SfM_Data file \"" << sSfM_Data_Filename
              << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  //Pull in an image here @todo: this gets replaced by a callback for a ROS image topic
  openMVG::image::Image<unsigned char> image;
  const string jpg_filename = stlplus::folder_up(
      "/home/icoderave/data/raven/images/test.jpg");
  ReadImage(jpg_filename.c_str(), &image);

  // Call Keypoint extractor
  using namespace openMVG::features;
  std::string sImage_describer_type = "SIFT";
  std::unique_ptr<Image_describer> image_describer(new SIFT_Image_describer);
  std::map<IndexT, std::unique_ptr<features::Regions> > regions_perImage;
  image_describer->Describe(image, regions_perImage[0]);
  const SIFT_Regions* regions = dynamic_cast<SIFT_Regions*>(regions_perImage.at(
      0).get());
  const PointFeatures
      featsL = regions_perImage.at(0)->GetRegionsPositions();

  ros::spin();
  return 0;
}
