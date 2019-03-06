#ifndef ASLAM_GRID_CALIBRATION_TARGET_APRILGRID_HOLO_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_APRILGRID_HOLO_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <sm/assert_macros.hpp>
#include <aslam/cameras/GridCalibrationTargetBase.hpp>
#include <boost/serialization/export.hpp>

// April tags detector and various tag families
#include "apriltags/TagDetector.h"
//#include "apriltags/Tag16h5.h"
//#include "apriltags/Tag25h7.h"
//#include "apriltags/Tag25h9.h"
//#include "apriltags/Tag36h9.h"
#include "apriltags/Tag36h11.h"

namespace aslam {
namespace cameras {

class GridCalibrationTargetAprilgridHolo : public GridCalibrationTargetBase {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  typedef boost::shared_ptr<GridCalibrationTargetAprilgridHolo> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetAprilgridHolo> ConstPtr;

  //target extraction options
  struct AprilgridOptionsHolo {
    AprilgridOptionsHolo() :
      doSubpixRefinement(true),
      maxSubpixDisplacement2(1.5),
      showExtractionVideo(false),
      minTagsForValidObs(4),
      minBorderDistance(4.0),
      blackTagBorder(1) {};

    //options
    /// \brief subpixel refinement of extracted corners
    bool doSubpixRefinement;

    /// \brief max. displacement squarred in subpixel refinement  [px^2]
    double maxSubpixDisplacement2;

    /// \brief show video during extraction
    bool showExtractionVideo;

    /// \brief min. number of tags for a valid observation
    unsigned int minTagsForValidObs;

    /// \brief min. distance form image border for valid points [px]
    double minBorderDistance;

    /// \brief size of black border around the tag code bits (in pixels)
    unsigned int blackTagBorder;

    /// \brief Serialization support
    enum {CLASS_SERIALIZATION_VERSION = 1};
    BOOST_SERIALIZATION_SPLIT_MEMBER();
    template<class Archive>
    void save(Archive & ar, const unsigned int /*version*/) const
    {
       ar << BOOST_SERIALIZATION_NVP(doSubpixRefinement);
       ar << BOOST_SERIALIZATION_NVP(maxSubpixDisplacement2);
       ar << BOOST_SERIALIZATION_NVP(showExtractionVideo);
       ar << BOOST_SERIALIZATION_NVP(minTagsForValidObs);
       ar << BOOST_SERIALIZATION_NVP(minBorderDistance);
       ar << BOOST_SERIALIZATION_NVP(blackTagBorder);
    }
    template<class Archive>
    void load(Archive & ar, const unsigned int /*version*/)
    {
       ar >> BOOST_SERIALIZATION_NVP(doSubpixRefinement);
       ar >> BOOST_SERIALIZATION_NVP(maxSubpixDisplacement2);
       ar >> BOOST_SERIALIZATION_NVP(showExtractionVideo);
       ar >> BOOST_SERIALIZATION_NVP(minTagsForValidObs);
       ar >> BOOST_SERIALIZATION_NVP(minBorderDistance);
       ar >> BOOST_SERIALIZATION_NVP(blackTagBorder);
    }
  };

  /// \brief initialize based on checkerboard geometry
  ///        assume all tag are set in one column
  GridCalibrationTargetAprilgridHolo(size_t tagRows, size_t tagCols, std::string tagDatabase_file,
                                     const AprilgridOptionsHolo &options = AprilgridOptionsHolo());

  virtual ~GridCalibrationTargetAprilgridHolo() {};

  /// \brief extract the calibration target points from an image and write to an observation
  bool computeObservation(const cv::Mat & image,
                          Eigen::MatrixXd & outImagePoints,
                          std::vector<bool> &outCornerObserved) const;

 private:
  /// \brief initialize the object
  void initialize();

  /// \brief initialize the grid with tag database
  void loadGridPoints(std::string tagDatabase_file);

  /// \brief load on point
  void loadPoint(std::ifstream& ifile, Eigen::Matrix<double, 1, 3>& pt);

  /// \brief target extraction options
  AprilgridOptionsHolo _options;

  /// whether this tag is valid
  std::vector<bool> tagStatus;

  // create a detector instance
  AprilTags::TagCodes _tagCodes;
  boost::shared_ptr<AprilTags::TagDetector> _tagDetector;

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
 public:
  enum {CLASS_SERIALIZATION_VERSION = 1};
  BOOST_SERIALIZATION_SPLIT_MEMBER()

  //serialization ctor
  GridCalibrationTargetAprilgridHolo();

 protected:
  friend class boost::serialization::access;

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    boost::serialization::void_cast_register<GridCalibrationTargetAprilgridHolo, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetAprilgridHolo *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar << BOOST_SERIALIZATION_NVP(_options);
  }
  template<class Archive>
  void load(Archive & ar, const unsigned int /* version */) {
    boost::serialization::void_cast_register<GridCalibrationTargetAprilgridHolo, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetAprilgridHolo *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar >> BOOST_SERIALIZATION_NVP(_options);
    initialize();
  }
};


}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetAprilgridHolo);
SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetAprilgridHolo::AprilgridOptionsHolo);
BOOST_CLASS_EXPORT_KEY(aslam::cameras::GridCalibrationTargetAprilgridHolo)

#endif /* ASLAM_GRID_CALIBRATION_TARGET_APRILGRID_HOLO_HPP */

