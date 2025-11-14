#include "gnss.h"
#include "utils/math.hpp"
#include "utm.h"
#include <glog/logging.h>

bool GNSS::convert_utm() {
  char hemi{};
  double easting{}, northing{};
  long res = Convert_Geodetic_To_UTM(geodetic_[0] * Math::kdeg2rad,
                                     geodetic_[1] * Math::kdeg2rad, &utm_.zone,
                                     &hemi, &easting, &northing);
  utm_.north_hemi = hemi == 'N';
  utm_.xy = Eigen::Vector2d(easting, northing);

  return res == UTM_NO_ERROR;
};

Sophus::SE3d GNSS::body_pose(const double antenna_x, const double antenna_y,
                             const double antenna_theta,
                             const Sophus::SE3d &origin) const {
  double z_angle = 0;
  if (heading_valid_) {
    z_angle =
        (90 - heading_) * Math::kdeg2rad; // north-east-down to east-north-up
  }
  Sophus::SE3d Twg(Sophus::SO3d::exp(Eigen::Vector3d(0, 0, z_angle)),
                   Eigen::Vector3d(utm_.xy[0], utm_.xy[1], geodetic_[2]));
  Sophus::SE3d Tbg(
      Sophus::SO3d::exp(Eigen::Vector3d(0, 0, antenna_theta * Math::kdeg2rad)),
      Eigen::Vector3d(antenna_x, antenna_y, 0));
  Sophus::SE3d Twb = Twg * Tbg.inverse();
  Sophus::SE3d Tob = origin.inverse() * Twb;

  return Tob;
}

Sophus::SE3d GNSS::body_pose(const Sophus::SE3d &origin) const {
  return body_pose(antenna_x_, antenna_y_, antenna_theta_, origin);
}