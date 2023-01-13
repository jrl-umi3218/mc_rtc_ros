#!/usr/bin/env bash

set -x

if [ $# -ne 2 ]
then
  echo "Usage: $0 [old-version] [new-version]"
  exit 1
fi

OLD_VERSION=$1
NEW_VERSION=$2

sed -i -e"s/project(mc_rtc_ros VERSION ${OLD_VERSION})/project(mc_rtc_ros VERSION ${NEW_VERSION})/" CMakeLists.txt
sed -i -e"s@<version>${OLD_VERSION}</version>@<version>${NEW_VERSION}</version>@" mc_rtc_ros/package.xml

bump_version_project()
{
  PROJECT=$1
  sed -i -e"s/\(project(.* VERSION \)${OLD_VERSION}/\1${NEW_VERSION}/" $PROJECT/CMakeLists.txt
  sed -i -e"s@<version>${OLD_VERSION}</version>@<version>${NEW_VERSION}</version>@" $PROJECT/package.xml
}

bump_version_project mc_convex_visualization
bump_version_project mc_log_visualization
bump_version_project mc_rtc_ticker
bump_version_project mc_surfaces_visualization

echo "All version changes done, update debian/changelog now"
