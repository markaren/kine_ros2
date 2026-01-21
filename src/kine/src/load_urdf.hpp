#ifndef KINE_LOAD_URDF_HPP
#define KINE_LOAD_URDF_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <threepp/loaders/URDFLoader.hpp>

inline std::shared_ptr<threepp::Robot> loadRobot(const std::string &urdf,
                                                 std::shared_ptr<threepp::Loader<threepp::Group> > geometryLoader =
                                                         nullptr) {
    threepp::URDFLoader urdfLoader;
    urdfLoader.setGeometryLoader(geometryLoader);

    const auto share = ament_index_cpp::get_package_share_directory("kine_robot_description");
    return urdfLoader.parse(share, urdf);
}

#endif //KINE_LOAD_URDF_HPP
