#include <common/utils.hpp>

std::string find_config(const std::string& filename)
{
    const std::string local = "config/" + filename;
    if (std::filesystem::exists(local)) return local;

    const std::string vp_local = "VisionPilot/config/" + filename;
    if (std::filesystem::exists(vp_local)) return vp_local;

    const std::string vp_build_local = "VisionPilot/build/config/" + filename;
    if (std::filesystem::exists(vp_build_local)) return vp_build_local;

    const std::string parent_local = "../config/" + filename;
    if (std::filesystem::exists(parent_local)) return parent_local;

    std::error_code ec;
    auto exe_path = std::filesystem::canonical("/proc/self/exe", ec);
    if (!ec) {
        auto exe_dir = exe_path.parent_path();
        if (std::filesystem::exists(exe_dir / "config" / filename))
            return (exe_dir / "config" / filename).string();
        if (std::filesystem::exists(exe_dir.parent_path() / "config" / filename))
            return (exe_dir.parent_path() / "config" / filename).string();
    }

    const std::string system = "/usr/share/visionpilot/config/" + filename;
    if (std::filesystem::exists(system)) return system;

    throw std::runtime_error("Config file not found: " + filename);
}

cv::Mat load_matrix(const std::string& filename, const std::string& matrix)
{
    const std::string path = find_config(filename);
    const cv::FileStorage fs(path, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        throw std::runtime_error("Failed to open calibration file: ");
    }
    cv::Mat M;
    fs[matrix] >> M;

    return M;
}
