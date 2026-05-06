//
// Created by Interweave on 2026/4/09.
//

#include <iomanip>
#include <iostream>
#include <toml++/toml.hpp>

int main(int argc, char **argv)
{
    std::cout << std::setprecision(17);

    std::string config_path = "../config/vision.toml";
    if (argc > 1) {
        config_path = argv[1];
    }

    try {
        auto config = toml::parse_file(config_path);

        std::cout << "Read TOML: " << config_path << std::endl;

        if (auto yolo_name = config["yolo_name"].value<std::string>()) {
            std::cout << "yolo_name: " << *yolo_name << std::endl;
        }

        if (auto min_confidence = config["min_confidence"].value<double>()) {
            std::cout << "min_confidence: " << *min_confidence << std::endl;
        }

        if (auto roi = config["roi"].as_table()) {
            std::cout << "roi.x: " << (*roi)["x"].value_or(0) << std::endl;
            std::cout << "roi.y: " << (*roi)["y"].value_or(0) << std::endl;
            std::cout << "roi.width: " << (*roi)["width"].value_or(0) << std::endl;
            std::cout << "roi.height: " << (*roi)["height"].value_or(0) << std::endl;
        }

    } catch (const toml::parse_error &err) {
        std::cerr << "TOML读取失败: " << err.description() << std::endl;
        std::cerr << "文件路径: " << config_path << std::endl;
        return -1;
    }

    return 0;
}