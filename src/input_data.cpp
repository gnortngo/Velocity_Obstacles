#include <iostream>
#include <fstream>  // Thêm thư viện fstream để sử dụng ofstream
#include <string>

int main() {
    double radius, height;

    std::cout << "Nhập bán kính của robot: ";
    std::cin >> radius;
    std::cout << "Nhập chiều cao của robot: ";
    std::cin >> height;

    // Lưu vào file
    std::ofstream outFile("robot_dimensions.txt");
    if (outFile.is_open()) {
        outFile << radius << " " << height << std::endl;
        outFile.close();
        std::cout << "Kích thước robot đã được lưu vào robot_dimensions.txt" << std::endl;
    } else {
        std::cerr << "Không thể mở file để ghi!" << std::endl;
    }

    return 0;
}

