#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

inline bool exists_test0(const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

int main() {
    std::string modelFile {};
    std::cout << "Enter model file path: ";
    getline(std::cin, modelFile);
    std::cout << modelFile << std::endl;
    if (!exists_test0(modelFile)) {
        std::cout << "Model file not found" << std::endl;
    }


    torch::jit::script::Module module;

    try {
        module = torch::jit::load(modelFile);
    } catch (const c10::Error& e) {
        std::cerr << "Error loading the model: " << e.what() << std::endl;
        return -1;
    }

    // Load and preprocess the image
    std::string image_path{};
    std::cout << "Enter image file path: ";
    std::cin >> image_path;


    cv::Mat image = cv::imread(image_path);

    // Ensure that the image is in the correct format, e.g., BGR for many pre-trained models
    // cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    // Resize the image to match the input size of your model
    // cv::resize(image, image, cv::Size(width, height));

    // Convert the OpenCV Mat to a PyTorch tensor
    torch::Tensor input_tensor = torch::from_blob(image.data, {1, image.rows, image.cols, 3}, torch::kByte);
    input_tensor = input_tensor.permute({0, 3, 1, 2});  // Adjust the dimensions

    // Perform inference
    torch::Tensor output = module.forward({input_tensor}).toTensor();

    return 0;
}
