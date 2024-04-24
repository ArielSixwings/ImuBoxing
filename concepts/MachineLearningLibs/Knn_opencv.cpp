#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>

using namespace cv::ml;

int main()
{
    // Define data properties
    const int numFeatures = 2;
    const int numSamplesPerClass = 50;
    const int numClasses = 2;

    // Generate sample data (replace with your actual data generation method)
    cv::Mat features(numSamplesPerClass * numClasses, numFeatures, CV_32F);
    cv::Mat labels(numSamplesPerClass * numClasses, 1, CV_32S);

    // ... (fill features and labels with your data)

    // Convert labels to floating-point for KNN
    cv::Mat floatLabels(labels.rows, labels.cols, CV_32F);
    labels.convertTo(floatLabels, CV_32F);

    // Create and train KNN classifier
    Ptr<KNearest> knn = KNearest::create();
    knn->setAlgorithmType(KNearest::BRUTE_FORCE); // Adjust parameters as needed
    knn->train(features, cv::ROW_SAMPLE, floatLabels);

    // Classify a new data point (example)
    cv::Mat newData(1, numFeatures, CV_32F);
    newData << 0.5, 0.0; // Replace with your new data point

    // Predict the class label for the new data point
    cv::Mat predictedLabel = knn->predict(newData);
    int predictedClass = predictedLabel.at<float>(0, 0); // Access the predicted class value

    std::cout << "Predicted class: " << predictedClass << std::endl;

    return 0;
}
