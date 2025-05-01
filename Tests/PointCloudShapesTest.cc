#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PointCloudShapes.h"

using ::testing::ContainerEq;

class Fixture : public ::testing::Test {
protected:
    void importExpectedCloud(const string&);
    void propertyCheck(shared_ptr<IPointCloudShape>);
    void transformationCheck(shared_ptr<IPointCloudShape>);
    void checkPoints(shared_ptr<IPointCloudShape>);

    bool expectedIsFilled;
    bool expectedisAreNormalsShown;
    bool expectedisIsColorable;
    bool expectedisIsDensitable;
    bool expectedisIsFillable;
    float exectedensity;
    std::string expectedName;
    std::vector<std::string> expectedLabels;
    std::vector<bool> expectedShowLabels;
    std::vector<float> expectedDimensions;
    Eigen::Vector3i expectedRotationValues;
    Eigen::Vector3f expectedTranslationValues;
    pcl::PointCloud<pcl::PointXYZ>::Ptr expectedCloud;

    void SetUp() override {
        expectedIsFilled = false;
        expectedisAreNormalsShown = false;
        expectedisIsColorable = true;
        expectedisIsDensitable = true;
        expectedisIsFillable = true;
        exectedensity = 1.0f;
        expectedName = "asd";
        expectedRotationValues = {0, 0, 0};
        expectedTranslationValues = {0.0f, 0.0f, 0.0f};
        expectedCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    void TearDown() override {
        expectedCloud->clear();
        expectedLabels.clear();
        expectedShowLabels.clear();
    }
};

void Fixture::importExpectedCloud(const string& file) {
    expectedCloud->clear();
    pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2);
    pcl::io::loadPCDFile(file + "_data.pcd", *cloud2);
    pcl::fromPCLPointCloud2(*cloud2, *expectedCloud);
}

void Fixture::propertyCheck(const shared_ptr<IPointCloudShape> cloud) {
    ASSERT_EQ(cloud->getIsFilled(), expectedIsFilled);
    ASSERT_EQ(cloud->getAreNormalsShown(), expectedisAreNormalsShown);
    ASSERT_EQ(cloud->getIsColorable(), expectedisIsColorable);
    ASSERT_EQ(cloud->getIsDensitable(), expectedisIsDensitable);
    ASSERT_EQ(cloud->getIsFillable(), expectedisIsFillable);
    ASSERT_FLOAT_EQ(cloud->getDensity(), exectedensity);
    ASSERT_EQ(cloud->getId(), expectedName);
    ASSERT_EQ(cloud->getNormalId(), expectedName + "_normals");
    EXPECT_THAT(cloud->getDimensions(), ContainerEq(expectedDimensions));
    EXPECT_THAT(cloud->getLabels(), ContainerEq(expectedLabels));
    EXPECT_THAT(cloud->getShowLabels(), ContainerEq(expectedShowLabels));
}

void Fixture::transformationCheck(const shared_ptr<IPointCloudShape> cloud) {
    EXPECT_TRUE(cloud->getTranslationValues().isApprox(expectedTranslationValues));
    EXPECT_EQ(Eigen::Vector3i(cloud->getRotationAt(0), cloud->getRotationAt(1), cloud->getRotationAt(2)),
              expectedRotationValues);
}

void Fixture::checkPoints(const shared_ptr<IPointCloudShape> cloud) {
    cloud->generateShape();
    auto cloudPoints { cloud->getShape()->points };
    for (int i=0; i<expectedCloud->points.size(); i++) {
        ASSERT_FLOAT_EQ(cloudPoints.at(i).x, expectedCloud->points.at(i).x);
        ASSERT_FLOAT_EQ(cloudPoints.at(i).y, expectedCloud->points.at(i).y);
        ASSERT_FLOAT_EQ(cloudPoints.at(i).z, expectedCloud->points.at(i).z);
    }
}

TEST_F(Fixture, TestImportPointCloudShape) {
    expectedIsFilled = false;
    expectedisAreNormalsShown = false;
    expectedisIsColorable = false;
    expectedisIsDensitable = false;
    expectedisIsFillable = false;
    expectedLabels = {};
    expectedShowLabels = {false,false,false};
    expectedDimensions = {};
    expectedTranslationValues = {0.0f, 0.0f, 0.0f};
    expectedRotationValues = {0, 0, 0};

    shared_ptr cloud = make_shared<ImportedPointCloudShape>("asd", "TestData/cube_filled_data.pcd");
    cloud->generateShape();

    propertyCheck(cloud);
    transformationCheck(cloud);

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            for (int k = -1; k <= 1; k++){
                expectedCloud->points.emplace_back(i, j, k);
            }
        }
    }
    checkPoints(cloud);
}

TEST_F(Fixture, TestRectanglePointCloudShape) {
    shared_ptr cloud { make_shared<RectanglePointCloudShape>("asd", false, 5.0f, 5.0f, 1.0f) };

    expectedIsFilled = false;
    expectedDimensions = {5.0f, 5.0f};
    expectedLabels = {"Width", "Height"};
    expectedShowLabels = {true, true, false};
    importExpectedCloud("TestData/rectangle");

    propertyCheck(cloud);
    transformationCheck(cloud);
    checkPoints(cloud);
}

TEST_F(Fixture, TestFilledRectanglePointCloudShape) {
    shared_ptr cloud { make_shared<RectanglePointCloudShape>("asd", true, 5.0f, 5.0f, 1.0f) };

    expectedIsFilled = true;
    expectedDimensions = {5.0f, 5.0f};
    expectedLabels = {"Width", "Height"};
    expectedShowLabels = {true, true, false};
    importExpectedCloud("TestData/rectangle_filled");

    propertyCheck(cloud);
    transformationCheck(cloud);
    checkPoints(cloud);
}

TEST_F(Fixture, TestCuboidPointCloudShape) {
    shared_ptr cloud { make_shared<CuboidPointCloudShape>("asd", false, 5.0f, 5.0f, 5.0f, 1.0f) };

    expectedIsFilled = false;
    expectedDimensions = {5.0f, 5.0f, 5.0f};
    expectedLabels = {"Width", "Height", "Length"};
    expectedShowLabels = {true, true, true};
    importExpectedCloud("TestData/cube");

    propertyCheck(cloud);
    transformationCheck(cloud);
    checkPoints(cloud);
}

TEST_F(Fixture, TestFilledCuboidPointCloudShape) {
    shared_ptr cloud { make_shared<CuboidPointCloudShape>("asd", true, 2.0f, 2.0f, 2.0f, 1.0f) };

    expectedIsFilled = true;
    expectedDimensions = {2.0f, 2.0f, 2.0f};
    expectedLabels = {"Width", "Height", "Length"};
    expectedShowLabels = {true, true, true};
    importExpectedCloud("TestData/cube_filled");

    propertyCheck(cloud);
    transformationCheck(cloud);
    checkPoints(cloud);
}

TEST_F(Fixture, TestCirclePointCloudShape) {
    shared_ptr cloud { make_shared<CirclePointCloudShape>("asd", false, 5.0f, 1.0f) };

    expectedIsFilled = false;
    expectedDimensions = {5.0f};
    expectedLabels = {"Radius"};
    expectedShowLabels = {true, false, false};
    importExpectedCloud("TestData/circle");

    propertyCheck(cloud);
    transformationCheck(cloud);
    checkPoints(cloud);
}

TEST_F(Fixture, TestFilledCirclePointCloudShape) {
    shared_ptr cloud { make_shared<CirclePointCloudShape>("asd", true, 5.0f, 1.0f) };

    expectedIsFilled = true;
    expectedDimensions = {5.0f};
    expectedLabels = {"Radius"};
    expectedShowLabels = {true, false, false};
    importExpectedCloud("TestData/circle_filled");

    propertyCheck(cloud);
    transformationCheck(cloud);
    checkPoints(cloud);
}

TEST_F(Fixture, TestSpherePointCloudShape) {
    shared_ptr cloud { make_shared<SpherePointCloudShape>("asd", false, 5.0f, 1.0f) };

    expectedIsFilled = false;
    expectedDimensions = {5.0f};
    expectedLabels = {"Radius"};
    expectedShowLabels = {true, false, false};
    importExpectedCloud("TestData/sphere");

    propertyCheck(cloud);
    transformationCheck(cloud);
    checkPoints(cloud);
}

TEST_F(Fixture, TestFilledSpherePointCloudShape) {
    shared_ptr cloud { make_shared<SpherePointCloudShape>("asd", true, 5.0f, 1.0f) };

    expectedIsFilled = true;
    expectedDimensions = {5.0f};
    expectedLabels = {"Radius"};
    expectedShowLabels = {true, false, false};
    importExpectedCloud("TestData/sphere_filled");

    propertyCheck(cloud);
    transformationCheck(cloud);
    checkPoints(cloud);
}

TEST_F(Fixture, TestCylinderPointCloudShape) {
    shared_ptr cloud { make_shared<CylinderPointCloudShape>("asd", false, 5.0f, 5.0f, 1.0f) };

    expectedIsFilled = false;
    expectedDimensions = {5.0f, 5.0f};
    expectedLabels = {"Radius", "Height"};
    expectedShowLabels = {true, true, false};
    importExpectedCloud("TestData/cylinder");

    propertyCheck(cloud);
    transformationCheck(cloud);
    checkPoints(cloud);
}

TEST_F(Fixture, TestFilledCylinderPointCloudShape) {
    shared_ptr cloud { make_shared<CylinderPointCloudShape>("asd", true, 5.0f, 5.0f, 1.0f) };

    expectedIsFilled = true;
    expectedDimensions = {5.0f, 5.0f};
    expectedLabels = {"Radius", "Height"};
    expectedShowLabels = {true, true, false};
    importExpectedCloud("TestData/cylinder_filled");

    propertyCheck(cloud);
    transformationCheck(cloud);
    checkPoints(cloud);
}

TEST_F(Fixture, TestConePointCloudShape) {
    shared_ptr cloud { make_shared<ConePointCloudShape>("asd", false, 5.0f, 5.0f, 1.0f) };

    expectedIsFilled = false;
    expectedDimensions = {5.0f, 5.0f};
    expectedLabels = {"Radius", "Height"};
    expectedShowLabels = {true, true, false};
    importExpectedCloud("TestData/cone");

    propertyCheck(cloud);
    transformationCheck(cloud);
    checkPoints(cloud);
}

TEST_F(Fixture, TestFilledConePointCloudShape) {
    shared_ptr cloud { make_shared<ConePointCloudShape>("asd", true, 5.0f, 5.0f, 1.0f) };

    expectedIsFilled = true;
    expectedDimensions = {5.0f, 5.0f};
    expectedLabels = {"Radius", "Height"};
    expectedShowLabels = {true, true, false};
    importExpectedCloud("TestData/cone_filled");

    propertyCheck(cloud);
    transformationCheck(cloud);
    checkPoints(cloud);
}

TEST_F(Fixture, TestIPointCloudShapeSetIsFilled) {
    shared_ptr cloud { make_shared<RectanglePointCloudShape>("asd", false, 5.0f, 5.0f, 1.0f) };
    ASSERT_EQ(cloud->getIsFilled(), expectedIsFilled);

    expectedIsFilled = true;
    cloud->setIsFilled(expectedIsFilled);
    ASSERT_EQ(cloud->getIsFilled(), expectedIsFilled);
}

TEST_F(Fixture, TestIPointCloudShapeSetAreNormalsShown) {
    shared_ptr cloud { make_shared<RectanglePointCloudShape>("asd", false, 5.0f, 5.0f, 1.0f) };
    ASSERT_EQ(cloud->getAreNormalsShown(), expectedisAreNormalsShown);

    expectedisAreNormalsShown = true;
    cloud->setAreNormalsShown(expectedisAreNormalsShown);
    ASSERT_EQ(cloud->getAreNormalsShown(), expectedisAreNormalsShown);
}

TEST_F(Fixture, TestIPointCloudShapeSetRotationAt) {
    shared_ptr cloud { make_shared<RectanglePointCloudShape>("asd", false, 5.0f, 5.0f, 1.0f) };
    EXPECT_EQ((Eigen::Vector3i(cloud->getRotationAt(0), cloud->getRotationAt(1), cloud->getRotationAt(2))),
              expectedRotationValues);

    cloud->setRotationAt(0, 1);
    cloud->setRotationAt(1, 1);
    cloud->setRotationAt(2, 1);
    expectedRotationValues = {1, 1, 1};
    EXPECT_EQ((Eigen::Vector3i(cloud->getRotationAt(0), cloud->getRotationAt(1), cloud->getRotationAt(2))),
              expectedRotationValues);
}

TEST_F(Fixture, TestIPointCloudShapeAddToTranslationValues) {
    shared_ptr cloud { make_shared<RectanglePointCloudShape>("asd", false, 5.0f, 5.0f, 1.0f) };
    EXPECT_TRUE(cloud->getTranslationValues().isApprox(expectedTranslationValues));

    cloud->addToTranslationValues( {1.0f, 1.0f, 1.0f} );
    expectedTranslationValues = {1, 1, 1};
    EXPECT_TRUE(cloud->getTranslationValues().isApprox(expectedTranslationValues));
}

TEST_F(Fixture, TestIPointCloudShapeSetDensity) {
    shared_ptr cloud { make_shared<RectanglePointCloudShape>("asd", false, 5.0f, 5.0f, 1.0f) };
    ASSERT_FLOAT_EQ(cloud->getDensity(), exectedensity);

    exectedensity = 2.0f;
    cloud->setDensity(exectedensity);
    ASSERT_FLOAT_EQ(cloud->getDensity(), exectedensity);
}