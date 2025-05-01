#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "Model.h"

using ::testing::ContainerEq;

TEST(ModelTest, TestModelAddCloud) {
    vector<shared_ptr<IPointCloudShape>> expectedClouds;
    expectedClouds.push_back(make_shared<RectanglePointCloudShape>("asd", false, 2.0f, 2.0f, 1.0f));
    expectedClouds.push_back(make_shared<RectanglePointCloudShape>("dsa", true, 2.0f, 2.0f, 1.0f));

    Model model;
    model.addCloud(expectedClouds.at(0));
    model.addCloud(expectedClouds.at(1));

    ASSERT_EQ(model.getClouds().size(), 2);
    ASSERT_EQ(model.getClouds().at(0)->getShape()->points.size(), expectedClouds.at(0)->getShape()->points.size());
    ASSERT_EQ(model.getClouds().at(1)->getShape()->points.size(), expectedClouds.at(1)->getShape()->points.size());

    for (int i = 0; i < expectedClouds.size(); i++) {
        PointCloudT::Ptr cloud = model.getClouds().at(i)->getShape();
        PointCloudT::Ptr expectedCloud = expectedClouds.at(i)->getShape();

        for (int j = 0; j < expectedCloud->points.size(); j++) {
            const auto& p = cloud->points.at(j);
            const auto& expectedPoint = expectedCloud->points.at(j);

            EXPECT_NEAR(p.x, expectedPoint.x, 1e-5);
            EXPECT_NEAR(p.y, expectedPoint.y, 1e-5);
            EXPECT_NEAR(p.z, expectedPoint.z, 1e-5);
            EXPECT_NEAR(p.normal_x, expectedPoint.normal_x, 1e-5);
            EXPECT_NEAR(p.normal_y, expectedPoint.normal_y, 1e-5);
            EXPECT_NEAR(p.normal_z, expectedPoint.normal_z, 1e-5);
            EXPECT_EQ(p.r, expectedPoint.r);
            EXPECT_EQ(p.g, expectedPoint.g);
            EXPECT_EQ(p.b, expectedPoint.b);
        }
    }
}

TEST(ModelTest, TestModelAddCloudWithSameName)
{
    Model model;
    model.addCloud(make_shared<RectanglePointCloudShape>("asd", false, 2.0f, 2.0f, 1.0f));

    EXPECT_THROW({
        model.addCloud(make_shared<RectanglePointCloudShape>("asd", false, 2.0f, 2.0f, 1.0f));
    }, std::runtime_error);
    EXPECT_THROW({
        model.addCloud(make_shared<RectanglePointCloudShape>("asd_normals", false, 2.0f, 2.0f, 1.0f));
    }, std::runtime_error);
    EXPECT_THROW({
        model.addCloud(make_shared<RectanglePointCloudShape>("BBOX", false, 2.0f, 2.0f, 1.0f));
    }, std::runtime_error);
}

TEST(ModelTest, TestModelSelectAndDeselectCloud)
{
    Model model;
    model.addCloud(make_shared<RectanglePointCloudShape>("asd", false, 2.0f, 2.0f, 1.0f));

    ASSERT_EQ(model.isCloudSelected(), false);
    ASSERT_EQ(model.getSelectedCloudShape(), nullptr);
    ASSERT_EQ(model.getSelectedCloudName(), "");
    ASSERT_EQ(model.getSelectedCloudNormalsName(), "");

    model.selectCloud("asd");
    ASSERT_EQ(model.isCloudSelected(), true);
    ASSERT_NE(model.getSelectedCloudShape(), nullptr);
    ASSERT_EQ(model.getSelectedCloudName(), "asd");
    ASSERT_EQ(model.getSelectedCloudNormalsName(), "asd_normals");

    model.deSelectCloud();
    ASSERT_EQ(model.isCloudSelected(), false);
    ASSERT_EQ(model.getSelectedCloudShape(), nullptr);
    ASSERT_EQ(model.getSelectedCloudName(), "");
    ASSERT_EQ(model.getSelectedCloudNormalsName(), "");
}

TEST(ModelTest, TestModelRemoveSelectedCloud)
{
    Model model;

    ASSERT_EQ(model.getClouds().size(), 0);
    model.removeSelectedCloud(); //wont remove anything, if no cloud is selected
    ASSERT_EQ(model.getClouds().size(), 0);

    model.addCloud(make_shared<RectanglePointCloudShape>("asd", false, 2.0f, 2.0f, 1.0f));
    ASSERT_EQ(model.getClouds().size(), 1);
    model.selectCloud("asd");
    model.removeSelectedCloud();
    ASSERT_EQ(model.getClouds().size(), 0);
}

TEST(ModelTest, TestModelColorSelectedCloud) {
    pcl::RGB expectedRGB = {0,0,0};

    shared_ptr shape { make_shared<RectanglePointCloudShape>("asd", false, 2.0f, 2.0f, 1.0f) };
    shape->generateShape();

    Model model;
    model.addCloud(shape);
    model.selectCloud("asd");
    model.colorSelectedCloud(expectedRGB);
    PointCloudT::ConstPtr cloud { model.getSelectedCloudShape() };
    ASSERT_EQ(cloud->points.size(), 8);
    for (int i = 0; i < cloud->points.size(); i++) {
        const auto& p = cloud->points.at(i);
        EXPECT_EQ(p.r, expectedRGB.r);
        EXPECT_EQ(p.g, expectedRGB.g);
        EXPECT_EQ(p.b, expectedRGB.b);
    }
}

TEST(ModelTest, TestModelColorUpdateSelectedCloudDimensions) {
    Model model;
    model.addCloud(make_shared<RectanglePointCloudShape>("asd", true, 5.0f, 5.0f, 1.0f));

    model.selectCloud("asd");
    model.updateSelectedCloudDimensions(2.0f, 2.0f, 2.0f);

    int count = 0;
    PointCloudT::ConstPtr shape { model.getSelectedCloudShape() };
    ASSERT_EQ(shape->points.size(), 9);
    for (float i = -1; i <= 1; i++) {
        for (float j = -1; j <= 1; j++) {
            pcl::PointXYZ expectedPoint { i,j,0 };
            EXPECT_NEAR(shape->points.at(count).x, expectedPoint.x, 1e-5);
            EXPECT_NEAR(shape->points.at(count).y, expectedPoint.y, 1e-5);
            EXPECT_NEAR(shape->points.at(count).z, expectedPoint.z, 1e-5);
            count++;
        }
    }
}

TEST(ModelTest, TestModelColorUpdateSelectedCloudDensity) {
    Model model;
    model.addCloud(make_shared<RectanglePointCloudShape>("asd", true, 2.0f, 2.0f, 1.0f));

    model.selectCloud("asd");
    model.updateSelectedCloudDensity(2.0f);

    int count = 0;
    PointCloudT::ConstPtr shape { model.getSelectedCloudShape() };
    ASSERT_EQ(shape->points.size(), 25);
    for (float i = -1; i <= 1; i += 0.5f) {
        for (float j = -1; j <= 1; j += 0.5f) {
            pcl::PointXYZ expectedPoint { i,j,0 };
            EXPECT_NEAR(shape->points.at(count).x, expectedPoint.x, 1e-5);
            EXPECT_NEAR(shape->points.at(count).y, expectedPoint.y, 1e-5);
            EXPECT_NEAR(shape->points.at(count).z, expectedPoint.z, 1e-5);
            count++;
        }
    }
}

TEST(ModelTest, TestModelColorUpdateSelectedCloudIsFilled) {
    Model model;
    model.addCloud(make_shared<RectanglePointCloudShape>("asd", false, 2.0f, 2.0f, 1.0f));

    model.selectCloud("asd");
    model.updateSelectedCloudIsFilled(true);

    int count = 0;
    PointCloudT::ConstPtr shape { model.getSelectedCloudShape() };
    ASSERT_EQ(shape->points.size(), 9);
    for (float i = -1; i <= 1; i++) {
        for (float j = -1; j <= 1; j++) {
            pcl::PointXYZ expectedPoint { i,j,0 };
            EXPECT_NEAR(shape->points.at(count).x, expectedPoint.x, 1e-5);
            EXPECT_NEAR(shape->points.at(count).y, expectedPoint.y, 1e-5);
            EXPECT_NEAR(shape->points.at(count).z, expectedPoint.z, 1e-5);
            count++;
        }
    }
}

TEST(ModelTest, TestModelUpdateSelectedCloudAreNormalsShown) {
    Model model;
    model.addCloud(make_shared<RectanglePointCloudShape>("asd", true, 2.0f, 2.0f, 1.0f));
    ASSERT_EQ(model.getSelectedCloudAreNormalsShown(), false);

    model.selectCloud("asd");
    model.updateSelectedCloudAreNormalsShown(true);
    ASSERT_EQ(model.getSelectedCloudAreNormalsShown(), true);

    model.updateSelectedCloudAreNormalsShown(false);
    ASSERT_EQ(model.getSelectedCloudAreNormalsShown(), false);
}

TEST(ModelTest, TestModelTranslateSelectedCloud) {
    Eigen::Affine3f dummy;
    float expectedZLocation = 0.0f;
    shared_ptr shape { make_shared<RectanglePointCloudShape>("asd", true, 2.0f, 2.0f, 1.0f) };
    shape->generateShape();

    Model model;
    model.addCloud(shape);
    model.selectCloud("asd");
    model.translateSelectedCloud(0.0f, 0.0f, expectedZLocation, dummy);

    int count = 0;
    PointCloudT::ConstPtr cloud { model.getSelectedCloudShape() };
    for (float i = -1; i <= 1; i++) {
        for (float j = -1; j <= 1; j++) {
            pcl::PointXYZ expectedPoint { i,j,expectedZLocation };
            EXPECT_NEAR(cloud->points.at(count).x, expectedPoint.x, 1e-5);
            EXPECT_NEAR(cloud->points.at(count).y, expectedPoint.y, 1e-5);
            EXPECT_NEAR(cloud->points.at(count).z, expectedPoint.z, 1e-5);
            count++;
        }
    }
}

TEST(ModelTest, TestModelRotateSelectedCloud) {
    Eigen::Affine3f dummy;
    shared_ptr shape { make_shared<RectanglePointCloudShape>("asd", true, 2.0f, 2.0f, 1.0f) };
    shape->generateShape();

    Model model;
    model.addCloud(shape);
    model.selectCloud("asd");
    model.rotateSelectedCloud(90, 'x', dummy);

    int count = 0;
    PointCloudT::ConstPtr cloud { model.getSelectedCloudShape() };
    for (float i = -1; i <= 1; i++) {
        for (float j = -1; j <= 1; j++) {
            pcl::PointXYZ expectedPoint { i,0.0f,j };
            EXPECT_NEAR(cloud->points.at(count).x, expectedPoint.x, 1e-5);
            EXPECT_NEAR(cloud->points.at(count).y, expectedPoint.y, 1e-5);
            EXPECT_NEAR(cloud->points.at(count).z, expectedPoint.z, 1e-5);
            count++;
        }
    }

    count = 0;
    model.rotateSelectedCloud(90, 'z', dummy);
    cloud = model.getSelectedCloudShape();
    for (float i = -1; i <= 1; i++) {
        for (float j = -1; j <= 1; j++) {
            pcl::PointXYZ expectedPoint { 0.0f, i,j };
            EXPECT_NEAR(cloud->points.at(count).x, expectedPoint.x, 1e-5);
            EXPECT_NEAR(cloud->points.at(count).y, expectedPoint.y, 1e-5);
            EXPECT_NEAR(cloud->points.at(count).z, expectedPoint.z, 1e-5);
            count++;
        }
    }

    count = 0;
    model.rotateSelectedCloud(90, 'y', dummy);
    cloud = model.getSelectedCloudShape();
    for (float i = -1; i <= 1; i++) {
        for (float j = -1; j <= 1; j++) {
            pcl::PointXYZ expectedPoint { j, i , 0.0f};
            EXPECT_NEAR(cloud->points.at(count).x, expectedPoint.x, 1e-5);
            EXPECT_NEAR(cloud->points.at(count).y, expectedPoint.y, 1e-5);
            EXPECT_NEAR(cloud->points.at(count).z, expectedPoint.z, 1e-5);
            count++;
        }
    }
}

TEST(ModelTest, TestModelGetEditCloudData) {
    Model model;
    model.addCloud(make_shared<RectanglePointCloudShape>("asd", true, 2.0f, 2.0f, 1.0f));

    EditCloudData data { model.getEditCloudData("asd") };
    ASSERT_EQ(data.name, "asd");
    ASSERT_EQ(data.isFilled, true);
    ASSERT_EQ(data.areNormalsShown, false);
    ASSERT_EQ(data.showFilledEdit, true);
    ASSERT_EQ(data.showColorEdit, true);
    ASSERT_EQ(data.showDensityEdit, true);
    ASSERT_EQ(data.density, 1.0f);
    EXPECT_THAT(data.rgb, ContainerEq(std::vector<int>{255, 255, 255}));
    EXPECT_THAT(data.rotation, ContainerEq(std::vector<int>{0, 0, 0}));
    EXPECT_THAT(data.dim, ContainerEq(std::vector<float>{2.0f, 2.0f}));
    EXPECT_THAT(data.labels, ContainerEq(std::vector<std::string>{"Width", "Height"}));
    EXPECT_THAT(data.showLabels, ContainerEq(std::vector<bool>{true, true,false}));
}

TEST(ModelTest, TestModelGetBoundingBoxDataAroundSelectedCloud) {
    Model model;
    shared_ptr shape { make_shared<RectanglePointCloudShape>("asd", true, 2.0f, 2.0f, 1.0f) };
    shape->generateShape();
    model.addCloud(shape);

    model.selectCloud("asd");
    ASSERT_EQ(model.getBoundingBoxDataAroundSelectedCloud().NAME, "BBOX");
    ASSERT_FLOAT_EQ(model.getBoundingBoxDataAroundSelectedCloud().minX, -1.0f);
    ASSERT_FLOAT_EQ(model.getBoundingBoxDataAroundSelectedCloud().minY, -1.0f);
    ASSERT_FLOAT_EQ(model.getBoundingBoxDataAroundSelectedCloud().minZ, 0.0f);
    ASSERT_FLOAT_EQ(model.getBoundingBoxDataAroundSelectedCloud().maxX, 1.0f);
    ASSERT_FLOAT_EQ(model.getBoundingBoxDataAroundSelectedCloud().maxY, 1.0f);
    ASSERT_FLOAT_EQ(model.getBoundingBoxDataAroundSelectedCloud().maxZ, 0.0f);
}

TEST(ModelTest, TestModelGetCloudNames) {
    Model model;
    EXPECT_THAT(model.getCloudNames(), ContainerEq(std::vector<std::string>{}));

    model.addCloud(make_shared<RectanglePointCloudShape>("asd", true, 2.0f, 2.0f, 1.0f));
    model.addCloud(make_shared<RectanglePointCloudShape>("asdd", true, 2.0f, 2.0f, 1.0f));
    model.addCloud(make_shared<RectanglePointCloudShape>("asddd", true, 2.0f, 2.0f, 1.0f));
    model.addCloud(make_shared<RectanglePointCloudShape>("asdddd", true, 2.0f, 2.0f, 1.0f));
    EXPECT_THAT(model.getCloudNames(), ContainerEq(std::vector<std::string>{"asd", "asdd", "asddd", "asdddd"}));
}

TEST(ModelTest, TestModelSelectedCloudShape) {
    Model model;
    shared_ptr shape { make_shared<RectanglePointCloudShape>("asd", true, 2.0f, 2.0f, 1.0f) };
    shape->generateShape();
    model.addCloud(shape);
    ASSERT_EQ(model.getSelectedCloudShape(), nullptr);

    model.selectCloud("asd");
    ASSERT_EQ(model.getSelectedCloudShape()->points.size(), 9);
}