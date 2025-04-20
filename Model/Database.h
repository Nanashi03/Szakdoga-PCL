//
// Created by kristof on 4/19/25.
//

#ifndef DATABASE_H
#define DATABASE_H

#include <sqlite3.h>
#include <boost/fusion/container/list/cons.hpp>
#include <pcl/common/common.h>

#include "EditCloudData.h"

using namespace std;

using PointType = pcl::PointXYZRGBNormal;
using PointCloudT = pcl::PointCloud<PointType>;

class Database {
private:
    static string filePath;
    static shared_ptr<Database> instance;
    sqlite3* db;
    sqlite3_stmt* insertCloudStmt;
    sqlite3_stmt* getCloudStmt;

    sqlite3_stmt* insertCloudIdStmt;
    sqlite3_stmt* getCloudIndexStmt;
    sqlite3_stmt* getCloudTypeStmt;
    sqlite3_stmt* getCloudNamesStmt;

    sqlite3_stmt* insertCloudPropertiesStmt;
    sqlite3_stmt* getCloudPropertiesStmt;

    sqlite3_stmt* insertCloudTransformationStmt;
    sqlite3_stmt* getCloudTransformationStmt;
    void runAndHandleError(sqlite3_stmt*&);
    Database();
    Database(const Database&) = delete;
    Database& operator=(const Database&) = delete;
public:
    static shared_ptr<Database> getInstance(const string& file) {
        if (!instance || file != filePath){
            filePath = file;
            instance = shared_ptr<Database>(new Database());
        }
        return instance;
    }
    void addPointCloudNameToDatabase(const string&, const string&);
    int getPointCloudIndexByName(const string&) const;
    string getPointCloudTypeByName(const string&) const;
    vector<string> getPointCloudNamesFromDatabase() const;

    void addPointCloudToDatabase(const string&, PointCloudT::Ptr);
    PointCloudT::Ptr getPointCloudFromDatabase(const string&) const;

    void addPointCloudPropertiesToDatabase(const string&, const EditCloudData&);
    EditCloudData getPointCloudPropertiesFromDatabase(const string&) const;

    void addPointCloudTransformationToDatabase(const string&, const Eigen::Vector3f&, const Eigen::Affine3f&);
    tuple<Eigen::Vector3f, Eigen::Affine3f> getPointCloudTransformationFromDatabase(const string&);
    ~Database();
};



#endif //DATABASE_H
