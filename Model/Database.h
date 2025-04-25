//
// Created by kristof on 4/19/25.
//

#ifndef DATABASE_H
#define DATABASE_H

#include <unordered_map>
#include <sqlite3.h>
#include <boost/fusion/container/list/cons.hpp>
#include <pcl/common/common.h>

#include "EditCloudData.h"

using namespace std;

using PointType = pcl::PointXYZRGBNormal;
using PointCloudT = pcl::PointCloud<PointType>;

class Database {
private:
    const unordered_map<string, string> tableSchemes = {
    {"PointCloudNames", "CREATE TABLE PointCloudNames (ind INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT UNIQUE, type TEXT)"},
    {"PointCloud", "CREATE TABLE PointCloud (cloudId INTEGER, x REAL, y REAL, z REAL, r INTEGER, g INTEGER, b INTEGER, normal_x REAL, normal_y REAL, normal_z REAL, FOREIGN KEY(cloudId) REFERENCES PointCloudNames(ind))"},
    {"PointCloudProperties", "CREATE TABLE PointCloudProperties (cloudId INTEGER, isFilled INTEGER, areNormalsShown INTEGER, r INTEGER, g INTEGER, b INTEGER, rot_x INTEGER, rot_y INTEGER, rot_z INTEGER, dim_x INTEGER, dim_y INTEGER, dim_z INTEGER, density REAL, FOREIGN KEY(cloudId) REFERENCES PointCloudNames(ind))"},
    {"PointCloudTransformation", "CREATE TABLE PointCloudTransformation (cloudId INTEGER, translationX REAL, translationY REAL, translationZ REAL, rotMatrix00 REAL, rotMatrix01 REAL, rotMatrix02 REAL, rotMatrix03 REAL, rotMatrix10 REAL, rotMatrix11 REAL, rotMatrix12 REAL, rotMatrix13 REAL, rotMatrix20 REAL, rotMatrix21 REAL, rotMatrix22 REAL, rotMatrix23 REAL, FOREIGN KEY(cloudId) REFERENCES PointCloudNames(ind))"}
    };

    static string filePath, command;
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

    Database();
    Database(const Database&) = delete;
    void runAndHandleError(sqlite3_stmt*&);
    void checkDatabaseBeforeImporting();
    void createBeforeExporting();
    void dropExistingTables();
    Database& operator=(const Database&) = delete;
public:
    static shared_ptr<Database> getInstance(const string& file, const string& cmd) {
        if (!instance || file != filePath || command != cmd){
            filePath = file;
            command = cmd;
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
