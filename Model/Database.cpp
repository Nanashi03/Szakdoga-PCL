//
// Created by kristof on 4/19/25.
//

#include "Database.h"

string Database::filePath = "point_cloud_database.db";
shared_ptr<Database> Database::instance = nullptr;

Database::Database() {
    if (sqlite3_open(filePath.c_str(), &db) != SQLITE_OK)
        throw runtime_error("Failed to open database: " + std::string(sqlite3_errmsg(db)));
    if (sqlite3_exec(db, "PRAGMA foreign_keys = ON;", nullptr, nullptr, nullptr) != SQLITE_OK)
        throw runtime_error("Failed to enable foreign key support: " + string(sqlite3_errmsg(db)));

    string querry = "CREATE TABLE IF NOT EXISTS PointCloudNames (ind INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT UNIQUE, type TEXT);";
    if (sqlite3_exec(db, querry.c_str(), nullptr, nullptr, nullptr) != SQLITE_OK)
        throw runtime_error("Failed to open database: " + string(sqlite3_errmsg(db)));

    querry = "CREATE TABLE IF NOT EXISTS PointCloud ("
                   "cloudId INTEGER, "
                   "x REAL, "
                   "y REAL, "
                   "z REAL, "
                   "r INTEGER, "
                   "g INTEGER, "
                   "b INTEGER, "
                   "normal_x REAL, "
                   "normal_y REAL, "
                   "normal_z REAL, "
                   "FOREIGN KEY(cloudId) REFERENCES PointCloudNames(ind));";
    if (sqlite3_exec(db, querry.c_str(), nullptr, nullptr, nullptr) != SQLITE_OK)
        throw runtime_error("Failed to open database: " + string(sqlite3_errmsg(db)));

    querry = "CREATE TABLE IF NOT EXISTS PointCloudProperties ("
               "cloudId INTEGER, "
               "isFilled INTEGER, "
               "areNormalsShown INTEGER, "
               "r INTEGER, "
               "g INTEGER, "
               "b INTEGER, "
               "rot_x INTEGER, "
               "rot_y INTEGER, "
               "rot_z INTEGER, "
               "dim_x INTEGER, "
               "dim_y INTEGER, "
               "dim_z INTEGER, "
               "density REAL, "
               "FOREIGN KEY(cloudId) REFERENCES PointCloudNames(ind));";
    if (sqlite3_exec(db, querry.c_str(), nullptr, nullptr, nullptr) != SQLITE_OK)
        throw runtime_error("Failed to open database: " + string(sqlite3_errmsg(db)));

    querry = "CREATE TABLE IF NOT EXISTS PointCloudTransformation ("
           "cloudId INTEGER, "
           "translationX REAL, "
           "translationY REAL, "
           "translationZ REAL, "
           "rotMatrix00 REAL, rotMatrix01 REAL, rotMatrix02 REAL, rotMatrix03 REAL, "
           "rotMatrix10 REAL, rotMatrix11 REAL, rotMatrix12 REAL, rotMatrix13 REAL, "
           "rotMatrix20 REAL, rotMatrix21 REAL, rotMatrix22 REAL, rotMatrix23 REAL, "
           "FOREIGN KEY(cloudId) REFERENCES PointCloudNames(ind));";
    if (sqlite3_exec(db, querry.c_str(), nullptr, nullptr, nullptr) != SQLITE_OK)
        throw runtime_error("Failed to open database: " + string(sqlite3_errmsg(db)));

    querry = "SELECT ind FROM PointCloudNames WHERE name = ?;";
    sqlite3_prepare_v2(db, querry.c_str(), -1, &getCloudIndexStmt, nullptr);
    querry = "INSERT INTO PointCloudNames (name, type) VALUES (?, ?);";
    sqlite3_prepare_v2(db, querry.c_str(), -1, &insertCloudIdStmt, nullptr);
    querry = "SELECT type FROM PointCloudNames WHERE name = ?;";
    sqlite3_prepare_v2(db, querry.c_str(), -1, &getCloudTypeStmt, nullptr);
    querry = "SELECT name FROM PointCloudNames;";
    sqlite3_prepare_v2(db, querry.c_str(), -1, &getCloudNamesStmt, nullptr);

    querry = "SELECT x,y,z,r,g,b,normal_x,normal_y,normal_z FROM PointCloud WHERE cloudId = ?;";
    sqlite3_prepare_v2(db, querry.c_str(), -1, &getCloudStmt, nullptr);
    querry = "INSERT INTO PointCloud (cloudId, x, y, z, r, g, b, normal_x, normal_y, normal_z) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";
    sqlite3_prepare_v2(db, querry.c_str(), -1, &insertCloudStmt, nullptr);

    querry = "SELECT isFilled, areNormalsShown, r, g, b, rot_x, rot_y, rot_z, dim_x, dim_y, dim_z, density "
             "FROM PointCloudProperties WHERE cloudId = ?;";
    sqlite3_prepare_v2(db, querry.c_str(), -1, &getCloudPropertiesStmt, nullptr);
    querry = "INSERT INTO PointCloudProperties(cloudId, isFilled, areNormalsShown, r, g, b, rot_x, rot_y, rot_z, dim_x, dim_y, dim_z, density) "
             "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?);";
    sqlite3_prepare_v2(db, querry.c_str(), -1, &insertCloudPropertiesStmt, nullptr);

    querry = "SELECT translationX, translationY, translationZ, rotMatrix00,  rotMatrix01,  rotMatrix02,  rotMatrix03,  rotMatrix10,  rotMatrix11,  rotMatrix12,  rotMatrix13,  rotMatrix20,  rotMatrix21,  rotMatrix22,  rotMatrix23 "
             "FROM PointCloudTransformation WHERE cloudId = ?;";
    sqlite3_prepare_v2(db, querry.c_str(), -1, &getCloudTransformationStmt, nullptr);
    querry = "INSERT INTO PointCloudTransformation(cloudId, translationX, translationY, translationZ, rotMatrix00,  rotMatrix01,  rotMatrix02,  rotMatrix03,  rotMatrix10,  rotMatrix11,  rotMatrix12,  rotMatrix13,  rotMatrix20,  rotMatrix21,  rotMatrix22,  rotMatrix23) "
             "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?);";
    sqlite3_prepare_v2(db, querry.c_str(), -1, &insertCloudTransformationStmt, nullptr);
}

void Database::runAndHandleError(sqlite3_stmt*& stmt) {
    int a { sqlite3_step(stmt) };
    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);
    if (a != SQLITE_DONE)
        throw runtime_error("Database action failed: " + string(sqlite3_errmsg(db)));
}


void Database::addPointCloudNameToDatabase(const string& name, const string& type)
{
    sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);
    sqlite3_bind_text(insertCloudIdStmt, 1, name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(insertCloudIdStmt, 2, type.c_str(), -1, SQLITE_TRANSIENT);
    runAndHandleError(insertCloudIdStmt);
    sqlite3_exec(db, "END TRANSACTION;", nullptr, nullptr, nullptr);
}

int Database::getPointCloudIndexByName(const string& name) const {
    sqlite3_reset(getCloudIndexStmt);
    sqlite3_clear_bindings(getCloudIndexStmt);
    sqlite3_bind_text(getCloudIndexStmt, 1, name.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(getCloudIndexStmt) == SQLITE_ROW)
        return sqlite3_column_int(getCloudIndexStmt, 0);
    return -1;
}

string Database::getPointCloudTypeByName(const string& name) const {
    sqlite3_reset(getCloudTypeStmt);
    sqlite3_clear_bindings(getCloudTypeStmt);
    sqlite3_bind_text(getCloudTypeStmt, 1, name.c_str(), -1, SQLITE_TRANSIENT);
    cout << "ASD" << endl;
    if (sqlite3_step(getCloudTypeStmt) == SQLITE_ROW)
        return reinterpret_cast<const char*>(sqlite3_column_text(getCloudTypeStmt, 0));
    return "";
}

vector<string> Database::getPointCloudNamesFromDatabase() const {
    vector<string> names;
    while (sqlite3_step(getCloudNamesStmt) == SQLITE_ROW) {
        names.emplace_back(reinterpret_cast<const char*>(sqlite3_column_text(getCloudNamesStmt, 0)));
    }
    return names;
}

void Database::addPointCloudToDatabase(const string& name, PointCloudT::Ptr cloud) {
    sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);
    int ind = getPointCloudIndexByName(name);
    for (PointType p : cloud->points) {
        sqlite3_bind_int(insertCloudStmt, 1, ind);
        sqlite3_bind_double(insertCloudStmt, 2, p.x);
        sqlite3_bind_double(insertCloudStmt, 3, p.y);
        sqlite3_bind_double(insertCloudStmt, 4, p.z);
        sqlite3_bind_int(insertCloudStmt, 5, p.r);
        sqlite3_bind_int(insertCloudStmt, 6, p.g);
        sqlite3_bind_int(insertCloudStmt, 7, p.b);
        sqlite3_bind_double(insertCloudStmt, 8, p.normal_x);
        sqlite3_bind_double(insertCloudStmt, 9, p.normal_y);
        sqlite3_bind_double(insertCloudStmt, 10, p.normal_z);

        runAndHandleError(insertCloudStmt);

    }
    sqlite3_exec(db, "END TRANSACTION;", nullptr, nullptr, nullptr);
}

PointCloudT::Ptr Database::getPointCloudFromDatabase(const string& name) const {
    int ind = getPointCloudIndexByName(name);
    sqlite3_bind_int(getCloudStmt, 1, ind);

    PointCloudT::Ptr pointCloud { make_shared<PointCloudT>() };
    while (sqlite3_step(getCloudStmt) == SQLITE_ROW) {
        PointType point { sqlite3_column_double(getCloudStmt, 0), sqlite3_column_double(getCloudStmt, 1), sqlite3_column_double(getCloudStmt, 2),
                          sqlite3_column_int(getCloudStmt, 3),    sqlite3_column_int(getCloudStmt, 4),    sqlite3_column_int(getCloudStmt, 5),
                          sqlite3_column_double(getCloudStmt, 6), sqlite3_column_double(getCloudStmt, 7), sqlite3_column_double(getCloudStmt, 8)};
        pointCloud->points.push_back(point);
    }

    sqlite3_reset(getCloudStmt);
    sqlite3_clear_bindings(getCloudStmt);
    return pointCloud;
}

void Database::addPointCloudPropertiesToDatabase(const string& name, const EditCloudData& data) {
    sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);
    int ind = getPointCloudIndexByName(name);
    sqlite3_bind_int(insertCloudPropertiesStmt, 1, ind);
    sqlite3_bind_int(insertCloudPropertiesStmt, 2, data.isFilled);
    sqlite3_bind_int(insertCloudPropertiesStmt, 3, data.areNormalsShown);
    sqlite3_bind_int(insertCloudPropertiesStmt, 4, data.rgb[0]);
    sqlite3_bind_int(insertCloudPropertiesStmt, 5, data.rgb[1]);
    sqlite3_bind_int(insertCloudPropertiesStmt, 6, data.rgb[2]);
    sqlite3_bind_int(insertCloudPropertiesStmt, 7, data.rotation[0]);
    sqlite3_bind_int(insertCloudPropertiesStmt, 8, data.rotation[1]);
    sqlite3_bind_int(insertCloudPropertiesStmt, 9, data.rotation[2]);
    sqlite3_bind_int(insertCloudPropertiesStmt, 10, data.dim.size() >= 1 ? data.dim.at(0) : 0);
    sqlite3_bind_int(insertCloudPropertiesStmt, 11, data.dim.size() >= 2 ? data.dim.at(1) : 0);
    sqlite3_bind_int(insertCloudPropertiesStmt, 12, data.dim.size() >= 3 ? data.dim.at(2) : 0);
    sqlite3_bind_double(insertCloudPropertiesStmt, 13, data.density);
    runAndHandleError(insertCloudPropertiesStmt);
    sqlite3_exec(db, "END TRANSACTION;", nullptr, nullptr, nullptr);
}

EditCloudData Database::getPointCloudPropertiesFromDatabase(const string& name) const {
    int ind = getPointCloudIndexByName(name);
    sqlite3_reset(getCloudPropertiesStmt);
    sqlite3_clear_bindings(getCloudPropertiesStmt);
    EditCloudData data;
    sqlite3_bind_int(getCloudPropertiesStmt, 1, ind);
    if (sqlite3_step(getCloudPropertiesStmt) == SQLITE_ROW) {
        data.isFilled = sqlite3_column_int(getCloudPropertiesStmt, 0);
        data.areNormalsShown = sqlite3_column_int(getCloudPropertiesStmt, 1);
        data.rgb.emplace_back(sqlite3_column_int(getCloudPropertiesStmt, 2));
        data.rgb.emplace_back(sqlite3_column_int(getCloudPropertiesStmt, 3));
        data.rgb.emplace_back(sqlite3_column_int(getCloudPropertiesStmt, 4));
        data.rotation.emplace_back(sqlite3_column_int(getCloudPropertiesStmt, 5));
        data.rotation.emplace_back(sqlite3_column_int(getCloudPropertiesStmt, 6));
        data.rotation.emplace_back(sqlite3_column_int(getCloudPropertiesStmt, 7));
        data.dim.emplace_back(sqlite3_column_int(getCloudPropertiesStmt, 8));
        data.dim.emplace_back(sqlite3_column_int(getCloudPropertiesStmt, 9));
        data.dim.emplace_back(sqlite3_column_int(getCloudPropertiesStmt, 10));
        data.density = sqlite3_column_double(getCloudPropertiesStmt, 11);
        return data;
    }
    return EditCloudData();
}

void Database::addPointCloudTransformationToDatabase(const string& name, const Eigen::Vector3f& translation, const Eigen::Affine3f& rotMatrix) {
    sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);
    int ind = getPointCloudIndexByName(name);
//cloudId, translationX, translationY, translationZ, rotMatrix00,  rotMatrix01,  rotMatrix02,  rotMatrix03,  rotMatrix10,  rotMatrix11,  rotMatrix12,  rotMatrix13,  rotMatrix20,  rotMatrix21,  rotMatrix22,  rotMatrix23
    sqlite3_bind_int(insertCloudTransformationStmt, 1, ind);
    sqlite3_bind_double(insertCloudTransformationStmt, 2, translation[0]);
    sqlite3_bind_double(insertCloudTransformationStmt, 3, translation[1]);
    sqlite3_bind_double(insertCloudTransformationStmt, 4, translation[2]);
    sqlite3_bind_double(insertCloudTransformationStmt, 5, rotMatrix(0,0));
    sqlite3_bind_double(insertCloudTransformationStmt, 6, rotMatrix(0,1));
    sqlite3_bind_double(insertCloudTransformationStmt, 7, rotMatrix(0,2));
    sqlite3_bind_double(insertCloudTransformationStmt, 8, rotMatrix(0,3));
    sqlite3_bind_double(insertCloudTransformationStmt, 9, rotMatrix(1,0));
    sqlite3_bind_double(insertCloudTransformationStmt, 10, rotMatrix(1,1));
    sqlite3_bind_double(insertCloudTransformationStmt, 11, rotMatrix(1,2));
    sqlite3_bind_double(insertCloudTransformationStmt, 12, rotMatrix(1,3));
    sqlite3_bind_double(insertCloudTransformationStmt, 13, rotMatrix(2,0));
    sqlite3_bind_double(insertCloudTransformationStmt, 14, rotMatrix(2,1));
    sqlite3_bind_double(insertCloudTransformationStmt, 15, rotMatrix(2,2));
    sqlite3_bind_double(insertCloudTransformationStmt, 16, rotMatrix(2,3));
    runAndHandleError(insertCloudTransformationStmt);
    sqlite3_exec(db, "END TRANSACTION;", nullptr, nullptr, nullptr);
}

tuple<Eigen::Vector3f, Eigen::Affine3f> Database::getPointCloudTransformationFromDatabase(const string& name) {
    int ind = getPointCloudIndexByName(name);
    sqlite3_reset(getCloudTransformationStmt);
    sqlite3_clear_bindings(getCloudTransformationStmt);

    Eigen::Vector3f translation { Eigen::Vector3f::Zero() };
    Eigen::Affine3f rotMatrix { Eigen::Affine3f::Identity() };
    sqlite3_bind_int(getCloudTransformationStmt, 1, ind);
    if (sqlite3_step(getCloudTransformationStmt) == SQLITE_ROW) {
        translation[0] = sqlite3_column_double(getCloudTransformationStmt, 0);
        translation[1] = sqlite3_column_double(getCloudTransformationStmt, 1);
        translation[2] = sqlite3_column_double(getCloudTransformationStmt, 2);
        rotMatrix(0,0) = sqlite3_column_double(getCloudTransformationStmt, 3);
        rotMatrix(0,1) = sqlite3_column_double(getCloudTransformationStmt, 4);
        rotMatrix(0,2) = sqlite3_column_double(getCloudTransformationStmt, 5);
        rotMatrix(0,3) = sqlite3_column_double(getCloudTransformationStmt, 6);
        rotMatrix(1,0) = sqlite3_column_double(getCloudTransformationStmt, 7);
        rotMatrix(1,1) = sqlite3_column_double(getCloudTransformationStmt, 8);
        rotMatrix(1,2) = sqlite3_column_double(getCloudTransformationStmt, 9);
        rotMatrix(1,3) = sqlite3_column_double(getCloudTransformationStmt, 10);
        rotMatrix(2,0) = sqlite3_column_double(getCloudTransformationStmt, 11);
        rotMatrix(2,1) = sqlite3_column_double(getCloudTransformationStmt, 12);
        rotMatrix(2,2) = sqlite3_column_double(getCloudTransformationStmt, 13);
        rotMatrix(2,3) = sqlite3_column_double(getCloudTransformationStmt, 14);
        return tuple(translation, rotMatrix);
    }
    return tuple(translation, rotMatrix);
}

Database::~Database() {
    if (insertCloudStmt) sqlite3_finalize(insertCloudStmt);
    if (getCloudStmt) sqlite3_finalize(getCloudStmt);
    if (insertCloudIdStmt) sqlite3_finalize(insertCloudIdStmt);
    if (getCloudIndexStmt) sqlite3_finalize(getCloudIndexStmt);
    if (getCloudNamesStmt) sqlite3_finalize(getCloudNamesStmt);
    if (insertCloudPropertiesStmt) sqlite3_finalize(insertCloudPropertiesStmt);
    if (getCloudPropertiesStmt) sqlite3_finalize(getCloudPropertiesStmt);
    if (insertCloudTransformationStmt) sqlite3_finalize(insertCloudTransformationStmt);
    if (getCloudTransformationStmt) sqlite3_finalize(getCloudTransformationStmt);
    sqlite3_close(db);
}

