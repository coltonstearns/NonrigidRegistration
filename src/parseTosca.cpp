#include <iostream>
#include <fstream>
#include <string>
#include <sstream>      // std::istringstream
#include <vector>
#include <array>

// ============ THESE TWO HEADERS NEED TO BE IN THIS ORDER! ===============
#include <pcl/io/pcd_io.h>  // for working with point cloud data structures
#include <pcl/impl/point_types.hpp>  // for defining point objects
// =========================================================================

#include "parseTosca.h"

// define local functions
bool hasEnding(std::string const &fullString, std::string const &ending);
pcl::PointCloud<pcl::PointXYZ>::Ptr parseToscaDataFile(char* filepath);


catData generateCatPointCloud() {
    char cat_baseline_path[] = "/home/colton/Documents/6.838 Final Project/test/TOSCA/wolf1_short.vert";
    char cat_transformed_path[] = "/home/colton/Documents/6.838 Final Project/test/TOSCA/wolf0_short.vert";
    catData vertex_data;
    vertex_data.source = parseToscaDataFile(cat_baseline_path);
    vertex_data.target = parseToscaDataFile(cat_transformed_path);
    return vertex_data;
}

/*
Given a Tsoca Ascii data file path, parses the data.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr parseToscaDataFile(char* filepath) {
    //initialize return value array
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // check to see if our file is a .vert file
    if (hasEnding(filepath, ".vert")) {
        std::string line;
        std::ifstream datafile(filepath);

        if (datafile.is_open())
        {
            while (std::getline(datafile, line))
            {
                std::istringstream iss(line);
                double x, y, z;
                iss >> x >> y >> z;
                pcl::PointXYZ point = pcl::PointXYZ(x, y, z);
                cloud->push_back(point);
            }
            datafile.close();
        }
    }

    else {
        std::cout << "File needs to end in .vert" << std::endl;
        }

    return cloud;
}


bool hasEnding(std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
    }
    else {
        return false;
    }
}










// // ============= Windows Directory Search ===============
// /*
// Given a directory path, goes through and parses all .vert ascii files into
// data.
// */
// std::vector<std::array<double, 3>> parseToscaData(wchar_t dirpath[]) {
//     //initialize return value array
//     std::vector<std::array<double, 3>> file_data_points;

//     // convert wchar to char* for future use
//     size_t dirReturnValue;
//     size_t dirSizeInBytes = wcslen(dirpath);
//     char char_dirpath[MAX_PATH];
//     wcstombs_s(&dirReturnValue, char_dirpath, dirSizeInBytes + 1, dirpath, MAX_PATH);
//     std::string str_dirpath = std::string(char_dirpath);
//     str_dirpath.pop_back(); //remove the * at the end

//     WIN32_FIND_DATA ffd;

//     // using windows name, find the directory
//     HANDLE hFind = FindFirstFile(dirpath, &ffd);

//     // check if invalid directory
//     if (INVALID_HANDLE_VALUE == hFind){
//         std::cout << "ERROR" << std::endl;
//         return file_data_points;
//     }

//     // Go through each file and extract data if it is the appropriate .vert file
//     do
//     {
//         if (~(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) { // if it's not a directory
//             //convert file name to char instead of wide char
//             const wchar_t* wide_char_filename = ffd.cFileName;
//             size_t pReturnValue;
//             //size_t sizeInBytes = sizeof(wide_char_filename);
//             size_t sizeInBytes = wcslen(wide_char_filename);
//             char filename[MAX_PATH];
//             wcstombs_s(&pReturnValue, filename, sizeInBytes+1, wide_char_filename, MAX_PATH);

//             // convert to string
//             std::string str_filename = str_dirpath + std::string(filename);
//             std::cout << str_filename << std::endl;

//             // check to see if our file is a .vert file
//             if (hasEnding(str_filename, ".vert")) {
//                 std::string line;
//                 std::ifstream datafile(str_filename); 

//                 if (datafile.is_open())
//                 {
//                     std::vector<std::array<double, 3>> data_points;
//                     while (std::getline(datafile, line))
//                     {
//                         std::istringstream iss(line);
//                         std::array<double, 3> point;
//                         for (int i = 0; i < 3; i++)
//                         {
//                             double coordinate;
//                             iss >> coordinate;
//                             point[i] = coordinate;
//                         }
//                         //std::cout << point[0] << "  " << point[1] << "  " << point[2] << "  " << std::endl;
//                         data_points.push_back(point);
//                     }
//                     datafile.close();
//                     file_data_points = data_points;

//                 }
//             }


//         }

//     } while (FindNextFile(hFind, &ffd) != 0);

//     FindClose(hFind);

//     return file_data_points;
// }












///*
//============= Linux Directory Search ================
//*/
//
//#include <fstream>
//#include <dirent> //contains opendir() and closedir() functions
//#include <libgen>
//
//double* data parseToscaData(char*[] dirpath) {
//    std::DIR* dir;
//    struct dirent *dir_pntr; //dirent contains DIR, and thus this defines DIR to be dir_pntr type, which is a single pointer
//
//    if ((dir = opendir(dirpath)) != NULL) {
//        /* print all the files and directories within directory */
//        while ((dir_pntr = readdir(dir)) != NULL) {
//            printf("%s\n", dir_pntr -> d_name);
//        }
//        closedir(dir);
//    }
//    else {
//        /* could not open directory */
//        perror("");
//        return EXIT_FAILURE;
//    }
//
//    DIR* data_directory = std::opendir(dirpath);
//    std::ifstream;
//
//    int x = 10;
//    return x * ;
//}
