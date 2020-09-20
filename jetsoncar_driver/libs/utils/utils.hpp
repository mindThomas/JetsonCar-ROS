/* Copyright (C) 2018-2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#ifndef UTILS_
#define UTILS_

#include <stdio.h>      /* printf */
#include <stdarg.h>     /* va_list, va_start, va_arg, va_end */
#include <stdlib.h>
#include <sstream>
#include <string.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/convenience.hpp>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <sys/time.h>
#include <iomanip>
#include <cmath>
#include <chrono>

// for file access (eg. directory traversing)
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <pwd.h>

#include <curl/curl.h>

/* sys/stat.h provides you with several integers you can bytewise-OR (|) together to create your mode_t:
 * User: S_IRUSR (read), S_IWUSR (write), S_IXUSR (execute)
 * Group: S_IRGRP (read), S_IWGRP (write), S_IXGRP (execute)
 * Others: S_IROTH (read), S_IWOTH (write), S_IXOTH (execute)
 */
int mkdir_recursive(const char* file_path_, mode_t mode = 775) {
    assert(file_path_ && *file_path_);
    char file_path[strlen(file_path_)];
    strcpy(file_path, file_path_);
    char * pfile_path = file_path;
    char* p;
    for (p=strchr(pfile_path+1, '/'); p; p=strchr(p+1, '/')) {
        *p='\0';
        if (mkdir(pfile_path, mode)==-1) {
            if (errno!=EEXIST) { *p='/'; return -1; }
        }
        *p='/';
    }
    return 0;
}

std::string FindFileFromPath(std::string filenameSubpart, std::string searchPath)
{
    DIR *dir;
    struct dirent *ent;
    std::string output;
    if ((dir = opendir (searchPath.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_type != DT_DIR) {
                if (strstr(ent->d_name, filenameSubpart.c_str()) != NULL) {
                    output = searchPath.substr(0, searchPath.find_last_of("/")) + "/" + std::string(ent->d_name);
                    return output;
                }
            }
        }
        closedir (dir);
    } else { // could not open directory
        printf("Error: Could not open directory: %s\n", searchPath.c_str());
    }

    return output;
}

std::vector<std::string> ListOfFilesPattern(std::string path, std::string pattern)
{
    DIR *dir;
    struct dirent *ent;
    std::string output;
    std::vector<std::string> files;

    if ((dir = opendir (path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_type != DT_DIR) {
                if (strstr(ent->d_name, pattern.c_str()) != NULL) {
                    files.push_back(std::string(ent->d_name));
                }
            }
        }
        closedir (dir);
    } else { // could not open directory
        printf("Error: Could not open directory: %s\n", path.c_str());
    }

    std::sort(std::begin(files), std::end(files));

    return files;
}

std::vector<std::string> ListOfFiles(std::string path)
{
    DIR *dir;
    struct dirent *ent;
    std::string output;
    std::vector<std::string> files;

    if ((dir = opendir (path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_type != DT_DIR) {
                files.push_back(std::string(ent->d_name));
            }
        }
        closedir (dir);
    } else { // could not open directory
        printf("Error: Could not open directory: %s\n", path.c_str());
    }

    std::sort(std::begin(files), std::end(files));

    return files;
}

std::string SystemCall(std::string cmd) {

    std::string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
        pclose(stream);
    }
    return data;
}

inline bool PathExist(const std::string& path) {
    struct stat sb;
    return (stat(path.c_str(), &sb) == 0);
}

inline bool FileExist(const std::string& path) {
    struct stat sb;
    return (stat(path.c_str(), &sb) == 0 && S_ISREG(sb.st_mode));
}

inline bool FolderExist(const std::string& path) {
    struct stat sb;
    return (stat(path.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)); // folder exists
}

inline bool DeleteFile(const std::string& path) {
    if (remove(path.c_str()) != 0)
        return false; // error
    else
        return true;
}

/*
 * Get File extension from File path or File Name
 */
std::string getFileExtension(std::string filePath)
{
    // Create a Path object from given string
    boost::filesystem::path pathObj(filePath);
    // Check if file name in the path object has extension
    if (pathObj.has_extension()) {
        // Fetch the extension from path object and return
        return pathObj.extension().string();
    }
    // In case of no extension return empty string
    return "";
}

inline std::string getFileNameWithExtensionFromPath(std::string filePath)
{
    return boost::filesystem::path(filePath).filename().string();
}

inline std::string getFileNameFromPath(std::string filePath)
{
    return boost::filesystem::path(filePath).stem().string();
}

inline std::string getPathFromFilePath(std::string filePath)
{
    return boost::filesystem::path(filePath).parent_path().string() + "/";
}

std::string getHomeDirectory()
{
    const char *homedir;

    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    return std::string(homedir);
}

/* Center-aligns string within a field of width w. Pads with blank spaces
    to enforce alignment. */
std::string stringCenter(const std::string s, const int w) {
    std::stringstream ss, spaces;
    int padding = w - s.size();                 // count excess room to pad
    for(int i=0; i<padding/2; ++i)
        spaces << " ";
    ss << spaces.str() << s << spaces.str();    // format with padding
    if(padding>0 && padding%2!=0)               // if odd #, add 1 space
        ss << " ";
    return ss.str();
}

/* Convert double to string with specified number of places after the decimal
   and left padding. */
std::string printDouble(const double x, const int decDigits, const int width) {
    std::stringstream ss;
    ss << std::fixed << std::right;
    ss.fill(' ');        // fill space around displayed #
    ss.width(width);     // set  width around displayed #
    ss.precision(decDigits); // set # places after decimal
    ss << x;
    return ss.str();
}

/* Convert double to string with specified number of places after the decimal
   and left padding. */
std::string stringRight(const std::string s, const int w) {
    std::stringstream ss;
    ss << std::fixed << std::right;
    ss.fill(' ');        // fill space around displayed #
    ss.width(w);     // set  width around displayed #
    ss << s;
    return ss.str();
}

/*
static int64_t utime_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

static void write_i32(uint8_t *buf, int32_t v)
{
    buf[0] = (v >> 24) & 0xFF;
    buf[1] = (v >> 16) & 0xFF;
    buf[2] = (v >>  8) & 0xFF;
    buf[3] = (v      ) & 0xFF;
}

static void write_i64(uint8_t *buf, int64_t v)
{
    uint32_t h = (uint32_t) (v >> 32);
    uint32_t l = (uint32_t) (v);

    write_i32(buf+0, h);
    write_i32(buf+4, l);
}
*/

/*size_t writeDataOnStream(void * buffer, size_t size, size_t nbytes, void * stream){
    size_t bytes_written = fwrite( buffer, size, nbytes, (FILE *) stream);
    return bytes_written;
}*/
typedef struct MemoryStruct {
    char *memory;
    size_t size;
} MemoryStruct;

static size_t
WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp);

int CURL_Download(std::string URL, std::string filePath);

void error(const char * appName, const char * fmt, ...)
{
    int n;
    int size = 100;     /* Guess we need no more than 100 bytes */
    char *p, *np;
    va_list args;

    if ((p = (char *)malloc(size)) == NULL)
        return;

    while (1) {
        /* Try to print in the allocated space */
        va_start(args, fmt);
        n = vsnprintf(p, size, fmt, args);
        va_end(args);

        /* Check error code */
        if (n < 0)
            return;

        /* If that worked, use the string */
        if (n < size)
            break;

        /* Else try again with more space */
        size = n + 1;       /* Allocate exactly what is needed */

        if ((np = (char *)realloc(p, size)) == NULL) {
            free(p);
            return;
        } else {
            p = np;
        }
    }

    printf("Error: %s", p);

    std::string notifyCmd = "notify-send '" + std::string(appName) + "' '" + std::string(p) + "'";
    system(notifyCmd.c_str());

    free(p);
}

void error(std::string appName, const char * fmt, ...)
{
    va_list args;

    va_start(args, fmt);

    error(appName.c_str(), fmt, args);

    va_end(args);
}

void textPopup(std::string text)
{
    std::string popupCmd = "yad --text '" + std::string(text) + "'";
    std::cout << text << std::endl;
    system(popupCmd.c_str());
}

float Parse2Float(std::string str)
{
    float value;
    try {
        value = std::stof(str);
    }
    catch (std::invalid_argument &e) {
        return 0;
    }  // value could not be parsed (eg. is not a number)
    catch (std::out_of_range &e) {
        return 0;
    }
    return value;
}

float Parse2RoundedFloat(std::string str)
{
    float value;
    try {
        value = std::stof(str);
        value = roundf(value * 1000) / 1000; // round to 3 decimals
    }
    catch (std::invalid_argument &e) {
        return 0;
    }  // value could not be parsed (eg. is not a number)
    catch (std::out_of_range &e) {
        return 0;
    }
    return value;
}

int Parse2Int(std::string str)
{
    int value;
    try {
        value = std::stoi(str);
    }
    catch (std::invalid_argument &e) {
        return 0;
    }  // value could not be parsed (eg. is not a number)
    catch (std::out_of_range &e) {
        return 0;
    }
    return value;
}

bool Parse2Bool(std::string str)
{
    if (!str.compare("true"))
        return true;
    else if (!str.compare("false"))
        return false;
    else {
        return false;
    }
}

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}


long int GetCurrentMicroseconds()
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long int us = tp.tv_sec * 1000000 + tp.tv_usec;
    return us;
}

std::chrono::milliseconds GetCurrentMicrosecondsChrono()
{
    using namespace std::chrono;
    milliseconds ms = duration_cast< milliseconds >(
            system_clock::now().time_since_epoch()
    );
}

void WriteFormattedTimestamp(std::string header, int64_t utime)
{
    std::time_t seconds = utime / 1000000;
    int milliseconds = (utime % 1000000) / 1000;
    std::tm* t = std::gmtime(&seconds); // std::localtime
    std::cout << header << ": ";
    std::cout << std::put_time(t, "%Y-%m-%d %H:%M:%S");
    printf(".%03d\n", milliseconds);
}

std::string GetFormattedTimestamp(int64_t utime)
{
    std::string output;
    std::stringstream ss;

    std::time_t seconds = utime / 1000000;
    int milliseconds = (utime % 1000000) / 1000;
    std::tm* t = std::localtime(&seconds);

    ss << std::put_time(t, "%Y-%m-%d_%H-%M-%S");
    ss << boost::format("-%03d") % milliseconds;
    output = ss.str();

    return output;
}

std::string GetFormattedTimestampCurrent()
{
    std::string output;
    std::stringstream ss;

    int64_t utime = GetCurrentMicroseconds();

    std::time_t seconds = utime / 1000000;
    int milliseconds = (utime % 1000000) / 1000;
    std::tm* t = std::localtime(&seconds);

    ss << std::put_time(t, "%Y-%m-%d_%H-%M-%S");
    ss << boost::format("-%03d") % milliseconds;
    output = ss.str();

    return output;
}

std::string GetLogFormattedTimestamp(int64_t utime)
{
    std::string output;
    std::stringstream ss;

    std::time_t seconds = utime / 1000000;
    int milliseconds = (utime % 1000000) / 1000;
    std::tm* t = std::gmtime(&seconds); // std::localtime

    ss << std::put_time(t, "%Y-%m-%d_%H-%M-%S");
    ss << boost::format("-%03d") % milliseconds;
    output = ss.str();

    return output;
}

int64_t GetTimestampFromImageFilename(std::string filename)
{
    std::tm t;
    std::istringstream ss(filename.substr(0, 19));
    std::istringstream ss2(filename.substr(20, 3));
    int milliseconds;
    int64_t utime;

    ss >> std::get_time(&t, "%Y-%m-%d_%H-%M-%S.jpg");
    if (ss.fail()) return 0;

    std::time_t seconds = std::mktime(&t);
    if (!(ss2 >> milliseconds)) return 0;

    utime = 1000000 * seconds + 1000 * milliseconds;
    return utime;
}


#endif
