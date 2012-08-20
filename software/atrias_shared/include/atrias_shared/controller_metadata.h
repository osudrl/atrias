/*
 * controller_metadata.h
 *
 *  Created on: May 13, 2012
 *      Author: michael
 */

#ifndef CONTROLLER_METADATA_H_
#define CONTROLLER_METADATA_H_

#include <string.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <functional>
#include <locale>

using namespace std;

namespace atrias {
namespace controllerMetadata {

typedef struct {
    string name;
    string description;
    string version;
    string author;
    string startScriptPath;
    string stopScriptPath;
    string guiLibPath;
    string guiDescriptionPath;
    string guiConfigPath;
    string guiTabWidgetName;
    bool loadSuccessful; //Set to true if metadata.txt was loaded properly
} ControllerMetadata;

ControllerMetadata loadControllerMetadata(string path, string packageName = string("Unknown Controller"), bool formatName = true);

string getKey(string line);
string getValue(string line);
string cleanUpPackageName(string packageName);
void toLowercase(string *str);
void trim(string &str);
void ltrim(string &str);
void rtrim(string &str);
int isWhiteSpace(int);

}
}

#endif /* CONTROLLER_METADATA_H_ */
