/*
 * ControllerMetadata.cpp
 *
 *  Created on: May 13, 2012
 *      Author: Michael Anderson
 */

#include <atrias_shared/controller_metadata.h>

namespace atrias {
namespace controllerMetadata {

ControllerMetadata loadControllerMetadata(string path, string packageName, bool formatName) {
	string line;
	string key;
	string value;
	ControllerMetadata result;

	if (formatName)
		result.name = cleanUpPackageName(packageName);
	else
		result.name = packageName;
	result.description = string("No description");
	result.version = string("Unknown version");
	result.author = string("Unknown author");
	result.startScriptPath = path + "/start.ops";
    result.stopScriptPath = path + "/stop.ops";
	result.guiLibPath = path + "/lib/libcontroller_gui.so";
	result.guiDescriptionPath = path + "/controller_gui.glade";
    result.guiConfigPath = path + "/gui_config.yaml";
	result.guiTabWidgetName = string("controller_tab");
	result.loadSuccessful = false;

	ifstream metadataFile((path + "/controller.txt").c_str(), ios::in);
	if (metadataFile.is_open()) {
		while (metadataFile.good()) {
			getline(metadataFile, line);
			key = getKey(line);
			trim(key);
			toLowercase(&key);
			if (key == "name") {
				value = getValue(line);
				trim(value);
				result.name = value;
			} else if (key == "description") {
				value = getValue(line);
				trim(value);
				result.description = value;
			} else if (key == "version") {
				value = getValue(line);
				trim(value);
				result.version = value;
			} else if (key == "author") {
				value = getValue(line);
				trim(value);
				result.author = value;
			} else if (key == "startscriptpath") {
				value = getValue(line);
				trim(value);
				result.startScriptPath = value;
            } else if (key == "stopscriptpath") {
                value = getValue(line);
                trim(value);
                result.stopScriptPath = value;
			} else if (key == "guilibpath") {
				value = getValue(line);
				trim(value);
				result.guiLibPath = value;
			} else if (key == "guitabwidgetname") {
				value = getValue(line);
				trim(value);
				result.guiTabWidgetName = value;
			} else if (key == "guidescriptionpath") {
				value = getValue(line);
				trim(value);
				result.guiDescriptionPath = value;
			}
		}
		metadataFile.close();
		result.loadSuccessful = true;
	}
	return result;
}

string getKey(string line) {
	size_t t = line.find('=');
	if (t != string::npos)
		return line.substr(0, t);
	else
		return string("");
}

string getValue(string line) {
	size_t t = line.find('=');
	if (t != string::npos)
		return line.substr(t + 1, string::npos);
	else
		return string("");
}

string cleanUpPackageName(string packageName) {
	//Remove ac_ prefix
	if (packageName.find("atc_") == 0) {
		packageName = packageName.substr(strlen("atc_"), string::npos);
	}

	//Remove whitespace on edges
	trim(packageName);

	/*
	 * Remove excess whitespace from inside string and
	 * convert underscores and dashes to spaces
	 *
	 * Also make the first letter of each word upper-case
	 */
	size_t t = 0;
	bool lastCharWasSpace = true;
	while (packageName[t]) {
		if (packageName[t] == '_' || packageName[t] == '-' || packageName[t] == '\t') {
			if (lastCharWasSpace)
				packageName.erase(t, 1);
			else {
				packageName[t] = ' ';
				lastCharWasSpace = true;
			}
		}
		else if (packageName[t] == ' ') {
			if (lastCharWasSpace)
				packageName.erase(t, 1);
		}
		else {
			if (lastCharWasSpace)
				packageName[t] = toupper(packageName[t]);
			lastCharWasSpace = false;
		}
		t++;
	}

	return packageName;
}

void toLowercase(string *str) {
	std::transform(str->begin(), str->end(), str->begin(), ::tolower);
}


// trim from both ends
inline void trim(std::string &s) {
	ltrim(s);
	rtrim(s);
}

// trim from start
inline void ltrim(std::string &s) {
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
}

// trim from end
inline void rtrim(std::string &s) {
	s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
}


int isWhiteSpace(int i) {
	if (i == ' ' || i == '\t')
		return 1;

	return 0;
}

}
}
