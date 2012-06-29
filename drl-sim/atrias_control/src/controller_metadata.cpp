/*
 * controller_metadata.cpp
 *
 *  Created on: May 13, 2012
 *      Author: michael
 */

#include <atrias_control/controller_metadata.h>

controller_metadata loadControllerMetadata(string path, string packageName, bool formatName) {
	using namespace metadata;

	string line;
	string key;
	string value;
	controller_metadata result;

	if (formatName)
		result.name = cleanUpPackageName(packageName);
	else
		result.name = packageName;
	result.description = string("No description");
	result.version = string("Unknown version");
	result.author = string("Unknown author");
	result.coreLibPath = path + "/lib/libcontroller_core.so";
	result.guiLibPath = path + "/lib/libcontroller_gui.so";
	result.guiDescriptionPath = path + "/controller_gui.glade";
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
			} else if (key == "corelibpath") {
				value = getValue(line);
				trim(value);
				result.coreLibPath = value;
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

string metadata::getKey(string line) {
	size_t t = line.find('=');
	if (t != string::npos)
		return line.substr(0, t);
	else
		return string("");
}

string metadata::getValue(string line) {
	size_t t = line.find('=');
	if (t != string::npos)
		return line.substr(t + 1, string::npos);
	else
		return string("");
}

string metadata::cleanUpPackageName(string packageName) {
	//Remove ac_ prefix
	if (packageName.find("ac_") == 0) {
		packageName = packageName.substr(3, string::npos);
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

void metadata::toLowercase(string *str) {
	std::transform(str->begin(), str->end(), str->begin(), ::tolower);
}


// trim from both ends
inline void metadata::trim(std::string &s) {
	ltrim(s);
	rtrim(s);
}

// trim from start
inline void metadata::ltrim(std::string &s) {
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
}

// trim from end
inline void metadata::rtrim(std::string &s) {
	s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
}


int metadata::isWhiteSpace(int i) {
	if (i == ' ' || i == '\t')
		return 1;

	return 0;
}
