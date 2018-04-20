#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iterator>
#include <vector>

#include "config_parser.h"

using namespace std;

static string trim(string s) {
    const auto sBegin = s.find_first_not_of(" \t");
    if (sBegin == string::npos) {
        return "";
    }
    
    const auto sEnd = s.find_last_not_of(" \t");
    const auto sRange = sEnd - sBegin + 1;
    
    return s.substr(sBegin, sRange);
}

void ConfigParser::parse(const char* filename) {
    tokens.clear();
    
    ifstream is_file(filename);
    string line_comment;
    string line;
    int lineNumber = 1;
    while (getline(is_file, line_comment)) {
        line_comment = trim(line_comment);
        istringstream is_line_comment(line_comment);
        getline(is_line_comment, line, '#');

        line = trim(line);
        istringstream is_line(line);
        string key;
		getline(is_line, key, '=');
        if (!is_line.eof()) {
            key = trim(key);
            string value;
            if (getline(is_line, value)) {
                value = trim(value);
                tokens.insert(pair<string, string>(key, value));
            } else {
                cerr << "Error parsing line " << lineNumber << endl;
            }
        }
        lineNumber++;
    }
}

const char* ConfigParser::getString(const char* key) {
    string k(key, strlen(key));
    if (tokens.find(k) == tokens.end()) {
        cerr << "No such key \"" << k << "\"" << endl;
        return NULL;
    }
    string v = tokens[k];
    return v.c_str();
}

int ConfigParser::getInt(const char* key) {
    string k(key, strlen(key));
    if (tokens.find(k) == tokens.end()) {
        cerr << "No such key \"" << k << "\"" << endl;
        return -1;
    }
    string v = tokens[k];
    int iv;
    sscanf(v.c_str(), "%d", &iv);
    return iv;
}

double ConfigParser::getDouble(const char* key) {
    string k(key, strlen(key));
    if (tokens.find(k) == tokens.end()) {
        cerr << "No such key \"" << k << "\"" << endl;
        return -1;
    }
    string v = tokens[k];
    double dv;
    sscanf(v.c_str(), "%lf", &dv);
    return dv;
}

static vector<string> &split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

cv::Mat ConfigParser::getMatrix(const char* key) {
    string k(key, strlen(key));
    if (tokens.find(k) == tokens.end()) {
        cerr << "No such key \"" << k << "\"" << endl;
        return cv::Mat();
    }
    string v = tokens[k];
    int nRows = count(v.begin(), v.end(), '|') + 1;
    int nCols = count(v.begin(), v.end(), ',') / nRows + 1;

    cv::Mat m = cv::Mat_<float>(nRows, nCols);

    vector<string> rows;
    split(v, '|', rows);

    int nc = -1;

    for (int i = 0; i < rows.size(); i++) {
        vector<string> cols;
        split(rows[i], ',', cols);
        if (nc != -1 && cols.size() != nc) {
            cerr << "Incorrect matrix notation." << endl;
            return cv::Mat();
        }
        nc = cols.size();

        for (int j = 0; j < cols.size(); j++) {
            float f;
            sscanf(cols[j].c_str(), "%f", &f);
            m.at<float>(i, j) = f;
        }
    }
    
    return m;
}
