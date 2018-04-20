/* 
 * File:   config_parser.h
 * Author: Karlo
 *
 * Created on November 12, 2015, 5:16 PM
 */

#ifndef CONFIG_PARSER_H
#define	CONFIG_PARSER_H

#include "opencv2/core/core.hpp"
#include <map>
#include <string>

using namespace std;

/**
 * Class which represents the parser for the configuration files.
 */
class ConfigParser {
    map<string, string> tokens;
public:
    
    /**
     * Parses the file specified in the input argument.
     * @param filename Path to the configuration file.
     */
    void parse(const char* filename);
    
    /**
     * Gets cstring value assigned to the specified key.
     * @param key Token key.
     * @return Token value as cstring.
     */
    const char* getString(const char* key);
    
    /**
     * Gets integer value assigned to the specified key.
     * @param key Token key.
     * @return Token value as integer.
     */
    int getInt(const char* key);
    
    /**
     * Gets double value assigned to the specified key.
     * @param key Token key.
     * @return Token value as double.
     */
    double getDouble(const char* key);
    
    /**
     * Gets matrix assigned to the specified key.
     * @param key Token key.
     * @return Token value as Mat.
     */
    cv::Mat getMatrix(const char* key);
    
};

#endif	/* CONFIG_PARSER_H */

