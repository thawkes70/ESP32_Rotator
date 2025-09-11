// Add to your main .ino file or create WebLogger.h

#ifndef WEBLOGGER_H
#define WEBLOGGER_H
#define WEB_LOG_WARN
#include <Arduino.h>
#include <vector>

enum LogLevel {
  LOG_DEBUG = 0,
  LOG_INFO = 1,
  LOG_WARNING = 2,
  LOG_ERROR = 3
};

struct LogEntry {
  unsigned long timestamp;
  LogLevel level;
  String message;
  String source;
};

class WebLogger {
private:
  std::vector<LogEntry> logs;
  const size_t maxLogs = 100;  // Keep last 100 log entries
  bool serialEnabled = true;   // Can disable serial for performance

public:
  void log(LogLevel level, const String& source, const String& message) {
    LogEntry entry;
    entry.timestamp = millis();
    entry.level = level;
    entry.message = message;
    entry.source = source;
    
    // Add to vector
    logs.push_back(entry);
    
    // Remove old entries if we exceed maxLogs
    if (logs.size() > maxLogs) {
      logs.erase(logs.begin());
    }
    
    // Optional: still send to Serial for debugging
    if (serialEnabled) {
      String levelStr = getLevelString(level);
      Serial.printf("[%lu] %s [%s]: %s\n", 
                    entry.timestamp, levelStr.c_str(), 
                    source.c_str(), message.c_str());
    }
  }
  
  void debug(const String& source, const String& message) {
    log(LOG_DEBUG, source, message);
  }
  
  void info(const String& source, const String& message) {
    log(LOG_INFO, source, message);
  }
  
  void warning(const String& source, const String& message) {
    log(LOG_WARNING, source, message);
  }
  
  void error(const String& source, const String& message) {
    log(LOG_ERROR, source, message);
  }
  
  String getLogsAsJSON() {
    String json = "[";
    for (size_t i = 0; i < logs.size(); i++) {
      if (i > 0) json += ",";
      json += "{";
      json += "\"timestamp\":" + String(logs[i].timestamp) + ",";
      json += "\"level\":" + String(logs[i].level) + ",";
      json += "\"levelName\":\"" + getLevelString(logs[i].level) + "\",";
      json += "\"source\":\"" + logs[i].source + "\",";
      json += "\"message\":\"" + logs[i].message + "\"";
      json += "}";
    }
    json += "]";
    return json;
  }
  
  void enableSerial(bool enable) {
    serialEnabled = enable;
  }
  
  void clearLogs() {
    logs.clear();
  }
  
  size_t getLogCount() {
    return logs.size();
  }

private:
  String getLevelString(LogLevel level) {
    switch(level) {
      case LOG_DEBUG: return "DEBUG";
      case LOG_INFO: return "INFO";
      case LOG_WARNING: return "WARN";
      case LOG_ERROR: return "ERROR";
      default: return "UNKNOWN";
    }
  }
};

// Global logger instance
extern WebLogger webLogger;

// Convenience macros for easy logging
#define WEB_LOG_DEBUG(source, message) webLogger.debug(source, message)
#define WEB_LOG_INFO(source, message) webLogger.info(source, message)
#define WEB_LOG_WARNING(source, message) webLogger.warning(source, message)
#define WEB_LOG_ERROR(source, message) webLogger.error(source, message)

// printf-style macros
#define WEB_LOG_DEBUGF(source, format, ...) do { \
  char buffer[256]; \
  snprintf(buffer, sizeof(buffer), format, ##__VA_ARGS__); \
  webLogger.debug(source, String(buffer)); \
} while(0)

#define WEB_LOG_INFOF(source, format, ...) do { \
  char buffer[256]; \
  snprintf(buffer, sizeof(buffer), format, ##__VA_ARGS__); \
  webLogger.info(source, String(buffer)); \
} while(0)

#define WEB_LOG_WARNINGF(source, format, ...) do { \
  char buffer[256]; \
  snprintf(buffer, sizeof(buffer), format, ##__VA_ARGS__); \
  webLogger.warning(source, String(buffer)); \
} while(0)

#define WEB_LOG_ERRORF(source, format, ...) do { \
  char buffer[256]; \
  snprintf(buffer, sizeof(buffer), format, ##__VA_ARGS__); \
  webLogger.error(source, String(buffer)); \
} while(0)

#endif
