#ifndef PLUGIN_DEBUG_LOGGER_HPP
#define PLUGIN_DEBUG_LOGGER_HPP

// C++ system headers
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

// Third-party library headers
#include <Eigen/Core>
#include <fmt/chrono.h>
#include <fmt/color.h>
#include <fmt/core.h>

// Avoid macro pollution
#ifdef INFO
#undef INFO
#endif
#ifdef DEBUG
#undef DEBUG
#endif
#ifdef WARNING
#undef WARNING
#endif
#ifdef ERROR
#undef ERROR
#endif
#ifdef SILENT
#undef SILENT
#endif

namespace debug {

namespace fmt = ::fmt;
namespace fs = std::filesystem;

enum class PrintMode {
    LOG,
    INFO,
    DEBUG,
    WARNING,
    ERROR,
    SILENT
};

// Eigen format for logging
inline const Eigen::IOFormat kLongCsvFmt(
    Eigen::FullPrecision, Eigen::FullPrecision, ", ", ";\n", "[", "]", "\n{", "}");

inline const std::unordered_map<PrintMode, fmt::color> PRINT_COLOR = {
    {PrintMode::LOG, fmt::color::green},
    {PrintMode::INFO, fmt::color::white},
    {PrintMode::WARNING, fmt::color::yellow},
    {PrintMode::ERROR, fmt::color::red},
    {PrintMode::DEBUG, fmt::color::cyan},
};

inline const std::unordered_map<PrintMode, std::string> PRINT_PREFIX = {
    {PrintMode::LOG, "[LOG ]"},
    {PrintMode::INFO, "[INFO]"},
    {PrintMode::WARNING, "[WARN]"},
    {PrintMode::ERROR, "[ERR ]"},
    {PrintMode::DEBUG, "[DBG ]"}
};

/**
 * @brief Logger state singleton - manages all logger state with thread safety
 */
class LoggerState {
public:
    static LoggerState& instance() {
        static LoggerState inst;
        return inst;
    }

    // Session management
    std::string session_path;
    std::string session_timestamp;

    // Log file
    std::ofstream log_file;
    std::mutex file_mutex;

    // Filter settings
    PrintMode min_mode = PrintMode::LOG;

private:
    LoggerState() = default;
    ~LoggerState() {
        if (log_file.is_open()) {
            log_file.close();
        }
    }
    LoggerState(const LoggerState&) = delete;
    LoggerState& operator=(const LoggerState&) = delete;
};

/**
 * @brief Get current time string with microsecond precision
 */
inline std::string get_current_time_string() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000;

    return fmt::format("{:%H:%M:%S}.{:03},{:03}", *std::localtime(&time_t_now), ms.count(), us.count());
}

/**
 * @brief Get timestamp string for filenames
 */
inline std::string get_timestamp_for_filename() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    return fmt::format("{:%Y-%m-%d_%H-%M-%S}", *std::localtime(&time_t_now));
}

/**
 * @brief Initialize a new session with timestamped directory
 * @param suffix 文件夹后缀，比赛模式传 "match" 等
 * @return Session directory path
 *
 * Directory structure: {LOG_DIR}/{timestamp}_{suffix}/ 或 {LOG_DIR}/{timestamp}/
 *   - run.log: log file
 *   - *.avi: video recordings
 */
inline std::string init_session(const std::string& suffix = "") {
    auto& state = LoggerState::instance();
    std::lock_guard<std::mutex> lock(state.file_mutex);

    if (state.log_file.is_open()) {
        state.log_file.close();
    }

    state.session_timestamp = get_timestamp_for_filename();
    if (suffix.empty()) {
        state.session_path = std::string(LOG_DIR) + "/" + state.session_timestamp;
    } else {
        state.session_path = std::string(LOG_DIR) + "/" + state.session_timestamp + "_" + suffix;
    }

    fs::create_directories(state.session_path);

    std::string log_path = state.session_path + "/run.log";
    state.log_file.open(log_path, std::ios::app);

    if (state.log_file.is_open()) {
        state.log_file << fmt::format("=== Session started at {} ===\n", get_current_time_string());
        state.log_file.flush();
    }

    return state.session_path;
}

/**
 * @brief Get current session path for video recording
 */
inline std::string get_session_path() {
    return LoggerState::instance().session_path;
}

inline void close_log_file() {
    auto& state = LoggerState::instance();
    std::lock_guard<std::mutex> lock(state.file_mutex);
    if (state.log_file.is_open()) {
        state.log_file.close();
    }
}

inline void set_min_level(PrintMode mode) {
    LoggerState::instance().min_mode = mode;
}

// Utility functions
template<typename T>
inline auto stream_to_str(T& x) -> std::string {
    std::stringstream buffer;
    buffer << x;
    return buffer.str();
}

template<typename T>
inline auto eigen_to_str(const T& x) -> std::string {
    std::ostringstream oss;
    oss << x.format(kLongCsvFmt);
    return oss.str();
}

template<typename T>
inline auto vec_to_str(const std::vector<T>& vec) -> std::string {
    std::string str = "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        str += fmt::format("{}", vec[i]);
        if (i < vec.size() - 1) str += ", ";
    }
    str += "]";
    return str;
}

inline auto string_to_mode(const std::string& mode_str) -> PrintMode {
    std::string lower_str = mode_str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
    if (lower_str == "log") return PrintMode::LOG;
    if (lower_str == "info") return PrintMode::INFO;
    if (lower_str == "debug") return PrintMode::DEBUG;
    if (lower_str == "warning" || lower_str == "warn") return PrintMode::WARNING;
    if (lower_str == "error" || lower_str == "err") return PrintMode::ERROR;
    return PrintMode::SILENT;
}

/**
 * @brief Print log message
 */
template<typename... T>
inline void print(const PrintMode& mode, const std::string& node_name,
                  const std::string& content, T&&... args) {
    auto& state = LoggerState::instance();

    if (mode < state.min_mode) return;

    std::string timestamp = get_current_time_string();
    std::string formatted_content;
    try {
        if constexpr (sizeof...(args) > 0) {
            formatted_content = fmt::format(fmt::runtime(content), std::forward<T>(args)...);
        } else {
            formatted_content = content;
        }
    } catch (const fmt::format_error& e) {
        formatted_content = content + " [format error: " + e.what() + "]";
    }

    std::string node_str = node_name.empty() ? "" : "@" + node_name;
    std::string full_message = fmt::format("{} {} {}: {}",
                                           timestamp, PRINT_PREFIX.at(mode),
                                           node_str, formatted_content);

    fmt::print(fmt::fg(PRINT_COLOR.at(mode)), "{}\n", full_message);

    {
        std::lock_guard<std::mutex> lock(state.file_mutex);
        if (state.log_file.is_open()) {
            state.log_file << full_message << "\n";
            state.log_file.flush();
        }
    }
}

template<typename... T>
inline void print(const std::string& mode_str, const std::string& node_name,
                  const std::string& content, T&&... args) {
    print(string_to_mode(mode_str), node_name, content, std::forward<T>(args)...);
}

} // namespace debug

#endif // PLUGIN_DEBUG_LOGGER_HPP
