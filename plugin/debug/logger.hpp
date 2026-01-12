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
    DEBUG,    // 调试信息
    INFO,     // 一般信息
    WARNING,  // 警告，但可继续
    ERROR,    // 错误，可能影响功能
    FATAL,    // 致命错误，程序终止
    SILENT    // 静默（过滤用）
};

// Eigen format for logging
inline const Eigen::IOFormat kLongCsvFmt(
    Eigen::FullPrecision, Eigen::FullPrecision, ", ", ";\n", "[", "]", "\n{", "}");

inline const std::unordered_map<PrintMode, fmt::color> PRINT_COLOR = {
    {PrintMode::DEBUG, fmt::color::cyan},
    {PrintMode::INFO, fmt::color::green},
    {PrintMode::WARNING, fmt::color::yellow},
    {PrintMode::ERROR, fmt::color::red},
    {PrintMode::FATAL, fmt::color::magenta},
};

inline const std::unordered_map<PrintMode, std::string> PRINT_PREFIX = {
    {PrintMode::DEBUG, "[DEBUG]"},
    {PrintMode::INFO, "[INFO ]"},
    {PrintMode::WARNING, "[WARN ]"},
    {PrintMode::ERROR, "[ERROR]"},
    {PrintMode::FATAL, "[FATAL]"},
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
    PrintMode min_mode = PrintMode::DEBUG;

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
 * @brief Initialize a new session
 * @param session_or_suffix 会话目录或后缀
 *   - 包含 '/' 时当作完整路径 (watchdog 传入)
 *   - 不包含 '/' 时当作后缀 (测试程序用)
 *   - 为空时自动创建无后缀目录
 * @return Session directory path
 *
 * 使用方式:
 *   1. 自动创建: init_session() -> log/2024-01-01_12-00-00/
 *   2. 带后缀:   init_session("test_param") -> log/2024-01-01_12-00-00_test_param/
 *   3. 完整路径: init_session("/path/to/session") -> /path/to/session/
 *
 * Directory structure:
 *   - run.log: debug::print 输出
 *   - screen.log: watchdog screen 输出 (由 watchdog 写入)
 *   - heartbeat: 心跳文件
 *   - config/: 配置文件快照
 *   - raw.mkv, debug.mkv, imu.csv: 录制文件
 */
inline std::string init_session(const std::string& session_or_suffix = "") {
    auto& state = LoggerState::instance();
    std::lock_guard<std::mutex> lock(state.file_mutex);

    if (state.log_file.is_open()) {
        state.log_file.close();
    }

    // 智能判断：包含 '/' 当作路径，否则当作后缀
    if (!session_or_suffix.empty() && session_or_suffix.find('/') != std::string::npos) {
        // 完整路径 (watchdog 传入)
        state.session_path = session_or_suffix;
        state.session_timestamp = fs::path(session_or_suffix).filename().string();
    } else {
        // 后缀模式 (测试程序用)
        state.session_timestamp = get_timestamp_for_filename();
        if (session_or_suffix.empty()) {
            state.session_path = std::string(LOG_DIR) + "/" + state.session_timestamp;
        } else {
            state.session_path = std::string(LOG_DIR) + "/" + state.session_timestamp + "_" + session_or_suffix;
        }
    }

    fs::create_directories(state.session_path);

    std::string log_path = state.session_path + "/run.log";
    state.log_file.open(log_path, std::ios::app);

    if (state.log_file.is_open()) {
        state.log_file << fmt::format("=== Session started at {} ===\n", get_current_time_string());
        state.log_file.flush();
    }

    // 保存配置快照
    std::string config_snapshot_dir = state.session_path + "/config";
    fs::create_directories(config_snapshot_dir);
    for (const auto& entry : fs::directory_iterator(CONFIG_DIR)) {
        if (entry.path().extension() == ".toml") {
            fs::copy_file(entry.path(),
                          config_snapshot_dir + "/" + entry.path().filename().string(),
                          fs::copy_options::overwrite_existing);
        }
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
    if (lower_str == "debug") return PrintMode::DEBUG;
    if (lower_str == "info") return PrintMode::INFO;
    if (lower_str == "warning" || lower_str == "warn") return PrintMode::WARNING;
    if (lower_str == "error" || lower_str == "err") return PrintMode::ERROR;
    if (lower_str == "fatal") return PrintMode::FATAL;
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
