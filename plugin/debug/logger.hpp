#ifndef PLUGIN_DEBUG_LOGGER_HPP
#define PLUGIN_DEBUG_LOGGER_HPP

#include <unordered_map>
#include <set>
#include <algorithm>
#include <string>
#include <map>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <mutex>

#include <Eigen/Core>
#include <fmt/core.h>
#include <fmt/color.h>
#include <fmt/chrono.h>

//总有傻逼宏定义污染资源
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

	enum class PrintMode {
		LOG,
		INFO,
		DEBUG,
		WARNING,
		ERROR,
		SILENT
	};

	static const Eigen::IOFormat kLongCsvFmt(
		Eigen::FullPrecision, Eigen::FullPrecision, ", ", ";\n", "[", "]", "\n{", "}");
	static const std::unordered_map<PrintMode, fmt::color> PRINT_COLOR = {
		{PrintMode::LOG, fmt::color::green},
		{PrintMode::INFO, fmt::color::white},
		{PrintMode::WARNING, fmt::color::yellow},
		{PrintMode::ERROR, fmt::color::red},
		{PrintMode::DEBUG, fmt::color::cyan},
	};
	static const std::unordered_map<PrintMode, std::string> PRINT_PREFIX = {
		{PrintMode::LOG, "[LOGG]"},
		{PrintMode::INFO, "[INFO]"},
		{PrintMode::WARNING, "[WARN]"},
		{PrintMode::ERROR, "[EROR]"},
		{PrintMode::DEBUG, "[DBUG]"}
	};
	//	static const std::unordered_map<PrintMode, std::string> HTML_COLOR = {
	//			{PrintMode::LOG, "green"},
	//			{PrintMode::INFO, ""},
	//			{PrintMode::DEBUG, "blue"},
	//			{PrintMode::WARNING, "orange"},
	//			{PrintMode::ERROR, "red"},
	//	};
	static PrintMode current_min_mode = PrintMode::LOG;
	static std::set<std::string> whitelist_nodes;
	static std::set<std::string> blacklist_nodes;
	static std::ofstream md_file;
	static std::mutex file_mutex;


	inline void add_whitenode(const std::string &node) {
		whitelist_nodes.insert(node);
	}

	inline void add_blacknode(const std::string &node) {
		blacklist_nodes.insert(node);
	}

	template<typename T>
	inline auto stream_to_str(T &x) -> std::string {
		std::stringstream buffer;
		buffer << x;
		return buffer.str();
	}

	template<typename T>
	inline auto eigen_to_str(const T &x) -> std::string {
		std::ostringstream oss;
		oss << x.format(kLongCsvFmt);
		return oss.str();
	}

	template<typename T>
	inline auto vec_to_str(const std::vector<T> &vec) -> std::string {
		std::string str = "[";
		for (const auto &ele: vec) {
			str += fmt::format("{}", ele);
			if (&ele != &vec.back()) {
				str += ", ";
			}
		}
		str += "]";
		return str;
	}

	template<typename K, typename V>
	inline auto map_to_str(const std::map<K, V> &m) -> std::string {
		std::string str = "{";
		for (auto it = m.begin(); it != m.end(); ++it) {
			const auto &key = it->first;
			const auto &data = it->second;

			// 格式化每个元素
			str += fmt::format("{}: {{{},{}}}", key, data.val, data.updated);

			// 添加逗号，除了最后一个元素
			if (std::next(it) != m.end()) {
				str += ", ";
			}
		}
		str += "}";
		return str;
	}

	inline std::string get_current_time_string() {
		auto now = std::chrono::system_clock::now();
		auto time_t_now = std::chrono::system_clock::to_time_t(now);
		auto microseconds =
				std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000 % 1000;
		auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

		// 使用 fmt::format 格式化输出时间，包括毫秒和微秒
		return fmt::format("{:%Y-%m-%d %H:%M:%S}.{:03},{:03}", *std::localtime(&time_t_now), milliseconds.count(),
		                   microseconds.count());
	}

	inline void init_md_file(const std::string &filename) {
		// Get current time for filename (without microseconds)
		auto now = std::chrono::system_clock::now();
		auto time_t_now = std::chrono::system_clock::to_time_t(now);
		std::string timestamp = fmt::format("{:%Y-%m-%d_%H-%M-%S}", *std::localtime(&time_t_now));

		// Create filename with timestamp
		std::string timestamped_filename = fmt::format("{}_{}", timestamp, filename);

		md_file.open(std::string(LOG_DIR) + "/" + timestamped_filename, std::ios::app);
		if (md_file.is_open()) {
			std::lock_guard<std::mutex> lock(file_mutex);
			std::string current_time = get_current_time_string();
			md_file << fmt::format("\n## Run started at {}\n", current_time);
			md_file.flush();
		}
	}

	inline void close_md_file() {
		if (md_file.is_open()) {
			md_file.close();
		}
	}

	template<typename... T>
	inline void print(
		const PrintMode &mode,
		const std::string &node_name,
		const std::string &content,
		T &&... args) {
		if (mode >= current_min_mode &&
		    (whitelist_nodes.empty() || whitelist_nodes.find(node_name) != whitelist_nodes.end()) &&
		    (blacklist_nodes.find(node_name) == blacklist_nodes.end())) {
			std::string timestamp = get_current_time_string();
			std::string formatted_content;
			try {
				if constexpr (sizeof...(args) > 0) {
					formatted_content = fmt::format(fmt::runtime(content), std::forward<T>(args)...);
				} else {
					formatted_content = content;
				}
			} catch (const fmt::format_error &e) {
				formatted_content = content + " [格式化错误: " + e.what() + "]";
			}

			std::string full_message = fmt::format("{} {} {}: {}",
			                                       timestamp, PRINT_PREFIX.at(mode),
			                                       (node_name.empty() ? "" : "@" + node_name),
			                                       formatted_content);

			fmt::print(fmt::fg(PRINT_COLOR.at(mode)), "{}\n", full_message);

			if (md_file.is_open()) {
				std::lock_guard<std::mutex> lock(file_mutex);
				//////////////
				md_file << fmt::format("- **{}** {} {}: {}\n",
				                       timestamp, PRINT_PREFIX.at(mode),
				                       (node_name.empty() ? "" : "@" + node_name),
				                       formatted_content);
				////////////
				//md_file << fmt::format("- **{}** <font color=\"{}\">{} {}: {}</font>\n",
				//                       timestamp, HTML_COLOR.at(mode),
				//                       PRINT_PREFIX.at(mode),
				//                       (node_name.empty() ? "" : "@" + node_name),
				//                       formatted_content);
				md_file.flush();
			}
		}
	}

	inline auto string_to_mode(const std::string &mode_str) -> PrintMode {
		std::string lower_str = mode_str;
		std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
		if (lower_str == "log") return PrintMode::LOG;
		if (lower_str == "info") return PrintMode::INFO;
		if (lower_str == "debug") return PrintMode::DEBUG;
		if (lower_str == "warning") return PrintMode::WARNING;
		if (lower_str == "error") return PrintMode::ERROR;
		return PrintMode::SILENT;
	}

	template<typename... T>
	inline void print(
		const std::string &mode_str,
		const std::string &node_name,
		const std::string &content,
		T &&... args) {
		print(string_to_mode(mode_str), node_name, content, std::forward<T>(args)...);
	}
}

#endif //PLUGIN_DEBUG_LOGGER_HPP
