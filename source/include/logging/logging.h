/**
 * File:   logging.h
 *
 * Date:   18.11.2021
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef TRIVIS_EXAMPLES_LOGGING_LOGGING_H_
#define TRIVIS_EXAMPLES_LOGGING_LOGGING_H_

#include <boost/log/trivial.hpp>

#define LOG_TRC BOOST_LOG_TRIVIAL(trace)
#define LOG_DBG BOOST_LOG_TRIVIAL(debug)
#define LOG_INF BOOST_LOG_TRIVIAL(info)
#define LOG_WRN BOOST_LOG_TRIVIAL(warning)
#define LOG_ERR BOOST_LOG_TRIVIAL(error)
#define LOG_FTL BOOST_LOG_TRIVIAL(fatal)

#define LOGF_TRC(msg) LOG_TRC << trivis_examples::logging::Color(trivis_examples::logging::kBlue) << msg << trivis_examples::logging::kReset
#define LOGF_DBG(msg) LOG_DBG << trivis_examples::logging::Color(trivis_examples::logging::kGreen) << msg << trivis_examples::logging::kReset
#define LOGF_INF(msg) LOG_INF << msg << trivis_examples::logging::kReset
#define LOGF_WRN(msg) LOG_WRN << trivis_examples::logging::Color(trivis_examples::logging::kYellow) << msg << trivis_examples::logging::kReset
#define LOGF_ERR(msg) LOG_ERR << trivis_examples::logging::Color(trivis_examples::logging::kRed) << msg << trivis_examples::logging::kReset
#define LOGF_FTL(msg) LOG_FTL << trivis_examples::logging::Style(trivis_examples::logging::kBold, trivis_examples::logging::kRed) << msg << trivis_examples::logging::kReset

#ifndef LOG_NAME
#define LOG_NAME ""
#endif

namespace trivis_examples::logging {

using severity_level = boost::log::trivial::severity_level;

static constexpr const char *kLibName = LOG_NAME;
static constexpr const char *kReset = "\u001b[0m";
static constexpr const char *kEndL = "\u001b[0m\n";

enum Style : int {
    kBold = 1,
    kUnderline = 4,
    kReversed = 7
};

enum Color : int {
    kBlack = 0,
    kRed = 1,
    kGreen = 2,
    kYellow = 3,
    kBlue = 4,
    kMagenta = 5,
    kCyan = 6,
    kWhite = 7,
};

inline std::string Color(enum Color txt_color) {
    return "\u001b[3" + std::to_string(txt_color) + "m";
}

inline std::string BgColor(enum Color bg_color) {
    return "\u001b[4" + std::to_string(bg_color) + "m";
}

inline std::string BgColor(enum Color bg_color, enum Color txt_color) {
    return "\u001b[4" + std::to_string(bg_color) + ";3" + std::to_string(txt_color) + "m";
}

inline std::string BgColor(enum Color bg_color, enum Style style) {
    return "\u001b[4" + std::to_string(bg_color) + ";" + std::to_string(style) + "m";
}

inline std::string Style(enum Style style) {
    return "\u001b[" + std::to_string(style) + "m";
}

inline std::string Style(enum Style style, enum Color txt_color) {
    return "\u001b[" + std::to_string(style) + ";3" + std::to_string(txt_color) + "m";
}

inline std::string Style(enum Style style, enum Color txt_color, enum Color bg_color) {
    return "\u001b[" + std::to_string(style) + ";3" + std::to_string(txt_color) + ";4" + std::to_string(bg_color) + "m";
}

void InitLogging(const boost::log::trivial::severity_level &level);

}

#endif //TRIVIS_EXAMPLES_LOGGING_LOGGING_H_
