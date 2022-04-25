/**
 * File:   logging.cc
 *
 * Date:   18.11.2021
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "logging/logging.h"

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>

using namespace trivis_examples::logging;

void trivis_examples::logging::InitLogging(const severity_level &level) {
    using namespace boost;
    log::add_common_attributes();
    log::register_simple_formatter_factory<severity_level, char>("Severity");
    std::string log_format = std::string("") + "[%TimeStamp%]" + (std::string(kLibName).empty() ? "" : " [" + std::string(kLibName) + "]") + " [%Severity%] >> %Message%";
    log::add_console_log(std::cout, log::keywords::format = log_format);
    log::core::get()->set_filter(log::trivial::severity >= log::trivial::trace);
    log::core::get()->set_filter(log::trivial::severity >= level);
}