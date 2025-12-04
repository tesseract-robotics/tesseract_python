/**
 * @file tesseract_motion_planners_ompl_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners OMPL
 */

#include "tesseract_nb.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract_motion_planners/core/types.h>

// tesseract_command_language (for Profile base class)
#include <tesseract_command_language/profile.h>
#include <tesseract_command_language/profile_dictionary.h>

// tesseract_motion_planners OMPL
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_plan_profile.h>

namespace tp = tesseract_planning;

NB_MODULE(_tesseract_motion_planners_ompl, m) {
    m.doc() = "tesseract_motion_planners_ompl Python bindings";

    // Import Profile type from tesseract_command_language for cross-module inheritance
    auto cl_module = nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");
    auto profile_type = cl_module.attr("Profile");

    // ========== OMPLPlannerType enum ==========
    nb::enum_<tp::OMPLPlannerType>(m, "OMPLPlannerType")
        .value("SBL", tp::OMPLPlannerType::SBL)
        .value("EST", tp::OMPLPlannerType::EST)
        .value("LBKPIECE1", tp::OMPLPlannerType::LBKPIECE1)
        .value("BKPIECE1", tp::OMPLPlannerType::BKPIECE1)
        .value("KPIECE1", tp::OMPLPlannerType::KPIECE1)
        .value("BiTRRT", tp::OMPLPlannerType::BiTRRT)
        .value("RRT", tp::OMPLPlannerType::RRT)
        .value("RRTConnect", tp::OMPLPlannerType::RRTConnect)
        .value("RRTstar", tp::OMPLPlannerType::RRTstar)
        .value("TRRT", tp::OMPLPlannerType::TRRT)
        .value("PRM", tp::OMPLPlannerType::PRM)
        .value("PRMstar", tp::OMPLPlannerType::PRMstar)
        .value("LazyPRMstar", tp::OMPLPlannerType::LazyPRMstar)
        .value("SPARS", tp::OMPLPlannerType::SPARS);

    // ========== OMPLPlannerConfigurator (base) ==========
    nb::class_<tp::OMPLPlannerConfigurator>(m, "OMPLPlannerConfigurator")
        .def("getType", &tp::OMPLPlannerConfigurator::getType);

    // ========== RRTConnectConfigurator ==========
    nb::class_<tp::RRTConnectConfigurator, tp::OMPLPlannerConfigurator>(m, "RRTConnectConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::RRTConnectConfigurator::range);

    // ========== RRTstarConfigurator ==========
    nb::class_<tp::RRTstarConfigurator, tp::OMPLPlannerConfigurator>(m, "RRTstarConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::RRTstarConfigurator::range)
        .def_rw("goal_bias", &tp::RRTstarConfigurator::goal_bias)
        .def_rw("delay_collision_checking", &tp::RRTstarConfigurator::delay_collision_checking);

    // ========== SBLConfigurator ==========
    nb::class_<tp::SBLConfigurator, tp::OMPLPlannerConfigurator>(m, "SBLConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::SBLConfigurator::range);

    // ========== OMPLPlanProfile (base) ==========
    // Import the Profile type from command_language and use it as base class
    nb::class_<tp::OMPLPlanProfile, tp::Profile>(m, "OMPLPlanProfile")
        .def("getKey", &tp::OMPLPlanProfile::getKey)
        .def_static("getStaticKey", &tp::OMPLPlanProfile::getStaticKey);

    // ========== OMPLRealVectorPlanProfile ==========
    nb::class_<tp::OMPLRealVectorPlanProfile, tp::OMPLPlanProfile>(m, "OMPLRealVectorPlanProfile")
        .def(nb::init<>());

    // Helper to convert OMPLPlanProfile to Profile::ConstPtr for ProfileDictionary
    // This explicitly casts to the base type for cross-module compatibility
    m.def("OMPLPlanProfile_as_ProfileConstPtr", [](std::shared_ptr<tp::OMPLPlanProfile> profile) -> tp::Profile::ConstPtr {
        return profile;  // implicit conversion to base class shared_ptr
    }, "profile"_a, "Convert OMPLPlanProfile to Profile::ConstPtr for use with ProfileDictionary.addProfile");

    // Helper to add OMPL plan profile to ProfileDictionary directly
    m.def("ProfileDictionary_addOMPLProfile", [](tp::ProfileDictionary& dict,
                                                  const std::string& ns,
                                                  const std::string& profile_name,
                                                  std::shared_ptr<tp::OMPLPlanProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add OMPL plan profile to ProfileDictionary (cross-module workaround)");

    // ========== OMPLMotionPlanner ==========
    // Note: Not binding inheritance from MotionPlanner to avoid cross-module issues
    nb::class_<tp::OMPLMotionPlanner>(m, "OMPLMotionPlanner")
        .def(nb::init<std::string>(), "name"_a)
        .def("getName", &tp::OMPLMotionPlanner::getName)
        .def("solve", &tp::OMPLMotionPlanner::solve, "request"_a)
        .def("terminate", &tp::OMPLMotionPlanner::terminate)
        .def("clear", &tp::OMPLMotionPlanner::clear);
}
