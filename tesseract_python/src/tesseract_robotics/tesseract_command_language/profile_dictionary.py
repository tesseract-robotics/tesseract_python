from tesseract_robotics import tesseract_command_language
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    ProfileDictionary_addProfile_TrajOptCompositeProfile,
    ProfileDictionary_addProfile_TrajOptPlanProfile,
    ProfileDictionary_addProfile_TrajOptSolverProfile,
    ProfileDictionary_getProfile_TrajOptCompositeProfile,
    ProfileDictionary_getProfile_TrajOptPlanProfile,
    ProfileDictionary_getProfile_TrajOptSolverProfile,
    ProfileDictionary_hasProfile_TrajOptCompositeProfile,
    ProfileDictionary_hasProfile_TrajOptPlanProfile,
    ProfileDictionary_hasProfile_TrajOptSolverProfile,
    TrajOptCompositeProfile,
    TrajOptPlanProfile,
    TrajOptSolverProfile,
)


class ProfileDictionary(tesseract_command_language.ProfileDictionary):
    def __init__(self):
        super().__init__()

    def add_profile(self, ns: str, profile_name: str, profile):
        if issubclass(profile.__class__, TrajOptCompositeProfile):
            ProfileDictionary_addProfile_TrajOptCompositeProfile(
                profile_dictionary=self, ns=ns, profile_name=profile_name, profile=profile
            )

        elif issubclass(profile.__class__, TrajOptPlanProfile):
            ProfileDictionary_addProfile_TrajOptPlanProfile(
                profile_dictionary=self, ns=ns, profile_name=profile_name, profile=profile
            )

        elif issubclass(profile.__class__, TrajOptSolverProfile):
            ProfileDictionary_addProfile_TrajOptSolverProfile(
                profile_dictionary=self, ns=ns, profile_name=profile_name, profile=profile
            )

        else:
            raise NotImplementedError()

    def get_profile(self, ns: str, profile_name: str):
        raise NotImplementedError()

    def has_profile(self, ns: str, profile_name: str):
        raise NotImplementedError()
