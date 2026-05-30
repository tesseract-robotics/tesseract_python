# adapted from https://github.com/colcon/colcon-core/blob/master/colcon_core/python_install_path.py

import sysconfig
import sys
from pathlib import Path

kwargs = {}
kwargs['vars'] = {'base': sys.argv[1]}
# Avoid deb_system because it means using --install-layout deb
# which ignores --prefix and hardcodes it to /usr
if 'deb_system' in sysconfig.get_scheme_names() or \
        'osx_framework_library' in sysconfig.get_scheme_names():
    kwargs['scheme'] = 'posix_prefix'
# The presence of the rpm_prefix scheme indicates that posix_prefix
# has been patched to inject `local` into the installation locations.
# The rpm_prefix scheme is a backup of what posix_prefix was before it was
# patched.
elif 'rpm_prefix' in sysconfig.get_scheme_names():
    kwargs['scheme'] = 'rpm_prefix'

raw_path = sysconfig.get_path('purelib', **kwargs)
posix_path = Path(raw_path).as_posix()
print(posix_path)
