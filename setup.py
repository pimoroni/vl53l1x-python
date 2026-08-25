# Metadata lives in pyproject.toml; this file exists only to describe the C
# extension, which the declarative config has no way to express.
from setuptools import Extension, setup

setup(ext_modules=[
    Extension(
        'vl53l1x_python',
        extra_compile_args=['-std=c99'],
        include_dirs=['api/core', 'api/platform', 'python_lib'],
        sources=['api/core/vl53l1_api_calibration.c',
                 'api/core/vl53l1_core.c',
                 'api/core/vl53l1_core_support.c',
                 'api/core/vl53l1_api_core.c',
                 'api/core/vl53l1_api_preset_modes.c',
                 'api/core/vl53l1_silicon_core.c',
                 'api/core/vl53l1_register_funcs.c',
                 'api/core/vl53l1_wait.c',
                 'api/core/vl53l1_error_strings.c',
                 'api/core/vl53l1_api_strings.c',
                 'api/core/vl53l1_api.c',
                 'api/platform/vl53l1_platform.c',
                 'python_lib/vl53l1x_python.c'])
])
