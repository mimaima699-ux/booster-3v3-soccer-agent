from pathlib import Path

from setuptools import find_packages, setup


package_name = "soccer_sim_debug"
repo_src = Path(__file__).resolve().parents[2] / "src"


def _discover_src_packages() -> tuple[list[str], dict[str, str]]:
    """Discover repository packages under ``src/`` and build setup metadata.

    The older setup required maintaining ``packages`` and ``package_dir`` by
    hand. Adding a directory then meant updating this file too, and omissions
    surfaced later as ``ModuleNotFoundError`` from ``ros2 run``. Using
    setuptools ``find_packages`` avoids that drift:

    - ``find_packages(where=str(repo_src))`` returns dotted package names for
      every ``__init__.py`` subdirectory below ``repo_src/``, excluding
      ``src`` itself.
    - Each discovered name is prefixed with ``src.``, and the top-level
      ``src`` package is added explicitly.
    - ``package_dir`` maps each ``src.<sub>`` package back to
      ``repo_src/<sub>/...``.

    New ``src/foo/`` subpackages therefore do not require setup.py edits.
    """

    discovered = find_packages(where=str(repo_src))
    packages: list[str] = ["src", *(f"src.{name}" for name in discovered)]
    package_dir: dict[str, str] = {"src": str(repo_src)}
    for name in discovered:
        package_dir[f"src.{name}"] = str(repo_src.joinpath(*name.split(".")))
    return packages, package_dir


_src_packages, _src_package_dir = _discover_src_packages()


setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name, *_src_packages],
    package_dir={**_src_package_dir, package_name: package_name},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "py_trees>=2.4,<3"],
    zip_safe=True,
    maintainer="ylb",
    maintainer_email="ylb@example.com",
    description="Standalone ROS2 debug node for SoccerSim team controller.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "soccer_sim_node = soccer_sim_debug.ros_node:main",
        ],
    },
)
