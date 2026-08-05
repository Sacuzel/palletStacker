"""Build and install the integrated Gazebo GUI forklift-control plugin.

The generated world loads the GUI plugin by the stable name ``ForkliftTeleop``.
Gazebo GUI uses that same name to locate both ``libForkliftTeleop.so`` and the
plugin's embedded QML resource, so an absolute library path cannot be written
into the world file.

``main.py`` therefore performs two deterministic stages:

1. build the shared library under the project directory;
2. install or update a user-local runtime copy at
   ``~/.gz/gui/plugins/libForkliftTeleop.so``.

Gazebo GUI searches that directory by default. After generation, the world can
be started with one ordinary command and no ROS bridge or environment variable::

    python code/main.py
    gz sim gazebo/worlds/pallet_stacker_world.sdf

The build is incremental. Source changes trigger recompilation; an unchanged
library is reused and only copied when the installed runtime copy is missing or
different.
"""

from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

from . import settings


class ForkliftGuiPluginError(RuntimeError):
    """Raised when the custom Gazebo GUI plugin cannot be prepared."""


@dataclass(frozen=True, slots=True)
class ForkliftGuiPluginResult:
    """Paths and update status for the custom Gazebo GUI plugin."""

    source_directory: Path
    build_directory: Path
    build_library_path: Path
    library_path: Path
    rebuilt: bool
    installed: bool

    @property
    def gazebo_filename(self) -> str:
        """Stable ``filename`` value written into the world's GUI block."""

        return settings.FORKLIFT_GUI_PLUGIN_NAME


_SOURCE_FILES: tuple[str, ...] = (
    "CMakeLists.txt",
    "ForkliftTeleop.cc",
    "ForkliftTeleop.hh",
    "ForkliftTeleop.qml",
    "ForkliftTeleop.qrc",
)
_FINGERPRINT_FILE = ".pallet_stacker_plugin.sha256"


def prepare_forklift_gui_plugin() -> ForkliftGuiPluginResult:
    """Return a loadable plugin, building and installing it when required."""

    source_directory = settings.FORKLIFT_GUI_PLUGIN_SOURCE_DIRECTORY.resolve()
    build_directory = settings.FORKLIFT_GUI_PLUGIN_BUILD_DIRECTORY.resolve()
    build_library = settings.FORKLIFT_GUI_PLUGIN_BUILD_LIBRARY_FILE.resolve()
    installed_library = settings.FORKLIFT_GUI_PLUGIN_LIBRARY_FILE.resolve()

    source_files = tuple(source_directory / name for name in _SOURCE_FILES)
    missing = [path for path in source_files if not path.is_file()]
    if missing:
        formatted = "\n".join(f"  - {path}" for path in missing)
        raise ForkliftGuiPluginError(
            "Forklift GUI plugin source files are missing:\n" + formatted
        )

    _reset_stale_cmake_cache(source_directory, build_directory)
    build_directory.mkdir(parents=True, exist_ok=True)

    fingerprint_file = build_directory / _FINGERPRINT_FILE
    current_fingerprint = _source_fingerprint(source_files)
    previous_fingerprint = (
        fingerprint_file.read_text(encoding="utf-8").strip()
        if fingerprint_file.is_file()
        else None
    )

    needs_build = (
        settings.GAZEBO_FORCE_REBUILD_FORKLIFT_GUI_PLUGIN
        or not build_library.is_file()
        or previous_fingerprint != current_fingerprint
    )
    rebuilt = False

    if needs_build:
        if not settings.GAZEBO_BUILD_FORKLIFT_GUI_PLUGIN:
            raise ForkliftGuiPluginError(
                "The integrated forklift-control plugin needs to be built, but "
                "GAZEBO_BUILD_FORKLIFT_GUI_PLUGIN is False. Expected project "
                f"build library: {build_library}"
            )

        _build_plugin(source_directory, build_directory)
        rebuilt = True

        actual_library = _find_built_library(build_directory, build_library)
        if actual_library != build_library:
            _copy_file_atomic(actual_library, build_library)

        if not build_library.is_file():
            raise ForkliftGuiPluginError(
                "The GUI plugin build did not create the expected shared library: "
                f"{build_library}"
            )

        _write_text_atomic(fingerprint_file, current_fingerprint + "\n")

    if not build_library.is_file():
        raise ForkliftGuiPluginError(
            "The project-local GUI plugin library is missing: "
            f"{build_library}"
        )

    installed = (
        not installed_library.is_file()
        or not _files_equal(build_library, installed_library)
    )
    if installed:
        _copy_file_atomic(build_library, installed_library)

    if not installed_library.is_file():
        raise ForkliftGuiPluginError(
            "The GUI plugin could not be installed in Gazebo's user plugin "
            f"directory: {installed_library}"
        )

    return ForkliftGuiPluginResult(
        source_directory=source_directory,
        build_directory=build_directory,
        build_library_path=build_library,
        library_path=installed_library,
        rebuilt=rebuilt,
        installed=installed,
    )


def _reset_stale_cmake_cache(
    source_directory: Path,
    build_directory: Path,
) -> None:
    """Discard a CMake cache that belongs to a different project location."""

    cache_file = build_directory / "CMakeCache.txt"
    if not cache_file.is_file():
        return

    try:
        lines = cache_file.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        raise ForkliftGuiPluginError(
            f"Could not inspect existing CMake cache {cache_file}: {exc}"
        ) from exc

    prefix = "CMAKE_HOME_DIRECTORY:INTERNAL="
    cached_source_text = next(
        (line[len(prefix):] for line in lines if line.startswith(prefix)),
        None,
    )
    if cached_source_text is None:
        return

    cached_source = Path(cached_source_text).expanduser().resolve(strict=False)
    if cached_source == source_directory:
        return

    try:
        shutil.rmtree(build_directory)
    except OSError as exc:
        raise ForkliftGuiPluginError(
            "The GUI plugin project moved, but its stale CMake build directory "
            f"could not be removed: {build_directory}: {exc}"
        ) from exc


def _build_plugin(source_directory: Path, build_directory: Path) -> None:
    cmake = shutil.which(settings.GAZEBO_CMAKE_EXECUTABLE)
    if cmake is None:
        raise ForkliftGuiPluginError(
            f"Could not find {settings.GAZEBO_CMAKE_EXECUTABLE!r} on PATH."
            + _dependency_hint()
        )

    build_type = settings.GAZEBO_GUI_PLUGIN_BUILD_TYPE.strip()
    if not build_type:
        raise ForkliftGuiPluginError(
            "GAZEBO_GUI_PLUGIN_BUILD_TYPE must not be empty."
        )
    jobs = settings.GAZEBO_GUI_PLUGIN_PARALLEL_JOBS
    if jobs < 0:
        raise ForkliftGuiPluginError(
            "GAZEBO_GUI_PLUGIN_PARALLEL_JOBS must be zero or positive."
        )

    configure_command = [
        cmake,
        "-S",
        str(source_directory),
        "-B",
        str(build_directory),
        f"-DCMAKE_BUILD_TYPE={build_type}",
    ]
    build_command = [
        cmake,
        "--build",
        str(build_directory),
        "--config",
        build_type,
        "--parallel",
    ]
    if jobs > 0:
        build_command.append(str(jobs))

    _run_build_command(configure_command, "CMake configuration")
    _run_build_command(build_command, "CMake build")


def _run_build_command(command: list[str], stage: str) -> None:
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError as exc:
        raise ForkliftGuiPluginError(f"{stage} could not start: {exc}") from exc

    if completed.returncode == 0:
        return

    output = "\n".join(
        part.strip()
        for part in (completed.stdout, completed.stderr)
        if part.strip()
    )
    if len(output) > 16_000:
        output = output[-16_000:]

    raise ForkliftGuiPluginError(
        f"{stage} failed with return code {completed.returncode}."
        + _dependency_hint()
        + (f"\n\nBuild output:\n{output}" if output else "")
    )


def _find_built_library(build_directory: Path, expected: Path) -> Path:
    if expected.is_file():
        return expected

    candidates = sorted(
        build_directory.rglob(f"lib{settings.FORKLIFT_GUI_PLUGIN_NAME}.so")
    )
    if not candidates:
        raise ForkliftGuiPluginError(
            "CMake reported success, but the ForkliftTeleop shared library "
            f"was not found under {build_directory}."
        )
    return candidates[0]


def _source_fingerprint(paths: tuple[Path, ...]) -> str:
    digest = hashlib.sha256()
    build_settings = (
        settings.GAZEBO_GUI_PLUGIN_BUILD_TYPE,
        settings.FORKLIFT_GUI_PLUGIN_NAME,
    )
    for value in build_settings:
        digest.update(value.encode("utf-8"))
        digest.update(b"\0")
    for path in paths:
        digest.update(path.name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def _files_equal(first: Path, second: Path) -> bool:
    """Return true when two regular files have identical byte content."""

    try:
        if first.stat().st_size != second.stat().st_size:
            return False
    except OSError:
        return False
    return _file_digest(first) == _file_digest(second)


def _file_digest(path: Path) -> bytes:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.digest()


def _copy_file_atomic(source: Path, destination: Path) -> None:
    """Copy a file through a temporary sibling and atomically replace it."""

    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(
        f".{destination.name}.tmp-{os.getpid()}"
    )
    try:
        shutil.copy2(source, temporary)
        temporary.replace(destination)
    except OSError as exc:
        raise ForkliftGuiPluginError(
            f"Could not install {source} as {destination}: {exc}"
        ) from exc
    finally:
        temporary.unlink(missing_ok=True)


def _write_text_atomic(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    try:
        temporary.write_text(text, encoding="utf-8")
        temporary.replace(path)
    except OSError as exc:
        raise ForkliftGuiPluginError(f"Could not write {path}: {exc}") from exc
    finally:
        temporary.unlink(missing_ok=True)


def _dependency_hint() -> str:
    return (
        "\n\nInstall the Gazebo Harmonic GUI-plugin development dependencies "
        "once on Ubuntu 24.04 with:\n"
        "  sudo apt install build-essential cmake libgz-gui8-dev "
        "libgz-transport13-dev libgz-msgs10-dev qtbase5-dev "
        "qtdeclarative5-dev qtquickcontrols2-5-dev"
    )


def main() -> int:
    result = prepare_forklift_gui_plugin()
    states: list[str] = []
    if result.rebuilt:
        states.append("rebuilt")
    if result.installed:
        states.append("installed/updated")
    if not states:
        states.append("already current")
    print(f"Forklift GUI plugin {', '.join(states)}: {result.library_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
