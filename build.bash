#!/usr/bin/env bash
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
[[ -f "${ROS_SETUP}" ]] || { echo "ROS setup not found: ${ROS_SETUP}" >&2; exit 1; }
source "${ROS_SETUP}"

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
colcon_path_args=(--build-base "${workspace_dir}/build-link" --install-base "${workspace_dir}/install-link")
case "${workspace_dir}" in
  *[![:ascii:]]*)
    ascii_workspace_link="/tmp/navi_minco_bit_jazzy_workspace"
    ln -sfn "${workspace_dir}" "${ascii_workspace_link}"
    cd "${ascii_workspace_link}"
    colcon_path_args=(
      --base-paths "${ascii_workspace_link}"
      # Keep build outputs persistent: --symlink-install makes install hooks
      # point into this directory, so a /tmp build tree breaks after cleanup.
      --build-base "${workspace_dir}/build-link"
      --install-base "${workspace_dir}/install-link"
    )
    ;;
  *)
    cd "${workspace_dir}"
    ;;
esac

set -u
BUILD_PROFILE="${BUILD_PROFILE:-full}"
case "$BUILD_PROFILE" in fast|ci|full) ;; *) echo "BUILD_PROFILE must be fast, ci, or full" >&2; exit 2 ;; esac

# Hard memory budget for build process (GB).
MEM_LIMIT_GB=${MEM_LIMIT_GB:-14}
# Estimated memory usage per concurrently building package (GB).
MEM_PER_WORKER_GB=${MEM_PER_WORKER_GB:-1}

if (( MEM_PER_WORKER_GB <= 0 )); then
	echo "MEM_PER_WORKER_GB must be > 0"
	exit 1
fi

cpu_workers=$(nproc)
max_workers_by_mem=$(( MEM_LIMIT_GB / MEM_PER_WORKER_GB ))
if (( max_workers_by_mem < 1 )); then
	max_workers_by_mem=1
fi

parallel_workers=$cpu_workers
if (( parallel_workers > max_workers_by_mem )); then
	parallel_workers=$max_workers_by_mem
fi

mem_limit_mb=$(( MEM_LIMIT_GB * 1024 ))

echo "[build] MEM_LIMIT_GB=${MEM_LIMIT_GB}, MEM_PER_WORKER_GB=${MEM_PER_WORKER_GB}, CPU_WORKERS=${cpu_workers}, PARALLEL_WORKERS=${parallel_workers}"

monitor_memory() {
	local limit_mb="$1"
	while true; do
		local mem_total_kb mem_available_kb mem_used_mb mem_available_mb
		mem_total_kb=$(awk '/MemTotal:/ {print $2}' /proc/meminfo)
		mem_available_kb=$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)
		mem_used_mb=$(( (mem_total_kb - mem_available_kb) / 1024 ))
		mem_available_mb=$(( mem_available_kb / 1024 ))

		if (( mem_used_mb > limit_mb )); then
			echo "[mem] WARNING used=${mem_used_mb}MB > limit=${limit_mb}MB, available=${mem_available_mb}MB"
		else
			echo "[mem] used=${mem_used_mb}MB, limit=${limit_mb}MB, available=${mem_available_mb}MB"
		fi
		sleep 2
	done
}

monitor_memory "$mem_limit_mb" &
monitor_pid=$!

cleanup() {
	kill "$monitor_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

cmake_args=(
	-DCMAKE_BUILD_TYPE=Release
	-DCMAKE_EXPORT_COMPILE_COMMANDS=1
	-DPython3_EXECUTABLE=/usr/bin/python3
	-DPYTHON_EXECUTABLE=/usr/bin/python3
)
if command -v ccache >/dev/null 2>&1; then
  cmake_args+=( -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache )
fi

serial_packages=()
if [[ -n "${COLCON_SERIAL_PACKAGES:-}" ]]; then
  read -r -a serial_packages <<< "$COLCON_SERIAL_PACKAGES"
fi

if [[ "$BUILD_PROFILE" == full ]]; then
  echo "[build] full profile: serial critical packages"
else
  serial_packages=()
fi
for pkg in "${serial_packages[@]}"; do
	echo "[build] Serial build package: ${pkg}"
	colcon build \
		"${colcon_path_args[@]}" \
		--symlink-install \
		--parallel-workers 1 \
		--packages-select "${pkg}" \
		--cmake-args "${cmake_args[@]}" \
		--event-handlers console_direct+ status+
done

package_args=()
if [[ -n "${COLCON_PACKAGES:-}" ]]; then
  read -r -a requested_packages <<< "$COLCON_PACKAGES"
  package_args+=( --packages-up-to "${requested_packages[@]}" )
elif [[ "$BUILD_PROFILE" != full ]]; then
  mapfile -t changed_packages < <(git diff --name-only -- src | awk -F/ 'NF > 2 {print $2}' | sort -u)
  if ((${#changed_packages[@]})); then
    package_args+=( --packages-up-to "${changed_packages[@]}" )
  fi
fi
echo "[build] Stage 2: build selected packages in parallel"
colcon build \
  "${colcon_path_args[@]}" \
  --symlink-install \
  --parallel-workers "$parallel_workers" \
  --packages-skip "${serial_packages[@]}" \
	"${package_args[@]}" \
	--cmake-args "${cmake_args[@]}" \
	--event-handlers console_direct+ status+
