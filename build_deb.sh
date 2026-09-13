#!/usr/bin/env bash
# Build a Debian package for Wuji Hand 2 glove teleop + ROS driver.
#
# Usage:
#   ./build_deb.sh [VERSION]
# Example:
#   ./build_deb.sh 2.0.0
#
# Output:
#   dist/wuji-hand2-glove-teleop_<VERSION>-1_<arch>.deb

set -euo pipefail

VERSION=${1:-2.0.0}
DEB_VERSION=$(echo "${VERSION}" | sed 's/-/~/g')
ARCH=$(dpkg --print-architecture)
PACKAGE_NAME="wuji-hand2-glove-teleop"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${ROOT}/dist"
STAGE="${ROOT}/.deb_stage"
PKG_DIR="${STAGE}/${PACKAGE_NAME}_${DEB_VERSION}-1_${ARCH}"
INSTALL_ROOT="${PKG_DIR}/opt/wuji-hand2-glove-teleop"

echo "Building ${PACKAGE_NAME} ${DEB_VERSION}-1 (${ARCH})..."

rm -rf "${STAGE}"
mkdir -p "${INSTALL_ROOT}/retargeting" \
         "${INSTALL_ROOT}/wuji_hand_2" \
         "${PKG_DIR}/DEBIAN" \
         "${PKG_DIR}/usr/bin" \
         "${OUT_DIR}"

# --- payload ---
install -m 0644 "${ROOT}/README.md" "${INSTALL_ROOT}/README.md"
install -m 0644 "${ROOT}/LICENSE" "${INSTALL_ROOT}/LICENSE"

for f in \
  0.retarget_session.py \
  1.teleop_real.py \
  2.teleop_tuned.py \
  3.save_home.py \
  home_pose_service.py \
  wujihand2_ros_driver.py
do
  install -m 0644 "${ROOT}/examples/python/retargeting/${f}" \
    "${INSTALL_ROOT}/retargeting/${f}"
done

for f in \
  0.subscribe_callback.py \
  1.subscribe_async.py \
  2.publish.py \
  3.fingertip_typed.py \
  change_hand_ip_to_10.py
do
  install -m 0644 "${ROOT}/examples/python/wuji_hand_2/${f}" \
    "${INSTALL_ROOT}/wuji_hand_2/${f}"
done

write_wrapper() {
  local name="$1"
  local rel="$2"
  local abs="/opt/wuji-hand2-glove-teleop/${rel}"
  cat > "${PKG_DIR}/usr/bin/${name}" <<EOF
#!/usr/bin/env bash
# ${name} — ${rel}
set -euo pipefail
PY="\${WUJI_PYTHON:-python3}"
cd "\$(dirname "${abs}")"
exec "\${PY}" "${abs}" "\$@"
EOF
  chmod 0755 "${PKG_DIR}/usr/bin/${name}"
}

write_wrapper wujihand2-teleop              retargeting/2.teleop_tuned.py
write_wrapper wujihand2-teleop-real         retargeting/1.teleop_real.py
write_wrapper wujihand2-ros-driver          retargeting/wujihand2_ros_driver.py
write_wrapper wujihand2-save-home           retargeting/3.save_home.py
write_wrapper wujihand2-change-ip           wuji_hand_2/change_hand_ip_to_10.py
write_wrapper wujihand2-fingertip           wuji_hand_2/3.fingertip_typed.py

cat > "${PKG_DIR}/DEBIAN/control" <<EOF
Package: ${PACKAGE_NAME}
Version: ${DEB_VERSION}-1
Section: misc
Priority: optional
Architecture: ${ARCH}
Depends: python3 (>= 3.10)
Recommends: ros-humble-rclpy, ros-humble-sensor-msgs, ros-humble-std-msgs, ros-humble-std-srvs
Maintainer: continuity3 <continuity3@users.noreply.github.com>
Homepage: https://github.com/continuity3/wuji-hand2-glove-teleop
Description: Wuji Hand 2 glove teleoperation stack
 Python teleop + ROS2 bridge for Wuji Hand 2 (Ethernet) driven by Wuji Glove.
 Installs scripts under /opt/wuji-hand2-glove-teleop and CLI wrappers
 (wujihand2-teleop, wujihand2-ros-driver, …).
 Requires pip package wuji-sdk (and numpy, pynput) in the Python used via
 WUJI_PYTHON or PATH.
EOF

cat > "${PKG_DIR}/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e
echo "wuji-hand2-glove-teleop installed under /opt/wuji-hand2-glove-teleop"
echo "  export WUJI_PYTHON=/path/to/python   # interpreter with wuji-sdk"
echo "  wujihand2-teleop --drive sdk --hand-model wujihand2 --no-footkey"
echo "  wujihand2-ros-driver --side both --no-footkey"
exit 0
EOF
chmod 0755 "${PKG_DIR}/DEBIAN/postinst"

DEB_FILENAME="${PACKAGE_NAME}_${DEB_VERSION}-1_${ARCH}.deb"
fakeroot dpkg-deb --build "${PKG_DIR}" "${OUT_DIR}/${DEB_FILENAME}"

rm -rf "${STAGE}"

echo ""
echo "Package built:"
echo "  ${OUT_DIR}/${DEB_FILENAME}"
echo ""
echo "Install:"
echo "  sudo dpkg -i ${OUT_DIR}/${DEB_FILENAME}"
echo "  export WUJI_PYTHON=\$(which python)   # must have wuji-sdk"
