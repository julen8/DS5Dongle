#!/bin/bash
set -e

# ---- 颜色输出 ----
info()  { echo -e "\033[1;34m[INFO]\033[0m  $*"; }
ok()    { echo -e "\033[1;32m[OK]\033[0m    $*"; }
warn()  { echo -e "\033[1;33m[WARN]\033[0m  $*"; }
die()   { echo -e "\033[1;31m[ERROR]\033[0m $*" >&2; exit 1; }

WORK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PICO_SDK_DIR="${WORK_DIR}/pico-sdk"
BUILD_DEBUG_DIR="${WORK_DIR}/build_debug"
BUILD_RELEASE_DIR="${WORK_DIR}/build_release"
PICO_SDK_VERSION="2.3.0"
TINYUSB_VERSION="0.21.0"

arch=$(uname -m)

if [ "$arch" = "x86_64" ]; then
  ARM_TOOLCHAIN_URL="https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/packages/generic/gnu-toolchain/15.3.rel1/arm-gnu-toolchain-15.3.rel1-x86_64-arm-none-eabi.tar.xz"
elif [ "$arch" = "aarch64" ]; then
  ARM_TOOLCHAIN_URL="https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/packages/generic/gnu-toolchain/15.3.rel1/arm-gnu-toolchain-15.3.rel1-aarch64-arm-none-eabi.tar.xz"
else
  die "未知架构: $arch"
fi

ARM_TOOLCHAIN_PATH="${WORK_DIR}/arm-gnu-toolchain"
export PATH="${ARM_TOOLCHAIN_PATH}/bin:${PATH}"


# ---- 0. 安装编译依赖 ----
info "检查并安装编译依赖..."
PACKAGES=(
    git
    cmake
    ninja-build
    python3
    gcc
    g++
    ca-certificates
    ccache
    curl
    xz-utils
)
MISSING=()
for pkg in "${PACKAGES[@]}"; do
    if ! dpkg -s "$pkg" &>/dev/null; then
        MISSING+=("$pkg")
    fi
done

if [ ${#MISSING[@]} -gt 0 ]; then
    info "安装缺失的包: ${MISSING[*]}"
    export DEBIAN_FRONTEND=noninteractive
    export APT_LISTCHANGES_FRONTEND=none
    sudo apt-get update -qq
    sudo -E apt-get install -y -qq \
        -o Dpkg::Options::="--force-confdef" \
        -o Dpkg::Options::="--force-confold" \
        --no-install-recommends "${MISSING[@]}"

    ok "依赖安装完成"
else
    ok "所有依赖已安装，跳过"
fi

# --- 0.1 检查 ARM 工具链 ----
if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    info "ARM 工具链未安装，开始下载..."
    mkdir -p "${ARM_TOOLCHAIN_PATH}"
    curl -sSL "${ARM_TOOLCHAIN_URL}" -o "${ARM_TOOLCHAIN_PATH}/arm-gnu-toolchain.tar.xz"
    info "解压 ARM 工具链..."
    tar -xf "${ARM_TOOLCHAIN_PATH}/arm-gnu-toolchain.tar.xz" -C "${ARM_TOOLCHAIN_PATH}" --strip-components=1
    ok "ARM 工具链安装完成"
else
    ok "ARM 工具链已安装，跳过"
fi

# ---- 1. 初始化源码子模块 ----
if [ -f "${WORK_DIR}/.gitmodules" ]; then
    info "初始化源码子模块..."
    git submodule update --init --recursive --depth 1
    ok "源码子模块初始化完成"
else
    warn "未找到 .gitmodules，跳过源码子模块初始化"
fi

# ---- 2. 克隆 / 复用 Pico SDK ----
if [ -f "${PICO_SDK_DIR}/pico_sdk_version.cmake" ]; then
    ok "Pico SDK 已存在于 ${PICO_SDK_DIR}，跳过克隆"
else
    info "克隆 Pico SDK ${PICO_SDK_VERSION} 到 ${PICO_SDK_DIR}..."
    git clone --depth 1 --branch "${PICO_SDK_VERSION}" \
        https://github.com/raspberrypi/pico-sdk.git "${PICO_SDK_DIR}"
    ok "Pico SDK 克隆完成"
fi

# ---- 3. 初始化 SDK 子模块（跳过已初始化的） ----
for mod in btstack cyw43-driver lwip mbedtls tinyusb; do
    if [ -n "$(ls -A "${PICO_SDK_DIR}/lib/${mod}" 2>/dev/null)" ]; then
        ok "子模块 ${mod} 已初始化，跳过"
    else
        info "初始化子模块: ${mod}..."
        git -C "${PICO_SDK_DIR}" submodule update --init --depth 1 "lib/${mod}"
        ok "子模块 ${mod} 初始化完成"
    fi
done

# ---- 4. 将 TinyUSB 升级到指定版本 ----
CURRENT_TINYUSB=$(git -C "${PICO_SDK_DIR}/lib/tinyusb" describe --tags 2>/dev/null || echo "unknown")
info "当前 TinyUSB 版本: ${CURRENT_TINYUSB}"
if [ "${CURRENT_TINYUSB}" = "${TINYUSB_VERSION}" ]; then
    ok "TinyUSB 已是目标版本 ${TINYUSB_VERSION}，跳过升级"
else
    info "升级 TinyUSB 到 ${TINYUSB_VERSION}..."
    git -C "${PICO_SDK_DIR}/lib/tinyusb" fetch --depth 1 origin tag "${TINYUSB_VERSION}"
    git -C "${PICO_SDK_DIR}/lib/tinyusb" checkout "${TINYUSB_VERSION}"
    ok "TinyUSB 升级完成: ${TINYUSB_VERSION}"
fi

# ---- 5. CMake 配置 ----
info "CMake 配置中..."
mkdir -p "${BUILD_DEBUG_DIR}" "${BUILD_RELEASE_DIR}"

export PICO_SDK_PATH="${PICO_SDK_DIR}"
PICO_SDK_PATH="${PICO_SDK_DIR}" cmake -S "${WORK_DIR}" -B "${BUILD_DEBUG_DIR}" -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DENABLE_LOG=ON
PICO_SDK_PATH="${PICO_SDK_DIR}" cmake -S "${WORK_DIR}" -B "${BUILD_RELEASE_DIR}" -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON  -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DENABLE_LOG=OFF
ok "CMake 配置完成"

# ---- 6. 编译 ----
info "开始编译（ninja）..."
PICO_SDK_PATH="${PICO_SDK_DIR}" cmake --build "${BUILD_DEBUG_DIR}" --config Debug --target clean
PICO_SDK_PATH="${PICO_SDK_DIR}" cmake --build "${BUILD_DEBUG_DIR}" --config Debug

PICO_SDK_PATH="${PICO_SDK_DIR}" cmake --build "${BUILD_RELEASE_DIR}" --config Release --target clean
PICO_SDK_PATH="${PICO_SDK_DIR}" cmake --build "${BUILD_RELEASE_DIR}" --config Release
ok "编译完成"

# ---- 7. 输出结果 ----
echo ""
echo "============================================================"
echo " 编译产物："
ls -lh "${BUILD_DEBUG_DIR}"/*.uf2 "${BUILD_DEBUG_DIR}"/*.elf 2>/dev/null || true
ls -lh "${BUILD_RELEASE_DIR}"/*.uf2 "${BUILD_RELEASE_DIR}"/*.elf 2>/dev/null || true
echo "============================================================"
echo " 刷机方法："
echo "   1. 按住 Pico 上的 BOOTSEL 按钮，插入 USB"
echo "   2. 将 build/ds5-bridge.uf2 拖入挂载的 RPI-RP2 磁盘"
echo "============================================================"
