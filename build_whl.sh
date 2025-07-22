# 本地编译 MuJoCo Python 脚本

# [1] 配置环境变量和路径
export REPO_DIR="$PWD"  # 当前在 MuJoCo 源码根目录
export TMPDIR="/tmp/mujoco_build"  # 临时目录

mkdir -p $TMPDIR # 创建临时目录编译项目
mkdir -p $REPO_DIR/build # 配置编译的源代码目录
mkdir -p $TMPDIR/mujoco_install/mujoco_plugin # 创建插件目录​


# ===================================[optional] start===============================
# [2] 推荐[python 3.11] 创建虚拟环境，安装构建依赖，也开始使用外部的conda虚拟环境，比如：
# sudo apt-get update && sudo apt-get install -y libgl1-mesa-dev libxinerama-dev libxcursor-dev libxrandr-dev libxi-dev ninja-build gcc-12 g++-12 python3.9 python3.9-venv
# python3.9 -m venv $TMPDIR/venv 

# 启动虚拟环境，安装依赖（根据 python/build_requirements.txt）
# source $TMPDIR/venv/bin/activate
# pip install --upgrade pip
# pip install -r $REPO_DIR/python/build_requirements.txt
# ===================================[optional] end===============================


# [3] 配置 CMake, 编译 MuJoCo，安装到临时目录
# 推荐：设置统一缓存路径
# export FETCHCONTENT_BASE_DIR=$HOME/.mujoco_deps_cache

cd $REPO_DIR/build
# cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF -DCMAKE_INSTALL_PREFIX=$TMPDIR/mujoco_install -DCMAKE_C_COMPILER=clang-10 -DCMAKE_CXX_COMPILER=clang++-10 -G Ninja
# cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF -DCMAKE_INSTALL_PREFIX=$TMPDIR/mujoco_install -DCMAKE_C_COMPILER=gcc-9 -DCMAKE_CXX_COMPILER=g++-9 -G Ninja -DFETCHCONTENT_BASE_DIR=$FETCHCONTENT_BASE_DIR
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF -DCMAKE_INSTALL_PREFIX=$TMPDIR/mujoco_install -DCMAKE_C_COMPILER=gcc-9 -DCMAKE_CXX_COMPILER=g++-9 -G Ninja
cmake --build . --config Release
cmake --install .


# [4] 安装插件，包含自定义插件的动态库
cp $REPO_DIR/build/lib/libactuator.* $TMPDIR/mujoco_install/mujoco_plugin/
cp $REPO_DIR/build/lib/libelasticity.* $TMPDIR/mujoco_install/mujoco_plugin/
cp $REPO_DIR/build/lib/libsensor.* $TMPDIR/mujoco_install/mujoco_plugin/
cp $REPO_DIR/build/lib/libsdf.* $TMPDIR/mujoco_install/mujoco_plugin/
cp $REPO_DIR/build/lib/libsimplesoft.* $TMPDIR/mujoco_install/mujoco_plugin/


# [5] 生成 sdist 包​，需要执行权限
cd $REPO_DIR/python
./make_sdist.sh
# ./make_sdist_quickly.sh


# [6] 构建 Wheel
cd dist
export MUJOCO_PATH="$TMPDIR/mujoco_install"
export MUJOCO_PLUGIN_PATH="$TMPDIR/mujoco_install/mujoco_plugin"
export MUJOCO_CMAKE_ARGS="-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF"
# export MUJOCO_CMAKE_ARGS="-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF -DFETCHCONTENT_BASE_DIR=$FETCHCONTENT_BASE_DIR"
pip wheel -v --no-deps mujoco-*.tar.gz
# 备注：最后一步消耗时间比较长，因为需要重新下载依赖的代码库


# [optional] 安装验证
# pip install --no-index mujoco-*.whl
# pytest -v --pyargs mujoco
# python example.py