cmake_minimum_required(VERSION 3.5)
project(first_challenge)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_lint_auto REQUIRED)  # 追加: Lint用のパッケージを探す

# package.xmlのdependをチェックして自動的にfind_packageしてくれる
ament_auto_find_build_dependencies()

# 実行ファイルを作成する
ament_auto_add_executable(${PROJECT_NAME}
  src/first_challenge.cpp)

# launchファイルをインストール（ament_package_indexが探せる場所に置く）
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Lintのテスト依存関係を自動で設定
ament_lint_auto_find_test_dependencies()  # 追加

# ヘッダーのインストールや共有ライブラリのエクスポート等を自動で行ってくれる
ament_auto_package()
