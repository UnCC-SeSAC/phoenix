#!/usr/bin/env bash
# 패키지 무결성 검사 및 복구.
#
#   bash check_package.sh          # 검사만
#   bash check_package.sh --fix    # 없으면 만들어줌
#
# 왜 필요한가:
#   ament_python 패키지에는 **0바이트 파일**과 **점(.) 파일**이 있는데,
#   웹 다운로드·복사·압축 과정에서 조용히 사라지기 쉽습니다.
#   없어도 로컬 테스트(pytest)는 멀쩡히 통과하지만 `colcon build` 후
#   `ros2 run` 이 실패해서, 원인을 엉뚱한 데서 찾게 됩니다.

set -u
cd "$(dirname "$0")"

FIX=0
[ "${1:-}" = "--fix" ] && FIX=1

MISSING=0
BUILD_BREAKING=0

check() {
	local path="$1" why="$2" critical="$3"
	if [ -e "$path" ]; then
		printf '  [OK]      %-30s\n' "$path"
		return
	fi
	MISSING=$((MISSING + 1))
	[ "$critical" = "yes" ] && BUILD_BREAKING=$((BUILD_BREAKING + 1))
	printf '  [MISSING] %-30s %s\n' "$path" "$why"

	if [ "$FIX" = "1" ]; then
		mkdir -p "$(dirname "$path")"
		case "$path" in
			resource/image_pipeline|image_pipeline/__init__.py)
				touch "$path"
				printf '            -> 빈 파일로 생성했습니다\n'
				;;
			setup.cfg)
				printf '[develop]\nscript_dir=$base/lib/image_pipeline\n[install]\ninstall_scripts=$base/lib/image_pipeline\n' > setup.cfg
				printf '            -> 재생성했습니다\n'
				;;
			.gitignore)
				cat > .gitignore <<'GITEOF'
__pycache__/
*.py[cod]
.pytest_cache/
local_check/
out/
testdata/
*.bag/
dataset/
raw_frames/
build/
install/
log/
.vscode/
.idea/
.DS_Store
GITEOF
				printf '            -> 재생성했습니다\n'
				;;
			*)
				printf '            -> 자동 복구 불가. 원본에서 가져오세요\n'
				;;
		esac
	fi
}

echo "=== ROS 빌드에 필수 (없으면 colcon/ros2 run 실패) ==="
check "resource/image_pipeline"     "ament index 마커. 0바이트라 누락되기 쉬움" yes
check "image_pipeline/__init__.py"  "파이썬 패키지 마커. 0바이트"              yes
check "setup.cfg"                    "실행파일 설치 경로 지정"                   yes
check "package.xml"                  "패키지 매니페스트"                         yes
check "setup.py"                     "entry_points 정의"                         yes

echo
echo "=== 있으면 좋음 ==="
check ".gitignore"                   "점 파일이라 다운로드 시 숨겨짐"            no

echo
echo "=== 소스 ==="
for f in image_pipeline/dehaze.py image_pipeline/pipeline.py \
         image_pipeline/intrinsics.py image_pipeline/autotune.py \
         image_pipeline/preprocess_node.py image_pipeline/fake_camera_node.py \
         image_pipeline/depth.py image_pipeline/detection_msgs.py \
         image_pipeline/fake_detection_node.py image_pipeline/detection_3d_node.py \
         image_pipeline/detection3d.py image_pipeline/detection_json.py \
         image_pipeline/aodnet.py \
         image_pipeline/yolo.py image_pipeline/yolo_node.py \
         image_pipeline/fire_status.py \
         tests/test_dehaze.py tests/test_depth.py tests/test_detection3d.py \
         tests/test_detection_json.py tests/test_yolo.py \
         tests/test_fire_status.py \
         tools/detect_offline.py tools/check_yolo_wiring.py \
         launch/full_chain_check.launch.py launch/yolo.launch.py \
         config/preprocess.yaml launch/preprocess.launch.py; do
	check "$f" "소스 파일" yes
done

echo
if [ "$MISSING" -eq 0 ]; then
	echo "전부 정상입니다."
	exit 0
fi

if [ "$FIX" = "1" ]; then
	echo "복구 시도 완료. 다시 검사하려면 인자 없이 실행하세요."
else
	echo "$MISSING개 누락 (빌드 차단 $BUILD_BREAKING개)."
	echo "복구: bash check_package.sh --fix"
fi
[ "$BUILD_BREAKING" -gt 0 ] && exit 1
exit 0
