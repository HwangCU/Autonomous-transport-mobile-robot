#!/bin/bash

# 기존 실행 중인 컨테이너 ID
CONTAINER_ID="99dbd0745403"

# DISPLAY 환경 변수 설정 (호스트의 DISPLAY를 컨테이너에 전달)
export DISPLAY=$DISPLAY

# X11 소켓 마운트 및 기존 컨테이너 시작
docker start -i $CONTAINER_ID

# X11 소켓 연결: 실행된 컨테이너 내에서 GUI 애플리케이션 실행
docker exec -it $CONTAINER_ID bash -c "export DISPLAY=$DISPLAY && bash"
