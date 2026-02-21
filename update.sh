#!/bin/bash

# 커밋 메시지 설정
if [ -z "$1" ]; then
  # 인자가 없으면 날짜 기반 메시지
  commit_msg="chore: $(date)"
else
  # 인자가 있으면 해당 메시지 사용
  commit_msg="$1"
fi

# Git 명령어 실행
git add .
git commit -m "$commit_msg"
git push origin master
