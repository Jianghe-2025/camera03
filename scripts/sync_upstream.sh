#!/usr/bin/env bash
# 将原作者仓库（默认远程名 upstream）的 main 合并进当前分支，按 docs/LOCAL_WORKTREE_OVERRIDES.md
# 对「整文件保留本 fork」的路径自动 checkout --ours，其余冲突则中止并提示手工处理。
# 合并成功后推送到自己的仓库（默认远程名 origin）。
#
# 用法（在仓库根目录）:
#   ./scripts/sync_upstream.sh
#   UPSTREAM_BRANCH=main ./scripts/sync_upstream.sh
# 仅拉取不合并: SYNC_UPSTREAM_FETCH_ONLY=1 ./scripts/sync_upstream.sh

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

UPSTREAM_REMOTE="${UPSTREAM_REMOTE:-upstream}"
ORIGIN_REMOTE="${ORIGIN_REMOTE:-origin}"
UPSTREAM_BRANCH="${UPSTREAM_BRANCH:-main}"
FETCH_ONLY="${SYNC_UPSTREAM_FETCH_ONLY:-0}"
NO_PUSH="${SYNC_UPSTREAM_NO_PUSH:-0}"

die() { echo "sync_upstream: $*" >&2; exit 1; }

git rev-parse --git-dir >/dev/null 2>&1 || die "请在 Git 仓库根目录下执行"

if ! git remote get-url "$UPSTREAM_REMOTE" >/dev/null 2>&1; then
  die "未找到远程「$UPSTREAM_REMOTE」。请先: git remote add $UPSTREAM_REMOTE git@github.com:asure1008/camera03.git"
fi
if ! git remote get-url "$ORIGIN_REMOTE" >/dev/null 2>&1; then
  die "未找到远程「$ORIGIN_REMOTE」。请先配置你的 fork 为 origin"
fi

if git diff --quiet && git diff --cached --quiet; then
  :
else
  die "工作区或暂存区有未提交改动，请先 commit 或 stash 后再同步"
fi

if [ -n "$(git ls-files -u)" ]; then
  die "当前已在合并/冲突状态，请先完成或中止（git merge --abort）后再运行"
fi

echo "==> git fetch $UPSTREAM_REMOTE"
git fetch "$UPSTREAM_REMOTE"

if [ "$FETCH_ONLY" = "1" ]; then
  echo "SYNC_UPSTREAM_FETCH_ONLY=1，已 fetch，跳过合并与推送。"
  exit 0
fi

UPSTREAM_REF="$UPSTREAM_REMOTE/$UPSTREAM_BRANCH"
git rev-parse --verify "$UPSTREAM_REF" >/dev/null 2>&1 || die "不存在引用: $UPSTREAM_REF"

CURRENT_BRANCH="$(git branch --show-current)"
if [ -z "$CURRENT_BRANCH" ]; then
  die "当前处于 detached HEAD，请先 checkout 到要同步的分支（如 main）"
fi

echo "==> 合并 $UPSTREAM_REF → 当前分支 $CURRENT_BRANCH"
set +e
git merge "$UPSTREAM_REF" -m "Merge $UPSTREAM_REF into $CURRENT_BRANCH"
MERGE_STATUS=$?
set -e

if [ "$MERGE_STATUS" -eq 0 ]; then
  echo "==> 合并完成（无冲突或已快进）"
  if [ "$NO_PUSH" = "1" ]; then
    echo "SYNC_UPSTREAM_NO_PUSH=1，跳过推送。"
    exit 0
  fi
  echo "==> git push $ORIGIN_REMOTE $CURRENT_BRANCH"
  git push "$ORIGIN_REMOTE" "$CURRENT_BRANCH"
  exit 0
fi

# 有冲突：对约定为「整文件保留本 fork」的路径自动 --ours（与 LOCAL_WORKTREE_OVERRIDES.md §6 一致）
KEEP_OURS=(
  ptz_config.yaml
  mediamtx.yml
  ptz_web_control.html
  test_onvif.py
  README.md
  docs/API.md
  .gitignore
)

MANUAL_HINT=(
  ptz_launcher.py
  ptz_stream.py
)

resolve_ours_for() {
  local path="$1"
  echo "    保留本 fork 整文件: $path"
  git checkout --ours -- "$path"
  git add -- "$path"
}

mapfile -t UNMERGED < <(git diff --name-only --diff-filter=U 2>/dev/null || true)
if [ "${#UNMERGED[@]}" -eq 0 ]; then
  die "合并失败但未检测到未合并文件，请手动查看 git status"
fi

echo "==> 检测到冲突，按规则自动处理「整文件保留本 fork」项…"
for f in "${UNMERGED[@]}"; do
  keep=0
  for k in "${KEEP_OURS[@]}"; do
    if [ "$f" = "$k" ]; then
      keep=1
      break
    fi
  done
  if [ "$keep" -eq 1 ]; then
    resolve_ours_for "$f"
  fi
done

mapfile -t STILL < <(git diff --name-only --diff-filter=U 2>/dev/null || true)
if [ "${#STILL[@]}" -gt 0 ]; then
  echo "sync_upstream: 以下文件仍需手工合并（参见 docs/LOCAL_WORKTREE_OVERRIDES.md）：" >&2
  for f in "${STILL[@]}"; do
    echo "  - $f" >&2
    for m in "${MANUAL_HINT[@]}"; do
      if [ "$f" = "$m" ]; then
        echo "    （核心逻辑文件，通常需对照文档逐段合并）" >&2
      fi
    done
  done
  echo >&2
  echo "处理完后执行: git add <文件> && git commit --no-edit  （或自行写合并说明）" >&2
  echo "若放弃本次合并: git merge --abort" >&2
  exit 1
fi

echo "==> 冲突已按规则解决，完成合并提交"
git commit --no-edit

if [ "$NO_PUSH" = "1" ]; then
  echo "SYNC_UPSTREAM_NO_PUSH=1，跳过推送。"
  exit 0
fi

echo "==> git push $ORIGIN_REMOTE $CURRENT_BRANCH"
git push "$ORIGIN_REMOTE" "$CURRENT_BRANCH"
