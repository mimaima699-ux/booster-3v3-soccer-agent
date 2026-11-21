#!/usr/bin/env bash
set -euo pipefail

# 方案 2：内部仓库 + 独立 GitHub 公开镜像仓库。
#
# 推荐目录结构：
#   SoccerSim/                  内部开发仓库，保留 note/ tests/ scripts/ 等完整内容
#   booster_champion_example/   GitHub 公开仓库，只保存可公开发布的快照
#
# 发布流程：
# 1. 先在内部仓库提交要发布的改动。本脚本默认从 HEAD 导出，不会包含未提交内容。
# 2. 脚本把白名单文件导出到独立公开仓库目录，默认是内部仓库同级的
#    ../booster_champion_example。
# 3. 脚本会清空公开仓库工作区中除 .git、.gitignore 和 LICENSE 以外的文件，
#    再复制公开文件，避免旧文件残留；公开仓库的 .gitignore 和 LICENSE 由公开仓库手动维护。
# 4. 公开范围只包含：
#      docs/ res/ src/ README.md agent.toml build.toml
# 5. 公开范围明确不包含：
#      .gitignore LICENSE template.yaml src/__init__.py pyproject.toml uv.lock note/ tests/ scripts/
# 6. 默认只导出并展示公开仓库状态；确认无误后，可以手动 commit/push，
#    或使用本脚本的 --commit / --push。

PUBLIC_REPO_NAME="booster_champion_example"
SOURCE_REF="HEAD"
OUT_DIR=""
ALLOW_DIRTY=0
INIT_REPO=0
COMMIT=0
PUSH=0
FORCE_PUSH=0
REMOTE="origin"
TARGET_BRANCH="main"
COMMIT_MESSAGE=""
YES=0

usage() {
  cat <<'USAGE'
Usage: scripts/export_github_repo.sh [options]

把内部仓库的公开白名单内容导出到独立 GitHub 公开仓库目录。
默认公开仓库目录为当前仓库同级的 ../booster_champion_example。

导出内容只包含：
  docs/ res/ src/ README.md agent.toml build.toml

导出内容明确排除：
  .gitignore LICENSE template.yaml src/__init__.py pyproject.toml uv.lock note/ tests/ scripts/

如果公开仓库需要 .gitignore 或 LICENSE，请在公开仓库中手动维护；
本脚本不会覆盖它们。

常用流程：
  # 1. 在内部仓库提交改动后，导出到同级公开仓库目录
  scripts/export_github_repo.sh

  # 2. 检查公开仓库内容
  cd ../booster_champion_example
  git status
  git diff --stat

  # 3. 手动提交并推送
  git add -A
  git commit -m "Update public template"
  git push origin main

也可以让脚本提交或推送：
  scripts/export_github_repo.sh --commit
  scripts/export_github_repo.sh --push

Options:
  --source REF          从指定 commit/ref 导出，默认 HEAD
  --out DIR             导出到指定公开仓库目录，默认 ../booster_champion_example
  --allow-dirty         允许内部仓库有未提交改动；这些改动不会被导出
  --init                如果目标目录还不是 git 仓库，则执行 git init
  --commit              导出后在公开仓库创建提交
  --message MSG         指定 --commit 使用的提交信息
  --push                导出后提交并推送到远端；隐含 --commit
  --remote NAME         推送使用的 remote，默认 origin
  --target-branch NAME  推送目标分支，默认 main
  --force-push          强制推送；只有确认要覆盖远端分支时才使用
  --yes                 允许清空非 git 目标目录中的已有文件
  -h, --help            显示帮助
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --source)
      SOURCE_REF="${2:-}"
      if [ -z "$SOURCE_REF" ]; then
        echo "--source requires a ref" >&2
        exit 2
      fi
      shift 2
      ;;
    --out)
      OUT_DIR="${2:-}"
      if [ -z "$OUT_DIR" ]; then
        echo "--out requires a directory" >&2
        exit 2
      fi
      shift 2
      ;;
    --allow-dirty)
      ALLOW_DIRTY=1
      shift
      ;;
    --init)
      INIT_REPO=1
      shift
      ;;
    --commit)
      COMMIT=1
      shift
      ;;
    --message)
      COMMIT_MESSAGE="${2:-}"
      if [ -z "$COMMIT_MESSAGE" ]; then
        echo "--message requires text" >&2
        exit 2
      fi
      shift 2
      ;;
    --push)
      COMMIT=1
      PUSH=1
      shift
      ;;
    --remote)
      REMOTE="${2:-}"
      if [ -z "$REMOTE" ]; then
        echo "--remote requires a name" >&2
        exit 2
      fi
      shift 2
      ;;
    --target-branch)
      TARGET_BRANCH="${2:-}"
      if [ -z "$TARGET_BRANCH" ]; then
        echo "--target-branch requires a branch name" >&2
        exit 2
      fi
      shift 2
      ;;
    --force-push)
      COMMIT=1
      PUSH=1
      FORCE_PUSH=1
      shift
      ;;
    --yes)
      YES=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

repo_root="$(git rev-parse --show-toplevel)"
repo_root_abs="$(cd "$repo_root" && pwd -P)"
repo_parent="$(dirname "$repo_root_abs")"

if [ -z "$OUT_DIR" ]; then
  OUT_DIR="$repo_parent/$PUBLIC_REPO_NAME"
fi

source_commit="$(git rev-parse --verify "${SOURCE_REF}^{commit}")"
source_short="$(git rev-parse --short "$source_commit")"

if [ "$ALLOW_DIRTY" -eq 0 ] && [ "$SOURCE_REF" = "HEAD" ]; then
  if [ -n "$(git -C "$repo_root_abs" status --porcelain)" ]; then
    echo "内部仓库有未提交改动。请先提交，或显式使用 --allow-dirty。" >&2
    exit 1
  fi
fi

mkdir -p "$OUT_DIR"
out_abs="$(cd "$OUT_DIR" && pwd -P)"

case "$out_abs" in
  "$repo_root_abs"|"$repo_root_abs"/*)
    echo "目标目录不能是内部仓库本身或其子目录：$out_abs" >&2
    exit 1
    ;;
esac

if [ "$out_abs" = "/" ]; then
  echo "拒绝使用根目录作为导出目标。" >&2
  exit 1
fi

if [ ! -d "$out_abs/.git" ]; then
  if [ "$INIT_REPO" -eq 1 ]; then
    git -C "$out_abs" init -b "$TARGET_BRANCH" >/dev/null 2>&1 || {
      git -C "$out_abs" init >/dev/null
      git -C "$out_abs" checkout -B "$TARGET_BRANCH" >/dev/null
    }
  elif [ -n "$(find "$out_abs" -mindepth 1 -maxdepth 1 -print -quit)" ] && [ "$YES" -eq 0 ]; then
    echo "目标目录不是 git 仓库且已有文件：$out_abs" >&2
    echo "为避免误删，请换空目录、使用 --init 初始化 git 仓库，或加 --yes。" >&2
    exit 1
  fi
fi

echo "导出来源：$SOURCE_REF ($source_short)"
echo "导出目标：$out_abs"

find "$out_abs" -mindepth 1 -maxdepth 1 ! -name .git ! -name .gitignore ! -name LICENSE -exec rm -rf {} +

git -C "$repo_root_abs" archive "$source_commit" \
  docs \
  res \
  src \
  README.md \
  agent.toml \
  build.toml | tar -x -C "$out_abs"

rm -f "$out_abs/src/__init__.py"

echo
echo "已导出的脚本管理文件："
find \
  "$out_abs/docs" \
  "$out_abs/res" \
  "$out_abs/src" \
  "$out_abs/README.md" \
  "$out_abs/agent.toml" \
  "$out_abs/build.toml" \
  -type f -print \
  | sed "s#^$out_abs/##" \
  | sort

if [ ! -d "$out_abs/.git" ]; then
  echo
  echo "目标目录不是 git 仓库。检查无误后，可在该目录执行 git init / git remote add / git push。"
  exit 0
fi

git -C "$out_abs" add -A -- \
  docs \
  res \
  src \
  README.md \
  agent.toml \
  build.toml

while IFS= read -r -d '' tracked_path; do
  git -C "$out_abs" rm -r --cached --ignore-unmatch -- "$tracked_path" >/dev/null
done < <(
  git -C "$out_abs" ls-files -z -- \
    template.yaml \
    pyproject.toml \
    uv.lock \
    note \
    tests \
    scripts
)

echo
echo "公开仓库待提交变更（不含未纳入脚本管理的手动文件）："
git -C "$out_abs" diff --cached --name-status

if git -C "$out_abs" diff --cached --quiet; then
  echo "没有需要提交的公开仓库改动。"
else
  if [ "$COMMIT" -eq 1 ]; then
    if [ -z "$COMMIT_MESSAGE" ]; then
      COMMIT_MESSAGE="Update public snapshot from $source_short"
    fi
    git -C "$out_abs" commit -m "$COMMIT_MESSAGE" >/dev/null
    echo "已在公开仓库提交：$COMMIT_MESSAGE"
  else
    echo "尚未提交。确认文件无误后，在目标目录手动提交，或重新运行并加 --commit。"
  fi
fi

if [ "$PUSH" -eq 1 ]; then
  if [ "$FORCE_PUSH" -eq 1 ]; then
    git -C "$out_abs" push --force "$REMOTE" "HEAD:$TARGET_BRANCH"
    echo "已强制推送到 $REMOTE/$TARGET_BRANCH。"
  else
    git -C "$out_abs" push "$REMOTE" "HEAD:$TARGET_BRANCH"
    echo "已推送到 $REMOTE/$TARGET_BRANCH。"
  fi
fi
