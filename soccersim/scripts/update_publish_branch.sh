#!/usr/bin/env bash
set -euo pipefail

PUBLISH_BRANCH="publish"
SOURCE_REF="HEAD"
PUSH=0
FORCE_PUSH=0
REMOTE="origin"

usage() {
  cat <<'USAGE'
Usage: scripts/update_publish_branch.sh [--source REF] [--branch NAME] [--push] [--force-push] [--remote NAME]

Updates a publish branch so it contains only:
  docs/ res/ src/ .gitignore README.md agent.toml build.toml template.yaml

The publish snapshot excludes src/__init__.py.

Local development tooling such as pyproject.toml and uv.lock is intentionally
excluded from the publish snapshot.

By default it uses the current HEAD and updates the local "publish" branch.
Existing publish branches keep their own commit history; each run creates a new
publish commit when the whitelist content changes.

Use --push to push the publish branch to the remote. Use --force-push only when
the remote branch must be overwritten intentionally.

Examples:
  scripts/update_publish_branch.sh
  scripts/update_publish_branch.sh --source main --push
  scripts/update_publish_branch.sh --source release/1.0 --branch publish-release-1.0 --push
  scripts/update_publish_branch.sh --source feat/lesson --branch publish-lesson --force-push
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
    --branch)
      PUBLISH_BRANCH="${2:-}"
      if [ -z "$PUBLISH_BRANCH" ]; then
        echo "--branch requires a branch name" >&2
        exit 2
      fi
      shift 2
      ;;
    --push)
      PUSH=1
      shift
      ;;
    --force-push)
      PUSH=1
      FORCE_PUSH=1
      shift
      ;;
    --remote)
      REMOTE="${2:-}"
      if [ -z "$REMOTE" ]; then
        echo "--remote requires a remote name" >&2
        exit 2
      fi
      shift 2
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
source_commit="$(git rev-parse --verify "${SOURCE_REF}^{commit}")"
source_short="$(git rev-parse --short "$source_commit")"
worktree_dir="$(mktemp -d "${TMPDIR:-/tmp}/soccersim-publish.XXXXXX")"

cleanup() {
  git -C "$repo_root" worktree remove --force "$worktree_dir" >/dev/null 2>&1 || true
}
trap cleanup EXIT

if git -C "$repo_root" show-ref --verify --quiet "refs/heads/$PUBLISH_BRANCH"; then
  git -C "$repo_root" worktree add "$worktree_dir" "$PUBLISH_BRANCH" >/dev/null
else
  git -C "$repo_root" worktree add --detach "$worktree_dir" "$source_commit" >/dev/null
  git -C "$worktree_dir" switch --orphan "$PUBLISH_BRANCH" >/dev/null
fi

git -C "$worktree_dir" rm -r --ignore-unmatch . >/dev/null

git -C "$worktree_dir" checkout "$source_commit" -- \
  docs \
  res \
  src \
  .gitignore \
  README.md \
  agent.toml \
  build.toml \
  template.yaml

git -C "$worktree_dir" rm -f --ignore-unmatch src/__init__.py >/dev/null

git -C "$worktree_dir" add \
  docs \
  res \
  src \
  .gitignore \
  README.md \
  agent.toml \
  build.toml \
  template.yaml

if git -C "$worktree_dir" diff --cached --quiet; then
  echo "No publish changes for $PUBLISH_BRANCH from $SOURCE_REF ($source_short)."
  exit 0
fi

git -C "$worktree_dir" commit -m "Update publish snapshot from $source_short" >/dev/null
publish_commit="$(git -C "$worktree_dir" rev-parse HEAD)"

echo "Updated local branch $PUBLISH_BRANCH to $publish_commit from $SOURCE_REF ($source_short)."

if [ "$PUSH" -eq 1 ]; then
  if [ "$FORCE_PUSH" -eq 1 ]; then
    git -C "$repo_root" push --force "$REMOTE" "$PUBLISH_BRANCH:$PUBLISH_BRANCH"
    echo "Force-pushed $PUBLISH_BRANCH to $REMOTE."
  else
    git -C "$repo_root" push "$REMOTE" "$PUBLISH_BRANCH:$PUBLISH_BRANCH"
    echo "Pushed $PUBLISH_BRANCH to $REMOTE."
  fi
fi
