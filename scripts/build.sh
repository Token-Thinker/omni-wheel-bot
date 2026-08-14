#!/usr/bin/env bash
set -euo pipefail

__DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
source "$__DIR/lib.sh"

build_firmware() {
  if [[ -z "${MAKER-}" || -z "${BOARD-}" || -z "${BIN-}" || -z "${TRIPLE-}" ]]; then
    err "Missing build context (MAKER/BOARD/BIN/TRIPLE)."
    return 1
  fi

  section "Build (${MAKER}/${BOARD}/${BIN})"

  local plugin="$__DIR/makers/${MAKER}.sh"
  if [[ -f "$plugin" ]]; then
    # shellcheck disable=SC1090
    source "$plugin"
    if is_dry_run; then
      note "[dry-run] maker_build for $MAKER ($BOARD/$BIN)"
    else
      if ! maker_build; then
        err "Build failed (${MAKER}/${BOARD}/${BIN})"
        return 1
      fi
    fi
  else
    note "No maker plugin for '$MAKER'. Using host cargo build fallback."
    if is_dry_run; then
      note "[dry-run] cargo build --release --target $TRIPLE --bin $BIN (in owb-bin/$MAKER)"
    else
      if ! ( cd "owb-bin/$MAKER" && cargo build --release --target "$TRIPLE" --bin "$BIN" -q ); then
        err "Build failed (${MAKER}/${BOARD}/${BIN})"
        return 1
      fi
    fi
  fi

  if ! is_dry_run && [[ ! "$MAKER" =~ ^(owb-host-sim|host|sim)$ ]]; then
    local artifact
    artifact=$(cargo_release_artifact_path) || {
      err "Missing build context (TRIPLE/BIN) for artifact lookup."
      return 1
    }
    if [[ ! -f "$artifact" ]]; then
      err "Build failed (missing artifact: $artifact)"
      return 1
    fi
  fi

  ok "Build complete"
  echo
}
