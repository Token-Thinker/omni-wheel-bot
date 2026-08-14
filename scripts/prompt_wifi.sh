#!/usr/bin/env bash
set -euo pipefail

__DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
source "$__DIR/lib.sh"
source "$__DIR/ui.sh"

prompt_wifi() {
  section "Why we ask for your Wi‑Fi password"
  echo -e "  • Injected only into this script’s environment"
  echo -e "  • Never written to disk or git"
  echo

  if [[ "${OWB_FORCE_SOFTAP:-0}" == "1" ]]; then
    SSID=""
    PASSWORD=""
    note "OWB_FORCE_SOFTAP=1 set: skipping STA credentials and using SoftAP fallback."
    export SSID PASSWORD
    return 0
  fi

  # --use-test-network no longer injects hardcoded credentials.
  # Set SSID and PASSWORD in your environment before calling this script, or
  # source a .env file: cp .env.example .env && source .env
  if [[ "${USE_TEST_NETWORK:-false}" == true ]]; then
    if [[ -z "${SSID-}" || -z "${PASSWORD-}" ]]; then
      warn "--use-test-network requires SSID and PASSWORD to be set in the environment."
      warn "Copy .env.example to .env, fill in your credentials, and run: source .env"
    fi
  fi

  if [[ -z "${SSID-}" && -z "${PASSWORD-}" ]]; then
    if tui_available; then
      local choice
      choice=$(choose "Wi‑Fi" "Enter custom network" "Use SoftAP (no STA)") || true
      if [[ "$choice" == "Use SoftAP (no STA)" ]]; then
        SSID=""
        PASSWORD=""
        export SSID PASSWORD
        return 0
      fi
    fi

    if [[ -z "${SSID-}" || -z "${PASSWORD-}" ]]; then
      read -r -p "$(echo -e "${YELLOW}"Enter Wi-Fi SSID:"${RESET}" )" SSID
      read -r -s -p "$(echo -e "${YELLOW}"Enter Wi-Fi Password:"${RESET}" )" PASSWORD
      echo
    fi
  fi

  if [[ "${DEBUG:-false}" == true ]]; then
    echo -e "${BLUE}${BOLD}[DEBUG] You entered:${RESET}"
    echo -e "  SSID:     ${GREEN}$SSID${RESET}"
    echo -e "  Password: ${GREEN}(hidden)${RESET}"
    read -r -p "$(echo -e "${YELLOW}Proceed with these? [y/N]:${RESET}" )" CONF
    case "$CONF" in
      [Yy]|[Yy][Ee][Ss]) echo -e "${GREEN}Continuing…${RESET}" ;;
      *) echo -e "${RED}Aborted by user.${RESET}"; return 1 ;;
    esac
  fi

  export SSID PASSWORD
}
