#!/usr/bin/env bash
# setup_x11_forwarding.sh
#
# Prepare X11 forwarding for the native-Linux Docker override so the
# carla_manual_control pygame viewer and rviz2 can open over an SSH-forwarded
# (TCP) display, e.g. DISPLAY=localhost:NN.0.
#
# Why this is needed every session:
#   * SSH X11 forwarding assigns a NEW display number each login (:10, :12, ...),
#     so a container created in a previous session points at a dead display.
#   * A container needs an X auth cookie; the host's cookies are FamilyLocal
#     (host/unix:NN) which a TCP "localhost" connection won't match, so we
#     rewrite the family to FamilyWild (ffff) — matches regardless of hostname.
#   * The cookie file MUST exist as a real file before `docker compose up`,
#     otherwise Docker creates the bind-mount source as an empty directory.
#
# Usage (on the remote host, inside the SSH session that has $DISPLAY set):
#   ./scripts/setup_x11_forwarding.sh           # prep cookie only
#   ./scripts/setup_x11_forwarding.sh --up       # prep cookie + (re)create services
set -euo pipefail

XAUTH=/tmp/.docker.xauth

if [ -z "${DISPLAY:-}" ]; then
  echo "ERROR: \$DISPLAY is not set. Connect with SSH X11 forwarding (ssh -X/-Y)." >&2
  exit 1
fi

# 1b. Stale-display check (informational, never fatal). With X11UseLocalhost=yes
#     (the sshd default) each forwarded session's display lives on a TCP socket at
#     127.0.0.1:(6000+N) -- NOT in /tmp/.X11-unix, which holds only LOCAL displays
#     like :0/:1. sshd hands out the lowest free N >= X11DisplayOffset (default 10),
#     so if an earlier SSH session is still alive the number CLIMBS (:10 -> :11 ...),
#     which is why $DISPLAY seems to "change every run". Report what holds the lower
#     numbers so stale sessions can be reaped.
check_stale_x11() {
  local host="${DISPLAY%%:*}" rest="${DISPLAY#*:}" dnum offset=10 cfg busy
  dnum="${rest%%.*}"
  cfg=$(grep -sE '^[[:space:]]*X11DisplayOffset[[:space:]]+[0-9]+' /etc/ssh/sshd_config 2>/dev/null | awk '{print $2}' | tail -1 || true)
  [ -n "$cfg" ] && offset="$cfg"

  # Local console display (e.g. :0/:1): only unix sockets are relevant.
  if [ -z "$host" ] || [ "$host" = "unix" ]; then
    echo "Local console display ($DISPLAY); X11 sockets in /tmp/.X11-unix:"
    ls -1 /tmp/.X11-unix/ 2>/dev/null | sed 's/^/  /' || true
    return 0
  fi

  # Forwarded (TCP) display: enumerate the active X11 forward ports.
  if ! command -v ss >/dev/null 2>&1; then
    echo "(install iproute2 'ss' to enumerate forwarded X11 displays)"
    return 0
  fi
  busy=$(ss -ltnH 2>/dev/null | grep -oE '127\.0\.0\.1:60[0-9][0-9]' | grep -oE '60[0-9][0-9]$' | sort -un || true)
  if [ -n "$busy" ]; then
    echo "Active X11 forward displays (TCP 6000+N):"
    while read -r p; do
      [ -z "$p" ] && continue
      if [ "$((p - 6000))" = "$dnum" ]; then
        printf '  :%s  (port %s)  <- current $DISPLAY\n' "$((p - 6000))" "$p"
      else
        printf '  :%s  (port %s)  <- held by another SSH session\n' "$((p - 6000))" "$p"
      fi
    done <<< "$busy"
  fi
  if [ "${dnum:-0}" -gt "$offset" ] 2>/dev/null; then
    echo "WARNING: \$DISPLAY has climbed to :$dnum (sshd offset $offset)." >&2
    echo "         Lower numbers are held by earlier SSH sessions that never released," >&2
    echo "         so each new login increments. Identify and close the stale ones:" >&2
    echo "           who                      # lingering pts/* logins" >&2
    echo "           ss -ltnp | grep ':60'    # which sshd PID owns each forward port" >&2
    echo "         Logging those sessions out frees the lower display numbers." >&2
  fi
  return 0
}
check_stale_x11

# 1. Ensure the cookie path is a fresh, world-readable FILE (not a directory
#    that a previous `docker compose up` may have auto-created).
if [ -d "$XAUTH" ]; then
  sudo rm -rf "$XAUTH"
else
  rm -f "$XAUTH"
fi
touch "$XAUTH"
chmod 644 "$XAUTH"

# 2. Write a wildcard (FamilyWild) cookie for THIS session's display only.
#    The ffff rewrite wildcards the HOSTNAME but keeps the display number, so
#    merging ALL host cookies is unsafe: stale cookies from a previous hostname
#    (e.g. a machine renamed Velox-169392 -> velox1) collapse to the same
#    "ffff:NN" key with a DIFFERENT cookie value. A localhost:NN connection then
#    matches whichever wild entry comes FIRST in the file, which can be the stale
#    one -> "MoTTY X11 proxy: Authorisation not recognised". Scoping to $DISPLAY
#    pulls only the live entry for the current display. Re-run this script after
#    switching SSH sessions (the display number changes each login).
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -

if ! xauth -f "$XAUTH" list | grep -q .; then
  echo "ERROR: no cookie written for DISPLAY=$DISPLAY. Is X11 forwarding active?" >&2
  exit 1
fi

echo "X11 cookie ready for DISPLAY=$DISPLAY:"
xauth -f "$XAUTH" list

# 3. Optionally (re)create the services so they pick up the live DISPLAY + cookie.
if [ "${1:-}" = "--up" ]; then
  docker compose \
    -f docker-compose.yml \
    -f docker-compose-native.override.yml \
    up -d --force-recreate carla-server carla-ros-bridge
fi
