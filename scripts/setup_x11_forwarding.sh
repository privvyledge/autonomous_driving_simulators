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
