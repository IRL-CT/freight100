#!/usr/bin/env bash
# Publish the D435i viewer on a public HTTPS URL via a Cloudflare quick tunnel.
#
# Usage:  ./stream_online.sh [stream_fps]      (default 12)
#
# Starts rs_view.py in web mode with a fresh random access token, tunnels
# port 8091 with cloudflared, and prints the public URL (which embeds the
# token). Ctrl+C stops both the viewer and the tunnel. The URL is different
# on every run — quick tunnels are ephemeral and carry no uptime guarantee.
set -euo pipefail
cd "$(dirname "$0")"

FPS=${1:-12}
CLOUDFLARED=${CLOUDFLARED:-$HOME/.local/bin/cloudflared}
TOKEN=$(python3 -c "import secrets; print(secrets.token_urlsafe(12))")
LOG=$(mktemp /tmp/rs_view_tunnel.XXXXXX.log)

python3 rs_view.py --web --token "$TOKEN" --stream-fps "$FPS" &
VIEW_PID=$!
"$CLOUDFLARED" tunnel --url http://localhost:8091 --no-autoupdate >"$LOG" 2>&1 &
TUN_PID=$!
trap 'kill $VIEW_PID $TUN_PID 2>/dev/null' EXIT INT TERM

URL=""
for _ in $(seq 1 30); do
    URL=$(grep -oE 'https://[a-z0-9-]+\.trycloudflare\.com' "$LOG" | head -1 || true)
    [ -n "$URL" ] && break
    sleep 1
done
if [ -z "$URL" ]; then
    echo "Tunnel did not come up — see $LOG" >&2
    exit 1
fi

echo
echo "  PUBLIC VIEW URL:   $URL/?token=$TOKEN"
echo
echo "Share that full URL (the token in it is the only access control)."
echo "Ctrl+C here stops the stream and invalidates the URL."
wait $VIEW_PID
