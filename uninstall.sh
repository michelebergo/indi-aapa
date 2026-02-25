#!/bin/bash
# ============================================================
#  INDI-AAPA Uninstaller
#  Removes all files installed by install.sh
# ============================================================
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Error: Please run as root (sudo ./uninstall.sh)${NC}"
    exit 1
fi

echo -e "${YELLOW}Removing INDI-AAPA files...${NC}"

FILES=(
    /usr/bin/indi_aapa_polaralignment
    /usr/share/indi/indi_aapa_polaralignment.xml
    /usr/local/bin/aapa_closed_loop.sh
    /usr/local/bin/auto_aapa.sh
)

for f in "${FILES[@]}"; do
    if [ -f "$f" ]; then
        rm -f "$f"
        echo "  ✓ Removed $f"
    else
        echo "  - Not found: $f (skipped)"
    fi
done

echo ""
echo -e "${GREEN}Uninstall complete.${NC}"
