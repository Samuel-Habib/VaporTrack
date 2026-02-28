#!/usr/bin/env bash
# ==============================================================================
# export_gerbers.sh — KiCad Automated Gerber & Drill Packager for JLCPCB/PCBWay
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PCB_FILE="${1:-}"

# Auto-detect PCB file if not specified
if [[ -z "$PCB_FILE" ]]; then
    PCB_CANDIDATES=( "$SCRIPT_DIR"/*.kicad_pcb )
    if [[ ${#PCB_CANDIDATES[@]} -eq 1 && -f "${PCB_CANDIDATES[0]}" ]]; then
        PCB_FILE="${PCB_CANDIDATES[0]}"
    else
        echo "Error: Please specify the .kicad_pcb file to export."
        echo "Usage: $0 [path/to/board.kicad_pcb]"
        exit 1
    fi
fi

if [[ ! -f "$PCB_FILE" ]]; then
    echo "Error: PCB file '$PCB_FILE' not found."
    exit 1
fi

BOARD_NAME="$(basename "$PCB_FILE" .kicad_pcb)"
BOARD_DIR="$(cd "$(dirname "$PCB_FILE")" && pwd)"
OUTPUT_DIR="${BOARD_DIR}/gerbers"
ZIP_FILE="${BOARD_DIR}/${BOARD_NAME}_Gerbers.zip"

echo "=========================================================="
echo "  Gerber & Drill Export: ${BOARD_NAME}"
echo "  Board file:  ${PCB_FILE}"
echo "  Output dir:  ${OUTPUT_DIR}"
echo "  Target zip:  ${ZIP_FILE}"
echo "=========================================================="

# Re-create clean output directory
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

# 1. Export standard 2-layer Gerbers
echo "--> [1/3] Exporting Gerber layers..."
kicad-cli pcb export gerbers \
    --layers "F.Cu,B.Cu,F.Mask,B.Mask,F.Silkscreen,B.Silkscreen,F.Paste,B.Paste,Edge.Cuts" \
    --output "$OUTPUT_DIR/" \
    "$PCB_FILE"

# 2. Export Excellon drill files (Plated & Non-Plated PTH/NPTH)
echo "--> [2/3] Exporting Excellon drill files..."
kicad-cli pcb export drill \
    --excellon-units mm \
    --excellon-separate-th \
    --generate-map \
    --map-format pdf \
    --output "$OUTPUT_DIR/" \
    "$PCB_FILE"

# 3. Create production zip archive for JLCPCB/PCBWay
echo "--> [3/3] Packaging production fabrication ZIP archive..."
rm -f "$ZIP_FILE"

FAB_FILES=(
    "${OUTPUT_DIR}/${BOARD_NAME}-F_Cu.gtl"
    "${OUTPUT_DIR}/${BOARD_NAME}-B_Cu.gbl"
    "${OUTPUT_DIR}/${BOARD_NAME}-F_Mask.gts"
    "${OUTPUT_DIR}/${BOARD_NAME}-B_Mask.gbs"
    "${OUTPUT_DIR}/${BOARD_NAME}-F_Silkscreen.gto"
    "${OUTPUT_DIR}/${BOARD_NAME}-B_Silkscreen.gbo"
    "${OUTPUT_DIR}/${BOARD_NAME}-F_Paste.gtp"
    "${OUTPUT_DIR}/${BOARD_NAME}-B_Paste.gbp"
    "${OUTPUT_DIR}/${BOARD_NAME}-Edge_Cuts.gm1"
    "${OUTPUT_DIR}/${BOARD_NAME}-PTH.drl"
    "${OUTPUT_DIR}/${BOARD_NAME}-NPTH.drl"
)

# Include job file if generated
if [[ -f "${OUTPUT_DIR}/${BOARD_NAME}-job.gbrjob" ]]; then
    FAB_FILES+=( "${OUTPUT_DIR}/${BOARD_NAME}-job.gbrjob" )
fi

(
    cd "$OUTPUT_DIR"
    zip -j "$ZIP_FILE" "${FAB_FILES[@]}"
)

echo ""
echo "=========================================================="
echo "  ✓ Gerber Export & Packaging Complete!"
echo "  Archive: ${ZIP_FILE}"
echo "=========================================================="
unzip -l "$ZIP_FILE"
echo ""
echo "Ready for direct upload to JLCPCB, PCBWay, or OSH Park."
