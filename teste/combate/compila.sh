#!/usr/bin/env bash
# Compilação do simulador de combate. Cada módulo do firmware entra duas vezes, uma no namespace
# fa (robô do Grupo D) e outra no fb (adversário), pelo fw_modulo.cpp; o firmware não muda.
# Uso: bash teste/combate/compila.sh [nativo|wasm] [saída]
#   nativo (padrão): executável de linha de comando (main_nativo.cpp), com o g++
#   wasm: módulo WebAssembly da página 3D (wasm_api.cpp), com o zig 0.14 (ZIG=/caminho/do/zig ou
#         ZIG="python -m ziglang" depois de pip install ziglang==0.14.0)
set -euo pipefail
cd "$(dirname "$0")/../.."
ALVO=${1:-nativo}
DIR=teste/combate
GEN=$(mktemp -d)
trap 'rm -rf "$GEN"' EXIT
MODULOS="percepcao oponente rastreador borda estados cinematica motores telemetria arena chave antitrava"
FLAGS="-std=c++17 -O2 -DGEOMETRIA_BANCADA=0 -DUSAR_MOTORES=1 -DQTD_TOF=7 -I $DIR/stub -I teste/sim/stub -I $DIR -I firmware/sumo_esp32"
FONTES=""
for ns in fa fb; do
  for m in $MODULOS; do
    printf '#define FW_NS %s\n#define FW_ARQ "%s.cpp"\n#include "fw_modulo.cpp"\n' "$ns" "$m" > "$GEN/${ns}_$m.cpp"
    FONTES="$FONTES $GEN/${ns}_$m.cpp"
  done
  api=$([ $ns = fa ] && echo fw_api_a || echo fw_api_b)
  printf '#define FW_NS %s\n#define FW_API %s\n#include "ponte_robo.cpp"\n' "$ns" "$api" > "$GEN/${ns}_ponte.cpp"
  FONTES="$FONTES $GEN/${ns}_ponte.cpp"
done
COMUNS="$DIR/fisica.cpp $DIR/hal_combate.cpp $DIR/combate.cpp"
if [ "$ALVO" = wasm ]; then
  SAIDA=${2:-$DIR/combate.wasm}
  ZIG=${ZIG:-zig}
  # shellcheck disable=SC2086
  $ZIG c++ -target wasm32-wasi -mexec-model=reactor $FLAGS -fno-exceptions -Wno-unused-parameter \
    $COMUNS $DIR/wasm_api.cpp $FONTES -o "$SAIDA" -Wl,--strip-all -Wl,--export-dynamic
else
  SAIDA=${2:-$DIR/combate}
  g++ $FLAGS -Wall -Wextra -Wno-unused-parameter $COMUNS ${MAIN:-$DIR/main_nativo.cpp} $FONTES -o "$SAIDA"
fi
echo "simulador de combate em $SAIDA"
