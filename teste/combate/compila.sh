#!/usr/bin/env bash
# Compilação do simulador de combate. Cada módulo do firmware entra duas vezes, uma no namespace
# fa (robô do Grupo D) e outra no fb (adversário), pelo fw_modulo.cpp; o firmware não muda.
# Uso: bash teste/combate/compila.sh [nativo|wasm|empurrao] [saída]
#   nativo (padrão): executável de linha de comando (main_nativo.cpp), com o g++
#   empurrao: só a física, sem firmware (empurrao.cpp): empurrão frente a frente, em malha aberta, para
#         comparar a física à mão com a do MuJoCo (compara_fisicas.py)
#   MUJOCO=1 (só no nativo e no empurrao): liga a física do MuJoCo (opção -F mujoco), com o pacote do pip
#         (pip install mujoco==3.16.0); a página 3D e o CI padrão continuam com a física à mão
#   wasm: módulo WebAssembly da página 3D (wasm_api.cpp), com o zig 0.14 (ZIG=/caminho/do/zig ou
#         ZIG="python -m ziglang" depois de pip install ziglang==0.14.0)
set -euo pipefail
cd "$(dirname "$0")/../.."
ALVO=${1:-nativo}
DIR=teste/combate
GEN=$(mktemp -d)
trap 'rm -rf "$GEN"' EXIT
FLAGS="-std=c++17 -O2 -DGEOMETRIA_BANCADA=0 -DUSAR_MOTORES=1 -DQTD_TOF=7 -I $DIR/stub -I teste/sim/stub -I $DIR -I firmware/sumo_esp32"
LIGA=""
EXTRA=()
if [ "${MUJOCO:-0}" = 1 ] && [ "$ALVO" != wasm ]; then
  MJ=$(python3 -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))")
  MJLIB=$(ls "$MJ"/libmujoco.so.* | head -1)
  EXTRA=(-DCB_MUJOCO "-DCB_MALHAS=\"$PWD/$DIR/mujoco/malhas\"" -I "$MJ/include")
  LIGA="$MJLIB -Wl,-rpath,$MJ"
fi
if [ "$ALVO" = empurrao ]; then
  SAIDA=${2:-$DIR/empurrao}
  g++ $FLAGS ${EXTRA[@]+"${EXTRA[@]}"} -Wall -Wextra -Wno-unused-parameter $DIR/fisica.cpp $DIR/fisica_mujoco.cpp $DIR/empurrao.cpp $LIGA -o "$SAIDA"
  echo "empurrão em $SAIDA"
  exit 0
fi
MODULOS="percepcao oponente rastreador borda estados cinematica motores telemetria arena chave antitrava"
COMUNS="$DIR/fisica.cpp $DIR/fisica_mujoco.cpp $DIR/hal_combate.cpp $DIR/combate.cpp"
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
if [ "$ALVO" = wasm ]; then
  SAIDA=${2:-$DIR/combate.wasm}
  ZIG=${ZIG:-zig}
  # shellcheck disable=SC2086
  $ZIG c++ -target wasm32-wasi -mexec-model=reactor $FLAGS -fno-exceptions -Wno-unused-parameter \
    $COMUNS $DIR/wasm_api.cpp $FONTES -o "$SAIDA" -Wl,--strip-all -Wl,--export-dynamic
else
  SAIDA=${2:-$DIR/combate}
  g++ $FLAGS ${EXTRA[@]+"${EXTRA[@]}"} -Wall -Wextra -Wno-unused-parameter $COMUNS ${MAIN:-$DIR/main_nativo.cpp} $FONTES $LIGA -o "$SAIDA"
fi
echo "simulador de combate em $SAIDA"
