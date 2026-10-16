#!/usr/bin/env bash
# Compilação do simulador de luta (software in the loop): os módulos do firmware entram como
# estão, com o Arduino de mentira de teste/sim/stub, na geometria do robô montado, com os 7
# sensores de oponente e com os motores ligados. Usado pelo CI e pelo workflow da página.
# Uso: bash teste/sim/compila.sh [saída] [flags extras do g++]
set -euo pipefail
cd "$(dirname "$0")/../.."
FW=firmware/sumo_esp32
SAIDA=${1:-teste/sim/luta}
shift || true
g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -I teste/sim/stub -I teste/sim -I $FW \
  -DGEOMETRIA_BANCADA=0 -DUSAR_MOTORES=1 -DQTD_TOF=7 "$@" \
  teste/sim/luta.cpp teste/sim/mundo.cpp teste/sim/hal.cpp \
  $FW/percepcao.cpp $FW/oponente.cpp $FW/rastreador.cpp $FW/borda.cpp $FW/estados.cpp \
  $FW/cinematica.cpp $FW/motores.cpp $FW/telemetria.cpp $FW/arena.cpp $FW/chave.cpp $FW/antitrava.cpp \
  -o "$SAIDA"
echo "simulador em $SAIDA"
