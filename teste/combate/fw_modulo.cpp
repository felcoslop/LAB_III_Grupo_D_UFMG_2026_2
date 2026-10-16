// fw_modulo.cpp
// Um módulo do firmware dentro do namespace de um robô. O script de compilação gera uma cópia
// deste arquivo por módulo e por robô, com FW_NS (fa ou fb) e FW_ARQ (o .cpp do firmware):
// assim os dois firmwares ficam no mesmo programa, cada um com as próprias variáveis globais,
// sem nenhuma linha do firmware mudar.
#include "fw_pre.h"
namespace FW_NS {
#include FW_ARQ
}
