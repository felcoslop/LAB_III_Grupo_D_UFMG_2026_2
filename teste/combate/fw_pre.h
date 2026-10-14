// fw_pre.h
// Cabeçalhos do sistema e do Arduino de mentira incluídos fora do namespace de cada robô. Os
// módulos do firmware entram depois dentro de "namespace fa" ou "namespace fb" (fw_modulo.cpp),
// e o #pragma once faz as inclusões deles para estes cabeçalhos não fazerem nada lá dentro.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <map>
#include <string>
#include "Arduino.h"
#include "Wire.h"
#include "VL53L0X.h"
#include "Preferences.h"
