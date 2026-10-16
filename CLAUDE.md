# CLAUDE.md

Guia do repositório para o Claude Code. O `README.md` explica o projeto; este arquivo resume onde
fica cada coisa, como conferir uma mudança e as regras do grupo.

## O projeto

Firmware (ESP32, Arduino) do robô sumô autônomo de 1 kg do Grupo D, disciplina ELE635 Laboratório
de Sistemas III da UFMG, com painel de telemetria, simuladores com o firmware de verdade e páginas
no GitHub Pages. Textos, comentários e mensagens de commit em português do Brasil.

## Onde fica cada coisa

| Pasta | Conteúdo |
|---|---|
| `firmware/sumo_esp32/` | sketch da Arduino IDE: `sumo_esp32.ino`, `config.h` e os módulos `.h`/`.cpp` |
| `painel/` | `painel_sumo.html` (gerado), `painel_src.html` (fonte) e `vendor/` (three.js) |
| `site/` | páginas do GitHub Pages; `combate.html` e `montagem.html` são geradas |
| `doc/` | `img/` do README, `pdf/` dos guias, dados das páginas em `combate/` e `montagem/` |
| `ferramentas/` | scripts Python que geram painel, páginas e PDFs |
| `teste/` | `unidade/` (g++), `sim/` (lutas) e `combate/` (dois firmwares, nativo e WebAssembly) |

O sketch fica plano, sem subpastas: a Arduino IDE só compila a pasta do `.ino` e a subpasta `src/`,
só mostra como abas os arquivos da pasta do `.ino` e exige que a pasta tenha o nome do `.ino`.
Módulo novo entra em `firmware/sumo_esp32/` e também nas listas de fontes de
`.github/workflows/compilacao.yml`, `teste/sim/compila.sh` e `teste/combate/compila.sh`.
Toda configuração de hardware fica no `config.h`.

## Como conferir uma mudança (o mesmo que o CI roda)

```bash
FW=firmware/sumo_esp32; INC="-I teste/unidade/stub -I $FW"
g++ -std=c++11 -Wall -Wextra -Werror $INC teste/unidade/teste_cinematica.cpp $FW/cinematica.cpp -o /tmp/t && /tmp/t
g++ -std=c++11 -Wall -Wextra -Werror $INC teste/unidade/teste_estados.cpp $FW/estados.cpp $FW/cinematica.cpp $FW/arena.cpp $FW/chave.cpp $FW/antitrava.cpp -o /tmp/t && /tmp/t
for g in 1 0; do g++ -std=c++11 -O2 -Wall -Wextra -Werror $INC -DGEOMETRIA_BANCADA=$g teste/unidade/teste_arena.cpp $FW/arena.cpp -o /tmp/t && /tmp/t; done
for v in 2 3; do g++ -std=c++11 -Wall -Wextra -Werror $INC -DUSAR_MOTORES=1 -DESP_ARDUINO_VERSION_MAJOR=$v teste/unidade/teste_motores.cpp $FW/motores.cpp -o /tmp/t && /tmp/t; done
bash teste/sim/compila.sh && python teste/sim/roda_lutas.py teste/sim/luta --sementes 8 --dips 1,2,4 --fora --perfil-pela-cor --exige --fantasma-max 3
bash teste/combate/compila.sh && python teste/combate/bateria.py teste/combate/combate --sementes 4 --exige-quedas-proprias 0
```

A compilação para o ESP32 (cores 2.0.17 e 3.3.0, com 5 e 7 sensores) roda no CI com a Arduino CLI
apontando para `firmware/sumo_esp32`.

## Arquivos gerados que ficam no repositório

O CI gera cada um de novo e falha se o versionado estiver diferente. Depois de mudar a fonte,
gerar de novo e fazer o commit junto:

| Arquivo | Muda quando muda | Comando |
|---|---|---|
| `painel/painel_sumo.html` | `painel/painel_src.html` | `python ferramentas/gera_painel.py` |
| `site/combate.html` | firmware, `teste/combate/`, `doc/combate/` | `pip install ziglang==0.14.0`, `ZIG="python -m ziglang" bash teste/combate/compila.sh wasm` e `python ferramentas/gera_combate.py --wasm teste/combate/combate.wasm` |
| `site/montagem.html` | `config.h` (sensores e GPIOs), `doc/montagem/` | `python ferramentas/gera_montagem.py` |
| `doc/pdf/*.pdf` | posição ou pino de sensor no `config.h` | `python ferramentas/gera_guia.py`, `python ferramentas/gera_guia.py --sensores 7` e `python ferramentas/gera_bancada.py` (ReportLab e fontes DejaVu; o CI não confere) |

O `site/index.html` é escrito à mão e o `python ferramentas/confere_site.py` confere as constantes
dele com o `config.h`.

## Regras do repositório

- Commits só no nome do Felipe Costa Lopes (e-mail noreply do GitHub), sem coautor e sem linha de
  atribuição ao Claude, em mensagens de commit e de pull request.
- Toda mudança entra na `main` por issue, branch e pull request com `Closes #N`, depois do CI verde,
  com merge commit, para o botão Revert desfazer um pull request por vez. Nada de push forçado na
  `main`.
- Antes de um merge em lote, uma branch `backup/main-AAAAMMDD-HHMM` guarda a `main` de antes. Nas
  sessões do Claude Code na nuvem o proxy não deixa criar tags nem usar a API GraphQL (branch ligada
  à issue); por isso o backup é branch e a ligação com a issue fica no `Closes #N`.
- Texto técnico (README, docs e comentários) sem primeira pessoa, sem travessão, sem "através de" no
  sentido de "por meio de", sem a partícula apassivadora "-se" e sem frase começando por verbo;
  números com vírgula decimal e citações no fim da frase, com a referência correspondente no fim do
  README.
