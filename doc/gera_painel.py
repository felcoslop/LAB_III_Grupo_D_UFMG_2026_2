# gera_painel.py
# Este script monta o painel completo, com o three.js embutido, e a versão comprimida que fica
# gravada dentro do ESP32. Uso: python gera_painel.py
# Saídas: ../painel_sumo.html (para abrir no PC) e ../src/painel_gz.h (servido em http://192.168.4.1
# pelo telemetria.cpp). O script precisa rodar de novo toda vez que o painel_src.html mudar.
import gzip, os
aqui = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(aqui, "painel_src.html"), encoding="utf-8").read()
three = open(os.path.join(aqui, "vendor", "three.min.js"), encoding="utf-8").read()
orbit = open(os.path.join(aqui, "vendor", "OrbitControls.js"), encoding="utf-8").read()
# as marcações /*THREE_JS*/ e /*ORBIT_JS*/ do painel_src.html são trocadas pelas bibliotecas
html = src.replace("<script>/*THREE_JS*/</script>", "<script>/* three.js r147 (MIT) */\n" + three + "</script>") \
          .replace("<script>/*ORBIT_JS*/</script>", "<script>/* OrbitControls (MIT) */\n" + orbit + "</script>")
open(os.path.join(aqui, "..", "painel_sumo.html"), "w", encoding="utf-8").write(html)
# mtime=0 deixa o gzip sempre igual para o mesmo HTML (sem data dentro do arquivo)
gz = gzip.compress(html.encode("utf-8"), compresslevel=9, mtime=0)
with open(os.path.join(aqui, "..", "src", "painel_gz.h"), "w") as f:
    f.write("// GERADO por doc/gera_painel.py a partir de doc/painel_src.html. Não edite à mão.\n")
    f.write("#pragma once\n#include <stdint.h>\n#include <stddef.h>\n")
    f.write(f"static const size_t PAINEL_GZ_LEN = {len(gz)};\n")
    f.write("static const uint8_t PAINEL_GZ[] = {\n")
    # o laço escreve os bytes em linhas de 24 valores para o arquivo .h ficar legível
    for i in range(0, len(gz), 24):
        f.write("  " + ",".join(str(b) for b in gz[i:i + 24]) + ",\n")
    f.write("};\n")
print(f"painel: {len(html)/1024:.0f} KB  | comprimido no ESP32: {len(gz)/1024:.0f} KB")
