# gera_painel.py
# Este script monta o painel completo, com o three.js embutido, em um único arquivo HTML que
# abre direto no navegador, sem internet. Uso: python gera_painel.py
# Saída: ../painel_sumo.html. O script precisa rodar de novo toda vez que o painel_src.html mudar.
import os
aqui = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(aqui, "painel_src.html"), encoding="utf-8").read()
three = open(os.path.join(aqui, "vendor", "three.min.js"), encoding="utf-8").read()
orbit = open(os.path.join(aqui, "vendor", "OrbitControls.js"), encoding="utf-8").read()
# as marcações /*THREE_JS*/ e /*ORBIT_JS*/ do painel_src.html são trocadas pelas bibliotecas
html = src.replace("<script>/*THREE_JS*/</script>", "<script>/* three.js r147 (MIT) */\n" + three + "</script>") \
          .replace("<script>/*ORBIT_JS*/</script>", "<script>/* OrbitControls (MIT) */\n" + orbit + "</script>")
# newline="\n" mantém o arquivo igual em Windows e Linux (o CI confere se ele está atualizado)
with open(os.path.join(aqui, "..", "painel_sumo.html"), "w", encoding="utf-8", newline="\n") as f:
    f.write(html)
print(f"painel: {len(html) / 1024:.0f} KB")
