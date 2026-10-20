#!/usr/bin/env python3
"""Divide o STEP final da montagem (LAB3-Modelagem_2.step) em um STL por objeto da árvore do CAD.

A árvore do FreeCAD mostra a montagem com um objeto por linha: Base, Frente1, Frente2, Lateral_R e
Lateral_L, Roda_Motor_D e Roda_Motor_L, Haste_L e Haste_R, Topo, Suporte_R e Suporte_L, os quatro
Linha_* (módulos TCRT5000), os sete suportes de sensor SS_*, a placa de expansão do ESP32 e o ESP32.
Este script grava um STL (binário, em milímetros) para cada um desses objetos, com o nome que o
objeto tem no STEP. Uma montagem (Roda_Motor_D, a placa de expansão, o ESP32) ou uma peça de vários
sólidos (Linha_L, com 11) sai em um arquivo só, como aparece na árvore, e não solta em parafusos e
pinos. O ferramentas/step_para_stl.py faz o contrário: abre a montagem em todas as peças soltas.

Cada STL fica na posição e no referencial do próprio CAD (a frente do robô em -x, o eixo das rodas em
x = 51 mm, o plano de simetria em y = 0 e a base em z = 0). Assim os arquivos importados juntos
no FreeCAD, no Blender ou no fatiador voltam a formar o robô montado, sem precisar de matriz. Para o
referencial do robô do config.h: x_robô = 51 - x, y_robô = -y, z_robô = z + 3.

Uso: python ferramentas/step_por_objeto.py LAB3-Modelagem_2.step cad/stl
Saída: um STL por objeto e o indice.json (caixa, triângulos, volume, tamanho e se a malha é fechada).
Precisa de: pip install cadquery-ocp trimesh numpy
"""
import json
import os
import re
import sys
import warnings

import numpy as np
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool

# Distância máxima entre a malha e a superfície do CAD e ângulo máximo entre triângulos vizinhos numa
# superfície curva. As peças impressas saem finas; as montagens compradas (rodas com motor, placa de
# expansão e ESP32, com centenas de sólidos) saem mais leves, porque servem só de referência de volume.
DESVIO_MM, DESVIO_ANG = 0.02, 0.25
DESVIO_REF_MM, DESVIO_REF_ANG = 0.05, 0.5


def nome_do(lab):
    n = TDataStd_Name()
    return n.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), n) else "objeto"


def nome_de_arquivo(nome):
    """Troca o que não serve em nome de arquivo e mantém o resto como está no CAD."""
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", nome.strip()).strip("_")


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    arq, saida = sys.argv[1], sys.argv[2]
    os.makedirs(saida, exist_ok=True)
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    leitor = STEPCAFControl_Reader()
    leitor.SetNameMode(True)
    leitor.SetColorMode(True)
    if leitor.ReadFile(arq) != IFSelect_RetDone:
        raise SystemExit(f"não consegui ler {arq}")
    leitor.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    raizes = TDF_LabelSequence()
    st.GetFreeShapes(raizes)
    if raizes.Length() != 1:
        raise SystemExit(f"esperava uma montagem na raiz, achei {raizes.Length()} objetos")
    raiz = raizes.Value(1)
    filhos = TDF_LabelSequence()
    st.GetComponents_s(raiz, filhos)
    print(f"{nome_do(raiz)}: {filhos.Length()} objetos no primeiro nível")

    indice = []
    usados = {}
    for i in range(1, filhos.Length() + 1):
        ref = filhos.Value(i)
        if st.IsReference_s(ref):
            alvo = TDF_Label()
            st.GetReferredShape_s(ref, alvo)
            nome = nome_do(ref)
            tipo = "montagem" if st.IsAssembly_s(alvo) else "peça"
        else:
            nome, tipo = nome_do(ref), "montagem" if st.IsAssembly_s(ref) else "peça"
        forma = st.GetShape_s(ref)          # já com a posição da peça dentro da montagem
        if tipo == "montagem":
            BRepMesh_IncrementalMesh(forma, DESVIO_REF_MM, False, DESVIO_REF_ANG, True)
        else:
            BRepMesh_IncrementalMesh(forma, DESVIO_MM, False, DESVIO_ANG, True)
        caixa = Bnd_Box()
        BRepBndLib.AddOptimal_s(forma, caixa, False, False)     # caixa exata, sem a folga da malha
        a, b = caixa.CornerMin(), caixa.CornerMax()
        arquivo = nome_de_arquivo(nome)
        usados[arquivo] = usados.get(arquivo, 0) + 1
        if usados[arquivo] > 1:
            arquivo = f"{arquivo}_{usados[arquivo]}"
        caminho = os.path.join(saida, arquivo + ".stl")
        escritor = StlAPI_Writer()
        escritor.ASCIIMode = False
        escritor.Write(forma, caminho)
        indice.append({"ordem": i, "nome": nome, "arquivo": arquivo + ".stl", "tipo": tipo,
                       "min": [round(a.X(), 2), round(a.Y(), 2), round(a.Z(), 2)],
                       "max": [round(b.X(), 2), round(b.Y(), 2), round(b.Z(), 2)]})
        print(f"{i:2d} {arquivo[:40]:40s} {tipo:8s} x {a.X():7.1f} {b.X():7.1f}  y {a.Y():7.1f} {b.Y():7.1f}  z {a.Z():6.1f} {b.Z():6.1f}")

    # conferência das malhas gravadas
    import trimesh
    warnings.filterwarnings("ignore")
    for item in indice:
        m = trimesh.load(os.path.join(saida, item["arquivo"]), process=True)
        item["triangulos"] = int(len(m.faces))
        item["malha_fechada"] = bool(m.is_watertight)
        # o volume vale quando a orientação das faces é consistente, mesmo com uma aresta compartilhada por 3 ou
        # mais faces (a Base tem uma, de 1 mm, no próprio STEP); nas montagens é a soma dos sólidos
        item["volume_cm3"] = round(float(m.volume) / 1000, 3) if m.is_winding_consistent else None
        item["tamanho_kb"] = round(os.path.getsize(os.path.join(saida, item["arquivo"])) / 1024)
        cmin, cmax = m.bounds
        # quanto a caixa da malha difere da caixa exata do CAD (deve ficar abaixo do desvio da malha)
        item["desvio_caixa_mm"] = round(float(max(np.abs(np.array(item["min"]) - cmin).max(),
                                                  np.abs(np.array(item["max"]) - cmax).max())), 3)
    with open(os.path.join(saida, "indice.json"), "w", encoding="utf-8") as f:
        json.dump(indice, f, ensure_ascii=False, indent=1)
    print(f"{len(indice)} STL em {saida}, {sum(i['tamanho_kb'] for i in indice) / 1024:.1f} MB")


if __name__ == "__main__":
    main()
