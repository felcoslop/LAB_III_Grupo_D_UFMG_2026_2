#!/usr/bin/env python3
"""Converte o STEP completo do chassi (LAB3-Modelagem.step) em um STL por peça.

O STEP vem do projeto mecânico com a montagem inteira: base, frente, laterais, hastes, suportes e
tampa (PLA), os dois motorredutores JGB37-520, os cubos, os pneus StickyMAX e os parafusos. Este
script percorre a árvore da montagem, aplica a posição de cada peça e grava um STL (milímetros,
binário) com o nome da peça. As malhas dos dois outros scripts (ferramentas/exporta_pecas.py e
ferramentas/exporta_malhas.py) saem desta pasta.

Referencial do STEP: o x do projeto aponta para trás (a frente do robô é -x), o eixo das rodas fica
em x = 51 mm e z = 13 mm no desenho, e o plano de simetria é y = 0. O exporta_pecas.py leva tudo
para o referencial do robô (origem no meio do eixo das rodas, no chão, x para a frente, y para a
esquerda): x_robô = 51 - x, y_robô = -y, z_robô = z + 3,0.

Uso: python ferramentas/step_para_stl.py LAB3-Modelagem.step PASTA_DE_SAIDA
Precisa de: pip install cadquery-ocp
"""
import os
import sys

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
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool


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
    usados = {}

    def nome(lab):
        n = TDataStd_Name()
        return n.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), n) else "peca"

    def visita(lab, loc):
        if st.IsReference_s(lab):
            ref = TDF_Label()
            st.GetReferredShape_s(lab, ref)
            return visita(ref, loc.Multiplied(st.GetLocation_s(lab)))
        if st.IsAssembly_s(lab):
            filhos = TDF_LabelSequence()
            st.GetComponents_s(lab, filhos)
            for i in range(1, filhos.Length() + 1):
                visita(filhos.Value(i), loc)
            return None
        forma = st.GetShape_s(lab).Located(loc)
        BRepMesh_IncrementalMesh(forma, 0.05, False, 0.3, True)
        caixa = Bnd_Box()
        BRepBndLib.Add_s(forma, caixa)
        a, b = caixa.CornerMin(), caixa.CornerMax()
        n = nome(lab).replace(" ", "_").replace(",", "").replace("/", "_")
        usados[n] = usados.get(n, 0) + 1
        if usados[n] > 1:
            n = f"{n}_{usados[n]}"
        escritor = StlAPI_Writer()
        escritor.ASCIIMode = False
        escritor.Write(forma, os.path.join(saida, n + ".stl"))
        print(f"{n[:48]:48s} x {a.X():7.1f} {b.X():7.1f}  y {a.Y():7.1f} {b.Y():7.1f}  z {a.Z():6.1f} {b.Z():6.1f}")
        return None

    raizes = TDF_LabelSequence()
    st.GetFreeShapes(raizes)
    for i in range(1, raizes.Length() + 1):
        visita(raizes.Value(i), TopLoc_Location())


if __name__ == "__main__":
    main()
