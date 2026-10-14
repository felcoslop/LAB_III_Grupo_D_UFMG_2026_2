"""Converte as peças do STL do robô em malhas compactas para a página 3D do combate.

Referencial de saída (mm): x para a frente, y para cima e z para a direita (o do three.js com o
robô olhando para +x), origem no meio do eixo das rodas, no chão. No STL a frente é -x, o eixo
das rodas fica em x = 56,18 mm e z = 14,43 mm, e o plano de simetria em y = -0,5 mm. O motor e a
abraçadeira vêm de um lado só e são espelhados; os cubos ficam 2 mm para dentro, com o corte da
ponta dos eixos que o projeto mecânico pede.

Uso: python doc/combate/exporta_malhas.py PASTA_DOS_STL doc/combate/malhas.js
Precisa de: pip install trimesh fast-simplification networkx scipy
"""
import trimesh, numpy as np, os, json, base64, sys, glob, warnings
warnings.filterwarnings('ignore')
D=sys.argv[1].rstrip('/')+'/'
EIXO_X, MEIO_Y, DZ = 56.18, -0.5, 15.97-14.43
def stl(nome):
    f=[g for g in glob.glob(D+'*') if g.endswith(nome)][0]
    return trimesh.load(f,force='mesh')
def corpo_de_stl(m, espelha=False):
    v=m.vertices.copy()
    if espelha:
        v[:,1]=2*MEIO_Y-v[:,1]
    xb=EIXO_X-v[:,0]; yb=-(v[:,1]-MEIO_Y); zb=v[:,2]+DZ
    f=m.faces.copy()
    # rotação de 180 graus em z mantém a orientação; o espelho inverte
    if espelha: f=f[:,::-1]
    return trimesh.Trimesh(np.c_[xb,zb,-yb], f, process=True)   # (x, y=cima, z=-y_corpo)
def roda(m, eixo_local, lado, deslocamento_y):
    # peça com eixo no x local (cubo) ou no z local (pneu) -> eixo no y do corpo
    v=m.vertices.copy()
    if eixo_local=='x': a=v[:,0]; b=v[:,1]; c=v[:,2]
    else: a=v[:,2]; b=v[:,0]; c=v[:,1]
    yb=lado*a+deslocamento_y; xb=b; zb=c+15.97
    f=m.faces.copy()
    if lado<0: f=f[:,::-1]
    # (a,b,c)->(y,x,z) troca a orientação: confere pelo volume
    t=trimesh.Trimesh(np.c_[xb,zb,-yb], f, process=True)
    if t.volume<0: t.invert()
    return t
pecas={}
pecas['base']=corpo_de_stl(stl('Base-Sumo.stl'))
pecas['corpo']=corpo_de_stl(stl('Corpo.stl'))
h=stl('Maker_Object_Holder.stl'); h.apply_translation([0,-7.5,0])   # ponte centrada na largura (no STL ela passa 5 mm do corpo)
pecas['suporte']=corpo_de_stl(h)
pecas['motorE']=corpo_de_stl(stl('motor.stl'))
pecas['motorD']=corpo_de_stl(stl('motor.stl'),True)
pecas['abracD']=corpo_de_stl(stl('Suporte-Final.stl'))
pecas['abracE']=corpo_de_stl(stl('Suporte-Final.stl'),True)
for k in list(pecas):
    if pecas[k].volume<0: pecas[k].invert()
pn=stl('pneu.stl'); r1=stl('Roda_1.stl'); r2=stl('Roda_2.stl')
Y=63.65
pecas['pneuE']=roda(pn,'z',+1,Y); pecas['pneuD']=roda(pn,'z',-1,-Y)
pecas['cuboE']=roda(r1,'x',+1,Y); pecas['cuboD']=roda(r2,'x',-1,-Y)
ALVO={'motorE':2500,'motorD':2500,'pneuE':2400,'pneuD':2400}
for k,n in ALVO.items():
    m=pecas[k]
    pecas[k]=m.simplify_quadric_decimation(face_count=n)
for k,m in pecas.items():
    print(f"{k:8s} tris {len(m.faces):6d} bounds {np.round(m.bounds,1).tolist()}")
saida={}
for k,m in pecas.items():
    sm=trimesh.graph.smooth_shade(m, angle=np.radians(35))
    v=sm.vertices; n=sm.vertex_normals; f=sm.faces.astype(np.uint32)
    q=np.round(v/0.01).astype(np.int16)           # 0,01 mm
    nn=np.round(n*127).astype(np.int8)
    ind=f.astype(np.uint16) if len(v)<65536 else f
    saida[k]={'p':base64.b64encode(q.tobytes()).decode(),'n':base64.b64encode(nn.tobytes()).decode(),
              'i':base64.b64encode(ind.tobytes()).decode(),'i32':len(v)>=65536,'nv':len(v)}
txt='window.MALHAS='+json.dumps(saida,separators=(',',':'))+';\n'
open(sys.argv[2],'w').write(txt)
print('bytes',len(txt))
